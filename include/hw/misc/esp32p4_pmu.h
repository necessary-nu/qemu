/*
 * ESP32-P4 power management unit (PMU)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_PMU_H
#define HW_MISC_ESP32P4_PMU_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_ana.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "qom/object.h"

#define TYPE_ESP32P4_PMU "esp32p4-pmu"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4PMUState, ESP32P4_PMU)

#define ESP32P4_PMU_BASE        0x50115000
#define ESP32P4_PMU_SIZE        0x1000
#define ESP32P4_PMU_NREGS       (0x400 / 4)

/* The wakeup and sleep-rejection sources, by bit (TRM table 15.4-1). */
enum {
    ESP32P4_PMU_WAKE_HP_SDIO = 0,
    ESP32P4_PMU_WAKE_SW = 1,
    ESP32P4_PMU_WAKE_GPIO = 2,
    ESP32P4_PMU_WAKE_USB = 3,
    ESP32P4_PMU_WAKE_UART4 = 4,
    ESP32P4_PMU_WAKE_UART3 = 5,
    ESP32P4_PMU_WAKE_UART2 = 6,
    ESP32P4_PMU_WAKE_UART1 = 7,
    ESP32P4_PMU_WAKE_UART0 = 8,
    ESP32P4_PMU_WAKE_LP_GPIO = 9,
    ESP32P4_PMU_WAKE_LP_UART = 10,
    ESP32P4_PMU_WAKE_TOUCH = 11,
    ESP32P4_PMU_WAKE_EXT = 12,
    ESP32P4_PMU_WAKE_RTC_TIMER0 = 13,
    ESP32P4_PMU_WAKE_BROWNOUT = 14,
    ESP32P4_PMU_WAKE_VBAT = 15,
    ESP32P4_PMU_WAKE_LP_CPU_EXC = 16,
    ESP32P4_PMU_WAKE_ETM = 17,
    ESP32P4_PMU_WAKE_RTC_TIMER1 = 18,
    ESP32P4_PMU_WAKE_LP_I2S = 19,
    ESP32P4_PMU_WAKE_GMAC_PMT = 20,
    ESP32P4_PMU_WAKE_GMAC_LPI = 21,
    ESP32P4_PMU_WAKE_LP_CPU = 22,
    ESP32P4_PMU_WAKE_HP_CPU = 23,
    ESP32P4_PMU_WAKE_SOURCES = 31,
};

/* The power domains of the "power" output. */
enum {
    ESP32P4_PMU_PD_TOP,
    ESP32P4_PMU_PD_HP_CPU,
    ESP32P4_PMU_PD_HP_CNNT,
    ESP32P4_PMU_PD_HP_MEM,
    ESP32P4_PMU_PD_LP_PERI,
    ESP32P4_PMU_NUM_PD,
};

/*
 * The output regulators' waiting counters, in register order: P0_0P1A
 * (VO1), P0_0P2A (VO3), P0_0P3A, P1_0P1A (VO2), P1_0P2A (VO4), P1_0P3A.
 */
#define ESP32P4_PMU_EXT_LDOS    6

/* The PMU's states: those of the main state machine and its switches. */
enum {
    ESP32P4_PMU_ST_HP_ACTIVE,
    ESP32P4_PMU_ST_ACTIVE_TO_SLEEP,
    ESP32P4_PMU_ST_HP_SLEEP,
    ESP32P4_PMU_ST_HP_TO_LP,
    ESP32P4_PMU_ST_LP_SLEEP,
    ESP32P4_PMU_ST_LP_TO_HP,
    ESP32P4_PMU_ST_SLEEP_TO_ACTIVE,
};

/*
 * Inputs and outputs (the device's comment says what each does):
 *
 *   - Clock inputs "fast" (LP_DYN_FAST_CLK) and "slow" (LP_DYN_SLOW_CLK),
 *     which time the state machine and the regulators' counters.
 *   - Links "clkrst", where the PMU records its resets of the cores;
 *     "lp-cpu", whose wait for an interrupt (its "in-wfi" property)
 *     LP_CPU_WAITI_RDY shows; and
 *     "lp-sys", where LP_SYSTEM's EXT_WAKEUP1_STATUS records the pads of
 *     an EXT wakeup.
 *   - Sysbus IRQs 0 and 1: PMU_INTR (the HP_INT_* registers) and the
 *     LP_INT_* registers' interrupt, interrupt matrix sources 6 and 7; the
 *     latter is also the named GPIO output "lp-int", for the LP core.
 *   - Named GPIO inputs: "wakeup", ESP32P4_PMU_WAKE_SOURCES lines, the
 *     levels of the wakeup sources by TRM table 15.4-1's bits, from the
 *     blocks that raise them (the PMU's own, 1, 12 and 22, are ORed in);
 *     "ext-pad", the levels of pads GPIO0 to GPIO15, for the EXT wakeup;
 *     "etm-task", the ETM's PMU_TASK_SLEEP_REQ.
 *   - Named GPIO outputs: "power", per ESP32P4_PMU_PD_*, high while the
 *     domain is powered; "xpd", per ESP32P4_CLKRST_XPD_*, the oscillators'
 *     power; "pll-xpd", per ESP32P4_ANA_PLL_*, the PLLs' power;
 *     "core-stall" per HP core; "icg", per bit of DIG_ICG_FUNC_EN, high
 *     while that clock runs; "sysclk", per ESP32P4_CLKRST_SYSCLK_*;
 *     "lp-sleep", high in LP_SLEEP; "lp-core-reset", "lp-core-clock" and
 *     "lp-core-stall"; "hold-all", the HP and LP pads' global holds;
 *     "pad-sleep", the pads' sleep configuration select; "pause-wdt", the
 *     RTC watchdog's pause; "xtal-off", high while the crystal is powered
 *     down; "bod-vbat-sel", BOD_SOURCE_SEL in the current state;
 *     "uart-wakeup-en", the UARTs' wakeup enable; "lp-core-intr-mask",
 *     high while the LP CPU's sleep process masks its interrupts;
 *     "etm-event", per ESP32P4_PMU_ETM_EVT_*, the ETM events
 *     PMU_EVT_SLEEP_WAKEUP, pulsed as the chip wakes, ULP_EVT_START_INTR,
 *     as the LP CPU's clock starts on its wakeup, and ULP_EVT_HALT, as
 *     the LP CPU's sleep process ends.
 */
#define ESP32P4_PMU_WAKEUP          "wakeup"
#define ESP32P4_PMU_EXT_PAD         "ext-pad"
#define ESP32P4_PMU_ETM_TASK        "etm-task"
#define ESP32P4_PMU_LP_INT          "lp-int"
#define ESP32P4_PMU_POWER           "power"
#define ESP32P4_PMU_XPD             "xpd"
#define ESP32P4_PMU_PLL_XPD         "pll-xpd"
#define ESP32P4_PMU_CORE_STALL      "core-stall"
#define ESP32P4_PMU_ICG             "icg"
#define ESP32P4_PMU_SYSCLK          "sysclk"
#define ESP32P4_PMU_LP_SLEEP        "lp-sleep"
#define ESP32P4_PMU_LP_CORE_RESET   "lp-core-reset"
#define ESP32P4_PMU_LP_CORE_CLOCK   "lp-core-clock"
#define ESP32P4_PMU_LP_CORE_STALL   "lp-core-stall"
#define ESP32P4_PMU_HOLD_ALL        "hold-all"
#define ESP32P4_PMU_PAD_SLEEP       "pad-sleep"
#define ESP32P4_PMU_PAUSE_WDT       "pause-wdt"
#define ESP32P4_PMU_XTAL_OFF        "xtal-off"
#define ESP32P4_PMU_BOD_VBAT_SEL    "bod-vbat-sel"
#define ESP32P4_PMU_UART_WAKEUP_EN  "uart-wakeup-en"
#define ESP32P4_PMU_LP_CORE_INTR_MASK "lp-core-intr-mask"
#define ESP32P4_PMU_ETM_EVENT       "etm-event"

enum {
    ESP32P4_PMU_ETM_EVT_SLEEP_WAKEUP,
    ESP32P4_PMU_ETM_EVT_LP_CPU_START,
    ESP32P4_PMU_ETM_EVT_LP_CPU_HALT,
    ESP32P4_PMU_ETM_EVTS,
};

#define ESP32P4_PMU_EXT_PADS        16

/* The holds of "hold-all". */
enum {
    ESP32P4_PMU_HOLD_HP,
    ESP32P4_PMU_HOLD_LP,
    ESP32P4_PMU_HOLDS,
};

/*
 * The controls the PMU applies: each state's settings as the state
 * machine last took them, or as the immediate registers since set them.
 */
typedef struct ESP32P4PMUControls {
    /* Digital power: PD_EN of PD_TOP, PD_HP_CPU, PD_HP_CNNT, HP memory. */
    bool top_pd;
    bool cpu_pd;
    bool cnnt_pd;
    bool mem_pd;
    bool lp_peri_pd;
    /* Analog: the crystal and the PLLs' power, the PLL I2C controllers. */
    bool xpd_xtal;
    uint8_t xpd_pll;
    uint8_t xpd_pll_i2c;
    bool i2c_iso;
    bool i2c_retention;
    uint8_t icg_global_pll;
    bool icg_global_xtal;
    /* Clocks: the gates and ROOT_CLK's control. */
    uint32_t icg_func;
    uint32_t icg_apb;
    uint8_t icg_modem_code;
    bool icg_slp_sel;
    bool sysclk_slp_sel;
    bool icg_sysclk_en;
    bool sysclk_nodiv;
    uint8_t sysclk_sel;
    bool lp_rootclk_sel;
    /* The system controller. */
    bool uart_wakeup_en;
    bool lp_pad_hold;
    bool hp_pad_hold;
    bool pad_slp_sel;
    bool pause_wdt;
    bool cpu_stall;
    /* Holds the PMU took of itself as a domain powered down. */
    bool hp_pad_hold_auto;
    bool lp_pad_hold_auto;
} ESP32P4PMUControls;

/* The levels last driven on the outputs, by output. */
typedef struct ESP32P4PMULevels {
    bool xpd[ESP32P4_CLKRST_NUM_XPD];
    bool pll_xpd[ESP32P4_ANA_NUM_PLLS];
    bool core_stall[ESP32P4_CLKRST_HP_CORES];
    bool icg[ESP32P4_CLKRST_NUM_PMU_ICG];
    bool sysclk[ESP32P4_CLKRST_NUM_SYSCLK];
    bool lp_sleep;
    bool lp_core_reset;
    bool lp_core_clock;
    bool lp_core_stall;
    bool hold_all[ESP32P4_PMU_HOLDS];
    bool pad_sleep;
    bool pause_wdt;
    bool xtal_off;
    bool bod_vbat_sel;
    bool uart_wakeup_en;
    bool lp_core_intr_mask;
} ESP32P4PMULevels;

typedef struct ESP32P4PMULdo {
    struct ESP32P4PMUState *s;
    QEMUTimer timer;
    /* Counting since start_ns, while running; the targets it has passed. */
    bool running;
    int64_t start_ns;
    bool reached[2];
} ESP32P4PMULdo;

struct ESP32P4PMUState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[2];
    qemu_irq lp_int;
    qemu_irq power[ESP32P4_PMU_NUM_PD];
    qemu_irq xpd[ESP32P4_CLKRST_NUM_XPD];
    qemu_irq pll_xpd[ESP32P4_ANA_NUM_PLLS];
    qemu_irq core_stall[ESP32P4_CLKRST_HP_CORES];
    qemu_irq icg[ESP32P4_CLKRST_NUM_PMU_ICG];
    qemu_irq sysclk[ESP32P4_CLKRST_NUM_SYSCLK];
    qemu_irq lp_sleep;
    qemu_irq lp_core_reset;
    qemu_irq lp_core_clock;
    qemu_irq lp_core_stall;
    qemu_irq hold_all[ESP32P4_PMU_HOLDS];
    qemu_irq pad_sleep;
    qemu_irq pause_wdt;
    qemu_irq xtal_off;
    qemu_irq bod_vbat_sel;
    qemu_irq uart_wakeup_en;
    qemu_irq lp_core_intr_mask;
    qemu_irq etm_event[ESP32P4_PMU_ETM_EVTS];
    Clock *fast;
    Clock *slow;
    ESP32P4ClkRstState *clkrst;
    CPUState *lp_cpu;
    ESP32P4LPSysState *lp_sys;

    uint32_t regs[ESP32P4_PMU_NREGS];

    /* The main state machine: its state, the last and the switch's goal. */
    uint32_t state;
    uint32_t last_state;
    uint32_t target_state;
    QEMUTimer seq_timer;
    /* The step's cycles, and whether it waits for its clock to run. */
    uint64_t seq_fast;
    uint64_t seq_slow;
    bool seq_waiting;
    /* When the chip entered its sleep state, for the minimum sleep. */
    int64_t sleep_start_ns;
    ESP32P4PMUControls cur;

    /* The wakeup sources' levels from outside, and the EXT wakeup's pads. */
    uint32_t wakeup_in;
    uint32_t ext_pads;

    /*
     * The LP CPU's power control: its clock on, its reset held, its stall;
     * its sleep process under way, which waits lp_sleep_timer out before
     * stopping it, and whether that process masks its interrupts.
     */
    bool lp_cpu_on;
    bool lp_cpu_held;
    bool lp_cpu_stalled;
    bool lp_cpu_sleeping;
    bool lp_cpu_masked;
    bool lp_sleep_waiting;
    QEMUTimer lp_sleep_timer;

    ESP32P4PMULdo ldo[ESP32P4_PMU_EXT_LDOS];

    bool power_level[ESP32P4_PMU_NUM_PD];
    ESP32P4PMULevels out;
};

#endif
