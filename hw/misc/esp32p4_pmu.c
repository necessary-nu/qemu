/*
 * ESP32-P4 power management unit (PMU)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 15 (Low-Power Management), and ESP-IDF's pmu_reg.h and pmu_struct.h for
 * v3, which give the registers' layout, access and reset values; its hal
 * pmu_ll.h, lp_core_ll.h, clk_tree_ll.h and ldo_ll.h, and
 * esp_hw_support's pmu_init.c, pmu_param.c, pmu_sleep.c and sleep_modes.c
 * and the LP core's lp_core_utils.c, which show how software drives it.
 *
 * The state machine (TRM 15.4.2.1). Three states, HP_ACTIVE, HP_SLEEP
 * (the HP system asleep, the LP system running) and LP_SLEEP (both
 * asleep), with the switches between them. PMU_SLEEP_REQ (or the ETM task
 * PMU_TASK_SLEEP_REQ) starts the switch from HP_ACTIVE to HP_SLEEP, and on
 * to LP_SLEEP if the LP CPU is asleep; the LP CPU's sleep takes the chip
 * from HP_SLEEP to LP_SLEEP, its wakeup back. A wakeup source that
 * PMU_WAKEUP_ENA enables wakes the chip to HP_ACTIVE, recording the
 * sources in PMU_WAKEUP_CAUSE and raising PMU_SOC_WAKEUP_INT; one that
 * PMU_SLEEP_REJECT_ENA enables (with PMU_SLP_REJECT_EN) rejects a sleep
 * requested while, or until the switch ends while, it is active, recording
 * it in PMU_REJECT_CAUSE and raising PMU_SOC_SLEEP_REJECT_INT. A source
 * already active when the chip falls asleep wakes it at once. The LP
 * interrupts mark the switches' starts and ends.
 *
 * The wakeup sources (TRM table 15.4-1) are levels: those of the blocks
 * that raise them (the "wakeup" inputs: the GPIO and LP GPIO wakeups, the
 * brown-out detector's, VDD_BAT's under-voltage, HP_SYSTEM's interrupt
 * wakeup, source 23, and the hooks the machine leaves for the UARTs, USB,
 * SDIO, touch, the ETM, LP I2S and the EMAC), the RTC timer's alarms (its
 * raw target interrupts, which hold until cleared), the EXT wakeup (a pad
 * PMU_EXT_WAKEUP_SEL selects, of GPIO0 to GPIO15, at the level of its
 * PMU_EXT_WAKEUP_LV bit; the pads are recorded in PMU_EXT_WAKEUP_ST as
 * they wake the chip, and PMU_EXT_WAKEUP_STATUS_CLR holds it clear, and
 * in LP_SYSTEM's EXT_WAKEUP1_STATUS, as ESP-IDF reads them), the
 * LP CPU's exception, and the CPUs' software triggers: PMU_LP_TRIGGER_HP
 * raises PMU_SW_INT, whose raw bit is source 1 (ESP-IDF's
 * PMU_LP_CORE_WAKEUP_HP_EN), and PMU_HP_TRIGGER_LP raises
 * PMU_HP_SW_TRIGGER_INT, whose raw bit is source 22 (the TRM's "LP CPU",
 * ESP-IDF's LP_CORE_LL_WAKEUP_SOURCE_HP_CPU). PMU_EXT_WAKEUP_FILTER and
 * PMU_LP_LITE_WAKEUP_ENA, which the TRM does not describe, are logged as
 * not modelled.
 *
 * The settings (15.4.2.3 to 15.4.2.7). Each state has its digital power,
 * clock gating, system control and analog settings: HP_ACTIVE's and
 * HP_SLEEP's HP_* registers for the HP system, LP_SLEEP reusing
 * HP_SLEEP's; and for the LP system HP_SLEEP_LP_* in HP_ACTIVE and
 * HP_SLEEP, LP_SLEEP_* in LP_SLEEP. The HP settings take effect as the
 * state machine switches to their state (the HP_MODEM registers, of a
 * state the P4 does not have, read 0); the immediate registers
 * (PMU_IMM_*) set the controls at once, until the next switch: TIE_HIGH
 * and TIE_LOW set or clear a control, UPDATE takes the current state's
 * register. The LP oscillators' and supply settings act at once in their
 * state, as ESP-IDF's clk_tree_ll.h relies on.
 *
 *   - Power domains: PD_TOP (the HP system), PD_HP_CPU, PD_HP_CNNT (USB,
 *     SD/MMC, EMAC), the HP memory (L2MEM and the TCM) and PD_LP_PERI (the
 *     LP CPU and peripherals), each powered unless its state's PD_EN says
 *     otherwise, forced up by FORCE_PU (set from reset) or down by
 *     FORCE_PD. PD_LP_PERI changes only between HP_SLEEP and LP_SLEEP and
 *     is up in HP_ACTIVE (15.4.2.4). A powered-down domain's blocks are
 *     held in reset by the reset and clock control, which records the
 *     power-up reset of the cores; the memories lose their contents. The
 *     domains' masks and their FORCE_RESET, which the TRM does not
 *     describe, are logged as not modelled; isolation has no effect the
 *     model can show.
 *   - Analog: the crystal (XPD_XTAL; LP_SLEEP_XPD_XTAL in LP_SLEEP), the
 *     CPU, system, audio and SDIO PLLs (XPD_PLL bits 0 to 3), the MSPI PLL
 *     (PMU_RF_PWC's MSPI_PHY_XPD, as ESP-IDF's clk_ll_mpll_enable() has
 *     it) and the SDIO PLL again (RF_PWC's SDIO_PLL_XPD); RC_FAST
 *     (XPD_FOSC_CLK), RC32K, XTAL32K and the LP PLL. The global clock
 *     gates of the PLLs and the crystal follow their power, as the PMU
 *     opens them once WAIT_PLL_STABLE and WAIT_XTL_STABLE have passed.
 *     PD_OSC_CLK's power-down of RC_SLOW is logged as not modelled. The
 *     regulators', DCDC's and bias settings, and the PLL I2C controllers'
 *     power and isolation, have no effect the model can show.
 *   - Clocks: with ICG_SLP_SEL the PMU gates the clocks of
 *     DIG_ICG_FUNC_EN (TRM table 15.4-2) and, with ICG_SYS_CLOCK_EN clear,
 *     ROOT_CLK; with SYS_CLK_SLP_SEL it selects ROOT_CLK's source,
 *     DIG_SYS_CLK_SEL. DIG_ICG_APB_EN, whose clocks the TRM does not list,
 *     and DIG_SYS_CLK_NO_DIV are kept: the latter is logged as not
 *     modelled. In LP_SLEEP, LP_DYN_FAST_CLK follows LP_SLOW_CLK.
 *   - System control: DIG_CPU_STALL stalls both HP cores (as does
 *     PMU_CPU_SW_STALL's code 0x86 for one, ESP-IDF's esp_cpu_stall()),
 *     DIG_PAUSE_WDT pauses the RTC watchdog, DIG_PAD_SLP_SEL switches the
 *     pads to their sleep configuration, HP_PAD_HOLD_ALL and
 *     LP_PAD_HOLD_ALL hold the HP and LP pads, and UART_WAKEUP_EN enables
 *     the UARTs' wakeup. A switch that powers PD_TOP or PD_LP_PERI down
 *     also holds their pads (TRM 10.9), until software ties the hold low,
 *     as the boot ROM's pmu_enable_unhold_pads() does.
 *   - The retention DMA's backup (BACKUP_EN), done by the PAU, which is
 *     not modelled, is logged as such.
 *
 * Timing. The switches take the waits their registers give: entering
 * sleep, SWITCH_ICG_CNTL_WAIT cycles of LP_DYN_FAST_CLK and, if an HP
 * domain powers down, DG_HP_POWERDOWN_TIMER more, then, to LP_SLEEP,
 * DG_LP_POWERDOWN_TIMER if PD_LP_PERI powers down; leaving LP_SLEEP,
 * LP_ANA_WAIT_TARGET (scaled by LP_ANA_WAIT_TARGET_EXPAND) cycles of
 * LP_DYN_SLOW_CLK and DG_LP_WAIT_TIMER + DG_LP_POWERUP_TIMER if
 * PD_LP_PERI powers up; waking, ANA_WAIT_TARGET cycles of LP_DYN_SLOW_CLK
 * and WAIT_XTL_STABLE, WAIT_PLL_STABLE, DG_HP_WAIT_TIMER +
 * DG_HP_POWERUP_TIMER and MODIFY_ICG_CNTL_WAIT cycles of LP_DYN_FAST_CLK
 * as the crystal, the PLLs and the HP domains power up. The chip stays
 * asleep at least HP_MIN_SLP_VAL (LP_MIN_SLP_VAL in LP_SLEEP) cycles of
 * LP_DYN_SLOW_CLK. The TRM gives the waits but not their sequence; this
 * sums them. A wait whose clock is stopped waits for it to run. Each step
 * ends at the time it falls due, however late the QEMU timer that runs
 * it, and the next is timed from there; a switch PMU_TASK_SLEEP_REQ
 * starts is timed from its event's time, and PMU_EVT_SLEEP_WAKEUP carries
 * the time the chip woke (see esp32_line.h).
 *
 * The LP CPU (TRM 15.4.2.2, 5.10). Held in reset from power-on, it
 * starts when a wakeup source PMU_LP_CPU_WAKEUP_EN enables is active,
 * recording the sources in PMU_LP_CPU_WAKEUP_CAUSE and raising
 * PMU_LP_CPU_WAKEUP_INT (from LP_SLEEP, after the switch to HP_SLEEP): its
 * stall and interrupt mask are lifted and its clock starts (the ETM event
 * ULP_EVT_START_INTR), its reset released if it was held.
 * PMU_LP_CPU_SLEEP_REQ starts its sleep process, unless a source
 * PMU_LP_CPU_REJECT_EN enables is active (PMU_LP_CPU_REJECT_CAUSE,
 * PMU_LP_CPU_SLEEP_REJECT_INT): with SLP_STALL_EN it is stalled at once,
 * with SLP_BYPASS_INTR_EN its interrupts are masked, and after
 * SLP_STALL_WAIT cycles of its clock (LP_DYN_FAST_CLK) the clock stops
 * (ULP_EVT_HALT) and, with SLP_RESET_EN, it is held in reset, recording
 * PMU LP CPU reset (0x0A) unless LP_CLKRST_LPCORE_RESET_CAUSE_PMU_LP_CPU_MASK
 * masks it. A wakeup source active by then wakes it again at once.
 * FORCE_STALL stalls it. STALL_RDY shows it stalled and WAITI_RDY it
 * waiting for an interrupt in WFI; SLP_WAITI_FLAG_EN's wait for that is
 * logged as not modelled. The LP CPU's exception, wakeup source 16, holds
 * from the exception until software clears PMU_LP_CPU_EXC_INT_RAW, as
 * ESP-IDF does before it sleeps.
 *
 * The output regulators' waiting counters (TRM 15.4.2.9): an enabled
 * regulator (its XPD) counts LP_DYN_SLOW_CLK / 16, and raises its
 * CNT_TARGET0_REACH and CNT_TARGET1_REACH interrupts, HP and LP, as the
 * count reaches TARGET0 and TARGET1; CNT_CLR restarts it, as bit 7 of
 * VO1's TARGET0 holds it clear (its target is the 7 bits below).
 * LDO_CNT_PRESCALER_SEL, which the TRM does not describe, is logged as not
 * modelled.
 *
 * [spec:nuos:req:emu.esp32p4.sd-uhs]
 * The output regulators' voltages (TRM 15.4.2.9.1, figure 15.4-4): VO2,
 * VO3 and VO4 (P1_0P1A, P0_0P2A, P1_0P2A, ESP-IDF's ldo_ll.h order) give
 * nothing while their XPD is clear, and otherwise, in bypass mode,
 * VDD_LDO (the "vdd-ldo-mv" property, 3.3 V on the board), in LDO mode
 * VREF x MUL from their ANA register: VREF 0.5 V + 50 mV x DREF up to
 * DREF 8, then 1.0 V + 100 mV x (DREF - 9); MUL 1 + MUL / 4 (TRM, and
 * ldo_ll.h's ideal formula, the eFuse calibration constants describing
 * silicon's departure from it), no more than VDD_LDO. The mode is the
 * output of the figure's second mux, TIEH_SEL (0: LDO mode, 1: bypass),
 * its select FORCE_TIEH_SEL ? TIEH_SEL : 0 (the figure's first mux, whose
 * input 0 is a constant 0 for VO2 to VO4; ldo_ll.h: "force_tieh_sel: 0:
 * efuse, i.e. by hardware; 1: tieh_sel, i.e. by software"; the TRM's
 * register text for VO2 to VO4's FORCE_TIEH_SEL repeats TIEH_SEL's
 * options instead): 0, the regulator's TIEH (1: bypass); 2, bypass; 1
 * and 3, the SD/MMC host's card 0 and card 1 signals (TRM: SDMMC_0 and
 * SDMMC_1; ldo_ll.h: "1: sdmmc0_tieh; 2: 3.3V; 3: sdmmc1_tieh"). Those
 * signals are TIEH's kind, 1 for bypass: ldo_ll.h names them for it,
 * whose "tieh" is "1: 3.3V", and the eFuse's EFUSE_0PXA_TIEH_SEL_0 (VO1's
 * TIEH_SEL as the eFuses give it, its values the register's with bit 1
 * inverted) calls the SDMMC peripheral a source of the regulator's
 * "pull-high (tieh)". So each is its card's UHS_REG.VOLT_REG inverted:
 * VOLT_REG, "0: 3.3 V VDD, 1: 1.8 V VDD" (TRM register SDHOST_UHS_REG),
 * clear gives bypass, VDD_LDO's 3.3 V, and set LDO mode, the voltage
 * DREF and MUL give, as the TRM's example of VO4 switching an SD card's
 * pads has them (15.4.2.9.2: bypass for 3.3 V, LDO mode at 0.9 V x 2.0
 * for 1.8 V). ESP-IDF itself switches VO4 by TIEH instead
 * (sd_pwr_ctrl_by_on_chip_ldo.c, ldo_ll_set_owner()'s TIEH_SEL 0).
 * TIEH_SEL's values 4 to 7 select as their low two bits do (logged).
 * Each voltage is the "ldo-mv" output line of its channel, in mV, as
 * soon as software sets it; VO1, the flash supply, which the state machine
 * and eFuses control, is not modelled and its line not driven.
 *
 * Status: PMU_MAIN_STATE gives the states in one-hot codes, which the TRM
 * does not give but its reset values (HP_ACTIVE 0x01, HP_SLEEP 0x04 the
 * reset target) suggest: HP_ACTIVE 0x01, HP_SLEEP 0x04 and LP_SLEEP 0x08;
 * the target is the switch's, or HP_SLEEP from HP_ACTIVE and HP_ACTIVE from
 * a sleep. PMU_CLK_STATE0 to 2 show the controls in effect. PMU_PWR_STATE
 * and DCM_CUR_ST, the power controllers' and the DCDC's internal states,
 * read their reset values; PMU_POR_DONE reads 1. PMU_VDDBAT_CFG's
 * ANA_VDDBAT_MODE is the current state's VDDBAT_MODE.
 *
 * PMU_TOUCH_PWR_CNTL, the touch sensor's timer, is the touch sensor's
 * (esp32p4_pmu_touch_pwr_cntl()), told of each write ("touch-pwr").
 *
 * Kept as written, as nothing documents an effect the model could show:
 * the SDIO idle controls (no SDIO_IDLE_INT is raised), the DCDC
 * requests, ECO and calibration controls.
 *
 * The PMU is in the LP always-on domain: only a chip or system reset
 * resets it, which brings the chip up in HP_ACTIVE with every domain
 * powered, the crystal, RC_FAST and the CPU and system PLLs running, the
 * other oscillators and PLLs down, and the LP CPU held in reset.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_pmu.h"
#include "migration/vmstate.h"

#define R(a)                    ((a) / 4)

/* Each HP state's registers (pmu_hp_hw_regmap_t), at HP_STATE_BASE(m). */
#define HP_ACTIVE_MODE          0
#define HP_MODEM_MODE           1
#define HP_SLEEP_MODE           2
#define HP_STATE_BASE(m)        (0x34 * (m))
#define HP_DIG_POWER            0x00
#define HP_ICG_HP_FUNC          0x04
#define HP_ICG_HP_APB           0x08
#define HP_ICG_MODEM            0x0c
#define HP_SYS_CNTL             0x10
#define HP_CK_POWER             0x14
#define HP_BACKUP               0x1c
#define HP_SYSCLK               0x24
#define HP_XTAL                 0x30

/* Each LP state's registers (pmu_lp_hw_regmap_t), at LP_STATE_BASE(m). */
#define LP_ACTIVE_MODE          0
#define LP_SLEEP_MODE           1
#define LP_STATE_BASE(m)        (0x9c + 0x18 * (m))
#define LP_XTAL                 0x08
#define LP_DIG_POWER            0x0c
#define LP_CK_POWER             0x10

#define A_IMM_HP_CK_POWER       0x0cc
#define A_IMM_SLEEP_SYSCLK      0x0d0
#define A_IMM_HP_FUNC_ICG       0x0d4
#define A_IMM_HP_APB_ICG        0x0d8
#define A_IMM_MODEM_ICG         0x0dc
#define A_IMM_LP_ICG            0x0e0
#define A_IMM_PAD_HOLD_ALL      0x0e4
#define A_IMM_I2C_ISO           0x0e8
#define A_POWER_WAIT_TIMER0     0x0ec
#define A_POWER_WAIT_TIMER1     0x0f0
#define A_POWER_PD_TOP_CNTL     0x0f4
#define A_POWER_PD_CNNT_CNTL    0x0f8
#define A_POWER_PD_HPMEM_CNTL   0x0fc
#define A_POWER_PD_TOP_MASK     0x100
#define A_POWER_PD_CNNT_MASK    0x104
#define A_POWER_PD_HPMEM_MASK   0x108
#define A_POWER_PD_LPPERI_CNTL  0x110
#define A_POWER_PD_LPPERI_MASK  0x114
#define A_POWER_CK_WAIT_CNTL    0x11c
#define A_SLP_WAKEUP_CNTL0      0x120
#define A_SLP_WAKEUP_CNTL1      0x124
#define A_SLP_WAKEUP_CNTL2      0x128
#define A_SLP_WAKEUP_CNTL3      0x12c
#define A_SLP_WAKEUP_CNTL4      0x130
#define A_SLP_WAKEUP_CNTL5      0x134
#define A_SLP_WAKEUP_CNTL7      0x13c
#define A_SLP_WAKEUP_CNTL8      0x140
#define A_SLP_WAKEUP_STATUS0    0x144
#define A_SLP_WAKEUP_STATUS1    0x148
#define A_HP_CK_CNTL            0x154
#define A_POR_STATUS            0x158
#define A_RF_PWC                0x15c
#define A_HP_INT_RAW            0x164
#define A_HP_INT_ST             0x168
#define A_HP_INT_ENA            0x16c
#define A_HP_INT_CLR            0x170
#define A_LP_INT_RAW            0x174
#define A_LP_INT_ST             0x178
#define A_LP_INT_ENA            0x17c
#define A_LP_INT_CLR            0x180
#define A_LP_CPU_PWR0           0x184
#define A_LP_CPU_PWR1           0x188
#define A_LP_CPU_PWR2           0x18c
#define A_LP_CPU_PWR3           0x190
#define A_LP_CPU_PWR4           0x194
#define A_LP_CPU_PWR5           0x198
#define A_HP_LP_CPU_COMM        0x19c
#define A_MAIN_STATE            0x1a4
#define A_PWR_STATE             0x1a8
#define A_CLK_STATE0            0x1ac
#define A_CLK_STATE1            0x1b0
#define A_CLK_STATE2            0x1b4
#define A_EXT_LDO(n)            (0x1b8 + 8 * (n))
#define A_EXT_WAKEUP_LV         0x1e8
#define A_EXT_WAKEUP_SEL        0x1ec
#define A_EXT_WAKEUP_ST         0x1f0
#define A_EXT_WAKEUP_CNTL       0x1f4
#define A_CPU_SW_STALL          0x200
#define A_VDDBAT_CFG            0x20c
#define A_TOUCH_PWR_CNTL        0x210
#define A_POWER_PD_HP_CPU_CNTL  0x218
#define A_POWER_PD_HP_CPU_MASK  0x21c
#define A_DATE                  0x3fc

/* HP_DIG_POWER */
#define DIG_MEM_PD              BIT(23)
#define DIG_CPU_PD              BIT(29)
#define DIG_CNNT_PD             BIT(30)
#define DIG_TOP_PD              BIT(31)
#define DIG_PD_MASK             (DIG_MEM_PD | DIG_CPU_PD | DIG_CNNT_PD | \
                                 DIG_TOP_PD)
/* HP_SYS_CNTL */
#define CNTL_UART_WAKEUP_EN     BIT(24)
#define CNTL_LP_PAD_HOLD_ALL    BIT(25)
#define CNTL_HP_PAD_HOLD_ALL    BIT(26)
#define CNTL_DIG_PAD_SLP_SEL    BIT(27)
#define CNTL_DIG_PAUSE_WDT      BIT(28)
#define CNTL_DIG_CPU_STALL      BIT(29)
/* HP_CK_POWER */
#define CK_I2C_ISO_EN           BIT(21)
#define CK_I2C_RETENTION        BIT(22)
#define CK_XPD_PLL_I2C(v)       extract32((v), 23, 4)
#define CK_XPD_PLL(v)           extract32((v), 27, 4)
/* HP_BACKUP: HP_ACTIVE's SLEEP2ACTIVE_BACKUP_EN, HP_SLEEP's ACTIVE2SLEEP. */
#define BACKUP_SLEEP2ACTIVE_EN  BIT(29)
#define BACKUP_ACTIVE2SLEEP_EN  BIT(31)
/* HP_SYSCLK */
#define SYSCLK_NODIV            BIT(26)
#define SYSCLK_ICG_SYSCLK_EN    BIT(27)
#define SYSCLK_SLP_SEL          BIT(28)
#define SYSCLK_ICG_SLP_SEL      BIT(29)
#define SYSCLK_SEL(v)           extract32((v), 30, 2)
/* HP_XTAL, LP_XTAL */
#define XPD_XTAL                BIT(31)
/* LP_DIG_POWER */
#define LP_BOD_SOURCE_SEL       BIT(27)
#define LP_VDDBAT_MODE(v)       extract32((v), 28, 2)
#define LP_PERI_PD              BIT(31)
/* LP_CK_POWER */
#define LP_XPD_LPPLL            BIT(27)
#define LP_XPD_XTAL32K          BIT(28)
#define LP_XPD_RC32K            BIT(29)
#define LP_XPD_FOSC             BIT(30)
#define LP_PD_OSC               BIT(31)
/* IMM_HP_CK_POWER: the TIE_LOW fields, and TIE_HIGH's 16 bits above. */
#define IMM_GLOBAL_PLL_ICG(v)   extract32((v), 1, 4)
#define IMM_GLOBAL_XTAL_ICG     BIT(5)
#define IMM_I2C_RETENTION       BIT(6)
#define IMM_XPD_PLL_I2C(v)      extract32((v), 7, 4)
#define IMM_XPD_PLL(v)          extract32((v), 11, 4)
#define IMM_XPD_XTAL            BIT(15)
#define IMM_TIE_HIGH_SHIFT      16
/* IMM_SLEEP_SYSCLK */
#define IMM_UPDATE_ICG_SWITCH   BIT(28)
#define IMM_TIE_LOW_ICG_SLP_SEL BIT(29)
#define IMM_TIE_HIGH_ICG_SLP_SEL BIT(30)
#define IMM_UPDATE_SYSCLK_SEL   BIT(31)
/* IMM_HP_FUNC_ICG, IMM_HP_APB_ICG, IMM_MODEM_ICG */
#define IMM_UPDATE              BIT(31)
/* IMM_LP_ICG */
#define IMM_TIE_LOW_LP_ROOTCLK  BIT(30)
#define IMM_TIE_HIGH_LP_ROOTCLK BIT(31)
/* IMM_PAD_HOLD_ALL */
#define PAD_SLP_SEL_ST          BIT(0)
#define LP_PAD_HOLD_ALL_ST      BIT(1)
#define HP_PAD_HOLD_ALL_ST      BIT(2)
#define TIE_HIGH_PAD_SLP_SEL    BIT(26)
#define TIE_LOW_PAD_SLP_SEL     BIT(27)
#define TIE_HIGH_LP_PAD_HOLD    BIT(28)
#define TIE_LOW_LP_PAD_HOLD     BIT(29)
#define TIE_HIGH_HP_PAD_HOLD    BIT(30)
#define TIE_LOW_HP_PAD_HOLD     BIT(31)
/* IMM_I2C_ISO */
#define TIE_HIGH_I2C_ISO_EN     BIT(30)
#define TIE_LOW_I2C_ISO_EN      BIT(31)
/* POWER_WAIT_TIMER0/1 */
#define POWERDOWN_TIMER(v)      extract32((v), 5, 9)
#define POWERUP_TIMER(v)        extract32((v), 14, 9)
#define WAIT_TIMER(v)           extract32((v), 23, 9)
/* POWER_PD_*_CNTL */
#define FORCE_RESET             BIT(0)
#define FORCE_PU                BIT(2)
#define FORCE_NO_RESET          BIT(3)
#define FORCE_PD                BIT(5)
/* POWER_CK_WAIT_CNTL */
#define WAIT_XTL_STABLE(v)      extract32((v), 0, 16)
#define WAIT_PLL_STABLE(v)      extract32((v), 16, 16)
/* SLP_WAKEUP_* */
#define SLEEP_REQ               BIT(31)
#define SLP_REJECT_EN           BIT(31)
#define SOURCES_MASK            0x7fffffff
#define LP_MIN_SLP_VAL(v)       extract32((v), 0, 8)
#define HP_MIN_SLP_VAL(v)       extract32((v), 8, 8)
#define SLP_REJECT_CAUSE_CLR    BIT(31)
#define LP_ANA_WAIT_EXPAND(v)   extract32((v), 22, 2)
#define LP_ANA_WAIT_TARGET(v)   extract32((v), 24, 8)
#define ANA_WAIT_TARGET(v)      extract32((v), 16, 16)
#define LP_LITE_WAKEUP_ENA      BIT(31)
/* HP_CK_CNTL */
#define MODIFY_ICG_CNTL_WAIT(v) extract32((v), 0, 8)
#define SWITCH_ICG_CNTL_WAIT(v) extract32((v), 8, 8)
/* POR_STATUS */
#define POR_DONE                BIT(31)
/* RF_PWC */
#define RF_MSPI_PHY_XPD         BIT(24)
#define RF_SDIO_PLL_XPD         BIT(25)
/* HP_INT_* */
#define HP_INT_LDO_MASK         0x03ffc000
#define HP_INT_LP_CPU_EXC       BIT(27)
#define HP_INT_SDIO_IDLE        BIT(28)
#define HP_INT_SW               BIT(29)
#define HP_INT_SOC_SLEEP_REJECT BIT(30)
#define HP_INT_SOC_WAKEUP       BIT(31)
#define HP_INT_MASK             0xfbffc000
/* LP_INT_* */
#define LP_INT_LP_CPU_SLEEP_REJECT  BIT(13)
#define LP_INT_LP_CPU_WAKEUP        BIT(26)
#define LP_INT_SLEEP_SWITCH_ACTIVE_END BIT(27)
#define LP_INT_ACTIVE_SWITCH_SLEEP_END BIT(28)
#define LP_INT_SLEEP_SWITCH_ACTIVE_START BIT(29)
#define LP_INT_ACTIVE_SWITCH_SLEEP_START BIT(30)
#define LP_INT_HP_SW_TRIGGER        BIT(31)
#define LP_INT_MASK             0xffffe000
/* The regulators' counter interrupts, HP and LP, by counter and target. */
#define LDO_INT(n, k)           BIT(14 + 4 * ((n) % 3) + 2 * ((n) / 3) + (k))
/* LP_CPU_PWR0 */
#define LP_CPU_WAITI_RDY        BIT(0)
#define LP_CPU_STALL_RDY        BIT(1)
#define LP_CPU_FORCE_STALL      BIT(18)
#define LP_CPU_SLP_WAITI_FLAG_EN BIT(19)
#define LP_CPU_SLP_STALL_WAIT(v) extract32((v), 21, 8)
#define LP_CPU_SLP_STALL_EN     BIT(29)
#define LP_CPU_SLP_RESET_EN     BIT(30)
#define LP_CPU_SLP_BYPASS_INTR_EN BIT(31)
/* LP_CPU_PWR1 */
#define LP_CPU_SLEEP_REQ        BIT(31)
/* HP_LP_CPU_COMM */
#define LP_TRIGGER_HP           BIT(30)
#define HP_TRIGGER_LP           BIT(31)
/* MAIN_STATE */
#define MAIN_LAST_SHIFT         11
#define MAIN_TARGET_SHIFT       18
#define MAIN_CURRENT_SHIFT      25
#define STATE_HP_ACTIVE         0x01
#define STATE_HP_SLEEP          0x04
#define STATE_LP_SLEEP          0x08
/* EXT_LDO */
#define LDO_CNT_CLR             BIT(6)
#define LDO_XPD                 BIT(8)
#define LDO_TARGET1(v)          extract32((v), 15, 8)
#define LDO_TARGET0(v)          extract32((v), 23, 8)
#define LDO_FORCE_TIEH_SEL      BIT(7)
#define LDO_TIEH_SEL(v)         extract32((v), 9, 3)
#define LDO_TIEH                BIT(14)
#define LDO_VO1_CNT_HOLD        BIT(30)
/* EXT_LDO_*_ANA */
#define LDO_ANA_MUL(v)          extract32((v), 23, 3)
#define LDO_ANA_DREF(v)         extract32((v), 28, 4)
#define LDO_MODE_MAX_MV         2700
#define LDO_PRESCALER_SEL       BIT(31)
#define LDO_CYCLES_PER_COUNT    16
/* EXT_WAKEUP_CNTL */
#define EXT_STATUS_CLR          BIT(30)
#define EXT_FILTER              BIT(31)
/* CPU_SW_STALL */
#define SW_STALL_CODE(v, n)     extract32((v), 24 - 8 * (n), 8)
#define SW_STALL_MAGIC          0x86
/* TOUCH_PWR_CNTL */
#define TOUCH_SLEEP_TIMER_EN    BIT(31)

/* The power-on state's PLLs: the CPU and system PLLs. */
#define POWER_ON_XPD_PLL        0x3

typedef struct {
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
    uint32_t ro;
    uint32_t wtc;
    bool valid;
} PMUReg;

#define REG(a, r, w, t, o, c) \
    [R(a)] = { .reset = (r), .rw = (w), .wt = (t), .ro = (o), .wtc = (c), \
               .valid = true }

/* From pmu_reg.h: reset, R/W, WT, RO and R/WTC/SS bits of each register. */
static const PMUReg esp32p4_pmu_regs[ESP32P4_PMU_NREGS] = {
    REG(0x000, 0x00000000, 0xe0e00000, 0, 0, 0),
    REG(0x004, 0xffffffff, 0xffffffff, 0, 0, 0),
    REG(0x008, 0xffffffff, 0xffffffff, 0, 0, 0),
    REG(0x00c, 0x00000000, 0xc0000000, 0, 0, 0),
    REG(0x010, 0x00000000, 0x3f800000, 0, 0, 0),
    REG(0x014, 0x00000000, 0x7fe00000, 0, 0, 0),
    REG(0x018, 0x00500000, 0xfffc0000, 0, 0, 0),
    REG(0x01c, 0x00000000, 0x63f3dcf0, 0, 0, 0),
    REG(0x020, 0x00000000, 0xffffffff, 0, 0, 0),
    REG(0x024, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x028, 0xc6677180, 0xffff4000, 0x00008000, 0x00003ff0, 0),
    REG(0x02c, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x030, 0x80000000, 0x80000000, 0, 0, 0),
    REG(0x034, 0x00000000, 0x00200000, 0xefc00000, 0, 0),
    REG(0x038, 0x00000000, 0, 0xffffffff, 0, 0),
    REG(0x03c, 0x00000000, 0, 0xffffffff, 0, 0),
    REG(0x040, 0x00000000, 0, 0xc0000000, 0, 0),
    REG(0x044, 0x00000000, 0, 0x3f800000, 0, 0),
    REG(0x048, 0x00000000, 0, 0x7fe00000, 0, 0),
    REG(0x04c, 0x00000000, 0, 0xfffc0000, 0, 0),
    REG(0x050, 0x00000000, 0, 0x2070cc30, 0, 0),
    REG(0x054, 0x00000000, 0, 0xffffffff, 0, 0),
    REG(0x058, 0x00000000, 0, 0xfc000000, 0, 0),
    REG(0x05c, 0x00000000, 0, 0xffff0000, 0, 0),
    REG(0x060, 0x00000000, 0, 0xffffff00, 0, 0),
    REG(0x064, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x068, 0x00000000, 0xe0e00000, 0, 0, 0),
    REG(0x06c, 0xffffffff, 0xffffffff, 0, 0, 0),
    REG(0x070, 0xffffffff, 0xffffffff, 0, 0, 0),
    REG(0x074, 0x00000000, 0xc0000000, 0, 0, 0),
    REG(0x078, 0x00000000, 0x3f800000, 0, 0, 0),
    REG(0x07c, 0x00000000, 0x7fe00000, 0, 0, 0),
    REG(0x080, 0x00500000, 0xfffc0000, 0, 0, 0),
    REG(0x084, 0x00000000, 0xdf8f37c0, 0, 0, 0),
    REG(0x088, 0x00000000, 0xffffffff, 0, 0, 0),
    REG(0x08c, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x090, 0xc6670000, 0xffff0000, 0, 0, 0),
    REG(0x094, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x098, 0x80000000, 0x80000000, 0, 0, 0),
    REG(0x09c, 0xc6600000, 0xffe00000, 0, 0, 0),
    REG(0x0a0, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x0a4, 0x00000000, 0, 0xffffffff, 0, 0),
    REG(0x0a8, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x0ac, 0x40000000, 0xf8000000, 0, 0, 0),
    REG(0x0b0, 0x00000000, 0, 0xffffffff, 0, 0),
    REG(0x0b4, 0xc6600000, 0xffe00000, 0, 0, 0),
    REG(0x0b8, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x0bc, 0x80000000, 0x80000000, 0, 0, 0),
    REG(0x0c0, 0x00000000, 0xfc000000, 0, 0, 0),
    REG(0x0c4, 0x40000000, 0xf8000000, 0, 0, 0),
    REG(0x0c8, 0x00000000, 0xfe000000, 0, 0, 0),
    REG(0x0cc, 0x00000000, 0x00010001, 0xfffefffe, 0, 0),
    REG(0x0d0, 0x00000000, 0, 0xf0000000, 0, 0),
    REG(0x0d4, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x0d8, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x0dc, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x0e0, 0x00000000, 0, 0xc0000000, 0, 0),
    REG(0x0e4, 0x00000000, 0, 0xfc000000, 0x00000007, 0),
    REG(0x0e8, 0x00000000, 0, 0xc0000000, 0, 0),
    REG(0x0ec, 0x7fbfdfe0, 0xffffffe0, 0, 0, 0),
    REG(0x0f0, 0x7fbfdfe0, 0xffffffe0, 0, 0, 0),
    REG(0x0f4, 0x0000001c, 0x0000003f, 0, 0, 0),
    REG(0x0f8, 0x0000001c, 0x0000003f, 0, 0, 0),
    REG(0x0fc, 0x0000001c, 0x0000003f, 0, 0, 0),
    REG(0x100, 0x00000000, 0xf800001f, 0, 0, 0),
    REG(0x104, 0x00000000, 0xf800001f, 0, 0, 0),
    REG(0x108, 0x00000000, 0xfc00003f, 0, 0, 0),
    REG(0x10c, 0x00000001, 0x00000003, 0, 0, 0),
    REG(0x110, 0x0000001c, 0x0000003f, 0, 0, 0),
    REG(0x114, 0x00000000, 0xf800001f, 0, 0, 0),
    REG(0x118, 0x00000000, 0x00000003, 0, 0, 0),
    REG(0x11c, 0x01000100, 0xffffffff, 0, 0, 0),
    REG(0x120, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x124, 0x00000000, 0xffffffff, 0, 0, 0),
    REG(0x128, 0x00000000, 0x7fffffff, 0, 0, 0),
    REG(0x12c, 0x00000000, 0x0003ffff, 0, 0, 0),
    REG(0x130, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x134, 0x01000080, 0xffcfffff, 0, 0, 0),
    REG(0x138, 0x00000080, 0xc00fffff, 0, 0, 0),
    REG(0x13c, 0x00010000, 0xffff0000, 0, 0, 0),
    REG(0x140, 0x00000000, 0x80000000, 0, 0, 0),
    REG(0x144, 0x00000000, 0, 0, 0x7fffffff, 0),
    REG(0x148, 0x00000000, 0, 0, 0x7fffffff, 0),
    REG(0x14c, 0x00000000, 0, 0, 0x80000000, 0),
    REG(0x150, 0x00000032, 0x000000ff, 0, 0, 0),
    REG(0x154, 0x00000a0a, 0x0000ffff, 0, 0, 0),
    REG(0x158, 0x00000000, 0, 0, 0x80000000, 0),
    REG(0x15c, 0x08000000, 0x7f000000, 0, 0, 0),
    REG(0x160, 0x80000000, 0x80000000, 0, 0, 0),
    REG(0x164, 0x00000000, 0, 0, 0, HP_INT_MASK),
    REG(0x168, 0x00000000, 0, 0, HP_INT_MASK, 0),
    REG(0x16c, 0x00000000, HP_INT_MASK, 0, 0, 0),
    REG(0x170, 0x00000000, 0, HP_INT_MASK, 0, 0),
    REG(0x174, 0x00000000, 0, 0, 0, LP_INT_MASK),
    REG(0x178, 0x00000000, 0, 0, LP_INT_MASK, 0),
    REG(0x17c, 0x00000000, LP_INT_MASK, 0, 0, 0),
    REG(0x180, 0x00000000, 0, LP_INT_MASK, 0, 0),
    REG(0x184, 0x1ff00000, 0xfffc0000, 0, 0x00000003, 0),
    REG(0x188, 0x00000000, 0, 0x80000000, 0, 0),
    REG(0x18c, 0x00000000, 0x7fffffff, 0, 0, 0),
    REG(0x190, 0x00000000, 0, 0, 0x7fffffff, 0),
    REG(0x194, 0x00000000, 0x7fffffff, 0, 0, 0),
    REG(0x198, 0x00000000, 0, 0, 0x7fffffff, 0),
    REG(0x19c, 0x00000000, 0, 0xc0000000, 0, 0),
    REG(0x1a0, 0x00000000, 0x80000000, 0, 0, 0),
    REG(0x1a4, 0x02100801, 0x00000001, 0, 0xfffff800, 0),
    REG(0x1a8, 0x00802000, 0, 0, 0xffffe000, 0),
    REG(0x1ac, 0x80004000, 0, 0, 0xfbfffc7f, 0),
    REG(0x1b0, 0xffffffff, 0, 0, 0xffffffff, 0),
    REG(0x1b4, 0xffffffff, 0, 0, 0xffffffff, 0),
    REG(0x1b8, 0x40200100, 0xffffff80, 0x00000040, 0, 0),
    REG(0x1bc, 0xb1000000, 0xff800000, 0, 0, 0),
    REG(0x1c0, 0x40200000, 0xffffff80, 0x00000040, 0, 0),
    REG(0x1c4, 0xa0000000, 0xff800000, 0, 0, 0),
    REG(0x1c8, 0x40200000, 0xffffff80, 0x00000040, 0, 0),
    REG(0x1cc, 0xa0000000, 0xff800000, 0, 0, 0),
    REG(0x1d0, 0x40200000, 0xffffff80, 0x00000040, 0, 0),
    REG(0x1d4, 0xa0000000, 0xff800000, 0, 0, 0),
    REG(0x1d8, 0x40200000, 0xffffff80, 0x00000040, 0, 0),
    REG(0x1dc, 0xa0000000, 0xff800000, 0, 0, 0),
    REG(0x1e0, 0x40200000, 0xffffff80, 0x00000040, 0, 0),
    REG(0x1e4, 0xa0000000, 0xff800000, 0, 0, 0),
    REG(0x1e8, 0x00000000, 0xffffffff, 0, 0, 0),
    REG(0x1ec, 0x00000000, 0xffffffff, 0, 0, 0),
    REG(0x1f0, 0x00000000, 0, 0, 0xffffffff, 0),
    REG(0x1f4, 0x00000000, 0xc0000000, 0, 0, 0),
    REG(0x1f8, 0x000003ff, 0x000003ff, 0, 0, 0),
    REG(0x1fc, 0x000f0000, 0xffff0000, 0, 0, 0),
    REG(0x200, 0x00000000, 0xffff0000, 0, 0, 0),
    REG(0x204, 0x00010000, 0x2000ff80, 0x0000000f, 0x00ff0000, 0),
    REG(0x208, 0x004b0205, 0x03ffffff, 0, 0, 0),
    REG(0x20c, 0x00000000, 0, 0x80000000, 0x00000003, 0),
    REG(0x210, 0x00190140, 0xffffffe0, 0, 0, 0),
    REG(0x214, 0x00000000, 0x80000000, 0, 0x00000001, 0),
    REG(0x218, 0x0000001c, 0x0000003f, 0, 0, 0),
    REG(0x21c, 0x00000000, 0xf800001f, 0, 0, 0),
    REG(0x3fc, 0x02501030, 0xffffffff, 0, 0, 0),
};

static uint32_t *pmu_reg(ESP32P4PMUState *s, hwaddr a)
{
    return &s->regs[R(a)];
}

static uint32_t pmu_hp_reg(ESP32P4PMUState *s, int mode, hwaddr off)
{
    return s->regs[R(HP_STATE_BASE(mode) + off)];
}

static uint32_t pmu_lp_reg(ESP32P4PMUState *s, int mode, hwaddr off)
{
    return s->regs[R(LP_STATE_BASE(mode) + off)];
}

/*
 * Whether the LP system is asleep, with LP_SLEEP's settings in effect:
 * from the end of the switch to LP_SLEEP to the start of the switch back,
 * whose waits but the analog one count LP_DYN_FAST_CLK at LP_FAST_CLK's
 * rate (ESP-IDF's pmu_sleep_calculate_lp_hw_wait_time() has the clocks
 * switch back before the digital power-up).
 */
static bool pmu_lp_asleep(ESP32P4PMUState *s)
{
    return s->state == ESP32P4_PMU_ST_LP_SLEEP;
}

/* The HP settings in effect: HP_ACTIVE's until the chip is asleep. */
static int pmu_hp_mode(ESP32P4PMUState *s)
{
    return s->state == ESP32P4_PMU_ST_HP_ACTIVE ||
           s->state == ESP32P4_PMU_ST_ACTIVE_TO_SLEEP ? HP_ACTIVE_MODE
                                                   : HP_SLEEP_MODE;
}

static int pmu_lp_mode(ESP32P4PMUState *s)
{
    return pmu_lp_asleep(s) ? LP_SLEEP_MODE : LP_ACTIVE_MODE;
}

/* The control register of power domain @pd. */
static uint32_t pmu_pd_cntl(ESP32P4PMUState *s, int pd)
{
    static const hwaddr cntl[ESP32P4_PMU_NUM_PD] = {
        [ESP32P4_PMU_PD_TOP] = A_POWER_PD_TOP_CNTL,
        [ESP32P4_PMU_PD_HP_CPU] = A_POWER_PD_HP_CPU_CNTL,
        [ESP32P4_PMU_PD_HP_CNNT] = A_POWER_PD_CNNT_CNTL,
        [ESP32P4_PMU_PD_HP_MEM] = A_POWER_PD_HPMEM_CNTL,
        [ESP32P4_PMU_PD_LP_PERI] = A_POWER_PD_LPPERI_CNTL,
    };

    return s->regs[R(cntl[pd])];
}

/* Whether domain @pd is powered, its PD_EN @pd_en: forces win. */
static bool pmu_pd_on(ESP32P4PMUState *s, int pd, bool pd_en)
{
    uint32_t cntl = pmu_pd_cntl(s, pd);

    if (cntl & FORCE_PU) {
        return true;
    }
    return !(cntl & FORCE_PD) && !pd_en;
}

static bool pmu_domain_on(ESP32P4PMUState *s, int pd)
{
    switch (pd) {
    case ESP32P4_PMU_PD_TOP:
        return pmu_pd_on(s, pd, s->cur.top_pd);
    case ESP32P4_PMU_PD_HP_CPU:
        return pmu_pd_on(s, pd, s->cur.cpu_pd);
    case ESP32P4_PMU_PD_HP_CNNT:
        return pmu_pd_on(s, pd, s->cur.cnnt_pd);
    case ESP32P4_PMU_PD_HP_MEM:
        return pmu_pd_on(s, pd, s->cur.mem_pd);
    default:
        return pmu_pd_on(s, pd, s->cur.lp_peri_pd);
    }
}

/* Whether HP state @mode's settings power an HP domain down that is up. */
static bool pmu_hp_powers_down(ESP32P4PMUState *s, int mode)
{
    uint32_t dig = pmu_hp_reg(s, mode, HP_DIG_POWER);

    return (pmu_domain_on(s, ESP32P4_PMU_PD_TOP) &&
            !pmu_pd_on(s, ESP32P4_PMU_PD_TOP, dig & DIG_TOP_PD)) ||
           (pmu_domain_on(s, ESP32P4_PMU_PD_HP_CPU) &&
            !pmu_pd_on(s, ESP32P4_PMU_PD_HP_CPU, dig & DIG_CPU_PD)) ||
           (pmu_domain_on(s, ESP32P4_PMU_PD_HP_CNNT) &&
            !pmu_pd_on(s, ESP32P4_PMU_PD_HP_CNNT, dig & DIG_CNNT_PD)) ||
           (pmu_domain_on(s, ESP32P4_PMU_PD_HP_MEM) &&
            !pmu_pd_on(s, ESP32P4_PMU_PD_HP_MEM, dig & DIG_MEM_PD));
}

/* Whether HP state @mode's settings power an HP domain up that is down. */
static bool pmu_hp_powers_up(ESP32P4PMUState *s, int mode)
{
    uint32_t dig = pmu_hp_reg(s, mode, HP_DIG_POWER);

    return (!pmu_domain_on(s, ESP32P4_PMU_PD_TOP) &&
            pmu_pd_on(s, ESP32P4_PMU_PD_TOP, dig & DIG_TOP_PD)) ||
           (!pmu_domain_on(s, ESP32P4_PMU_PD_HP_CPU) &&
            pmu_pd_on(s, ESP32P4_PMU_PD_HP_CPU, dig & DIG_CPU_PD)) ||
           (!pmu_domain_on(s, ESP32P4_PMU_PD_HP_CNNT) &&
            pmu_pd_on(s, ESP32P4_PMU_PD_HP_CNNT, dig & DIG_CNNT_PD)) ||
           (!pmu_domain_on(s, ESP32P4_PMU_PD_HP_MEM) &&
            pmu_pd_on(s, ESP32P4_PMU_PD_HP_MEM, dig & DIG_MEM_PD));
}

/* Whether the crystal is powered: LP_SLEEP's setting, or the HP control. */
static bool pmu_xtal_on(ESP32P4PMUState *s)
{
    if (pmu_lp_asleep(s)) {
        return pmu_lp_reg(s, LP_SLEEP_MODE, LP_XTAL) & XPD_XTAL;
    }
    return s->cur.xpd_xtal;
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * The sources' levels, by TRM table 15.4-1's bits: from the blocks that
 * raise them, and the PMU's own.
 */
static uint32_t pmu_sources(ESP32P4PMUState *s)
{
    uint32_t v = s->wakeup_in;
    uint32_t sel = s->regs[R(A_EXT_WAKEUP_SEL)];
    uint32_t lv = s->regs[R(A_EXT_WAKEUP_LV)];

    if (~(s->ext_pads ^ lv) & sel & MAKE_64BIT_MASK(0, ESP32P4_PMU_EXT_PADS)) {
        v |= BIT(ESP32P4_PMU_WAKE_EXT);
    }
    if (s->regs[R(A_HP_INT_RAW)] & HP_INT_SW) {
        v |= BIT(ESP32P4_PMU_WAKE_SW);
    }
    if (s->regs[R(A_LP_INT_RAW)] & LP_INT_HP_SW_TRIGGER) {
        v |= BIT(ESP32P4_PMU_WAKE_LP_CPU);
    }
    if (s->regs[R(A_HP_INT_RAW)] & HP_INT_LP_CPU_EXC) {
        v |= BIT(ESP32P4_PMU_WAKE_LP_CPU_EXC);
    }
    return v & SOURCES_MASK;
}

/* The pads of the EXT wakeup now at their wakeup levels. */
static uint32_t pmu_ext_pads_active(ESP32P4PMUState *s)
{
    return ~(s->ext_pads ^ s->regs[R(A_EXT_WAKEUP_LV)]) &
           s->regs[R(A_EXT_WAKEUP_SEL)] &
           MAKE_64BIT_MASK(0, ESP32P4_PMU_EXT_PADS);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * Take HP state @mode's settings as the controls, as a switch to it does.
 */
static void pmu_apply_hp(ESP32P4PMUState *s, int mode)
{
    ESP32P4PMUControls *c = &s->cur;
    uint32_t dig = pmu_hp_reg(s, mode, HP_DIG_POWER);
    uint32_t cntl = pmu_hp_reg(s, mode, HP_SYS_CNTL);
    uint32_t ck = pmu_hp_reg(s, mode, HP_CK_POWER);
    uint32_t sysclk = pmu_hp_reg(s, mode, HP_SYSCLK);

    c->top_pd = dig & DIG_TOP_PD;
    c->cpu_pd = dig & DIG_CPU_PD;
    c->cnnt_pd = dig & DIG_CNNT_PD;
    c->mem_pd = dig & DIG_MEM_PD;
    c->icg_func = pmu_hp_reg(s, mode, HP_ICG_HP_FUNC);
    c->icg_apb = pmu_hp_reg(s, mode, HP_ICG_HP_APB);
    c->icg_modem_code = extract32(pmu_hp_reg(s, mode, HP_ICG_MODEM), 30, 2);
    c->uart_wakeup_en = cntl & CNTL_UART_WAKEUP_EN;
    c->lp_pad_hold = cntl & CNTL_LP_PAD_HOLD_ALL;
    c->hp_pad_hold = cntl & CNTL_HP_PAD_HOLD_ALL;
    c->pad_slp_sel = cntl & CNTL_DIG_PAD_SLP_SEL;
    c->pause_wdt = cntl & CNTL_DIG_PAUSE_WDT;
    c->cpu_stall = cntl & CNTL_DIG_CPU_STALL;
    c->i2c_iso = ck & CK_I2C_ISO_EN;
    c->i2c_retention = ck & CK_I2C_RETENTION;
    c->xpd_pll_i2c = CK_XPD_PLL_I2C(ck);
    c->xpd_pll = CK_XPD_PLL(ck);
    c->icg_global_pll = c->xpd_pll;
    c->xpd_xtal = pmu_hp_reg(s, mode, HP_XTAL) & XPD_XTAL;
    c->icg_global_xtal = c->xpd_xtal;
    c->sysclk_nodiv = sysclk & SYSCLK_NODIV;
    c->icg_sysclk_en = sysclk & SYSCLK_ICG_SYSCLK_EN;
    c->sysclk_slp_sel = sysclk & SYSCLK_SLP_SEL;
    c->icg_slp_sel = sysclk & SYSCLK_ICG_SLP_SEL;
    c->sysclk_sel = SYSCLK_SEL(sysclk);
    if (c->sysclk_nodiv) {
        qemu_log_mask(LOG_UNIMP, "%s: DIG_SYS_CLK_NO_DIV is not modelled\n",
                      TYPE_ESP32P4_PMU);
    }
    if ((mode == HP_ACTIVE_MODE &&
         (pmu_hp_reg(s, HP_ACTIVE_MODE, HP_BACKUP) &
          BACKUP_SLEEP2ACTIVE_EN)) ||
        (mode == HP_SLEEP_MODE &&
         (pmu_hp_reg(s, HP_SLEEP_MODE, HP_BACKUP) &
          BACKUP_ACTIVE2SLEEP_EN))) {
        qemu_log_mask(LOG_UNIMP, "%s: the retention DMA's backup and "
                      "restore (the PAU) is not modelled\n",
                      TYPE_ESP32P4_PMU);
    }
}

static void pmu_update_irqs(ESP32P4PMUState *s)
{
    uint32_t *r = s->regs;
    bool lp = (r[R(A_LP_INT_RAW)] & r[R(A_LP_INT_ENA)]) != 0;

    qemu_set_irq(s->irq[0], (r[R(A_HP_INT_RAW)] & r[R(A_HP_INT_ENA)]) != 0);
    qemu_set_irq(s->irq[1], lp);
    qemu_set_irq(s->lp_int, lp);
}

static void pmu_drive(qemu_irq irq, bool *last, bool level, bool force)
{
    if (force || *last != level) {
        *last = level;
        qemu_set_irq(irq, level);
    }
}

static void pmu_drive_outputs(ESP32P4PMUState *s, bool force)
{
    ESP32P4PMUControls *c = &s->cur;
    ESP32P4PMULevels *o = &s->out;
    uint32_t lp_ck = pmu_lp_reg(s, pmu_lp_mode(s), LP_CK_POWER);
    uint32_t lp_dig = pmu_lp_reg(s, pmu_lp_mode(s), LP_DIG_POWER);
    uint32_t rf = s->regs[R(A_RF_PWC)];
    uint32_t sw_stall = s->regs[R(A_CPU_SW_STALL)];
    static const int pd_order[ESP32P4_PMU_NUM_PD] = {
        ESP32P4_PMU_PD_HP_MEM, ESP32P4_PMU_PD_HP_CPU, ESP32P4_PMU_PD_HP_CNNT,
        ESP32P4_PMU_PD_LP_PERI, ESP32P4_PMU_PD_TOP,
    };
    int i, pd;

    pmu_drive(s->hold_all[ESP32P4_PMU_HOLD_HP],
              &o->hold_all[ESP32P4_PMU_HOLD_HP],
              c->hp_pad_hold || c->hp_pad_hold_auto, force);
    pmu_drive(s->hold_all[ESP32P4_PMU_HOLD_LP],
              &o->hold_all[ESP32P4_PMU_HOLD_LP],
              c->lp_pad_hold || c->lp_pad_hold_auto, force);
    for (i = 0; i < ESP32P4_PMU_NUM_PD; i++) {
        pd = pd_order[i];
        pmu_drive(s->power[pd], &s->power_level[pd], pmu_domain_on(s, pd),
                  force);
    }

    pmu_drive(s->xpd[ESP32P4_CLKRST_XPD_RC_FAST],
              &o->xpd[ESP32P4_CLKRST_XPD_RC_FAST], lp_ck & LP_XPD_FOSC,
              force);
    pmu_drive(s->xpd[ESP32P4_CLKRST_XPD_RC32K],
              &o->xpd[ESP32P4_CLKRST_XPD_RC32K], lp_ck & LP_XPD_RC32K,
              force);
    pmu_drive(s->xpd[ESP32P4_CLKRST_XPD_XTAL32K],
              &o->xpd[ESP32P4_CLKRST_XPD_XTAL32K], lp_ck & LP_XPD_XTAL32K,
              force);
    pmu_drive(s->xpd[ESP32P4_CLKRST_XPD_LP_PLL],
              &o->xpd[ESP32P4_CLKRST_XPD_LP_PLL], lp_ck & LP_XPD_LPPLL,
              force);
    pmu_drive(s->xpd[ESP32P4_CLKRST_XPD_XTAL],
              &o->xpd[ESP32P4_CLKRST_XPD_XTAL], pmu_xtal_on(s), force);
    pmu_drive(s->xtal_off, &o->xtal_off, !pmu_xtal_on(s), force);

    pmu_drive(s->pll_xpd[ESP32P4_ANA_PLL_CPLL],
              &o->pll_xpd[ESP32P4_ANA_PLL_CPLL], c->xpd_pll & BIT(0), force);
    pmu_drive(s->pll_xpd[ESP32P4_ANA_PLL_SPLL],
              &o->pll_xpd[ESP32P4_ANA_PLL_SPLL], c->xpd_pll & BIT(1), force);
    pmu_drive(s->pll_xpd[ESP32P4_ANA_PLL_APLL],
              &o->pll_xpd[ESP32P4_ANA_PLL_APLL], c->xpd_pll & BIT(2), force);
    pmu_drive(s->pll_xpd[ESP32P4_ANA_PLL_SDIO],
              &o->pll_xpd[ESP32P4_ANA_PLL_SDIO],
              (c->xpd_pll & BIT(3)) || (rf & RF_SDIO_PLL_XPD), force);
    pmu_drive(s->pll_xpd[ESP32P4_ANA_PLL_MPLL],
              &o->pll_xpd[ESP32P4_ANA_PLL_MPLL], rf & RF_MSPI_PHY_XPD,
              force);

    for (i = 0; i < ESP32P4_CLKRST_HP_CORES; i++) {
        pmu_drive(s->core_stall[i], &o->core_stall[i],
                  c->cpu_stall ||
                  SW_STALL_CODE(sw_stall, i) == SW_STALL_MAGIC, force);
    }
    for (i = 0; i < ESP32P4_CLKRST_NUM_PMU_ICG; i++) {
        pmu_drive(s->icg[i], &o->icg[i],
                  !c->icg_slp_sel || (c->icg_func & BIT(i)), force);
    }
    pmu_drive(s->sysclk[ESP32P4_CLKRST_SYSCLK_PMU_SEL],
              &o->sysclk[ESP32P4_CLKRST_SYSCLK_PMU_SEL], c->sysclk_slp_sel,
              force);
    pmu_drive(s->sysclk[ESP32P4_CLKRST_SYSCLK_SEL0],
              &o->sysclk[ESP32P4_CLKRST_SYSCLK_SEL0], c->sysclk_sel & 1,
              force);
    pmu_drive(s->sysclk[ESP32P4_CLKRST_SYSCLK_SEL1],
              &o->sysclk[ESP32P4_CLKRST_SYSCLK_SEL1], c->sysclk_sel >> 1,
              force);
    pmu_drive(s->sysclk[ESP32P4_CLKRST_SYSCLK_GATE],
              &o->sysclk[ESP32P4_CLKRST_SYSCLK_GATE],
              c->icg_slp_sel && !c->icg_sysclk_en, force);
    pmu_drive(s->lp_sleep, &o->lp_sleep, pmu_lp_asleep(s), force);

    pmu_drive(s->lp_core_reset, &o->lp_core_reset, s->lp_cpu_held, force);
    pmu_drive(s->lp_core_clock, &o->lp_core_clock, s->lp_cpu_on, force);
    pmu_drive(s->lp_core_stall, &o->lp_core_stall,
              s->lp_cpu_stalled ||
              (s->regs[R(A_LP_CPU_PWR0)] & LP_CPU_FORCE_STALL), force);

    pmu_drive(s->pad_sleep, &o->pad_sleep, c->pad_slp_sel, force);
    pmu_drive(s->pause_wdt, &o->pause_wdt, c->pause_wdt, force);
    pmu_drive(s->bod_vbat_sel, &o->bod_vbat_sel, lp_dig & LP_BOD_SOURCE_SEL,
              force);
    pmu_drive(s->uart_wakeup_en, &o->uart_wakeup_en, c->uart_wakeup_en,
              force);
    pmu_drive(s->lp_core_intr_mask, &o->lp_core_intr_mask, s->lp_cpu_masked,
              force);
    pmu_update_irqs(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * Drive what the controls, the domains and the LP CPU's state say. The
 * pads' holds go before the domains, so that a pad latches its
 * configuration before a power-down resets it; the memory goes before
 * PD_TOP, so that the machine knows the memories were lost as the HP
 * system comes back.
 */
static void pmu_update_outputs(ESP32P4PMUState *s)
{
    pmu_drive_outputs(s, false);
}

static void pmu_raise_hp(ESP32P4PMUState *s, uint32_t bits)
{
    s->regs[R(A_HP_INT_RAW)] |= bits;
}

static void pmu_raise_lp(ESP32P4PMUState *s, uint32_t bits)
{
    s->regs[R(A_LP_INT_RAW)] |= bits;
}

/* Pulse ETM event line @n, the event carrying the change's time. */
static void pmu_etm_pulse(ESP32P4PMUState *s, int n)
{
    esp32_line_set_at(s->etm_event[n], 1, esp32_line_time());
    esp32_line_set_at(s->etm_event[n], 0, esp32_line_time());
}

/* Arm the sequencer's timer for @when. */
static void pmu_seq_at(ESP32P4PMUState *s, int64_t when)
{
    s->seq_due_ns = when;
    timer_mod(&s->seq_timer, when);
}

/*
 * Time a step of a switch: @fast cycles of LP_DYN_FAST_CLK and @slow of
 * LP_DYN_SLOW_CLK from the change that starts it (esp32_line_time()), or,
 * while a clock they need is stopped, when it runs.
 */
/* When @cycles of @clk after @from have come: @from for none. */
static int64_t pmu_clk_after(Clock *clk, int64_t from, uint64_t cycles)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(clk, from);

    return esp32p4_clk_edges_after(&e, from, cycles);
}

static void pmu_seq_arm(ESP32P4PMUState *s, uint64_t fast, uint64_t slow)
{
    int64_t now = esp32_line_time(), t = now;

    s->seq_fast = fast;
    s->seq_slow = slow;
    timer_del(&s->seq_timer);
    if ((fast && !clock_is_enabled(s->fast)) ||
        (slow && !clock_is_enabled(s->slow))) {
        s->seq_waiting = true;
        return;
    }
    s->seq_waiting = false;
    if (fast) {
        t = pmu_clk_after(s->fast, t, fast);
    }
    if (slow) {
        t = pmu_clk_after(s->slow, t, slow);
    }
    pmu_seq_at(s, MAX(t, now + 1));
}

/* The fast cycles a wake to HP_ACTIVE waits besides ANA_WAIT_TARGET. */
static uint64_t pmu_wake_fast_cycles(ESP32P4PMUState *s)
{
    uint32_t ck_wait = s->regs[R(A_POWER_CK_WAIT_CNTL)];
    uint32_t wait0 = s->regs[R(A_POWER_WAIT_TIMER0)];
    uint32_t active_ck = pmu_hp_reg(s, HP_ACTIVE_MODE, HP_CK_POWER);
    uint64_t fast = MODIFY_ICG_CNTL_WAIT(s->regs[R(A_HP_CK_CNTL)]);

    if (!pmu_xtal_on(s) &&
        (pmu_hp_reg(s, HP_ACTIVE_MODE, HP_XTAL) & XPD_XTAL)) {
        fast += WAIT_XTL_STABLE(ck_wait);
    }
    if (CK_XPD_PLL(active_ck) & ~s->cur.xpd_pll) {
        fast += WAIT_PLL_STABLE(ck_wait);
    }
    if (pmu_hp_powers_up(s, HP_ACTIVE_MODE)) {
        fast += WAIT_TIMER(wait0) + POWERUP_TIMER(wait0);
    }
    return fast;
}

static void pmu_check(ESP32P4PMUState *s);

/*
 * Start the switch from LP_SLEEP back to HP_SLEEP, @target the state it
 * goes on to: the analog wait in LP_DYN_SLOW_CLK cycles, then, with
 * LP_DYN_FAST_CLK back on LP_FAST_CLK, PD_LP_PERI's power-up.
 */
static void pmu_start_lp_wake(ESP32P4PMUState *s, int target)
{
    uint32_t cntl5 = s->regs[R(A_SLP_WAKEUP_CNTL5)];
    uint32_t wait1 = s->regs[R(A_POWER_WAIT_TIMER1)];
    uint64_t fast = 0;

    s->target_state = target;
    s->state = ESP32P4_PMU_ST_LP_TO_HP;
    if (!pmu_domain_on(s, ESP32P4_PMU_PD_LP_PERI) &&
        pmu_pd_on(s, ESP32P4_PMU_PD_LP_PERI,
                  pmu_lp_reg(s, LP_ACTIVE_MODE, LP_DIG_POWER) &
                  LP_PERI_PD)) {
        fast = WAIT_TIMER(wait1) + POWERUP_TIMER(wait1);
    }
    pmu_update_outputs(s);
    pmu_seq_arm(s, fast, LP_ANA_WAIT_TARGET(cntl5) <<
                         LP_ANA_WAIT_EXPAND(cntl5));
}

/* Start the wake of the HP system from a sleep state, @sources waking it. */
static void pmu_start_wake(ESP32P4PMUState *s, uint32_t sources)
{
    s->regs[R(A_SLP_WAKEUP_STATUS0)] = sources;
    if (sources & BIT(ESP32P4_PMU_WAKE_EXT)) {
        uint32_t pads = pmu_ext_pads_active(s);

        if (!(s->regs[R(A_EXT_WAKEUP_CNTL)] & EXT_STATUS_CLR)) {
            s->regs[R(A_EXT_WAKEUP_ST)] |= pads;
        }
        if (s->lp_sys) {
            esp32p4_lp_sys_ext_wakeup1(s->lp_sys, pads);
        }
    }
    pmu_raise_lp(s, LP_INT_SLEEP_SWITCH_ACTIVE_START);
    if (s->state == ESP32P4_PMU_ST_LP_SLEEP) {
        pmu_start_lp_wake(s, ESP32P4_PMU_ST_HP_ACTIVE);
    } else {
        s->target_state = ESP32P4_PMU_ST_HP_ACTIVE;
        s->state = ESP32P4_PMU_ST_SLEEP_TO_ACTIVE;
        pmu_seq_arm(s, pmu_wake_fast_cycles(s),
                    ANA_WAIT_TARGET(s->regs[R(A_SLP_WAKEUP_CNTL7)]));
    }
    pmu_update_irqs(s);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void pmu_lp_cpu_wake(ESP32P4PMUState *s, uint32_t sources)
{
    s->regs[R(A_LP_CPU_PWR3)] = sources;
    s->lp_cpu_on = true;
    s->lp_cpu_held = false;
    s->lp_cpu_stalled = false;
    s->lp_cpu_masked = false;
    pmu_raise_lp(s, LP_INT_LP_CPU_WAKEUP);
    pmu_etm_pulse(s, ESP32P4_PMU_ETM_EVT_LP_CPU_START);
}

/* Switch from HP_SLEEP to LP_SLEEP, the LP CPU being asleep. */
static void pmu_start_lp_sleep(ESP32P4PMUState *s)
{
    uint32_t wait1 = s->regs[R(A_POWER_WAIT_TIMER1)];
    uint64_t fast = 1;

    if (pmu_domain_on(s, ESP32P4_PMU_PD_LP_PERI) &&
        !pmu_pd_on(s, ESP32P4_PMU_PD_LP_PERI,
                   pmu_lp_reg(s, LP_SLEEP_MODE, LP_DIG_POWER) &
                   LP_PERI_PD)) {
        fast += POWERDOWN_TIMER(wait1);
    }
    s->target_state = ESP32P4_PMU_ST_LP_SLEEP;
    s->state = ESP32P4_PMU_ST_HP_TO_LP;
    pmu_seq_arm(s, fast, 0);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * In a stable state, act on the sources: wake the HP system or the LP
 * CPU, or put the chip into LP_SLEEP.
 */
static void pmu_check(ESP32P4PMUState *s)
{
    uint32_t src = pmu_sources(s);
    uint32_t wake = src & s->regs[R(A_SLP_WAKEUP_CNTL2)];
    uint32_t lp_wake = src & s->regs[R(A_LP_CPU_PWR2)];
    uint32_t cntl3 = s->regs[R(A_SLP_WAKEUP_CNTL3)];
    int64_t now = esp32_line_time();
    int64_t earliest;

    if (!s->lp_cpu_on && lp_wake && s->state != ESP32P4_PMU_ST_LP_SLEEP &&
        s->state != ESP32P4_PMU_ST_HP_TO_LP &&
        s->state != ESP32P4_PMU_ST_LP_TO_HP) {
        pmu_lp_cpu_wake(s, lp_wake);
    }

    switch (s->state) {
    case ESP32P4_PMU_ST_HP_SLEEP:
        earliest = s->sleep_start_ns;
        if (clock_is_enabled(s->slow)) {
            earliest = pmu_clk_after(s->slow, earliest,
                                     HP_MIN_SLP_VAL(cntl3));
        }
        if (wake && now >= earliest) {
            pmu_start_wake(s, wake);
        } else if (!s->lp_cpu_on) {
            pmu_start_lp_sleep(s);
        } else if (wake) {
            pmu_seq_at(s, earliest);
        }
        break;
    case ESP32P4_PMU_ST_LP_SLEEP:
        if (wake || (!s->lp_cpu_on && lp_wake)) {
            earliest = s->sleep_start_ns;
            if (clock_is_enabled(s->slow)) {
                earliest = pmu_clk_after(s->slow, earliest,
                                         MAX(HP_MIN_SLP_VAL(cntl3),
                                             LP_MIN_SLP_VAL(cntl3)));
            }
            if (now < earliest) {
                pmu_seq_at(s, earliest);
                return;
            }
            if (wake) {
                pmu_start_wake(s, wake);
            } else {
                pmu_start_lp_wake(s, ESP32P4_PMU_ST_HP_SLEEP);
            }
        }
        break;
    default:
        break;
    }
}

/* Whether a reject-enabled source rejects the HP system's sleep now. */
static uint32_t pmu_rejection(ESP32P4PMUState *s)
{
    uint32_t cntl1 = s->regs[R(A_SLP_WAKEUP_CNTL1)];

    if (!(cntl1 & SLP_REJECT_EN)) {
        return 0;
    }
    return pmu_sources(s) & cntl1 & SOURCES_MASK;
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void pmu_reject(ESP32P4PMUState *s, uint32_t cause)
{
    s->regs[R(A_SLP_WAKEUP_STATUS1)] |= cause;
    pmu_raise_hp(s, HP_INT_SOC_SLEEP_REJECT);
    s->target_state = ESP32P4_PMU_ST_HP_SLEEP;
    s->state = ESP32P4_PMU_ST_HP_ACTIVE;
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * PMU_SLEEP_REQ: start the switch to sleep, unless a source rejects it.
 */
static void pmu_sleep_request(ESP32P4PMUState *s)
{
    uint32_t cause;
    uint64_t fast;

    if (s->state != ESP32P4_PMU_ST_HP_ACTIVE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: sleep requested while not in "
                      "HP_ACTIVE\n", TYPE_ESP32P4_PMU);
        return;
    }
    s->regs[R(A_SLP_WAKEUP_STATUS0)] = 0;
    if (s->regs[R(A_SLP_WAKEUP_CNTL8)] & LP_LITE_WAKEUP_ENA) {
        qemu_log_mask(LOG_UNIMP, "%s: the LP lite wakeup is not "
                      "modelled\n", TYPE_ESP32P4_PMU);
    }
    cause = pmu_rejection(s);
    if (cause) {
        pmu_reject(s, cause);
        pmu_update_outputs(s);
        return;
    }
    pmu_raise_lp(s, LP_INT_ACTIVE_SWITCH_SLEEP_START);
    s->last_state = STATE_HP_ACTIVE;
    s->target_state = s->lp_cpu_on ? ESP32P4_PMU_ST_HP_SLEEP
                                    : ESP32P4_PMU_ST_LP_SLEEP;
    s->state = ESP32P4_PMU_ST_ACTIVE_TO_SLEEP;
    fast = SWITCH_ICG_CNTL_WAIT(s->regs[R(A_HP_CK_CNTL)]);
    if (pmu_hp_powers_down(s, HP_SLEEP_MODE)) {
        fast += POWERDOWN_TIMER(s->regs[R(A_POWER_WAIT_TIMER0)]);
    }
    pmu_seq_arm(s, MAX(fast, 1), 0);
    pmu_update_outputs(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * A step of a switch ends, or, in a sleep state, the minimum sleep, at
 * the time it falls due, esp32_line_time().
 */
static void pmu_seq_step(void *opaque)
{
    ESP32P4PMUState *s = opaque;
    uint32_t cause;

    switch (s->state) {
    case ESP32P4_PMU_ST_ACTIVE_TO_SLEEP:
        cause = pmu_rejection(s);
        if (cause) {
            pmu_reject(s, cause);
            break;
        }
        pmu_apply_hp(s, HP_SLEEP_MODE);
        if (!pmu_domain_on(s, ESP32P4_PMU_PD_TOP)) {
            s->cur.hp_pad_hold_auto = true;
        }
        s->state = ESP32P4_PMU_ST_HP_SLEEP;
        s->sleep_start_ns = esp32_line_time();
        pmu_raise_lp(s, LP_INT_ACTIVE_SWITCH_SLEEP_END);
        pmu_check(s);
        break;
    case ESP32P4_PMU_ST_HP_TO_LP:
        s->cur.lp_peri_pd = pmu_lp_reg(s, LP_SLEEP_MODE, LP_DIG_POWER) &
                            LP_PERI_PD;
        if (!pmu_domain_on(s, ESP32P4_PMU_PD_LP_PERI)) {
            s->cur.lp_pad_hold_auto = true;
        }
        s->last_state = STATE_HP_SLEEP;
        s->state = ESP32P4_PMU_ST_LP_SLEEP;
        pmu_check(s);
        break;
    case ESP32P4_PMU_ST_LP_TO_HP:
        s->cur.lp_peri_pd = pmu_lp_reg(s, LP_ACTIVE_MODE, LP_DIG_POWER) &
                            LP_PERI_PD;
        s->last_state = STATE_LP_SLEEP;
        s->state = ESP32P4_PMU_ST_HP_SLEEP;
        if (s->target_state == ESP32P4_PMU_ST_HP_ACTIVE) {
            s->state = ESP32P4_PMU_ST_SLEEP_TO_ACTIVE;
            pmu_seq_arm(s, pmu_wake_fast_cycles(s),
                        ANA_WAIT_TARGET(s->regs[R(A_SLP_WAKEUP_CNTL7)]));
            break;
        }
        if (!s->lp_cpu_on) {
            pmu_lp_cpu_wake(s, pmu_sources(s) &
                               s->regs[R(A_LP_CPU_PWR2)]);
        }
        pmu_check(s);
        break;
    case ESP32P4_PMU_ST_SLEEP_TO_ACTIVE:
        pmu_apply_hp(s, HP_ACTIVE_MODE);
        s->cur.lp_peri_pd = false;
        s->last_state = STATE_HP_SLEEP;
        s->target_state = ESP32P4_PMU_ST_HP_SLEEP;
        s->state = ESP32P4_PMU_ST_HP_ACTIVE;
        pmu_raise_lp(s, LP_INT_SLEEP_SWITCH_ACTIVE_END);
        pmu_raise_hp(s, HP_INT_SOC_WAKEUP);
        pmu_etm_pulse(s, ESP32P4_PMU_ETM_EVT_SLEEP_WAKEUP);
        pmu_check(s);
        break;
    default:
        pmu_check(s);
        break;
    }
    pmu_update_outputs(s);
}

/* The step falls due: end it at its time, which a late timer has passed. */
static void pmu_seq_cb(void *opaque)
{
    ESP32P4PMUState *s = opaque;

    esp32_line_run_at(s->seq_due_ns, pmu_seq_step, s);
}

/*
 * Time the LP CPU's sleep process: PMU_LP_CPU_SLP_STALL_WAIT cycles of its
 * clock, LP_DYN_FAST_CLK, from the change that starts it
 * (esp32_line_time()), or, while that clock is stopped, from when it runs.
 */
static void pmu_lp_sleep_arm(ESP32P4PMUState *s)
{
    uint32_t cycles = LP_CPU_SLP_STALL_WAIT(s->regs[R(A_LP_CPU_PWR0)]);

    timer_del(&s->lp_sleep_timer);
    s->lp_sleep_waiting = !clock_is_enabled(s->fast);
    if (s->lp_sleep_waiting) {
        return;
    }
    s->lp_sleep_due_ns = MAX(pmu_clk_after(s->fast, esp32_line_time(),
                                           cycles),
                             esp32_line_time() + 1);
    timer_mod(&s->lp_sleep_timer, s->lp_sleep_due_ns);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * [spec:nuos:req:emu.esp32p4.lp-core]
 * PMU_LP_CPU_SLEEP_REQ: start the LP CPU's sleep process, unless a source
 * rejects it, stalling it and masking its interrupts as the process says.
 */
static void pmu_lp_cpu_sleep_request(ESP32P4PMUState *s)
{
    uint32_t pwr0 = s->regs[R(A_LP_CPU_PWR0)];
    uint32_t reject = pmu_sources(s) & s->regs[R(A_LP_CPU_PWR4)];

    if (!s->lp_cpu_on || s->lp_cpu_sleeping) {
        return;
    }
    if (reject) {
        s->regs[R(A_LP_CPU_PWR5)] |= reject;
        pmu_raise_lp(s, LP_INT_LP_CPU_SLEEP_REJECT);
        return;
    }
    if (pwr0 & LP_CPU_SLP_WAITI_FLAG_EN) {
        qemu_log_mask(LOG_UNIMP, "%s: LP_CPU_SLP_WAITI_FLAG_EN's wait for "
                      "the LP CPU's WFI is not modelled\n", TYPE_ESP32P4_PMU);
    }
    s->lp_cpu_sleeping = true;
    s->lp_cpu_stalled = pwr0 & LP_CPU_SLP_STALL_EN;
    s->lp_cpu_masked = pwr0 & LP_CPU_SLP_BYPASS_INTR_EN;
    pmu_lp_sleep_arm(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * [spec:nuos:req:emu.esp32p4.lp-core]
 * The LP CPU's sleep process ends: its clock stops and, with
 * SLP_RESET_EN, its reset is held; then, with the HP system asleep, the
 * chip goes on to LP_SLEEP. A wakeup source already active wakes it at
 * once, after it has stopped, so that it starts again from its reset.
 */
static void pmu_lp_sleep_step(void *opaque)
{
    ESP32P4PMUState *s = opaque;

    s->lp_cpu_sleeping = false;
    s->lp_cpu_on = false;
    if (s->regs[R(A_LP_CPU_PWR0)] & LP_CPU_SLP_RESET_EN) {
        s->lp_cpu_held = true;
        esp32p4_clkrst_record_reset(s->clkrst, ESP32P4_CLKRST_CORE_LP,
                                    ESP32P4_RESET_CAUSE_PMU_LP_CPU);
    }
    pmu_etm_pulse(s, ESP32P4_PMU_ETM_EVT_LP_CPU_HALT);
    pmu_update_outputs(s);
    pmu_check(s);
    pmu_update_outputs(s);
}

/* The sleep process's wait ends at its due time. */
static void pmu_lp_sleep_cb(void *opaque)
{
    ESP32P4PMUState *s = opaque;

    esp32_line_run_at(s->lp_sleep_due_ns, pmu_lp_sleep_step, s);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void pmu_imm_hp_ck_power(ESP32P4PMUState *s, uint32_t v)
{
    ESP32P4PMUControls *c = &s->cur;
    uint32_t hi = v >> IMM_TIE_HIGH_SHIFT;

    c->icg_global_pll &= ~IMM_GLOBAL_PLL_ICG(v);
    c->xpd_pll_i2c &= ~IMM_XPD_PLL_I2C(v);
    c->xpd_pll &= ~IMM_XPD_PLL(v);
    if (v & IMM_GLOBAL_XTAL_ICG) {
        c->icg_global_xtal = false;
    }
    if (v & IMM_I2C_RETENTION) {
        c->i2c_retention = false;
    }
    if (v & IMM_XPD_XTAL) {
        c->xpd_xtal = false;
    }
    c->icg_global_pll |= IMM_GLOBAL_PLL_ICG(hi);
    c->xpd_pll_i2c |= IMM_XPD_PLL_I2C(hi);
    c->xpd_pll |= IMM_XPD_PLL(hi);
    if (hi & IMM_GLOBAL_XTAL_ICG) {
        c->icg_global_xtal = true;
    }
    if (hi & IMM_I2C_RETENTION) {
        c->i2c_retention = true;
    }
    if (hi & IMM_XPD_XTAL) {
        c->xpd_xtal = true;
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void pmu_imm_sleep_sysclk(ESP32P4PMUState *s, uint32_t v)
{
    ESP32P4PMUControls *c = &s->cur;
    uint32_t sysclk = pmu_hp_reg(s, pmu_hp_mode(s), HP_SYSCLK);

    if (v & IMM_UPDATE_ICG_SWITCH) {
        c->icg_slp_sel = sysclk & SYSCLK_ICG_SLP_SEL;
        c->icg_sysclk_en = sysclk & SYSCLK_ICG_SYSCLK_EN;
    }
    if (v & IMM_TIE_LOW_ICG_SLP_SEL) {
        c->icg_slp_sel = false;
    }
    if (v & IMM_TIE_HIGH_ICG_SLP_SEL) {
        c->icg_slp_sel = true;
    }
    if (v & IMM_UPDATE_SYSCLK_SEL) {
        c->sysclk_slp_sel = sysclk & SYSCLK_SLP_SEL;
        c->sysclk_sel = SYSCLK_SEL(sysclk);
        c->sysclk_nodiv = sysclk & SYSCLK_NODIV;
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void pmu_imm_pad_hold_all(ESP32P4PMUState *s, uint32_t v)
{
    ESP32P4PMUControls *c = &s->cur;

    if (v & TIE_LOW_PAD_SLP_SEL) {
        c->pad_slp_sel = false;
    }
    if (v & TIE_HIGH_PAD_SLP_SEL) {
        c->pad_slp_sel = true;
    }
    if (v & TIE_LOW_LP_PAD_HOLD) {
        c->lp_pad_hold = false;
        c->lp_pad_hold_auto = false;
    }
    if (v & TIE_HIGH_LP_PAD_HOLD) {
        c->lp_pad_hold = true;
    }
    if (v & TIE_LOW_HP_PAD_HOLD) {
        c->hp_pad_hold = false;
        c->hp_pad_hold_auto = false;
    }
    if (v & TIE_HIGH_HP_PAD_HOLD) {
        c->hp_pad_hold = true;
    }
}

/*
 * The regulators' waiting counters. Counter @n counts LP_DYN_SLOW_CLK
 * cycles since start_ns, a target of T counts reached at 16 T cycles.
 */
static uint32_t pmu_ldo_target(ESP32P4PMUState *s, int n, int k)
{
    uint32_t v = s->regs[R(A_EXT_LDO(n))];

    if (k == 0) {
        return n == 0 ? LDO_TARGET0(v) & 0x7f : LDO_TARGET0(v);
    }
    return LDO_TARGET1(v);
}

static bool pmu_ldo_enabled(ESP32P4PMUState *s, int n)
{
    uint32_t v = s->regs[R(A_EXT_LDO(n))];

    return (v & LDO_XPD) && !(n == 0 && (v & LDO_VO1_CNT_HOLD));
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void pmu_ldo_schedule(ESP32P4PMUState *s, int n)
{
    ESP32P4PMULdo *l = &s->ldo[n];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ESP32P4ClkEdges slow = esp32p4_clk_edges_or_at(s->slow, l->start_ns);
    uint64_t cycles, due = UINT64_MAX;
    int k;

    timer_del(&l->timer);
    if (!l->running || !clock_is_enabled(s->slow)) {
        return;
    }
    cycles = esp32p4_clk_edges_between(&slow, l->start_ns, now);
    for (k = 0; k < 2; k++) {
        uint64_t at = (uint64_t)pmu_ldo_target(s, n, k) * LDO_CYCLES_PER_COUNT;

        if (l->reached[k]) {
            continue;
        }
        if (cycles >= at) {
            l->reached[k] = true;
            pmu_raise_hp(s, LDO_INT(n, k));
            pmu_raise_lp(s, LDO_INT(n, k));
            continue;
        }
        due = MIN(due, at);
    }
    if (due != UINT64_MAX) {
        timer_mod(&l->timer, esp32p4_clk_edges_after(&slow, l->start_ns,
                                                     due));
    }
    pmu_update_irqs(s);
}

/* Start, restart or stop counter @n as its regulator's enable says. */
static void pmu_ldo_update(ESP32P4PMUState *s, int n, bool restart)
{
    ESP32P4PMULdo *l = &s->ldo[n];
    bool en = pmu_ldo_enabled(s, n);

    if (en && (!l->running || restart)) {
        l->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        l->reached[0] = false;
        l->reached[1] = false;
    }
    l->running = en;
    if (s->regs[R(A_EXT_LDO(n))] & LDO_PRESCALER_SEL) {
        qemu_log_mask(LOG_UNIMP, "%s: LDO_CNT_PRESCALER_SEL is not "
                      "modelled\n", TYPE_ESP32P4_PMU);
    }
    pmu_ldo_schedule(s, n);
}

static void pmu_ldo_cb(void *opaque)
{
    ESP32P4PMULdo *l = opaque;

    pmu_ldo_schedule(l->s, l - l->s->ldo);
}

/* The EXT_LDO register pair of regulator VO@ch + 1 (ldo_ll.h's order) */
static const int8_t pmu_vo_ldo[ESP32P4_PMU_LDO_CHANNELS] = { -1, 3, 1, 4 };

/* [spec:nuos:req:emu.esp32p4.sd-uhs] */
/* Regulator VO@ch + 1's output, mV; whether in LDO mode in *@ldo_mode. */
static uint32_t pmu_vo_mv(ESP32P4PMUState *s, unsigned ch, bool *ldo_mode)
{
    int n = pmu_vo_ldo[ch];
    uint32_t v = s->regs[R(A_EXT_LDO(n))];
    uint32_t ana = s->regs[R(A_EXT_LDO(n) + 4)];
    unsigned sel = (v & LDO_FORCE_TIEH_SEL) ? LDO_TIEH_SEL(v) & 3 : 0;
    uint32_t dref = LDO_ANA_DREF(ana), vref;
    bool bypass;

    *ldo_mode = false;
    if (!(v & LDO_XPD)) {
        return 0;
    }
    switch (sel) {
    case 0:
        bypass = v & LDO_TIEH;
        break;
    case 1:
        bypass = !(s->sdmmc_volt & 1);
        break;
    case 2:
        bypass = true;
        break;
    default:
        bypass = !(s->sdmmc_volt & 2);
        break;
    }
    if (bypass) {
        return s->vdd_ldo_mv;
    }
    *ldo_mode = true;
    vref = dref < 9 ? 500 + 50 * dref : 1000 + 100 * (dref - 9);
    return MIN(vref * (4 + LDO_ANA_MUL(ana)) / 4, s->vdd_ldo_mv);
}

/* [spec:nuos:req:emu.esp32p4.sd-uhs] */
static void pmu_vo_drive(ESP32P4PMUState *s, bool force)
{
    for (unsigned ch = 1; ch < ESP32P4_PMU_LDO_CHANNELS; ch++) {
        uint32_t v = s->regs[R(A_EXT_LDO(pmu_vo_ldo[ch]))];
        bool ldo_mode;
        uint32_t mv = pmu_vo_mv(s, ch, &ldo_mode);

        if (mv == s->vo_mv[ch] && !force) {
            continue;
        }
        if ((v & LDO_FORCE_TIEH_SEL) && LDO_TIEH_SEL(v) > 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: VO%u's TIEH_SEL %u, which "
                          "the mux does not have: taken as %u\n",
                          TYPE_ESP32P4_PMU, ch + 1, LDO_TIEH_SEL(v),
                          LDO_TIEH_SEL(v) & 3);
        }
        if (ldo_mode && mv > LDO_MODE_MAX_MV) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: VO%u set to %u mV in LDO "
                          "mode, beyond its 2.7 V\n", TYPE_ESP32P4_PMU,
                          ch + 1, mv);
        }
        s->vo_mv[ch] = mv;
        qemu_set_irq(s->ldo_mv[ch], mv);
    }
}

/* The SD/MMC host's UHS_REG.VOLT_REG bit for card @n. */
static void esp32p4_pmu_sdmmc_volt(void *opaque, int n, int level)
{
    ESP32P4PMUState *s = opaque;

    s->sdmmc_volt = deposit32(s->sdmmc_volt, n, 1, level != 0);
    pmu_vo_drive(s, false);
}

/* The value a read of register @reg gives. */
static uint32_t pmu_value(ESP32P4PMUState *s, hwaddr reg)
{
    ESP32P4PMUControls *c = &s->cur;
    uint32_t *r = s->regs;
    static const uint8_t code[] = {
        [ESP32P4_PMU_ST_HP_ACTIVE] = STATE_HP_ACTIVE,
        [ESP32P4_PMU_ST_ACTIVE_TO_SLEEP] = STATE_HP_ACTIVE,
        [ESP32P4_PMU_ST_HP_SLEEP] = STATE_HP_SLEEP,
        [ESP32P4_PMU_ST_HP_TO_LP] = STATE_HP_SLEEP,
        [ESP32P4_PMU_ST_LP_SLEEP] = STATE_LP_SLEEP,
        [ESP32P4_PMU_ST_LP_TO_HP] = STATE_LP_SLEEP,
        [ESP32P4_PMU_ST_SLEEP_TO_ACTIVE] = STATE_HP_SLEEP,
    };
    uint32_t v, tar;

    switch (reg) {
    case A_HP_INT_ST:
        return r[R(A_HP_INT_RAW)] & r[R(A_HP_INT_ENA)];
    case A_LP_INT_ST:
        return r[R(A_LP_INT_RAW)] & r[R(A_LP_INT_ENA)];
    case A_IMM_PAD_HOLD_ALL:
        return (c->pad_slp_sel ? PAD_SLP_SEL_ST : 0) |
               (c->lp_pad_hold || c->lp_pad_hold_auto ? LP_PAD_HOLD_ALL_ST
                                                     : 0) |
               (c->hp_pad_hold || c->hp_pad_hold_auto ? HP_PAD_HOLD_ALL_ST
                                                     : 0);
    case A_POR_STATUS:
        return POR_DONE;
    case A_LP_CPU_PWR0:
        v = r[R(reg)];
        if (s->lp_cpu_stalled || (v & LP_CPU_FORCE_STALL)) {
            v |= LP_CPU_STALL_RDY;
        }
        if (s->lp_cpu && s->lp_cpu_on && !s->lp_cpu_held &&
            object_property_get_bool(OBJECT(s->lp_cpu), "in-wfi", NULL)) {
            v |= LP_CPU_WAITI_RDY;
        }
        return v;
    case A_MAIN_STATE:
        switch (s->state) {
        case ESP32P4_PMU_ST_HP_ACTIVE:
            tar = STATE_HP_SLEEP;
            break;
        case ESP32P4_PMU_ST_HP_SLEEP:
        case ESP32P4_PMU_ST_LP_SLEEP:
            tar = STATE_HP_ACTIVE;
            break;
        default:
            tar = code[s->target_state];
            break;
        }
        return (r[R(reg)] & 1) | (s->last_state << MAIN_LAST_SHIFT) |
               (tar << MAIN_TARGET_SHIFT) |
               (code[s->state] << MAIN_CURRENT_SHIFT);
    case A_CLK_STATE0:
        return extract32(c->xpd_pll, 0, 3) |
               (pmu_xtal_on(s) ? BIT(3) : 0) |
               (extract32(c->xpd_pll_i2c, 0, 3) << 4) |
               (c->sysclk_slp_sel ? BIT(10) : 0) |
               (c->sysclk_sel << 11) |
               (c->sysclk_nodiv ? BIT(13) : 0) |
               (c->icg_sysclk_en ? BIT(14) : 0) |
               (c->icg_modem_code << 16) |
               (c->icg_slp_sel ? BIT(18) : 0) |
               (c->icg_global_xtal ? BIT(19) : 0) |
               (c->icg_global_pll << 20) |
               (c->i2c_iso ? BIT(24) : 0) |
               (c->i2c_retention ? BIT(25) : 0) |
               (c->xpd_pll << 27) |
               (pmu_xtal_on(s) ? BIT(31) : 0);
    case A_CLK_STATE1:
        return c->icg_func;
    case A_CLK_STATE2:
        return c->icg_apb;
    case A_VDDBAT_CFG:
        return LP_VDDBAT_MODE(pmu_lp_reg(s, pmu_lp_mode(s), LP_DIG_POWER));
    default:
        return r[R(reg)];
    }
}

/* [spec:nuos:req:emu.esp32p4.analog] */
uint32_t esp32p4_pmu_touch_pwr_cntl(ESP32P4PMUState *s)
{
    return s->regs[R(A_TOUCH_PWR_CNTL)];
}

static uint64_t esp32p4_pmu_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4PMUState *s = opaque;
    hwaddr reg = addr & ~3;

    if (!esp32p4_pmu_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_PMU, size,
                      addr);
        return 0;
    }
    return extract32(pmu_value(s, reg), (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void pmu_log_unmodelled(ESP32P4PMUState *s, hwaddr reg, uint32_t v)
{
    switch (reg) {
    case A_POWER_PD_TOP_CNTL:
    case A_POWER_PD_CNNT_CNTL:
    case A_POWER_PD_HPMEM_CNTL:
    case A_POWER_PD_LPPERI_CNTL:
    case A_POWER_PD_HP_CPU_CNTL:
        if ((v & FORCE_RESET) && !(v & FORCE_NO_RESET)) {
            qemu_log_mask(LOG_UNIMP, "%s: a power domain's FORCE_RESET is "
                          "not modelled\n", TYPE_ESP32P4_PMU);
        }
        break;
    case A_POWER_PD_TOP_MASK:
    case A_POWER_PD_CNNT_MASK:
    case A_POWER_PD_HPMEM_MASK:
    case A_POWER_PD_LPPERI_MASK:
    case A_POWER_PD_HP_CPU_MASK:
        if (v) {
            qemu_log_mask(LOG_UNIMP, "%s: the power domains' masks are not "
                          "modelled\n", TYPE_ESP32P4_PMU);
        }
        break;
    case LP_STATE_BASE(LP_ACTIVE_MODE) + LP_CK_POWER:
    case LP_STATE_BASE(LP_SLEEP_MODE) + LP_CK_POWER:
        if (v & LP_PD_OSC) {
            qemu_log_mask(LOG_UNIMP, "%s: powering RC_SLOW down "
                          "(PD_OSC_CLK) is not modelled\n",
                          TYPE_ESP32P4_PMU);
        }
        break;
    case A_EXT_WAKEUP_CNTL:
        if (v & EXT_FILTER) {
            qemu_log_mask(LOG_UNIMP, "%s: the EXT wakeup's filter is not "
                          "modelled\n", TYPE_ESP32P4_PMU);
        }
        break;
    default:
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_pmu_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4PMUState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;
    const PMUReg *info = &esp32p4_pmu_regs[R(reg)];
    uint32_t *r = pmu_reg(s, reg);
    uint32_t wt = v & info->wt;
    uint32_t old = *r;
    int n;

    if (!info->valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_PMU, size, addr, value);
        return;
    }
    if (!(info->rw | info->wt | info->wtc)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_PMU, addr);
        return;
    }
    *r = (*r & ~(info->rw & lanes)) | (v & info->rw);
    *r &= ~(v & info->wtc);
    pmu_log_unmodelled(s, reg, *r & ~old);

    switch (reg) {
    case A_IMM_HP_CK_POWER:
        pmu_imm_hp_ck_power(s, wt);
        break;
    case A_IMM_SLEEP_SYSCLK:
        pmu_imm_sleep_sysclk(s, wt);
        break;
    case A_IMM_HP_FUNC_ICG:
        if (wt & IMM_UPDATE) {
            s->cur.icg_func = pmu_hp_reg(s, pmu_hp_mode(s), HP_ICG_HP_FUNC);
        }
        break;
    case A_IMM_HP_APB_ICG:
        if (wt & IMM_UPDATE) {
            s->cur.icg_apb = pmu_hp_reg(s, pmu_hp_mode(s), HP_ICG_HP_APB);
        }
        break;
    case A_IMM_MODEM_ICG:
        if (wt & IMM_UPDATE) {
            s->cur.icg_modem_code =
                extract32(pmu_hp_reg(s, pmu_hp_mode(s), HP_ICG_MODEM), 30, 2);
        }
        break;
    case A_IMM_LP_ICG:
        if (wt & IMM_TIE_LOW_LP_ROOTCLK) {
            s->cur.lp_rootclk_sel = false;
        }
        if (wt & IMM_TIE_HIGH_LP_ROOTCLK) {
            s->cur.lp_rootclk_sel = true;
            qemu_log_mask(LOG_UNIMP, "%s: the LP root clock select "
                          "(TIE_HIGH_LP_ROOTCLK_SEL) is not modelled\n",
                          TYPE_ESP32P4_PMU);
        }
        break;
    case A_IMM_PAD_HOLD_ALL:
        pmu_imm_pad_hold_all(s, wt);
        break;
    case A_IMM_I2C_ISO:
        if (wt & TIE_LOW_I2C_ISO_EN) {
            s->cur.i2c_iso = false;
        }
        if (wt & TIE_HIGH_I2C_ISO_EN) {
            s->cur.i2c_iso = true;
        }
        break;
    case A_SLP_WAKEUP_CNTL0:
        if (wt & SLEEP_REQ) {
            pmu_sleep_request(s);
        }
        break;
    case A_TOUCH_PWR_CNTL:
        qemu_irq_pulse(s->touch_pwr);
        break;
    case A_SLP_WAKEUP_CNTL4:
        if (wt & SLP_REJECT_CAUSE_CLR) {
            s->regs[R(A_SLP_WAKEUP_STATUS1)] = 0;
        }
        break;
    case A_HP_INT_CLR:
        s->regs[R(A_HP_INT_RAW)] &= ~wt;
        break;
    case A_LP_INT_CLR:
        s->regs[R(A_LP_INT_RAW)] &= ~wt;
        break;
    case A_LP_CPU_PWR1:
        if (wt & LP_CPU_SLEEP_REQ) {
            pmu_lp_cpu_sleep_request(s);
        }
        break;
    case A_HP_LP_CPU_COMM:
        if (wt & LP_TRIGGER_HP) {
            pmu_raise_hp(s, HP_INT_SW);
        }
        if (wt & HP_TRIGGER_LP) {
            pmu_raise_lp(s, LP_INT_HP_SW_TRIGGER);
        }
        break;
    case A_EXT_WAKEUP_CNTL:
        if (*r & EXT_STATUS_CLR) {
            s->regs[R(A_EXT_WAKEUP_ST)] = 0;
        }
        break;
    default:
        for (n = 0; n < ESP32P4_PMU_EXT_LDOS; n++) {
            if (reg == A_EXT_LDO(n)) {
                pmu_ldo_update(s, n, wt & LDO_CNT_CLR);
            }
        }
        break;
    }
    pmu_check(s);
    pmu_update_outputs(s);
    pmu_vo_drive(s, false);
}

static const MemoryRegionOps esp32p4_pmu_ops = {
    .read = esp32p4_pmu_read,
    .write = esp32p4_pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * A wakeup source's level; the LP CPU's exception, rising, also raises
 * PMU_LP_CPU_EXC_INT.
 */
static void esp32p4_pmu_set_wakeup(void *opaque, int n, int level)
{
    ESP32P4PMUState *s = opaque;

    if (n == ESP32P4_PMU_WAKE_LP_CPU_EXC && level &&
        !(s->wakeup_in & BIT(n))) {
        pmu_raise_hp(s, HP_INT_LP_CPU_EXC);
    }
    s->wakeup_in = deposit32(s->wakeup_in, n, 1, level != 0);
    pmu_check(s);
    pmu_update_outputs(s);
}

static void esp32p4_pmu_set_ext_pad(void *opaque, int n, int level)
{
    ESP32P4PMUState *s = opaque;

    s->ext_pads = deposit32(s->ext_pads, n, 1, level != 0);
    pmu_check(s);
    pmu_update_outputs(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * PMU_TASK_SLEEP_REQ, at its event's time, which its pulse carries.
 */
static void esp32p4_pmu_etm_task(void *opaque, int n, int level)
{
    if (level) {
        pmu_sleep_request(opaque);
    }
}

static void esp32p4_pmu_clock_update(void *opaque, ClockEvent event)
{
    ESP32P4PMUState *s = opaque;
    int n;

    if (s->seq_waiting) {
        pmu_seq_arm(s, s->seq_fast, s->seq_slow);
    }
    if (s->lp_sleep_waiting) {
        pmu_lp_sleep_arm(s);
    }
    for (n = 0; n < ESP32P4_PMU_EXT_LDOS; n++) {
        pmu_ldo_schedule(s, n);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * The power-on state: HP_ACTIVE, every domain powered, the crystal and
 * the CPU and system PLLs running, the ICGs open and the LP CPU held. The
 * power domains are driven here, in the hold phase, while the reset and
 * clock control is itself in reset and so records the reset's own cause
 * rather than the power-up's.
 */
static void esp32p4_pmu_reset_hold(Object *obj, ResetType type)
{
    ESP32P4PMUState *s = ESP32P4_PMU(obj);
    ESP32P4PMUControls *c = &s->cur;
    int i;

    for (i = 0; i < ESP32P4_PMU_NREGS; i++) {
        s->regs[i] = esp32p4_pmu_regs[i].reset;
    }
    timer_del(&s->seq_timer);
    s->seq_waiting = false;
    s->state = ESP32P4_PMU_ST_HP_ACTIVE;
    s->last_state = STATE_HP_ACTIVE;
    s->target_state = ESP32P4_PMU_ST_HP_SLEEP;
    s->sleep_start_ns = 0;
    memset(c, 0, sizeof(*c));
    c->xpd_xtal = true;
    c->icg_global_xtal = true;
    c->xpd_pll = POWER_ON_XPD_PLL;
    c->icg_global_pll = POWER_ON_XPD_PLL;
    c->icg_func = UINT32_MAX;
    c->icg_apb = UINT32_MAX;
    c->icg_sysclk_en = true;
    s->lp_cpu_on = false;
    s->lp_cpu_held = true;
    s->lp_cpu_stalled = false;
    s->lp_cpu_sleeping = false;
    s->lp_cpu_masked = false;
    s->lp_sleep_waiting = false;
    timer_del(&s->lp_sleep_timer);
    for (i = 0; i < ESP32P4_PMU_EXT_LDOS; i++) {
        timer_del(&s->ldo[i].timer);
        s->ldo[i].running = false;
        s->ldo[i].reached[0] = false;
        s->ldo[i].reached[1] = false;
    }
    for (i = 0; i < ESP32P4_PMU_NUM_PD; i++) {
        s->power_level[i] = true;
        qemu_set_irq(s->power[i], 1);
    }
}

static void esp32p4_pmu_reset_exit(Object *obj, ResetType type)
{
    ESP32P4PMUState *s = ESP32P4_PMU(obj);
    int i;

    for (i = 0; i < ESP32P4_PMU_EXT_LDOS; i++) {
        pmu_ldo_update(s, i, true);
    }
    pmu_drive_outputs(s, true);
    pmu_vo_drive(s, true);
    qemu_irq_pulse(s->touch_pwr);
}

static void esp32p4_pmu_init(Object *obj)
{
    ESP32P4PMUState *s = ESP32P4_PMU(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &esp32p4_pmu_ops, s,
                          "esp32p4.pmu", ESP32P4_PMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (i = 0; i < 2; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
    s->fast = qdev_init_clock_in(dev, "fast", esp32p4_pmu_clock_update, s,
                                 ClockUpdate);
    s->slow = qdev_init_clock_in(dev, "slow", esp32p4_pmu_clock_update, s,
                                 ClockUpdate);
    timer_init_ns(&s->seq_timer, QEMU_CLOCK_VIRTUAL, pmu_seq_cb, s);
    timer_init_ns(&s->lp_sleep_timer, QEMU_CLOCK_VIRTUAL, pmu_lp_sleep_cb, s);
    for (i = 0; i < ESP32P4_PMU_EXT_LDOS; i++) {
        s->ldo[i].s = s;
        timer_init_ns(&s->ldo[i].timer, QEMU_CLOCK_VIRTUAL, pmu_ldo_cb,
                      &s->ldo[i]);
    }

    qdev_init_gpio_in_named(dev, esp32p4_pmu_set_wakeup, ESP32P4_PMU_WAKEUP,
                            ESP32P4_PMU_WAKE_SOURCES);
    qdev_init_gpio_in_named(dev, esp32p4_pmu_set_ext_pad, ESP32P4_PMU_EXT_PAD,
                            ESP32P4_PMU_EXT_PADS);
    qdev_init_gpio_in_named(dev, esp32p4_pmu_etm_task, ESP32P4_PMU_ETM_TASK,
                            1);
    qdev_init_gpio_out_named(dev, &s->lp_int, ESP32P4_PMU_LP_INT, 1);
    qdev_init_gpio_out_named(dev, s->power, ESP32P4_PMU_POWER,
                             ESP32P4_PMU_NUM_PD);
    qdev_init_gpio_out_named(dev, s->xpd, ESP32P4_PMU_XPD,
                             ESP32P4_CLKRST_NUM_XPD);
    qdev_init_gpio_out_named(dev, s->pll_xpd, ESP32P4_PMU_PLL_XPD,
                             ESP32P4_ANA_NUM_PLLS);
    qdev_init_gpio_out_named(dev, s->core_stall, ESP32P4_PMU_CORE_STALL,
                             ESP32P4_CLKRST_HP_CORES);
    qdev_init_gpio_out_named(dev, s->icg, ESP32P4_PMU_ICG,
                             ESP32P4_CLKRST_NUM_PMU_ICG);
    qdev_init_gpio_out_named(dev, s->sysclk, ESP32P4_PMU_SYSCLK,
                             ESP32P4_CLKRST_NUM_SYSCLK);
    qdev_init_gpio_out_named(dev, &s->lp_sleep, ESP32P4_PMU_LP_SLEEP, 1);
    qdev_init_gpio_out_named(dev, &s->lp_core_reset,
                             ESP32P4_PMU_LP_CORE_RESET, 1);
    qdev_init_gpio_out_named(dev, &s->lp_core_clock,
                             ESP32P4_PMU_LP_CORE_CLOCK, 1);
    qdev_init_gpio_out_named(dev, &s->lp_core_stall,
                             ESP32P4_PMU_LP_CORE_STALL, 1);
    qdev_init_gpio_out_named(dev, s->hold_all, ESP32P4_PMU_HOLD_ALL,
                             ESP32P4_PMU_HOLDS);
    qdev_init_gpio_out_named(dev, &s->pad_sleep, ESP32P4_PMU_PAD_SLEEP, 1);
    qdev_init_gpio_out_named(dev, &s->pause_wdt, ESP32P4_PMU_PAUSE_WDT, 1);
    qdev_init_gpio_out_named(dev, &s->xtal_off, ESP32P4_PMU_XTAL_OFF, 1);
    qdev_init_gpio_out_named(dev, &s->bod_vbat_sel, ESP32P4_PMU_BOD_VBAT_SEL,
                             1);
    qdev_init_gpio_out_named(dev, &s->uart_wakeup_en,
                             ESP32P4_PMU_UART_WAKEUP_EN, 1);
    qdev_init_gpio_out_named(dev, &s->lp_core_intr_mask,
                             ESP32P4_PMU_LP_CORE_INTR_MASK, 1);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_PMU_ETM_EVENT,
                             ESP32P4_PMU_ETM_EVTS);
    qdev_init_gpio_out_named(dev, &s->touch_pwr, ESP32P4_PMU_TOUCH_PWR, 1);
    qdev_init_gpio_out_named(dev, s->ldo_mv, ESP32P4_PMU_LDO_MV,
                             ESP32P4_PMU_LDO_CHANNELS);
    qdev_init_gpio_in_named(dev, esp32p4_pmu_sdmmc_volt,
                            ESP32P4_PMU_SDMMC_VOLT, 2);
    for (i = 1; i < ESP32P4_PMU_LDO_CHANNELS; i++) {
        g_autofree char *name = g_strdup_printf("vo%d-mv", i + 1);

        object_property_add_uint32_ptr(obj, name, &s->vo_mv[i],
                                       OBJ_PROP_FLAG_READ);
    }
}

static void esp32p4_pmu_realize(DeviceState *dev, Error **errp)
{
    if (!ESP32P4_PMU(dev)->clkrst) {
        error_setg(errp, "%s: the \"clkrst\" link is not set",
                   TYPE_ESP32P4_PMU);
    }
}

static void esp32p4_pmu_finalize(Object *obj)
{
    ESP32P4PMUState *s = ESP32P4_PMU(obj);
    int i;

    timer_del(&s->seq_timer);
    timer_del(&s->lp_sleep_timer);
    for (i = 0; i < ESP32P4_PMU_EXT_LDOS; i++) {
        timer_del(&s->ldo[i].timer);
    }
}

static const VMStateDescription vmstate_esp32p4_pmu_controls = {
    .name = TYPE_ESP32P4_PMU "/controls",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(top_pd, ESP32P4PMUControls),
        VMSTATE_BOOL(cpu_pd, ESP32P4PMUControls),
        VMSTATE_BOOL(cnnt_pd, ESP32P4PMUControls),
        VMSTATE_BOOL(mem_pd, ESP32P4PMUControls),
        VMSTATE_BOOL(lp_peri_pd, ESP32P4PMUControls),
        VMSTATE_BOOL(xpd_xtal, ESP32P4PMUControls),
        VMSTATE_UINT8(xpd_pll, ESP32P4PMUControls),
        VMSTATE_UINT8(xpd_pll_i2c, ESP32P4PMUControls),
        VMSTATE_BOOL(i2c_iso, ESP32P4PMUControls),
        VMSTATE_BOOL(i2c_retention, ESP32P4PMUControls),
        VMSTATE_UINT8(icg_global_pll, ESP32P4PMUControls),
        VMSTATE_BOOL(icg_global_xtal, ESP32P4PMUControls),
        VMSTATE_UINT32(icg_func, ESP32P4PMUControls),
        VMSTATE_UINT32(icg_apb, ESP32P4PMUControls),
        VMSTATE_UINT8(icg_modem_code, ESP32P4PMUControls),
        VMSTATE_BOOL(icg_slp_sel, ESP32P4PMUControls),
        VMSTATE_BOOL(sysclk_slp_sel, ESP32P4PMUControls),
        VMSTATE_BOOL(icg_sysclk_en, ESP32P4PMUControls),
        VMSTATE_BOOL(sysclk_nodiv, ESP32P4PMUControls),
        VMSTATE_UINT8(sysclk_sel, ESP32P4PMUControls),
        VMSTATE_BOOL(lp_rootclk_sel, ESP32P4PMUControls),
        VMSTATE_BOOL(uart_wakeup_en, ESP32P4PMUControls),
        VMSTATE_BOOL(lp_pad_hold, ESP32P4PMUControls),
        VMSTATE_BOOL(hp_pad_hold, ESP32P4PMUControls),
        VMSTATE_BOOL(pad_slp_sel, ESP32P4PMUControls),
        VMSTATE_BOOL(pause_wdt, ESP32P4PMUControls),
        VMSTATE_BOOL(cpu_stall, ESP32P4PMUControls),
        VMSTATE_BOOL(hp_pad_hold_auto, ESP32P4PMUControls),
        VMSTATE_BOOL(lp_pad_hold_auto, ESP32P4PMUControls),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_pmu_ldo = {
    .name = TYPE_ESP32P4_PMU "/ldo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4PMULdo),
        VMSTATE_BOOL(running, ESP32P4PMULdo),
        VMSTATE_INT64(start_ns, ESP32P4PMULdo),
        VMSTATE_BOOL_ARRAY(reached, ESP32P4PMULdo, 2),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_pmu_levels = {
    .name = TYPE_ESP32P4_PMU "/levels",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL_ARRAY(xpd, ESP32P4PMULevels, ESP32P4_CLKRST_NUM_XPD),
        VMSTATE_BOOL_ARRAY(pll_xpd, ESP32P4PMULevels, ESP32P4_ANA_NUM_PLLS),
        VMSTATE_BOOL_ARRAY(core_stall, ESP32P4PMULevels,
                           ESP32P4_CLKRST_HP_CORES),
        VMSTATE_BOOL_ARRAY(icg, ESP32P4PMULevels, ESP32P4_CLKRST_NUM_PMU_ICG),
        VMSTATE_BOOL_ARRAY(sysclk, ESP32P4PMULevels,
                           ESP32P4_CLKRST_NUM_SYSCLK),
        VMSTATE_BOOL(lp_sleep, ESP32P4PMULevels),
        VMSTATE_BOOL(lp_core_reset, ESP32P4PMULevels),
        VMSTATE_BOOL(lp_core_clock, ESP32P4PMULevels),
        VMSTATE_BOOL(lp_core_stall, ESP32P4PMULevels),
        VMSTATE_BOOL_ARRAY(hold_all, ESP32P4PMULevels, ESP32P4_PMU_HOLDS),
        VMSTATE_BOOL(pad_sleep, ESP32P4PMULevels),
        VMSTATE_BOOL(pause_wdt, ESP32P4PMULevels),
        VMSTATE_BOOL(xtal_off, ESP32P4PMULevels),
        VMSTATE_BOOL(bod_vbat_sel, ESP32P4PMULevels),
        VMSTATE_BOOL(uart_wakeup_en, ESP32P4PMULevels),
        VMSTATE_BOOL(lp_core_intr_mask, ESP32P4PMULevels),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_pmu = {
    .name = TYPE_ESP32P4_PMU,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(fast, ESP32P4PMUState),
        VMSTATE_CLOCK(slow, ESP32P4PMUState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4PMUState, ESP32P4_PMU_NREGS),
        VMSTATE_UINT32(state, ESP32P4PMUState),
        VMSTATE_UINT32(last_state, ESP32P4PMUState),
        VMSTATE_UINT32(target_state, ESP32P4PMUState),
        VMSTATE_TIMER(seq_timer, ESP32P4PMUState),
        VMSTATE_INT64(seq_due_ns, ESP32P4PMUState),
        VMSTATE_BOOL(seq_waiting, ESP32P4PMUState),
        VMSTATE_UINT64(seq_fast, ESP32P4PMUState),
        VMSTATE_UINT64(seq_slow, ESP32P4PMUState),
        VMSTATE_INT64(sleep_start_ns, ESP32P4PMUState),
        VMSTATE_STRUCT(cur, ESP32P4PMUState, 1, vmstate_esp32p4_pmu_controls,
                       ESP32P4PMUControls),
        VMSTATE_UINT32(wakeup_in, ESP32P4PMUState),
        VMSTATE_UINT32(ext_pads, ESP32P4PMUState),
        VMSTATE_BOOL(lp_cpu_on, ESP32P4PMUState),
        VMSTATE_BOOL(lp_cpu_held, ESP32P4PMUState),
        VMSTATE_BOOL(lp_cpu_stalled, ESP32P4PMUState),
        VMSTATE_BOOL(lp_cpu_sleeping, ESP32P4PMUState),
        VMSTATE_BOOL(lp_cpu_masked, ESP32P4PMUState),
        VMSTATE_BOOL(lp_sleep_waiting, ESP32P4PMUState),
        VMSTATE_TIMER(lp_sleep_timer, ESP32P4PMUState),
        VMSTATE_INT64(lp_sleep_due_ns, ESP32P4PMUState),
        VMSTATE_STRUCT_ARRAY(ldo, ESP32P4PMUState, ESP32P4_PMU_EXT_LDOS, 1,
                             vmstate_esp32p4_pmu_ldo, ESP32P4PMULdo),
        VMSTATE_BOOL_ARRAY(power_level, ESP32P4PMUState, ESP32P4_PMU_NUM_PD),
        VMSTATE_STRUCT(out, ESP32P4PMUState, 2, vmstate_esp32p4_pmu_levels,
                       ESP32P4PMULevels),
        VMSTATE_UINT8(sdmmc_volt, ESP32P4PMUState),
        VMSTATE_UINT32_ARRAY(vo_mv, ESP32P4PMUState,
                             ESP32P4_PMU_LDO_CHANNELS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_pmu_properties[] = {
    DEFINE_PROP_LINK("clkrst", ESP32P4PMUState, clkrst, TYPE_ESP32P4_CLKRST,
                     ESP32P4ClkRstState *),
    DEFINE_PROP_LINK("lp-cpu", ESP32P4PMUState, lp_cpu, TYPE_CPU,
                     CPUState *),
    DEFINE_PROP_LINK("lp-sys", ESP32P4PMUState, lp_sys, TYPE_ESP32P4_LP_SYS,
                     ESP32P4LPSysState *),
    DEFINE_PROP_UINT32("vdd-ldo-mv", ESP32P4PMUState, vdd_ldo_mv, 3300),
};

static void esp32p4_pmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_pmu_reset_hold;
    rc->phases.exit = esp32p4_pmu_reset_exit;
    dc->realize = esp32p4_pmu_realize;
    dc->vmsd = &vmstate_esp32p4_pmu;
    device_class_set_props(dc, esp32p4_pmu_properties);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static const TypeInfo esp32p4_pmu_types[] = {
    {
        .name          = TYPE_ESP32P4_PMU,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4PMUState),
        .instance_init = esp32p4_pmu_init,
        .instance_finalize = esp32p4_pmu_finalize,
        .class_init    = esp32p4_pmu_class_init,
    },
};

DEFINE_TYPES(esp32p4_pmu_types)
