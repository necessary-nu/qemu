/*
 * ESP32-P4 LP system registers (LP_SYS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LP_SYS_H
#define HW_MISC_ESP32P4_LP_SYS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_SYS "esp32p4-lp-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPSysState, ESP32P4_LP_SYS)

#define ESP32P4_LP_SYS_SIZE 0x1000

/* The registers, 0x000 to 0x204, one word each. */
#define ESP32P4_LP_SYS_NREGS (0x208 / 4)

/* PAD_RTC_HOLD_CTRL0/1 hold GPIO0 to GPIO56. */
#define ESP32P4_LP_SYS_HOLDS 57

/*
 * Named GPIO outputs
 * ==================
 *
 * ESP32P4_LP_SYS_ERR_RESP_DIS: LP_CORE_ERR_RESP_DIS's bits for the LP
 *   core's IBUS, DBUS and AHB bus (ESP32P4_BUS_*), to the LP core's
 *   ESP32P4_CPU_ERR_RESP_DIS input.
 * ESP32P4_LP_SYS_PAD_HOLD: line n, GPIO n held by PAD_RTC_HOLD_CTRL0/1, to
 *   the GPIO block's "pad-hold" input, ORed with HP_SYSTEM's hold of the
 *   same pad.
 * ESP32P4_LP_SYS_SW_RESET: a pulse when software writes SYS_SW_RST, for
 *   the machine to make the software system reset (cause 0x03).
 * ESP32P4_LP_SYS_LP_CORE_DISABLE: LP_CORE_DISABLE, for the LP core.
 * ESP32P4_LP_SYS_STALL_SEL: SYSTIMER_STALL_SEL, high when the RTC timer
 *   follows HP core 1's stall rather than core 0's.
 * ESP32P4_LP_SYS_SWD_RESET_EN: high while the super watchdog may reset
 *   the system: FIB bit 2 clear, DIG_FIB's if LP_FIB_SEL bit 2 is set,
 *   else the analog state ANA_FIB's. Low from reset.
 * ESP32P4_LP_SYS_BOD_RESET_EN: the same for the brownout detector, bit 1.
 * ESP32P4_LP_SYS_HP_ROOT_CLK: HP_ROOT_CLK_CTRL's CPU_CLK_EN and
 *   SYS_CLK_EN, the gates of the HP system's 400 MHz CPU and 480 MHz
 *   system root clocks, for the reset and clock control.
 * ESP32P4_LP_SYS_USB_PHY_SEL: USB_CTRL's SW_HW_USB_PHY_SEL and
 *   SW_USB_PHY_SEL, to USB_WRAP's "phy-sel" inputs.
 *
 * Named GPIO inputs
 * =================
 *
 * ESP32P4_LP_SYS_LP_CORE_ETM_WAKEUP: a pulse of the ETM's task
 *   ULP_TASK_WAKEUP_CPU, the LP core's wakeup, which sets
 *   LP_CORE_ETM_WAKEUP_FLAG.
 * ESP32P4_LP_SYS_ETM_TASK_ULP_INT: a pulse of the ETM's ULP_TASK_INT_CPU,
 *   which raises ETM_TASK_ULP_INT.
 * ESP32P4_LP_SYS_SLOW_CLK_TICK: a pulse of the slow clock tick, which
 *   raises SLOW_CLK_TICK_INT.
 *
 * Sysbus IRQ 0 is LP_SYSREG_INTR, source 19 of the interrupt matrix.
 */
#define ESP32P4_LP_SYS_ERR_RESP_DIS     "lp-core-err-resp-dis"
#define ESP32P4_LP_SYS_PAD_HOLD         "pad-hold"
#define ESP32P4_LP_SYS_SW_RESET         "sys-sw-reset"
#define ESP32P4_LP_SYS_LP_CORE_DISABLE  "lp-core-disable"
#define ESP32P4_LP_SYS_STALL_SEL        "systimer-stall-sel"
#define ESP32P4_LP_SYS_SWD_RESET_EN     "swd-reset-en"
#define ESP32P4_LP_SYS_BOD_RESET_EN     "bod-reset-en"
#define ESP32P4_LP_SYS_HP_ROOT_CLK      "hp-root-clk"
#define ESP32P4_LP_SYS_USB_PHY_SEL      "usb-phy-sel"
#define ESP32P4_LP_SYS_LP_CORE_ETM_WAKEUP "lp-core-etm-wakeup"
#define ESP32P4_LP_SYS_ETM_TASK_ULP_INT "etm-task-ulp-int"
#define ESP32P4_LP_SYS_SLOW_CLK_TICK    "slow-clk-tick"

struct ESP32P4LPSysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    /* The LP core, whose PC LP_CPU_DBG_PC and LP_CPU_EXC_PC show. */
    struct ESP32P4CPU *lp_cpu;

    /* The registers as stored, indexed by offset / 4. */
    uint32_t regs[ESP32P4_LP_SYS_NREGS];

    qemu_irq err_resp_dis[3];
    qemu_irq pad_hold[ESP32P4_LP_SYS_HOLDS];
    qemu_irq sw_reset;
    qemu_irq lp_core_disable;
    qemu_irq stall_sel;
    qemu_irq swd_reset_en;
    qemu_irq bod_reset_en;
    qemu_irq hp_root_clk[2];
    qemu_irq usb_phy_sel[2];
};

/*
 * An access the LP AHB matrix refused: one the LP APM did not permit
 * (@illegal false) or one to an address nothing decodes (@illegal true),
 * by the master with ID @id (the AXI ICM's CPU master IDs), at @addr.
 */
void esp32p4_lp_sys_addrhole(ESP32P4LPSysState *s, uint32_t addr,
                             uint8_t id, bool write, bool illegal);

/*
 * The PMU's ext1 wakeup by the RTC pads in @pads (bit n for GPIO n of
 * EXT_WAKEUP1_SEL's 16), which EXT_WAKEUP1_STATUS records until
 * EXT_WAKEUP1_STATUS_CLR.
 */
void esp32p4_lp_sys_ext_wakeup1(ESP32P4LPSysState *s, uint32_t pads);

/* LP_CORE_BOOT_ADDR, the LP core's boot address, for the LP core. */
uint32_t esp32p4_lp_sys_lp_core_boot_addr(ESP32P4LPSysState *s);

/*
 * A register as software reads it, for the blocks whose configuration
 * lives here (the PAU's BACKUP_DMA_CFG*, the memories' aux controls, the
 * analog comparators, ...).
 */
uint32_t esp32p4_lp_sys_reg(ESP32P4LPSysState *s, hwaddr offset);

#endif
