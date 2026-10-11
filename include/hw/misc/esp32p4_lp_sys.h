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

/*
 * Named GPIO input "etm-task-ulp-int": the ETM's task ULP_TASK_INT_CPU,
 * which sets ETM_TASK_ULP_INT_RAW as it rises.
 */
#define ESP32P4_LP_SYS_ETM_TASK_ULP_INT "etm-task-ulp-int"

/*
 * Named GPIO output ESP32P4_LP_SYS_USB_PHY_SEL: LP_SYSTEM_REG_USB_CTRL's
 * SW_HW_USB_PHY_SEL and SW_USB_PHY_SEL, to USB_WRAP's "phy-sel" inputs.
 */
#define ESP32P4_LP_SYS_USB_PHY_SEL "usb-phy-sel"

struct ESP32P4LPSysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t boot_addr_hp_core1;
    uint32_t lp_addrhole_addr;
    uint32_t lp_addrhole_info;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t usb_ctrl;
    uint32_t hp_usb_otghs_phy_ctrl;
    qemu_irq usb_phy_sel[2];
};

/*
 * An access the LP AHB matrix refused: one the LP APM did not permit
 * (@illegal false) or one to an address nothing decodes (@illegal true),
 * by the master with ID @id (the AXI ICM's CPU master IDs), at @addr.
 */
void esp32p4_lp_sys_addrhole(ESP32P4LPSysState *s, uint32_t addr,
                             uint8_t id, bool write, bool illegal);

#endif
