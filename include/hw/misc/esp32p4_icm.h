/*
 * ESP32-P4 AXI interconnect (ICM) registers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ICM_H
#define HW_MISC_ESP32P4_ICM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ICM "esp32p4-icm"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4IcmState, ESP32P4_ICM)

/*
 * The ICM's own registers at 0x500A4000; its QoS command interface at
 * 0x500A4400 is not part of this device.
 */
#define ESP32P4_ICM_BASE            0x500a4000
#define ESP32P4_ICM_SIZE            0x400

/*
 * The master IDs the CPU address-hole registers record (ICM_CPU_ADDRHOLE_ID,
 * and LP_SYSTEM_LP_ADDRHOLE_ID for the LP APM).
 */
#define ESP32P4_ICM_ID_HP_CORE0     0
#define ESP32P4_ICM_ID_HP_CORE1     1
#define ESP32P4_ICM_ID_LP_CORE      2
#define ESP32P4_ICM_ID_USB_OTG_FS   3
#define ESP32P4_ICM_ID_REGDMA       4
#define ESP32P4_ICM_ID_GMAC         5
#define ESP32P4_ICM_ID_SDMMC        6
#define ESP32P4_ICM_ID_USB_OTG_HS   7
#define ESP32P4_ICM_ID_TRACE0       8
#define ESP32P4_ICM_ID_TRACE1       9
#define ESP32P4_ICM_ID_TCM_MON      10
#define ESP32P4_ICM_ID_L2MEM_MON    11
/* A GDMA-AHB channel serving peripheral @sel (I3C 0, UHCI 2, I2S0 3, ...). */
#define ESP32P4_ICM_ID_AHB_PDMA(sel) (16 + (sel))
/* An AXI master's ID: its ICM port (CID) and the unit within it (UID). */
#define ESP32P4_ICM_ID_AXI(cid, uid) (((cid) << 4) | (uid))
#define ESP32P4_ICM_ID_CID(id)      (((id) >> 4) & 0xf)
/*
 * VDMA's two AXI master ports: master 1, which reaches the memories and
 * the CSI and DSI bridges' buffers, and master 2, the memories only.
 */
#define ESP32P4_ICM_CID_VDMA_M1     5
#define ESP32P4_ICM_CID_VDMA_M2     6

struct ESP32P4IcmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t ver_date;
    uint32_t clk_en;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t mst_arb_priority;
    uint32_t slv_arb_priority;
    uint32_t mst_arqos;
    uint32_t mst_awqos;
    uint32_t sys_addrhole_addr;
    uint32_t sys_addrhole_info;
    uint32_t cpu_addrhole_addr;
    uint32_t cpu_addrhole_info;
    uint32_t dlock_timeout;
    uint32_t rdn_eco_cs;
    uint32_t rdn_eco_low;
    uint32_t rdn_eco_high;
};

/*
 * An access the bus refused: one an APM did not permit (@illegal false)
 * or one to an address the master's port on the matrix does not decode
 * (@illegal true), by the master with ID @id, at @addr. @sys selects the
 * record of the AXI masters (ICM_SYS_ADDRHOLE_INT) over that of the CPUs
 * and AHB masters (ICM_CPU_ADDRHOLE_INT).
 */
void esp32p4_icm_addrhole(ESP32P4IcmState *s, bool sys, uint32_t addr,
                          uint8_t id, bool write, bool illegal);

#endif
