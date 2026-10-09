/*
 * RP2350 bus access control (ACCESSCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ACCESSCTRL holds one permission register per bus endpoint and filters
 * every access to that endpoint by bus master and by security and
 * privilege. The filters sit in the bus fabric, so the model provides
 * each bus master with its own view of the system bus: board memory with
 * a checking window over every governed block. Devices behind the filters
 * need no knowledge of ACCESSCTRL.
 *
 * Bus masters and how an access identifies itself:
 *
 *  - Core 0 and core 1 reach the bus through their own view
 *    (rp2350_accessctrl_view() with RP2350_MASTER_CORE0/CORE1), which the
 *    SoC hands to each core as its memory. The access's MemTxAttrs give
 *    security (secure) and privilege (user).
 *  - The DMA reaches the bus through the RP2350_MASTER_DMA view. A DMA
 *    model builds an AddressSpace on that view and issues each channel's
 *    transfers with rp2350_accessctrl_dma_attrs() of the channel's
 *    security level. A denied transfer completes with MEMTX_ERROR, which
 *    the DMA model must turn into the channel's bus error.
 *  - The debugger is any access with attrs.debug set (gdbstub and monitor
 *    accesses through a core's address space, which mirror the core's
 *    security and privilege, as an Arm AHB Mem-AP does), and any access
 *    with unspecified attributes or made straight to system memory
 *    (qtest, loaders), which counts as a Secure, privileged debugger.
 */

#ifndef HW_MISC_RP2350_ACCESSCTRL_H
#define HW_MISC_RP2350_ACCESSCTRL_H

#include "hw/core/sysbus.h"
#include "qemu/notify.h"
#include "qom/object.h"

#define TYPE_RP2350_ACCESSCTRL "rp2350-accessctrl"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350AccessCtrlState, RP2350_ACCESSCTRL)

#define RP2350_ACCESSCTRL_BASE 0x40060000

typedef enum RP2350BusMaster {
    RP2350_MASTER_CORE0,
    RP2350_MASTER_CORE1,
    RP2350_MASTER_DMA,
    RP2350_MASTER_DEBUG,
    RP2350_NUM_MASTERS,
} RP2350BusMaster;

/* Masters with a view of their own; the debugger uses the cores' views. */
#define RP2350_ACCESSCTRL_NUM_VIEWS RP2350_MASTER_DEBUG

/* Every register, LOCK (0x000) through XIP_AUX (0x0e8). */
#define RP2350_ACCESSCTRL_NUM_REGS (0xe8 / 4 + 1)

/* DMA channel security levels (DMA SECCFG_CHx.S and .P). */
#define RP2350_DMA_SECLEVEL_NSU 0
#define RP2350_DMA_SECLEVEL_NSP 1
#define RP2350_DMA_SECLEVEL_SU  2
#define RP2350_DMA_SECLEVEL_SP  3

typedef struct RP2350AccessCtrlRange RP2350AccessCtrlRange;

/* A bus master's port into the register block. */
typedef struct RP2350AccessCtrlPort {
    RP2350AccessCtrlState *s;
    RP2350BusMaster master;
    MemoryRegion iomem;
} RP2350AccessCtrlPort;

/* A filter between one master's view and one governed block. */
typedef struct RP2350AccessCtrlGate {
    RP2350AccessCtrlState *s;
    RP2350BusMaster master;
    const RP2350AccessCtrlRange *range;
    MemoryRegion iomem;
} RP2350AccessCtrlGate;

struct RP2350AccessCtrlState {
    SysBusDevice parent_obj;

    /* The system bus the filters guard. */
    MemoryRegion *bus;
    AddressSpace bus_as;

    MemoryRegion view[RP2350_ACCESSCTRL_NUM_VIEWS];
    MemoryRegion view_bus[RP2350_ACCESSCTRL_NUM_VIEWS];
    RP2350AccessCtrlPort port[RP2350_NUM_MASTERS];
    RP2350AccessCtrlGate *gate[RP2350_ACCESSCTRL_NUM_VIEWS];

    /* Indexed by register offset / 4. */
    uint32_t regs[RP2350_ACCESSCTRL_NUM_REGS];

    NotifierList config_notifiers;
};

/*
 * The system bus as `master` sees it (core 0, core 1 or DMA). Valid once
 * the device is realized.
 */
MemoryRegion *rp2350_accessctrl_view(RP2350AccessCtrlState *s,
                                     RP2350BusMaster master);

/* Transaction attributes of a DMA channel at the given security level. */
MemTxAttrs rp2350_accessctrl_dma_attrs(unsigned seclevel);

/*
 * GPIO_NSMASK0 (bank 0) or GPIO_NSMASK1 (bank 1): the pins Non-secure
 * code may access, laid out as the SIO GPIO registers.
 */
uint32_t rp2350_accessctrl_gpio_nsmask(RP2350AccessCtrlState *s, int bank);

/*
 * Whether the governed block at `addr` is accessible to Non-secure code
 * (its NSP bit). Peripherals that are not are kept off Non-secure GPIOs,
 * and their RESETS controls are Secure-only.
 */
bool rp2350_accessctrl_ns_accessible(RP2350AccessCtrlState *s, hwaddr addr);

/* Called after any ACCESSCTRL register changes value. */
void rp2350_accessctrl_add_notifier(RP2350AccessCtrlState *s, Notifier *n);

#endif
