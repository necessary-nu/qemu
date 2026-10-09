/*
 * RP2350 bus fabric control (BUSCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_BUSCTRL_H
#define HW_MISC_RP2350_BUSCTRL_H

#include "hw/core/sysbus.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_RP2350_BUSCTRL "rp2350-busctrl"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350BusCtrlState, RP2350_BUSCTRL)

#define RP2350_BUSCTRL_COUNTERS 4
#define RP2350_BUSCTRL_CORES 2

/*
 * Address ranges of the crossbar's downstream ports, as the counting
 * overlays group them: one overlay can feed several ports (the striped
 * SRAM banks and the two XIP ports).
 */
typedef enum RP2350BusCtrlGroup {
    RP2350_BUSCTRL_GROUP_ROM,
    RP2350_BUSCTRL_GROUP_XIP,
    RP2350_BUSCTRL_GROUP_SRAM0_3,
    RP2350_BUSCTRL_GROUP_SRAM4_7,
    RP2350_BUSCTRL_GROUP_SRAM8,
    RP2350_BUSCTRL_GROUP_SRAM9,
    RP2350_BUSCTRL_GROUP_APB,
    RP2350_BUSCTRL_GROUP_FASTPERI,
    RP2350_BUSCTRL_GROUP_SIO,
    RP2350_BUSCTRL_GROUPS,
} RP2350BusCtrlGroup;

typedef struct RP2350BusCtrlOverlay {
    MemoryRegion mr;
    RP2350BusCtrlState *s;
    uint8_t core;
    uint8_t group;
    hwaddr base;
} RP2350BusCtrlOverlay;

struct RP2350BusCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /*
     * Each core's view of the fabric without the counting overlays: the
     * board memory plus that core's SIO port. Counted accesses are
     * forwarded through it.
     */
    MemoryRegion fabric[RP2350_BUSCTRL_CORES];
    MemoryRegion fabric_board[RP2350_BUSCTRL_CORES];
    MemoryRegion fabric_sio[RP2350_BUSCTRL_CORES][2];
    AddressSpace fabric_as[RP2350_BUSCTRL_CORES];
    bool attached[RP2350_BUSCTRL_CORES];

    RP2350BusCtrlOverlay overlay[RP2350_BUSCTRL_CORES][RP2350_BUSCTRL_GROUPS];

    MemoryRegion *board_memory;

    uint32_t bus_priority;
    uint32_t bus_priority_ack;
    uint32_t perfctr_en;
    uint32_t perfctr[RP2350_BUSCTRL_COUNTERS];
    uint32_t perfsel[RP2350_BUSCTRL_COUNTERS];
};

/*
 * Route `core`'s bus accesses through the performance counters: the
 * counting overlays go into `container`, the core's address space, above
 * the board memory and the SIO views. `sio` and `sio_nonsec` are the
 * core's SIO views, which the core sees at the SIO and SIO_NONSEC bases.
 */
void rp2350_busctrl_attach_core(RP2350BusCtrlState *s, int core,
                                MemoryRegion *container, MemoryRegion *sio,
                                MemoryRegion *sio_nonsec);

#endif
