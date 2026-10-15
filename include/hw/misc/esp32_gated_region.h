/*
 * A register block mapped behind a bus clock gate and reset, as Espressif
 * SoCs put their peripherals
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_GATED_REGION_H
#define HW_MISC_ESP32_GATED_REGION_H

#include "system/memory.h"
#include "hw/core/qdev.h"

/*
 * A block's register region behind its gate. What is mapped, @mr, takes
 * accesses of 1 to 4 bytes, aligned (any other ends in a decode error),
 * asks the gate about each, and hands those it lets through to the block,
 * split and checked as the block's region splits and checks them.
 *
 * Where the block's region allows it (a single I/O region whose accesses
 * are split as @mr's are: at most 4 bytes, aligned), @mr holds the
 * block's region itself, and the gate decides each access to it in place
 * (memory_region_set_gate(), the gate's own region @through, unmapped,
 * the front whose checks come first): an access is one dispatch.
 * Otherwise @mr holds @through, which takes the accesses and passes
 * those the gate lets through on: to the I/O region that holds the
 * access, where the block's region is a container of them, as the RMT's
 * is (its registers and its RAM, which share a page: mapped side by side
 * in place they would make TCG find the region of each access through
 * the page's own small address space, costing more than the gate's); to
 * the block's region through an address space of its own for the rest.
 */
typedef struct Esp32GatedRegion {
    MemoryRegion mr;
    MemoryRegion through;
    MemoryRegion *block;
    AddressSpace block_as;
    DeviceState *dev;
    MemoryRegionGateFn *pass;
    void *opaque;
    bool in_place;
} Esp32GatedRegion;

/*
 * Put @block, a region of @dev, behind the gate @pass decides, as @r->mr
 * for the caller to map: @pass says whether an access to the block at an
 * offset goes through, or ends, a read returning 0 and a write having no
 * effect (which a gate refusing it says in the log). @name names the
 * gate's regions, as the log names an access to them that is not 1 to 4
 * bytes.
 */
void esp32_gated_region_init(Esp32GatedRegion *r, DeviceState *dev,
                             MemoryRegion *block, const char *name,
                             MemoryRegionGateFn *pass, void *opaque);

/*
 * The block behind @mr if @mr is a gate's own region: an access to it
 * reaches the block as one of the block's own, which the memory core
 * refuses while the block is in the middle of an access; or NULL.
 */
DeviceState *esp32_gated_region_device(MemoryRegion *mr);

#endif
