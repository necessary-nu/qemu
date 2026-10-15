/*
 * A register block mapped behind a bus clock gate and reset, as Espressif
 * SoCs put their peripherals
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/misc/esp32_gated_region.h.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "system/address-spaces.h"
#include "hw/misc/esp32_gated_region.h"

/*
 * Whether an access to @mr, an I/O region, is split as one to the gate's
 * own region would be before the gate passes it on: at most 4 bytes and
 * aligned (memory_access_size()). A region allowing wider accesses would
 * take an access whole that the gate's region splits (or, from a CPU,
 * refuses).
 */
static bool esp32_gated_region_fits(MemoryRegion *mr)
{
    return mr->terminates && !mr->alias && !mr->ram && !mr->ram_device &&
           !mr->rom_device && !mr->is_iommu &&
           QTAILQ_EMPTY(&mr->subregions) &&
           (mr->ops->valid.max_access_size == 0 ||
            mr->ops->valid.max_access_size == 4) &&
           !mr->ops->impl.unaligned;
}

/*
 * The region an access at @addr of @size bytes reaches when the block's
 * region is a container of I/O regions, as the block's address space would
 * find it (the first enabled subregion, in priority order, holding @addr),
 * with the access's offset in it in *@offset; NULL where that is not one
 * region that wholly holds the access, aligned, and splits accesses as the
 * gate does, so that the address space must decide.
 */
static MemoryRegion *esp32_gated_region_leaf(Esp32GatedRegion *r,
                                             hwaddr addr, unsigned size,
                                             hwaddr *offset)
{
    MemoryRegion *sub;

    QTAILQ_FOREACH(sub, &r->block->subregions, subregions_link) {
        uint64_t len = int128_get64(sub->size);

        if (!sub->enabled || addr < sub->addr || addr - sub->addr >= len) {
            continue;
        }
        *offset = addr - sub->addr;
        if (*offset + size > len || (*offset & (size - 1)) ||
            !esp32_gated_region_fits(sub)) {
            return NULL;
        }
        return sub;
    }
    return NULL;
}

static MemTxResult esp32_gated_region_read(void *opaque, hwaddr addr,
                                           uint64_t *data, unsigned size,
                                           MemTxAttrs attrs)
{
    Esp32GatedRegion *r = opaque;
    MemoryRegion *leaf;
    hwaddr offset;
    uint8_t buf[8];
    MemTxResult res;

    if (!r->pass(r->opaque, addr, size, false, attrs)) {
        *data = 0;
        return MEMTX_OK;
    }
    leaf = esp32_gated_region_leaf(r, addr, size, &offset);
    if (leaf) {
        return memory_region_dispatch_read(leaf, offset, data,
                                           size_memop(size) | MO_LE, attrs);
    }
    res = address_space_read(&r->block_as, addr, attrs, buf, size);
    *data = ldn_le_p(buf, size);
    return res;
}

static MemTxResult esp32_gated_region_write(void *opaque, hwaddr addr,
                                            uint64_t data, unsigned size,
                                            MemTxAttrs attrs)
{
    Esp32GatedRegion *r = opaque;
    MemoryRegion *leaf;
    hwaddr offset;
    uint8_t buf[8];

    if (!r->pass(r->opaque, addr, size, true, attrs)) {
        return MEMTX_OK;
    }
    leaf = esp32_gated_region_leaf(r, addr, size, &offset);
    if (leaf) {
        return memory_region_dispatch_write(leaf, offset, data,
                                            size_memop(size) | MO_LE, attrs);
    }
    stn_le_p(buf, size, data);
    return address_space_write(&r->block_as, addr, attrs, buf, size);
}

static const MemoryRegionOps esp32_gated_region_ops = {
    .read_with_attrs = esp32_gated_region_read,
    .write_with_attrs = esp32_gated_region_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

void esp32_gated_region_init(Esp32GatedRegion *r, DeviceState *dev,
                             MemoryRegion *block, const char *name,
                             MemoryRegionGateFn *pass, void *opaque)
{
    uint64_t size = memory_region_size(block);
    g_autofree char *as_name = g_strdup_printf("%s-block", name);

    r->dev = dev;
    r->block = block;
    r->pass = pass;
    r->opaque = opaque;
    memory_region_init(&r->mr, NULL, name, size);
    /*
     * No owner: an owner's re-entrancy guard would block the forwarded
     * access to the block's own region, and one shared by all gates would
     * block a block reaching another through its gate.
     */
    memory_region_init_io(&r->through, NULL, &esp32_gated_region_ops, r,
                          name, size);
    r->in_place = esp32_gated_region_fits(block) &&
                  memory_region_set_gate(block, &r->through, pass, opaque);
    if (r->in_place) {
        memory_region_add_subregion(&r->mr, 0, block);
    } else {
        address_space_init(&r->block_as, block, as_name);
        memory_region_add_subregion(&r->mr, 0, &r->through);
    }
}

DeviceState *esp32_gated_region_device(MemoryRegion *mr)
{
    if (mr->ops != &esp32_gated_region_ops) {
        return NULL;
    }
    return ((Esp32GatedRegion *)mr->opaque)->dev;
}
