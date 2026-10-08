/*
 * RP2350 subsystem resets (RESETS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Subsystem resets".
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_resets.h"
#include "migration/vmstate.h"

#define A_RESET      0x0
#define A_WDSEL      0x4
#define A_RESET_DONE 0x8

static uint64_t rp2350_resets_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350ResetsState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_RESET:
        return s->reset;
    case A_WDSEL:
        return s->wdsel;
    case A_RESET_DONE:
        /*
         * Blocks come out of reset immediately, so a block is done exactly
         * when its reset is deasserted.
         */
        /* [spec:nuos:req:emu.resets] */
        return ~s->reset & RP2350_RESETS_ALL;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-resets: read of bad offset 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }
}

static void rp2350_resets_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    RP2350ResetsState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_RESET:
        s->reset = rp2350_atomic_apply(addr, s->reset, value) &
                   RP2350_RESETS_ALL;
        break;
    case A_WDSEL:
        s->wdsel = rp2350_atomic_apply(addr, s->wdsel, value) &
                   RP2350_RESETS_ALL;
        break;
    case A_RESET_DONE:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-resets: write to bad offset 0x%" HWADDR_PRIx
                      "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_resets_ops = {
    .read = rp2350_resets_read,
    .write = rp2350_resets_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_resets_hold_reset(Object *obj, ResetType type)
{
    RP2350ResetsState *s = RP2350_RESETS(obj);

    s->reset = RP2350_RESETS_ALL;
    s->wdsel = 0;
}

static void rp2350_resets_init(Object *obj)
{
    RP2350ResetsState *s = RP2350_RESETS(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_resets_ops, s,
                          TYPE_RP2350_RESETS, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_rp2350_resets = {
    .name = TYPE_RP2350_RESETS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reset, RP2350ResetsState),
        VMSTATE_UINT32(wdsel, RP2350ResetsState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_resets_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_resets_hold_reset;
    dc->vmsd = &vmstate_rp2350_resets;
}

static const TypeInfo rp2350_resets_info = {
    .name          = TYPE_RP2350_RESETS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350ResetsState),
    .instance_init = rp2350_resets_init,
    .class_init    = rp2350_resets_class_init,
};

static void rp2350_resets_register_types(void)
{
    type_register_static(&rp2350_resets_info);
}
type_init(rp2350_resets_register_types)
