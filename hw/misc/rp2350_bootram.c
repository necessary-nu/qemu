/*
 * RP2350 boot RAM (BOOTRAM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Boot RAM". 1 KiB of SRAM for the boot ROM,
 * with WRITE_ONCE registers and eight boot locks above it at +0x800. Like
 * the APB peripherals it has the atomic XOR/SET/CLR aliases, and it is
 * hardwired to Secure access only.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_bootram.h"
#include "migration/vmstate.h"

#define A_WRITE_ONCE0   0x800
#define A_WRITE_ONCE1   0x804
#define A_BOOTLOCK_STAT 0x808
#define A_BOOTLOCK0     0x80c
#define A_BOOTLOCK7     0x828

#define LOCK_MASK       0xff

static uint32_t ram_read(RP2350BootRAMState *s, hwaddr reg, unsigned size)
{
    uint32_t v = 0;

    memcpy(&v, &s->ram[reg], size);
    return le32_to_cpu(v);
}

static void ram_write(RP2350BootRAMState *s, hwaddr reg, uint32_t v,
                      unsigned size)
{
    v = cpu_to_le32(v);
    memcpy(&s->ram[reg], &v, size);
}

/* [spec:nuos:req:emu.bootram] */
static MemTxResult rp2350_bootram_read(void *opaque, hwaddr addr,
                                       uint64_t *data, unsigned size,
                                       MemTxAttrs attrs)
{
    RP2350BootRAMState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t bit;

    if (!attrs.secure && !attrs.unspecified) {
        return MEMTX_ERROR;
    }
    if (reg + size <= RP2350_BOOTRAM_SIZE) {
        *data = ram_read(s, reg, size);
        return MEMTX_OK;
    }
    if (size != 4) {
        return MEMTX_ERROR;
    }
    switch (reg) {
    case A_WRITE_ONCE0:
    case A_WRITE_ONCE1:
        *data = s->write_once[(reg - A_WRITE_ONCE0) / 4];
        break;
    case A_BOOTLOCK_STAT:
        *data = s->lock_stat;
        break;
    case A_BOOTLOCK0 ... A_BOOTLOCK7:
        /* Reading claims the lock, returning its bit if it was free. */
        bit = 1u << ((reg - A_BOOTLOCK0) / 4);
        *data = s->lock_stat & bit;
        s->lock_stat &= ~bit;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-bootram: read of bad offset 0x%"
                      HWADDR_PRIx "\n", addr);
        *data = 0;
        break;
    }
    return MEMTX_OK;
}

static MemTxResult rp2350_bootram_write(void *opaque, hwaddr addr,
                                        uint64_t value, unsigned size,
                                        MemTxAttrs attrs)
{
    RP2350BootRAMState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t *w;

    if (!attrs.secure && !attrs.unspecified) {
        return MEMTX_ERROR;
    }
    if (reg + size <= RP2350_BOOTRAM_SIZE) {
        ram_write(s, reg, rp2350_atomic_apply(addr, ram_read(s, reg, size),
                                              value), size);
        return MEMTX_OK;
    }
    if (size != 4) {
        return MEMTX_ERROR;
    }
    switch (reg) {
    case A_WRITE_ONCE0:
    case A_WRITE_ONCE1:
        /* Bits only ever set, until reset. */
        w = &s->write_once[(reg - A_WRITE_ONCE0) / 4];
        *w |= rp2350_atomic_apply(addr, *w, value);
        break;
    case A_BOOTLOCK_STAT:
        s->lock_stat = rp2350_atomic_apply(addr, s->lock_stat, value) &
                       LOCK_MASK;
        break;
    case A_BOOTLOCK0 ... A_BOOTLOCK7:
        /* Any write releases the lock. */
        s->lock_stat |= 1u << ((reg - A_BOOTLOCK0) / 4);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-bootram: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_bootram_ops = {
    .read_with_attrs = rp2350_bootram_read,
    .write_with_attrs = rp2350_bootram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

void rp2350_bootram_reset_regs(RP2350BootRAMState *s)
{
    s->write_once[0] = 0;
    s->write_once[1] = 0;
    s->lock_stat = LOCK_MASK;
}

/*
 * Boot RAM is in the XIP memory power domain, which stays powered through
 * the power manager's own resets (RESET_TYPE_WAKEUP); the power manager
 * clears it when the domain powers down.
 */
/* [spec:nuos:req:emu.powman] */
static void rp2350_bootram_hold_reset(Object *obj, ResetType type)
{
    RP2350BootRAMState *s = RP2350_BOOTRAM(obj);

    if (type != RESET_TYPE_WAKEUP) {
        memset(s->ram, 0, sizeof(s->ram));
    }
    rp2350_bootram_reset_regs(s);
}

static void rp2350_bootram_init(Object *obj)
{
    RP2350BootRAMState *s = RP2350_BOOTRAM(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_bootram_ops, s,
                          TYPE_RP2350_BOOTRAM, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_rp2350_bootram = {
    .name = TYPE_RP2350_BOOTRAM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(ram, RP2350BootRAMState, RP2350_BOOTRAM_SIZE),
        VMSTATE_UINT32_ARRAY(write_once, RP2350BootRAMState, 2),
        VMSTATE_UINT32(lock_stat, RP2350BootRAMState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_bootram_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_bootram_hold_reset;
    dc->vmsd = &vmstate_rp2350_bootram;
}

static const TypeInfo rp2350_bootram_info = {
    .name          = TYPE_RP2350_BOOTRAM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350BootRAMState),
    .instance_init = rp2350_bootram_init,
    .class_init    = rp2350_bootram_class_init,
};

static void rp2350_bootram_register_types(void)
{
    type_register_static(&rp2350_bootram_info);
}
type_init(rp2350_bootram_register_types)
