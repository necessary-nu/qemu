/*
 * RP2350 Global Exclusive Monitor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Global Exclusive Monitor".
 *
 * The monitor covers main SRAM (0x20000000 to 0x20082000) only. Its
 * processing elements are the two cores' load/store ports and the DMA's
 * write port. Each core holds at most one reservation: a naturally
 * aligned 16-byte granule, tagged with the size and the Security and
 * privilege state of the exclusive read that took it. A core's
 * exclusive write succeeds only if its reservation matches in granule,
 * size and tag and no other processing element has written the granule
 * since; a failing exclusive write is shot down before SRAM commits it.
 * Any exclusive write by a core clears its own reservation; its own
 * non-exclusive writes never do, and CLREX does not reach the monitor.
 * Outside SRAM every exclusive reports failure, writes are not
 * suppressed, and an exclusive read clears the core's reservation.
 *
 * Exclusive writes by the cores and DMA writes reach the monitor
 * directly. A core's normal stores go straight to SRAM under TCG, so the
 * monitor sees them only through the granule's contents: it captures the
 * granule, reading SRAM directly, when a reservation is taken and fails
 * the exclusive write if any byte has changed. A normal store by another
 * core that leaves the granule's contents unchanged therefore goes
 * unseen, and a core's own normal store that changes its reserved
 * granule fails its exclusive write, where hardware would let it
 * succeed.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "system/memory.h"
#include "hw/core/cpu.h"
#include "hw/misc/rp2350_exclmon.h"
#include "migration/vmstate.h"
#include "target/arm/tcg/excl-monitor.h"

#define SRAM_BASE   0x20000000
#define SRAM_LIMIT  0x20082000

static int exclmon_core(RP2350ExclMonState *s, CPUState *cs)
{
    int i;

    for (i = 0; i < RP2350_EXCLMON_CORES; i++) {
        if (s->core[i] == cs) {
            return i;
        }
    }
    g_assert_not_reached();
}

/* Clear every reservation, except core `except`'s, on [addr, addr+len). */
static void exclmon_written(RP2350ExclMonState *s, int except, uint32_t addr,
                            unsigned len)
{
    uint32_t first = addr & ~(RP2350_EXCLMON_GRANULE - 1);
    uint32_t last = (addr + len - 1) & ~(RP2350_EXCLMON_GRANULE - 1);
    int i;

    for (i = 0; i < RP2350_EXCLMON_CORES; i++) {
        RP2350ExclReservation *r = &s->res[i];

        if (i != except && r->valid &&
            r->granule >= first && r->granule <= last) {
            r->valid = false;
        }
    }
}

/* [spec:nuos:req:emu.machine+1] */
static bool rp2350_exclmon_supported(ARMExclMonitor *m, uint32_t addr)
{
    return addr >= SRAM_BASE && addr < SRAM_LIMIT;
}

/* The reserved granule in SRAM, which the monitor reads directly. */
static const uint8_t *exclmon_granule(RP2350ExclMonState *s,
                                      RP2350ExclReservation *r)
{
    return s->ram + (r->granule - SRAM_BASE);
}

/* [spec:nuos:req:emu.machine+1] */
static void rp2350_exclmon_load(ARMExclMonitor *m, CPUState *cs,
                                uint32_t addr, unsigned size, unsigned tag,
                                const void *host, uint32_t *val)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(m);
    RP2350ExclReservation *r = &s->res[exclmon_core(s, cs)];
    unsigned off = addr & (RP2350_EXCLMON_GRANULE - 1);
    const uint32_t *g;
    int i;

    qemu_mutex_lock(&s->lock);
    r->valid = true;
    r->size = size;
    r->tag = tag;
    r->granule = addr - off;
    g = (const uint32_t *)exclmon_granule(s, r);
    for (i = 0; i < RP2350_EXCLMON_GRANULE / 4; i++) {
        uint32_t w = qatomic_read(&g[i]);

        memcpy(&r->data[i * 4], &w, 4);
    }
    if (host) {
        *val = ldn_le_p(&r->data[off], 1 << size);
    }
    qemu_mutex_unlock(&s->lock);
}

/*
 * Whether the granule still holds what the reservation captured, except
 * for the `len` bytes at `off` when `skip_target` is set.
 */
static bool exclmon_unchanged(RP2350ExclMonState *s, RP2350ExclReservation *r,
                              unsigned off, unsigned len, bool skip_target)
{
    const uint8_t *g = exclmon_granule(s, r);
    int i;

    for (i = 0; i < RP2350_EXCLMON_GRANULE; i++) {
        if (skip_target && i >= off && i < off + len) {
            continue;
        }
        if (qatomic_read(&g[i]) != r->data[i]) {
            return false;
        }
    }
    return true;
}

/*
 * Make the exclusive write at `host` if the target bytes still hold what
 * the reservation captured; return whether it did.
 */
static bool exclmon_commit(RP2350ExclReservation *r, void *host,
                           unsigned off, unsigned len, uint32_t val)
{
    uint32_t old = ldn_le_p(&r->data[off], len);

    switch (len) {
    case 1:
        return qatomic_cmpxchg((uint8_t *)host, (uint8_t)old,
                               (uint8_t)val) == (uint8_t)old;
    case 2:
        return qatomic_cmpxchg((uint16_t *)host, cpu_to_le16(old),
                               cpu_to_le16(val)) == cpu_to_le16(old);
    default:
        return qatomic_cmpxchg((uint32_t *)host, cpu_to_le32(old),
                               cpu_to_le32(val)) == cpu_to_le32(old);
    }
}

/* [spec:nuos:req:emu.machine+1] */
static bool rp2350_exclmon_store(ARMExclMonitor *m, CPUState *cs,
                                 uint32_t addr, unsigned size, unsigned tag,
                                 void *host, uint32_t val)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(m);
    int core = exclmon_core(s, cs);
    RP2350ExclReservation *r = &s->res[core];
    unsigned off = addr & (RP2350_EXCLMON_GRANULE - 1);
    unsigned len = 1 << size;
    bool pass;

    qemu_mutex_lock(&s->lock);
    pass = r->valid && r->granule == addr - off && r->size == size &&
           r->tag == tag;
    r->valid = false;
    if (pass) {
        /*
         * Through I/O the core writes after a true return, with the other
         * core stopped, so the whole granule can be checked here.
         */
        pass = exclmon_unchanged(s, r, off, len, host != NULL) &&
               (!host || exclmon_commit(r, host, off, len, val));
    }
    if (pass) {
        exclmon_written(s, core, addr, len);
    }
    qemu_mutex_unlock(&s->lock);
    return pass;
}

/* [spec:nuos:req:emu.machine+1] */
static void rp2350_exclmon_clear(ARMExclMonitor *m, CPUState *cs)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(m);

    qemu_mutex_lock(&s->lock);
    s->res[exclmon_core(s, cs)].valid = false;
    qemu_mutex_unlock(&s->lock);
}

/* [spec:nuos:req:emu.machine+1] */
static void rp2350_exclmon_write(ARMExclMonitor *m, CPUState *cs,
                                 uint32_t addr, unsigned size)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(m);

    qemu_mutex_lock(&s->lock);
    exclmon_written(s, exclmon_core(s, cs), addr, 1 << size);
    qemu_mutex_unlock(&s->lock);
}

/* [spec:nuos:req:emu.machine+1] */
void rp2350_exclmon_dma_write(RP2350ExclMonState *s, uint32_t addr,
                              unsigned len)
{
    if (addr >= SRAM_LIMIT || addr + len <= SRAM_BASE) {
        return;
    }
    qemu_mutex_lock(&s->lock);
    exclmon_written(s, -1, addr, len);
    qemu_mutex_unlock(&s->lock);
}

void rp2350_exclmon_attach(RP2350ExclMonState *s, int core, CPUState *cpu)
{
    assert(core >= 0 && core < RP2350_EXCLMON_CORES);
    s->core[core] = cpu;
}

static void rp2350_exclmon_realize(DeviceState *dev, Error **errp)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(dev);

    if (!s->sram || memory_region_size(s->sram) != SRAM_LIMIT - SRAM_BASE) {
        error_setg(errp, "sram must be linked to the 520 KiB SRAM");
        return;
    }
    s->ram = memory_region_get_ram_ptr(s->sram);
}

static void rp2350_exclmon_hold_reset(Object *obj, ResetType type)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(obj);

    memset(s->res, 0, sizeof(s->res));
}

static void rp2350_exclmon_init(Object *obj)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(obj);

    qemu_mutex_init(&s->lock);
}

static void rp2350_exclmon_finalize(Object *obj)
{
    RP2350ExclMonState *s = RP2350_EXCLMON(obj);

    qemu_mutex_destroy(&s->lock);
}

static const VMStateDescription vmstate_rp2350_excl_reservation = {
    .name = "rp2350-excl-reservation",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(valid, RP2350ExclReservation),
        VMSTATE_UINT8(size, RP2350ExclReservation),
        VMSTATE_UINT8(tag, RP2350ExclReservation),
        VMSTATE_UINT32(granule, RP2350ExclReservation),
        VMSTATE_UINT8_ARRAY(data, RP2350ExclReservation,
                            RP2350_EXCLMON_GRANULE),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_exclmon = {
    .name = TYPE_RP2350_EXCLMON,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(res, RP2350ExclMonState, RP2350_EXCLMON_CORES,
                             1, vmstate_rp2350_excl_reservation,
                             RP2350ExclReservation),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_exclmon_properties[] = {
    DEFINE_PROP_LINK("sram", RP2350ExclMonState, sram, TYPE_MEMORY_REGION,
                     MemoryRegion *),
};

static void rp2350_exclmon_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    ARMExclMonitorClass *emc = ARM_EXCL_MONITOR_CLASS(klass);

    dc->realize = rp2350_exclmon_realize;
    rc->phases.hold = rp2350_exclmon_hold_reset;
    device_class_set_props(dc, rp2350_exclmon_properties);
    dc->vmsd = &vmstate_rp2350_exclmon;
    emc->supported = rp2350_exclmon_supported;
    emc->load = rp2350_exclmon_load;
    emc->store = rp2350_exclmon_store;
    emc->clear = rp2350_exclmon_clear;
    emc->write = rp2350_exclmon_write;
}

/* [spec:nuos:req:emu.machine+1] */
static const TypeInfo rp2350_exclmon_info = {
    .name              = TYPE_RP2350_EXCLMON,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(RP2350ExclMonState),
    .instance_init     = rp2350_exclmon_init,
    .instance_finalize = rp2350_exclmon_finalize,
    .class_init        = rp2350_exclmon_class_init,
    .interfaces        = (const InterfaceInfo[]) {
        { TYPE_ARM_EXCL_MONITOR_INTERFACE },
        { }
    },
};

static void rp2350_exclmon_register_types(void)
{
    type_register_static(&rp2350_exclmon_info);
}
type_init(rp2350_exclmon_register_types)
