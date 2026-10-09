/*
 * RP2350 trace capture FIFO (CORESIGHT_TRACE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Trace FIFO". An 8-entry FIFO samples the
 * TPIU's 32-bit TRACEDATA output so the DMA (DREQ 53) can move trace into
 * SRAM. CTRL_STATUS.TRACE_CAPTURE_FIFO_FLUSH holds the FIFO empty, and
 * TRACE_CAPTURE_FIFO_OVERFLOW records a sample dropped because the FIFO
 * was full. The block has the atomic XOR/SET/CLR aliases.
 *
 * Nothing upstream generates trace in this model (the ITM and ETM produce
 * no ATB data), so with capture enabled the FIFO stays empty; that is
 * logged as LOG_UNIMP.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_coresight_trace.h"
#include "migration/vmstate.h"

#define A_CTRL_STATUS           0x000
#define A_TRACE_CAPTURE_FIFO    0x004

#define CTRL_FLUSH              (1u << 0)
#define CTRL_OVERFLOW           (1u << 1)
#define CTRL_RESET              CTRL_FLUSH

static void trace_update(RP2350CoreSightTraceState *s)
{
    qemu_set_irq(s->dreq, s->fifo_count != 0);
}

static void trace_flush(RP2350CoreSightTraceState *s)
{
    s->fifo_head = 0;
    s->fifo_count = 0;
}

/* [spec:nuos:req:emu.coresight] */
void rp2350_coresight_trace_capture(RP2350CoreSightTraceState *s,
                                    uint32_t word)
{
    if (s->ctrl_status & CTRL_FLUSH) {
        return;
    }
    if (s->fifo_count == RP2350_TRACE_FIFO_DEPTH) {
        s->ctrl_status |= CTRL_OVERFLOW;
        return;
    }
    s->fifo[(s->fifo_head + s->fifo_count) % RP2350_TRACE_FIFO_DEPTH] = word;
    s->fifo_count++;
    trace_update(s);
}

static uint32_t trace_pop(RP2350CoreSightTraceState *s)
{
    uint32_t v;

    if (!s->fifo_count) {
        return 0;
    }
    v = s->fifo[s->fifo_head];
    s->fifo_head = (s->fifo_head + 1) % RP2350_TRACE_FIFO_DEPTH;
    s->fifo_count--;
    trace_update(s);
    return v;
}

static void trace_write_ctrl(RP2350CoreSightTraceState *s, uint32_t old,
                             uint32_t v)
{
    /* OVERFLOW is cleared by writing 1 to it; FLUSH is plain RW. */
    s->ctrl_status = (v & CTRL_FLUSH) | (old & CTRL_OVERFLOW & ~v);
    if (s->ctrl_status & CTRL_FLUSH) {
        trace_flush(s);
        trace_update(s);
    } else if (old & CTRL_FLUSH) {
        qemu_log_mask(LOG_UNIMP, "rp2350-coresight-trace: capture enabled, "
                      "but no trace source produces data\n");
    }
}

/* [spec:nuos:req:emu.coresight] */
static uint64_t rp2350_coresight_trace_read(void *opaque, hwaddr addr,
                                            unsigned size)
{
    RP2350CoreSightTraceState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr & ~3);
    uint32_t v;

    if (addr >= RP2350_ATOMIC_REGION_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight-trace: read of "
                      "unmapped offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
    switch (reg) {
    case A_CTRL_STATUS:
        v = s->ctrl_status;
        break;
    case A_TRACE_CAPTURE_FIFO:
        v = trace_pop(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight-trace: read of bad "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.coresight] */
static void rp2350_coresight_trace_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size)
{
    RP2350CoreSightTraceState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr & ~3);
    uint32_t old;

    /* A narrow write is replicated across the bus and hits the register. */
    if (size == 1) {
        value = (value & 0xff) * 0x01010101u;
    } else if (size == 2) {
        value = (value & 0xffff) * 0x00010001u;
    }

    if (addr >= RP2350_ATOMIC_REGION_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight-trace: write to "
                      "unmapped offset 0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    switch (reg) {
    case A_CTRL_STATUS:
        /*
         * An alias write acts on the register's value: SET of OVERFLOW
         * writes 1 to it and so clears it, CLR leaves it alone.
         */
        old = s->ctrl_status;
        trace_write_ctrl(s, old, rp2350_atomic_apply(addr & ~3, old, value));
        break;
    case A_TRACE_CAPTURE_FIFO:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight-trace: write to the "
                      "read-only capture FIFO\n");
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-coresight-trace: write to bad "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_coresight_trace_ops = {
    .read = rp2350_coresight_trace_read,
    .write = rp2350_coresight_trace_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void rp2350_coresight_trace_reset_hold(Object *obj, ResetType type)
{
    RP2350CoreSightTraceState *s = RP2350_CORESIGHT_TRACE(obj);

    s->ctrl_status = CTRL_RESET;
    memset(s->fifo, 0, sizeof(s->fifo));
    trace_flush(s);
}

static void rp2350_coresight_trace_reset_exit(Object *obj, ResetType type)
{
    trace_update(RP2350_CORESIGHT_TRACE(obj));
}

static void rp2350_coresight_trace_init(Object *obj)
{
    RP2350CoreSightTraceState *s = RP2350_CORESIGHT_TRACE(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_coresight_trace_ops, s,
                          TYPE_RP2350_CORESIGHT_TRACE,
                          RP2350_CORESIGHT_TRACE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dreq);
}

static bool trace_fifo_valid(void *opaque, int version_id)
{
    RP2350CoreSightTraceState *s = opaque;

    return s->fifo_head < RP2350_TRACE_FIFO_DEPTH &&
           s->fifo_count <= RP2350_TRACE_FIFO_DEPTH;
}

static const VMStateDescription vmstate_rp2350_coresight_trace = {
    .name = TYPE_RP2350_CORESIGHT_TRACE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl_status, RP2350CoreSightTraceState),
        VMSTATE_UINT32_ARRAY(fifo, RP2350CoreSightTraceState,
                             RP2350_TRACE_FIFO_DEPTH),
        VMSTATE_UINT8(fifo_head, RP2350CoreSightTraceState),
        VMSTATE_UINT8(fifo_count, RP2350CoreSightTraceState),
        VMSTATE_VALIDATE("fifo in range", trace_fifo_valid),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_coresight_trace_class_init(ObjectClass *klass,
                                              const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_coresight_trace_reset_hold;
    rc->phases.exit = rp2350_coresight_trace_reset_exit;
    dc->vmsd = &vmstate_rp2350_coresight_trace;
}

/* [spec:nuos:req:emu.coresight] */
static const TypeInfo rp2350_coresight_trace_info = {
    .name          = TYPE_RP2350_CORESIGHT_TRACE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350CoreSightTraceState),
    .instance_init = rp2350_coresight_trace_init,
    .class_init    = rp2350_coresight_trace_class_init,
};

static void rp2350_coresight_trace_register_types(void)
{
    type_register_static(&rp2350_coresight_trace_info);
}
type_init(rp2350_coresight_trace_register_types)
