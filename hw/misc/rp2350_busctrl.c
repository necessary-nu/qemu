/*
 * RP2350 bus fabric control (BUSCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Bus fabric" and "BUSCTRL". The block holds
 * the crossbar's manager priorities and four 24-bit saturating performance
 * counters, each counting one event selected from four per downstream port
 * of the main AHB5 crossbar.
 *
 * QEMU's bus has no arbitration and no wait states, so of the four event
 * types only ACCESS can happen: no access is ever contested and no manager
 * or port ever stalls. Contested-access and stall events count zero, and
 * the priority levels change nothing.
 *
 * ACCESS events are counted by overlays in each core's address space that
 * forward every access to the fabric behind them and count it on the way.
 * An overlay is mapped only while counting is enabled and some counter
 * selects an ACCESS event of a port behind it, so the rest of the time
 * accesses take QEMU's direct paths (TCG executes and loads from RAM
 * without leaving generated code). While an overlay is mapped, code
 * executed from behind it is fetched through it as well, so instruction
 * fetches count; QEMU fetches per Thumb halfword where the core fetches
 * words, so counts that include fetches are approximate. Likewise TCG
 * performs a store-exclusive as a compare-and-swap, which counts a read
 * as well as the write.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qemu/rcu.h"
#include "qapi/error.h"
#include "exec/tb-flush.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_busctrl.h"
#include "migration/vmstate.h"

#define A_BUS_PRIORITY      0x00
#define A_BUS_PRIORITY_ACK  0x04
#define A_PERFCTR_EN        0x08
#define A_PERFCTR0          0x0c
#define A_PERFSEL3          0x28

#define BUS_PRIORITY_MASK   0x00001111
#define PERFCTR_EN_MASK     0x1
#define PERFCTR_MAX         0x00ffffff
#define PERFSEL_MASK        0x7f
#define PERFSEL_RESET       0x1f

/*
 * PERFSEL numbers the downstream ports from 0 and gives each four events:
 * event 4 * port + type.
 */
#define PORT_SIO_PROC1      0
#define PORT_SIO_PROC0      1
#define PORT_APB            2
#define PORT_FASTPERI       3
#define PORT_SRAM(n)        (13 - (n))
#define PORT_XIP_MAIN1      14
#define PORT_XIP_MAIN0      15
#define PORT_ROM            16
#define PORTS               17

#define EVENT_STALL_UPSTREAM    0
#define EVENT_STALL_DOWNSTREAM  1
#define EVENT_ACCESS_CONTESTED  2
#define EVENT_ACCESS            3
#define EVENTS                  (4 * PORTS)

#define SIO_BASE            0xd0000000
#define SIO_NONSEC_BASE     0xd0020000

typedef struct RP2350BusCtrlRange {
    const char *name;
    hwaddr base;
    hwaddr size;
} RP2350BusCtrlRange;

/*
 * The address ranges behind each overlay. ROM, XIP, the APB bridge and the
 * fast peripherals take their whole bus segment, so that addresses no
 * device decodes fault exactly as they do without the overlay.
 */
static const RP2350BusCtrlRange rp2350_busctrl_ranges[] = {
    [RP2350_BUSCTRL_GROUP_ROM]      = { "rom",      0x00000000, 0x10000000 },
    [RP2350_BUSCTRL_GROUP_XIP]      = { "xip",      0x10000000, 0x10000000 },
    [RP2350_BUSCTRL_GROUP_SRAM0_3]  = { "sram0-3",  0x20000000, 0x40000 },
    [RP2350_BUSCTRL_GROUP_SRAM4_7]  = { "sram4-7",  0x20040000, 0x40000 },
    [RP2350_BUSCTRL_GROUP_SRAM8]    = { "sram8",    0x20080000, 0x1000 },
    [RP2350_BUSCTRL_GROUP_SRAM9]    = { "sram9",    0x20081000, 0x1000 },
    [RP2350_BUSCTRL_GROUP_APB]      = { "apb",      0x40000000, 0x10000000 },
    [RP2350_BUSCTRL_GROUP_FASTPERI] = { "fastperi", 0x50000000, 0x10000000 },
    [RP2350_BUSCTRL_GROUP_SIO]      = { "sio",      SIO_BASE, 0x40000 },
};

/* The ports an overlay feeds, as a mask of port numbers. */
static uint32_t rp2350_busctrl_group_ports(int core, int group)
{
    switch (group) {
    case RP2350_BUSCTRL_GROUP_ROM:
        return 1u << PORT_ROM;
    case RP2350_BUSCTRL_GROUP_XIP:
        return (1u << PORT_XIP_MAIN0) | (1u << PORT_XIP_MAIN1);
    case RP2350_BUSCTRL_GROUP_SRAM0_3:
        return (1u << PORT_SRAM(0)) | (1u << PORT_SRAM(1)) |
               (1u << PORT_SRAM(2)) | (1u << PORT_SRAM(3));
    case RP2350_BUSCTRL_GROUP_SRAM4_7:
        return (1u << PORT_SRAM(4)) | (1u << PORT_SRAM(5)) |
               (1u << PORT_SRAM(6)) | (1u << PORT_SRAM(7));
    case RP2350_BUSCTRL_GROUP_SRAM8:
        return 1u << PORT_SRAM(8);
    case RP2350_BUSCTRL_GROUP_SRAM9:
        return 1u << PORT_SRAM(9);
    case RP2350_BUSCTRL_GROUP_APB:
        return 1u << PORT_APB;
    case RP2350_BUSCTRL_GROUP_FASTPERI:
        return 1u << PORT_FASTPERI;
    case RP2350_BUSCTRL_GROUP_SIO:
        return 1u << (core ? PORT_SIO_PROC1 : PORT_SIO_PROC0);
    default:
        g_assert_not_reached();
    }
}

/*
 * The port serving the 32-bit word at `addr`. SRAM0-3 and SRAM4-7 are
 * word-striped on address bits 3:2; the XIP cache interleaves 8-byte
 * lines over its two ports, even lines on XIP_MAIN0.
 */
static int rp2350_busctrl_port(int core, int group, hwaddr addr)
{
    switch (group) {
    case RP2350_BUSCTRL_GROUP_ROM:
        return PORT_ROM;
    case RP2350_BUSCTRL_GROUP_XIP:
        return addr & 8 ? PORT_XIP_MAIN1 : PORT_XIP_MAIN0;
    case RP2350_BUSCTRL_GROUP_SRAM0_3:
        return PORT_SRAM((addr >> 2) & 3);
    case RP2350_BUSCTRL_GROUP_SRAM4_7:
        return PORT_SRAM(4 + ((addr >> 2) & 3));
    case RP2350_BUSCTRL_GROUP_SRAM8:
        return PORT_SRAM(8);
    case RP2350_BUSCTRL_GROUP_SRAM9:
        return PORT_SRAM(9);
    case RP2350_BUSCTRL_GROUP_APB:
        return PORT_APB;
    case RP2350_BUSCTRL_GROUP_FASTPERI:
        return PORT_FASTPERI;
    case RP2350_BUSCTRL_GROUP_SIO:
        return core ? PORT_SIO_PROC1 : PORT_SIO_PROC0;
    default:
        g_assert_not_reached();
    }
}

/* The ports whose ACCESS event some running counter selects. */
static uint32_t rp2350_busctrl_counted_ports(RP2350BusCtrlState *s)
{
    uint32_t ports = 0;
    int n;

    if (!(s->perfctr_en & PERFCTR_EN_MASK)) {
        return 0;
    }
    for (n = 0; n < RP2350_BUSCTRL_COUNTERS; n++) {
        if (s->perfsel[n] < EVENTS &&
            (s->perfsel[n] & 3) == EVENT_ACCESS) {
            ports |= 1u << (s->perfsel[n] / 4);
        }
    }
    return ports;
}

/*
 * Map exactly the overlays that feed a counted port. Code translated while
 * an overlay was unmapped runs from RAM without fetching through it, so
 * mapping one discards all translations: from the next block on, fetches
 * from behind it are counted too.
 */
/* [spec:nuos:req:emu.busctrl] */
static void rp2350_busctrl_update_overlays(RP2350BusCtrlState *s)
{
    uint32_t ports = rp2350_busctrl_counted_ports(s);
    bool mapped = false;
    int core, group;

    memory_region_transaction_begin();
    for (core = 0; core < RP2350_BUSCTRL_CORES; core++) {
        if (!s->attached[core]) {
            continue;
        }
        for (group = 0; group < RP2350_BUSCTRL_GROUPS; group++) {
            MemoryRegion *mr = &s->overlay[core][group].mr;
            bool on = ports & rp2350_busctrl_group_ports(core, group);

            if (on && !mr->enabled) {
                mapped = true;
            }
            memory_region_set_enabled(mr, on);
        }
    }
    memory_region_transaction_commit();

    if (mapped) {
        CPUState *cs = current_cpu ? current_cpu : first_cpu;

        if (cs) {
            queue_tb_flush(cs);
        }
    }
}

/* [spec:nuos:req:emu.busctrl] */
static void rp2350_busctrl_count(RP2350BusCtrlState *s, int port)
{
    int n;

    if (!(s->perfctr_en & PERFCTR_EN_MASK)) {
        return;
    }
    for (n = 0; n < RP2350_BUSCTRL_COUNTERS; n++) {
        /* The counters saturate rather than wrap. */
        if (s->perfsel[n] == 4 * port + EVENT_ACCESS &&
            s->perfctr[n] < PERFCTR_MAX) {
            s->perfctr[n]++;
        }
    }
}

/*
 * An access counts once per 32-bit word it touches, each on the port that
 * serves that word: the bus is 32 bits wide, and the core splits unaligned
 * accesses at word boundaries.
 */
static void rp2350_busctrl_count_access(RP2350BusCtrlOverlay *o, hwaddr addr,
                                        unsigned size)
{
    hwaddr first = (o->base + addr) & ~(hwaddr)3;
    hwaddr last = (o->base + addr + size - 1) & ~(hwaddr)3;
    hwaddr word;

    for (word = first; word <= last; word += 4) {
        rp2350_busctrl_count(o->s, rp2350_busctrl_port(o->core, o->group,
                                                       word));
    }
}

/*
 * An access completes on its port when the fabric behind the overlay
 * completes it; accesses that fail decode or are refused count nothing.
 * A core's write to read-only memory is discarded without a fault, as on
 * QEMU's direct path, rather than raising the decode error a write through
 * an address space would.
 */
/* [spec:nuos:req:emu.busctrl] */
void rp2350_busctrl_dma_access(RP2350BusCtrlState *s, hwaddr addr,
                               unsigned size)
{
    hwaddr word;
    int group;

    if (!(s->perfctr_en & PERFCTR_EN_MASK)) {
        return;
    }
    for (group = 0; group < RP2350_BUSCTRL_GROUP_SIO; group++) {
        const RP2350BusCtrlRange *r = &rp2350_busctrl_ranges[group];

        if (addr >= r->base && addr - r->base < r->size) {
            for (word = addr & ~(hwaddr)3; word <= ((addr + size - 1) & ~3);
                 word += 4) {
                /* The core argument only selects among the SIO ports. */
                rp2350_busctrl_count(s, rp2350_busctrl_port(0, group, word));
            }
            return;
        }
    }
}

static MemTxResult rp2350_busctrl_overlay_read(void *opaque, hwaddr addr,
                                               uint64_t *data, unsigned size,
                                               MemTxAttrs attrs)
{
    RP2350BusCtrlOverlay *o = opaque;
    uint8_t buf[8] = {};
    MemTxResult r;

    r = address_space_read(&o->s->fabric_as[o->core], o->base + addr, attrs,
                           buf, size);
    if (r == MEMTX_OK) {
        *data = ldn_le_p(buf, size);
        rp2350_busctrl_count_access(o, addr, size);
    }
    return r;
}

static bool rp2350_busctrl_write_discarded(AddressSpace *as, hwaddr addr,
                                           unsigned size, MemTxAttrs attrs)
{
    MemoryRegion *mr;
    hwaddr xlat, len = size;

    RCU_READ_LOCK_GUARD();
    mr = address_space_translate(as, addr, &xlat, &len, true, attrs);
    return memory_region_is_ram(mr) && mr->readonly;
}

static MemTxResult rp2350_busctrl_overlay_write(void *opaque, hwaddr addr,
                                                uint64_t value, unsigned size,
                                                MemTxAttrs attrs)
{
    RP2350BusCtrlOverlay *o = opaque;
    AddressSpace *as = &o->s->fabric_as[o->core];
    uint8_t buf[8];
    MemTxResult r = MEMTX_OK;

    if (!rp2350_busctrl_write_discarded(as, o->base + addr, size, attrs)) {
        stn_le_p(buf, size, value);
        r = address_space_write(as, o->base + addr, attrs, buf, size);
    }
    if (r == MEMTX_OK) {
        rp2350_busctrl_count_access(o, addr, size);
    }
    return r;
}

static const MemoryRegionOps rp2350_busctrl_overlay_ops = {
    .read_with_attrs = rp2350_busctrl_overlay_read,
    .write_with_attrs = rp2350_busctrl_overlay_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = true,
};

static uint64_t rp2350_busctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350BusCtrlState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);

    switch (reg) {
    case A_BUS_PRIORITY:
        return s->bus_priority;
    case A_BUS_PRIORITY_ACK:
        return s->bus_priority_ack;
    case A_PERFCTR_EN:
        return s->perfctr_en;
    case A_PERFCTR0 ... A_PERFSEL3:
        if ((reg - A_PERFCTR0) & 4) {
            return s->perfsel[(reg - A_PERFCTR0) / 8];
        }
        return s->perfctr[(reg - A_PERFCTR0) / 8];
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-busctrl: read of bad offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void rp2350_busctrl_write_perfsel(RP2350BusCtrlState *s, int n,
                                         uint32_t sel)
{
    sel &= PERFSEL_MASK;
    if (sel >= EVENTS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-busctrl: PERFSEL%d selects no event (0x%x)\n",
                      n, sel);
    } else if ((sel & 3) != EVENT_ACCESS && sel != s->perfsel[n]) {
        qemu_log_mask(LOG_UNIMP,
                      "rp2350-busctrl: PERFSEL%d event 0x%x is a stall or "
                      "contention event, which never occurs without bus "
                      "arbitration timing; PERFCTR%d counts zero\n",
                      n, sel, n);
    }
    s->perfsel[n] = sel;
}

/* [spec:nuos:req:emu.busctrl] */
static void rp2350_busctrl_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    RP2350BusCtrlState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v;
    int n;

    switch (reg) {
    case A_BUS_PRIORITY:
        v = rp2350_atomic_apply(addr, s->bus_priority, value) &
            BUS_PRIORITY_MASK;
        if (v != s->bus_priority) {
            qemu_log_mask(LOG_UNIMP,
                          "rp2350-busctrl: bus priority 0x%x has no effect: "
                          "the bus has no arbitration\n", v);
        }
        s->bus_priority = v;
        /*
         * Arbiters take up new priorities on their next nonsequential
         * access. Without arbitration latency every arbiter has done so
         * as soon as the write completes.
         */
        s->bus_priority_ack = 1;
        break;
    case A_BUS_PRIORITY_ACK:
        break;
    case A_PERFCTR_EN:
        s->perfctr_en = rp2350_atomic_apply(addr, s->perfctr_en, value) &
                        PERFCTR_EN_MASK;
        rp2350_busctrl_update_overlays(s);
        break;
    case A_PERFCTR0 ... A_PERFSEL3:
        n = (reg - A_PERFCTR0) / 8;
        if ((reg - A_PERFCTR0) & 4) {
            rp2350_busctrl_write_perfsel(
                s, n, rp2350_atomic_apply(addr, s->perfsel[n], value));
            rp2350_busctrl_update_overlays(s);
        } else {
            /* Any write, through any alias, clears the counter. */
            s->perfctr[n] = 0;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-busctrl: write to bad offset 0x%" HWADDR_PRIx
                      "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_busctrl_ops = {
    .read = rp2350_busctrl_read,
    .write = rp2350_busctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.busctrl] */
void rp2350_busctrl_attach_core(RP2350BusCtrlState *s, int core,
                                MemoryRegion *container, MemoryRegion *sio,
                                MemoryRegion *sio_nonsec)
{
    Object *obj = OBJECT(s);
    g_autofree char *name = g_strdup_printf("rp2350-busctrl.fabric%d", core);
    int group;

    assert(core >= 0 && core < RP2350_BUSCTRL_CORES && !s->attached[core]);
    assert(s->board_memory);

    memory_region_init(&s->fabric[core], obj, name, UINT64_MAX);
    memory_region_init_alias(&s->fabric_board[core], obj,
                             "rp2350-busctrl.fabric-board", s->board_memory,
                             0, UINT64_MAX);
    memory_region_add_subregion_overlap(&s->fabric[core], 0,
                                        &s->fabric_board[core], -1);
    memory_region_init_alias(&s->fabric_sio[core][0], obj,
                             "rp2350-busctrl.fabric-sio", sio, 0,
                             memory_region_size(sio));
    memory_region_add_subregion(&s->fabric[core], SIO_BASE,
                                &s->fabric_sio[core][0]);
    memory_region_init_alias(&s->fabric_sio[core][1], obj,
                             "rp2350-busctrl.fabric-sio-nonsec", sio_nonsec,
                             0, memory_region_size(sio_nonsec));
    memory_region_add_subregion(&s->fabric[core], SIO_NONSEC_BASE,
                                &s->fabric_sio[core][1]);
    address_space_init(&s->fabric_as[core], &s->fabric[core], name);

    for (group = 0; group < RP2350_BUSCTRL_GROUPS; group++) {
        const RP2350BusCtrlRange *range = &rp2350_busctrl_ranges[group];
        RP2350BusCtrlOverlay *o = &s->overlay[core][group];
        g_autofree char *oname =
            g_strdup_printf("rp2350-busctrl.count-%s%d", range->name, core);

        o->s = s;
        o->core = core;
        o->group = group;
        o->base = range->base;
        memory_region_init_io(&o->mr, obj, &rp2350_busctrl_overlay_ops, o,
                              oname, range->size);
        /*
         * Counted accesses re-enter this device when they reach its own
         * registers through the APB overlay.
         */
        o->mr.disable_reentrancy_guard = true;
        memory_region_set_enabled(&o->mr, false);
        /* Above the board memory (-1) and the SIO views (0). */
        memory_region_add_subregion_overlap(container, range->base, &o->mr,
                                            1);
    }
    s->attached[core] = true;
    rp2350_busctrl_update_overlays(s);
}

static void rp2350_busctrl_hold_reset(Object *obj, ResetType type)
{
    RP2350BusCtrlState *s = RP2350_BUSCTRL(obj);
    int n;

    s->bus_priority = 0;
    s->bus_priority_ack = 0;
    s->perfctr_en = 0;
    for (n = 0; n < RP2350_BUSCTRL_COUNTERS; n++) {
        s->perfctr[n] = 0;
        s->perfsel[n] = PERFSEL_RESET;
    }
    rp2350_busctrl_update_overlays(s);
}

static void rp2350_busctrl_init(Object *obj)
{
    RP2350BusCtrlState *s = RP2350_BUSCTRL(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_busctrl_ops, s,
                          TYPE_RP2350_BUSCTRL, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static void rp2350_busctrl_realize(DeviceState *dev, Error **errp)
{
    RP2350BusCtrlState *s = RP2350_BUSCTRL(dev);

    if (!s->board_memory) {
        error_setg(errp, "memory property was not set");
    }
}

static int rp2350_busctrl_post_load(void *opaque, int version_id)
{
    rp2350_busctrl_update_overlays(opaque);
    return 0;
}

static const VMStateDescription vmstate_rp2350_busctrl = {
    .name = TYPE_RP2350_BUSCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rp2350_busctrl_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bus_priority, RP2350BusCtrlState),
        VMSTATE_UINT32(bus_priority_ack, RP2350BusCtrlState),
        VMSTATE_UINT32(perfctr_en, RP2350BusCtrlState),
        VMSTATE_UINT32_ARRAY(perfctr, RP2350BusCtrlState,
                             RP2350_BUSCTRL_COUNTERS),
        VMSTATE_UINT32_ARRAY(perfsel, RP2350BusCtrlState,
                             RP2350_BUSCTRL_COUNTERS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_busctrl_properties[] = {
    DEFINE_PROP_LINK("memory", RP2350BusCtrlState, board_memory,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void rp2350_busctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_busctrl_realize;
    rc->phases.hold = rp2350_busctrl_hold_reset;
    dc->vmsd = &vmstate_rp2350_busctrl;
    device_class_set_props(dc, rp2350_busctrl_properties);
}

/* [spec:nuos:req:emu.busctrl] */
static const TypeInfo rp2350_busctrl_info = {
    .name          = TYPE_RP2350_BUSCTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350BusCtrlState),
    .instance_init = rp2350_busctrl_init,
    .class_init    = rp2350_busctrl_class_init,
};

static void rp2350_busctrl_register_types(void)
{
    type_register_static(&rp2350_busctrl_info);
}
type_init(rp2350_busctrl_register_types)
