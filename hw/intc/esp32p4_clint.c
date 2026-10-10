/*
 * ESP32-P4 HP cores' core-local interrupts (CLINT): the machine timer and
 * software interrupts
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 2.8
 * (Memory-Mapped Registers) and 2.9.3 (Core-Local Interrupts).
 *
 * The two HP cores share one 64-bit counter, mtime, and each has its own
 * software interrupt (msip, 0x0) and timer compare value (mtimecmplo and
 * mtimecmphi, 0x4000 and 0x4004), with mtimeloadlo and mtimeloadhi
 * (0x4008, 0x400C), mtimectl (0x4010) and the count, mtimelo and mtimehi
 * (0xBFF8, 0xBFFC). Each core sees its own registers at 0x20000000 and
 * the other core's at 0x20010000; other addresses in the window answer
 * with a bus error, and so does an access that does not come from an HP
 * core, such as the LP core's or a DMA master's; qtest and the debugger
 * see core 0's registers. Registers are 32 bits wide; a narrower access
 * reads or writes its bytes of one.
 *
 * msip's bit 0 is the core's machine software interrupt, CLIC interrupt 3,
 * which the other core can set through its window. The machine timer
 * interrupt, CLIC interrupt 7, is pending while the count is at or above
 * the core's mtimecmp; it is a level, so the handler clears it by
 * raising mtimecmp (or disabling it in the CLIC).
 *
 * The counter counts while core 0's mtimectl.MTIME_EN is set, which it is
 * from reset: only core 0's register has the bit, and only core 0 sets
 * the count, by writing its mtimeloadlo or mtimeloadhi, either of which
 * loads the count with the two together. Core 1 can reach both through
 * core 0's window. MTIME_OVF is set when the count reaches
 * 0xFFFFFFFFFFFFFFFF, and cleared by writing 0. MTIME_SAM samples the
 * other half of the count when one half is read, for the next read of the
 * other half to return: on reading mtimelo (1), mtimehi (2), or either
 * (3).
 *
 * The TRM does not say what clocks the counter; it counts the clock on
 * its "mtime" input, which the machine connects to CPU_CLK. Each core's
 * registers are behind its CLIC bus clock: while that is stopped they
 * drop writes. Each core's time CSR reads the count.
 *
 * Under icount the counter is exact: a timer interrupt is raised at the
 * virtual time the count reaches mtimecmp.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/esp32p4_clint.h"
#include "migration/vmstate.h"
#include "system/qtest.h"

#define A_MSIP          0x0000
#define A_MTIMECMPLO    0x4000
#define A_MTIMECMPHI    0x4004
#define A_MTIMELOADLO   0x4008
#define A_MTIMELOADHI   0x400c
#define A_MTIMECTL      0x4010
#define A_MTIMELO       0xbff8
#define A_MTIMEHI       0xbffc

#define CORE_WINDOW     0x10000

/* mtimectl */
#define MTIMECTL_EN         BIT(0)
#define MTIMECTL_OVF        BIT(1)
#define MTIMECTL_SAM_SHIFT  2
#define MTIMECTL_SAM_LO     BIT(2)
#define MTIMECTL_SAM_HI     BIT(3)
#define MTIMECTL_SAM        (MTIMECTL_SAM_LO | MTIMECTL_SAM_HI)

/* What a core's sample buffer holds, for its next read of that half. */
#define ESP32P4_CLINT_SAMPLED_NONE  0
#define ESP32P4_CLINT_SAMPLED_HI    1
#define ESP32P4_CLINT_SAMPLED_LO    2

static bool clint_should_run(ESP32P4ClintState *s)
{
    return (s->core[0].mtimectl & MTIMECTL_EN) &&
           clock_is_enabled(s->mtime_clk) &&
           !device_is_in_reset(DEVICE(s));
}

/*
 * Virtual time taken by n counts at the current rate, which is not zero,
 * rounded up; a time too far off to schedule is capped.
 */
static int64_t clint_counts_ns(ESP32P4ClintState *s, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(s->mtime_clk, n);

    if (ns >= INT64_MAX / 2) {
        return INT64_MAX / 2;
    }
    while (clock_ns_to_ticks(s->mtime_clk, ns) < n) {
        ns++;
    }
    return ns;
}

/* Whole counts since sync_ns. */
static uint64_t clint_elapsed(ESP32P4ClintState *s, int64_t now)
{
    if (!s->running || now <= s->sync_ns) {
        return 0;
    }
    return clock_ns_to_ticks(s->mtime_clk, now - s->sync_ns);
}

static uint64_t clint_count(ESP32P4ClintState *s)
{
    return s->count + clint_elapsed(s,
                                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void clint_set_ovf(ESP32P4ClintState *s)
{
    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        s->core[i].mtimectl |= MTIMECTL_OVF;
    }
}

/*
 * Fold elapsed counts into the count, keeping sync_ns on a count boundary
 * so that the time towards the next count is not lost; note an overflow
 * on the way. Called before anything that changes the count, rate or run
 * state, or that reads the overflow flag.
 */
static void clint_sync(ESP32P4ClintState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t n = clint_elapsed(s, now);

    if (!s->running) {
        s->sync_ns = now;
        return;
    }
    if (n && UINT64_MAX - s->count < n) {
        clint_set_ovf(s);
    }
    s->count += n;
    s->sync_ns += clint_counts_ns(s, n);
    if (s->count == UINT64_MAX) {
        clint_set_ovf(s);
    }
}

/*
 * Drive a core's timer interrupt from the count and its mtimecmp, and
 * schedule it for when the count reaches mtimecmp. The deadline is rounded
 * up, so the count has reached mtimecmp when the callback runs.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
static void clint_update_timer(ESP32P4ClintCore *c)
{
    ESP32P4ClintState *s = c->s;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t count = s->count + clint_elapsed(s, now);
    bool mtip = count >= c->mtimecmp;

    if (mtip != c->mtip) {
        c->mtip = mtip;
        qemu_set_irq(c->mtip_irq, mtip);
    }
    if (mtip || !s->running) {
        timer_del(c->timer);
        return;
    }
    timer_mod(c->timer, s->sync_ns + clint_counts_ns(s, c->mtimecmp -
                                                        s->count));
}

static void clint_update_timers(ESP32P4ClintState *s)
{
    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        clint_update_timer(&s->core[i]);
    }
}

/* Re-evaluate whether the counter runs, after a clint_sync(). */
static void clint_restart(ESP32P4ClintState *s)
{
    s->running = clint_should_run(s);
    clint_update_timers(s);
}

static void clint_timer_cb(void *opaque)
{
    clint_update_timer(opaque);
}

/*
 * The clock the counter counts changes rate, starts or stops: count up to
 * the change at the old rate, then start the next count afresh at the new
 * one.
 */
static void clint_mtime_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4ClintState *s = opaque;

    if (event == ClockPreUpdate) {
        clint_sync(s);
        return;
    }
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    clint_restart(s);
}

/* The time CSR. */
static uint64_t clint_rdtime(void *opaque)
{
    return clint_count(opaque);
}

/*
 * Read mtimelo or mtimehi, giving back the half a read of the other one
 * sampled, if it did, and sampling the other half if MTIME_SAM says so.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
static uint32_t clint_read_mtime(ESP32P4ClintCore *c, bool hi)
{
    uint64_t count;
    uint32_t v;

    if (c->sampled == (hi ? ESP32P4_CLINT_SAMPLED_HI
                          : ESP32P4_CLINT_SAMPLED_LO)) {
        c->sampled = ESP32P4_CLINT_SAMPLED_NONE;
        return c->sample;
    }
    count = clint_count(c->s);
    v = hi ? count >> 32 : (uint32_t)count;
    if (c->mtimectl & (hi ? MTIMECTL_SAM_HI : MTIMECTL_SAM_LO)) {
        c->sample = hi ? (uint32_t)count : count >> 32;
        c->sampled = hi ? ESP32P4_CLINT_SAMPLED_LO : ESP32P4_CLINT_SAMPLED_HI;
    }
    return v;
}

static uint32_t clint_read(ESP32P4ClintCore *c, hwaddr reg)
{
    switch (reg) {
    case A_MSIP:
        return c->msip;
    case A_MTIMECMPLO:
        return (uint32_t)c->mtimecmp;
    case A_MTIMECMPHI:
        return c->mtimecmp >> 32;
    case A_MTIMELOADLO:
        return c->mtimeload_lo;
    case A_MTIMELOADHI:
        return c->mtimeload_hi;
    case A_MTIMECTL:
        clint_sync(c->s);
        return c->mtimectl;
    case A_MTIMELO:
        return clint_read_mtime(c, false);
    default:
        return clint_read_mtime(c, true);
    }
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static void clint_write(ESP32P4ClintCore *c, hwaddr reg, uint32_t v)
{
    ESP32P4ClintState *s = c->s;
    uint32_t ctl;

    switch (reg) {
    case A_MSIP:
        c->msip = v & 1;
        qemu_set_irq(c->msip_irq, c->msip);
        break;
    case A_MTIMECMPLO:
        c->mtimecmp = deposit64(c->mtimecmp, 0, 32, v);
        clint_update_timer(c);
        break;
    case A_MTIMECMPHI:
        c->mtimecmp = deposit64(c->mtimecmp, 32, 32, v);
        clint_update_timer(c);
        break;
    case A_MTIMELOADLO:
    case A_MTIMELOADHI:
        if (reg == A_MTIMELOADLO) {
            c->mtimeload_lo = v;
        } else {
            c->mtimeload_hi = v;
        }
        if (c->index != 0) {
            break;
        }
        clint_sync(s);
        s->count = deposit64(c->mtimeload_lo, 32, 32, c->mtimeload_hi);
        if (s->count == UINT64_MAX) {
            clint_set_ovf(s);
        }
        clint_update_timers(s);
        break;
    case A_MTIMECTL:
        clint_sync(s);
        ctl = (c->mtimectl & ~MTIMECTL_SAM) | (v & MTIMECTL_SAM);
        if (!(v & MTIMECTL_OVF)) {
            ctl &= ~MTIMECTL_OVF;
        }
        if (c->index == 0) {
            ctl = (ctl & ~MTIMECTL_EN) | (v & MTIMECTL_EN);
        }
        if ((ctl ^ c->mtimectl) & MTIMECTL_SAM) {
            c->sampled = ESP32P4_CLINT_SAMPLED_NONE;
        }
        c->mtimectl = ctl;
        clint_restart(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only mtime%s\n",
                      TYPE_ESP32P4_CLINT, reg == A_MTIMELO ? "lo" : "hi");
        break;
    }
}

/*
 * The core whose view an access has: -1 if it does not come from an HP
 * core, as a DMA master's or the LP core's; core 0's for qtest and the
 * debugger.
 */
static int clint_accessor(ESP32P4ClintState *s, MemTxAttrs attrs)
{
    if (!current_cpu) {
        return qtest_enabled() || attrs.debug ? 0 : -1;
    }
    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        if (current_cpu == CPU(s->core[i].cpu)) {
            return i;
        }
    }
    return -1;
}

static MemTxResult clint_access(ESP32P4ClintState *s, hwaddr addr,
                                uint64_t *data, unsigned size, bool write,
                                MemTxAttrs attrs)
{
    int self = clint_accessor(s, attrs);
    hwaddr reg = (addr % CORE_WINDOW) & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);
    ESP32P4ClintCore *c;
    uint32_t v;

    switch (reg) {
    case A_MSIP:
    case A_MTIMECMPLO:
    case A_MTIMECMPHI:
    case A_MTIMELOADLO:
    case A_MTIMELOADHI:
    case A_MTIMECTL:
    case A_MTIMELO:
    case A_MTIMEHI:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at 0x%" HWADDR_PRIx ", which "
                      "has no register\n", TYPE_ESP32P4_CLINT,
                      write ? "write" : "read", addr);
        return MEMTX_ERROR;
    }
    if (self < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: access at 0x%" HWADDR_PRIx
                      " from outside the HP cores\n", TYPE_ESP32P4_CLINT,
                      addr);
        return MEMTX_ERROR;
    }
    c = &s->core[addr >= CORE_WINDOW ? !self : self];

    if (!write) {
        *data = (clint_read(c, reg) & mask) >> shift;
        return MEMTX_OK;
    }
    if (!clock_is_enabled(c->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " to core %d's registers while its CLIC clock is "
                      "stopped\n", TYPE_ESP32P4_CLINT, addr, c->index);
        return MEMTX_OK;
    }
    v = *data << shift;
    if (size < 4) {
        uint32_t cur;

        switch (reg) {
        case A_MTIMELO:
        case A_MTIMEHI:
            cur = 0;
            break;
        default:
            cur = clint_read(c, reg);
            break;
        }
        v = (cur & ~mask) | (v & mask);
    }
    clint_write(c, reg, v);
    return MEMTX_OK;
}

static MemTxResult clint_mmio_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    return clint_access(opaque, addr, data, size, false, attrs);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static MemTxResult clint_mmio_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size, MemTxAttrs attrs)
{
    return clint_access(opaque, addr, &data, size, true, attrs);
}

static const MemoryRegionOps clint_ops = {
    .read_with_attrs = clint_mmio_read,
    .write_with_attrs = clint_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_clint_init(Object *obj)
{
    ESP32P4ClintState *s = ESP32P4_CLINT(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &clint_ops, s, "esp32p4.clint",
                          ESP32P4_CLINT_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->mtime_clk = qdev_init_clock_in(dev, "mtime", clint_mtime_clk_update,
                                      s, ClockPreUpdate | ClockUpdate);
    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        g_autofree char *name = g_strdup_printf("core%d", i);
        ESP32P4ClintCore *c = &s->core[i];

        c->s = s;
        c->index = i;
        c->clk = qdev_init_clock_in(dev, name, NULL, NULL, 0);
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clint_timer_cb, c);
    }
    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        qdev_init_gpio_out_named(dev, &s->core[i].msip_irq,
                                 ESP32P4_CLINT_MSIP, 1);
    }
    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        qdev_init_gpio_out_named(dev, &s->core[i].mtip_irq,
                                 ESP32P4_CLINT_MTIP, 1);
    }
}

static void esp32p4_clint_realize(DeviceState *dev, Error **errp)
{
    ESP32P4ClintState *s = ESP32P4_CLINT(dev);

    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        CPURISCVState *env;

        if (!s->core[i].cpu) {
            error_setg(errp, "%s: cpu%d is not set", TYPE_ESP32P4_CLINT, i);
            return;
        }
        env = &s->core[i].cpu->parent_obj.env;
        env->rdtime_fn = clint_rdtime;
        env->rdtime_fn_arg = s;
    }
}

static void esp32p4_clint_reset_enter(Object *obj, ResetType type)
{
    ESP32P4ClintState *s = ESP32P4_CLINT(obj);

    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        ESP32P4ClintCore *c = &s->core[i];

        c->msip = 0;
        c->mtimecmp = UINT64_MAX;
        c->mtimeload_lo = 0;
        c->mtimeload_hi = 0;
        c->mtimectl = i == 0 ? MTIMECTL_EN : 0;
        c->sample = 0;
        c->sampled = ESP32P4_CLINT_SAMPLED_NONE;
        c->mtip = false;
        timer_del(c->timer);
    }
    s->count = 0;
    s->running = false;
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void esp32p4_clint_reset_hold(Object *obj, ResetType type)
{
    ESP32P4ClintState *s = ESP32P4_CLINT(obj);

    for (int i = 0; i < ESP32P4_CLINT_CORES; i++) {
        qemu_set_irq(s->core[i].msip_irq, 0);
        qemu_set_irq(s->core[i].mtip_irq, 0);
    }
}

static void esp32p4_clint_reset_exit(Object *obj, ResetType type)
{
    ESP32P4ClintState *s = ESP32P4_CLINT(obj);

    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    clint_restart(s);
}

static const VMStateDescription vmstate_esp32p4_clint_core = {
    .name = TYPE_ESP32P4_CLINT "/core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(msip, ESP32P4ClintCore),
        VMSTATE_UINT64(mtimecmp, ESP32P4ClintCore),
        VMSTATE_UINT32(mtimeload_lo, ESP32P4ClintCore),
        VMSTATE_UINT32(mtimeload_hi, ESP32P4ClintCore),
        VMSTATE_UINT32(mtimectl, ESP32P4ClintCore),
        VMSTATE_UINT32(sample, ESP32P4ClintCore),
        VMSTATE_UINT8(sampled, ESP32P4ClintCore),
        VMSTATE_BOOL(mtip, ESP32P4ClintCore),
        VMSTATE_TIMER_PTR(timer, ESP32P4ClintCore),
        VMSTATE_CLOCK(clk, ESP32P4ClintCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_clint = {
    .name = TYPE_ESP32P4_CLINT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(count, ESP32P4ClintState),
        VMSTATE_INT64(sync_ns, ESP32P4ClintState),
        VMSTATE_BOOL(running, ESP32P4ClintState),
        VMSTATE_CLOCK(mtime_clk, ESP32P4ClintState),
        VMSTATE_STRUCT_ARRAY(core, ESP32P4ClintState, ESP32P4_CLINT_CORES, 1,
                             vmstate_esp32p4_clint_core, ESP32P4ClintCore),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_clint_properties[] = {
    DEFINE_PROP_LINK("cpu0", ESP32P4ClintState, core[0].cpu,
                     TYPE_ESP32P4_CPU, ESP32P4CPU *),
    DEFINE_PROP_LINK("cpu1", ESP32P4ClintState, core[1].cpu,
                     TYPE_ESP32P4_CPU, ESP32P4CPU *),
};

static void esp32p4_clint_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_clint_realize;
    dc->vmsd = &vmstate_esp32p4_clint;
    rc->phases.enter = esp32p4_clint_reset_enter;
    rc->phases.hold = esp32p4_clint_reset_hold;
    rc->phases.exit = esp32p4_clint_reset_exit;
    device_class_set_props(dc, esp32p4_clint_properties);
}

/* [spec:nuos:req:emu.esp32p4.clic] */
static const TypeInfo esp32p4_clint_info = {
    .name          = TYPE_ESP32P4_CLINT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4ClintState),
    .instance_init = esp32p4_clint_init,
    .class_init    = esp32p4_clint_class_init,
};

static void esp32p4_clint_register_types(void)
{
    type_register_static(&esp32p4_clint_info);
}
type_init(esp32p4_clint_register_types)
