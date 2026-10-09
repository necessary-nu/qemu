/*
 * RP2350 power-on state machine (PSM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "System resets (Power-on State Machine)".
 * The PSM releases the system resets in sequence, each stage waiting for
 * the one before it. A watchdog reset or a write to FRCE_OFF runs a
 * partial sequence: the sequence restarts from the earliest selected
 * stage, so that stage and every stage after it are reset. The boot ROM
 * relies on this when it treats any WDSEL bit at or before ROSC as having
 * reset the ring oscillator. The two processor stages are the exception:
 * both follow ACCESSCTRL, and either can be reset on its own, as pico-sdk's
 * multicore_reset_core1() does through FRCE_OFF.PROC1.
 *
 * The sequence itself is instantaneous, so DONE shows every stage that is
 * not forced off. FRCE_ON does nothing on production devices and is only
 * stored. The resets are carried out by the SoC, through the function set
 * with rp2350_psm_set_reset_fn(). A sequence that resets a processor runs
 * in a bottom half that first pauses every vCPU, since a core can request
 * its own reset and must stop before it is reset.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/core/irq.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_psm.h"
#include "migration/vmstate.h"
#include "system/cpus.h"
#include "system/qtest.h"

#define A_FRCE_ON  0x0
#define A_FRCE_OFF 0x4
#define A_WDSEL    0x8
#define A_DONE     0xc

#define PSM_PROCS  (BIT(RP2350_PSM_PROC0) | BIT(RP2350_PSM_PROC1))

/* The stages a sequence restarting at the stages in `sel` resets. */
static uint32_t psm_sequence_from(uint32_t sel)
{
    uint32_t chain = sel & RP2350_PSM_ALL & ~PSM_PROCS;
    uint32_t stages = sel & PSM_PROCS;

    if (chain) {
        stages |= RP2350_PSM_ALL & ~((chain & -chain) - 1);
    }
    return stages;
}

static void psm_sequence(RP2350PSMState *s)
{
    uint32_t reset = s->pending;
    bool watchdog = s->pending_watchdog;

    s->pending = 0;
    s->pending_watchdog = false;
    if (s->reset_fn) {
        s->reset_fn(s->reset_opaque, reset, psm_sequence_from(s->frce_off),
                    watchdog);
    }
}

static void psm_run(void *opaque)
{
    RP2350PSMState *s = opaque;

    if (!s->pending && !s->pending_watchdog) {
        return;
    }
    pause_all_vcpus();
    psm_sequence(s);
    resume_all_vcpus();
}

/*
 * Run the sequence for `stages`. Device resets take effect at once, but
 * a sequence that resets a processor waits for every vCPU to pause; a
 * core requesting it stops at the end of its current instruction block,
 * as on hardware it stops at once. Under qtest no vCPU executes, so there
 * is nothing to wait for.
 */
static void psm_request(RP2350PSMState *s, uint32_t stages, bool watchdog)
{
    s->pending |= stages;
    s->pending_watchdog |= watchdog;
    if (!(s->pending & PSM_PROCS) || qtest_enabled()) {
        psm_sequence(s);
        return;
    }
    cpu_stop_current();
    qemu_bh_schedule(s->bh);
}

/* [spec:nuos:req:emu.watchdog] */
static void psm_watchdog_reset(void *opaque, int n, int level)
{
    RP2350PSMState *s = opaque;

    if (level) {
        psm_request(s, psm_sequence_from(s->wdsel), true);
    }
}

static void psm_reset_regs(RP2350PSMState *s)
{
    s->frce_on = 0;
    s->frce_off = 0;
    s->wdsel = 0;
}

/*
 * A chip-level reset from the power manager that resets the PSM: its
 * registers return to their reset values and it runs the full sequence,
 * from the processor cold reset on.
 */
/* [spec:nuos:req:emu.powman] */
static void psm_powman_reset(void *opaque, int n, int level)
{
    RP2350PSMState *s = opaque;

    if (level) {
        psm_reset_regs(s);
        psm_request(s, RP2350_PSM_ALL, false);
    }
}

/* [spec:nuos:req:emu.watchdog] */
static uint64_t rp2350_psm_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350PSMState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_FRCE_ON:
        return s->frce_on;
    case A_FRCE_OFF:
        return s->frce_off;
    case A_WDSEL:
        return s->wdsel;
    case A_DONE:
        return RP2350_PSM_ALL & ~psm_sequence_from(s->frce_off);
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-psm: read of bad offset 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void rp2350_psm_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    RP2350PSMState *s = opaque;
    uint32_t held;

    switch (rp2350_atomic_reg(addr)) {
    case A_FRCE_ON:
        s->frce_on = rp2350_atomic_apply(addr, s->frce_on, value) &
                     RP2350_PSM_ALL;
        break;
    case A_FRCE_OFF:
        /*
         * Stages newly forced off are reset and held; stages released
         * start again. Both need the sequence to run.
         */
        held = psm_sequence_from(s->frce_off);
        s->frce_off = rp2350_atomic_apply(addr, s->frce_off, value) &
                      RP2350_PSM_ALL;
        held ^= psm_sequence_from(s->frce_off);
        if (held) {
            psm_request(s, held, false);
        }
        break;
    case A_WDSEL:
        s->wdsel = rp2350_atomic_apply(addr, s->wdsel, value) &
                   RP2350_PSM_ALL;
        break;
    case A_DONE:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-psm: write to bad offset 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps rp2350_psm_ops = {
    .read = rp2350_psm_read,
    .write = rp2350_psm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

void rp2350_psm_set_reset_fn(RP2350PSMState *s, RP2350PSMResetFn *fn,
                             void *opaque)
{
    s->reset_fn = fn;
    s->reset_opaque = opaque;
}

static void rp2350_psm_hold_reset(Object *obj, ResetType type)
{
    RP2350PSMState *s = RP2350_PSM(obj);

    psm_reset_regs(s);
    /* A chip-level reset supersedes any partial sequence still queued. */
    s->pending = 0;
    s->pending_watchdog = false;
    qemu_bh_cancel(s->bh);
}

static int rp2350_psm_post_load(void *opaque, int version_id)
{
    RP2350PSMState *s = opaque;

    if (s->pending || s->pending_watchdog) {
        qemu_bh_schedule(s->bh);
    }
    return 0;
}

static void rp2350_psm_init(Object *obj)
{
    RP2350PSMState *s = RP2350_PSM(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_psm_ops, s,
                          TYPE_RP2350_PSM, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_in_named(DEVICE(obj), psm_watchdog_reset, "watchdog", 1);
    qdev_init_gpio_in_named(DEVICE(obj), psm_powman_reset, "powman-reset", 1);
    s->bh = qemu_bh_new(psm_run, s);
}

static const VMStateDescription vmstate_rp2350_psm = {
    .name = TYPE_RP2350_PSM,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rp2350_psm_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(frce_on, RP2350PSMState),
        VMSTATE_UINT32(frce_off, RP2350PSMState),
        VMSTATE_UINT32(wdsel, RP2350PSMState),
        VMSTATE_UINT32(pending, RP2350PSMState),
        VMSTATE_BOOL(pending_watchdog, RP2350PSMState),
        VMSTATE_END_OF_LIST()
    },
};

/* [spec:nuos:req:emu.watchdog] */
static void rp2350_psm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_psm_hold_reset;
    dc->vmsd = &vmstate_rp2350_psm;
}

static const TypeInfo rp2350_psm_info = {
    .name          = TYPE_RP2350_PSM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350PSMState),
    .instance_init = rp2350_psm_init,
    .class_init    = rp2350_psm_class_init,
};

static void rp2350_psm_register_types(void)
{
    type_register_static(&rp2350_psm_info);
}
type_init(rp2350_psm_register_types)
