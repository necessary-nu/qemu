/*
 * Espressif ESP32-P4 HP and LP RISC-V cores
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 2
 * (High-Performance CPU), chapter 5 (Low-Power CPU) and chapter 11 (Reset
 * and Clock).
 *
 * The two HP cores implement RV32IMAFC with Zcb, Zcmp and Zcmt (Zc
 * specification v1.0.4-3) and the Zb extensions (Zba, Zbb, Zbc and Zbs of
 * the v1.0.0 specification), in machine and user mode. The LP core
 * implements RV32IMAC in machine mode only. These types carry the standard
 * ISA only: none of Espressif's custom CSRs or instructions (PIE, the
 * hardware loops), the CLIC or the PMA checker. QEMU only provides the Zc
 * and Zb extensions to cores of privileged specification 1.12, so the HP
 * cores are of that version; the TRM gives 1.10, which the LP core is.
 *
 * Each core has a reset input and a clock enable, which the chip's reset
 * and clock control drive. A core whose reset is held does not run and
 * stays in its reset state, starting from its reset vector once released;
 * a core whose clock is stopped stops where it is and carries on when the
 * clock restarts. Neither takes an interrupt meanwhile, not even to leave
 * WFI. Both act at the end of the code block the core is executing, so
 * when a core stops itself, the instructions after the store in the same
 * straight-line run complete first.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "hw/core/qdev.h"
#include "hw/core/sysemu-cpu-ops.h"
#include "migration/vmstate.h"
#include "target/riscv/esp32p4_cpu.h"
#include "internals.h"
#ifdef CONFIG_TCG
#include "accel/tcg/cpu-ops.h"
#endif

#define ESP32P4_HP_MVENDORID 0x612
#define ESP32P4_HP_MARCHID   0x80000003
#define ESP32P4_HP_MIMPID    0x1

#define ESP32P4_HP_PMP_REGIONS     32
#define ESP32P4_HP_PMP_GRANULARITY 128

static bool esp32p4_cpu_held(ESP32P4CPU *cpu)
{
    return qatomic_read(&cpu->reset) || qatomic_read(&cpu->clock_stopped);
}

static bool esp32p4_cpu_has_work(CPUState *cs)
{
    ESP32P4CPU *cpu = container_of(cs, ESP32P4CPU, parent_obj.parent_obj);

    return !esp32p4_cpu_held(cpu) && riscv_cpu_has_work(cs);
}

/* Run on the core when its reset input rises. */
static void esp32p4_cpu_reset_asserted(CPUState *cs, run_on_cpu_data data)
{
    cpu_reset(cs);
}

/* Run on the core when its reset input falls. */
static void esp32p4_cpu_reset_released(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (qatomic_read(&cpu->reset)) {
        return;
    }
    if (qatomic_read(&cpu->clock_stopped)) {
        cpu->clock_halted = true;
        return;
    }
    cs->halted = 0;
}

/* Run on the core when its clock stops. */
static void esp32p4_cpu_clock_stop(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (!qatomic_read(&cpu->clock_stopped) || qatomic_read(&cpu->reset) ||
        cs->halted) {
        return;
    }
    cs->halted = 1;
    cpu->clock_halted = true;
}

/* Run on the core when its clock restarts. */
static void esp32p4_cpu_clock_start(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(cs);

    if (qatomic_read(&cpu->clock_stopped) || qatomic_read(&cpu->reset) ||
        !cpu->clock_halted) {
        return;
    }
    cpu->clock_halted = false;
    cs->halted = 0;
}

/*
 * The inputs take effect on the core's own thread; until then the core
 * neither leaves a halt nor, once it stops at the end of its code block,
 * starts again.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_cpu_set_reset(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;
    bool reset = level;

    if (cpu->reset == reset) {
        return;
    }
    qatomic_set(&cpu->reset, reset);
    async_run_on_cpu(CPU(cpu), reset ? esp32p4_cpu_reset_asserted
                                     : esp32p4_cpu_reset_released,
                     RUN_ON_CPU_NULL);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_cpu_set_clock(void *opaque, int n, int level)
{
    ESP32P4CPU *cpu = opaque;
    bool stopped = !level;

    if (cpu->clock_stopped == stopped) {
        return;
    }
    qatomic_set(&cpu->clock_stopped, stopped);
    async_run_on_cpu(CPU(cpu), stopped ? esp32p4_cpu_clock_stop
                                       : esp32p4_cpu_clock_start,
                     RUN_ON_CPU_NULL);
}

/*
 * The reset inputs are not the core's own state and survive its reset: a
 * core reset while its reset is held, or its clock stopped, stays halted.
 */
static void esp32p4_cpu_reset_hold(Object *obj, ResetType type)
{
    ESP32P4CPU *cpu = ESP32P4_CPU(obj);
    ESP32P4CPUClass *ecc = ESP32P4_CPU_GET_CLASS(obj);

    if (ecc->parent_phases.hold) {
        ecc->parent_phases.hold(obj, type);
    }
    CPU(obj)->halted = esp32p4_cpu_held(cpu);
    cpu->clock_halted = !cpu->reset && cpu->clock_stopped;
}

static void esp32p4_cpu_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_cpu_set_reset,
                            ESP32P4_CPU_RESET, 1);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_cpu_set_clock,
                            ESP32P4_CPU_CLOCK, 1);
}

static bool esp32p4_cpu_get_in_reset(Object *obj, Error **errp)
{
    return ESP32P4_CPU(obj)->reset;
}

static bool esp32p4_cpu_get_clock_running(Object *obj, Error **errp)
{
    return !ESP32P4_CPU(obj)->clock_stopped;
}

static const VMStateDescription vmstate_esp32p4_cpu = {
    .name = "esp32p4-cpu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj.parent_obj, ESP32P4CPU, 0,
                       vmstate_cpu_common, CPUState),
        VMSTATE_BOOL(reset, ESP32P4CPU),
        VMSTATE_BOOL(clock_stopped, ESP32P4CPU),
        VMSTATE_BOOL(clock_halted, ESP32P4CPU),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * The hooks that ask whether a halted core has work, wrapped so that a
 * held core never does. Both types share one copy of each table.
 */
static SysemuCPUOps esp32p4_sysemu_ops;
#ifdef CONFIG_TCG
static TCGCPUOps esp32p4_tcg_ops;
#endif

static void esp32p4_cpu_class_init(ObjectClass *oc, const void *data)
{
    CPUClass *cc = CPU_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);
    ESP32P4CPUClass *ecc = ESP32P4_CPU_CLASS(oc);

    resettable_class_set_parent_phases(RESETTABLE_CLASS(oc), NULL,
                                       esp32p4_cpu_reset_hold, NULL,
                                       &ecc->parent_phases);

    esp32p4_sysemu_ops = *cc->sysemu_ops;
    esp32p4_sysemu_ops.has_work = esp32p4_cpu_has_work;
    cc->sysemu_ops = &esp32p4_sysemu_ops;
#ifdef CONFIG_TCG
    esp32p4_tcg_ops = *cc->tcg_ops;
    esp32p4_tcg_ops.cpu_exec_halt = esp32p4_cpu_has_work;
    cc->tcg_ops = &esp32p4_tcg_ops;
#endif

    dc->vmsd = &vmstate_esp32p4_cpu;

    object_class_property_add_bool(oc, "in-reset", esp32p4_cpu_get_in_reset,
                                   NULL);
    object_class_property_set_description(oc, "in-reset",
        "The core's reset input is held");
    object_class_property_add_bool(oc, "clock-running",
                                   esp32p4_cpu_get_clock_running, NULL);
    object_class_property_set_description(oc, "clock-running",
        "The core's clock enable is high");
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static const TypeInfo esp32p4_cpu_types[] = {
    {
        .name = TYPE_ESP32P4_CPU,
        .parent = TYPE_RISCV_VENDOR_CPU,
        .abstract = true,
        .instance_size = sizeof(ESP32P4CPU),
        .instance_init = esp32p4_cpu_init,
        .class_size = sizeof(ESP32P4CPUClass),
        .class_init = esp32p4_cpu_class_init,
    },
    {
        .name = TYPE_ESP32P4_HP_CPU,
        .parent = TYPE_ESP32P4_CPU,
        .class_data = &(const RISCVCPUDef) {
            .misa_mxl_max = MXL_RV32,
            .misa_ext = RVI | RVM | RVA | RVF | RVC | RVU,
            .priv_spec = PRIV_VERSION_1_12_0,
            .vext_spec = RISCV_PROFILE_ATTR_UNUSED,
            .cfg.max_satp_mode = VM_1_10_MBARE,
            .cfg.ext_zicsr = true,
            .cfg.ext_zifencei = true,
            .cfg.ext_zcb = true,
            .cfg.ext_zcmp = true,
            .cfg.ext_zcmt = true,
            .cfg.ext_zba = true,
            .cfg.ext_zbb = true,
            .cfg.ext_zbc = true,
            .cfg.ext_zbs = true,
            .cfg.pmp = true,
            .cfg.pmp_regions = ESP32P4_HP_PMP_REGIONS,
            .cfg.pmp_granularity = ESP32P4_HP_PMP_GRANULARITY,
            .cfg.mvendorid = ESP32P4_HP_MVENDORID,
            .cfg.marchid = ESP32P4_HP_MARCHID,
            .cfg.mimpid = ESP32P4_HP_MIMPID,
        },
    },
    {
        .name = TYPE_ESP32P4_LP_CPU,
        .parent = TYPE_ESP32P4_CPU,
        .class_data = &(const RISCVCPUDef) {
            .misa_mxl_max = MXL_RV32,
            .misa_ext = RVI | RVM | RVA | RVC,
            .priv_spec = PRIV_VERSION_1_10_0,
            .vext_spec = RISCV_PROFILE_ATTR_UNUSED,
            .cfg.max_satp_mode = VM_1_10_MBARE,
            .cfg.ext_zicsr = true,
            .cfg.ext_zifencei = true,
        },
    },
};

DEFINE_TYPES(esp32p4_cpu_types)
