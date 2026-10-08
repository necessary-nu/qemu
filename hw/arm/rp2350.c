/*
 * Raspberry Pi RP2350 machine (Arm Cortex-M33 configuration)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Two mutually exclusive boot paths:
 *
 *   -bios FILE    loads a raw boot ROM image at 0x00000000 and resets
 *                 core 0 through the ROM's vector table, as hardware does.
 *   -kernel FILE  loads an ELF (or raw image into flash) directly and
 *                 resets core 0 through a vector table at the start of the
 *                 XIP flash window, skipping the boot ROM. This is a
 *                 bring-up convenience; images that boot this way must be
 *                 linked with their vector table at 0x10000000.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/rp2350_soc.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"

struct RP2350MachineState {
    MachineState parent;

    RP2350State soc;
};

#define TYPE_RP2350_MACHINE MACHINE_TYPE_NAME("rp2350")
OBJECT_DECLARE_SIMPLE_TYPE(RP2350MachineState, RP2350_MACHINE)

/* [spec:nuos:req:emu.direct-load] */
static void rp2350_init(MachineState *machine)
{
    RP2350MachineState *s = RP2350_MACHINE(machine);
    DeviceState *soc;
    bool direct = machine->kernel_filename != NULL;
    int i;

    /* [spec:nuos:req:emu.ram-size] */
    if (machine->ram_size != RP2350_SRAM_SIZE) {
        error_report("rp2350: SRAM is fixed at %u KiB; -m is not supported",
                     (unsigned)(RP2350_SRAM_SIZE / KiB));
        exit(1);
    }
    if (direct && machine->firmware) {
        error_report("rp2350: -kernel and -bios are mutually exclusive");
        exit(1);
    }
    if (!direct && !machine->firmware) {
        error_report("rp2350: a boot ROM image (-bios) or a directly loaded "
                     "image (-kernel) is required");
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_RP2350_SOC);
    soc = DEVICE(&s->soc);
    object_property_set_link(OBJECT(soc), "memory",
                             OBJECT(get_system_memory()), &error_abort);
    qdev_prop_set_uint32(soc, "init-svtor",
                         direct ? RP2350_XIP_BASE : RP2350_ROM_BASE);
    sysbus_realize(SYS_BUS_DEVICE(soc), &error_fatal);

    if (!direct) {
        if (load_image_targphys(machine->firmware, RP2350_ROM_BASE,
                                RP2350_ROM_SIZE, NULL) < 0) {
            error_report("rp2350: could not load boot ROM image '%s'",
                         machine->firmware);
            exit(1);
        }
    }

    /*
     * armv7m_load_kernel also registers each CPU's reset handler, so it
     * must be called for every core even when there is nothing to load.
     */
    armv7m_load_kernel(s->soc.armv7m[0].cpu, machine->kernel_filename,
                       RP2350_XIP_BASE, s->soc.flash_size);
    for (i = 1; i < RP2350_NUM_CORES; i++) {
        armv7m_load_kernel(s->soc.armv7m[i].cpu, NULL, 0, 0);
    }
}

static void rp2350_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Raspberry Pi RP2350 (2x Cortex-M33)";
    mc->init = rp2350_init;
    mc->default_ram_size = RP2350_SRAM_SIZE;
    mc->min_cpus = RP2350_NUM_CORES;
    mc->max_cpus = RP2350_NUM_CORES;
    mc->default_cpus = RP2350_NUM_CORES;
}

static const TypeInfo rp2350_machine_types[] = {
    {
        .name          = TYPE_RP2350_MACHINE,
        .parent        = TYPE_MACHINE,
        .instance_size = sizeof(RP2350MachineState),
        .class_init    = rp2350_machine_class_init,
    },
};

DEFINE_TYPES(rp2350_machine_types)
