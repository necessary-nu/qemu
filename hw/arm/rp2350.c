/*
 * Raspberry Pi RP2350 machine (Arm Cortex-M33 configuration)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Two boot paths:
 *
 *   -bios FILE    loads a raw boot ROM image at 0x00000000 and resets
 *                 core 0 through the ROM's vector table, as hardware does.
 *   -kernel FILE  loads an ELF (or raw image into flash) directly and
 *                 resets core 0 through a vector table at the start of the
 *                 XIP flash window, skipping the boot ROM. This is a
 *                 bring-up convenience; images that boot this way must be
 *                 linked with their vector table at 0x10000000. A -bios
 *                 image given as well is mapped, so ROM function lookups
 *                 work, but not executed.
 *
 * The RP2350 has no internal flash. Boards set its size with
 * -M rp2350,flash-size=SIZE (a Pico 2 has 4M); there is no default.
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
#include "qapi/visitor.h"
#include "system/address-spaces.h"
#include "system/reset.h"

struct RP2350MachineState {
    MachineState parent;

    RP2350State soc;
    uint64_t flash_size;
};

#define TYPE_RP2350_MACHINE MACHINE_TYPE_NAME("rp2350")
OBJECT_DECLARE_SIMPLE_TYPE(RP2350MachineState, RP2350_MACHINE)

/* Core 0 starts as the boot ROM would leave it for a flash image. */
static void rp2350_direct_reset(void *opaque)
{
    RP2350MachineState *s = opaque;

    rp2350_soc_boot_rom_handoff(&s->soc, 0);
}

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
    /* [spec:nuos:req:emu.flash] */
    if (direct && s->flash_size == 0) {
        error_report("rp2350: -kernel loads into flash, which needs "
                     "-M rp2350,flash-size=SIZE");
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
    qdev_prop_set_uint32(soc, "flash-size", s->flash_size);
    /* With no ROM executing, the machine launches core 1 itself. */
    qdev_prop_set_bit(soc, "core1-launch", direct);
    qdev_prop_set_uint32(soc, "init-svtor",
                         direct ? RP2350_XIP_BASE : RP2350_ROM_BASE);
    sysbus_realize(SYS_BUS_DEVICE(soc), &error_fatal);

    /* [spec:nuos:req:emu.bootrom+2] */
    if (machine->firmware) {
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
    /* Registered after the CPU resets, so it runs after them. */
    if (direct) {
        qemu_register_reset(rp2350_direct_reset, s);
    }
}

static void rp2350_get_flash_size(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    RP2350MachineState *s = RP2350_MACHINE(obj);

    visit_type_size(v, name, &s->flash_size, errp);
}

static void rp2350_set_flash_size(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    RP2350MachineState *s = RP2350_MACHINE(obj);
    uint64_t size;

    if (!visit_type_size(v, name, &size, errp)) {
        return;
    }
    if (size > RP2350_FLASH_MAX_SIZE) {
        error_setg(errp, "flash-size must be at most %u MiB",
                   (unsigned)(RP2350_FLASH_MAX_SIZE / MiB));
        return;
    }
    s->flash_size = size;
}

/*
 * The machine as a whole is what runs unmodified pico-sdk programs; see
 * emu/tests/run-pico-conformance.sh in NuOS.
 */
/* [spec:nuos:req:emu.pico-sdk] */
static void rp2350_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    object_class_property_add(oc, "flash-size", "size",
                              rp2350_get_flash_size, rp2350_set_flash_size,
                              NULL, NULL);
    object_class_property_set_description(oc, "flash-size",
        "Size of the board's QSPI flash on chip select 0 (no default)");

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
