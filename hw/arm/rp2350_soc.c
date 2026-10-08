/*
 * Raspberry Pi RP2350 SoC (Arm Cortex-M33 configuration)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, Raspberry Pi Ltd.
 * Addresses and IRQ numbers match the pico-sdk hardware_regs headers.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/rp2350_soc.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "target/arm/cpu-qom.h"

typedef struct RP2350Peripheral {
    const char *name;
    hwaddr base;
    hwaddr size;
} RP2350Peripheral;

/*
 * Peripheral blocks without a device model. Mapping them as unimplemented
 * devices makes guest accesses log the block's name rather than raise a
 * bus fault on unassigned memory.
 */
static const RP2350Peripheral rp2350_peripherals[] = {
    { "rp2350.sysinfo",         0x40000000, 0x8000 },
    { "rp2350.syscfg",          0x40008000, 0x8000 },
    { "rp2350.clocks",          0x40010000, 0x8000 },
    { "rp2350.psm",             0x40018000, 0x8000 },
    { "rp2350.resets",          0x40020000, 0x8000 },
    { "rp2350.io_bank0",        0x40028000, 0x8000 },
    { "rp2350.io_qspi",         0x40030000, 0x8000 },
    { "rp2350.pads_bank0",      0x40038000, 0x8000 },
    { "rp2350.pads_qspi",       0x40040000, 0x8000 },
    { "rp2350.xosc",            0x40048000, 0x8000 },
    { "rp2350.pll_sys",         0x40050000, 0x8000 },
    { "rp2350.pll_usb",         0x40058000, 0x8000 },
    { "rp2350.accessctrl",      0x40060000, 0x8000 },
    { "rp2350.busctrl",         0x40068000, 0x8000 },
    { "rp2350.uart0",           0x40070000, 0x8000 },
    { "rp2350.uart1",           0x40078000, 0x8000 },
    { "rp2350.spi0",            0x40080000, 0x8000 },
    { "rp2350.spi1",            0x40088000, 0x8000 },
    { "rp2350.i2c0",            0x40090000, 0x8000 },
    { "rp2350.i2c1",            0x40098000, 0x8000 },
    { "rp2350.adc",             0x400a0000, 0x8000 },
    { "rp2350.pwm",             0x400a8000, 0x8000 },
    { "rp2350.timer0",          0x400b0000, 0x8000 },
    { "rp2350.timer1",          0x400b8000, 0x8000 },
    { "rp2350.hstx_ctrl",       0x400c0000, 0x8000 },
    { "rp2350.xip_ctrl",        0x400c8000, 0x8000 },
    { "rp2350.xip_qmi",         0x400d0000, 0x8000 },
    { "rp2350.watchdog",        0x400d8000, 0x8000 },
    { "rp2350.bootram",         0x400e0000, 0x8000 },
    { "rp2350.rosc",            0x400e8000, 0x8000 },
    { "rp2350.trng",            0x400f0000, 0x8000 },
    { "rp2350.sha256",          0x400f8000, 0x8000 },
    { "rp2350.powman",          0x40100000, 0x8000 },
    { "rp2350.ticks",           0x40108000, 0x8000 },
    { "rp2350.otp",             0x40120000, 0x10000 },
    { "rp2350.otp_data",        0x40130000, 0x10000 },
    { "rp2350.coresight_periph", 0x40140000, 0x10000 },
    { "rp2350.dft",             0x40150000, 0x8000 },
    { "rp2350.glitch_detector", 0x40158000, 0x8000 },
    { "rp2350.tbman",           0x40160000, 0x8000 },
    { "rp2350.dma",             0x50000000, 0x100000 },
    { "rp2350.usbctrl",         0x50100000, 0x100000 },
    { "rp2350.pio0",            0x50200000, 0x100000 },
    { "rp2350.pio1",            0x50300000, 0x100000 },
    { "rp2350.pio2",            0x50400000, 0x100000 },
    { "rp2350.xip_aux",         0x50500000, 0x100000 },
    { "rp2350.hstx_fifo",       0x50600000, 0x100000 },
    { "rp2350.coresight_trace", 0x50700000, 0x100000 },
    { "rp2350.xip_sram",        0x13ffc000, 0x4000 },
    { "rp2350.sio",             0xd0000000, 0x20000 },
    { "rp2350.sio_nonsec",      0xd0020000, 0x20000 },
};

/* [spec:nuos:req:emu.machine] */
static void rp2350_soc_realize(DeviceState *dev_soc, Error **errp)
{
    RP2350State *s = RP2350_SOC(dev_soc);
    Object *obj = OBJECT(dev_soc);
    int i;

    if (!s->board_memory) {
        error_setg(errp, "memory property was not set");
        return;
    }
    if (s->flash_size == 0 || s->flash_size > RP2350_XIP_WINDOW_SIZE) {
        error_setg(errp, "flash-size must be between 1 byte and %u MiB",
                   (unsigned)(RP2350_XIP_WINDOW_SIZE / MiB));
        return;
    }

    if (!memory_region_init_rom(&s->rom, obj, "rp2350.rom", RP2350_ROM_SIZE,
                                errp)) {
        return;
    }
    memory_region_add_subregion(s->board_memory, RP2350_ROM_BASE, &s->rom);

    if (!memory_region_init_rom(&s->flash, obj, "rp2350.flash", s->flash_size,
                                errp)) {
        return;
    }
    memory_region_add_subregion(s->board_memory, RP2350_XIP_BASE, &s->flash);
    memory_region_init_alias(&s->flash_nocache_alias, obj,
                             "rp2350.flash.nocache-noalloc", &s->flash, 0,
                             s->flash_size);
    memory_region_add_subregion(s->board_memory,
                                RP2350_XIP_NOCACHE_NOALLOC_BASE,
                                &s->flash_nocache_alias);
    memory_region_init_alias(&s->flash_notranslate_alias, obj,
                             "rp2350.flash.nocache-noalloc-notranslate",
                             &s->flash, 0, s->flash_size);
    memory_region_add_subregion(s->board_memory,
                                RP2350_XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE,
                                &s->flash_notranslate_alias);

    /*
     * SRAM0-9 as one contiguous region: the striped SRAM0-7 window
     * (512 KiB) followed by the SRAM8/9 scratch banks (4 KiB each).
     */
    /* [spec:nuos:req:emu.ram-size] */
    if (!memory_region_init_ram(&s->sram, obj, "rp2350.sram",
                                RP2350_SRAM_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(s->board_memory, RP2350_SRAM_BASE, &s->sram);

    clock_set_hz(s->sysclk, RP2350_SYSCLK_HZ);
    clock_set_hz(s->refclk, RP2350_REFCLK_HZ);

    for (i = 0; i < RP2350_NUM_CORES; i++) {
        DeviceState *armv7m = DEVICE(&s->armv7m[i]);

        qdev_prop_set_uint32(armv7m, "num-irq", RP2350_NUM_IRQS);
        qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
        qdev_prop_set_string(armv7m, "cpu-type",
                             ARM_CPU_TYPE_NAME("cortex-m33"));
        qdev_prop_set_uint32(armv7m, "init-svtor", s->init_svtor);
        /*
         * On hardware the boot ROM holds core 1 until core 0 launches it
         * through the SIO FIFO. Without that path modelled, core 1 stays
         * powered off.
         */
        qdev_prop_set_bit(armv7m, "start-powered-off", i != 0);
        qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
        qdev_connect_clock_in(armv7m, "refclk", s->refclk);
        memory_region_init_alias(&s->core_memory[i], obj, "rp2350.core-memory",
                                 s->board_memory, 0, UINT64_MAX);
        object_property_set_link(OBJECT(armv7m), "memory",
                                 OBJECT(&s->core_memory[i]), &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(armv7m), errp)) {
            return;
        }
    }

    for (i = 0; i < ARRAY_SIZE(rp2350_peripherals); i++) {
        create_unimplemented_device(rp2350_peripherals[i].name,
                                    rp2350_peripherals[i].base,
                                    rp2350_peripherals[i].size);
    }
}

static void rp2350_soc_init(Object *obj)
{
    RP2350State *s = RP2350_SOC(obj);
    int i;

    for (i = 0; i < RP2350_NUM_CORES; i++) {
        object_initialize_child(obj, "armv7m[*]", &s->armv7m[i], TYPE_ARMV7M);
    }

    s->sysclk = qdev_init_clock_out(DEVICE(s), "sysclk");
    s->refclk = qdev_init_clock_out(DEVICE(s), "refclk");
}

static const Property rp2350_soc_properties[] = {
    DEFINE_PROP_LINK("memory", RP2350State, board_memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("flash-size", RP2350State, flash_size,
                       RP2350_FLASH_DEFAULT_SIZE),
    DEFINE_PROP_UINT32("init-svtor", RP2350State, init_svtor, RP2350_ROM_BASE),
};

static void rp2350_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = rp2350_soc_realize;
    device_class_set_props(dc, rp2350_soc_properties);
}

static const TypeInfo rp2350_soc_info = {
    .name          = TYPE_RP2350_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350State),
    .instance_init = rp2350_soc_init,
    .class_init    = rp2350_soc_class_init,
};

static void rp2350_soc_types(void)
{
    type_register_static(&rp2350_soc_info);
}
type_init(rp2350_soc_types)
