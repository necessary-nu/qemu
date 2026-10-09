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
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/unimp.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include "target/arm/multiprocessing.h"

typedef struct RP2350Peripheral {
    const char *name;
    hwaddr base;
    hwaddr size;
} RP2350Peripheral;

/*
 * The whole peripheral address map. Each block is mapped as a low-priority
 * unimplemented device, so guest accesses outside a modelled block log the
 * block's name rather than raise a bus fault on unassigned memory. Device
 * models are mapped over their block at normal priority.
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
    { "rp2350.xip",             RP2350_XIP_BASE,
                                RP2350_XIP_SRAM_BASE - RP2350_XIP_BASE },
    { "rp2350.xip_sram",        RP2350_XIP_SRAM_BASE, 0x4000 },
    { "rp2350.xip_nocache_noalloc", RP2350_XIP_NOCACHE_NOALLOC_BASE,
                                RP2350_XIP_WINDOW_SIZE },
    { "rp2350.xip_maintenance", RP2350_XIP_MAINTENANCE_BASE,
                                RP2350_XIP_WINDOW_SIZE },
    { "rp2350.xip_nocache_noalloc_notranslate",
                                RP2350_XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE,
                                RP2350_XIP_WINDOW_SIZE },
    { "rp2350.sio",             0xd0000000, 0x20000 },
    { "rp2350.sio_nonsec",      0xd0020000, 0x20000 },
};

/* The boot ROM enables the RCP (coprocessor 7) for Secure and Non-secure. */
#define CPACR_CP7 (3u << 14)
#define NSACR_CP7 (1u << 7)

/* [spec:nuos:req:emu.rcp-handoff] */
void rp2350_soc_boot_rom_handoff(RP2350State *s, int core)
{
    CPUARMState *env = &s->armv7m[core].cpu->env;

    env->v7m.cpacr[M_REG_S] |= CPACR_CP7;
    env->v7m.cpacr[M_REG_NS] |= CPACR_CP7;
    env->v7m.nsacr |= NSACR_CP7;
}

typedef struct RP2350Core1Start {
    RP2350State *soc;
    uint32_t sp;
    uint32_t entry;
} RP2350Core1Start;

/* Runs on core 1 after its reset, which loaded VTOR from init-svtor. */
static void rp2350_core1_start(CPUState *cs, run_on_cpu_data data)
{
    RP2350Core1Start *start = data.host_ptr;

    ARM_CPU(cs)->env.regs[13] = start->sp & ~3u;
    cpu_set_pc(cs, start->entry);
    rp2350_soc_boot_rom_handoff(start->soc, 1);
    g_free(start);
}

/*
 * Core 1 is launched the way the boot ROM does it: reset onto the given
 * vector table, then jump to the entry point on the given stack.
 */
/* [spec:nuos:req:emu.core1-launch] */
static void rp2350_core1_launch(void *opaque, uint32_t vtor, uint32_t sp,
                                uint32_t entry)
{
    RP2350State *s = opaque;
    ARMCPU *cpu = s->armv7m[1].cpu;
    RP2350Core1Start *start = g_new(RP2350Core1Start, 1);

    object_property_set_uint(OBJECT(cpu), "init-svtor", vtor, &error_abort);
    if (arm_set_cpu_on_and_reset(arm_cpu_mp_affinity(cpu)) !=
        QEMU_ARM_POWERCTL_RET_SUCCESS) {
        g_free(start);
        return;
    }
    start->soc = s;
    start->sp = sp;
    start->entry = entry;
    async_run_on_cpu(CPU(cpu), rp2350_core1_start, RUN_ON_CPU_HOST_PTR(start));
}

/*
 * Atomic XOR/SET/CLR aliases in front of a device model that only has the
 * plain register window: alias writes become read-modify-write on the
 * target register.
 */
static MemTxResult rp2350_alias_read(void *opaque, hwaddr addr,
                                     uint64_t *data, unsigned size,
                                     MemTxAttrs attrs)
{
    return memory_region_dispatch_read(opaque, rp2350_atomic_reg(addr), data,
                                       size_memop(size), attrs);
}

static MemTxResult rp2350_alias_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size,
                                      MemTxAttrs attrs)
{
    hwaddr reg = rp2350_atomic_reg(addr);
    uint64_t old = 0;
    MemTxResult r;

    if (reg != addr) {
        r = memory_region_dispatch_read(opaque, reg, &old, size_memop(size),
                                        attrs);
        if (r != MEMTX_OK) {
            return r;
        }
        value = rp2350_atomic_apply(addr, old, value);
    }
    return memory_region_dispatch_write(opaque, reg, value, size_memop(size),
                                        attrs);
}

static const MemoryRegionOps rp2350_alias_ops = {
    .read_with_attrs = rp2350_alias_read,
    .write_with_attrs = rp2350_alias_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * System-level interrupts are wired to the same IRQ number on both cores'
 * NVICs; each core masks the lines it does not service. Core-local
 * sources (SIO FIFOs, doorbells and MTIMECMP, and the GPIO interrupts) do
 * not use these inputs: their models connect each core's source straight
 * to that core's NVIC.
 */
/* [spec:nuos:req:emu.irq-routing+1] */
static void rp2350_soc_set_irq(void *opaque, int n, int level)
{
    RP2350State *s = opaque;
    int i;

    for (i = 0; i < RP2350_NUM_CORES; i++) {
        qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->armv7m[i]), n), level);
    }
}

/* [spec:nuos:req:emu.machine+1] */
static void rp2350_soc_realize(DeviceState *dev_soc, Error **errp)
{
    RP2350State *s = RP2350_SOC(dev_soc);
    Object *obj = OBJECT(dev_soc);
    int i;

    if (!s->board_memory) {
        error_setg(errp, "memory property was not set");
        return;
    }
    if (s->flash_size > RP2350_FLASH_MAX_SIZE) {
        error_setg(errp, "flash-size must be at most %u MiB",
                   (unsigned)(RP2350_FLASH_MAX_SIZE / MiB));
        return;
    }

    if (!memory_region_init_rom(&s->rom, obj, "rp2350.rom", RP2350_ROM_SIZE,
                                errp)) {
        return;
    }
    memory_region_add_subregion(s->board_memory, RP2350_ROM_BASE, &s->rom);

    /*
     * The chip has no flash of its own; the board supplies it. Without
     * any, the XIP windows hold only their unimplemented-device stubs.
     */
    /* [spec:nuos:req:emu.flash] */
    if (s->flash_size) {
        if (!memory_region_init_rom(&s->flash, obj, "rp2350.flash",
                                    s->flash_size, errp)) {
            return;
        }
        memory_region_add_subregion(s->board_memory, RP2350_XIP_BASE,
                                    &s->flash);
        memory_region_init_alias(&s->flash_nocache_alias, obj,
                                 "rp2350.flash.nocache-noalloc", &s->flash, 0,
                                 s->flash_size);
        memory_region_add_subregion(s->board_memory,
                                    RP2350_XIP_NOCACHE_NOALLOC_BASE,
                                    &s->flash_nocache_alias);
        memory_region_init_alias(&s->flash_notranslate_alias, obj,
                                 "rp2350.flash.nocache-noalloc-notranslate",
                                 &s->flash, 0, s->flash_size);
        memory_region_add_subregion(
            s->board_memory, RP2350_XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE,
            &s->flash_notranslate_alias);
    }

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
        /* QEMU's Cortex-M33 defaults to 16 regions; the RP2350 has 8 each. */
        /* [spec:nuos:req:emu.mpu] */
        qdev_prop_set_uint32(armv7m, "mpu-s-regions", RP2350_MPU_REGIONS);
        qdev_prop_set_uint32(armv7m, "mpu-ns-regions", RP2350_MPU_REGIONS);
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
        /* A core that locks up stops; the other core carries on. */
        /* [spec:nuos:req:emu.lockup] */
        s->armv7m[i].cpu->m_lockup_halts = true;

        /*
         * The extended PPB is core-local and sits inside the PPB range
         * that each armv7m container claims, so it is mapped per core
         * rather than in board memory.
         */
        qdev_prop_set_string(DEVICE(&s->eppb[i]), "name", "rp2350.eppb");
        qdev_prop_set_uint64(DEVICE(&s->eppb[i]), "size", RP2350_EPPB_SIZE);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->eppb[i]), errp)) {
            return;
        }
        memory_region_add_subregion_overlap(
            &s->armv7m[i].container, RP2350_EPPB_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->eppb[i]), 0), 0);
    }

    /* [spec:nuos:req:emu.rcp] */
    qdev_prop_set_bit(DEVICE(&s->rcp), "boot-rom-handoff", s->core1_launch);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcp), errp)) {
        return;
    }
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        rp2350_rcp_attach(&s->rcp, i, s->armv7m[i].cpu);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->rcp), i,
                           qdev_get_gpio_in_named(DEVICE(&s->armv7m[i]),
                                                  "NMI", 0));
    }

    /* [spec:nuos:req:emu.dcp-state] */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->dcp), errp)) {
        return;
    }
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        rp2350_dcp_attach(&s->dcp, i, s->armv7m[i].cpu);
    }

    /* [spec:nuos:req:emu.sio] */
    qdev_prop_set_bit(DEVICE(&s->sio), "core1-launch", s->core1_launch);
    rp2350_sio_set_core1_launch(&s->sio, rp2350_core1_launch, s);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sio), errp)) {
        return;
    }
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        int bank;

        rp2350_sio_attach_gpioc(&s->sio, i, s->armv7m[i].cpu);
        memory_region_add_subregion_overlap(&s->armv7m[i].container,
                                            RP2350_SIO_BASE,
                                            rp2350_sio_view(&s->sio, i, false),
                                            0);
        memory_region_add_subregion_overlap(&s->armv7m[i].container,
                                            RP2350_SIO_NONSEC_BASE,
                                            rp2350_sio_view(&s->sio, i, true),
                                            0);
        for (bank = 0; bank < RP2350_SIO_BANKS; bank++) {
            int irq = (bank * RP2350_SIO_CORES + i) * 2;

            sysbus_connect_irq(SYS_BUS_DEVICE(&s->sio), irq,
                qdev_get_gpio_in(DEVICE(&s->armv7m[i]),
                                 RP2350_SIO_IRQ_FIFO + 2 * bank));
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->sio), irq + 1,
                qdev_get_gpio_in(DEVICE(&s->armv7m[i]),
                                 RP2350_SIO_IRQ_BELL + 2 * bank));
        }
    }
    memory_region_init_alias(&s->sio_sysmem[0], obj, "rp2350-sio.sysmem",
                             rp2350_sio_view(&s->sio, 0, false), 0,
                             RP2350_SIO_VIEW_SIZE);
    memory_region_add_subregion(s->board_memory, RP2350_SIO_BASE,
                                &s->sio_sysmem[0]);
    memory_region_init_alias(&s->sio_sysmem[1], obj,
                             "rp2350-sio-nonsec.sysmem",
                             rp2350_sio_view(&s->sio, 0, true), 0,
                             RP2350_SIO_VIEW_SIZE);
    memory_region_add_subregion(s->board_memory, RP2350_SIO_NONSEC_BASE,
                                &s->sio_sysmem[1]);

    {
        const struct {
            SysBusDevice *dev;
            hwaddr base;
        } blocks[] = {
            { SYS_BUS_DEVICE(&s->resets), RP2350_RESETS_BASE },
            { SYS_BUS_DEVICE(&s->clocks), RP2350_CLOCKS_BASE },
            { SYS_BUS_DEVICE(&s->xosc), RP2350_XOSC_BASE },
            { SYS_BUS_DEVICE(&s->pll_sys), RP2350_PLL_SYS_BASE },
            { SYS_BUS_DEVICE(&s->pll_usb), RP2350_PLL_USB_BASE },
            { SYS_BUS_DEVICE(&s->ticks), RP2350_TICKS_BASE },
            { SYS_BUS_DEVICE(&s->bootram), RP2350_BOOTRAM_BASE },
            { SYS_BUS_DEVICE(&s->sysinfo), RP2350_SYSINFO_BASE },
            { SYS_BUS_DEVICE(&s->syscfg), RP2350_SYSCFG_BASE },
            { SYS_BUS_DEVICE(&s->tbman), RP2350_TBMAN_BASE },
            { SYS_BUS_DEVICE(&s->glitch_detector),
              RP2350_GLITCH_DETECTOR_BASE },
            { SYS_BUS_DEVICE(&s->dft), RP2350_DFT_BASE },
        };

        for (i = 0; i < ARRAY_SIZE(blocks); i++) {
            if (!sysbus_realize(blocks[i].dev, errp)) {
                return;
            }
            sysbus_mmio_map(blocks[i].dev, 0, blocks[i].base);
        }
    }

    /* [spec:nuos:req:emu.timer] */
    for (i = 0; i < RP2350_NUM_TIMERS; i++) {
        static const hwaddr base[] = {
            RP2350_TIMER0_BASE, RP2350_TIMER1_BASE,
        };
        static const int irq[] = { RP2350_TIMER0_IRQ_0, RP2350_TIMER1_IRQ_0 };
        static const int tick[] = { RP2350_TICK_TIMER0, RP2350_TICK_TIMER1 };
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->timer[i]);
        int n;

        object_property_set_link(OBJECT(sbd), "ticks", OBJECT(&s->ticks),
                                 &error_abort);
        qdev_prop_set_uint32(DEVICE(sbd), "tick", tick[i]);
        qdev_prop_set_uint32(DEVICE(sbd), "sysclk-hz", RP2350_SYSCLK_HZ);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, base[i]);
        for (n = 0; n < RP2350_TIMER_ALARMS; n++) {
            sysbus_connect_irq(sbd, n, qdev_get_gpio_in(dev_soc, irq[i] + n));
        }
    }

    /* [spec:nuos:req:emu.uart] */
    for (i = 0; i < RP2350_NUM_UARTS; i++) {
        static const hwaddr base[] = { RP2350_UART0_BASE, RP2350_UART1_BASE };
        static const int irq[] = { RP2350_UART0_IRQ, RP2350_UART1_IRQ };
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->uart[i]);

        qdev_prop_set_chr(DEVICE(sbd), "chardev", serial_hd(i));
        qdev_connect_clock_in(DEVICE(sbd), "clk", s->sysclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        memory_region_init_io(&s->uart_alias[i], obj, &rp2350_alias_ops,
                              sysbus_mmio_get_region(sbd, 0),
                              i ? "rp2350-uart1" : "rp2350-uart0",
                              RP2350_ATOMIC_REGION_SIZE);
        memory_region_add_subregion(s->board_memory, base[i],
                                    &s->uart_alias[i]);
        /* Output 0 is the PL011's combined UARTINTR. */
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(dev_soc, irq[i]));
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
        object_initialize_child(obj, "eppb[*]", &s->eppb[i],
                                TYPE_UNIMPLEMENTED_DEVICE);
    }

    object_initialize_child(obj, "resets", &s->resets, TYPE_RP2350_RESETS);
    object_initialize_child(obj, "sio", &s->sio, TYPE_RP2350_SIO);
    object_initialize_child(obj, "rcp", &s->rcp, TYPE_RP2350_RCP);
    object_initialize_child(obj, "bootram", &s->bootram, TYPE_RP2350_BOOTRAM);
    object_initialize_child(obj, "dcp", &s->dcp, TYPE_RP2350_DCP);
    object_initialize_child(obj, "clocks", &s->clocks, TYPE_RP2350_CLOCKS);
    object_initialize_child(obj, "xosc", &s->xosc, TYPE_RP2350_XOSC);
    object_initialize_child(obj, "pll_sys", &s->pll_sys, TYPE_RP2350_PLL);
    object_initialize_child(obj, "pll_usb", &s->pll_usb, TYPE_RP2350_PLL);
    object_initialize_child(obj, "ticks", &s->ticks, TYPE_RP2350_TICKS);
    /* [spec:nuos:req:emu.system-regs] */
    object_initialize_child(obj, "sysinfo", &s->sysinfo, TYPE_RP2350_SYSINFO);
    object_initialize_child(obj, "syscfg", &s->syscfg, TYPE_RP2350_SYSCFG);
    object_initialize_child(obj, "tbman", &s->tbman, TYPE_RP2350_TBMAN);
    object_initialize_child(obj, "glitch_detector", &s->glitch_detector,
                            TYPE_RP2350_GLITCH_DETECTOR);
    object_initialize_child(obj, "dft", &s->dft, TYPE_RP2350_DFT);
    for (i = 0; i < RP2350_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_RP2350_TIMER);
    }
    for (i = 0; i < RP2350_NUM_UARTS; i++) {
        object_initialize_child(obj, "uart[*]", &s->uart[i], TYPE_PL011);
    }

    qdev_init_gpio_in(DEVICE(s), rp2350_soc_set_irq, RP2350_NUM_IRQS);

    s->sysclk = qdev_init_clock_out(DEVICE(s), "sysclk");
    s->refclk = qdev_init_clock_out(DEVICE(s), "refclk");
}

static const Property rp2350_soc_properties[] = {
    DEFINE_PROP_LINK("memory", RP2350State, board_memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("flash-size", RP2350State, flash_size, 0),
    DEFINE_PROP_UINT32("init-svtor", RP2350State, init_svtor, RP2350_ROM_BASE),
    /* Emulate the boot ROM's core 1 launch handshake (no ROM executing). */
    DEFINE_PROP_BOOL("core1-launch", RP2350State, core1_launch, false),
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
