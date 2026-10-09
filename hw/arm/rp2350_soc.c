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
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/rp2350_soc.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/block/aps6404l.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/unimp.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "system/cpus.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "system/system.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include "target/arm/internals.h"
#include "target/arm/multiprocessing.h"
#include "target/arm/tcg/idau.h"

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
    { "rp2350.sio",             0xd0000000, 0x20000 },
    { "rp2350.sio_nonsec",      0xd0020000, 0x20000 },
};

/*
 * The RP2350 IDAU (datasheet "IDAU address map"), a fixed address decode.
 * Within the 32 KiB ROM map, mirrored every 32 KiB below the XIP window:
 * the Arm boot code is Exempt, the USB/RISC-V boot code is Non-secure for
 * instruction fetch but Exempt for loads and stores, and the final 512
 * bytes hold the Secure Gateways, Secure and Non-secure-callable. The
 * peripherals and SIO are Exempt; XIP, SRAM and everything else are
 * Non-secure. Only the Secure Gateway region reports a region number, 2,
 * as the boot ROM's start-up checks of its TT results expect.
 */
#define RP2350_IDAU_ROM_MAP      0x8000
#define RP2350_IDAU_NSBOOT_START 0x4300
#define RP2350_IDAU_SG_START     0x7e00
#define RP2350_IDAU_SG_REGION    2

/* [spec:nuos:req:emu.machine+1] */
static void rp2350_idau_check(IDAUInterface *ii, uint32_t address,
                              MMUAccessType access_type, int *iregion,
                              bool *exempt, bool *ns, bool *nsc,
                              uint32_t *base, uint32_t *limit)
{
    uint32_t block = address & ~(uint32_t)(RP2350_XIP_BASE - 1);

    *iregion = IREGION_NOTVALID;
    *exempt = false;
    *ns = true;
    *nsc = false;

    if (address < RP2350_XIP_BASE) {
        uint32_t map = address & ~(uint32_t)(RP2350_IDAU_ROM_MAP - 1);
        uint32_t off = address - map;

        if (off < RP2350_IDAU_NSBOOT_START) {
            *exempt = true;
            *base = map;
            *limit = map + RP2350_IDAU_NSBOOT_START - 1;
        } else if (off < RP2350_IDAU_SG_START) {
            /* The attribute depends on the access type. */
            *exempt = access_type != MMU_INST_FETCH;
            *base = address;
            *limit = address;
        } else {
            *ns = false;
            *nsc = true;
            *iregion = RP2350_IDAU_SG_REGION;
            *base = map + RP2350_IDAU_SG_START;
            *limit = map + RP2350_IDAU_ROM_MAP - 1;
        }
        return;
    }

    switch (block >> 28) {
    case 0x4: /* APB peripherals */
    case 0x5: /* AHB peripherals */
    case 0xd: /* SIO */
        *exempt = true;
        break;
    default:
        break;
    }
    *base = block;
    *limit = block + (RP2350_XIP_BASE - 1);
}

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

typedef struct RP2350ResetWindow {
    hwaddr base;
    hwaddr size;
} RP2350ResetWindow;

/*
 * Each RESETS subsystem and the bus windows of its registers. JTAG has no
 * registers on the system bus.
 */
static const struct {
    const char *name;
    RP2350ResetWindow window[RP2350_RESET_MAX_WINDOWS];
} rp2350_reset_blocks[RP2350_NUM_RESETS] = {
    [RP2350_RESET_ADC]        = { "adc", { { 0x400a0000, 0x8000 } } },
    [RP2350_RESET_BUSCTRL]    = { "busctrl", { { 0x40068000, 0x8000 } } },
    [RP2350_RESET_DMA]        = { "dma", { { 0x50000000, 0x100000 } } },
    [RP2350_RESET_HSTX]       = { "hstx", { { 0x400c0000, 0x8000 },
                                            { 0x50600000, 0x100000 } } },
    [RP2350_RESET_I2C0]       = { "i2c0", { { 0x40090000, 0x8000 } } },
    [RP2350_RESET_I2C1]       = { "i2c1", { { 0x40098000, 0x8000 } } },
    [RP2350_RESET_IO_BANK0]   = { "io_bank0", { { 0x40028000, 0x8000 } } },
    [RP2350_RESET_IO_QSPI]    = { "io_qspi", { { 0x40030000, 0x8000 } } },
    [RP2350_RESET_JTAG]       = { "jtag" },
    [RP2350_RESET_PADS_BANK0] = { "pads_bank0", { { 0x40038000, 0x8000 } } },
    [RP2350_RESET_PADS_QSPI]  = { "pads_qspi", { { 0x40040000, 0x8000 } } },
    [RP2350_RESET_PIO0]       = { "pio0", { { 0x50200000, 0x100000 } } },
    [RP2350_RESET_PIO1]       = { "pio1", { { 0x50300000, 0x100000 } } },
    [RP2350_RESET_PIO2]       = { "pio2", { { 0x50400000, 0x100000 } } },
    [RP2350_RESET_PLL_SYS]    = { "pll_sys", { { 0x40050000, 0x8000 } } },
    [RP2350_RESET_PLL_USB]    = { "pll_usb", { { 0x40058000, 0x8000 } } },
    [RP2350_RESET_PWM]        = { "pwm", { { 0x400a8000, 0x8000 } } },
    [RP2350_RESET_SHA256]     = { "sha256", { { 0x400f8000, 0x8000 } } },
    [RP2350_RESET_SPI0]       = { "spi0", { { 0x40080000, 0x8000 } } },
    [RP2350_RESET_SPI1]       = { "spi1", { { 0x40088000, 0x8000 } } },
    [RP2350_RESET_SYSCFG]     = { "syscfg", { { 0x40008000, 0x8000 } } },
    [RP2350_RESET_SYSINFO]    = { "sysinfo", { { 0x40000000, 0x8000 } } },
    [RP2350_RESET_TBMAN]      = { "tbman", { { 0x40160000, 0x8000 } } },
    [RP2350_RESET_TIMER0]     = { "timer0", { { 0x400b0000, 0x8000 } } },
    [RP2350_RESET_TIMER1]     = { "timer1", { { 0x400b8000, 0x8000 } } },
    [RP2350_RESET_TRNG]       = { "trng", { { 0x400f0000, 0x8000 } } },
    [RP2350_RESET_UART0]      = { "uart0", { { 0x40070000, 0x8000 } } },
    [RP2350_RESET_UART1]      = { "uart1", { { 0x40078000, 0x8000 } } },
    [RP2350_RESET_USBCTRL]    = { "usbctrl", { { 0x50100000, 0x100000 } } },
};

/*
 * The IO and pad banks are one device model, whose register blocks are
 * held in reset individually.
 */
static const struct {
    int reset;
    unsigned block;
} rp2350_gpio_resets[] = {
    { RP2350_RESET_IO_BANK0, RP2350_GPIO_IO_BANK0 },
    { RP2350_RESET_IO_QSPI, RP2350_GPIO_IO_QSPI },
    { RP2350_RESET_PADS_BANK0, RP2350_GPIO_PADS_BANK0 },
    { RP2350_RESET_PADS_QSPI, RP2350_GPIO_PADS_QSPI },
};

/*
 * A subsystem in reset does not answer on the bus: reads of its registers
 * return zero and writes have no effect, without a bus error. Erratum
 * RP2350-E23 documents this: SYSINFO, left in reset, reads as zero.
 */
/* [spec:nuos:req:emu.resets] */
static MemTxResult rp2350_reset_gate_read(void *opaque, hwaddr addr,
                                          uint64_t *data, unsigned size,
                                          MemTxAttrs attrs)
{
    RP2350ResetGate *gate = opaque;

    qemu_log_mask(LOG_GUEST_ERROR, "rp2350: read of %s offset 0x%"
                  HWADDR_PRIx " while it is in reset\n", gate->name, addr);
    *data = 0;
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.resets] */
static MemTxResult rp2350_reset_gate_write(void *opaque, hwaddr addr,
                                           uint64_t value, unsigned size,
                                           MemTxAttrs attrs)
{
    RP2350ResetGate *gate = opaque;

    qemu_log_mask(LOG_GUEST_ERROR, "rp2350: write to %s offset 0x%"
                  HWADDR_PRIx " while it is in reset\n", gate->name, addr);
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_reset_gate_ops = {
    .read_with_attrs = rp2350_reset_gate_read,
    .write_with_attrs = rp2350_reset_gate_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.resets] */
void rp2350_soc_attach_reset(RP2350State *s, int reset, DeviceState *dev)
{
    int i;

    assert(reset >= 0 && reset < RP2350_NUM_RESETS);
    for (i = 0; i < RP2350_RESET_MAX_DEVICES; i++) {
        if (!s->reset_dev[reset][i]) {
            s->reset_dev[reset][i] = dev;
            return;
        }
    }
    g_assert_not_reached();
}

/*
 * RESETS puts subsystems into reset and takes them out. Each attached
 * device model is held in reset through its Resettable reset, so it
 * resets on entry and stays inert until released; the subsystem's bus
 * windows are gated for as long.
 */
/* [spec:nuos:req:emu.resets] */
static void rp2350_soc_reset_hold(void *opaque, uint32_t blocks, bool hold)
{
    RP2350State *s = opaque;
    unsigned gpio = 0;
    int i, n;

    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        if (!(blocks & BIT(i))) {
            continue;
        }
        for (n = 0; n < RP2350_RESET_MAX_DEVICES && s->reset_dev[i][n];
             n++) {
            Object *obj = OBJECT(s->reset_dev[i][n]);

            if (hold) {
                resettable_assert_reset(obj, RESET_TYPE_COLD);
            } else {
                resettable_release_reset(obj, RESET_TYPE_COLD);
            }
        }
        for (n = 0; n < RP2350_RESET_MAX_WINDOWS; n++) {
            if (rp2350_reset_blocks[i].window[n].size) {
                memory_region_set_enabled(&s->reset_gate[i][n].mr, hold);
            }
        }
    }
    for (i = 0; i < ARRAY_SIZE(rp2350_gpio_resets); i++) {
        if (blocks & BIT(rp2350_gpio_resets[i].reset)) {
            gpio |= rp2350_gpio_resets[i].block;
        }
    }
    if (gpio) {
        rp2350_gpio_hold_blocks(&s->gpio, gpio, hold);
    }
}

static void rp2350_soc_init_reset_gates(RP2350State *s)
{
    int i, n;

    for (i = 0; i < RP2350_NUM_RESETS; i++) {
        for (n = 0; n < RP2350_RESET_MAX_WINDOWS; n++) {
            const RP2350ResetWindow *w = &rp2350_reset_blocks[i].window[n];
            RP2350ResetGate *gate = &s->reset_gate[i][n];
            g_autofree char *name = NULL;

            if (!w->size) {
                continue;
            }
            gate->name = rp2350_reset_blocks[i].name;
            name = g_strdup_printf("rp2350.%s-in-reset", gate->name);
            memory_region_init_io(&gate->mr, OBJECT(s),
                                  &rp2350_reset_gate_ops, gate, name,
                                  w->size);
            memory_region_set_enabled(&gate->mr, false);
            memory_region_add_subregion_overlap(s->board_memory, w->base,
                                                &gate->mr, 1);
        }
    }
}

/*
 * The boot ROM's boot vectors, POWMAN BOOT0-3 and then watchdog
 * SCRATCH4-7: the magic number, the entry point XORed with the magic
 * number's negation, the stack pointer and the entry point.
 */
#define VECTORED_BOOT_MAGIC 0xb007c0d3u

/*
 * What the boot ROM's try_vector does with one vector, with no ROM
 * executing: a valid vector is consumed (its magic number cleared) and,
 * unless BOOTDIS disables vectors, called on its stack. A vector that
 * returns continues into the directly loaded image, as the ROM continues
 * into flash boot. The watchdog vector's one-shot boot types (an entry
 * point equal to the magic number) select BOOTSEL, RAM image or flash
 * update boots, which need the ROM itself; the POWMAN vector has none, so
 * the ROM calls that entry point like any other. Returns whether core 0
 * was sent to the vector.
 */
/* [spec:nuos:req:emu.watchdog] */
/* [spec:nuos:req:emu.powman] */
static bool rp2350_soc_try_vector(RP2350State *s, uint32_t *vector,
                                  bool bootdis, bool boot_types)
{
    uint32_t pc = vector[3];
    ARMCPU *cpu = s->armv7m[0].cpu;
    CPUARMState *env = &cpu->env;

    if (vector[0] != VECTORED_BOOT_MAGIC ||
        vector[1] != (pc ^ -VECTORED_BOOT_MAGIC)) {
        return false;
    }
    vector[0] = 0;
    if (pc == VECTORED_BOOT_MAGIC && boot_types) {
        qemu_log_mask(LOG_UNIMP, "rp2350: watchdog boot type %" PRIu32
                      " needs the boot ROM; starting the loaded image\n",
                      vector[2]);
        return false;
    }
    if (bootdis) {
        return false;
    }
    if (!(pc & 1)) {
        /* The ROM hangs rather than enter code of the wrong architecture. */
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350: boot vector entry point "
                      "0x%08" PRIx32 " is not Thumb code; core 0 hangs\n", pc);
        CPU(cpu)->halted = 1;
        return true;
    }
    env->regs[14] = env->regs[15] | 1;
    env->regs[13] = vector[2] & ~3u;
    env->regs[15] = pc & ~1u;
    return true;
}

/*
 * The boot ROM's vector checks, POWMAN's first. The ROM tries the
 * watchdog vector once a POWMAN vector returns; here a POWMAN vector
 * returns into the loaded image instead. Either BOOTDIS.NOW flag (OTP or
 * POWMAN) disables both, and the ROM then clears both.
 */
/* [spec:nuos:req:emu.powman] */
static void rp2350_soc_boot_vectors(RP2350State *s)
{
    bool bootdis = (s->otp.bootdis | s->powman.bootdis) &
                   RP2350_POWMAN_BOOTDIS_NOW;

    if (!rp2350_soc_try_vector(s, s->powman.boot, bootdis, false)) {
        rp2350_soc_try_vector(s, &s->watchdog.scratch[4], bootdis, true);
    }
    s->otp.bootdis &= ~RP2350_POWMAN_BOOTDIS_NOW;
    s->powman.bootdis &= ~RP2350_POWMAN_BOOTDIS_NOW;
}

/* Power a core off: it stays halted until a reset powers it on. */
static void rp2350_soc_core_off(RP2350State *s, int n)
{
    ARMCPU *cpu = s->armv7m[n].cpu;

    CPU(cpu)->halted = 1;
    arm_set_cpu_power_state(cpu, PSCI_OFF);
}

/*
 * Reset one core and what belongs to it: its NVIC, EPPB and SysTicks, and
 * its RCP and DCP state. A core held in reset is left powered off. Otherwise
 * it starts as at power-on: from the ROM, or, with no ROM executing,
 * core 0 as the ROM would hand it over and core 1 waiting to be launched.
 */
/* [spec:nuos:req:emu.watchdog] */
static void rp2350_soc_reset_core(RP2350State *s, int n, bool hold,
                                  bool sio_reset)
{
    ARMv7MState *m = &s->armv7m[n];
    CPUState *cs = CPU(m->cpu);

    device_cold_reset(DEVICE(&m->nvic));
    device_cold_reset(DEVICE(&s->eppb[n]));
    device_cold_reset(DEVICE(&m->systick[M_REG_NS]));
    if (DEVICE(&m->systick[M_REG_S])->realized) {
        device_cold_reset(DEVICE(&m->systick[M_REG_S]));
    }
    rp2350_rcp_reset_core(&s->rcp, n);
    rp2350_dcp_reset_core(&s->dcp, n);
    cpu_reset(cs);
    if (hold) {
        rp2350_soc_core_off(s, n);
        return;
    }
    if (!s->core1_launch) {
        return;
    }
    if (n == 0) {
        rp2350_resets_boot_rom_handoff(&s->resets);
        rp2350_soc_boot_rom_handoff(s, 0);
        rp2350_soc_boot_vectors(s);
    } else if (!sio_reset) {
        rp2350_sio_core1_reset(&s->sio);
    }
}

/*
 * Carry out a PSM sequence: reset the device models in each stage being
 * reset, and the subsystems the watchdog resets. Stages without device
 * models (OTP, PSM_READY, BUSFABRIC, ROM, SRAM0-9) have no state here to
 * reset; SRAM keeps its contents, as on hardware. The watchdog and PSM
 * are reset only by chip-level resets, so the watchdog's scratch
 * registers and REASON survive.
 */
/* [spec:nuos:req:emu.watchdog] */
static void rp2350_soc_psm_reset(void *opaque, uint32_t reset, uint32_t held,
                                 bool watchdog)
{
    RP2350State *s = opaque;
    uint32_t subsys = watchdog ? s->resets.wdsel : 0;
    bool sio_reset = reset & BIT(RP2350_PSM_SIO);
    int i;

    if (reset & BIT(RP2350_PSM_ROSC)) {
        device_cold_reset(DEVICE(&s->rosc));
    }
    if (reset & BIT(RP2350_PSM_XOSC)) {
        device_cold_reset(DEVICE(&s->xosc));
    }
    if (reset & BIT(RP2350_PSM_RESETS)) {
        /* The reset controller asserts every subsystem reset. */
        device_cold_reset(DEVICE(&s->resets));
        subsys = 0;
    }
    if (reset & BIT(RP2350_PSM_CLOCKS)) {
        device_cold_reset(DEVICE(&s->clocks));
        device_cold_reset(DEVICE(&s->ticks));
    }
    if (reset & BIT(RP2350_PSM_BOOTRAM)) {
        rp2350_bootram_reset_regs(&s->bootram);
    }
    if (reset & BIT(RP2350_PSM_XIP)) {
        rp2350_xip_reset_block(&s->xip);
    }
    if (sio_reset) {
        device_cold_reset(DEVICE(&s->sio));
    }
    if (reset & BIT(RP2350_PSM_ACCESSCTRL)) {
        device_cold_reset(DEVICE(&s->accessctrl));
    }

    /*
     * A watchdog reset of a subsystem pulses its reset: the block restarts
     * from its reset state, but RESETS.RESET keeps what software wrote.
     */
    rp2350_resets_pulse(&s->resets, subsys);

    for (i = 0; i < RP2350_NUM_CORES; i++) {
        uint32_t stage = BIT(RP2350_PSM_PROC0 + i);

        if (reset & (stage | BIT(RP2350_PSM_PROC_COLD))) {
            rp2350_soc_reset_core(s, i, held & stage, sio_reset);
        }
    }
}

/*
 * A core's AIRCR.SYSRESETREQ is a warm reset of that core alone, "not the
 * wider system" (datasheet, M33 AIRCR): the other core, the peripherals,
 * SRAM, the PSM and POWMAN carry on, and no chip-level reset reason is
 * recorded. The core restarts as after any processor reset: from the ROM,
 * or with no ROM executing, core 0 into the loaded image (through the boot
 * vectors) and core 1 back to waiting for its launch. As on RP2350 a warm
 * reset of either core by SYSRESETREQ clears the watchdog's REASON, so
 * that code started afresh does not see a stale timeout.
 *
 * The requesting core stops at the end of its current instruction block
 * and the reset runs once every vCPU has paused, as the PSM's processor
 * resets do; the other core resumes where it was.
 */
/* [spec:nuos:req:emu.watchdog] */
static void rp2350_soc_sysresetreq_run(void *opaque)
{
    RP2350State *s = opaque;
    uint32_t cores = s->sysresetreq_pending;
    bool pause = !qtest_enabled();
    int i;

    if (!cores) {
        return;
    }
    s->sysresetreq_pending = 0;
    if (pause) {
        pause_all_vcpus();
    }
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        if (cores & BIT(i)) {
            rp2350_soc_reset_core(s, i, false, false);
        }
    }
    s->watchdog.reason = 0;
    if (pause) {
        resume_all_vcpus();
    }
}

/* [spec:nuos:req:emu.watchdog] */
static void rp2350_soc_sysresetreq(void *opaque, int n, int level)
{
    RP2350State *s = opaque;

    if (!level) {
        return;
    }
    s->sysresetreq_pending |= BIT(n);
    if (qtest_enabled()) {
        rp2350_soc_sysresetreq_run(s);
        return;
    }
    cpu_stop_current();
    qemu_bh_schedule(s->sysresetreq_bh);
}

/*
 * Atomic XOR/SET/CLR aliases in front of a device model that only has the
 * plain register window: alias writes become read-modify-write on the
 * target register. The registers are 32 bits wide: a narrow write (from a
 * core or a byte-wide DMA channel) is replicated across the bus and
 * writes the whole register, and a narrow read returns its lanes.
 */
static MemTxResult rp2350_alias_read(void *opaque, hwaddr addr,
                                     uint64_t *data, unsigned size,
                                     MemTxAttrs attrs)
{
    uint64_t v = 0;
    MemTxResult r;

    r = memory_region_dispatch_read(opaque, rp2350_atomic_reg(addr & ~3), &v,
                                    MO_32, attrs);
    *data = extract64(v, (addr & 3) * 8, size * 8);
    return r;
}

static MemTxResult rp2350_alias_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size,
                                      MemTxAttrs attrs)
{
    hwaddr reg = rp2350_atomic_reg(addr & ~3);
    uint64_t old = 0;
    MemTxResult r;

    if (size == 1) {
        value = (value & 0xff) * 0x01010101u;
    } else if (size == 2) {
        value = (value & 0xffff) * 0x00010001u;
    }
    if (reg != (addr & ~3)) {
        r = memory_region_dispatch_read(opaque, reg, &old, MO_32, attrs);
        if (r != MEMTX_OK) {
            return r;
        }
        value = rp2350_atomic_apply(addr, old, value);
    }
    return memory_region_dispatch_write(opaque, reg, value, MO_32, attrs);
}

static const MemoryRegionOps rp2350_alias_ops = {
    .read_with_attrs = rp2350_alias_read,
    .write_with_attrs = rp2350_alias_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
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
        qemu_set_irq(rp2350_soc_core_irq(s, i, n), level);
    }
}

/*
 * A core's IRQ lines pass through its EPPB on the way to its NVIC, as the
 * EPPB's NMI mask taps every system IRQ the core receives.
 */
/* [spec:nuos:req:emu.eppb] */
qemu_irq rp2350_soc_core_irq(RP2350State *s, int core, int n)
{
    return qdev_get_gpio_in(DEVICE(&s->eppb[core]), n);
}

/* The IO and pad banks follow ACCESSCTRL's GPIO Non-secure masks. */
/* [spec:nuos:req:emu.accessctrl] */
static void rp2350_soc_accessctrl_changed(Notifier *n, void *data)
{
    RP2350State *s = container_of(n, RP2350State, accessctrl_notifier);

    rp2350_gpio_set_nsmask(&s->gpio,
        (uint64_t)rp2350_accessctrl_gpio_nsmask(&s->accessctrl, 1) << 32 |
        rp2350_accessctrl_gpio_nsmask(&s->accessctrl, 0));
}

/*
 * nSSPCTLOE of SPI controller n, the active-low output enable of its
 * clock and frame select, which drive SCK and CSn: outputs in master
 * mode, inputs in slave mode.
 */
/* [spec:nuos:req:emu.spi] */
static void rp2350_soc_spi_ctloe(void *opaque, int n, int level)
{
    RP2350State *s = opaque;
    RP2350GPIOPort port = n ? RP2350_GPIO_PORT_SPI1 : RP2350_GPIO_PORT_SPI0;

    qemu_set_irq(rp2350_gpio_oe_line(&s->gpio, port, RP2350_GPIO_SPI_SCK),
                 !level);
    qemu_set_irq(rp2350_gpio_oe_line(&s->gpio, port, RP2350_GPIO_SPI_CSN),
                 !level);
}

/*
 * SSI devices on the SPI buses ("spi0", "spi1", e.g. -device
 * w25q80bl,bus=spi0,cs=17) are off-chip devices: a device's chip select
 * follows the level of the GPIO pin its "cs" property numbers, whether
 * the SPI controller's CSn, SIO or anything else drives that pin. Devices
 * are plugged after the SoC is realized, so they are wired once the
 * machine is complete. Devices without an SSI chip select, such as a
 * PL022 slave port, are left alone.
 */
/* [spec:nuos:req:emu.spi] */
static void rp2350_soc_spi_wire_cs(Notifier *notifier, void *data)
{
    RP2350State *s = container_of(notifier, RP2350State, spi_cs_notifier);
    DECLARE_BITMAP(used, RP2350_GPIO_PINS) = {};
    int i;

    for (i = 0; i < RP2350_NUM_SPIS; i++) {
        BusChild *kid;

        QTAILQ_FOREACH(kid, &BUS(s->spi[i].ssi)->children, sibling) {
            DeviceState *dev = kid->child;
            unsigned pin = SSI_PERIPHERAL(dev)->cs_index;

            if (!object_property_find(OBJECT(dev), SSI_GPIO_CS "[0]")) {
                continue;
            }
            if (pin >= RP2350_GPIO_PINS || test_bit(pin, used)) {
                error_report("rp2350: %s on spi%d: cs=%u must name a GPIO "
                             "pin (0-%d) no other SPI device uses",
                             object_get_typename(OBJECT(dev)), i, pin,
                             RP2350_GPIO_PINS - 1);
                exit(1);
            }
            set_bit(pin, used);
            qdev_connect_gpio_out_named(DEVICE(&s->gpio), RP2350_GPIO_PAD_OUT,
                                        pin,
                                        qdev_get_gpio_in_named(dev,
                                                               SSI_GPIO_CS,
                                                               0));
        }
    }
}

/*
 * The SIO's pin inputs, which also reach the power manager for its
 * power-up and time reference sources. The SIO numbers the QSPI bank's
 * pins from bit 56; the power manager numbers pins as the GPIO block.
 */
/* [spec:nuos:req:emu.powman] */
static void rp2350_soc_sio_in(void *opaque, int n, int level)
{
    RP2350State *s = opaque;

    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->sio), "gpio-in", n),
                 level);
    if (n < RP2350_GPIO_BANK0_PINS) {
        qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->powman),
                                            RP2350_POWMAN_GPIO, n), level);
    } else if (n >= RP2350_GPIO_BANK0_PINS + 8) {
        qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->powman),
                                            RP2350_POWMAN_GPIO, n - 8),
                     level);
    }
}

/*
 * An SRAM power domain that is down. Its contents were lost when it
 * powered down; reads return zero and writes are lost. The datasheet does
 * not say whether the bus faults such accesses, so they do not fault.
 */
static MemTxResult rp2350_sram_off_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350: read of powered-down SRAM\n");
    *data = 0;
    return MEMTX_OK;
}

static MemTxResult rp2350_sram_off_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350: write to powered-down SRAM\n");
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_sram_off_ops = {
    .read_with_attrs = rp2350_sram_off_read,
    .write_with_attrs = rp2350_sram_off_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* The SRAM domain's window in sram_off order: SRAM0, then SRAM1. */
static int rp2350_sram_domain(int domain)
{
    return domain == RP2350_POWMAN_SRAM0 ? 0 : 1;
}

/* [spec:nuos:req:emu.powman] */
static bool rp2350_soc_procs_asleep(void *opaque)
{
    RP2350State *s = opaque;
    int i;

    /* Under qtest no processor executes. */
    if (qtest_enabled()) {
        return true;
    }
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        if (!CPU(s->armv7m[i].cpu)->halted) {
            return false;
        }
    }
    return true;
}

/* [spec:nuos:req:emu.powman] */
static void rp2350_soc_swcore_stop(void *opaque)
{
    RP2350State *s = opaque;
    int i;

    for (i = 0; i < RP2350_NUM_CORES; i++) {
        rp2350_soc_core_off(s, i);
    }
}

/*
 * A memory power domain went down or came up. SRAM and the XIP domain's
 * Boot RAM and cache memories lose their contents when they power down.
 */
/* [spec:nuos:req:emu.powman] */
static void rp2350_soc_domain_power(void *opaque, int domain, bool on)
{
    RP2350State *s = opaque;
    int i;

    if (domain == RP2350_POWMAN_XIP) {
        if (!on) {
            memset(s->bootram.ram, 0, sizeof(s->bootram.ram));
            rp2350_xip_power_down(&s->xip);
        }
        return;
    }
    i = rp2350_sram_domain(domain);
    if (!on) {
        /* The address space's root keeps its address in board memory. */
        address_space_set(&s->sram_as, RP2350_SRAM_BASE +
                          (i ? RP2350_SRAM0_DOMAIN_SIZE : 0), 0,
                          i ? RP2350_SRAM1_DOMAIN_SIZE
                            : RP2350_SRAM0_DOMAIN_SIZE,
                          MEMTXATTRS_UNSPECIFIED);
    }
    memory_region_set_enabled(&s->sram_off[i], !on);
}

static const RP2350PowmanOps rp2350_soc_powman_ops = {
    .procs_asleep = rp2350_soc_procs_asleep,
    .swcore_stop = rp2350_soc_swcore_stop,
    .domain_power = rp2350_soc_domain_power,
};

/* [spec:nuos:req:emu.powman] */
void rp2350_soc_system_reset(RP2350State *s, ResetType type)
{
    int i;

    if (type == RESET_TYPE_COLD) {
        type = rp2350_powman_next_reset_type(&s->powman);
    }
    qemu_devices_reset(type);
    memory_region_set_enabled(&s->sram_off[0],
        !rp2350_powman_domain_on(&s->powman, RP2350_POWMAN_SRAM0));
    memory_region_set_enabled(&s->sram_off[1],
        !rp2350_powman_domain_on(&s->powman, RP2350_POWMAN_SRAM1));
    if (!rp2350_powman_domain_on(&s->powman, RP2350_POWMAN_SWCORE)) {
        for (i = 0; i < RP2350_NUM_CORES; i++) {
            rp2350_soc_core_off(s, i);
        }
        return;
    }
    if (s->core1_launch) {
        rp2350_soc_boot_rom_handoff(s, 0);
        rp2350_soc_boot_vectors(s);
    }
}

/* The Winbond W25Q parts that fit each supported flash size. */
static const char *rp2350_flash_part(uint32_t size)
{
    switch (size) {
    case 1 * MiB:
        return "w25q80bl";
    case 2 * MiB:
        return "w25q16";
    case 4 * MiB:
        return "w25q32";
    case 8 * MiB:
        return "w25q64";
    default:
        return "w25q128";
    }
}

/*
 * The XIP subsystem and the board's QSPI devices: flash on chip select 0,
 * a W25Q part of the configured size, and optionally an APS6404L PSRAM on
 * chip select 1. Without flash, chip select 0 has no device and the XIP
 * windows hold no flash. The flash is erased unless backed by the first
 * -drive if=mtd, a raw image of exactly the flash size, which then holds
 * its contents and takes its erases and programs.
 */
/* [spec:nuos:req:emu.flash] */
/* [spec:nuos:req:emu.xip+1] */
static bool rp2350_soc_realize_xip(RP2350State *s, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(&s->xip);
    const struct {
        uint32_t size;
        const char *type;
    } devices[RP2350_QMI_CS] = {
        { s->flash_size, rp2350_flash_part(s->flash_size) },
        { s->psram_size, TYPE_APS6404L },
    };
    int cs;

    qdev_prop_set_uint32(DEVICE(sbd), "cs0-size", s->flash_size);
    qdev_prop_set_uint32(DEVICE(sbd), "cs1-size", s->psram_size);
    qdev_prop_set_uint32(DEVICE(sbd), "sysclk-hz", RP2350_SYSCLK_HZ);
    if (!sysbus_realize(sbd, errp)) {
        return false;
    }
    sysbus_mmio_map(sbd, RP2350_XIP_MMIO_CTRL, RP2350_XIP_CTRL_BASE);
    sysbus_mmio_map(sbd, RP2350_XIP_MMIO_QMI, RP2350_XIP_QMI_BASE);
    sysbus_mmio_map(sbd, RP2350_XIP_MMIO_AUX, RP2350_XIP_AUX_BASE);
    sysbus_mmio_map(sbd, RP2350_XIP_MMIO_SPACE, RP2350_XIP_BASE);

    for (cs = 0; cs < RP2350_QMI_CS; cs++) {
        DeviceState *dev;

        if (!devices[cs].size) {
            continue;
        }
        dev = qdev_new(devices[cs].type);
        qdev_prop_set_uint8(dev, "cs", cs);
        if (cs == 0) {
            DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);

            qdev_prop_set_bit(dev, "write-enable-autoclear", true);
            if (dinfo &&
                !qdev_prop_set_drive_err(dev, "drive",
                                         blk_by_legacy_dinfo(dinfo), errp)) {
                object_unref(OBJECT(dev));
                return false;
            }
        }
        object_property_set_link(OBJECT(dev), "memory",
                                 OBJECT(rp2350_xip_array(&s->xip, cs)),
                                 &error_abort);
        if (!ssi_realize_and_unref(dev, s->xip.qspi, errp)) {
            return false;
        }
        qdev_connect_gpio_out_named(DEVICE(sbd), "cs", cs,
                                    qdev_get_gpio_in_named(dev, SSI_GPIO_CS,
                                                           0));
    }
    return true;
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
    if (s->flash_size && (!is_power_of_2(s->flash_size) ||
                          s->flash_size < RP2350_FLASH_MIN_SIZE)) {
        error_setg(errp, "flash-size must be a power of two from 1 MiB");
        return;
    }
    if (s->psram_size && s->psram_size != APS6404L_SIZE) {
        error_setg(errp, "psram-size must be %u MiB, the APS6404L",
                   (unsigned)(APS6404L_SIZE / MiB));
        return;
    }

    if (!memory_region_init_rom(&s->rom, obj, "rp2350.rom", RP2350_ROM_SIZE,
                                errp)) {
        return;
    }
    memory_region_add_subregion(s->board_memory, RP2350_ROM_BASE, &s->rom);

    if (!rp2350_soc_realize_xip(s, errp)) {
        return;
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
    address_space_init(&s->sram_as, &s->sram, "rp2350-sram");
    for (i = 0; i < ARRAY_SIZE(s->sram_off); i++) {
        memory_region_init_io(&s->sram_off[i], obj, &rp2350_sram_off_ops, s,
                              i ? "rp2350.sram1-off" : "rp2350.sram0-off",
                              i ? RP2350_SRAM1_DOMAIN_SIZE
                                : RP2350_SRAM0_DOMAIN_SIZE);
        memory_region_set_enabled(&s->sram_off[i], false);
        memory_region_add_subregion_overlap(s->board_memory,
            RP2350_SRAM_BASE + (i ? RP2350_SRAM0_DOMAIN_SIZE : 0),
            &s->sram_off[i], 1);
    }

    /*
     * The USB controller's 4 KiB data DPRAM. It is ordinary memory to the
     * system bus, and software that does not use USB uses it as such: the
     * boot ROM keeps its flash boot workspace there. The controller's
     * registers are not modelled.
     */
    /* [spec:nuos:req:emu.machine+1] */
    if (!memory_region_init_ram(&s->usb_dpram, obj, "rp2350.usb-dpram",
                                RP2350_USB_DPRAM_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion(s->board_memory, RP2350_USB_DPRAM_BASE,
                                &s->usb_dpram);

    clock_set_hz(s->sysclk, RP2350_SYSCLK_HZ);
    clock_set_hz(s->refclk, RP2350_REFCLK_HZ);
    clock_set_hz(s->periclk, RP2350_CLK_PERI_HZ);
    clock_set_hz(s->adcclk, RP2350_CLK_ADC_HZ);

    /*
     * The bus filters stand between each core and board memory, so every
     * core reaches the board through its ACCESSCTRL view of the bus.
     */
    /* [spec:nuos:req:emu.accessctrl] */
    object_property_set_link(OBJECT(&s->accessctrl), "bus",
                             OBJECT(s->board_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->accessctrl), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->accessctrl), 0,
                    RP2350_ACCESSCTRL_BASE);

    for (i = 0; i < RP2350_NUM_CORES; i++) {
        DeviceState *armv7m = DEVICE(&s->armv7m[i]);
        int n;

        qdev_prop_set_uint32(armv7m, "num-irq", RP2350_NUM_IRQS);
        qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
        qdev_prop_set_string(armv7m, "cpu-type",
                             ARM_CPU_TYPE_NAME("cortex-m33"));
        /* [spec:nuos:req:emu.machine+1] */
        qdev_prop_set_uint64(armv7m, "midr", RP2350_M33_CPUID);
        qdev_prop_set_uint32(armv7m, "init-svtor", s->init_svtor);
        /* QEMU's Cortex-M33 defaults to 16 regions; the RP2350 has 8 each. */
        /* [spec:nuos:req:emu.mpu] */
        qdev_prop_set_uint32(armv7m, "mpu-s-regions", RP2350_MPU_REGIONS);
        qdev_prop_set_uint32(armv7m, "mpu-ns-regions", RP2350_MPU_REGIONS);
        /*
         * Both cores leave reset into the boot ROM, which holds core 1 in
         * its wait-for-launch code until core 0 launches it through the
         * SIO FIFO. With no ROM executing, core 1 stays powered off until
         * the machine's own launch handshake starts it.
         */
        /* [spec:nuos:req:emu.core1-launch] */
        qdev_prop_set_bit(armv7m, "start-powered-off",
                          i != 0 && s->core1_launch);
        qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
        qdev_connect_clock_in(armv7m, "refclk", s->refclk);
        object_property_set_link(OBJECT(armv7m), "memory",
                                 OBJECT(rp2350_accessctrl_view(&s->accessctrl,
                                                               i)),
                                 &error_abort);
        object_property_set_link(OBJECT(armv7m), "idau", OBJECT(s),
                                 &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(armv7m), errp)) {
            return;
        }
        /* A core that locks up stops; the other core carries on. */
        /* [spec:nuos:req:emu.lockup] */
        s->armv7m[i].cpu->m_lockup_halts = true;
        /* SYSRESETREQ resets only the core that asserts it. */
        /* [spec:nuos:req:emu.watchdog] */
        qdev_connect_gpio_out_named(armv7m, "SYSRESETREQ", 0,
                                    qdev_get_gpio_in_named(dev_soc,
                                                           "sysresetreq", i));

        /*
         * The extended PPB is core-local and sits inside the PPB range
         * that each armv7m container claims, so it is mapped per core
         * rather than in board memory.
         */
        /* [spec:nuos:req:emu.eppb] */
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->eppb[i]), errp)) {
            return;
        }
        memory_region_add_subregion_overlap(
            &s->armv7m[i].container, RP2350_EPPB_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->eppb[i]), 0), 0);
        for (n = 0; n < RP2350_NUM_IRQS; n++) {
            qdev_connect_gpio_out_named(DEVICE(&s->eppb[i]), "irq", n,
                                        qdev_get_gpio_in(armv7m, n));
        }
        qdev_connect_gpio_out_named(DEVICE(&s->eppb[i]), "nmi", 0,
                                    qdev_get_gpio_in_named(armv7m, "NMI", 0));

        /*
         * The processor's own debug components and PPB ROM table. The SCS
         * identification block lies over the NVIC's SCS and its NS alias,
         * so these take priority over the armv7m container's regions.
         */
        /* [spec:nuos:req:emu.coresight] */
        qdev_connect_clock_in(DEVICE(&s->m33_debug[i]), "cpuclk", s->sysclk);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->m33_debug[i]), errp)) {
            return;
        }
        for (n = 0; n < RP2350_M33_DEBUG_NUM_REGIONS; n++) {
            memory_region_add_subregion_overlap(
                &s->armv7m[i].container, rp2350_m33_debug_base[n],
                sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->m33_debug[i]), n),
                2);
        }
    }
    memory_region_init_alias(&s->eppb_sysmem, obj, "rp2350-eppb.sysmem",
                             sysbus_mmio_get_region(
                                 SYS_BUS_DEVICE(&s->eppb[0]), 0),
                             0, RP2350_ATOMIC_REGION_SIZE);
    memory_region_add_subregion(s->board_memory, RP2350_EPPB_BASE,
                                &s->eppb_sysmem);

    /* [spec:nuos:req:emu.rcp] */
    qdev_prop_set_bit(DEVICE(&s->rcp), "boot-rom-handoff", s->core1_launch);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcp), errp)) {
        return;
    }
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        rp2350_rcp_attach(&s->rcp, i, s->armv7m[i].cpu);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->rcp), i,
                           qdev_get_gpio_in_named(DEVICE(&s->eppb[i]),
                                                  "rcp-nmi", 0));
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
    object_property_set_link(OBJECT(&s->sio), "accessctrl",
                             OBJECT(&s->accessctrl), &error_abort);
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
                rp2350_soc_core_irq(s, i, RP2350_SIO_IRQ_FIFO + 2 * bank));
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->sio), irq + 1,
                rp2350_soc_core_irq(s, i, RP2350_SIO_IRQ_BELL + 2 * bank));
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

    /*
     * Counted core accesses go through the core's ACCESSCTRL view, so the
     * bus security filters refuse them as they refuse uncounted ones.
     */
    /* [spec:nuos:req:emu.busctrl] */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->busctrl), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->busctrl), 0, RP2350_BUSCTRL_BASE);
    for (i = 0; i < RP2350_NUM_CORES; i++) {
        rp2350_busctrl_attach_core(&s->busctrl, i, &s->armv7m[i].container,
                                   rp2350_accessctrl_view(&s->accessctrl, i),
                                   rp2350_sio_view(&s->sio, i, false),
                                   rp2350_sio_view(&s->sio, i, true));
    }

    /*
     * IO_BANK0, IO_QSPI, PADS_BANK0 and PADS_QSPI. The machine is an
     * RP2350A, which bonds out GPIOs 0-29. GPIO interrupts are core-local:
     * each core's outputs reach only that core's NVIC.
     */
    /* [spec:nuos:req:emu.gpio] */
    /* [spec:nuos:req:emu.irq-routing+1] */
    qdev_prop_set_uint32(DEVICE(&s->gpio), "bonded-gpios",
                         RP2350_GPIO_QFN60_PINS);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }
    s->accessctrl_notifier.notify = rp2350_soc_accessctrl_changed;
    rp2350_accessctrl_add_notifier(&s->accessctrl, &s->accessctrl_notifier);
    {
        static const hwaddr base[] = {
            RP2350_IO_BANK0_BASE, RP2350_IO_QSPI_BASE,
            RP2350_PADS_BANK0_BASE, RP2350_PADS_QSPI_BASE,
        };
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->gpio);
        int n;

        for (n = 0; n < ARRAY_SIZE(base); n++) {
            sysbus_mmio_map(sbd, n, base[n]);
        }
        for (i = 0; i < RP2350_NUM_CORES; i++) {
            for (n = 0; n < RP2350_GPIO_CORE_IRQS; n++) {
                sysbus_connect_irq(sbd, i * RP2350_GPIO_CORE_IRQS + n,
                    rp2350_soc_core_irq(s, i, RP2350_IO_IRQ_BANK0 + n));
            }
        }
        for (n = 0; n < RP2350_SIO_GPIO_BITS; n++) {
            qdev_connect_gpio_out_named(DEVICE(&s->sio), "gpio-out", n,
                rp2350_gpio_out_line(&s->gpio, RP2350_GPIO_PORT_SIO, n));
            qdev_connect_gpio_out_named(DEVICE(&s->sio), "gpio-oe", n,
                rp2350_gpio_oe_line(&s->gpio, RP2350_GPIO_PORT_SIO, n));
            rp2350_gpio_connect_in(&s->gpio, RP2350_GPIO_PORT_SIO, n,
                qdev_get_gpio_in_named(DEVICE(s), "sio-in", n));
        }
    }

    {
        const struct {
            SysBusDevice *dev;
            hwaddr base;
        } blocks[] = {
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
            { SYS_BUS_DEVICE(&s->rosc), RP2350_ROSC_BASE },
        };

        for (i = 0; i < ARRAY_SIZE(blocks); i++) {
            if (!sysbus_realize(blocks[i].dev, errp)) {
                return;
            }
            sysbus_mmio_map(blocks[i].dev, 0, blocks[i].base);
        }
    }

    /* [spec:nuos:req:emu.watchdog] */
    rp2350_psm_set_reset_fn(&s->psm, rp2350_soc_psm_reset, s);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->psm), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->psm), 0, RP2350_PSM_BASE);
    object_property_set_link(OBJECT(&s->watchdog), "ticks", OBJECT(&s->ticks),
                             &error_abort);
    qdev_prop_set_uint32(DEVICE(&s->watchdog), "ref-hz", RP2350_CLK_REF_HZ);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->watchdog), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->watchdog), 0, RP2350_WATCHDOG_BASE);

    /*
     * POWMAN. The watchdog's reset request passes through it on the way
     * to the PSM, so that its WDSEL can make it a chip-level reset, and a
     * glitch detector trigger is a chip-level reset it records.
     */
    /* [spec:nuos:req:emu.powman] */
    {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->powman);
        DeviceState *powman = DEVICE(&s->powman);

        object_property_set_link(OBJECT(sbd), "psm", OBJECT(&s->psm),
                                 &error_abort);
        qdev_prop_set_uint32(powman, "ref-hz", RP2350_CLK_REF_HZ);
        qdev_prop_set_uint32(powman, "bonded-gpios", RP2350_GPIO_QFN60_PINS);
        rp2350_powman_set_ops(&s->powman, &rp2350_soc_powman_ops, s);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_POWMAN_BASE);
        sysbus_connect_irq(sbd, RP2350_POWMAN_IRQ_POW,
                           qdev_get_gpio_in(dev_soc, RP2350_POWMAN_POW_IRQ));
        sysbus_connect_irq(sbd, RP2350_POWMAN_IRQ_TIMER,
                           qdev_get_gpio_in(dev_soc,
                                            RP2350_POWMAN_TIMER_IRQ));
        qdev_connect_gpio_out(DEVICE(&s->watchdog), 0,
            qdev_get_gpio_in_named(powman, RP2350_POWMAN_WATCHDOG, 0));
        qdev_connect_gpio_out_named(powman, RP2350_POWMAN_PSM_WATCHDOG, 0,
            qdev_get_gpio_in_named(DEVICE(&s->psm), "watchdog", 0));
        qdev_connect_gpio_out_named(powman, RP2350_POWMAN_PSM_RESET, 0,
            qdev_get_gpio_in_named(DEVICE(&s->psm), "powman-reset", 0));
        qdev_connect_gpio_out_named(DEVICE(&s->glitch_detector), "chip-reset",
            0, qdev_get_gpio_in_named(powman, RP2350_POWMAN_GLITCH_RESET, 0));
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

    /* [spec:nuos:req:emu.rosc-trng] */
    qdev_prop_set_uint32(DEVICE(&s->trng), "sysclk-hz", RP2350_SYSCLK_HZ);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->trng), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->trng), 0, RP2350_TRNG_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->trng), 0,
                       qdev_get_gpio_in(dev_soc, RP2350_TRNG_IRQ));

    /* The SHA-256 DREQ is connected with the DMA's other sources. */
    /* [spec:nuos:req:emu.sha256] */
    qdev_connect_clock_in(DEVICE(&s->sha256), "clk", s->sysclk);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sha256), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sha256), 0, RP2350_SHA256_BASE);

    /*
     * PWM. Each slice's A and B drive their GPIO function signals, and B
     * returns the OR of the pins selecting it. The DREQ_PWM_WRAP0-11
     * outputs ("dreq-wrap") are connected with the DMA's other sources.
     */
    /* [spec:nuos:req:emu.pwm] */
    {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->pwm);
        DeviceState *dev = DEVICE(&s->pwm);
        int n;

        qdev_connect_clock_in(dev, "clk", s->sysclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_PWM_BASE);
        sysbus_connect_irq(sbd, 0,
                           qdev_get_gpio_in(dev_soc, RP2350_PWM_IRQ_WRAP_0));
        sysbus_connect_irq(sbd, 1,
                           qdev_get_gpio_in(dev_soc, RP2350_PWM_IRQ_WRAP_1));
        for (n = 0; n < RP2350_GPIO_PWM_SIGNALS; n++) {
            qdev_connect_gpio_out_named(dev, RP2350_PWM_OUT, n,
                rp2350_gpio_out_line(&s->gpio, RP2350_GPIO_PORT_PWM, n));
            qdev_connect_gpio_out_named(dev, RP2350_PWM_OE, n,
                rp2350_gpio_oe_line(&s->gpio, RP2350_GPIO_PORT_PWM, n));
        }
        for (n = 0; n < RP2350_PWM_SLICES; n++) {
            rp2350_gpio_connect_in(&s->gpio, RP2350_GPIO_PORT_PWM,
                                   RP2350_GPIO_PWM(n, 1),
                                   qdev_get_gpio_in_named(dev, RP2350_PWM_B_IN,
                                                          n));
        }
    }

    /*
     * ADC. Its external inputs share GPIO 26-29's pads; the DREQ_ADC
     * output is connected with the DMA's other sources.
     */
    /* [spec:nuos:req:emu.adc] */
    object_property_set_link(OBJECT(&s->adc), "gpio", OBJECT(&s->gpio),
                             &error_abort);
    qdev_connect_clock_in(DEVICE(&s->adc), "clk", s->adcclk);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adc), 0, RP2350_ADC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc), 0,
                       qdev_get_gpio_in(dev_soc, RP2350_ADC_IRQ_FIFO));

    /*
     * HSTX, clocked by clk_hstx. Clock frequencies are not modelled: at
     * reset CLK_HSTX_CTRL selects clk_sys undivided, which pico-sdk keeps,
     * so clk_hstx is clk_sys. Its eight outputs drive the HSTX function
     * of GPIOs 12-19. DREQ_HSTX is connected with the DMA's other
     * sources.
     */
    /* [spec:nuos:req:emu.hstx] */
    {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->hstx);
        DeviceState *dev = DEVICE(&s->hstx);
        int n;

        qdev_connect_clock_in(dev, "clk", s->sysclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_HSTX_CTRL_BASE);
        sysbus_mmio_map(sbd, 1, RP2350_HSTX_FIFO_BASE);
        for (n = 0; n < RP2350_GPIO_HSTX_SIGNALS; n++) {
            qdev_connect_gpio_out_named(dev, RP2350_HSTX_OUT, n,
                rp2350_gpio_out_line(&s->gpio, RP2350_GPIO_PORT_HSTX, n));
            qdev_connect_gpio_out_named(dev, RP2350_HSTX_OE, n,
                rp2350_gpio_oe_line(&s->gpio, RP2350_GPIO_PORT_HSTX, n));
        }
    }

    /*
     * The UARTs are r1p5 PL011s with 32-entry FIFOs, clocked by clk_peri.
     * They have no character device: their TX, RX, CTS and RTS signals go
     * through the GPIO muxing like any other function's, and the board
     * attaches the console to pins. TX and RTS are always outputs.
     */
    /* [spec:nuos:req:emu.uart] */
    /* [spec:nuos:req:emu.gpio] */
    for (i = 0; i < RP2350_NUM_UARTS; i++) {
        static const hwaddr base[] = { RP2350_UART0_BASE, RP2350_UART1_BASE };
        static const int irq[] = { RP2350_UART0_IRQ, RP2350_UART1_IRQ };
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->uart[i]);
        DeviceState *uart = DEVICE(sbd);
        RP2350GPIOPort port = i ? RP2350_GPIO_PORT_UART1
                                : RP2350_GPIO_PORT_UART0;

        qdev_prop_set_bit(uart, "line-level", true);
        qdev_prop_set_uint32(uart, "fifo-depth", PL011_FIFO_MAX);
        qdev_connect_clock_in(uart, "clk", s->periclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        qdev_connect_gpio_out_named(uart, PL011_TXD, 0,
            rp2350_gpio_out_line(&s->gpio, port, RP2350_GPIO_UART_TX));
        qdev_connect_gpio_out_named(uart, PL011_NRTS, 0,
            rp2350_gpio_out_line(&s->gpio, port, RP2350_GPIO_UART_RTS));
        qemu_set_irq(rp2350_gpio_oe_line(&s->gpio, port, RP2350_GPIO_UART_TX),
                     1);
        qemu_set_irq(rp2350_gpio_oe_line(&s->gpio, port,
                                         RP2350_GPIO_UART_RTS), 1);
        rp2350_gpio_connect_in(&s->gpio, port, RP2350_GPIO_UART_RX,
                               qdev_get_gpio_in_named(uart, PL011_RXD, 0));
        rp2350_gpio_connect_in(&s->gpio, port, RP2350_GPIO_UART_CTS,
                               qdev_get_gpio_in_named(uart, PL011_NCTS, 0));
        memory_region_init_io(&s->uart_alias[i], obj, &rp2350_alias_ops,
                              sysbus_mmio_get_region(sbd, 0),
                              i ? "rp2350-uart1" : "rp2350-uart0",
                              RP2350_ATOMIC_REGION_SIZE);
        memory_region_add_subregion(s->board_memory, base[i],
                                    &s->uart_alias[i]);
        /* Output 0 is the PL011's combined UARTINTR. */
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(dev_soc, irq[i]));
    }

    /*
     * SPI0 and SPI1, PL022 r1p4 controllers timed by clk_peri, which is
     * modelled at clk_sys's rate (pico-sdk runs clk_peri from clk_sys).
     * Each has its SSI bus "spi0"/"spi1" for off-chip devices, and drives
     * its SCK, CSn and TX function signals; CSn's pin input is SSPFSSIN,
     * the select of the controller in slave mode. The DREQs are connected
     * with the DMA's other sources.
     */
    /* [spec:nuos:req:emu.spi] */
    qdev_init_gpio_in_named(dev_soc, rp2350_soc_spi_ctloe, "spi-nctloe",
                            RP2350_NUM_SPIS);
    for (i = 0; i < RP2350_NUM_SPIS; i++) {
        static const hwaddr base[] = { RP2350_SPI0_BASE, RP2350_SPI1_BASE };
        static const int irq[] = { RP2350_SPI0_IRQ, RP2350_SPI1_IRQ };
        RP2350GPIOPort port = i ? RP2350_GPIO_PORT_SPI1
                                : RP2350_GPIO_PORT_SPI0;
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->spi[i]);
        DeviceState *dev = DEVICE(sbd);

        qdev_prop_set_string(dev, "bus-name", i ? "spi1" : "spi0");
        qdev_prop_set_uint8(dev, "revision", 3);
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        memory_region_init_io(&s->spi_alias[i], obj, &rp2350_alias_ops,
                              sysbus_mmio_get_region(sbd, 0),
                              i ? "rp2350-spi1" : "rp2350-spi0",
                              RP2350_ATOMIC_REGION_SIZE);
        memory_region_add_subregion(s->board_memory, base[i],
                                    &s->spi_alias[i]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(dev_soc, irq[i]));

        qdev_connect_gpio_out_named(dev, PL022_SSP_OUT, PL022_SSPCLKOUT,
            rp2350_gpio_out_line(&s->gpio, port, RP2350_GPIO_SPI_SCK));
        qdev_connect_gpio_out_named(dev, PL022_SSP_OUT, PL022_SSPFSSOUT,
            rp2350_gpio_out_line(&s->gpio, port, RP2350_GPIO_SPI_CSN));
        qdev_connect_gpio_out_named(dev, PL022_SSP_OUT, PL022_SSPTXD,
            rp2350_gpio_out_line(&s->gpio, port, RP2350_GPIO_SPI_TX));
        qdev_connect_gpio_out_named(dev, PL022_SSP_OUT, PL022_NSSPCTLOE,
            qdev_get_gpio_in_named(dev_soc, "spi-nctloe", i));
        qdev_connect_gpio_out_named(dev, PL022_SSP_OUT, PL022_NSSPOE,
            qemu_irq_invert(rp2350_gpio_oe_line(&s->gpio, port,
                                                RP2350_GPIO_SPI_TX)));
        rp2350_gpio_connect_in(&s->gpio, port, RP2350_GPIO_SPI_CSN,
            qdev_get_gpio_in_named(dev, PL022_SSPFSSIN, 0));
    }
    s->spi_cs_notifier.notify = rp2350_soc_spi_wire_cs;
    qemu_add_machine_init_done_notifier(&s->spi_cs_notifier);

    /*
     * I2C0 and I2C1. A controller's QEMU I2C bus ("i2c0", "i2c1") stands
     * for the board's wires on the pins its SDA and SCL are routed to.
     * The controller drives the pins open-drain: output level 0, its
     * output enable pulling SCL low. It sees the pin levels, so a bus is
     * usable once the pins select the I2C function with their input
     * enabled and are pulled up. Otherwise SCL and SDA read low and
     * transfers wait for an idle bus, as on hardware.
     */
    /* [spec:nuos:req:emu.i2c] */
    for (i = 0; i < RP2350_NUM_I2C; i++) {
        static const hwaddr base[] = { RP2350_I2C0_BASE, RP2350_I2C1_BASE };
        static const int irq[] = { RP2350_I2C0_IRQ, RP2350_I2C1_IRQ };
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->i2c[i]);
        DeviceState *dev = DEVICE(sbd);
        RP2350GPIOPort port = i ? RP2350_GPIO_PORT_I2C1
                                : RP2350_GPIO_PORT_I2C0;
        int n;

        qdev_prop_set_string(dev, "bus-name", i ? "i2c1" : "i2c0");
        qdev_connect_clock_in(dev, "clk", s->sysclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        memory_region_init_io(&s->i2c_alias[i], obj, &rp2350_alias_ops,
                              sysbus_mmio_get_region(sbd, 0),
                              i ? "rp2350-i2c1" : "rp2350-i2c0",
                              RP2350_ATOMIC_REGION_SIZE);
        memory_region_add_subregion(s->board_memory, base[i],
                                    &s->i2c_alias[i]);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(dev_soc, irq[i]));
        for (n = 0; n < RP2350_GPIO_I2C_SIGNALS; n++) {
            qemu_set_irq(rp2350_gpio_out_line(&s->gpio, port, n), 0);
        }
        qdev_connect_gpio_out_named(dev, DESIGNWARE_I2C_SCL_OE, 0,
            rp2350_gpio_oe_line(&s->gpio, port, RP2350_GPIO_I2C_SCL));
        rp2350_gpio_connect_in(&s->gpio, port, RP2350_GPIO_I2C_SCL,
            qdev_get_gpio_in_named(dev, DESIGNWARE_I2C_SCL_IN, 0));
        rp2350_gpio_connect_in(&s->gpio, port, RP2350_GPIO_I2C_SDA,
            qdev_get_gpio_in_named(dev, DESIGNWARE_I2C_SDA_IN, 0));
    }

    /*
     * The self-hosted debug window. Each core sees it through its own view
     * so that it is refused its own AHB-AP; each AHB-AP masters its core's
     * bus.
     */
    /* [spec:nuos:req:emu.coresight] */
    {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->coresight);

        object_property_set_link(OBJECT(sbd), "core0-memory",
                                 OBJECT(&s->armv7m[0].container),
                                 &error_abort);
        object_property_set_link(OBJECT(sbd), "core1-memory",
                                 OBJECT(&s->armv7m[1].container),
                                 &error_abort);
        qdev_connect_clock_in(DEVICE(sbd), "clk", s->sysclk);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_CORESIGHT_PERIPH_BASE);
        for (i = 0; i < RP2350_NUM_CORES; i++) {
            memory_region_add_subregion_overlap(
                &s->armv7m[i].container, RP2350_CORESIGHT_PERIPH_BASE,
                rp2350_coresight_view(&s->coresight,
                                      RP2350_CORESIGHT_CORE0 + i), 0);
        }

        sbd = SYS_BUS_DEVICE(&s->coresight_trace);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_CORESIGHT_TRACE_BASE);
    }

    /*
     * OTP. Its power-up state machine drives the debug disables and the
     * glitch detectors' arming from the critical flags at every reset.
     */
    /* [spec:nuos:req:emu.otp] */
    {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->otp);
        DeviceState *otp = DEVICE(&s->otp);

        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_OTP_BASE);
        sysbus_mmio_map(sbd, 1, RP2350_OTP_DATA_BASE);
        sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(dev_soc, RP2350_OTP_IRQ));
        qdev_connect_gpio_out_named(otp, "debug-disable", 0,
            qdev_get_gpio_in_named(DEVICE(&s->coresight), "debug-disable", 0));
        qdev_connect_gpio_out_named(otp, "secure-debug-disable", 0,
            qdev_get_gpio_in_named(DEVICE(&s->coresight),
                                   "secure-debug-disable", 0));
        qdev_connect_gpio_out_named(otp, "glitch-detector-enable", 0,
            qdev_get_gpio_in_named(DEVICE(&s->glitch_detector),
                                   "otp-enable", 0));
    }

    /*
     * The DMA masters the bus through its own ACCESSCTRL view, and raises
     * DMA_IRQ_0-3 as system IRQs. Each DREQ source drives the DMA input of
     * its DREQ number.
     */
    /* [spec:nuos:req:emu.dma] */
    {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->dma);
        DeviceState *dma = DEVICE(&s->dma);

        object_property_set_link(OBJECT(sbd), "bus",
            OBJECT(rp2350_accessctrl_view(&s->accessctrl, RP2350_MASTER_DMA)),
            &error_abort);
        object_property_set_link(OBJECT(sbd), "accessctrl",
                                 OBJECT(&s->accessctrl), &error_abort);
        object_property_set_link(OBJECT(sbd), "busctrl", OBJECT(&s->busctrl),
                                 &error_abort);
        qdev_prop_set_uint32(dma, "sysclk-hz", RP2350_SYSCLK_HZ);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_DMA_BASE);
        for (i = 0; i < RP2350_DMA_IRQS; i++) {
            sysbus_connect_irq(sbd, i,
                               qdev_get_gpio_in(dev_soc, RP2350_DMA_IRQ_0 + i));
        }
        for (i = 0; i < RP2350_NUM_UARTS; i++) {
            DeviceState *uart = DEVICE(&s->uart[i]);
            int tx = i ? RP2350_DREQ_UART1_TX : RP2350_DREQ_UART0_TX;

            qdev_connect_gpio_out_named(uart, PL011_DMA_REQ, PL011_DMA_TX,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, tx));
            qdev_connect_gpio_out_named(uart, PL011_DMA_REQ, PL011_DMA_RX,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, tx + 1));
        }
        for (i = 0; i < RP2350_NUM_SPIS; i++) {
            DeviceState *spi = DEVICE(&s->spi[i]);
            int tx = i ? RP2350_DREQ_SPI1_TX : RP2350_DREQ_SPI0_TX;

            qdev_connect_gpio_out_named(spi, PL022_DMA_REQ, PL022_DMA_TX,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, tx));
            qdev_connect_gpio_out_named(spi, PL022_DMA_REQ, PL022_DMA_RX,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, tx + 1));
        }
        for (i = 0; i < RP2350_NUM_I2C; i++) {
            DeviceState *i2c = DEVICE(&s->i2c[i]);
            int tx = i ? RP2350_DREQ_I2C1_TX : RP2350_DREQ_I2C0_TX;

            qdev_connect_gpio_out_named(i2c, DESIGNWARE_I2C_DMA_TX_REQ, 0,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, tx));
            qdev_connect_gpio_out_named(i2c, DESIGNWARE_I2C_DMA_RX_REQ, 0,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, tx + 1));
        }
        for (i = 0; i < RP2350_PWM_SLICES; i++) {
            qdev_connect_gpio_out_named(DEVICE(&s->pwm), RP2350_PWM_DREQ, i,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ,
                                       RP2350_DREQ_PWM_WRAP0 + i));
        }
        for (i = 0; i < RP2350_XIP_NUM_DREQ; i++) {
            qdev_connect_gpio_out_named(DEVICE(&s->xip), "dreq", i,
                qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ,
                                       RP2350_DREQ_XIP_STREAM + i));
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->coresight_trace), 0,
            qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ,
                                   RP2350_DREQ_CORESIGHT));
        qdev_connect_gpio_out_named(DEVICE(&s->adc), RP2350_ADC_DREQ, 0,
            qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, RP2350_DREQ_ADC));
        qdev_connect_gpio_out_named(DEVICE(&s->sha256), RP2350_SHA256_DREQ, 0,
            qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, RP2350_DREQ_SHA256));
        qdev_connect_gpio_out_named(DEVICE(&s->hstx), RP2350_HSTX_DREQ, 0,
            qdev_get_gpio_in_named(dma, RP2350_DMA_DREQ, RP2350_DREQ_HSTX));
    }

    /*
     * RESETS last, once every subsystem's device model is attached. With
     * no ROM executing, the subsystems the ROM's flash boot path takes out
     * of reset (the QSPI IO and pads, connecting the flash) start out of
     * reset.
     */
    /* [spec:nuos:req:emu.resets] */
    {
        const struct {
            int reset;
            void *dev;
        } devices[] = {
            { RP2350_RESET_ADC, &s->adc },
            { RP2350_RESET_BUSCTRL, &s->busctrl },
            { RP2350_RESET_DMA, &s->dma },
            { RP2350_RESET_PLL_SYS, &s->pll_sys },
            { RP2350_RESET_PLL_USB, &s->pll_usb },
            { RP2350_RESET_PWM, &s->pwm },
            { RP2350_RESET_SHA256, &s->sha256 },
            { RP2350_RESET_SYSCFG, &s->syscfg },
            { RP2350_RESET_SYSINFO, &s->sysinfo },
            { RP2350_RESET_TBMAN, &s->tbman },
            { RP2350_RESET_TIMER0, &s->timer[0] },
            { RP2350_RESET_TIMER1, &s->timer[1] },
            { RP2350_RESET_TRNG, &s->trng },
            { RP2350_RESET_UART0, &s->uart[0] },
            { RP2350_RESET_UART1, &s->uart[1] },
            { RP2350_RESET_SPI0, &s->spi[0] },
            { RP2350_RESET_SPI1, &s->spi[1] },
            { RP2350_RESET_HSTX, &s->hstx },
            { RP2350_RESET_I2C0, &s->i2c[0] },
            { RP2350_RESET_I2C1, &s->i2c[1] },
        };
        SysBusDevice *sbd = SYS_BUS_DEVICE(&s->resets);

        for (i = 0; i < ARRAY_SIZE(devices); i++) {
            rp2350_soc_attach_reset(s, devices[i].reset,
                                    DEVICE(devices[i].dev));
        }
        rp2350_soc_init_reset_gates(s);
        rp2350_resets_set_hold_fn(&s->resets, rp2350_soc_reset_hold, s);
        qdev_prop_set_uint32(DEVICE(sbd), "rom-unreset",
                             s->core1_launch ?
                             BIT(RP2350_RESET_IO_QSPI) |
                             BIT(RP2350_RESET_PADS_QSPI) : 0);
        if (!sysbus_realize(sbd, errp)) {
            return;
        }
        sysbus_mmio_map(sbd, 0, RP2350_RESETS_BASE);
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
                                TYPE_RP2350_EPPB);
        object_initialize_child(obj, "m33-debug[*]", &s->m33_debug[i],
                                TYPE_RP2350_M33_DEBUG);
    }

    object_initialize_child(obj, "accessctrl", &s->accessctrl,
                            TYPE_RP2350_ACCESSCTRL);
    object_initialize_child(obj, "resets", &s->resets, TYPE_RP2350_RESETS);
    object_initialize_child(obj, "psm", &s->psm, TYPE_RP2350_PSM);
    object_initialize_child(obj, "watchdog", &s->watchdog,
                            TYPE_RP2350_WATCHDOG);
    object_initialize_child(obj, "powman", &s->powman, TYPE_RP2350_POWMAN);
    object_initialize_child(obj, "sio", &s->sio, TYPE_RP2350_SIO);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_RP2350_GPIO);
    object_initialize_child(obj, "rcp", &s->rcp, TYPE_RP2350_RCP);
    object_initialize_child(obj, "bootram", &s->bootram, TYPE_RP2350_BOOTRAM);
    object_initialize_child(obj, "busctrl", &s->busctrl, TYPE_RP2350_BUSCTRL);
    object_initialize_child(obj, "dcp", &s->dcp, TYPE_RP2350_DCP);
    object_initialize_child(obj, "xip", &s->xip, TYPE_RP2350_XIP);
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
    object_initialize_child(obj, "rosc", &s->rosc, TYPE_RP2350_ROSC);
    object_initialize_child(obj, "trng", &s->trng, TYPE_RP2350_TRNG);
    object_initialize_child(obj, "sha256", &s->sha256, TYPE_RP2350_SHA256);
    object_initialize_child(obj, "pwm", &s->pwm, TYPE_RP2350_PWM);
    object_initialize_child(obj, "adc", &s->adc, TYPE_RP2350_ADC);
    object_initialize_child(obj, "hstx", &s->hstx, TYPE_RP2350_HSTX);
    for (i = 0; i < RP2350_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_RP2350_TIMER);
    }
    for (i = 0; i < RP2350_NUM_UARTS; i++) {
        object_initialize_child(obj, "uart[*]", &s->uart[i], TYPE_PL011);
    }
    for (i = 0; i < RP2350_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i], TYPE_PL022);
    }
    for (i = 0; i < RP2350_NUM_I2C; i++) {
        object_initialize_child(obj, "i2c[*]", &s->i2c[i], TYPE_RP2350_I2C);
    }
    object_initialize_child(obj, "coresight", &s->coresight,
                            TYPE_RP2350_CORESIGHT);
    object_initialize_child(obj, "coresight-trace", &s->coresight_trace,
                            TYPE_RP2350_CORESIGHT_TRACE);
    object_initialize_child(obj, "otp", &s->otp, TYPE_RP2350_OTP);
    object_initialize_child(obj, "dma", &s->dma, TYPE_RP2350_DMA);

    qdev_init_gpio_in(DEVICE(s), rp2350_soc_set_irq, RP2350_NUM_IRQS);
    qdev_init_gpio_in_named(DEVICE(s), rp2350_soc_sio_in, "sio-in",
                            RP2350_GPIO_SIO_BITS);
    qdev_init_gpio_in_named(DEVICE(s), rp2350_soc_sysresetreq, "sysresetreq",
                            RP2350_NUM_CORES);
    s->sysresetreq_bh = qemu_bh_new(rp2350_soc_sysresetreq_run, s);

    s->sysclk = qdev_init_clock_out(DEVICE(s), "sysclk");
    s->refclk = qdev_init_clock_out(DEVICE(s), "refclk");
    s->periclk = qdev_init_clock_out(DEVICE(s), "periclk");
    s->adcclk = clock_new(obj, "adcclk");
}

static const Property rp2350_soc_properties[] = {
    DEFINE_PROP_LINK("memory", RP2350State, board_memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("flash-size", RP2350State, flash_size, 0),
    DEFINE_PROP_UINT32("psram-size", RP2350State, psram_size, 0),
    DEFINE_PROP_UINT32("init-svtor", RP2350State, init_svtor, RP2350_ROM_BASE),
    /* Emulate the boot ROM's core 1 launch handshake (no ROM executing). */
    DEFINE_PROP_BOOL("core1-launch", RP2350State, core1_launch, false),
};

/* A chip-level reset supersedes a core reset still queued. */
static void rp2350_soc_hold_reset(Object *obj, ResetType type)
{
    RP2350State *s = RP2350_SOC(obj);

    s->sysresetreq_pending = 0;
    qemu_bh_cancel(s->sysresetreq_bh);
}

static int rp2350_soc_post_load(void *opaque, int version_id)
{
    RP2350State *s = opaque;

    if (s->sysresetreq_pending) {
        qemu_bh_schedule(s->sysresetreq_bh);
    }
    return 0;
}

static const VMStateDescription vmstate_rp2350_soc = {
    .name = TYPE_RP2350_SOC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rp2350_soc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sysresetreq_pending, RP2350State),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    IDAUInterfaceClass *iic = IDAU_INTERFACE_CLASS(klass);

    dc->realize = rp2350_soc_realize;
    dc->vmsd = &vmstate_rp2350_soc;
    rc->phases.hold = rp2350_soc_hold_reset;
    device_class_set_props(dc, rp2350_soc_properties);
    iic->check = rp2350_idau_check;
}

static const TypeInfo rp2350_soc_info = {
    .name          = TYPE_RP2350_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350State),
    .instance_init = rp2350_soc_init,
    .class_init    = rp2350_soc_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_IDAU_INTERFACE },
        { }
    },
};

static void rp2350_soc_types(void)
{
    type_register_static(&rp2350_soc_info);
}
type_init(rp2350_soc_types)
