/*
 * ESP32 SoC and machine
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "hw/core/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "qemu/bswap.h"
#include "hw/xtensa/esp32.h"
#include "exec/watchpoint.h"
#include "system/address-spaces.h"
#include "hw/misc/ssi_psram.h"
#include "hw/sd/dwc_sdmmc.h"
#include "core-esp32/core-isa.h"
#include "qemu/datadir.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/cpus.h"
#include "system/runstate.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "net/net.h"
#include "elf.h"

#define TYPE_ESP32_SOC "xtensa.esp32"
#define ESP32_SOC(obj) OBJECT_CHECK(Esp32SocState, (obj), TYPE_ESP32_SOC)

#define TYPE_ESP32_CPU XTENSA_CPU_TYPE_NAME("esp32")



enum {
    ESP32_MEMREGION_IROM,
    ESP32_MEMREGION_DROM,
    ESP32_MEMREGION_DRAM,
    ESP32_MEMREGION_IRAM,
    ESP32_MEMREGION_ICACHE0,
    ESP32_MEMREGION_ICACHE1,
    ESP32_MEMREGION_RTCSLOW,
    ESP32_MEMREGION_RTCFAST_D,
    ESP32_MEMREGION_RTCFAST_I,
    ESP32_MEMREGION_FRAMEBUF,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32_memmap[] = {
    [ESP32_MEMREGION_DROM] = { 0x3ff90000, 0x10000 },
    [ESP32_MEMREGION_IROM] = { 0x40000000, 0x70000 },
    [ESP32_MEMREGION_DRAM] = { 0x3ffae000, 0x52000 },
    [ESP32_MEMREGION_IRAM] = { 0x40080000, 0x40000 },
    [ESP32_MEMREGION_ICACHE0] = { 0x40070000, 0x8000 },
    [ESP32_MEMREGION_ICACHE1] = { 0x40078000, 0x8000 },
    [ESP32_MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_I] = { 0x400C0000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_D] = { 0x3ff80000, 0x2000 },
    /* Virtual Framebuffer, used for the graphical interface */
    [ESP32_MEMREGION_FRAMEBUF] = { 0x20000000, ESP_RGB_MAX_VRAM_SIZE }
};


#define ESP32_SOC_RESET_PROCPU    0x1
#define ESP32_SOC_RESET_APPCPU    0x2
#define ESP32_SOC_RESET_PERIPH    0x4
#define ESP32_SOC_RESET_DIG       (ESP32_SOC_RESET_PROCPU | ESP32_SOC_RESET_APPCPU | ESP32_SOC_RESET_PERIPH)
#define ESP32_SOC_RESET_RTC       0x8
#define ESP32_SOC_RESET_ALL       (ESP32_SOC_RESET_RTC | ESP32_SOC_RESET_DIG)




static void esp32_soc_update_gates(Esp32SocState *s, bool from_reset);
static void esp32_soc_update_clocks(Esp32SocState *s);

static void remove_cpu_watchpoints(XtensaCPU* xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void esp32_dig_reset(void *opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        esp32_dport_clear_ill_trap_state(&s->dport);
        s->requested_reset = ESP32_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->rtc_cntl.reset_cause[n] = ESP32_SW_CPU_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32_timg_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->rtc_cntl.reset_cause[n] = ESP32_TGWDT_CPU_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32_timg_sys_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        esp32_dport_clear_ill_trap_state(&s->dport);
        s->requested_reset = ESP32_SOC_RESET_DIG;
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            s->rtc_cntl.reset_cause[i] = ESP32_TG0WDT_SYS_RESET + n;
        }
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_soc_reset(DeviceState *dev)
{
    Esp32SocState *s = ESP32_SOC(dev);

    uint32_t strap_mode = s->gpio.strap_mode;

    bool flash_boot_mode = ((strap_mode & 0x10) || (strap_mode & 0x1f) == 0x0c);
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_DL_MODE_GPIO, 0), !flash_boot_mode);

    if (s->requested_reset == 0) {
        s->requested_reset = ESP32_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32_SOC_RESET_RTC) {
        device_cold_reset(DEVICE(&s->rtc_cntl));
    }
    if (s->requested_reset & ESP32_SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->dport));
        device_cold_reset(DEVICE(&s->apb_ctrl));
        device_cold_reset(DEVICE(&s->intmatrix));
        device_cold_reset(DEVICE(&s->aes));
        device_cold_reset(DEVICE(&s->rsa));
        device_cold_reset(DEVICE(&s->gpio));
        for (int i = 0; i < ESP32_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart[i]));
        }
        for (int i = 0; i < ESP32_UHCI_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uhci[i]));
        }
        for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->frc_timer[i]));
        }
        for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->timg[i]));
        }
        s->timg[0].flash_boot_mode = flash_boot_mode;
        for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->spi[i]));
        }
        for (int i = 0; i < ESP32_I2C_COUNT; i++) {
            device_cold_reset(DEVICE(&s->i2c[i]));
        }
        device_cold_reset(DEVICE(&s->twai));
        device_cold_reset(DEVICE(&s->efuse));
        device_cold_reset(DEVICE(&s->ledc));
        device_cold_reset(DEVICE(&s->sha));
        device_cold_reset(DEVICE(&s->rng));
        device_cold_reset(DEVICE(&s->sdmmc));
        if (s->eth) {
            device_cold_reset(s->eth);
        }

        device_cold_reset(DEVICE(&s->rgb));
    }
    if (s->requested_reset & ESP32_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    if (s->requested_reset & ESP32_SOC_RESET_APPCPU) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[1]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    s->requested_reset = 0;
    esp32_soc_update_gates(s, true);
    esp32_soc_update_clocks(s);
}

static void esp32_cpu_stall(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);

    bool stall;
    if (n == 0) {
        stall = s->rtc_cntl.cpu_stall_state[0];
    } else {
        stall = s->rtc_cntl.cpu_stall_state[1] || s->dport.appcpu_stall_state || (!s->dport.appcpu_clkgate_state);
    }

    if (stall != s->cpu[n].env.runstall) {
        xtensa_runstall(&s->cpu[n].env, stall);
    }
}

/* TWAI's interrupt register, the SJA1000's IR (register 3): read clears it */
#define ESP32_TWAI_INT_RAW_OFFSET 0x0c

/* RC_FAST_CLK, the internal 8 MHz oscillator, at its nominal frequency */
#define ESP32_RC_FAST_HZ 8000000
/* PLL_CLK-derived CPU_CLK frequencies for CPUPERIOD_SEL 0, 1 and 2 */
static const uint32_t esp32_pll_cpu_hz[] = { 80000000, 160000000, 240000000 };
#define ESP32_PLL_APB_HZ 80000000
/*
 * APLL_CLK's coefficients are programmed through the analog I2C bus, which
 * is not modelled. Until it is, the APLL is taken to run at the output of
 * its formula with every coefficient 0: 40 MHz * 4 / (2 * 2) = 40 MHz.
 */
#define ESP32_APLL_UNMODELLED_HZ 40000000

static bool esp32_gate_running(Esp32PeriphGate *g)
{
    return g->clk_on && !g->held;
}

static void esp32_gate_update_clocks(Esp32SocState *s, Esp32PeriphGate *g)
{
    bool running = esp32_gate_running(g);

    if (g->apb_clk) {
        clock_update_hz(g->apb_clk, running ? s->apb_hz : 0);
    }
    if (g->ref_tick_clk) {
        clock_update_hz(g->ref_tick_clk, running ? s->ref_tick_hz : 0);
    }
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * Derive CPU_CLK, APB_CLK and REF_TICK from RTC_CNTL_SOC_CLK_SEL,
 * DPORT_CPUPERIOD_SEL and the SYSCON dividers (TRM tables 7.2-2, 7.2-4 and
 * 7.2-5), and pass them to the CPUs and the APB peripherals.
 */
static void esp32_soc_update_clocks(Esp32SocState *s)
{
    uint32_t sel = s->dport.cpuperiod_sel;
    uint32_t xtal_hz = s->rtc_cntl.xtal_apb_freq;
    uint32_t cpu_hz, apb_hz, apll_div;

    switch (s->rtc_cntl.soc_clk) {
    case ESP32_SOC_CLK_XTAL:
        cpu_hz = xtal_hz / esp32_apb_ctrl_pre_div(&s->apb_ctrl);
        apb_hz = cpu_hz;
        break;
    case ESP32_SOC_CLK_PLL:
        if (sel >= ARRAY_SIZE(esp32_pll_cpu_hz)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32: reserved CPUPERIOD_SEL %u with PLL_CLK\n",
                          sel);
            sel = 0;
        }
        cpu_hz = esp32_pll_cpu_hz[sel];
        apb_hz = ESP32_PLL_APB_HZ;
        break;
    case ESP32_SOC_CLK_8M:
        cpu_hz = ESP32_RC_FAST_HZ / esp32_apb_ctrl_pre_div(&s->apb_ctrl);
        apb_hz = cpu_hz;
        break;
    case ESP32_SOC_CLK_APLL:
    default:
        qemu_log_mask(LOG_UNIMP,
                      "esp32: APLL coefficients not modelled, APLL_CLK "
                      "taken as %u Hz\n", ESP32_APLL_UNMODELLED_HZ);
        if (sel > 1) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32: reserved CPUPERIOD_SEL %u with APLL_CLK\n",
                          sel);
        }
        apll_div = sel == 0 ? 4 : 2;
        cpu_hz = ESP32_APLL_UNMODELLED_HZ / apll_div;
        apb_hz = cpu_hz / 2;
        break;
    }

    s->apb_hz = apb_hz;
    s->ref_tick_hz = apb_hz / esp32_apb_ctrl_tick_div(&s->apb_ctrl,
                                                      s->rtc_cntl.soc_clk);
    clock_update_hz(s->cpu_clk, cpu_hz);
    for (unsigned i = 0; i < s->n_gates; i++) {
        esp32_gate_update_clocks(s, &s->gate[i]);
    }
}

static void esp32_clk_update(void *opaque, int n, int level)
{
    if (level) {
        esp32_soc_update_clocks(ESP32_SOC(opaque));
    }
}

static uint32_t esp32_gate_clk_reg(Esp32SocState *s, Esp32GateRegs regs)
{
    switch (regs) {
    case ESP32_GATE_PERI:
        return s->dport.peri_clk_en;
    case ESP32_GATE_PERIP:
        return s->dport.perip_clk_en;
    case ESP32_GATE_WIFI:
    default:
        return s->dport.wifi_clk_en;
    }
}

static uint32_t esp32_gate_rst_reg(Esp32SocState *s, Esp32GateRegs regs)
{
    switch (regs) {
    case ESP32_GATE_PERI:
        return s->dport.peri_rst_en;
    case ESP32_GATE_PERIP:
        return s->dport.perip_rst_en;
    case ESP32_GATE_WIFI:
    default:
        return s->dport.core_rst_en;
    }
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * Apply DPORT's clock-enable and reset bits. Asserting a peripheral's
 * reset resets it and holds it there; releasing the reset resets it again,
 * with its clock running, so that it starts from its reset state. After a
 * reset of the whole SoC the peripherals have just been reset, so
 * from_reset only adopts the new state.
 */
static void esp32_soc_update_gates(Esp32SocState *s, bool from_reset)
{
    for (unsigned i = 0; i < s->n_gates; i++) {
        Esp32PeriphGate *g = &s->gate[i];
        uint32_t clk = esp32_gate_clk_reg(s, g->regs);
        bool held = (esp32_gate_rst_reg(s, g->regs) & g->rst_mask) != 0;
        bool assert_rst = !from_reset && held && !g->held;
        bool release_rst = !from_reset && !held && g->held;

        if (assert_rst) {
            device_cold_reset(g->dev);
        }
        g->clk_on = (clk & g->clk_mask) == g->clk_mask;
        g->held = held;
        esp32_gate_update_clocks(s, g);
        if (release_rst) {
            device_cold_reset(g->dev);
        }
    }
}

static void esp32_periph_clk_update(void *opaque, int n, int level)
{
    if (level) {
        esp32_soc_update_gates(ESP32_SOC(opaque), false);
    }
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * Register access through the gate. The TRM does not say how a gated or
 * reset-held peripheral's registers read; the model takes the hardware's
 * register flops to keep their values with the clock stopped. Writes are
 * dropped, as the flops cannot latch them, and reads return the values the
 * registers hold: their last values, or their reset values while the
 * peripheral is held in reset. A read that would have a side effect, such
 * as popping a FIFO, returns 0 and has none.
 */
static MemTxResult esp32_gate_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    Esp32GateWindow *w = opaque;
    Esp32PeriphGate *g = w->gate;
    uint8_t buf[8];
    MemTxResult r;

    if (!esp32_gate_running(g) && g->has_volatile_reg && w == &g->window[0] &&
        addr < g->volatile_reg + 4 && addr + size > g->volatile_reg) {
        *data = 0;
        return MEMTX_OK;
    }
    r = address_space_read(&w->dev_as, addr, attrs, buf, size);
    *data = ldn_le_p(buf, size);
    return r;
}

static MemTxResult esp32_gate_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size, MemTxAttrs attrs)
{
    Esp32GateWindow *w = opaque;
    Esp32PeriphGate *g = w->gate;
    uint8_t buf[8];

    if (!esp32_gate_running(g)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32: write to %s at 0x%" HWADDR_PRIx " while its "
                      "clock is gated or it is held in reset\n",
                      object_get_canonical_path_component(OBJECT(g->dev)),
                      addr);
        return MEMTX_OK;
    }
    stn_le_p(buf, size, data);
    return address_space_write(&w->dev_as, addr, attrs, buf, size);
}

static const MemoryRegionOps esp32_gate_ops = {
    .read_with_attrs = esp32_gate_read,
    .write_with_attrs = esp32_gate_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * Map a register block of a gated peripheral at addr, and, like the other
 * APB peripherals, at its alias in the 0x60000000 window if apb_alias.
 */
static void esp32_gate_map(Esp32PeriphGate *g, MemoryRegion *dev_mr,
                           hwaddr addr, bool apb_alias)
{
    MemoryRegion *sys_mem = get_system_memory();
    Esp32GateWindow *w;
    char *name;

    assert(g->n_windows < ESP32_GATE_MAX_MR);
    w = &g->window[g->n_windows++];
    w->gate = g;
    name = g_strdup_printf("%s-gate",
                           object_get_canonical_path_component(OBJECT(g->dev)));
    address_space_init(&w->dev_as, dev_mr, name);
    /*
     * No owner: an owner's re-entrancy guard would block the forwarded
     * access to the peripheral's own region, and one shared by all gates
     * would block a peripheral reaching another through its gate.
     */
    memory_region_init_io(&w->iomem, NULL, &esp32_gate_ops, w,
                          name, memory_region_size(dev_mr));
    memory_region_add_subregion_overlap(sys_mem, addr, &w->iomem, 0);
    if (apb_alias) {
        MemoryRegion *alias = g_new(MemoryRegion, 1);
        char *alias_name = g_strdup_printf("mr-apb-0x%08x", (uint32_t)addr);

        memory_region_init_alias(alias, NULL, alias_name,
                                 &w->iomem, 0, memory_region_size(dev_mr));
        memory_region_add_subregion_overlap(sys_mem,
                                            addr - DR_REG_DPORT_APB_BASE +
                                            APB_REG_BASE, alias, 0);
        g_free(alias_name);
    }
    g_free(name);
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * Put a realized peripheral's first register block behind DPORT's clock
 * gate and reset bits.
 */
static Esp32PeriphGate *esp32_soc_add_gated_device(Esp32SocState *s,
                                                   void *dev, hwaddr addr,
                                                   Esp32GateRegs regs,
                                                   uint32_t clk_mask,
                                                   uint32_t rst_mask)
{
    Esp32PeriphGate *g;

    assert(s->n_gates < ESP32_GATE_MAX);
    g = &s->gate[s->n_gates++];
    g->dev = DEVICE(dev);
    g->regs = regs;
    g->clk_mask = clk_mask;
    g->rst_mask = rst_mask;
    esp32_gate_map(g, sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0), addr,
                   true);
    return g;
}

static void esp32_soc_add_periph_device(MemoryRegion *dest, void* dev, hwaddr dport_base_addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    memory_region_add_subregion_overlap(dest, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, mr_apb, 0);
    g_free(name);
}

static void esp32_soc_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size)
{
    create_unimplemented_device(name, dport_base_addr, size);
    char * name_apb = g_strdup_printf("%s-apb", name);
    create_unimplemented_device(name_apb, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, size);
    g_free(name_apb);
}

static void esp32_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32SocState *s = ESP32_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    const struct MemmapEntry *memmap = esp32_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *icache0 = g_new(MemoryRegion, 1);
    MemoryRegion *icache1 = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_i = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_d = g_new(MemoryRegion, 1);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        assert(i >= 0 && i <= 9);
        MemoryRegion *drom = g_new(MemoryRegion, 1);
        MemoryRegion *irom = g_new(MemoryRegion, 1);

        char name[18];
        snprintf(name, sizeof(name), "esp32.irom.cpu%d", i);
        memory_region_init_rom(irom, NULL, name,
                            memmap[ESP32_MEMREGION_IROM].size, &error_fatal);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[ESP32_MEMREGION_IROM].base, irom);


        snprintf(name, sizeof(name), "esp32.drom.cpu%d", i);
        memory_region_init_alias(drom, NULL, name, irom, 0x60000, memmap[ESP32_MEMREGION_DROM].size);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[ESP32_MEMREGION_DROM].base, drom);
    }

    memory_region_init_ram(dram, NULL, "esp32.dram",
                           memmap[ESP32_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_DRAM].base, dram);

    memory_region_init_ram(iram, NULL, "esp32.iram",
                           memmap[ESP32_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(icache0, NULL, "esp32.icache0",
                           memmap[ESP32_MEMREGION_ICACHE0].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE0].base, icache0);

    memory_region_init_ram(icache1, NULL, "esp32.icache1",
                           memmap[ESP32_MEMREGION_ICACHE1].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE1].base, icache1);

    memory_region_init_ram(rtcslow, NULL, "esp32.rtcslow",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_RTCSLOW].base, rtcslow);

    /* RTC Fast memory is only accessible by the PRO CPU */

    memory_region_init_ram(rtcfast_i, NULL, "esp32.rtcfast_i",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32_MEMREGION_RTCFAST_I].base, rtcfast_i);

    memory_region_init_alias(rtcfast_d, NULL, "esp32.rtcfast_d", rtcfast_i, 0, memmap[ESP32_MEMREGION_RTCFAST_D].size);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32_MEMREGION_RTCFAST_D].base, rtcfast_d);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }

    qdev_realize(DEVICE(&s->dport), &s->periph_bus, &error_fatal);
    MemoryRegion* dport_mem = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dport), 0);

    memory_region_add_subregion(sys_mem, DR_REG_DPORT_BASE, dport_mem);
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_APPCPU_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, 1));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_APPCPU_STALL_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, 1));
    qdev_connect_gpio_out_named(DEVICE(&s->dport),
                                ESP32_DPORT_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev,
                                    ESP32_RTC_CLK_UPDATE_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport),
                                ESP32_DPORT_PERIPH_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev,
                                    ESP32_DPORT_PERIPH_CLK_UPDATE_GPIO, 0));

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), name, OBJECT(qemu_get_cpu(i)), &error_abort);
    }
    qdev_realize(DEVICE(&s->intmatrix), &s->periph_bus, &error_fatal);
    DeviceState* intmatrix_dev = DEVICE(&s->intmatrix);
    memory_region_add_subregion_overlap(dport_mem, ESP32_DPORT_PRO_INTMATRIX_BASE, sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->intmatrix), 0), -1);

    bool init_cache_err = false;
    if (s->dport.flash_blk) {
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            Esp32CacheRegionState *drom0 = &s->dport.cache_state[i].drom0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->mem, -1);
            Esp32CacheRegionState *iram0 = &s->dport.cache_state[i].iram0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->mem, -1);
        }
        init_cache_err = true;
    }
    if (s->dport.has_psram) {
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            Esp32CacheRegionState *dram1 = &s->dport.cache_state[i].dram1;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], dram1->base, &dram1->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], dram1->base, &dram1->mem, -1);
        }
        init_cache_err = true;
    }
    if (init_cache_err) {
        qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_CACHE_ILL_IRQ_GPIO, 0,
                                    qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_CACHE_IA_INTR_SOURCE));
    }

    int n_crosscore_irqs = ESP32_DPORT_CROSSCORE_INT_COUNT;
    object_property_set_int(OBJECT(&s->crosscore_int), "n_irqs", n_crosscore_irqs, &error_abort);
    qdev_realize(DEVICE(&s->crosscore_int), &s->periph_bus, &error_fatal);
    memory_region_add_subregion_overlap(dport_mem, ESP32_DPORT_CROSSCORE_INT_BASE, &s->crosscore_int.iomem, -1);

    for (int index = 0; index < ESP32_DPORT_CROSSCORE_INT_COUNT; ++index) {
        qemu_irq target = qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_FROM_CPU_INTR0_SOURCE + index);
        assert(target);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->crosscore_int), index, target);
    }

    qdev_realize(DEVICE(&s->rsa), &s->periph_bus, &error_fatal);
    esp32_soc_add_gated_device(s, &s->rsa, DR_REG_RSA_BASE, ESP32_GATE_PERI,
                               R_DPORT_PERI_RSA_MASK, R_DPORT_PERI_RSA_MASK);

    qdev_realize(DEVICE(&s->sha), &s->periph_bus, &error_fatal);
    esp32_soc_add_gated_device(s, &s->sha, DR_REG_SHA_BASE, ESP32_GATE_PERI,
                               R_DPORT_PERI_SHA_MASK, R_DPORT_PERI_SHA_MASK);

    qdev_realize(DEVICE(&s->aes), &s->periph_bus, &error_fatal);
    esp32_soc_add_gated_device(s, &s->aes, DR_REG_AES_BASE, ESP32_GATE_PERI,
                               R_DPORT_PERI_AES_MASK, R_DPORT_PERI_AES_MASK);

    qdev_realize(DEVICE(&s->ledc), &s->periph_bus, &error_fatal);
    esp32_soc_add_gated_device(s, &s->ledc, DR_REG_LEDC_BASE,
                               ESP32_GATE_PERIP, R_DPORT_PERIP_LEDC_MASK,
                               R_DPORT_PERIP_LEDC_MASK);

    qdev_realize(DEVICE(&s->apb_ctrl), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->apb_ctrl, DR_REG_APB_CTRL_BASE);
    qdev_connect_gpio_out_named(DEVICE(&s->apb_ctrl),
                                ESP32_APB_CTRL_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev,
                                    ESP32_RTC_CLK_UPDATE_GPIO, 0));

    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTCCNTL_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, i));
    }

    qdev_realize(DEVICE(&s->gpio), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->gpio, DR_REG_GPIO_BASE);

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE};
        const uint32_t uart_bit[] = {
            R_DPORT_PERIP_UART_MASK, R_DPORT_PERIP_UART1_MASK,
            R_DPORT_PERIP_UART2_MASK
        };
        Esp32PeriphGate *g;

        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        /* The UART needs the clock of the FIFO RAM it shares as well. */
        g = esp32_soc_add_gated_device(s, &s->uart[i], uart_base[i],
                                       ESP32_GATE_PERIP,
                                       uart_bit[i] |
                                       R_DPORT_PERIP_UART_MEM_MASK,
                                       uart_bit[i]);
        g->apb_clk = s->uart_apb_clk[i];
        g->ref_tick_clk = s->uart_ref_tick_clk[i];
        g->volatile_reg = A_UART_FIFO;
        g->has_volatile_reg = true;
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_UHCI_COUNT; ++i) {
        const hwaddr uhci_base[] = { DR_REG_UHCI0_BASE, DR_REG_UHCI1_BASE };
        const uint32_t uhci_bit[] = {
            R_DPORT_PERIP_UHCI0_MASK, R_DPORT_PERIP_UHCI1_MASK
        };
        const int uhci_intr[] = {
            ETS_UHCI0_INTR_SOURCE, ETS_UHCI1_INTR_SOURCE
        };
        Esp32PeriphGate *g;

        object_property_set_link(OBJECT(&s->uhci[i]), "dma-mr",
                                 OBJECT(sys_mem), &error_abort);
        for (int u = 0; u < ESP32_UART_COUNT; ++u) {
            char name[8];

            snprintf(name, sizeof(name), "uart%d", u);
            object_property_set_link(OBJECT(&s->uhci[i]), name,
                                     OBJECT(&s->uart[u]), &error_abort);
        }
        qdev_realize(DEVICE(&s->uhci[i]), &s->periph_bus, &error_fatal);
        /* [spec:nuos:req:emu.esp32.uhci] */
        g = esp32_soc_add_gated_device(s, &s->uhci[i], uhci_base[i],
                                       ESP32_GATE_PERIP, uhci_bit[i],
                                       uhci_bit[i]);
        g->apb_clk = s->uhci_apb_clk[i];
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uhci[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, uhci_intr[i]));
    }

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        Esp32PeriphGate *g;

        qdev_realize(DEVICE(&s->frc_timer[i]), &s->periph_bus, &error_fatal);

        g = esp32_soc_add_gated_device(s, &s->frc_timer[i],
                                       DR_REG_FRC_TIMER_BASE +
                                       i * ESP32_FRC_TIMER_STRIDE,
                                       ESP32_GATE_PERIP,
                                       R_DPORT_PERIP_TIMERS_MASK,
                                       R_DPORT_PERIP_TIMERS_MASK);
        g->apb_clk = s->frc_apb_clk[i];

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->frc_timer[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_TIMER1_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        s->timg[i].id = i;

        const hwaddr timg_base[] = {DR_REG_TIMERGROUP0_BASE, DR_REG_TIMERGROUP1_BASE};
        const uint32_t timg_bit[] = {
            R_DPORT_PERIP_TIMERGROUP_MASK, R_DPORT_PERIP_TIMERGROUP1_MASK
        };
        Esp32PeriphGate *g;

        qdev_realize(DEVICE(&s->timg[i]), &s->periph_bus, &error_fatal);

        g = esp32_soc_add_gated_device(s, &s->timg[i], timg_base[i],
                                       ESP32_GATE_PERIP, timg_bit[i],
                                       timg_bit[i]);
        g->apb_clk = s->timg_apb_clk[i];

        int timg_level_int[] = { ETS_TG0_T0_LEVEL_INTR_SOURCE, ETS_TG1_T0_LEVEL_INTR_SOURCE };
        int timg_edge_int[] = { ETS_TG0_T0_EDGE_INTR_SOURCE, ETS_TG1_T0_EDGE_INTR_SOURCE };
        for (Esp32TimgInterruptType it = TIMG_T0_INT; it < TIMG_INT_MAX; ++it) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), it, qdev_get_gpio_in(intmatrix_dev, timg_level_int[i] + it));
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), TIMG_INT_MAX + it, qdev_get_gpio_in(intmatrix_dev, timg_edge_int[i] + it));
        }

        qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_CPU_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_SYS_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_SYS_RESET_GPIO, i));
    }
    s->timg[0].wdt_en_at_reset = true;

    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
        const hwaddr spi_base[] = {
            DR_REG_SPI0_BASE, DR_REG_SPI1_BASE, DR_REG_SPI2_BASE, DR_REG_SPI3_BASE
        };
        /* SPI0 (the cache's) and SPI1 share a bit. */
        const uint32_t spi_bit[] = {
            R_DPORT_PERIP_SPI01_MASK, R_DPORT_PERIP_SPI01_MASK,
            R_DPORT_PERIP_SPI2_MASK, R_DPORT_PERIP_SPI3_MASK
        };
        qdev_realize(DEVICE(&s->spi[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_gated_device(s, &s->spi[i], spi_base[i],
                                   ESP32_GATE_PERIP, spi_bit[i], spi_bit[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SPI0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_I2C_COUNT; i++) {
        const hwaddr i2c_base[] = {
            DR_REG_I2C_EXT_BASE, DR_REG_I2C1_EXT_BASE
        };
        const uint32_t i2c_bit[] = {
            R_DPORT_PERIP_I2C_EXT0_MASK, R_DPORT_PERIP_I2C_EXT1_MASK
        };
        Esp32PeriphGate *g;

        qdev_realize(DEVICE(&s->i2c[i]), &s->periph_bus, &error_fatal);

        g = esp32_soc_add_gated_device(s, &s->i2c[i], i2c_base[i],
                                       ESP32_GATE_PERIP, i2c_bit[i],
                                       i2c_bit[i]);
        g->volatile_reg = A_I2C_FIFO_DATA;
        g->has_volatile_reg = true;

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_I2C_EXT0_INTR_SOURCE + i));
    }

    /* TWAI model passes intmatrix IRQs to the SJA1000 controller model
     * in realize function. That means that irq linking MUST be
     * performed before realization of TWAI peripheral.
     */
    qdev_realize(DEVICE(&s->twai), &s->periph_bus, &error_fatal);
    {
        /* Reading the SJA1000 interrupt register clears it. */
        Esp32PeriphGate *g = esp32_soc_add_gated_device(
            s, &s->twai, DR_REG_CAN_BASE, ESP32_GATE_PERIP,
            R_DPORT_PERIP_TWAI_MASK, R_DPORT_PERIP_TWAI_MASK);
        g->volatile_reg = ESP32_TWAI_INT_RAW_OFFSET;
        g->has_volatile_reg = true;
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->twai), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_CAN_INTR_SOURCE));

    qdev_realize(DEVICE(&s->rng), &s->periph_bus, &error_fatal);
    /* The RNG has a clock bit but no reset bit. */
    esp32_soc_add_gated_device(s, &s->rng, ESP32_RNG_BASE, ESP32_GATE_WIFI,
                               R_DPORT_WIFI_CLK_EN_RNG_MASK, 0);

    qdev_realize(DEVICE(&s->efuse), &s->periph_bus, &error_fatal);
    esp32_soc_add_gated_device(s, &s->efuse, DR_REG_EFUSE_BASE,
                               ESP32_GATE_PERIP, R_DPORT_PERIP_EFUSE_MASK,
                               R_DPORT_PERIP_EFUSE_MASK);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));

    qdev_realize(DEVICE(&s->flash_enc), &s->periph_bus, &error_abort);
    esp32_soc_add_periph_device(sys_mem, &s->flash_enc, DR_REG_SPI_ENCRYPT_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->efuse), ESP32_EFUSE_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_EFUSE_UPDATE_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_FLASH_ENC_EN_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_ENC_EN_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_FLASH_DEC_EN_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_DEC_EN_GPIO, 0));

    qdev_realize(DEVICE(&s->sdmmc), &s->periph_bus, &error_abort);
    esp32_soc_add_gated_device(s, &s->sdmmc, DR_REG_SDMMC_BASE,
                               ESP32_GATE_WIFI,
                               R_DPORT_WIFI_CLK_EN_SDIO_HOST_MASK,
                               R_DPORT_CORE_RST_EN_SDIO_HOST_MASK);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdmmc), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_SDIO_HOST_INTR_SOURCE));

    /* Provide internal RAM MemoryRegion to the RGB display */
    s->rgb.intram = dram;
    qdev_realize(DEVICE(&s->rgb), &s->periph_bus, &error_abort);
    esp32_soc_add_periph_device(sys_mem, &s->rgb, DR_REG_FRAMEBUF_BASE);
    memory_region_add_subregion_overlap(sys_mem, esp32_memmap[ESP32_MEMREGION_FRAMEBUF].base, &s->rgb.vram, 0);

    esp32_soc_add_unimp_device(sys_mem, "esp32.analog", DR_REG_ANA_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.rtcio", DR_REG_RTCIO_BASE, 0x400);
    esp32_soc_add_unimp_device(sys_mem, "esp32.rtcio", DR_REG_SENS_BASE, 0x400);
    esp32_soc_add_unimp_device(sys_mem, "esp32.iomux", DR_REG_IO_MUX_BASE, 0x2000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.hinf", DR_REG_HINF_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.slc", DR_REG_SLC_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.slchost", DR_REG_SLCHOST_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.i2s0", DR_REG_I2S_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.i2s1", DR_REG_I2S1_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.rmt", DR_REG_RMT_BASE, 0x1000);
    esp32_soc_add_unimp_device(sys_mem, "esp32.pcnt", DR_REG_PCNT_BASE, 0x1000);

    qemu_register_reset((QEMUResetHandler*) esp32_soc_reset, dev);
}

static void esp32_soc_init(Object *obj)
{
    Esp32SocState *s = ESP32_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];

    MemoryRegion *system_memory = get_system_memory();

    qbus_init(&s->periph_bus, sizeof(s->periph_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-periph-bus");
    qbus_init(&s->rtc_bus, sizeof(s->rtc_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-rtc-bus");

    /* CPU_CLK after reset: XTAL_CLK, undivided */
    s->cpu_clk = clock_new(obj, "cpu-clk");
    clock_set_hz(s->cpu_clk, 40000000);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);
        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32_CPU);
        qdev_connect_clock_in(DEVICE(&s->cpu[i]), "clk-in", s->cpu_clk);

        const uint32_t cpuid[ESP32_CPU_COUNT] = { 0xcdcd, 0xabab };
        s->cpu[i].env.sregs[PRID] = cpuid[i];

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState* cs = CPU(&s->cpu[i]);
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32_UART);
        snprintf(name, sizeof(name), "uart%d-apb", i);
        s->uart_apb_clk[i] = clock_new(obj, name);
        qdev_connect_clock_in(DEVICE(&s->uart[i]), "apb", s->uart_apb_clk[i]);
        snprintf(name, sizeof(name), "uart%d-ref", i);
        s->uart_ref_tick_clk[i] = clock_new(obj, name);
        qdev_connect_clock_in(DEVICE(&s->uart[i]), "ref_tick",
                              s->uart_ref_tick_clk[i]);
    }

    for (int i = 0; i < ESP32_UHCI_COUNT; ++i) {
        snprintf(name, sizeof(name), "uhci%d", i);
        object_initialize_child(obj, name, &s->uhci[i], TYPE_ESP32_UHCI);
        snprintf(name, sizeof(name), "uhci%d-apb", i);
        s->uhci_apb_clk[i] = clock_new(obj, name);
        qdev_connect_clock_in(DEVICE(&s->uhci[i]), "apb", s->uhci_apb_clk[i]);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]), "chardev");

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_ESP32_GPIO);

    object_initialize_child(obj, "dport", &s->dport, TYPE_ESP32_DPORT);

    object_initialize_child(obj, "apb_ctrl", &s->apb_ctrl, TYPE_ESP32_APB_CTRL);

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32_INTMATRIX);

    object_initialize_child(obj, "crosscore_int", &s->crosscore_int, TYPE_ESP32_CROSSCORE_INT);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32_RTC_CNTL);

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        snprintf(name, sizeof(name), "frc%d", i);
        object_initialize_child(obj, name, &s->frc_timer[i], TYPE_ESP32_FRC_TIMER);
        snprintf(name, sizeof(name), "frc%d-apb", i);
        s->frc_apb_clk[i] = clock_new(obj, name);
        qdev_connect_clock_in(DEVICE(&s->frc_timer[i]), "apb",
                              s->frc_apb_clk[i]);
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        snprintf(name, sizeof(name), "timg%d", i);
        object_initialize_child(obj, name, &s->timg[i], TYPE_ESP32_TIMG);
        snprintf(name, sizeof(name), "timg%d-apb", i);
        s->timg_apb_clk[i] = clock_new(obj, name);
        qdev_connect_clock_in(DEVICE(&s->timg[i]), "apb", s->timg_apb_clk[i]);
    }

    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
        snprintf(name, sizeof(name), "spi%d", i);
        object_initialize_child(obj, name, &s->spi[i], TYPE_ESP32_SPI);
    }

    for (int i = 0; i < ESP32_I2C_COUNT; ++i) {
        snprintf(name, sizeof(name), "i2c%d", i);
        object_initialize_child(obj, name, &s->i2c[i], TYPE_ESP32_I2C);
    }

    object_initialize_child(obj, "twai", &s->twai, TYPE_ESP32_TWAI);

    object_initialize_child(obj, "rng", &s->rng, TYPE_ESP32_RNG);

    object_initialize_child(obj, "sha", &s->sha, TYPE_ESP32_SHA);

    object_initialize_child(obj, "aes", &s->aes, TYPE_ESP32_AES);

    object_initialize_child(obj, "ledc", &s->ledc, TYPE_ESP32_LEDC);

    object_initialize_child(obj, "rsa", &s->rsa, TYPE_ESP32_RSA);

    object_initialize_child(obj, "efuse", &s->efuse, TYPE_ESP32_EFUSE);

    object_initialize_child(obj, "flash_enc", &s->flash_enc, TYPE_ESP32_FLASH_ENCRYPTION);

    object_initialize_child(obj, "sdmmc", &s->sdmmc, TYPE_DWC_SDMMC);

    object_initialize_child(obj, "rgb", &s->rgb, TYPE_ESP_RGB);

    qdev_init_gpio_in_named(DEVICE(s), esp32_dig_reset, ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_reset, ESP32_RTC_CPU_RESET_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_stall, ESP32_RTC_CPU_STALL_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32_clk_update, ESP32_RTC_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_periph_clk_update,
                            ESP32_DPORT_PERIPH_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_timg_cpu_reset, ESP32_TIMG_WDT_CPU_RESET_GPIO, 2);
    qdev_init_gpio_in_named(DEVICE(s), esp32_timg_sys_reset, ESP32_TIMG_WDT_SYS_RESET_GPIO, 2);
}

static void esp32_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32_soc_realize;
}

static const TypeInfo esp32_soc_info = {
    .name = TYPE_ESP32_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32SocState),
    .instance_init = esp32_soc_init,
    .class_init = esp32_soc_class_init
};

static void esp32_soc_register_types(void)
{
    type_register_static(&esp32_soc_info);
}

type_init(esp32_soc_register_types)


static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    TranslateForDebugResult tres;

    if (!cpu_translate_for_debug(CPU(cpu), addr, &tres)) {
        return -1;
    }
    return tres.physaddr;
}


struct Esp32MachineState {
    MachineState parent;

    Esp32SocState esp32;
    DeviceState *flash_dev;
};
#define TYPE_ESP32_MACHINE MACHINE_TYPE_NAME("esp32")

OBJECT_DECLARE_SIMPLE_TYPE(Esp32MachineState, ESP32_MACHINE)


static void esp32_machine_init_spi_flash(Esp32SocState *ss, BlockBackend* blk)
{
    /* "main" flash chip is attached to SPI1, CS0 */
    DeviceState *spi_master = DEVICE(&ss->spi[1]);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");

    /* select the flash chip based on the image size */
    int64_t image_size = blk_getlength(blk);
    const char* flash_chip_model = NULL;
    switch (image_size) {
        case 2 * 1024 * 1024: flash_chip_model = "w25x16"; break;
        case 4 * 1024 * 1024: flash_chip_model = "gd25q32"; break;
        case 8 * 1024 * 1024: flash_chip_model = "gd25q64"; break;
        case 16 * 1024 * 1024: flash_chip_model = "is25lp128"; break;
        default: error_report("Error: only 2, 4, 8, 16 MB flash images are supported"); return;
    }

    DeviceState *flash_dev = qdev_new(flash_chip_model);
    qdev_prop_set_drive(flash_dev, "drive", blk);
    qdev_prop_set_uint8(flash_dev, "cs", 0);
    qdev_realize_and_unref(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void esp32_machine_init_psram(Esp32SocState *ss, uint32_t size_mbytes)
{
    /* PSRAM attached to SPI1, CS1 */
    DeviceState *spi_master = DEVICE(&ss->spi[1]);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *psram = qdev_new(TYPE_SSI_PSRAM);
    qdev_prop_set_uint32(psram, "size_mbytes", size_mbytes);
    qdev_prop_set_uint8(psram, "cs", 1);
    qdev_realize_and_unref(psram, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 1,
                                qdev_get_gpio_in_named(psram, SSI_GPIO_CS, 0));
}

static void esp32_machine_init_i2c(Esp32SocState *s)
{
    /* It should be possible to create an I2C device from the command line,
     * however for this to work the I2C bus must be reachable from sysbus-default.
     * At the moment the peripherals are added to an unrelated bus, to avoid being
     * reset on CPU reset.
     * If we find a way to decouple peripheral reset from sysbus reset,
     * we can move them to the sysbus and thus enable creation of i2c devices.
     */
    DeviceState *i2c_master = DEVICE(&s->i2c[0]);
    I2CBus* i2c_bus = I2C_BUS(qdev_get_child_bus(i2c_master, "i2c"));
    I2CSlave* tmp105 = i2c_slave_create_simple(i2c_bus, "tmp105", 0x48);
    object_property_set_int(OBJECT(tmp105), "temperature", 25 * 1000, &error_fatal);
}

static void esp32_machine_init_openeth(Esp32SocState *ss)
{
    SysBusDevice *sbd;
    Esp32PeriphGate *g;
    hwaddr reg_base = DR_REG_EMAC_BASE;
    hwaddr desc_base = reg_base + 0x400;
    qemu_irq irq = qdev_get_gpio_in(DEVICE(&ss->intmatrix), ETS_ETH_MAC_INTR_SOURCE);

    DeviceState* open_eth_dev = qemu_create_nic_device("open_eth", true, NULL);
    if (!open_eth_dev) {
        return;
    }

    ss->eth = open_eth_dev;
    sbd = SYS_BUS_DEVICE(open_eth_dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_connect_irq(sbd, 0, irq);
    /* The OpenCores MAC stands in for the EMAC, behind the EMAC's bits. */
    assert(ss->n_gates < ESP32_GATE_MAX);
    g = &ss->gate[ss->n_gates++];
    g->dev = open_eth_dev;
    g->regs = ESP32_GATE_WIFI;
    g->clk_mask = R_DPORT_WIFI_CLK_EN_EMAC_MASK;
    g->rst_mask = R_DPORT_CORE_RST_EN_EMAC_MASK;
    esp32_gate_map(g, sysbus_mmio_get_region(sbd, 0), reg_base, false);
    esp32_gate_map(g, sysbus_mmio_get_region(sbd, 1), desc_base, false);
}

static void esp32_machine_init_sd(Esp32SocState *ss)
{
    DriveInfo *dinfo = drive_get(IF_SD, 0, 0);
    if (dinfo) {
        DeviceState *card;

        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
        /* See the comment on not using sysbus-default in esp32_machine_init_i2c */
        DeviceState *sdmmc = DEVICE(&ss->sdmmc);
        SDBus* sd_bus = SD_BUS(qdev_get_child_bus(sdmmc, "sd-bus"));
        qdev_realize_and_unref(card, BUS(sd_bus), &error_fatal);
    }
}

static void esp32_machine_init(MachineState *machine)
{
    BlockBackend* blk = NULL;
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    Esp32MachineState *ms = ESP32_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->esp32, TYPE_ESP32_SOC);
    Esp32SocState *ss = ESP32_SOC(&ms->esp32);

    if (blk) {
        ss->dport.flash_blk = blk;
    }
    qdev_prop_set_chr(DEVICE(ss), "serial0", serial_hd(0));
    qdev_prop_set_chr(DEVICE(ss), "serial1", serial_hd(1));
    qdev_prop_set_chr(DEVICE(ss), "serial2", serial_hd(2));
    if (machine->ram_size > 4 * MiB) {
        error_report("PSRAM larger than 4 MiB is not supported");
        exit(1);
    }
    if (machine->ram_size > 0) {
        qdev_prop_set_bit(DEVICE(&ss->dport), "has_psram", true);
    }

    qdev_realize(DEVICE(ss), NULL, &error_fatal);

    if (blk) {
        esp32_machine_init_spi_flash(ss, blk);
    }

    if (machine->ram_size > 0) {
        esp32_machine_init_psram(ss, machine->ram_size <= 2 * MiB ? 2 : 4);
    }

    esp32_machine_init_i2c(ss);

    esp32_machine_init_openeth(ss);

    esp32_machine_init_sd(ss);

    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&ss->cpu[0]));

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        if (machine->firmware) {
            warn_report("both -bios and -kernel given; loading only -kernel");
        }
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int size = load_elf(load_elf_filename, NULL,
                               translate_phys_addr, &ss->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, NULL, 0, EM_XTENSA, 0, 0);
        if (size < 0) {
            error_report("Error: could not load ELF file '%s'", load_elf_filename);
            exit(1);
        }

        if (elf_entry != XCHAL_RESET_VECTOR_PADDR) {
            // Since ROM is empty when loading elf file AND
            // PC value is 0x40000400 after reset
            // need to jump to elf entry point to run a programm
            uint8_t p[4];
            memcpy(p, &elf_entry, 4);
            uint8_t boot[] = {
                0x06, 0x01, 0x00,       /* j    1 */
                0x00,                   /* .literal_position */
                p[0], p[1], p[2], p[3], /* .literal elf_entry */
                                        /* 1: */
                0x01, 0xff, 0xff,       /* l32r a0, elf_entry */
                0xa0, 0x00, 0x00,       /* jx   a0 */
            };
            // Write boot function to reset-vector address (0x40000400) of the CPU 0
            rom_add_blob_fixed_as("boot", boot, sizeof(boot), XCHAL_RESET_VECTOR_PADDR, CPU(&ss->cpu[0])->as);
            ss->cpu[0].env.pc = XCHAL_RESET_VECTOR_PADDR;
        }
    } else {
        char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32-v3-rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found (1)");
            exit(1);
        }

        load_image_targphys_as(rom_binary, esp32_memmap[ESP32_MEMREGION_IROM].base, esp32_memmap[ESP32_MEMREGION_IROM].size, CPU(&ss->cpu[0])->as,
                               &error_fatal);
        g_free(rom_binary);

        rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32-v3-rom-app.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found (2)");
            exit(1);
        }

        load_image_targphys_as(rom_binary, esp32_memmap[ESP32_MEMREGION_IROM].base, esp32_memmap[ESP32_MEMREGION_IROM].size, CPU(&ss->cpu[1])->as,
                               &error_fatal);
        g_free(rom_binary);
    }
}

/* Initialize machine type */
static void esp32_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32 machine";
    mc->init = esp32_machine_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_ram_size = 0;
}

static const TypeInfo esp32_info = {
    .name = TYPE_ESP32_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32MachineState),
    .class_init = esp32_machine_class_init,
};

static void esp32_machine_type_init(void)
{
    type_register_static(&esp32_info);
}

type_init(esp32_machine_type_init);
