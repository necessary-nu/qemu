/*
 * Espressif ESP32-P4 machine (ESP32-P4NRW32X)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 9
 * (System and Memory), and ESP-IDF's reg_base.h for v3, which is the
 * authority for the peripheral address map: the TRM's table 9.3-2 omits
 * the MSPI controllers, PAU, PVT, the DDR PHY, the cache controller, the
 * memory monitor, the audio ADDC, the RTC calibration block, LP TRNG and
 * LP touch, orders the ECC, digital signature and HMAC blocks otherwise,
 * and has typing errors in the ends of the CSI host and HP_SYS ranges.
 *
 * An ESP32-P4NRW32X, v3.x silicon, on a board with off-package flash:
 *
 *   - Two HP cores (RV32IMAFC with Zc and Zb) and the LP core (RV32IMAC).
 *     The HP cores start at the HP ROM, 0x4FC00000, where
 *     LP_CLKRST_HPCORE<n>_STAT_VECTOR_SEL points them out of reset (TRM
 *     section 11.1.3 gives 0x30100000, which the register's description,
 *     ESP-IDF and the ROM's own entry point contradict), or at LP SRAM
 *     once software clears it; core 1 is held in reset with its clock
 *     gated until software releases it through HP_SYS_CLKRST. The LP core,
 *     which starts at the LP ROM, is held in reset: the PMU, which starts
 *     it, is not modelled. Each core executes at the clock that drives it,
 *     the HP cores at CPU_CLK and the LP core at LP_DYN_FAST_CLK, and
 *     under icount an instruction takes a cycle of it.
 *   - The HP ROM (128 KiB) at 0x4FC00000, and at 0x8FC00000 for direct
 *     access; HP L2MEM (768 KiB) at 0x4FF00000, and at 0x8FF00000 for
 *     direct access; the HP TCM (8 KiB) at 0x30100000; the LP ROM (16 KiB)
 *     at 0x50100000; LP SRAM (32 KiB) at 0x50108000. L2MEM is all RAM:
 *     none of it is given to the L2 cache.
 *   - The external memory windows, 64 MiB each: flash at 0x40000000 and
 *     PSRAM at 0x48000000, both again at 0x8xxxxxxx for direct access.
 *     The cache's MMU maps them onto the flash and PSRAM in 64 KiB pages,
 *     and is not modelled, so the windows are unimplemented devices. Behind
 *     them, the 32 MiB of in-package PSRAM is a RAM region of its own, and
 *     the flash a W25Q part on the bus of the flash MSPI controllers; as
 *     nothing maps them, and the controllers are not modelled either, the
 *     cores cannot reach them.
 *   - The reset and clock control, HP_SYS_CLKRST, LP_AON_CLKRST and
 *     LPPERI, with the clock tree from the crystal, the RC oscillators and
 *     the PLLs to every block's clocks, and the analog I2C controller
 *     through which software configures the PLLs; and LP_SYS's
 *     BOOT_ADDR_HP_CORE1, the rest of whose registers are logged as
 *     unimplemented.
 *   - The eFuse controller and its array; nothing takes its interrupt, as
 *     the interrupt matrix is not modelled.
 *   - Every other peripheral block the chip decodes, as an unimplemented
 *     device named after the block. Addresses the chip does not decode,
 *     the reserved ranges of TRM table 9.3-1 and the unassigned slots
 *     between blocks, answer with a bus error, which the cores take as an
 *     access fault.
 *   - The CLINT and CLIC address ranges of the HP cores, also as
 *     unimplemented devices: the cores have no interrupt controller.
 *
 * The eFuses start as a factory-fresh chip's and are lost when QEMU exits,
 * unless a raw file keeps them: 336 bytes of data, or the whole array in
 * a file longer than 512 bytes. A blank, all-zero, file starts as a
 * factory-fresh chip:
 *
 *   -drive file=efuse.bin,if=none,format=raw,id=efuse
 *   -global driver=nvram.esp32p4.efuse,property=drive,value=efuse
 *
 * The flash size is a property of the board, with no default:
 *
 *   -M esp32p4,flash-size=16M
 *
 * Without it the board has no flash, and a flash image is refused. The
 * flash starts erased; to give it contents, and keep what the guest erases
 * and programs, back it with a raw image of exactly that size:
 *
 *   -drive if=mtd,format=raw,file=flash.img
 *
 * The HP ROM image is the esp32p4-rev3-rom.bin firmware file, or the raw
 * image -bios names. With -kernel, an ELF's segments are loaded into
 * L2MEM, the TCM or LP SRAM, and core 0 starts at the ELF's entry point
 * rather than in the ROM; core 1 is still held for software to release.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "elf.h"
#include "exec/icount.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "hw/riscv/esp32p4.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/reset.h"

/*
 * The blocks of the address map that are not modelled, each an
 * unimplemented device "esp32p4.<name>" over the whole range the chip
 * decodes for it.
 */
static const struct {
    const char *name;
    hwaddr base;
    hwaddr size;
} esp32p4_unimplemented[] = {
    /* The HP cores' local interrupt controllers (TRM table 2.8-1). */
    { "clint",            0x20000000, 128 * KiB },
    { "clic",             0x20800000, 128 * KiB },
    { "clic-user",        0x20b00000, 128 * KiB },

    /* HP CPU peripherals. */
    { "trace0",           0x3ff04000, 4 * KiB },
    { "trace1",           0x3ff05000, 4 * KiB },
    { "bus-monitor",      0x3ff06000, 4 * KiB },
    { "l2mem-monitor",    0x3ff0e000, 4 * KiB },
    { "tcm-monitor",      0x3ff0f000, 4 * KiB },
    { "cache",            0x3ff10000, 4 * KiB },

    /* The external memory windows. */
    { "flash-window",        ESP32P4_FLASH_WINDOW_BASE,
                             ESP32P4_EXT_WINDOW_SIZE },
    { "psram-window",        ESP32P4_PSRAM_WINDOW_BASE,
                             ESP32P4_EXT_WINDOW_SIZE },
    { "flash-window-direct", ESP32P4_FLASH_WINDOW_DIRECT_BASE,
                             ESP32P4_EXT_WINDOW_SIZE },
    { "psram-window-direct", ESP32P4_PSRAM_WINDOW_DIRECT_BASE,
                             ESP32P4_EXT_WINDOW_SIZE },

    /* HP peripherals 0. */
    { "usb-otg-hs",       0x50000000, 256 * KiB },
    { "usb-otg-fs",       0x50040000, 256 * KiB },
    { "usb-wrap",         0x50080000, 4 * KiB },
    { "vdma",             0x50081000, 4 * KiB },
    { "pau",              0x50082000, 4 * KiB },
    { "sdmmc",            0x50083000, 4 * KiB },
    { "h264",             0x50084000, 4 * KiB },
    { "ahb-dma",          0x50085000, 4 * KiB },
    { "jpeg",             0x50086000, 4 * KiB },
    { "ppa",              0x50087000, 4 * KiB },
    { "dma2d",            0x50088000, 4 * KiB },
    { "keymng",           0x50089000, 4 * KiB },
    { "axi-dma",          0x5008a000, 8 * KiB },
    { "flash-spi0",       0x5008c000, 4 * KiB },
    /* flash-spi1 at 0x5008d000 carries the flash's bus. */
    { "psram-mspi0",      0x5008e000, 4 * KiB },
    { "psram-mspi1",      0x5008f000, 4 * KiB },
    { "aes",              0x50090000, 4 * KiB },
    { "sha",              0x50091000, 4 * KiB },
    { "rsa",              0x50092000, 4 * KiB },
    { "ecc",              0x50093000, 4 * KiB },
    { "ds",               0x50094000, 4 * KiB },
    { "hmac",             0x50095000, 4 * KiB },
    { "ecdsa",            0x50096000, 4 * KiB },
    { "emac",             0x50098000, 16 * KiB },
    { "usb-phy",          0x5009c000, 4 * KiB },
    { "ddr-phy",          0x5009d000, 4 * KiB },
    { "pvt",              0x5009e000, 4 * KiB },
    { "csi-host",         0x5009f000, 2 * KiB },
    { "csi-bridge",       0x5009f800, 2 * KiB },
    { "dsi-host",         0x500a0000, 2 * KiB },
    { "dsi-bridge",       0x500a0800, 2 * KiB },
    { "isp",              0x500a1000, 4 * KiB },
    { "rmt",              0x500a2000, 4 * KiB },
    { "bitscrambler",     0x500a3000, 4 * KiB },
    { "axi-icm",          0x500a4000, 4 * KiB },
    { "hp-peri-pms",      0x500a5000, 2 * KiB },
    { "lp2hp-peri-pms",   0x500a5800, 2 * KiB },
    { "dma-pms",          0x500a6000, 4 * KiB },
    { "h264-dma",         0x500a7000, 4 * KiB },

    /* HP peripherals 1. */
    { "mcpwm0",           0x500c0000, 4 * KiB },
    { "mcpwm1",           0x500c1000, 4 * KiB },
    { "timg0",            0x500c2000, 4 * KiB },
    { "timg1",            0x500c3000, 4 * KiB },
    { "i2c0",             0x500c4000, 4 * KiB },
    { "i2c1",             0x500c5000, 4 * KiB },
    { "i2s0",             0x500c6000, 4 * KiB },
    { "i2s1",             0x500c7000, 4 * KiB },
    { "i2s2",             0x500c8000, 4 * KiB },
    { "pcnt",             0x500c9000, 4 * KiB },
    { "uart0",            0x500ca000, 4 * KiB },
    { "uart1",            0x500cb000, 4 * KiB },
    { "uart2",            0x500cc000, 4 * KiB },
    { "uart3",            0x500cd000, 4 * KiB },
    { "uart4",            0x500ce000, 4 * KiB },
    { "parl-io",          0x500cf000, 4 * KiB },
    { "spi2",             0x500d0000, 4 * KiB },
    { "spi3",             0x500d1000, 4 * KiB },
    { "usb-serial-jtag",  0x500d2000, 4 * KiB },
    { "ledc",             0x500d3000, 4 * KiB },
    { "etm",              0x500d5000, 4 * KiB },
    { "intmtx",           0x500d6000, 4 * KiB },
    { "twai0",            0x500d7000, 4 * KiB },
    { "twai1",            0x500d8000, 4 * KiB },
    { "twai2",            0x500d9000, 4 * KiB },
    { "i3c-mst",          0x500da000, 4 * KiB },
    { "i3c-slv",          0x500db000, 4 * KiB },
    { "lcd-cam",          0x500dc000, 4 * KiB },
    { "adc",              0x500de000, 4 * KiB },
    { "uhci",             0x500df000, 4 * KiB },
    { "gpio",             0x500e0000, 4 * KiB },
    { "io-mux",           0x500e1000, 4 * KiB },
    { "systimer",         0x500e2000, 4 * KiB },
    { "mem-monitor",      0x500e3000, 4 * KiB },
    { "audio-addc",       0x500e4000, 4 * KiB },
    { "hp-sys",           0x500e5000, 4 * KiB },

    /* The LP ROM, whose contents Espressif does not publish. */
    { "lp-rom",           ESP32P4_LP_ROM_BASE, ESP32P4_LP_ROM_SIZE },

    /* The MIPI bridges' memories. */
    { "mipi-csi-mem",     0x50104000, 4 * KiB },
    { "mipi-dsi-mem",     0x50105000, 4 * KiB },

    /* LP always-on peripherals. */
    { "lp-timer",         0x50112000, 4 * KiB },
    { "lp-analog-peri",   0x50113000, 4 * KiB },
    { "lp-huk",           0x50114000, 4 * KiB },
    { "pmu",              0x50115000, 4 * KiB },
    { "lp-wdt",           0x50116000, 4 * KiB },
    { "lp-mailbox",       0x50118000, 4 * KiB },
    { "rtc-cali",         0x50119000, 4 * KiB },

    /* LP peripherals. */
    { "lp-uart",          0x50121000, 4 * KiB },
    { "lp-i2c",           0x50122000, 4 * KiB },
    { "lp-spi",           0x50123000, 4 * KiB },
    { "lp-i2s",           0x50125000, 4 * KiB },
    { "lp-trng",          0x50126000, 4 * KiB },
    { "lp-adc",           0x50127000, 4 * KiB },
    { "lp-touch",         0x50128000, 4 * KiB },
    { "lp-gpio",          0x5012a000, 4 * KiB },
    { "lp-iomux",         0x5012b000, 4 * KiB },
    { "lp-intr",          0x5012c000, 4 * KiB },
    { "lp-peri-pms",      0x5012e000, 2 * KiB },
    { "hp2lp-peri-pms",   0x5012e800, 2 * KiB },
    { "lp-tsens",         0x5012f000, 4 * KiB },
};

#define ESP32P4_FLASH_SPI1_BASE 0x5008d000

static DeviceState *esp32p4_unimplemented_device(const char *name,
                                                 hwaddr size)
{
    g_autofree char *full = g_strdup_printf("esp32p4.%s", name);
    DeviceState *dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);

    qdev_prop_set_string(dev, "name", full);
    qdev_prop_set_uint64(dev, "size", size);
    return dev;
}

static void esp32p4_map(DeviceState *dev, hwaddr base)
{
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_init_memory(ESP32P4MachineState *s)
{
    MemoryRegion *sys_mem = get_system_memory();
    /* The machine is not a device, so cannot own RAM. */
    Object *owner = NULL;

    memory_region_init_ram(&s->tcm, owner, "esp32p4.tcm", ESP32P4_TCM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sys_mem, ESP32P4_TCM_BASE, &s->tcm);

    memory_region_init_rom(&s->hp_rom, owner, "esp32p4.hp-rom",
                           ESP32P4_HP_ROM_SIZE, &error_fatal);
    memory_region_add_subregion(sys_mem, ESP32P4_HP_ROM_BASE, &s->hp_rom);
    memory_region_init_alias(&s->hp_rom_direct, owner,
                             "esp32p4.hp-rom-direct", &s->hp_rom, 0,
                             ESP32P4_HP_ROM_SIZE);
    memory_region_add_subregion(sys_mem, ESP32P4_HP_ROM_DIRECT_BASE,
                                &s->hp_rom_direct);

    memory_region_init_ram(&s->l2mem, owner, "esp32p4.l2mem",
                           ESP32P4_L2MEM_SIZE, &error_fatal);
    memory_region_add_subregion(sys_mem, ESP32P4_L2MEM_BASE, &s->l2mem);
    memory_region_init_alias(&s->l2mem_direct, owner, "esp32p4.l2mem-direct",
                             &s->l2mem, 0, ESP32P4_L2MEM_SIZE);
    memory_region_add_subregion(sys_mem, ESP32P4_L2MEM_DIRECT_BASE,
                                &s->l2mem_direct);

    memory_region_init_ram(&s->lp_sram, owner, "esp32p4.lp-sram",
                           ESP32P4_LP_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sys_mem, ESP32P4_LP_SRAM_BASE, &s->lp_sram);

    /* Behind the PSRAM windows, for the MMU to map. */
    memory_region_init_ram(&s->psram, owner, "esp32p4.psram",
                           ESP32P4_PSRAM_SIZE, &error_fatal);
}

/* The Winbond W25Q parts of each flash size. */
static const char *esp32p4_flash_part(uint64_t size)
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
    case 16 * MiB:
        return "w25q128";
    case 32 * MiB:
        return "w25q256";
    case 64 * MiB:
        return "w25q512jv";
    default:
        return "w25q01jvq";
    }
}

/*
 * The flash chip on the bus the flash MSPI controllers drive, SPI1 for
 * software's commands and SPI0 for the cache's reads, neither of which is
 * modelled: the bus hangs off SPI1's unimplemented device, and nothing
 * selects the chip. It is erased unless backed by the first -drive if=mtd.
 *
 * A board with no flash has nothing on the bus. Nothing then drives the
 * controllers' data inputs, which out of reset have no pull-up or
 * pull-down (IOMUX_MSPI_PIN_FLASH_*_WPU and _WPD clear), so this model
 * takes what they read as zero, as a QEMU SSI bus with no device returns.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_init_flash(ESP32P4MachineState *s)
{
    DeviceState *spi1 = esp32p4_unimplemented_device("flash-spi1", 4 * KiB);
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    DeviceState *flash;

    s->flash_bus = ssi_create_bus(spi1, "flash-spi");
    esp32p4_map(spi1, ESP32P4_FLASH_SPI1_BASE);

    if (!s->flash_size) {
        qemu_log("esp32p4: no flash is configured (-M esp32p4,flash-size="
                 "SIZE); the flash bus has no chip on it\n");
        return;
    }
    flash = qdev_new(esp32p4_flash_part(s->flash_size));
    qdev_prop_set_bit(flash, "write-enable-autoclear", true);
    if (dinfo) {
        BlockBackend *blk = blk_by_legacy_dinfo(dinfo);
        int64_t len = blk_getlength(blk);

        if (len != s->flash_size) {
            error_report("esp32p4: the flash image (-drive if=mtd) is %"
                         PRId64 " bytes; it must be flash-size, %" PRIu64
                         " bytes", len, s->flash_size);
            exit(1);
        }
        qdev_prop_set_drive_err(flash, "drive", blk, &error_fatal);
    }
    ssi_realize_and_unref(flash, s->flash_bus, &error_fatal);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_init_cpus(ESP32P4MachineState *s)
{
    int i;

    object_initialize_child(OBJECT(s), "hp-cluster", &s->hp_cluster,
                            TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(&s->hp_cluster), "cluster-id", 0);
    for (i = 0; i < ESP32P4_HP_CORES; i++) {
        object_initialize_child(OBJECT(&s->hp_cluster), "cpu[*]",
                                &s->hp_cpu[i], TYPE_ESP32P4_HP_CPU);
        qdev_prop_set_uint64(DEVICE(&s->hp_cpu[i]), "resetvec",
                             ESP32P4_HP_RESET_VECTOR);
        s->hp_cpu[i].parent_obj.env.mhartid = i;
        qdev_realize(DEVICE(&s->hp_cpu[i]), NULL, &error_fatal);
        qemu_register_resettable(OBJECT(&s->hp_cpu[i]));
    }
    qdev_realize(DEVICE(&s->hp_cluster), NULL, &error_fatal);

    /* The LP core is a different core, so in a cluster of its own. */
    object_initialize_child(OBJECT(s), "lp-cluster", &s->lp_cluster,
                            TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(&s->lp_cluster), "cluster-id", 1);
    object_initialize_child(OBJECT(&s->lp_cluster), "cpu", &s->lp_cpu,
                            TYPE_ESP32P4_LP_CPU);
    qdev_prop_set_uint64(DEVICE(&s->lp_cpu), "resetvec",
                         ESP32P4_LP_RESET_VECTOR);
    s->lp_cpu.parent_obj.env.mhartid = 0;
    qdev_realize(DEVICE(&s->lp_cpu), NULL, &error_fatal);
    qemu_register_resettable(OBJECT(&s->lp_cpu));
    qdev_realize(DEVICE(&s->lp_cluster), NULL, &error_fatal);
}

/*
 * LP_CLKRST_HPCORE<n>_STAT_VECTOR_SEL: the HP ROM, where core 0 with
 * -kernel starts at the ELF's entry point instead, or LP SRAM.
 */
/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_core_vector(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;
    uint64_t vector = ESP32P4_LP_SRAM_BASE;

    if (level) {
        vector = n == 0 && s->kernel_entry ? s->kernel_entry
                                           : ESP32P4_HP_RESET_VECTOR;
    }
    s->hp_cpu[n].parent_obj.env.resetvec = vector;
}

/*
 * The reset and clock control and the analog blocks it takes the PLLs
 * from, through the analog I2C controller; the cores' resets, clock
 * enables and reset vectors; and the clocks each core executes at. The
 * entry point the ROM on core 1 takes comes from LP_SYS.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_init_clocks(ESP32P4MachineState *s)
{
    DeviceState *clkrst, *ana, *mst;
    int i;

    object_initialize_child(OBJECT(s), "ana", &s->ana, TYPE_ESP32P4_ANA);
    object_initialize_child(OBJECT(s), "clkrst", &s->clkrst,
                            TYPE_ESP32P4_CLKRST);
    object_initialize_child(OBJECT(s), "lp-i2c-ana-mst", &s->i2c_ana_mst,
                            TYPE_ESP32P4_I2C_ANA_MST);
    ana = DEVICE(&s->ana);
    clkrst = DEVICE(&s->clkrst);
    mst = DEVICE(&s->i2c_ana_mst);

    qdev_connect_clock_in(ana, "xtal", qdev_get_clock_out(clkrst,
                                                          "osc-xtal"));
    for (i = 0; i < ESP32P4_ANA_NUM_PLLS; i++) {
        g_autofree char *in = g_strdup_printf("%s-in",
                                              esp32p4_ana_pll_names[i]);

        qdev_connect_clock_in(clkrst, in,
                              qdev_get_clock_out(ana,
                                                 esp32p4_ana_pll_names[i]));
    }
    sysbus_realize(SYS_BUS_DEVICE(ana), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(clkrst), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(clkrst), ESP32P4_CLKRST_MMIO_HP,
                    ESP32P4_HP_SYS_CLKRST_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(clkrst), ESP32P4_CLKRST_MMIO_LP,
                    ESP32P4_LP_CLKRST_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(clkrst), ESP32P4_CLKRST_MMIO_LPPERI,
                    ESP32P4_LPPERI_BASE);
    for (i = 0; i < ESP32P4_ANA_NUM_PLLS; i++) {
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_PLL_CAL_STOP, i,
                                    qdev_get_gpio_in_named(ana,
                                        ESP32P4_ANA_CAL_STOP, i));
        qdev_connect_gpio_out_named(ana, ESP32P4_ANA_CAL_END, i,
                                    qdev_get_gpio_in_named(clkrst,
                                        ESP32P4_CLKRST_PLL_CAL_END, i));
    }

    object_property_set_link(OBJECT(mst), "ana", OBJECT(ana), &error_abort);
    qdev_connect_clock_in(mst, "bus", qdev_get_clock_out(clkrst,
                                                         "lp-i2cmst"));
    qdev_connect_clock_in(mst, "lp-fast", qdev_get_clock_out(clkrst,
                                                             "lp-fast"));
    qdev_connect_clock_in(mst, "pll-f160m", qdev_get_clock_out(clkrst,
                                                               "pll-f160m"));
    sysbus_realize(SYS_BUS_DEVICE(mst), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(mst), 0,
                             ESP32P4_I2C_ANA_MST_BASE, ESP32P4_CLK_LP_I2CMST,
                             ESP32P4_RST_LP_I2CMST, 0);

    s->core_vector = qemu_allocate_irqs(esp32p4_core_vector, s,
                                        ESP32P4_HP_CORES);
    for (i = 0; i < ESP32P4_HP_CORES; i++) {
        DeviceState *cpu = DEVICE(&s->hp_cpu[i]);
        g_autofree char *clk = g_strdup_printf("core%d-cpu", i);

        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_CORE_RESET, i,
                                    qdev_get_gpio_in_named(cpu,
                                        ESP32P4_CPU_RESET, 0));
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_CORE_CLOCK, i,
                                    qdev_get_gpio_in_named(cpu,
                                        ESP32P4_CPU_CLOCK, 0));
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_CORE_VECTOR, i,
                                    s->core_vector[i]);
        /*
         * Under icount a core executes an instruction per cycle of its
         * CPU_CLK, at its current frequency, alongside the other cores.
         */
        icount_set_cpu_clock(CPU(cpu), qdev_get_clock_out(clkrst, clk));
    }
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_LP_CORE_RESET, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_cpu),
                                    ESP32P4_CPU_RESET, 0));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_LP_CORE_CLOCK, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_cpu),
                                    ESP32P4_CPU_CLOCK, 0));
    icount_set_cpu_clock(CPU(&s->lp_cpu), qdev_get_clock_out(clkrst,
                                                             "lp-core"));

    object_initialize_child(OBJECT(s), "lp-sys", &s->lp_sys,
                            TYPE_ESP32P4_LP_SYS);
    sysbus_realize(SYS_BUS_DEVICE(&s->lp_sys), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->lp_sys), 0, ESP32P4_LP_SYS_BASE);
}

/*
 * The eFuse controller. Its interrupt, LP_EFUSE_INTR (source 17 of TRM
 * table 13.4-1), goes to the interrupt matrix, which is not modelled.
 */
/* [spec:nuos:req:emu.esp32p4.efuse] */
static void esp32p4_init_efuse(ESP32P4MachineState *s)
{
    object_initialize_child(OBJECT(s), "efuse", &s->efuse,
                            TYPE_ESP32P4_EFUSE);
    sysbus_realize(SYS_BUS_DEVICE(&s->efuse), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->efuse), 0, ESP32P4_EFUSE_BASE);
}

/* The memories -kernel can load into: those the cores address directly. */
static bool esp32p4_kernel_segment_fits(uint32_t addr, uint32_t size)
{
    static const struct {
        hwaddr base;
        hwaddr size;
    } ram[] = {
        { ESP32P4_TCM_BASE, ESP32P4_TCM_SIZE },
        { ESP32P4_L2MEM_BASE, ESP32P4_L2MEM_SIZE },
        { ESP32P4_L2MEM_DIRECT_BASE, ESP32P4_L2MEM_SIZE },
        { ESP32P4_LP_SRAM_BASE, ESP32P4_LP_SRAM_SIZE },
    };
    int i;

    for (i = 0; i < ARRAY_SIZE(ram); i++) {
        if (addr >= ram[i].base && addr - ram[i].base < ram[i].size &&
            size <= ram[i].size - (addr - ram[i].base)) {
            return true;
        }
    }
    return false;
}

/*
 * Refuse an ELF with a segment outside those memories: the loader would
 * drop one in the flash or PSRAM windows without a word, as nothing maps
 * them, and one in the ROM would replace the ROM. Files that are not
 * 32-bit little-endian ELFs are left for the loader to reject.
 */
static void esp32p4_check_kernel(const char *filename)
{
    g_autofree char *data = NULL;
    gsize len;
    Elf32_Ehdr eh;
    uint32_t phoff;
    uint16_t phnum, phentsize;
    int i;

    if (!g_file_get_contents(filename, &data, &len, NULL) ||
        len < sizeof(eh) || memcmp(data, ELFMAG, SELFMAG) ||
        data[EI_CLASS] != ELFCLASS32 || data[EI_DATA] != ELFDATA2LSB) {
        return;
    }
    memcpy(&eh, data, sizeof(eh));
    phoff = le32_to_cpu(eh.e_phoff);
    phnum = le16_to_cpu(eh.e_phnum);
    phentsize = le16_to_cpu(eh.e_phentsize);
    if (phentsize < sizeof(Elf32_Phdr)) {
        return;
    }
    for (i = 0; i < phnum; i++) {
        Elf32_Phdr ph;
        uint32_t paddr, memsz;

        if (phoff + (uint64_t)(i + 1) * phentsize > len) {
            return;
        }
        memcpy(&ph, data + phoff + i * phentsize, sizeof(ph));
        paddr = le32_to_cpu(ph.p_paddr);
        memsz = le32_to_cpu(ph.p_memsz);
        if (le32_to_cpu(ph.p_type) != PT_LOAD || !memsz) {
            continue;
        }
        if (!esp32p4_kernel_segment_fits(paddr, memsz)) {
            error_report("esp32p4: '%s' has a segment at 0x%08" PRIx32
                         ", 0x%" PRIx32 " bytes, outside L2MEM, the TCM and "
                         "LP SRAM; -kernel loads only into those, as the "
                         "flash and PSRAM need the MMU, which is not "
                         "modelled", filename, paddr, memsz);
            exit(1);
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_load(ESP32P4MachineState *s)
{
    MachineState *machine = MACHINE(s);
    const char *rom = machine->firmware ? machine->firmware
                                        : ESP32P4_ROM_FILE;
    g_autofree char *path = qemu_find_file(QEMU_FILE_TYPE_BIOS, rom);
    uint64_t entry;

    if (!path) {
        error_report("esp32p4: cannot find the HP ROM image '%s'", rom);
        exit(1);
    }
    if (load_image_targphys(path, ESP32P4_HP_ROM_BASE, ESP32P4_HP_ROM_SIZE,
                            NULL) < 0) {
        error_report("esp32p4: cannot load the HP ROM image '%s' (at most "
                     "%u KiB)", path, (unsigned)(ESP32P4_HP_ROM_SIZE / KiB));
        exit(1);
    }

    if (!machine->kernel_filename) {
        return;
    }
    esp32p4_check_kernel(machine->kernel_filename);
    if (load_elf(machine->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                 NULL, NULL, ELFDATA2LSB, EM_RISCV, 1, 0) < 0) {
        error_report("esp32p4: cannot load the ELF file '%s'",
                     machine->kernel_filename);
        exit(1);
    }
    s->kernel_entry = entry;
    s->hp_cpu[0].parent_obj.env.resetvec = entry;
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_init(MachineState *machine)
{
    ESP32P4MachineState *s = ESP32P4_MACHINE(machine);
    int i;

    if (machine->ram_size != ESP32P4_L2MEM_SIZE) {
        error_report("esp32p4: L2MEM is fixed at %u KiB; -m is not "
                     "supported", (unsigned)(ESP32P4_L2MEM_SIZE / KiB));
        exit(1);
    }
    if (drive_get(IF_MTD, 0, 0) && !s->flash_size) {
        error_report("esp32p4: a flash image (-drive if=mtd) needs "
                     "-M esp32p4,flash-size=SIZE");
        exit(1);
    }

    esp32p4_init_memory(s);
    esp32p4_init_cpus(s);
    esp32p4_init_clocks(s);
    esp32p4_init_efuse(s);
    esp32p4_init_flash(s);
    for (i = 0; i < ARRAY_SIZE(esp32p4_unimplemented); i++) {
        esp32p4_map(esp32p4_unimplemented_device(
                        esp32p4_unimplemented[i].name,
                        esp32p4_unimplemented[i].size),
                    esp32p4_unimplemented[i].base);
    }
    qemu_log_mask(LOG_UNIMP, "esp32p4: the LP ROM's contents are not "
                  "published, so it is an unimplemented device\n");

    esp32p4_load(s);
}

static void esp32p4_get_flash_size(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    visit_type_size(v, name, &ESP32P4_MACHINE(obj)->flash_size, errp);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_set_flash_size(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint64_t size;

    if (!visit_type_size(v, name, &size, errp)) {
        return;
    }
    /* Zero is a board with no flash. */
    if (size && (!is_power_of_2(size) || size < ESP32P4_FLASH_MIN_SIZE ||
                 size > ESP32P4_FLASH_MAX_SIZE)) {
        error_setg(errp, "flash-size must be a power of two from %u MiB to "
                   "%u MiB", (unsigned)(ESP32P4_FLASH_MIN_SIZE / MiB),
                   (unsigned)(ESP32P4_FLASH_MAX_SIZE / MiB));
        return;
    }
    ESP32P4_MACHINE(obj)->flash_size = size;
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    object_class_property_add(oc, "flash-size", "size",
                              esp32p4_get_flash_size, esp32p4_set_flash_size,
                              NULL, NULL);
    object_class_property_set_description(oc, "flash-size",
        "Size of the board's off-package flash (no default: no flash)");

    mc->desc = "Espressif ESP32-P4NRW32X (2x HP RV32IMAFC, LP RV32IMAC)";
    mc->init = esp32p4_init;
    mc->default_ram_size = ESP32P4_L2MEM_SIZE;
    mc->min_cpus = ESP32P4_HP_CORES + 1;
    mc->max_cpus = ESP32P4_HP_CORES + 1;
    mc->default_cpus = ESP32P4_HP_CORES + 1;
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static const TypeInfo esp32p4_machine_types[] = {
    {
        .name          = TYPE_ESP32P4_MACHINE,
        .parent        = TYPE_MACHINE,
        .instance_size = sizeof(ESP32P4MachineState),
        .class_init    = esp32p4_machine_class_init,
    },
};

DEFINE_TYPES(esp32p4_machine_types)
