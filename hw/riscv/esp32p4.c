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
 *     at 0x50100000; LP SRAM (32 KiB) at 0x50108000. The L2 cache occupies
 *     the start of L2MEM, 128 KiB from reset (TRM 9.3.2's configurations;
 *     esp32p4_cache.c), which no master can then reach.
 *   - The external memory windows, 64 MiB each: flash at 0x40000000 and
 *     PSRAM at 0x48000000, both again at 0x8xxxxxxx for direct access,
 *     which the MMUs of the flash and PSRAM MSPI controllers
 *     (esp32p4_mspi.c, at 0x5008C000 to 0x5008FFFF) map onto the flash and
 *     the 32 MiB of in-package PSRAM, a RAM region of its own, in 64 KiB
 *     pages. The flash is a W25Q part on the flash controller's SSI bus.
 *   - The caches (esp32p4_cache.c, registers at 0x3FF10000): the HP cores
 *     reach the external memory through their L1 and the L2 cache, and
 *     L2MEM through their L1, at the 0x4xxxxxxx addresses; the cores'
 *     address space, of their own, has the caches' views there over the
 *     system's. Every other master, the LP core and DMA among them, and
 *     the cores at 0x8xxxxxxx, reach the memories directly.
 *   - The reset and clock control, HP_SYS_CLKRST, LP_AON_CLKRST and
 *     LPPERI, with the clock tree from the crystal, the RC oscillators and
 *     the PLLs to every block's clocks, and the analog I2C controller
 *     through which software configures the PLLs; and LP_SYS's
 *     BOOT_ADDR_HP_CORE1 and its record of refused accesses, the rest of
 *     whose registers are logged as unimplemented.
 *   - The eFuse controller and its array, on its clock and reset from
 *     LPPERI, its interrupt a source of the interrupt matrix.
 *   - The system timer, the two timer groups (each two timers, an MWDT
 *     and, in group 0, the RTC calibration), the RTC watchdog, the super
 *     watchdog and the RTC timer, on their clocks and resets; the
 *     watchdogs reset the HP cores, the HP core (the HP blocks and both
 *     cores, the LP system running on) or the whole system, each with its
 *     reset cause in LP_CLKRST.
 *   - Permission control (TRM chapter 20): the HP, LP and DMA access
 *     permission managers, through which every core (and DMA master)
 *     reaches the bus, each access carrying its master and privilege mode,
 *     with their registers HP_PERI_PMS, LP2HP_PERI_PMS, HP_DMA_PMS,
 *     LP_PERI_PMS and HP2LP_PERI_PMS; an access they refuse answers with a
 *     bus error and is recorded by the AXI ICM's registers (0x500A4000),
 *     raising SYS_ICM_INTR, or by LP_SYS's, raising LP_SYSREG_INTR.
 *   - Every other peripheral block the chip decodes, as an unimplemented
 *     device named after the block. Addresses the chip does not decode,
 *     the reserved ranges of TRM table 9.3-1 and the unassigned slots
 *     between blocks, answer with a bus error, which the cores take as an
 *     access fault.
 *   - Each HP core's CLIC and CLINT (TRM 2.8, 2.9): the CLIC registers at
 *     0x20800000 and their user-mode view at 0x20B00000, the CLINT at
 *     0x20000000, each core seeing its own at the base and the other
 *     core's 64 KiB above; the CLINT's counter counts CPU_CLK, each
 *     core's registers behind its CLIC bus clock, and each core's CLIC
 *     resets with the core.
 *   - The interrupt matrix (TRM chapter 13) at 0x500D6000, core 0's and
 *     core 1's 2 KiB above, behind its bus clock and reset: every
 *     peripheral interrupt source, as its source input, to the CLIC
 *     external interrupts 16 to 47 of either HP core, remapped as each
 *     core's mode and its CLIC's user-mode interrupts decide. Of HP_SYSTEM
 *     (0x500E5000), the cores' software interrupts CPU_INTR_FROM_CPU_0 to
 *     _3, sources of the matrix, the L1 caches' resets, the external
 *     memory encryption enables and the L2MEM error response and
 *     interrupt; the rest of its registers are logged as unimplemented. A
 *     source no modelled block drives stays low.
 *   - The GPIO matrix and IO MUX (TRM chapter 10), HP and LP, with
 *     GPIO_EXT's sigma-delta modulators, glitch filters and ETM, behind
 *     IOMUX_APB_CLK and the LP IO MUX's clock and their resets, timed by
 *     the IO MUX operating clocks: the 55 pads, their interrupts as
 *     interrupt sources 74 to 78 and 10, and the HP cores' fast GPIO as
 *     matrix signals 214 to 229. GPIO_STRAP latches the board's strapping
 *     at reset, the machine's strap-mode property, by default SPI boot:
 *
 *       -M esp32p4,strap-mode=0x08
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
 * L2MEM (past the L2 cache), the TCM or LP SRAM, and core 0 starts at the
 * ELF's entry point rather than in the ROM, with the caches empty; core 1
 * is still held for software to release.
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
    /* HP CPU peripherals. */
    { "trace0",           0x3ff04000, 4 * KiB },
    { "trace1",           0x3ff05000, 4 * KiB },
    { "bus-monitor",      0x3ff06000, 4 * KiB },
    { "l2mem-monitor",    0x3ff0e000, 4 * KiB },
    { "tcm-monitor",      0x3ff0f000, 4 * KiB },

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
    /* The AXI ICM's QoS command interface, above its own registers. */
    { "axi-icm-qos",      0x500a4400, 3 * KiB },
    { "h264-dma",         0x500a7000, 4 * KiB },

    /* HP peripherals 1. */
    { "mcpwm0",           0x500c0000, 4 * KiB },
    { "mcpwm1",           0x500c1000, 4 * KiB },
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
    { "twai0",            0x500d7000, 4 * KiB },
    { "twai1",            0x500d8000, 4 * KiB },
    { "twai2",            0x500d9000, 4 * KiB },
    { "i3c-mst",          0x500da000, 4 * KiB },
    { "i3c-slv",          0x500db000, 4 * KiB },
    { "lcd-cam",          0x500dc000, 4 * KiB },
    { "adc",              0x500de000, 4 * KiB },
    { "uhci",             0x500df000, 4 * KiB },
    { "iomux-mspi-pin",   0x500e1200, 4 * KiB - 0x200 },
    { "mem-monitor",      0x500e3000, 4 * KiB },
    { "audio-addc",       0x500e4000, 4 * KiB },

    /* The LP ROM, whose contents Espressif does not publish. */
    { "lp-rom",           ESP32P4_LP_ROM_BASE, ESP32P4_LP_ROM_SIZE },

    /* The MIPI bridges' memories. */
    { "mipi-csi-mem",     0x50104000, 4 * KiB },
    { "mipi-dsi-mem",     0x50105000, 4 * KiB },

    /* LP always-on peripherals. */
    { "lp-analog-peri",   0x50113000, 4 * KiB },
    { "lp-huk",           0x50114000, 4 * KiB },
    { "pmu",              0x50115000, 4 * KiB },
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
    { "lp-intr",          0x5012c000, 4 * KiB },
    { "lp-tsens",         0x5012f000, 4 * KiB },
};

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
 * The flash chip on the flash MSPI controller's bus, chip select 0, which
 * SPI1 drives with software's commands and SPI0 with the cache's reads. It
 * is erased unless backed by the first -drive if=mtd.
 *
 * A board with no flash has nothing on the bus. Nothing then drives the
 * controllers' data inputs, which out of reset have no pull-up or
 * pull-down (IOMUX_MSPI_PIN_FLASH_*_WPU and _WPD clear), so this model
 * takes what they read as zero, as a QEMU SSI bus with no device returns.
 */
/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_init_flash(ESP32P4MachineState *s)
{
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    DeviceState *flash;

    s->flash_bus = (SSIBus *)qdev_get_child_bus(DEVICE(&s->flash_mspi),
                                                "ssi");
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
    qdev_connect_gpio_out_named(DEVICE(&s->flash_mspi), SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash, SSI_GPIO_CS,
                                                       0));
}

/*
 * The bus as every master sees it, through the access permission managers
 * (hw/misc/esp32p4_pms.c): the cores take it as their memory as they are
 * realized, before the blocks the APMs record refusals in exist. The HP
 * cores take it with the caches' views over it at 0x4xxxxxxx
 * (esp32p4_init_ext_memory()): their cached accesses are for their PMP
 * alone.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_init_bus(ESP32P4MachineState *s)
{
    object_initialize_child(OBJECT(s), "pms", &s->pms, TYPE_ESP32P4_PMS);
    memory_region_init(&s->hp_mem, OBJECT(s), "esp32p4.hp-mem", UINT64_MAX);
    memory_region_init_alias(&s->hp_bus, OBJECT(s), "esp32p4.hp-bus",
                             esp32p4_pms_bus(&s->pms), 0, UINT64_MAX);
    memory_region_add_subregion_overlap(&s->hp_mem, 0, &s->hp_bus, 0);
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
        object_property_set_link(OBJECT(&s->hp_cpu[i]), "memory",
                                 OBJECT(&s->hp_mem), &error_abort);
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
    object_property_set_link(OBJECT(&s->lp_cpu), "memory",
                             OBJECT(esp32p4_pms_bus(&s->pms)), &error_abort);
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

        /* A core's reset also resets its CLIC. */
        object_initialize_child(OBJECT(s), "core-reset-split[*]",
                                &s->core_reset_split[i], TYPE_SPLIT_IRQ);
        qdev_prop_set_uint16(DEVICE(&s->core_reset_split[i]), "num-lines",
                             2);
        qdev_realize(DEVICE(&s->core_reset_split[i]), NULL, &error_fatal);
        qdev_connect_gpio_out(DEVICE(&s->core_reset_split[i]), 0,
                              qdev_get_gpio_in_named(cpu, ESP32P4_CPU_RESET,
                                                     0));
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_CORE_RESET, i,
                                    qdev_get_gpio_in(DEVICE(
                                        &s->core_reset_split[i]), 0));
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
 * The HP cores' CLINT and CLICs: the CLINT's software and timer interrupts
 * go to each core's CLIC interrupts 3 and 7, and its counter counts
 * CPU_CLK, which the TRM does not name; each core's CLIC and CLINT
 * registers are behind its CLIC bus clock, and its CLIC resets with the
 * core.
 */
/* [spec:nuos:req:emu.esp32p4.clic] */
static void esp32p4_init_core_local(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *clint, *clic;
    int i;

    object_initialize_child(OBJECT(s), "clint", &s->clint,
                            TYPE_ESP32P4_CLINT);
    object_initialize_child(OBJECT(s), "clic", &s->clic, TYPE_ESP32P4_CLIC);
    clint = DEVICE(&s->clint);
    clic = DEVICE(&s->clic);

    qdev_connect_clock_in(clint, "mtime", qdev_get_clock_out(clkrst, "cpu"));
    for (i = 0; i < ESP32P4_HP_CORES; i++) {
        g_autofree char *link = g_strdup_printf("cpu%d", i);
        g_autofree char *in = g_strdup_printf("core%d", i);
        g_autofree char *bus = g_strdup_printf("core%d-clic", i);

        object_property_set_link(OBJECT(clint), link, OBJECT(&s->hp_cpu[i]),
                                 &error_abort);
        object_property_set_link(OBJECT(clic), link, OBJECT(&s->hp_cpu[i]),
                                 &error_abort);
        qdev_connect_clock_in(clint, in, qdev_get_clock_out(clkrst, bus));
        qdev_connect_clock_in(clic, in, qdev_get_clock_out(clkrst, bus));
    }
    sysbus_realize(SYS_BUS_DEVICE(clint), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(clic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(clint), 0, ESP32P4_CLINT_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(clic), ESP32P4_CLIC_MMIO_M,
                    ESP32P4_CLIC_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(clic), ESP32P4_CLIC_MMIO_U,
                    ESP32P4_CLIC_U_BASE);

    for (i = 0; i < ESP32P4_HP_CORES; i++) {
        qdev_connect_gpio_out_named(clint, ESP32P4_CLINT_MSIP, i,
            qdev_get_gpio_in_named(clic, ESP32P4_CLIC_IRQ,
                ESP32P4_CLIC_IRQ_LINE(i, ESP32P4_CLIC_IRQ_MSIP)));
        qdev_connect_gpio_out_named(clint, ESP32P4_CLINT_MTIP, i,
            qdev_get_gpio_in_named(clic, ESP32P4_CLIC_IRQ,
                ESP32P4_CLIC_IRQ_LINE(i, ESP32P4_CLIC_IRQ_MTIP)));
        qdev_connect_gpio_out(DEVICE(&s->core_reset_split[i]), 1,
                              qdev_get_gpio_in_named(clic,
                                  ESP32P4_CLIC_CORE_RESET, i));
    }
}

/*
 * The interrupt matrix's input for interrupt source @source, an
 * ESP32P4_INTR_* number, to which the machine connects the interrupt
 * output of the block that raises it (hw/intc/esp32p4_intmatrix.h).
 */
static qemu_irq esp32p4_intr_source(ESP32P4MachineState *s, int source)
{
    return qdev_get_gpio_in_named(DEVICE(&s->intmatrix),
                                  ESP32P4_INTMATRIX_SOURCE, source);
}

/*
 * The interrupt matrix, its registers behind INTRMTX_APB_CLK and its
 * reset, between the sources and the CLIC external interrupts of both HP
 * cores; for its remapping, each core's machine mode and which of its
 * CLIC's interrupts are user-mode ones. And HP_SYSTEM, behind
 * SYSREG_APB_CLK, whose software interrupts and HP_SYSREG interrupt are
 * among its sources.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_intmatrix(ESP32P4MachineState *s)
{
    DeviceState *clic = DEVICE(&s->clic);
    DeviceState *intmatrix, *hp_sys;
    int core, n;

    object_initialize_child(OBJECT(s), "intmatrix", &s->intmatrix,
                            TYPE_ESP32P4_INTMATRIX);
    intmatrix = DEVICE(&s->intmatrix);
    sysbus_realize(SYS_BUS_DEVICE(intmatrix), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(intmatrix), 0,
                             ESP32P4_INTMATRIX_BASE, ESP32P4_CLK_INTRMTX_APB,
                             ESP32P4_RST_INTRMTX, 0);
    for (core = 0; core < ESP32P4_HP_CORES; core++) {
        qdev_connect_gpio_out_named(DEVICE(&s->hp_cpu[core]),
                                    ESP32P4_CPU_M_MODE, 0,
                                    qdev_get_gpio_in_named(intmatrix,
                                        ESP32P4_INTMATRIX_CORE_M_MODE,
                                        core));
        for (n = 0; n < ESP32P4_INTMATRIX_LINES; n++) {
            int line = core * ESP32P4_INTMATRIX_LINES + n;
            int id = ESP32P4_CLIC_IRQ_LINE(core, ESP32P4_CLIC_IRQ_EXT0 + n);

            qdev_connect_gpio_out_named(intmatrix, ESP32P4_INTMATRIX_IRQ,
                                        line,
                                        qdev_get_gpio_in_named(clic,
                                            ESP32P4_CLIC_IRQ, id));
            qdev_connect_gpio_out_named(clic, ESP32P4_CLIC_USER, id,
                                        qdev_get_gpio_in_named(intmatrix,
                                            ESP32P4_INTMATRIX_LINE_USER,
                                            line));
        }
    }

    object_initialize_child(OBJECT(s), "hp-sys", &s->hp_sys,
                            TYPE_ESP32P4_HP_SYS);
    hp_sys = DEVICE(&s->hp_sys);
    sysbus_realize(SYS_BUS_DEVICE(hp_sys), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(hp_sys), 0,
                             ESP32P4_HP_SYS_BASE, ESP32P4_CLK_SYSREG_APB,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    for (n = 0; n < ESP32P4_HP_SYS_CPU_INTRS; n++) {
        qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_CPU_INTR, n,
                                    esp32p4_intr_source(s,
                                        ESP32P4_INTR_FROM_CPU_INTR0 + n));
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(hp_sys), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_HP_SYSREG));
}

/*
 * The eFuse controller, on its clock and reset from LPPERI, its registers
 * behind its bus clock. Its interrupt is LP_EFUSE_INTR, source 17 of the
 * interrupt matrix.
 */
/* [spec:nuos:req:emu.esp32p4.efuse] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_efuse(ESP32P4MachineState *s)
{
    DeviceState *efuse, *clkrst = DEVICE(&s->clkrst);

    object_initialize_child(OBJECT(s), "efuse", &s->efuse,
                            TYPE_ESP32P4_EFUSE);
    efuse = DEVICE(&s->efuse);
    qdev_connect_clock_in(efuse, "clk", qdev_get_clock_out(clkrst, "efuse"));
    sysbus_realize(SYS_BUS_DEVICE(efuse), &error_fatal);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_LP_EFUSE,
                                qdev_get_gpio_in_named(efuse,
                                    ESP32P4_EFUSE_RESET, 0));
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(efuse), 0,
                             ESP32P4_EFUSE_BASE, ESP32P4_CLK_LP_EFUSE,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(efuse), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_EFUSE));
}

/*
 * The HP core reset of the HP blocks with no reset line of their own,
 * HP_SYSTEM's registers, the CLINT and the AXI ICM; the rest take theirs
 * from the reset and clock control, which holds every HP block reset line
 * and the HP cores through it. (The HP and DMA access permission managers
 * have no reset of their own in the PMS model, which resets them only with
 * the system.)
 */
/* [spec:nuos:req:emu.esp32p4.timers] */
static void esp32p4_hp_core_reset(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;

    if (level) {
        device_cold_reset(DEVICE(&s->hp_sys));
        device_cold_reset(DEVICE(&s->clint));
        device_cold_reset(DEVICE(&s->icm));
    }
}

/*
 * The timers and watchdogs (TRM chapters 16 to 19), each on its clocks and
 * reset from the reset and clock control and its registers behind its bus
 * clock: the system timer, counting SYSTIMER_CLK and stopping with the HP
 * cores' stalls; the two timer groups, their timers on TIMERGRPn_Tx_CLK,
 * their MWDTs on TIMERGRPn_WDT_CLK resetting the HP cores (a CPU reset) or
 * the HP core through the reset and clock control, and timer group 0's
 * RTC calibration measuring TIMERGRP0_TGRT_CLK against XTAL_CLK; the RTC
 * watchdog on LP_DYN_SLOW_CLK and the super watchdog on the RC_SLOW
 * oscillator, resetting the cores, the HP core or the system; and the RTC
 * timer on LP_DYN_SLOW_CLK. The LP blocks, in the always-on domain, have
 * no bus clock gate: their registers are behind LP_DYN_FAST_CLK and their
 * LP_CLKRST resets.
 *
 * Their interrupts are sources of the interrupt matrix. Their ETM events
 * and tasks, the RTC timer's wakeups and the LP core's timer interrupt
 * are named GPIO lines (see the devices' headers) that nothing drives or
 * takes until the ETM, the PMU and the LP core's interrupts are modelled.
 * MWDT0 and the RTC watchdog run their flash boot protection when the
 * chip boots from flash through the boot ROM in SPI boot mode: when the
 * board's strapping (the strap-mode property) has GPIO35 high and there
 * is no -kernel, which loads the program in the ROM's place, with no flash
 * boot to protect. Pads driven from outside at reset, which GPIO_STRAP
 * latches in place of the strapping, and LP_SYSTEM_FORCE_DOWNLOAD_BOOT do
 * not change it. LP_SYSTEM's FIB control of the super watchdog's reset is
 * not modelled: the "swd-reset-en" input stays low, as the FIB's analog
 * default leaves it.
 */
/* [spec:nuos:req:emu.esp32p4.timers] */
static void esp32p4_init_timers(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *dev, *split;
    bool spi_boot = !MACHINE(s)->kernel_filename &&
                    (s->strap_mode & ESP32P4_GPIO_STRAP_SPI_BOOT);
    int i, n;

    for (n = 0; n < ESP32P4_HP_CORES; n++) {
        object_initialize_child(OBJECT(s), "core-stall-split[*]",
                                &s->core_stall_split[n], TYPE_SPLIT_IRQ);
        qdev_prop_set_uint16(DEVICE(&s->core_stall_split[n]), "num-lines",
                             2);
        qdev_realize(DEVICE(&s->core_stall_split[n]), NULL, &error_fatal);
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_CORE_STALL, n,
                                    qdev_get_gpio_in(
                                        DEVICE(&s->core_stall_split[n]), 0));
    }

    object_initialize_child(OBJECT(s), "systimer", &s->systimer,
                            TYPE_ESP32P4_SYSTIMER);
    dev = DEVICE(&s->systimer);
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(clkrst, "systimer"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_SYSTIMER_BASE, ESP32P4_CLK_SYSTIMER_APB,
                             ESP32P4_RST_STIMER, 0);
    for (i = 0; i < ESP32P4_SYSTIMER_COMPS; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
                           esp32p4_intr_source(s,
                               ESP32P4_INTR_SYSTIMER_TARGET0 + i));
    }
    for (n = 0; n < ESP32P4_HP_CORES; n++) {
        qdev_connect_gpio_out(DEVICE(&s->core_stall_split[n]), 0,
                              qdev_get_gpio_in_named(dev,
                                  ESP32P4_SYSTIMER_CORE_STALL, n));
    }

    for (i = 0; i < 2; i++) {
        static const hwaddr base[] = { ESP32P4_TIMG0_BASE,
                                       ESP32P4_TIMG1_BASE };
        static const unsigned bus[] = { ESP32P4_CLK_TIMERGRP0_APB,
                                        ESP32P4_CLK_TIMERGRP1_APB };
        static const unsigned rst[] = { ESP32P4_RST_TIMERGRP0,
                                        ESP32P4_RST_TIMERGRP1 };
        static const int intr[] = { ESP32P4_INTR_TG0_T0,
                                    ESP32P4_INTR_TG1_T0 };
        static const char *const clk[] = { "t0", "t1", "wdt" };
        int k;

        object_initialize_child(OBJECT(s), "timg[*]", &s->timg[i],
                                TYPE_ESP32P4_TIMG);
        dev = DEVICE(&s->timg[i]);
        qdev_prop_set_uint32(dev, "group", i);
        object_property_set_link(OBJECT(dev), "clkrst", OBJECT(&s->clkrst),
                                 &error_abort);
        for (k = 0; k < ARRAY_SIZE(clk); k++) {
            g_autofree char *out = g_strdup_printf("timergrp%d-%s", i,
                                                   clk[k]);

            qdev_connect_clock_in(dev, clk[k],
                                  qdev_get_clock_out(clkrst, out));
        }
        qdev_connect_clock_in(dev, "xtal", qdev_get_clock_out(clkrst,
                                                              "xtal"));
        if (i == 0) {
            qdev_connect_clock_in(dev, "cal",
                                  qdev_get_clock_out(clkrst,
                                                     "timergrp0-tgrt"));
        }
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 base[i], bus[i], rst[i], 0);
        for (k = 0; k <= ESP32P4_TIMG_TIMERS; k++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), k,
                               esp32p4_intr_source(s, intr[i] + k));
        }
        for (n = 0; n < ESP32P4_HP_CORES; n++) {
            qdev_connect_gpio_out_named(dev, ESP32P4_TIMG_CPU_RESET, n,
                                        qdev_get_gpio_in_named(clkrst,
                                            ESP32P4_CLKRST_MWDT_CORE_RESET,
                                            ESP32P4_CLKRST_MWDT_LINE(i, n)));
        }
    }
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->timg[0]),
                                        ESP32P4_TIMG_SPI_BOOT, 0), spi_boot);

    object_initialize_child(OBJECT(s), "lp-wdt", &s->lp_wdt,
                            TYPE_ESP32P4_LP_WDT);
    dev = DEVICE(&s->lp_wdt);
    object_property_set_link(OBJECT(dev), "clkrst", OBJECT(&s->clkrst),
                             &error_abort);
    object_property_set_link(OBJECT(dev), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    qdev_connect_clock_in(dev, "slow", qdev_get_clock_out(clkrst,
                                                          "lp-dyn-slow"));
    qdev_connect_clock_in(dev, "swd", qdev_get_clock_out(clkrst,
                                                         "osc-rc-slow"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_LP_WDT_BASE, ESP32P4_CLK_LP_DYN_FAST,
                             ESP32P4_RST_LP_WDT, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_WDT));
    for (n = 0; n < ESP32P4_HP_CORES; n++) {
        qdev_connect_gpio_out_named(dev, ESP32P4_LP_WDT_CPU_RESET, n,
                                    qdev_get_gpio_in_named(clkrst,
                                        ESP32P4_CLKRST_RWDT_CORE_RESET, n));
    }
    qemu_set_irq(qdev_get_gpio_in_named(dev, ESP32P4_LP_WDT_SPI_BOOT, 0),
                 spi_boot);

    object_initialize_child(OBJECT(s), "lp-timer", &s->lp_timer,
                            TYPE_ESP32P4_LP_TIMER);
    dev = DEVICE(&s->lp_timer);
    object_property_set_link(OBJECT(dev), "clkrst", OBJECT(&s->clkrst),
                             &error_abort);
    qdev_connect_clock_in(dev, "slow", qdev_get_clock_out(clkrst,
                                                          "lp-dyn-slow"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_LP_TIMER_BASE, ESP32P4_CLK_LP_DYN_FAST,
                             ESP32P4_RST_LP_TIMER, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_TIMER_REG_0));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_TIMER_REG_1));
    for (n = 0; n < ESP32P4_HP_CORES; n++) {
        qdev_connect_gpio_out(DEVICE(&s->core_stall_split[n]), 1,
                              qdev_get_gpio_in_named(dev,
                                  ESP32P4_LP_TIMER_CORE_STALL, n));
    }

    object_initialize_child(OBJECT(s), "hp-core-reset-split",
                            &s->hp_core_reset_split, TYPE_SPLIT_IRQ);
    split = DEVICE(&s->hp_core_reset_split);
    qdev_prop_set_uint16(split, "num-lines", 2);
    qdev_realize(split, NULL, &error_fatal);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_HP_CORE_RESET, 0,
                                qdev_get_gpio_in(split, 0));
    qdev_connect_gpio_out(split, 0,
                          qemu_allocate_irq(esp32p4_hp_core_reset, s, 0));
    qdev_connect_gpio_out(split, 1,
                          qdev_get_gpio_in_named(dev,
                              ESP32P4_LP_TIMER_HP_SYS_RESET, 0));
}

/*
 * One of the MSPI controllers: its SPI0 and SPI1 registers behind its bus
 * clock, its 64 MiB window at its direct address and, for the bus's other
 * masters, at its cached one, its clocks, resets and interrupt, and the
 * encryption enables from HP_SYSTEM (through `crypto_split` output
 * `psram`).
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_init_mspi(ESP32P4MachineState *s, ESP32P4MSPIState *m,
                              bool psram)
{
    DeviceState *dev, *clkrst = DEVICE(&s->clkrst);
    SysBusDevice *sbd;
    MemoryRegion *window, *alias = psram ? &s->psram_window_alias
                                         : &s->flash_window_alias;
    hwaddr regs = psram ? ESP32P4_PSRAM_MSPI_BASE : ESP32P4_FLASH_MSPI_BASE;
    int n;

    object_initialize_child(OBJECT(s), psram ? "psram-mspi" : "flash-mspi",
                            m, TYPE_ESP32P4_MSPI);
    dev = DEVICE(m);
    sbd = SYS_BUS_DEVICE(m);
    qdev_prop_set_bit(dev, "psram", psram);
    object_property_set_link(OBJECT(m), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    if (psram) {
        object_property_set_link(OBJECT(m), "psram-mr", OBJECT(&s->psram),
                                 &error_abort);
    }
    qdev_connect_clock_in(dev, "sys", qdev_get_clock_out(clkrst,
                          psram ? "psram-sys" : "flash-sys"));
    qdev_connect_clock_in(dev, "core", qdev_get_clock_out(clkrst,
                          psram ? "psram-core" : "flash-core"));
    sysbus_realize(sbd, &error_fatal);

    esp32p4_clkrst_map_gated(&s->clkrst, sbd, ESP32P4_MSPI_MMIO_SPI0, regs,
                             psram ? ESP32P4_CLK_PSRAM_SYS
                                   : ESP32P4_CLK_FLASH_SYS,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    esp32p4_clkrst_map_gated(&s->clkrst, sbd, ESP32P4_MSPI_MMIO_SPI1,
                             regs + ESP32P4_MSPI_REGS_SIZE,
                             psram ? ESP32P4_CLK_PSRAM_SYS
                                   : ESP32P4_CLK_FLASH_SYS,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    window = sysbus_mmio_get_region(sbd, ESP32P4_MSPI_MMIO_WINDOW);
    memory_region_add_subregion(get_system_memory(),
                                psram ? ESP32P4_PSRAM_WINDOW_DIRECT_BASE
                                      : ESP32P4_FLASH_WINDOW_DIRECT_BASE,
                                window);
    memory_region_init_alias(alias, OBJECT(s),
                             psram ? "esp32p4.psram-window-bus"
                                   : "esp32p4.flash-window-bus",
                             window, 0, ESP32P4_EXT_WINDOW_SIZE);
    memory_region_add_subregion(get_system_memory(),
                                psram ? ESP32P4_PSRAM_WINDOW_BASE
                                      : ESP32P4_FLASH_WINDOW_BASE, alias);

    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                psram ? ESP32P4_RST_DUAL_MSPI_APB
                                      : ESP32P4_RST_MSPI_APB,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_MSPI_APB_RESET, 0));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                psram ? ESP32P4_RST_DUAL_MSPI_AXI
                                      : ESP32P4_RST_MSPI_AXI,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_MSPI_AXI_RESET, 0));
    for (n = 0; n < ESP32P4_HP_SYS_CRYPTO_CTRL_LINES; n++) {
        qdev_connect_gpio_out(DEVICE(&s->crypto_split[n]), psram,
                              qdev_get_gpio_in_named(dev,
                                  ESP32P4_MSPI_CRYPTO_CTRL, n));
    }
    sysbus_connect_irq(sbd, 0, esp32p4_intr_source(s,
                       psram ? ESP32P4_INTR_PSRAM_MSPI : ESP32P4_INTR_MSPI));
}

/*
 * The flash and PSRAM MSPI controllers and the flash, and the caches:
 * their registers behind L1CACHE_CPU_CLK and RST_EN_HP_CACHE, the part of
 * L2MEM the L2 cache occupies over L2MEM at both its addresses, and their
 * views in the HP cores' address space.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void esp32p4_init_ext_memory(ESP32P4MachineState *s)
{
    DeviceState *cache, *clkrst = DEVICE(&s->clkrst);
    SysBusDevice *sbd;
    static const struct {
        hwaddr base;
        int region;
    } views[] = {
        { ESP32P4_FLASH_WINDOW_BASE,
          ESP32P4_CACHE_MMIO_VIEW0 + ESP32P4_CACHE_VIEW_FLASH },
        { ESP32P4_PSRAM_WINDOW_BASE,
          ESP32P4_CACHE_MMIO_VIEW0 + ESP32P4_CACHE_VIEW_PSRAM },
        { ESP32P4_L2MEM_BASE,
          ESP32P4_CACHE_MMIO_VIEW0 + ESP32P4_CACHE_VIEW_L2MEM },
    };
    static const int cache_resets[ESP32P4_CACHE_NRESETS] = {
        [ESP32P4_CACHE_RESET_L1_I0] = ESP32P4_RST_L1_I0_CACHE,
        [ESP32P4_CACHE_RESET_L1_I1] = ESP32P4_RST_L1_I1_CACHE,
        [ESP32P4_CACHE_RESET_L1_D] = ESP32P4_RST_L1_D_CACHE,
        [ESP32P4_CACHE_RESET_L2] = ESP32P4_RST_L2_CACHE,
    };
    int i;

    /* HP_SYSTEM_CRYPTO_CTRL's enables go to both controllers. */
    for (i = 0; i < ESP32P4_HP_SYS_CRYPTO_CTRL_LINES; i++) {
        object_initialize_child(OBJECT(s), "crypto-ctrl-split[*]",
                                &s->crypto_split[i], TYPE_SPLIT_IRQ);
        qdev_prop_set_uint16(DEVICE(&s->crypto_split[i]), "num-lines", 2);
        qdev_realize(DEVICE(&s->crypto_split[i]), NULL, &error_fatal);
        qdev_connect_gpio_out_named(DEVICE(&s->hp_sys),
                                    ESP32P4_HP_SYS_CRYPTO_CTRL, i,
                                    qdev_get_gpio_in(
                                        DEVICE(&s->crypto_split[i]), 0));
    }
    esp32p4_init_mspi(s, &s->flash_mspi, false);
    esp32p4_init_mspi(s, &s->psram_mspi, true);
    esp32p4_init_flash(s);

    object_initialize_child(OBJECT(s), "cache", &s->cache,
                            TYPE_ESP32P4_CACHE);
    cache = DEVICE(&s->cache);
    sbd = SYS_BUS_DEVICE(cache);
    object_property_set_link(OBJECT(cache), "flash-mspi",
                             OBJECT(&s->flash_mspi), &error_abort);
    object_property_set_link(OBJECT(cache), "psram-mspi",
                             OBJECT(&s->psram_mspi), &error_abort);
    object_property_set_link(OBJECT(cache), "l2mem", OBJECT(&s->l2mem),
                             &error_abort);
    object_property_set_link(OBJECT(cache), "hp-sys", OBJECT(&s->hp_sys),
                             &error_abort);
    sysbus_realize(sbd, &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, sbd, ESP32P4_CACHE_MMIO_REGS,
                             ESP32P4_CACHE_BASE, ESP32P4_CLK_L1CACHE_CPU,
                             ESP32P4_RST_HP_CACHE, 0);
    sysbus_mmio_map_overlap(sbd, ESP32P4_CACHE_MMIO_L2MEM_CACHED,
                            ESP32P4_L2MEM_BASE, 1);
    sysbus_mmio_map_overlap(sbd, ESP32P4_CACHE_MMIO_L2MEM_DIRECT,
                            ESP32P4_L2MEM_DIRECT_BASE, 1);
    for (i = 0; i < ARRAY_SIZE(views); i++) {
        memory_region_add_subregion_overlap(&s->hp_mem, views[i].base,
            sysbus_mmio_get_region(sbd, views[i].region), 1);
    }
    for (i = 0; i < ESP32P4_CACHE_NRESETS; i++) {
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                    cache_resets[i],
                                    qdev_get_gpio_in_named(cache,
                                        ESP32P4_CACHE_CACHE_RESET, i));
    }
    for (i = 0; i < ESP32P4_HP_SYS_CACHE_RESET_LINES; i++) {
        qdev_connect_gpio_out_named(DEVICE(&s->hp_sys),
                                    ESP32P4_HP_SYS_CACHE_RESET, i,
                                    qdev_get_gpio_in_named(cache,
                                        ESP32P4_CACHE_SYS_CACHE_RESET, i));
    }
    qdev_connect_gpio_out_named(DEVICE(&s->hp_sys),
                                ESP32P4_HP_SYS_L2MEM_ERR_RESP, 0,
                                qdev_get_gpio_in_named(cache,
                                    ESP32P4_CACHE_L2MEM_ERR_RESP, 0));
    sysbus_connect_irq(sbd, 0, esp32p4_intr_source(s, ESP32P4_INTR_CACHE));
}

/*
 * The GPIO matrices and IO MUXes, HP and LP, their registers behind
 * IOMUX_APB_CLK and the LP IO MUX's clock, their resets taken directly
 * (the strapping latch survives them); their interrupts, sources 74 to 78
 * and 10 of the interrupt matrix; and each HP core's fast GPIO, matrix
 * signals 214 + 8 * core + n. The cores' cpu_gpio_oen is the outputs'
 * enable inverted: ESP-IDF's dedic_gpio_cpu_ll.h for the P4 enables an
 * output by clearing its bit ("the OEN register is active low"), where
 * the TRM's register description has 1 enable it.
 */
/* [spec:nuos:req:emu.esp32p4.gpio] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_gpio(ESP32P4MachineState *s)
{
    static const struct {
        hwaddr base;
        unsigned clock;
    } map[] = {
        [ESP32P4_GPIO_MMIO_GPIO] = { ESP32P4_GPIO_BASE,
                                     ESP32P4_CLK_IOMUX_APB },
        [ESP32P4_GPIO_MMIO_IO_MUX] = { ESP32P4_IO_MUX_BASE,
                                       ESP32P4_CLK_IOMUX_APB },
        [ESP32P4_GPIO_MMIO_LP_GPIO] = { ESP32P4_LP_GPIO_BASE,
                                        ESP32P4_CLK_LP_IOMUX },
        [ESP32P4_GPIO_MMIO_LP_IOMUX] = { ESP32P4_LP_IOMUX_BASE,
                                         ESP32P4_CLK_LP_IOMUX },
    };
    DeviceState *gpio, *clkrst = DEVICE(&s->clkrst);
    int i, core;

    object_initialize_child(OBJECT(s), "gpio", &s->gpio, TYPE_ESP32P4_GPIO);
    gpio = DEVICE(&s->gpio);
    qdev_prop_set_uint32(gpio, "strap-mode", s->strap_mode);
    qdev_connect_clock_in(gpio, "iomux", qdev_get_clock_out(clkrst, "iomux"));
    qdev_connect_clock_in(gpio, "lp", qdev_get_clock_out(clkrst, "lp-iomux"));
    sysbus_realize(SYS_BUS_DEVICE(gpio), &error_fatal);
    for (i = 0; i < ARRAY_SIZE(map); i++) {
        esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(gpio), i,
                                 map[i].base, map[i].clock,
                                 ESP32P4_CLKRST_GATE_NO_RESET, 0);
    }
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_IOMUX,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_RESET,
                                    ESP32P4_GPIO_RESET_HP));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_LP_IOMUX,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_RESET,
                                    ESP32P4_GPIO_RESET_LP));
    for (i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(gpio), ESP32P4_GPIO_IRQ_INTR0 + i,
                           esp32p4_intr_source(s, ESP32P4_INTR_GPIO_INTR0 + i));
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(gpio), ESP32P4_GPIO_IRQ_PAD_COMP,
                       esp32p4_intr_source(s, ESP32P4_INTR_GPIO_PAD_COMP));
    sysbus_connect_irq(SYS_BUS_DEVICE(gpio), ESP32P4_GPIO_IRQ_LP,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_GPIO));

    for (core = 0; core < ESP32P4_HP_CORES; core++) {
        DeviceState *cpu = DEVICE(&s->hp_cpu[core]);

        for (i = 0; i < ESP32P4_CPU_GPIOS; i++) {
            int n = core * ESP32P4_CPU_GPIOS + i;

            qdev_connect_gpio_out_named(cpu, ESP32P4_CPU_GPIO_OUT, i,
                qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                    ESP32P4_SIG_CORE_GPIO_OUT_PAD_OUT0 + n));
            qdev_connect_gpio_out_named(cpu, ESP32P4_CPU_GPIO_OE, i,
                qemu_irq_invert(qdev_get_gpio_in_named(gpio,
                    ESP32P4_GPIO_SIG_OE,
                    ESP32P4_SIG_CORE_GPIO_OUT_PAD_OUT0 + n)));
            qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                ESP32P4_SIG_CORE_GPIO_IN_PAD_IN0 + n,
                qdev_get_gpio_in_named(cpu, ESP32P4_CPU_GPIO_IN, i));
        }
    }
}

/*
 * The access permission managers and the blocks that record what they
 * refuse: the AXI ICM, its registers behind ICM_APB_CLK, whose interrupt
 * is SYS_ICM_INTR (source 21), and LP_SYS, whose interrupt is
 * LP_SYSREG_INTR (source 19). The LP APM's registers, LP_PERI_PMS and
 * HP2LP_PERI_PMS, are behind LPPERI's LP_PMS clock and reset; the HP and
 * DMA APMs' have neither clock gate nor reset in HP_SYS_CLKRST, and reset
 * with the system.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_pms(ESP32P4MachineState *s)
{
    SysBusDevice *pms = SYS_BUS_DEVICE(&s->pms);
    SysBusDevice *icm;

    object_initialize_child(OBJECT(s), "icm", &s->icm, TYPE_ESP32P4_ICM);
    icm = SYS_BUS_DEVICE(&s->icm);
    sysbus_realize(icm, &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, icm, 0, ESP32P4_ICM_BASE,
                             ESP32P4_CLK_ICM_APB,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    sysbus_connect_irq(icm, 0, esp32p4_intr_source(s, ESP32P4_INTR_SYS_ICM));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lp_sys), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_SYSREG));

    object_property_set_link(OBJECT(pms), "icm", OBJECT(icm), &error_abort);
    object_property_set_link(OBJECT(pms), "lp-sys", OBJECT(&s->lp_sys),
                             &error_abort);
    sysbus_realize(pms, &error_fatal);
    sysbus_mmio_map(pms, ESP32P4_PMS_MMIO_HP_PERI, ESP32P4_HP_PERI_PMS_BASE);
    sysbus_mmio_map(pms, ESP32P4_PMS_MMIO_LP2HP_PERI,
                    ESP32P4_LP2HP_PERI_PMS_BASE);
    sysbus_mmio_map(pms, ESP32P4_PMS_MMIO_DMA, ESP32P4_DMA_PMS_BASE);
    esp32p4_clkrst_map_gated(&s->clkrst, pms, ESP32P4_PMS_MMIO_LP_PERI,
                             ESP32P4_LP_PERI_PMS_BASE, ESP32P4_CLK_LP_PMS,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    esp32p4_clkrst_map_gated(&s->clkrst, pms, ESP32P4_PMS_MMIO_HP2LP_PERI,
                             ESP32P4_HP2LP_PERI_PMS_BASE, ESP32P4_CLK_LP_PMS,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    qdev_connect_gpio_out_named(DEVICE(&s->clkrst), ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_LP_PMS,
                                qdev_get_gpio_in_named(DEVICE(pms),
                                    ESP32P4_PMS_LP_RESET, 0));
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
                         "flash and PSRAM windows reach the memories only "
                         "through the MMU software sets up", filename,
                         paddr, memsz);
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
    esp32p4_init_bus(s);
    esp32p4_init_cpus(s);
    esp32p4_init_clocks(s);
    esp32p4_init_core_local(s);
    esp32p4_init_intmatrix(s);
    esp32p4_init_efuse(s);
    esp32p4_init_timers(s);
    esp32p4_init_gpio(s);
    esp32p4_init_pms(s);
    esp32p4_init_ext_memory(s);
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

static void esp32p4_get_strap_mode(Object *obj, Visitor *v,
                                   const char *name, void *opaque,
                                   Error **errp)
{
    visit_type_uint32(v, name, &ESP32P4_MACHINE(obj)->strap_mode, errp);
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
static void esp32p4_set_strap_mode(Object *obj, Visitor *v,
                                   const char *name, void *opaque,
                                   Error **errp)
{
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value >> ESP32P4_GPIO_STRAP_PINS) {
        error_setg(errp, "strap-mode has a bit for each of GPIO38 (bit 0) "
                   "to GPIO34 (bit 4); 0x%x has more", value);
        return;
    }
    ESP32P4_MACHINE(obj)->strap_mode = value;
}

static void esp32p4_machine_instance_init(Object *obj)
{
    ESP32P4_MACHINE(obj)->strap_mode = ESP32P4_GPIO_STRAP_SPI_BOOT;
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
    object_class_property_add(oc, "strap-mode", "uint32",
                              esp32p4_get_strap_mode, esp32p4_set_strap_mode,
                              NULL, NULL);
    object_class_property_set_description(oc, "strap-mode",
        "The board's strapping of GPIO38 (bit 0) to GPIO34 (bit 4), which "
        "GPIO_STRAP latches at reset (default 0x08: SPI boot)");

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
        .instance_init = esp32p4_machine_instance_init,
        .class_init    = esp32p4_machine_class_init,
    },
};

DEFINE_TYPES(esp32p4_machine_types)
