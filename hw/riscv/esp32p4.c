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
 *     BOOT_ADDR_HP_CORE1 and its record of refused accesses, the rest of
 *     whose registers are logged as unimplemented.
 *   - The eFuse controller and its array, on its clock and reset from
 *     LPPERI, its interrupt a source of the interrupt matrix.
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
 *     _3, sources of the matrix; the rest of its registers are logged as
 *     unimplemented. A source no modelled block drives stays low.
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
 *   - I2C0, I2C1 and the LP I2C (TRM chapter 47), and the I3C master and
 *     slave, at the level of their lines through the GPIO matrices, timed
 *     by their function clocks, their interrupts through the interrupt
 *     matrix. Each master's QEMU bus, "i2c0", "i2c1", "lp-i2c" and "i3c"
 *     (with "i3c-legacy-i2c" for I2C devices on the I3C bus), stands for
 *     the board's wires on the pads it is routed to; the board has no
 *     devices on them, and -device puts devices there:
 *
 *       -device tmp105,bus=i2c0,address=0x48
 *
 *     Pads the board wires to each other, so that two of the chip's
 *     controllers share a bus, are the GPIO block's "pad-wires":
 *
 *       -global esp32p4-gpio.pad-wires=7:9,8:10
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
    /* The AXI ICM's QoS command interface, above its own registers. */
    { "axi-icm-qos",      0x500a4400, 3 * KiB },
    { "h264-dma",         0x500a7000, 4 * KiB },

    /* HP peripherals 1. */
    { "mcpwm0",           0x500c0000, 4 * KiB },
    { "mcpwm1",           0x500c1000, 4 * KiB },
    { "timg0",            0x500c2000, 4 * KiB },
    { "timg1",            0x500c3000, 4 * KiB },
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
    { "lcd-cam",          0x500dc000, 4 * KiB },
    { "adc",              0x500de000, 4 * KiB },
    { "uhci",             0x500df000, 4 * KiB },
    { "iomux-mspi-pin",   0x500e1200, 4 * KiB - 0x200 },
    { "systimer",         0x500e2000, 4 * KiB },
    { "mem-monitor",      0x500e3000, 4 * KiB },
    { "audio-addc",       0x500e4000, 4 * KiB },

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
    { "lp-spi",           0x50123000, 4 * KiB },
    { "lp-i2s",           0x50125000, 4 * KiB },
    { "lp-trng",          0x50126000, 4 * KiB },
    { "lp-adc",           0x50127000, 4 * KiB },
    { "lp-touch",         0x50128000, 4 * KiB },
    { "lp-intr",          0x5012c000, 4 * KiB },
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

/*
 * The bus as every master sees it, through the access permission managers
 * (hw/misc/esp32p4_pms.c): the cores take it as their memory as they are
 * realized, before the blocks the APMs record refusals in exist.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
static void esp32p4_init_bus(ESP32P4MachineState *s)
{
    object_initialize_child(OBJECT(s), "pms", &s->pms, TYPE_ESP32P4_PMS);
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
                                 OBJECT(esp32p4_pms_bus(&s->pms)),
                                 &error_abort);
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
 * CLIC's interrupts are user-mode ones. And HP_SYSTEM's software
 * interrupts, behind SYSREG_APB_CLK, which are among its sources.
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
 * I2C0 and I2C1 (0x500C4000, 0x500C5000), their registers behind
 * I2Cn_APB_CLK and HP_SYS_CLKRST's I2Cn reset, timed by I2Cn_CLK, their
 * interrupts sources 44 and 45 and their lines the GPIO matrix's signals
 * 68 to 71; and the LP I2C (0x50122000), behind LPPERI's LP_I2C clock
 * gate and reset, timed by LP_I2C_SCLK, its interrupt source 11 and its
 * lines the LP GPIO matrix's signals 0 and 1. Their QEMU I2C buses,
 * "i2c0", "i2c1" and "lp-i2c", stand for the board's wires on the pads
 * each controller is routed to; the board has nothing on them, and
 * -device puts i2c-slave devices there:
 *
 *   -device tmp105,bus=i2c0,address=0x48
 */
/* [spec:nuos:req:emu.esp32p4.i2c] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_i2c(ESP32P4MachineState *s)
{
    static const struct {
        const char *bus;
        hwaddr base;
        unsigned bus_clock;
        unsigned reset;
        const char *sclk;
        int intr;
        unsigned scl;
        unsigned sda;
    } hp[] = {
        { "i2c0", 0x500c4000, ESP32P4_CLK_I2C0_APB, ESP32P4_RST_I2C0, "i2c0",
          ESP32P4_INTR_I2C0, ESP32P4_SIG_I2C0_SCL_PAD_OUT,
          ESP32P4_SIG_I2C0_SDA_PAD_OUT },
        { "i2c1", 0x500c5000, ESP32P4_CLK_I2C1_APB, ESP32P4_RST_I2C1, "i2c1",
          ESP32P4_INTR_I2C1, ESP32P4_SIG_I2C1_SCL_PAD_OUT,
          ESP32P4_SIG_I2C1_SDA_PAD_OUT },
    };
    DeviceState *gpio = DEVICE(&s->gpio), *clkrst = DEVICE(&s->clkrst);
    ESP32P4ClkRstGate *g;
    DeviceState *dev;
    int i;

    for (i = 0; i < ARRAY_SIZE(hp); i++) {
        object_initialize_child(OBJECT(s), hp[i].bus, &s->i2c[i],
                                TYPE_ESP32P4_I2C);
        dev = DEVICE(&s->i2c[i]);
        qdev_prop_set_string(dev, "bus-name", hp[i].bus);
        qdev_connect_clock_in(dev, "sclk",
                              qdev_get_clock_out(clkrst, hp[i].sclk));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                     hp[i].base, hp[i].bus_clock,
                                     hp[i].reset, 0);
        esp32p4_clkrst_gate_volatile(g, 0x1c, 4);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           esp32p4_intr_source(s, hp[i].intr));
        qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SCL_OUT, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT, hp[i].scl));
        qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SCL_OE, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE, hp[i].scl));
        qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SDA_OUT, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT, hp[i].sda));
        qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SDA_OE, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE, hp[i].sda));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, hp[i].scl,
            qdev_get_gpio_in_named(dev, ESP32P4_I2C_SCL_IN, 0));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, hp[i].sda,
            qdev_get_gpio_in_named(dev, ESP32P4_I2C_SDA_IN, 0));
    }

    object_initialize_child(OBJECT(s), "lp-i2c", &s->lp_i2c,
                            TYPE_ESP32P4_LP_I2C);
    dev = DEVICE(&s->lp_i2c);
    qdev_prop_set_string(dev, "bus-name", "lp-i2c");
    qdev_connect_clock_in(dev, "sclk",
                          qdev_get_clock_out(clkrst, "lp-i2c-sclk"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 0x50122000, ESP32P4_CLK_LP_I2C,
                                 ESP32P4_RST_LP_I2C, 0);
    esp32p4_clkrst_gate_volatile(g, 0x1c, 4);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_I2C));
    qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SCL_OUT, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OUT,
                               ESP32P4_LP_SIG_I2C_SCL_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SCL_OE, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OE,
                               ESP32P4_LP_SIG_I2C_SCL_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SDA_OUT, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OUT,
                               ESP32P4_LP_SIG_I2C_SDA_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_I2C_SDA_OE, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OE,
                               ESP32P4_LP_SIG_I2C_SDA_PAD_OUT));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_LP_SIG_IN,
                                ESP32P4_LP_SIG_I2C_SCL_PAD_IN,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_I2C_SCL_IN, 0));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_LP_SIG_IN,
                                ESP32P4_LP_SIG_I2C_SDA_PAD_IN,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_I2C_SDA_IN, 0));
}

/*
 * The I3C master (0x500DA000) and slave (0x500DB000), behind
 * I3C_MST_APB_CLK and I3C_SLV_APB_CLK and their resets, the master timed
 * by I3C_MST_CLK; their interrupts sources 101 and 102; their lines the
 * GPIO matrix's signals 134 to 137, and the master's pull-up enables 138
 * and 139. The master's QEMU I3C bus, "i3c", stands for the board's
 * wires on the pads it is routed to, i2c-slave devices going on its
 * legacy I2C bus, "i3c-legacy-i2c"; the board has nothing on them:
 *
 *   -device mock-i3c-target,bus=i3c,static-address=0x30,pid=...
 *
 * The slave's target logic sits on that bus, and takes part in the
 * master's transfers when the board wires the two controllers' pads
 * together.
 */
/* [spec:nuos:req:emu.esp32p4.i2c] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_i3c(ESP32P4MachineState *s)
{
    DeviceState *gpio = DEVICE(&s->gpio), *clkrst = DEVICE(&s->clkrst);
    DeviceState *mst, *slv;
    ESP32P4ClkRstGate *g;
    static const struct {
        const char *name;
        unsigned sig;
    } mst_lines[] = {
        { ESP32P4_I3C_SCL_OUT, ESP32P4_SIG_I3C_MST_SCL_PAD_OUT },
        { ESP32P4_I3C_SDA_OUT, ESP32P4_SIG_I3C_MST_SDA_PAD_OUT },
        { ESP32P4_I3C_SCL_PULLUP, ESP32P4_SIG_I3C_MST_SCL_PULLUP_EN_PAD_OUT },
        { ESP32P4_I3C_SDA_PULLUP, ESP32P4_SIG_I3C_MST_SDA_PULLUP_EN_PAD_OUT },
    };
    int i;

    object_initialize_child(OBJECT(s), "i3c-mst", &s->i3c_mst,
                            TYPE_ESP32P4_I3C_MST);
    mst = DEVICE(&s->i3c_mst);
    qdev_prop_set_string(mst, "bus-name", "i3c");
    qdev_connect_clock_in(mst, "clk", qdev_get_clock_out(clkrst, "i3c-mst"));
    sysbus_realize(SYS_BUS_DEVICE(mst), &error_fatal);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(mst), 0,
                                 0x500da000, ESP32P4_CLK_I3C_MST_APB,
                                 ESP32P4_RST_I3CMST, 0);
    esp32p4_clkrst_gate_volatile(g, 0x0c, 8);
    esp32p4_clkrst_gate_volatile(g, 0x18, 4);
    esp32p4_clkrst_gate_volatile(g, 0x40, 4);
    sysbus_connect_irq(SYS_BUS_DEVICE(mst), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_I3C_MST));
    for (i = 0; i < ARRAY_SIZE(mst_lines); i++) {
        qdev_connect_gpio_out_named(mst, mst_lines[i].name, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                                   mst_lines[i].sig));
    }
    qdev_connect_gpio_out_named(mst, ESP32P4_I3C_SCL_OE, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE,
                               ESP32P4_SIG_I3C_MST_SCL_PAD_OUT));
    qdev_connect_gpio_out_named(mst, ESP32P4_I3C_SDA_OE, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE,
                               ESP32P4_SIG_I3C_MST_SDA_PAD_OUT));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                ESP32P4_SIG_I3C_MST_SCL_PAD_IN,
                                qdev_get_gpio_in_named(mst,
                                    ESP32P4_I3C_SCL_IN, 0));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                ESP32P4_SIG_I3C_MST_SDA_PAD_IN,
                                qdev_get_gpio_in_named(mst,
                                    ESP32P4_I3C_SDA_IN, 0));

    object_initialize_child(OBJECT(s), "i3c-slv", &s->i3c_slv,
                            TYPE_ESP32P4_I3C_SLV);
    slv = DEVICE(&s->i3c_slv);
    object_property_set_link(OBJECT(slv), "i3c-bus", OBJECT(s->i3c_mst.bus),
                             &error_abort);
    qdev_connect_clock_in(slv, "clk", qdev_get_clock_out(clkrst,
                                                         "i3c-slv-apb"));
    sysbus_realize(SYS_BUS_DEVICE(slv), &error_fatal);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(slv), 0,
                                 0x500db000, ESP32P4_CLK_I3C_SLV_APB,
                                 ESP32P4_RST_I3CSLV, 0);
    esp32p4_clkrst_gate_volatile(g, 0x40, 4);
    esp32p4_clkrst_gate_volatile(g, 0x48, 4);
    sysbus_connect_irq(SYS_BUS_DEVICE(slv), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_I3C_SLV));
    qdev_connect_gpio_out_named(slv, ESP32P4_I3C_SCL_OUT, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                               ESP32P4_SIG_I3C_SLV_SCL_PAD_OUT));
    qdev_connect_gpio_out_named(slv, ESP32P4_I3C_SCL_OE, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE,
                               ESP32P4_SIG_I3C_SLV_SCL_PAD_OUT));
    qdev_connect_gpio_out_named(slv, ESP32P4_I3C_SDA_OUT, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                               ESP32P4_SIG_I3C_SLV_SDA_PAD_OUT));
    qdev_connect_gpio_out_named(slv, ESP32P4_I3C_SDA_OE, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE,
                               ESP32P4_SIG_I3C_SLV_SDA_PAD_OUT));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                ESP32P4_SIG_I3C_SLV_SCL_PAD_IN,
                                qdev_get_gpio_in_named(slv,
                                    ESP32P4_I3C_SCL_IN, 0));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                ESP32P4_SIG_I3C_SLV_SDA_PAD_IN,
                                qdev_get_gpio_in_named(slv,
                                    ESP32P4_I3C_SDA_IN, 0));
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
    esp32p4_init_bus(s);
    esp32p4_init_cpus(s);
    esp32p4_init_clocks(s);
    esp32p4_init_core_local(s);
    esp32p4_init_intmatrix(s);
    esp32p4_init_efuse(s);
    esp32p4_init_gpio(s);
    esp32p4_init_i2c(s);
    esp32p4_init_i3c(s);
    esp32p4_init_pms(s);
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
