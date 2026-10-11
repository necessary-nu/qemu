/*
 * ESP32-P4 permission control: the HP, LP and DMA access permission
 * managers (APMs)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 20 (Permission Control (PMS)) and 21.2.1.14, 21.2.2.11 (Illegal Access
 * and Unauthorized Access); ESP-IDF's hp_peri_pms_reg.h,
 * lp2hp_peri_pms_reg.h, dma_pms_reg.h, lp_peri_pms_reg.h and
 * hp2lp_peri_pms_reg.h for v3, which are the authority for the registers
 * and reset values, and its apm_ll.h, apm_hal.c and P4 APM tests.
 *
 * The APMs sit at the bus ports and check every access a master makes to
 * the slaves they manage (TRM table 20.1-1):
 *
 * - The HP APM (HP_PERI_PMS, LP2HP_PERI_PMS): the HP cores' accesses, in
 *   machine and in user mode, to the HP CPU peripherals (0x3FF00000), the
 *   HP peripherals (0x50000000) and, without the cache, to HP ROM, HP L2MEM
 *   and the external memory windows (0x8FC00000, 0x8FF00000, 0x80000000,
 *   0x88000000); and the LP core's accesses to the same peripherals and to
 *   those memories at the addresses it uses (0x4FC00000, 0x4FF00000,
 *   0x40000000, 0x48000000). An HP core's cached accesses to the memories
 *   and to the TCM are for its PMP alone.
 *
 * - The LP APM (LP_PERI_PMS, HP2LP_PERI_PMS): the HP cores' accesses, in
 *   either mode, to the LP peripherals (0x50110000 to 0x5012FFFF) and LP
 *   SRAM, and the LP core's to the LP peripherals.
 *
 * - The DMA APM (HP_DMA_PMS): each DMA master's reads and writes of HP ROM,
 *   HP L2MEM and the external memory windows, against 32 regions, each
 *   [LOW, HIGH) at 4 KiB granularity, and the master's read and write
 *   permission for each; an access is permitted if a region holding it
 *   permits it, so one in no region is refused (ESP-IDF's P4 DMA test
 *   needs a region over internal memory for exactly this reason). REGDMA's
 *   accesses to the HP and LP peripherals and to LP SRAM are for the HP
 *   and LP APMs, against REGDMA_HP_PERI_PMS and REGDMA_LP_PERI_PMS.
 *
 * A peripheral's slot is decided by one bit of the master's (and mode's)
 * register, ESP-IDF's apm_slave_*_id_t; a slot no bit names (the DDR PHY,
 * the memory monitor, the audio ADDC, LP GPIO, reserved slots) is not
 * checked. Before the fixed slots, the eight configurable regions of
 * LP_PERI_PMS, [LOW, HIGH) at 4-byte granularity, apply to every
 * peripheral register, HP CPU, HP and LP alike: an access in a region
 * whose bit for its master and mode is clear is refused, whatever the
 * slot allows. The ends of both kinds of region are exclusive, as ESP-IDF's
 * P4 tests program them (the next block's base as a region's end); the TRM
 * says only that the registers hold an address's high bits.
 *
 * A refused access does not reach its slave: the bus answers it with an
 * error, which a core takes as an access fault, a read returns no data and
 * a write is dropped. The refusal is recorded, with the master's ID, the
 * address and whether it was a write, by the AXI ICM for the HP and DMA
 * APMs (hw/misc/esp32p4_icm.c, raising SYS_ICM_INTR) and by LP_SYS for the
 * LP APM (LP_SYSREG_INTR). An HP core's access its PMP refuses never
 * reaches the bus, so no APM sees it.
 *
 * The model is an IOMMU over the whole bus, which every master reaches
 * through (include/hw/misc/esp32p4_pms.h): for each 4 KiB page and each
 * master and mode, a page the APMs permit throughout maps straight to
 * system memory, RAM included, so that the cores keep the TLB's fast
 * path; any other page goes to a region that checks each access and
 * either passes it on or refuses it. Every change to the registers
 * flushes the cores' TLBs.
 *
 * A refused access answers with a bus error, which a core takes as an
 * access fault unless HP_SYSTEM_CORE_ERR_RESP_DIS or
 * LP_SYSTEM_LP_CORE_ERR_RESP_DIS disables the error response of the bus
 * it went out on (target/riscv/esp32p4_cpu.c): then it completes without
 * one, a read returning 0.
 *
 * Not modelled: a register's CLK_EN (automatic clock gating, invisible to
 * software but for power) is kept as written; and the buses' record of
 * an access to an address nothing decodes (SECURE 1 in the same records)
 * is not kept, though the access still answers with a bus error.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "exec/cputlb.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_pms.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/tcg.h"

#define PMS_PAGE                0x1000u

/* The DATE registers. */
#define PERI_PMS_DATE_RESET     0x241227
#define DMA_PMS_DATE_RESET      0x20230314

/*
 * HP_PERI_PMS register k (k = 0..3) of core c in mode m (0 machine,
 * 1 user), and LP2HP_PERI_PMS register k.
 */
#define A_HP_PERI(c, m, k)      (0x08 + 0x20 * (c) + 0x10 * (m) + 4 * (k))
#define A_REGDMA_HP_PERI        0x48
static const hwaddr esp32p4_lp2hp_regs[4] = { 0x08, 0x30, 0xa4, 0x11c };

/* The bits each of the four registers has; all reset to 1. */
static const uint32_t esp32p4_hp_peri_bits[4] = {
    0x00000fcf, 0x3fffffff, 0xdfefffff, 0x0000001f,
};

#define A_DMA_LOW(n)            (0x08 + 8 * (n))
#define A_DMA_HIGH(n)           (0x0c + 8 * (n))
#define DMA_REGION_MASK         0xfffff000u

#define A_LP_PERI_LP            0x08
#define A_PERI_REGION_PMS       0x1c
#define A_PERI_REGION_2_TO_7    0x20
#define PERI_REGION_MASK        0xfffffffcu
#define LP_PERI_BITS            0x01ffffff

/* HP2LP_PERI_PMS register of core c in mode m. */
#define A_HP2LP(c, m)           (0x08 + 8 * (c) + 4 * (m))
#define A_REGDMA_LP_PERI        0x18
#define REGDMA_LP_SRAM          BIT(0)
#define REGDMA_LP_PERI          BIT(1)

/* LP SRAM's bit in HP2LP_PERI_PMS and LP_PERI_PMS. */
#define LP_SRAM_BIT             23

static const hwaddr esp32p4_pms_block_size[ESP32P4_PMS_NUM_BLOCKS] = {
    [ESP32P4_PMS_MMIO_HP_PERI] = 0x800,
    [ESP32P4_PMS_MMIO_LP2HP_PERI] = 0x800,
    [ESP32P4_PMS_MMIO_DMA] = 0x1000,
    [ESP32P4_PMS_MMIO_LP_PERI] = 0x800,
    [ESP32P4_PMS_MMIO_HP2LP_PERI] = 0x800,
};

static const char *const esp32p4_pms_block_name[ESP32P4_PMS_NUM_BLOCKS] = {
    [ESP32P4_PMS_MMIO_HP_PERI] = "esp32p4.hp-peri-pms",
    [ESP32P4_PMS_MMIO_LP2HP_PERI] = "esp32p4.lp2hp-peri-pms",
    [ESP32P4_PMS_MMIO_DMA] = "esp32p4.dma-pms",
    [ESP32P4_PMS_MMIO_LP_PERI] = "esp32p4.lp-peri-pms",
    [ESP32P4_PMS_MMIO_HP2LP_PERI] = "esp32p4.hp2lp-peri-pms",
};

/*
 * The offset of DMA master @m's read permission register; its write one
 * follows, but for GDMA-AHB RMT's. -1 for a number with no master.
 */
static int esp32p4_dma_pms_r(int m)
{
    if (m < 0 || m >= ESP32P4_DMA_MASTERS || m == 11 || m == 12 || m == 29) {
        return -1;
    }
    return 0x108 + 8 * m + (m >= ESP32P4_DMA_MASTER_AXI_PDMA_LCDCAM ? 4 : 0);
}

static int esp32p4_dma_pms_w(int m)
{
    if (m == ESP32P4_DMA_MASTER_AHB_PDMA_RMT) {
        return 0x170;
    }
    return esp32p4_dma_pms_r(m) < 0 ? -1 : esp32p4_dma_pms_r(m) + 4;
}

/* The offsets of configurable peripheral region n's bounds. */
static hwaddr esp32p4_peri_region_low(int n)
{
    return n < 2 ? 0x0c + 8 * n : 0x24 + 8 * (n - 2);
}

/*
 * The reset value and writable bits of the register at @offset of
 * @block; false for a reserved offset.
 */
static bool esp32p4_pms_reg_info(int block, hwaddr offset, uint32_t *reset,
                                 uint32_t *mask)
{
    int i;

    *mask = 0xffffffff;
    switch (offset) {
    case 0x0:
        *reset = block == ESP32P4_PMS_MMIO_DMA ? DMA_PMS_DATE_RESET
                                               : PERI_PMS_DATE_RESET;
        return true;
    case 0x4:
        *reset = *mask = 1;
        return true;
    }

    switch (block) {
    case ESP32P4_PMS_MMIO_HP_PERI:
        if (offset >= A_HP_PERI(0, 0, 0) && offset < A_REGDMA_HP_PERI) {
            *reset = *mask =
                esp32p4_hp_peri_bits[(offset - A_HP_PERI(0, 0, 0)) / 4 % 4];
            return true;
        }
        if (offset == A_REGDMA_HP_PERI) {
            *reset = *mask = 1;
            return true;
        }
        return false;
    case ESP32P4_PMS_MMIO_LP2HP_PERI:
        for (i = 0; i < 4; i++) {
            if (offset == esp32p4_lp2hp_regs[i]) {
                *reset = *mask = esp32p4_hp_peri_bits[i];
                return true;
            }
        }
        return false;
    case ESP32P4_PMS_MMIO_DMA:
        if (offset >= A_DMA_LOW(0) &&
            offset <= A_DMA_HIGH(ESP32P4_DMA_PMS_REGIONS - 1)) {
            *mask = DMA_REGION_MASK;
            *reset = offset % 8 ? DMA_REGION_MASK : 0;
            return true;
        }
        for (i = 0; i < ESP32P4_DMA_MASTERS; i++) {
            if (offset == esp32p4_dma_pms_r(i) ||
                offset == esp32p4_dma_pms_w(i)) {
                *reset = 0xffffffff;
                return true;
            }
        }
        return false;
    case ESP32P4_PMS_MMIO_LP_PERI:
        if (offset == A_LP_PERI_LP) {
            *reset = *mask = LP_PERI_BITS;
            return true;
        }
        if (offset == A_PERI_REGION_PMS) {
            *reset = *mask = 0x3ff;
            return true;
        }
        if (offset == A_PERI_REGION_2_TO_7) {
            *reset = *mask = 0x3fffffff;
            return true;
        }
        for (i = 0; i < ESP32P4_PERI_PMS_REGIONS; i++) {
            if (offset == esp32p4_peri_region_low(i) ||
                offset == esp32p4_peri_region_low(i) + 4) {
                *mask = PERI_REGION_MASK;
                *reset = offset == esp32p4_peri_region_low(i)
                         ? 0 : PERI_REGION_MASK;
                return true;
            }
        }
        return false;
    case ESP32P4_PMS_MMIO_HP2LP_PERI:
        if (offset >= A_HP2LP(0, 0) && offset < A_REGDMA_LP_PERI) {
            *reset = *mask = LP_PERI_BITS;
            return true;
        }
        if (offset == A_REGDMA_LP_PERI) {
            *reset = *mask = REGDMA_LP_SRAM | REGDMA_LP_PERI;
            return true;
        }
        return false;
    default:
        g_assert_not_reached();
    }
}

static uint32_t esp32p4_pms_reg(ESP32P4PmsState *s, int block, hwaddr offset)
{
    return s->regs[block][offset / 4];
}

static void esp32p4_pms_reset_block(ESP32P4PmsState *s, int block)
{
    uint32_t reset, mask;

    for (hwaddr off = 0; off < ESP32P4_PMS_MAX_WORDS * 4; off += 4) {
        s->regs[block][off / 4] =
            esp32p4_pms_reg_info(block, off, &reset, &mask) ? reset : 0;
    }
}

/*
 * The cores' TLBs hold the APMs' decisions page by page, so every change
 * of a register flushes them: at once for the core making the change, and
 * before it next executes for any other.
 */
static void esp32p4_pms_flush_work(CPUState *cpu, run_on_cpu_data data)
{
    tlb_flush(cpu);
}

static void esp32p4_pms_changed(ESP32P4PmsState *s)
{
    CPUState *cpu;

    if (!tcg_enabled()) {
        return;
    }
    CPU_FOREACH(cpu) {
        if (!cpu->created) {
            continue;
        }
        if (qemu_cpu_is_self(cpu)) {
            tlb_flush(cpu);
        } else {
            async_run_on_cpu(cpu, esp32p4_pms_flush_work, RUN_ON_CPU_NULL);
        }
    }
}

/* Decisions */

/* The APM that refuses an access, or none. */
enum {
    APM_NONE,
    APM_HP,
    APM_LP,
    APM_DMA,
};

/* The directions of an access. */
#define PMS_R                   BIT(0)
#define PMS_W                   BIT(1)

/*
 * The masters as the IOMMU indexes them: accesses no APM checks, each HP
 * core in machine and in user mode, the LP core, and each DMA master.
 */
enum {
    IDX_OPEN,
    IDX_CORE0_M,
    IDX_CORE0_U,
    IDX_CORE1_M,
    IDX_CORE1_U,
    IDX_LP_CORE,
    IDX_DMA,
    NUM_IDX = IDX_DMA + ESP32P4_DMA_MASTERS,
};

/* The groups of slots, the first four the registers k = 0..3. */
enum {
    SLOT_CPU_PERI,
    SLOT_HP_PERI0,
    SLOT_HP_PERI1,
    SLOT_HP_PERI2,
    /* HP ROM, L2MEM and the external windows without the cache. */
    SLOT_MEM_DIRECT,
    /* The same, as the LP core and the DMA masters address them. */
    SLOT_MEM,
    SLOT_LP_PERI,
    SLOT_LP_SRAM,
};

typedef struct {
    uint32_t base;
    uint32_t size;
    uint8_t group;
    uint8_t bit;
} ESP32P4PmsSlot;

#define SLOT(b, sz, g, n) { (b), (sz), SLOT_##g, (n) }
#define HP_PERI1(n) SLOT(0x500c0000 + 0x1000 * (n), 0x1000, HP_PERI1, n)

/* Every slot an APM checks, by address. */
static const ESP32P4PmsSlot esp32p4_pms_slots[] = {
    /* HP CPU peripherals: trace 0 and 1, bus, L2MEM, TCM monitors, cache */
    SLOT(0x3ff04000, 0x1000, CPU_PERI, 6),
    SLOT(0x3ff05000, 0x1000, CPU_PERI, 7),
    SLOT(0x3ff06000, 0x1000, CPU_PERI, 8),
    SLOT(0x3ff0e000, 0x1000, CPU_PERI, 9),
    SLOT(0x3ff0f000, 0x1000, CPU_PERI, 10),
    SLOT(0x3ff10000, 0x1000, CPU_PERI, 11),
    /* Flash, PSRAM, HP ROM, HP L2MEM */
    SLOT(0x40000000, 64 * MiB, MEM, 1),
    SLOT(0x48000000, 64 * MiB, MEM, 0),
    SLOT(0x4fc00000, 128 * KiB, MEM, 3),
    SLOT(0x4ff00000, 768 * KiB, MEM, 2),
    /* HP peripherals 0 */
    SLOT(0x50000000, 256 * KiB, HP_PERI0, 0),   /* USB OTG HS */
    SLOT(0x50040000, 256 * KiB, HP_PERI0, 1),   /* USB OTG FS */
    SLOT(0x50080000, 0x1000, HP_PERI0, 2),      /* its wrap */
    SLOT(0x50081000, 0x1000, HP_PERI0, 3),      /* VDMA */
    SLOT(0x50082000, 0x1000, HP_PERI0, 4),      /* REGDMA */
    SLOT(0x50083000, 0x1000, HP_PERI0, 5),      /* SDMMC */
    SLOT(0x50084000, 0x1000, HP_PERI0, 21),     /* H264 */
    SLOT(0x50085000, 0x1000, HP_PERI0, 6),      /* GDMA-AHB */
    SLOT(0x50086000, 0x1000, HP_PERI0, 7),      /* JPEG */
    SLOT(0x50087000, 0x1000, HP_PERI0, 8),      /* PPA */
    SLOT(0x50088000, 0x1000, HP_PERI0, 9),      /* 2D-DMA */
    SLOT(0x50089000, 0x1000, HP_PERI0, 10),     /* key manager */
    SLOT(0x5008a000, 0x2000, HP_PERI0, 11),     /* GDMA-AXI */
    SLOT(0x5008c000, 0x2000, HP_PERI0, 12),     /* flash MSPI */
    SLOT(0x5008e000, 0x2000, HP_PERI0, 13),     /* PSRAM MSPI */
    SLOT(0x50090000, 0x7000, HP_PERI0, 14),     /* crypto */
    SLOT(0x50098000, 0x4000, HP_PERI0, 15),     /* EMAC */
    SLOT(0x5009c000, 0x1000, HP_PERI0, 16),     /* USB PHY */
    SLOT(0x5009e000, 0x1000, HP_PERI0, 17),     /* PVT */
    SLOT(0x5009f000, 0x1000, HP_PERI0, 18),     /* CSI host and bridge */
    SLOT(0x500a0000, 0x1000, HP_PERI0, 19),     /* DSI host and bridge */
    SLOT(0x500a1000, 0x1000, HP_PERI0, 20),     /* ISP */
    SLOT(0x500a2000, 0x1000, HP_PERI0, 22),     /* RMT */
    SLOT(0x500a3000, 0x1000, HP_PERI0, 23),     /* bit scrambler */
    SLOT(0x500a4000, 0x1000, HP_PERI0, 24),     /* AXI ICM */
    SLOT(0x500a5000, 0x0800, HP_PERI0, 25),     /* HP_PERI_PMS */
    SLOT(0x500a5800, 0x0800, HP_PERI0, 26),     /* LP2HP_PERI_PMS */
    SLOT(0x500a6000, 0x1000, HP_PERI0, 27),     /* HP_DMA_PMS */
    SLOT(0x500a7000, 0x1000, HP_PERI0, 28),     /* H264 DMA */
    /* HP peripherals 1: a bit per 4 KiB slot, but the two reserved ones */
    HP_PERI1(0), HP_PERI1(1), HP_PERI1(2), HP_PERI1(3), HP_PERI1(4),
    HP_PERI1(5), HP_PERI1(6), HP_PERI1(7), HP_PERI1(8), HP_PERI1(9),
    HP_PERI1(10), HP_PERI1(11), HP_PERI1(12), HP_PERI1(13), HP_PERI1(14),
    HP_PERI1(15), HP_PERI1(16), HP_PERI1(17), HP_PERI1(18), HP_PERI1(19),
    HP_PERI1(21), HP_PERI1(22), HP_PERI1(23), HP_PERI1(24), HP_PERI1(25),
    HP_PERI1(26), HP_PERI1(27), HP_PERI1(28), HP_PERI1(30), HP_PERI1(31),
    SLOT(0x500e0000, 0x1000, HP_PERI2, 0),      /* GPIO */
    SLOT(0x500e1000, 0x1000, HP_PERI2, 1),      /* IO MUX */
    SLOT(0x500e2000, 0x1000, HP_PERI2, 2),      /* SYSTIMER */
    SLOT(0x500e5000, 0x1000, HP_PERI2, 3),      /* HP_SYSTEM */
    SLOT(0x500e6000, 0x1000, HP_PERI2, 4),      /* HP_SYS_CLKRST */
    SLOT(0x50108000, 32 * KiB, LP_SRAM, LP_SRAM_BIT),
    /* LP peripherals */
    SLOT(0x50110000, 0x1000, LP_PERI, 0),       /* LP_SYS */
    SLOT(0x50111000, 0x1000, LP_PERI, 1),       /* LP_AON_CLKRST */
    SLOT(0x50112000, 0x1000, LP_PERI, 2),       /* LP timer */
    SLOT(0x50113000, 0x1000, LP_PERI, 3),       /* LP analog peripherals */
    SLOT(0x50114000, 0x1000, LP_PERI, 22),      /* HUK */
    SLOT(0x50115000, 0x1000, LP_PERI, 4),       /* PMU */
    SLOT(0x50116000, 0x1000, LP_PERI, 5),       /* LP WDT */
    SLOT(0x50118000, 0x1000, LP_PERI, 6),       /* LP mailbox */
    SLOT(0x50119000, 0x1000, LP_PERI, 7),       /* RTC */
    SLOT(0x50120000, 0x1000, LP_PERI, 8),       /* LPPERI */
    SLOT(0x50121000, 0x1000, LP_PERI, 9),       /* LP UART */
    SLOT(0x50122000, 0x1000, LP_PERI, 10),      /* LP I2C */
    SLOT(0x50123000, 0x1000, LP_PERI, 11),      /* LP SPI */
    SLOT(0x50124000, 0x1000, LP_PERI, 12),      /* LP analog I2C master */
    SLOT(0x50125000, 0x1000, LP_PERI, 13),      /* LP I2S */
    SLOT(0x50126000, 0x1000, LP_PERI, 24),      /* LP TRNG */
    SLOT(0x50127000, 0x1000, LP_PERI, 14),      /* LP ADC */
    SLOT(0x50128000, 0x1000, LP_PERI, 15),      /* LP touch */
    SLOT(0x5012b000, 0x1000, LP_PERI, 16),      /* LP IO MUX */
    SLOT(0x5012c000, 0x1000, LP_PERI, 17),      /* LP INTR */
    SLOT(0x5012d000, 0x1000, LP_PERI, 18),      /* eFuse */
    SLOT(0x5012e000, 0x0800, LP_PERI, 19),      /* LP_PERI_PMS */
    SLOT(0x5012e800, 0x0800, LP_PERI, 20),      /* HP2LP_PERI_PMS */
    SLOT(0x5012f000, 0x1000, LP_PERI, 21),      /* LP temperature sensor */
    /* Flash, PSRAM, HP ROM, HP L2MEM without the cache */
    SLOT(0x80000000, 64 * MiB, MEM_DIRECT, 1),
    SLOT(0x88000000, 64 * MiB, MEM_DIRECT, 0),
    SLOT(0x8fc00000, 128 * KiB, MEM_DIRECT, 3),
    SLOT(0x8ff00000, 768 * KiB, MEM_DIRECT, 2),
};

/* The index of the first slot that ends above @addr. */
static int esp32p4_pms_slot_from(uint32_t addr)
{
    int lo = 0, hi = ARRAY_SIZE(esp32p4_pms_slots);

    while (lo < hi) {
        int mid = (lo + hi) / 2;
        const ESP32P4PmsSlot *sl = &esp32p4_pms_slots[mid];

        if ((uint64_t)sl->base + sl->size <= addr) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

static const ESP32P4PmsSlot *esp32p4_pms_slot(uint32_t addr)
{
    int i = esp32p4_pms_slot_from(addr);

    if (i < ARRAY_SIZE(esp32p4_pms_slots) &&
        esp32p4_pms_slots[i].base <= addr) {
        return &esp32p4_pms_slots[i];
    }
    return NULL;
}

/* The peripheral registers, which the configurable regions apply to. */
static int esp32p4_pms_peri_apm(uint32_t addr)
{
    if ((addr >= 0x3ff00000 && addr < 0x3ff20000) ||
        (addr >= 0x50000000 && addr < 0x50100000)) {
        return APM_HP;
    }
    if (addr >= 0x50110000 && addr < 0x50130000) {
        return APM_LP;
    }
    return APM_NONE;
}

/*
 * Whether configurable region @n permits the CPU master @cpu (IDX_CORE0_M
 * onwards): its bit for the master in PERI_REGION_PMS (regions 0 and 1)
 * or PERI_REGION_2_TO_7_PMS.
 */
static bool esp32p4_pms_region_permits(ESP32P4PmsState *s, int cpu, int n)
{
    static const uint8_t base01[] = {
        [IDX_CORE0_M] = 4, [IDX_CORE0_U] = 2, [IDX_CORE1_M] = 8,
        [IDX_CORE1_U] = 6, [IDX_LP_CORE] = 0,
    };
    static const uint8_t base27[] = {
        [IDX_CORE0_M] = 12, [IDX_CORE0_U] = 6, [IDX_CORE1_M] = 24,
        [IDX_CORE1_U] = 18, [IDX_LP_CORE] = 0,
    };

    if (n < 2) {
        return extract32(esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI,
                                         A_PERI_REGION_PMS),
                         base01[cpu] + n, 1);
    }
    return extract32(esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI,
                                     A_PERI_REGION_2_TO_7),
                     base27[cpu] + n - 2, 1);
}

static bool esp32p4_pms_in_peri_region(ESP32P4PmsState *s, int n,
                                       uint32_t addr)
{
    hwaddr low = esp32p4_peri_region_low(n);

    return addr >= esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI, low) &&
           addr < esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI, low + 4);
}

/* The HP and LP APMs on an access by a core. */
static int esp32p4_pms_check_cpu(ESP32P4PmsState *s, int idx, uint32_t addr)
{
    const ESP32P4PmsSlot *sl = esp32p4_pms_slot(addr);
    int peri = esp32p4_pms_peri_apm(addr);
    bool lp = idx == IDX_LP_CORE;
    int core = (idx - IDX_CORE0_M) / 2;
    int user = (idx - IDX_CORE0_M) % 2;
    uint32_t bits;
    int n;

    if (peri != APM_NONE) {
        for (n = 0; n < ESP32P4_PERI_PMS_REGIONS; n++) {
            if (esp32p4_pms_in_peri_region(s, n, addr) &&
                !esp32p4_pms_region_permits(s, idx, n)) {
                return peri;
            }
        }
    }
    if (!sl) {
        return APM_NONE;
    }

    switch (sl->group) {
    case SLOT_CPU_PERI:
    case SLOT_HP_PERI0:
    case SLOT_HP_PERI1:
    case SLOT_HP_PERI2:
        bits = lp ? esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP2HP_PERI,
                                    esp32p4_lp2hp_regs[sl->group])
                  : esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_HP_PERI,
                                    A_HP_PERI(core, user, sl->group));
        return bits & BIT(sl->bit) ? APM_NONE : APM_HP;
    case SLOT_MEM_DIRECT:
        if (lp) {
            return APM_NONE;
        }
        bits = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_HP_PERI,
                               A_HP_PERI(core, user, 0));
        return bits & BIT(sl->bit) ? APM_NONE : APM_HP;
    case SLOT_MEM:
        if (!lp) {
            return APM_NONE;
        }
        bits = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP2HP_PERI,
                               esp32p4_lp2hp_regs[0]);
        return bits & BIT(sl->bit) ? APM_NONE : APM_HP;
    case SLOT_LP_PERI:
        bits = lp ? esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI,
                                    A_LP_PERI_LP)
                  : esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_HP2LP_PERI,
                                    A_HP2LP(core, user));
        return bits & BIT(sl->bit) ? APM_NONE : APM_LP;
    case SLOT_LP_SRAM:
        /* The LP core reaches LP SRAM by its own buses, which no APM sees. */
        if (lp) {
            return APM_NONE;
        }
        bits = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_HP2LP_PERI,
                               A_HP2LP(core, user));
        return bits & BIT(sl->bit) ? APM_NONE : APM_LP;
    default:
        g_assert_not_reached();
    }
}

/* Whether a DMA region permits master @m the directions @dirs at @addr. */
static bool esp32p4_pms_dma_permits(ESP32P4PmsState *s, int m, uint32_t addr,
                                    unsigned dirs)
{
    uint32_t r = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_DMA,
                                 esp32p4_dma_pms_r(m));
    uint32_t w = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_DMA,
                                 esp32p4_dma_pms_w(m));
    unsigned granted = 0;

    for (int n = 0; n < ESP32P4_DMA_PMS_REGIONS; n++) {
        if (addr >= esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_DMA, A_DMA_LOW(n)) &&
            addr < esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_DMA, A_DMA_HIGH(n))) {
            granted |= (r & BIT(n) ? PMS_R : 0) | (w & BIT(n) ? PMS_W : 0);
        }
    }
    return (granted & dirs) == dirs;
}

/* The DMA APM, and the HP and LP APMs for REGDMA, on a DMA access. */
static int esp32p4_pms_check_dma(ESP32P4PmsState *s, int m, uint32_t addr,
                                 unsigned dirs)
{
    const ESP32P4PmsSlot *sl = esp32p4_pms_slot(addr);
    uint32_t bits;

    if (!sl) {
        return APM_NONE;
    }
    if (sl->group == SLOT_MEM) {
        return esp32p4_pms_dma_permits(s, m, addr, dirs) ? APM_NONE
                                                         : APM_DMA;
    }
    if (m != ESP32P4_DMA_MASTER_REGDMA) {
        return APM_NONE;
    }
    switch (sl->group) {
    case SLOT_HP_PERI0:
    case SLOT_HP_PERI1:
    case SLOT_HP_PERI2:
        bits = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_HP_PERI,
                               A_REGDMA_HP_PERI);
        return bits & 1 ? APM_NONE : APM_HP;
    case SLOT_LP_PERI:
    case SLOT_LP_SRAM:
        bits = esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_HP2LP_PERI,
                               A_REGDMA_LP_PERI);
        return bits & (sl->group == SLOT_LP_SRAM ? REGDMA_LP_SRAM
                                                 : REGDMA_LP_PERI)
               ? APM_NONE : APM_LP;
    default:
        return APM_NONE;
    }
}

/*
 * The APM that refuses the access by master @idx to the byte at @addr in
 * the directions @dirs, or APM_NONE.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
static int esp32p4_pms_check(ESP32P4PmsState *s, int idx, uint32_t addr,
                             unsigned dirs)
{
    if (idx == IDX_OPEN) {
        return APM_NONE;
    }
    if (idx >= IDX_DMA) {
        return esp32p4_pms_check_dma(s, idx - IDX_DMA, addr, dirs);
    }
    return esp32p4_pms_check_cpu(s, idx, addr);
}

/*
 * Whether the APMs permit master @idx every byte of the page at @page:
 * the decision can only change where a slot or a region starts or ends,
 * so it is taken at the page's start and at each such point inside it.
 */
static bool esp32p4_pms_page_open(ESP32P4PmsState *s, int idx, uint32_t page,
                                  unsigned dirs)
{
    uint32_t last = page + (PMS_PAGE - 1);
    int i, n;

    if (esp32p4_pms_check(s, idx, page, dirs) != APM_NONE) {
        return false;
    }
    for (i = esp32p4_pms_slot_from(page);
         i < ARRAY_SIZE(esp32p4_pms_slots) &&
         esp32p4_pms_slots[i].base <= last; i++) {
        uint64_t start = esp32p4_pms_slots[i].base;
        uint64_t end = start + esp32p4_pms_slots[i].size;

        if ((start > page &&
             esp32p4_pms_check(s, idx, start, dirs) != APM_NONE) ||
            (end <= last &&
             esp32p4_pms_check(s, idx, end, dirs) != APM_NONE)) {
            return false;
        }
    }
    if (idx >= IDX_DMA || esp32p4_pms_peri_apm(page) == APM_NONE) {
        return true;
    }
    for (n = 0; n < ESP32P4_PERI_PMS_REGIONS; n++) {
        hwaddr low = esp32p4_peri_region_low(n);
        uint32_t bound[2] = {
            esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI, low),
            esp32p4_pms_reg(s, ESP32P4_PMS_MMIO_LP_PERI, low + 4),
        };

        for (i = 0; i < 2; i++) {
            if (bound[i] > page && bound[i] <= last &&
                esp32p4_pms_check(s, idx, bound[i], dirs) != APM_NONE) {
                return false;
            }
        }
    }
    return true;
}

static int esp32p4_pms_attrs_to_idx(MemTxAttrs attrs)
{
    uint16_t rid = attrs.requester_id;
    int m;

    if (attrs.unspecified || attrs.debug ||
        !(rid & ESP32P4_REQUESTER_VALID)) {
        return IDX_OPEN;
    }
    if (rid & ESP32P4_REQUESTER_IS_DMA) {
        m = ESP32P4_REQUESTER_MASTER(rid);
        return esp32p4_dma_pms_r(m) < 0 ? IDX_OPEN : IDX_DMA + m;
    }
    switch (ESP32P4_REQUESTER_ID(rid)) {
    case ESP32P4_MASTER_HP_CORE0:
        return attrs.user ? IDX_CORE0_U : IDX_CORE0_M;
    case ESP32P4_MASTER_HP_CORE1:
        return attrs.user ? IDX_CORE1_U : IDX_CORE1_M;
    case ESP32P4_MASTER_LP_CORE:
        return IDX_LP_CORE;
    default:
        return IDX_OPEN;
    }
}

/* Refusals */

static void esp32p4_pms_refuse(ESP32P4PmsState *s, int apm, int idx,
                               MemTxAttrs attrs, uint32_t addr, bool write)
{
    static const char *const apm_names[] = {
        [APM_HP] = "HP", [APM_LP] = "LP", [APM_DMA] = "DMA",
    };
    static const char *const cpu_names[] = {
        [IDX_CORE0_M] = "HP core 0 in machine mode",
        [IDX_CORE0_U] = "HP core 0 in user mode",
        [IDX_CORE1_M] = "HP core 1 in machine mode",
        [IDX_CORE1_U] = "HP core 1 in user mode",
        [IDX_LP_CORE] = "the LP core",
    };
    uint8_t id = ESP32P4_REQUESTER_ID(attrs.requester_id);
    int m = idx - IDX_DMA;

    if (idx >= IDX_DMA) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the %s APM refuses DMA master "
                      "%d's %s at 0x%08" PRIx32 "\n", TYPE_ESP32P4_PMS,
                      apm_names[apm], m, write ? "write" : "read", addr);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the %s APM refuses %s's %s at "
                      "0x%08" PRIx32 "\n", TYPE_ESP32P4_PMS, apm_names[apm],
                      cpu_names[idx], write ? "write" : "read", addr);
    }

    switch (apm) {
    case APM_HP:
        esp32p4_icm_addrhole(s->icm, false, addr, id, write, false);
        break;
    case APM_LP:
        esp32p4_lp_sys_addrhole(s->lp_sys, addr, id, write, false);
        break;
    case APM_DMA:
        esp32p4_icm_addrhole(s->icm, !ESP32P4_DMA_MASTER_ON_AHB(m), addr, id,
                             write, false);
        break;
    default:
        g_assert_not_reached();
    }
}

/*
 * An access to a page the APMs do not permit throughout: checked at its
 * first and last bytes, then passed on to system memory or refused.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
static MemTxResult esp32p4_pms_check_access(ESP32P4PmsState *s, hwaddr addr,
                                            uint64_t *data, unsigned size,
                                            MemTxAttrs attrs, bool write)
{
    int idx = esp32p4_pms_attrs_to_idx(attrs);
    unsigned dir = write ? PMS_W : PMS_R;
    int apm = esp32p4_pms_check(s, idx, addr, dir);
    uint8_t buf[8];
    MemTxResult r;

    if (apm == APM_NONE && size > 1) {
        apm = esp32p4_pms_check(s, idx, addr + size - 1, dir);
    }
    if (apm != APM_NONE) {
        esp32p4_pms_refuse(s, apm, idx, attrs, addr, write);
        if (!write) {
            *data = 0;
        }
        return MEMTX_ERROR;
    }

    if (write) {
        stn_le_p(buf, size, *data);
    }
    r = address_space_rw(&address_space_memory, addr, attrs, buf, size,
                         write);
    if (!write) {
        *data = ldn_le_p(buf, size);
    }
    return r;
}

static MemTxResult esp32p4_pms_check_read(void *opaque, hwaddr addr,
                                          uint64_t *data, unsigned size,
                                          MemTxAttrs attrs)
{
    return esp32p4_pms_check_access(opaque, addr, data, size, attrs, false);
}

static MemTxResult esp32p4_pms_check_write(void *opaque, hwaddr addr,
                                           uint64_t value, unsigned size,
                                           MemTxAttrs attrs)
{
    return esp32p4_pms_check_access(opaque, addr, &value, size, attrs, true);
}

/* The check fronts memories as well as registers, so takes any access. */
static const MemoryRegionOps esp32p4_pms_check_ops = {
    .read_with_attrs = esp32p4_pms_check_read,
    .write_with_attrs = esp32p4_pms_check_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .impl.unaligned = true,
};

/* The IOMMU */

/* [spec:nuos:req:emu.esp32p4.pms] */
static IOMMUTLBEntry esp32p4_pms_translate(IOMMUMemoryRegion *iommu,
                                           hwaddr addr,
                                           IOMMUAccessFlags flags,
                                           int iommu_idx)
{
    ESP32P4PmsState *s = container_of(iommu, ESP32P4PmsState, iommu);
    uint32_t page = addr & ~(hwaddr)(PMS_PAGE - 1);
    unsigned dirs = (flags & IOMMU_RO ? PMS_R : 0) |
                    (flags & IOMMU_WO ? PMS_W : 0);
    bool open;

    /* A core's TLB asks for every permission at once. */
    if (!dirs) {
        dirs = PMS_R | PMS_W;
    }
    open = iommu_idx == IDX_OPEN ||
           esp32p4_pms_page_open(s, iommu_idx, page, dirs);
    return (IOMMUTLBEntry) {
        .target_as = open ? &address_space_memory : &s->check_as,
        .iova = page,
        .translated_addr = page,
        .addr_mask = PMS_PAGE - 1,
        .perm = IOMMU_RW,
    };
}

static int esp32p4_pms_attrs_to_index(IOMMUMemoryRegion *iommu,
                                      MemTxAttrs attrs)
{
    return esp32p4_pms_attrs_to_idx(attrs);
}

static int esp32p4_pms_num_indexes(IOMMUMemoryRegion *iommu)
{
    return NUM_IDX;
}

MemoryRegion *esp32p4_pms_bus(ESP32P4PmsState *s)
{
    return &s->bus;
}

AddressSpace *esp32p4_pms_bus_as(ESP32P4PmsState *s)
{
    return &s->bus_as;
}

/* Registers */

static uint64_t esp32p4_pms_read(ESP32P4PmsState *s, int block, hwaddr addr,
                                 unsigned size)
{
    uint32_t reset, mask;

    if (!esp32p4_pms_reg_info(block, addr & ~3, &reset, &mask)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", esp32p4_pms_block_name[block], addr);
        return 0;
    }
    return extract32(s->regs[block][addr / 4], (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.pms] */
static void esp32p4_pms_write(ESP32P4PmsState *s, int block, hwaddr addr,
                              uint64_t value, unsigned size)
{
    uint32_t reset, mask, *reg, v;

    if (!esp32p4_pms_reg_info(block, addr & ~3, &reset, &mask)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", esp32p4_pms_block_name[block], addr);
        return;
    }
    if (s->lp_held && (block == ESP32P4_PMS_MMIO_LP_PERI ||
                       block == ESP32P4_PMS_MMIO_HP2LP_PERI)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while LPPERI_RST_EN_LP_PMS holds it in reset\n",
                      esp32p4_pms_block_name[block], addr);
        return;
    }
    reg = &s->regs[block][addr / 4];
    v = deposit32(*reg, (addr & 3) * 8, size * 8, value) & mask;
    if (v != *reg) {
        *reg = v;
        esp32p4_pms_changed(s);
    }
}

#define ESP32P4_PMS_BLOCK_OPS(n, name)                                      \
    static uint64_t esp32p4_pms_read_##name(void *opaque, hwaddr addr,      \
                                            unsigned size)                  \
    {                                                                       \
        return esp32p4_pms_read(opaque, n, addr, size);                     \
    }                                                                       \
    static void esp32p4_pms_write_##name(void *opaque, hwaddr addr,         \
                                         uint64_t value, unsigned size)     \
    {                                                                       \
        esp32p4_pms_write(opaque, n, addr, value, size);                    \
    }

ESP32P4_PMS_BLOCK_OPS(ESP32P4_PMS_MMIO_HP_PERI, hp_peri)
ESP32P4_PMS_BLOCK_OPS(ESP32P4_PMS_MMIO_LP2HP_PERI, lp2hp_peri)
ESP32P4_PMS_BLOCK_OPS(ESP32P4_PMS_MMIO_DMA, dma)
ESP32P4_PMS_BLOCK_OPS(ESP32P4_PMS_MMIO_LP_PERI, lp_peri)
ESP32P4_PMS_BLOCK_OPS(ESP32P4_PMS_MMIO_HP2LP_PERI, hp2lp_peri)

#define ESP32P4_PMS_OPS(name) {                                             \
        .read = esp32p4_pms_read_##name,                                    \
        .write = esp32p4_pms_write_##name,                                  \
        .endianness = DEVICE_LITTLE_ENDIAN,                                 \
        .valid.min_access_size = 1,                                         \
        .valid.max_access_size = 4,                                         \
        .impl.min_access_size = 1,                                          \
        .impl.max_access_size = 4,                                          \
    }

static const MemoryRegionOps esp32p4_pms_ops[ESP32P4_PMS_NUM_BLOCKS] = {
    [ESP32P4_PMS_MMIO_HP_PERI] = ESP32P4_PMS_OPS(hp_peri),
    [ESP32P4_PMS_MMIO_LP2HP_PERI] = ESP32P4_PMS_OPS(lp2hp_peri),
    [ESP32P4_PMS_MMIO_DMA] = ESP32P4_PMS_OPS(dma),
    [ESP32P4_PMS_MMIO_LP_PERI] = ESP32P4_PMS_OPS(lp_peri),
    [ESP32P4_PMS_MMIO_HP2LP_PERI] = ESP32P4_PMS_OPS(hp2lp_peri),
};

/*
 * LPPERI_RST_EN_LP_PMS: the LP APM's registers take their reset values as
 * the reset is asserted and again as it is released, and while it is held
 * they keep them, taking no writes.
 */
/* [spec:nuos:req:emu.esp32p4.pms] */
static void esp32p4_pms_lp_reset(void *opaque, int n, int level)
{
    ESP32P4PmsState *s = opaque;

    if (s->lp_held == !!level) {
        return;
    }
    s->lp_held = level;
    esp32p4_pms_reset_block(s, ESP32P4_PMS_MMIO_LP_PERI);
    esp32p4_pms_reset_block(s, ESP32P4_PMS_MMIO_HP2LP_PERI);
    esp32p4_pms_changed(s);
}

/*
 * The qtest hook through which a test makes an access as any master, for
 * the masters with no model yet, the DMA engines first: setting "probe" to
 * "<requester_id> <user> r <addr>" or "<requester_id> <user> w <addr>
 * <value>" makes a 32-bit access through the bus with those attributes,
 * and reading it back gives "<MemTxResult> <value read>".
 */
static char *esp32p4_pms_get_probe(Object *obj, Error **errp)
{
    ESP32P4PmsState *s = ESP32P4_PMS(obj);

    return g_strdup(s->probe ? s->probe : "");
}

static void esp32p4_pms_set_probe(Object *obj, const char *value,
                                  Error **errp)
{
    ESP32P4PmsState *s = ESP32P4_PMS(obj);
    g_auto(GStrv) f = g_strsplit(value, " ", 0);
    unsigned int rid, user, addr, data = 0;
    MemTxAttrs attrs = {};
    uint8_t buf[4];
    bool write;
    MemTxResult r;

    if (g_strv_length(f) < 4 ||
        qemu_strtoui(f[0], NULL, 0, &rid) ||
        qemu_strtoui(f[1], NULL, 0, &user) ||
        (strcmp(f[2], "r") && strcmp(f[2], "w")) ||
        qemu_strtoui(f[3], NULL, 0, &addr)) {
        error_setg(errp, "probe: want \"<requester_id> <user> r|w <addr> "
                   "[<value>]\"");
        return;
    }
    write = !strcmp(f[2], "w");
    if (write && (g_strv_length(f) < 5 ||
                  qemu_strtoui(f[4], NULL, 0, &data))) {
        error_setg(errp, "probe: a write wants a value");
        return;
    }
    attrs.requester_id = rid;
    attrs.user = user;
    stl_le_p(buf, data);
    r = address_space_rw(&s->bus_as, addr, attrs, buf, 4, write);
    g_free(s->probe);
    s->probe = g_strdup_printf("%u 0x%08x", r, write ? 0 : ldl_le_p(buf));
}

static void esp32p4_pms_reset_hold(Object *obj, ResetType type)
{
    ESP32P4PmsState *s = ESP32P4_PMS(obj);

    for (int i = 0; i < ESP32P4_PMS_NUM_BLOCKS; i++) {
        esp32p4_pms_reset_block(s, i);
    }
}

static void esp32p4_pms_reset_exit(Object *obj, ResetType type)
{
    esp32p4_pms_changed(ESP32P4_PMS(obj));
}

static void esp32p4_pms_init(Object *obj)
{
    ESP32P4PmsState *s = ESP32P4_PMS(obj);

    /*
     * The bus exists from the start, so that the cores, realized before
     * the blocks the APMs record in, can take it as their memory.
     */
    memory_region_init(&s->bus, obj, "esp32p4.bus", UINT64_MAX);
    memory_region_init_iommu(&s->iommu, sizeof(s->iommu),
                             TYPE_ESP32P4_PMS_IOMMU, obj, "esp32p4.apm",
                             4 * GiB);
    memory_region_add_subregion(&s->bus, 0, MEMORY_REGION(&s->iommu));

    for (int i = 0; i < ESP32P4_PMS_NUM_BLOCKS; i++) {
        memory_region_init_io(&s->iomem[i], obj, &esp32p4_pms_ops[i], s,
                              esp32p4_pms_block_name[i],
                              esp32p4_pms_block_size[i]);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem[i]);
    }
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_pms_lp_reset,
                            ESP32P4_PMS_LP_RESET, 1);
    object_property_add_str(obj, "probe", esp32p4_pms_get_probe,
                            esp32p4_pms_set_probe);
}

static void esp32p4_pms_realize(DeviceState *dev, Error **errp)
{
    ESP32P4PmsState *s = ESP32P4_PMS(dev);

    if (!s->icm || !s->lp_sys) {
        error_setg(errp, "%s: the \"icm\" and \"lp-sys\" links must be set",
                   TYPE_ESP32P4_PMS);
        return;
    }
    /*
     * No owner: an owner's re-entrancy guard would refuse an access the
     * check passes on to the APMs' own registers.
     */
    memory_region_init_io(&s->check, NULL, &esp32p4_pms_check_ops, s,
                          "esp32p4.apm-check", 4 * GiB);
    address_space_init(&s->check_as, &s->check, "esp32p4.apm-check");
    address_space_init(&s->bus_as, &s->bus, "esp32p4.bus");
}

static void esp32p4_pms_finalize(Object *obj)
{
    g_free(ESP32P4_PMS(obj)->probe);
}

static int esp32p4_pms_post_load(void *opaque, int version_id)
{
    esp32p4_pms_changed(opaque);
    return 0;
}

static const VMStateDescription vmstate_esp32p4_pms = {
    .name = TYPE_ESP32P4_PMS,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32p4_pms_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, ESP32P4PmsState, ESP32P4_PMS_NUM_BLOCKS,
                               ESP32P4_PMS_MAX_WORDS),
        VMSTATE_BOOL(lp_held, ESP32P4PmsState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_pms_properties[] = {
    DEFINE_PROP_LINK("icm", ESP32P4PmsState, icm, TYPE_ESP32P4_ICM,
                     ESP32P4IcmState *),
    DEFINE_PROP_LINK("lp-sys", ESP32P4PmsState, lp_sys, TYPE_ESP32P4_LP_SYS,
                     ESP32P4LPSysState *),
};

static void esp32p4_pms_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_pms_realize;
    dc->vmsd = &vmstate_esp32p4_pms;
    rc->phases.hold = esp32p4_pms_reset_hold;
    rc->phases.exit = esp32p4_pms_reset_exit;
    device_class_set_props(dc, esp32p4_pms_properties);
}

static void esp32p4_pms_iommu_class_init(ObjectClass *klass, const void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = esp32p4_pms_translate;
    imrc->attrs_to_index = esp32p4_pms_attrs_to_index;
    imrc->num_indexes = esp32p4_pms_num_indexes;
}

/* [spec:nuos:req:emu.esp32p4.pms] */
static const TypeInfo esp32p4_pms_types[] = {
    {
        .name          = TYPE_ESP32P4_PMS,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4PmsState),
        .instance_init = esp32p4_pms_init,
        .instance_finalize = esp32p4_pms_finalize,
        .class_init    = esp32p4_pms_class_init,
    },
    {
        .name          = TYPE_ESP32P4_PMS_IOMMU,
        .parent        = TYPE_IOMMU_MEMORY_REGION,
        .class_init    = esp32p4_pms_iommu_class_init,
    },
};

DEFINE_TYPES(esp32p4_pms_types)
