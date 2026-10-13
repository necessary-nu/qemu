/*
 * ESP32-P4 permission control: the HP, LP and DMA access permission
 * managers (APMs)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_PMS_H
#define HW_MISC_ESP32P4_PMS_H

#include "exec/memattrs.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_icm.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "qom/object.h"
#include "system/memory.h"

#define TYPE_ESP32P4_PMS "esp32p4-pms"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4PmsState, ESP32P4_PMS)

#define TYPE_ESP32P4_PMS_IOMMU "esp32p4-pms-iommu"

/*
 * The five register blocks, the device's MMIO regions in this order:
 * HP_PERI_PMS (0x500A5000, 2 KiB) and LP2HP_PERI_PMS (0x500A5800, 2 KiB),
 * the HP APM's; HP_DMA_PMS (0x500A6000, 4 KiB), the DMA APM's; and
 * LP_PERI_PMS (0x5012E000, 2 KiB) and HP2LP_PERI_PMS (0x5012E800, 2 KiB),
 * the LP APM's, which also holds the eight configurable peripheral
 * regions both CPU APMs check.
 */
enum {
    ESP32P4_PMS_MMIO_HP_PERI,
    ESP32P4_PMS_MMIO_LP2HP_PERI,
    ESP32P4_PMS_MMIO_DMA,
    ESP32P4_PMS_MMIO_LP_PERI,
    ESP32P4_PMS_MMIO_HP2LP_PERI,
    ESP32P4_PMS_NUM_BLOCKS,
};

#define ESP32P4_HP_PERI_PMS_BASE    0x500a5000
#define ESP32P4_LP2HP_PERI_PMS_BASE 0x500a5800
#define ESP32P4_DMA_PMS_BASE        0x500a6000
#define ESP32P4_LP_PERI_PMS_BASE    0x5012e000
#define ESP32P4_HP2LP_PERI_PMS_BASE 0x5012e800

/* HP_DMA_PMS, the largest block, has 0x224 bytes of registers. */
#define ESP32P4_PMS_MAX_WORDS       (0x224 / 4)

/*
 * The named GPIO input that holds the LP APM's registers, LP_PERI_PMS and
 * HP2LP_PERI_PMS, in reset while high: LPPERI_RST_EN_LP_PMS. The HP and
 * DMA APMs reset only with the system.
 */
#define ESP32P4_PMS_LP_RESET        "lp-reset"

/*
 * Bus masters and how the APMs know them
 * ======================================
 *
 * Every master reaches the bus through the APMs: the cores, through
 * esp32p4_pms_bus(), the MemoryRegion the machine gives each core as its
 * memory, and a DMA master, through esp32p4_pms_bus_as(). An access
 * carries its master's identity in MemTxAttrs.requester_id, and an HP
 * core's user mode in MemTxAttrs.user:
 *
 *   - ESP32P4_REQUESTER_CPU(id): HP core 0 (id 0), HP core 1 (1) or the
 *     LP core (2), each core's TLB fill marking its accesses so. The HP
 *     APM checks them against the core's (and mode's) registers in
 *     HP_PERI_PMS, or LP2HP_PERI_PMS for the LP core, the LP APM against
 *     HP2LP_PERI_PMS, or LP_PERI_PMS for the LP core.
 *
 *   - ESP32P4_REQUESTER_DMA(master, id): DMA master @master, the
 *     ESP32P4_DMA_MASTER_* number of its read and write permission
 *     registers in HP_DMA_PMS (ESP-IDF's apm_master_dma_id_t), with @id
 *     the master ID the AXI ICM records for an access the DMA APM
 *     refuses. A DMA engine sets this on every access it makes:
 *
 *       address_space_read(esp32p4_pms_bus_as(pms), addr,
 *                          esp32p4_dma_attrs(ESP32P4_DMA_MASTER_GMAC,
 *                                            ESP32P4_ICM_ID_GMAC),
 *                          buf, len);
 *
 *     A master on the HP AHB matrix (ESP32P4_DMA_MASTER_ON_AHB()) is
 *     recorded in the ICM's CPU address-hole registers by its 5-bit ID,
 *     ESP32P4_ICM_ID_*: GMAC 5, SDMMC 6, the OTGs 3 and 7, the traces 8
 *     and 9, the TCM and L2MEM monitors 10 and 11, REGDMA 4, and a GDMA-AHB
 *     channel 16 plus the peripheral it serves (I3C 0, UHCI 2, I2S0 3,
 *     I2S1 4, I2S2 5, ADC 8, RMT 10). A master on the AXI matrix (VDMA,
 *     GDMA-AXI, 2D-DMA, the H264 DMA) is recorded in the ICM's system
 *     address-hole registers by its 8-bit ID, its ICM port number (CID)
 *     in bits 7:4 and the unit within it (UID) in bits 3:0
 *     (ESP32P4_ICM_ID_AXI()): VDMA ports 5 and 6, GDMA-AXI 8, 2D-DMA 10,
 *     H264 ports 11 and 12.
 *
 *     A DMA master reaches HP ROM, HP L2MEM and the external memory
 *     windows only, at the addresses the bus matrix gives them
 *     (0x4FC00000, 0x4FF00000, 0x40000000 and 0x48000000), where the DMA
 *     APM checks it against the master's read or write permission for the
 *     32 regions. VDMA's master 1 (ICM port ESP32P4_ICM_CID_VDMA_M1) also
 *     reaches the CSI and DSI bridges' buffers (0x50104000, 0x50105000),
 *     which no APM checks. REGDMA also reaches the HP and LP peripherals
 *     and LP SRAM, through the HP or LP APM, against REGDMA_HP_PERI_PMS
 *     and REGDMA_LP_PERI_PMS.
 *
 * Each master reaches only the slaves its port on the bus matrix has a
 * path to (hw/misc/esp32p4_pms.c): the cores as TRM figure 9.3-1 maps
 * them, the DMA masters as above. An access to an address the master
 * has no path to reaches nothing: it answers with a decode error, and a
 * DMA master's is recorded by the ICM as an illegal access. Anything
 * else, MEMTXATTRS_UNSPECIFIED (qtest, the loader, a debugger) included,
 * passes unchecked. A refused access does not reach its target: a read
 * returns a bus error and no data, a write is dropped and returns a bus
 * error, which a core takes as an access fault and a DMA engine reports
 * as its bus errors.
 */
#define ESP32P4_REQUESTER_VALID     0x8000
#define ESP32P4_REQUESTER_IS_DMA    0x4000
#define ESP32P4_REQUESTER_CPU(id)   (ESP32P4_REQUESTER_VALID | (id))
#define ESP32P4_REQUESTER_DMA(master, id) \
    (ESP32P4_REQUESTER_VALID | ESP32P4_REQUESTER_IS_DMA | \
     ((master) << 8) | ((id) & 0xff))
#define ESP32P4_REQUESTER_MASTER(rid)   (((rid) >> 8) & 0x3f)
#define ESP32P4_REQUESTER_ID(rid)       ((rid) & 0xff)

/* The cores' master IDs. */
#define ESP32P4_MASTER_HP_CORE0     0
#define ESP32P4_MASTER_HP_CORE1     1
#define ESP32P4_MASTER_LP_CORE      2

/* The DMA masters, numbered as their registers in HP_DMA_PMS. */
enum {
    ESP32P4_DMA_MASTER_GDMA_CH0 = 0,
    ESP32P4_DMA_MASTER_GDMA_CH1 = 1,
    ESP32P4_DMA_MASTER_GDMA_CH2 = 2,
    ESP32P4_DMA_MASTER_GDMA_CH3 = 3,
    ESP32P4_DMA_MASTER_AHB_PDMA_ADC = 4,
    ESP32P4_DMA_MASTER_AHB_PDMA_I2S0 = 5,
    ESP32P4_DMA_MASTER_AHB_PDMA_I2S1 = 6,
    ESP32P4_DMA_MASTER_AHB_PDMA_I2S2 = 7,
    ESP32P4_DMA_MASTER_AHB_PDMA_I3C_MST = 8,
    ESP32P4_DMA_MASTER_AHB_PDMA_UHCI0 = 9,
    ESP32P4_DMA_MASTER_AHB_PDMA_RMT = 10,
    ESP32P4_DMA_MASTER_AXI_PDMA_LCDCAM = 13,
    ESP32P4_DMA_MASTER_AXI_PDMA_GPSPI2 = 14,
    ESP32P4_DMA_MASTER_AXI_PDMA_GPSPI3 = 15,
    ESP32P4_DMA_MASTER_AXI_PDMA_PARLIO = 16,
    ESP32P4_DMA_MASTER_AXI_PDMA_AES = 17,
    ESP32P4_DMA_MASTER_AXI_PDMA_SHA = 18,
    ESP32P4_DMA_MASTER_DMA2D_JPEG = 19,
    ESP32P4_DMA_MASTER_USB_OTG_HS = 20,
    ESP32P4_DMA_MASTER_GMAC = 21,
    ESP32P4_DMA_MASTER_SDMMC = 22,
    ESP32P4_DMA_MASTER_USB_OTG_FS = 23,
    ESP32P4_DMA_MASTER_TRACE0 = 24,
    ESP32P4_DMA_MASTER_TRACE1 = 25,
    ESP32P4_DMA_MASTER_L2MEM_MON = 26,
    ESP32P4_DMA_MASTER_SPM_MON = 27,
    ESP32P4_DMA_MASTER_REGDMA = 28,
    ESP32P4_DMA_MASTER_H264 = 30,
    ESP32P4_DMA_MASTER_DMA2D_PPA = 31,
    ESP32P4_DMA_MASTER_DMA2D_DUMMY = 32,
    ESP32P4_DMA_MASTER_AHB_PDMA_DUMMY = 33,
    ESP32P4_DMA_MASTER_AXI_PDMA_DUMMY = 34,
    ESP32P4_DMA_MASTERS = 35,
};

/* Whether DMA master @m is on the HP AHB matrix rather than the AXI one. */
#define ESP32P4_DMA_MASTER_ON_AHB(m) \
    (((m) >= ESP32P4_DMA_MASTER_AHB_PDMA_ADC &&                         \
      (m) <= ESP32P4_DMA_MASTER_AHB_PDMA_RMT) ||                        \
     ((m) >= ESP32P4_DMA_MASTER_USB_OTG_HS &&                           \
      (m) <= ESP32P4_DMA_MASTER_REGDMA) ||                              \
     (m) == ESP32P4_DMA_MASTER_AHB_PDMA_DUMMY)

/* The 32 DMA regions and the 8 configurable peripheral regions. */
#define ESP32P4_DMA_PMS_REGIONS     32
#define ESP32P4_PERI_PMS_REGIONS    8

/* The attributes of an access DMA master @master makes, recorded as @id. */
static inline MemTxAttrs esp32p4_dma_attrs(unsigned master, unsigned id)
{
    return (MemTxAttrs) {
        .requester_id = ESP32P4_REQUESTER_DMA(master, id),
    };
}

/*
 * The records a refusal goes to: the ICM's of the CPUs and AHB masters,
 * the ICM's of the AXI masters, and LP_SYS's.
 */
enum {
    ESP32P4_PMS_REC_ICM_CPU,
    ESP32P4_PMS_REC_ICM_SYS,
    ESP32P4_PMS_REC_LP_SYS,
    ESP32P4_PMS_RECORDS,
};

/* A refusal's record, held until a core's block of code ends. */
typedef struct ESP32P4PmsRecord {
    bool valid;
    bool write;
    bool illegal;
    uint8_t id;
    uint32_t addr;
} ESP32P4PmsRecord;

struct ESP32P4PmsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem[ESP32P4_PMS_NUM_BLOCKS];
    /*
     * The bus as the masters see it: the APMs over the whole address
     * space, passing an access to system memory or to the per-access
     * check.
     */
    MemoryRegion bus;
    IOMMUMemoryRegion iommu;
    AddressSpace bus_as;
    MemoryRegion check;
    AddressSpace check_as;

    ESP32P4IcmState *icm;
    ESP32P4LPSysState *lp_sys;

    uint32_t regs[ESP32P4_PMS_NUM_BLOCKS][ESP32P4_PMS_MAX_WORDS];
    /* LPPERI_RST_EN_LP_PMS is set. */
    bool lp_held;

    /*
     * The first refusal for each record made from the middle of a core's
     * block of translated code (a trace encoder's write as the core
     * reports an instruction), which raises its interrupt only once the
     * block ends, as icount requires.
     */
    ESP32P4PmsRecord pending[ESP32P4_PMS_RECORDS];
    QEMUTimer *record_timer;

    /* The qtest hook's last result, "<MemTxResult> <data>". */
    char *probe;
};

/* The bus the cores take as their "memory". */
MemoryRegion *esp32p4_pms_bus(ESP32P4PmsState *s);

/* The bus as a DMA master's AddressSpace. */
AddressSpace *esp32p4_pms_bus_as(ESP32P4PmsState *s);

#endif
