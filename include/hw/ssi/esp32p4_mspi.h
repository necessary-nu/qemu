/*
 * ESP32-P4 MSPI: the flash and PSRAM memory SPI controllers and their MMUs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_ESP32P4_MSPI_H
#define HW_SSI_ESP32P4_MSPI_H

#include "qemu/units.h"
#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_keymng.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "hw/ssi/ssi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_MSPI "esp32p4-mspi"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4MSPIState, ESP32P4_MSPI)

/*
 * The device's MMIO regions, in this order: SPI0's registers (flash
 * 0x5008C000, PSRAM 0x5008E000), SPI1's (0x5008D000, 0x5008F000), each
 * 4 KiB; and the 64 MiB window the MMU maps, which the bus reaches the
 * memory through (0x80000000 or 0x88000000, and 0x40000000 or 0x48000000
 * for the masters that do not go through the HP cores' caches).
 */
enum {
    ESP32P4_MSPI_MMIO_SPI0,
    ESP32P4_MSPI_MMIO_SPI1,
    ESP32P4_MSPI_MMIO_WINDOW,
};
#define ESP32P4_MSPI_REGS_SIZE      0x1000
#define ESP32P4_MSPI_WINDOW_SIZE    (64 * MiB)
#define ESP32P4_MSPI_NREGS          (0x400 / 4)
#define ESP32P4_MSPI_MMU_ENTRIES    1024
#define ESP32P4_MSPI_PSRAM_MRS      16

/*
 * Named GPIO inputs:
 *
 * "apb-reset", "axi-reset": HP_SYS_CLKRST's RST_EN_MSPI_APB and
 *   RST_EN_MSPI_AXI (RST_EN_DUAL_MSPI_* for the PSRAM controller), high
 *   while held. Either holds the whole controller in reset.
 * "crypto-ctrl": 4 lines, HP_SYSTEM_CRYPTO_CTRL's ENABLE_SPI_MANUAL_ENCRYPT,
 *   ENABLE_DOWNLOAD_DB_ENCRYPT, ENABLE_DOWNLOAD_G0CB_DECRYPT and
 *   ENABLE_DOWNLOAD_MANUAL_ENCRYPT, from the HP system registers (low until
 *   they are modelled).
 * "download-boot": high when the chip booted in joint download mode rather
 *   than SPI boot mode (low until the boot strapping is modelled).
 * "brown-out": a pulse of the brown-out detector's request to close the
 *   flash, which SPI1 records in its BROWN_OUT interrupt.
 *
 * Named GPIO outputs: SSI_GPIO_CS, SPI1's two chip selects, low while
 * selected (the flash is on CS0); and the controller's interrupt, sysbus
 * IRQ 0: FLASH_MSPI_INTR (source 84) for the flash controller,
 * PSRAM_MSPI_INTR (source 109) for the PSRAM one.
 */
#define ESP32P4_MSPI_APB_RESET      "apb-reset"
#define ESP32P4_MSPI_AXI_RESET      "axi-reset"
#define ESP32P4_MSPI_CRYPTO_CTRL    "crypto-ctrl"
#define ESP32P4_MSPI_DOWNLOAD_BOOT  "download-boot"
#define ESP32P4_MSPI_BROWN_OUT      "brown-out"

struct ESP32P4MSPIState {
    SysBusDevice parent_obj;

    MemoryRegion spi0_iomem;
    MemoryRegion spi1_iomem;
    MemoryRegion window;

    /* Properties: the PSRAM controller rather than the flash one. */
    bool psram;
    MemoryRegion *psram_mr;
    ESP32P4EfuseState *efuse;
    /* The Key Manager, whose encryption key it takes when selected. */
    ESP32P4KeyMngState *keymng;

    SSIBus *bus;
    qemu_irq cs[2];
    qemu_irq irq;
    /* The bus clock its registers and AXI port run at, and its core clock. */
    Clock *sys_clk;
    Clock *core_clk;

    uint32_t spi0[ESP32P4_MSPI_NREGS];
    uint32_t spi1[ESP32P4_MSPI_NREGS];
    uint32_t xts_plain[16];
    uint32_t mmu[ESP32P4_MSPI_MMU_ENTRIES];

    /*
     * SPI1's transaction in progress: the CMD and FLASH_SUS_CTRL bits that
     * clear, the FLASH_SUS change and the interrupts that raise when it
     * ends, and its length in core clock cycles, timed once the clock runs.
     */
    QEMUTimer spi1_timer;
    uint32_t spi1_busy_cmd;
    uint32_t spi1_busy_sus;
    uint32_t spi1_end_int;
    int32_t spi1_sus_after;
    uint64_t spi1_cycles;
    bool spi1_waiting;

    /* The manual encryption block: its state and the ciphertext. */
    uint32_t xts_state;
    uint8_t xts_cipher[64];

    /* The PSRAM die: its mode registers, and whether it is half asleep. */
    uint8_t psram_mr_regs[ESP32P4_MSPI_PSRAM_MRS];
    bool psram_halfsleep;

    bool apb_reset;
    bool axi_reset;
    uint32_t crypto_ctrl;
    bool download_boot;
};

/*
 * The AXI port through which the cache, the direct windows and the DMA
 * masters reach the memory: `len` bytes at `addr`, an offset into the
 * 64 MiB virtual window the MMU maps, which may span MMU pages. Returns
 * MEMTX_OK, or MEMTX_ERROR where the controller answers with an error
 * response (SLV_ERR); a read the controller answers without data, as
 * through an invalid MMU entry, reads as zero.
 */
MemTxResult esp32p4_mspi_axi_read(ESP32P4MSPIState *s, uint32_t addr,
                                  uint8_t *buf, uint32_t len);
MemTxResult esp32p4_mspi_axi_write(ESP32P4MSPIState *s, uint32_t addr,
                                   const uint8_t *buf, uint32_t len);

#endif
