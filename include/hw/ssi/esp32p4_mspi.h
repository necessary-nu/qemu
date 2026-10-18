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
#include "hw/gpio/esp32p4_mspi_iomux.h"
#include "hw/misc/esp32p4_keymng.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "hw/ssi/ssi.h"
#include "qemu/notify.h"
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
 * Link "pins": IOMUX_MSPI_PIN (hw/gpio/esp32p4_mspi_iomux.c), the pads its
 * reads go through: the flash's pulls on data lines nothing drives, and
 * the PSRAM's strobes and delay lines in a DDR read. Without it a read
 * sees the memory as it is, and an undriven line reads 0.
 *
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
    /* IOMUX_MSPI_PIN, the pads' configuration its reads go through. */
    ESP32P4MspiIomuxState *pins;

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
    /* When its current phase ends, as the timer cannot tell its callback. */
    int64_t spi1_end_at;
    /* The flash's chip select, held until the transaction's last cycle. */
    bool spi1_cs_held;
    /* The status reads FLASH_WAITI_CTRL makes follow the transaction. */
    bool spi1_waiti;
    /*
     * The hazards logged for SPI0's set-up as its registers last set it
     * (MSPI_LOGGED_*): logged once until software writes SPI0's registers.
     */
    uint32_t spi0_logged;
    /*
     * The cores (bit n for CPU index n) whose access the AXI port holds
     * until it takes accesses again (esp32p4_mspi_axi_ready()), and the
     * DMA masters, each waiting with a Notifier (esp32p4_mspi_dma_wait()),
     * which it has logged since it last took accesses.
     */
    uint32_t axi_waiting;
    NotifierList dma_waiters;
    bool dma_wait_logged;

    /*
     * The flash's timing calibration as TIMING_CALI_UPDATE last took it
     * up: SPI0's and SPI1's TIMING_CALI, SPI0's DIN_MODE and DIN_NUM,
     * which both ports' reads use; and the DIN modes logged as
     * unimplemented, a bit per line.
     */
    uint32_t tune_cali[2];
    uint32_t tune_din_mode;
    uint32_t tune_din_num;
    uint32_t tune_logged;

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
 * Whether the AXI port takes an access by `attrs`'s master now. While the
 * controller's bus clock is stopped or it is held in reset, its AXI slave
 * answers nothing; while its core clock is stopped, or CACHE_FCTRL's
 * AXI_REQ_EN is clear, it takes the access but never runs it on the SPI
 * bus: on the chip the access never completes, and the bus hangs. A
 * core's access waits on the bus (esp32p4_cpu_bus_hold()) until the
 * controller takes accesses again, then is made: returns false with *res
 * the handler's result that holds it. Another master's, which cannot wait
 * here, is refused with an error response (logged): returns false with
 * *res MEMTX_ERROR. Returns true when the access may go on.
 */
bool esp32p4_mspi_axi_ready(ESP32P4MSPIState *s, MemTxAttrs attrs,
                            MemTxResult *res);

/*
 * For a DMA master about to make an access of `len` bytes at `addr` on its
 * bus `as` (with `attrs`, a write if `write`): whether it reaches a memory
 * controller's window (flash or PSRAM) whose AXI port cannot take it now,
 * its bus clock stopped, the controller held in reset, its core clock
 * stopped or AXI_REQ_EN clear (esp32p4_mspi_axi_ready()). On the chip the
 * AXI transaction then waits, and the master with it, until the port
 * takes it: the master does not make the access, and `n` is notified once
 * the port takes accesses again (it is in the controller's list until
 * then, and its callback removes it), when the master makes it. A master
 * that makes an access without asking is answered with an error.
 */
bool esp32p4_mspi_dma_wait(AddressSpace *as, hwaddr addr, hwaddr len,
                           MemTxAttrs attrs, bool write, Notifier *n);

/*
 * Whether any memory controller's AXI port holds accesses now: while none
 * does, no DMA master's access can wait, and a master that works out its
 * next transaction's accesses before making them (to ask
 * esp32p4_mspi_dma_wait() about each) can skip that.
 */
bool esp32p4_mspi_dma_holds(void);

/*
 * A DMA master's wait on a held AXI port, for a master that goes on from
 * a callback: esp32p4_mspi_dma_wait_on() asks esp32p4_mspi_dma_wait()
 * about an access, and once the port takes accesses again `ready` is
 * called from a QEMU timer at that moment (esp32_line_time()), not from
 * within the controller's change, so that the master makes its accesses
 * outside the controller's own MMIO handlers.
 */
typedef struct ESP32P4MspiDmaWait {
    Notifier n;
    QEMUTimer *timer;
    void (*ready)(void *opaque);
    void *opaque;
} ESP32P4MspiDmaWait;

void esp32p4_mspi_dma_wait_init(ESP32P4MspiDmaWait *w,
                                void (*ready)(void *opaque), void *opaque);
bool esp32p4_mspi_dma_wait_on(ESP32P4MspiDmaWait *w, AddressSpace *as,
                              hwaddr addr, hwaddr len, MemTxAttrs attrs,
                              bool write);
/* Forget a wait (the master's reset): `ready` is not called for it. */
void esp32p4_mspi_dma_wait_cancel(ESP32P4MspiDmaWait *w);

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

/*
 * Raise SPI0's and SPI1's SLV_ST_END and MST_ST_END, as transactions
 * ending on both ports do: for the machine's presentation of the state the
 * boot ROM leaves (esp32p4_boot.c), which does not repeat the ROM's
 * transactions.
 */
void esp32p4_mspi_raise_state_ends(ESP32P4MSPIState *s);

/*
 * The three bytes a JEDEC ID read (9Fh) on chip select 0 reads from the
 * flash bus now, the first in bits 23:16: the attached part's ID, or with
 * no flash the data line's level as its pad's pulls hold it, for each
 * bit. For the machine's presentation of the boot ROM's hand-off, which
 * does not repeat the ROM's RDID on the bus.
 */
uint32_t esp32p4_mspi_flash_jedec_id(ESP32P4MSPIState *s);

#endif
