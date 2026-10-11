/*
 * ESP32-P4 MSPI: the flash and PSRAM memory SPI controllers, their MMUs
 * and the external memory encryption
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapters
 * 9.3.3 (External Memory) and 34 (External Memory Encryption and
 * Decryption); ESP-IDF's spi_mem_c_reg.h, spi_mem_s_reg.h,
 * spi1_mem_c_reg.h and spi1_mem_s_reg.h for v3, which give the registers'
 * layout, access and reset values (the TRM does not document the MSPI
 * controllers); ext_mem_defs.h and the HAL's mmu_ll.h, spimem_flash_ll.h,
 * psram_ctrlr_ll.h, mspi_pms_ll.h and spi_flash_encrypted_ll.h, the v3 ROM's
 * flash and cache routines and esp_psram's hex PSRAM driver, which show how
 * software drives them; and espsecure's and espefuse's handling of the
 * XTS-AES keys.
 *
 * The chip has two of these controllers, one per external memory: the
 * flash's (SPI_MEM_C and SPI1_MEM_C) and the PSRAM's (SPI_MEM_S and
 * SPI1_MEM_S), one IP with the same register map. Each has two masters:
 *
 *   - SPI0, the AXI side: the caches' fills and write-backs, the direct
 *     accesses at 0x8xxxxxxx, and the DMA masters, reach the memory
 *     through it and its MMU. CACHE_FCTRL_CLOSE_AXI_INF_EN, set at reset
 *     and cleared by the ROM (Cache_Enable_L2_Cache) and ESP-IDF, closes
 *     the port: every access is answered with an error response.
 *   - SPI1: commands software issues from its registers: the built-in
 *     flash commands of SPI1_MEM_C_CMD (read, page program, sector, block
 *     and chip erase, status register read and write, JEDEC ID, power-down
 *     and release, high performance mode, write enable and disable),
 *     program/erase suspend and resume (FLASH_SUS_CTRL), and user-defined
 *     ones (USR: command, address, dummy, write-data and read-data phases,
 *     with the 64-byte W0..W15 buffer).
 *
 * The MMU: 1024 entries, ITEM_INDEX selecting the one ITEM_CONTENT reads
 * and writes, each mapping one page of the 64 MiB virtual window, 64 KiB
 * at MMU_POWER_CTRL_MMU_PAGE_SIZE's reset value, or 32, 16 or 8 KiB, to a
 * page of the memory. A flash entry is valid with bit 12 set, its page in
 * bits 10:0 and bit 13 its "sensitive" (encrypted) mark; a PSRAM entry
 * with bit 11, its page in bits 9:0, bit 10 its PSRAM target and bit 12
 * its sensitive mark (ext_mem_defs.h). Entries keep bits 13:0. An access
 * through an invalid entry raises AXI_RADDR_ERR (reads) or AXI_WADDR_ERR
 * (writes, the PSRAM controller only) and records the first such address
 * in AXI_ERR_ADDR; it is answered with an error response if
 * AXI_ERR_RESP_EN's MMU_VLD bit for its direction is set, and otherwise
 * reads as zero and writes nothing (the hardware's data is not
 * documented). A write to the flash through the AXI port is dropped, as
 * the controller does not program the flash; ESP-IDF's headers give the
 * flash controller no AXI_WR_FLASH_ERR. Sub-word accesses are refused with
 * an error response while CTRL1_AR/AW_SIZE0_1_SUPPORT_EN are clear. The
 * MMU table is SRAM: the controller's resets leave it, and only a power-on
 * or system reset clears it. The physical page wraps at the memory's size.
 *
 * The flash: a W25Q part on the SSI bus "ssi", selected by chip select 0
 * (SSI_GPIO_CS line 0). SPI1's transactions are driven onto the bus as
 * QEMU's flash models take them, a byte a transfer whatever the line count:
 * the command, the address (USR_ADDR_BITLEN + 1 bits, most significant byte
 * first), the dummy phase as (cycles * address lines + 7) / 8 bytes (as
 * the mode bits of a DIO or QIO read go in the address or dummy phase),
 * then the data. Reads through SPI0 are a READ4 (0x13) of the flash's
 * contents: SPI0's command, line, dummy and timing settings are held but
 * do not change what a read returns, since the TRM does not document
 * them, so a configuration the hardware could not read with reads as one
 * it could. The built-in block erase is the 64 KiB one (0xD8), which the
 * ROM and ESP-IDF erase blocks with; the headers' "(32KB)" contradicts
 * them. A page program takes its length from ADDR[31:24].
 *
 * The PSRAM: the in-package AP Memory 256 Mbit x16 die, on SPI1_MEM_S's
 * chip select 1, as a RAM region (the "psram-mr" link) behind its mode
 * registers. SPI1 speaks its 16-bit OPI commands: synchronous and burst
 * read (0x0000, 0x2020) and write (0x8080, 0xA0A0) at byte addresses, the
 * same the MMU's pages reach, and mode register read (0x4040) and write
 * (0xC0C0) at the register's number. MR1 holds vendor 0x0D (AP Memory),
 * MR2 0xDF (256 Mbit, generation 4, known good die) and MR3 0x20 (slow
 * refresh), the values ESP-IDF's driver reads from this die; MR0, MR4 and
 * MR8, which the driver sets, start at zero, as their reset values are not
 * published. Writing 0xF0 to MR6 puts the die into half sleep, where it
 * keeps its contents and takes no command until a transaction, or SPI1's
 * MISC_CS_KEEP_ACTIVE with its chip select enabled, wakes it.
 *
 * External memory encryption (TRM chapter 34): XTS-AES with 128-byte
 * data units, the tweak (type << 30) | (address & 0x3FFFFF80), type 0 for
 * flash and 1 for PSRAM, and each unit byte-reversed before and after the
 * cipher, as espsecure computes it. The key is the Key Manager's
 * (esp32p4_keymng.c, the "keymng" link) unless KEYMNG_USE_EFUSE_KEY
 * selects the eFuses' for the flash or the PSRAM, as the boot ROM does
 * for an encrypted flash; the Key Manager's is 256 or 512 bits as
 * KEYMNG_FLASH_KEY_LEN or _PSRAM_KEY_LEN says. The eFuses' key (TRM table
 * 34.4-1): blocks of purpose XTS_AES_256_KEY_1 and _2 give a 512-bit key,
 * a missing one counting as zeros; else XTS_AES_128_KEY's block gives a
 * 256-bit key; else it is zero. Each block holds its half byte-reversed,
 * as espefuse burns it. For the PSRAM the PSRAM key purposes (13 to 15)
 * take the place of those when a block has one, as SOC_PSRAM_ENCRYPTION_
 * SEPARATE_KEY allows; the TRM does not say which the hardware prefers.
 *
 *   - Auto decryption and encryption: reads, and the PSRAM's writes,
 *     through an MMU entry marked sensitive are decrypted and encrypted,
 *     while the blocks operate: in SPI boot mode while
 *     EFUSE_SPI_BOOT_CRYPT_CNT has an odd number of bits set; in joint
 *     download boot mode while HP_SYSTEM_CRYPTO_CTRL's
 *     ENABLE_DOWNLOAD_G0CB_DECRYPT (decryption) and _DB_ENCRYPT
 *     (encryption) are set. A partial write re-encrypts the 16-byte blocks
 *     it touches.
 *   - Manual encryption (XTS_*): software writes up to 64 bytes of
 *     plaintext to XTS_PLAIN_n, the size (16 << XTS_LINESIZE), the
 *     destination and the 30-bit physical address, and triggers it;
 *     XTS_STATE goes from 0 to 2 (the calculation takes no time), RELEASE
 *     makes it 3, and DESTROY clears the ciphertext and returns it to 0.
 *     While it is 3, the bytes an SPI1 write-data phase programs at the
 *     target addresses are the ciphertext's. It operates in SPI boot mode
 *     with ENABLE_SPI_MANUAL_ENCRYPT set, and in download mode with
 *     ENABLE_DOWNLOAD_MANUAL_ENCRYPT set and EFUSE_DIS_DOWNLOAD_MANUAL_
 *     ENCRYPT clear; a trigger otherwise, or any action in the wrong
 *     state, is ignored and logged as a guest error.
 *
 *   The Anti-DPA settings (DPA_CTRL, XTS_PSEUDO_ROUND_CONF) change only the
 *   hardware's power profile and timing, and are held.
 *
 * SPI1's MSPI PMS (FMEM_PMSn for flash, SMEM_PMSn for PSRAM, with
 * PMS_REJECT_PM_EN): an SPI1 access with an address that no section
 * covers, that more than one covers, or whose section's secure-mode RD or
 * WR bit refuses it, is not made; PMS_REJECT records the first one and
 * SPI0's PMS_REJECT interrupt raises. A section covers ADDR_S to ADDR_S +
 * SIZE * 4 KiB. Reads are read-data phases with an address, writes
 * write-data phases with an address and the program/erase commands; chip
 * erase and commands without an address are not checked. The non-secure
 * attributes are held: the requesters' security mode is not modelled.
 *
 * Timing: an SPI1 transaction takes the SPI clock cycles of its phases,
 * each at its line count (double rate in DDR mode), with the SPI clock the
 * core clock ("core" clock input) divided by CLOCK_CLKCNT_N + 1 (unless
 * CLK_EQU_SYSCLK), plus the wait after RES, DP and HPM (CS_HOLD_DLY_RES
 * times 4, or 128 with SUS_STATUS's *_DLY_128) and the automatic status
 * read after a program, erase or status write while FLASH_WAITI_CTRL_
 * WAITI_EN is set (the flash model completes programs and erases at once,
 * so one read finds it idle). Its CMD bits clear, its read data reaches
 * W0..W15 and RD_STATUS, and SLV_ST_END, MST_ST_END and, after a program,
 * erase or status write, WPE_END raise when it ends. It waits while the
 * core clock is stopped. SPI0's accesses take no time; each raises SPI0's
 * SLV_ST_END and MST_ST_END.
 *
 * The registers are behind the "sys" bus clock (mapped through the clock
 * and reset tree's gate); the AXI port refuses accesses while it is
 * stopped or the controller is held in reset. The "apb-reset" and
 * "axi-reset" inputs hold the controller in reset.
 *
 * Not modelled: ECC on the external memory (FMEM/SMEM ECC, the PSRAM's
 * 16-to-18 byte mode and its address conversion), logged as unimplemented
 * when enabled; the timing tuning registers, which are held and change
 * nothing as every setting reads correctly; the flash's busy time, which
 * QEMU's flash model does not have, so that suspend and resume find no
 * operation in progress and auto-suspend never acts; SPI0's own user
 * command (USR, which the headers mark reserved).
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "crypto/aes.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "migration/vmstate.h"
#include "system/memory.h"

/*
 * Each register's name, reset value and fields by access, from the v3
 * headers: stored as written (rw), acting when written with 1 and reading
 * 0 (wt), and raw status bits a write of 1 clears (w1c). The rest are
 * read-only to software: the hardware's status, and the reserved fields,
 * which read as their reset values.
 */
typedef struct {
    const char *name;
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
    uint32_t w1c;
} ESP32P4MSPIReg;

#define REG(off, n, rst, w, t, c) \
    [(off) / 4] = { .name = (n), .reset = (rst), .rw = (w), .wt = (t), \
                    .w1c = (c) }

/* SPI_MEM_C, the flash controller's SPI0. */
static const ESP32P4MSPIReg esp32p4_spi0_flash_regs[ESP32P4_MSPI_NREGS] = {
    REG(0x000, "CMD", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x008, "CTRL", 0x802c200c, 0x81bc610e, 0x00000000, 0x00000000),
    REG(0x00c, "CTRL1", 0x28600000, 0x21600003, 0xc0000000, 0x00000000),
    REG(0x010, "CTRL2", 0x01002c21, 0x7f0003ff, 0x80000000, 0x00000000),
    REG(0x014, "CLOCK", 0x00030103, 0x80ffffff, 0x00000000, 0x00000000),
    REG(0x018, "USER", 0x00000000, 0x240002c0, 0x00000000, 0x00000000),
    REG(0x01c, "USER1", 0x5c000047, 0xfc00003f, 0x00000000, 0x00000000),
    REG(0x020, "USER2", 0x70000000, 0xf000ffff, 0x00000000, 0x00000000),
    REG(0x02c, "RD_STATUS", 0x00000000, 0x00ff0000, 0x00000000, 0x00000000),
    REG(0x034, "MISC", 0x00000070, 0x00000670, 0x00000000, 0x00000000),
    REG(0x03c, "CACHE_FCTRL", 0xc0000000, 0x800001ff, 0x00000000, 0x00000000),
    REG(0x040, "CACHE_SCTRL", 0x0055c070, 0x00000000, 0x00000000, 0x00000000),
    REG(0x044, "SRAM_CMD", 0xc0400000, 0x00400000, 0x00000000, 0x00000000),
    REG(0x048, "SRAM_DRD_CMD", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x04c, "SRAM_DWR_CMD", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x050, "SRAM_CLK", 0x00030103, 0x00000000, 0x00000000, 0x00000000),
    REG(0x054, "FSM", 0x00000200, 0x0007ff80, 0x00000000, 0x00000000),
    REG(0x0c0, "INT_ENA", 0x00000000, 0x000000d8, 0x00000000, 0x00000000),
    REG(0x0c4, "INT_CLR", 0x00000000, 0x00000000, 0x000000d8, 0x00000000),
    REG(0x0c8, "INT_RAW", 0x00000000, 0x00000000, 0x00000000, 0x0c0000d8),
    REG(0x0cc, "INT_ST", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d4, "DDR", 0x00003020, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d8, "SMEM_DDR", 0x00003020, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0dc, "DLL_DLY_DB", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0e0, "DLL_DB_ST0", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0e4, "DLL_DB_ST1", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x100, "FMEM__PMS0_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x104, "FMEM__PMS1_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x108, "FMEM__PMS2_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x10c, "FMEM__PMS3_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x110, "FMEM__PMS0_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x114, "FMEM__PMS1_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x118, "FMEM__PMS2_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x11c, "FMEM__PMS3_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x120, "FMEM__PMS0_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x124, "FMEM__PMS1_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x128, "FMEM__PMS2_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x12c, "FMEM__PMS3_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x130, "SMEM_PMS0_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x134, "SMEM_PMS1_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x138, "SMEM_PMS2_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x13c, "SMEM_PMS3_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x140, "SMEM_PMS0_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x144, "SMEM_PMS1_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x148, "SMEM_PMS2_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x14c, "SMEM_PMS3_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x150, "SMEM_PMS0_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x154, "SMEM_PMS1_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x158, "SMEM_PMS2_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x15c, "SMEM_PMS3_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x164, "PMS_REJECT", 0x00000000, 0x08000000, 0x00000000, 0x00000000),
    REG(0x168, "ECC_CTRL", 0x01005000, 0x000c0000, 0x00000000, 0x00000000),
    REG(0x16c, "ECC_ERR_ADDR", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x170, "AXI_ERR_ADDR", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x174, "SMEM_ECC_CTRL", 0x00080000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x178, "SMEM_AXI_ADDR_CTRL",
        0xfc000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x17c, "AXI_ERR_RESP_EN",
        0x00000000, 0x00000f80, 0x00000000, 0x00000000),
    REG(0x180, "TIMING_CALI", 0x00000001, 0x0000001f, 0x00000040, 0x00000000),
    REG(0x184, "DIN_MODE", 0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x188, "DIN_NUM", 0x00000000, 0x0003ffff, 0x00000000, 0x00000000),
    REG(0x18c, "DOUT_MODE", 0x00000000, 0x000001ff, 0x00000000, 0x00000000),
    REG(0x190, "SMEM_TIMING_CALI",
        0x00000001, 0x00000000, 0x00000000, 0x00000000),
    REG(0x194, "SMEM_DIN_MODE", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x198, "SMEM_DIN_NUM", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x19c, "SMEM_DOUT_MODE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1a0, "SMEM_AC", 0x8000b084, 0x00000000, 0x00000000, 0x00000000),
    REG(0x200, "CLOCK_GATE", 0x00000001, 0x00000001, 0x00000000, 0x00000000),
    REG(0x300, "XTS_PLAIN_BASE",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x340, "XTS_LINESIZE", 0x00000000, 0x00000003, 0x00000000, 0x00000000),
    REG(0x344, "XTS_DESTINATION",
        0x00000000, 0x00000001, 0x00000000, 0x00000000),
    REG(0x348, "XTS_PHYSICAL_ADDRESS",
        0x00000000, 0x3fffffff, 0x00000000, 0x00000000),
    REG(0x34c, "XTS_TRIGGER", 0x00000000, 0x00000000, 0x00000001, 0x00000000),
    REG(0x350, "XTS_RELEASE", 0x00000000, 0x00000000, 0x00000001, 0x00000000),
    REG(0x354, "XTS_DESTROY", 0x00000000, 0x00000000, 0x00000001, 0x00000000),
    REG(0x358, "XTS_STATE", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x35c, "XTS_DATE", 0x20210907, 0x3fffffff, 0x00000000, 0x00000000),
    REG(0x37c, "MMU_ITEM_CONTENT",
        0x0000037c, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x380, "MMU_ITEM_INDEX",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x384, "MMU_POWER_CTRL",
        0x13200004, 0x0000001f, 0x00000000, 0x00000000),
    REG(0x388, "DPA_CTRL", 0x0000000f, 0x0000001f, 0x00000000, 0x00000000),
    REG(0x38c, "XTS_PSEUDO_ROUND_CONF",
        0x0000045c, 0x000007ff, 0x00000000, 0x00000000),
    REG(0x3f0, "REGISTERRND_ECO_HIGH",
        0x0000037c, 0x00000000, 0x00000000, 0x00000000),
    REG(0x3f4, "REGISTERRND_ECO_LOW",
        0x0000037c, 0x00000000, 0x00000000, 0x00000000),
    REG(0x3fc, "DATE", 0x02502180, 0x0fffffff, 0x00000000, 0x00000000),
};

/* SPI_MEM_S, the PSRAM controller's SPI0. */
static const ESP32P4MSPIReg esp32p4_spi0_psram_regs[ESP32P4_MSPI_NREGS] = {
    REG(0x000, "CMD", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x008, "CTRL", 0x802c200c, 0xc1bc637f, 0x00000000, 0x00000000),
    REG(0x00c, "CTRL1", 0x28600000, 0x27600003, 0xc0000000, 0x00000000),
    REG(0x010, "CTRL2", 0x01002c21, 0x7f007fff, 0x80000000, 0x00000000),
    REG(0x014, "CLOCK", 0x00030103, 0x80ffffff, 0x00000000, 0x00000000),
    REG(0x018, "USER", 0x00000000, 0x240002c0, 0x00000000, 0x00000000),
    REG(0x01c, "USER1", 0x5c000047, 0xfc00003f, 0x00000000, 0x00000000),
    REG(0x020, "USER2", 0x70000000, 0xf000ffff, 0x00000000, 0x00000000),
    REG(0x02c, "RD_STATUS", 0x00000000, 0x00ff0000, 0x00000000, 0x00000000),
    REG(0x034, "MISC", 0x00000070, 0x00000670, 0x00000000, 0x00000000),
    REG(0x03c, "CACHE_FCTRL", 0xc0000000, 0xc00fffff, 0x00000000, 0x00000000),
    REG(0x040, "CACHE_SCTRL", 0x0055c070, 0x0fffcfff, 0x00000000, 0x00000000),
    REG(0x044, "SRAM_CMD", 0x80c00000, 0xcfffdfff, 0x00000000, 0x00000000),
    REG(0x048, "SRAM_DRD_CMD", 0x00000000, 0xf000ffff, 0x00000000, 0x00000000),
    REG(0x04c, "SRAM_DWR_CMD", 0x00000000, 0xf000ffff, 0x00000000, 0x00000000),
    REG(0x050, "SRAM_CLK", 0x00030103, 0x80ffffff, 0x00000000, 0x00000000),
    REG(0x054, "FSM", 0x00000200, 0x0007ff80, 0x00000000, 0x00000000),
    REG(0x0c0, "INT_ENA", 0x00000000, 0xfc0003f8, 0x00000000, 0x00000000),
    REG(0x0c4, "INT_CLR", 0x00000000, 0x00000000, 0xfc0003f8, 0x00000000),
    REG(0x0c8, "INT_RAW", 0x00000000, 0x00000000, 0x00000000, 0xfc0003f8),
    REG(0x0cc, "INT_ST", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d4, "DDR", 0x00003020, 0x7d3fffff, 0x00000000, 0x00000000),
    REG(0x0d8, "SMEM_DDR", 0x00003020, 0x7d3fffff, 0x00000000, 0x00000000),
    REG(0x0dc, "DLL_DLY_DB", 0x00000000, 0x00003fff, 0x00000000, 0x00000000),
    REG(0x0e0, "DLL_DB_ST0", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0e4, "DLL_DB_ST1", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x100, "FMEM_PMS0_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x104, "FMEM_PMS1_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x108, "FMEM_PMS2_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x10c, "FMEM_PMS3_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x110, "FMEM_PMS0_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x114, "FMEM_PMS1_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x118, "FMEM_PMS2_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x11c, "FMEM_PMS3_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x120, "FMEM_PMS0_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x124, "FMEM_PMS1_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x128, "FMEM_PMS2_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x12c, "FMEM_PMS3_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x130, "SMEM_PMS0_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x134, "SMEM_PMS1_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x138, "SMEM_PMS2_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x13c, "SMEM_PMS3_ATTR",
        0x0000001b, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x140, "SMEM_PMS0_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x144, "SMEM_PMS1_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x148, "SMEM_PMS2_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x14c, "SMEM_PMS3_ADDR",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x150, "SMEM_PMS0_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x154, "SMEM_PMS1_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x158, "SMEM_PMS2_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x15c, "SMEM_PMS3_SIZE",
        0x00001000, 0x00007fff, 0x00000000, 0x00000000),
    REG(0x164, "PMS_REJECT", 0x00000000, 0x08000000, 0x00000000, 0x00000000),
    REG(0x168, "ECC_CTRL", 0x01005000, 0x013ff800, 0x00000000, 0x00000000),
    REG(0x16c, "ECC_ERR_ADDR", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x170, "AXI_ERR_ADDR", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x174, "SMEM_ECC_CTRL", 0x00080000, 0x001e0000, 0x00000000, 0x00000000),
    REG(0x178, "SMEM_AXI_ADDR_CTRL",
        0xfc000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x17c, "AXI_ERR_RESP_EN",
        0x00000000, 0x00000fff, 0x00000000, 0x00000000),
    REG(0x180, "TIMING_CALI", 0x00000001, 0x0000003f, 0x00000040, 0x00000000),
    REG(0x184, "DIN_MODE", 0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x188, "DIN_NUM", 0x00000000, 0x0003ffff, 0x00000000, 0x00000000),
    REG(0x18c, "DOUT_MODE", 0x00000000, 0x000001ff, 0x00000000, 0x00000000),
    REG(0x190, "SMEM_TIMING_CALI",
        0x00000001, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x194, "SMEM_DIN_MODE", 0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x198, "SMEM_DIN_NUM", 0x00000000, 0x0003ffff, 0x00000000, 0x00000000),
    REG(0x19c, "SMEM_DOUT_MODE",
        0x00000000, 0x000001ff, 0x00000000, 0x00000000),
    REG(0x1a0, "SMEM_AC", 0x8000b084, 0xfe01ffff, 0x00000000, 0x00000000),
    REG(0x1a4, "SMEM_DIN_HEX_MODE",
        0x00000000, 0x07ffffff, 0x00000000, 0x00000000),
    REG(0x1a8, "SMEM_DIN_HEX_NUM",
        0x00000000, 0x0003ffff, 0x00000000, 0x00000000),
    REG(0x1ac, "SMEM_DOUT_HEX_MODE",
        0x00000000, 0x000001ff, 0x00000000, 0x00000000),
    REG(0x200, "CLOCK_GATE", 0x00000001, 0x00000001, 0x00000000, 0x00000000),
    REG(0x300, "XTS_PLAIN_BASE",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x340, "XTS_LINESIZE", 0x00000000, 0x00000003, 0x00000000, 0x00000000),
    REG(0x344, "XTS_DESTINATION",
        0x00000000, 0x00000001, 0x00000000, 0x00000000),
    REG(0x348, "XTS_PHYSICAL_ADDRESS",
        0x00000000, 0x3fffffff, 0x00000000, 0x00000000),
    REG(0x34c, "XTS_TRIGGER", 0x00000000, 0x00000000, 0x00000001, 0x00000000),
    REG(0x350, "XTS_RELEASE", 0x00000000, 0x00000000, 0x00000001, 0x00000000),
    REG(0x354, "XTS_DESTROY", 0x00000000, 0x00000000, 0x00000001, 0x00000000),
    REG(0x358, "XTS_STATE", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x35c, "XTS_DATE", 0x20210907, 0x3fffffff, 0x00000000, 0x00000000),
    REG(0x37c, "MMU_ITEM_CONTENT",
        0x0000037c, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x380, "MMU_ITEM_INDEX",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x384, "MMU_POWER_CTRL",
        0x13200004, 0x7fff001f, 0x00000000, 0x00000000),
    REG(0x388, "DPA_CTRL", 0x0000000f, 0x0000001f, 0x00000000, 0x00000000),
    REG(0x38c, "XTS_PSEUDO_ROUND_CONF",
        0x0000045c, 0x000007ff, 0x00000000, 0x00000000),
    REG(0x3f0, "REGISTERRND_ECO_HIGH",
        0x0000037c, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x3f4, "REGISTERRND_ECO_LOW",
        0x0000037c, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x3fc, "DATE", 0x02501240, 0x0fffffff, 0x00000000, 0x00000000),
};

/* SPI1_MEM_C, the flash controller's SPI1. */
static const ESP32P4MSPIReg esp32p4_spi1_flash_regs[ESP32P4_MSPI_NREGS] = {
    REG(0x000, "CMD", 0x00000000, 0xfffe0000, 0x00000000, 0x00000000),
    REG(0x004, "ADDR", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x008, "CTRL", 0x002ca00c, 0x01fce10c, 0x00000000, 0x00000000),
    REG(0x00c, "CTRL1", 0x001ffffc, 0x009fffff, 0x00000000, 0x00000000),
    REG(0x010, "CTRL2", 0x00000000, 0x00000000, 0x80000000, 0x00000000),
    REG(0x014, "CLOCK", 0x00030103, 0x80ffffff, 0x00000000, 0x00000000),
    REG(0x018, "USER", 0x80000000, 0xfc00f200, 0x00000000, 0x00000000),
    REG(0x01c, "USER1", 0x5c000007, 0xfc00003f, 0x00000000, 0x00000000),
    REG(0x020, "USER2", 0x70000000, 0xf000ffff, 0x00000000, 0x00000000),
    REG(0x024, "MOSI_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
    REG(0x028, "MISO_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
    REG(0x02c, "RD_STATUS", 0x00000000, 0x00ffffff, 0x00000000, 0x00000000),
    REG(0x034, "MISC", 0x00000002, 0x00000603, 0x00000000, 0x00000000),
    REG(0x038, "TX_CRC", 0xffffffff, 0x00000000, 0x00000000, 0x00000000),
    REG(0x03c, "CACHE_FCTRL", 0x00000000, 0x000001fa, 0x00000000, 0x00000000),
    REG(0x058, "W0", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x05c, "W1", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x060, "W2", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x064, "W3", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x068, "W4", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x06c, "W5", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x070, "W6", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x074, "W7", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x078, "W8", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x07c, "W9", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x080, "W10", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x084, "W11", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x088, "W12", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x08c, "W13", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x090, "W14", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x094, "W15", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x098, "FLASH_WAITI_CTRL",
        0x00050001, 0xfffffe1f, 0x00000000, 0x00000000),
    REG(0x09c, "FLASH_SUS_CTRL",
        0x08002000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0a0, "FLASH_SUS_CMD", 0x00057575, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0a4, "SUS_STATUS", 0x7a7a0000, 0xffff80ff, 0x00000000, 0x00000000),
    REG(0x0c0, "INT_ENA", 0x00000000, 0x0000041f, 0x00000000, 0x00000000),
    REG(0x0c4, "INT_CLR", 0x00000000, 0x00000000, 0x0000041f, 0x00000000),
    REG(0x0c8, "INT_RAW", 0x00000000, 0x00000000, 0x00000000, 0x0000041f),
    REG(0x0cc, "INT_ST", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d4, "DDR", 0x00000020, 0x00000000, 0x00000000, 0x00000000),
    REG(0x180, "TIMING_CALI", 0x00000000, 0x0000001e, 0x00000000, 0x00000000),
    REG(0x200, "CLOCK_GATE", 0x00000001, 0x00000001, 0x00000000, 0x00000000),
    REG(0x3fc, "DATE", 0x02501130, 0x0fffffff, 0x00000000, 0x00000000),
};

/* SPI1_MEM_S, the PSRAM controller's SPI1. */
static const ESP32P4MSPIReg esp32p4_spi1_psram_regs[ESP32P4_MSPI_NREGS] = {
    REG(0x000, "CMD", 0x00000000, 0xfffe0000, 0x00000000, 0x00000000),
    REG(0x004, "ADDR", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x008, "CTRL", 0x002ca00c, 0x01fcef7c, 0x00000000, 0x00000000),
    REG(0x00c, "CTRL1", 0x001ffffc, 0x009fffff, 0x00000000, 0x00000000),
    REG(0x010, "CTRL2", 0x00000000, 0x00000000, 0x80000000, 0x00000000),
    REG(0x014, "CLOCK", 0x00030103, 0x80ffffff, 0x00000000, 0x00000000),
    REG(0x018, "USER", 0x80000000, 0xff00f200, 0x00000000, 0x00000000),
    REG(0x01c, "USER1", 0x5c000007, 0xfc00003f, 0x00000000, 0x00000000),
    REG(0x020, "USER2", 0x70000000, 0xf000ffff, 0x00000000, 0x00000000),
    REG(0x024, "MOSI_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
    REG(0x028, "MISO_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
    REG(0x02c, "RD_STATUS", 0x00000000, 0x00ffffff, 0x00000000, 0x00000000),
    REG(0x034, "MISC", 0x00000002, 0x00000603, 0x00000000, 0x00000000),
    REG(0x038, "TX_CRC", 0xffffffff, 0x00000000, 0x00000000, 0x00000000),
    REG(0x03c, "CACHE_FCTRL", 0x00000000, 0x000001fa, 0x00000000, 0x00000000),
    REG(0x058, "W0", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x05c, "W1", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x060, "W2", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x064, "W3", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x068, "W4", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x06c, "W5", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x070, "W6", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x074, "W7", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x078, "W8", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x07c, "W9", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x080, "W10", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x084, "W11", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x088, "W12", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x08c, "W13", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x090, "W14", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x094, "W15", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x098, "FLASH_WAITI_CTRL",
        0x00050001, 0xfffffe1f, 0x00000000, 0x00000000),
    REG(0x09c, "FLASH_SUS_CTRL",
        0x08002000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0a0, "FLASH_SUS_CMD", 0x00057575, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0a4, "SUS_STATUS", 0x7a7a0000, 0xffff80ff, 0x00000000, 0x00000000),
    REG(0x0c0, "INT_ENA", 0x00000000, 0x0000041f, 0x00000000, 0x00000000),
    REG(0x0c4, "INT_CLR", 0x00000000, 0x00000000, 0x0000041f, 0x00000000),
    REG(0x0c8, "INT_RAW", 0x00000000, 0x00000000, 0x00000000, 0x0000041f),
    REG(0x0cc, "INT_ST", 0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d4, "DDR", 0x00000020, 0x7d3fcfff, 0x00000000, 0x00000000),
    REG(0x180, "TIMING_CALI", 0x00000000, 0x0000001e, 0x00000000, 0x00000000),
    REG(0x200, "CLOCK_GATE", 0x00000001, 0x00000001, 0x00000000, 0x00000000),
    REG(0x3fc, "DATE", 0x02501130, 0x0fffffff, 0x00000000, 0x00000000),
};

#undef REG

/* SPI0's registers. */
#define R0_CTRL1                (0x00c / 4)
#define R0_CACHE_FCTRL          (0x03c / 4)
#define R0_INT_ENA              (0x0c0 / 4)
#define R0_INT_CLR              (0x0c4 / 4)
#define R0_INT_RAW              (0x0c8 / 4)
#define R0_INT_ST               (0x0cc / 4)
#define R0_FMEM_PMS_ATTR        (0x100 / 4)
#define R0_FMEM_PMS_ADDR        (0x110 / 4)
#define R0_FMEM_PMS_SIZE        (0x120 / 4)
#define R0_SMEM_PMS_ATTR        (0x130 / 4)
#define R0_SMEM_PMS_ADDR        (0x140 / 4)
#define R0_SMEM_PMS_SIZE        (0x150 / 4)
#define R0_PMS_REJECT           (0x164 / 4)
#define R0_ECC_CTRL             (0x168 / 4)
#define R0_AXI_ERR_ADDR         (0x170 / 4)
#define R0_SMEM_ECC_CTRL        (0x174 / 4)
#define R0_AXI_ERR_RESP_EN      (0x17c / 4)
#define R0_XTS_PLAIN            (0x300 / 4)
#define R0_XTS_LINESIZE         (0x340 / 4)
#define R0_XTS_DESTINATION      (0x344 / 4)
#define R0_XTS_PHYSICAL_ADDRESS (0x348 / 4)
#define R0_XTS_TRIGGER          (0x34c / 4)
#define R0_XTS_RELEASE          (0x350 / 4)
#define R0_XTS_DESTROY          (0x354 / 4)
#define R0_XTS_STATE            (0x358 / 4)
#define R0_MMU_ITEM_CONTENT     (0x37c / 4)
#define R0_MMU_ITEM_INDEX       (0x380 / 4)
#define R0_MMU_POWER_CTRL       (0x384 / 4)
#define R0_CTRL2                (0x010 / 4)

#define CTRL1_AR_SIZE0_1_SUPPORT_EN BIT(21)
#define CTRL1_AW_SIZE0_1_SUPPORT_EN BIT(22)
#define CTRL2_ECC_16TO18_BYTE_EN    BIT(14)
#define CACHE_FCTRL_CLOSE_AXI_INF_EN BIT(31)

#define INT0_SLV_ST_END         BIT(3)
#define INT0_MST_ST_END         BIT(4)
#define INT0_PMS_REJECT         BIT(6)
#define INT0_AXI_RADDR_ERR      BIT(7)
#define INT0_AXI_WR_FLASH_ERR   BIT(8)
#define INT0_AXI_WADDR_ERR      BIT(9)
#define INT0_AXI_ERRS (INT0_AXI_RADDR_ERR | INT0_AXI_WR_FLASH_ERR | \
                       INT0_AXI_WADDR_ERR)

#define PMS_REJECT_ADDR_MASK    0x07ffffffu
#define PMS_REJECT_PM_EN        BIT(27)
#define PMS_REJECT_LD           BIT(28)
#define PMS_REJECT_ST           BIT(29)
#define PMS_REJECT_MULTI_HIT    BIT(30)
#define PMS_REJECT_IVD          BIT(31)
#define PMS_ATTR_RD             BIT(0)
#define PMS_ATTR_WR             BIT(1)
#define PMS_ATTR_ECC            BIT(2)
#define PMS_ATTR_NONSECURE_ECC  BIT(5)
#define PMS_REGION_ALIGN        0x1000

#define ECC_CTRL_FMEM_ECC_ADDR_EN   BIT(20)
#define ECC_CTRL_USR_ECC_ADDR_EN    BIT(21)
#define SMEM_ECC_CTRL_ECC_ADDR_EN   BIT(20)

#define RESP_EN_AW_MMU_VLD      BIT(0)
#define RESP_EN_AR_MMU_VLD      BIT(7)

#define MMU_POWER_FORCE_PD      BIT(1)
#define MMU_ENTRY_MASK          0x3fffu
#define MMU_FLASH_PAGE_MASK     0x7ffu
#define MMU_FLASH_VALID         BIT(12)
#define MMU_FLASH_SENSITIVE     BIT(13)
#define MMU_PSRAM_PAGE_MASK     0x3ffu
#define MMU_ACCESS_PSRAM        BIT(10)
#define MMU_PSRAM_VALID         BIT(11)
#define MMU_PSRAM_SENSITIVE     BIT(12)
#define MMU_MAX_PAGE_SHIFT      16

/* The manual encryption block's states (XTS_STATE). */
enum {
    XTS_IDLE,
    XTS_BUSY,
    XTS_DONE,
    XTS_RELEASED,
};

/* SPI1's registers. */
#define R1_CMD                  (0x000 / 4)
#define R1_ADDR                 (0x004 / 4)
#define R1_CTRL                 (0x008 / 4)
#define R1_CTRL1                (0x00c / 4)
#define R1_CTRL2                (0x010 / 4)
#define R1_CLOCK                (0x014 / 4)
#define R1_USER                 (0x018 / 4)
#define R1_USER1                (0x01c / 4)
#define R1_USER2                (0x020 / 4)
#define R1_MOSI_DLEN            (0x024 / 4)
#define R1_MISO_DLEN            (0x028 / 4)
#define R1_RD_STATUS            (0x02c / 4)
#define R1_MISC                 (0x034 / 4)
#define R1_CACHE_FCTRL          (0x03c / 4)
#define R1_W0                   (0x058 / 4)
#define R1_FLASH_WAITI_CTRL     (0x098 / 4)
#define R1_FLASH_SUS_CTRL       (0x09c / 4)
#define R1_FLASH_SUS_CMD        (0x0a0 / 4)
#define R1_SUS_STATUS           (0x0a4 / 4)
#define R1_INT_ENA              (0x0c0 / 4)
#define R1_INT_CLR              (0x0c4 / 4)
#define R1_INT_RAW              (0x0c8 / 4)
#define R1_INT_ST               (0x0cc / 4)
#define R1_DDR                  (0x0d4 / 4)

#define CMD_FLASH_PE            BIT(17)
#define CMD_USR                 BIT(18)
#define CMD_FLASH_HPM           BIT(19)
#define CMD_FLASH_RES           BIT(20)
#define CMD_FLASH_DP            BIT(21)
#define CMD_FLASH_CE            BIT(22)
#define CMD_FLASH_BE            BIT(23)
#define CMD_FLASH_SE            BIT(24)
#define CMD_FLASH_PP            BIT(25)
#define CMD_FLASH_WRSR          BIT(26)
#define CMD_FLASH_RDSR          BIT(27)
#define CMD_FLASH_RDID          BIT(28)
#define CMD_FLASH_WRDI          BIT(29)
#define CMD_FLASH_WREN          BIT(30)
#define CMD_FLASH_READ          BIT(31)
#define CMD_OPS                 0xfffc0000u

#define CTRL_FDOUT_OCT          BIT(4)
#define CTRL_FDIN_OCT           BIT(5)
#define CTRL_FADDR_OCT          BIT(6)
#define CTRL_FCMD_QUAD          BIT(8)
#define CTRL_FCMD_OCT           BIT(9)
#define CTRL_FASTRD_MODE        BIT(13)
#define CTRL_FREAD_DUAL         BIT(14)
#define CTRL_RESANDRES          BIT(15)
#define CTRL_FREAD_QUAD         BIT(20)
#define CTRL_WRSR_2B            BIT(22)
#define CTRL_FREAD_DIO          BIT(23)
#define CTRL_FREAD_QIO          BIT(24)

#define CTRL1_CS_HOLD_DLY_RES(v)    extract32((v), 2, 10)
#define CTRL1_CS_HOLD_DLY_PER(v)    extract32((v), 12, 9)
#define CTRL1_CS_HOLD_DLY_PER_EN    BIT(23)
#define CTRL2_SYNC_RESET            BIT(31)
#define CLOCK_CLKCNT_N(v)           extract32((v), 16, 8)
#define CLOCK_CLK_EQU_SYSCLK        BIT(31)

#define USER_FWRITE_DUAL        BIT(12)
#define USER_FWRITE_QUAD        BIT(13)
#define USER_FWRITE_DIO         BIT(14)
#define USER_FWRITE_QIO         BIT(15)
#define USER_MISO_HIGHPART      BIT(24)
#define USER_MOSI_HIGHPART      BIT(25)
#define USER_USR_MOSI           BIT(27)
#define USER_USR_MISO           BIT(28)
#define USER_USR_DUMMY          BIT(29)
#define USER_USR_ADDR           BIT(30)
#define USER_USR_COMMAND        BIT(31)
#define USER1_DUMMY_CYCLELEN(v) extract32((v), 0, 6)
#define USER1_ADDR_BITLEN(v)    extract32((v), 26, 6)
#define USER2_COMMAND_VALUE(v)  extract32((v), 0, 16)
#define USER2_COMMAND_BITLEN(v) extract32((v), 28, 4)
#define DLEN(v)                 extract32((v), 0, 10)
#define RD_STATUS_STATUS        0x0000ffffu
#define MISC_CS0_DIS            BIT(0)
#define MISC_CS1_DIS            BIT(1)
#define MISC_CS_KEEP_ACTIVE     BIT(10)
#define FCTRL_FDIN_DUAL         BIT(3)
#define FCTRL_FDOUT_DUAL        BIT(4)
#define FCTRL_FADDR_DUAL        BIT(5)
#define FCTRL_FDIN_QUAD         BIT(6)
#define FCTRL_FDOUT_QUAD        BIT(7)
#define FCTRL_FADDR_QUAD        BIT(8)
#define WAITI_EN                BIT(0)
#define WAITI_DUMMY             BIT(1)
#define WAITI_CMD_2B            BIT(9)
#define WAITI_DUMMY_CYCLELEN(v) extract32((v), 10, 6)
#define SUS_FLASH_PER           BIT(0)
#define SUS_FLASH_PES           BIT(1)
#define SUS_FLASH_PER_WAIT_EN   BIT(2)
#define SUS_FLASH_PES_WAIT_EN   BIT(3)
#define SUS_PES_COMMAND(v)      extract32((v), 0, 16)
#define SUS_STATUS_FLASH_SUS    BIT(0)
#define SUS_STATUS_HPM_DLY_128  BIT(2)
#define SUS_STATUS_RES_DLY_128  BIT(3)
#define SUS_STATUS_DP_DLY_128   BIT(4)
#define SUS_STATUS_PER_DLY_128  BIT(5)
#define SUS_STATUS_PES_DLY_128  BIT(6)
#define SUS_STATUS_PESR_CMD_2B  BIT(15)
#define SUS_PER_COMMAND(v)      extract32((v), 16, 16)
#define DDR_FMEM_DDR_EN         BIT(0)
#define DDR_FMEM_DDR_CMD_DIS    BIT(4)

#define INT1_PER_END            BIT(0)
#define INT1_PES_END            BIT(1)
#define INT1_WPE_END            BIT(2)
#define INT1_SLV_ST_END         BIT(3)
#define INT1_MST_ST_END         BIT(4)
#define INT1_BROWN_OUT          BIT(10)

/* The flash commands SPI1's built-in operations and SPI0's reads send. */
#define FLASH_WRSR      0x01
#define FLASH_PP        0x02
#define FLASH_READ      0x03
#define FLASH_WRDI      0x04
#define FLASH_RDSR      0x05
#define FLASH_WREN      0x06
#define FLASH_FAST_READ 0x0b
#define FLASH_READ4     0x13
#define FLASH_SE        0x20
#define FLASH_DOR       0x3b
#define FLASH_CE        0x60
#define FLASH_QOR       0x6b
#define FLASH_RDID      0x9f
#define FLASH_HPM       0xa3
#define FLASH_RES       0xab
#define FLASH_DP        0xb9
#define FLASH_DIOR      0xbb
#define FLASH_BE        0xd8
#define FLASH_QIOR      0xeb

/* The PSRAM die's OPI commands. */
#define PSRAM_SYNC_READ     0x0000
#define PSRAM_BURST_READ    0x2020
#define PSRAM_REG_READ      0x4040
#define PSRAM_SYNC_WRITE    0x8080
#define PSRAM_BURST_WRITE   0xa0a0
#define PSRAM_REG_WRITE     0xc0c0
#define PSRAM_HALFSLEEP     0xf0

/* The key purposes of TRM table 9.4-2 and ESP-IDF's for v3. */
#define PURPOSE_XTS_AES_256_KEY_1       2
#define PURPOSE_XTS_AES_256_KEY_2       3
#define PURPOSE_XTS_AES_128_KEY         4
#define PURPOSE_XTS_AES_256_PSRAM_KEY_1 13
#define PURPOSE_XTS_AES_256_PSRAM_KEY_2 14
#define PURPOSE_XTS_AES_128_PSRAM_KEY   15

/* HP_SYSTEM_CRYPTO_CTRL, as the "crypto-ctrl" lines give it. */
#define CRYPTO_ENABLE_SPI_MANUAL_ENCRYPT        BIT(0)
#define CRYPTO_ENABLE_DOWNLOAD_DB_ENCRYPT       BIT(1)
#define CRYPTO_ENABLE_DOWNLOAD_G0CB_DECRYPT     BIT(2)
#define CRYPTO_ENABLE_DOWNLOAD_MANUAL_ENCRYPT   BIT(3)

#define XTS_UNIT        128
#define XTS_BLOCK       16

static const ESP32P4MSPIReg *spi0_regs(ESP32P4MSPIState *s)
{
    return s->psram ? esp32p4_spi0_psram_regs : esp32p4_spi0_flash_regs;
}

static const ESP32P4MSPIReg *spi1_regs(ESP32P4MSPIState *s)
{
    return s->psram ? esp32p4_spi1_psram_regs : esp32p4_spi1_flash_regs;
}

static const char *mspi_name(ESP32P4MSPIState *s)
{
    return s->psram ? "esp32p4.psram-mspi" : "esp32p4.flash-mspi";
}

static bool mspi_in_reset(ESP32P4MSPIState *s)
{
    return s->apb_reset || s->axi_reset;
}

static bool mspi_bus_running(ESP32P4MSPIState *s)
{
    return !mspi_in_reset(s) && clock_is_enabled(s->sys_clk);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void mspi_update_irq(ESP32P4MSPIState *s)
{
    bool level = (s->spi0[R0_INT_RAW] & s->spi0[R0_INT_ENA]) ||
                 (s->spi1[R1_INT_RAW] & s->spi1[R1_INT_ENA]);

    qemu_set_irq(s->irq, level);
}

/* Raise the SPI0 interrupt bits of `bits` this controller implements. */
static void spi0_raise(ESP32P4MSPIState *s, uint32_t bits)
{
    s->spi0[R0_INT_RAW] |= bits & spi0_regs(s)[R0_INT_RAW].w1c;
}

static void spi1_raise(ESP32P4MSPIState *s, uint32_t bits)
{
    s->spi1[R1_INT_RAW] |= bits & spi1_regs(s)[R1_INT_RAW].w1c;
}

/* External memory encryption */

typedef struct {
    AES_KEY data_enc;
    AES_KEY data_dec;
    AES_KEY tweak_enc;
} ESP32P4XtsKey;

/* A key block's 32 bytes in the order the XTS key takes them. */
static void xts_key_block(ESP32P4MSPIState *s, int block, uint8_t out[32])
{
    uint8_t raw[32];
    int i;

    if (block < 0) {
        memset(out, 0, 32);
        return;
    }
    esp32p4_efuse_key(s->efuse, block, raw);
    for (i = 0; i < 32; i++) {
        out[i] = raw[31 - i];
    }
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void xts_key(ESP32P4MSPIState *s, bool psram, ESP32P4XtsKey *k)
{
    int a = -1, b = -1, c = -1;
    uint8_t key[64];
    unsigned bits;

    if (s->keymng && esp32p4_keymng_xts_key(s->keymng, psram, key, &bits)) {
        if (bits == 512) {
            AES_set_encrypt_key(key, 256, &k->data_enc);
            AES_set_decrypt_key(key, 256, &k->data_dec);
            AES_set_encrypt_key(key + 32, 256, &k->tweak_enc);
        } else {
            AES_set_encrypt_key(key, 128, &k->data_enc);
            AES_set_decrypt_key(key, 128, &k->data_dec);
            AES_set_encrypt_key(key + 16, 128, &k->tweak_enc);
        }
        return;
    }
    if (psram) {
        a = esp32p4_efuse_find_purpose(s->efuse,
                                       PURPOSE_XTS_AES_256_PSRAM_KEY_1);
        b = esp32p4_efuse_find_purpose(s->efuse,
                                       PURPOSE_XTS_AES_256_PSRAM_KEY_2);
        c = esp32p4_efuse_find_purpose(s->efuse,
                                       PURPOSE_XTS_AES_128_PSRAM_KEY);
    }
    if (a < 0 && b < 0 && c < 0) {
        a = esp32p4_efuse_find_purpose(s->efuse, PURPOSE_XTS_AES_256_KEY_1);
        b = esp32p4_efuse_find_purpose(s->efuse, PURPOSE_XTS_AES_256_KEY_2);
        c = esp32p4_efuse_find_purpose(s->efuse, PURPOSE_XTS_AES_128_KEY);
    }
    if (a >= 0 || b >= 0) {
        xts_key_block(s, a, key);
        xts_key_block(s, b, key + 32);
        AES_set_encrypt_key(key, 256, &k->data_enc);
        AES_set_decrypt_key(key, 256, &k->data_dec);
        AES_set_encrypt_key(key + 32, 256, &k->tweak_enc);
    } else {
        xts_key_block(s, c, key);
        AES_set_encrypt_key(key, 128, &k->data_enc);
        AES_set_decrypt_key(key, 128, &k->data_dec);
        AES_set_encrypt_key(key + 16, 128, &k->tweak_enc);
    }
}

/* Multiply an XTS tweak by the primitive element of GF(2^128). */
static void xts_mul_alpha(uint8_t t[XTS_BLOCK])
{
    uint8_t carry = t[XTS_BLOCK - 1] >> 7;
    int i;

    for (i = XTS_BLOCK - 1; i > 0; i--) {
        t[i] = (t[i] << 1) | (t[i - 1] >> 7);
    }
    t[0] = (t[0] << 1) ^ (carry ? 0x87 : 0);
}

/*
 * En- or decrypt the 16-byte block at `addr` (16-byte aligned) of memory
 * type `type`. The unit is byte-reversed around the cipher, so this block
 * is the reversed unit's block 7 - its index, its bytes reversed.
 */
static void xts_crypt_block(const ESP32P4XtsKey *k, uint32_t type,
                            uint32_t addr, uint8_t data[XTS_BLOCK], bool enc)
{
    uint8_t t[XTS_BLOCK] = { 0 };
    uint8_t x[XTS_BLOCK];
    unsigned j = (XTS_UNIT / XTS_BLOCK - 1) - (addr % XTS_UNIT) / XTS_BLOCK;
    unsigned i;

    stl_le_p(t, (type << 30) | (addr & 0x3fffff80u));
    AES_encrypt(t, t, &k->tweak_enc);
    for (i = 0; i < j; i++) {
        xts_mul_alpha(t);
    }
    for (i = 0; i < XTS_BLOCK; i++) {
        x[i] = data[XTS_BLOCK - 1 - i] ^ t[i];
    }
    if (enc) {
        AES_encrypt(x, x, &k->data_enc);
    } else {
        AES_decrypt(x, x, &k->data_dec);
    }
    for (i = 0; i < XTS_BLOCK; i++) {
        data[XTS_BLOCK - 1 - i] = x[i] ^ t[i];
    }
}

/* En- or decrypt `len` bytes at `addr`, both multiples of 16. */
static void xts_crypt(const ESP32P4XtsKey *k, uint32_t type, uint32_t addr,
                      uint8_t *buf, uint32_t len, bool enc)
{
    uint32_t i;

    for (i = 0; i < len; i += XTS_BLOCK) {
        xts_crypt_block(k, type, addr + i, buf + i, enc);
    }
}

static bool xts_crypt_cnt_odd(ESP32P4MSPIState *s)
{
    /* EFUSE_SPI_BOOT_CRYPT_CNT, RD_REPEAT_DATA1[20:18]. */
    uint32_t cnt = extract32(esp32p4_efuse_word(s->efuse, 0, 2), 18, 3);

    return ctpop32(cnt) & 1;
}

/* Whether the auto decryption block operates. */
static bool xts_auto_decrypt(ESP32P4MSPIState *s)
{
    if (s->download_boot) {
        return s->crypto_ctrl & CRYPTO_ENABLE_DOWNLOAD_G0CB_DECRYPT;
    }
    return xts_crypt_cnt_odd(s);
}

/* Whether the auto encryption block operates. */
static bool xts_auto_encrypt(ESP32P4MSPIState *s)
{
    if (s->download_boot) {
        return s->crypto_ctrl & CRYPTO_ENABLE_DOWNLOAD_DB_ENCRYPT;
    }
    return xts_crypt_cnt_odd(s);
}

/* Whether the manual encryption block operates. */
static bool xts_manual_enabled(ESP32P4MSPIState *s)
{
    /* EFUSE_DIS_DOWNLOAD_MANUAL_ENCRYPT, RD_REPEAT_DATA0[20]. */
    bool dis = extract32(esp32p4_efuse_word(s->efuse, 0, 1), 20, 1);

    if (s->download_boot) {
        return (s->crypto_ctrl & CRYPTO_ENABLE_DOWNLOAD_MANUAL_ENCRYPT) &&
               !dis;
    }
    return s->crypto_ctrl & CRYPTO_ENABLE_SPI_MANUAL_ENCRYPT;
}

static uint32_t xts_size(ESP32P4MSPIState *s)
{
    return 16u << MIN(s->spi0[R0_XTS_LINESIZE] & 3, 2);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void xts_trigger(ESP32P4MSPIState *s)
{
    ESP32P4XtsKey key;
    uint32_t size = xts_size(s);
    uint32_t type = s->spi0[R0_XTS_DESTINATION] & 1;
    uint32_t base = s->spi0[R0_XTS_PHYSICAL_ADDRESS] & 0x3fffffffu;
    uint8_t plain[64];
    uint8_t block[XTS_BLOCK];
    uint32_t a;

    if (s->xts_state != XTS_IDLE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_TRIGGER in state %u, not "
                      "idle; ignored\n", mspi_name(s), s->xts_state);
        return;
    }
    if (!xts_manual_enabled(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_TRIGGER while the manual "
                      "encryption block is not enabled "
                      "(HP_SYSTEM_CRYPTO_CTRL, boot mode); ignored\n",
                      mspi_name(s));
        return;
    }
    if ((s->spi0[R0_XTS_LINESIZE] & 3) == 3) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_LINESIZE 3 is reserved; "
                      "taken as 64 bytes\n", mspi_name(s));
    }
    if (type) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_DESTINATION 1 (PSRAM) is "
                      "reserved; only flash may be the destination\n",
                      mspi_name(s));
    }
    if (base % size) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_PHYSICAL_ADDRESS 0x%" PRIx32
                      " is not aligned to the %" PRIu32 "-byte size\n",
                      mspi_name(s), base, size);
        base &= ~(size - 1);
    }

    /* XTS_PLAIN_n holds the word at an address with (address % 64) / 4 n. */
    for (a = 0; a < 16; a++) {
        stl_le_p(plain + 4 * a, s->xts_plain[a]);
    }
    xts_key(s, type, &key);
    memset(s->xts_cipher, 0, sizeof(s->xts_cipher));
    for (a = base; a < base + size; a += XTS_BLOCK) {
        memcpy(block, plain + a % 64, XTS_BLOCK);
        xts_crypt_block(&key, type, a, block, true);
        memcpy(s->xts_cipher + a % 64, block, XTS_BLOCK);
    }
    s->xts_state = XTS_DONE;
}

/*
 * The bytes an SPI1 write-data phase sends to the flash at `addr`, which
 * while the manual encryption block's result is released to SPI1 are the
 * ciphertext's where they fall in its target space.
 */
static void xts_substitute(ESP32P4MSPIState *s, uint32_t addr, uint8_t *data,
                           uint32_t len)
{
    uint32_t size = xts_size(s);
    uint32_t base = s->spi0[R0_XTS_PHYSICAL_ADDRESS] & 0x3fffffffu &
                    ~(size - 1);
    uint32_t i;

    if (s->xts_state != XTS_RELEASED) {
        return;
    }
    for (i = 0; i < len; i++) {
        uint32_t a = addr + i;

        if (a >= base && a < base + size) {
            data[i] = s->xts_cipher[a % 64];
        }
    }
}

/* The PSRAM die */

static uint32_t psram_size(ESP32P4MSPIState *s)
{
    return memory_region_size(s->psram_mr);
}

static void psram_access(ESP32P4MSPIState *s, uint32_t addr, uint8_t *buf,
                         uint32_t len, bool write)
{
    uint8_t *mem = memory_region_get_ram_ptr(s->psram_mr);
    uint32_t size = psram_size(s);

    while (len) {
        uint32_t off = addr % size;
        uint32_t n = MIN(len, size - off);

        if (write) {
            memcpy(mem + off, buf, n);
            memory_region_set_dirty(s->psram_mr, off, n);
        } else {
            memcpy(buf, mem + off, n);
        }
        addr += n;
        buf += n;
        len -= n;
    }
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void psram_reset_mrs(ESP32P4MSPIState *s)
{
    memset(s->psram_mr_regs, 0, sizeof(s->psram_mr_regs));
    s->psram_mr_regs[1] = 0x0d;
    s->psram_mr_regs[2] = 0xdf;
    s->psram_mr_regs[3] = 0x20;
    s->psram_halfsleep = false;
}

static void psram_write_mr(ESP32P4MSPIState *s, unsigned mr, uint8_t v)
{
    switch (mr) {
    case 0:
    case 4:
    case 8:
        s->psram_mr_regs[mr] = v;
        break;
    case 6:
        s->psram_mr_regs[mr] = v;
        if (v == PSRAM_HALFSLEEP) {
            s->psram_halfsleep = true;
        }
        break;
    case 1:
    case 2:
    case 3:
        /* Read-only: the die's identification and refresh. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of 0x%02x to PSRAM mode "
                      "register %u, which the die does not have\n",
                      mspi_name(s), v, mr);
        break;
    }
}

/* The flash on the SSI bus */

static void flash_select(ESP32P4MSPIState *s, int cs, bool selected)
{
    qemu_set_irq(s->cs[cs], !selected);
}

/* SPI0's read of `len` bytes of the flash at `addr`. */
static void flash_read(ESP32P4MSPIState *s, uint32_t addr, uint8_t *buf,
                       uint32_t len)
{
    uint32_t i;

    flash_select(s, 0, true);
    ssi_transfer(s->bus, FLASH_READ4);
    for (i = 0; i < 4; i++) {
        ssi_transfer(s->bus, extract32(addr, 24 - 8 * i, 8));
    }
    for (i = 0; i < len; i++) {
        buf[i] = ssi_transfer(s->bus, 0);
    }
    flash_select(s, 0, false);
}

/* The AXI port and the MMU */

static unsigned mmu_page_shift(ESP32P4MSPIState *s)
{
    return MMU_MAX_PAGE_SHIFT - extract32(s->spi0[R0_MMU_POWER_CTRL], 3, 2);
}

static bool mmu_valid(ESP32P4MSPIState *s, uint32_t entry)
{
    return entry & (s->psram ? MMU_PSRAM_VALID : MMU_FLASH_VALID);
}

static bool mmu_sensitive(ESP32P4MSPIState *s, uint32_t entry)
{
    return entry & (s->psram ? MMU_PSRAM_SENSITIVE : MMU_FLASH_SENSITIVE);
}

static uint32_t mmu_page(ESP32P4MSPIState *s, uint32_t entry)
{
    return entry & (s->psram ? MMU_PSRAM_PAGE_MASK : MMU_FLASH_PAGE_MASK);
}

/*
 * An access through an invalid MMU entry: record it, and answer it as
 * AXI_ERR_RESP_EN configures.
 */
static MemTxResult axi_invalid(ESP32P4MSPIState *s, uint32_t addr,
                               bool write)
{
    uint32_t bit = write ? INT0_AXI_WADDR_ERR : INT0_AXI_RADDR_ERR;
    uint32_t resp = write ? RESP_EN_AW_MMU_VLD : RESP_EN_AR_MMU_VLD;

    if (!(s->spi0[R0_INT_RAW] & INT0_AXI_ERRS)) {
        s->spi0[R0_AXI_ERR_ADDR] = addr & 0x07ffffffu;
    }
    spi0_raise(s, bit);
    return (s->spi0[R0_AXI_ERR_RESP_EN] & resp) ? MEMTX_ERROR : MEMTX_OK;
}

/* Read `len` bytes of the memory at `paddr`, decrypting if `key`. */
static void axi_mem_read(ESP32P4MSPIState *s, uint32_t paddr, uint8_t *buf,
                         uint32_t len, const ESP32P4XtsKey *key)
{
    uint32_t start = key ? QEMU_ALIGN_DOWN(paddr, XTS_BLOCK) : paddr;
    uint32_t end = key ? QEMU_ALIGN_UP(paddr + len, XTS_BLOCK) : paddr + len;
    g_autofree uint8_t *tmp = g_malloc(end - start);

    if (s->psram) {
        psram_access(s, start, tmp, end - start, false);
    } else {
        flash_read(s, start, tmp, end - start);
    }
    if (key) {
        xts_crypt(key, s->psram, start, tmp, end - start, false);
    }
    memcpy(buf, tmp + (paddr - start), len);
}

/* Write `len` bytes to the PSRAM at `paddr`, encrypting if `key`. */
static void axi_psram_write(ESP32P4MSPIState *s, uint32_t paddr,
                            const uint8_t *buf, uint32_t len,
                            const ESP32P4XtsKey *key)
{
    uint32_t start, end;
    g_autofree uint8_t *tmp = NULL;

    if (!key) {
        psram_access(s, paddr, (uint8_t *)buf, len, true);
        return;
    }
    start = QEMU_ALIGN_DOWN(paddr, XTS_BLOCK);
    end = QEMU_ALIGN_UP(paddr + len, XTS_BLOCK);
    tmp = g_malloc(end - start);
    if (start != paddr || end != paddr + len) {
        psram_access(s, start, tmp, end - start, false);
        xts_crypt(key, 1, start, tmp, end - start, false);
    }
    memcpy(tmp + (paddr - start), buf, len);
    xts_crypt(key, 1, start, tmp, end - start, true);
    psram_access(s, start, tmp, end - start, true);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult mspi_axi(ESP32P4MSPIState *s, uint32_t addr, uint8_t *buf,
                            uint32_t len, bool write)
{
    MemTxResult result = MEMTX_OK;
    ESP32P4XtsKey key;
    bool have_key = false;

    if (!mspi_bus_running(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: AXI access at 0x%" PRIx32
                      " while the controller's clock is stopped or it is "
                      "held in reset\n", mspi_name(s), addr);
        if (!write) {
            memset(buf, 0, len);
        }
        return MEMTX_ERROR;
    }
    if (s->spi0[R0_CACHE_FCTRL] & CACHE_FCTRL_CLOSE_AXI_INF_EN) {
        if (!write) {
            memset(buf, 0, len);
        }
        return MEMTX_ERROR;
    }
    if (s->psram && s->psram_halfsleep) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: AXI access at 0x%" PRIx32
                      " while the PSRAM is in half sleep\n", mspi_name(s),
                      addr);
        if (!write) {
            memset(buf, 0, len);
        }
        return MEMTX_OK;
    }

    while (len) {
        unsigned shift = mmu_page_shift(s);
        uint32_t page_size = 1u << shift;
        uint32_t idx = addr >> shift;
        uint32_t off = addr & (page_size - 1);
        uint32_t n = MIN(len, page_size - off);
        uint32_t entry = idx < ESP32P4_MSPI_MMU_ENTRIES ? s->mmu[idx] : 0;

        if (!mmu_valid(s, entry)) {
            if (axi_invalid(s, addr, write) != MEMTX_OK) {
                result = MEMTX_ERROR;
            }
            if (!write) {
                memset(buf, 0, n);
            }
        } else {
            uint32_t paddr = (mmu_page(s, entry) << shift) | off;
            bool crypt = mmu_sensitive(s, entry) &&
                         (write ? xts_auto_encrypt(s) : xts_auto_decrypt(s));

            if (crypt && !have_key) {
                xts_key(s, s->psram, &key);
                have_key = true;
            }
            if (!s->psram && write) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: AXI write at 0x%" PRIx32
                              " to the flash, which the controller does "
                              "not program; dropped\n", mspi_name(s), addr);
                spi0_raise(s, INT0_AXI_WR_FLASH_ERR);
            } else if (write) {
                axi_psram_write(s, paddr, buf, n, crypt ? &key : NULL);
            } else {
                axi_mem_read(s, paddr, buf, n, crypt ? &key : NULL);
            }
            spi0_raise(s, INT0_SLV_ST_END | INT0_MST_ST_END);
        }
        addr += n;
        buf += n;
        len -= n;
    }
    mspi_update_irq(s);
    return result;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
MemTxResult esp32p4_mspi_axi_read(ESP32P4MSPIState *s, uint32_t addr,
                                  uint8_t *buf, uint32_t len)
{
    return mspi_axi(s, addr, buf, len, false);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
MemTxResult esp32p4_mspi_axi_write(ESP32P4MSPIState *s, uint32_t addr,
                                   const uint8_t *buf, uint32_t len)
{
    return mspi_axi(s, addr, (uint8_t *)buf, len, true);
}

/* The window: direct accesses, and the accesses of masters without cache. */

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult mspi_window_read(void *opaque, hwaddr addr, uint64_t *data,
                                    unsigned size, MemTxAttrs attrs)
{
    ESP32P4MSPIState *s = opaque;
    uint8_t buf[8];
    MemTxResult r;

    if (size < 4 && !(s->spi0[R0_CTRL1] & CTRL1_AR_SIZE0_1_SUPPORT_EN)) {
        *data = 0;
        return MEMTX_ERROR;
    }
    r = esp32p4_mspi_axi_read(s, addr, buf, size);
    *data = ldn_le_p(buf, size);
    return r;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult mspi_window_write(void *opaque, hwaddr addr, uint64_t data,
                                     unsigned size, MemTxAttrs attrs)
{
    ESP32P4MSPIState *s = opaque;
    uint8_t buf[8];

    if (size < 4 && !(s->spi0[R0_CTRL1] & CTRL1_AW_SIZE0_1_SUPPORT_EN)) {
        return MEMTX_ERROR;
    }
    stn_le_p(buf, size, data);
    return esp32p4_mspi_axi_write(s, addr, buf, size);
}

static const MemoryRegionOps mspi_window_ops = {
    .read_with_attrs = mspi_window_read,
    .write_with_attrs = mspi_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* SPI1 */

/* An SPI1 transaction: its phases, and how many lines each takes. */
typedef struct {
    uint32_t cmd;
    unsigned cmd_bits;
    uint32_t addr;
    unsigned addr_bits;
    unsigned dummy;
    unsigned mosi_bytes;
    unsigned miso_bytes;
    unsigned cmd_lines;
    unsigned addr_lines;
    unsigned dout_lines;
    unsigned din_lines;
    /* For the MSPI PMS: whether it reads, or writes, at its address. */
    bool reads;
    bool writes;
    uint8_t mosi[64];
    uint8_t miso[64];
} ESP32P4MSPIXfer;

static unsigned spi1_cmd_lines(ESP32P4MSPIState *s)
{
    uint32_t ctrl = s->spi1[R1_CTRL];

    return ctrl & CTRL_FCMD_OCT ? 8 : ctrl & CTRL_FCMD_QUAD ? 4 : 1;
}

static unsigned spi1_addr_lines(ESP32P4MSPIState *s)
{
    uint32_t ctrl = s->spi1[R1_CTRL];
    uint32_t fctrl = s->spi1[R1_CACHE_FCTRL];

    if (ctrl & CTRL_FADDR_OCT) {
        return 8;
    }
    if ((ctrl & CTRL_FREAD_QIO) || (fctrl & FCTRL_FADDR_QUAD)) {
        return 4;
    }
    if ((ctrl & CTRL_FREAD_DIO) || (fctrl & FCTRL_FADDR_DUAL)) {
        return 2;
    }
    return 1;
}

static unsigned spi1_din_lines(ESP32P4MSPIState *s)
{
    uint32_t ctrl = s->spi1[R1_CTRL];
    uint32_t fctrl = s->spi1[R1_CACHE_FCTRL];

    if (ctrl & CTRL_FDIN_OCT) {
        return 8;
    }
    if ((ctrl & (CTRL_FREAD_QIO | CTRL_FREAD_QUAD)) ||
        (fctrl & FCTRL_FDIN_QUAD)) {
        return 4;
    }
    if ((ctrl & (CTRL_FREAD_DIO | CTRL_FREAD_DUAL)) ||
        (fctrl & FCTRL_FDIN_DUAL)) {
        return 2;
    }
    return 1;
}

static unsigned spi1_dout_lines(ESP32P4MSPIState *s)
{
    uint32_t user = s->spi1[R1_USER];
    uint32_t fctrl = s->spi1[R1_CACHE_FCTRL];

    if (s->spi1[R1_CTRL] & CTRL_FDOUT_OCT) {
        return 8;
    }
    if ((user & (USER_FWRITE_QIO | USER_FWRITE_QUAD)) ||
        (fctrl & FCTRL_FDOUT_QUAD)) {
        return 4;
    }
    if ((user & (USER_FWRITE_DIO | USER_FWRITE_DUAL)) ||
        (fctrl & FCTRL_FDOUT_DUAL)) {
        return 2;
    }
    return 1;
}

static void spi1_buf_read(ESP32P4MSPIState *s, unsigned first, uint8_t *out,
                          unsigned len)
{
    unsigned i;

    for (i = 0; i < len; i++) {
        unsigned b = first * 4 + i;

        out[i] = extract32(s->spi1[R1_W0 + (b / 4) % 16], 8 * (b % 4), 8);
    }
}

/* The default phases: SPI1's line settings, and nothing in them. */
static void spi1_xfer_init(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    memset(x, 0, sizeof(*x));
    x->cmd_lines = spi1_cmd_lines(s);
    x->addr_lines = spi1_addr_lines(s);
    x->din_lines = spi1_din_lines(s);
    x->dout_lines = spi1_dout_lines(s);
}

/* The address phase of the built-in commands: ADDR, 24 or 32 bits. */
static void spi1_xfer_addr(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    x->addr_bits = USER1_ADDR_BITLEN(s->spi1[R1_USER1]) + 1 > 24 ? 32 : 24;
    x->addr = s->spi1[R1_ADDR];
    if (x->addr_bits == 24) {
        x->addr &= 0xffffff;
    }
}

/* The user-defined transaction of SPI1's USER, USER1, USER2 and buffer. */
static void spi1_xfer_user(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    uint32_t user = s->spi1[R1_USER];
    unsigned bits;

    spi1_xfer_init(s, x);
    if (user & USER_USR_COMMAND) {
        x->cmd = USER2_COMMAND_VALUE(s->spi1[R1_USER2]);
        x->cmd_bits = USER2_COMMAND_BITLEN(s->spi1[R1_USER2]) + 1;
    }
    if (user & USER_USR_ADDR) {
        x->addr_bits = USER1_ADDR_BITLEN(s->spi1[R1_USER1]) + 1;
        x->addr = s->spi1[R1_ADDR];
        if (x->addr_bits < 32) {
            x->addr &= MAKE_64BIT_MASK(0, x->addr_bits);
        }
    }
    if (user & USER_USR_DUMMY) {
        x->dummy = USER1_DUMMY_CYCLELEN(s->spi1[R1_USER1]) + 1;
    }
    if (user & USER_USR_MOSI) {
        bits = DLEN(s->spi1[R1_MOSI_DLEN]) + 1;
        x->mosi_bytes = DIV_ROUND_UP(bits, 8);
    }
    if (user & USER_USR_MISO) {
        bits = DLEN(s->spi1[R1_MISO_DLEN]) + 1;
        x->miso_bytes = DIV_ROUND_UP(bits, 8);
    }
    if (x->mosi_bytes > 64 || x->miso_bytes > 64) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SPI1 data phase of more than the "
                      "64-byte buffer; truncated\n", mspi_name(s));
        x->mosi_bytes = MIN(x->mosi_bytes, 64);
        x->miso_bytes = MIN(x->miso_bytes, 64);
    }
    spi1_buf_read(s, (user & USER_MOSI_HIGHPART) ? 8 : 0, x->mosi,
                  x->mosi_bytes);
    x->reads = x->addr_bits && x->miso_bytes;
    x->writes = x->addr_bits &&
                (x->mosi_bytes || (s->spi1[R1_CMD] & CMD_FLASH_PE));
}

/*
 * The SPI clock cycles a transaction takes: each phase's bits over its
 * lines, twice as fast in DDR mode.
 */
static uint64_t spi1_xfer_cycles(ESP32P4MSPIState *s,
                                 const ESP32P4MSPIXfer *x)
{
    bool ddr = s->spi1[R1_DDR] & DDR_FMEM_DDR_EN;
    unsigned rate = ddr ? 2 : 1;
    unsigned cmd_rate = ddr && !(s->spi1[R1_DDR] & DDR_FMEM_DDR_CMD_DIS) ?
                        2 : 1;

    return DIV_ROUND_UP(x->cmd_bits, x->cmd_lines * cmd_rate) +
           DIV_ROUND_UP(x->addr_bits, x->addr_lines * rate) + x->dummy +
           DIV_ROUND_UP(x->mosi_bytes * 8, x->dout_lines * rate) +
           DIV_ROUND_UP(x->miso_bytes * 8, x->din_lines * rate);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static bool spi1_pms_check(ESP32P4MSPIState *s, const ESP32P4MSPIXfer *x)
{
    unsigned attr0 = s->psram ? R0_SMEM_PMS_ATTR : R0_FMEM_PMS_ATTR;
    unsigned addr0 = s->psram ? R0_SMEM_PMS_ADDR : R0_FMEM_PMS_ADDR;
    unsigned size0 = s->psram ? R0_SMEM_PMS_SIZE : R0_FMEM_PMS_SIZE;
    uint32_t reject = 0;
    int hits = 0, hit = -1;
    int i;

    if (!(s->spi0[R0_PMS_REJECT] & PMS_REJECT_PM_EN) ||
        !(x->reads || x->writes)) {
        return true;
    }
    for (i = 0; i < 4; i++) {
        uint64_t lo = s->spi0[addr0 + i];
        uint64_t hi = lo + (uint64_t)s->spi0[size0 + i] * PMS_REGION_ALIGN;

        if (x->addr >= lo && x->addr < hi) {
            hits++;
            hit = i;
        }
    }
    if (hits == 0) {
        reject = PMS_REJECT_IVD;
    } else if (hits > 1) {
        reject = PMS_REJECT_MULTI_HIT;
    } else {
        uint32_t attr = s->spi0[attr0 + hit];

        if (x->reads && !(attr & PMS_ATTR_RD)) {
            reject |= PMS_REJECT_LD;
        }
        if (x->writes && !(attr & PMS_ATTR_WR)) {
            reject |= PMS_REJECT_ST;
        }
    }
    if (!reject) {
        return true;
    }
    if (!(s->spi0[R0_INT_RAW] & INT0_PMS_REJECT)) {
        s->spi0[R0_PMS_REJECT] = (s->spi0[R0_PMS_REJECT] & PMS_REJECT_PM_EN) |
                                 reject | (x->addr & PMS_REJECT_ADDR_MASK);
    }
    spi0_raise(s, INT0_PMS_REJECT);
    return false;
}

/* Run a transaction on the flash's bus, chip select `cs`. */
static void spi1_flash_xfer(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x, int cs)
{
    unsigned dummy_bytes = DIV_ROUND_UP(x->dummy * x->addr_lines, 8);
    unsigned i;

    if (cs != 0) {
        /* Nothing is on chip select 1: the data lines read as zero. */
        return;
    }
    if (x->mosi_bytes && x->addr_bits) {
        xts_substitute(s, x->addr, x->mosi, x->mosi_bytes);
    }
    flash_select(s, 0, true);
    for (i = DIV_ROUND_UP(x->cmd_bits, 8); i > 0; i--) {
        ssi_transfer(s->bus, extract32(x->cmd, 8 * (i - 1), 8));
    }
    for (i = DIV_ROUND_UP(x->addr_bits, 8); i > 0; i--) {
        ssi_transfer(s->bus, extract32(x->addr, 8 * (i - 1), 8));
    }
    for (i = 0; i < dummy_bytes; i++) {
        ssi_transfer(s->bus, 0);
    }
    for (i = 0; i < x->mosi_bytes; i++) {
        ssi_transfer(s->bus, x->mosi[i]);
    }
    for (i = 0; i < x->miso_bytes; i++) {
        x->miso[i] = ssi_transfer(s->bus, 0);
    }
    flash_select(s, 0, false);
}

/* Run a transaction on the PSRAM die, chip select 1. */
static void spi1_psram_xfer(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x, int cs)
{
    unsigned i;

    if (cs != 1) {
        return;
    }
    if (s->psram_halfsleep) {
        /* The chip select's pulse wakes the die; it takes no command. */
        s->psram_halfsleep = false;
        return;
    }
    if (x->cmd_bits != 16) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: PSRAM command of %u bits; the "
                      "die takes 16-bit OPI commands\n", mspi_name(s),
                      x->cmd_bits);
        return;
    }
    switch (x->cmd) {
    case PSRAM_SYNC_READ:
    case PSRAM_BURST_READ:
        psram_access(s, x->addr, x->miso, x->miso_bytes, false);
        break;
    case PSRAM_SYNC_WRITE:
    case PSRAM_BURST_WRITE:
        psram_access(s, x->addr, x->mosi, x->mosi_bytes, true);
        break;
    case PSRAM_REG_READ:
        for (i = 0; i < x->miso_bytes; i++) {
            unsigned mr = x->addr + i;

            x->miso[i] = mr < ESP32P4_MSPI_PSRAM_MRS ? s->psram_mr_regs[mr]
                                                     : 0;
        }
        break;
    case PSRAM_REG_WRITE:
        for (i = 0; i < x->mosi_bytes; i++) {
            psram_write_mr(s, x->addr + i, x->mosi[i]);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: unknown PSRAM command 0x%04x\n",
                      mspi_name(s), x->cmd);
        break;
    }
}

/* Run `x` on the device its enabled chip select picks. */
static void spi1_xfer(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    uint32_t misc = s->spi1[R1_MISC];
    int cs;

    for (cs = 0; cs < 2; cs++) {
        if (misc & (cs ? MISC_CS1_DIS : MISC_CS0_DIS)) {
            continue;
        }
        if (s->psram) {
            spi1_psram_xfer(s, x, cs);
        } else {
            spi1_flash_xfer(s, x, cs);
        }
    }
}

/* The SPI clock's divisor of the core clock. */
static uint64_t spi1_clock_div(ESP32P4MSPIState *s)
{
    uint32_t clock = s->spi1[R1_CLOCK];

    return clock & CLOCK_CLK_EQU_SYSCLK ? 1 : CLOCK_CLKCNT_N(clock) + 1;
}

static void spi1_schedule(ESP32P4MSPIState *s)
{
    if (!clock_is_enabled(s->core_clk)) {
        s->spi1_waiting = true;
        return;
    }
    s->spi1_waiting = false;
    timer_mod(&s->spi1_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              clock_ticks_to_ns(s->core_clk,
                                s->spi1_cycles * spi1_clock_div(s)));
}

/* The cycles of the status read FLASH_WAITI_CTRL makes after an operation. */
static uint64_t spi1_waiti_cycles(ESP32P4MSPIState *s)
{
    uint32_t waiti = s->spi1[R1_FLASH_WAITI_CTRL];

    if (!(waiti & WAITI_EN)) {
        return 0;
    }
    return (waiti & WAITI_CMD_2B ? 16 : 8) + 8 +
           (waiti & WAITI_DUMMY ? WAITI_DUMMY_CYCLELEN(waiti) + 1 : 0);
}

/* The wait after a RES, DP, HPM or suspend command, in SPI clock cycles. */
static uint64_t spi1_hold_cycles(ESP32P4MSPIState *s, uint32_t dly_128)
{
    return (uint64_t)CTRL1_CS_HOLD_DLY_RES(s->spi1[R1_CTRL1]) *
           (s->spi1[R1_SUS_STATUS] & dly_128 ? 128 : 4);
}

/*
 * The command and line setting of SPI1's built-in read, from CTRL's read
 * mode.
 */
static void spi1_read_cmd(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    uint32_t ctrl = s->spi1[R1_CTRL];

    if (!(ctrl & CTRL_FASTRD_MODE)) {
        x->cmd = FLASH_READ;
    } else if (ctrl & CTRL_FREAD_QIO) {
        x->cmd = FLASH_QIOR;
    } else if (ctrl & CTRL_FREAD_DIO) {
        x->cmd = FLASH_DIOR;
    } else if (ctrl & CTRL_FREAD_QUAD) {
        x->cmd = FLASH_QOR;
    } else if (ctrl & CTRL_FREAD_DUAL) {
        x->cmd = FLASH_DOR;
    } else {
        x->cmd = FLASH_FAST_READ;
    }
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void spi1_start(ESP32P4MSPIState *s, uint32_t cmd)
{
    ESP32P4MSPIXfer x;
    uint32_t op = cmd & CMD_USR ? CMD_USR : BIT(31 - clz32(cmd & CMD_OPS));
    uint32_t busy = op;
    uint32_t end = INT1_SLV_ST_END | INT1_MST_ST_END;
    uint64_t extra = 0;
    bool pe = false, to_status = false, to_buf = false;
    unsigned status_bytes = 0;
    unsigned i;

    if (ctpop32(cmd & CMD_OPS) > 1 && !(cmd == (CMD_USR | CMD_FLASH_PE))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SPI1_CMD 0x%08" PRIx32 " starts "
                      "more than one operation; only one runs\n",
                      mspi_name(s), cmd);
    }
    spi1_xfer_init(s, &x);
    x.cmd_bits = 8;
    x.cmd_lines = 1;

    switch (op) {
    case CMD_USR:
        spi1_xfer_user(s, &x);
        busy |= cmd & CMD_FLASH_PE;
        pe = cmd & CMD_FLASH_PE;
        to_buf = x.miso_bytes;
        break;
    case CMD_FLASH_READ:
        spi1_read_cmd(s, &x);
        spi1_xfer_addr(s, &x);
        if (s->spi1[R1_USER] & USER_USR_DUMMY) {
            x.dummy = USER1_DUMMY_CYCLELEN(s->spi1[R1_USER1]) + 1;
        }
        x.miso_bytes = MIN(DIV_ROUND_UP(DLEN(s->spi1[R1_MISO_DLEN]) + 1, 8),
                           64);
        x.reads = true;
        to_buf = true;
        break;
    case CMD_FLASH_WREN:
        x.cmd = FLASH_WREN;
        break;
    case CMD_FLASH_WRDI:
        x.cmd = FLASH_WRDI;
        break;
    case CMD_FLASH_RDID:
        x.cmd = FLASH_RDID;
        x.miso_bytes = 3;
        to_buf = true;
        break;
    case CMD_FLASH_RDSR:
        x.cmd = FLASH_RDSR;
        x.miso_bytes = 1;
        to_status = true;
        status_bytes = 1;
        break;
    case CMD_FLASH_WRSR:
        x.cmd = FLASH_WRSR;
        x.mosi_bytes = s->spi1[R1_CTRL] & CTRL_WRSR_2B ? 2 : 1;
        x.mosi[0] = extract32(s->spi1[R1_RD_STATUS], 0, 8);
        x.mosi[1] = extract32(s->spi1[R1_RD_STATUS], 8, 8);
        pe = true;
        break;
    case CMD_FLASH_PP:
        x.cmd = FLASH_PP;
        spi1_xfer_addr(s, &x);
        if (x.addr_bits == 24) {
            x.mosi_bytes = extract32(s->spi1[R1_ADDR], 24, 8);
        } else {
            x.mosi_bytes = DIV_ROUND_UP(DLEN(s->spi1[R1_MOSI_DLEN]) + 1, 8);
        }
        if (x.mosi_bytes > 64) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: page program of %u bytes, "
                          "more than the 64-byte buffer\n", mspi_name(s),
                          x.mosi_bytes);
            x.mosi_bytes = 64;
        }
        spi1_buf_read(s, 0, x.mosi, x.mosi_bytes);
        x.writes = true;
        pe = true;
        break;
    case CMD_FLASH_SE:
    case CMD_FLASH_BE:
        x.cmd = op == CMD_FLASH_SE ? FLASH_SE : FLASH_BE;
        spi1_xfer_addr(s, &x);
        x.writes = true;
        pe = true;
        break;
    case CMD_FLASH_CE:
        x.cmd = FLASH_CE;
        pe = true;
        break;
    case CMD_FLASH_DP:
        x.cmd = FLASH_DP;
        extra = spi1_hold_cycles(s, SUS_STATUS_DP_DLY_128);
        break;
    case CMD_FLASH_RES:
        x.cmd = FLASH_RES;
        if (s->spi1[R1_CTRL] & CTRL_RESANDRES) {
            /* Three dummy bytes, then the device ID. */
            x.dummy = 24;
            x.miso_bytes = 1;
            to_status = true;
            status_bytes = 1;
        }
        extra = spi1_hold_cycles(s, SUS_STATUS_RES_DLY_128);
        break;
    case CMD_FLASH_HPM:
        x.cmd = FLASH_HPM;
        x.dummy = 24;
        extra = spi1_hold_cycles(s, SUS_STATUS_HPM_DLY_128);
        break;
    default:
        g_assert_not_reached();
    }

    if (spi1_pms_check(s, &x)) {
        spi1_xfer(s, &x);
    } else {
        x.miso_bytes = 0;
        to_status = to_buf = false;
    }
    if (pe) {
        extra += spi1_waiti_cycles(s);
        end |= INT1_WPE_END;
    }

    if (to_buf) {
        uint32_t base = (op == CMD_USR &&
                         (s->spi1[R1_USER] & USER_MISO_HIGHPART)) ? 8 : 0;

        for (i = 0; i < x.miso_bytes; i++) {
            unsigned b = base * 4 + i;
            unsigned w = R1_W0 + (b / 4) % 16;

            s->spi1[w] = deposit32(s->spi1[w], 8 * (b % 4), 8, x.miso[i]);
        }
    }
    if (to_status) {
        uint32_t st = 0;

        for (i = 0; i < status_bytes; i++) {
            st |= x.miso[i] << (8 * i);
        }
        s->spi1[R1_RD_STATUS] = (s->spi1[R1_RD_STATUS] & ~RD_STATUS_STATUS) |
                                st;
    }

    s->spi1[R1_CMD] |= busy;
    s->spi1_busy_cmd = busy;
    s->spi1_busy_sus = 0;
    s->spi1_sus_after = -1;
    s->spi1_end_int = end;
    s->spi1_cycles = spi1_xfer_cycles(s, &x) + extra;
    spi1_schedule(s);
    mspi_update_irq(s);
}

/* Program/erase suspend (PES) or resume (PER), from FLASH_SUS_CTRL. */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void spi1_suspend(ESP32P4MSPIState *s, bool resume)
{
    ESP32P4MSPIXfer x;
    bool cmd_2b = s->spi1[R1_SUS_STATUS] & SUS_STATUS_PESR_CMD_2B;
    uint32_t sus = s->spi1[R1_FLASH_SUS_CTRL];
    uint64_t wait = 0;

    spi1_xfer_init(s, &x);
    x.cmd_lines = 1;
    x.cmd_bits = cmd_2b ? 16 : 8;
    x.cmd = resume ? SUS_PER_COMMAND(s->spi1[R1_SUS_STATUS])
                   : SUS_PES_COMMAND(s->spi1[R1_FLASH_SUS_CMD]);
    if (!cmd_2b) {
        x.cmd &= 0xff;
    }
    spi1_xfer(s, &x);

    if (sus & (resume ? SUS_FLASH_PER_WAIT_EN : SUS_FLASH_PES_WAIT_EN)) {
        if (resume && (s->spi1[R1_CTRL1] & CTRL1_CS_HOLD_DLY_PER_EN)) {
            wait = (uint64_t)CTRL1_CS_HOLD_DLY_PER(s->spi1[R1_CTRL1]) * 128;
        } else {
            wait = spi1_hold_cycles(s, resume ? SUS_STATUS_PER_DLY_128
                                              : SUS_STATUS_PES_DLY_128);
        }
    }
    s->spi1_busy_cmd = 0;
    s->spi1_busy_sus = resume ? SUS_FLASH_PER : SUS_FLASH_PES;
    s->spi1[R1_FLASH_SUS_CTRL] |= s->spi1_busy_sus;
    s->spi1_sus_after = resume ? 0 : 1;
    s->spi1_end_int = INT1_SLV_ST_END | INT1_MST_ST_END |
                      (resume ? INT1_PER_END : INT1_PES_END);
    s->spi1_cycles = spi1_xfer_cycles(s, &x) + wait;
    spi1_schedule(s);
}

static void spi1_end(void *opaque)
{
    ESP32P4MSPIState *s = opaque;

    s->spi1[R1_CMD] &= ~s->spi1_busy_cmd;
    s->spi1[R1_FLASH_SUS_CTRL] &= ~s->spi1_busy_sus;
    if (s->spi1_sus_after >= 0) {
        s->spi1[R1_SUS_STATUS] = deposit32(s->spi1[R1_SUS_STATUS], 0, 1,
                                           s->spi1_sus_after);
    }
    spi1_raise(s, s->spi1_end_int);
    s->spi1_busy_cmd = 0;
    s->spi1_busy_sus = 0;
    s->spi1_sus_after = -1;
    s->spi1_end_int = 0;
    mspi_update_irq(s);
}

static bool spi1_busy(ESP32P4MSPIState *s)
{
    return s->spi1_busy_cmd || s->spi1_busy_sus;
}

/* End SPI1's transaction now, as its FSM's reset does. */
static void spi1_abort(ESP32P4MSPIState *s)
{
    timer_del(&s->spi1_timer);
    s->spi1[R1_CMD] &= ~s->spi1_busy_cmd;
    s->spi1[R1_FLASH_SUS_CTRL] &= ~s->spi1_busy_sus;
    s->spi1_busy_cmd = 0;
    s->spi1_busy_sus = 0;
    s->spi1_sus_after = -1;
    s->spi1_end_int = 0;
    s->spi1_waiting = false;
}

/* Register access */

/*
 * A write to register `r` of a block described by `regs`, of `v` in the
 * bytes `mask` enables: its read-write fields take their bits, and the
 * write-1-to-clear ones clear. Returns the write-to-trigger bits set.
 */
static uint32_t reg_write(const ESP32P4MSPIReg *regs, uint32_t *file,
                          unsigned r, uint32_t v, uint32_t mask)
{
    file[r] = (file[r] & ~(regs[r].rw & mask)) | (v & regs[r].rw & mask);
    file[r] &= ~(v & regs[r].w1c & mask);
    return v & regs[r].wt & mask;
}

static void log_bad_offset(ESP32P4MSPIState *s, const char *block,
                           hwaddr addr, bool write)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s %s of reserved offset 0x%03"
                  HWADDR_PRIx "\n", mspi_name(s), block,
                  write ? "write" : "read", addr);
}

static uint32_t spi0_reg_read(ESP32P4MSPIState *s, unsigned r)
{
    switch (r) {
    case R0_INT_ST:
        return s->spi0[R0_INT_RAW] & s->spi0[R0_INT_ENA];
    case R0_MMU_ITEM_CONTENT: {
        uint32_t idx = s->spi0[R0_MMU_ITEM_INDEX];

        return idx < ESP32P4_MSPI_MMU_ENTRIES ? s->mmu[idx] : 0;
    }
    case R0_XTS_STATE:
        return s->xts_state;
    default:
        if (r >= R0_XTS_PLAIN && r < R0_XTS_PLAIN + 16) {
            return s->xts_plain[r - R0_XTS_PLAIN];
        }
        return s->spi0[r];
    }
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static uint64_t mspi_spi0_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4MSPIState *s = opaque;
    unsigned r = addr / 4;
    uint32_t v;

    if (!spi0_regs(s)[r].name &&
        !(r >= R0_XTS_PLAIN && r < R0_XTS_PLAIN + 16)) {
        log_bad_offset(s, "SPI0", addr, false);
        return 0;
    }
    v = spi0_reg_read(s, r);
    return extract32(v, 8 * (addr & 3), 8 * size);
}

static void spi0_mmu_write(ESP32P4MSPIState *s, uint32_t v)
{
    uint32_t idx = s->spi0[R0_MMU_ITEM_INDEX];

    if (idx >= ESP32P4_MSPI_MMU_ENTRIES) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: MMU_ITEM_CONTENT write with "
                      "MMU_ITEM_INDEX %" PRIu32 ", past the %d entries\n",
                      mspi_name(s), idx, ESP32P4_MSPI_MMU_ENTRIES);
        return;
    }
    v &= MMU_ENTRY_MASK;
    if (mmu_valid(s, v) && !!(v & MMU_ACCESS_PSRAM) != s->psram) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: MMU entry %" PRIu32 " 0x%04"
                      PRIx32 " has the other memory's target bit; each MMU "
                      "maps only its own memory\n", mspi_name(s), idx, v);
    }
    s->mmu[idx] = v;
}

static void spi0_check_unimp(ESP32P4MSPIState *s, unsigned r)
{
    bool ecc = false;
    int i;

    switch (r) {
    case R0_CTRL2:
        ecc = s->spi0[r] & CTRL2_ECC_16TO18_BYTE_EN;
        break;
    case R0_ECC_CTRL:
        ecc = s->spi0[r] & (ECC_CTRL_FMEM_ECC_ADDR_EN |
                            ECC_CTRL_USR_ECC_ADDR_EN);
        break;
    case R0_SMEM_ECC_CTRL:
        ecc = s->spi0[r] & SMEM_ECC_CTRL_ECC_ADDR_EN;
        break;
    case R0_MMU_POWER_CTRL:
        if (s->spi0[r] & MMU_POWER_FORCE_PD) {
            qemu_log_mask(LOG_UNIMP, "%s: powering the MMU's memory down "
                          "(MMU_MEM_FORCE_PD) is not modelled; it keeps its "
                          "entries\n", mspi_name(s));
        }
        break;
    default:
        for (i = 0; i < 4; i++) {
            if (r == R0_FMEM_PMS_ATTR + i || r == R0_SMEM_PMS_ATTR + i) {
                ecc = s->spi0[r] & (PMS_ATTR_ECC | PMS_ATTR_NONSECURE_ECC);
            }
        }
        break;
    }
    if (ecc) {
        qemu_log_mask(LOG_UNIMP, "%s: ECC on the external memory is not "
                      "modelled\n", mspi_name(s));
    }
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void mspi_spi0_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    ESP32P4MSPIState *s = opaque;
    const ESP32P4MSPIReg *regs = spi0_regs(s);
    unsigned r = addr / 4;
    unsigned shift = 8 * (addr & 3);
    uint32_t v = val << shift;
    uint32_t mask = MAKE_64BIT_MASK(shift, 8 * size);
    uint32_t trig;

    if (r >= R0_XTS_PLAIN && r < R0_XTS_PLAIN + 16) {
        s->xts_plain[r - R0_XTS_PLAIN] =
            (s->xts_plain[r - R0_XTS_PLAIN] & ~mask) | (v & mask);
        return;
    }
    if (!regs[r].name) {
        log_bad_offset(s, "SPI0", addr, true);
        return;
    }

    switch (r) {
    case R0_INT_CLR:
        v &= regs[R0_INT_CLR].wt & mask;
        s->spi0[R0_INT_RAW] &= ~v;
        if (v & INT0_PMS_REJECT) {
            s->spi0[R0_PMS_REJECT] &= PMS_REJECT_PM_EN;
        }
        if (v & INT0_AXI_ERRS) {
            s->spi0[R0_AXI_ERR_ADDR] = 0;
        }
        break;
    case R0_MMU_ITEM_CONTENT:
        spi0_mmu_write(s, (spi0_reg_read(s, r) & ~mask) | (v & mask));
        break;
    default:
        trig = reg_write(regs, s->spi0, r, v, mask);
        if (r == R0_INT_RAW) {
            v &= regs[R0_INT_RAW].w1c & mask;
            if (v & INT0_PMS_REJECT) {
                s->spi0[R0_PMS_REJECT] &= PMS_REJECT_PM_EN;
            }
            if (v & INT0_AXI_ERRS) {
                s->spi0[R0_AXI_ERR_ADDR] = 0;
            }
        }
        if (trig && r == R0_XTS_TRIGGER) {
            xts_trigger(s);
        } else if (trig && r == R0_XTS_RELEASE) {
            if (s->xts_state == XTS_DONE) {
                s->xts_state = XTS_RELEASED;
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_RELEASE in state %u, "
                              "not 2; ignored\n", mspi_name(s), s->xts_state);
            }
        } else if (trig && r == R0_XTS_DESTROY) {
            if (s->xts_state == XTS_RELEASED) {
                memset(s->xts_cipher, 0, sizeof(s->xts_cipher));
                s->xts_state = XTS_IDLE;
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: XTS_DESTROY in state %u, "
                              "not 3; ignored\n", mspi_name(s), s->xts_state);
            }
        }
        spi0_check_unimp(s, r);
        break;
    }
    mspi_update_irq(s);
}

static const MemoryRegionOps mspi_spi0_ops = {
    .read = mspi_spi0_read,
    .write = mspi_spi0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.memory] */
static uint64_t mspi_spi1_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4MSPIState *s = opaque;
    unsigned r = addr / 4;
    uint32_t v;

    if (!spi1_regs(s)[r].name) {
        log_bad_offset(s, "SPI1", addr, false);
        return 0;
    }
    v = r == R1_INT_ST ? s->spi1[R1_INT_RAW] & s->spi1[R1_INT_ENA]
                       : s->spi1[r];
    return extract32(v, 8 * (addr & 3), 8 * size);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void mspi_spi1_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    ESP32P4MSPIState *s = opaque;
    const ESP32P4MSPIReg *regs = spi1_regs(s);
    unsigned r = addr / 4;
    unsigned shift = 8 * (addr & 3);
    uint32_t v = val << shift;
    uint32_t mask = MAKE_64BIT_MASK(shift, 8 * size);
    uint32_t start;

    if (!regs[r].name) {
        log_bad_offset(s, "SPI1", addr, true);
        return;
    }

    switch (r) {
    case R1_CMD:
        start = v & mask & CMD_OPS;
        if (start && spi1_busy(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: SPI1_CMD 0x%08" PRIx32
                          " while an operation is in progress; ignored\n",
                          mspi_name(s), v);
            break;
        }
        if (start) {
            /* FLASH_PE marks a user operation; on its own it runs nothing. */
            if (!(start & ~CMD_FLASH_PE)) {
                break;
            }
            s->spi1[R1_CMD] |= start;
            spi1_start(s, s->spi1[R1_CMD] & CMD_OPS);
        }
        break;
    case R1_FLASH_SUS_CTRL:
        start = v & mask & (SUS_FLASH_PER | SUS_FLASH_PES);
        reg_write(regs, s->spi1, r, v & ~(SUS_FLASH_PER | SUS_FLASH_PES),
                  mask);
        if (start && spi1_busy(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: suspend or resume while an "
                          "operation is in progress; ignored\n",
                          mspi_name(s));
        } else if (start) {
            spi1_suspend(s, start & SUS_FLASH_PER);
        }
        break;
    case R1_INT_CLR:
        s->spi1[R1_INT_RAW] &= ~(v & regs[R1_INT_CLR].wt & mask);
        break;
    default:
        if (reg_write(regs, s->spi1, r, v, mask) && r == R1_CTRL2) {
            spi1_abort(s);
        }
        if (r == R1_MISC && s->psram && s->psram_halfsleep &&
            (s->spi1[R1_MISC] & MISC_CS_KEEP_ACTIVE) &&
            !(s->spi1[R1_MISC] & MISC_CS1_DIS)) {
            s->psram_halfsleep = false;
        }
        break;
    }
    mspi_update_irq(s);
}

static const MemoryRegionOps mspi_spi1_ops = {
    .read = mspi_spi1_read,
    .write = mspi_spi1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* Reset, hooks and clocks */

/* The controller's registers, and its SPI1 and manual encryption state. */
static void mspi_reset_regs(ESP32P4MSPIState *s)
{
    const ESP32P4MSPIReg *r0 = spi0_regs(s), *r1 = spi1_regs(s);
    int i;

    spi1_abort(s);
    for (i = 0; i < ESP32P4_MSPI_NREGS; i++) {
        s->spi0[i] = r0[i].reset;
        s->spi1[i] = r1[i].reset;
    }
    s->spi0[R0_MMU_ITEM_CONTENT] = 0;
    memset(s->xts_plain, 0, sizeof(s->xts_plain));
    memset(s->xts_cipher, 0, sizeof(s->xts_cipher));
    s->xts_state = XTS_IDLE;
}

static void mspi_set_reset(ESP32P4MSPIState *s, bool *line, bool level)
{
    bool was = mspi_in_reset(s);

    *line = level;
    if (mspi_in_reset(s) != was) {
        mspi_reset_regs(s);
        mspi_update_irq(s);
    }
}

static void mspi_apb_reset(void *opaque, int n, int level)
{
    ESP32P4MSPIState *s = opaque;

    mspi_set_reset(s, &s->apb_reset, level);
}

static void mspi_axi_reset(void *opaque, int n, int level)
{
    ESP32P4MSPIState *s = opaque;

    mspi_set_reset(s, &s->axi_reset, level);
}

static void mspi_crypto_ctrl(void *opaque, int n, int level)
{
    ESP32P4MSPIState *s = opaque;

    s->crypto_ctrl = deposit32(s->crypto_ctrl, n, 1, level != 0);
}

static void mspi_download_boot(void *opaque, int n, int level)
{
    ESP32P4MSPIState *s = opaque;

    s->download_boot = level;
}

static void mspi_brown_out(void *opaque, int n, int level)
{
    ESP32P4MSPIState *s = opaque;

    if (level) {
        spi1_raise(s, INT1_BROWN_OUT);
        mspi_update_irq(s);
    }
}

static void mspi_core_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4MSPIState *s = opaque;

    if (s->spi1_waiting && clock_is_enabled(s->core_clk)) {
        spi1_schedule(s);
    }
}

static void mspi_reset_hold(Object *obj, ResetType type)
{
    ESP32P4MSPIState *s = ESP32P4_MSPI(obj);

    mspi_reset_regs(s);
    memset(s->mmu, 0, sizeof(s->mmu));
    psram_reset_mrs(s);
}

static void mspi_reset_exit(Object *obj, ResetType type)
{
    ESP32P4MSPIState *s = ESP32P4_MSPI(obj);

    flash_select(s, 0, false);
    flash_select(s, 1, false);
    mspi_update_irq(s);
}

static void mspi_realize(DeviceState *dev, Error **errp)
{
    ESP32P4MSPIState *s = ESP32P4_MSPI(dev);

    if (!s->efuse) {
        error_setg(errp, "%s: the 'efuse' link is not set", mspi_name(s));
        return;
    }
    if (s->psram && !s->psram_mr) {
        error_setg(errp, "%s: the 'psram-mr' link is not set", mspi_name(s));
        return;
    }
    if (s->psram && !memory_region_is_ram(s->psram_mr)) {
        error_setg(errp, "%s: 'psram-mr' is not RAM", mspi_name(s));
        return;
    }
    memory_region_init_io(&s->spi0_iomem, OBJECT(s), &mspi_spi0_ops, s,
                          s->psram ? "esp32p4.psram-mspi0"
                                   : "esp32p4.flash-spi0",
                          ESP32P4_MSPI_REGS_SIZE);
    memory_region_init_io(&s->spi1_iomem, OBJECT(s), &mspi_spi1_ops, s,
                          s->psram ? "esp32p4.psram-mspi1"
                                   : "esp32p4.flash-spi1",
                          ESP32P4_MSPI_REGS_SIZE);
    memory_region_init_io(&s->window, OBJECT(s), &mspi_window_ops, s,
                          s->psram ? "esp32p4.psram-window"
                                   : "esp32p4.flash-window",
                          ESP32P4_MSPI_WINDOW_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->spi0_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->spi1_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->window);
}

static void mspi_init(Object *obj)
{
    ESP32P4MSPIState *s = ESP32P4_MSPI(obj);
    DeviceState *dev = DEVICE(obj);

    s->bus = ssi_create_bus(dev, "ssi");
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->cs, SSI_GPIO_CS, 2);
    qdev_init_gpio_in_named(dev, mspi_apb_reset, ESP32P4_MSPI_APB_RESET, 1);
    qdev_init_gpio_in_named(dev, mspi_axi_reset, ESP32P4_MSPI_AXI_RESET, 1);
    qdev_init_gpio_in_named(dev, mspi_crypto_ctrl, ESP32P4_MSPI_CRYPTO_CTRL,
                            4);
    qdev_init_gpio_in_named(dev, mspi_download_boot,
                            ESP32P4_MSPI_DOWNLOAD_BOOT, 1);
    qdev_init_gpio_in_named(dev, mspi_brown_out, ESP32P4_MSPI_BROWN_OUT, 1);
    s->sys_clk = qdev_init_clock_in(dev, "sys", NULL, NULL, 0);
    s->core_clk = qdev_init_clock_in(dev, "core", mspi_core_clk_update, s,
                                     ClockUpdate);
    timer_init_ns(&s->spi1_timer, QEMU_CLOCK_VIRTUAL, spi1_end, s);
}

static const VMStateDescription vmstate_esp32p4_mspi = {
    .name = TYPE_ESP32P4_MSPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(spi0, ESP32P4MSPIState, ESP32P4_MSPI_NREGS),
        VMSTATE_UINT32_ARRAY(spi1, ESP32P4MSPIState, ESP32P4_MSPI_NREGS),
        VMSTATE_UINT32_ARRAY(xts_plain, ESP32P4MSPIState, 16),
        VMSTATE_UINT32_ARRAY(mmu, ESP32P4MSPIState, ESP32P4_MSPI_MMU_ENTRIES),
        VMSTATE_TIMER(spi1_timer, ESP32P4MSPIState),
        VMSTATE_UINT32(spi1_busy_cmd, ESP32P4MSPIState),
        VMSTATE_UINT32(spi1_busy_sus, ESP32P4MSPIState),
        VMSTATE_UINT32(spi1_end_int, ESP32P4MSPIState),
        VMSTATE_INT32(spi1_sus_after, ESP32P4MSPIState),
        VMSTATE_UINT64(spi1_cycles, ESP32P4MSPIState),
        VMSTATE_BOOL(spi1_waiting, ESP32P4MSPIState),
        VMSTATE_UINT32(xts_state, ESP32P4MSPIState),
        VMSTATE_UINT8_ARRAY(xts_cipher, ESP32P4MSPIState, 64),
        VMSTATE_UINT8_ARRAY(psram_mr_regs, ESP32P4MSPIState,
                            ESP32P4_MSPI_PSRAM_MRS),
        VMSTATE_BOOL(psram_halfsleep, ESP32P4MSPIState),
        VMSTATE_BOOL(apb_reset, ESP32P4MSPIState),
        VMSTATE_BOOL(axi_reset, ESP32P4MSPIState),
        VMSTATE_UINT32(crypto_ctrl, ESP32P4MSPIState),
        VMSTATE_BOOL(download_boot, ESP32P4MSPIState),
        VMSTATE_CLOCK(sys_clk, ESP32P4MSPIState),
        VMSTATE_CLOCK(core_clk, ESP32P4MSPIState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_mspi_properties[] = {
    DEFINE_PROP_BOOL("psram", ESP32P4MSPIState, psram, false),
    DEFINE_PROP_LINK("psram-mr", ESP32P4MSPIState, psram_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("keymng", ESP32P4MSPIState, keymng, TYPE_ESP32P4_KEYMNG,
                     ESP32P4KeyMngState *),
    DEFINE_PROP_LINK("efuse", ESP32P4MSPIState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
};

static void esp32p4_mspi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = mspi_reset_hold;
    rc->phases.exit = mspi_reset_exit;
    dc->realize = mspi_realize;
    dc->vmsd = &vmstate_esp32p4_mspi;
    device_class_set_props(dc, esp32p4_mspi_properties);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static const TypeInfo esp32p4_mspi_info = {
    .name          = TYPE_ESP32P4_MSPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4MSPIState),
    .instance_init = mspi_init,
    .class_init    = esp32p4_mspi_class_init,
};

static void esp32p4_mspi_register_types(void)
{
    type_register_static(&esp32p4_mspi_info);
}
type_init(esp32p4_mspi_register_types)
