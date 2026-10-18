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
 * The flash: the M25P80 model's part on the SSI bus "ssi", selected by
 * chip select 0 (SSI_GPIO_CS line 0). SPI0's reads and SPI1's
 * transactions take one path onto it (flash_wire_xfer()): a clock of the
 * SPI bus at a time, the controller drives the flash's four data lines in
 * each phase as its registers set it up, and the flash takes and sends
 * each byte on the lines its own command uses (m25p80_io_lines()), bytes
 * passing to and from the M25P80 model as their clocks complete. So a
 * set-up the flash does not expect reads as the chip would: a command,
 * address or mode byte on the wrong lines garbled, the data shifted by a
 * dummy phase too short or too long, the bits of a line nothing drives at
 * its pad's pull, a flash that refuses the command (a quad one with QE
 * clear) not driving the lines at all. A phase on one or two lines leaves
 * IO2 and IO3, WP# and HOLD#, driven at CTRL's WP_REG level and high. The
 * mode bits a DIO or QIO read sends after the address, M7-M0, come from
 * the address or the dummy phase, whichever covers them, and a flash in
 * continuous read mode (M5-4 = 10b) takes the next transaction's command
 * as address bits. SPI1's chip select is raised at its transaction's last
 * cycle, when the flash starts the program or erase it was given.
 *
 * SPI0's read (spi0_flash_xfer()): the command USER2 gives with
 * CACHE_FCTRL's CACHE_FLASH_USR_CMD set, as the ROM and ESP-IDF set it,
 * else the one CTRL's read mode selects (03h, 0Bh, 3Bh, 6Bh, BBh, EBh, or
 * with CACHE_USR_ADDR_4BYTE 13h, 0Ch, 3Ch, 6Ch, BCh, ECh: the TRM does not
 * name them), on four lines with FCMD_QUAD; USER1's address phase on the
 * read mode's lines, from the top of the 24-bit address followed by
 * RD_STATUS's WB_MODE (the mode bits the ROM's 28-bit address phase ends
 * with), or of the 32-bit address with CACHE_USR_ADDR_4BYTE; USER1's dummy
 * cycles with USR_DUMMY, no SPI clock in them with USR_DUMMY_IDLE, the
 * first half of them driven at the dummy-out levels (D_POL on IO0, Q_POL
 * on IO1, WP_REG on IO2, IO3 high) with FDUMMY_RIN, as the headers give
 * SPI0's FDUMMY_RIN; the data on the read mode's lines. CTRL's FREAD_*
 * bits count with FASTRD_MODE set, as the headers say, on SPI0 and SPI1
 * alike; CACHE_FCTRL's FADDR/FDIN bits whatever it says. SPI1's user
 * transactions take CTRL's read mode for their address and data phases
 * too, as ESP-IDF's setting it for each command implies; its built-in
 * read does, and its other built-in commands are on one line, as the
 * ROM's status reads in DIO mode need. Its dummy phase is driven
 * throughout with FDUMMY_RIN for a transaction that reads data,
 * FDUMMY_WOUT for one that does not. An address phase longer than the 32
 * bits the controller holds sends, after them, bits the TRM leaves
 * undefined: the model sends the worst case, the mode bits 0x20 that put
 * a flash in continuous read mode, and logs the hazard as a guest error.
 * Each AXI read of the flash is one SPI0 transaction.
 *
 * The built-in block erase is the 64 KiB one (0xD8), which the ROM and
 * ESP-IDF erase blocks with; the headers' "(32KB)" contradicts them. A
 * page program takes its length from ADDR[31:24].
 *
 * The PSRAM: the in-package AP Memory 256 Mbit x16 die (APS256XXN), on
 * SPI1_MEM_S's chip select 1 and SPI0's window, as a RAM region (the
 * "psram-mr" link) behind its mode registers. SPI0's accesses and SPI1's
 * transactions both reach it as the die takes them (psram_die_xfer()):
 * only in its OPI DDR form, a 16-bit command and a 32-bit address on 8
 * lines; its synchronous and linear burst read (0x0000, 0x2020) and write
 * (0x8080, 0xA0A0) with MR8's burst wrap, at its row (address bits 24:11)
 * and column, of bytes in X8 and of 16-bit words in X16, CA[10] ignored
 * (psram_cell()); mode register read (0x4040) and write (0xC0C0) at the
 * register's number, global reset (0xFFFF); its data from the latency its
 * mode registers give (MR0's read latency code, variable or fixed; MR4's
 * write latency code; 1 for a register write), which a dummy phase of 2 *
 * (latency - 1) clock edges matches, a variable-latency read that meets
 * the die's refresh coming at the longest push-out, unless the port's
 * variable dummy has the controller follow the die's strobe
 * (mspi_var_dummy()); a memory read or write at a clock faster than its
 * latency code allows going wrong; a memory access at an odd address
 * starting at the even one before; CE# held low past tCEM logged; data a
 * beat on each clock edge, two bytes on DQ0 to DQ15 with MR8's X16, else
 * one on DQ0 to DQ7. A port's data lines are 16 with SPI0's SRAM_CMD
 * SDIN_HEX or SDOUT_HEX (for both ports), else CTRL's or SRAM_CMD's; on 16
 * lines SPI0 sends the address of the word holding the window's byte
 * (spi0_psram_wire_addr()), SPI1 the address software gives it; SPI0
 * starts at the die's even address. So SPI0 left as from reset (an SPI
 * command on one line, SDR) reaches nothing, the die's power-up latencies
 * under dummy phases set for others shift the data, and an X8 die read on
 * 16 lines leaves every other byte to the undriven lines. MR1 holds vendor
 * 0x0D (AP Memory), MR2 0xDF (256 Mbit, generation 4, known good die) and
 * MR3 0xA0 (row boundary crossing, 4x refresh), MR3's refresh flag
 * following MR4's refresh rate; MR0, MR4 and MR8 power up as the
 * datasheet gives them (0x08, 0x40, 0x05: latency codes 5, X8, 32-byte
 * hybrid wrap). Writing 0xF0 to MR6 puts the die into half sleep, where it
 * keeps its contents and takes no command until a transaction, or SPI1's
 * MISC_CS_KEEP_ACTIVE with its chip select enabled, wakes it. SPI0 splits
 * its accesses at SMEM_ECC_CTRL's page corners with SMEM_AC's
 * SMEM_SPLIT_TRANS_EN (set from reset); the die wraps a burst at its
 * 2048-byte row, but for a linear burst read with MR8's RBX, which goes on
 * into the next row (psram_rbx_wait()).
 *
 * The pads (hw/gpio/esp32p4_mspi_iomux.c, the "pins" link). A DDR read of
 * the die, SPI1's or SPI0's (their DDR registers' DDR_EN set, DQS_LOOP
 * clear), captures each beat on the die's strobe through IOMUX_MSPI_PIN:
 * at the strobe's phase and delay lines, less each data line's, for the
 * SPI clock the read runs at (the core clock over CLOCK's divider for
 * SPI1, SRAM_CLK's for SPI0); a line sampled outside its beat's valid
 * window takes the bit of a neighbouring beat, and with the strobe's
 * input powered down a line captures 0. The flash's data lines, where no
 * flash drives them (no flash on the bus, or chip select 1 alone), read
 * as their pads' pulls hold them; where it does, the controller samples
 * them through SPI_MEM_C's timing calibration (flash_sample()).
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
 * times 4, or 128 with SUS_STATUS's *_DLY_128) and, after a program, erase
 * or status write while FLASH_WAITI_CTRL_WAITI_EN is set, the automatic
 * status reads, one and then another while the flash reads busy (the
 * reads a flash with busy times, m25p80_busy_until(), would take, without
 * placing each on the bus, as they change nothing). SPI0's accesses wait
 * for no SPI1 transaction here: one arriving while SPI1 holds the chip
 * select ends that transaction on the bus first. Its CMD bits clear, its
 * read data reaches
 * W0..W15 and RD_STATUS, and SLV_ST_END, MST_ST_END and, after a program,
 * erase or status write, WPE_END raise when it ends, which an access to
 * SPI1's registers finds however late the timer ending it runs. It waits
 * while the core clock is stopped. SPI0's accesses take no time; each
 * raises SPI0's SLV_ST_END and MST_ST_END.
 *
 * The registers are behind the "sys" bus clock (mapped through the clock
 * and reset tree's gate). The "apb-reset" and "axi-reset" inputs hold the
 * controller in reset. The AXI port takes no access while its bus clock
 * is stopped or the controller is held in reset, nor, unless closed (when
 * it answers errors), while its core clock is stopped or CACHE_FCTRL's
 * AXI_REQ_EN is clear (esp32p4_mspi_axi_ready()): on the chip the access
 * never completes and the bus hangs. A core's access through the window
 * or the caches waits on the bus until the port takes accesses again,
 * and is then made. A DMA master's waits likewise, with its channel, for
 * GDMA-AHB, GDMA-AXI and VDMA, which ask before each transaction
 * (esp32p4_mspi_dma_wait()) and make it once the port takes it; another
 * master's (qtest's, or a DMA master that does not ask) is refused with
 * an error response and logged.
 *
 * Not modelled: ECC on the external memory (FMEM/SMEM ECC, the PSRAM's
 * 16-to-18 byte mode and its address conversion), logged as unimplemented
 * when enabled; the PSRAM controller's own sampling (SMEM_DIN_MODE,
 * SMEM_DIN_NUM, the HEX ones, and a DDR read with DQS_LOOP set, which
 * captures every beat right), as ESP-IDF captures on the strobe; the
 * flash controller's FMEM_VAR_DUMMY, for flashes with a strobe, which the
 * chip's quad flash has not; auto-suspend and
 * auto-resume (FLASH_SUS_CTRL's PES_EN and PER_EN), with which the
 * controller would suspend a busy flash for SPI0's reads: SPI0 reads a
 * busy flash as it answers, ignoring them; SPI0's own user command (USR,
 * which the headers mark reserved).
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
#include "hw/block/flash.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_mspi_iomux.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "hw/core/cpu.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "target/riscv/esp32p4_cpu.h"

/*
 * Each register's name, reset value and fields by access, from the v3
 * headers: stored as written (rw), acting when written with 1 and reading
 * 0 (wt), and raw status bits a write of 1 clears (w1c). The rest are
 * read-only to software: the hardware's status, and the reserved fields,
 * which read as their reset values.
 *
 * SPI0's MOSI_DLEN and MISO_DLEN (0x024, 0x028), the lengths of a user
 * command's write and read data in bits less one (bits 9:0), are the
 * IP's, which the v3 headers leave out of SPI0 as reserved: SPI1 has them
 * at those offsets, the SPI0 of the IP's earlier chips (ESP32-S3, -C6,
 * -H2) has them, the headers hide registers of SPI0 that it has (v1's hid
 * RD_STATUS and CACHE_SCTRL, which v3's name), and the v3 boot ROM's
 * SPI_init() sets both to 256 bits in every flash boot. They hold what is
 * written: SPI0's user command, which would use them, is not modelled.
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
    REG(0x024, "MOSI_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
    REG(0x028, "MISO_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
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
    REG(0x024, "MOSI_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
    REG(0x028, "MISO_DLEN", 0x00000000, 0x000003ff, 0x00000000, 0x00000000),
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
#define R0_CTRL                 (0x008 / 4)
#define R0_CTRL1                (0x00c / 4)
#define R0_USER                 (0x018 / 4)
#define R0_USER1                (0x01c / 4)
#define R0_USER2                (0x020 / 4)
#define R0_RD_STATUS            (0x02c / 4)
#define R0_CACHE_FCTRL          (0x03c / 4)
#define R0_CACHE_SCTRL          (0x040 / 4)
#define R0_SRAM_CMD             (0x044 / 4)
#define R0_SRAM_DRD_CMD         (0x048 / 4)
#define R0_SRAM_DWR_CMD         (0x04c / 4)
#define R0_SRAM_CLK             (0x050 / 4)
#define R0_SMEM_DDR             (0x0d8 / 4)
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
#define R0_SMEM_AC              (0x1a0 / 4)
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
#define R0_CLOCK                (0x014 / 4)
#define R0_TIMING_CALI          (0x180 / 4)
#define R0_DIN_MODE             (0x184 / 4)
#define R0_DIN_NUM              (0x188 / 4)
#define R1_TIMING_CALI          (0x180 / 4)

/* TIMING_CALI, the flash's timing calibration (SPI0's; SPI1's has CALI). */
#define TIMING_CLK_ENA          BIT(0)
#define TIMING_CALI_EN          BIT(1)
#define TIMING_EXTRA_DUMMY(v)   extract32((v), 2, 3)
#define TIMING_CALI_UPDATE      BIT(6)

#define CTRL1_AR_SIZE0_1_SUPPORT_EN BIT(21)
#define CTRL1_AW_SIZE0_1_SUPPORT_EN BIT(22)
#define CTRL2_ECC_16TO18_BYTE_EN    BIT(14)
#define CACHE_FCTRL_CLOSE_AXI_INF_EN BIT(31)
#define CACHE_FCTRL_AXI_REQ_EN      BIT(0)

/* The PSRAM controller's SPI0 set-up for its window. */
#define SCTRL_SADDR_4BYTE           BIT(0)
#define SCTRL_SRAM_DIO              BIT(1)
#define SCTRL_SRAM_QIO              BIT(2)
#define SCTRL_WR_SRAM_DUMMY         BIT(3)
#define SCTRL_RD_SRAM_DUMMY         BIT(4)
#define SCTRL_SRAM_USR_RCMD         BIT(5)
#define SCTRL_RDUMMY_CYCLELEN(v)    extract32((v), 6, 6)
#define SCTRL_ADDR_BITLEN(v)        extract32((v), 14, 6)
#define SCTRL_SRAM_USR_WCMD         BIT(20)
#define SCTRL_WDUMMY_CYCLELEN(v)    extract32((v), 22, 6)
#define SCMD_SWB_MODE(v)            extract32((v), 2, 8)
#define SCMD_SDIN_DUAL              BIT(10)
#define SCMD_SDOUT_DUAL             BIT(11)
#define SCMD_SADDR_DUAL             BIT(12)
#define SCMD_SDIN_QUAD              BIT(14)
#define SCMD_SDOUT_QUAD             BIT(15)
#define SCMD_SADDR_QUAD             BIT(16)
#define SCMD_SCMD_QUAD              BIT(17)
#define SCMD_SDIN_OCT               BIT(18)
#define SCMD_SDOUT_OCT              BIT(19)
#define SCMD_SADDR_OCT              BIT(20)
#define SCMD_SCMD_OCT               BIT(21)
#define SCMD_SDIN_HEX               BIT(26)
#define SCMD_SDOUT_HEX              BIT(27)
#define SRAM_CMD_VALUE(v)           extract32((v), 0, 16)
#define SRAM_CMD_BITLEN(v)          extract32((v), 28, 4)
#define SMEM_AC_SPLIT_TRANS_EN      BIT(31)
#define SMEM_ECC_PAGE_SIZE(v)       extract32((v), 18, 2)
#define CACHE_FCTRL_USR_ADDR_4BYTE  BIT(1)
#define CACHE_FCTRL_FLASH_USR_CMD   BIT(2)

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

/* The hazards of SPI0's set-up logged until its registers change. */
#define MSPI_LOGGED_ADDR        BIT(0)
#define MSPI_LOGGED_PSRAM_CMD   BIT(1)
#define MSPI_LOGGED_PSRAM_LAT   BIT(2)
#define MSPI_LOGGED_PSRAM_FMAX  BIT(3)
#define MSPI_LOGGED_PSRAM_REFRESH BIT(4)
#define MSPI_LOGGED_PSRAM_ODD   BIT(5)
#define MSPI_LOGGED_PSRAM_TCEM  BIT(6)
#define MSPI_LOGGED_PSRAM_RBX   BIT(7)

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

#define CTRL_FDUMMY_RIN         BIT(2)
#define CTRL_FDUMMY_WOUT        BIT(3)
#define CTRL_FDOUT_OCT          BIT(4)
#define CTRL_FDIN_OCT           BIT(5)
#define CTRL_FADDR_OCT          BIT(6)
#define CTRL_FCMD_QUAD          BIT(8)
#define CTRL_FCMD_OCT           BIT(9)
#define CTRL_FASTRD_MODE        BIT(13)
#define CTRL_FREAD_DUAL         BIT(14)
#define CTRL_RESANDRES          BIT(15)
#define CTRL_Q_POL              BIT(18)
#define CTRL_D_POL              BIT(19)
#define CTRL_FREAD_QUAD         BIT(20)
#define CTRL_WP_REG             BIT(21)
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
#define USER_USR_DUMMY_IDLE     BIT(26)
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
#define DDR_FMEM_VAR_DUMMY      BIT(1)
#define DDR_DQS_LOOP            BIT(21)

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
#define FLASH_FAST_READ4 0x0c
#define FLASH_READ4     0x13
#define FLASH_SE        0x20
#define FLASH_DOR       0x3b
#define FLASH_DOR4      0x3c
#define FLASH_CE        0x60
#define FLASH_QOR       0x6b
#define FLASH_QOR4      0x6c
#define FLASH_RDID      0x9f
#define FLASH_HPM       0xa3
#define FLASH_RES       0xab
#define FLASH_DP        0xb9
#define FLASH_DIOR      0xbb
#define FLASH_DIOR4     0xbc
#define FLASH_BE        0xd8
#define FLASH_QIOR      0xeb
#define FLASH_QIOR4     0xec

/* The PSRAM die's MR6 value that puts it in half sleep. */
#define PSRAM_HALFSLEEP     0xf0
/* MR8's X16: the die's data on DQ0 to DQ15, else on DQ0 to DQ7. */
#define PSRAM_MR8_X16       BIT(6)
#define PSRAM_MR8_RBX       BIT(3)
#define PSRAM_MR8_BT        BIT(2)
#define PSRAM_MR0_LT        BIT(5)
#define PSRAM_MR3_RBXEN     BIT(7)
/* The instructions of the die's 16-bit commands, their first byte. */
#define PSRAM_INST_SYNC_READ    0x00
#define PSRAM_INST_BURST_READ   0x20
#define PSRAM_INST_REG_READ     0x40
#define PSRAM_INST_SYNC_WRITE   0x80
#define PSRAM_INST_BURST_WRITE  0xa0
#define PSRAM_INST_REG_WRITE    0xc0
#define PSRAM_INST_RESET        0xff
/* The die's page: its row, 2048 bytes (or 1024 16-bit words in X16). */
#define PSRAM_PAGE          2048

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

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * The `len` bytes the die sends in a read, as the controller captures them
 * through the pads in a DDR read (SPI1's DDR, or SPI0's SMEM_DDR, with
 * DDR_EN set and DDR_DQS_LOOP clear), on the strobes' edges at their
 * phase and delay lines (hw/gpio/esp32p4_mspi_iomux.c): each data line
 * sampled outside its beat's valid window takes the bit of the beat
 * before or after, and one whose strobe is powered down captures 0. Beats
 * are two bytes on DQ0 to DQ15 with `x16`, the even byte on DQ0 to DQ7,
 * else a byte on DQ0 to DQ7; the first is on a rising strobe edge.
 */
static void psram_capture(ESP32P4MSPIState *s, bool spi1, bool x16,
                          uint8_t *buf, uint32_t len)
{
    uint32_t ddr = spi1 ? s->spi1[R1_DDR] : s->spi0[R0_SMEM_DDR];
    uint32_t clk = spi1 ? s->spi1[R1_CLOCK] : s->spi0[R0_SRAM_CLK];
    uint64_t div = clk & CLOCK_CLK_EQU_SYSCLK ? 1 : CLOCK_CLKCNT_N(clk) + 1;
    uint64_t hz = clock_get_hz(s->core_clk);
    unsigned width = x16 ? 2 : 1, beats = DIV_ROUND_UP(len, width);
    ESP32P4MspiSample how[ESP32P4_MSPI_IOMUX_PSRAM_DQ][2];
    g_autofree uint8_t *sent = NULL;
    bool good = true;

    if (!s->pins || !len || !hz || !(ddr & DDR_FMEM_DDR_EN) ||
        (ddr & DDR_DQS_LOOP)) {
        return;
    }
    for (unsigned dq = 0; dq < 8 * width; dq++) {
        for (unsigned edge = 0; edge < 2; edge++) {
            how[dq][edge] = esp32p4_mspi_iomux_psram_sample(s->pins, dq,
                                edge, div * 1000000000000ull / hz);
            good &= how[dq][edge] == ESP32P4_MSPI_SAMPLE_GOOD;
        }
    }
    if (good) {
        return;
    }
    sent = g_memdup2(buf, len);
    for (unsigned i = 0; i < len; i++) {
        unsigned lane = i % width, beat = i / width;
        uint8_t v = 0;

        for (unsigned bit = 0; bit < 8; bit++) {
            ESP32P4MspiSample h = how[8 * lane + bit][beat & 1];
            int src = beat + (h == ESP32P4_MSPI_SAMPLE_EARLY ? -1 :
                              h == ESP32P4_MSPI_SAMPLE_LATE ? 1 : 0);

            if (h != ESP32P4_MSPI_SAMPLE_NONE && src >= 0 && src < beats &&
                src * width + lane < len) {
                v |= sent[src * width + lane] & BIT(bit);
            }
        }
        buf[i] = v;
    }
}

/* Whether the die's data reads are 16 lines wide (MR8's X16). */
static bool psram_x16(ESP32P4MSPIState *s)
{
    return s->psram_mr_regs[8] & PSRAM_MR8_X16;
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * The die's mode registers at power-up and after a global reset
 * (APS256XXN datasheet, tables 3 to 21): MR0 0x08, variable latency with
 * read latency code 2 (LC 5) and full drive strength; MR3 0xA0, RBXen
 * (the die has row boundary crossing, which its features list "enabled
 * via Mode Register") and 4x refresh; MR4 0x40, write latency code 2
 * (WLC 5), 4x refresh, the whole array refreshed; MR8 0x05, X8, 32-byte
 * hybrid wrap bursts.
 */
static void psram_reset_mrs(ESP32P4MSPIState *s)
{
    memset(s->psram_mr_regs, 0, sizeof(s->psram_mr_regs));
    s->psram_mr_regs[0] = 0x08;
    s->psram_mr_regs[1] = 0x0d;
    s->psram_mr_regs[2] = 0xdf;
    s->psram_mr_regs[3] = PSRAM_MR3_RBXEN | 0x20;
    s->psram_mr_regs[4] = 0x40;
    s->psram_mr_regs[8] = 0x05;
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

/*
 * An MSPI transaction: its phases, and how many lines each takes. The
 * address phase goes on the wire as the top `addr_bits` bits of
 * `addr_wire`; `addr` is the address it is for, as the MSPI PMS and the
 * PSRAM die take it.
 */
typedef struct {
    uint32_t cmd;
    unsigned cmd_bits;
    uint32_t addr;
    uint64_t addr_wire;
    unsigned addr_bits;
    unsigned dummy;
    /*
     * The dummy phase: its first `dummy_out` cycles with the controller
     * driving IO0 to IO3 at `dummy_level` (bit n for IOn, as CTRL's
     * FDUMMY_RIN or FDUMMY_WOUT, D_POL, Q_POL and WP_REG give them); no
     * SPI clock at all in it with `dummy_idle` (USER's USR_DUMMY_IDLE).
     */
    unsigned dummy_out;
    uint8_t dummy_level;
    bool dummy_idle;
    /* The levels it drives WP# (IO2) and HOLD# (IO3) at, bits 2 and 3. */
    uint8_t idle_level;
    unsigned mosi_bytes;
    uint32_t miso_bytes;
    unsigned cmd_lines;
    unsigned addr_lines;
    unsigned dout_lines;
    unsigned din_lines;
    /* For the MSPI PMS: whether it reads, or writes, at its address. */
    bool reads;
    bool writes;
    /*
     * For the flash's read data: the port's CLOCK register, which with the
     * core clock gives the SPI clock's period, and the extra dummy cycles
     * its timing calibration adds before the data (flash_sample()).
     */
    uint32_t clock;
    unsigned extra_dummy;
    uint8_t mosi[64];
    uint8_t miso[64];
} ESP32P4MSPIXfer;

/*
 * The mode bits the address phase's bits past the controller's 32 carry
 * (see xfer_set_addr()): M7-M0 = 0x20, M5-4 = 10b, which the GigaDevice
 * and Winbond parts take as continuous read mode.
 */
#define ADDR_UNDEFINED_BITS 0x20

/*
 * The address phase of `bits` bits from the 32-bit value `reg` the
 * controller holds, sent from its top. The controller holds no more than
 * 32 bits: what an address phase longer than that sends after them, the
 * TRM leaves undefined. Silicon sends something there, which a flash
 * reading with mode bits takes as its M7-M0, so the model sends the worst
 * case, the bits that put it in continuous read mode, ADDR_UNDEFINED_BITS
 * over and over from the 33rd bit on; continuous read mode takes the
 * next transaction's command as address bits. Returns whether it did.
 */
static bool xfer_set_addr(ESP32P4MSPIXfer *x, uint32_t reg, unsigned bits)
{
    unsigned i;

    x->addr_bits = bits;
    x->addr_wire = (uint64_t)reg << 32;
    for (i = 32; i < bits; i++) {
        uint64_t b = (ADDR_UNDEFINED_BITS >> (7 - (i - 32) % 8)) & 1;

        x->addr_wire |= b << (63 - i);
    }
    return bits > 32;
}

static void log_addr_undefined(ESP32P4MSPIState *s, const char *port,
                               unsigned bits)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s's address phase is %u bits, "
                  "longer than the 32-bit address; the %u bits after the "
                  "address are undefined, and are sent as the mode bits "
                  "0x%02x that put a flash in continuous read mode\n",
                  mspi_name(s), port, bits, bits - 32, ADDR_UNDEFINED_BITS);
}

/*
 * The lines a read's address and data phases take: CTRL's read mode,
 * whose FREAD_* bits count with FASTRD_MODE set, or CACHE_FCTRL's bits
 * for each phase.
 */
static unsigned addr_lines(uint32_t ctrl, uint32_t fctrl)
{
    uint32_t fr = ctrl & CTRL_FASTRD_MODE ? ctrl : 0;

    if (ctrl & CTRL_FADDR_OCT) {
        return 8;
    }
    if ((fr & CTRL_FREAD_QIO) || (fctrl & FCTRL_FADDR_QUAD)) {
        return 4;
    }
    if ((fr & CTRL_FREAD_DIO) || (fctrl & FCTRL_FADDR_DUAL)) {
        return 2;
    }
    return 1;
}

static unsigned din_lines(uint32_t ctrl, uint32_t fctrl)
{
    uint32_t fr = ctrl & CTRL_FASTRD_MODE ? ctrl : 0;

    if (ctrl & CTRL_FDIN_OCT) {
        return 8;
    }
    if ((fr & (CTRL_FREAD_QIO | CTRL_FREAD_QUAD)) ||
        (fctrl & FCTRL_FDIN_QUAD)) {
        return 4;
    }
    if ((fr & (CTRL_FREAD_DIO | CTRL_FREAD_DUAL)) ||
        (fctrl & FCTRL_FDIN_DUAL)) {
        return 2;
    }
    return 1;
}

/*
 * The lines CTRL's dummy-out levels set: D_POL on IO0 (MOSI), Q_POL on
 * IO1 (MISO), WP_REG on IO2; HOLD# (IO3) high.
 */
static uint8_t ctrl_dummy_level(uint32_t ctrl)
{
    return (ctrl & CTRL_D_POL ? BIT(0) : 0) | (ctrl & CTRL_Q_POL ? BIT(1) : 0) |
           (ctrl & CTRL_WP_REG ? BIT(2) : 0) | BIT(3);
}

static uint8_t ctrl_idle_level(uint32_t ctrl)
{
    return (ctrl & CTRL_WP_REG ? BIT(2) : 0) | BIT(3);
}

/* Whether a flash chip is on the bus. */
static bool flash_present(ESP32P4MSPIState *s)
{
    return !QTAILQ_EMPTY(&BUS(s->bus)->children);
}

/* The levels the flash's four data lines rest at, bit n for IOn. */
static uint8_t flash_pads_idle(ESP32P4MSPIState *s)
{
    uint8_t v = 0;

    if (!s->pins) {
        return 0;
    }
    for (unsigned io = 0; io < ESP32P4_MSPI_IOMUX_FLASH_IOS && io < 4; io++) {
        v |= (uint8_t)esp32p4_mspi_iomux_flash_idle(s->pins, io) << io;
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * A byte read over `lines` data lines that nothing drives: each line at
 * the level its pad's pulls give it (hw/gpio/esp32p4_mspi_iomux.c), the
 * highest-numbered line first each clock, MISO (IO1) alone with one line.
 */
static uint8_t flash_idle_byte(ESP32P4MSPIState *s, unsigned lines)
{
    unsigned clock = flash_pads_idle(s);

    switch (lines) {
    case 1:
        return clock & BIT(1) ? 0xff : 0;
    case 2:
        return (clock & 3) * 0x55;
    case 4:
        return (clock & 15) * 0x11;
    default:
        /* The flash has no IO4 to IO7 pads. */
        return clock & 15;
    }
}

static void flash_select(ESP32P4MSPIState *s, int cs, bool selected)
{
    qemu_set_irq(s->cs[cs], !selected);
}

/* End SPI1's flash transaction on the bus: the flash acts on /CS rising. */
static void spi1_release_cs(ESP32P4MSPIState *s)
{
    if (s->spi1_cs_held) {
        s->spi1_cs_held = false;
        flash_select(s, 0, false);
    }
}

/*
 * The flash's side of a transaction, a clock at a time: the bits of each
 * byte it takes or sends, on the lines its command and phase use
 * (m25p80_io_lines()), the byte passing to or from the M25P80 model once
 * its clocks are done.
 */
typedef struct {
    DeviceState *flash;
    SSIBus *bus;
    unsigned lines;
    unsigned clock;
    bool drives;
    uint8_t in;
    uint8_t out;
} ESP32P4FlashWire;

/*
 * One clock of the flash, `level` on its lines: returns the levels it
 * drives, and in *oe the lines it drives.
 */
static uint8_t flash_wire_clock(ESP32P4FlashWire *f, uint8_t level,
                                uint8_t *oe)
{
    unsigned shift;
    uint8_t mask;

    if (f->clock == 0) {
        f->lines = m25p80_io_lines(f->flash);
        f->drives = m25p80_drives_output(f->flash);
        f->in = 0;
        if (f->drives) {
            f->out = ssi_transfer(f->bus, 0xff);
        }
    }
    mask = MAKE_64BIT_MASK(0, f->lines);
    shift = 8 - (f->clock + 1) * f->lines;
    *oe = 0;
    if (f->drives) {
        uint8_t bits = (f->out >> shift) & mask;

        *oe = f->lines == 1 ? BIT(1) : mask;
        level = f->lines == 1 ? bits << 1 : bits;
    } else {
        f->in |= (level & mask) << shift;
        level = 0;
    }
    if (++f->clock * f->lines == 8) {
        if (!f->drives) {
            ssi_transfer(f->bus, f->in);
        }
        f->clock = 0;
    }
    return level;
}

/*
 * The flash's byte the chip select ends part way: the clocks it took, the
 * rest high, as an input byte goes to the model whole (the parts refuse
 * to act on a program, erase or status write clocked past its last byte).
 */
static void flash_wire_end(ESP32P4FlashWire *f)
{
    if (f->clock && !f->drives) {
        ssi_transfer(f->bus,
                     f->in | MAKE_64BIT_MASK(0, 8 - f->clock * f->lines));
    }
}

/*
 * The bits clock `k` of a phase carries from a value of `nbits` bits
 * aligned to bit 63, `w` a clock: the earliest on the highest line.
 */
static uint8_t wire_bits(uint64_t v, unsigned nbits, uint64_t k, unsigned w)
{
    uint8_t r = 0;

    for (unsigned j = 0; j < w; j++) {
        uint64_t idx = k * w + j;

        r = r << 1 | (idx < nbits ? (v >> (63 - idx)) & 1 : 0);
    }
    return r;
}

/* What the controller drives on IO0 to IO3 for a phase of `w` lines. */
static uint8_t wire_oe(unsigned w)
{
    return w == 1 ? 0xd : 0xf;
}

/*
 * The flash's read data as the controller's samplers see them, in the
 * timing calibration of SPI_MEM_C (TIMING_CALI, DIN_MODE, DIN_NUM; SPI1's
 * TIMING_CALI for its extra dummy cycles), which the controller takes up
 * when TIMING_CALI_UPDATE is set, as ESP-IDF's mspi_timing_ll sets it
 * after each change. The TRM does not document it; the headers name the
 * fields, and ESP-IDF's timing tuning (mspi_timing_by_flash_delay.c, the
 * P4's mspi_timing_tuning_configs.h) is what shows their effect.
 *
 * A line's level for a clock of the read data is the flash's from a
 * time after the falling edge that launches it until a time after the
 * next one: the model takes them as FLASH_VALID_PS and FLASH_HOLD_PS, as
 * the controller's sampler sees them, the flash's output delays (the
 * GD25Q256E's tCLQV and tCLQX) and the board's and pads' together, which
 * no document gives. Between those, while the line changes, a sample
 * reads the opposite of the level it changes to. The controller samples
 * each data clock at its rising edge (half a clock after the launch),
 * moved by the calibration:
 *
 *   - EXTRA_DUMMY_CYCLELEN (with TIMING_CALI set) more dummy clocks
 *     before the data, so that each data clock is sampled that many
 *     clocks later, catching a flash whose data come late;
 *   - each line's DINn_MODE and DINn_NUM (with TIMING_CLK_ENA set),
 *     which delay the line before it is sampled, so that it is sampled
 *     earlier (flash_din_advance_ps()). The headers name the modes: "the
 *     input signals are delayed by system clock cycles, 0: input without
 *     delayed, 1: input with the posedge of clk_apb, 2 input with the
 *     negedge of clk_apb, 3: input with the posedge of clk_160, 4 input
 *     with the negedge of clk_160, 5: input with the spi_clk high edge,
 *     6: input with the spi_clk low edge"; 7 they leave out. Mode 0 no
 *     delay; mode 2 samples (NUM + 5) quarters of a core clock earlier,
 *     as ESP-IDF's tuning table orders its settings at a 120 MHz core
 *     clock: its twelve {mode, num, extra dummy} points step the sample a
 *     quarter clock at a time, from a quarter clock before the edge
 *     ({2, 0, 1}) to two and a half clocks after ({2, 1, 4}), mode 0
 *     points at whole clocks. ESP-IDF's P4 tables use no other mode, and
 *     no document times them, so the model derives them from mode 2 by
 *     the edges the headers name: mode 1, the same clock's other edge,
 *     half a core clock less, (NUM + 3) quarters (where the ESP32-S3's
 *     tables order {1, 0, 1} between {2, 2, 2} and {2, 0, 2}); modes 3
 *     and 4 as 1 and 2 on a 160 MHz clock's period; modes 5 and 6 to the
 *     SPI clock's (NUM + 1)th rising edge, or its falling edge before
 *     that, NUM + 1 or NUM + 1/2 SPI clocks earlier. Mode 7 is logged as
 *     a guest error and taken as mode 0. Board checks: at 120 MHz,
 *     {6, 0, 1}, {3, 0, 1} and {1, 2, 2} read right, {5, 0, 1},
 *     {4, 0, 1} and {1, 1, 2} wrong.
 *
 * The constants put the window where ESP-IDF finds it: at 80 MHz and
 * below, where ESP-IDF does not tune and the boot ROM and bootloader read
 * with the calibration as from reset, the untuned sample reads right; at
 * the 120 MHz ESP-IDF tunes (a 120 MHz core clock undivided), it does
 * not, and of ESP-IDF's twelve points the three from {2, 2, 2} to
 * {2, 0, 2} read right, its sweep keeping the middle one, {2, 1, 2}, and
 * its default {2, 2, 2} reading right too. A sample before the window
 * reads the clock before's level, or a changing line; after it, the
 * clock after's.
 */
#define FLASH_VALID_PS      5000
#define FLASH_HOLD_PS       3000

/* The core clock's period in ps, 0 while it is stopped. */
static int64_t mspi_core_ps(ESP32P4MSPIState *s)
{
    return clock_is_enabled(s->core_clk)
           ? (int64_t)((clock_get(s->core_clk) * 1000) >> 32) : 0;
}

/* clk_160's period, ps, for DIN modes 3 and 4. */
#define FLASH_CLK160_PS     6250

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * How much earlier than its edge flash data line `io` is sampled, ps, at
 * core clock period `core_ps` and SPI clock period `t_ps`.
 */
static int64_t flash_din_advance_ps(ESP32P4MSPIState *s, unsigned io,
                                    int64_t core_ps, int64_t t_ps)
{
    unsigned mode = extract32(s->tune_din_mode, 3 * io, 3);
    int64_t num = extract32(s->tune_din_num, 2 * io, 2);

    if (!(s->tune_cali[0] & TIMING_CLK_ENA)) {
        return 0;
    }
    switch (mode) {
    case 0:
        return 0;
    case 1:
        return (num + 3) * core_ps / 4;
    case 2:
        return (num + 5) * core_ps / 4;
    case 3:
        return (num + 3) * FLASH_CLK160_PS / 4;
    case 4:
        return (num + 5) * FLASH_CLK160_PS / 4;
    case 5:
        return (num + 1) * t_ps;
    case 6:
        return num * t_ps + t_ps / 2;
    default:
        if (!(s->tune_logged & BIT(io))) {
            s->tune_logged |= BIT(io);
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DIN%u_MODE %u, which no "
                          "document describes: line IO%u is sampled as with "
                          "mode 0\n", mspi_name(s), io, mode, io);
        }
        return 0;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * The level line `io` is sampled at for read data clock `c` (counted from
 * the transaction's first clock), from the levels `bus` the lines had at
 * each of the transaction's `total` clocks, `idle` past them, at SPI
 * clock period `t_ps` (0: sampled at the clock, as the timing cannot be
 * told) with the data's extra dummy clocks already in `c`.
 */
static bool flash_sample(ESP32P4MSPIState *s, const uint8_t *bus,
                         uint64_t total, uint8_t idle, uint64_t c,
                         unsigned io, int64_t t_ps, int64_t core_ps)
{
    int64_t at, u, k, w;

    if (!t_ps) {
        return (bus[c] >> io) & 1;
    }
    at = (int64_t)c * t_ps + t_ps / 2 -
         flash_din_advance_ps(s, io, core_ps, t_ps);
    u = at - FLASH_VALID_PS;
    k = u >= 0 ? u / t_ps : -((-u + t_ps - 1) / t_ps);
    w = u - k * t_ps;
    if (w <= t_ps + FLASH_HOLD_PS - FLASH_VALID_PS) {
        return k >= 0 && k < (int64_t)total ? (bus[k] >> io) & 1
                                            : (idle >> io) & 1;
    }
    k++;
    return !(k >= 0 && k < (int64_t)total ? (bus[k] >> io) & 1
                                          : (idle >> io) & 1);
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * Transaction `x` on the flash on chip select 0, which the caller
 * selects, a clock of the SPI bus at a time: the command, the address and
 * the write data from the controller's lines in each phase, its dummy
 * phase's levels, and the read data from the lines it samples into `in`,
 * as the flash takes and sends each byte on its own lines for its command
 * (m25p80_io_lines()), whatever the controller's phases are. A phase on
 * one or two lines leaves IO2 and IO3, WP# and HOLD#, driven at the
 * controller's idle levels; a line neither drives rests at its pad's
 * pull. While HOLD# is its function (QE clear) and IO3 is low, the flash
 * ignores the clock. A transaction that reads data has the timing
 * calibration's extra dummy clocks before them, which the flash takes as
 * any other clocks, and its data sampled as flash_sample() has it.
 */
static void flash_wire_xfer(ESP32P4MSPIState *s, const ESP32P4MSPIXfer *x,
                            uint8_t *in)
{
    ESP32P4FlashWire f = { .flash = ssi_get_cs(s->bus, 0), .bus = s->bus };
    unsigned widest = MAX(MAX(x->cmd_lines, x->addr_lines),
                          MAX(x->din_lines, x->dout_lines));
    uint8_t idle_oe = widest <= 2 ? 0xc : 0;
    uint8_t pads = flash_pads_idle(s);
    bool hold_en = m25p80_hold_enabled(f.flash);
    uint64_t addr_at = DIV_ROUND_UP(x->cmd_bits, x->cmd_lines);
    uint64_t dummy_at = addr_at + DIV_ROUND_UP(x->addr_bits, x->addr_lines);
    uint64_t out_at = dummy_at + x->dummy;
    uint64_t in_at = out_at + DIV_ROUND_UP(x->mosi_bytes * 8, x->dout_lines);
    uint64_t data_at = in_at + (x->miso_bytes ? x->extra_dummy : 0);
    uint64_t total = data_at + DIV_ROUND_UP((uint64_t)x->miso_bytes * 8,
                                            x->din_lines);
    g_autofree uint8_t *bus_at = g_malloc0(total + 1);
    int64_t core_ps = mspi_core_ps(s);
    uint64_t div = x->clock & CLOCK_CLK_EQU_SYSCLK
                   ? 1 : CLOCK_CLKCNT_N(x->clock) + 1;
    int64_t t_ps = core_ps * div;
    uint64_t got = 0;

    memset(in, 0, x->miso_bytes);
    for (uint64_t c = 0; c < total; c++) {
        uint8_t level = x->idle_level, oe = idle_oe, foe = 0, fv = 0;
        uint8_t at_flash;

        if (c < addr_at) {
            oe = wire_oe(x->cmd_lines);
            level = wire_bits((uint64_t)x->cmd << (64 - x->cmd_bits),
                              x->cmd_bits, c, x->cmd_lines) |
                    (x->cmd_lines <= 2 ? x->idle_level : 0);
        } else if (c < dummy_at) {
            oe = wire_oe(x->addr_lines);
            level = wire_bits(x->addr_wire, x->addr_bits, c - addr_at,
                              x->addr_lines) |
                    (x->addr_lines <= 2 ? x->idle_level : 0);
        } else if (c < out_at) {
            if (x->dummy_idle) {
                bus_at[c] = pads;
                continue;
            }
            if (c - dummy_at < x->dummy_out) {
                oe = 0xf;
                level = x->dummy_level;
            }
        } else if (c < in_at) {
            unsigned w = x->dout_lines;
            uint64_t k = c - out_at, b = k * w / 8;

            oe = wire_oe(w);
            level = wire_bits((uint64_t)x->mosi[b] << 56, 8, k - b * 8 / w,
                              w) |
                    (w <= 2 ? x->idle_level : 0);
        }
        level &= oe;
        at_flash = level | (pads & ~oe);
        if (!(hold_en && !(at_flash & BIT(3)))) {
            fv = flash_wire_clock(&f, at_flash, &foe);
        }
        bus_at[c] = (fv & foe) | (level & ~foe) | (pads & ~(foe | oe));
    }
    flash_wire_end(&f);

    for (uint64_t c = data_at; c < total; c++) {
        unsigned in_w = x->din_lines;

        for (unsigned j = 0; j < in_w && got < x->miso_bytes * 8ull;
             j++, got++) {
            unsigned io = in_w == 1 ? 1 : in_w - 1 - j;
            bool bit = io < 4 && flash_sample(s, bus_at, total, pads, c, io,
                                              t_ps, core_ps);

            if (bit) {
                in[got / 8] |= 0x80 >> (got % 8);
            }
        }
    }
}

/* The command SPI0 and SPI1's built-in read send for CTRL's read mode. */
static uint32_t read_mode_cmd(uint32_t ctrl, bool four)
{
    if (!(ctrl & CTRL_FASTRD_MODE)) {
        return four ? FLASH_READ4 : FLASH_READ;
    } else if (ctrl & CTRL_FREAD_QIO) {
        return four ? FLASH_QIOR4 : FLASH_QIOR;
    } else if (ctrl & CTRL_FREAD_DIO) {
        return four ? FLASH_DIOR4 : FLASH_DIOR;
    } else if (ctrl & CTRL_FREAD_QUAD) {
        return four ? FLASH_QOR4 : FLASH_QOR;
    } else if (ctrl & CTRL_FREAD_DUAL) {
        return four ? FLASH_DOR4 : FLASH_DOR;
    }
    return four ? FLASH_FAST_READ4 : FLASH_FAST_READ;
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * SPI0's read of `len` bytes of the flash at `addr`, as its registers set
 * it up: the command USER2 gives with CACHE_FCTRL's CACHE_FLASH_USR_CMD,
 * else CTRL's read mode's, on four lines with FCMD_QUAD; USER1's address
 * phase on the read mode's lines, of the 24-bit address followed by
 * RD_STATUS's WB_MODE, or with CACHE_USR_ADDR_4BYTE of the 32-bit
 * address; USER1's dummy cycles with USR_DUMMY, the first half of them
 * driven at CTRL's dummy-out levels with FDUMMY_RIN; the data on the read
 * mode's lines.
 */
static void spi0_flash_xfer(ESP32P4MSPIState *s, uint32_t addr, uint32_t len,
                            ESP32P4MSPIXfer *x)
{
    uint32_t ctrl = s->spi0[R0_CTRL], fctrl = s->spi0[R0_CACHE_FCTRL];
    uint32_t user = s->spi0[R0_USER], user1 = s->spi0[R0_USER1];
    bool four = fctrl & CACHE_FCTRL_USR_ADDR_4BYTE;
    uint32_t wb = extract32(s->spi0[R0_RD_STATUS], 16, 8);

    memset(x, 0, sizeof(*x));
    x->cmd_lines = ctrl & CTRL_FCMD_QUAD ? 4 : 1;
    x->addr_lines = addr_lines(ctrl, fctrl);
    x->din_lines = din_lines(ctrl, fctrl);
    x->dout_lines = 1;
    if (fctrl & CACHE_FCTRL_FLASH_USR_CMD) {
        x->cmd = USER2_COMMAND_VALUE(s->spi0[R0_USER2]);
        x->cmd_bits = USER2_COMMAND_BITLEN(s->spi0[R0_USER2]) + 1;
    } else {
        x->cmd = read_mode_cmd(ctrl, four);
        x->cmd_bits = 8;
    }
    x->addr = four ? addr : addr & 0xffffff;
    if (xfer_set_addr(x, four ? addr : (addr & 0xffffff) << 8 | wb,
                      USER1_ADDR_BITLEN(user1) + 1) &&
        !(s->spi0_logged & MSPI_LOGGED_ADDR)) {
        s->spi0_logged |= MSPI_LOGGED_ADDR;
        log_addr_undefined(s, "SPI0", x->addr_bits);
    }
    if (user & USER_USR_DUMMY) {
        x->dummy = USER1_DUMMY_CYCLELEN(user1) + 1;
    }
    if (ctrl & CTRL_FDUMMY_RIN) {
        x->dummy_out = DIV_ROUND_UP(x->dummy, 2);
    }
    x->dummy_level = ctrl_dummy_level(ctrl);
    x->dummy_idle = user & USER_USR_DUMMY_IDLE;
    x->idle_level = ctrl_idle_level(ctrl);
    x->miso_bytes = len;
    x->reads = true;
    x->clock = s->spi0[R0_CLOCK];
    if (s->tune_cali[0] & TIMING_CALI_EN) {
        x->extra_dummy = TIMING_EXTRA_DUMMY(s->tune_cali[0]);
    }
}

/* SPI0's read of `len` bytes of the flash at `addr`. */
static void flash_read(ESP32P4MSPIState *s, uint32_t addr, uint8_t *buf,
                       uint32_t len)
{
    ESP32P4MSPIXfer x;

    if (!flash_present(s)) {
        memset(buf, flash_idle_byte(s, din_lines(s->spi0[R0_CTRL],
                                                 s->spi0[R0_CACHE_FCTRL])),
               len);
        return;
    }
    spi0_flash_xfer(s, addr, len, &x);
    spi1_release_cs(s);
    flash_select(s, 0, true);
    flash_wire_xfer(s, &x, buf);
    flash_select(s, 0, false);
}

/* The PSRAM die's transactions */

/*
 * The die's latencies in SPI clocks (APS256XXN datasheet, tables 5, 6 and
 * 15): reads by MR0's read latency code, variable (LC) and fixed (FLC,
 * which is also table 5's "max push out", the latest a refresh pushes a
 * variable-latency read's data); writes by MR4's write latency code (its
 * bits in the datasheet's order). The reserved codes 111 are taken as the
 * longest.
 */
static const uint8_t psram_lc[8] = { 3, 4, 5, 6, 7, 9, 10, 10 };
static const uint8_t psram_flc[8] = { 6, 8, 10, 12, 14, 16, 18, 18 };
static const uint8_t psram_wlc[8] = { 3, 7, 5, 9, 4, 8, 6, 9 };

/*
 * The fastest clock each latency code allows, kHz: table 5's "max input
 * CLK freq" for a read latency code, variable or fixed, and table 15's
 * Fmax for a write latency code (by MR4[7:5]'s value), the 66, 133 and
 * 166 MHz grades taken as the 15, 7.5 and 6 ns clocks they stand for (the
 * last two table 30's). The reserved codes as the longest's.
 */
static const uint32_t psram_lc_fmax_khz[8] = {
    66667, 109000, 133333, 166667, 200000, 225000, 250000, 250000,
};
static const uint32_t psram_wlc_fmax_khz[8] = {
    66667, 200000, 133333, 250000, 109000, 225000, 166667, 250000,
};

/*
 * Whether `hz` is faster than `khz` allows: by more than the clock tree's
 * rounding of a period to a whole unit, so that 250 MHz from a 500 MHz
 * MPLL is not.
 */
static bool psram_too_fast(uint64_t hz, uint32_t khz)
{
    return hz > (uint64_t)khz * 1000 + khz;
}

static unsigned psram_rl_code(ESP32P4MSPIState *s)
{
    return extract32(s->psram_mr_regs[0], 2, 3);
}

static unsigned psram_wl_code(ESP32P4MSPIState *s)
{
    return extract32(s->psram_mr_regs[4], 5, 3);
}

/*
 * The die's self-refresh (it manages its own, the datasheet's features),
 * which a read in variable latency (MR0's LT clear) can find under way:
 * "in case of internal refresh insertion, variable latency output data
 * may be delayed by up to (LCx2)" (7.5, figure 7), with nothing on DQS
 * during the command to tell the controller, DQS low from A1 until the
 * data come. The datasheet gives no refresh interval. The model refreshes
 * the die's 16384 rows (RA[13:0]) each within 64 ms at 1x, DRAM's
 * retention period, a row every 3.906 us, which tCEM, the longest CE#
 * may stay low (4 us at standard temperature, table 30) for the die to
 * keep up, fits; MR4's RF sets the rate: always 4x (x0, the default), 1x
 * (01) or 0.5x (11) as temperature allows, which at the model's room
 * temperature it does. A refresh takes tRC (60 ns, table 30); the phase
 * of the die's refresh timer, which no document gives, puts each
 * refresh midway through its interval from power-on (virtual time 0, the
 * die keeping its timer through every reset of the chip). A memory read
 * whose command reaches the die during a refresh collides with it, and
 * its data come at the longest push-out, table 5's: FLC clocks. A
 * register read is never pushed out (table 6), nor is a write, nor a
 * read in fixed latency, whose FLC covers the refresh.
 */
#define PSRAM_ROWS          16384
#define PSRAM_RETENTION_NS  64000000
#define PSRAM_TRC_NS        60
/*
 * The refreshes repeat their timing every 15625 ns: the retention period
 * over its greatest common divisor with every rate's count of refreshes
 * (4096).
 */
#define PSRAM_REFRESH_CYCLE_NS 15625

/* The refreshes the die makes in the retention period, by MR4's RF. */
static uint64_t psram_refreshes(ESP32P4MSPIState *s)
{
    switch (extract32(s->psram_mr_regs[4], 3, 2)) {
    case 1:
        return PSRAM_ROWS;
    case 3:
        return PSRAM_ROWS / 2;
    default:
        return PSRAM_ROWS * 4;
    }
}

/* MR3's SRF, the refresh rate MR4's RF gives (table 14). */
static uint8_t psram_mr3(ESP32P4MSPIState *s)
{
    static const uint8_t srf[4] = { 2, 0, 2, 1 };

    return deposit32(s->psram_mr_regs[3], 4, 2,
                     srf[extract32(s->psram_mr_regs[4], 3, 2)]);
}

/*
 * Whether a command at virtual time `when` (ns, -1 unknown) meets a
 * refresh: refresh k of n in the retention period at (k + 1/2) times
 * the period over n.
 */
static bool psram_refresh_collides(ESP32P4MSPIState *s, int64_t when)
{
    uint64_t n = psram_refreshes(s);
    uint64_t phase;

    if (when < 0) {
        return false;
    }
    phase = ((uint64_t)when % PSRAM_REFRESH_CYCLE_NS * n +
             PSRAM_RETENTION_NS / 2) % PSRAM_RETENTION_NS;
    return phase < PSRAM_TRC_NS * n;
}

/*
 * The die's latency for instruction `inst` at SPI clock `hz`, its command
 * at `when` (see psram_refresh_collides()): a register write's is 1; a
 * register read's LC, less one above 200 MHz; a memory read's FLC in fixed
 * latency, else LC, or FLC when a refresh collides; a memory write's WLC.
 */
static unsigned psram_latency(ESP32P4MSPIState *s, uint8_t inst,
                              uint64_t hz, int64_t when)
{
    uint8_t mr0 = s->psram_mr_regs[0];
    unsigned rl = psram_rl_code(s);

    switch (inst) {
    case PSRAM_INST_REG_WRITE:
        return 1;
    case PSRAM_INST_REG_READ:
        return psram_lc[rl] - (hz > 200000000 ? 1 : 0);
    case PSRAM_INST_SYNC_WRITE:
    case PSRAM_INST_BURST_WRITE:
        return psram_wlc[psram_wl_code(s)];
    default:
        if ((mr0 & PSRAM_MR0_LT) || psram_refresh_collides(s, when)) {
            return psram_flc[rl];
        }
        return psram_lc[rl];
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * Where unit `i` of a burst from die address `a` lies in the array, as a
 * byte offset into it (rows of PSRAM_PAGE bytes, one after another). The
 * die decodes its 32-bit address as its row, RA[13:0] in bits 24:11, and
 * its column, CA[10:0] in bits 10:0 (7.4's command table, A3 to A0), the
 * bits above 24 unused. It is byte-addressed in X8 and word-addressed in
 * X16 (7.1): a unit is a byte, CA[10:0] counting bytes of the row, or in
 * X16 a 16-bit word, CA[9:0] counting the row's 1024 words and CA[10]
 * ignored (7.4, table 17), the word's even byte on DQ0 to DQ7. A linear
 * burst (20h, A0h) wraps within the row, or with `cross` (a linear burst
 * read crossing rows, psram_rbx_wait()) goes on into the next row, from
 * its column 0 (RA+1, table 21); a synchronous one within MR8's burst
 * length (16, 32 or 64 units, or the row), and with hybrid wrap (MR8's
 * BT) only once, going on through the row from the end of the wrapped
 * burst (tables 20 and 21).
 */
static uint32_t psram_cell(ESP32P4MSPIState *s, uint32_t a, uint32_t i,
                           bool linear, bool cross)
{
    static const uint32_t lens[3] = { 16, 32, 64 };
    bool x16 = psram_x16(s);
    uint32_t cols = x16 ? PSRAM_PAGE / 2 : PSRAM_PAGE;
    uint32_t col = a & (cols - 1);
    uint32_t row = extract32(a, 11, 14);
    uint8_t mr8 = s->psram_mr_regs[8];

    if (linear && cross) {
        row = (row + (col + (uint64_t)i) / cols) % PSRAM_ROWS;
        col = (col + (uint64_t)i) % cols;
        return row * PSRAM_PAGE + (x16 ? 2 * col : col);
    }
    if (!linear && (mr8 & 3) != 3) {
        uint32_t bl = lens[mr8 & 3];
        uint32_t block = col & ~(bl - 1);

        if (!(mr8 & PSRAM_MR8_BT) || i < bl) {
            col = block + (col - block + i) % bl;
            i = 0;
        } else {
            col = block + bl;
            i -= bl;
        }
    }
    col = (col + i) % cols;
    return row * PSRAM_PAGE + (x16 ? 2 * col : col);
}

static uint8_t psram_byte(ESP32P4MSPIState *s, uint32_t cell)
{
    uint8_t *mem = memory_region_get_ram_ptr(s->psram_mr);

    return mem[cell % psram_size(s)];
}

static void psram_set_byte(ESP32P4MSPIState *s, uint32_t cell, uint8_t v)
{
    uint8_t *mem = memory_region_get_ram_ptr(s->psram_mr);
    uint32_t off = cell % psram_size(s);

    mem[off] = v;
    memory_region_set_dirty(s->psram_mr, off, 1);
}

/*
 * Whether to log a hazard `bit`: each time without `logged`, else once
 * until `logged` is cleared.
 */
static bool psram_log(uint32_t *logged, uint32_t bit)
{
    if (!logged) {
        return true;
    }
    if (*logged & bit) {
        return false;
    }
    *logged |= bit;
    return true;
}

/* The mode register a register read sends after register `ma`'s. */
static unsigned psram_mr_next(unsigned ma)
{
    return ma == 4 ? 8 : ma == 8 ? 0 : ma + 1;
}

static uint8_t psram_read_mr(ESP32P4MSPIState *s, unsigned mr)
{
    if (mr >= ESP32P4_MSPI_PSRAM_MRS) {
        return 0;
    }
    return mr == 3 ? psram_mr3(s) : s->psram_mr_regs[mr];
}

/*
 * The levels the die drives on DQ0 to DQ15 (DQ0 to DQ7 in bits 7:0) with
 * unit `j` of a read with instruction `inst` from `addr` (the register,
 * for a register read): nothing for a negative `j` (before its first
 * beat, or while it waits to cross a row), and on DQ8 to DQ15 nothing in
 * X8 nor for a register read, whose two registers repeat on DQ0 to DQ7
 * (figure 12). The lines nothing drives read 0. With `garbled` (a memory
 * read the die cannot fetch in time) every driven line carries the
 * opposite of its data's bit. With `cross` a linear burst goes on across
 * rows.
 */
static uint16_t psram_die_beat(ESP32P4MSPIState *s, uint8_t inst,
                               uint32_t addr, int64_t j, bool garbled,
                               bool cross)
{
    bool linear = inst == PSRAM_INST_BURST_READ;
    uint32_t cell;

    if (j < 0) {
        return 0;
    }
    if (inst == PSRAM_INST_REG_READ) {
        unsigned mr = j & 1 ? psram_mr_next(addr & 0xff) : addr & 0xff;

        return psram_read_mr(s, mr);
    }
    cell = psram_cell(s, addr, j, linear, cross);
    if (psram_x16(s)) {
        return (psram_byte(s, cell) | psram_byte(s, cell + 1) << 8) ^
               (garbled ? 0xffff : 0);
    }
    return psram_byte(s, cell) ^ (garbled ? 0xff : 0);
}

/*
 * The controller's data beat `i` of `data` on `lines` data lines: two
 * bytes on DQ0 to DQ15 (the first on DQ0 to DQ7), one on DQ0 to DQ7, or
 * the bits of each beat on DQ0 up (one line: DQ0).
 */
static uint16_t psram_ctrl_beat(const uint8_t *data, uint32_t len,
                                unsigned lines, uint64_t i)
{
    uint16_t v = 0;

    if (lines >= 8) {
        for (unsigned b = 0; b < lines / 8; b++) {
            uint64_t at = i * (lines / 8) + b;

            v |= (at < len ? data[at] : 0) << (8 * b);
        }
        return v;
    }
    for (unsigned j = 0; j < lines; j++) {
        uint64_t bit = i * lines + j;

        v = v << 1 | (bit < len * 8ull ? (data[bit / 8] >> (7 - bit % 8)) & 1
                                       : 0);
    }
    return v;
}

/* Store beat `i` the controller samples, `v` on DQ0 to DQ15, in `data`. */
static void psram_ctrl_take(uint8_t *data, uint32_t len, unsigned lines,
                            uint64_t i, uint16_t v)
{
    if (lines >= 8) {
        for (unsigned b = 0; b < lines / 8; b++) {
            uint64_t at = i * (lines / 8) + b;

            if (at < len) {
                data[at] = v >> (8 * b);
            }
        }
        return;
    }
    if (lines == 1) {
        v >>= 1;
    }
    for (unsigned j = 0; j < lines; j++) {
        uint64_t bit = i * lines + j;

        if (bit < len * 8ull && (v >> (lines - 1 - j)) & 1) {
            data[bit / 8] |= 0x80 >> (bit % 8);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * Row boundary crossing (RBX; APS256XXN datasheet 7.4, 7.5, figure 6,
 * tables 13 and 21): a linear burst read (20h) with MR8's RBX set goes on
 * from the end of its row into the next (RA+1, from column 0) on a die
 * that has the function, as this one does (MR3's RBXen), the die waiting
 * tRBXwait at each crossing, DQS low and its data lines undriven. Table
 * 30 gives tRBXwait from VL to VL + 2 clocks (VL the read latency code's
 * LC, up to 200 MHz), not which: the model takes the longest, VL + 2. With
 * read latency code 5 or 6 (225 and 250 MHz) the function "cannot be
 * used" (table 5, note 1): the die keeps such a read within its row, as
 * without it, which is logged. Every other read, and every write, stays
 * within its row (table 21). Returns the beats of the die's wait at each
 * crossing for a read with instruction `inst` (two a clock), 0 where it
 * does not cross.
 */
static unsigned psram_rbx_wait(ESP32P4MSPIState *s, uint8_t inst)
{
    unsigned rl = psram_rl_code(s);

    if (inst != PSRAM_INST_BURST_READ ||
        !(s->psram_mr_regs[8] & PSRAM_MR8_RBX) ||
        !(psram_read_mr(s, 3) & PSRAM_MR3_RBXEN) || rl == 5 || rl == 6) {
        return 0;
    }
    return 2 * (psram_lc[rl] + 2);
}

/*
 * The unit of the burst from `addr` the die sends at its data beat `j`,
 * from its first, with a wait of `wait` beats at each row it crosses
 * (psram_rbx_wait()): -1 for a beat of a wait.
 */
static int64_t psram_beat_unit(ESP32P4MSPIState *s, uint32_t addr, int64_t j,
                               unsigned wait)
{
    int64_t cols = psram_x16(s) ? PSRAM_PAGE / 2 : PSRAM_PAGE;
    int64_t first = cols - (addr & (cols - 1)), m;

    if (!wait || j < first) {
        return j;
    }
    j -= first;
    m = j % (wait + cols);
    if (m < wait) {
        return -1;
    }
    return first + j / (wait + cols) * cols + (m - wait);
}

/*
 * The longest CE# may stay low for the die to keep up its refresh, tCEM:
 * 4 us at standard temperature (table 30).
 */
#define PSRAM_TCEM_NS       4000

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * Transaction `x` on the PSRAM die, as the die takes it (APS256XXN
 * datasheet, 7.1 to 7.7): its data phase `data`, `len` bytes, written to
 * it (`write`) or read, on `lines` data lines, at SPI clock `hz`, its
 * command at virtual time `when` (ns; -1 where it cannot be known). The
 * die takes only its OPI DDR form: a 16-bit command on DQ0 to DQ7 in one
 * clock, both its edges (the instruction its first byte), and a 32-bit
 * address in two; anything else it does not decode, and it answers
 * nothing (`what` names the transaction in the log). It then counts its
 * latency for the instruction from its mode registers
 * (psram_latency()), and sends or takes its data from there, a beat on
 * each clock edge: the controller's dummy phase, in clock edges in DDR,
 * matches it at 2 * (latency - 1) edges (as ESP-IDF's settings have it),
 * and a dummy phase longer or shorter shifts the data by as many beats,
 * as does a refresh that pushes a variable-latency read out, and a row
 * boundary crossing's wait (psram_rbx_wait()) the data after it; unless
 * the controller follows the die's strobe (`follow`, its port's variable
 * dummy, see mspi_var_dummy()), when a read takes the die's data from its
 * first strobe, and its beats as the die strobes them, whatever the dummy
 * phase. A read beat before the die drives the lines reads 0; the die
 * takes the beats a write sends late from the lines the dummy phase
 * leaves, as 0, and misses those it sends early; the controller masks the
 * bytes of a beat past its data, and its data phase's first `lead` bytes
 * (where SPI0 starts an access at the die's even address before it). A
 * register write takes its first beat's DQ0 to DQ7 alone. A read the die
 * answers as a write (an instruction for the other direction) writes the
 * undriven lines' 0s; a write it answers as a read writes nothing.
 *
 * A memory access "must start on even addresses (A[0] = 0)" (7.1), the
 * die moving two units a clock; what it does with an odd one no document
 * says: the model's die ignores A[0], starting at the even address below
 * (the pair holding the unit), and logs it. Mode register accesses may
 * start at either.
 *
 * A memory read or write at a clock faster than its latency code allows
 * (table 5, table 15) goes wrong as the datasheet has it: a write's data
 * are corrupted (table 15), and a read's, whose data the die cannot have
 * fetched by its latency, are undefined; the model makes each driven bit
 * of such a read, and each bit such a write stores, the opposite of the
 * data's, and logs it. Register reads and writes, which the die answers
 * from its registers, not its array, keep to their latencies at any
 * clock, as ESP-IDF's reading and writing them at 250 MHz with the
 * power-up codes needs.
 *
 * CE# held low longer than tCEM (PSRAM_TCEM_NS) keeps the die from its
 * refresh; bursts have no limit "as long as tCEM is met" (7.2). What the
 * die loses then no document says: the model keeps its data and logs it.
 */
static void psram_die_xfer(ESP32P4MSPIState *s, const ESP32P4MSPIXfer *x,
                           bool ddr, bool cmd_ddr, unsigned lines,
                           uint8_t *data, uint32_t len, bool write,
                           unsigned lead, bool follow, uint64_t hz,
                           int64_t when, const char *what, uint32_t *logged)
{
    uint32_t addr = x->addr_wire >> 32;
    uint64_t beats = DIV_ROUND_UP((uint64_t)len * 8, lines);
    uint8_t inst = x->cmd >> 8;
    bool die_writes, memory, garbled = false, cross;
    unsigned latency, wait;
    int64_t k, end;

    if (!write) {
        memset(data, 0, len);
    }
    if (!ddr || !cmd_ddr || x->cmd_bits != 16 || x->cmd_lines != 8 ||
        x->addr_bits != 32 || x->addr_lines != 8) {
        if (psram_log(logged, MSPI_LOGGED_PSRAM_CMD)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s to the PSRAM die is not an "
                          "OPI DDR command (a %u-bit command on %u lines, a "
                          "%u-bit address on %u lines, %s): the die takes no "
                          "command and answers nothing\n", mspi_name(s),
                          what, x->cmd_bits, x->cmd_lines, x->addr_bits,
                          x->addr_lines, ddr ? (cmd_ddr ? "DDR"
                                                        : "command in SDR")
                                             : "SDR");
        }
        return;
    }
    switch (inst) {
    case PSRAM_INST_SYNC_READ:
    case PSRAM_INST_BURST_READ:
    case PSRAM_INST_REG_READ:
        die_writes = false;
        break;
    case PSRAM_INST_SYNC_WRITE:
    case PSRAM_INST_BURST_WRITE:
    case PSRAM_INST_REG_WRITE:
        die_writes = true;
        break;
    case PSRAM_INST_RESET:
        psram_reset_mrs(s);
        return;
    default:
        if (psram_log(logged, MSPI_LOGGED_PSRAM_CMD)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: unknown PSRAM instruction "
                          "0x%02x\n", mspi_name(s), what, inst);
        }
        return;
    }
    memory = inst != PSRAM_INST_REG_READ && inst != PSRAM_INST_REG_WRITE;
    if (memory && (addr & 1)) {
        if (psram_log(logged, MSPI_LOGGED_PSRAM_ODD)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at odd PSRAM address "
                          "0x%08" PRIx32 ": a memory access must start at an "
                          "even one; the die starts at 0x%08" PRIx32 "\n",
                          mspi_name(s), what, addr, addr & ~1u);
        }
        addr &= ~1u;
    }
    follow = follow && !die_writes && !write;
    latency = psram_latency(s, inst, hz, when);
    if (hz) {
        /* The command, the address, the dummy phase and the data. */
        uint64_t clocks = 3 + DIV_ROUND_UP((follow ? 2 * (latency - 1)
                                                   : x->dummy) + beats, 2);

        if (clocks * NANOSECONDS_PER_SECOND / hz > PSRAM_TCEM_NS &&
            psram_log(logged, MSPI_LOGGED_PSRAM_TCEM)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s holds the PSRAM die's CE# "
                          "low for %" PRIu64 " clocks at %" PRIu64 " Hz, "
                          "longer than tCEM (%u ns): the die cannot keep up "
                          "its refresh\n", mspi_name(s), what, clocks, hz,
                          PSRAM_TCEM_NS);
        }
    }
    if (memory && !die_writes &&
        psram_too_fast(hz, psram_lc_fmax_khz[psram_rl_code(s)])) {
        garbled = true;
        if (psram_log(logged, MSPI_LOGGED_PSRAM_FMAX)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at %" PRIu64 " Hz, faster "
                          "than the PSRAM die's read latency code %u allows "
                          "(%u kHz): its data are undefined, and read as "
                          "their opposite\n", mspi_name(s), what, hz,
                          psram_rl_code(s),
                          psram_lc_fmax_khz[psram_rl_code(s)]);
        }
    } else if (memory && die_writes &&
               psram_too_fast(hz, psram_wlc_fmax_khz[psram_wl_code(s)])) {
        garbled = true;
        if (psram_log(logged, MSPI_LOGGED_PSRAM_FMAX)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at %" PRIu64 " Hz, faster "
                          "than the PSRAM die's write latency code %u allows "
                          "(%u kHz): the data written are corrupted, stored "
                          "as their opposite\n", mspi_name(s), what, hz,
                          psram_wl_code(s),
                          psram_wlc_fmax_khz[psram_wl_code(s)]);
        }
    }
    if (memory && !die_writes && !follow &&
        !(s->psram_mr_regs[0] & PSRAM_MR0_LT) &&
        latency != psram_lc[psram_rl_code(s)] &&
        psram_log(logged, MSPI_LOGGED_PSRAM_REFRESH)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s collided with the PSRAM die's "
                      "refresh, which pushed its variable-latency data out "
                      "from %u to %u clocks\n", mspi_name(s), what,
                      psram_lc[psram_rl_code(s)], latency);
    }
    k = follow ? 0 : (int64_t)x->dummy - 2 * ((int64_t)latency - 1);
    if (k && inst != PSRAM_INST_REG_READ &&
        psram_log(logged, MSPI_LOGGED_PSRAM_LAT)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s's dummy phase of %u edges "
                      "does not match the PSRAM die's latency of %u clocks "
                      "(%u edges): its data is %" PRId64 " beats %s\n",
                      mspi_name(s), what, x->dummy, latency,
                      2 * (latency - 1), k < 0 ? -k : k,
                      k < 0 ? "early" : "late");
    }
    if (die_writes != write && psram_log(logged, MSPI_LOGGED_PSRAM_CMD)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s with PSRAM instruction 0x%02x "
                      "for the other direction\n", mspi_name(s), what, inst);
    }
    if (!die_writes) {
        uint32_t cols = psram_x16(s) ? PSRAM_PAGE / 2 : PSRAM_PAGE;

        wait = psram_rbx_wait(s, inst);
        cross = wait != 0;
        if (inst == PSRAM_INST_BURST_READ && !cross &&
            (s->psram_mr_regs[8] & PSRAM_MR8_RBX) &&
            (addr & (cols - 1)) + beats + (k > 0 ? k : 0) > cols &&
            psram_log(logged, MSPI_LOGGED_PSRAM_RBX)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s reaches the end of a PSRAM "
                          "row with RBX set at read latency code %u, with "
                          "which the die cannot cross rows: it wraps within "
                          "the row\n", mspi_name(s), what, psram_rl_code(s));
        }
        if (!write) {
            for (uint64_t i = 0; i < beats; i++) {
                int64_t u = follow ? (int64_t)i
                            : (int64_t)i + k < 0 ? -1
                            : psram_beat_unit(s, addr, (int64_t)i + k, wait);

                psram_ctrl_take(data, len, lines, i,
                                psram_die_beat(s, inst, addr, u, garbled,
                                               cross));
            }
        }
        return;
    }
    /* The die takes beats 0 to end - 1, beat j the controller's j - k. */
    end = (int64_t)beats + k;
    if (inst == PSRAM_INST_REG_WRITE) {
        uint16_t v = write && -k >= 0 && -k < (int64_t)beats
                     ? psram_ctrl_beat(data, len, lines, -k) : 0;

        if (end > 0) {
            psram_write_mr(s, addr & 0xff, v & 0xff);
        }
        return;
    }
    for (int64_t j = 0; j < end; j++) {
        int64_t i = j - k;
        uint16_t v = write && i >= 0 ? psram_ctrl_beat(data, len, lines, i)
                                     : 0;
        bool linear = inst == PSRAM_INST_BURST_WRITE;
        uint32_t cell = psram_cell(s, addr, j, linear, false);
        /*
         * The data mask: the controller masks a beat's bytes past its
         * data (an AXI write narrower than the beat), and the `lead`
         * bytes before its own.
         */
        bool lo = !write || i < 0 || lines < 8 ||
                  ((uint64_t)i * (lines / 8) < len &&
                   (uint64_t)i * (lines / 8) >= lead);
        bool hi = !write || i < 0 || lines < 16 ||
                  ((uint64_t)i * 2 + 1 < len && (uint64_t)i * 2 + 1 >= lead);

        if (garbled) {
            v = ~v;
        }
        if (psram_x16(s)) {
            if (lo) {
                psram_set_byte(s, cell, v);
            }
            if (hi) {
                psram_set_byte(s, cell + 1, v >> 8);
            }
        } else if (lo) {
            psram_set_byte(s, cell, v);
        }
    }
}

/* The SPI clock of a port's CLOCK or SRAM_CLK register `clk`. */
static uint64_t mspi_spi_hz(ESP32P4MSPIState *s, uint32_t clk)
{
    uint64_t div = clk & CLOCK_CLK_EQU_SYSCLK ? 1 : CLOCK_CLKCNT_N(clk) + 1;

    return clock_get_hz(s->core_clk) / div;
}

/* The lines of an SPI0 PSRAM phase: SRAM_CMD's and CACHE_SCTRL's bits. */
static unsigned spi0_psram_lines(uint32_t cmd, uint32_t sctrl, uint32_t hex,
                                 uint32_t oct, uint32_t quad, uint32_t dual)
{
    if (cmd & hex) {
        return 16;
    }
    if (cmd & oct) {
        return 8;
    }
    if ((cmd & quad) || (sctrl & SCTRL_SRAM_QIO)) {
        return 4;
    }
    if ((cmd & dual) || (sctrl & SCTRL_SRAM_DIO)) {
        return 2;
    }
    return 1;
}

/*
 * The time of an access made now (ns), for the PSRAM die's refresh: the
 * present, or the time a late round runs at (esp32_line_time()); -1 in
 * the middle of a core's block of code, where under icount the virtual
 * clock cannot be read (an access the caches make for an instruction
 * that does not end its block).
 */
static int64_t mspi_access_time(void)
{
    if (esp32p4_cpu_mid_block()) {
        return -1;
    }
    return esp32_line_time();
}

/*
 * The die address SPI0 sends for byte address `addr` (CACHE_SCTRL's
 * 4-byte form, or the 24-bit address and SWB_MODE): with its data on 16
 * lines (`hex`), where the die it serves is word-addressed in X16, the
 * address of the word holding the byte, its row in bits 24:11 and its
 * word in bits 9:0 ({addr[31:11], 0, addr[10:1]}), so that the window's
 * byte addresses reach the die's array byte for byte as they do in X8 on
 * 8 lines (each 2048-byte row 1024 words). Without it, ESP-IDF's
 * byte-addressed window would reach in X16 only every other word of each
 * row's first half, and each row's second half not at all. On 8 or 16
 * lines the address is the die's even one at or before (the die's memory
 * accesses start there, 7.1): *`lead` is the bytes before `addr` from it,
 * which the access then drops (a read) or masks (a write).
 */
static uint32_t spi0_psram_wire_addr(uint32_t addr, unsigned lines,
                                     uint32_t *lead)
{
    *lead = lines == 16 ? addr & 3 : lines == 8 ? addr & 1 : 0;
    addr -= *lead;
    return lines == 16 ? (addr & ~(PSRAM_PAGE - 1u)) |
                         ((addr & (PSRAM_PAGE - 1u)) >> 1)
                       : addr;
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * Whether a port's DDR register `ddr` (SPI0's SMEM_DDR, SPI1's DDR) has
 * the controller follow the die's strobe for a read's data: its
 * SMEM_VAR_DUMMY (FMEM_VAR_DUMMY), "variable dummy cycle in spi DDR mode"
 * as ESP-IDF's headers have it, the only word any document gives. ESP-IDF
 * sets it on both ports for the PSRAM in use (esp_psram_impl_ap_hex.c,
 * fixed latency, its dummy phases set for FLC), clears it on SPI1 while
 * it tunes the strobe's delay lines (mspi_timing_by_dqs.c) and, on the
 * ESP32-S3, for the flash's timing tuning ("GD flash will read error in
 * variable mode with 20MHz"). The die tells the length of a read's
 * latency only by its strobe, its data beginning at "the 1st DQS/DM
 * rising edge after read pre-amble" (APS256XXN 7.5), anywhere from LC to
 * 2 LC in variable latency: the model takes the bit as a dummy phase as
 * long as that, the controller taking the data from the die's first
 * strobe after the address phase, and each beat as the die strobes it,
 * whatever the dummy phase it was given; which a refresh's push-out, a
 * dummy phase that does not match the die's latency and a row crossing's
 * wait then do not shift. It does so only in DDR mode, on the die's
 * strobe: not with DQS_LOOP (the controller's own) nor with the strobe's
 * input powered down; a read the die does not answer keeps the dummy
 * phase as given. Board checks: in fixed latency with the bit set, a
 * read with a dummy phase 4 edges too long reads the data; in variable
 * latency with it set, no read meets a push-out.
 */
static bool mspi_var_dummy(ESP32P4MSPIState *s, uint32_t ddr)
{
    return (ddr & DDR_FMEM_DDR_EN) && (ddr & DDR_FMEM_VAR_DUMMY) &&
           !(ddr & DDR_DQS_LOOP) &&
           (!s->pins || esp32p4_mspi_iomux_psram_strobe_on(s->pins, 0));
}

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * SPI0's access to the PSRAM die of `len` bytes at `addr` (no more than
 * its page holds), as SPI0's registers set it up: the command of
 * SRAM_DRD_CMD (reads) or SRAM_DWR_CMD (writes) with CACHE_SCTRL's
 * CACHE_SRAM_USR_RCMD or _WCMD (else an SPI READ, 03h, or WRITE, 02h, of
 * 8 bits: the TRM does not name SPI0's own), on SRAM_CMD's command lines;
 * CACHE_SCTRL's address phase, of the 32-bit address with
 * CACHE_USR_SADDR_4BYTE, else of the 24-bit one followed by SWB_MODE, on
 * SRAM_CMD's address lines, a word address with the data on 16 lines
 * (spi0_psram_wire_addr()), the die's even address, an access starting
 * after it masking or dropping the bytes before; the read or write dummy
 * cycles CACHE_SCTRL enables; the data on SRAM_CMD's data lines, 16 with
 * SDIN_HEX or SDOUT_HEX; SMEM_DDR's DDR mode, and its variable dummy
 * (mspi_var_dummy()). The read goes through the pads' capture as SPI0's
 * DDR read.
 */
static void spi0_psram_access(ESP32P4MSPIState *s, uint32_t addr,
                              uint8_t *data, uint32_t len, bool write)
{
    uint32_t sctrl = s->spi0[R0_CACHE_SCTRL], cmd = s->spi0[R0_SRAM_CMD];
    uint32_t ddr = s->spi0[R0_SMEM_DDR];
    uint32_t ucmd = s->spi0[write ? R0_SRAM_DWR_CMD : R0_SRAM_DRD_CMD];
    bool four = sctrl & SCTRL_SADDR_4BYTE;
    unsigned lines;
    uint32_t wire, lead;
    g_autofree uint8_t *buf = NULL;
    ESP32P4MSPIXfer x;

    memset(&x, 0, sizeof(x));
    if (sctrl & (write ? SCTRL_SRAM_USR_WCMD : SCTRL_SRAM_USR_RCMD)) {
        x.cmd = SRAM_CMD_VALUE(ucmd);
        x.cmd_bits = SRAM_CMD_BITLEN(ucmd) + 1;
    } else {
        x.cmd = write ? FLASH_PP : FLASH_READ;
        x.cmd_bits = 8;
    }
    x.cmd_lines = spi0_psram_lines(cmd, sctrl, 0, SCMD_SCMD_OCT,
                                   SCMD_SCMD_QUAD, 0);
    x.addr_lines = spi0_psram_lines(cmd, sctrl, 0, SCMD_SADDR_OCT,
                                    SCMD_SADDR_QUAD, SCMD_SADDR_DUAL);
    lines = write ? spi0_psram_lines(cmd, sctrl, SCMD_SDOUT_HEX,
                                     SCMD_SDOUT_OCT, SCMD_SDOUT_QUAD,
                                     SCMD_SDOUT_DUAL)
                  : spi0_psram_lines(cmd, sctrl, SCMD_SDIN_HEX,
                                     SCMD_SDIN_OCT, SCMD_SDIN_QUAD,
                                     SCMD_SDIN_DUAL);
    x.addr = addr;
    wire = spi0_psram_wire_addr(addr, lines, &lead);
    if (xfer_set_addr(&x, four ? wire : (wire & 0xffffff) << 8 |
                      SCMD_SWB_MODE(cmd),
                      SCTRL_ADDR_BITLEN(sctrl) + 1) &&
        !(s->spi0_logged & MSPI_LOGGED_ADDR)) {
        s->spi0_logged |= MSPI_LOGGED_ADDR;
        log_addr_undefined(s, "SPI0", x.addr_bits);
    }
    if (sctrl & (write ? SCTRL_WR_SRAM_DUMMY : SCTRL_RD_SRAM_DUMMY)) {
        x.dummy = (write ? SCTRL_WDUMMY_CYCLELEN(sctrl)
                         : SCTRL_RDUMMY_CYCLELEN(sctrl)) + 1;
    }
    buf = g_malloc0(len + lead);
    if (write) {
        memcpy(buf + lead, data, len);
    }
    psram_die_xfer(s, &x, ddr & DDR_FMEM_DDR_EN, !(ddr & DDR_FMEM_DDR_CMD_DIS),
                   lines, buf, len + lead, write, lead,
                   mspi_var_dummy(s, ddr),
                   mspi_spi_hz(s, s->spi0[R0_SRAM_CLK]), mspi_access_time(),
                   write ? "SPI0's write" : "SPI0's read", &s->spi0_logged);
    if (!write) {
        psram_capture(s, false, lines == 16, buf, len + lead);
        memcpy(data, buf + lead, len);
    }
}

/*
 * SPI0's access to the PSRAM of `len` bytes at `addr`, split, with
 * SMEM_AC's SMEM_SPLIT_TRANS_EN (set from reset), into a transaction for
 * each page of SMEM_ECC_CTRL's SMEM_PAGE_SIZE it touches. ESP-IDF sets the
 * page to the die's 2048-byte row (psram_ctrlr_ll_set_page_size()), so
 * that no transaction reaches a row's end: the synchronous reads and
 * writes it sends wrap within the row, and only a linear burst read with
 * MR8's RBX crosses it (psram_rbx_wait()). Without the split, a
 * transaction past the die's row wraps there or crosses as the die takes
 * its command.
 */
static void psram_spi0(ESP32P4MSPIState *s, uint32_t addr, uint8_t *data,
                       uint32_t len, bool write)
{
    uint32_t page = 256u << SMEM_ECC_PAGE_SIZE(s->spi0[R0_SMEM_ECC_CTRL]);

    while (len) {
        uint32_t n = len;

        if (s->spi0[R0_SMEM_AC] & SMEM_AC_SPLIT_TRANS_EN) {
            n = MIN(len, page - addr % page);
        }
        spi0_psram_access(s, addr, data, n, write);
        addr += n;
        data += n;
        len -= n;
    }
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
        psram_spi0(s, start, tmp, end - start, false);
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
        psram_spi0(s, paddr, (uint8_t *)buf, len, true);
        return;
    }
    start = QEMU_ALIGN_DOWN(paddr, XTS_BLOCK);
    end = QEMU_ALIGN_UP(paddr + len, XTS_BLOCK);
    tmp = g_malloc(end - start);
    if (start != paddr || end != paddr + len) {
        psram_spi0(s, start, tmp, end - start, false);
        xts_crypt(key, 1, start, tmp, end - start, false);
    }
    memcpy(tmp + (paddr - start), buf, len);
    xts_crypt(key, 1, start, tmp, end - start, true);
    psram_spi0(s, start, tmp, end - start, true);
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

/*
 * Why the AXI port takes no access now (see esp32p4_mspi_axi_ready()), or
 * NULL when it does: an AXI slave held in reset or without its bus clock
 * answers nothing; a closed port answers errors at once; a controller
 * without its core clock, or with AXI_REQ_EN clear, queues the access but
 * never runs it.
 */
static const char *mspi_axi_stall(ESP32P4MSPIState *s)
{
    if (mspi_in_reset(s)) {
        return "the controller is held in reset";
    }
    if (!clock_is_enabled(s->sys_clk)) {
        return "the controller's bus clock is stopped";
    }
    if (s->spi0[R0_CACHE_FCTRL] & CACHE_FCTRL_CLOSE_AXI_INF_EN) {
        return NULL;
    }
    if (!clock_is_enabled(s->core_clk)) {
        return "the controller's core clock is stopped";
    }
    if (!(s->spi0[R0_CACHE_FCTRL] & CACHE_FCTRL_AXI_REQ_EN)) {
        return "CACHE_FCTRL's AXI_REQ_EN is clear";
    }
    return NULL;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
bool esp32p4_mspi_axi_ready(ESP32P4MSPIState *s, MemTxAttrs attrs,
                            MemTxResult *res)
{
    const char *why = mspi_axi_stall(s);
    ESP32P4CPU *cpu;

    if (!why) {
        return true;
    }
    cpu = esp32p4_cpu_bus_waiter(attrs);
    if (cpu) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: core %d's access waits on the "
                      "bus, never completing while %s\n", mspi_name(s),
                      CPU(cpu)->cpu_index, why);
        s->axi_waiting |= BIT(CPU(cpu)->cpu_index);
        *res = esp32p4_cpu_bus_hold(cpu);
        return false;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: an access by a master that cannot "
                  "wait here, while %s: refused\n", mspi_name(s), why);
    *res = MEMTX_ERROR;
    return false;
}

static const MemoryRegionOps mspi_window_ops;

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * A DMA master's access the AXI port holds, as it holds a core's: checked
 * by the master before it makes the access, so that it makes the access
 * once the port takes it, not before.
 */
bool esp32p4_mspi_dma_wait(AddressSpace *as, hwaddr addr, hwaddr len,
                           MemTxAttrs attrs, bool write, Notifier *n)
{
    RCU_READ_LOCK_GUARD();

    for (int i = 0; i < 2 && len; i++) {
        hwaddr xlat, l = 1;
        MemoryRegion *mr = address_space_translate(as, i ? addr + len - 1
                                                         : addr,
                                                   &xlat, &l, write, attrs);
        ESP32P4MSPIState *s;
        const char *why;

        if (mr->ops != &mspi_window_ops) {
            continue;
        }
        s = mr->opaque;
        why = mspi_axi_stall(s);
        if (!why) {
            continue;
        }
        if (!QLIST_IS_INSERTED(n, node)) {
            notifier_list_add(&s->dma_waiters, n);
        }
        if (!s->dma_wait_logged) {
            s->dma_wait_logged = true;
            qemu_log_mask(LOG_GUEST_ERROR, "%s: a DMA master's %s at "
                          "0x%08" HWADDR_PRIx " waits on the bus, not "
                          "completing while %s\n", mspi_name(s),
                          write ? "write" : "read", addr, why);
        }
        return true;
    }
    return false;
}

/* The controllers, for esp32p4_mspi_dma_holds(). */
static GSList *mspi_instances;

bool esp32p4_mspi_dma_holds(void)
{
    for (GSList *l = mspi_instances; l; l = l->next) {
        if (mspi_axi_stall(l->data)) {
            return true;
        }
    }
    return false;
}

static void mspi_dma_wait_notify(Notifier *n, void *data)
{
    ESP32P4MspiDmaWait *w = container_of(n, ESP32P4MspiDmaWait, n);

    notifier_remove(n);
    timer_mod(w->timer, esp32_line_time());
}

static void mspi_dma_wait_timer(void *opaque)
{
    ESP32P4MspiDmaWait *w = opaque;

    w->ready(w->opaque);
}

void esp32p4_mspi_dma_wait_init(ESP32P4MspiDmaWait *w,
                                void (*ready)(void *opaque), void *opaque)
{
    w->n.notify = mspi_dma_wait_notify;
    w->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mspi_dma_wait_timer, w);
    w->ready = ready;
    w->opaque = opaque;
}

bool esp32p4_mspi_dma_wait_on(ESP32P4MspiDmaWait *w, AddressSpace *as,
                              hwaddr addr, hwaddr len, MemTxAttrs attrs,
                              bool write)
{
    return len && esp32p4_mspi_dma_wait(as, addr, len, attrs, write, &w->n);
}

void esp32p4_mspi_dma_wait_cancel(ESP32P4MspiDmaWait *w)
{
    if (QLIST_IS_INSERTED(&w->n, node)) {
        notifier_remove(&w->n);
    }
    timer_del(w->timer);
}

void esp32p4_mspi_dma_wait_retry(ESP32P4MspiDmaWait *w)
{
    timer_mod(w->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* Let the cores' held accesses go once the AXI port takes them. */
static void mspi_axi_wake(ESP32P4MSPIState *s)
{
    uint32_t waiting = s->axi_waiting;

    if (mspi_axi_stall(s)) {
        return;
    }
    s->dma_wait_logged = false;
    notifier_list_notify(&s->dma_waiters, s);
    if (!waiting) {
        return;
    }
    s->axi_waiting = 0;
    while (waiting) {
        int n = ctz32(waiting);
        CPUState *cs = qemu_get_cpu(n);

        waiting &= waiting - 1;
        if (cs && object_dynamic_cast(OBJECT(cs), TYPE_ESP32P4_CPU)) {
            esp32p4_cpu_bus_release(ESP32P4_CPU(cs));
        }
    }
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

/* [spec:nuos:req:emu.esp32p4.boot] */
void esp32p4_mspi_raise_state_ends(ESP32P4MSPIState *s)
{
    spi0_raise(s, INT0_SLV_ST_END | INT0_MST_ST_END);
    spi1_raise(s, INT1_SLV_ST_END | INT1_MST_ST_END);
    mspi_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.boot] */
uint32_t esp32p4_mspi_flash_jedec_id(ESP32P4MSPIState *s)
{
    if (!flash_present(s)) {
        return flash_idle_byte(s, 1) * 0x010101u;
    }
    return m25p80_jedec_id(ssi_get_cs(s->bus, 0));
}

/* The window: direct accesses, and the accesses of masters without cache. */

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult mspi_window_read(void *opaque, hwaddr addr, uint64_t *data,
                                    unsigned size, MemTxAttrs attrs)
{
    ESP32P4MSPIState *s = opaque;
    uint8_t buf[8];
    MemTxResult r;

    if (!esp32p4_mspi_axi_ready(s, attrs, &r)) {
        *data = 0;
        return r;
    }
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
    MemTxResult r;

    if (!esp32p4_mspi_axi_ready(s, attrs, &r)) {
        return r;
    }
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

static unsigned spi1_cmd_lines(ESP32P4MSPIState *s)
{
    uint32_t ctrl = s->spi1[R1_CTRL];

    return ctrl & CTRL_FCMD_OCT ? 8 : ctrl & CTRL_FCMD_QUAD ? 4 : 1;
}

static unsigned spi1_addr_lines(ESP32P4MSPIState *s)
{
    return addr_lines(s->spi1[R1_CTRL], s->spi1[R1_CACHE_FCTRL]);
}

static unsigned spi1_din_lines(ESP32P4MSPIState *s)
{
    return din_lines(s->spi1[R1_CTRL], s->spi1[R1_CACHE_FCTRL]);
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
    x->dummy_level = ctrl_dummy_level(s->spi1[R1_CTRL]);
    x->idle_level = ctrl_idle_level(s->spi1[R1_CTRL]);
    x->clock = s->spi1[R1_CLOCK];
    if (!s->psram && (s->tune_cali[1] & TIMING_CALI_EN)) {
        x->extra_dummy = TIMING_EXTRA_DUMMY(s->tune_cali[1]);
    }
}

/*
 * The address phase of `bits` bits from ADDR, its low bits up to 32, as
 * software sets it (ESP-IDF's spimem_flash_ll_set_usr_address()).
 */
static void spi1_xfer_set_addr(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x,
                               unsigned bits)
{
    uint32_t reg = s->spi1[R1_ADDR];

    x->addr = bits < 32 ? reg & MAKE_64BIT_MASK(0, bits) : reg;
    if (xfer_set_addr(x, bits < 32 ? x->addr << (32 - bits) : reg, bits)) {
        log_addr_undefined(s, "SPI1", bits);
    }
}

/*
 * The dummy phase's driven cycles: all of them with CTRL's FDUMMY_RIN for
 * a transaction that reads data, FDUMMY_WOUT for one that does not.
 */
static void spi1_xfer_dummy_out(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    uint32_t ctrl = s->spi1[R1_CTRL];

    x->dummy_out = (ctrl & (x->miso_bytes ? CTRL_FDUMMY_RIN
                                          : CTRL_FDUMMY_WOUT)) ? x->dummy : 0;
}

/*
 * The address phase of the built-in commands: ADDR, 24 or 32 bits; and
 * their lines, one for everything but the read's, which CTRL's read mode
 * sets.
 */
static void spi1_xfer_addr(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x)
{
    spi1_xfer_set_addr(s, x,
                       USER1_ADDR_BITLEN(s->spi1[R1_USER1]) + 1 > 24 ? 32
                                                                     : 24);
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
        spi1_xfer_set_addr(s, x, USER1_ADDR_BITLEN(s->spi1[R1_USER1]) + 1);
    }
    if (user & USER_USR_DUMMY) {
        x->dummy = USER1_DUMMY_CYCLELEN(s->spi1[R1_USER1]) + 1;
    }
    x->dummy_idle = user & USER_USR_DUMMY_IDLE;
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
    spi1_xfer_dummy_out(s, x);
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
           (x->miso_bytes ? x->extra_dummy : 0) +
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

/*
 * Run a transaction on the flash's bus, chip select `cs`, as
 * flash_wire_xfer() puts it on the lines. Nothing is on chip select 1,
 * nor on chip select 0 of a board without flash: unless the flash drives
 * them, selected too, the data lines read as their pads' pulls hold them.
 */
static void spi1_flash_xfer(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x, int cs)
{
    if (cs != 0 || !flash_present(s)) {
        if (cs == 0 || (s->spi1[R1_MISC] & MISC_CS0_DIS) ||
            !flash_present(s)) {
            memset(x->miso, flash_idle_byte(s, x->din_lines),
                   x->miso_bytes);
        }
        return;
    }
    if (x->mosi_bytes && x->addr_bits) {
        xts_substitute(s, x->addr, x->mosi, x->mosi_bytes);
    }
    flash_select(s, 0, true);
    flash_wire_xfer(s, x, x->miso);
    s->spi1_cs_held = true;
}

/*
 * Run a transaction on the PSRAM die, chip select 1, as the die takes it
 * (psram_die_xfer()): SPI1's command, address and dummy phases on CTRL's
 * lines (FCMD_OCT, FADDR_OCT), its data on FDIN_OCT's or FDOUT_OCT's, or
 * on 16 with SPI0's SRAM_CMD SDIN_HEX or SDOUT_HEX, which set the PSRAM
 * controller's data lines for both ports (ESP-IDF's
 * psram_ctrlr_ll_enable_hex_data_line_mode() for either sets them), in
 * SPI1's DDR mode, and its variable dummy (mspi_var_dummy()). A read goes
 * through the pads' capture.
 */
static void spi1_psram_xfer(ESP32P4MSPIState *s, ESP32P4MSPIXfer *x, int cs)
{
    uint32_t ddr = s->spi1[R1_DDR], scmd = s->spi0[R0_SRAM_CMD];
    bool read = x->miso_bytes;
    unsigned lines;

    if (cs != 1) {
        return;
    }
    if (s->psram_halfsleep) {
        /* The chip select's pulse wakes the die; it takes no command. */
        s->psram_halfsleep = false;
        return;
    }
    if (read) {
        lines = scmd & SCMD_SDIN_HEX ? 16 : x->din_lines;
    } else {
        lines = scmd & SCMD_SDOUT_HEX ? 16 : x->dout_lines;
    }
    psram_die_xfer(s, x, ddr & DDR_FMEM_DDR_EN, !(ddr & DDR_FMEM_DDR_CMD_DIS),
                   lines, read ? x->miso : x->mosi,
                   read ? x->miso_bytes : x->mosi_bytes, !read, 0,
                   mspi_var_dummy(s, ddr),
                   mspi_spi_hz(s, s->spi1[R1_CLOCK]), mspi_access_time(),
                   "SPI1's transaction", NULL);
    if (read) {
        psram_capture(s, true, lines == 16, x->miso, x->miso_bytes);
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

/* Time SPI1's cycles from @from: the present, or a phase's end. */
static void spi1_schedule_from(ESP32P4MSPIState *s, int64_t from)
{
    ESP32P4ClkEdges e;

    if (!clock_is_enabled(s->core_clk)) {
        s->spi1_waiting = true;
        return;
    }
    s->spi1_waiting = false;
    e = esp32p4_clk_edges_or_at(s->core_clk, from);
    s->spi1_end_at = esp32p4_clk_edges_after(&e, from,
                                             s->spi1_cycles *
                                             spi1_clock_div(s));
    timer_mod(&s->spi1_timer, s->spi1_end_at);
}

static void spi1_schedule(ESP32P4MSPIState *s)
{
    spi1_schedule_from(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
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

/* [spec:nuos:req:emu.esp32p4.memory] */
static void spi1_start(ESP32P4MSPIState *s, uint32_t cmd)
{
    ESP32P4MSPIXfer x;
    uint32_t op = cmd & CMD_USR ? CMD_USR : BIT(31 - clz32(cmd & CMD_OPS));
    /* FLASH_PE, with any operation, clears as the operation ends. */
    uint32_t busy = op | (cmd & CMD_FLASH_PE);
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
    if (op != CMD_USR) {
        x.cmd_lines = 1;
        if (op != CMD_FLASH_READ) {
            x.addr_lines = 1;
            x.din_lines = 1;
        }
    }

    switch (op) {
    case CMD_USR:
        spi1_xfer_user(s, &x);
        pe = cmd & CMD_FLASH_PE;
        to_buf = x.miso_bytes;
        break;
    case CMD_FLASH_READ:
        x.cmd = read_mode_cmd(s->spi1[R1_CTRL], false);
        spi1_xfer_addr(s, &x);
        if (s->spi1[R1_USER] & USER_USR_DUMMY) {
            x.dummy = USER1_DUMMY_CYCLELEN(s->spi1[R1_USER1]) + 1;
        }
        x.miso_bytes = MIN(DIV_ROUND_UP(DLEN(s->spi1[R1_MISO_DLEN]) + 1, 8),
                           64);
        spi1_xfer_dummy_out(s, &x);
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
        s->spi1_waiti = spi1_waiti_cycles(s);
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

/*
 * The automatic status reads from `from` on: one, then another for as
 * long as the flash, whose cycle ends at `ready`, reads busy. The read
 * starting at or after `ready` finds it idle.
 */
/* [spec:nuos:req:emu.flash-timing] */
static uint64_t spi1_waiti_reads(ESP32P4MSPIState *s, int64_t from,
                                 int64_t ready)
{
    uint64_t per = spi1_waiti_cycles(s) * spi1_clock_div(s);
    ESP32P4ClkEdges e;
    int64_t per_ns;
    uint64_t k;

    if (ready <= from || !clock_is_enabled(s->core_clk)) {
        return 1;
    }
    e = esp32p4_clk_edges_or_at(s->core_clk, from);
    per_ns = MAX(esp32p4_clk_edges_after(&e, from, per) - from, 1);
    k = (ready - from + per_ns - 1) / per_ns;
    while (k > 0 && esp32p4_clk_edges_after(&e, from, (k - 1) * per) >=
                    ready) {
        k--;
    }
    while (esp32p4_clk_edges_after(&e, from, k * per) < ready) {
        k++;
    }
    return k + 1;
}

static void spi1_end(void *opaque)
{
    ESP32P4MSPIState *s = opaque;

    spi1_release_cs(s);
    if (s->spi1_waiti) {
        DeviceState *flash = ssi_get_cs(s->bus, 0);
        int64_t ready = flash ? m25p80_busy_until(flash) : 0;

        s->spi1_waiti = false;
        s->spi1_cycles = spi1_waiti_reads(s, s->spi1_end_at, ready) *
                         spi1_waiti_cycles(s);
        spi1_schedule_from(s, s->spi1_end_at);
        return;
    }
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

/*
 * [spec:nuos:req:emu.esp32p4.memory]
 * End SPI1's transaction if its time has come, and the automatic status
 * reads (FLASH_WAITI_CTRL) that follow it, timed from its end: the timer
 * that ends each may run late on the host's clock, after an access that
 * must find them ended.
 */
static void spi1_catch_up(ESP32P4MSPIState *s)
{
    while (timer_pending(&s->spi1_timer) &&
           timer_expire_time_ns(&s->spi1_timer) <=
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)) {
        timer_del(&s->spi1_timer);
        spi1_end(s);
    }
}

static bool spi1_busy(ESP32P4MSPIState *s)
{
    return s->spi1_busy_cmd || s->spi1_busy_sus;
}

/* End SPI1's transaction now, as its FSM's reset does. */
static void spi1_abort(ESP32P4MSPIState *s)
{
    timer_del(&s->spi1_timer);
    spi1_release_cs(s);
    s->spi1[R1_CMD] &= ~s->spi1_busy_cmd;
    s->spi1[R1_FLASH_SUS_CTRL] &= ~s->spi1_busy_sus;
    s->spi1_busy_cmd = 0;
    s->spi1_busy_sus = 0;
    s->spi1_sus_after = -1;
    s->spi1_end_int = 0;
    s->spi1_waiting = false;
    s->spi1_waiti = false;
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
        if (trig && r == R0_TIMING_CALI && !s->psram) {
            /* TIMING_CALI_UPDATE: both ports take the calibration up. */
            s->tune_cali[0] = s->spi0[R0_TIMING_CALI];
            s->tune_cali[1] = s->spi1[R1_TIMING_CALI];
            s->tune_din_mode = s->spi0[R0_DIN_MODE];
            s->tune_din_num = s->spi0[R0_DIN_NUM];
        } else if (trig && r == R0_XTS_TRIGGER) {
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
        s->spi0_logged = 0;
        break;
    }
    mspi_axi_wake(s);
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
    spi1_catch_up(s);
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
    spi1_catch_up(s);

    switch (r) {
    case R1_CMD:
        start = v & mask & (CMD_OPS | CMD_FLASH_PE);
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
            spi1_start(s, s->spi1[R1_CMD] & (CMD_OPS | CMD_FLASH_PE));
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
    s->spi0_logged = 0;
    s->tune_cali[0] = s->spi0[R0_TIMING_CALI];
    s->tune_cali[1] = s->spi1[R1_TIMING_CALI];
    s->tune_din_mode = s->spi0[R0_DIN_MODE];
    s->tune_din_num = s->spi0[R0_DIN_NUM];
    s->tune_logged = 0;
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
        mspi_axi_wake(s);
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
    mspi_axi_wake(s);
}

static void mspi_sys_clk_update(void *opaque, ClockEvent event)
{
    mspi_axi_wake(opaque);
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
    mspi_instances = g_slist_prepend(mspi_instances, s);
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
    notifier_list_init(&s->dma_waiters);
    s->sys_clk = qdev_init_clock_in(dev, "sys", mspi_sys_clk_update, s,
                                    ClockUpdate);
    s->core_clk = qdev_init_clock_in(dev, "core", mspi_core_clk_update, s,
                                     ClockUpdate);
    timer_init_ns(&s->spi1_timer, QEMU_CLOCK_VIRTUAL, spi1_end, s);
}

static const VMStateDescription vmstate_esp32p4_mspi = {
    .name = TYPE_ESP32P4_MSPI,
    .version_id = 4,
    .minimum_version_id = 4,
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
        VMSTATE_INT64(spi1_end_at, ESP32P4MSPIState),
        VMSTATE_BOOL(spi1_cs_held, ESP32P4MSPIState),
        VMSTATE_BOOL(spi1_waiti, ESP32P4MSPIState),
        VMSTATE_UINT32(xts_state, ESP32P4MSPIState),
        VMSTATE_UINT8_ARRAY(xts_cipher, ESP32P4MSPIState, 64),
        VMSTATE_UINT8_ARRAY(psram_mr_regs, ESP32P4MSPIState,
                            ESP32P4_MSPI_PSRAM_MRS),
        VMSTATE_BOOL(psram_halfsleep, ESP32P4MSPIState),
        VMSTATE_BOOL(apb_reset, ESP32P4MSPIState),
        VMSTATE_BOOL(axi_reset, ESP32P4MSPIState),
        VMSTATE_UINT32(crypto_ctrl, ESP32P4MSPIState),
        VMSTATE_BOOL(download_boot, ESP32P4MSPIState),
        VMSTATE_UINT32(axi_waiting, ESP32P4MSPIState),
        VMSTATE_UINT32_ARRAY(tune_cali, ESP32P4MSPIState, 2),
        VMSTATE_UINT32(tune_din_mode, ESP32P4MSPIState),
        VMSTATE_UINT32(tune_din_num, ESP32P4MSPIState),
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
    DEFINE_PROP_LINK("pins", ESP32P4MSPIState, pins, TYPE_ESP32P4_MSPI_IOMUX,
                     ESP32P4MspiIomuxState *),
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
