/*
 * ESP32-P4 boot ROM hand-off: the state -kernel presents
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: the v3.x boot ROM (ESP-ROM esp32p4-eco5-20250430, Espressif's
 * esp32p4_rev300_rom.elf, whose symbols name its routines and data), its
 * _init, main() and ets_run_flash_bootloader(), and the register accesses
 * it makes in this machine as it boots an image from flash, read back at
 * the image's entry point; ESP-IDF's rev3 register headers for the
 * registers' names; ESP-IDF's esptool image format for what the ROM loads.
 *
 * With -kernel the ELF stands for the image the boot ROM boots from flash,
 * the second-stage bootloader at flash offset 0x2000. Whenever HP core 0
 * starts at the ROM with the ROM about to boot that image, the machine
 * presents the state the ROM leaves as it jumps to the image's entry point,
 * esp32p4_present_rom_handoff(), and core 0 starts there rather than in the
 * ROM. The ROM would boot the image when (main() and _init):
 *
 *   - the boot mode is SPI boot: GPIO_STRAP's GPIO35 bit set (the strapping
 *     latched at reset, LP_SYSTEM_FORCE_DOWNLOAD_BOOT able to override it);
 *     the ROM otherwise enters a download mode: joint download, SPI
 *     download, or for 0x2 and 0x0, which the TRM calls invalid, UART0
 *     and USB download alone (its UART0_BOOT and USB_BOOT), stopping for
 *     0x3;
 *   - core 0's reset cause is one main() accepts (mask 0x0DDDBAAA: the
 *     causes its table names but cause 6); for any other the ROM prints
 *     "invalid reset" and resets the system;
 *   - it is not a wakeup from a light sleep that powered the HP CPU down
 *     (cause 5 or 6 with LP_SYSTEM_LP_STORE8 bit 0 clear and an address in
 *     the rest of the word), after which _init jumps to that address;
 *   - Secure Boot is off (EFUSE_SECURE_BOOT_EN): with it on the ROM checks
 *     the image's signature, which an ELF does not carry.
 *
 * Otherwise core 0 runs the ROM itself, as on the chip.
 *
 * The state presented, each item as the ROM leaves it in this machine (its
 * accesses traced from reset to the jump, and the registers read back at
 * the entry point of an image it booted):
 *
 *   - Core 0: mstatus 0x9 (MIE set: the ROM's _init enables interrupts,
 *     none of which its CLIC configuration lets through), mcounteren 0x7
 *     (its value from reset, which the ROM does not write), mie 0, mtvec
 *     0x4FC00B03 (the ROM's vector table in CLIC mode), mhcr 0x1030 (the
 *     return stack, branch prediction and branch target buffer on), PMA
 *     entry 15 cfg 0xC000001D addr 0x11FFFFFF (CPU_ATTR_Cacheable: L2MEM,
 *     the ROM and the external memory windows cacheable), PMP clear; ra
 *     0x4FC05542 (in ets_run_flash_bootloader()), sp 0x4FFBCDF0 (the ROM's
 *     stack, which ESP-IDF's bootloader runs on), gp and tp 0, a5 the
 *     entry point, which pc holds. The other registers hold values of the
 *     ROM's that depend on the image, and are 0. Returning from the entry
 *     point, which on the chip lands back in the ROM, is not supported:
 *     the ROM's stack frames below sp are not presented.
 *   - The clocks: as from reset, CPU_CLK on XTAL_CLK at 40 MHz (the ROM's
 *     delays count 40 cycles a microsecond) and the PLLs as from reset;
 *     the ROM changes only the flash controller's core clock divider
 *     (PERI_CLK_CTRL00's FLASH_CORE_CLK_DIV_NUM, from 3 to 0: XTAL_CLK
 *     undivided) and the SHA accelerator's clock.
 *   - Core 1: held in reset with its clock off, as the ROM on core 0
 *     leaves it; released, it runs the ROM, which waits for
 *     LP_SYSTEM_BOOT_ADDR_HP_CORE1.
 *   - The CLIC: CLICCFG 6 (3 level bits); interrupt 21 (the UART0
 *     receiver's in download mode) a machine-mode one, CLICINTCTL 0x20,
 *     disabled.
 *   - The interrupt matrix: UART0's source mapped to CLIC interrupt 21.
 *   - UART0: 115200 baud 8N1 from XTAL_CLK (CLKDIV 0x0030015B, CONF0
 *     0x0004001C, CONF1 0x6001), RXFIFO_FULL's interrupt enabled, on its
 *     IO MUX pads, U0TXD GPIO37 and U0RXD GPIO38 (IO_MUX_GPIO37 0x900,
 *     GPIO38 0xB00), its core reset by the ROM's Uart_Init(). The ROM's
 *     messages are not sent: UART0's TX FIFO, which on the chip still
 *     holds the tail of the last, and USB Serial/JTAG's IN buffer, which
 *     holds their first 64 bytes, are empty, and the raw interrupt bits and
 *     FIFO pointers sending them would leave (UART0 INT_RAW's TX_DONE and
 *     TX_BRK_IDLE_DONE, MEM_TX_STATUS) are as from reset.
 *   - The caches: the L1 caches reset and invalidated
 *     (HP_SYSTEM_CACHE_RESET_CONFIG pulses, ROM_L1_Cache_Init()), the L2
 *     invalidated, L1D written back; L1_CACHE_ACS_FAIL_CTRL 0x13,
 *     L2_CACHE_ACS_FAIL_CTRL 1, SYNC_MAP 0x10; the L2 cache its reset 128
 *     KiB of L2MEM. No line is dirty, so their contents, the ROM's on the
 *     chip, are not presented.
 *   - The flash MSPI controller: reset (HP_RST_EN0), then configured for
 *     the image header's DIO reads at 80 MHz, both ports in DIO mode and
 *     SPI0 reading with DIO's command 0xBB, 28 address bits (the address
 *     and the mode bits' upper half) and 2 dummy cycles (SPI0 CTRL
 *     0x00AC200C, CTRL2 0x01002C41, CLOCK 0x80000000, USER 0x200000C0,
 *     USER1 0x6C000041, USER2 0x700000BB, MOSI_DLEN and MISO_DLEN 0xFF
 *     (SPI_init()'s 256 bits), CACHE_FCTRL 0x40000005 with the AXI port
 *     open; SPI1 CTRL 0x00AC200C, CTRL1 0x001FF7FC, CLOCK 0x80000000),
 *     SPI1's buffer W0 the flash's JEDEC ID from the ROM's RDID, and both
 *     ports' SLV_ST_END and MST_ST_END raised by the ROM's transactions
 *     (INT_RAW 0x18); the flash MMU all invalid (Cache_FLASH_MMU_Init(),
 *     ITEM_INDEX 0x3FF), so nothing maps the flash window. The ROM's
 *     commands to the flash itself (release from power-down, status
 *     reads, write disable) leave it as it is from power-on, and are not
 *     repeated.
 *   - HP_SYS_CLKRST: PERI_CLK_CTRL00 0xC00C; the SHA accelerator's clock
 *     and reset as ets_sha_enable() leaves them, and SHA_MODE SHA-256. The
 *     SHA's digest of the image and the last block it hashed (SHA_H_n,
 *     SHA_M_n) are not presented.
 *   - The debug assistant: its clock on, both cores' PC and SP recording
 *     restarted (CORE_n_RCD_EN 3) once the ROM has saved the PCs recorded
 *     up to the reset.
 *   - LP_SYSTEM_LP_RST_CTRL 0 (main() clears the reset bypasses);
 *     PMU_IMM_PAD_HOLD_ALL's unhold (pmu_enable_unhold_pads()) releasing
 *     the pads a deep sleep held.
 *   - The super watchdog: running since the reset, unfed, its
 *     SWD_RESET_FLAG cleared (main()'s clear_super_wdt_reset_flag(), on
 *     core 0 at every boot), and its reset not enabled by LP_SYSTEM's FIB
 *     control until software enables it; the ROM neither feeds it nor sets
 *     SWD_AUTO_FEED_EN nor writes the FIB control.
 *   - The watchdogs: MWDT0's and the RTC watchdog's flash boot protection
 *     running, as they do from reset in SPI boot mode (the GPIO block
 *     gives them the boot mode); the ROM does not stop them. They count
 *     from the reset: the ROM's own time, about 23 ms here, is not taken
 *     from them, nor from any other timer or counter (mcycle, minstret).
 *   - The ROM's own RAM in L2MEM (0x4FFBEFC0 to 0x4FFBFFFF), on which its
 *     routines that software calls depend: its .data and .bss as _init
 *     sets them up from the ROM's tables, then what main() sets: the
 *     printf channels (_putc1 ets_write_char_uart, _putc2
 *     ets_write_char_usb, as EFUSE_UART_PRINT_CONTROL, GPIO36,
 *     LP_SYSTEM_LP_STORE4 bit 0 and the USB Serial/JTAG eFuses select),
 *     UartDev, the flash's JEDEC ID, g_offset_of_active_bootloader 0x2000,
 *     usb_uart_connected 0, km_log 1, and g_saved_pc, the PC each core's
 *     record in the debug assistant kept through the reset. This needs the
 *     ROM image the machine knows (its build string is checked); with
 *     another, the RAM is left and a message logged.
 *   - The image: every segment of the ELF loaded again where the ROM
 *     copies it, as the ROM reloads the image from flash on every boot.
 *
 * The ROM's register writes are made here as it makes them, through the
 * bus, so that each block takes them as it takes the ROM's (UART0's
 * REG_UPDATE among them, which carries the configuration into its core a
 * few cycles after core 0 starts). Blocks the machine does not model take
 * the writes as unimplemented devices do. The writes are the ROM's, made
 * in its place, not the guest's: HP_SYS_CLKRST takes them without judging
 * them against the TRM's procedures for changing clocks, which the ROM
 * does not keep to (it changes the flash core clock's divider with the
 * clock's gate on), so that a guest's error log starts with the guest's.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/cpu.h"
#include "hw/riscv/esp32p4.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "target/riscv/cpu.h"

/* Registers the ROM's boot leaves configured, by ESP-IDF's names. */
#define CLIC_MCLICCFG               0x20800000
#define CLIC_INT_IE(n)              (0x20801001 + 4 * (n))
#define CLIC_INT_ATTR(n)            (0x20801002 + 4 * (n))
#define CLIC_INT_CTL(n)             (0x20801003 + 4 * (n))
#define ROM_UART_CLIC_INT           21

#define CACHE_L1_ICACHE_CTRL        0x3ff10000
#define CACHE_L1_DCACHE_CTRL        0x3ff10004
#define CACHE_L1_BYPASS_CACHE_CONF  0x3ff10008
#define CACHE_SYNC_CTRL             0x3ff10098
#define CACHE_SYNC_MAP              0x3ff1009c
#define CACHE_SYNC_ADDR             0x3ff100a0
#define CACHE_SYNC_SIZE             0x3ff100a4
#define CACHE_L1_CACHE_ACS_FAIL_CTRL 0x3ff10168
#define CACHE_L2_CACHE_CTRL         0x3ff10270
#define L2_CACHE_SHUT_DMA           BIT(4)
#define CACHE_L2_CACHE_FREEZE_CTRL  0x3ff1028c
#define CACHE_L2_CACHE_ACS_FAIL_CTRL 0x3ff102e8
/* SYNC_CTRL's operations, SYNC_MAP's caches. */
#define SYNC_INVALIDATE_ENA         0x11
#define SYNC_WRITEBACK_ENA          0x14
#define SYNC_IDLE                   0x10
#define SYNC_MAP_L1                 0x13
#define SYNC_MAP_L1_DCACHE          0x10
#define SYNC_MAP_L2                 0x20
#define L2_FREEZE_EN                BIT(20)
#define L2_FREEZE_DONE              BIT(22)

#define ASSIST_DEBUG_CORE_0_RCD_EN  0x3ff06044
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC 0x3ff06048
#define ASSIST_DEBUG_CORE_1_RCD_EN  0x3ff060c4
#define ASSIST_DEBUG_CORE_1_RCD_PDEBUGPC 0x3ff060c8
#define ASSIST_DEBUG_CLOCK_GATE     0x3ff06108
#define RCD_PDEBUG_EN_AND_RECORD    3

#define SPI0_BASE                   ESP32P4_FLASH_MSPI_BASE
#define SPI1_BASE                   (ESP32P4_FLASH_MSPI_BASE + 0x1000)
#define SPI_MEM_CTRL                0x008
#define SPI_MEM_CTRL1               0x00c
#define SPI_MEM_CTRL2               0x010
#define SPI_MEM_CLOCK               0x014
#define SPI_MEM_USER                0x018
#define SPI_MEM_USER1               0x01c
#define SPI_MEM_USER2               0x020
#define SPI_MEM_RD_STATUS           0x02c
#define SPI_MEM_MOSI_DLEN           0x024
#define SPI_MEM_MISO_DLEN           0x028
#define SPI_MEM_MISC                0x034
#define SPI_MEM_CACHE_FCTRL         0x03c
#define SPI_MEM_W0                  0x058
#define SPI1_MEM_SUS_STATUS         0x0a4
#define SPI_MEM_SMEM_AC             0x1a0
#define SPI_MEM_MMU_ITEM_CONTENT    0x37c
#define SPI_MEM_MMU_ITEM_INDEX      0x380
#define FLASH_MMU_ENTRIES   1024
#define CACHE_FCTRL_CLOSE_AXI_INF   BIT(31)

#define UART0_BASE                  0x500ca000
#define UART_INT_ENA                0x00c
#define UART_INT_CLR                0x010
#define UART_CLKDIV                 0x014
#define UART_CONF0                  0x020
#define UART_CONF1                  0x024
#define UART_CLK_CONF               0x088
#define UART_REG_UPDATE             0x098
#define UART_RXFIFO_FULL_INT        BIT(0)
#define UART_CLK_CONF_RST_CORES     (BIT(26) | BIT(27))

#define INTMTX_CORE0_UART0_MAP      0x500d607c
#define GPIO_STRAP                  0x500e0038
#define IO_MUX_GPIO(n)              (0x500e1004 + 4 * (n))
#define IOMUX_MSPI_PIN_FLASH(n)     (0x500e1204 + 4 * (n))
#define MSPI_PIN_WPU                BIT(2)
#define MSPI_PIN_WPD                BIT(3)
#define MSPI_PIN_DRV                (3u << 4)
#define MSPI_PIN_DRV_2              (2u << 4)

#define HP_SYS_CACHE_RESET_CONFIG   0x500e5024
#define HP_SYS_GPIO_O_HOLD_CTRL0    0x500e5074
#define HP_SYS_GPIO_DED_HOLD_CTRL   0x500e50f0
#define L1_D_CACHE_RESET            BIT(1)
#define L1_I1_CACHE_RESET           BIT(4)
#define L1_I0_CACHE_RESET           BIT(5)

#define HP_SYS_CLKRST_SOC_CLK_CTRL0 0x500e6014
#define HP_SYS_CLKRST_SOC_CLK_CTRL1 0x500e6018
#define HP_SYS_CLKRST_PERI_CLK_CTRL00 0x500e6030
#define HP_SYS_CLKRST_PERI_CLK_CTRL110 0x500e6068
#define HP_SYS_CLKRST_PERI_CLK_CTRL25 0x500e60a8
#define HP_SYS_CLKRST_HP_RST_EN0    0x500e60c0
#define HP_SYS_CLKRST_HP_RST_EN1    0x500e60c4
#define HP_SYS_CLKRST_HP_RST_EN2    0x500e60c8
/*
 * PERI_CLK_CTRL00's FLASH_CORE_CLK_DIV_NUM, 3 from reset, which the ROM
 * clears: the flash MSPI core clock XTAL_CLK undivided.
 */
#define FLASH_CORE_CLK_DIV_RESET    (3u << 4)
#define RST_EN_SPI01                BIT(23)
#define RST_EN_MSPI_AXI             BIT(24)
#define RST_EN_UART0_APB            BIT(13)
#define SHA_SYS_CLK_EN              BIT(26)
#define CRYPTO_SHA_CLK_EN           BIT(20)
#define RST_EN_CRYPTO_ALL           (BIT(14) | BIT(17) | BIT(18) | BIT(19) | \
                                     BIT(20))

#define SHA_MODE                    0x50091000
#define SHA_MODE_SHA256             2

#define LP_SYS_LP_RST_CTRL          0x50110010
#define LP_SYS_LP_STORE4            0x5011003c
#define LP_SYS_LP_STORE8            0x5011004c
#define ANA_RST_BYPASS              BIT(0)
#define SYS_RST_BYPASS              BIT(1)
#define LP_CLKRST_RESET_CAUSE       0x50111010
#define PMU_IMM_PAD_HOLD_ALL        0x501150e4
#define PMU_TIE_LOW_HP_PAD_HOLD_ALL BIT(31)

/* The ROM's entry and data. */
#define ROM_VECTOR_TABLE            0x4fc00b00
#define ROM_FLASH_BOOT_RETURN       0x4fc05542
#define ROM_FLASH_BOOT_SP           0x4ffbcdf0
#define ROM_BUILD_STRING            0x4fc1d4dc
#define ROM_DATA_TABLE              0x4fc1c860
#define ROM_BSS_TABLE               0x4fc1c9ec
#define ROM_BSS_TABLE_END           0x4fc1caec
#define ROM_BOOTLOADER_OFFSET       0x2000
/* The ROM's variables main() sets (its symbols). */
#define ROM_G_OFFSET_OF_ACTIVE_BOOTLOADER 0x4ffbf008
#define ROM_USB_UART_CONNECTED      0x4ffbfea4
#define ROM_UART_DEV                0x4ffbfeb0
#define ROM_SPIFLASH_CHIP_DEVICE_ID 0x4ffbfee8
#define ROM_PUTC1                   0x4ffbff7c
#define ROM_PUTC2                   0x4ffbff80
#define ROM_KM_LOG                  0x4ffbffa0
#define ROM_G_SAVED_PC              0x4ffbfff8
#define ROM_ETS_WRITE_CHAR_UART     0x4fc02f02
#define ROM_ETS_WRITE_CHAR_USB      0x4fc02f24
/* The ROM's data and stack, which it refuses to load a segment over. */
#define ROM_DATA_END                0x4ffbffa4

/* main()'s accepted reset causes, by bit (ets_main.c's mask). */
#define ROM_VALID_RESET_CAUSES      0x0dddbaaau

/* The eFuse words of block 0 main() reads (RD_REPEAT_DATAn is word n + 1). */
#define EFUSE_DATA0_DIS_USB_SERIAL_JTAG BIT(11)
#define EFUSE_DATA2_SECURE_BOOT_EN  BIT(20)
#define EFUSE_DATA3_DIS_USJ_ROM_PRINT BIT(2)
#define EFUSE_DATA3_UART_PRINT_CONTROL(w) extract32((w), 6, 2)

static const char rom_build[] = "esp32p4-eco5-20250430";

static uint32_t rom_rd(hwaddr addr)
{
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static void rom_wr(hwaddr addr, uint32_t v)
{
    address_space_stl_le(&address_space_memory, addr, v,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static void rom_set(hwaddr addr, uint32_t bits)
{
    rom_wr(addr, rom_rd(addr) | bits);
}

static void rom_clr(hwaddr addr, uint32_t bits)
{
    rom_wr(addr, rom_rd(addr) & ~bits);
}

/* A reset bit set and cleared again, as the ROM resets a block. */
static void rom_pulse(hwaddr addr, uint32_t bits)
{
    rom_set(addr, bits);
    rom_clr(addr, bits);
}

/* A byte of core 0's CLIC (the accesses are core 0's, see below). */
static void rom_clic_wb(hwaddr addr, uint8_t v)
{
    address_space_stb(&address_space_memory, addr, v,
                      MEMTXATTRS_UNSPECIFIED, NULL);
}

static uint32_t rom_efuse(ESP32P4MachineState *s, unsigned data)
{
    return esp32p4_efuse_word(&s->efuse, 0, data + 1);
}

/* A cache operation over the whole of the caches in @map, as Cache_*_All. */
static void rom_cache_sync(uint32_t map, uint32_t op)
{
    rom_wr(CACHE_SYNC_ADDR, 0);
    rom_wr(CACHE_SYNC_SIZE, 0);
    rom_wr(CACHE_SYNC_MAP, map);
    rom_wr(CACHE_SYNC_CTRL, SYNC_IDLE);
    rom_wr(CACHE_SYNC_CTRL, op);
}

/*
 * The JEDEC ID the ROM's RDID reads from the flash: the W25Q part of the
 * board's flash size, or nothing, zeros, from a board without flash.
 */
static uint32_t rom_flash_jedec_id(ESP32P4MachineState *s)
{
    switch (s->flash_size) {
    case 0:
        return 0;
    case 1 * MiB:
        return 0xef4014;
    case 2 * MiB:
        return 0xef4015;
    case 4 * MiB:
        return 0xef4016;
    case 8 * MiB:
        return 0xef4017;
    case 16 * MiB:
        return 0xef4018;
    case 32 * MiB:
        return 0xef4019;
    case 64 * MiB:
        return 0xef4020;
    default:
        return 0xef4021;
    }
}

/*
 * main() on core 0, up to the flash boot: the reset bypasses cleared, the
 * L1 caches reset and invalidated (ROM_L1_Cache_Init()), the super
 * watchdog's reset flag cleared, DMA shut out of the L2
 * (Cache_Mask_All()), the pads a deep sleep held released, UART0 attached
 * and set up (uartAttach(), Uart_Init()), its receiver's interrupt routed,
 * and the debug assistant's PC recording restarted.
 */
static void rom_main(ESP32P4MachineState *s, uint32_t saved_pc[2])
{
    rom_clr(LP_SYS_LP_RST_CTRL, SYS_RST_BYPASS);
    rom_clr(LP_SYS_LP_RST_CTRL, ANA_RST_BYPASS);

    rom_pulse(HP_SYS_CACHE_RESET_CONFIG, L1_I0_CACHE_RESET);
    rom_pulse(HP_SYS_CACHE_RESET_CONFIG, L1_I1_CACHE_RESET);
    rom_pulse(HP_SYS_CACHE_RESET_CONFIG, L1_D_CACHE_RESET);
    rom_cache_sync(SYNC_MAP_L1, SYNC_INVALIDATE_ENA);
    rom_clr(CACHE_L1_ICACHE_CTRL, 3);
    rom_clr(CACHE_L1_DCACHE_CTRL, 3);
    rom_clr(CACHE_L1_BYPASS_CACHE_CONF, 0x13);
    rom_set(CACHE_L1_CACHE_ACS_FAIL_CTRL, 0x13);

    /* clear_super_wdt_reset_flag(): SWD_RST_FLAG_CLR, without the key. */
    esp32p4_lp_wdt_swd_rst_flag_clr(&s->lp_wdt);
    rom_set(CACHE_L2_CACHE_CTRL, L2_CACHE_SHUT_DMA);
    rom_set(PMU_IMM_PAD_HOLD_ALL, PMU_TIE_LOW_HP_PAD_HOLD_ALL);

    /* uartAttach(): the receiver's CLIC interrupt off. */
    rom_clic_wb(CLIC_INT_IE(ROM_UART_CLIC_INT), 0);

    /*
     * Uart_Init(0): the clock, UART0 reset (its registers, then its core
     * with them; the ROM's write of CLK_CONF while the registers are held
     * in reset, which takes no effect, is left out), the pads, 115200 8N1.
     */
    rom_wr(HP_SYS_CLKRST_SOC_CLK_CTRL1, rom_rd(HP_SYS_CLKRST_SOC_CLK_CTRL1));
    rom_pulse(HP_SYS_CLKRST_HP_RST_EN1, RST_EN_UART0_APB);
    rom_set(UART0_BASE + UART_CLK_CONF, UART_CLK_CONF_RST_CORES);
    rom_pulse(HP_SYS_CLKRST_HP_RST_EN1, RST_EN_UART0_APB);
    rom_wr(IO_MUX_GPIO(37), 0x900);
    rom_clr(HP_SYS_GPIO_O_HOLD_CTRL0, BIT(37 - 16));
    rom_wr(IO_MUX_GPIO(38), 0xb00);
    rom_clr(HP_SYS_GPIO_O_HOLD_CTRL0, BIT(38 - 16));
    rom_wr(HP_SYS_CLKRST_PERI_CLK_CTRL110,
           rom_rd(HP_SYS_CLKRST_PERI_CLK_CTRL110));
    rom_wr(UART0_BASE + UART_CLKDIV, 0x0030015b);
    rom_wr(UART0_BASE + UART_CONF0, 0x0004001c);
    rom_wr(UART0_BASE + UART_CONF1, 0x00006001);
    rom_wr(UART0_BASE + UART_REG_UPDATE, 1);
    rom_wr(UART0_BASE + UART_INT_CLR, 0xffffffff);
    rom_wr(UART0_BASE + UART_INT_ENA, UART_RXFIFO_FULL_INT);

    /* ets_isr_attach() and the interrupt's type and priority. */
    rom_wr(INTMTX_CORE0_UART0_MAP, ROM_UART_CLIC_INT);
    rom_clic_wb(CLIC_INT_ATTR(ROM_UART_CLIC_INT), 0xc0);
    rom_clic_wb(CLIC_INT_CTL(ROM_UART_CLIC_INT), 0x20);

    /*
     * ets_clk_assist_debug_clock_enable(); the PC each core's record kept
     * through the reset saved (g_saved_pc, set up with the ROM's RAM), and
     * the records restarted.
     */
    rom_wr(ASSIST_DEBUG_CLOCK_GATE, 1);
    saved_pc[0] = rom_rd(ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC);
    saved_pc[1] = rom_rd(ASSIST_DEBUG_CORE_1_RCD_PDEBUGPC);
    rom_wr(ASSIST_DEBUG_CORE_0_RCD_EN, RCD_PDEBUG_EN_AND_RECORD);
    rom_wr(ASSIST_DEBUG_CORE_1_RCD_EN, RCD_PDEBUG_EN_AND_RECORD);
}

/*
 * spi_flash_boot_attach() and the flash reads of ets_run_flash_bootloader()
 * up to the image's last segment: the flash pins, the controller reset and
 * set up for the header's mode and clock, the L2 cache invalidated
 * (ROM_Boot_Cache_Init()), and the controller's last command, RDID.
 */
static void rom_flash_boot(ESP32P4MachineState *s)
{
    /* SelectSpiFunction(): the flash pads pulled up, released, drive 2. */
    static const uint8_t pads[] = { 4, 1, 5, 3, 2, 0 };
    int i;

    for (i = 0; i < ARRAY_SIZE(pads); i++) {
        rom_clr(IOMUX_MSPI_PIN_FLASH(pads[i]), MSPI_PIN_WPD);
        rom_set(IOMUX_MSPI_PIN_FLASH(pads[i]), MSPI_PIN_WPU);
        rom_clr(HP_SYS_GPIO_DED_HOLD_CTRL, BIT(pads[i]));
    }
    for (i = 0; i < ARRAY_SIZE(pads); i++) {
        hwaddr pin = IOMUX_MSPI_PIN_FLASH(pads[i]);

        rom_wr(pin, (rom_rd(pin) & ~MSPI_PIN_DRV) | MSPI_PIN_DRV_2);
    }
    rom_wr(SPI0_BASE + SPI_MEM_CACHE_FCTRL, 0xc0000000);
    rom_wr(HP_SYS_CLKRST_SOC_CLK_CTRL0, rom_rd(HP_SYS_CLKRST_SOC_CLK_CTRL0));
    rom_clr(HP_SYS_CLKRST_PERI_CLK_CTRL00, FLASH_CORE_CLK_DIV_RESET);
    rom_set(HP_SYS_CLKRST_HP_RST_EN0, RST_EN_SPI01 | RST_EN_MSPI_AXI);
    rom_clr(HP_SYS_CLKRST_HP_RST_EN0, RST_EN_MSPI_AXI);
    rom_clr(HP_SYS_CLKRST_HP_RST_EN0, RST_EN_SPI01);

    rom_wr(SPI1_BASE + SPI1_MEM_SUS_STATUS, 0x7a7a0000);
    rom_wr(SPI0_BASE + SPI_MEM_MISC, 0x72);
    rom_wr(SPI0_BASE + SPI_MEM_CTRL2, 0x01002c41);
    rom_wr(SPI0_BASE + SPI_MEM_SMEM_AC, 0x8000b084);
    rom_wr(SPI1_BASE + SPI_MEM_CTRL2, 0x00000001);
    rom_wr(SPI0_BASE + SPI_MEM_RD_STATUS, 0);

    /* ROM_Boot_Cache_Init(): the L2 invalidated. */
    rom_cache_sync(SYNC_MAP_L2, SYNC_INVALIDATE_ENA);
    rom_wr(CACHE_L2_CACHE_ACS_FAIL_CTRL, 1);

    /* The controller as the header's DIO, 80 MHz reads leave it. */
    rom_wr(SPI1_BASE + SPI_MEM_CTRL1, 0x001ff7fc);
    rom_wr(SPI1_BASE + SPI_MEM_MISO_DLEN, 7);
    rom_wr(SPI1_BASE + SPI_MEM_USER, 0x80000000);
    rom_wr(SPI1_BASE + SPI_MEM_USER1, 0x5c000007);
    rom_wr(SPI1_BASE + SPI_MEM_USER2, 0x70000000);
    rom_wr(SPI1_BASE + SPI_MEM_W0, 0);
    rom_wr(SPI1_BASE + SPI_MEM_RD_STATUS, 0);
    rom_wr(SPI0_BASE + SPI_MEM_CLOCK, 0x80000000);
    rom_wr(SPI1_BASE + SPI_MEM_CLOCK, 0x80000000);
    rom_wr(SPI0_BASE + SPI_MEM_MISO_DLEN, 0xff);
    rom_wr(SPI0_BASE + SPI_MEM_MOSI_DLEN, 0xff);
    rom_wr(SPI0_BASE + SPI_MEM_USER, 0x200000c0);
    rom_wr(SPI0_BASE + SPI_MEM_USER1, 0x6c000041);
    rom_wr(SPI0_BASE + SPI_MEM_USER2, 0x700000bb);
    rom_wr(SPI0_BASE + SPI_MEM_CACHE_FCTRL, 0x40000005);
    rom_wr(SPI0_BASE + SPI_MEM_CTRL, 0x00ac200c);
    rom_wr(SPI1_BASE + SPI_MEM_CTRL, 0x00ac200c);

    /*
     * The flash's JEDEC ID, as RDID leaves it in W0, least significant
     * byte first.
     */
    rom_wr(SPI1_BASE + SPI_MEM_W0, bswap32(rom_flash_jedec_id(s)) >> 8);
    esp32p4_mspi_raise_state_ends(&s->flash_mspi);
}

/*
 * ets_sha_enable(), with which the ROM hashed the image: the SHA
 * accelerator's clocks on and the crypto blocks' resets released; and
 * ets_sha_init()'s SHA-256, in SHA_MODE.
 */
static void rom_sha_enable(void)
{
    rom_set(HP_SYS_CLKRST_SOC_CLK_CTRL1, SHA_SYS_CLK_EN);
    rom_set(HP_SYS_CLKRST_PERI_CLK_CTRL25, CRYPTO_SHA_CLK_EN);
    rom_clr(HP_SYS_CLKRST_HP_RST_EN2, RST_EN_CRYPTO_ALL);
    rom_wr(SHA_MODE, SHA_MODE_SHA256);
}

/*
 * The end of ets_run_flash_bootloader(): the L2 frozen and thawed around
 * its invalidation (Cache_Suspend_L2_Cache(), Cache_Invalidate_All(),
 * Cache_Enable_L2_Cache()), the flash MMU emptied (Cache_FLASH_MMU_Init()),
 * the L1 data cache written back (ROM_Boot_Cache_WriteBack()).
 */
static void rom_boot_cache_finish(void)
{
    int i;

    rom_wr(CACHE_L2_CACHE_FREEZE_CTRL, 0);
    rom_wr(CACHE_L2_CACHE_FREEZE_CTRL, L2_FREEZE_EN);
    rom_wr(CACHE_L2_CACHE_FREEZE_CTRL, L2_FREEZE_DONE);
    rom_set(SPI0_BASE + SPI_MEM_CACHE_FCTRL, CACHE_FCTRL_CLOSE_AXI_INF);
    rom_cache_sync(SYNC_MAP_L2, SYNC_INVALIDATE_ENA);
    rom_clr(SPI0_BASE + SPI_MEM_CACHE_FCTRL, CACHE_FCTRL_CLOSE_AXI_INF);
    for (i = 0; i < FLASH_MMU_ENTRIES; i++) {
        rom_wr(SPI0_BASE + SPI_MEM_MMU_ITEM_INDEX, i);
        rom_wr(SPI0_BASE + SPI_MEM_MMU_ITEM_CONTENT, 0);
    }
    rom_cache_sync(SYNC_MAP_L1_DCACHE, SYNC_WRITEBACK_ENA);
}

/*
 * The ROM's RAM: _init's set-up of its .data (each table entry a
 * destination start and end and a source in the ROM) and .bss (start and
 * end), then what main() sets.
 */
static void rom_ram(ESP32P4MachineState *s, const uint32_t saved_pc[2])
{
    char build[sizeof(rom_build)];
    uint32_t strap = rom_rd(GPIO_STRAP);
    uint32_t data3 = rom_efuse(s, 3);
    bool uart_print;
    hwaddr t;

    address_space_read(&address_space_memory, ROM_BUILD_STRING,
                       MEMTXATTRS_UNSPECIFIED, build, sizeof(build));
    if (memcmp(build, rom_build, sizeof(rom_build))) {
        qemu_log_mask(LOG_UNIMP, "esp32p4: the HP ROM is not ESP-ROM %s; "
                      "-kernel does not set up its RAM\n", rom_build);
        return;
    }
    for (t = ROM_DATA_TABLE; t < ROM_BSS_TABLE; t += 12) {
        uint32_t dst = rom_rd(t), end = rom_rd(t + 4), src = rom_rd(t + 8);

        for (; dst < end; dst += 4, src += 4) {
            rom_wr(dst, rom_rd(src));
        }
    }
    for (t = ROM_BSS_TABLE; t < ROM_BSS_TABLE_END; t += 8) {
        uint32_t dst = rom_rd(t), end = rom_rd(t + 4);

        for (; dst < end; dst += 4) {
            rom_wr(dst, 0);
        }
    }

    /*
     * The printf channels: UART0 as EFUSE_UART_PRINT_CONTROL and GPIO36
     * allow it (TRM table 12.2-3), USB Serial/JTAG unless its eFuses
     * disable it; neither with the ROM's messages suppressed
     * (LP_SYSTEM_LP_STORE4 bit 0, RTC_DISABLE_ROM_LOG).
     */
    if (!(rom_rd(LP_SYS_LP_STORE4) & 1)) {
        switch (EFUSE_DATA3_UART_PRINT_CONTROL(data3)) {
        case 1:
            uart_print = !(strap & BIT(2));
            break;
        case 2:
            uart_print = strap & BIT(2);
            break;
        case 3:
            uart_print = false;
            break;
        default:
            uart_print = true;
            break;
        }
        if (uart_print) {
            rom_wr(ROM_PUTC1, ROM_ETS_WRITE_CHAR_UART);
        }
        if (!(data3 & EFUSE_DATA3_DIS_USJ_ROM_PRINT) &&
            !(rom_efuse(s, 0) & EFUSE_DATA0_DIS_USB_SERIAL_JTAG)) {
            rom_wr(ROM_PUTC2, ROM_ETS_WRITE_CHAR_USB);
        }
    }

    /* uartAttach() and Uart_Init(): 115200 baud, 8 data bits, 1 stop. */
    rom_wr(ROM_UART_DEV, 115200);
    rom_wr(ROM_UART_DEV + 4, 3);
    rom_wr(ROM_UART_DEV + 16, 1);
    rom_wr(ROM_UART_DEV + 40, 1);
    /* No USB host took the ROM's output in its boot, so it gave up. */
    rom_wr(ROM_USB_UART_CONNECTED, 0);
    /* esp_rom_spi_flash_update_id() */
    rom_wr(ROM_SPIFLASH_CHIP_DEVICE_ID, rom_flash_jedec_id(s));
    /* esp_rom_check_recover_key(1) */
    rom_wr(ROM_KM_LOG, 1);
    rom_wr(ROM_G_OFFSET_OF_ACTIVE_BOOTLOADER, ROM_BOOTLOADER_OFFSET);
    rom_wr(ROM_G_SAVED_PC, saved_pc[0]);
    rom_wr(ROM_G_SAVED_PC + 4, saved_pc[1]);
}

/* The image's segments, where the ROM copies them. */
static void rom_load_image(ESP32P4MachineState *s)
{
    guint i;

    for (i = 0; i < s->kernel_segments->len; i++) {
        ESP32P4KernelSegment *seg = &g_array_index(s->kernel_segments,
                                                   ESP32P4KernelSegment, i);
        gsize filesz;
        const void *bytes = g_bytes_get_data(seg->data, &filesz);
        g_autofree uint8_t *zeros = g_malloc0(seg->memsz - filesz);

        address_space_write(&address_space_memory, seg->paddr,
                            MEMTXATTRS_UNSPECIFIED, bytes, filesz);
        address_space_write(&address_space_memory, seg->paddr + filesz,
                            MEMTXATTRS_UNSPECIFIED, zeros,
                            seg->memsz - filesz);
    }
}

/*
 * Core 0 as the ROM jumps to the image: _init's mstatus, mie, mtvec and gp,
 * main()'s CPU_ATTR_Cacheable and mhcr, and the registers of
 * ets_run_flash_bootloader()'s jalr to the entry point.
 */
static void rom_core0(ESP32P4MachineState *s)
{
    CPURISCVState *env = &s->hp_cpu[0].parent_obj.env;
    uint64_t mhcr;
    int i;

    riscv_csr_write_i64(env, CSR_MSTATUS, 0);
    riscv_csr_write_i64(env, CSR_MIE, 0);
    riscv_csr_write_i64(env, CSR_MTVEC, ROM_VECTOR_TABLE | 1);
    riscv_csr_write_i64(env, CSR_MSTATUS, MSTATUS_MIE | 1);
    riscv_csr_write_i64(env, 0xbdf, 0x11ffffff);
    riscv_csr_write_i64(env, 0xbcf, 0xc000001d);
    riscv_csr_read_i64(env, 0x7c1, &mhcr);
    riscv_csr_write_i64(env, 0x7c1, mhcr | 0x1030);

    for (i = 1; i < 32; i++) {
        env->gpr[i] = 0;
    }
    env->gpr[xRA] = ROM_FLASH_BOOT_RETURN;
    env->gpr[xSP] = ROM_FLASH_BOOT_SP;
    env->gpr[15] = s->kernel_entry;
    env->pc = s->kernel_entry;
}

/*
 * [spec:nuos:req:emu.esp32p4.boot]
 * The ROM's accesses are core 0's: the blocks with a view for each core,
 * the CLIC's and the CLINT's, take them as the accessing core's, which
 * they tell from current_cpu.
 */
void esp32p4_present_rom_handoff(ESP32P4MachineState *s)
{
    CPUState *self = current_cpu;
    uint32_t saved_pc[2];

    current_cpu = CPU(&s->hp_cpu[0]);
    esp32p4_clkrst_set_rom_handoff(&s->clkrst, true);
    /* _init: the CLIC's 3 level bits (MNLBITS). */
    rom_set(CLIC_MCLICCFG, 6);
    rom_main(s, saved_pc);
    rom_flash_boot(s);
    rom_sha_enable();
    rom_ram(s, saved_pc);
    rom_load_image(s);
    rom_boot_cache_finish();
    rom_core0(s);
    esp32p4_clkrst_set_rom_handoff(&s->clkrst, false);
    current_cpu = self;
}

/*
 * [spec:nuos:req:emu.esp32p4.boot]
 * Whether the ROM, started on core 0 now, would boot the image in flash
 * (see above).
 */
static bool esp32p4_rom_boots_flash(ESP32P4MachineState *s)
{
    uint32_t cause = extract32(rom_rd(LP_CLKRST_RESET_CAUSE), 7, 6);
    uint32_t store8 = rom_rd(LP_SYS_LP_STORE8);

    if ((cause == 5 || cause == 6) && !(store8 & 1) && (store8 & ~3u)) {
        return false;
    }
    if (!(rom_rd(GPIO_STRAP) & ESP32P4_GPIO_STRAP_SPI_BOOT)) {
        return false;
    }
    if (cause > 27 || !(ROM_VALID_RESET_CAUSES & BIT(cause))) {
        return false;
    }
    if (rom_efuse(s, 2) & EFUSE_DATA2_SECURE_BOOT_EN) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4: Secure Boot is enabled: the "
                      "ROM boots only signed images from flash, so it runs "
                      "in place of -kernel's ELF\n");
        return false;
    }
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.boot]
 * HP core 0 starts at its reset vector: with -kernel, if that is the ROM
 * and the ROM would boot the image, it starts at the image's entry point
 * with the ROM's hand-off presented; otherwise it runs what is there.
 */
void esp32p4_boot_core0(ESP32P4MachineState *s)
{
    if (!s->kernel_entry || !s->core0_rom_vector ||
        !esp32p4_rom_boots_flash(s)) {
        return;
    }
    esp32p4_present_rom_handoff(s);
}

/*
 * The region a segment may not overlap, as ets_run_flash_bootloader()
 * refuses one over its own stack frame (from 1 KiB below its stack pointer)
 * and data, and whether [addr, addr + size) does.
 */
bool esp32p4_rom_refuses_segment(uint32_t addr, uint32_t size)
{
    uint32_t lo = ROM_FLASH_BOOT_SP - 1024;

    return addr + (uint64_t)size > lo && addr < ROM_DATA_END;
}
