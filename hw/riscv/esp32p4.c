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
 *     gated until software releases it through HP_SYS_CLKRST. The LP core
 *     is held in reset until the PMU starts it, at its boot address
 *     (LP_SYSTEM_LP_CORE_BOOT_ADDR, by default the LP ROM's); it executes
 *     from LP SRAM, HP L2MEM (directly, past the caches) and the LP ROM,
 *     takes its interrupts from LP_INTR (0x5012C000), the LP peripherals'
 *     and HP_SYSTEM's HP_INTR, sleeps and wakes through the PMU and wakes
 *     the HP system through it. Each core executes at the clock that
 *     drives it, the HP cores at CPU_CLK and the LP core at
 *     LP_DYN_FAST_CLK, and under icount an instruction takes a cycle of
 *     it.
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
 *     through which software configures the PLLs.
 *   - The system registers (TRM chapter 21), HP_SYSTEM at 0x500E5000 and
 *     LP_SYSTEM at 0x50110000, every register of each (esp32p4_hp_sys.c,
 *     esp32p4_lp_sys.c), driving what the machine models of the blocks
 *     they configure (esp32p4_init_debug()).
 *   - The eFuse controller and its array, on its clock and reset from
 *     LPPERI, its interrupt a source of the interrupt matrix.
 *   - The system timer, the two timer groups (each two timers, an MWDT
 *     and, in group 0, the RTC calibration), the RTC watchdog, the super
 *     watchdog and the RTC timer, on their clocks and resets; the
 *     watchdogs reset the HP cores, the HP core (the HP blocks and both
 *     cores, the LP system running on) or the whole system, each with its
 *     reset cause in LP_CLKRST.
 *   - Low-power management (TRM chapters 15, 24 to 26): the PMU
 *     (0x50115000), its state machine taking the chip between HP_ACTIVE,
 *     HP_SLEEP and LP_SLEEP, powering the domains, the oscillators and the
 *     PLLs, gating the HP system's clocks, stalling the HP cores and
 *     starting and stopping the LP core, its interrupts sources 6 and 7;
 *     LP_ANALOG_PERI (0x50113000), the brown-out detector on the board's
 *     VDD_ANA and VDD_BAT, with the glitch detectors and the power supply
 *     detector, its interrupt source 8; and the LP mailbox (0x50118000),
 *     its interrupts sources 4 and 5.
 *   - The event task matrix (TRM chapter 14) at 0x500D5000, mapping the
 *     events of the GPIO matrix, the timers, the RTC timer, GDMA, the LED
 *     PWM controller and MCPWM to the tasks of the GPIO matrix, the timer
 *     groups, GDMA, the LED PWM controller, MCPWM and the LP core.
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
 *     core's mode and its CLIC's user-mode interrupts decide. HP_SYSTEM's
 *     software interrupts CPU_INTR_FROM_CPU_0 to _3 are sources of the
 *     matrix. A source no modelled block drives stays low.
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
 *   - The SD/MMC host controller (TRM chapter 57) at 0x50083000, behind
 *     SDMMC_SYS_CLK and its reset, its card interface clocked by the
 *     SDIO_LS clocks, its internal DMA through the permission managers as
 *     DMA master SDMMC, its interrupt SDIO_HOST_INTR (source 23), and its
 *     card detect, write protect and SDIO interrupt inputs and card reset
 *     outputs matrix signals 126 to 131 and 146 to 148. A card on slot N
 *     is -drive if=sd,index=N: the board's microSD socket is slot 0, on
 *     the IO MUX pads GPIO39 to GPIO44; slot 1 is where the board's
 *     ESP32-C6 sits, as an SDIO slave, and has no card unless given one:
 *
 *       -drive if=sd,index=0,format=raw,file=sd.img
 *
 *     A card is on the board, not the chip: no reset of the chip or the
 *     controller reaches it, only a power cycle (QEMU's system reset).
 *
 *   - The UARTs (TRM chapter 45): UART0 to UART4 at 0x500CA000 to
 *     0x500CE000, behind UARTn_APB_CLK and their APB resets, their cores
 *     on UARTn_SCLK and their core resets, their FIFO RAM ports on
 *     UARTn_SYS_CLK, their interrupts sources 31 to 35; and the LP UART at
 *     0x50121000, behind LPPERI's LP_UART clock and reset, its core on
 *     LP_UART_SCLK, its interrupt source 16. Their wakeups, which the
 *     PMU's UART_WAKEUP_EN arms, are PMU wakeup sources, the LP UART's
 *     also LPPERI's LP_UART_WAKEUP_FLAG. Their TXD, RTS and DTR and RXD,
 *     CTS and DSR are GPIO matrix signals (the LP UART's the LP matrix's),
 *     reaching pads where software routes them.
 *
 *     The board's serial ports are terminals on UART pads, each sending and
 *     receiving frames on its pads at the format and baud rate its UART is
 *     set to, as a USB-serial bridge matched to the chip would; a byte gets
 *     through only where software has routed the UART to those pads. Port
 *     0 (-serial, the console) is on UART0's IO MUX pads, U0TXD GPIO37 and
 *     U0RXD GPIO38, where the boot ROM prints; port 1 on UART1's IO MUX
 *     pads, GPIO10 (TXD) and GPIO11 (RXD); ports 2 to 4, for UART2 to
 *     UART4, which have no IO MUX pads, on GPIO16/17, GPIO18/19 and
 *     GPIO20/21 (TXD/RXD), which the matrix can route them to; port 5 on
 *     the LP UART's LP IO MUX pads, GPIO14 (TXD) and GPIO15 (RXD). A port
 *     drives its RXD pad, idle high, only while its chardev is connected:
 *
 *       -serial stdio -serial null -serial file:uart2.log
 *
 *     UHCI0 (0x500DF000), behind UHCI_APB_CLK and its reset, its
 *     interrupt source 30, moves the data of the UART CONF0.UART_SEL
 *     names through GDMA-AHB as peripheral UHCI0.
 *
 *   - The USB Serial/JTAG controller (TRM chapter 54) at 0x500D2000,
 *     behind USB_DEVICE_APB_CLK and LP_CLKRST's USB_DEVICE reset, its USB
 *     side on USB_DEVICE_48M_CLK, its interrupt source 22, its PHY's
 *     pull-ups on GPIO24 and GPIO25. Its CDC-ACM serial port reaches a
 *     chardev, a computer at the other end of the board's USB cable,
 *     which enumerates the device as the boot ROM starts and reads and
 *     writes the port while the chardev is open; with none, no cable:
 *
 *       -chardev stdio,id=usj -global esp32p4-usb-serial-jtag.chardev=usj
 *
 *   - The DMA controllers (TRM chapters 6 and 7): GDMA-AHB at 0x50085000
 *     and GDMA-AXI at 0x5008A000, three RX and three TX channels each,
 *     behind AHB_PDMA_SYS_CLK and AXI_PDMA_SYS_CLK and their resets, their
 *     channels' interrupts sources 56 to 67; and VDMA, a DesignWare AXI DMA
 *     controller, at 0x50081000, behind GDMA_SYS_CLK and the GDMA reset,
 *     its interrupt source 24. Their accesses go through the DMA APM as
 *     the DMA masters they are, to memory directly, past the caches. A
 *     peripheral served by GDMA attaches to its controller as
 *     include/hw/dma/esp32p4_gdma.h describes; VDMA's handshaking
 *     interfaces, for MIPI DSI and the ISP, are its named GPIOs.
 *   - GP-SPI2 and GP-SPI3 (TRM chapter 46) at 0x500D0000 and 0x500D1000,
 *     masters or slaves, through GDMA-AXI, their bus signals through the
 *     GPIO matrix and (GP-SPI2's) the IO MUX, their interrupts sources 25
 *     and 26. The board's SPI devices are QEMU SSI devices on their buses,
 *     "spi2" and "spi3", wired to whichever pads carry the controller's
 *     chip select and data lines (include/hw/ssi/esp32p4_spi.h):
 *
 *       -device w25q32,bus=spi2,cs=0
 *
 *   - The BitScrambler (TRM chapter 62) at 0x500A3000, behind
 *     BITSCRAMBLER_SYS_CLK and its reset, its TX and RX cores on
 *     BITSCRAMBLER_TX/RX_SYS_CLK with resets of their own, in the DMA
 *     paths HP_SYSTEM_BITSCRAMBLER_PERI_SEL attaches them to.
 *   - The parallel IO controller (TRM chapter 61) at 0x500CF000, behind
 *     PARLIO_APB_CLK and the PARLIO reset, its units on PARLIO_TX_CLK and
 *     PARLIO_RX_CLK (or their pads, through the GPIO matrix) with their
 *     own resets and its DMA on PARLIO_SYS_CLK through GDMA-AXI, its pad
 *     signals matrix signals 186 to 203 and 242 (TX CS), its interrupts
 *     HP_PARLIO_RX_INTR and _TX_INTR (sources 113 and 114).
 *   - The debug blocks (TRM chapters 3, 22 and 23): the debug assistant at
 *     0x3FF06000, watching the HP cores' SP, their accesses to four
 *     regions and their PCs; each HP core's RISC-V trace encoder, at
 *     0x3FF04000 and 0x3FF05000, writing Efficient Trace packets to
 *     memory; and the AXI performance monitor at 0x500A8000.
 *
 *   - The USB 2.0 OTG controllers (TRM chapters 52 and 53,
 *     hw/usb/esp32p4_usb.c): OTG_HS (0x50000000) on its UTMI+ PHY
 *     (0x5009C000) and OTG_FS (0x50040000) on a full-speed PHY that
 *     USB_WRAP (0x50080000) controls, behind their bus clocks and resets,
 *     their DMA through the DMA access permission manager, their
 *     interrupts sources 93, 94 and 103, OTG_FS's OTG signals through the
 *     GPIO matrix. In host mode QEMU's USB devices plug into a
 *     controller's port, the bus named after it: usb-otg-hs.0 and
 *     usb-otg-fs.0,
 *
 *       -device usb-kbd,bus=usb-otg-hs.0,port=1
 *
 *     and in device mode one of the built-in hosts of hw/usb/usb-devport.h
 *     is at the other end of the cable, the token port or the CDC-ACM
 *     bridge on a chardev:
 *
 *       -chardev socket,id=tok,path=tok.sock,server=on,wait=off
 *       -global esp32p4-usb-otg-fs.token-chardev=tok
 *
 *   - The TWAI controllers TWAI0 to TWAI2 (TRM chapter 56) at 0x500D7000,
 *     0x500D8000 and 0x500D9000, each behind its bus clock, on its function
 *     clock and reset, its interrupt TWAIn_INTR (sources 40 to 42), its
 *     twai_tx, twai_rx, twai_bus_off_on, twai_clkout and twai_standby
 *     signals through the GPIO matrix. Each controller's pads can lead to a
 *     QEMU CAN bus, the machine's canbus0 to canbus2; controllers on one
 *     bus arbitrate and acknowledge each other, and can-host-socketcan on
 *     the bus connects it to the host's (on Linux):
 *
 *       -object can-bus,id=can0 -M esp32p4,canbus0=can0,canbus1=can0
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
 *   - I2S0 to I2S2 (TRM chapter 49) at 0x500C6000, each behind its
 *     I2Sn_APB_CLK and reset, its units on I2Sn_TX_CLK and I2Sn_RX_CLK,
 *     moving data through GDMA-AHB, its interrupt I2Sn_INTR (sources 27 to
 *     29), its signals matrix signals 25 to 45 and its ETM events and
 *     tasks; and the LP I2S with its voice activity detection (chapters
 *     50 and 44) at 0x50125000, behind LPPERI's LP_I2S gate and reset, on
 *     LP_I2S_RX_CLK, its memory read at 0x50125C00, its interrupt
 *     LP_I2S_INTR (source 12), its signals LP matrix signals 9 to 11. The
 *     controllers clock and feed each other through whatever pads the
 *     matrices route between them, and I2Sn_MCLK_out reaches another's
 *     I2Sn_MCLK_in, a source of its unit clocks, the same way. The
 *     machine's audiodev stands for a codec on every controller's bus:
 *
 *       -audiodev coreaudio,id=snd0 -M esp32p4,audiodev=snd0
 *
 *   - The RMT (TRM chapter 60) at 0x500A2000, behind RMT_SYS_CLK and its
 *     reset, timed by RMT_CLK, its interrupt RMT_INTR (source 43), its four
 *     TX outputs and four RX inputs matrix signals 246 to 249, and TX
 *     channel 3 and RX channel 7 served by GDMA-AHB (PERI_SEL 10); and the
 *     PCNT (TRM chapter 51) at 0x500C9000, behind PCNT_APB_CLK and its
 *     reset, which also filters its inputs, its interrupt PCNT_INTR
 *     (source 111), its four units' signal, control and clear inputs
 *     matrix signals 141 to 156 and 63 to 66.
 *
 *   - The cryptography blocks (TRM chapters 27 to 36): the AES, SHA, RSA,
 *     ECC and HMAC accelerators, the RSA_DS and ECDSA_DS signature
 *     peripherals and the Key Manager (0x50089000 to 0x50096FFF), each on
 *     its CRYPTO_*_CLK, its registers behind CRYPTO_SYS_CLK
 *     (KEY_MANAGER_SYS_CLK for the Key Manager) and held by its resets and
 *     those of the blocks that reuse it; AES and SHA as GDMA-AXI's AES and
 *     SHA peripherals; their keys from the eFuses or the Key Manager, which
 *     also gives the MSPI controllers their encryption keys. In the LP
 *     system, the HUK Generator (0x50114000), whose SRAM PUF is the
 *     chip's, and the random number generator (0x50126000), which gives
 *     the host's random numbers. A chip's PUF follows its eFuses and the
 *     HUK Generator's puf-seed property:
 *
 *       -global esp32p4-huk.puf-seed=0x1234
 *
 *   - The LED PWM controller (TRM chapter 58) at 0x500D3000, behind
 *     LEDC_APB_CLK and its reset, its timers on LEDC_CLK, its interrupt
 *     LEDC_INTR (source 52) and its eight outputs matrix signals 126 to
 *     133; and MCPWM0 and MCPWM1 (TRM chapter 59) at 0x500C0000 and
 *     0x500C1000, each behind MCPWMn_APB_CLK and its reset, on MCPWMn_CLK,
 *     its interrupt PWMn_INTR (sources 38 and 39), its six outputs, sync,
 *     fault and capture inputs matrix signals 89 to 106. Their ETM events
 *     and tasks are the ETM's.
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
 * The LP ROM's contents are not published, and the machine ships no image
 * of it: the LP ROM is absent. Every access to it is logged as
 * unavailable, and a start of the LP core at the LP ROM's entry continues
 * at the application's boot address software has set
 * (LP_SYSTEM_BOOT_ADDR_HP_LP, where the ROM would start it), as ESP-IDF's
 * skip_lp_rom_boot does.
 *
 * The HP ROM image is the esp32p4-rev3-rom.bin firmware file, or the raw
 * image -bios names. With -kernel, an ELF's segments are loaded into
 * L2MEM (past the L2 cache), the TCM or LP SRAM, and core 0 starts at the
 * ELF's entry point rather than in the ROM, with the caches empty; core 1
 * is still held for software to release. The program finds the super
 * watchdog as the ROM leaves the program it boots: running since the
 * reset, unfed, its SWD_RESET_FLAG cleared (the ROM's main() clears it on
 * core 0 at every boot), and its reset not enabled by LP_SYSTEM's FIB
 * control until software enables it.
 *
 * The ELF stands for the program the ROM boots from flash, and takes the
 * ROM's place whenever the HP system starts: after an HP core reset that
 * selects the ROM (LP_CLKRST_HPCORE0_STAT_VECTOR_SEL), and so after a
 * deep sleep, core 0 starts at the entry point. The ROM on such a wakeup
 * (PMU HP power down reset, 0x05) takes LP_SYSTEM_LP_STORE8 bit 0 to tell
 * a deep sleep, after which it boots from flash as from power-on, from a
 * light sleep, after which it jumps to the address in the rest of the
 * word; it runs no deep-sleep wake stub itself, ESP-IDF's bootloader and
 * application doing that. As the deep sleep powered the HP memories down,
 * losing L2MEM and the TCM, the machine first loads the ELF's segments
 * there again, as the boot from flash would (leaving LP SRAM, which keeps
 * its contents, as ESP-IDF's bootloader leaves the LP memory on a deep
 * sleep wakeup); the HP pads stay held until the program releases them,
 * as the ROM does at boot (PMU_TIE_LOW_HP_PAD_HOLD_ALL). Without -kernel
 * the ROM runs again, as on the chip.
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
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/unimp.h"
#include "hw/riscv/esp32p4.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/tcg.h"
#include "exec/translation-block.h"

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
    { "l2mem-monitor",    0x3ff0e000, 4 * KiB },
    { "tcm-monitor",      0x3ff0f000, 4 * KiB },

    /* HP peripherals 0. */
    { "pau",              0x50082000, 4 * KiB },
    { "h264",             0x50084000, 4 * KiB },
    { "jpeg",             0x50086000, 4 * KiB },
    { "ppa",              0x50087000, 4 * KiB },
    { "dma2d",            0x50088000, 4 * KiB },
    { "ddr-phy",          0x5009d000, 4 * KiB },
    { "pvt",              0x5009e000, 4 * KiB },
    { "csi-host",         0x5009f000, 2 * KiB },
    { "csi-bridge",       0x5009f800, 2 * KiB },
    { "dsi-host",         0x500a0000, 2 * KiB },
    { "dsi-bridge",       0x500a0800, 2 * KiB },
    { "isp",              0x500a1000, 4 * KiB },
    /* The AXI ICM's QoS command interface, above its own registers. */
    { "axi-icm-qos",      0x500a4400, 3 * KiB },
    { "h264-dma",         0x500a7000, 4 * KiB },

    /* HP peripherals 1. */
    { "lcd-cam",          0x500dc000, 4 * KiB },
    { "adc",              0x500de000, 4 * KiB },
    { "iomux-mspi-pin",   0x500e1200, 4 * KiB - 0x200 },
    { "mem-monitor",      0x500e3000, 4 * KiB },
    { "audio-addc",       0x500e4000, 4 * KiB },

    /* The MIPI bridges' memories. */
    { "mipi-csi-mem",     0x50104000, 4 * KiB },
    { "mipi-dsi-mem",     0x50105000, 4 * KiB },

    /* LP always-on peripherals. */
    { "rtc-cali",         0x50119000, 4 * KiB },

    /* LP peripherals. */
    { "lp-spi",           0x50123000, 4 * KiB },
    { "lp-adc",           0x50127000, 4 * KiB },
    { "lp-touch",         0x50128000, 4 * KiB },
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

/*
 * The LP ROM, absent: Espressif does not publish its contents. An access
 * reads 0, as an unimplemented device's does, and is logged.
 */
/* [spec:nuos:req:emu.esp32p4.lp-rom] */
static uint64_t esp32p4_lp_rom_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "esp32p4.lp-rom: read of %u bytes at 0x%"
                  HWADDR_PRIx ": the LP ROM is unavailable\n", size, addr);
    return 0;
}

/* [spec:nuos:req:emu.esp32p4.lp-rom] */
static void esp32p4_lp_rom_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "esp32p4.lp-rom: write of %u bytes at 0x%"
                  HWADDR_PRIx ": the LP ROM is unavailable\n", size, addr);
}

static const MemoryRegionOps esp32p4_lp_rom_ops = {
    .read = esp32p4_lp_rom_read,
    .write = esp32p4_lp_rom_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/*
 * The LP core's start (TRM 5.4): 0x80 into the vector table at its boot
 * address, LP_SYSTEM_LP_CORE_BOOT_ADDR, the table's base taking the
 * address's upper 24 bits, as mtvec's does; mtvec takes the table. The
 * TRM gives the start as the boot address plus 0x80, from 0x50100000, the
 * LP ROM, after reset; ESP-IDF's skip_lp_rom_boot sets the boot address to
 * the application's reset vector, itself 0x80 into its 256-byte aligned
 * vector table, and the core starts there. Both hold when the core starts
 * at the boot address's 256-byte aligned base plus 0x80, which the LP
 * ROM's layout fits too: the jump table ESP-IDF links against begins at
 * 0x50100090, just past the entry.
 *
 * With the LP ROM absent, a start at its entry continues at the
 * application's boot address, LP_SYSTEM_BOOT_ADDR_HP_LP, to which the ROM
 * would hand over (ESP-IDF's lp_core_ll_set_app_boot_address()).
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
/* [spec:nuos:req:emu.esp32p4.lp-rom] */
static void esp32p4_lp_core_boot(void *opaque, uint32_t *pc, uint32_t *mtvec)
{
    ESP32P4MachineState *s = opaque;
    uint32_t base = esp32p4_lp_sys_lp_core_boot_addr(&s->lp_sys) & ~0xffu;

    *mtvec = base | 1;
    *pc = base + ESP32P4_LP_BOOT_OFFSET;
    if (!s->lp_rom_loaded && *pc == ESP32P4_LP_RESET_VECTOR) {
        *pc = esp32p4_lp_sys_lp_core_app_addr(&s->lp_sys);
        qemu_log_mask(LOG_UNIMP, "esp32p4: the LP ROM is unavailable: the LP "
                      "core starts at the application's boot address 0x%08"
                      PRIx32 "\n", *pc);
    }
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

    memory_region_init_io(&s->lp_rom, OBJECT(s), &esp32p4_lp_rom_ops, s,
                          "esp32p4.lp-rom", ESP32P4_LP_ROM_SIZE);
    memory_region_add_subregion(sys_mem, ESP32P4_LP_ROM_BASE, &s->lp_rom);

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
    /*
     * The board's RMII reference clock reaches PAD_EMAC_TXRX_CLK through
     * the EMAC's IO MUX function on its pad (esp32p4_init_emac()).
     */
    qdev_connect_clock_in(clkrst, "pad-emac-txrx-in",
                          qdev_get_clock_out(DEVICE(&s->emac), "pad-txrx"));
    /*
     * PAD_I2Sn_MCLK: I2Sn_MCLK_in, which the pads may connect to another
     * controller's I2Sn_MCLK_out (esp32p4_init_i2s()).
     */
    for (i = 0; i < ESP32P4_I2S_HP_COUNT; i++) {
        g_autofree char *in = g_strdup_printf("pad-i2s%d-mclk-in", i);

        qdev_connect_clock_in(clkrst, in,
                              qdev_get_clock_out(DEVICE(&s->i2s[i]),
                                                 "pad-mclk"));
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

        /*
         * A core's reset also resets its CLIC, stops its PC record in the
         * debug assistant and is its trace encoder's reset sideband.
         */
        object_initialize_child(OBJECT(s), "core-reset-split[*]",
                                &s->core_reset_split[i], TYPE_SPLIT_IRQ);
        qdev_prop_set_uint16(DEVICE(&s->core_reset_split[i]), "num-lines",
                             4);
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
    clock_set_source(qdev_get_clock_in(DEVICE(&s->lp_cpu), ESP32P4_CPU_LP_CLK),
                     qdev_get_clock_out(clkrst, "lp-core"));

    object_initialize_child(OBJECT(s), "lp-sys", &s->lp_sys,
                            TYPE_ESP32P4_LP_SYS);
    object_property_set_link(OBJECT(&s->lp_sys), "lp-cpu",
                             OBJECT(&s->lp_cpu), &error_abort);
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
 * HP_SYSTEM's registers, the CLINT, the AXI ICM and the debug assistant
 * (whose PC records survive it); the rest take theirs
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
        device_cold_reset(DEVICE(&s->assist_debug));
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
 * Their interrupts are sources of the interrupt matrix, and their ETM
 * events and tasks the ETM's (esp32p4_init_etm()). The RTC timer's wakeups
 * go to the PMU (esp32p4_init_low_power()); the LP core's timer interrupt
 * is a named GPIO line (see the device's header) that nothing takes until
 * the LP core's interrupts are modelled.
 * MWDT0 and the RTC watchdog run their flash boot protection when the
 * chip boots from flash through the boot ROM in SPI boot mode: when the
 * board's strapping (the strap-mode property) has GPIO35 high and there
 * is no -kernel, which loads the program in the ROM's place, with no flash
 * boot to protect. Pads driven from outside at reset, which GPIO_STRAP
 * latches in place of the strapping, and LP_SYSTEM_FORCE_DOWNLOAD_BOOT do
 * not change it. LP_SYSTEM's FIB control enables the super watchdog's
 * reset (see esp32p4_init_debug()).
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
                             3);
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
    object_property_set_link(OBJECT(m), "keymng", OBJECT(&s->keymng),
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
 * by I3C_MST_CLK and served by GDMA-AHB as its peripheral 0; their
 * interrupts sources 101 and 102; their lines the
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
    object_property_set_link(OBJECT(mst), "gdma", OBJECT(&s->gdma[0]),
                             &error_abort);
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

/* The EMAC's system time, for HP_SYSTEM_GMAC_CTRL1 and _CTRL2 */
static uint64_t esp32p4_emac_ptp_time(void *opaque)
{
    return esp32_dwmac_ptp_time(ESP32_DWMAC(opaque));
}

/*
 * The board's Ethernet: the EMAC, its registers behind EMAC_SYS_CLK, held
 * in reset by LP_CLKRST_RST_EN_EMAC, its clocks from HP_SYS_CLKRST, its
 * DMA through the APMs, its interrupts ETH_MAC_INTR, PMT_INTR and
 * LPI_INTR (sources 92, 91 and 90), HP_SYSTEM_GMAC_CTRL0, and its MII and
 * PPS signals in the GPIO matrix; and the Waveshare ESP32-P4-WIFI6-POE-
 * ETH's IP101GRI PHY, whose medium is the first -nic. The PHY's MDIO
 * address is 1 (PHYAD0 pulled up, PHYAD3 down); MDIO has a pull-up
 * (R59), and RESET_N, on GPIO51, is taken to be pulled up inside the PHY,
 * as the board fits neither of its own resistors there.
 */
/* [spec:nuos:req:emu.esp32p4.emac] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_emac(ESP32P4MachineState *s)
{
    static const struct {
        const char *in;
        const char *out;
    } clocks[] = {
        { "sys", "emac-sys" },
        { "rmii", "emac-rmii" },
        { "rx", "emac-rx" },
        { "tx", "emac-tx" },
        { "ptp-ref", "emac-ptp-ref" },
    };
    DeviceState *emac = DEVICE(&s->emac), *phy = DEVICE(&s->phy);
    DeviceState *clkrst = DEVICE(&s->clkrst), *gpio = DEVICE(&s->gpio);
    DeviceState *hp_sys = DEVICE(&s->hp_sys), *pps;
    SysBusDevice *sbd = SYS_BUS_DEVICE(emac);
    int i;

    qdev_prop_set_uint8(phy, "addr", 1);
    qdev_realize(phy, NULL, &error_fatal);
    qemu_register_resettable(OBJECT(phy));

    object_property_set_link(OBJECT(emac), "pms", OBJECT(&s->pms),
                             &error_abort);
    object_property_set_link(OBJECT(emac), "gpio", OBJECT(&s->gpio),
                             &error_abort);
    object_property_set_link(OBJECT(emac), "phy", OBJECT(phy), &error_abort);
    for (i = 0; i < ARRAY_SIZE(clocks); i++) {
        qdev_connect_clock_in(emac, clocks[i].in,
                              qdev_get_clock_out(clkrst, clocks[i].out));
    }
    qdev_connect_clock_in(emac, "phy-ref",
                          qdev_get_clock_out(phy, "rmii-ref"));
    qemu_configure_nic_device(emac, true, NULL);
    sysbus_realize(sbd, &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, sbd, 0, ESP32P4_EMAC_BASE,
                             ESP32P4_CLK_EMAC_SYS,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_EMAC,
                                qdev_get_gpio_in_named(emac,
                                    ESP32P4_EMAC_RESET, 0));
    sysbus_connect_irq(sbd, ESP32_DWMAC_IRQ_SBD,
                       esp32p4_intr_source(s, ESP32P4_INTR_ETH_MAC));
    sysbus_connect_irq(sbd, ESP32_DWMAC_IRQ_PMT,
                       esp32p4_intr_source(s, ESP32P4_INTR_PMT));
    sysbus_connect_irq(sbd, ESP32_DWMAC_IRQ_LPI,
                       esp32p4_intr_source(s, ESP32P4_INTR_LPI));

    for (i = 0; i < ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES; i++) {
        qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_GMAC_PHY_INTF, i,
                                    qdev_get_gpio_in_named(emac,
                                        ESP32P4_EMAC_PHY_INTF, i));
    }
    qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL, 0,
                                qdev_get_gpio_in_named(emac,
                                    ESP32_DWMAC_SBD_FLOWCTRL, 0));
    qdev_connect_gpio_out_named(emac, ESP32P4_EMAC_RST_N, 0,
                                qdev_get_gpio_in_named(hp_sys,
                                    ESP32P4_HP_SYS_GMAC_STATUS,
                                    ESP32P4_HP_SYS_GMAC_RST_CLK_TX_N));
    qdev_connect_gpio_out_named(emac, ESP32P4_EMAC_RST_N, 1,
                                qdev_get_gpio_in_named(hp_sys,
                                    ESP32P4_HP_SYS_GMAC_STATUS,
                                    ESP32P4_HP_SYS_GMAC_RST_CLK_RX_N));
    esp32p4_hp_sys_set_gmac_ptp(&s->hp_sys, esp32p4_emac_ptp_time,
                                &s->emac);

    object_initialize_child(OBJECT(s), "pps-split", &s->pps_split,
                            TYPE_SPLIT_IRQ);
    pps = DEVICE(&s->pps_split);
    qdev_prop_set_uint16(pps, "num-lines", 2);
    qdev_realize(pps, NULL, &error_fatal);
    qdev_connect_gpio_out_named(emac, ESP32_DWMAC_PTP_PPS, 0,
                                qdev_get_gpio_in(pps, 0));
    qdev_connect_gpio_out(pps, 0,
                          qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                              ESP32P4_SIG_EMAC_PTP_PPS_PAD_OUT));
    qdev_connect_gpio_out(pps, 1,
                          qdev_get_gpio_in_named(hp_sys,
                              ESP32P4_HP_SYS_GMAC_STATUS,
                              ESP32P4_HP_SYS_GMAC_PTP_PPS));

    qdev_connect_gpio_out_named(emac, ESP32P4_EMAC_MDC, 0,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_SIG_OUT,
                                    ESP32P4_SIG_MII_MDC_PAD_OUT));
    qdev_connect_gpio_out_named(emac, ESP32P4_EMAC_MDO, 0,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_SIG_OUT,
                                    ESP32P4_SIG_MII_MDO_PAD_OUT));
    qdev_connect_gpio_out_named(emac, ESP32P4_EMAC_MDO_OE, 0,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_SIG_OE,
                                    ESP32P4_SIG_MII_MDO_PAD_OUT));
    qdev_connect_gpio_out_named(emac, ESP32P4_EMAC_TX_EN, 0,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_SIG_OUT,
                                    ESP32P4_SIG_EMAC_PHY_TXEN_PAD_OUT));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                ESP32P4_SIG_MII_MDI_PAD_IN,
                                qdev_get_gpio_in_named(emac,
                                    ESP32P4_EMAC_MDI, 0));

    /* The board's pull-ups, and RESET_N following its pad. */
    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_PAD_PULL_UP,
                                        s->emac.pad_mdio), 1);
    qemu_set_irq(qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_PAD_PULL_UP,
                                        ESP32P4_BOARD_PHY_RESET_PAD), 1);
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_PAD_OUT,
                                ESP32P4_BOARD_PHY_RESET_PAD,
                                qemu_irq_invert(qdev_get_gpio_in_named(phy,
                                    IP101_PHY_RESET, 0)));
}

/*
 * The SD/MMC host controller and the cards on its slots. Its registers
 * are behind SDMMC_SYS_CLK and LP_CLKRST's SDMMC reset, BUFFIFO, whose
 * read pops the FIFO, reading 0 while gated; its CIU takes
 * SDIO_LS_SLF_CLK, and needs
 * SDIO_LS_DRV_CLK and SDIO_LS_SAM_CLK to reach a card. Its card-side
 * inputs and outputs are GPIO matrix signals: SD_CARD_DETECT_N_1 and _2,
 * SD_CARD_INT_N_1 and _2, SD_CARD_WRITE_PRT_1 and _2 in, SD_RST_N_1 and _2
 * and SD_CCMD_OD_PULLUP_EN_N out (slot 0 is the matrix's card 1).
 */
/* [spec:nuos:req:emu.esp32p4.sdmmc] */
static void esp32p4_init_sdmmc(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst), *gpio = DEVICE(&s->gpio);
    DeviceState *dev;
    ESP32P4ClkRstGate *g;
    int slot;

    object_initialize_child(OBJECT(s), "sdmmc", &s->sdmmc,
                            TYPE_ESP32P4_SDMMC);
    dev = DEVICE(&s->sdmmc);
    object_property_set_link(OBJECT(dev), "pms", OBJECT(&s->pms),
                             &error_abort);
    qdev_connect_clock_in(dev, "hclk",
                          qdev_get_clock_out(clkrst, "sdmmc-sys"));
    qdev_connect_clock_in(dev, "cclk-in",
                          qdev_get_clock_out(clkrst, "sdio-slf"));
    qdev_connect_clock_in(dev, "drv", qdev_get_clock_out(clkrst, "sdio-drv"));
    qdev_connect_clock_in(dev, "sam", qdev_get_clock_out(clkrst, "sdio-sam"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 ESP32P4_SDMMC_BASE, ESP32P4_CLK_SDMMC_SYS,
                                 ESP32P4_RST_SDMMC, 0);
    esp32p4_clkrst_gate_volatile(g, 0x200, 4);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_SDIO_HOST));

    for (slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
        BusState *bus = qdev_get_child_bus(dev, slot ? "sd-bus1" : "sd-bus0");
        DriveInfo *dinfo = drive_get(IF_SD, 0, slot);

        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
            ESP32P4_SIG_SD_CARD_DETECT_N_1_PAD_IN + slot,
            qdev_get_gpio_in_named(dev, DWC_SDMMC_CARD_DETECT_N, slot));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
            ESP32P4_SIG_SD_CARD_WRITE_PRT_1_PAD_IN + slot,
            qdev_get_gpio_in_named(dev, DWC_SDMMC_CARD_WRITE_PRT, slot));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
            ESP32P4_SIG_SD_CARD_INT_N_1_PAD_IN + slot,
            qdev_get_gpio_in_named(dev, DWC_SDMMC_CARD_INT_N, slot));
        qdev_connect_gpio_out_named(dev, DWC_SDMMC_CARD_RST_N, slot,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                                   ESP32P4_SIG_SD_RST_N_1_PAD_OUT + slot));

        /* The card resets at power-on only, not with the controller. */
        qbus_set_reset_domain(bus);
        qemu_register_resettable(OBJECT(bus));
        if (dinfo) {
            DeviceState *card = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(card, "drive",
                                    blk_by_legacy_dinfo(dinfo),
                                    &error_fatal);
            qdev_realize_and_unref(card, bus, &error_fatal);
        }
    }
    qdev_connect_gpio_out_named(dev, DWC_SDMMC_CCMD_OD_PULLUP_EN_N, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                               ESP32P4_SIG_SD_CCMD_OD_PULLUP_EN_N_PAD_OUT));
}

/*
 * The DMA controllers, on the bus through the APMs: GDMA-AHB and GDMA-AXI,
 * each behind its bus clock and reset, which also clock its transfers,
 * and VDMA behind GDMA_SYS_CLK and its reset.
 */
/* [spec:nuos:req:emu.esp32p4.dma] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_dma(ESP32P4MachineState *s)
{
    static const struct {
        const char *type;
        const char *clock;
        hwaddr base;
        unsigned bus_clock;
        unsigned reset;
        int intr;
    } gdma[2] = {
        { TYPE_ESP32P4_AHB_GDMA, "ahb-pdma-sys", ESP32P4_AHB_GDMA_BASE,
          ESP32P4_CLK_AHB_PDMA_SYS, ESP32P4_RST_AHB_PDMA,
          ESP32P4_INTR_AHB_PDMA_IN_CH0 },
        { TYPE_ESP32P4_AXI_GDMA, "axi-pdma-sys", ESP32P4_AXI_GDMA_BASE,
          ESP32P4_CLK_AXI_PDMA_SYS, ESP32P4_RST_AXI_PDMA,
          ESP32P4_INTR_AXI_PDMA_IN_CH0 },
    };
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *dev;
    int i, n;

    for (i = 0; i < 2; i++) {
        object_initialize_child(OBJECT(s), i ? "axi-gdma" : "ahb-gdma",
                                &s->gdma[i], gdma[i].type);
        dev = DEVICE(&s->gdma[i]);
        object_property_set_link(OBJECT(dev), "pms", OBJECT(&s->pms),
                                 &error_abort);
        qdev_connect_clock_in(dev, "clk",
                              qdev_get_clock_out(clkrst, gdma[i].clock));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 gdma[i].base, gdma[i].bus_clock,
                                 gdma[i].reset, 0);
        for (n = 0; n < 2 * ESP32P4_GDMA_CHANNELS; n++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), n,
                               esp32p4_intr_source(s, gdma[i].intr + n));
        }
    }

    object_initialize_child(OBJECT(s), "vdma", &s->vdma,
                            TYPE_ESP32P4_DW_GDMA);
    dev = DEVICE(&s->vdma);
    object_property_set_link(OBJECT(dev), "pms", OBJECT(&s->pms),
                             &error_abort);
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(clkrst, "gdma-sys"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_DW_GDMA_BASE, ESP32P4_CLK_GDMA_SYS,
                             ESP32P4_RST_GDMA, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_DW_GDMA));
}

/*
 * GP-SPI2 and GP-SPI3, their registers behind GPSPI2_APB_CLK and
 * GPSPI3_APB_CLK and their resets, their data paths on GPSPI*_SYS_CLK and
 * their masters on clk_spi_mst (GPSPI*_MST_CLK); their DMA through
 * GDMA-AXI, their interrupts sources 25 and 26, their bus signals through
 * the GPIO matrix (and, for GP-SPI2, the IO MUX), and each the other's
 * peer on the pads. Their buses are "spi2" and "spi3".
 */
/* [spec:nuos:req:emu.esp32p4.spi] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_spi(ESP32P4MachineState *s)
{
    static const struct {
        hwaddr base;
        unsigned bus_clock;
        unsigned reset;
        int intr;
        const char *mst;
        const char *sys;
    } spi[2] = {
        { ESP32P4_SPI2_BASE, ESP32P4_CLK_GPSPI2_APB, ESP32P4_RST_SPI2,
          ESP32P4_INTR_SPI2, "gpspi2-mst", "gpspi2-sys" },
        { ESP32P4_SPI3_BASE, ESP32P4_CLK_GPSPI3_APB, ESP32P4_RST_SPI3,
          ESP32P4_INTR_SPI3, "gpspi3-mst", "gpspi3-sys" },
    };
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *gpio = DEVICE(&s->gpio);
    int i, line;

    for (i = 0; i < 2; i++) {
        object_initialize_child(OBJECT(s), i ? "spi3" : "spi2", &s->spi[i],
                                TYPE_ESP32P4_SPI);
    }
    for (i = 0; i < 2; i++) {
        DeviceState *dev = DEVICE(&s->spi[i]);
        SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
        unsigned host = 2 + i;

        qdev_prop_set_uint8(dev, "host", host);
        object_property_set_link(OBJECT(dev), "gdma", OBJECT(&s->gdma[1]),
                                 &error_abort);
        object_property_set_link(OBJECT(dev), "gpio", OBJECT(gpio),
                                 &error_abort);
        object_property_set_link(OBJECT(dev), "peer", OBJECT(&s->spi[!i]),
                                 &error_abort);
        qdev_connect_clock_in(dev, "mst",
                              qdev_get_clock_out(clkrst, spi[i].mst));
        qdev_connect_clock_in(dev, "sys",
                              qdev_get_clock_out(clkrst, spi[i].sys));
        sysbus_realize(sbd, &error_fatal);
        esp32p4_clkrst_map_gated(&s->clkrst, sbd, 0, spi[i].base,
                                 spi[i].bus_clock, spi[i].reset, 0);
        sysbus_connect_irq(sbd, 0, esp32p4_intr_source(s, spi[i].intr));
        for (line = 0; line < ESP32P4_SPI_LINES; line++) {
            int sig = esp32p4_spi_line_signal(host, line);

            if (sig < 0) {
                continue;
            }
            qdev_connect_gpio_out_named(dev, ESP32P4_SPI_SIG_OUT, line,
                qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT, sig));
            qdev_connect_gpio_out_named(dev, ESP32P4_SPI_SIG_OE, line,
                qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OE, sig));
            if (esp32p4_spi_line_has_input(host, line)) {
                qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, sig,
                    qdev_get_gpio_in_named(dev, ESP32P4_SPI_SIG_IN, line));
            }
        }
    }
}

/* What the HP memories hold as they come back from a power-down. */
#define ESP32P4_LOST_MEM_PATTERN 0xa5

/* Fill RAM region @mr as a powered-down memory comes back. */
static void esp32p4_lose_memory(MemoryRegion *mr)
{
    uint64_t size = memory_region_size(mr);
    ram_addr_t ra = memory_region_get_ram_addr(mr);

    memset(memory_region_get_ram_ptr(mr), ESP32P4_LOST_MEM_PATTERN, size);
    memory_region_set_dirty(mr, 0, size);
    if (tcg_enabled()) {
        tb_invalidate_phys_range(NULL, ra, ra + size - 1);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * The PMU powers the HP memories down: L2MEM and the TCM lose their
 * contents (TRM 15.4.1.2; LP SRAM, in the LP system, keeps its own).
 */
static void esp32p4_hp_mem_power(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;

    if (!level) {
        esp32p4_lose_memory(&s->l2mem);
        esp32p4_lose_memory(&s->tcm);
        s->hp_mem_lost = true;
    }
}

static void esp32p4_reload_kernel(ESP32P4MachineState *s);

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * The PMU powers PD_TOP, the HP system, down or up. As it comes back after
 * the HP memories were lost, -kernel's ELF is loaded again in place of the
 * ROM's boot from flash, before the HP cores start.
 */
static void esp32p4_hp_top_power(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;

    if (level && s->hp_mem_lost) {
        esp32p4_reload_kernel(s);
        s->hp_mem_lost = false;
    }
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->clkrst),
                                        ESP32P4_CLKRST_PMU_POWER,
                                        ESP32P4_CLKRST_PD_TOP), level);
}

/*
 * Low-power management (TRM chapters 15, 24 to 26), in the LP always-on
 * domain, its registers behind LP_DYN_FAST_CLK:
 *
 *   - The PMU, timed by LP_DYN_FAST_CLK and LP_DYN_SLOW_CLK, its PMU_INTR
 *     and LP interrupts sources 6 and 7: it powers the domains (PD_TOP and
 *     the HP memories through the machine, which keeps the memories' loss),
 *     the oscillators, the crystal and the PLLs, gates the HP system's
 *     clocks and ROOT_CLK, stalls the HP cores, starts and stops the LP
 *     core, holds the pads and switches them to their sleep configuration,
 *     pauses the RTC watchdog and lets the brown-out detector see VDD_BAT.
 *     Its wakeup sources: the GPIO and LP GPIO wakeups, the RTC timer's
 *     targets, the brown-out detector and VDD_BAT's under-voltage,
 *     HP_SYSTEM's interrupt wakeup (the TRM's "HP CPU" source) and the EXT
 *     wakeup on pads GPIO0 to GPIO15, which LP_SYSTEM records too. Its ETM
 *     task, PMU_TASK_SLEEP_REQ, and event, PMU_EVT_SLEEP_WAKEUP, are the
 *     ETM's (esp32p4_init_etm()), and the LP I2S's VAD wakeup is
 *     connected with the LP I2S (esp32p4_init_i2s()); the UARTs' wakeups
 *     and UART_WAKEUP_EN are the UARTs' (esp32p4_init_uart()); the LP
 *     core's exception and ETM wakeup sources and its interrupt mask are
 *     the LP core's (esp32p4_init_lp_core()). The other sources' "wakeup"
 *     lines wait for the blocks that take them (USB, SDIO, touch, EMAC);
 *     the LP core takes the PMU's interrupts as sources 6 and 7.
 *   - LP_ANALOG_PERI, behind its LP_CLKRST reset, its interrupt source 8:
 *     the brown-out detector, on BIAS's thresholds, and the glitch and
 *     power supply detectors, the detector enabled by LP_SYSTEM's FIB
 *     control (esp32p4_init_debug()).
 *   - The LP mailbox, behind its LP_CLKRST reset, MB_HP_INTR and MB_LP_INTR
 *     sources 4 and 5.
 */
/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_init_low_power(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *gpio = DEVICE(&s->gpio);
    DeviceState *pmu, *ana, *mb;
    int i;

    object_initialize_child(OBJECT(s), "pmu", &s->pmu, TYPE_ESP32P4_PMU);
    pmu = DEVICE(&s->pmu);
    object_property_set_link(OBJECT(pmu), "clkrst", OBJECT(&s->clkrst),
                             &error_abort);
    object_property_set_link(OBJECT(pmu), "lp-cpu", OBJECT(&s->lp_cpu),
                             &error_abort);
    object_property_set_link(OBJECT(pmu), "lp-sys", OBJECT(&s->lp_sys),
                             &error_abort);
    qdev_connect_clock_in(pmu, "fast", qdev_get_clock_out(clkrst,
                                                          "lp-dyn-fast"));
    qdev_connect_clock_in(pmu, "slow", qdev_get_clock_out(clkrst,
                                                          "lp-dyn-slow"));
    sysbus_realize(SYS_BUS_DEVICE(pmu), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(pmu), 0,
                             ESP32P4_PMU_BASE, ESP32P4_CLK_LP_DYN_FAST,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(pmu), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_PMU_0));
    sysbus_connect_irq(SYS_BUS_DEVICE(pmu), 1,
                       esp32p4_intr_source(s, ESP32P4_INTR_PMU_1));

    s->hp_top_power = qemu_allocate_irq(esp32p4_hp_top_power, s, 0);
    s->hp_mem_power = qemu_allocate_irq(esp32p4_hp_mem_power, s, 0);
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_POWER, ESP32P4_PMU_PD_TOP,
                                s->hp_top_power);
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_POWER,
                                ESP32P4_PMU_PD_HP_MEM, s->hp_mem_power);
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_POWER,
                                ESP32P4_PMU_PD_HP_CPU,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_POWER,
                                    ESP32P4_CLKRST_PD_HP_CPU));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_POWER,
                                ESP32P4_PMU_PD_HP_CNNT,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_POWER,
                                    ESP32P4_CLKRST_PD_HP_CNNT));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_POWER,
                                ESP32P4_PMU_PD_LP_PERI,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_POWER,
                                    ESP32P4_CLKRST_PD_LP_PERI));
    for (i = 0; i < ESP32P4_CLKRST_NUM_XPD; i++) {
        qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_XPD, i,
                                    qdev_get_gpio_in_named(clkrst,
                                        ESP32P4_CLKRST_PMU_XPD, i));
    }
    for (i = 0; i < ESP32P4_ANA_NUM_PLLS; i++) {
        qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_PLL_XPD, i,
                                    qdev_get_gpio_in_named(DEVICE(&s->ana),
                                        ESP32P4_ANA_XPD, i));
    }
    for (i = 0; i < ESP32P4_HP_CORES; i++) {
        qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_CORE_STALL, i,
                                    qdev_get_gpio_in_named(clkrst,
                                        ESP32P4_CLKRST_PMU_CORE_STALL, i));
    }
    for (i = 0; i < ESP32P4_CLKRST_NUM_PMU_ICG; i++) {
        qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_ICG, i,
                                    qdev_get_gpio_in_named(clkrst,
                                        ESP32P4_CLKRST_PMU_ICG, i));
    }
    for (i = 0; i < ESP32P4_CLKRST_NUM_SYSCLK; i++) {
        qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_SYSCLK, i,
                                    qdev_get_gpio_in_named(clkrst,
                                        ESP32P4_CLKRST_PMU_SYSCLK, i));
    }
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_LP_SLEEP, 0,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_LP_SLEEP, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_LP_CORE_RESET, 0,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_LP_CORE_RESET, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_LP_CORE_CLOCK, 0,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_LP_CORE_CLOCK, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_LP_CORE_STALL, 0,
                                qdev_get_gpio_in_named(clkrst,
                                    ESP32P4_CLKRST_PMU_LP_CORE_STALL, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_HOLD_ALL,
                                ESP32P4_PMU_HOLD_HP,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_HOLD_ALL,
                                    ESP32P4_GPIO_HOLD_ALL_HP));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_HOLD_ALL,
                                ESP32P4_PMU_HOLD_LP,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_HOLD_ALL,
                                    ESP32P4_GPIO_HOLD_ALL_LP));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_PAD_SLEEP, 0,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_LIGHT_SLEEP, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_PAUSE_WDT, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_wdt),
                                    ESP32P4_LP_WDT_SLEEP, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_XTAL_OFF, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_timer),
                                    ESP32P4_LP_TIMER_XTAL_OFF, 0));

    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_WAKEUP, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_GPIO));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_LP_WAKEUP, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_LP_GPIO));
    qdev_connect_gpio_out_named(DEVICE(&s->hp_sys), ESP32P4_HP_SYS_HP_WAKEUP,
                                0, qdev_get_gpio_in_named(pmu,
                                       ESP32P4_PMU_WAKEUP,
                                       ESP32P4_PMU_WAKE_HP_CPU));
    qdev_connect_gpio_out_named(DEVICE(&s->lp_timer),
                                ESP32P4_LP_TIMER_SOC_WAKEUP, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_RTC_TIMER0));
    qdev_connect_gpio_out_named(DEVICE(&s->lp_timer),
                                ESP32P4_LP_TIMER_LP_WAKEUP, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_RTC_TIMER1));
    for (i = 0; i < ESP32P4_PMU_EXT_PADS; i++) {
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_PAD_OUT, i,
                                    qdev_get_gpio_in_named(pmu,
                                        ESP32P4_PMU_EXT_PAD, i));
    }

    object_initialize_child(OBJECT(s), "lp-ana", &s->lp_ana,
                            TYPE_ESP32P4_LP_ANA);
    ana = DEVICE(&s->lp_ana);
    object_property_set_link(OBJECT(ana), "clkrst", OBJECT(&s->clkrst),
                             &error_abort);
    object_property_set_link(OBJECT(ana), "ana", OBJECT(&s->ana),
                             &error_abort);
    object_property_set_link(OBJECT(ana), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    qdev_connect_clock_in(ana, "fast", qdev_get_clock_out(clkrst,
                                                          "lp-dyn-fast"));
    sysbus_realize(SYS_BUS_DEVICE(ana), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(ana), 0,
                             ESP32P4_LP_ANA_BASE, ESP32P4_CLK_LP_DYN_FAST,
                             ESP32P4_RST_LP_ANAPERI, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(ana), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_ANAPERI));
    qdev_connect_gpio_out_named(DEVICE(&s->ana), ESP32P4_ANA_BIAS_UPDATE, 0,
                                qdev_get_gpio_in_named(ana,
                                    ESP32P4_LP_ANA_BIAS_UPDATE, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_BOD_VBAT_SEL, 0,
                                qdev_get_gpio_in_named(ana,
                                    ESP32P4_LP_ANA_VBAT_SEL, 0));
    qdev_connect_gpio_out_named(ana, ESP32P4_LP_ANA_BROWNOUT, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_BROWNOUT));
    qdev_connect_gpio_out_named(ana, ESP32P4_LP_ANA_VBAT_UNDERVOLTAGE, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_VBAT));

    object_initialize_child(OBJECT(s), "lp-mailbox", &s->lp_mailbox,
                            TYPE_ESP32P4_LP_MAILBOX);
    mb = DEVICE(&s->lp_mailbox);
    sysbus_realize(SYS_BUS_DEVICE(mb), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(mb), 0,
                             ESP32P4_LP_MAILBOX_BASE, ESP32P4_CLK_LP_DYN_FAST,
                             ESP32P4_RST_LP_MAILBOX, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(mb), ESP32P4_LP_MAILBOX_IRQ_HP,
                       esp32p4_intr_source(s, ESP32P4_INTR_MB_HP));
    sysbus_connect_irq(SYS_BUS_DEVICE(mb), ESP32P4_LP_MAILBOX_IRQ_LP,
                       esp32p4_intr_source(s, ESP32P4_INTR_MB_LP));
}

/*
 * The LP core (TRM chapter 5):
 *
 *   - Its interrupt controller, LP_INTR (0x5012C000), behind LPPERI's
 *     LP_INTR clock and reset, taking the LP interrupt sources from the
 *     interrupt matrix's fan-out of every source's level and HP_INTR from
 *     HP_SYSTEM, its own software interrupt LP_SW_INTR being source 18;
 *     the PMU's sleep process masks it.
 *   - Its start, from LP_SYSTEM's boot address and the LP ROM
 *     (esp32p4_lp_core_boot()), and LP_SYSTEM's LP_CORE_DISABLE.
 *   - Its exception, the PMU's LP CPU exception wakeup source, and
 *     LP_SYSTEM's LP_CORE_ETM_WAKEUP_FLAG, the PMU's ETM wakeup source.
 */
/* [spec:nuos:req:emu.esp32p4.lp-core] */
static void esp32p4_init_lp_core(ESP32P4MachineState *s)
{
    DeviceState *lp_cpu = DEVICE(&s->lp_cpu);
    DeviceState *lp_sys = DEVICE(&s->lp_sys);
    DeviceState *pmu = DEVICE(&s->pmu);
    DeviceState *intr;
    int n;

    object_initialize_child(OBJECT(s), "lp-intr", &s->lp_intr,
                            TYPE_ESP32P4_LP_INTR);
    intr = DEVICE(&s->lp_intr);
    sysbus_realize(SYS_BUS_DEVICE(intr), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(intr), 0,
                             ESP32P4_LP_INTR_BASE, ESP32P4_CLK_LP_INTR,
                             ESP32P4_RST_LP_INTR, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(intr), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_SW));
    for (n = 0; n < ESP32P4_LP_INTR_SOURCES; n++) {
        if (n == ESP32P4_INTR_LP_SW) {
            continue;
        }
        qdev_connect_gpio_out_named(DEVICE(&s->intmatrix),
                                    ESP32P4_INTMATRIX_SOURCE_OUT, n,
                                    qdev_get_gpio_in_named(intr,
                                        ESP32P4_LP_INTR_SOURCE, n));
    }
    qdev_connect_gpio_out_named(DEVICE(&s->hp_sys), ESP32P4_HP_SYS_HP_INTR, 0,
                                qdev_get_gpio_in_named(intr,
                                    ESP32P4_LP_INTR_HP, 0));
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_LP_CORE_INTR_MASK, 0,
                                qdev_get_gpio_in_named(intr,
                                    ESP32P4_LP_INTR_MASK, 0));
    for (n = 0; n < ESP32P4_LP_INTR_CPU_IRQS; n++) {
        qdev_connect_gpio_out_named(intr, ESP32P4_LP_INTR_CPU_IRQ, n,
                                    qdev_get_gpio_in_named(lp_cpu,
                                        ESP32P4_CPU_LP_IRQ, n));
    }

    esp32p4_lp_cpu_set_boot(&s->lp_cpu, esp32p4_lp_core_boot, s);
    qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_LP_CORE_DISABLE, 0,
                                qdev_get_gpio_in_named(lp_cpu,
                                    ESP32P4_CPU_LP_DISABLE, 0));
    qdev_connect_gpio_out_named(lp_cpu, ESP32P4_CPU_LP_EXCEPTION, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_LP_CPU_EXC));
    qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_ETM_WAKEUP_FLAG, 0,
                                qdev_get_gpio_in_named(pmu,
                                    ESP32P4_PMU_WAKEUP,
                                    ESP32P4_PMU_WAKE_ETM));
}

/* LP_SYSTEM_SYS_SW_RST: the software system reset, an HP core reset. */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_sw_sys_reset(void *opaque, int n, int level)
{
    ESP32P4MachineState *s = opaque;

    if (level) {
        esp32p4_clkrst_hp_core_reset(&s->clkrst, ESP32P4_HP_CORE_RESET_SW,
                                     ESP32P4_SW_SYS_RESET_NS);
    }
}

/*
 * The system registers' connections and the debug blocks (TRM chapters 21
 * to 23 and 3):
 *
 *   - HP_SYSTEM's and LP_SYSTEM's bus error response disables to the HP
 *     cores and the LP core; HP_SYSTEM's WFI wakeup events to the HP cores;
 *     its HP2LP interrupt groups over sources 21 to 130 of the interrupt
 *     matrix, whose HP_INTR is the LP core's interrupt 30
 *     (esp32p4_init_lp_core()) and whose HP wakeup is the PMU's
 *     (esp32p4_init_low_power()); and the HP cores' stalls for
 *     CPU_CORESTALLED_ST.
 *   - The pads' holds: each pad's GPIO block "pad-hold" input is the OR of
 *     LP_SYSTEM_PAD_RTC_HOLD_CTRL's bit for it and, for GPIO16 to GPIO54,
 *     HP_SYSTEM_GPIO_O_HOLD_CTRL's (the PMU's holds of all the HP or LP
 *     pads are the GPIO block's "hold-all", esp32p4_init_low_power()).
 *   - LP_SYSTEM's software system reset, an HP core reset with cause 0x03
 *     (TRM table 11.1-1), its choice of the core whose stall the RTC
 *     timer follows, and its FIB control (TRM 21.2.2.2) of the super
 *     watchdog's reset, to the RTC watchdog block, and of the brownout
 *     detector, to LP_ANALOG_PERI. Its LP core disable is the LP core's
 *     (esp32p4_init_lp_core()).
 *   - The debug assistant (0x3FF06000), not gated by BUSMON_CPU_CLK_EN, its
 *     interrupt ASSIST_DEBUG_INTR (source 127), watching the HP cores and
 *     taking their resets for its PC records.
 *   - The AXI performance monitor (0x500A8000), behind AXI_PERF_MON_SYS_CLK
 *     and its reset, counting SYS_CLK, the AXI matrix's clock, its
 *     interrupt AXI_PERF_MON_INTR (source 130).
 *   - Each HP core's trace encoder (0x3FF04000, 0x3FF05000), behind
 *     TRACE_CPU_CLK and its RST_EN_CORETRACE<n> reset, its AHB master on
 *     TRACE_SYS_CLK writing the trace memory through the access permission
 *     managers, its interrupt CORE<n>_TRACE_INTR (sources 97, 98), taking
 *     its core's reset as its reset sideband.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_init_debug(ESP32P4MachineState *s)
{
    DeviceState *hp_sys = DEVICE(&s->hp_sys);
    DeviceState *lp_sys = DEVICE(&s->lp_sys);
    DeviceState *gpio = DEVICE(&s->gpio);
    DeviceState *dev;
    int core, n;

    for (core = 0; core < ESP32P4_HP_CORES; core++) {
        DeviceState *cpu = DEVICE(&s->hp_cpu[core]);

        for (n = 0; n < ESP32P4_BUSES; n++) {
            qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_ERR_RESP_DIS,
                                        core * ESP32P4_BUSES + n,
                                        qdev_get_gpio_in_named(cpu,
                                            ESP32P4_CPU_ERR_RESP_DIS, n));
        }
        qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_CPU_WAKEUP, core,
                                    qdev_get_gpio_in_named(cpu,
                                        ESP32P4_CPU_WAKEUP, 0));
        qdev_connect_gpio_out(DEVICE(&s->core_stall_split[core]), 2,
                              qdev_get_gpio_in_named(hp_sys,
                                  ESP32P4_HP_SYS_CORE_STALLED, core));
    }
    for (n = 0; n < ESP32P4_HP_SYS_HP2LP_SOURCES; n++) {
        qdev_connect_gpio_out_named(DEVICE(&s->intmatrix),
                                    ESP32P4_INTMATRIX_SOURCE_OUT,
                                    ESP32P4_HP_SYS_HP2LP_FIRST + n,
                                    qdev_get_gpio_in_named(hp_sys,
                                        ESP32P4_HP_SYS_HP_SOURCE, n));
    }
    for (n = 0; n < ESP32P4_BUSES; n++) {
        qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_ERR_RESP_DIS, n,
                                    qdev_get_gpio_in_named(DEVICE(&s->lp_cpu),
                                        ESP32P4_CPU_ERR_RESP_DIS, n));
    }

    for (n = 0; n < ESP32P4_GPIO_PADS; n++) {
        qemu_irq hold = qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_PAD_HOLD, n);
        int h = n - ESP32P4_HP_SYS_HOLD_FIRST;

        if (h < 0) {
            qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_PAD_HOLD, n,
                                        hold);
            continue;
        }
        object_initialize_child(OBJECT(s), "pad-hold-or[*]",
                                &s->pad_hold_or[h], TYPE_OR_IRQ);
        dev = DEVICE(&s->pad_hold_or[h]);
        qdev_prop_set_uint16(dev, "num-lines", 2);
        qdev_realize(dev, NULL, &error_fatal);
        qdev_connect_gpio_out(dev, 0, hold);
        qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_PAD_HOLD, n,
                                    qdev_get_gpio_in(dev, 0));
        qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_PAD_HOLD, h,
                                    qdev_get_gpio_in(dev, 1));
    }

    qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_SW_RESET, 0,
                                qemu_allocate_irq(esp32p4_sw_sys_reset, s, 0));
    qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_STALL_SEL, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_timer),
                                    ESP32P4_LP_TIMER_STALL_SEL, 0));
    qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_SWD_RESET_EN, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_wdt),
                                    ESP32P4_LP_WDT_SWD_RESET_EN, 0));
    qdev_connect_gpio_out_named(lp_sys, ESP32P4_LP_SYS_BOD_RESET_EN, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->lp_ana),
                                    ESP32P4_LP_ANA_BOD_EN, 0));

    object_initialize_child(OBJECT(s), "assist-debug", &s->assist_debug,
                            TYPE_ESP32P4_ASSIST_DEBUG);
    dev = DEVICE(&s->assist_debug);
    for (core = 0; core < ESP32P4_HP_CORES; core++) {
        g_autofree char *link = g_strdup_printf("cpu%d", core);

        object_property_set_link(OBJECT(dev), link,
                                 OBJECT(&s->hp_cpu[core]), &error_abort);
    }
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, ESP32P4_ASSIST_DEBUG_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_ASSIST_DEBUG));
    for (core = 0; core < ESP32P4_HP_CORES; core++) {
        qdev_connect_gpio_out(DEVICE(&s->core_reset_split[core]), 2,
                              qdev_get_gpio_in_named(dev,
                                  ESP32P4_ASSIST_DEBUG_CORE_RESET, core));
    }

    object_initialize_child(OBJECT(s), "axi-perf-mon", &s->axi_perf_mon,
                            TYPE_ESP32P4_AXI_PERF_MON);
    dev = DEVICE(&s->axi_perf_mon);
    qdev_connect_clock_in(dev, "axi", qdev_get_clock_out(DEVICE(&s->clkrst),
                                                         "sys"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_AXI_PERF_MON_BASE,
                             ESP32P4_CLK_AXI_PERF_MON_SYS,
                             ESP32P4_RST_AXI_PERF_MON, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_AXI_PERF_MON));

    for (core = 0; core < ESP32P4_HP_CORES; core++) {
        static const hwaddr base[] = { ESP32P4_TRACE0_BASE,
                                       ESP32P4_TRACE1_BASE };

        object_initialize_child(OBJECT(s), "trace[*]", &s->trace[core],
                                TYPE_ESP32P4_TRACE);
        dev = DEVICE(&s->trace[core]);
        qdev_prop_set_uint8(dev, "core", core);
        object_property_set_link(OBJECT(dev), "cpu",
                                 OBJECT(&s->hp_cpu[core]), &error_abort);
        object_property_set_link(OBJECT(dev), "pms", OBJECT(&s->pms),
                                 &error_abort);
        qdev_connect_clock_in(dev, "sys-clk",
                              qdev_get_clock_out(DEVICE(&s->clkrst),
                                                 "trace-sys"));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 base[core], ESP32P4_CLK_TRACE_CPU,
                                 core ? ESP32P4_RST_CORETRACE1
                                      : ESP32P4_RST_CORETRACE0, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           esp32p4_intr_source(s, core ?
                                               ESP32P4_INTR_CORE1_TRACE :
                                               ESP32P4_INTR_CORE0_TRACE));
        qdev_connect_gpio_out(DEVICE(&s->core_reset_split[core]), 3,
                              qdev_get_gpio_in_named(dev,
                                  ESP32P4_TRACE_CORE_RESET, 0));
    }
}

/*
 * The TWAI controllers, each behind TWAIn_APB_CLK, timed by TWAIn_CLK and
 * reset by HP_SYS_CLKRST_RST_EN_CANn, which the device takes itself as it
 * resets one controller of three; their interrupts TWAI0_INTR to
 * TWAI2_INTR (sources 40 to 42) and their pad signals; and the QEMU CAN
 * bus on each one's pads. Reading the interrupt register clears it, and
 * reading the capture registers re-arms them.
 */
/* [spec:nuos:req:emu.esp32p4.twai] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_twai(ESP32P4MachineState *s)
{
    DeviceState *twai, *gpio = DEVICE(&s->gpio);
    DeviceState *clkrst = DEVICE(&s->clkrst);
    int i;

    object_initialize_child(OBJECT(s), "twai", &s->twai, TYPE_ESP32P4_TWAI);
    twai = DEVICE(&s->twai);
    object_property_set_link(OBJECT(twai), "gpio", OBJECT(&s->gpio),
                             &error_abort);
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        g_autofree char *link = g_strdup_printf("canbus%d", i);
        g_autofree char *clk = g_strdup_printf("twai%d", i);

        if (s->canbus[i]) {
            object_property_set_link(OBJECT(twai), link,
                                     OBJECT(s->canbus[i]), &error_abort);
        }
        qdev_connect_clock_in(twai, clk, qdev_get_clock_out(clkrst, clk));
    }
    sysbus_realize(SYS_BUS_DEVICE(twai), &error_fatal);
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4ClkRstGate *g;

        g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(twai), i,
                                     ESP32P4_TWAI0_BASE +
                                     i * ESP32P4_TWAI_STRIDE,
                                     ESP32P4_CLK_TWAI0_APB + i,
                                     ESP32P4_CLKRST_GATE_NO_RESET, 0);
        esp32p4_clkrst_gate_volatile(g, 0x0c, 4);
        esp32p4_clkrst_gate_volatile(g, 0x2c, 4);
        esp32p4_clkrst_gate_volatile(g, 0x30, 4);
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                    ESP32P4_RST_CAN0 + i,
                                    qdev_get_gpio_in_named(twai,
                                        ESP32P4_TWAI_RESET, i));
        sysbus_connect_irq(SYS_BUS_DEVICE(twai), i,
                           esp32p4_intr_source(s, ESP32P4_INTR_TWAI0 + i));
        qdev_connect_gpio_out_named(twai, ESP32P4_TWAI_TX, i,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                ESP32P4_SIG_TWAI0_TX_PAD_OUT + 3 * i));
        qdev_connect_gpio_out_named(twai, ESP32P4_TWAI_BUS_OFF, i,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                ESP32P4_SIG_TWAI0_BUS_OFF_ON_PAD_OUT + 3 * i));
        qdev_connect_gpio_out_named(twai, ESP32P4_TWAI_CLKOUT, i,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                ESP32P4_SIG_TWAI0_CLKOUT_PAD_OUT + 3 * i));
        qdev_connect_gpio_out_named(twai, ESP32P4_TWAI_STANDBY, i,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                ESP32P4_SIG_TWAI0_STANDBY_PAD_OUT + i));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
            ESP32P4_SIG_TWAI0_RX_PAD_IN + 3 * i,
            qdev_get_gpio_in_named(twai, ESP32P4_TWAI_RX, i));
    }
}

/*
 * The LED PWM controller and MCPWM0 and MCPWM1: each behind its bus clock
 * and reset (RST_EN_LEDC, RST_EN_PWMn), timed by its function clock, its
 * interrupt a source of the interrupt matrix, its pad signals through the
 * GPIO matrix.
 */
/* [spec:nuos:req:emu.esp32p4.pwm] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
static void esp32p4_init_pwm(ESP32P4MachineState *s)
{
    DeviceState *dev, *gpio = DEVICE(&s->gpio);
    DeviceState *clkrst = DEVICE(&s->clkrst);
    int i, n;

    object_initialize_child(OBJECT(s), "ledc", &s->ledc, TYPE_ESP32P4_LEDC);
    dev = DEVICE(&s->ledc);
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(clkrst, "ledc"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_LEDC_BASE, ESP32P4_CLK_LEDC_APB,
                             ESP32P4_RST_LEDC, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LEDC));
    for (n = 0; n < ESP32P4_LEDC_CHANNELS; n++) {
        qdev_connect_gpio_out_named(dev, ESP32P4_LEDC_OUT, n,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                ESP32P4_SIG_LEDC_LS_SIG_OUT_PAD_OUT0 + n));
    }

    for (i = 0; i < ESP32P4_MCPWM_NUM; i++) {
        static const struct {
            int out, sync, fault, cap;
        } sig[ESP32P4_MCPWM_NUM] = {
            { ESP32P4_SIG_PWM0_CH0_A_PAD_OUT, ESP32P4_SIG_PWM0_SYNC0_PAD_IN,
              ESP32P4_SIG_PWM0_F0_PAD_IN, ESP32P4_SIG_PWM0_CAP0_PAD_IN },
            { ESP32P4_SIG_PWM1_CH0_A_PAD_OUT, ESP32P4_SIG_PWM1_SYNC0_PAD_IN,
              ESP32P4_SIG_PWM1_F0_PAD_IN, ESP32P4_SIG_PWM1_CAP0_PAD_IN },
        };
        g_autofree char *name = g_strdup_printf("mcpwm%d", i);

        object_initialize_child(OBJECT(s), name, &s->mcpwm[i],
                                TYPE_ESP32P4_MCPWM);
        dev = DEVICE(&s->mcpwm[i]);
        qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(clkrst, name));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 ESP32P4_MCPWM0_BASE +
                                 i * ESP32P4_MCPWM_STRIDE,
                                 ESP32P4_CLK_MCPWM0_APB + i,
                                 ESP32P4_RST_PWM0 + i, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           esp32p4_intr_source(s, ESP32P4_INTR_PWM0 + i));
        for (n = 0; n < ESP32_MCPWM_OUTS; n++) {
            qdev_connect_gpio_out_named(dev, ESP32_MCPWM_OUT, n,
                qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                                       sig[i].out + n));
        }
        for (n = 0; n < ESP32_MCPWM_SYNCS; n++) {
            qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                sig[i].sync + n,
                qdev_get_gpio_in_named(dev, ESP32_MCPWM_SYNC_IN, n));
        }
        for (n = 0; n < ESP32_MCPWM_FAULTS; n++) {
            qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                sig[i].fault + n,
                qdev_get_gpio_in_named(dev, ESP32_MCPWM_FAULT_IN, n));
        }
        for (n = 0; n < ESP32_MCPWM_CAPS; n++) {
            qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                sig[i].cap + n,
                qdev_get_gpio_in_named(dev, ESP32_MCPWM_CAP_IN, n));
        }
    }
}

/*
 * The BitScrambler, between GDMA and the peripherals HP_SYSTEM attaches it
 * to; and the parallel IO controller, served by GDMA-AXI, its pads through
 * the GPIO matrix.
 */
/* [spec:nuos:req:emu.esp32p4.parlio] */
static void esp32p4_init_parlio(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst), *gpio = DEVICE(&s->gpio);
    DeviceState *hp_sys = DEVICE(&s->hp_sys);
    DeviceState *dev;
    SysBusDevice *sbd;
    int i;

    object_initialize_child(OBJECT(s), "bitscrambler", &s->bitscrambler,
                            TYPE_ESP32P4_BITSCRAMBLER);
    dev = DEVICE(&s->bitscrambler);
    object_property_set_link(OBJECT(dev), "ahb-gdma", OBJECT(&s->gdma[0]),
                             &error_abort);
    object_property_set_link(OBJECT(dev), "axi-gdma", OBJECT(&s->gdma[1]),
                             &error_abort);
    qdev_connect_clock_in(dev, "tx-clk",
                          qdev_get_clock_out(clkrst, "bitscrambler-tx-sys"));
    qdev_connect_clock_in(dev, "rx-clk",
                          qdev_get_clock_out(clkrst, "bitscrambler-rx-sys"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_BITSCRAMBLER_BASE,
                             ESP32P4_CLK_BITSCRAMBLER_SYS,
                             ESP32P4_RST_BITSCRAMBLER, 0);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_BITSCRAMBLER_TX,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_BS_CORE_RESET, ESP32P4_BS_TX));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_BITSCRAMBLER_RX,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_BS_CORE_RESET, ESP32P4_BS_RX));
    for (i = 0; i < ESP32P4_HP_SYS_BS_PERI_SEL_LINES; i++) {
        qdev_connect_gpio_out_named(hp_sys, ESP32P4_HP_SYS_BS_PERI_SEL, i,
                                    qdev_get_gpio_in_named(dev,
                                        ESP32P4_BS_PERI_SEL, i));
    }

    object_initialize_child(OBJECT(s), "parlio", &s->parlio,
                            TYPE_ESP32P4_PARLIO);
    dev = DEVICE(&s->parlio);
    sbd = SYS_BUS_DEVICE(dev);
    object_property_set_link(OBJECT(dev), "gdma", OBJECT(&s->gdma[1]),
                             &error_abort);
    object_property_set_link(OBJECT(dev), "clkrst", OBJECT(&s->clkrst),
                             &error_abort);
    qdev_connect_clock_in(dev, "tx", qdev_get_clock_out(clkrst, "parlio-tx"));
    qdev_connect_clock_in(dev, "rx", qdev_get_clock_out(clkrst, "parlio-rx"));
    qdev_connect_clock_in(dev, "sys", qdev_get_clock_out(clkrst, "parlio-sys"));
    sysbus_realize(sbd, &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, sbd, 0, ESP32P4_PARLIO_BASE,
                             ESP32P4_CLK_PARLIO_APB, ESP32P4_RST_PARLIO, 0);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_PARLIO_TX,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_PARLIO_CORE_RESET,
                                    ESP32P4_PARLIO_TX));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_PARLIO_RX,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_PARLIO_CORE_RESET,
                                    ESP32P4_PARLIO_RX));
    sysbus_connect_irq(sbd, 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_HP_PARLIO_RX));
    sysbus_connect_irq(sbd, 1,
                       esp32p4_intr_source(s, ESP32P4_INTR_HP_PARLIO_TX));

    for (i = 0; i < ESP32P4_PARLIO_LINES; i++) {
        qdev_connect_gpio_out_named(dev, ESP32P4_PARLIO_TXD, i,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                                   ESP32P4_SIG_PARLIO_TX_DATA0_PAD_OUT + i));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
            ESP32P4_SIG_PARLIO_RX_DATA0_PAD_IN + i,
            qdev_get_gpio_in_named(dev, ESP32P4_PARLIO_RXD, i));
    }
    qdev_connect_gpio_out_named(dev, ESP32P4_PARLIO_TX_CLK_OUT, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                               ESP32P4_SIG_PARLIO_TX_CLK_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_PARLIO_RX_CLK_OUT, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                               ESP32P4_SIG_PARLIO_RX_CLK_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_PARLIO_TX_CS, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                               ESP32P4_SIG_PARLIO_TX_CS_PAD_OUT));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
        ESP32P4_SIG_PARLIO_TX_CLK_PAD_IN,
        qdev_get_gpio_in_named(dev, ESP32P4_PARLIO_TX_CLK_IN, 0));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
        ESP32P4_SIG_PARLIO_RX_CLK_PAD_IN,
        qdev_get_gpio_in_named(dev, ESP32P4_PARLIO_RX_CLK_IN, 0));
}

/*
 * I2S0 to I2S2 and the LP I2S (TRM chapters 49, 50 and 44). Each HP
 * controller's registers are behind I2Sn_APB_CLK and its reset; its units
 * run on I2Sn_TX_CLK and I2Sn_RX_CLK, and I2Sn_MCLK_out is
 * I2Sn_MST_CLK; GDMA-AHB serves it; its interrupt is I2Sn_INTR (sources
 * 27 to 29) and its signals are matrix signals 25 to 45. The LP I2S is
 * behind LPPERI's LP_I2S clock gate and reset, receives on LP_I2S_RX_CLK,
 * interrupts with LP_I2S_INTR (source 12) and has LP matrix signals 9 to
 * 11; its VAD's LP_I2S_WAKEUP is the PMU's wakeup source 19. Every
 * controller sees the others, so that one's master clock can drive
 * another's slave units through the pads.
 */
/* [spec:nuos:req:emu.esp32p4.i2s] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_i2s(ESP32P4MachineState *s)
{
    static const char *const links[ESP32P4_I2S_COUNT] = {
        "i2s0", "i2s1", "i2s2", "lp-i2s",
    };
    MachineState *ms = MACHINE(s);
    DeviceState *clkrst = DEVICE(&s->clkrst), *gpio = DEVICE(&s->gpio);
    ESP32P4ClkRstGate *g;
    int i, k, l;

    for (i = 0; i < ESP32P4_I2S_COUNT; i++) {
        DeviceState *dev = DEVICE(&s->i2s[i]);
        bool lp = i == ESP32P4_I2S_LP_ID;

        for (k = 0; k < ESP32P4_I2S_COUNT; k++) {
            object_property_set_link(OBJECT(dev), links[k],
                                     OBJECT(&s->i2s[k]), &error_abort);
        }
        object_property_set_link(OBJECT(dev), "gpio", OBJECT(&s->gpio),
                                 &error_abort);
        if (ms->audiodev) {
            qdev_prop_set_string(dev, "audiodev", ms->audiodev);
        }
        if (lp) {
            qdev_connect_clock_in(dev, "apb",
                                  qdev_get_clock_out(clkrst, "lp-i2s"));
            qdev_connect_clock_in(dev, "rx",
                                  qdev_get_clock_out(clkrst, "lp-i2s-rx"));
        } else {
            static const char *const clk[] = { "apb", "tx", "rx", "mst" };

            qdev_prop_set_uint8(dev, "id", i);
            object_property_set_link(OBJECT(dev), "gdma",
                                     OBJECT(&s->gdma[0]), &error_abort);
            for (k = 0; k < ARRAY_SIZE(clk); k++) {
                g_autofree char *out = g_strdup_printf("i2s%d-%s", i, clk[k]);

                qdev_connect_clock_in(dev, clk[k],
                                      qdev_get_clock_out(clkrst, out));
            }
        }
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        if (lp) {
            g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                         ESP32P4_LP_I2S_BASE,
                                         ESP32P4_CLK_LP_I2S,
                                         ESP32P4_RST_LP_I2S, 0);
            /* LP_I2S_RAM_BASE: reading pops the memory. */
            esp32p4_clkrst_gate_volatile(g, 0xc00, 0x400);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               esp32p4_intr_source(s, ESP32P4_INTR_LP_I2S));
            qdev_connect_gpio_out_named(dev, ESP32P4_I2S_WAKEUP, 0,
                                        qdev_get_gpio_in_named(
                                            DEVICE(&s->pmu),
                                            ESP32P4_PMU_WAKEUP,
                                            ESP32P4_PMU_WAKE_LP_I2S));
        } else {
            esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                     ESP32P4_I2S0_BASE + i * ESP32P4_I2S_SIZE,
                                     ESP32P4_CLK_I2S0_APB + i,
                                     ESP32P4_RST_I2S0_APB + i, 0);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               esp32p4_intr_source(s, ESP32P4_INTR_I2S0 + i));
        }
        for (l = 0; l < ESP32P4_I2S_OUT_COUNT; l++) {
            int sig = esp32p4_i2s_out_signal(i, l);

            if (sig >= 0) {
                qdev_connect_gpio_out_named(dev, ESP32P4_I2S_SIG_OUT, l,
                    qdev_get_gpio_in_named(gpio, lp ? ESP32P4_GPIO_LP_SIG_OUT
                                                    : ESP32P4_GPIO_SIG_OUT,
                                           sig));
            }
        }
        for (l = 0; l < ESP32P4_I2S_IN_COUNT; l++) {
            int sig = esp32p4_i2s_in_signal(i, l);

            if (sig >= 0) {
                qdev_connect_gpio_out_named(gpio, lp ? ESP32P4_GPIO_LP_SIG_IN
                                                     : ESP32P4_GPIO_SIG_IN,
                                            sig,
                                            qdev_get_gpio_in_named(dev,
                                                ESP32P4_I2S_SIG_IN, l));
            }
        }
    }
}

/*
 * The RMT, its registers and RAM behind RMT_SYS_CLK and held by RST_EN_RMT,
 * on RMT_CLK, its DMA channels on GDMA-AHB; and the PCNT, behind
 * PCNT_APB_CLK, held by RST_EN_PCNT. Their pad signals go through the GPIO
 * matrix. The RX channels' FIFO registers, whose reads move the RMT's
 * pointers, are volatile while the block is gated.
 */
/* [spec:nuos:req:emu.esp32p4.rmt-pcnt] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
static void esp32p4_init_rmt_pcnt(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst), *gpio = DEVICE(&s->gpio);
    DeviceState *dev;
    ESP32P4ClkRstGate *g;
    int n, u;

    object_initialize_child(OBJECT(s), "rmt", &s->rmt, TYPE_ESP32P4_RMT);
    dev = DEVICE(&s->rmt);
    object_property_set_link(OBJECT(dev), "gdma", OBJECT(&s->gdma[0]),
                             &error_abort);
    qdev_connect_clock_in(dev, "sclk", qdev_get_clock_out(clkrst, "rmt"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 ESP32P4_RMT_BASE, ESP32P4_CLK_RMT_SYS,
                                 ESP32P4_RST_RMT, 0);
    esp32p4_clkrst_gate_volatile(g, 0x10, 4 * ESP32P4_RMT_RX_CHANNELS);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_RMT));
    for (n = 0; n < ESP32P4_RMT_TX_CHANNELS; n++) {
        qdev_connect_gpio_out_named(dev, ESP32P4_RMT_SIG_OUT, n,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
                                   ESP32P4_SIG_RMT_SIG_PAD_OUT0 + n));
    }
    for (n = 0; n < ESP32P4_RMT_RX_CHANNELS; n++) {
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
            ESP32P4_SIG_RMT_SIG_PAD_IN0 + n,
            qdev_get_gpio_in_named(dev, ESP32P4_RMT_SIG_IN, n));
    }

    object_initialize_child(OBJECT(s), "pcnt", &s->pcnt, TYPE_ESP32P4_PCNT);
    dev = DEVICE(&s->pcnt);
    qdev_connect_clock_in(dev, "apb", qdev_get_clock_out(clkrst, "pcnt-apb"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_PCNT_BASE, ESP32P4_CLK_PCNT_APB,
                             ESP32P4_RST_PCNT, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_PCNT));
    for (u = 0; u < ESP32P4_PCNT_UNITS; u++) {
        static const int sig[ESP32P4_PCNT_UNIT_INPUTS] = {
            [ESP32P4_PCNT_SIG(0)] = ESP32P4_SIG_PCNT_SIG_CH0_PAD_IN0,
            [ESP32P4_PCNT_SIG(1)] = ESP32P4_SIG_PCNT_SIG_CH1_PAD_IN0,
            [ESP32P4_PCNT_CTRL(0)] = ESP32P4_SIG_PCNT_CTRL_CH0_PAD_IN0,
            [ESP32P4_PCNT_CTRL(1)] = ESP32P4_SIG_PCNT_CTRL_CH1_PAD_IN0,
            [ESP32P4_PCNT_RST] = ESP32P4_SIG_PCNT_RST_PAD_IN0,
        };

        for (n = 0; n < ESP32P4_PCNT_UNIT_INPUTS; n++) {
            qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, sig[n] + u,
                qdev_get_gpio_in_named(dev, ESP32P4_PCNT_INPUT,
                                       u * ESP32P4_PCNT_UNIT_INPUTS + n));
        }
    }
}

/*
 * The event task matrix (TRM chapter 14), its registers behind
 * ETM_APB_CLK, running on ETM_SYS_CLK, held by RST_EN_ETM; and every
 * modelled block's events and tasks, by their IDs, as its header
 * (hw/misc/esp32p4_etm.h) describes: a block that generates events or
 * takes tasks adds a line here for each.
 *
 * The RMT and the PCNT have no events or tasks on the P4.
 *
 * Not connected, as their blocks are not modelled: the ADC, REGDMA,
 * temperature sensor and 2D-DMA events and tasks; and the RTC timer's
 * tasks (RTC_TASK_*), which the TRM says it does not take.
 */
/* [spec:nuos:req:emu.esp32p4.etm] */
static void esp32p4_init_etm(ESP32P4MachineState *s)
{
    ESP32P4EtmState *etm = &s->etm;
    DeviceState *dev, *clkrst = DEVICE(&s->clkrst);
    int ch, g, x, k;

    object_initialize_child(OBJECT(s), "etm", etm, TYPE_ESP32P4_ETM);
    dev = DEVICE(etm);
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(clkrst, "etm-sys"));
    qdev_connect_clock_in(dev, "rtc-evt-tick",
                          qdev_get_clock_out(DEVICE(&s->lp_timer),
                                             "etm-tick"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_ETM_BASE, ESP32P4_CLK_ETM_APB,
                             ESP32P4_CLKRST_GATE_NO_RESET, 0);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET, ESP32P4_RST_ETM,
                                qdev_get_gpio_in_named(dev,
                                                       ESP32P4_ETM_RESET, 0));

    /* GPIO_EXT's eight channels and the analog comparators. */
    dev = DEVICE(&s->gpio);
    for (ch = 0; ch < ESP32P4_GPIO_ETM_CHANNELS; ch++) {
        esp32p4_etm_connect_event(etm, ESP32P4_ETM_GPIO_EVT_CH0_RISE_EDGE + ch,
                                  dev, ESP32P4_GPIO_ETM_EVENT,
                                  ESP32P4_GPIO_ETM_LINE(ch,
                                      ESP32P4_GPIO_ETM_RISE));
        esp32p4_etm_connect_event(etm, ESP32P4_ETM_GPIO_EVT_CH0_FALL_EDGE + ch,
                                  dev, ESP32P4_GPIO_ETM_EVENT,
                                  ESP32P4_GPIO_ETM_LINE(ch,
                                      ESP32P4_GPIO_ETM_FALL));
        esp32p4_etm_connect_event(etm, ESP32P4_ETM_GPIO_EVT_CH0_ANY_EDGE + ch,
                                  dev, ESP32P4_GPIO_ETM_EVENT,
                                  ESP32P4_GPIO_ETM_LINE(ch,
                                      ESP32P4_GPIO_ETM_ANY));
        esp32p4_etm_connect_task(etm, ESP32P4_ETM_GPIO_TASK_CH0_SET + ch,
                                 dev, ESP32P4_GPIO_ETM_TASK,
                                 ESP32P4_GPIO_ETM_LINE(ch,
                                     ESP32P4_GPIO_ETM_SET));
        esp32p4_etm_connect_task(etm, ESP32P4_ETM_GPIO_TASK_CH0_CLEAR + ch,
                                 dev, ESP32P4_GPIO_ETM_TASK,
                                 ESP32P4_GPIO_ETM_LINE(ch,
                                     ESP32P4_GPIO_ETM_CLEAR));
        esp32p4_etm_connect_task(etm, ESP32P4_ETM_GPIO_TASK_CH0_TOGGLE + ch,
                                 dev, ESP32P4_GPIO_ETM_TASK,
                                 ESP32P4_GPIO_ETM_LINE(ch,
                                     ESP32P4_GPIO_ETM_TOGGLE));
    }
    for (k = 0; k < ESP32P4_GPIO_ETM_ZERO_DET_LINES; k++) {
        esp32p4_etm_connect_event(etm, ESP32P4_ETM_GPIO_EVT_ZERO_DET_POS0 + k,
                                  dev, ESP32P4_GPIO_ETM_ZERO_DET, k);
    }

    /* The timer groups' timers, each its alarm and five tasks. */
    for (g = 0; g < 2; g++) {
        dev = DEVICE(&s->timg[g]);
        for (x = 0; x < ESP32P4_TIMG_TIMERS; x++) {
            esp32p4_etm_connect_event(etm, ESP32P4_ETM_TG0_EVT_CNT_CMP_TIMER0 +
                                      g * ESP32P4_TIMG_TIMERS + x,
                                      dev, ESP32P4_TIMG_ETM_EVENT, x);
            for (k = 0; k < ESP32P4_TIMG_TASKS; k++) {
                int line = x * ESP32P4_TIMG_TASKS + k;

                esp32p4_etm_connect_task(etm,
                    ESP32P4_ETM_TG0_TASK_CNT_START_TIMER0 +
                    g * ESP32P4_TIMG_TIMERS * ESP32P4_TIMG_TASKS + line,
                    dev, ESP32P4_TIMG_ETM_TASK, line);
            }
        }
    }

    /* The system timer's comparators. */
    for (x = 0; x < ESP32P4_SYSTIMER_COMPS; x++) {
        esp32p4_etm_connect_event(etm, ESP32P4_ETM_SYSTIMER_EVT_CNT_CMP0 + x,
                                  DEVICE(&s->systimer),
                                  ESP32P4_SYSTIMER_ETM_EVENT, x);
    }

    /* The RTC timer; RTC_EVT_TICK is its counting clock, above. */
    esp32p4_etm_connect_event(etm, ESP32P4_ETM_RTC_EVT_CMP,
                              DEVICE(&s->lp_timer), ESP32P4_LP_TIMER_ETM_EVENT,
                              ESP32P4_LP_TIMER_EVT_CMP);
    esp32p4_etm_connect_event(etm, ESP32P4_ETM_RTC_EVT_OVF,
                              DEVICE(&s->lp_timer), ESP32P4_LP_TIMER_ETM_EVENT,
                              ESP32P4_LP_TIMER_EVT_OVF);

    /* GDMA-AHB's and GDMA-AXI's channels. */
    for (g = 0; g < 2; g++) {
        static const int evt[] = {
            ESP32P4_ETM_PDMA_AHB_EVT_IN_DONE_CH0,
            ESP32P4_ETM_PDMA_AXI_EVT_IN_DONE_CH0,
        };
        static const int task[] = {
            ESP32P4_ETM_PDMA_AHB_TASK_IN_START_CH0,
            ESP32P4_ETM_PDMA_AXI_TASK_IN_START_CH0,
        };

        dev = DEVICE(&s->gdma[g]);
        for (k = 0; k < ESP32P4_GDMA_ETM_EVENTS; k++) {
            esp32p4_etm_connect_event(etm, evt[g] + k, dev,
                                      ESP32P4_GDMA_ETM_EVENT, k);
        }
        for (k = 0; k < ESP32P4_GDMA_ETM_TASKS; k++) {
            esp32p4_etm_connect_task(etm, task[g] + k, dev,
                                     ESP32P4_GDMA_ETM_TASK, k);
        }
    }

    /* The LED PWM controller's channels and timers. */
    dev = DEVICE(&s->ledc);
    for (k = 0; k < ESP32P4_LEDC_ETM_EVENTS; k++) {
        esp32p4_etm_connect_event(etm,
                                  ESP32P4_ETM_LEDC_EVT_DUTY_CHNG_END_CH0 + k,
                                  dev, ESP32P4_LEDC_ETM_EVENT, k);
    }
    for (k = 0; k < ESP32P4_LEDC_ETM_TASKS; k++) {
        esp32p4_etm_connect_task(etm,
                                 ESP32P4_ETM_LEDC_TASK_TIMER0_RES_UPDATE + k,
                                 dev, ESP32P4_LEDC_ETM_TASK, k);
    }

    /* MCPWM0 and MCPWM1. */
    for (g = 0; g < ESP32P4_MCPWM_NUM; g++) {
        static const int evt[] = {
            ESP32P4_ETM_MCPWM0_EVT_TIMER0_STOP,
            ESP32P4_ETM_MCPWM1_EVT_TIMER0_STOP,
        };
        static const int task[] = {
            ESP32P4_ETM_MCPWM0_TASK_CMPR0_A_UP,
            ESP32P4_ETM_MCPWM1_TASK_CMPR0_A_UP,
        };

        dev = DEVICE(&s->mcpwm[g]);
        for (k = 0; k < ESP32P4_MCPWM_ETM_EVENTS; k++) {
            esp32p4_etm_connect_event(etm, evt[g] + k, dev,
                                      ESP32_MCPWM_ETM_EVENT, k);
        }
        for (k = 0; k < ESP32P4_MCPWM_ETM_TASKS; k++) {
            esp32p4_etm_connect_task(etm, task[g] + k, dev,
                                     ESP32_MCPWM_ETM_TASK, k);
        }
    }

    /* The PMU's sleep request and its wakeup to HP_ACTIVE. */
    esp32p4_etm_connect_event(etm, ESP32P4_ETM_PMU_EVT_SLEEP_WEEKUP,
                              DEVICE(&s->pmu), ESP32P4_PMU_ETM_EVENT,
                              ESP32P4_PMU_ETM_EVT_SLEEP_WAKEUP);
    esp32p4_etm_connect_task(etm, ESP32P4_ETM_PMU_TASK_SLEEP_REQ,
                             DEVICE(&s->pmu), ESP32P4_PMU_ETM_TASK, 0);

    /*
     * I2S0 to I2S2: RX_DONE, TX_DONE, X_WORDS_RECEIVED and X_WORDS_SENT,
     * and START_RX, START_TX, STOP_RX and STOP_TX, each.
     */
    for (g = 0; g < ESP32P4_I2S_HP_COUNT; g++) {
        dev = DEVICE(&s->i2s[g]);
        for (k = 0; k < ESP32P4_I2S_ETM_EVENTS; k++) {
            esp32p4_etm_connect_event(etm, ESP32P4_ETM_I2S0_EVT_RX_DONE +
                                      g * ESP32P4_I2S_ETM_EVENTS + k,
                                      dev, ESP32P4_I2S_ETM_EVENT, k);
        }
        for (k = 0; k < ESP32P4_I2S_ETM_TASKS; k++) {
            esp32p4_etm_connect_task(etm, ESP32P4_ETM_I2S0_TASK_START_RX +
                                     g * ESP32P4_I2S_ETM_TASKS + k,
                                     dev, ESP32P4_I2S_ETM_TASK, k);
        }
    }

    /*
     * The LP core's interrupt, through LP_SYS's ETM_TASK_ULP_INT, and its
     * wakeup, which LP_SYS's LP_CORE_ETM_WAKEUP_FLAG records for the PMU;
     * its exception, and the PMU's start and halt of it.
     */
    esp32p4_etm_connect_event(etm, ESP32P4_ETM_ULP_EVT_ERR_INTR,
                              DEVICE(&s->lp_cpu), ESP32P4_CPU_ETM_EVENT, 0);
    esp32p4_etm_connect_event(etm, ESP32P4_ETM_ULP_EVT_START_INTR,
                              DEVICE(&s->pmu), ESP32P4_PMU_ETM_EVENT,
                              ESP32P4_PMU_ETM_EVT_LP_CPU_START);
    esp32p4_etm_connect_event(etm, ESP32P4_ETM_ULP_EVT_HALT,
                              DEVICE(&s->pmu), ESP32P4_PMU_ETM_EVENT,
                              ESP32P4_PMU_ETM_EVT_LP_CPU_HALT);
    esp32p4_etm_connect_task(etm, ESP32P4_ETM_ULP_TASK_INT_CPU,
                             DEVICE(&s->lp_sys),
                             ESP32P4_LP_SYS_ETM_TASK_ULP_INT, 0);
    esp32p4_etm_connect_task(etm, ESP32P4_ETM_ULP_TASK_WAKEUP_CPU,
                             DEVICE(&s->lp_sys),
                             ESP32P4_LP_SYS_LP_CORE_ETM_WAKEUP, 0);
}

/*
 * The full-speed OTG's UTMI+ OTG signals through the GPIO matrix (TRM
 * table 10.12-1, signals 110 to 115), as ESP-IDF's usb_phy.c routes them:
 * the session end through USB_WRAP, which can override it.
 */
static const struct {
    int sig;
    int line;
} esp32p4_usb_fs_sense[] = {
    { ESP32P4_SIG_USB_OTG11_IDDIG_PAD_IN, DWC2_OTG_SENSE_IDDIG },
    { ESP32P4_SIG_USB_OTG11_AVALID_PAD_IN, DWC2_OTG_SENSE_AVALID },
    { ESP32P4_SIG_USB_SRP_BVALID_PAD_IN, DWC2_OTG_SENSE_BVALID },
    { ESP32P4_SIG_USB_OTG11_VBUSVALID_PAD_IN, DWC2_OTG_SENSE_VBUSVALID },
}, esp32p4_usb_fs_ctl[] = {
    { ESP32P4_SIG_USB_OTG11_IDPULLUP_PAD_OUT, DWC2_OTG_CTL_IDPULLUP },
    { ESP32P4_SIG_USB_OTG11_DPPULLDOWN_PAD_OUT, DWC2_OTG_CTL_DPPULLDOWN },
    { ESP32P4_SIG_USB_OTG11_DMPULLDOWN_PAD_OUT, DWC2_OTG_CTL_DMPULLDOWN },
    { ESP32P4_SIG_USB_OTG11_DRVVBUS_PAD_OUT, DWC2_OTG_CTL_DRVVBUS },
    { ESP32P4_SIG_USB_SRP_CHRGVBUS_PAD_OUT, DWC2_OTG_CTL_CHRGVBUS },
    { ESP32P4_SIG_USB_SRP_DISCHRGVBUS_PAD_OUT, DWC2_OTG_CTL_DISCHRGVBUS },
};

/* Map a DWC2 core's registers and FIFOs behind its bus clock and reset. */
static void esp32p4_map_usb_core(ESP32P4MachineState *s, DWC2OTGState *core,
                                 hwaddr base, unsigned clock, unsigned reset)
{
    ESP32P4ClkRstGate *g;

    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(core),
                                 DWC2_OTG_MMIO_CSR, base, clock, reset, 0);
    /* GRXSTSP pops the receive FIFO. */
    esp32p4_clkrst_gate_volatile(g, DWC2_OTG_GRXSTSP, 4);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(core),
                                 DWC2_OTG_MMIO_FIFO,
                                 base + DWC2_OTG_FIFO_BASE, clock, reset, 0);
    /* Reading a push/pop window pops the receive FIFO. */
    esp32p4_clkrst_gate_volatile(g, 0, DWC2_OTG_MAX_EPS * 0x1000);
}

/*
 * The USB 2.0 OTG controllers. OTG_HS: its registers and the UTMI+ PHY's
 * behind USB_OTG20_SYS_CLK, the core reset by RST_EN_USB_OTG20 and the
 * PHY by RST_EN_USB_OTG20_PHY, the PHY's PLL on USB_OTG20_PHYREF_CLK; its
 * PHY controls in HP_SYSTEM; its interrupts USB_OTG_INTR (93) and
 * USB_OTG_ENDP_MULTI_PROC_INTR (94). OTG_FS: its registers and USB_WRAP's
 * behind USB_OTG11_SYS_CLK, both reset by RST_EN_USB_OTG11, its PHY on
 * USB_OTG11_48M_CLK; its PHY selection in LP_SYS and the eFuses; its
 * interrupt USB_OTG11_INTR (103). Both DMA through the DMA APM.
 */
/* [spec:nuos:req:emu.esp32p4.usb] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
/* [spec:nuos:req:emu.esp32p4.gpio] */
static void esp32p4_init_usb(ESP32P4MachineState *s)
{
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *gpio = DEVICE(&s->gpio);
    DeviceState *hs, *fs, *utmi, *wrap;
    int i;

    object_initialize_child(OBJECT(s), "usb-utmi", &s->usb_utmi,
                            TYPE_ESP32P4_USB_UTMI);
    utmi = DEVICE(&s->usb_utmi);
    qdev_connect_clock_in(utmi, "refclk",
                          qdev_get_clock_out(clkrst, "usb-otg20-phyref"));
    sysbus_realize(SYS_BUS_DEVICE(utmi), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(utmi), 0,
                             ESP32P4_USB_UTMI_BASE, ESP32P4_CLK_USB_OTG20_SYS,
                             ESP32P4_RST_USB_OTG20_PHY, 0);
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                ESP32P4_RST_USB_OTG20_PHY,
                                qdev_get_gpio_in_named(utmi,
                                    ESP32P4_USB_UTMI_PHY_RESET, 0));
    for (i = 0; i < ESP32P4_HP_SYS_USBOTG20_CTRL_LINES; i++) {
        qdev_connect_gpio_out_named(DEVICE(&s->hp_sys),
                                    ESP32P4_HP_SYS_USBOTG20_CTRL, i,
                                    qdev_get_gpio_in_named(utmi,
                                        ESP32P4_USB_UTMI_SYS_CTRL, i));
    }
    for (i = 0; i < 4; i++) {
        qdev_connect_gpio_out_named(utmi, ESP32P4_USB_UTMI_PHY_ST, i,
                                    qdev_get_gpio_in_named(
                                        DEVICE(&s->hp_sys),
                                        ESP32P4_HP_SYS_USB_OTGHS_PHY_ST, i));
    }

    object_initialize_child(OBJECT(s), "usb-otg-hs", &s->usb_hs,
                            TYPE_ESP32P4_USB_OTG_HS);
    hs = DEVICE(&s->usb_hs);
    dwc2_otg_set_dma(&s->usb_hs, esp32p4_pms_bus_as(&s->pms),
                     esp32p4_dma_attrs(ESP32P4_DMA_MASTER_USB_OTG_HS,
                                       ESP32P4_ICM_ID_USB_OTG_HS));
    qdev_connect_clock_in(hs, "utmi", qdev_get_clock_out(utmi, "utmi"));
    sysbus_realize(SYS_BUS_DEVICE(hs), &error_fatal);
    esp32p4_map_usb_core(s, &s->usb_hs, ESP32P4_USB_OTG_HS_BASE,
                         ESP32P4_CLK_USB_OTG20_SYS, ESP32P4_RST_USB_OTG20);
    sysbus_connect_irq(SYS_BUS_DEVICE(hs), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_USB_OTG));
    sysbus_connect_irq(SYS_BUS_DEVICE(hs), 1,
                       esp32p4_intr_source(s,
                           ESP32P4_INTR_USB_OTG_ENDP_MULTI_PROC));
    for (i = 0; i < DWC2_OTG_SENSE_NUM; i++) {
        qdev_connect_gpio_out_named(utmi, DWC2_OTG_SENSE, i,
                                    qdev_get_gpio_in_named(hs,
                                        DWC2_OTG_SENSE, i));
    }
    qdev_connect_gpio_out_named(hs, DWC2_OTG_CTL, DWC2_OTG_CTL_DRVVBUS,
                                qdev_get_gpio_in_named(utmi,
                                    ESP32P4_USB_UTMI_DRVVBUS, 0));
    qdev_connect_gpio_out_named(hs, DWC2_OTG_HOST_ATTACHED, 0,
                                qdev_get_gpio_in_named(utmi,
                                    ESP32P4_USB_UTMI_HOST_ATTACHED, 0));

    object_initialize_child(OBJECT(s), "usb-wrap", &s->usb_wrap,
                            TYPE_ESP32P4_USB_WRAP);
    wrap = DEVICE(&s->usb_wrap);
    object_property_set_link(OBJECT(wrap), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(wrap), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(wrap), 0,
                             ESP32P4_USB_WRAP_BASE, ESP32P4_CLK_USB_OTG11_SYS,
                             ESP32P4_RST_USB_OTG11, 0);
    for (i = 0; i < 2; i++) {
        qdev_connect_gpio_out_named(DEVICE(&s->lp_sys),
                                    ESP32P4_LP_SYS_USB_PHY_SEL, i,
                                    qdev_get_gpio_in_named(wrap,
                                        ESP32P4_USB_WRAP_PHY_SEL, i));
    }

    object_initialize_child(OBJECT(s), "usb-otg-fs", &s->usb_fs,
                            TYPE_ESP32P4_USB_OTG_FS);
    fs = DEVICE(&s->usb_fs);
    dwc2_otg_set_dma(&s->usb_fs, esp32p4_pms_bus_as(&s->pms),
                     esp32p4_dma_attrs(ESP32P4_DMA_MASTER_USB_OTG_FS,
                                       ESP32P4_ICM_ID_USB_OTG_FS));
    qdev_connect_clock_in(fs, "fsphy",
                          qdev_get_clock_out(clkrst, "usb-otg11-48m"));
    sysbus_realize(SYS_BUS_DEVICE(fs), &error_fatal);
    esp32p4_map_usb_core(s, &s->usb_fs, ESP32P4_USB_OTG_FS_BASE,
                         ESP32P4_CLK_USB_OTG11_SYS, ESP32P4_RST_USB_OTG11);
    sysbus_connect_irq(SYS_BUS_DEVICE(fs), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_USB_OTG11_CH0));
    qdev_connect_gpio_out_named(wrap, DWC2_OTG_PHY_CONNECTED, 0,
                                qdev_get_gpio_in_named(fs,
                                    DWC2_OTG_PHY_CONNECTED, 0));
    for (i = 0; i < 2; i++) {
        qdev_connect_gpio_out_named(wrap, DWC2_OTG_PULL_OVERRIDE, i,
                                    qdev_get_gpio_in_named(fs,
                                        DWC2_OTG_PULL_OVERRIDE, i));
    }
    for (i = 0; i < ARRAY_SIZE(esp32p4_usb_fs_sense); i++) {
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                    esp32p4_usb_fs_sense[i].sig,
                                    qdev_get_gpio_in_named(fs,
                                        DWC2_OTG_SENSE,
                                        esp32p4_usb_fs_sense[i].line));
    }
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
                                ESP32P4_SIG_USB_SRP_SESSEND_PAD_IN,
                                qdev_get_gpio_in_named(wrap,
                                    ESP32P4_USB_WRAP_SESSEND_IN, 0));
    qdev_connect_gpio_out_named(wrap, ESP32P4_USB_WRAP_SESSEND, 0,
                                qdev_get_gpio_in_named(fs, DWC2_OTG_SENSE,
                                    DWC2_OTG_SENSE_SESSEND));
    /*
     * The D+ pull-up OTG_FS's PHY puts on its pad: GPIO27, of the PHY the
     * OTG has by default. GPIO25's pull-up is USB Serial/JTAG's line.
     */
    qdev_connect_gpio_out_named(fs, DWC2_OTG_DP_PULLUP, 0,
                                qdev_get_gpio_in_named(wrap,
                                    ESP32P4_USB_WRAP_CORE_DP_PULLUP, 0));
    qdev_connect_gpio_out_named(wrap, ESP32P4_USB_WRAP_PHY_DP_PULLUP, 1,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_PAD_PULL_UP, 27));
    for (i = 0; i < ARRAY_SIZE(esp32p4_usb_fs_ctl); i++) {
        qdev_connect_gpio_out_named(fs, DWC2_OTG_CTL,
                                    esp32p4_usb_fs_ctl[i].line,
                                    qdev_get_gpio_in_named(gpio,
                                        ESP32P4_GPIO_SIG_OUT,
                                        esp32p4_usb_fs_ctl[i].sig));
    }
}

/*
 * The UARTs and the board's serial ports on their pads (see the head
 * comment): each UART's registers behind its bus clock and APB reset,
 * FIFO_REG's read popping the RX FIFO and reading 0 while gated; its core
 * on its function clock and core reset; its lines through the GPIO
 * matrix. Their wakeup logic is armed by the PMU's UART_WAKEUP_EN, and
 * their wakeups are its wakeup sources UART0 to UART4 and LP_UART; the
 * LP UART's also sets LPPERI's LP_UART_WAKEUP_FLAG. Then UHCI0, between
 * the UARTs and GDMA-AHB.
 */
/* [spec:nuos:req:emu.esp32p4.uart] */
static void esp32p4_init_uart(ESP32P4MachineState *s)
{
    static const struct {
        int txd, rts, dtr, rxd, cts, dsr;
        int tx_pad, rx_pad;
    } lines[ESP32P4_UART_COUNT] = {
        { ESP32P4_SIG_UART0_TXD_PAD_OUT, ESP32P4_SIG_UART0_RTS_PAD_OUT,
          ESP32P4_SIG_UART0_DTR_PAD_OUT, ESP32P4_SIG_UART0_RXD_PAD_IN,
          ESP32P4_SIG_UART0_CTS_PAD_IN, ESP32P4_SIG_UART0_DSR_PAD_IN,
          37, 38 },
        { ESP32P4_SIG_UART1_TXD_PAD_OUT, ESP32P4_SIG_UART1_RTS_PAD_OUT,
          ESP32P4_SIG_UART1_DTR_PAD_OUT, ESP32P4_SIG_UART1_RXD_PAD_IN,
          ESP32P4_SIG_UART1_CTS_PAD_IN, ESP32P4_SIG_UART1_DSR_PAD_IN,
          10, 11 },
        { ESP32P4_SIG_UART2_TXD_PAD_OUT, ESP32P4_SIG_UART2_RTS_PAD_OUT,
          ESP32P4_SIG_UART2_DTR_PAD_OUT, ESP32P4_SIG_UART2_RXD_PAD_IN,
          ESP32P4_SIG_UART2_CTS_PAD_IN, ESP32P4_SIG_UART2_DSR_PAD_IN,
          16, 17 },
        { ESP32P4_SIG_UART3_TXD_PAD_OUT, ESP32P4_SIG_UART3_RTS_PAD_OUT,
          ESP32P4_SIG_UART3_DTR_PAD_OUT, ESP32P4_SIG_UART3_RXD_PAD_IN,
          ESP32P4_SIG_UART3_CTS_PAD_IN, ESP32P4_SIG_UART3_DSR_PAD_IN,
          18, 19 },
        { ESP32P4_SIG_UART4_TXD_PAD_OUT, ESP32P4_SIG_UART4_RTS_PAD_OUT,
          ESP32P4_SIG_UART4_DTR_PAD_OUT, ESP32P4_SIG_UART4_RXD_PAD_IN,
          ESP32P4_SIG_UART4_CTS_PAD_IN, ESP32P4_SIG_UART4_DSR_PAD_IN,
          20, 21 },
    };
    DeviceState *clkrst = DEVICE(&s->clkrst), *gpio = DEVICE(&s->gpio);
    DeviceState *pmu = DEVICE(&s->pmu), *wake_en, *lp_wake;
    DeviceState *dev, *term;
    ESP32P4ClkRstGate *g;
    int i;

    object_initialize_child(OBJECT(s), "uart-wakeup-en-split",
                            &s->uart_wakeup_en_split, TYPE_SPLIT_IRQ);
    wake_en = DEVICE(&s->uart_wakeup_en_split);
    qdev_prop_set_uint16(wake_en, "num-lines", ESP32P4_UART_COUNT + 1);
    qdev_realize(wake_en, NULL, &error_fatal);
    qdev_connect_gpio_out_named(pmu, ESP32P4_PMU_UART_WAKEUP_EN, 0,
                                qdev_get_gpio_in(wake_en, 0));

    for (i = 0; i < ESP32P4_UART_COUNT; i++) {
        g_autofree char *sclk = g_strdup_printf("uart%d-sclk", i);
        g_autofree char *sys = g_strdup_printf("uart%d-sys", i);
        g_autofree char *name = g_strdup_printf("esp32p4.uart%d", i);

        object_initialize_child(OBJECT(s), "uart[*]", &s->uart[i],
                                TYPE_ESP32P4_UART);
        dev = DEVICE(&s->uart[i]);
        qdev_prop_set_string(dev, "name", name);
        qdev_connect_clock_in(dev, "sclk", qdev_get_clock_out(clkrst, sclk));
        qdev_connect_clock_in(dev, "sys", qdev_get_clock_out(clkrst, sys));
        sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
        g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                     ESP32P4_UART0_BASE +
                                     i * ESP32P4_UART_STRIDE,
                                     ESP32P4_CLK_UART0_APB + i,
                                     ESP32P4_RST_UART0_APB + i, 0);
        esp32p4_clkrst_gate_volatile(g, 0, 4);
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                    ESP32P4_RST_UART0_CORE + i,
                                    qdev_get_gpio_in_named(dev,
                                        ESP32P4_UART_CORE_RESET, 0));
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           esp32p4_intr_source(s, ESP32P4_INTR_UART0 + i));
        qdev_connect_gpio_out(wake_en, i,
                              qdev_get_gpio_in_named(dev,
                                  ESP32P4_UART_WAKEUP_EN, 0));
        qdev_connect_gpio_out_named(dev, ESP32P4_UART_WAKEUP, 0,
                                    qdev_get_gpio_in_named(pmu,
                                        ESP32P4_PMU_WAKEUP,
                                        ESP32P4_PMU_WAKE_UART0 - i));

        qdev_connect_gpio_out_named(dev, ESP32P4_UART_TXD, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT, lines[i].txd));
        qdev_connect_gpio_out_named(dev, ESP32P4_UART_RTS, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT, lines[i].rts));
        qdev_connect_gpio_out_named(dev, ESP32P4_UART_DTR, 0,
            qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT, lines[i].dtr));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, lines[i].rxd,
            qdev_get_gpio_in_named(dev, ESP32P4_UART_RXD, 0));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, lines[i].cts,
            qdev_get_gpio_in_named(dev, ESP32P4_UART_CTS, 0));
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN, lines[i].dsr,
            qdev_get_gpio_in_named(dev, ESP32P4_UART_DSR, 0));
    }

    object_initialize_child(OBJECT(s), "lp-uart", &s->lp_uart,
                            TYPE_ESP32P4_UART);
    dev = DEVICE(&s->lp_uart);
    qdev_prop_set_bit(dev, "lp", true);
    qdev_prop_set_string(dev, "name", "esp32p4.lp-uart");
    qdev_connect_clock_in(dev, "sclk",
                          qdev_get_clock_out(clkrst, "lp-uart-sclk"));
    qdev_connect_clock_in(dev, "sys", qdev_get_clock_out(clkrst, "lp-uart"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    g = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                                 ESP32P4_LP_UART_BASE, ESP32P4_CLK_LP_UART,
                                 ESP32P4_RST_LP_UART, 0);
    esp32p4_clkrst_gate_volatile(g, 0, 4);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_UART));
    qdev_connect_gpio_out(wake_en, ESP32P4_UART_COUNT,
                          qdev_get_gpio_in_named(dev,
                              ESP32P4_UART_WAKEUP_EN, 0));
    object_initialize_child(OBJECT(s), "lp-uart-wakeup-split",
                            &s->lp_uart_wakeup_split, TYPE_SPLIT_IRQ);
    lp_wake = DEVICE(&s->lp_uart_wakeup_split);
    qdev_prop_set_uint16(lp_wake, "num-lines", 2);
    qdev_realize(lp_wake, NULL, &error_fatal);
    qdev_connect_gpio_out_named(dev, ESP32P4_UART_WAKEUP, 0,
                                qdev_get_gpio_in(lp_wake, 0));
    qdev_connect_gpio_out(lp_wake, 0,
                          qdev_get_gpio_in_named(clkrst,
                              ESP32P4_CLKRST_LP_UART_WAKEUP, 0));
    qdev_connect_gpio_out(lp_wake, 1,
                          qdev_get_gpio_in_named(pmu, ESP32P4_PMU_WAKEUP,
                                                 ESP32P4_PMU_WAKE_LP_UART));
    qdev_connect_gpio_out_named(dev, ESP32P4_UART_TXD, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OUT,
                               ESP32P4_LP_SIG_UART_TXD_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_UART_RTS, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OUT,
                               ESP32P4_LP_SIG_UART_RTSN_PAD_OUT));
    qdev_connect_gpio_out_named(dev, ESP32P4_UART_DTR, 0,
        qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_LP_SIG_OUT,
                               ESP32P4_LP_SIG_UART_DTRN_PAD_OUT));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_LP_SIG_IN,
                                ESP32P4_LP_SIG_UART_RXD_PAD_IN,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_UART_RXD, 0));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_LP_SIG_IN,
                                ESP32P4_LP_SIG_UART_CTSN_PAD_IN,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_UART_CTS, 0));
    qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_LP_SIG_IN,
                                ESP32P4_LP_SIG_UART_DSRN_PAD_IN,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_UART_DSR, 0));

    for (i = 0; i < ESP32P4_SERIAL_PORTS; i++) {
        bool lp = i == ESP32P4_UART_COUNT;
        int tx_pad = lp ? 14 : lines[i].tx_pad;
        int rx_pad = lp ? 15 : lines[i].rx_pad;

        object_initialize_child(OBJECT(s), "terminal[*]", &s->terminal[i],
                                TYPE_ESP32_UART_TERMINAL);
        term = DEVICE(&s->terminal[i]);
        qdev_prop_set_chr(term, "chardev", serial_hd(i));
        object_property_set_link(OBJECT(term), "uart",
                                 OBJECT(lp ? &s->lp_uart : &s->uart[i]),
                                 &error_abort);
        qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_PAD_OUT, tx_pad,
                                    qdev_get_gpio_in_named(term,
                                        ESP32_UART_TERMINAL_RX, 0));
        qdev_connect_gpio_out_named(term, ESP32_UART_TERMINAL_TX, 0,
                                    qdev_get_gpio_in_named(gpio,
                                        ESP32P4_GPIO_PAD_IN, rx_pad));
        qdev_realize(term, NULL, &error_fatal);
    }

    object_initialize_child(OBJECT(s), "uhci", &s->uhci, TYPE_ESP32P4_UHCI);
    dev = DEVICE(&s->uhci);
    object_property_set_link(OBJECT(dev), "gdma", OBJECT(&s->gdma[0]),
                             &error_abort);
    for (i = 0; i < ESP32P4_UART_COUNT; i++) {
        g_autofree char *link = g_strdup_printf("uart%d", i);

        object_property_set_link(OBJECT(dev), link, OBJECT(&s->uart[i]),
                                 &error_abort);
    }
    qdev_connect_clock_in(dev, "clk", qdev_get_clock_out(clkrst, "uhci-sys"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_UHCI_BASE, ESP32P4_CLK_UHCI_APB,
                             ESP32P4_RST_UHCI, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_UHCI0));
}

/*
 * The USB Serial/JTAG controller: its registers behind
 * USB_DEVICE_APB_CLK and its reset, its USB side on USB_DEVICE_48M_CLK,
 * its D- and D+ pull-ups on GPIO24 and GPIO25.
 */
/* [spec:nuos:req:emu.esp32p4.uart] */
static void esp32p4_init_usb_serial_jtag(ESP32P4MachineState *s)
{
    DeviceState *dev, *gpio = DEVICE(&s->gpio);

    object_initialize_child(OBJECT(s), "usb-serial-jtag", &s->usj,
                            TYPE_ESP32P4_USB_SERIAL_JTAG);
    dev = DEVICE(&s->usj);
    qdev_connect_clock_in(dev, "usb48",
                          qdev_get_clock_out(DEVICE(&s->clkrst),
                                             "usb-device-48m"));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0,
                             ESP32P4_USJ_BASE, ESP32P4_CLK_USB_DEVICE_APB,
                             ESP32P4_RST_USB_DEVICE, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_USB_SERIAL_JTAG));
    qdev_connect_gpio_out_named(dev, ESP32P4_USJ_PAD_PULL_UP, 0,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_PAD_PULL_UP, 24));
    qdev_connect_gpio_out_named(dev, ESP32P4_USJ_PAD_PULL_UP, 1,
                                qdev_get_gpio_in_named(gpio,
                                    ESP32P4_GPIO_PAD_PULL_UP, 25));
}

/*
 * A cryptography block: its clock, its registers behind @bus_clock, and
 * its interrupt (or none for -1).
 */
static void esp32p4_crypto_block(ESP32P4MachineState *s, DeviceState *dev,
                                 const char *clock, hwaddr base,
                                 unsigned bus_clock, int intr)
{
    qdev_connect_clock_in(dev, "clk",
                          qdev_get_clock_out(DEVICE(&s->clkrst), clock));
    sysbus_realize(SYS_BUS_DEVICE(dev), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(dev), 0, base,
                             bus_clock, ESP32P4_CLKRST_GATE_NO_RESET, 0);
    if (intr >= 0) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           esp32p4_intr_source(s, intr));
    }
}

/*
 * The cryptography blocks. A block's reset also holds the blocks it reuses
 * (TRM 27.3, 28.5, 30.3, 31.3): RST_EN_DS the AES, SHA and RSA
 * accelerators, RST_EN_ECDSA the SHA, RSA and ECC accelerators, RST_EN_KM
 * the AES accelerator and RST_EN_HMAC the SHA accelerator; RST_EN_CRYPTO
 * holds them all (esp32p4_clkrst.c).
 */
/* [spec:nuos:req:emu.esp32p4.crypto] */
/* [spec:nuos:req:emu.esp32p4.clocks] */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_init_crypto(ESP32P4MachineState *s)
{
    static const struct {
        unsigned reset;
        int lines;
    } shared[ESP32P4_CRYPTO_RST_SPLITS] = {
        { ESP32P4_RST_DS, 4 },
        { ESP32P4_RST_ECDSA, 4 },
        { ESP32P4_RST_KM, 2 },
        { ESP32P4_RST_HMAC, 2 },
    };
    DeviceState *clkrst = DEVICE(&s->clkrst);
    DeviceState *aes, *sha, *rsa, *ecc, *hmac, *ds, *ecdsa, *km, *huk, *trng;
    DeviceState *split[ESP32P4_CRYPTO_RST_SPLITS];
    ESP32P4ClkRstGate *gate;
    Object *gdma_axi = OBJECT(&s->gdma[1]);
    int i;

    object_initialize_child(OBJECT(s), "trng", &s->trng, TYPE_ESP32P4_TRNG);
    object_initialize_child(OBJECT(s), "huk", &s->huk, TYPE_ESP32P4_HUK);
    object_initialize_child(OBJECT(s), "keymng", &s->keymng,
                            TYPE_ESP32P4_KEYMNG);
    object_initialize_child(OBJECT(s), "aes", &s->aes, TYPE_ESP32P4_AES);
    object_initialize_child(OBJECT(s), "sha", &s->sha, TYPE_ESP32P4_SHA);
    object_initialize_child(OBJECT(s), "rsa", &s->rsa, TYPE_ESP32P4_RSA);
    object_initialize_child(OBJECT(s), "ecc", &s->ecc, TYPE_ESP32P4_ECC);
    object_initialize_child(OBJECT(s), "hmac", &s->hmac, TYPE_ESP32P4_HMAC);
    object_initialize_child(OBJECT(s), "ds", &s->ds, TYPE_ESP32P4_DS);
    object_initialize_child(OBJECT(s), "ecdsa", &s->ecdsa,
                            TYPE_ESP32P4_ECDSA);
    trng = DEVICE(&s->trng);
    huk = DEVICE(&s->huk);
    km = DEVICE(&s->keymng);
    aes = DEVICE(&s->aes);
    sha = DEVICE(&s->sha);
    rsa = DEVICE(&s->rsa);
    ecc = DEVICE(&s->ecc);
    hmac = DEVICE(&s->hmac);
    ds = DEVICE(&s->ds);
    ecdsa = DEVICE(&s->ecdsa);

    /* The LP blocks, in the always-on domain and LPPERI's. */
    qdev_connect_clock_in(trng, "clk", qdev_get_clock_out(clkrst, "lp-rng"));
    sysbus_realize(SYS_BUS_DEVICE(trng), &error_fatal);
    gate = esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(trng), 0,
                                    ESP32P4_TRNG_BASE, ESP32P4_CLK_LP_RNG,
                                    ESP32P4_RST_LP_RNG, 0);
    esp32p4_clkrst_gate_volatile(gate, ESP32P4_TRNG_DATA, 4);

    object_property_set_link(OBJECT(huk), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    qdev_connect_clock_in(huk, "clk",
                          qdev_get_clock_out(clkrst, "lp-dyn-fast"));
    sysbus_realize(SYS_BUS_DEVICE(huk), &error_fatal);
    esp32p4_clkrst_map_gated(&s->clkrst, SYS_BUS_DEVICE(huk), 0,
                             ESP32P4_HUK_BASE, ESP32P4_CLK_LP_DYN_FAST,
                             ESP32P4_RST_LP_HUK, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(huk), 0,
                       esp32p4_intr_source(s, ESP32P4_INTR_LP_HUK));

    object_property_set_link(OBJECT(km), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    object_property_set_link(OBJECT(km), "huk", OBJECT(huk), &error_abort);
    esp32p4_crypto_block(s, km, "crypto-km", ESP32P4_KEYMNG_BASE,
                         ESP32P4_CLK_KEY_MANAGER_SYS, ESP32P4_INTR_KM);

    object_property_set_link(OBJECT(aes), "gdma", gdma_axi, &error_abort);
    esp32p4_crypto_block(s, aes, "crypto-aes", ESP32P4_AES_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, ESP32P4_INTR_AES);
    object_property_set_link(OBJECT(sha), "gdma", gdma_axi, &error_abort);
    esp32p4_crypto_block(s, sha, "crypto-sha", ESP32P4_SHA_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, ESP32P4_INTR_SHA);
    esp32p4_crypto_block(s, rsa, "crypto-rsa", ESP32P4_RSA_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, ESP32P4_INTR_RSA);
    object_property_set_link(OBJECT(ecc), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    esp32p4_crypto_block(s, ecc, "crypto-ecc", ESP32P4_ECC_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, ESP32P4_INTR_ECC);

    object_property_set_link(OBJECT(hmac), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    object_property_set_link(OBJECT(hmac), "keymng", OBJECT(km),
                             &error_abort);
    object_property_set_link(OBJECT(hmac), "sha", OBJECT(sha), &error_abort);
    esp32p4_crypto_block(s, hmac, "crypto-hmac", ESP32P4_HMAC_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, -1);

    object_property_set_link(OBJECT(ds), "hmac", OBJECT(hmac), &error_abort);
    object_property_set_link(OBJECT(ds), "keymng", OBJECT(km), &error_abort);
    object_property_set_link(OBJECT(ds), "aes", OBJECT(aes), &error_abort);
    object_property_set_link(OBJECT(ds), "sha", OBJECT(sha), &error_abort);
    object_property_set_link(OBJECT(ds), "rsa", OBJECT(rsa), &error_abort);
    esp32p4_crypto_block(s, ds, "crypto-ds", ESP32P4_DS_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, -1);
    qdev_connect_gpio_out_named(hmac, ESP32P4_HMAC_DS_KEY, 0,
                                qdev_get_gpio_in_named(ds,
                                    ESP32P4_DS_HMAC_KEY, 0));

    object_property_set_link(OBJECT(ecdsa), "efuse", OBJECT(&s->efuse),
                             &error_abort);
    object_property_set_link(OBJECT(ecdsa), "keymng", OBJECT(km),
                             &error_abort);
    object_property_set_link(OBJECT(ecdsa), "ecc", OBJECT(ecc),
                             &error_abort);
    object_property_set_link(OBJECT(ecdsa), "sha", OBJECT(sha),
                             &error_abort);
    esp32p4_crypto_block(s, ecdsa, "crypto-ecdsa", ESP32P4_ECDSA_BASE,
                         ESP32P4_CLK_CRYPTO_SYS, ESP32P4_INTR_ECDSA);

    /* The resets. */
    for (i = 0; i < ESP32P4_CRYPTO_RST_SPLITS; i++) {
        object_initialize_child(OBJECT(s), "crypto-reset-split[*]",
                                &s->crypto_rst_split[i], TYPE_SPLIT_IRQ);
        split[i] = DEVICE(&s->crypto_rst_split[i]);
        qdev_prop_set_uint16(split[i], "num-lines", shared[i].lines);
        qdev_realize(split[i], NULL, &error_fatal);
        qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET,
                                    shared[i].reset,
                                    qdev_get_gpio_in(split[i], 0));
    }
    /* RST_EN_DS: RSA_DS, AES, SHA, RSA. */
    qdev_connect_gpio_out(split[0], 0,
                          qdev_get_gpio_in_named(ds, ESP32P4_DS_RESET, 0));
    qdev_connect_gpio_out(split[0], 1,
                          qdev_get_gpio_in_named(aes, ESP32P4_AES_RESET, 1));
    qdev_connect_gpio_out(split[0], 2,
                          qdev_get_gpio_in_named(sha, ESP32P4_SHA_RESET, 1));
    qdev_connect_gpio_out(split[0], 3,
                          qdev_get_gpio_in_named(rsa, ESP32P4_RSA_RESET, 1));
    /* RST_EN_ECDSA: ECDSA_DS, SHA, RSA, ECC. */
    qdev_connect_gpio_out(split[1], 0,
                          qdev_get_gpio_in_named(ecdsa, ESP32P4_ECDSA_RESET,
                                                 0));
    qdev_connect_gpio_out(split[1], 1,
                          qdev_get_gpio_in_named(sha, ESP32P4_SHA_RESET, 3));
    qdev_connect_gpio_out(split[1], 2,
                          qdev_get_gpio_in_named(rsa, ESP32P4_RSA_RESET, 2));
    qdev_connect_gpio_out(split[1], 3,
                          qdev_get_gpio_in_named(ecc, ESP32P4_ECC_RESET, 1));
    /* RST_EN_KM: the Key Manager, AES. */
    qdev_connect_gpio_out(split[2], 0,
                          qdev_get_gpio_in_named(km, ESP32P4_KEYMNG_RESET,
                                                 0));
    qdev_connect_gpio_out(split[2], 1,
                          qdev_get_gpio_in_named(aes, ESP32P4_AES_RESET, 2));
    /* RST_EN_HMAC: HMAC, SHA. */
    qdev_connect_gpio_out(split[3], 0,
                          qdev_get_gpio_in_named(hmac, ESP32P4_HMAC_RESET,
                                                 0));
    qdev_connect_gpio_out(split[3], 1,
                          qdev_get_gpio_in_named(sha, ESP32P4_SHA_RESET, 2));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET, ESP32P4_RST_AES,
                                qdev_get_gpio_in_named(aes,
                                    ESP32P4_AES_RESET, 0));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET, ESP32P4_RST_SHA,
                                qdev_get_gpio_in_named(sha,
                                    ESP32P4_SHA_RESET, 0));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET, ESP32P4_RST_RSA,
                                qdev_get_gpio_in_named(rsa,
                                    ESP32P4_RSA_RESET, 0));
    qdev_connect_gpio_out_named(clkrst, ESP32P4_CLKRST_RESET, ESP32P4_RST_ECC,
                                qdev_get_gpio_in_named(ecc,
                                    ESP32P4_ECC_RESET, 0));

    /* HP_SYSTEM's power-down of the RSA and ECC memories. */
    qdev_connect_gpio_out_named(DEVICE(&s->hp_sys), ESP32P4_HP_SYS_MEM_PD, 0,
                                qdev_get_gpio_in_named(rsa,
                                    ESP32P4_RSA_MEM_PD, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->hp_sys), ESP32P4_HP_SYS_MEM_PD, 1,
                                qdev_get_gpio_in_named(ecc,
                                    ESP32P4_ECC_MEM_PD, 0));
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

/* An ELF segment -kernel loaded outside LP SRAM, for its reload. */
typedef struct ESP32P4KernelSegment {
    uint32_t paddr;
    uint32_t memsz;
    GBytes *data;
} ESP32P4KernelSegment;

/*
 * Refuse an ELF with a segment outside those memories: the loader would
 * drop one in the flash or PSRAM windows without a word, as nothing maps
 * them, and one in the ROM would replace the ROM. Files that are not
 * 32-bit little-endian ELFs are left for the loader to reject. Keep the
 * segments outside LP SRAM, which a deep sleep loses.
 */
static void esp32p4_check_kernel(ESP32P4MachineState *s,
                                 const char *filename)
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
        ESP32P4KernelSegment seg;
        uint32_t paddr, memsz, offset, filesz;

        if (phoff + (uint64_t)(i + 1) * phentsize > len) {
            return;
        }
        memcpy(&ph, data + phoff + i * phentsize, sizeof(ph));
        paddr = le32_to_cpu(ph.p_paddr);
        memsz = le32_to_cpu(ph.p_memsz);
        offset = le32_to_cpu(ph.p_offset);
        filesz = MIN(le32_to_cpu(ph.p_filesz), memsz);
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
        if ((paddr >= ESP32P4_LP_SRAM_BASE &&
             paddr - ESP32P4_LP_SRAM_BASE < ESP32P4_LP_SRAM_SIZE) ||
            (uint64_t)offset + filesz > len) {
            continue;
        }
        seg.paddr = paddr;
        seg.memsz = memsz;
        seg.data = g_bytes_new(data + offset, filesz);
        g_array_append_val(s->kernel_segments, seg);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * Load the -kernel ELF's segments outside LP SRAM again, as the ROM's boot
 * from flash would once the HP memories have lost their contents.
 */
static void esp32p4_reload_kernel(ESP32P4MachineState *s)
{
    guint i;

    for (i = 0; s->kernel_segments && i < s->kernel_segments->len; i++) {
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
 * [spec:nuos:req:emu.esp32p4.boot]
 * What the boot ROM does at boot that a program -kernel starts in its
 * place finds done, as a system reset completes. Of the super watchdog,
 * the ROM's main() only clears SWD_RESET_FLAG on core 0
 * (clear_super_wdt_reset_flag()); it neither feeds the watchdog nor sets
 * SWD_AUTO_FEED_EN, nor writes LP_SYSTEM's FIB control, so the watchdog
 * runs on, unfed, as it does here.
 */
static void esp32p4_rom_boot_state(void *opaque)
{
    ESP32P4MachineState *s = opaque;

    esp32p4_lp_wdt_swd_rst_flag_clr(&s->lp_wdt);
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
    s->kernel_segments = g_array_new(false, false,
                                     sizeof(ESP32P4KernelSegment));
    esp32p4_check_kernel(s, machine->kernel_filename);
    if (load_elf(machine->kernel_filename, NULL, NULL, NULL, &entry, NULL,
                 NULL, NULL, ELFDATA2LSB, EM_RISCV, 1, 0) < 0) {
        error_report("esp32p4: cannot load the ELF file '%s'",
                     machine->kernel_filename);
        exit(1);
    }
    s->kernel_entry = entry;
    s->hp_cpu[0].parent_obj.env.resetvec = entry;
    qemu_register_reset(esp32p4_rom_boot_state, s);
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
    esp32p4_init_i2c(s);
    esp32p4_init_pms(s);
    esp32p4_init_dma(s);
    esp32p4_init_crypto(s);
    esp32p4_init_ext_memory(s);
    esp32p4_init_sdmmc(s);
    esp32p4_init_emac(s);
    esp32p4_init_spi(s);
    esp32p4_init_parlio(s);
    esp32p4_init_low_power(s);
    esp32p4_init_lp_core(s);
    esp32p4_init_uart(s);
    esp32p4_init_usb_serial_jtag(s);
    esp32p4_init_i2s(s);
    esp32p4_init_i3c(s);
    esp32p4_init_twai(s);
    esp32p4_init_pwm(s);
    esp32p4_init_rmt_pcnt(s);
    esp32p4_init_etm(s);
    esp32p4_init_usb(s);
    esp32p4_init_debug(s);
    for (i = 0; i < ARRAY_SIZE(esp32p4_unimplemented); i++) {
        esp32p4_map(esp32p4_unimplemented_device(
                        esp32p4_unimplemented[i].name,
                        esp32p4_unimplemented[i].size),
                    esp32p4_unimplemented[i].base);
    }
    qemu_log_mask(LOG_UNIMP, "esp32p4: the LP ROM's contents are not "
                  "published, so it is absent\n");

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

/* [spec:nuos:req:emu.esp32p4.twai] */
static void esp32p4_machine_instance_init(Object *obj)
{
    ESP32P4MachineState *s = ESP32P4_MACHINE(obj);

    s->strap_mode = ESP32P4_GPIO_STRAP_SPI_BOOT;
    /* Early, for the clock tree to take the EMAC's and I2S's pad clocks. */
    object_initialize_child(obj, "emac", &s->emac, TYPE_ESP32P4_EMAC);
    object_initialize_child(obj, "phy", &s->phy, TYPE_IP101_PHY);
    for (int i = 0; i < ESP32P4_I2S_HP_COUNT; i++) {
        g_autofree char *name = g_strdup_printf("i2s%d", i);

        object_initialize_child(obj, name, &s->i2s[i], TYPE_ESP32P4_I2S);
    }
    object_initialize_child(obj, "lp-i2s", &s->i2s[ESP32P4_I2S_LP_ID],
                            TYPE_ESP32P4_LP_I2S);
    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        g_autofree char *name = g_strdup_printf("canbus%d", i);
        g_autofree char *desc = g_strdup_printf("The QEMU CAN bus (-object "
                                                "can-bus) on TWAI%d's pads",
                                                i);

        object_property_add_link(obj, name, TYPE_CAN_BUS,
                                 (Object **)&s->canbus[i],
                                 object_property_allow_set_link, 0);
        object_property_set_description(obj, name, desc);
    }
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
    machine_add_audiodev_property(mc);
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
