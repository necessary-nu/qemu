/*
 * ESP32-P4 HP system registers (HP_SYSTEM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_HP_SYS_H
#define HW_MISC_ESP32P4_HP_SYS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_HP_SYS "esp32p4-hp-sys"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4HPSysState, ESP32P4_HP_SYS)

#define ESP32P4_HP_SYS_BASE 0x500e5000
#define ESP32P4_HP_SYS_SIZE 0x1000

/* The registers, 0x000 to 0x21C, one word each. */
#define ESP32P4_HP_SYS_NREGS (0x220 / 4)

/* The software interrupts CPU_INT_FROM_CPU_0 to _3. */
#define ESP32P4_HP_SYS_CPU_INTRS 4

#define ESP32P4_HP_SYS_CORES 2

/*
 * The HP cores' buses whose error response HP_SYSTEM_CORE_ERR_RESP_DIS
 * disables, its bits in this order.
 */
enum {
    ESP32P4_BUS_IBUS,
    ESP32P4_BUS_DBUS,
    ESP32P4_BUS_AHB,
    ESP32P4_BUSES,
};

/*
 * The peripheral interrupt sources grouped into the LP core's HP_INTR and
 * the PMU's HP wakeup by HP2LP_INTR_GROUP0..3 and HP2LP_WAKEUP_GROUP0..3:
 * sources 21 (SYS_ICM_INTR) to 130 (AXI_PERF_MON_INTR), bit k of the
 * groups being source ESP32P4_HP_SYS_HP2LP_FIRST + k.
 */
#define ESP32P4_HP_SYS_HP2LP_FIRST   21
#define ESP32P4_HP_SYS_HP2LP_SOURCES 110

/* HP_SYSTEM_GPIO_O_HOLD_CTRL0/1 hold GPIO16 to GPIO56. */
#define ESP32P4_HP_SYS_HOLD_FIRST    16
#define ESP32P4_HP_SYS_HOLDS         41

/*
 * Named GPIO outputs
 * ==================
 *
 * ESP32P4_HP_SYS_CPU_INTR: CPU_INTR_FROM_CPU_0 to _3, interrupt sources
 *   ESP32P4_INTR_FROM_CPU_INTR0 to _3 of the interrupt matrix, each high
 *   while its register's bit is set.
 * ESP32P4_HP_SYS_CRYPTO_CTRL: HP_SYSTEM_CRYPTO_CTRL's four enables,
 *   ENABLE_SPI_MANUAL_ENCRYPT, ENABLE_DOWNLOAD_DB_ENCRYPT,
 *   ENABLE_DOWNLOAD_G0CB_DECRYPT and ENABLE_DOWNLOAD_MANUAL_ENCRYPT, to the
 *   MSPI controllers' "crypto-ctrl" inputs.
 * ESP32P4_HP_SYS_L2MEM_ERR_RESP: HP_SYSTEM_L2_MEM_ERR_RESP_EN, high while
 *   an access to the part of L2MEM the L2 cache occupies is answered with
 *   an error, to the cache controller.
 * ESP32P4_HP_SYS_CACHE_RESET: HP_SYSTEM_CACHE_RESET_CONFIG's
 *   L1_I0_CACHE_RESET, L1_I1_CACHE_RESET and L1_D_CACHE_RESET, in that
 *   order, each high while set, to the cache controller's
 *   "sys-cache-reset" inputs.
 * ESP32P4_HP_SYS_ERR_RESP_DIS: line core * ESP32P4_BUSES + bus,
 *   HP_SYSTEM_CORE_ERR_RESP_DIS's bit for the bus, to each HP core's
 *   ESP32P4_CPU_ERR_RESP_DIS input: high while the bus answers the core's
 *   refused or undecoded accesses without an error.
 * ESP32P4_HP_SYS_PAD_HOLD: line n, GPIO(16 + n) held by
 *   HP_SYSTEM_GPIO_O_HOLD_CTRL0/1, to the GPIO block's "pad-hold" input
 *   through the OR with LP_SYSTEM's hold of the same pad.
 * ESP32P4_HP_SYS_HP_INTR: HP_INTR, high while an HP interrupt source that
 *   HP2LP_INTR_GROUP<n>_EN enables is high: the LP core's interrupt 30.
 * ESP32P4_HP_SYS_HP_WAKEUP: high while an HP interrupt source that
 *   HP2LP_WAKEUP_GROUP<n>_EN enables is high, for the PMU's wakeup logic.
 * ESP32P4_HP_SYS_CPU_WAKEUP: per HP core, HP_SYSTEM_CPU_WAKEUP_EVENT's
 *   bit, to the core's ESP32P4_CPU_WAKEUP input, which wakes it from WFI.
 * ESP32P4_HP_SYS_DMA_ADDR_SEL: SYS_DMA_ADDR_SEL, high while the DMA
 *   masters address the external memories at 0x4xxxxxxx rather than
 *   0x8xxxxxxx, for the DMA engines.
 * ESP32P4_HP_SYS_ADDR_INTERCHANGE: PSRAM_FLASH_ADDR_INTERCHANGE_CPU and
 *   _DMA, the swap of the flash and PSRAM windows in the AXI matrix for
 *   the cache's accesses and for the other masters', for the external
 *   memory model.
 * ESP32P4_HP_SYS_GMAC_PHY_INTF: three lines, SYS_GMAC_CTRL0's
 *   PHY_INTF_SEL bits, and ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL, its
 *   SBD_FLOWCTRL, to the EMAC.
 * ESP32P4_HP_SYS_USBOTG20_CTRL: SYS_USBOTG20_CTRL's PHY_SUSPENDM,
 *   PHY_SUSPEND_FORCE_EN, PHY_RSTN, PHY_RESET_FORCE_EN, PHY_PLL_FORCE_EN,
 *   PHY_PLL_EN and OTG_SUSPENDM (bits 15 to 21), to the USB OTG HS UTMI+
 *   PHY's "sys-ctrl" inputs.
 *
 * Named GPIO inputs
 * =================
 *
 * ESP32P4_HP_SYS_HP_SOURCE: the levels of interrupt sources 21 to 130
 *   (line k for source 21 + k), the interrupt matrix's
 *   ESP32P4_INTMATRIX_SOURCE_OUT, for HP2LP_INTR_GROUP<n>_ST.
 * ESP32P4_HP_SYS_CORE_STALLED: per HP core, high while it is stalled, for
 *   CPU_CORESTALLED_ST.
 * ESP32P4_HP_SYS_DMACTIVE_LPCORE: the LP core's debug module is active,
 *   for CORE_DMACTIVE_LPCORE.
 * ESP32P4_HP_SYS_GMAC_STATUS: the EMAC's outputs that SYS_GMAC_CTRL0
 *   shows, by line ESP32P4_HP_SYS_GMAC_*: its PPS output and its resets
 *   in its transmit and receive clock domains.
 * ESP32P4_HP_SYS_USB_OTGHS_PHY_ST: the USB OTG HS PHY's session end,
 *   VBUS valid, B-valid and session valid, USB_OTGHS_PHY_ST's bits 1 to 4.
 *
 * Sysbus IRQ 0 is HP_SYSREG_INTR, source 110 of the interrupt matrix.
 */
#define ESP32P4_HP_SYS_CPU_INTR         "cpu-intr-from-cpu"
#define ESP32P4_HP_SYS_CRYPTO_CTRL      "crypto-ctrl"
#define ESP32P4_HP_SYS_CRYPTO_CTRL_LINES 4
#define ESP32P4_HP_SYS_L2MEM_ERR_RESP   "l2mem-err-resp"
#define ESP32P4_HP_SYS_CACHE_RESET      "cache-reset"
#define ESP32P4_HP_SYS_CACHE_RESET_LINES 3
#define ESP32P4_HP_SYS_ERR_RESP_DIS     "core-err-resp-dis"
#define ESP32P4_HP_SYS_PAD_HOLD         "pad-hold"
#define ESP32P4_HP_SYS_HP_INTR          "hp-intr"
#define ESP32P4_HP_SYS_HP_WAKEUP        "hp-wakeup"
#define ESP32P4_HP_SYS_CPU_WAKEUP       "cpu-wakeup"
#define ESP32P4_HP_SYS_DMA_ADDR_SEL     "dma-addr-sel"
#define ESP32P4_HP_SYS_ADDR_INTERCHANGE "psram-flash-interchange"
#define ESP32P4_HP_SYS_GMAC_PHY_INTF    "gmac-phy-intf-sel"
#define ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES 3
#define ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL "gmac-sbd-flowctrl"
#define ESP32P4_HP_SYS_USBOTG20_CTRL    "usbotg20-ctrl"
#define ESP32P4_HP_SYS_USBOTG20_CTRL_LINES 7
#define ESP32P4_HP_SYS_HP_SOURCE        "hp-source"
#define ESP32P4_HP_SYS_CORE_STALLED     "core-stalled"
#define ESP32P4_HP_SYS_DMACTIVE_LPCORE  "dmactive-lpcore"
#define ESP32P4_HP_SYS_GMAC_STATUS      "gmac-status"
enum {
    ESP32P4_HP_SYS_GMAC_PTP_PPS,
    ESP32P4_HP_SYS_GMAC_RST_CLK_TX_N,
    ESP32P4_HP_SYS_GMAC_RST_CLK_RX_N,
    ESP32P4_HP_SYS_GMAC_STATUS_LINES,
};
#define ESP32P4_HP_SYS_USB_OTGHS_PHY_ST "usb-otghs-phy-st"
#define ESP32P4_HP_SYS_USB_OTGHS_PHY_ST_LINES 4

struct ESP32P4HPSysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* The registers as stored, indexed by offset / 4. */
    uint32_t regs[ESP32P4_HP_SYS_NREGS];
    /* The levels of interrupt sources 21 to 130. */
    uint32_t hp_source[4];
    /* The inputs' levels. */
    uint32_t core_stalled;
    bool dmactive_lpcore;
    uint32_t gmac_status;
    uint32_t usb_otghs_phy_st;
    /* The EMAC's system time, which SYS_GMAC_CTRL1 and _CTRL2 read. */
    uint64_t (*gmac_ptp_time)(void *opaque);
    void *gmac_ptp_opaque;

    qemu_irq cpu_intr[ESP32P4_HP_SYS_CPU_INTRS];
    qemu_irq cache_reset[ESP32P4_HP_SYS_CACHE_RESET_LINES];
    qemu_irq crypto_ctrl_out[ESP32P4_HP_SYS_CRYPTO_CTRL_LINES];
    qemu_irq l2mem_err_resp;
    qemu_irq err_resp_dis[ESP32P4_HP_SYS_CORES * ESP32P4_BUSES];
    qemu_irq pad_hold[ESP32P4_HP_SYS_HOLDS];
    qemu_irq hp_intr;
    qemu_irq hp_wakeup;
    qemu_irq cpu_wakeup[ESP32P4_HP_SYS_CORES];
    qemu_irq dma_addr_sel;
    qemu_irq addr_interchange[2];
    qemu_irq gmac_phy_intf[ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES];
    qemu_irq gmac_sbd_flowctrl;
    qemu_irq usbotg20_ctrl_out[ESP32P4_HP_SYS_USBOTG20_CTRL_LINES];
    /* HP_SYSREG_INTR, sysbus IRQ 0. */
    qemu_irq sysreg_irq;
};

/*
 * An access to the part of L2MEM the L2 cache occupies, at `offset` into
 * L2MEM: the cache controller's report, which raises EXCEED_ADDR.
 */
void esp32p4_hp_sys_l2mem_exceed(ESP32P4HPSysState *s, uint32_t offset,
                                 bool write);

/*
 * The EMAC's system time output (ptp_timestamp_o): @time returns its
 * seconds in the high 32 bits and sub-seconds in the low, for
 * SYS_GMAC_CTRL2 and _CTRL1.
 */
void esp32p4_hp_sys_set_gmac_ptp(ESP32P4HPSysState *s,
                                 uint64_t (*time)(void *opaque),
                                 void *opaque);

/*
 * A register as software reads it, for the blocks whose configuration
 * lives here (BITSCRAMBLER_PERI_SEL, the memories' power controls, ...).
 */
uint32_t esp32p4_hp_sys_reg(ESP32P4HPSysState *s, hwaddr offset);

#endif
