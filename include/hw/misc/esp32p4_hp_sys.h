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

/* The software interrupts CPU_INT_FROM_CPU_0 to _3. */
#define ESP32P4_HP_SYS_CPU_INTRS 4

/*
 * Named GPIO output ESP32P4_HP_SYS_CPU_INTR: CPU_INTR_FROM_CPU_0 to _3,
 * interrupt sources ESP32P4_INTR_FROM_CPU_INTR0 to _3 of the interrupt
 * matrix, each high while its register's bit is set.
 */
#define ESP32P4_HP_SYS_CPU_INTR "cpu-intr-from-cpu"

/*
 * Named GPIO output ESP32P4_HP_SYS_CRYPTO_CTRL: HP_SYSTEM_CRYPTO_CTRL's
 * four enables, ENABLE_SPI_MANUAL_ENCRYPT, ENABLE_DOWNLOAD_DB_ENCRYPT,
 * ENABLE_DOWNLOAD_G0CB_DECRYPT and ENABLE_DOWNLOAD_MANUAL_ENCRYPT, to the
 * MSPI controllers' "crypto-ctrl" inputs.
 */
#define ESP32P4_HP_SYS_CRYPTO_CTRL "crypto-ctrl"
#define ESP32P4_HP_SYS_CRYPTO_CTRL_LINES 4

/*
 * Named GPIO output ESP32P4_HP_SYS_L2MEM_ERR_RESP: HP_SYSTEM_L2_MEM_ERR_
 * RESP_EN, high while an access to the part of L2MEM the L2 cache occupies
 * is answered with an error, to the cache controller.
 */
#define ESP32P4_HP_SYS_L2MEM_ERR_RESP "l2mem-err-resp"

/*
 * Named GPIO output ESP32P4_HP_SYS_CACHE_RESET: HP_SYSTEM_CACHE_RESET_
 * CONFIG's L1_I0_CACHE_RESET, L1_I1_CACHE_RESET and L1_D_CACHE_RESET, in
 * that order, each high while set, to the cache controller's
 * "sys-cache-reset" inputs.
 */
#define ESP32P4_HP_SYS_CACHE_RESET "cache-reset"
#define ESP32P4_HP_SYS_CACHE_RESET_LINES 3

/*
 * Named GPIO outputs to the EMAC from HP_SYSTEM_GMAC_CTRL0:
 * ESP32P4_HP_SYS_GMAC_PHY_INTF, three lines, PHY_INTF_SEL's bits, and
 * ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL, SBD_FLOWCTRL.
 */
#define ESP32P4_HP_SYS_GMAC_PHY_INTF "gmac-phy-intf-sel"
#define ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES 3
#define ESP32P4_HP_SYS_GMAC_SBD_FLOWCTRL "gmac-sbd-flowctrl"

/*
 * Named GPIO input ESP32P4_HP_SYS_GMAC_STATUS, the EMAC's outputs that
 * HP_SYSTEM_GMAC_CTRL0 shows, by line ESP32P4_HP_SYS_GMAC_*: its PPS
 * output and its resets in its transmit and receive clock domains.
 */
#define ESP32P4_HP_SYS_GMAC_STATUS "gmac-status"
enum {
    ESP32P4_HP_SYS_GMAC_PTP_PPS,
    ESP32P4_HP_SYS_GMAC_RST_CLK_TX_N,
    ESP32P4_HP_SYS_GMAC_RST_CLK_RX_N,
    ESP32P4_HP_SYS_GMAC_STATUS_LINES,
};

/*
 * Named GPIO output ESP32P4_HP_SYS_BS_PERI_SEL: HP_SYSTEM_BITSCRAMBLER_
 * PERI_SEL's eight bits, PERI_RX_SEL in lines 0 to 3 and PERI_TX_SEL in 4
 * to 7, to the BitScrambler's "peri-sel" inputs.
 */
#define ESP32P4_HP_SYS_BS_PERI_SEL "bitscrambler-peri-sel"
#define ESP32P4_HP_SYS_BS_PERI_SEL_LINES 8

struct ESP32P4HPSysState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t cpu_int_from_cpu[ESP32P4_HP_SYS_CPU_INTRS];
    qemu_irq cpu_intr[ESP32P4_HP_SYS_CPU_INTRS];
    uint32_t cache_clk_config;
    uint32_t cache_reset_config;
    qemu_irq cache_reset[ESP32P4_HP_SYS_CACHE_RESET_LINES];
    uint32_t crypto_ctrl;
    qemu_irq crypto_ctrl_out[ESP32P4_HP_SYS_CRYPTO_CTRL_LINES];
    uint32_t l2_mem_int_raw;
    uint32_t l2_mem_int_ena;
    uint32_t l2_mem_int_record0;
    uint32_t l2_mem_err_resp_ctrl;
    qemu_irq l2mem_err_resp;
    /* HP_SYSREG_INTR, sysbus IRQ 0. */
    qemu_irq sysreg_irq;
    /* HP_SYSTEM_GMAC_CTRL0's writable fields, and the EMAC's status */
    uint32_t gmac_ctrl0;
    uint32_t gmac_status;
    qemu_irq gmac_phy_intf[ESP32P4_HP_SYS_GMAC_PHY_INTF_LINES];
    qemu_irq gmac_sbd_flowctrl;
    uint32_t bitscrambler_peri_sel;
    qemu_irq bs_peri_sel[ESP32P4_HP_SYS_BS_PERI_SEL_LINES];
    /* The EMAC's system time, which GMAC_CTRL1 and _CTRL2 read */
    uint64_t (*gmac_ptp_time)(void *opaque);
    void *gmac_ptp_opaque;
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
 * HP_SYSTEM_GMAC_CTRL2 and _CTRL1.
 */
void esp32p4_hp_sys_set_gmac_ptp(ESP32P4HPSysState *s,
                                 uint64_t (*time)(void *opaque),
                                 void *opaque);

#endif
