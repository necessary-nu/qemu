/*
 * ESP32-P4 Ethernet MAC (EMAC), a Synopsys DesignWare GMAC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ESP32P4_EMAC_H
#define HW_NET_ESP32P4_EMAC_H

#include "hw/net/esp32_dwmac.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_pms.h"

#define TYPE_ESP32P4_EMAC "esp32p4-emac"
OBJECT_DECLARE_TYPE(ESP32P4EmacState, ESP32P4EmacClass, ESP32P4_EMAC)

#define ESP32P4_EMAC_BASE           0x50098000
#define ESP32P4_EMAC_SIZE           0x4000

/*
 * How the machine connects the EMAC
 * =================================
 *
 * Clock inputs: "sys", the bus clock its registers, DMA and MDC run on
 * (EMAC_SYS_CLK); "rmii", "rx" and "tx", EMAC_RMII_CLK, EMAC_RX_CLK and
 * EMAC_TX_CLK; "ptp-ref", EMAC_PTP_REF_CLK; and "phy-ref", the board
 * PHY's 50 MHz reference output, which reaches HP_SYS_CLKRST's
 * PAD_EMAC_TXRX_CLK, the clock output "pad-txrx", only through a pad
 * whose IO MUX selects EMAC_RMII_CLK ("pad-ref-clk").
 *
 * Named GPIO inputs: ESP32P4_EMAC_RESET, LP_CLKRST_RST_EN_EMAC, holding
 * the EMAC in reset while high; ESP32P4_EMAC_PHY_INTF, three lines,
 * HP_SYSTEM_GMAC_CTRL0's PHY_INTF_SEL; ESP32P4_EMAC_MDI, the MII_MDI
 * input signal from the GPIO matrix; and the GMAC's
 * ESP32_DWMAC_SBD_FLOWCTRL, HP_SYSTEM_GMAC_CTRL0's SBD_FLOWCTRL.
 *
 * Named GPIO outputs: ESP32P4_EMAC_MDC, ESP32P4_EMAC_MDO and
 * ESP32P4_EMAC_MDO_OE, the MII_MDC and MII_MDO output signals and the
 * latter's enable; ESP32P4_EMAC_TX_EN, EMAC_PHY_TXEN; ESP32P4_EMAC_RST_N,
 * two lines, HP_SYSTEM_GMAC_CTRL0's GMAC_RST_CLK_TX_N and _RX_N; and the
 * GMAC's ESP32_DWMAC_PTP_PPS, EMAC_PTP_PPS.
 *
 * Sysbus IRQs: the GMAC's, ETH_MAC_INTR, PMT_INTR and LPI_INTR.
 *
 * Links: "pms", whose bus the DMA reaches memory through as the GMAC
 * master; "gpio", whose routing decides whether the MII reaches the
 * PHY's pins, the board's wiring the "pad-*" properties give (without
 * it, the PHY is wired straight to the MAC); and the GMAC's "phy".
 */
#define ESP32P4_EMAC_RESET          "reset"
#define ESP32P4_EMAC_PHY_INTF       "phy-intf-sel"
#define ESP32P4_EMAC_PHY_INTF_LINES 3
#define ESP32P4_EMAC_MDI            "mdi"
#define ESP32P4_EMAC_MDC            "mdc"
#define ESP32P4_EMAC_MDO            "mdo"
#define ESP32P4_EMAC_MDO_OE         "mdo-oe"
#define ESP32P4_EMAC_TX_EN          "tx-en"
#define ESP32P4_EMAC_RST_N          "rst-clk-n"

/* The IO MUX function that takes the RMII reference clock from a pad */
#define ESP32P4_EMAC_RMII_CLK_FUNC  3

/* A "pad-*" property's value for a pin the board leaves unconnected */
#define ESP32P4_EMAC_NO_PAD         0xff

struct ESP32P4EmacClass {
    Esp32DwmacClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

struct ESP32P4EmacState {
    Esp32DwmacState parent_obj;

    MemoryRegion iomem;
    ESP32P4PmsState *pms;
    ESP32P4GpioState *gpio;
    Notifier route_notifier;

    Clock *rmii_clk;
    Clock *rx_clk;
    Clock *tx_clk;
    Clock *phy_ref_clk;
    Clock *pad_txrx_clk;

    /* PHY_INTF_SEL as HP_SYSTEM drives it, and as the last reset took it */
    uint8_t phy_intf_sel;
    uint8_t phy_intf;
    /* The MII_MDI input's level */
    bool mdi;

    qemu_irq mdc;
    qemu_irq mdo;
    qemu_irq mdo_oe;
    qemu_irq tx_en;
    qemu_irq rst_clk_n[2];

    /* The board's wiring: the pads the PHY's pins are on */
    uint8_t pad_mdc;
    uint8_t pad_mdio;
    uint8_t pad_tx_en;
    uint8_t pad_txd0;
    uint8_t pad_txd1;
    uint8_t pad_crs_dv;
    uint8_t pad_rxd0;
    uint8_t pad_rxd1;
    uint8_t pad_ref_clk;
};

#endif
