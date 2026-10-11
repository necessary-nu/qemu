/*
 * ESP32 Ethernet MAC (EMAC), a Synopsys DesignWare GMAC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ESP32_EMAC_H
#define HW_NET_ESP32_EMAC_H

#include "hw/net/esp32_dwmac.h"

#define TYPE_ESP32_EMAC "esp32.emac"
OBJECT_DECLARE_TYPE(Esp32EmacState, Esp32EmacClass, ESP32_EMAC)

/* DMA registers at 0x0000, the ESP32 clock wrapper at 0x0800, MAC 0x1000 */
#define ESP32_EMAC_REG_SIZE     0x2000

struct Esp32EmacClass {
    Esp32DwmacClass parent_class;

    DeviceRealize parent_realize;
    DeviceUnrealize parent_unrealize;
    ResettablePhases parent_phases;
};

struct Esp32EmacState {
    Esp32DwmacState parent_obj;

    MemoryRegion iomem;
    /* The bus the AHB master reaches internal SRAM through */
    MemoryRegion *dma_mr;
    AddressSpace dma_as;

    /* Clock wrapper */
    uint32_t ex_clkout_conf;
    uint32_t ex_oscclk_conf;
    uint32_t ex_clk_ctrl;
    uint32_t ex_phyinf_conf;
    uint32_t ex_pd_sel;
    uint32_t ex_date;
};

#endif
