/*
 * ESP32-P4 SD/MMC host controller (SDHOST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_ESP32P4_SDMMC_H
#define HW_SD_ESP32P4_SDMMC_H

#include "hw/sd/dwc_sdmmc.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/misc/esp32p4_pms.h"

#define TYPE_ESP32P4_SDMMC "esp32p4-sdmmc"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4SdmmcState, ESP32P4_SDMMC)

#define ESP32P4_SDMMC_BASE      0x50083000

/*
 * The DesignWare controller (hw/sd/dwc_sdmmc.h) with the P4's own
 * registers and connections: its IDMAC reaches memory through the
 * permission managers ("pms" link) as DMA master SDMMC; besides "hclk"
 * (HP_SYS_CLKRST's SDMMC_SYS_CLK) and "cclk-in" (SDIO_LS_SLF_CLK, the
 * CIU's own clock), it takes "drv" and "sam" (SDIO_LS_DRV_CLK and
 * SDIO_LS_SAM_CLK), which drive its outputs and sample its inputs: with
 * either stopped, the CIU reaches no card. Its "clkrst" link gives the
 * sample clock's phase in low-speed mode, and "board-delay-ps" the board's
 * round trip from the clock pad to the card and back to the data pads.
 */
struct ESP32P4SdmmcState {
    DWCSDMMCState parent_obj;

    ESP32P4PmsState *pms;
    ESP32P4ClkRstState *clkrst;
    uint32_t board_delay_ps;
    Clock *drv_clk;
    Clock *sam_clk;

    uint32_t dll_clk_conf;
    uint32_t dll_conf;
};

#endif
