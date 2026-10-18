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
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/misc/esp32p4_pms.h"
#include "hw/ssi/esp32p4_mspi.h"

#define TYPE_ESP32P4_SDMMC "esp32p4-sdmmc"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4SdmmcState, ESP32P4_SDMMC)

#define ESP32P4_SDMMC_BASE      0x50083000

/*
 * Slot 0's IO MUX pads, function 0 (SD1_*), for CLK, CMD and DAT0 to
 * DAT7; slot 1 has none, its lines GPIO matrix signals 0 to 9.
 */
#define ESP32P4_SDMMC_SLOT0_FUNC    0
#define ESP32P4_SDMMC_SLOT0_PADS    { 43, 44, 39, 40, 41, 42, 45, 46, 47, 48 }
/* A slot line the board does not wire to the card */
#define ESP32P4_SDMMC_NO_PAD        0xff

/* The pad outputs, a line per slot line (slot * DWC_SDMMC_LINES + line) */
#define ESP32P4_SDMMC_PAD_OUT       "pad-out"
#define ESP32P4_SDMMC_PAD_OE        "pad-oe"

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
 *
 * Its "gpio" link is the GPIO matrix and IO MUX between it and its cards
 * (hw/sd/esp32p4_sdmmc.c), the board's wiring its properties:
 * "slot0-dat-lines", the DAT lines of slot 0's IO MUX pads the board
 * wires to the card (4 on the microSD socket), and "slot1-pad-clk",
 * "-cmd", "-d0" to "-d7", the pads the board wires slot 1's card's pins
 * to (ESP32P4_SDMMC_NO_PAD: none). Its outputs "pad-out" and "pad-oe"
 * give the levels its lines hold between transfers, for the pads: SDCLK
 * low, CMD and DAT high, all released.
 */
struct ESP32P4SdmmcState {
    DWCSDMMCState parent_obj;

    ESP32P4PmsState *pms;
    ESP32P4ClkRstState *clkrst;
    ESP32P4GpioState *gpio;
    uint32_t board_delay_ps;
    uint8_t slot0_dat_lines;
    uint8_t slot1_pad[DWC_SDMMC_LINES];
    Clock *drv_clk;
    Clock *sam_clk;
    Notifier route_notifier;
    Notifier line_source;
    /* The IDMAC's wait on a memory controller's held AXI port */
    ESP32P4MspiDmaWait dma_wait;
    qemu_irq pad_out[DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES];
    qemu_irq pad_oe[DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES];

    uint32_t dll_clk_conf;
    uint32_t dll_conf;
};

#endif
