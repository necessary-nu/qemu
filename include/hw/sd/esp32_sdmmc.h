/*
 * ESP32 SD/MMC host controller (SDHOST) on its IO_MUX pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_ESP32_SDMMC_H
#define HW_SD_ESP32_SDMMC_H

#include "hw/sd/dwc_sdmmc.h"
#include "hw/gpio/esp32_gpio.h"

#define TYPE_ESP32_SDMMC "esp32.sdmmc"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32SdmmcState, ESP32_SDMMC)

/*
 * The slots' IO_MUX pads, function 3 (HS1_* for slot 0, HS2_* for slot
 * 1), for CLK, CMD and DAT0 to DAT7 (TRM table 6.10-1; ESP-IDF's
 * sdmmc_periph.c, SDMMC_LL_IOMUX_FUNC). Slot 1 is 4 bits wide: it has no
 * DAT4 to DAT7.
 */
#define ESP32_SDMMC_FUNC        3
#define ESP32_SDMMC_NO_PAD      0xff
#define ESP32_SDMMC_SLOT0_PADS  { 6, 11, 7, 8, 9, 10, 16, 17, 5, 18 }
#define ESP32_SDMMC_SLOT1_PADS  { 14, 15, 2, 4, 12, 13, ESP32_SDMMC_NO_PAD, \
                                  ESP32_SDMMC_NO_PAD, ESP32_SDMMC_NO_PAD,   \
                                  ESP32_SDMMC_NO_PAD }

/*
 * The host's outputs on its IO_MUX functions, a line per slot line (slot
 * * DWC_SDMMC_LINES + line), for the GPIO block's ESP32_GPIO_IOMUX_OUT and
 * ESP32_GPIO_IOMUX_OE: the levels its lines hold between transfers.
 */
#define ESP32_SDMMC_PAD_OUT     "pad-out"
#define ESP32_SDMMC_PAD_OE      "pad-oe"

/*
 * The DesignWare controller (hw/sd/dwc_sdmmc.h) on the ESP32's pads: its
 * "gpio" link is the GPIO matrix and IO_MUX between it and its cards
 * (hw/sd/esp32_sdmmc.c). A slot's card is wired to the slot's IO_MUX pads
 * above; a line joins the host to it only as those pads are set up.
 */
struct Esp32SdmmcState {
    DWCSDMMCState parent_obj;

    Esp32GpioState *gpio;
    Notifier route_notifier;
    Notifier line_source;
    qemu_irq pad_out[DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES];
    qemu_irq pad_oe[DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES];
};

#endif /* HW_SD_ESP32_SDMMC_H */
