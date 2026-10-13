/*
 * The ESP32 SDIO slave's pins, set up as ESP-IDF's sdio_slave driver sets
 * them up, for the qtests
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The slave's lines are function 4 (SD_*) of slot 1's IO_MUX pads, the
 * pads the SD/MMC host's slot 1 uses on function 3 (esp32-sd-pins.h):
 * CLK GPIO14, CMD GPIO15, DAT0 GPIO2, DAT1 GPIO4, DAT2 GPIO12, DAT3
 * GPIO13, all RTC pads, whose pulls RTCIO's RUE and RDE set (errata
 * [GPIO-3.6]). ESP-IDF's configure_pin() enables each pin's input,
 * selects function 4, sets drive strength 3 and disables its pull-down;
 * with SDIO_SLAVE_FLAG_INTERNAL_PULLUP it enables the pull-ups on CMD and
 * DAT0 to DAT3 (CLK keeps its own from reset).
 */

#ifndef ESP32_SDIO_PINS_H
#define ESP32_SDIO_PINS_H

#include "esp32-sd-pins.h"

#define SDP_FUNC_SDIO           4
#define SDP_FUN_DRV(n)          ((n) << 10)
#define SDP_FUN_DRV_MASK        (3u << 10)

/* configure_pin() */
static inline void sdio_pin(QTestState *qts, unsigned pin, bool pullup)
{
    uint32_t mux = sd_pin_mux_addr(pin);

    qtest_writel(qts, mux, qtest_readl(qts, mux) | SDP_FUN_IE);
    qtest_writel(qts, mux, (qtest_readl(qts, mux) &
                            ~(SDP_MCU_SEL_MASK | SDP_FUN_DRV_MASK)) |
                 SDP_MCU_SEL(SDP_FUNC_SDIO) | SDP_FUN_DRV(3));
    sd_pin_rtc_bits(qts, pin, pullup ? SDP_RTC_RUE : 0, SDP_RTC_RDE);
}

/* sdio_slave_hw_init(): every pin, with the internal pull-ups if asked */
static inline void sdio_pins(QTestState *qts, bool pullup)
{
    sdio_pin(qts, SD_PIN_CLK, false);
    for (unsigned pin = SD_PIN_CMD; pin < SD_PINS; pin++) {
        sdio_pin(qts, pin, pullup);
    }
}

#endif
