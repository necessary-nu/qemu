/*
 * ESP32-P4 SD/MMC host pins, set up as ESP-IDF sets them up, for qtests
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ESP-IDF's esp_driver_sdmmc (sd_host_sdmmc.c, sdmmc_slot_io_config())
 * puts slot 0 on its IO MUX pins, configure_pin_iomux(): function 0
 * (SD1_*), the pull-down off, the input enabled, drive strength 3; and
 * slot 1 on the pins its configuration names through the GPIO matrix,
 * configure_pin_gpio_matrix(): gpio_reset_pin() (the GPIO function, the
 * pull-up on, input and output off), the direction (CLK an output, CMD
 * and DAT inputs and outputs), the pull-down off, then the ROM's
 * esp_rom_gpio_connect_in_signal() and _out_signal() (the host's output
 * enable, GPIO_ENABLE set). The board's microSD socket is slot 0's CLK,
 * CMD and DAT0 to DAT3; slot 1's card is where the ESP32-C6 is, on
 * GPIO18, GPIO19 and GPIO14 to GPIO17.
 */

#ifndef ESP32P4_SD_PINS_H
#define ESP32P4_SD_PINS_H

#include "libqtest.h"

#define SDP_GPIO            0x500e0000
#define SDP_ENABLE_W1TS     (SDP_GPIO + 0x024)
#define SDP_ENABLE_W1TC     (SDP_GPIO + 0x028)
#define SDP_ENABLE1_W1TS    (SDP_GPIO + 0x030)
#define SDP_ENABLE1_W1TC    (SDP_GPIO + 0x034)
#define SDP_FUNC_IN(sig)    (SDP_GPIO + 0x158 + 4 * (sig))
#define SDP_FUNC_OUT(n)     (SDP_GPIO + 0x558 + 4 * (n))
#define SDP_IO_MUX(n)       (0x500e1000 + 0x004 + 4 * (n))
#define SDP_FUN_WPD         (1u << 7)
#define SDP_FUN_WPU         (1u << 8)
#define SDP_FUN_IE          (1u << 9)
#define SDP_FUN_DRV(v)      ((uint32_t)(v) << 10)
#define SDP_MCU_SEL(v)      ((uint32_t)(v) << 12)
#define SDP_IN_SIG_IN_SEL   (1u << 7)
#define SDP_OUT_OEN_SEL     (1u << 10)
#define SDP_SIG_GPIO_OUT    0x100

/* The slots' pins: CLK, CMD, DAT0 to DAT3 */
#define SD_PIN_CLK          0
#define SD_PIN_CMD          1
#define SD_PIN_D(n)         (2 + (n))
#define SD_PINS             6

static const uint8_t sd_pin_pad[2][SD_PINS] = {
    { 43, 44, 39, 40, 41, 42 },
    { 18, 19, 14, 15, 16, 17 },
};

/* As ESP-IDF's configure_pin_iomux() */
static inline void sd_pin_iomux(QTestState *qts, unsigned pad)
{
    uint32_t v = qtest_readl(qts, SDP_IO_MUX(pad));

    v &= ~(SDP_FUN_WPD | SDP_MCU_SEL(7) | SDP_FUN_DRV(3));
    qtest_writel(qts, SDP_IO_MUX(pad), v | SDP_FUN_IE | SDP_MCU_SEL(0) |
                 SDP_FUN_DRV(3));
}

/* As configure_pin_gpio_matrix(), for matrix signal @sig, CLK an output */
static inline void sd_pin_matrix(QTestState *qts, unsigned pad,
                                 unsigned sig, bool input)
{
    uint32_t bit = 1u << (pad % 32);

    qtest_writel(qts, SDP_IO_MUX(pad), SDP_FUN_WPU | SDP_MCU_SEL(1) |
                 SDP_FUN_DRV(2) | (input ? SDP_FUN_IE : 0));
    qtest_writel(qts, SDP_FUNC_OUT(pad), SDP_SIG_GPIO_OUT);
    if (input) {
        qtest_writel(qts, SDP_FUNC_IN(sig), SDP_IN_SIG_IN_SEL | pad);
    }
    qtest_writel(qts, SDP_FUNC_OUT(pad), sig);
    qtest_writel(qts, pad < 32 ? SDP_ENABLE_W1TS : SDP_ENABLE1_W1TS, bit);
}

/* Slot @slot's CLK, CMD and DAT0 to DAT3, as sdmmc_slot_io_config(). */
static inline void sd_pins(QTestState *qts, unsigned slot)
{
    for (unsigned i = 0; i < SD_PINS; i++) {
        if (slot == 0) {
            sd_pin_iomux(qts, sd_pin_pad[0][i]);
        } else {
            sd_pin_matrix(qts, sd_pin_pad[1][i], i, i != SD_PIN_CLK);
        }
    }
}

#endif
