/*
 * The ESP32-WROOM-32E's flash pins, set up as the ESP32 ROM and ESP-IDF's
 * bootloader set them up, for the qtests
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The module's flash is on GPIO6 to GPIO11: CLK GPIO6, DO (SPIQ) GPIO7,
 * DI (SPID) GPIO8, HOLD# (SPIHD) GPIO9, WP# (SPIWP) GPIO10, CS# (SPICS0)
 * GPIO11. From reset those pads are on IO_MUX function 0, the SDIO
 * slave's: SPI0 and SPI1 do not reach the flash until software puts them
 * on function 1, as the ROM does before it reads the flash (in DIO mode it
 * leaves HD and WP on function 0, pulled up), or routes the SPI signals
 * to them through the GPIO matrix, as ESP-IDF's bootloader does for all
 * but CLK. A qtest runs no ROM, so it sets them up itself.
 */

#ifndef ESP32_FLASH_PINS_H
#define ESP32_FLASH_PINS_H

#define FP_IO_MUX               0x3ff49000
#define FP_GPIO                 0x3ff44000
#define FP_FUNC_IN(sig)         (FP_GPIO + 0x130 + 4 * (sig))
#define FP_FUNC_OUT(n)          (FP_GPIO + 0x530 + 4 * (n))
#define FP_IN_SIG_IN_SEL        (1u << 7)

/* The SPI signals, as GPIO matrix numbers */
#define FP_SIG_SPIQ             1
#define FP_SIG_SPID             2
#define FP_SIG_SPIHD            3
#define FP_SIG_SPIWP            4
#define FP_SIG_SPICS0           5

enum {
    FLASH_PIN_CLK,
    FLASH_PIN_Q,
    FLASH_PIN_D,
    FLASH_PIN_HD,
    FLASH_PIN_WP,
    FLASH_PIN_CS,
    FLASH_PINS,
};

/* Each pin's pad, its IO_MUX register's offset, its SPI signal */
static const struct {
    uint8_t pad;
    uint8_t io_mux;
    uint8_t sig;
} flash_pin[FLASH_PINS] = {
    [FLASH_PIN_CLK] = { 6, 0x60, 0 },
    [FLASH_PIN_Q] = { 7, 0x64, FP_SIG_SPIQ },
    [FLASH_PIN_D] = { 8, 0x68, FP_SIG_SPID },
    [FLASH_PIN_HD] = { 9, 0x54, FP_SIG_SPIHD },
    [FLASH_PIN_WP] = { 10, 0x58, FP_SIG_SPIWP },
    [FLASH_PIN_CS] = { 11, 0x5c, FP_SIG_SPICS0 },
};

static inline uint32_t flash_pin_mux_addr(unsigned pin)
{
    return FP_IO_MUX + flash_pin[pin].io_mux;
}

/*
 * What the ROM writes before it reads the flash in DIO mode: CLK on
 * function 1 with drive strength 1, Q, D and CS0 on function 1 with drive
 * strength 2, HD and WP on function 0; all with their input and pull-up
 * on.
 */
static inline void flash_pins_rom(QTestState *qts)
{
    static const uint32_t mux[FLASH_PINS] = {
        [FLASH_PIN_CLK] = 0x1700,
        [FLASH_PIN_Q] = 0x1b00,
        [FLASH_PIN_D] = 0x1b00,
        [FLASH_PIN_HD] = 0x0b00,
        [FLASH_PIN_WP] = 0x0b00,
        [FLASH_PIN_CS] = 0x1b00,
    };

    for (unsigned pin = 0; pin < FLASH_PINS; pin++) {
        qtest_writel(qts, flash_pin_mux_addr(pin), mux[pin]);
    }
}

/*
 * ESP-IDF's bootloader_flash_gpio_config() with the default pins: CS0, Q,
 * D, WP and HD through the GPIO matrix, out and (but CS0) in, their pads
 * on the GPIO function; CLK on function 1.
 */
static inline void flash_pins_idf(QTestState *qts)
{
    flash_pins_rom(qts);
    for (unsigned pin = FLASH_PIN_Q; pin < FLASH_PINS; pin++) {
        uint32_t mux = flash_pin_mux_addr(pin);

        qtest_writel(qts, FP_FUNC_OUT(flash_pin[pin].pad), flash_pin[pin].sig);
        if (pin != FLASH_PIN_CS) {
            qtest_writel(qts, FP_FUNC_IN(flash_pin[pin].sig),
                         FP_IN_SIG_IN_SEL | flash_pin[pin].pad);
        }
        qtest_writel(qts, mux, (qtest_readl(qts, mux) & ~(7u << 12)) |
                     (2u << 12));
    }
}

#endif
