/*
 * QTest helper for the ESP32-P4 board's ES8311 codec, for tests that use
 * its pads for something else
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The codec drives GPIO11, its ASDOUT, from power-on: a test that uses
 * GPIO11 for something else first has it let go (TRI_ADCDAT), as software
 * on the board would, over the board's ESP_I2C wires (GPIO8 SCL, GPIO7
 * SDA, with their 2.2 kOhm pull-ups), driven open-drain as GPIOs.
 */

#ifndef ESP32P4_BOARD_CODEC_H
#define ESP32P4_BOARD_CODEC_H

#define BC_GPIO_OUT_W1TC        0x500e000cu
#define BC_GPIO_ENABLE_W1TS     0x500e0024u
#define BC_GPIO_ENABLE_W1TC     0x500e0028u
#define BC_GPIO_FUNC_OUT(n)     (0x500e0558u + 4u * (n))
#define BC_IO_MUX_GPIO(n)       (0x500e1004u + 4u * (n))
#define BC_MCU_SEL_GPIO         (1u << 12)
#define BC_FUN_IE               (1u << 9)
#define BC_SIG_GPIO_OUT         256
#define BC_SCL                  8
#define BC_SDA                  7

/* A wire low (driven) or high (released to its pull-up). */
static inline void bc_line(QTestState *qts, int pad, bool high)
{
    qtest_writel(qts, high ? BC_GPIO_ENABLE_W1TC : BC_GPIO_ENABLE_W1TS,
                 1u << pad);
}

static inline void bc_byte(QTestState *qts, uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        bc_line(qts, BC_SDA, (b >> i) & 1);
        bc_line(qts, BC_SCL, true);
        bc_line(qts, BC_SCL, false);
    }
    /* The acknowledge's clock, SDA released */
    bc_line(qts, BC_SDA, true);
    bc_line(qts, BC_SCL, true);
    bc_line(qts, BC_SCL, false);
}

/* Write register @reg of the codec (0x18) with @val. */
static inline void esp32p4_codec_write(QTestState *qts, uint8_t reg,
                                       uint8_t val)
{
    for (int pad = BC_SDA; pad <= BC_SCL; pad++) {
        qtest_writel(qts, BC_IO_MUX_GPIO(pad), BC_MCU_SEL_GPIO | BC_FUN_IE);
        qtest_writel(qts, BC_GPIO_FUNC_OUT(pad), BC_SIG_GPIO_OUT);
        qtest_writel(qts, BC_GPIO_OUT_W1TC, 1u << pad);
        bc_line(qts, pad, true);
    }
    /* START, the address, the register, the data, STOP */
    bc_line(qts, BC_SDA, false);
    bc_line(qts, BC_SCL, false);
    bc_byte(qts, 0x18 << 1);
    bc_byte(qts, reg);
    bc_byte(qts, val);
    bc_line(qts, BC_SDA, false);
    bc_line(qts, BC_SCL, true);
    bc_line(qts, BC_SDA, true);
}

/* The codec lets GPIO11 go: TRI_ADCDAT (register 0x07 bit 4). */
static inline void esp32p4_codec_release_gpio11(QTestState *qts)
{
    esp32p4_codec_write(qts, 0x07, 0x10);
}

#endif
