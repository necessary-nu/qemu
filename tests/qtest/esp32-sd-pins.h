/*
 * The ESP32 SD/MMC host's slot 1 pins, set up as ESP-IDF's
 * sdmmc_slot_io_config() sets them up, for the qtests
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Slot 1's lines are function 3 (HS2_*) of their IO_MUX pads: CLK
 * GPIO14, CMD GPIO15, DAT0 GPIO2, DAT1 GPIO4, DAT2 GPIO12, DAT3 GPIO13,
 * all pads with an RTC function, whose pulls RTCIO's RUE and RDE set
 * (errata [GPIO-3.6]). ESP-IDF disables each pin's pull-down, enables its
 * input and selects function 3 with OEN_SEL clear; with
 * SDMMC_SLOT_FLAG_INTERNAL_PULLUP it first enables the pull-ups on CMD
 * and DAT0 to DAT3; it routes the card detect input to a constant 0, the
 * write protect to a constant 1 inverted, the interrupt to a constant 1.
 */

#ifndef ESP32_SD_PINS_H
#define ESP32_SD_PINS_H

#define SDP_GPIO                0x3ff44000
#define SDP_ENABLE_W1TS         (SDP_GPIO + 0x024)
#define SDP_ENABLE_W1TC         (SDP_GPIO + 0x028)
#define SDP_PIN(n)              (SDP_GPIO + 0x088 + 4 * (n))
#define SDP_PIN_PAD_DRIVER      (1u << 2)
#define SDP_FUNC_IN(sig)        (SDP_GPIO + 0x130 + 4 * (sig))
#define SDP_FUNC_OUT(n)         (SDP_GPIO + 0x530 + 4 * (n))
#define SDP_IN_SIG_IN_SEL       (1u << 7)
#define SDP_IN_INV              (1u << 6)
#define SDP_IN_CONST_LOW        0x30
#define SDP_IN_CONST_HIGH       0x38
#define SDP_OUT_OEN_SEL         (1u << 10)
#define SDP_OUT_OEN_INV         (1u << 11)
#define SDP_SIG_GPIO_OUT        256

#define SDP_IO_MUX              0x3ff49000
#define SDP_MCU_SEL(f)          ((f) << 12)
#define SDP_MCU_SEL_MASK        (7u << 12)
#define SDP_FUN_IE              (1u << 9)
#define SDP_FUN_WPU             (1u << 8)
#define SDP_FUN_WPD             (1u << 7)
#define SDP_FUNC_SD             3
#define SDP_FUNC_GPIO           2

#define SDP_RTCIO               0x3ff48400
#define SDP_TOUCH_PAD(n)        (SDP_RTCIO + 0x94 + 4 * (n))
#define SDP_RTC_MUX_SEL         (1u << 19)
#define SDP_RTC_RUE             (1u << 27)
#define SDP_RTC_RDE             (1u << 28)

/* The SD/MMC host's matrix inputs, slot 1's */
#define SDP_SIG_CARD_DETECT_N   98
#define SDP_SIG_CARD_WRITE_PRT  100
#define SDP_SIG_CARD_INT_N      102

enum {
    SD_PIN_CLK,
    SD_PIN_CMD,
    SD_PIN_D0,
    SD_PIN_D1,
    SD_PIN_D2,
    SD_PIN_D3,
    SD_PINS,
};

/* Each pin's pad, its IO_MUX register's offset, its RTCIO touch pad */
static const struct {
    uint8_t pad;
    uint8_t io_mux;
    uint8_t touch;
} sd_pin[SD_PINS] = {
    [SD_PIN_CLK] = { 14, 0x30, 6 },
    [SD_PIN_CMD] = { 15, 0x3c, 3 },
    [SD_PIN_D0] = { 2, 0x40, 2 },
    [SD_PIN_D1] = { 4, 0x48, 0 },
    [SD_PIN_D2] = { 12, 0x34, 5 },
    [SD_PIN_D3] = { 13, 0x38, 4 },
};

static inline uint32_t sd_pin_mux_addr(unsigned pin)
{
    return SDP_IO_MUX + sd_pin[pin].io_mux;
}

static inline void sd_pin_rtc_bits(QTestState *qts, unsigned pin,
                                   uint32_t set, uint32_t clear)
{
    uint32_t addr = SDP_TOUCH_PAD(sd_pin[pin].touch);

    qtest_writel(qts, addr, (qtest_readl(qts, addr) & ~clear) | set);
}

/*
 * configure_pin_iomux(): the pull-down off (RTCIO's RDE), the input on,
 * function 3 with the host's own output enable (OEN_SEL clear).
 */
static inline void sd_pin_iomux(QTestState *qts, unsigned pin)
{
    uint32_t mux = sd_pin_mux_addr(pin);
    uint32_t out = SDP_FUNC_OUT(sd_pin[pin].pad);

    sd_pin_rtc_bits(qts, pin, 0, SDP_RTC_RDE);
    qtest_writel(qts, mux, qtest_readl(qts, mux) | SDP_FUN_IE);
    qtest_writel(qts, out, qtest_readl(qts, out) &
                 ~(SDP_OUT_OEN_SEL | SDP_OUT_OEN_INV));
    qtest_writel(qts, mux, (qtest_readl(qts, mux) & ~SDP_MCU_SEL_MASK) |
                 SDP_MCU_SEL(SDP_FUNC_SD));
}

/* sdmmc_host_pullup_en_internal(): the pull-ups on CMD and DAT0 to DAT3 */
static inline void sd_pull_ups(QTestState *qts)
{
    for (unsigned pin = SD_PIN_CMD; pin < SD_PINS; pin++) {
        sd_pin_rtc_bits(qts, pin, SDP_RTC_RUE, 0);
    }
}

/* The card detect, write protect and interrupt inputs, as ESP-IDF has them */
static inline void sd_side_band(QTestState *qts)
{
    qtest_writel(qts, SDP_FUNC_IN(SDP_SIG_CARD_INT_N),
                 SDP_IN_SIG_IN_SEL | SDP_IN_CONST_HIGH);
    qtest_writel(qts, SDP_FUNC_IN(SDP_SIG_CARD_DETECT_N),
                 SDP_IN_SIG_IN_SEL | SDP_IN_CONST_LOW);
    qtest_writel(qts, SDP_FUNC_IN(SDP_SIG_CARD_WRITE_PRT),
                 SDP_IN_SIG_IN_SEL | SDP_IN_INV | SDP_IN_CONST_HIGH);
}

/*
 * Slot 1 as ESP-IDF sets it up with SDMMC_SLOT_FLAG_INTERNAL_PULLUP, its
 * bus 4 bits wide (DAT3 given to the host).
 */
static inline void sd_pins(QTestState *qts)
{
    sd_pull_ups(qts);
    for (unsigned pin = 0; pin < SD_PINS; pin++) {
        sd_pin_iomux(qts, pin);
    }
    sd_side_band(qts);
}

#endif
