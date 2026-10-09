/*
 * ESP32 RTCIO: the RTC IO MUX, which controls the 18 RTC-capable pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"

#define TYPE_ESP32_RTCIO "esp32.rtcio"
typedef struct Esp32RtcIoState Esp32RtcIoState;

#define ESP32_RTCIO(obj) OBJECT_CHECK(Esp32RtcIoState, (obj), TYPE_ESP32_RTCIO)

#define ESP32_RTCIO_PAD_COUNT 18
#define ESP32_RTCIO_SIZE 0x400

/*
 * Pad control towards the GPIO block, one line per RTC GPIO number (TRM
 * table 6.11-1; esp32_rtcio_gpio[] gives each one's GPIO number):
 *
 * - ESP32_RTCIO_PAD_MUX (gpio-out): the pad's MUX_SEL. While it is high the
 *   RTC IO MUX, not the IO_MUX and GPIO matrix, drives the pad and controls
 *   its input enable and pulls, with:
 * - ESP32_RTCIO_PAD_OUT, ESP32_RTCIO_PAD_OE (gpio-out): the pad's output
 *   level and output enable;
 * - ESP32_RTCIO_PAD_PU, ESP32_RTCIO_PAD_PD (gpio-out): its pull-up and
 *   pull-down;
 * - ESP32_RTCIO_PAD_IE (gpio-out): its input enable.
 * - ESP32_RTCIO_PAD_IN (gpio-in): the pad's input as its input buffer
 *   delivers it, low while the buffer is disabled.
 */
#define ESP32_RTCIO_PAD_MUX "esp32-rtcio-pad-mux"
#define ESP32_RTCIO_PAD_OUT "esp32-rtcio-pad-out"
#define ESP32_RTCIO_PAD_OE  "esp32-rtcio-pad-oe"
#define ESP32_RTCIO_PAD_PU  "esp32-rtcio-pad-pu"
#define ESP32_RTCIO_PAD_PD  "esp32-rtcio-pad-pd"
#define ESP32_RTCIO_PAD_IE  "esp32-rtcio-pad-ie"
#define ESP32_RTCIO_PAD_IN  "esp32-rtcio-pad-in"

/* The GPIO number of each RTC GPIO */
extern const uint8_t esp32_rtcio_gpio[ESP32_RTCIO_PAD_COUNT];

REG32(RTCIO_RTC_GPIO_OUT, 0x00)
REG32(RTCIO_RTC_GPIO_OUT_W1TS, 0x04)
REG32(RTCIO_RTC_GPIO_OUT_W1TC, 0x08)
REG32(RTCIO_RTC_GPIO_ENABLE, 0x0c)
REG32(RTCIO_RTC_GPIO_ENABLE_W1TS, 0x10)
REG32(RTCIO_RTC_GPIO_ENABLE_W1TC, 0x14)
REG32(RTCIO_RTC_GPIO_STATUS, 0x18)
REG32(RTCIO_RTC_GPIO_STATUS_W1TS, 0x1c)
REG32(RTCIO_RTC_GPIO_STATUS_W1TC, 0x20)
REG32(RTCIO_RTC_GPIO_IN, 0x24)
REG32(RTCIO_RTC_GPIO_PIN0, 0x28)
    FIELD(RTCIO_RTC_GPIO_PIN0, WAKEUP_ENABLE, 10, 1)
    FIELD(RTCIO_RTC_GPIO_PIN0, INT_TYPE, 7, 3)
    FIELD(RTCIO_RTC_GPIO_PIN0, PAD_DRIVER, 2, 1)
REG32(RTCIO_RTC_GPIO_PIN17, 0x6c)
REG32(RTCIO_DEBUG_SEL, 0x70)
REG32(RTCIO_DIG_PAD_HOLD, 0x74)
REG32(RTCIO_HALL_SENS, 0x78)
REG32(RTCIO_SENSOR_PADS, 0x7c)
REG32(RTCIO_ADC_PAD, 0x80)
REG32(RTCIO_PAD_DAC1, 0x84)
REG32(RTCIO_PAD_DAC2, 0x88)
REG32(RTCIO_XTAL_32K_PAD, 0x8c)
REG32(RTCIO_TOUCH_CFG, 0x90)
REG32(RTCIO_TOUCH_PAD0, 0x94)
REG32(RTCIO_TOUCH_PAD7, 0xb0)
REG32(RTCIO_TOUCH_PAD8, 0xb4)
REG32(RTCIO_TOUCH_PAD9, 0xb8)
REG32(RTCIO_EXT_WAKEUP0, 0xbc)
    FIELD(RTCIO_EXT_WAKEUP0, SEL, 27, 5)
REG32(RTCIO_XTL_EXT_CTR, 0xc0)
REG32(RTCIO_SAR_I2C_IO, 0xc4)
REG32(RTCIO_DATE, 0xc8)

#define ESP32_RTCIO_REG_COUNT (A_RTCIO_DATE / 4 + 1)
/* The RTC GPIOs' bits in the RTC_GPIO_* registers */
#define ESP32_RTCIO_GPIO_SHIFT 14

struct Esp32RtcIoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* RTC_CNTL, for the hold forces, and to report wakeup changes to */
    struct Esp32RtcCntlState *rtc_cntl;

    qemu_irq pad_mux[ESP32_RTCIO_PAD_COUNT];
    qemu_irq pad_out[ESP32_RTCIO_PAD_COUNT];
    qemu_irq pad_oe[ESP32_RTCIO_PAD_COUNT];
    qemu_irq pad_pu[ESP32_RTCIO_PAD_COUNT];
    qemu_irq pad_pd[ESP32_RTCIO_PAD_COUNT];
    qemu_irq pad_ie[ESP32_RTCIO_PAD_COUNT];

    uint32_t regs[ESP32_RTCIO_REG_COUNT];

    /* The pads' inputs, bit n for RTC GPIO n */
    uint32_t pad_in;
    /*
     * The control each pad presents, bit n for RTC GPIO n: as its registers
     * give it, or as latched when its hold took effect.
     */
    uint32_t ctl_mux, ctl_out, ctl_oe, ctl_pu, ctl_pd, ctl_ie;
    uint32_t held;
    /* Drive every line on the next update, not only changed ones */
    bool resync;
};

/* The RTC GPIOs' inputs as the RTC domain sees them, bit n for RTC GPIO n */
uint32_t esp32_rtcio_inputs(Esp32RtcIoState *s);
/* The RTC GPIO EXT0 wakes on: RTCIO_EXT_WAKEUP0_SEL */
unsigned esp32_rtcio_ext0_sel(Esp32RtcIoState *s);
/* Some RTC GPIO with WAKEUP_ENABLE is at its INT_TYPE's wakeup level */
bool esp32_rtcio_gpio_wakeup(Esp32RtcIoState *s);
/* RTC_CNTL's pad hold forces changed */
void esp32_rtcio_hold_changed(Esp32RtcIoState *s);
