/*
 * ESP32 SENS: the SAR ADCs, DACs, touch sensor and Hall sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_SENS_H
#define HW_MISC_ESP32_SENS_H

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "qemu/timer.h"

#define TYPE_ESP32_SENS "misc.esp32.sens"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32SensState, ESP32_SENS)

/*
 * Lines:
 * - ESP32_SENS_TOUCH_INT (gpio-out): pulsed when a touch measurement finds
 *   the pads touched, RTC_CNTL's TOUCH interrupt.
 * - ESP32_SENS_TOUCH_WAKEUP (gpio-out): the touch wakeup request, high
 *   while the touched pads satisfy the wakeup condition.
 * - ESP32_SENS_TOUCH_TIMER_IN (gpio-in): RTC_CNTL_TOUCH_SLP_TIMER_EN, which
 *   lets the touch FSM's timer start measurements.
 * - ESP32_SENS_SARADC_CTRL_IN (gpio-in): pulsed when SYSCON's
 *   SARADC_CTRL is written, for the pattern table pointers' clear bits.
 */
#define ESP32_SENS_TOUCH_INT        "esp32-sens-touch-int"
#define ESP32_SENS_TOUCH_WAKEUP     "esp32-sens-touch-wakeup"
#define ESP32_SENS_TOUCH_TIMER_IN   "esp32-sens-touch-timer"
#define ESP32_SENS_SARADC_CTRL_IN   "esp32-sens-saradc-ctrl"

#define ESP32_SENS_SIZE 0x400
#define ESP32_SENS_REG_COUNT 64

#define ESP32_ADC_UNITS 2
#define ESP32_ADC1_CHANNELS 8
#define ESP32_ADC2_CHANNELS 10
#define ESP32_TOUCH_PADS 10
#define ESP32_DAC_CHANNELS 2

REG32(SENS_SAR_READ_CTRL, 0x00)
    FIELD(SENS_SAR_READ_CTRL, CLK_DIV, 0, 8)
    FIELD(SENS_SAR_READ_CTRL, SAMPLE_CYCLE, 8, 8)
    FIELD(SENS_SAR_READ_CTRL, SAMPLE_BIT, 16, 2)
    FIELD(SENS_SAR_READ_CTRL, DIG_FORCE, 27, 1)
    FIELD(SENS_SAR_READ_CTRL, DATA_INV, 28, 1)
REG32(SENS_SAR_READ_STATUS1, 0x04)
REG32(SENS_SAR_MEAS_WAIT1, 0x08)
REG32(SENS_SAR_MEAS_WAIT2, 0x0c)
REG32(SENS_SAR_MEAS_CTRL, 0x10)
REG32(SENS_SAR_READ_STATUS2, 0x14)
REG32(SENS_ULP_CP_SLEEP_CYC0, 0x18)
REG32(SENS_ULP_CP_SLEEP_CYC4, 0x28)
REG32(SENS_SAR_START_FORCE, 0x2c)
    FIELD(SENS_SAR_START_FORCE, SAR1_BIT_WIDTH, 0, 2)
    FIELD(SENS_SAR_START_FORCE, SAR2_BIT_WIDTH, 2, 2)
    FIELD(SENS_SAR_START_FORCE, SAR2_EN_TEST, 4, 1)
    FIELD(SENS_SAR_START_FORCE, ULP_CP_FORCE_START_TOP, 8, 1)
    FIELD(SENS_SAR_START_FORCE, ULP_CP_START_TOP, 9, 1)
    FIELD(SENS_SAR_START_FORCE, SAR2_STOP, 22, 1)
    FIELD(SENS_SAR_START_FORCE, SAR1_STOP, 23, 1)
REG32(SENS_SAR_MEM_WR_CTRL, 0x30)
    FIELD(SENS_SAR_MEM_WR_CTRL, OFFST_CLR, 22, 1)
REG32(SENS_SAR_ATTEN1, 0x34)
REG32(SENS_SAR_ATTEN2, 0x38)
REG32(SENS_SAR_SLAVE_ADDR1, 0x3c)
    FIELD(SENS_SAR_SLAVE_ADDR1, MEAS_STATUS, 22, 8)
REG32(SENS_SAR_SLAVE_ADDR2, 0x40)
REG32(SENS_SAR_SLAVE_ADDR3, 0x44)
REG32(SENS_SAR_SLAVE_ADDR4, 0x48)
REG32(SENS_SAR_TSENS_CTRL, 0x4c)
    FIELD(SENS_SAR_TSENS_CTRL, POWER_UP, 24, 1)
    FIELD(SENS_SAR_TSENS_CTRL, POWER_UP_FORCE, 25, 1)
REG32(SENS_SAR_I2C_CTRL, 0x50)
    FIELD(SENS_SAR_I2C_CTRL, START, 28, 1)
    FIELD(SENS_SAR_I2C_CTRL, START_FORCE, 29, 1)
REG32(SENS_SAR_MEAS_START1, 0x54)
    FIELD(SENS_SAR_MEAS_START1, DATA, 0, 16)
    FIELD(SENS_SAR_MEAS_START1, DONE, 16, 1)
    FIELD(SENS_SAR_MEAS_START1, START_SAR, 17, 1)
    FIELD(SENS_SAR_MEAS_START1, START_FORCE, 18, 1)
    FIELD(SENS_SAR_MEAS_START1, EN_PAD, 19, 12)
    FIELD(SENS_SAR_MEAS_START1, EN_PAD_FORCE, 31, 1)
REG32(SENS_SAR_TOUCH_CTRL1, 0x58)
    FIELD(SENS_SAR_TOUCH_CTRL1, MEAS_DELAY, 0, 16)
    FIELD(SENS_SAR_TOUCH_CTRL1, XPD_WAIT, 16, 8)
    FIELD(SENS_SAR_TOUCH_CTRL1, OUT_SEL, 24, 1)
    FIELD(SENS_SAR_TOUCH_CTRL1, OUT_1EN, 25, 1)
    FIELD(SENS_SAR_TOUCH_CTRL1, XPD_HALL_FORCE, 26, 1)
    FIELD(SENS_SAR_TOUCH_CTRL1, HALL_PHASE_FORCE, 27, 1)
REG32(SENS_SAR_TOUCH_THRES1, 0x5c)
REG32(SENS_SAR_TOUCH_THRES5, 0x6c)
REG32(SENS_SAR_TOUCH_OUT1, 0x70)
REG32(SENS_SAR_TOUCH_OUT5, 0x80)
REG32(SENS_SAR_TOUCH_CTRL2, 0x84)
    FIELD(SENS_SAR_TOUCH_CTRL2, MEAS_EN, 0, 10)
    FIELD(SENS_SAR_TOUCH_CTRL2, MEAS_DONE, 10, 1)
    FIELD(SENS_SAR_TOUCH_CTRL2, START_FSM_EN, 11, 1)
    FIELD(SENS_SAR_TOUCH_CTRL2, START_EN, 12, 1)
    FIELD(SENS_SAR_TOUCH_CTRL2, START_FORCE, 13, 1)
    FIELD(SENS_SAR_TOUCH_CTRL2, SLEEP_CYCLES, 14, 16)
    FIELD(SENS_SAR_TOUCH_CTRL2, MEAS_EN_CLR, 30, 1)
REG32(SENS_SAR_TOUCH_ENABLE, 0x8c)
    FIELD(SENS_SAR_TOUCH_ENABLE, WORKEN, 0, 10)
    FIELD(SENS_SAR_TOUCH_ENABLE, OUTEN2, 10, 10)
    FIELD(SENS_SAR_TOUCH_ENABLE, OUTEN1, 20, 10)
REG32(SENS_SAR_READ_CTRL2, 0x90)
    FIELD(SENS_SAR_READ_CTRL2, PWDET_FORCE, 27, 1)
    FIELD(SENS_SAR_READ_CTRL2, DIG_FORCE, 28, 1)
    FIELD(SENS_SAR_READ_CTRL2, DATA_INV, 29, 1)
REG32(SENS_SAR_MEAS_START2, 0x94)
REG32(SENS_SAR_DAC_CTRL1, 0x98)
    FIELD(SENS_SAR_DAC_CTRL1, SW_FSTEP, 0, 16)
    FIELD(SENS_SAR_DAC_CTRL1, SW_TONE_EN, 16, 1)
    FIELD(SENS_SAR_DAC_CTRL1, DAC_DIG_FORCE, 22, 1)
REG32(SENS_SAR_DAC_CTRL2, 0x9c)
    FIELD(SENS_SAR_DAC_CTRL2, DC1, 0, 8)
    FIELD(SENS_SAR_DAC_CTRL2, DC2, 8, 8)
    FIELD(SENS_SAR_DAC_CTRL2, SCALE1, 16, 2)
    FIELD(SENS_SAR_DAC_CTRL2, SCALE2, 18, 2)
    FIELD(SENS_SAR_DAC_CTRL2, INV1, 20, 2)
    FIELD(SENS_SAR_DAC_CTRL2, INV2, 22, 2)
    FIELD(SENS_SAR_DAC_CTRL2, CW_EN1, 24, 1)
    FIELD(SENS_SAR_DAC_CTRL2, CW_EN2, 25, 1)
REG32(SENS_SAR_MEAS_CTRL2, 0xa0)
REG32(SENS_SAR_NOUSE, 0xf8)
REG32(SENS_SARDATE, 0xfc)

/* The touch FSM's phases */
typedef enum Esp32TouchPhase {
    ESP32_TOUCH_IDLE,
    ESP32_TOUCH_XPD_WAIT,
    ESP32_TOUCH_MEASURE,
    ESP32_TOUCH_SLEEP,
} Esp32TouchPhase;

typedef struct Esp32RtcCntlState Esp32RtcCntlState;
typedef struct Esp32RtcIoState Esp32RtcIoState;
typedef struct Esp32GpioState Esp32GpioState;
typedef struct Esp32ApbCtrlState Esp32ApbCtrlState;

struct Esp32SensState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq touch_int;
    qemu_irq touch_wakeup;

    Esp32RtcCntlState *rtc_cntl;
    Esp32RtcIoState *rtcio;
    Esp32GpioState *gpio;
    Esp32ApbCtrlState *apb_ctrl;

    uint32_t regs[ESP32_SENS_REG_COUNT];

    /* The RTC controllers' conversions in progress, and their results */
    QEMUTimer adc_timer[ESP32_ADC_UNITS];
    bool adc_busy[ESP32_ADC_UNITS];
    uint32_t adc_result[ESP32_ADC_UNITS];

    /* The DIG controllers' pattern table pointers, and alternate mode's */
    uint8_t patt_ptr[ESP32_ADC_UNITS];
    uint8_t alt_unit;

    QEMUTimer touch_timer;
    uint32_t touch_phase;
    bool touch_timer_en;
    bool touch_wakeup_level;

    /* The cosine generator's phase, PHASE at virtual time cw_anchor_ns */
    int64_t cw_anchor_ns;
    uint32_t cw_phase;
    /* The DAC codes the I2S0 DMA interface delivered last */
    uint8_t dac_dma[ESP32_DAC_CHANNELS];

    /* Analog inputs, set from QEMU */
    uint32_t adc_mv[ESP32_ADC_UNITS][ESP32_ADC2_CHANNELS];
    int32_t hall_ut;
    uint32_t touch_ff[ESP32_TOUCH_PADS];
};

/*
 * [spec:nuos:req:emu.esp32.analog]
 * I2S0's ADC interface: the WS edge that starts a scan step of the DIG
 * controllers. Fills out[] with the 16-bit DMA words the step produced, one
 * or, in double mode, two, and returns how many.
 */
unsigned esp32_sens_dig_sample(Esp32SensState *s, uint16_t out[2]);

/* SENS_SAR_DAC_DIG_FORCE: the DACs take their codes from I2S0 */
bool esp32_sens_dac_dma_enabled(Esp32SensState *s);
/* I2S0's DAC interface: channel ch (0 for DAC1) takes code */
void esp32_sens_dac_dma(Esp32SensState *s, unsigned ch, uint8_t code);

#endif
