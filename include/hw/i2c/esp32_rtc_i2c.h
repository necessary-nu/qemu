/*
 * ESP32 RTC_I2C: the RTC domain's I2C master, used by the ULP coprocessor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_ESP32_RTC_I2C_H
#define HW_I2C_ESP32_RTC_I2C_H

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/i2c/i2c.h"

#define TYPE_ESP32_RTC_I2C "esp32.rtc_i2c"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32RtcI2cState, ESP32_RTC_I2C)

#define ESP32_RTC_I2C_SIZE 0x400
#define ESP32_RTC_I2C_CMD_COUNT 16

/*
 * Lines, to and from RTCIO, which puts them on the pads RTCIO_SAR_I2C_IO
 * selects in RTC function 1:
 * - ESP32_RTC_I2C_SCL_OUT, ESP32_RTC_I2C_SDA_OUT (gpio-out): the level
 *   the controller drives;
 * - ESP32_RTC_I2C_SCL_OE, ESP32_RTC_I2C_SDA_OE (gpio-out): whether it
 *   drives it. Open drain, the default, drives only a low level.
 * - ESP32_RTC_I2C_SCL_IN, ESP32_RTC_I2C_SDA_IN (gpio-in): the lines as the
 *   pads read them, high while no pad is routed.
 */
#define ESP32_RTC_I2C_SCL_OUT   "esp32-rtc-i2c-scl-out"
#define ESP32_RTC_I2C_SCL_OE    "esp32-rtc-i2c-scl-oe"
#define ESP32_RTC_I2C_SDA_OUT   "esp32-rtc-i2c-sda-out"
#define ESP32_RTC_I2C_SDA_OE    "esp32-rtc-i2c-sda-oe"
#define ESP32_RTC_I2C_SCL_IN    "esp32-rtc-i2c-scl-in"
#define ESP32_RTC_I2C_SDA_IN    "esp32-rtc-i2c-sda-in"

REG32(RTC_I2C_SCL_LOW_PERIOD, 0x00)
REG32(RTC_I2C_CTRL, 0x04)
    FIELD(RTC_I2C_CTRL, SDA_FORCE_OUT, 0, 1)
    FIELD(RTC_I2C_CTRL, SCL_FORCE_OUT, 1, 1)
    FIELD(RTC_I2C_CTRL, MS_MODE, 4, 1)
    FIELD(RTC_I2C_CTRL, TRANS_START, 5, 1)
    FIELD(RTC_I2C_CTRL, TX_LSB_FIRST, 6, 1)
    FIELD(RTC_I2C_CTRL, RX_LSB_FIRST, 7, 1)
REG32(RTC_I2C_DEBUG_STATUS, 0x08)
    FIELD(RTC_I2C_DEBUG_STATUS, ACK_VAL, 0, 1)
    FIELD(RTC_I2C_DEBUG_STATUS, TIMED_OUT, 2, 1)
    FIELD(RTC_I2C_DEBUG_STATUS, ARB_LOST, 3, 1)
    FIELD(RTC_I2C_DEBUG_STATUS, BUS_BUSY, 4, 1)
    FIELD(RTC_I2C_DEBUG_STATUS, BYTE_TRANS, 6, 1)
    FIELD(RTC_I2C_DEBUG_STATUS, MAIN_STATE, 25, 3)
    FIELD(RTC_I2C_DEBUG_STATUS, SCL_STATE, 28, 3)
REG32(RTC_I2C_TIMEOUT, 0x0c)
REG32(RTC_I2C_SLAVE_ADDR, 0x10)
REG32(RTC_I2C_DATA, 0x1c)
REG32(RTC_I2C_INT_RAW, 0x20)
    FIELD(RTC_I2C_INT_RAW, SLAVE_TRANS_COMPLETE, 3, 1)
    FIELD(RTC_I2C_INT_RAW, ARBITRATION_LOST, 4, 1)
    FIELD(RTC_I2C_INT_RAW, MASTER_TRANS_COMPLETE, 5, 1)
    FIELD(RTC_I2C_INT_RAW, TRANS_COMPLETE, 6, 1)
    FIELD(RTC_I2C_INT_RAW, TIME_OUT, 7, 1)
REG32(RTC_I2C_INT_CLR, 0x24)
REG32(RTC_I2C_INT_EN, 0x28)
REG32(RTC_I2C_INT_ST, 0x2c)
REG32(RTC_I2C_SDA_DUTY, 0x30)
REG32(RTC_I2C_SCL_HIGH_PERIOD, 0x38)
REG32(RTC_I2C_SCL_START_PERIOD, 0x40)
REG32(RTC_I2C_SCL_STOP_PERIOD, 0x44)
REG32(RTC_I2C_CMD0, 0x48)
REG32(RTC_I2C_CMD15, 0x84)

#define ESP32_RTC_I2C_REG_COUNT (A_RTC_I2C_CMD15 / 4 + 1)

struct Esp32RtcI2cState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq scl_out, scl_oe, sda_out, sda_oe;

    uint32_t regs[ESP32_RTC_I2C_REG_COUNT];
    /* The lines as the pads deliver them */
    bool scl_in, sda_in;
};

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * The transaction of the ULP's I2C_RD or I2C_WR (TRM 1.6.2): to the 7-bit
 * slave addr, register sub, writing wdata, or reading the byte into
 * *rdata (0xff when nothing answers). Returns how long it took, in
 * RTC_FAST_CLK cycles.
 */
uint32_t esp32_rtc_i2c_transfer(Esp32RtcI2cState *s, uint8_t addr,
                                uint8_t sub, bool write, uint8_t wdata,
                                uint8_t *rdata);

#endif
