/*
 * ESP32 analog block: the internal analog I2C bus to the BBPLL and APLL
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_ANA_H
#define HW_MISC_ESP32_ANA_H

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"

#define TYPE_ESP32_ANA "misc.esp32.ana"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32AnaState, ESP32_ANA)

/* Pulsed when PLL_CLK's or APLL_CLK's frequency may have changed */
#define ESP32_ANA_CLK_UPDATE_GPIO "clk-update"

#define ESP32_ANA_SIZE 0x1000

/*
 * The I2C master's host registers, one per host id, at 4 * id. The ROM's
 * rom_i2c_readReg / rom_i2c_writeReg (ESP-IDF's regi2c) use host 3 for the
 * APLL and host 4 for the BBPLL.
 */
#define ESP32_ANA_HOST_COUNT 8
REG32(ANA_I2C_HOST0, 0x00)
    FIELD(ANA_I2C_HOST0, SLAVE, 0, 8)
    FIELD(ANA_I2C_HOST0, ADDR, 8, 8)
    FIELD(ANA_I2C_HOST0, DATA, 16, 8)
    FIELD(ANA_I2C_HOST0, WR, 24, 1)
    FIELD(ANA_I2C_HOST0, BUSY, 25, 1)
REG32(ANA_CONFIG, 0x44)
    FIELD(ANA_CONFIG, I2C_APLL_PD, 14, 1)
    FIELD(ANA_CONFIG, I2C_BBPLL_PD, 17, 1)

/* The analog blocks' slave ids on the bus (soc/regi2c_bbpll.h, apll.h) */
#define ESP32_ANA_SLAVE_BBPLL 0x66
#define ESP32_ANA_SLAVE_APLL  0x6d
#define ESP32_ANA_BANK_SIZE 16

struct Esp32AnaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq clk_update;
    /* RTC_CNTL, whose OPTIONS0 and ANA_CONF power the bus and the PLLs */
    struct Esp32RtcCntlState *rtc_cntl;

    uint32_t host[ESP32_ANA_HOST_COUNT];
    uint32_t ana_config;
    uint8_t bbpll[ESP32_ANA_BANK_SIZE];
    uint8_t apll[ESP32_ANA_BANK_SIZE];
    /* The BBPLL divider configuration last reported as uncharacterised */
    uint32_t logged_bbpll_cfg;
};

/* PLL_CLK in Hz: 320 MHz or 480 MHz, or 0 while the BBPLL is off */
uint32_t esp32_ana_pll_hz(Esp32AnaState *s);
/* Report BBPLL dividers that do not give PLL_CLK's frequency */
void esp32_ana_check_pll(Esp32AnaState *s);
/* APLL_CLK in Hz, or 0 while the APLL is off */
uint32_t esp32_ana_apll_hz(Esp32AnaState *s);

#endif
