/*
 * ESP32-P4 analog blocks on the analog I2C bus: the PLLs, BIAS, DIG_REG
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ANA_H
#define HW_MISC_ESP32P4_ANA_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ANA "esp32p4-ana"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4AnaState, ESP32P4_ANA)

/*
 * The PLLs, in the order of HP_SYS_CLKRST_ANA_PLL_CTRL0's calibration
 * controls: the audio PLL (PLLA), the CPU PLL, the SDIO PLL, the system
 * PLL and the MSPI PLL.
 */
enum {
    ESP32P4_ANA_PLL_APLL,
    ESP32P4_ANA_PLL_CPLL,
    ESP32P4_ANA_PLL_SDIO,
    ESP32P4_ANA_PLL_SPLL,
    ESP32P4_ANA_PLL_MPLL,
    ESP32P4_ANA_NUM_PLLS,
};

/* The slaves on the analog I2C bus, each a file of 8-bit registers. */
enum {
    ESP32P4_ANA_SLAVE_SDIO_PLL,
    ESP32P4_ANA_SLAVE_MSPI,
    ESP32P4_ANA_SLAVE_SYS_PLL,
    ESP32P4_ANA_SLAVE_CPU_PLL,
    ESP32P4_ANA_SLAVE_SAR,
    ESP32P4_ANA_SLAVE_BIAS,
    ESP32P4_ANA_SLAVE_DIG_REG,
    ESP32P4_ANA_SLAVE_PLLA,
    ESP32P4_ANA_NUM_SLAVES,
};
#define ESP32P4_ANA_SLAVE_REGS 16

/*
 * Clock input "xtal", the PLLs' reference; clock outputs "apll", "cpll",
 * "sdio-pll", "spll" and "mpll", each PLL's output, stopped while it is
 * powered down or not locked.
 *
 * Named GPIO inputs, per PLL: "xpd", its power-up by the PMU, and
 * "cal-stop", HP_SYS_CLKRST_ANA_PLL_CTRL0's *_CAL_STOP. Named GPIO output,
 * per PLL: "cal-end", its calibration done; and "bias-update", a pulse as
 * a BIAS register is written, for the brown-out detector, whose thresholds
 * BIAS holds.
 */
#define ESP32P4_ANA_XPD         "xpd"
#define ESP32P4_ANA_CAL_STOP    "cal-stop"
#define ESP32P4_ANA_CAL_END     "cal-end"
#define ESP32P4_ANA_BIAS_UPDATE "bias-update"

/*
 * BIAS's brown-out thresholds (TRM 25.4): VDD_BAT's, VDD_ANA's and the
 * battery charger's, each its under-voltage threshold in bits 4:2 and the
 * threshold that releases it in bits 7:5.
 */
#define ESP32P4_ANA_BIAS_DREF_VBAT          8
#define ESP32P4_ANA_BIAS_DREF_VDDA          9
#define ESP32P4_ANA_BIAS_DREF_VBAT_CHARGER  10

extern const char *const esp32p4_ana_pll_names[ESP32P4_ANA_NUM_PLLS];

typedef struct ESP32P4AnaPllTimer {
    ESP32P4AnaState *s;
    int pll;
} ESP32P4AnaPllTimer;

struct ESP32P4AnaState {
    SysBusDevice parent_obj;

    Clock *xtal;
    Clock *out[ESP32P4_ANA_NUM_PLLS];
    qemu_irq cal_end_out[ESP32P4_ANA_NUM_PLLS];
    qemu_irq bias_update;

    uint8_t regs[ESP32P4_ANA_NUM_SLAVES][ESP32P4_ANA_SLAVE_REGS];

    bool xpd[ESP32P4_ANA_NUM_PLLS];
    bool cal_stop[ESP32P4_ANA_NUM_PLLS];
    bool locked[ESP32P4_ANA_NUM_PLLS];
    bool cal_end[ESP32P4_ANA_NUM_PLLS];
    QEMUTimer *lock_timer[ESP32P4_ANA_NUM_PLLS];
    QEMUTimer *cal_timer[ESP32P4_ANA_NUM_PLLS];
    ESP32P4AnaPllTimer timer_ctx[ESP32P4_ANA_NUM_PLLS];
};

/* The slave at 7-bit I2C address @addr, or -1 if there is none. */
int esp32p4_ana_slave(uint8_t addr);

/*
 * The analog I2C master's transactions with @slave: a write of @data to
 * register @reg, or a read of it into *@data. They return false where
 * the slave does not acknowledge, and then do nothing.
 */
bool esp32p4_ana_write(ESP32P4AnaState *s, int slave, uint8_t reg,
                       uint8_t data);
bool esp32p4_ana_read(ESP32P4AnaState *s, int slave, uint8_t reg,
                      uint8_t *data);

#endif
