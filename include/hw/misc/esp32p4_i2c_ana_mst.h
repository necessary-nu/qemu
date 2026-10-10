/*
 * ESP32-P4 analog I2C controller (LP_I2C_ANA_MST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_I2C_ANA_MST_H
#define HW_MISC_ESP32P4_I2C_ANA_MST_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_ana.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_I2C_ANA_MST "esp32p4-i2c-ana-mst"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4I2CAnaMstState, ESP32P4_I2C_ANA_MST)

#define ESP32P4_I2C_ANA_MST_SIZE 0x1000
#define ESP32P4_I2C_ANA_MST_NREGS 15
#define ESP32P4_I2C_ANA_MST_MASTERS 2

typedef struct ESP32P4I2CAnaMstTimer {
    ESP32P4I2CAnaMstState *s;
    int n;
} ESP32P4I2CAnaMstTimer;

/*
 * Clock inputs: "bus", the controller's clock as LPPERI_CK_EN_LP_I2CMST
 * gates it; "lp-fast", LP_FAST_CLK, and "pll-f160m", PLL_F160M_CLK, the
 * clocks the masters' timing counts, as CLK_I2C_MST_SEL_160M selects.
 * Link "ana": the analog blocks on the bus.
 */
struct ESP32P4I2CAnaMstState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ESP32P4AnaState *ana;
    Clock *bus;
    Clock *lp_fast;
    Clock *pll_f160m;

    uint32_t regs[ESP32P4_I2C_ANA_MST_NREGS];

    /* Each master's transaction in progress. */
    bool busy[ESP32P4_I2C_ANA_MST_MASTERS];
    QEMUTimer *timer[ESP32P4_I2C_ANA_MST_MASTERS];
    /* Cycles of the timing clock still to go, counted at xfer_mark. */
    uint64_t xfer_left[ESP32P4_I2C_ANA_MST_MASTERS];
    int64_t xfer_mark[ESP32P4_I2C_ANA_MST_MASTERS];
    /* The timing clock's period since xfer_mark; 0 while it is stopped. */
    uint64_t xfer_period[ESP32P4_I2C_ANA_MST_MASTERS];
    ESP32P4I2CAnaMstTimer timer_ctx[ESP32P4_I2C_ANA_MST_MASTERS];
};

#endif
