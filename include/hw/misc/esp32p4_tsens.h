/*
 * ESP32-P4 temperature sensor (LP_TSENS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_TSENS_H
#define HW_MISC_ESP32P4_TSENS_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkwait.h"
#include "qom/object.h"

#define TYPE_ESP32P4_TSENS "esp32p4-tsens"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4TsensState, ESP32P4_TSENS)

#define ESP32P4_TSENS_BASE      0x5012f000
#define ESP32P4_TSENS_SIZE      0x1000
#define ESP32P4_TSENS_NREGS     (0x2c / 4)

/* The ETM tasks, named GPIO input ESP32P4_TSENS_ETM_TASK. */
enum {
    ESP32P4_TSENS_TASK_START_SAMPLE,
    ESP32P4_TSENS_TASK_STOP_SAMPLE,
    ESP32P4_TSENS_ETM_TASKS,
};

/*
 * Inputs and outputs
 * ==================
 *
 *   - Clock input "clk": LP_PERI_CLK as LPPERI_CK_EN_LP_TSENS gates it,
 *     the sensor's only clock (TRM 64.4.2).
 *   - Named GPIO inputs: ESP32P4_TSENS_ETM_TASK, TMPSNSR_TASK_START_SAMPLE
 *     and _STOP_SAMPLE; ESP32P4_TSENS_DAC_UPDATE, a pulse as software
 *     writes the analog sensors' registers on the analog I2C bus, where
 *     I2C_SARADC_TSENS_DAC sets the sensor's range.
 *   - Named GPIO output ESP32P4_TSENS_ETM_EVENT: TMPSNSR_EVT_OVER_LIMIT.
 *   - Sysbus IRQ 0: LP_TSENS_INTR (interrupt matrix source 15).
 *   - Links: "ana", the analog blocks, whose ANA_SENSOR register 6 holds
 *     I2C_SARADC_TSENS_DAC; "efuse", the eFuses, whose
 *     TEMPERATURE_SENSOR field (BLOCK10) holds the sensor's calibration.
 *
 * The die temperature, in thousandths of a degree Celsius, is the
 * property "temp-mc", settable with -global before the machine starts and
 * with qom-set while it runs:
 *
 *   -global esp32p4-tsens.temp-mc=45000
 */
#define ESP32P4_TSENS_ETM_TASK      "etm-task"
#define ESP32P4_TSENS_ETM_EVENT     "etm-event"
#define ESP32P4_TSENS_DAC_UPDATE    "dac-update"

typedef struct ESP32P4AnaState ESP32P4AnaState;
typedef struct ESP32P4EfuseState ESP32P4EfuseState;

struct ESP32P4TsensState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq etm_event;
    Clock *clk;

    ESP32P4AnaState *ana;
    ESP32P4EfuseState *efuse;

    uint32_t regs[ESP32P4_TSENS_NREGS];

    /* Powered, and through its TSENS_XPD_WAIT since power-up. */
    bool powered;
    bool ready;
    ESP32P4ClkWait xpd_wait;

    /*
     * The automatic monitor: whether it samples, the virtual time its
     * sampling period counts from, and its last sample. A sample that
     * would change nothing is not run; the timer runs to the next sample
     * that would, due at next_ns.
     */
    bool sampling;
    int64_t anchor_ns;
    uint32_t prev;
    bool prev_valid;
    QEMUTimer sample_timer;
    int64_t next_ns;

    int32_t temp_mc;
};

#endif
