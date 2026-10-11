/*
 * ESP32-P4 analog voltage comparators
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ANA_CMPR_H
#define HW_MISC_ESP32P4_ANA_CMPR_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ANA_CMPR "esp32p4-ana-cmpr"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4AnaCmprState, ESP32P4_ANA_CMPR)

#define ESP32P4_ANA_CMPRS       2
/*
 * Comparator n's reference pad, PAD0, is GPIO51 + 2n; its source, PAD1,
 * GPIO52 + 2n.
 */
#define ESP32P4_ANA_CMPR_PAD0   51
#define ESP32P4_ANA_CMPR_PADS   (2 * ESP32P4_ANA_CMPRS)
#define ESP32P4_ANA_CMPR_PAD_LINES (2 * ESP32P4_ANA_CMPR_PADS)

/*
 * Inputs and outputs
 * ==================
 *
 * The comparators have no registers of their own: LP_SYSTEM's
 * PAD_COMP0 and PAD_COMP1 configure them and GPIO_EXT takes their
 * outputs, raising the interrupts and ETM events.
 *
 *   - Named GPIO inputs: ESP32P4_ANA_CMPR_CONFIG (2 lines), LP_SYSTEM's
 *     pulse as software writes PAD_COMPn; ESP32P4_ANA_CMPR_PAD_CHANGE, the
 *     ADC's pulse as an analog pad's voltage is set; ESP32P4_ANA_CMPR_PAD
 *     (8 lines), GPIO51 to GPIO54 held high (line 2k) and low (2k + 1) by
 *     a digital driver, the GPIO block's "pad-drive-high" and
 *     "pad-drive-low".
 *   - Named GPIO output ESP32P4_ANA_CMPR_OUT (2 lines): COMP_OUT, to the
 *     GPIO block's "ana-comp".
 *   - Links: "adc", whose properties hold the pads' voltages
 *     (esp32p4_adc_pad_mv()); "lp-sys", whose PAD_COMPn configure the
 *     comparators.
 *   - Property "vdd-io-mv": VDD_IO_6, the comparators' supply, of which
 *     the internal reference is a tenth a step; 3300 by default.
 */
#define ESP32P4_ANA_CMPR_CONFIG     "config"
#define ESP32P4_ANA_CMPR_PAD_CHANGE "pad-change"
#define ESP32P4_ANA_CMPR_PAD        "pad"
#define ESP32P4_ANA_CMPR_OUT        "out"

typedef struct ESP32P4AdcState ESP32P4AdcState;
typedef struct ESP32P4LPSysState ESP32P4LPSysState;

struct ESP32P4AnaCmprState {
    SysBusDevice parent_obj;

    qemu_irq out[ESP32P4_ANA_CMPRS];
    ESP32P4AdcState *adc;
    ESP32P4LPSysState *lp_sys;
    uint32_t vdd_io_mv;

    bool level[ESP32P4_ANA_CMPRS];
};

#endif
