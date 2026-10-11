/*
 * ESP32-P4 LP analog peripherals (LP_ANALOG_PERI): the brown-out detector,
 * the glitch detectors and the power supply detector
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LP_ANA_H
#define HW_MISC_ESP32P4_LP_ANA_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_ana.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_ANA "esp32p4-lp-ana"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPAnaState, ESP32P4_LP_ANA)

#define ESP32P4_LP_ANA_BASE     0x50113000
#define ESP32P4_LP_ANA_SIZE     0x1000
#define ESP32P4_LP_ANA_NREGS    (ESP32P4_LP_ANA_SIZE / 4)

/* The touch sensor's configuration registers, TOUCH_* (0xFC to 0x1FC). */
#define ESP32P4_LP_ANA_TOUCH_FIRST 0x0fc
#define ESP32P4_LP_ANA_TOUCH_LAST  0x1fc

typedef struct ESP32P4TouchState ESP32P4TouchState;

/*
 * Inputs and outputs:
 *
 *   - Clock input "fast": LP_DYN_FAST_CLK, which the brown-out counter and
 *     the battery's under-voltage filters count.
 *   - Links "clkrst", which carries out the detectors' resets and records
 *     their causes; "ana", whose BIAS registers hold the thresholds;
 *     "efuse", whose EFUSE_PVT_GLITCH_EN enables the power supply detector;
 *     and "touch", the touch sensor, which the TOUCH_* registers configure
 *     and which is told of each write to them
 *     (esp32p4_touch_ana_write()) and of their reset.
 *   - Properties "vdda-mv" and "vbat-mv": the board's VDD_ANA and VDD_BAT
 *     supplies in millivolts, 3300 each by default, settable with -global
 *     before the machine starts and with qom-set while it runs.
 *   - Sysbus IRQ 0: LP_ANAPERI_INTR, interrupt matrix source 8, from the
 *     INT_* registers.
 *   - Named GPIO outputs: "lp-int", the LP_INT_* registers' interrupt, for
 *     the LP core's interrupt controller; "brownout", high while the
 *     detector sees a brown-out, and "vbat-undervoltage", high while
 *     VDDBAT_UNDERVOLTAGE_FLAG is set, the PMU's wakeup sources 14 and 15.
 *   - Named GPIO inputs: "bod-en", LP_SYSTEM's FIB control of the detector
 *     (high, the analog default, until LP_SYSTEM drives it); "vbat-sel",
 *     the PMU's BOD_SOURCE_SEL in its current state, which lets VDD_BAT's
 *     detection through; "bias-update", the analog block's pulse as BIAS
 *     changes; and the glitches of the board's supplies, high for as long
 *     as each lasts: "power-glitch" (VDD, for the power glitch detector),
 *     "clock-glitch" (the crystal, for the clock glitch detector) and
 *     "hp-supply-glitch" (the HP system regulator, for the power supply
 *     detector).
 */
#define ESP32P4_LP_ANA_LP_INT           "lp-int"
#define ESP32P4_LP_ANA_BROWNOUT         "brownout"
#define ESP32P4_LP_ANA_VBAT_UNDERVOLTAGE "vbat-undervoltage"
#define ESP32P4_LP_ANA_BOD_EN           "bod-en"
#define ESP32P4_LP_ANA_VBAT_SEL         "vbat-sel"
#define ESP32P4_LP_ANA_BIAS_UPDATE      "bias-update"
#define ESP32P4_LP_ANA_POWER_GLITCH     "power-glitch"
#define ESP32P4_LP_ANA_CLOCK_GLITCH     "clock-glitch"
#define ESP32P4_LP_ANA_HP_SUPPLY_GLITCH "hp-supply-glitch"

/* VDD_BAT's two under-voltage filters (TRM figure 25.4-1). */
enum {
    ESP32P4_LP_ANA_FILTER_VBAT,
    ESP32P4_LP_ANA_FILTER_CHARGER,
    ESP32P4_LP_ANA_FILTERS,
};

typedef struct ESP32P4LPAnaFilter {
    struct ESP32P4LPAnaState *s;
    QEMUTimer timer;
    /*
     * The count as of LP_DYN_FAST_CLK's cycle @synced, counted from when
     * the filter started counting or the clock changed (@edges), and
     * whether it counts up (1) or down (-1).
     */
    uint32_t count;
    ESP32P4ClkCount edges;
    uint64_t synced;
    int32_t dir;
    bool flag;
} ESP32P4LPAnaFilter;

struct ESP32P4LPAnaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq lp_int;
    qemu_irq brownout_out;
    qemu_irq vbat_undervoltage_out;
    Clock *fast;
    ESP32P4ClkRstState *clkrst;
    ESP32P4AnaState *ana;
    ESP32P4EfuseState *efuse;
    ESP32P4TouchState *touch;

    uint32_t regs[ESP32P4_LP_ANA_NREGS];

    uint32_t vdda_mv;
    uint32_t vbat_mv;
    bool bod_en;
    bool vbat_sel;

    /* The comparators' outputs: below their thresholds. */
    bool vdda_low;
    bool vbat_low;
    bool charger_low;
    bool brownout;

    /*
     * The mode-0 brown-out counter, while bod_counting: bod_base counts and
     * LP_DYN_FAST_CLK's cycles since, counted from when it started or the
     * clock changed (bod_edges); the levels of its interrupt and reset
     * comparators.
     */
    bool bod_counting;
    uint64_t bod_base;
    ESP32P4ClkCount bod_edges;
    QEMUTimer bod_timer;
    bool bod_int;
    bool bod_reset;

    /* Mode 1's reset, a cycle of LP_DYN_FAST_CLK after the brown-out. */
    QEMUTimer mode1_timer;

    ESP32P4LPAnaFilter filter[ESP32P4_LP_ANA_FILTERS];

    /* The power supply detector's view of an HP supply glitch. */
    QEMUTimer psdet_timer;
    bool hp_glitch;

    bool brownout_level;
    bool vbat_undervoltage_level;
};

/* A TOUCH_* register at @offset as stored, for the touch sensor. */
uint32_t esp32p4_lp_ana_touch_reg(ESP32P4LPAnaState *s, hwaddr offset);

#endif
