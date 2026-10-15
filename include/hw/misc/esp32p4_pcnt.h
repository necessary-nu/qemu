/*
 * ESP32-P4 pulse count controller (PCNT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_PCNT_H
#define HW_MISC_ESP32P4_PCNT_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_PCNT "esp32p4-pcnt"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4PcntState, ESP32P4_PCNT)

#define ESP32P4_PCNT_BASE       0x500c9000
#define ESP32P4_PCNT_SIZE       0x1000
#define ESP32P4_PCNT_REG_SIZE   0x100
#define ESP32P4_PCNT_UNITS      4

/*
 * Each unit's inputs, in this order: sig_ch0, sig_ch1, ctrl_ch0, ctrl_ch1
 * and the clear signal (rst).
 */
#define ESP32P4_PCNT_SIG(ch)    (ch)
#define ESP32P4_PCNT_CTRL(ch)   (2 + (ch))
#define ESP32P4_PCNT_RST        4
#define ESP32P4_PCNT_UNIT_INPUTS 5
#define ESP32P4_PCNT_INPUTS     (ESP32P4_PCNT_UNITS * ESP32P4_PCNT_UNIT_INPUTS)

/*
 * Inputs and outputs:
 *
 *   - Clock input "apb": PCNT_APB_CLK, which samples and filters the
 *     inputs.
 *   - Named GPIO input ESP32P4_PCNT_INPUT, ESP32P4_PCNT_INPUTS lines: line
 *     u * ESP32P4_PCNT_UNIT_INPUTS + k is unit u's input k above, GPIO
 *     matrix input signals 141 + u (sig_ch0), 145 + u (sig_ch1), 149 + u
 *     (ctrl_ch0), 153 + u (ctrl_ch1) and 63 + u (rst).
 *   - Sysbus IRQ 0: PCNT_INTR (interrupt matrix source 111).
 *
 * The PCNT has no ETM events or tasks on the P4.
 */
#define ESP32P4_PCNT_INPUT      "input"

typedef struct ESP32P4PcntUnit {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t conf2;
    uint32_t change_conf;
    int16_t cnt;
    uint32_t status;
    /* The limits in effect, CONF2 as last taken */
    int16_t h_lim;
    int16_t l_lim;
    /* The step comparator's last step point */
    int16_t step_ref;
} ESP32P4PcntUnit;

struct ESP32P4PcntState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *apb_clk;

    ESP32P4PcntUnit unit[ESP32P4_PCNT_UNITS];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t ctrl;
    uint32_t date;

    bool clk_running;
    /*
     * APB_CLK's edges, as the filters count them: as the clock last
     * changed, until it changes again.
     */
    ESP32P4ClkEdges apb_edges;

    /*
     * Per input: the level on the wire, the level past the glitch filter,
     * and when the wire last changed. The two levels differ while the
     * filter is holding a change back; the timer runs to the earliest
     * time such a change matures. The clear signals are not filtered.
     */
    uint8_t raw[ESP32P4_PCNT_INPUTS];
    uint8_t filt[ESP32P4_PCNT_INPUTS];
    int64_t raw_since_ns[ESP32P4_PCNT_INPUTS];
    QEMUTimer filter_timer;
};

#endif
