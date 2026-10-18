/*
 * ESP32 pulse count controller (PCNT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_PCNT_H
#define HW_MISC_ESP32_PCNT_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/gpio/esp32_line.h"
#include "qemu/timer.h"

#define TYPE_ESP32_PCNT "esp32.pcnt"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32PcntState, ESP32_PCNT)

#define ESP32_PCNT_REG_SIZE     0x100
#define ESP32_PCNT_UNIT_COUNT   8
/* Each unit's inputs: sig_ch0, sig_ch1, ctrl_ch0, ctrl_ch1 */
#define ESP32_PCNT_UNIT_INPUTS  4
#define ESP32_PCNT_INPUT_COUNT  (ESP32_PCNT_UNIT_COUNT * ESP32_PCNT_UNIT_INPUTS)

/*
 * The inputs, as gpio-in lines: line u * ESP32_PCNT_UNIT_INPUTS + k is
 * unit u's input k, in the order above. Unit u's inputs are GPIO matrix
 * input signals ESP32_PCNT_SIG(u) + k (TRM table 6.9-1: 39..58 for units
 * 0..4, 71..82 for units 5..7).
 */
#define ESP32_PCNT_INPUT        "esp32-pcnt-input"
#define ESP32_PCNT_SIG(u)       ((u) < 5 ? 39 + 4 * (u) : 71 + 4 * ((u) - 5))

typedef struct Esp32PcntState Esp32PcntState;

/*
 * A signal input's wave (hw/gpio/esp32_line.h): the one the GPIO matrix
 * (link "gpio") connects it to, whether the path inverts it, and whether
 * the input takes it, its output describing its edges, rather than its
 * line.
 */
typedef struct Esp32PcntBulk {
    Esp32PcntState *s;
    unsigned input;
    Notifier taker;
    Esp32LineWave *wave;
    bool inv;
    bool on;
} Esp32PcntBulk;

typedef struct Esp32PcntUnit {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t conf2;
    int16_t cnt;
    uint32_t status;
} Esp32PcntUnit;

struct Esp32PcntState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    /* APB_CLK, which samples and filters the inputs; 0 while gated */
    Clock *apb_clk;

    Esp32PcntUnit unit[ESP32_PCNT_UNIT_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t ctrl;
    uint32_t date;

    /* APB_CLK is running: the block is neither gated nor held in reset */
    bool clk_running;
    /*
     * An edge of APB_CLK, as the filters see its edges: the moment the
     * block last saw the clock change, its edges every period after it.
     */
    int64_t apb_edge_ns;

    /*
     * Per input: the level on the wire, the level past the glitch filter,
     * and when the wire last changed. The two levels differ while the
     * filter is holding a change back; the timer runs to the earliest
     * time such a change matures. A change held back counts the edges of
     * APB_CLK after raw_since_ns that see it, having counted `held`
     * before then: up to a change of the clock, or up to a dip back to
     * the filter's level (raw_since_ns then the dip's start, the levels
     * alike), which the count runs on through unless an edge sees it.
     */
    uint8_t raw[ESP32_PCNT_INPUT_COUNT];
    uint8_t filt[ESP32_PCNT_INPUT_COUNT];
    int64_t raw_since_ns[ESP32_PCNT_INPUT_COUNT];
    uint16_t held[ESP32_PCNT_INPUT_COUNT];
    QEMUTimer filter_timer;

    /*
     * Per input: the level on its line and when the input last took a
     * change from it; and for a signal input taking a wave, the moment
     * it has been brought to (raw, filt and raw_since_ns are as of then).
     */
    uint8_t line[ESP32_PCNT_INPUT_COUNT];
    int64_t line_ns[ESP32_PCNT_INPUT_COUNT];
    int64_t bulk_ns[ESP32_PCNT_INPUT_COUNT];
    Esp32PcntBulk bulk[ESP32_PCNT_INPUT_COUNT];
    /* bulk[].on, for migration */
    bool bulk_on[ESP32_PCNT_INPUT_COUNT];
    DeviceState *gpio;
    Notifier route_notifier;
};

#endif
