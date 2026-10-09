/*
 * RP2350 analogue-digital converter (ADC) and temperature sensor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ADC_RP2350_ADC_H
#define HW_ADC_RP2350_ADC_H

#include "hw/core/sysbus.h"
#include "hw/gpio/rp2350_gpio.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_ADC "rp2350-adc"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350ADCState, RP2350_ADC)

#define RP2350_ADC_BASE 0x400a0000

/*
 * External inputs: four on GPIO 26-29 in the QFN-60 RP2350A, eight on
 * GPIO 40-47 in the QFN-80 RP2350B. The temperature sensor is the channel
 * after the last external one.
 */
#define RP2350_ADC_MAX_EXT 8
#define RP2350_ADC_FIFO_DEPTH 8

/*
 * Inputs and outputs:
 *
 *  - Sysbus IRQ 0: ADC_IRQ_FIFO.
 *  - Named GPIO output "dreq": DREQ_ADC, held high while FCS.DREQ_EN is
 *    set and the FIFO holds at least FCS.THRESH samples.
 *  - Clock input "clk": clk_adc, 48 MHz for the specified 96-cycle,
 *    2 us conversion. No conversions happen while it is stopped.
 *  - Link "gpio": the pads the external inputs share with GPIOs.
 *
 * QOM properties, settable at any time (qom-set), give the analogue world
 * outside the chip:
 *
 *  - "ain<n>-uv": the voltage, in microvolts, a source outside the chip
 *    applies to external input n's pin. A pin driven digitally, by the
 *    chip's own output driver or through the GPIO block's "pad-in", is at
 *    0 V or IOVDD instead.
 *  - "temperature": the die temperature, in millidegrees Celsius.
 *  - "avdd-uv": ADC_AVDD, the ADC's reference; "iovdd-uv": IOVDD, the
 *    level of a pin driven high.
 *  - "conversion-errors": a mask of channels whose conversions fail to
 *    converge (CS.ERR), as comparator metastability makes them on
 *    hardware.
 */
#define RP2350_ADC_DREQ "dreq"

struct RP2350ADCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;
    qemu_irq irq;
    qemu_irq dreq;
    QEMUTimer *timer;
    RP2350GPIOState *gpio;

    /* Properties: the package's external inputs (4 or 8). */
    uint32_t ext_channels;

    /* The analogue inputs (QOM properties). */
    int32_t ain_uv[RP2350_ADC_MAX_EXT];
    int32_t temp_mc;
    uint32_t avdd_uv;
    uint32_t iovdd_uv;
    uint32_t err_channels;

    /* CS: EN, TS_EN, START_MANY, AINSEL, RROBIN, ERR, ERR_STICKY. */
    uint32_t cs;
    uint32_t result;
    /* FCS: EN, SHIFT, ERR, DREQ_EN, THRESH, OVER, UNDER. */
    uint32_t fcs;
    uint16_t fifo[RP2350_ADC_FIFO_DEPTH];
    uint32_t fifo_head;
    uint32_t fifo_level;
    uint32_t div;
    uint32_t inte;
    uint32_t intf;

    /* clk_adc cycle count at virtual time base_ns. */
    int64_t base_ns;
    uint64_t base_cycle;

    /*
     * The conversion in progress: its channel, the code it samples and
     * whether it fails to converge, and the cycle it completes in.
     */
    bool busy;
    uint32_t conv_ch;
    uint32_t conv_code;
    bool conv_err;
    uint64_t conv_end;
    /*
     * Free-running sampling: the pacing timer's ticks fall DIV apart from
     * pace_base; idle_since is the cycle the ADC last became ready.
     */
    uint64_t pace_base;
    uint64_t idle_since;

    /*
     * Channels whose misuse (digital input enabled, temperature sensor
     * unbiased) has been reported, so it is reported once.
     */
    uint32_t reported;
};

#endif
