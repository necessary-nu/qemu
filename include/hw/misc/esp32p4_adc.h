/*
 * ESP32-P4 SAR ADCs: the HP and LP ADC controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ADC_H
#define HW_MISC_ESP32P4_ADC_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkwait.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ADC "esp32p4-adc"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4AdcState, ESP32P4_ADC)

/*
 * The device's register blocks, its MMIO regions in this order: the HP
 * ADC controller ("ADC Controller" in reg_base.h) and the LP ADC
 * controller.
 */
enum {
    ESP32P4_ADC_MMIO_HP,
    ESP32P4_ADC_MMIO_LP,
};
#define ESP32P4_ADC_BASE        0x500de000
#define ESP32P4_LP_ADC_BASE     0x50127000
#define ESP32P4_ADC_SIZE        0x1000

#define ESP32P4_ADC_UNITS       2
#define ESP32P4_ADC1_CHANNELS   8
#define ESP32P4_ADC2_CHANNELS   6
/* A SAR ADC's input multiplexer: 8 positions, ADC2's channels at 2 to 7. */
#define ESP32P4_ADC_MUX_POS     8
#define ESP32P4_ADC_CHANNELS    (ESP32P4_ADC1_CHANNELS + ESP32P4_ADC2_CHANNELS)

/* The two filters and two threshold monitors of the HP controllers. */
#define ESP32P4_ADC_FILTERS     2
#define ESP32P4_ADC_MONITORS    2

#define ESP32P4_ADC_HP_NREGS    (0x400 / 4)
#define ESP32P4_ADC_LP_NREGS    (0x74 / 4)

/*
 * The ETM events, named GPIO output ESP32P4_ADC_ETM_EVENT, in the order
 * of soc_etm_source.h from ADC_EVT_CONV_CMPLT0, less ADC_EVT_RESULT_DONE0,
 * which the TRM does not describe and the model does not raise.
 */
enum {
    ESP32P4_ADC_EVT_CONV_CMPLT0,
    ESP32P4_ADC_EVT_EQ_ABOVE_THRESH0,
    ESP32P4_ADC_EVT_EQ_ABOVE_THRESH1,
    ESP32P4_ADC_EVT_EQ_BELOW_THRESH0,
    ESP32P4_ADC_EVT_EQ_BELOW_THRESH1,
    ESP32P4_ADC_EVT_STOPPED0,
    ESP32P4_ADC_EVT_STARTED0,
    ESP32P4_ADC_ETM_EVENTS,
};

/* The ETM tasks, named GPIO input ESP32P4_ADC_ETM_TASK, from 157. */
enum {
    ESP32P4_ADC_TASK_SAMPLE0,
    ESP32P4_ADC_TASK_SAMPLE1,
    ESP32P4_ADC_TASK_START0,
    ESP32P4_ADC_TASK_STOP0,
    ESP32P4_ADC_ETM_TASKS,
};

/*
 * Inputs and outputs
 * ==================
 *
 *   - Clock inputs: "adc", HPADC_CLK, the HP controllers' function
 *     clock; "lp-adc", LPADC_CLK, the LP controllers'; "lp-adc-sar1" and
 *     "lp-adc-sar2", the LP readers' SAR clocks.
 *   - Named GPIO inputs: ESP32P4_ADC_HP_RESET, high while
 *     HP_SYS_CLKRST_RST_EN_ADC holds the HP controllers in reset;
 *     ESP32P4_ADC_LP_RESET, LPPERI's RST_EN_LP_ADC, the LP controllers';
 *     ESP32P4_ADC_ETM_TASK, the ETM tasks above, each acting on a rising
 *     level at the time its pulse carries.
 *   - Named GPIO outputs: ESP32P4_ADC_ETM_EVENT, the ETM events above,
 *     each a pulse at the event's virtual time; ESP32P4_ADC_PAD_CHANGE, a
 *     pulse whenever an analog pad's voltage is set, for the blocks that
 *     sample the pads (the analog comparators).
 *   - Sysbus IRQs: 0, ADC_INTR (interrupt matrix source 37), the HP
 *     controllers'; 1, LP_ADC_INTR (source 9), the LP controllers'.
 *   - Links: "gdma", GDMA-AHB, which serves the HP controllers' DMA as
 *     peripheral 8; "ana", the analog blocks on the analog I2C bus, whose
 *     ANA_SENSOR registers hold the SARs' initial codes and calibration
 *     switches; "efuse", the eFuse controller, whose BLOCK2 and BLOCK10
 *     hold the chip's ADC calibration; "gpio", the pads, which a digital
 *     driver holds at the supply or ground.
 *
 * Analog inputs
 * =============
 *
 * The voltage on each ADC channel's pad, in millivolts, is a property:
 * "adc1-ch0-mv" to "adc1-ch7-mv" (GPIO16 to GPIO23) and "adc2-ch0-mv" to
 * "adc2-ch5-mv" (GPIO49 to GPIO54), settable with -global before the
 * machine starts and with qom-set while it runs:
 *
 *   -global esp32p4-adc.adc1-ch3-mv=1200
 *
 * GPIO51 to GPIO54 are also the analog comparators' pads
 * (esp32p4_adc_pad_mv()).
 */
#define ESP32P4_ADC_HP_RESET    "hp-reset"
#define ESP32P4_ADC_LP_RESET    "lp-reset"
#define ESP32P4_ADC_ETM_TASK    "etm-task"
#define ESP32P4_ADC_ETM_EVENT   "etm-event"
#define ESP32P4_ADC_PAD_CHANGE  "pad-change"

typedef struct ESP32P4GdmaState ESP32P4GdmaState;
typedef struct ESP32P4AnaState ESP32P4AnaState;
typedef struct ESP32P4EfuseState ESP32P4EfuseState;
typedef struct ESP32P4GpioState ESP32P4GpioState;

/* Who is converting on a SAR ADC. */
typedef enum ESP32P4AdcOwner {
    ESP32P4_ADC_IDLE,
    ESP32P4_ADC_BY_HP,
    ESP32P4_ADC_BY_LP,
} ESP32P4AdcOwner;

/* A SAR ADC and its two controllers' state. */
typedef struct ESP32P4AdcUnit {
    /* The conversion in progress and its result. */
    uint8_t owner;
    uint32_t code;
    /* The HP FSM's: its pattern pointer and the entry converting. */
    uint8_t patt_ptr;
    uint8_t hp_pos;
    bool hp_pending;
    /* The LP controller's: a start waiting for the SAR, and its origin. */
    bool lp_pending;
    bool lp_pending_hw;
    bool lp_hw;
    /* The LP automatic monitor's last sample, for its change mode. */
    uint32_t hw_prev;
    bool hw_prev_valid;
    /* The SAR's conversion; the LP monitor's sampling timer. */
    ESP32P4ClkWait hp_conv;
    ESP32P4ClkWait lp_conv;
    ESP32P4ClkWait hw_timer;
} ESP32P4AdcUnit;

struct ESP32P4AdcState {
    SysBusDevice parent_obj;

    MemoryRegion hp_iomem;
    MemoryRegion lp_iomem;
    qemu_irq hp_irq;
    qemu_irq lp_irq;
    qemu_irq etm_event[ESP32P4_ADC_ETM_EVENTS];
    qemu_irq pad_change;
    Clock *clk;
    Clock *lp_clk;
    Clock *lp_sar_clk[ESP32P4_ADC_UNITS];

    ESP32P4GdmaState *gdma;
    ESP32P4AnaState *ana;
    ESP32P4EfuseState *efuse;
    ESP32P4GpioState *gpio;

    uint32_t hp_regs[ESP32P4_ADC_HP_NREGS];
    uint32_t lp_regs[ESP32P4_ADC_LP_NREGS];
    bool hp_in_reset;
    bool lp_in_reset;

    ESP32P4AdcUnit unit[ESP32P4_ADC_UNITS];

    /*
     * The HP controllers' sampling: whether the timer runs, the samples
     * taken since it started, which FSM the next alternate-mode request
     * goes to, and the samples since the DMA's last EOF.
     */
    bool hp_running;
    uint32_t meas_count;
    uint8_t alt_unit;
    uint32_t eof_count;
    ESP32P4ClkWait hp_timer;
    /* The filters' last outputs, valid once they have had a sample. */
    uint32_t filter_val[ESP32P4_ADC_FILTERS];
    bool filter_valid[ESP32P4_ADC_FILTERS];
    /* An HP request refused because the SAR is not the HP side's. */
    bool hp_denied_logged;

    /* The analog inputs: each channel's pad voltage in millivolts. */
    uint32_t pad_mv[ESP32P4_ADC_CHANNELS];
};

/*
 * The voltage, in millivolts, on analog pad @gpio as the analog blocks
 * see it: the supply or ground where a digital driver holds the pad, else
 * the voltage set from QEMU for an ADC channel's pad, else 0.
 */
uint32_t esp32p4_adc_pad_mv(ESP32P4AdcState *s, unsigned gpio);

#endif
