/*
 * ESP32-P4 GPIO matrix, IO MUX, GPIO_EXT, LP GPIO matrix and LP IO MUX
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_ESP32P4_GPIO_H
#define HW_GPIO_ESP32P4_GPIO_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio_sig.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_GPIO "esp32p4-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4GpioState, ESP32P4_GPIO)

/*
 * The register blocks, the device's MMIO regions in this order: the HP
 * GPIO matrix with GPIO_EXT (sigma-delta, glitch filters, ETM) at 0xF00,
 * the HP IO MUX, the LP GPIO matrix and the LP IO MUX.
 */
enum {
    ESP32P4_GPIO_MMIO_GPIO,
    ESP32P4_GPIO_MMIO_IO_MUX,
    ESP32P4_GPIO_MMIO_LP_GPIO,
    ESP32P4_GPIO_MMIO_LP_IOMUX,
};
#define ESP32P4_GPIO_BASE       0x500e0000
#define ESP32P4_GPIO_SIZE       0x1000
#define ESP32P4_IO_MUX_BASE     0x500e1000
/* IOMUX_MSPI_PIN, the flash and PSRAM pads' block, follows at 0x200. */
#define ESP32P4_IO_MUX_SIZE     0x200
#define ESP32P4_LP_GPIO_BASE    0x5012a000
#define ESP32P4_LP_GPIO_SIZE    0x1000
#define ESP32P4_LP_IOMUX_BASE   0x5012b000
#define ESP32P4_LP_IOMUX_SIZE   0x1000

/*
 * GPIO0 to GPIO54 are pads. The registers have slots for GPIO55 and
 * GPIO56 too, which no package bonds out: they hold what software writes
 * and drive nothing. GPIO0 to GPIO15 are the LP pads, which the LP IO MUX
 * can take from the HP IO MUX.
 */
#define ESP32P4_GPIO_PADS       55
#define ESP32P4_GPIO_SLOTS      57
#define ESP32P4_GPIO_LP_PADS    16
/* HP GPIO matrix signals, input and output, 0 to 255; LP, 0 to 31. */
#define ESP32P4_GPIO_SIGS       256
#define ESP32P4_GPIO_LP_SIGS    32
/* IO_MUX_GPIOn_MCU_SEL functions with something behind them: 0 to 4. */
#define ESP32P4_IOMUX_FUNCS     5
#define ESP32P4_GPIO_SDMS       8
#define ESP32P4_GPIO_GLITCH_FILTERS 8
#define ESP32P4_GPIO_ETM_CHANNELS 8
/* GPIO_STRAP: GPIO38 (bit 0) down to GPIO34 (bit 4). */
#define ESP32P4_GPIO_STRAP_PINS 5
/* Bit 3, GPIO35, set: SPI boot, as GPIO35's internal pull-up gives. */
#define ESP32P4_GPIO_STRAP_SPI_BOOT 0x08

/*
 * How peripherals connect to the pads
 * ===================================
 *
 * A peripheral's pad signals are the "Signal No." of TRM table 10.12-1
 * (HP) or 10.13-1 (LP), ESP32P4_SIG_* and ESP32P4_LP_SIG_* in
 * hw/gpio/esp32p4_gpio_sig.h. The peripheral's model drives its output
 * signals and reads its input signals as qemu_irq levels, and the machine
 * connects them to this device's named GPIOs by those numbers, as the
 * ESP32 machine connects its peripherals to the ESP32 matrix, so that a
 * peripheral model ported from the ESP32 keeps its wiring code and
 * changes only the numbers:
 *
 *   ESP32P4_GPIO_SIG_OUT, "sig-out" (input, ESP32P4_GPIO_SIGS lines): the
 *     peripheral drives output signal s on line s,
 *       qdev_connect_gpio_out_named(uart, "txd", 0,
 *           qdev_get_gpio_in_named(gpio, ESP32P4_GPIO_SIG_OUT,
 *                                  ESP32P4_SIG_UART0_TXD_PAD_OUT));
 *   ESP32P4_GPIO_SIG_OE, "sig-oe" (input): the output enable of output
 *     signal s, for the signals whose enable is the peripheral's ("Output
 *     enable signal when GPIO_FUNCn_OE_SEL = 0" in table 10.12-1: SPI,
 *     I2C, I3C, SD, GMII MDIO, the cores' fast GPIO). It is high until
 *     driven, which is right for the signals whose enable is 1'd1.
 *   ESP32P4_GPIO_SIG_IN, "sig-in" (output): input signal s as the GPIO
 *     matrix or the IO MUX delivers it,
 *       qdev_connect_gpio_out_named(gpio, ESP32P4_GPIO_SIG_IN,
 *           ESP32P4_SIG_UART0_RXD_PAD_IN,
 *           qdev_get_gpio_in_named(uart, "rxd", 0));
 *   ESP32P4_GPIO_LP_SIG_OUT, _LP_SIG_OE, _LP_SIG_IN, "lp-sig-out",
 *     "lp-sig-oe", "lp-sig-in" (ESP32P4_GPIO_LP_SIGS lines): the same for
 *     the LP peripherals' signals through the LP GPIO matrix.
 *
 * The output signals of peripherals that hold them high out of reset
 * (UART TXD, RTS and DTR, SPI chip selects, I2C and I3C lines, TWAI TX,
 * LCD CS, and the LP UART's, LP I2C's and LP SPI CS) start high, so that
 * routing a signal whose block is not modelled does not pull its pad low.
 * Signals the matrix makes itself are not lines: 72 to 79, the
 * sigma-delta modulators; 212 and 213, constant 0 and 1; 244 and 245, the
 * analog comparators (ESP32P4_GPIO_ANA_COMP below); and 250 to 255,
 * output signal n looping back to input signal n.
 *
 * The IO MUX's direct functions (TRM table 10.14-1) that carry a matrix
 * signal (UART0_TXD on GPIO37, SPI2's pads, the EMAC's RMII data, ...)
 * use the same lines: a pad whose MCU_SEL selects such a function is
 * connected to the signal without the matrix, its output enable the
 * peripheral's own, and, while GPIO_SIGn_IN_SEL is clear, input signal s
 * is the pad's level, unfiltered. Those that carry none (JTAG, SD1, the
 * RMII and 50 MHz reference clocks, pad BIST, debug) have lines of their
 * own, indexed ESP32P4_GPIO_IOMUX_LINE(pad, function):
 *
 *   ESP32P4_GPIO_IOMUX_OUT, _IOMUX_OE (input): the block's output and
 *     output enable for that pad and function.
 *   ESP32P4_GPIO_IOMUX_IN (output): the pad's input while its MCU_SEL
 *     selects that function, otherwise low.
 *
 * The HP cores' fast GPIO (cpu_gpio_out, _oen, _in) is matrix signals 214
 * to 221 for core 0 and 222 to 229 for core 1 (CORE_GPIO_*_PAD_*0 to 15,
 * ESP-IDF's dedic_gpio_periph.c), connected by the machine.
 *
 * A peripheral that models a link above the level of single pad
 * transitions (a QEMU SSI or I2C bus) finds its routing with
 * esp32p4_gpio_sig_out_pads() and esp32p4_gpio_sig_in_pad() below, and
 * is told of routing changes through esp32p4_gpio_add_route_notifier().
 * Line changes are timed as hw/gpio/esp32_line.h describes; a source of
 * fast-changing output registers with esp32_line_add_source(), and every
 * sample of the pads by software brings such sources up to the present.
 *
 * Pads, for boards and tests
 * ==========================
 *
 *   ESP32P4_GPIO_PAD_IN, "pad-in" (input, ESP32P4_GPIO_PADS lines): drive
 *     pad n from outside the chip at the level given. An outside driver
 *     wins over the chip's driver and pulls.
 *   ESP32P4_GPIO_PAD_RELEASE, "pad-release" (input): a high level stops
 *     driving pad n from outside.
 *   ESP32P4_GPIO_PAD_OUT, "pad" (output): the level on pad n.
 *   Properties "pad-ext-driven" and "pad-ext-level", masks with bit n for
 *     pad n: the pads driven from outside and their levels, settable
 *     with -global before the machine starts and with qom-set while it
 *     runs, for a board's fixed wiring.
 *   Property "strap-mode": the board's strapping of GPIO34 to GPIO38, in
 *     GPIO_STRAP's layout, which GPIO_STRAP latches at reset for each of
 *     those pads not then driven from outside. The default is SPI boot.
 *
 * Hooks for the blocks around the pads
 * ====================================
 *
 *   Clock inputs "iomux", the HP IO MUX operating clock (HP_SYS_CLKRST's
 *     IOMUX_CLK), which times the pin and glitch filters and the
 *     sigma-delta modulators; and "lp", the LP IO MUX's, which times the
 *     LP pin filters.
 *   ESP32P4_GPIO_RESET, "reset" (input): line 0, high while
 *     HP_SYS_CLKRST_RST_EN_IOMUX holds the HP GPIO matrix, IO MUX and
 *     GPIO_EXT in reset; line 1, LPPERI_RST_EN_LP_IOMUX for the LP ones.
 *   ESP32P4_GPIO_PAD_PULL_UP, "pad-pull-up" (input): a pull-up on pad n
 *     from another block, the USB PHYs' D+ pull-ups on GPIO25 and GPIO27.
 *     GPIO25's is high until driven: the USB Serial/JTAG controller pulls
 *     D+ up out of reset (table 10.14-1, reset "3*").
 *   ESP32P4_GPIO_PAD_HOLD, "pad-hold" (input): pad n held, from
 *     HP_SYSTEM_GPIO_O_HOLD_CTRL*, LP_SYSTEM_REG_PAD_RTC_HOLD_CTRL* and the
 *     PMU's hold-all (TRM 10.9), ORed by whoever drives it. A held pad
 *     keeps the drive, pulls and input enable it had as the hold began.
 *     LP_IOMUX_LP_PAD_HOLD holds LP pads from inside this block.
 *   ESP32P4_GPIO_LIGHT_SLEEP, "light-sleep" (input): high while the PMU
 *     has the chip in light sleep, when pads with SLP_SEL set take their
 *     sleep configuration (TRM table 10.8-1).
 *   ESP32P4_GPIO_ANA_COMP, "ana-comp" (input, 2 lines): the analog
 *     voltage comparators' outputs, matrix output signals 244 and 245,
 *     whose edges GPIO_INT_RAW's COMP<n>_POS, _NEG and _ALL latch.
 *   ESP32P4_GPIO_WAKEUP, "wakeup" (output): the HP GPIO wakeup from light
 *     sleep (TRM 10.4.6.1): a pad with GPIO_PINn_WAKEUP_ENABLE is at the
 *     level its INT_TYPE, 4 or 5, names.
 *   ESP32P4_GPIO_LP_WAKEUP, "lp-wakeup" (output): the LP GPIO wakeup
 *     (TRM 10.4.6.2) for the PMU: a level type's level, or an edge type's
 *     edge until LP_GPIO_PINn_EDGE_WAKEUP_CLR, on a pad with
 *     LP_GPIO_PINn_WAKEUP_ENABLE.
 *   ESP32P4_GPIO_ETM_EVENT, "etm-event" (output, 24 lines): the GPIO ETM
 *     events, line ESP32P4_GPIO_ETM_LINE(channel, ESP32P4_GPIO_ETM_RISE,
 *     _FALL or _ANY), each pulsed on its edge of the pin-filtered input
 *     of the pad GPIO_EXT_ETM_CHn_EVENT_SEL names while EVENT_EN is set.
 *   ESP32P4_GPIO_ETM_TASK, "etm-task" (input, 24 lines): the GPIO ETM
 *     tasks, line ESP32P4_GPIO_ETM_LINE(channel, ESP32P4_GPIO_ETM_SET,
 *     _CLEAR or _TOGGLE): a rising level sets, clears or toggles GPIO_OUT
 *     of each pad whose GPIO_EXT_ETM_TASK_GPIOn_EN is set and whose
 *     _SEL names the channel.
 *   ESP32P4_GPIO_ETM_ZERO_DET, "etm-zero-det" (output, 4 lines): the
 *     analog comparators' ETM events GPIO_EVT_ZERO_DET_POS<n> (line 2n)
 *     and _NEG<n> (line 2n + 1), pulsed on the rising and falling edges
 *     that set COMP<n>_POS_INT_RAW and _NEG_INT_RAW, through the same
 *     GPIO_ZERO_DET<n>_FILTER_CNT filter.
 *
 * The interrupts are the sysbus IRQs, in the order of ESP32P4GpioIrq,
 * each a source of the interrupt matrix; the LP GPIO interrupt reaches
 * the LP core through the matrix's fan-out of its source.
 */
#define ESP32P4_GPIO_SIG_OUT        "sig-out"
#define ESP32P4_GPIO_SIG_OE         "sig-oe"
#define ESP32P4_GPIO_SIG_IN         "sig-in"
#define ESP32P4_GPIO_LP_SIG_OUT     "lp-sig-out"
#define ESP32P4_GPIO_LP_SIG_OE      "lp-sig-oe"
#define ESP32P4_GPIO_LP_SIG_IN      "lp-sig-in"
#define ESP32P4_GPIO_IOMUX_OUT      "iomux-func-out"
#define ESP32P4_GPIO_IOMUX_OE       "iomux-func-oe"
#define ESP32P4_GPIO_IOMUX_IN       "iomux-func-in"
#define ESP32P4_GPIO_PAD_IN         "pad-in"
#define ESP32P4_GPIO_PAD_RELEASE    "pad-release"
#define ESP32P4_GPIO_PAD_OUT        "pad"
#define ESP32P4_GPIO_RESET          "reset"
#define ESP32P4_GPIO_PAD_PULL_UP    "pad-pull-up"
#define ESP32P4_GPIO_PAD_HOLD       "pad-hold"
#define ESP32P4_GPIO_LIGHT_SLEEP    "light-sleep"
#define ESP32P4_GPIO_ANA_COMP       "ana-comp"
#define ESP32P4_GPIO_WAKEUP         "wakeup"
#define ESP32P4_GPIO_LP_WAKEUP      "lp-wakeup"
#define ESP32P4_GPIO_ETM_EVENT      "etm-event"
#define ESP32P4_GPIO_ETM_TASK       "etm-task"
#define ESP32P4_GPIO_ETM_ZERO_DET   "etm-zero-det"

#define ESP32P4_GPIO_IOMUX_LINE(pad, func) \
    ((pad) * ESP32P4_IOMUX_FUNCS + (func))
#define ESP32P4_GPIO_IOMUX_LINES \
    (ESP32P4_GPIO_PADS * ESP32P4_IOMUX_FUNCS)

enum {
    ESP32P4_GPIO_RESET_HP,
    ESP32P4_GPIO_RESET_LP,
    ESP32P4_GPIO_RESETS,
};

/* An ETM channel's events and tasks. */
enum {
    ESP32P4_GPIO_ETM_RISE,
    ESP32P4_GPIO_ETM_FALL,
    ESP32P4_GPIO_ETM_ANY,
};
enum {
    ESP32P4_GPIO_ETM_SET,
    ESP32P4_GPIO_ETM_CLEAR,
    ESP32P4_GPIO_ETM_TOGGLE,
};
#define ESP32P4_GPIO_ETM_LINE(ch, kind) ((ch) * 3 + (kind))
#define ESP32P4_GPIO_ETM_LINES (ESP32P4_GPIO_ETM_CHANNELS * 3)

#define ESP32P4_GPIO_ANA_COMPS 2
#define ESP32P4_GPIO_ETM_ZERO_DET_LINES (ESP32P4_GPIO_ANA_COMPS * 2)

/*
 * The interrupt outputs, sources of the interrupt matrix: GPIO_INTR0 to
 * _3 (74 to 77), each a pad's GPIO_STATUS bit while its GPIO_PINn_INT_ENA
 * bit 0, 1, 3 or 4 is set; GPIO_PAD_COMP_INTR (78); and LP_GPIO_INTR (10).
 */
typedef enum ESP32P4GpioIrq {
    ESP32P4_GPIO_IRQ_INTR0,
    ESP32P4_GPIO_IRQ_INTR1,
    ESP32P4_GPIO_IRQ_INTR2,
    ESP32P4_GPIO_IRQ_INTR3,
    ESP32P4_GPIO_IRQ_PAD_COMP,
    ESP32P4_GPIO_IRQ_LP,
    ESP32P4_GPIO_IRQS,
} ESP32P4GpioIrq;

/*
 * A filter that samples its input on every cycle of its clock and changes
 * its output once enough of the last samples differ from it: the pin
 * filters and the glitch filters.
 */
typedef struct ESP32P4GpioFilter {
    bool in;
    bool out;
    /* Bit 0 the newest sample: 1 for a sample that differed from out. */
    uint64_t window;
    /* The last sample taken, counted in cycles from the clock's anchor. */
    uint64_t tick;
} ESP32P4GpioFilter;

/* An MMIO region's block, ESP32P4_GPIO_MMIO_*, for its accessors. */
typedef struct ESP32P4GpioRegion {
    ESP32P4GpioState *s;
    unsigned block;
} ESP32P4GpioRegion;

struct ESP32P4GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem[4];
    ESP32P4GpioRegion region[4];
    Clock *iomux_clk;
    Clock *lp_clk;
    QEMUTimer filter_timer;
    qemu_irq irq[ESP32P4_GPIO_IRQS];
    qemu_irq pad_out[ESP32P4_GPIO_PADS];
    qemu_irq sig_in_out[ESP32P4_GPIO_SIGS];
    qemu_irq lp_sig_in_out[ESP32P4_GPIO_LP_SIGS];
    qemu_irq iomux_in_out[ESP32P4_GPIO_IOMUX_LINES];
    qemu_irq wakeup_out;
    qemu_irq lp_wakeup_out;
    qemu_irq etm_event[ESP32P4_GPIO_ETM_LINES];
    qemu_irq etm_zero_det[ESP32P4_GPIO_ETM_ZERO_DET_LINES];
    Notifier line_source;
    NotifierList route_notifiers;

    uint32_t strap_mode;

    /* HP GPIO matrix registers */
    uint32_t bt_select;
    uint32_t out[2];
    uint32_t enable[2];
    uint32_t strap;
    uint32_t status[2];
    uint32_t pin[ESP32P4_GPIO_SLOTS];
    uint32_t func_in_sel[ESP32P4_GPIO_SIGS];
    uint32_t func_out_sel[ESP32P4_GPIO_SLOTS];
    uint32_t clock_gate;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t zero_det[ESP32P4_GPIO_ANA_COMPS];
    uint32_t send_seq;
    uint32_t bistin_sel;
    uint32_t bist_ctrl;
    uint32_t date;

    /* HP IO MUX registers */
    uint32_t iomux[ESP32P4_GPIO_SLOTS];
    uint32_t iomux_date;

    /* GPIO_EXT registers */
    uint32_t sdm[ESP32P4_GPIO_SDMS];
    uint32_t sdm_misc;
    uint32_t glitch[ESP32P4_GPIO_GLITCH_FILTERS];
    uint32_t etm_event_cfg[ESP32P4_GPIO_ETM_CHANNELS];
    uint32_t etm_task_cfg[14];
    uint32_t ext_version;

    /* LP GPIO matrix registers */
    uint32_t lp_clk_en;
    uint32_t lp_ver_date;
    uint32_t lp_out;
    uint32_t lp_enable;
    uint32_t lp_status;
    uint32_t lp_pin[ESP32P4_GPIO_LP_PADS];
    uint32_t lp_func_in_sel[ESP32P4_GPIO_LP_SIGS];
    uint32_t lp_func_out_sel[ESP32P4_GPIO_LP_PADS];

    /* LP IO MUX registers */
    uint32_t lp_iomux_clk_en;
    uint32_t lp_iomux_ver_date;
    uint32_t lp_pad[ESP32P4_GPIO_LP_PADS];
    uint32_t lp_ext_wakeup0_sel;
    uint32_t lp_pad_hold;
    uint32_t lp_pad_hys;

    /* The blocks' resets held, ESP32P4_GPIO_RESET_* */
    bool in_reset[ESP32P4_GPIO_RESETS];

    /*
     * Sigma-delta modulators: each one's accumulator and output, as of
     * its anchor, when its settings and clock last changed, plus the
     * ticks taken since.
     */
    int64_t sdm_anchor_ns[ESP32P4_GPIO_SDMS];
    uint64_t sdm_ticks[ESP32P4_GPIO_SDMS];
    uint32_t sdm_acc[ESP32P4_GPIO_SDMS];
    uint8_t sdm_out[ESP32P4_GPIO_SDMS];

    /*
     * The filters, their sample cycles counted from each clock's anchor,
     * when its frequency last changed.
     */
    ESP32P4GpioFilter pin_filter[ESP32P4_GPIO_PADS];
    ESP32P4GpioFilter glitch_filter[ESP32P4_GPIO_GLITCH_FILTERS];
    ESP32P4GpioFilter lp_pin_filter[ESP32P4_GPIO_LP_PADS];
    int64_t iomux_anchor_ns;
    int64_t lp_anchor_ns;

    /* The analog comparators' outputs, and their last interrupts' times. */
    uint8_t ana_comp[ESP32P4_GPIO_ANA_COMPS];
    int64_t ana_comp_int_ns[ESP32P4_GPIO_ANA_COMPS];

    /* Levels driven into the block from outside */
    uint64_t ext_driven;
    uint64_t ext_level;
    uint64_t ext_pull_up;
    uint64_t ext_hold;
    bool light_sleep;
    uint8_t sig_out[ESP32P4_GPIO_SIGS];
    uint8_t sig_oe[ESP32P4_GPIO_SIGS];
    uint8_t lp_sig_out[ESP32P4_GPIO_LP_SIGS];
    uint8_t lp_sig_oe[ESP32P4_GPIO_LP_SIGS];
    uint8_t iomux_func_out[ESP32P4_GPIO_IOMUX_LINES];
    uint8_t iomux_func_oe[ESP32P4_GPIO_IOMUX_LINES];

    /*
     * The held pads, and what they hold: their drive (level and enable,
     * after open drain), pulls and input enable as the hold began.
     */
    uint64_t held;
    uint64_t hold_val;
    uint64_t hold_oe;
    uint64_t hold_pu;
    uint64_t hold_ie;

    /* LP_GPIO_PINn edge wakeups latched, until EDGE_WAKEUP_CLR */
    uint32_t lp_wakeup_edge;

    /* Levels the block computes, as last driven */
    uint64_t pad_level;
    /* Each pad's input buffer: its level while its input is enabled */
    uint64_t raw_in;
    /* The HP chain: the pin filters' outputs, then the glitch filters' */
    uint64_t pf_in;
    uint64_t hp_in;
    /* The LP chain: the LP pin filters' outputs */
    uint32_t lp_in;
    uint8_t sig_in[ESP32P4_GPIO_SIGS];
    uint8_t lp_sig_in[ESP32P4_GPIO_LP_SIGS];
    uint8_t iomux_in[ESP32P4_GPIO_IOMUX_LINES];
    uint8_t irq_level[ESP32P4_GPIO_IRQS];
    bool wakeup_level;
    bool lp_wakeup_level;
    /* Drive every output on the next update, not only those that change */
    bool resync;
    bool updating;
    bool update_pending;
};

/*
 * Routing queries, for peripherals whose model has a side that is not a
 * line (a QEMU bus): the pads output signal sig drives with their output
 * enabled, and the pad input signal sig takes its level from (-1 when it
 * takes a constant or its default, or the pad's input is disabled); and
 * the same for the LP matrix's signals.
 */
uint64_t esp32p4_gpio_sig_out_pads(ESP32P4GpioState *s, unsigned sig);
int esp32p4_gpio_sig_in_pad(ESP32P4GpioState *s, unsigned sig);
uint32_t esp32p4_gpio_lp_sig_out_pads(ESP32P4GpioState *s, unsigned sig);
int esp32p4_gpio_lp_sig_in_pad(ESP32P4GpioState *s, unsigned sig);
void esp32p4_gpio_add_route_notifier(ESP32P4GpioState *s, Notifier *n);

/*
 * Whether pad n is driven digitally, from outside the chip or by its own
 * output driver, and if so, at what *level: for the analog blocks, which
 * see such a pad at the supply or ground.
 */
bool esp32p4_gpio_pad_driven(ESP32P4GpioState *s, unsigned n, bool *level);

/*
 * The output signal that reaches input signal sig through a pad: the pad
 * the GPIO matrix or IO MUX connects to sig has its input enabled, is not
 * driven from outside and is driven, push-pull, by output signal *src
 * through the matrix or its IO MUX function. *inverted says whether the
 * path inverts the level. Returns false if sig has no such source. For a
 * peripheral that models a serial link a frame at a time, to find which
 * of its own outputs one of its inputs listens to.
 */
bool esp32p4_gpio_sig_in_source(ESP32P4GpioState *s, unsigned sig,
                                unsigned *src, bool *inverted);

/*
 * Whether HP pad n's IO MUX selects function func, with its input
 * enabled: for a block whose IO MUX function takes a clock from the pad,
 * which no line carries.
 */
bool esp32p4_gpio_iomux_in_selected(ESP32P4GpioState *s, unsigned n,
                                    unsigned func);

#endif
