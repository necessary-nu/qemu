/*
 * ESP32 GPIO matrix, IO_MUX and sigma-delta modulators
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/clock.h"
#include "qemu/notify.h"
#include "hw/gpio/esp32_line.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

REG32(GPIO_STRAP, 0x0038)

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

/*
 * Pads GPIO0-39. GPIO20, GPIO24 and GPIO28-31 are not bonded out (they have
 * registers but no pad); GPIO34-39 are input-only, with no output driver and
 * no pulls.
 */
#define ESP32_GPIO_PIN_COUNT 40
/* GPIO matrix peripheral signals, input and output, numbered 0-255 */
#define ESP32_GPIO_SIG_COUNT 256
/* IO_MUX functions per pad, selected by MCU_SEL */
#define ESP32_IOMUX_FUNC_COUNT 6
#define ESP32_GPIO_SDM_COUNT 8

/*
 * Signal routing between peripheral models and the pads.
 *
 * Peripherals connect by GPIO matrix signal index, the "Signal No." column
 * of TRM table 6.9-1 (ESP-IDF's soc/gpio_sig_map.h):
 *
 * - ESP32_GPIO_SIG_OUT (gpio-in, ESP32_GPIO_SIG_COUNT lines): a peripheral
 *   drives its output signal s on line s, e.g. U0TXD_out on 14.
 * - ESP32_GPIO_SIG_OE (gpio-in, ESP32_GPIO_SIG_COUNT lines): the output
 *   enable of output signal s, for the signals with a peripheral-driven
 *   enable (SPI, EMAC). It is high until driven, which is right for the
 *   signals whose enable is 1'd1.
 * - ESP32_GPIO_SIG_IN (gpio-out, ESP32_GPIO_SIG_COUNT lines): input signal
 *   s as the GPIO matrix or IO_MUX delivers it, e.g. U0RXD_in on 14.
 *
 * Those signals reach the IO_MUX's direct functions too (U0TXD on GPIO1,
 * HSPICLK on GPIO14, ...): a pad whose MCU_SEL selects a function that
 * carries a matrix signal is connected to that signal without the matrix.
 * The IO_MUX functions that carry no matrix signal (SD, HS1/HS2, EMAC,
 * JTAG, CLK_OUTn) have their own lines, indexed pad * ESP32_IOMUX_FUNC_COUNT
 * + function:
 *
 * - ESP32_GPIO_IOMUX_OUT, ESP32_GPIO_IOMUX_OE (gpio-in): the peripheral's
 *   output and output enable for that pad and function.
 * - ESP32_GPIO_IOMUX_IN (gpio-out): the pad's input while the pad has that
 *   function selected (low with its input disabled); while the pad is on
 *   another function, as the function's input type in the datasheet's
 *   IO_MUX table has it: the pad's input still for an I input (the
 *   EMAC's), 1 for an I1 input (the SD/MMC host's HS1_* and HS2_* data
 *   and command, the SDIO slave's SD_* but its clocks and SD_DATA3 on
 *   GPIO10, MTDI, MTCK), else 0.
 *
 * Pads, for boards and tests:
 *
 * - ESP32_GPIO_PAD_IN (gpio-in, ESP32_GPIO_PIN_COUNT lines): drive pad n
 *   from outside the chip at the given level, as outside driver 0,
 *   "outside" (a test, or a board's fixed wiring).
 * - ESP32_GPIO_PAD_RELEASE (gpio-in): a high level stops driver 0 driving
 *   pad n.
 * - esp32_gpio_ext_driver() (below): another driver outside the chip, a
 *   board's chip or connector (a serial port's terminal), with lines of
 *   its own to drive pads and release them.
 * - A driver, outside the chip or the pad's own output driver (output
 *   enabled, push-pull, or open drain driving low), wins over every pull.
 *   Pad n takes the level of its drivers while they agree; drivers at
 *   different levels make a divider of like strengths, each a strong
 *   pull to its rail: the pad is mid-rail, between VIL and VIH, its level
 *   undefined, and it reads low, logged as a guest error naming the
 *   drivers.
 * - ESP32_GPIO_PAD_OUT (gpio-out): the level on pad n.
 * - ESP32_GPIO_BOARD_PULL_UP, ESP32_GPIO_BOARD_PULL_DOWN (gpio-in): a
 *   resistor the board has from pad n to the pad's supply, or to ground,
 *   the level its resistance in ohms (0: none). With the pad's own 45 kOhm
 *   pulls they make a divider (hw/gpio/esp32_gpio.c).
 *
 * The RTC pads, from the RTC IO MUX (RTCIO), indexed by GPIO number:
 *
 * - ESP32_GPIO_RTC_MUX (gpio-in): high hands the pad to the RTC IO MUX, its
 *   MUX_SEL. The IO_MUX's input enable and the GPIO matrix's output then
 *   no longer apply to it; instead
 * - ESP32_GPIO_RTC_OUT, ESP32_GPIO_RTC_OE, ESP32_GPIO_RTC_IE (gpio-in)
 *   drive it.
 * - ESP32_GPIO_RTC_PU, ESP32_GPIO_RTC_PD (gpio-in): the RTC pad's RUE and
 *   RDE, its pulls whatever MUX_SEL says (the IO_MUX's FUN_WPU and FUN_WPD
 *   do nothing on these pads, errata [GPIO-3.6]).
 * - ESP32_GPIO_RTC_IN (gpio-out): the pad's input as its input buffer
 *   delivers it, whoever controls the pad; low while the buffer is off.
 * - ESP32_GPIO_WAKEUP (gpio-out): a pad with GPIO_PINn_WAKEUP_ENABLE is at
 *   the level its level interrupt type names, the light-sleep GPIO wakeup.
 *
 * The block's interrupt outputs are its sysbus IRQs, in the order of
 * Esp32GpioIrq.
 */
#define ESP32_GPIO_SIG_OUT      "esp32-gpio-sig-out"
#define ESP32_GPIO_SIG_OE       "esp32-gpio-sig-oe"
#define ESP32_GPIO_SIG_IN       "esp32-gpio-sig-in"
#define ESP32_GPIO_IOMUX_OUT    "esp32-iomux-func-out"
#define ESP32_GPIO_IOMUX_OE     "esp32-iomux-func-oe"
#define ESP32_GPIO_IOMUX_IN     "esp32-iomux-func-in"
#define ESP32_GPIO_PAD_IN       "esp32-gpio-pad-in"
#define ESP32_GPIO_PAD_RELEASE  "esp32-gpio-pad-release"
#define ESP32_GPIO_PAD_OUT      "esp32-gpio-pad"
#define ESP32_GPIO_BOARD_PULL_UP   "esp32-gpio-board-pull-up"
#define ESP32_GPIO_BOARD_PULL_DOWN "esp32-gpio-board-pull-down"
#define ESP32_GPIO_RTC_MUX      "esp32-gpio-rtc-mux"
#define ESP32_GPIO_RTC_OUT      "esp32-gpio-rtc-out"
#define ESP32_GPIO_RTC_OE       "esp32-gpio-rtc-oe"
#define ESP32_GPIO_RTC_PU       "esp32-gpio-rtc-pu"
#define ESP32_GPIO_RTC_PD       "esp32-gpio-rtc-pd"
#define ESP32_GPIO_RTC_IE       "esp32-gpio-rtc-ie"
#define ESP32_GPIO_RTC_IN       "esp32-gpio-rtc-in"
#define ESP32_GPIO_WAKEUP       "esp32-gpio-wakeup"

/*
 * GPIO matrix signal indices used by the SoC's connections. A peripheral
 * line with the same number in and out (U0RXD_in and U0TXD_out, ...) shares
 * one constant.
 */
/* SPI0 and SPI1's shared lines, the flash's: SPICLK, SPIQ, SPID, ... */
#define ESP32_SIG_SPICLK        0
#define ESP32_SIG_SPIQ          1
#define ESP32_SIG_SPID          2
#define ESP32_SIG_SPIHD         3
#define ESP32_SIG_SPIWP         4
#define ESP32_SIG_SPICS0        5
#define ESP32_SIG_HSPICS0       11
#define ESP32_SIG_U0RXD_IN      14
#define ESP32_SIG_U0TXD_OUT     14
#define ESP32_SIG_U0CTS_IN      15
#define ESP32_SIG_U0RTS_OUT     15
#define ESP32_SIG_U0DSR_IN      16
#define ESP32_SIG_U0DTR_OUT     16
#define ESP32_SIG_U1RXD_IN      17
#define ESP32_SIG_U1TXD_OUT     17
#define ESP32_SIG_U1CTS_IN      18
#define ESP32_SIG_U1RTS_OUT     18
#define ESP32_SIG_I2CEXT0_SCL   29
#define ESP32_SIG_I2CEXT0_SDA   30
#define ESP32_SIG_PWM0_SYNC0_IN 31
#define ESP32_SIG_PWM0_OUT0A    32
#define ESP32_SIG_PWM0_F0_IN    34
#define ESP32_SIG_HSPICS1       61
#define ESP32_SIG_HSPICS2       62
#define ESP32_SIG_VSPICS0       68
#define ESP32_SIG_VSPICS1       69
#define ESP32_SIG_VSPICS2       70
/* LEDC_HS_SIG_OUT0-7, then LEDC_LS_SIG_OUT0-7 */
#define ESP32_SIG_LEDC_HS0      71
#define ESP32_SIG_TWAI_RX       94
/*
 * The SD/MMC host's: in, host_card_detect_n_1 and _2, host_card_write_prt_1
 * and _2, host_card_int_n_1 and _2; out, host_ccmd_od_pullup_en_n,
 * host_rst_n_1 and _2.
 */
#define ESP32_SIG_HOST_CARD_DETECT_N_1  97
#define ESP32_SIG_HOST_CARD_WRITE_PRT_1 99
#define ESP32_SIG_HOST_CARD_INT_N_1     101
#define ESP32_SIG_HOST_CCMD_OD_PULLUP_EN_N 97
#define ESP32_SIG_HOST_RST_N_1          98
#define ESP32_SIG_I2CEXT1_SCL   95
#define ESP32_SIG_I2CEXT1_SDA   96
#define ESP32_SIG_GPIO_SD0_OUT  100
#define ESP32_SIG_TWAI_TX       123
#define ESP32_SIG_TWAI_BUS_OFF  124
#define ESP32_SIG_TWAI_CLKOUT   125
#define ESP32_SIG_U2RXD_IN      198
#define ESP32_SIG_U2TXD_OUT     198
#define ESP32_SIG_U2CTS_IN      199
#define ESP32_SIG_U2RTS_OUT     199
#define ESP32_SIG_PWM1_SYNC0_IN 103
#define ESP32_SIG_PWM1_F0_IN    106
#define ESP32_SIG_PWM1_OUT0A    108
#define ESP32_SIG_PWM0_CAP0_IN  109
#define ESP32_SIG_PWM1_CAP0_IN  112
/* GPIO_FUNCn_OUT_SEL: GPIO_OUT and GPIO_ENABLE drive the pad */
#define ESP32_SIG_GPIO_OUT      256

/*
 * The GPIO interrupt's four sources in the interrupt matrix. Each can only
 * be routed to its own CPU (TRM 8.3.1): source 22 is GPIO_INTERRUPT_PRO on
 * the PRO CPU and GPIO_INTERRUPT_APP on the APP CPU, source 23 the NMIs.
 */
typedef enum Esp32GpioIrq {
    ESP32_GPIO_IRQ_PRO,
    ESP32_GPIO_IRQ_PRO_NMI,
    ESP32_GPIO_IRQ_APP,
    ESP32_GPIO_IRQ_APP_NMI,
    ESP32_GPIO_IRQ_COUNT,
} Esp32GpioIrq;

/* Drivers outside the chip, the "outside" lines' among them */
#define ESP32_GPIO_EXT_DRIVERS  8

typedef struct Esp32GpioState Esp32GpioState;

/* An outside driver's lines, ESP32_GPIO_PIN_COUNT each */
typedef struct Esp32GpioExtDriver {
    Esp32GpioState *s;
    unsigned id;
    qemu_irq *in;
    qemu_irq *release;
} Esp32GpioExtDriver;

struct Esp32GpioState {
    SysBusDevice parent_obj;

    /* GPIO matrix registers, with the sigma-delta block at 0xf00 */
    MemoryRegion iomem;
    /* IO_MUX registers */
    MemoryRegion iomux_iomem;
    /* APB_CLK, which clocks the sigma-delta modulators */
    Clock *apb_clk;
    qemu_irq irq[ESP32_GPIO_IRQ_COUNT];
    qemu_irq pad_out[ESP32_GPIO_PIN_COUNT];
    qemu_irq sig_in_out[ESP32_GPIO_SIG_COUNT];
    qemu_irq iomux_in_out[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];
    qemu_irq rtc_in_out[ESP32_GPIO_PIN_COUNT];
    qemu_irq wakeup_out;

    uint32_t strap_mode;
    /* MTDI's level as the strapping latch took it at the last chip reset */
    bool strap_mtdi;

    /* GPIO matrix registers */
    uint32_t bt_select;
    uint32_t sdio_select;
    uint32_t out;
    uint32_t out1;
    uint32_t enable;
    uint32_t enable1;
    uint32_t status;
    uint32_t status1;
    uint32_t pin[ESP32_GPIO_PIN_COUNT];
    uint32_t cali_conf;
    uint32_t func_in_sel[ESP32_GPIO_SIG_COUNT];
    uint32_t func_out_sel[ESP32_GPIO_PIN_COUNT];

    /* IO_MUX registers, the per-pad ones indexed by GPIO number */
    uint32_t pin_ctrl;
    uint32_t iomux[ESP32_GPIO_PIN_COUNT];

    /*
     * Sigma-delta modulators: GPIO_SIGMADELTAn, and the state of each
     * modulator's accumulator since sdm_anchor_ns, when its current duty,
     * prescaler and APB_CLK took effect.
     */
    uint32_t sdm_reg[ESP32_GPIO_SDM_COUNT];
    uint32_t sdm_cg;
    uint32_t sdm_misc;
    uint32_t sdm_version;
    int64_t sdm_anchor_ns[ESP32_GPIO_SDM_COUNT];
    uint64_t sdm_ticks[ESP32_GPIO_SDM_COUNT];
    uint32_t sdm_acc[ESP32_GPIO_SDM_COUNT];
    uint8_t sdm_out[ESP32_GPIO_SDM_COUNT];

    /*
     * The pads each driver outside the chip drives, and at what levels;
     * of them all, the pads one drives high and those one drives low
     */
    uint64_t ext_driven[ESP32_GPIO_EXT_DRIVERS];
    uint64_t ext_level[ESP32_GPIO_EXT_DRIVERS];
    uint64_t ext_high;
    uint64_t ext_low;
    unsigned ext_drivers;
    char *ext_name[ESP32_GPIO_EXT_DRIVERS];
    Esp32GpioExtDriver ext[ESP32_GPIO_EXT_DRIVERS];
    /* The board's pull resistors on each pad, ohms (0: none) */
    uint32_t board_pull_up[ESP32_GPIO_PIN_COUNT];
    uint32_t board_pull_down[ESP32_GPIO_PIN_COUNT];
    uint8_t sig_out[ESP32_GPIO_SIG_COUNT];
    uint8_t sig_oe[ESP32_GPIO_SIG_COUNT];
    uint8_t iomux_func_out[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];
    uint8_t iomux_func_oe[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];
    /* RTC IO MUX control of the RTC pads, bit n for GPIO n */
    uint64_t rtc_mux;
    uint64_t rtc_out;
    uint64_t rtc_oe;
    uint64_t rtc_pu;
    uint64_t rtc_pd;
    uint64_t rtc_ie;

    /* Levels the block drives, as last computed */
    uint64_t pad_level;
    uint64_t in_level;
    uint64_t rtc_in_level;
    uint8_t sig_in[ESP32_GPIO_SIG_COUNT];
    uint8_t iomux_in[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];
    uint8_t irq_level[ESP32_GPIO_IRQ_COUNT];
    bool wakeup_level;
    /*
     * The pads whose pulls hold them between VIL and VIH, and those whose
     * drivers are at odds, as resolved
     */
    uint64_t undefined;
    uint64_t contended;
    /* Drive every output line on the next update, not only changed ones */
    bool resync;
    bool updating;
    bool update_pending;
    /*
     * The time of the round being resolved, and the changes held back
     * from it, at other times (see hw/core/line-time.h).
     */
    int64_t round_ns;
    LineDeferred deferred;

    /*
     * Told after software or the board changes how signals are routed to
     * pads or how the pads are set up, once the change is resolved.
     */
    NotifierList route_notifiers;
    bool setup_changed;
};

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;

/*
 * Routing queries, for peripherals whose model has a side that is not a
 * line (a QEMU bus): the pads output signal sig drives with their output
 * enabled, and the pad input signal sig takes its level from (-1 when it
 * takes a constant or its default, or the pad's input is disabled).
 * A route notifier is told after every change to the routing or to a
 * pad's set-up (the GPIO matrix's and IO_MUX's registers, RTCIO's pad
 * controls, the board's pulls, an outside driver), once it is resolved.
 */
uint64_t esp32_gpio_sig_out_pads(Esp32GpioState *s, unsigned sig);
int esp32_gpio_sig_in_pad(Esp32GpioState *s, unsigned sig);
void esp32_gpio_add_route_notifier(Esp32GpioState *s, Notifier *n);

/*
 * For a peripheral on IO_MUX functions of its own that drives an output
 * only at times (the SD/MMC host's CMD and DAT, which it drives only
 * while it sends), modelled above single pad transitions: whether pad n
 * carries function func's output while the peripheral enables it (*on:
 * the function selected, OEN_SEL and its inversion, open drain against
 * the pad's pulls, the RTC IO MUX, an outside driver, all as they stand),
 * and whether the chip or an outside driver still drives the pad, and at
 * what *off_level, while the peripheral does not (*off).
 */
void esp32_gpio_iomux_out_path(Esp32GpioState *s, unsigned n, unsigned func,
                               bool *on, bool *off, bool *off_level);

/*
 * Whether pad n's IO_MUX function func takes the pad's input: the pad on
 * that function with its input buffer on.
 */
bool esp32_gpio_iomux_in_selected(Esp32GpioState *s, unsigned n,
                                  unsigned func);

/*
 * For a peripheral on GPIO matrix signals, modelled above single pad
 * transitions, whose output out_sig and input in_sig are one line to a
 * chip on pad n (an SPI flash's data line; -1 for a line that is only an
 * output or only an input), with the peripheral's output enable for
 * out_sig forced on and off:
 *
 * - out_on: pad n carries out_sig's output while the peripheral enables
 *   it, through its IO_MUX function or the matrix, OEN_SEL and its
 *   inversion, open drain against the pulls, the RTC IO MUX and an outside
 *   driver as they stand; out_inv: inverted on the way (OUT_INV_SEL);
 * - pad_level: the pad's level while the peripheral's output is off: an
 *   outside driver, another of the chip's, else its pulls;
 * - in_on: in_sig takes pad n's input, its input enabled, with nothing
 *   driving the pad while the peripheral's output is off, so that the
 *   chip on the pad reaches it; in_inv: inverted on the way (IN_INV_SEL);
 * - in_level: in_sig's level while the peripheral's output is off and the
 *   chip on pad n drives nothing;
 * - in_matrix: in_sig takes pad n's input through the GPIO matrix
 *   (SIG_IN_SEL), which delays it by two APB_CLK cycles (TRM 20.4.2),
 *   rather than through the pad's IO_MUX function.
 */
typedef struct Esp32GpioSigPath {
    bool out_on;
    bool out_inv;
    bool pad_level;
    bool in_on;
    bool in_inv;
    bool in_level;
    bool in_matrix;
} Esp32GpioSigPath;

void esp32_gpio_sig_path(Esp32GpioState *s, unsigned n, int out_sig,
                         int in_sig, Esp32GpioSigPath *p);

/*
 * Levels as the last resolution left them: pad n's IO_MUX function
 * func's input line (ESP32_GPIO_IOMUX_IN), and pad n itself.
 */
bool esp32_gpio_iomux_in_level(Esp32GpioState *s, unsigned n, unsigned func);
bool esp32_gpio_pad_level(Esp32GpioState *s, unsigned n);

/*
 * Whether pad n is driven digitally, from outside the chip or by its own
 * output driver, and if so, at what *level: for the analog blocks, which
 * see such a pad at the supply or ground (low, as it reads, for drivers
 * at odds).
 */
bool esp32_gpio_pad_driven(Esp32GpioState *s, unsigned n, bool *level);

/*
 * Another driver outside the chip, named @name in guest errors: returns
 * its number, d, whose lines esp32_gpio_ext_in(s, d, n) drives pad n at
 * the line's level and esp32_gpio_ext_release(s, d, n) stops driving it
 * at a high level. Driver 0 is ESP32_GPIO_PAD_IN's and _RELEASE's.
 */
unsigned esp32_gpio_ext_driver(Esp32GpioState *s, const char *name);
qemu_irq esp32_gpio_ext_in(Esp32GpioState *s, unsigned d, unsigned n);
qemu_irq esp32_gpio_ext_release(Esp32GpioState *s, unsigned d, unsigned n);

/*
 * The output signal that reaches input signal sig through a pad: the pad
 * the GPIO matrix or IO_MUX connects to sig has its input enabled, is not
 * driven from outside and is driven, push-pull, by output signal *src
 * through the matrix or its IO_MUX function. *inverted says whether the
 * path inverts the level. Returns false if sig has no such source.
 *
 * A peripheral that models a serial link at a coarser grain than single
 * pad transitions, such as I2S passing whole frames, uses this to find
 * which of its own output signals another of its inputs listens to.
 */
bool esp32_gpio_sig_in_source(Esp32GpioState *s, unsigned sig,
                              unsigned *src, bool *inverted);
