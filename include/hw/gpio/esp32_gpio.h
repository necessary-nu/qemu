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
 *   function selected, otherwise low.
 *
 * Pads, for boards and tests:
 *
 * - ESP32_GPIO_PAD_IN (gpio-in, ESP32_GPIO_PIN_COUNT lines): drive pad n
 *   from outside the chip at the given level. An outside driver overrides
 *   the chip's own driver and pulls.
 * - ESP32_GPIO_PAD_RELEASE (gpio-in): a high level stops driving pad n
 *   from outside.
 * - ESP32_GPIO_PAD_OUT (gpio-out): the level on pad n.
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

/* GPIO matrix signal indices used by the SoC's connections */
#define ESP32_SIG_SPICS0        5
#define ESP32_SIG_HSPICS0       11
#define ESP32_SIG_U0RXD_IN      14
#define ESP32_SIG_U0TXD_OUT     14
#define ESP32_SIG_HSPICS1       61
#define ESP32_SIG_HSPICS2       62
#define ESP32_SIG_VSPICS0       68
#define ESP32_SIG_VSPICS1       69
#define ESP32_SIG_VSPICS2       70
#define ESP32_SIG_GPIO_SD0_OUT  100
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

typedef struct Esp32GpioState {
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

    uint32_t strap_mode;

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

    /* Levels driven into the block from outside */
    uint64_t ext_driven;
    uint64_t ext_level;
    uint8_t sig_out[ESP32_GPIO_SIG_COUNT];
    uint8_t sig_oe[ESP32_GPIO_SIG_COUNT];
    uint8_t iomux_func_out[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];
    uint8_t iomux_func_oe[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];

    /* Levels the block drives, as last computed */
    uint64_t pad_level;
    uint64_t in_level;
    uint8_t sig_in[ESP32_GPIO_SIG_COUNT];
    uint8_t iomux_in[ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT];
    uint8_t irq_level[ESP32_GPIO_IRQ_COUNT];
    /* Drive every output line on the next update, not only changed ones */
    bool resync;
    bool updating;
    bool update_pending;
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;
