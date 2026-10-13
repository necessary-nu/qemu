/*
 * ESP32 SD/MMC host controller (SDHOST) on its IO_MUX pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, chapters 6 (IO_MUX and GPIO Matrix) and 27
 * (SD/MMC Host Controller); the ESP32 Series Datasheet's IO_MUX table
 * (appendix A) and its section 4.10, Peripheral Pin Configurations;
 * ESP-IDF's esp_hal_sd sdmmc_periph.c and sdmmc_ll.h for the ESP32, and
 * esp_driver_sdmmc's sd_host_sdmmc.c, sdmmc_slot_io_config().
 *
 * [spec:nuos:req:emu.esp32.gpio]
 * The ESP32's SDHOST reaches its cards through IO_MUX functions only: its
 * CLK, CMD and DAT lines carry no GPIO matrix signal (TRM table 6.9-1 has
 * none; ESP-IDF's soc_caps.h has SOC_SDMMC_USE_IOMUX and no
 * SOC_SDMMC_USE_GPIO_MATRIX, its sdmmc_ll.h SDMMC_LL_SLOT_SUPPORT_GPIO_
 * MATRIX() 0 for both slots, and sdmmc_slot_io_config() refuses other
 * pins: "doesn't support routing from GPIO matrix, driver uses dedicated
 * IOs"). Its card detect, write protect and interrupt inputs and its
 * card reset and CMD open-drain pull-up outputs are matrix signals 97 to
 * 102 (hw/xtensa/esp32.c connects them). The slots' pads, function 3 of
 * each (TRM table 6.10-1; ESP-IDF's SDMMC_LL_IOMUX_FUNC 3):
 *
 *   slot 0 (HS1_*, 8 bits): CLK GPIO6, CMD GPIO11, DAT0 to DAT3 GPIO7 to
 *     GPIO10, DAT4 GPIO16, DAT5 GPIO17, DAT6 GPIO5, DAT7 GPIO18;
 *   slot 1 (HS2_*, 4 bits): CLK GPIO14, CMD GPIO15, DAT0 GPIO2, DAT1
 *     GPIO4, DAT2 GPIO12, DAT3 GPIO13.
 *
 * Their types in the datasheet's IO_MUX table: HS*_CLK O, HS*_CMD and
 * HS*_DATAn I1/O/T, an input reading 1 while its pad is on another
 * function (hw/gpio/esp32_gpio.c). A line joins the host to its card as
 * the pads between them are set up:
 *
 *   - the host's output reaches the card's pin while the pad takes it
 *     when the host drives it: the pad on function 3 (MCU_SEL), not handed
 *     to the RTC IO MUX (MUX_SEL; slot 1's pads are all RTC pads), its
 *     output enable the host's (OEN_SEL clear) or, with OEN_SEL set,
 *     GPIO_ENABLE's as OEN_INV_SEL leaves it; an open-drain pad
 *     (GPIO_PINn_PAD_DRIVER) only where its pulls hold it high; no
 *     outside driver on it;
 *   - the card's output reaches the host's input while the pad is on
 *     function 3 with its input enabled (FUN_IE, or RTCIO's FUN_IE with
 *     MUX_SEL set), nothing else driving it while the host does not;
 *   - an input that does not take the card reads what the GPIO block
 *     gives it throughout: 0 with its input disabled on function 3, 1 off
 *     it; and a pin the host's output does not reach sees its pad's
 *     level: a GPIO output, an outside driver, the pulls;
 *   - a line the card releases reads as the pad's pulls hold it: its own
 *     45 kOhm pull-up and pull-down, RTCIO's RUE and RDE on slot 1's pads
 *     (errata [GPIO-3.6]: their IO_MUX FUN_WPU and FUN_WPD do nothing),
 *     and the board's ("sd-pull-up" on the machine); with none it floats,
 *     and reads low.
 *
 * hw/sd/dwc_sdmmc.c has what each makes of the bus: a command whose CLK
 * or CMD does not reach the card goes unanswered, RTO where CMD reads
 * high, a response of 0s (RE) where it reads low, as with CMD's input
 * disabled or CMD floating; DAT0 read low holds the card busy
 * (STATUS.DATA_BUSY, a WAIT_PRVDATA command never taken); DAT0 read high
 * hides a read block's start bit (DRTO); another DAT line read low or
 * high fails a read block's end bit (EBE) or CRC (DCRC); a write block
 * with a DAT line not reaching the card fails the card's CRC check: the
 * card discards it (DCRC), in a multiple block write the rest too.
 *
 * ESP-IDF sets a slot up (sdmmc_slot_io_config(), configure_pin_iomux())
 * with each pin's pull-down disabled, its input enabled and function 3
 * selected with OEN_SEL clear (gpio_iomux_output()), the drive strength
 * left at 2 on the ESP32; with SDMMC_SLOT_FLAG_INTERNAL_PULLUP it enables
 * the pads' pull-ups on CMD and DAT (for slot 1's RTC pads through
 * RTCIO), which its documentation calls no substitute for the board's
 * 10 kOhm ones. In 4-bit Default and High Speed mode it holds DAT3 high
 * as a GPIO output until it sets the bus width, so that the card enters
 * SD mode, then gives DAT3 to the host. It routes the card detect input
 * to a constant 0 (present), the write protect to a constant 1,
 * inverted (writable), and the interrupt input to a constant 1.
 *
 * The board's card: on an ESP32-WROOM-32E, GPIO6 to GPIO11 are wired to
 * the module's flash and are not led out (module datasheet, note 2 to its
 * pin table: pins 17 to 22 are NC), so no card can be wired to slot 0's
 * CLK, CMD and DAT0 to DAT3; the machine takes no card in slot 0, and a
 * board's socket is on slot 1, where ESP-IDF looks for it by default.
 * Slot 0's host inputs read its pads all the same (the flash's lines).
 *
 * Not modelled: the pads' drive strength (FUN_DRV, or RTCIO's DRV for
 * slot 1's pads), which the datasheet gives only as currents (note 8),
 * not as edge rates into the card's load; the IO_MUX's light-sleep
 * settings (MCU_*, SLP_SEL) and the hold of a pad under the IO_MUX's
 * control, which hw/gpio/esp32_gpio.c and hw/gpio/esp32_rtcio.c do not
 * model; and the card's own 50 kOhm DAT3 pull-up (the SD Physical Layer
 * Specification's card detect resistor), which the card model does not
 * give its pin.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/sd/esp32_sdmmc.h"
#include "hw/gpio/esp32_line.h"

static const uint8_t esp32_sdmmc_pads[DWC_SDMMC_MAX_SLOTS][DWC_SDMMC_LINES] = {
    ESP32_SDMMC_SLOT0_PADS,
    ESP32_SDMMC_SLOT1_PADS,
};

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Slot @slot's lines as the IO_MUX and GPIO matrix join the host and its
 * card: see above. A line the slot has no pad for (slot 1's DAT4 to DAT7)
 * reaches nothing, and the host's input reads 1 on it.
 */
static void esp32_sdmmc_lines(DWCSDMMCState *d, unsigned slot,
                              DWCSDMMCLines *l)
{
    Esp32SdmmcState *s = ESP32_SDMMC(d);
    Esp32GpioState *g = s->gpio;

    if (!g) {
        return;
    }
    l->to_card = 0;
    l->to_host = 0;
    l->host_level = 0;
    l->card_level = 0;
    for (unsigned i = 0; i < DWC_SDMMC_LINES; i++) {
        unsigned pad = esp32_sdmmc_pads[slot][i];
        uint16_t bit = 1u << i;
        bool on, off, off_level;

        if (pad == ESP32_SDMMC_NO_PAD) {
            l->host_level |= bit;
            l->card_level |= bit;
            continue;
        }
        esp32_gpio_iomux_out_path(g, pad, ESP32_SDMMC_FUNC, &on, &off,
                                  &off_level);
        l->to_card |= on ? bit : 0;
        l->card_level |= esp32_gpio_pad_level(g, pad) ? bit : 0;
        if (bit == DWC_SDMMC_LINE_CLK) {
            continue;
        }
        if (esp32_gpio_iomux_in_selected(g, pad, ESP32_SDMMC_FUNC) && !off) {
            l->to_host |= bit;
        }
        if (esp32_gpio_iomux_in_level(g, pad, ESP32_SDMMC_FUNC)) {
            l->host_level |= bit;
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * The routing or a pad's set-up changed: the controller follows the pads
 * as they now are.
 */
static void esp32_sdmmc_route_changed(Notifier *n, void *data)
{
    Esp32SdmmcState *s = container_of(n, Esp32SdmmcState, route_notifier);

    dwc_sdmmc_lines_changed(DWC_SDMMC(s));
}

/*
 * The pads are about to change, or are sampled: the controller first
 * takes its events due by then, as the pads were.
 */
static void esp32_sdmmc_line_sync(Notifier *n, void *data)
{
    Esp32SdmmcState *s = container_of(n, Esp32SdmmcState, line_source);

    dwc_sdmmc_catch_up(DWC_SDMMC(s));
}

static void esp32_sdmmc_init(Object *obj)
{
    Esp32SdmmcState *s = ESP32_SDMMC(obj);

    qdev_init_gpio_out_named(DEVICE(obj), s->pad_out, ESP32_SDMMC_PAD_OUT,
                             DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), s->pad_oe, ESP32_SDMMC_PAD_OE,
                             DWC_SDMMC_MAX_SLOTS * DWC_SDMMC_LINES);
    s->route_notifier.notify = esp32_sdmmc_route_changed;
    s->line_source.notify = esp32_sdmmc_line_sync;
}

static void esp32_sdmmc_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    Esp32SdmmcState *s = ESP32_SDMMC(dev);

    DWC_SDMMC_GET_CLASS(dev)->parent_realize(dev, errp);
    if (*errp) {
        return;
    }
    if (s->gpio) {
        esp32_gpio_add_route_notifier(s->gpio, &s->route_notifier);
        esp32_line_add_source(&s->line_source);
    }
}

static void esp32_sdmmc_unrealize(DeviceState *dev)
{
    Esp32SdmmcState *s = ESP32_SDMMC(dev);

    if (s->gpio) {
        notifier_remove(&s->route_notifier);
        notifier_remove(&s->line_source);
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * The lines between transfers, as the pads see them: CMD and DAT
 * released, SDCLK driven low (HS*_CLK is an output only, type O). The
 * transfers themselves are above single pad transitions
 * (hw/sd/dwc_sdmmc.c).
 */
static void esp32_sdmmc_reset_exit(Object *obj, ResetType type)
{
    Esp32SdmmcState *s = ESP32_SDMMC(obj);
    ResettableClass *prc = RESETTABLE_CLASS(
        object_class_get_parent(object_get_class(obj)));

    if (prc->phases.exit) {
        prc->phases.exit(obj, type);
    }
    for (unsigned slot = 0; slot < DWC_SDMMC_MAX_SLOTS; slot++) {
        for (unsigned i = 0; i < DWC_SDMMC_LINES; i++) {
            unsigned n = slot * DWC_SDMMC_LINES + i;
            bool clk = (1u << i) == DWC_SDMMC_LINE_CLK;

            qemu_set_irq(s->pad_out[n], !clk);
            qemu_set_irq(s->pad_oe[n], clk);
        }
    }
}

static const Property esp32_sdmmc_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32SdmmcState, gpio, TYPE_ESP32_GPIO,
                     Esp32GpioState *),
};

static void esp32_sdmmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DWCSDMMCClass *sc = DWC_SDMMC_CLASS(klass);

    device_class_set_parent_realize(dc, esp32_sdmmc_realize,
                                    &sc->parent_realize);
    dc->unrealize = esp32_sdmmc_unrealize;
    device_class_set_props(dc, esp32_sdmmc_properties);
    rc->phases.exit = esp32_sdmmc_reset_exit;
    sc->lines = esp32_sdmmc_lines;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_sdmmc_types[] = {
    {
        .name          = TYPE_ESP32_SDMMC,
        .parent        = TYPE_DWC_SDMMC,
        .instance_size = sizeof(Esp32SdmmcState),
        .instance_init = esp32_sdmmc_init,
        .class_init    = esp32_sdmmc_class_init,
    },
};

DEFINE_TYPES(esp32_sdmmc_types)
