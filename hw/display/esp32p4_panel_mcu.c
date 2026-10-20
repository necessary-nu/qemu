/*
 * The control chip of a Waveshare DSI TOUCH panel module, at I2C 0x45,
 * as the ESP32-P4 board's DSI connector takes it
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: Linux's gpio-waveshare-dsi.c and its binding
 * waveshare,dsi-touch-gpio.yaml (the series "drm/panel: support Waveshare
 * DSI TOUCH kits", April 2026), the Raspberry Pi kernel's
 * waveshare-panel-regulator.c and vc4-kms-dsi-waveshare-panel-v2 overlay;
 * Waveshare's esp_lcd_hx8394 and its ESP32-P4 BSPs (esp32_p4_nano,
 * esp32_p4_platform). No datasheet describes the chip.
 *
 * [spec:nuos:req:emu.esp32p4.panel-control]
 * Its registers, each written as a transfer of its address and a byte:
 *
 *   - 94h REG_TP: lines 8 to 15; 95h REG_LCD: lines 0 to 7; 96h REG_PWM:
 *     the backlight's PWM duty, 0 to 255 (the drivers' backlight range).
 *   - 97h REG_SIZE, 98h REG_ID, 99h REG_VERSION, read only: the panel's
 *     diagonal in tenths of an inch ("size"), the hardware ID ("hw-id")
 *     and the firmware version ("version"), which Linux reads at probe.
 *
 * Its lines: 0 AVDD, 1 the panel's RESX, 2 the backlight's enable, 4
 * IOVCC, 8 VCC, 9 the touch controller's reset (gpio-waveshare-dsi.c's
 * names), each high while its bit is set: the overlay's panel reset is
 * GPIO_ACTIVE_HIGH on line 1, which Linux's panel driver sets low, then
 * high, to reset the panel, and ESP-IDF's driver writes REG_LCD 11h
 * (AVDD and IOVCC) then 17h (RESX released, the backlight on) before it
 * talks to the panel. The module wires the lines (esp32p4_dsi_panel.c);
 * the rest go nowhere.
 *
 * A read gives the register the last write addressed: the address
 * written alone, then a read, with a repeated START (gpio-waveshare-dsi.c's
 * i2c_smbus_read_byte_data()) or after a STOP (waveshare-panel-
 * regulator.c, 5 to 10 ms later). Reads of 94h to 96h give what was
 * written, of other addresses 0 (logged). The drivers access one register
 * a transfer, and nothing documents an address that advances: a write's
 * bytes after its first data byte are ignored (logged), and each byte of a
 * read gives the same register. The chip answers its own address only.
 *
 * From power-on REG_LCD is 00h (the panel unpowered, in reset, unlit),
 * REG_TP 03h (VCC on, the touch controller out of reset) and REG_PWM 00h:
 * nothing documents them, but Waveshare's ESP-IDF drivers power the
 * panel through REG_LCD before using it and never write REG_TP, yet find
 * the touch controller. The chip is the module's, on the module's supply:
 * only the board's power-on resets it, not the chip's resets.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/esp32p4_mipi_dsi.h"
#include "migration/vmstate.h"

#define REG_TP          0x94
#define REG_LCD         0x95
#define REG_PWM         0x96
#define REG_SIZE        0x97
#define REG_ID          0x98
#define REG_VERSION     0x99

#define MCU_ADDR        0x45
#define MCU_TP_RESET    0x03

static void mcu_drive(WavesharePanelMcuState *s)
{
    for (int i = 0; i < WAVESHARE_PANEL_MCU_LINES; i++) {
        uint8_t reg = i < 8 ? s->lcd : s->tp;

        qemu_set_irq(s->line[i], (reg >> (i % 8)) & 1);
    }
    qemu_set_irq(s->pwm_out, s->pwm);
}

/* [spec:nuos:req:emu.esp32p4.panel-control] */
static void mcu_write(WavesharePanelMcuState *s, uint8_t reg, uint8_t v)
{
    switch (reg) {
    case REG_TP:
        s->tp = v;
        break;
    case REG_LCD:
        s->lcd = v;
        break;
    case REG_PWM:
        s->pwm = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %02xh to register "
                      "%02xh, which %s\n", TYPE_WAVESHARE_PANEL_MCU, v, reg,
                      reg >= REG_SIZE && reg <= REG_VERSION ? "is read only"
                                                            : "it lacks");
        return;
    }
    mcu_drive(s);
}

/* [spec:nuos:req:emu.esp32p4.panel-control] */
static uint8_t mcu_read(WavesharePanelMcuState *s, uint8_t reg)
{
    switch (reg) {
    case REG_TP:
        return s->tp;
    case REG_LCD:
        return s->lcd;
    case REG_PWM:
        return s->pwm;
    case REG_SIZE:
        return s->size;
    case REG_ID:
        return s->hw_id;
    case REG_VERSION:
        return s->version;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of register %02xh, which "
                      "it lacks: 0\n", TYPE_WAVESHARE_PANEL_MCU, reg);
        return 0;
    }
}

static int mcu_event(I2CSlave *i2c, enum i2c_event event)
{
    WavesharePanelMcuState *s = WAVESHARE_PANEL_MCU(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_next = true;
        s->written = 0;
    }
    return 0;
}

static int mcu_send(I2CSlave *i2c, uint8_t data)
{
    WavesharePanelMcuState *s = WAVESHARE_PANEL_MCU(i2c);

    if (s->ptr_next) {
        s->ptr = data;
        s->ptr_next = false;
    } else if (!s->written++) {
        mcu_write(s, s->ptr, data);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: byte %02xh after register "
                      "%02xh's: ignored\n", TYPE_WAVESHARE_PANEL_MCU, data,
                      s->ptr);
    }
    return 0;
}

static uint8_t mcu_recv(I2CSlave *i2c)
{
    return mcu_read(WAVESHARE_PANEL_MCU(i2c), WAVESHARE_PANEL_MCU(i2c)->ptr);
}

static bool mcu_match(I2CSlave *candidate, uint8_t address, bool broadcast,
                      I2CNodeList *current_devs)
{
    I2CNode *node;

    if (candidate->address != address) {
        return false;
    }
    node = g_new(struct I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

static void mcu_reset_hold(Object *obj, ResetType type)
{
    WavesharePanelMcuState *s = WAVESHARE_PANEL_MCU(obj);

    if (s->powered) {
        return;
    }
    s->powered = true;
    s->tp = MCU_TP_RESET;
    s->lcd = 0;
    s->pwm = 0;
    s->ptr = 0;
    s->ptr_next = false;
    s->written = 0;
    mcu_drive(s);
}

static void mcu_init(Object *obj)
{
    WavesharePanelMcuState *s = WAVESHARE_PANEL_MCU(obj);
    DeviceState *dev = DEVICE(obj);

    I2C_SLAVE(obj)->address = MCU_ADDR;
    qdev_init_gpio_out_named(dev, s->line, WAVESHARE_PANEL_MCU_LINE,
                             WAVESHARE_PANEL_MCU_LINES);
    qdev_init_gpio_out_named(dev, &s->pwm_out, WAVESHARE_PANEL_MCU_PWM, 1);
}

static const VMStateDescription vmstate_mcu = {
    .name = TYPE_WAVESHARE_PANEL_MCU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, WavesharePanelMcuState),
        VMSTATE_BOOL(powered, WavesharePanelMcuState),
        VMSTATE_UINT8(tp, WavesharePanelMcuState),
        VMSTATE_UINT8(lcd, WavesharePanelMcuState),
        VMSTATE_UINT8(pwm, WavesharePanelMcuState),
        VMSTATE_UINT8(ptr, WavesharePanelMcuState),
        VMSTATE_BOOL(ptr_next, WavesharePanelMcuState),
        VMSTATE_UINT8(written, WavesharePanelMcuState),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * The 5-DSI-TOUCH-A's own values are not published. REG_SIZE gives the
 * diagonal in tenths of an inch: a 10.1" panel's reads 101 in the log of
 * Linux's probe in a Rockchip kernel issue, its hw id and version 0x1.
 */
static const Property mcu_properties[] = {
    DEFINE_PROP_UINT8("size", WavesharePanelMcuState, size, 50),
    DEFINE_PROP_UINT8("hw-id", WavesharePanelMcuState, hw_id, 0x01),
    DEFINE_PROP_UINT8("version", WavesharePanelMcuState, version, 0x01),
};

static void mcu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Waveshare DSI TOUCH panel module's control chip";
    dc->vmsd = &vmstate_mcu;
    /* It is part of a panel module, which wires its lines. */
    dc->user_creatable = false;
    rc->phases.hold = mcu_reset_hold;
    sc->event = mcu_event;
    sc->send = mcu_send;
    sc->recv = mcu_recv;
    sc->match_and_add = mcu_match;
    device_class_set_props(dc, mcu_properties);
}

static const TypeInfo mcu_types[] = {
    {
        .name = TYPE_WAVESHARE_PANEL_MCU,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(WavesharePanelMcuState),
        .instance_init = mcu_init,
        .class_init = mcu_class_init,
    },
};

DEFINE_TYPES(mcu_types)
