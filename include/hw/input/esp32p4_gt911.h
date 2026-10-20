/*
 * A Goodix GT911 capacitive touch controller, as the ESP32-P4 board's
 * display modules carry it on the board's I2C wires
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INPUT_ESP32P4_GT911_H
#define HW_INPUT_ESP32P4_GT911_H

#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "ui/input.h"

#define TYPE_GT911 "gt911"
OBJECT_DECLARE_SIMPLE_TYPE(GT911State, GT911)

/*
 * Inputs: "vdd", its supply, on while high; "rstb", its RSTB, running
 * while high; "int", the level another device drives on its INT pin,
 * which at RSTB's rise selects its address (low 0x5D, high 0x14), from
 * power-on the "int-level" property's.
 */
#define GT911_VDD           "vdd"
#define GT911_RSTB          "rstb"
#define GT911_INT           "int"

#define GT911_POINTS        5
/* The configuration, 0x8047 to 0x80FE, then Config_Chksum, Config_Fresh. */
#define GT911_CFG_BASE      0x8047
#define GT911_CFG_LEN       (0x8100 - GT911_CFG_BASE + 1)

typedef struct GT911Contact {
    bool down;
    uint8_t track;
    /* Where on the panel, 0 to INPUT_EVENT_ABS_MAX across and down. */
    uint16_t x;
    uint16_t y;
} GT911Contact;

struct GT911State {
    I2CSlave parent_obj;

    QemuInputHandlerState *input;
    QEMUTimer *scan;
    bool int_level_reset;
    uint16_t fw_version;
    uint8_t vendor_id;

    bool powered;
    bool vdd;
    bool rstb;
    bool int_level;
    bool running;
    bool sleeping;
    uint8_t addr;
    int64_t ready_ns;
    int64_t touch_ns;
    int64_t scan_due_ns;

    /* The configuration registers, and the one in effect (its flash's). */
    uint8_t cfg[GT911_CFG_LEN];
    uint8_t flash[GT911_CFG_LEN];
    uint8_t cmd[7];
    uint8_t status;
    uint8_t point[GT911_POINTS * 8];
    uint16_t ptr;
    uint8_t addr_bytes;

    /* The pointer's touch, then the multi-touch slots'. */
    GT911Contact contact[1 + GT911_POINTS];
    uint16_t ptr_x;
    uint16_t ptr_y;
    /* The last report: whether it had points, and theirs. */
    bool reported;
    uint8_t last_n;
    uint16_t last_x[GT911_POINTS];
    uint16_t last_y[GT911_POINTS];
};

/* Take touches from the QEMU input events on @display's console. */
void gt911_bind_console(GT911State *s, DeviceState *display);

#endif
