/*
 * An I2C bus on two board wires, its devices answering what a master puts
 * on the wires
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_I2C_WIRE_BUS_H
#define HW_I2C_I2C_WIRE_BUS_H

#include "hw/core/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_I2C_WIRE_BUS "i2c-wire-bus"
OBJECT_DECLARE_SIMPLE_TYPE(I2CWireBusState, I2C_WIRE_BUS)

/*
 * The board's SCL and SDA wires, with QEMU I2C devices on them (its QEMU
 * bus, named by "bus-name"), for a board whose master is modelled at the
 * level of the wires rather than of bytes:
 *
 *   - Inputs "scl" and "sda": the levels on the wires, as the pads (or
 *     whatever drives the wires) leave them. A START is SDA falling while
 *     SCL is high, a STOP SDA rising while it is high; a bit is taken on
 *     each rising edge of SCL.
 *   - Output "sda-pull": high while the bus's devices pull SDA low, an
 *     open-drain driver that the board turns into the wire's level. The
 *     devices change SDA only on SCL's falling edges, as the bus's hold
 *     time requires: their acknowledge for the byte just in, and each bit
 *     of a byte they send, from the falling edge that ends the bit before
 *     to the one that ends theirs.
 *
 * The devices take an address after a START (i2c_start_transfer(); a
 * repeated START to the device already addressed continues its
 * transfer, any other ends it), and a byte at each eighth bit
 * (i2c_send(), whose result is the acknowledge); a byte read is fetched
 * (i2c_recv()) as its first bit goes out, and the master's NACK ends the
 * read (i2c_nack()). A STOP ends the transfer. The devices stretch no
 * clock.
 */
#define I2C_WIRE_BUS_SCL        "scl"
#define I2C_WIRE_BUS_SDA        "sda"
#define I2C_WIRE_BUS_SDA_PULL   "sda-pull"

struct I2CWireBusState {
    SysBusDevice parent_obj;

    I2CBus *bus;
    char *bus_name;
    qemu_irq sda_pull;

    /* The wires' levels */
    bool scl;
    bool sda;
    /* Where in a transfer the bus is (I2CWireState in the source) */
    uint8_t state;
    /* The bits of the byte coming in, or the byte going out */
    uint8_t shift;
    uint8_t bits;
    /* The master's acknowledge of a byte read */
    bool master_ack;
    /* A device is in a transfer, at what address, and which way */
    bool active;
    uint8_t addr;
    bool addr_read;
    bool pulling;
};

#endif
