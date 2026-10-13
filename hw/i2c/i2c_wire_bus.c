/*
 * An I2C bus on two board wires, its devices answering what a master puts
 * on the wires
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See include/hw/i2c/i2c_wire_bus.h. Unlike bitbang_i2c.c, which answers
 * a bit as SCL rises and so suits a master that reads SDA back at once,
 * the devices here change SDA only while SCL is low, so that a master
 * watching the wires sees no START or STOP in their bits.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c_wire_bus.h"
#include "migration/vmstate.h"

typedef enum I2CWireState {
    /* No transfer for the bus's devices: idle, or another's */
    WIRE_IDLE,
    /* The address byte after a START */
    WIRE_ADDR,
    /* The ninth clock of a byte the devices acknowledge */
    WIRE_ACK,
    /* The bytes a master writes */
    WIRE_WRITE,
    /* A byte the devices send, and the master's acknowledge after it */
    WIRE_READ,
    WIRE_READ_ACK,
} I2CWireState;

static void wire_pull(I2CWireBusState *s, bool pull)
{
    if (s->pulling != pull) {
        s->pulling = pull;
        qemu_set_irq(s->sda_pull, pull);
    }
}

static void wire_end(I2CWireBusState *s)
{
    if (s->active) {
        i2c_end_transfer(s->bus);
        s->active = false;
    }
}

/*
 * The address byte: a repeated START to the device in the transfer
 * continues it (a register address written, then read), another address
 * ends it first. Returns whether a device acknowledges.
 */
static bool wire_address(I2CWireBusState *s, uint8_t byte)
{
    uint8_t addr = byte >> 1;

    if (s->active && s->addr != addr) {
        wire_end(s);
    }
    s->active = i2c_start_transfer(s->bus, addr, byte & 1) == 0;
    s->addr = addr;
    s->addr_read = byte & 1;
    if (!s->active) {
        /* Nothing answered: the core forgets the address (a broadcast). */
        i2c_end_transfer(s->bus);
    }
    return s->active;
}

/* Put the next bit of the byte going out on SDA. */
static void wire_send_bit(I2CWireBusState *s)
{
    wire_pull(s, !((s->shift >> (7 - s->bits)) & 1));
}

/* SCL rose: a bit is on SDA. */
static void wire_scl_rise(I2CWireBusState *s)
{
    switch (s->state) {
    case WIRE_ADDR:
    case WIRE_WRITE:
        s->shift = (s->shift << 1) | s->sda;
        s->bits++;
        break;
    case WIRE_READ_ACK:
        s->master_ack = !s->sda;
        break;
    default:
        break;
    }
}

/* SCL fell: the bit is over, and the devices may change SDA. */
static void wire_scl_fall(I2CWireBusState *s)
{
    switch (s->state) {
    case WIRE_ADDR:
        if (s->bits < 8) {
            break;
        }
        if (wire_address(s, s->shift)) {
            wire_pull(s, true);
            s->state = WIRE_ACK;
        } else {
            s->state = WIRE_IDLE;
        }
        break;
    case WIRE_WRITE:
        if (s->bits < 8) {
            break;
        }
        if (i2c_send(s->bus, s->shift) == 0) {
            wire_pull(s, true);
            s->state = WIRE_ACK;
        } else {
            /* A NACK: the master stops or starts again. */
            s->state = WIRE_IDLE;
        }
        break;
    case WIRE_ACK:
        /* The acknowledge is over: the next byte, either way */
        wire_pull(s, false);
        s->bits = 0;
        s->shift = 0;
        if (s->addr_read) {
            s->shift = i2c_recv(s->bus);
            s->state = WIRE_READ;
            wire_send_bit(s);
        } else {
            s->state = WIRE_WRITE;
        }
        break;
    case WIRE_READ:
        if (++s->bits < 8) {
            wire_send_bit(s);
        } else {
            /* SDA is the master's for its acknowledge. */
            wire_pull(s, false);
            s->state = WIRE_READ_ACK;
        }
        break;
    case WIRE_READ_ACK:
        if (s->master_ack) {
            s->bits = 0;
            s->shift = i2c_recv(s->bus);
            s->state = WIRE_READ;
            wire_send_bit(s);
        } else {
            i2c_nack(s->bus);
            s->state = WIRE_IDLE;
        }
        break;
    default:
        break;
    }
}

static void i2c_wire_bus_scl(void *opaque, int n, int level)
{
    I2CWireBusState *s = opaque;

    if (s->scl == !!level) {
        return;
    }
    s->scl = level;
    if (level) {
        wire_scl_rise(s);
    } else {
        wire_scl_fall(s);
    }
}

static void i2c_wire_bus_sda(void *opaque, int n, int level)
{
    I2CWireBusState *s = opaque;

    if (s->sda == !!level) {
        return;
    }
    s->sda = level;
    if (!s->scl) {
        return;
    }
    if (!level) {
        /* START, or a repeated START */
        wire_pull(s, false);
        s->state = WIRE_ADDR;
        s->bits = 0;
        s->shift = 0;
    } else {
        /* STOP */
        wire_pull(s, false);
        wire_end(s);
        s->state = WIRE_IDLE;
    }
}

static void i2c_wire_bus_reset_hold(Object *obj, ResetType type)
{
    I2CWireBusState *s = I2C_WIRE_BUS(obj);

    s->state = WIRE_IDLE;
    s->bits = 0;
    s->shift = 0;
    s->master_ack = false;
    s->active = false;
    s->addr = 0;
    s->addr_read = false;
}

static void i2c_wire_bus_reset_exit(Object *obj, ResetType type)
{
    I2CWireBusState *s = I2C_WIRE_BUS(obj);

    s->pulling = true;
    wire_pull(s, false);
}

static void i2c_wire_bus_realize(DeviceState *dev, Error **errp)
{
    I2CWireBusState *s = I2C_WIRE_BUS(dev);

    s->bus = i2c_init_bus(dev, s->bus_name ? s->bus_name : "i2c");
}

static void i2c_wire_bus_init(Object *obj)
{
    I2CWireBusState *s = I2C_WIRE_BUS(obj);
    DeviceState *dev = DEVICE(obj);

    qdev_init_gpio_in_named(dev, i2c_wire_bus_scl, I2C_WIRE_BUS_SCL, 1);
    qdev_init_gpio_in_named(dev, i2c_wire_bus_sda, I2C_WIRE_BUS_SDA, 1);
    qdev_init_gpio_out_named(dev, &s->sda_pull, I2C_WIRE_BUS_SDA_PULL, 1);
    /* Released wires are high, as their pull-ups hold them. */
    s->scl = true;
    s->sda = true;
}

static const VMStateDescription vmstate_i2c_wire_bus = {
    .name = TYPE_I2C_WIRE_BUS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(scl, I2CWireBusState),
        VMSTATE_BOOL(sda, I2CWireBusState),
        VMSTATE_UINT8(state, I2CWireBusState),
        VMSTATE_UINT8(shift, I2CWireBusState),
        VMSTATE_UINT8(bits, I2CWireBusState),
        VMSTATE_BOOL(master_ack, I2CWireBusState),
        VMSTATE_BOOL(active, I2CWireBusState),
        VMSTATE_UINT8(addr, I2CWireBusState),
        VMSTATE_BOOL(addr_read, I2CWireBusState),
        VMSTATE_BOOL(pulling, I2CWireBusState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property i2c_wire_bus_properties[] = {
    DEFINE_PROP_STRING("bus-name", I2CWireBusState, bus_name),
};

static void i2c_wire_bus_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "I2C bus on two wires";
    dc->realize = i2c_wire_bus_realize;
    dc->vmsd = &vmstate_i2c_wire_bus;
    rc->phases.hold = i2c_wire_bus_reset_hold;
    rc->phases.exit = i2c_wire_bus_reset_exit;
    device_class_set_props(dc, i2c_wire_bus_properties);
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
}

static const TypeInfo i2c_wire_bus_types[] = {
    {
        .name = TYPE_I2C_WIRE_BUS,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(I2CWireBusState),
        .instance_init = i2c_wire_bus_init,
        .class_init = i2c_wire_bus_class_init,
    },
};

DEFINE_TYPES(i2c_wire_bus_types)
