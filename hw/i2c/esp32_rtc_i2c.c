/*
 * ESP32 RTC_I2C: the RTC domain's I2C master, used by the ULP coprocessor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "ULP Coprocessor", section "RTC_I2C Controller",
 * and ESP-IDF's soc/esp32 rtc_i2c_reg.h for the registers.
 *
 * The controller runs the fixed transactions of the ULP's I2C_RD and
 * I2C_WR instructions (and of SENS_SAR_I2C_CTRL's software start), at the
 * SCL timing its registers give in RTC_FAST_CLK cycles. Its lines reach
 * the pads through RTCIO's RTC function 1. The devices on its QEMU I2C bus
 * sit on the lines it is routed to: they take part in a transaction only
 * if each line, pulled low by the controller, reads back low from the
 * pads, as the digital I2C controllers' devices do.
 *
 * Its interrupt status registers are for debugging; the interrupt is not
 * connected anywhere (TRM 1.7.2). The command list (RTC_I2C_CMDn with
 * RTC_I2C_TRANS_START), which the ULP does not use and the TRM does not
 * describe, is stored but not run.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "hw/core/irq.h"
#include "hw/i2c/esp32_rtc_i2c.h"
#include "migration/vmstate.h"

/* INT_CLR and INT_EN hold each interrupt one bit above its INT_RAW bit */
#define INT_RAW_MASK    0xf8
#define INT_EN_MASK     (INT_RAW_MASK << 1)

static const uint32_t writable[ESP32_RTC_I2C_REG_COUNT] = {
    [R_RTC_I2C_SCL_LOW_PERIOD] = 0x01ffffff,
    [R_RTC_I2C_CTRL] = 0xf3,
    [R_RTC_I2C_DEBUG_STATUS] = 0x7e00007f,
    [R_RTC_I2C_TIMEOUT] = 0xfffff,
    [R_RTC_I2C_SLAVE_ADDR] = 0x80007fff,
    [R_RTC_I2C_DATA] = 0xffffffff,
    [R_RTC_I2C_INT_EN] = INT_EN_MASK,
    [R_RTC_I2C_SDA_DUTY] = 0xfffff,
    [R_RTC_I2C_SCL_HIGH_PERIOD] = 0xfffff,
    [R_RTC_I2C_SCL_START_PERIOD] = 0xfffff,
    [R_RTC_I2C_SCL_STOP_PERIOD] = 0xfffff,
    [R_RTC_I2C_CMD0 ... R_RTC_I2C_CMD15] = 0x80003fff,
};

#define REG(s, name) ((s)->regs[R_##name])

static void raise_int(Esp32RtcI2cState *s, uint32_t raw_mask)
{
    REG(s, RTC_I2C_INT_RAW) |= raw_mask;
}

/* Drive SCL and SDA: a 0 pulls the line low, a 1 releases it. */
static void drive(Esp32RtcI2cState *s, bool scl, bool sda)
{
    uint32_t ctrl = REG(s, RTC_I2C_CTRL);
    bool scl_pp = FIELD_EX32(ctrl, RTC_I2C_CTRL, SCL_FORCE_OUT);
    bool sda_pp = FIELD_EX32(ctrl, RTC_I2C_CTRL, SDA_FORCE_OUT);

    qemu_set_irq(s->scl_out, scl);
    qemu_set_irq(s->scl_oe, scl_pp || !scl);
    qemu_set_irq(s->sda_out, sda);
    qemu_set_irq(s->sda_oe, sda_pp || !sda);
}

/* A bit as SDA reads it with the controller releasing the line */
static bool released_sda(Esp32RtcI2cState *s)
{
    drive(s, false, true);
    return s->sda_in;
}

/* A byte as it goes on the wire, MSB first unless TX_LSB_FIRST */
static uint8_t tx_byte(Esp32RtcI2cState *s, uint8_t b)
{
    return FIELD_EX32(REG(s, RTC_I2C_CTRL), RTC_I2C_CTRL, TX_LSB_FIRST) ?
           revbit8(b) : b;
}

static uint8_t rx_byte(Esp32RtcI2cState *s, uint8_t b)
{
    return FIELD_EX32(REG(s, RTC_I2C_CTRL), RTC_I2C_CTRL, RX_LSB_FIRST) ?
           revbit8(b) : b;
}

typedef struct Xfer {
    Esp32RtcI2cState *s;
    /* The devices on the QEMU bus see the transaction */
    bool connected;
    /* A START has addressed a device on the QEMU bus */
    bool bus_open;
} Xfer;

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * A START (or repeated START): with both lines released SCL must be high,
 * or the controller times out, and SDA high, or it has lost arbitration;
 * then SDA falls while SCL is high, then SCL falls.
 */
static bool xfer_start(Xfer *x)
{
    Esp32RtcI2cState *s = x->s;
    uint32_t *st = &REG(s, RTC_I2C_DEBUG_STATUS);

    drive(s, true, true);
    if (!s->scl_in) {
        *st = FIELD_DP32(*st, RTC_I2C_DEBUG_STATUS, TIMED_OUT, 1);
        raise_int(s, R_RTC_I2C_INT_RAW_TIME_OUT_MASK);
        return false;
    }
    if (!s->sda_in) {
        *st = FIELD_DP32(*st, RTC_I2C_DEBUG_STATUS, ARB_LOST, 1);
        raise_int(s, R_RTC_I2C_INT_RAW_ARBITRATION_LOST_MASK);
        return false;
    }
    drive(s, true, false);
    x->connected = !s->sda_in;
    drive(s, false, false);
    x->connected &= !s->scl_in;
    return true;
}

/* A STOP: SDA low, SCL released, then SDA released. */
static void xfer_stop(Xfer *x)
{
    Esp32RtcI2cState *s = x->s;

    if (x->bus_open) {
        i2c_end_transfer(s->bus);
        x->bus_open = false;
    }
    drive(s, false, false);
    drive(s, true, false);
    drive(s, true, true);
    raise_int(s, R_RTC_I2C_INT_RAW_TRANS_COMPLETE_MASK);
}

/* Send a byte; true if the slave acknowledged it. */
static bool xfer_send(Xfer *x, uint8_t byte, bool address)
{
    Esp32RtcI2cState *s = x->s;
    uint8_t wire = tx_byte(s, byte);
    bool nack;

    if (!x->connected) {
        nack = released_sda(s);
    } else if (address) {
        nack = i2c_start_transfer(s->bus, wire >> 1, wire & 1) != 0;
        x->bus_open = !nack;
    } else {
        nack = i2c_send(s->bus, wire) != 0;
    }
    REG(s, RTC_I2C_DEBUG_STATUS) =
        FIELD_DP32(REG(s, RTC_I2C_DEBUG_STATUS), RTC_I2C_DEBUG_STATUS,
                   ACK_VAL, nack);
    return !nack;
}

/* Receive a byte and answer it with a NACK, the last byte of a read. */
static uint8_t xfer_recv(Xfer *x)
{
    Esp32RtcI2cState *s = x->s;
    uint8_t wire;

    if (x->connected && x->bus_open) {
        wire = i2c_recv(s->bus);
        i2c_nack(s->bus);
    } else {
        wire = released_sda(s) ? 0xff : 0x00;
    }
    REG(s, RTC_I2C_DEBUG_STATUS) |= R_RTC_I2C_DEBUG_STATUS_ACK_VAL_MASK;
    return rx_byte(s, wire);
}

static uint32_t at_least_1(uint32_t v)
{
    return MAX(v, 1);
}

uint32_t esp32_rtc_i2c_transfer(Esp32RtcI2cState *s, uint8_t addr,
                                uint8_t sub, bool write, uint8_t wdata,
                                uint8_t *rdata)
{
    uint32_t high = at_least_1(REG(s, RTC_I2C_SCL_HIGH_PERIOD));
    uint32_t bit = at_least_1(REG(s, RTC_I2C_SCL_LOW_PERIOD)) + high;
    uint32_t start = at_least_1(REG(s, RTC_I2C_SCL_START_PERIOD));
    uint32_t stop = at_least_1(REG(s, RTC_I2C_SCL_STOP_PERIOD));
    uint32_t timeout = at_least_1(REG(s, RTC_I2C_TIMEOUT));
    uint32_t *st = &REG(s, RTC_I2C_DEBUG_STATUS);
    uint8_t first = (addr << 1) | 0, second = (addr << 1) | (write ? 0 : 1);
    Xfer x = { .s = s };
    unsigned bytes = 0;
    uint32_t cycles;

    *rdata = 0xff;
    *st &= ~(R_RTC_I2C_DEBUG_STATUS_TIMED_OUT_MASK |
             R_RTC_I2C_DEBUG_STATUS_ARB_LOST_MASK |
             R_RTC_I2C_DEBUG_STATUS_BYTE_TRANS_MASK);
    if (!FIELD_EX32(REG(s, RTC_I2C_CTRL), RTC_I2C_CTRL, MS_MODE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_i2c: transaction with RTC_I2C_MS_MODE "
                      "clear: the controller is a slave and times out\n");
        *st = FIELD_DP32(*st, RTC_I2C_DEBUG_STATUS, TIMED_OUT, 1);
        raise_int(s, R_RTC_I2C_INT_RAW_TIME_OUT_MASK);
        return timeout;
    }
    if (!xfer_start(&x)) {
        drive(s, true, true);
        return start + timeout;
    }
    cycles = start;

    /* START, slave address W, register, repeated START, address R/W */
    if (xfer_send(&x, first, true)) {
        bytes++;
        if (xfer_send(&x, sub, false)) {
            bytes++;
            drive(s, false, true);
            cycles += high + start;
            if (xfer_start(&x) && xfer_send(&x, second, true)) {
                bytes++;
                if (write) {
                    if (xfer_send(&x, wdata, false)) {
                        bytes++;
                    }
                } else {
                    *rdata = xfer_recv(&x);
                    REG(s, RTC_I2C_DATA) = *rdata;
                    bytes++;
                }
            }
        }
    }
    if (bytes == 4) {
        raise_int(s, R_RTC_I2C_INT_RAW_MASTER_TRANS_COMPLETE_MASK);
        *st |= R_RTC_I2C_DEBUG_STATUS_BYTE_TRANS_MASK;
    } else {
        /* An unacknowledged byte ends the transaction there */
        bytes++;
    }
    xfer_stop(&x);
    return cycles + bytes * 9 * bit + stop;
}

static void update_int_st(Esp32RtcI2cState *s)
{
    REG(s, RTC_I2C_INT_ST) = REG(s, RTC_I2C_INT_RAW) &
                             (REG(s, RTC_I2C_INT_EN) >> 1);
}

static bool reg_valid(hwaddr addr)
{
    return addr < ESP32_RTC_I2C_REG_COUNT * 4 &&
           (writable[addr / 4] || addr == A_RTC_I2C_INT_RAW ||
            addr == A_RTC_I2C_INT_CLR || addr == A_RTC_I2C_INT_ST);
}

static uint64_t esp32_rtc_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32RtcI2cState *s = ESP32_RTC_I2C(opaque);

    if (!reg_valid(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_i2c: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", addr);
        return 0;
    }
    switch (addr) {
    case A_RTC_I2C_INT_CLR:
        return 0;
    case A_RTC_I2C_INT_ST:
        update_int_st(s);
        return REG(s, RTC_I2C_INT_ST);
    default:
        return s->regs[addr / 4];
    }
}

static void esp32_rtc_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    Esp32RtcI2cState *s = ESP32_RTC_I2C(opaque);
    uint32_t v = value;

    if (!reg_valid(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_i2c: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", addr);
        return;
    }
    switch (addr) {
    case A_RTC_I2C_INT_RAW:
    case A_RTC_I2C_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtc_i2c: write to read-only register 0x%"
                      HWADDR_PRIx "\n", addr);
        return;
    case A_RTC_I2C_INT_CLR:
        REG(s, RTC_I2C_INT_RAW) &= ~((v & INT_EN_MASK) >> 1);
        return;
    case A_RTC_I2C_CTRL:
        if (FIELD_EX32(v & ~REG(s, RTC_I2C_CTRL), RTC_I2C_CTRL,
                       TRANS_START)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_rtc_i2c: running the command list "
                          "(RTC_I2C_TRANS_START) is not modelled\n");
        }
        s->regs[addr / 4] = v & writable[addr / 4];
        drive(s, true, true);
        return;
    default:
        s->regs[addr / 4] = v & writable[addr / 4];
        return;
    }
}

static const MemoryRegionOps esp32_rtc_i2c_ops = {
    .read = esp32_rtc_i2c_read,
    .write = esp32_rtc_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_rtc_i2c_scl_in(void *opaque, int n, int level)
{
    ESP32_RTC_I2C(opaque)->scl_in = level != 0;
}

static void esp32_rtc_i2c_sda_in(void *opaque, int n, int level)
{
    ESP32_RTC_I2C(opaque)->sda_in = level != 0;
}

static void esp32_rtc_i2c_reset_hold(Object *obj, ResetType type)
{
    Esp32RtcI2cState *s = ESP32_RTC_I2C(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void esp32_rtc_i2c_reset_exit(Object *obj, ResetType type)
{
    drive(ESP32_RTC_I2C(obj), true, true);
}

static void esp32_rtc_i2c_init(Object *obj)
{
    Esp32RtcI2cState *s = ESP32_RTC_I2C(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rtc_i2c_ops, s,
                          TYPE_ESP32_RTC_I2C, ESP32_RTC_I2C_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->bus = i2c_init_bus(dev, "i2c");
    qdev_init_gpio_out_named(dev, &s->scl_out, ESP32_RTC_I2C_SCL_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->scl_oe, ESP32_RTC_I2C_SCL_OE, 1);
    qdev_init_gpio_out_named(dev, &s->sda_out, ESP32_RTC_I2C_SDA_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->sda_oe, ESP32_RTC_I2C_SDA_OE, 1);
    qdev_init_gpio_in_named(dev, esp32_rtc_i2c_scl_in, ESP32_RTC_I2C_SCL_IN,
                            1);
    qdev_init_gpio_in_named(dev, esp32_rtc_i2c_sda_in, ESP32_RTC_I2C_SDA_IN,
                            1);
    s->scl_in = true;
    s->sda_in = true;
}

static const VMStateDescription vmstate_esp32_rtc_i2c = {
    .name = TYPE_ESP32_RTC_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Esp32RtcI2cState, ESP32_RTC_I2C_REG_COUNT),
        VMSTATE_BOOL(scl_in, Esp32RtcI2cState),
        VMSTATE_BOOL(sda_in, Esp32RtcI2cState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32_rtc_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_rtc_i2c_reset_hold;
    rc->phases.exit = esp32_rtc_i2c_reset_exit;
    dc->vmsd = &vmstate_esp32_rtc_i2c;
}

/* [spec:nuos:req:emu.esp32.ulp] */
static const TypeInfo esp32_rtc_i2c_info = {
    .name = TYPE_ESP32_RTC_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RtcI2cState),
    .instance_init = esp32_rtc_i2c_init,
    .class_init = esp32_rtc_i2c_class_init,
};

static void esp32_rtc_i2c_register_types(void)
{
    type_register_static(&esp32_rtc_i2c_info);
}

type_init(esp32_rtc_i2c_register_types)
