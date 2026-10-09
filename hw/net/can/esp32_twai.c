/*
 * ESP32 TWAI (Two-Wire Automotive Interface) emulation
 *
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * The ESP32 TWAI peripheral is a CAN 2.0B controller based on SJA1000.
 * It supports standard frame format (11-bit ID) and extended frame format
 * (29-bit ID) with programmable bit rate up to 1 Mbps.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * Frames move whole between the SJA1000 model and the CAN bus backend, so
 * twai_tx and twai_rx carry the bus's recessive level rather than each
 * frame's bits; what the lines decide is whether the controller is on the
 * bus at all (see Esp32TWAIState.connected). twai_bus_off_on follows the
 * status register's bus status, and twai_clkout is the CLKOUT clock,
 * evaluated whenever software samples the pads rather than edge by edge.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/net/can/esp32_twai.h"
#include "hw/net/can/can_sja1000.h"
#include "qom/object.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "net/can_emu.h"
#include "hw/core/qdev-clock.h"
#include "qemu/timer.h"

static void esp32_twai_route_changed(Notifier *n, void *data);

/* Rejoin the CAN bus backend as the restored routing says. */
static int esp32_twai_post_load(void *opaque, int version_id)
{
    Esp32TWAIState *s = opaque;

    esp32_twai_route_changed(&s->route_notifier, s->gpio);
    return 0;
}

/* Migration state description */
static const VMStateDescription vmstate_esp32_twai = {
    .name = TYPE_ESP32_TWAI,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32_twai_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(sja_state, Esp32TWAIState, 0, vmstate_can_sja, CanSJA1000State),
        VMSTATE_CLOCK(apb_clk, Esp32TWAIState),
        VMSTATE_BOOL(clkout_level, Esp32TWAIState),
        VMSTATE_BOOL(rx_level, Esp32TWAIState),
        VMSTATE_END_OF_LIST()
    }
};

/* The SJA1000's status register's bus status bit: bus-off */
#define TWAI_SR_BUS_OFF     0x80
/* Clock divider register: CD, the CLKOUT divider, and CLOCK_OFF */
#define TWAI_CDR_CD_MASK    0x07
#define TWAI_CDR_CD_APB     0x07
#define TWAI_CDR_CLOCK_OFF  0x08

/* Reset handler */
static void esp32_twai_reset(Object *obj, ResetType type)
{
    Esp32TWAIState *s = ESP32_TWAI(obj);

    /* Reset underlying SJA1000 hardware to its default state */
    can_sja_hardware_reset(&s->sja_state);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * CLKOUT: APB_CLK divided by 2 * (CD + 1), or APB_CLK itself for CD = 7,
 * low while CLOCK_OFF is set or APB_CLK is stopped. Its level is a
 * function of the virtual time, taken when the pads are sampled.
 */
static bool esp32_twai_clkout(Esp32TWAIState *s, int64_t now)
{
    uint8_t cdr = s->sja_state.clock;
    uint64_t half_periods, div;

    if ((cdr & TWAI_CDR_CLOCK_OFF) || !clock_is_enabled(s->apb_clk)) {
        return false;
    }
    div = (cdr & TWAI_CDR_CD_MASK) == TWAI_CDR_CD_APB ? 1 :
          2 * ((cdr & TWAI_CDR_CD_MASK) + 1);
    half_periods = clock_ns_to_ticks(s->apb_clk, 2 * now);
    return (half_periods / div) & 1;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Drive the outputs: twai_tx recessive (high), twai_bus_off_on high while
 * the controller is bus-off, and CLKOUT at its level now.
 */
static void esp32_twai_update_lines(Esp32TWAIState *s)
{
    uint8_t sr = (s->sja_state.clock & 0x80) ? s->sja_state.status_pel :
                                              s->sja_state.status_bas;
    bool clkout = esp32_twai_clkout(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

    qemu_set_irq(s->tx_out, 1);
    qemu_set_irq(s->bus_off_out, (sr & TWAI_SR_BUS_OFF) != 0);
    if (clkout != s->clkout_level) {
        s->clkout_level = clkout;
        qemu_set_irq(s->clkout_out, clkout);
    }
}

static void esp32_twai_reset_exit(Object *obj, ResetType type)
{
    esp32_twai_update_lines(ESP32_TWAI(obj));
}

static void esp32_twai_line_sync(Notifier *n, void *data)
{
    esp32_twai_update_lines(container_of(n, Esp32TWAIState, line_source));
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Join or leave the CAN bus backend as software routes twai_tx to a pad
 * and twai_rx from one, or stops doing so.
 */
static void esp32_twai_route_changed(Notifier *n, void *data)
{
    Esp32TWAIState *s = container_of(n, Esp32TWAIState, route_notifier);
    bool routed = esp32_gpio_sig_out_pads(s->gpio, ESP32_SIG_TWAI_TX) != 0 &&
                  esp32_gpio_sig_in_pad(s->gpio, ESP32_SIG_TWAI_RX) >= 0;

    if (!s->canbus || routed == s->connected) {
        return;
    }
    if (routed) {
        if (can_sja_connect_to_bus(&s->sja_state, s->canbus) < 0) {
            return;
        }
    } else {
        can_sja_disconnect(&s->sja_state);
    }
    s->connected = routed;
}

/* Interrupt handler for SJA1000 events */
static void esp32_twai_irq_handler(void *opaque, int irq_num, int level)
{
    Esp32TWAIState *d = opaque;

    qemu_set_irq(d->irq, level);
}

/* Memory-mapped I/O read handler for the TWAI peripheral.
 * Maps ESP32 TWAI register accesses to the underlying SJA1000 controller.
 */
static uint64_t esp32_twai_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32TWAIState *s = ESP32_TWAI(opaque);
    /* 
    * ESP32 TWAI registers are 32-bit aligned, but SJA1000 expects byte offsets.
    * Shift addr right by 2 to convert from word address to SJA1000 register index.
    */
    const uint64_t sja_addr = addr >> 2;
    uint8_t value;
    if ((s->sja_state.clock & 0x80) && sja_addr == SJA_RMC) {
        /* PeliCAN Mode */
        value = s->sja_state.rxmsg_cnt;
    } else {
        value = can_sja_mem_read(&s->sja_state, sja_addr, 1) & 0xFF;
    }

    return value;
}

/* Memory-mapped I/O write handler for the TWAI peripheral.
 * Maps ESP32 TWAI register accesses to the underlying SJA1000 controller.
 */
static void esp32_twai_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned int size)
{
    Esp32TWAIState *s = ESP32_TWAI(opaque);
    /* 
    * ESP32 TWAI registers are 32-bit aligned, but SJA1000 expects byte offsets.
    * Shift addr right by 2 to convert from word address to SJA1000 register index.
    */
    const uint64_t sja_addr = addr >> 2;

    if (sja_addr == SJA_CDR) {
        value |= 0x80;
    }

    can_sja_mem_write(&s->sja_state, sja_addr, value, size);
    esp32_twai_update_lines(s);
}

/* Whether the controller is on the CAN bus backend, for inspection */
static bool esp32_twai_get_bus_connected(Object *obj, Error **errp)
{
    return ESP32_TWAI(obj)->connected;
}

static void esp32_twai_rx_in(void *opaque, int n, int level)
{
    ESP32_TWAI(opaque)->rx_level = level != 0;
}

static void esp32_twai_init(Object * obj)
{
    Esp32TWAIState *s = ESP32_TWAI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    Esp32TWAIClass *twai_class = ESP32_TWAI_GET_CLASS(obj);

    /* Set up MMIO operations */
    s->twai_ops = (MemoryRegionOps) {
        .read = twai_class->twai_read,
        .write = twai_class->twai_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    };

    /* Initialize MMIO region */
    memory_region_init_io(&s->iomem, obj, &s->twai_ops, s,
                         TYPE_ESP32_TWAI, ESP32_TWAI_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    /* Add CAN bus link property */
    object_property_add_link(obj, "canbus", TYPE_CAN_BUS,
                           (Object **)&s->canbus,
                           qdev_prop_allow_set_link_before_realize,
                           0);
    object_property_add_link(obj, "gpio", TYPE_ESP32_GPIO,
                             (Object **)&s->gpio,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);

    object_property_add_bool(obj, "bus-connected",
                             esp32_twai_get_bus_connected, NULL);

    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", NULL, NULL, 0);
    qdev_init_gpio_out_named(DEVICE(obj), &s->tx_out, ESP32_TWAI_TX, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->bus_off_out,
                             ESP32_TWAI_BUS_OFF, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->clkout_out,
                             ESP32_TWAI_CLKOUT, 1);
    qdev_init_gpio_in_named(DEVICE(obj), esp32_twai_rx_in, ESP32_TWAI_RX, 1);
    s->rx_level = true;
    s->route_notifier.notify = esp32_twai_route_changed;
    s->line_source.notify = esp32_twai_line_sync;
}

/* Device realization */
static void esp32_twai_realize(DeviceState *dev, Error **errp)
{
    Esp32TWAIState *s = ESP32_TWAI(dev);

    /* Set up interrupt handling */
    s->irq_handler = qemu_allocate_irq(esp32_twai_irq_handler, s, 0);
    
    /* Initialize SJA1000 controller */
    can_sja_init(&s->sja_state, s->irq_handler);

    /* The bus is joined once the lines are routed to pads */
    if (!s->gpio) {
        error_setg(errp, "esp32.twai: 'gpio' link not set");
        return;
    }
    esp32_gpio_add_route_notifier(s->gpio, &s->route_notifier);
    esp32_line_add_source(&s->line_source);
}

/* Class initialization */
static void esp32_twai_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    Esp32TWAIClass *twai_class = ESP32_TWAI_CLASS(klass);
    
    /* Set up virtual methods */
    twai_class->twai_read = esp32_twai_read;
    twai_class->twai_write = esp32_twai_write;
    
    /* Set up device class methods */
    rc->phases.hold = esp32_twai_reset;
    rc->phases.exit = esp32_twai_reset_exit;
    dc->realize = esp32_twai_realize;
    dc->vmsd = &vmstate_esp32_twai;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_twai_type_info = {
    .name = TYPE_ESP32_TWAI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32TWAIState),
    .instance_init = esp32_twai_init,
    .class_size = sizeof(Esp32TWAIClass),
    .class_init = esp32_twai_class_init,
};

static void esp32_twai_register_types(void)
{
    type_register_static(&esp32_twai_type_info);
}

type_init(esp32_twai_register_types)