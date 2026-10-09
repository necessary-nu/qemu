/*
 * ESP32 TWAI (Two-Wire Automotive Interface) emulation
 *
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#ifndef ESP32_TWAI_H
#define ESP32_TWAI_H

#include "hw/core/sysbus.h"
#include "net/can_emu.h"
#include "hw/core/irq.h"
#include "hw/net/can/can_sja1000.h"
#include "hw/core/clock.h"
#include "hw/gpio/esp32_gpio.h"

#define TYPE_ESP32_TWAI "esp32.twai"
#define ESP32_TWAI(obj) OBJECT_CHECK(Esp32TWAIState, (obj), TYPE_ESP32_TWAI)
#define ESP32_TWAI_CLASS(klass) OBJECT_CLASS_CHECK(Esp32TWAIClass, klass, TYPE_ESP32_TWAI)
#define ESP32_TWAI_GET_CLASS(obj) OBJECT_GET_CLASS(Esp32TWAIClass, obj, TYPE_ESP32_TWAI)

/* ESP32 uses 32-bit aligned addresses, so multiply SJA1000 memory size by 4 */
#define ESP32_TWAI_MEM_SIZE (CAN_SJA_MEM_SIZE << 2)

/*
 * The controller's lines, by the GPIO matrix's signals: twai_tx,
 * twai_bus_off_on and twai_clkout out (named gpio-outs), twai_rx in.
 */
#define ESP32_TWAI_TX       "esp32-twai-tx"
#define ESP32_TWAI_BUS_OFF  "esp32-twai-bus-off"
#define ESP32_TWAI_CLKOUT   "esp32-twai-clkout"
#define ESP32_TWAI_RX       "esp32-twai-rx"

typedef struct Esp32TWAIState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegionOps twai_ops;     /* TWAI MMIO operations */
    CanSJA1000State sja_state;    /* Underlying SJA1000 controller state */
    qemu_irq irq;                 /* System bus IRQ */
    qemu_irq irq_handler;         /* Interrupt proxy handler */
    CanBusState *canbus;         /* CAN bus interface */

    /*
     * The CAN bus backend stands for the bus behind a transceiver on the
     * pads the controller is routed to: the controller is on it only
     * while twai_tx drives a pad and twai_rx comes from one.
     */
    Esp32GpioState *gpio;
    bool connected;
    Notifier route_notifier;

    /* APB_CLK, which CLKOUT divides; the SoC stops it when gated */
    Clock *apb_clk;
    qemu_irq tx_out;
    qemu_irq bus_off_out;
    qemu_irq clkout_out;
    bool clkout_level;
    /* twai_rx as the matrix delivers it */
    bool rx_level;
    Notifier line_source;
} Esp32TWAIState;

typedef struct Esp32TWAIClass {
    SysBusDeviceClass parent_class;
    
    /* Virtual methods for MMIO operations */
    void (*twai_write)(void *opaque, hwaddr addr, uint64_t value, unsigned int size);
    uint64_t (*twai_read)(void *opaque, hwaddr addr, unsigned int size);
} Esp32TWAIClass;

#endif /* ESP32_TWAI_H */