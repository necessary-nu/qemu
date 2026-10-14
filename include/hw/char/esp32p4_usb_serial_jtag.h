/*
 * ESP32-P4 USB Serial/JTAG controller: the CDC-ACM serial port
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_ESP32P4_USB_SERIAL_JTAG_H
#define HW_CHAR_ESP32P4_USB_SERIAL_JTAG_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_USB_SERIAL_JTAG "esp32p4-usb-serial-jtag"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4UsjState, ESP32P4_USB_SERIAL_JTAG)

#define ESP32P4_USJ_BASE        0x500d2000
#define ESP32P4_USJ_MMIO_SIZE   0x1000
#define ESP32P4_USJ_NREGS       (0x8c / 4)
/* The CDC-ACM bulk endpoints' packets */
#define ESP32P4_USJ_EP_SIZE     64

/*
 * How the machine connects it
 * ===========================
 *
 * Property "chardev": the USB host's end of the CDC-ACM serial port, a
 * computer on the other end of the board's USB cable. With none, nothing
 * is plugged in. The host enumerates the device once it sees its D+
 * pull-up, then moves data while the chardev is open, as the host's
 * terminal program having the port open:
 *
 *   -chardev stdio,id=usj -global esp32p4-usb-serial-jtag.chardev=usj
 *
 * Clock inputs: "usb48", the PHY and device clock (USB_DEVICE_48M_CLK),
 * without which the USB side stops. The registers are mapped behind
 * USB_DEVICE_APB_CLK and the USB_DEVICE reset. Sysbus IRQ 0 is
 * USB_DEVICE_INTR.
 *
 * Named GPIO outputs, to the full-speed PHY the controller has, which
 * USB_WRAP routes (hw/usb/esp32p4_usb.c): ESP32P4_USJ_PAD_PULL_UP, lines
 * 0 and 1 the pull-ups the PHY puts on its first pad (D-; GPIO24 on PHY
 * 0) and its second (D+; GPIO25); ESP32P4_USJ_PAD_ENABLE, high while
 * USB_PAD_ENABLE gives the PHY its pads.
 */
#define ESP32P4_USJ_PAD_PULL_UP "pad-pull-up"
#define ESP32P4_USJ_PAD_ENABLE  "pad-enable"

/* The host's view of the device, from plug-in to configured. */
enum {
    ESP32P4_USJ_DETACHED,
    ESP32P4_USJ_DEBOUNCE,
    ESP32P4_USJ_BUS_RESET,
    ESP32P4_USJ_RECOVERY,
    ESP32P4_USJ_ENUMERATING,
    ESP32P4_USJ_CONFIGURED,
};

struct ESP32P4UsjState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq pull_up[2];
    qemu_irq pad_enable;
    Clock *usb48;
    CharFrontend chr;

    uint32_t regs[ESP32P4_USJ_NREGS];
    uint32_t int_raw;

    /* The IN endpoint's buffer, and whether it is flushed for the host. */
    uint8_t in_buf[ESP32P4_USJ_EP_SIZE];
    uint8_t in_len;
    bool in_flushed;
    /* The OUT endpoint's packet and how much of it software has read. */
    uint8_t out_buf[ESP32P4_USJ_EP_SIZE];
    uint8_t out_len;
    uint8_t out_rd;

    /* The host: its state, the frame it is in, and the requests to make. */
    QEMUTimer host_timer;
    /* When host_timer is due: the next state change or frame. */
    int64_t host_next;
    uint8_t host_state;
    uint32_t host_step;
    int64_t host_deadline;
    uint32_t frame;
    bool host_dtr;
    bool host_rts;
    bool line_state_pending;
    /* A bulk transaction on the bus: IN (true) or OUT, ending at xfer_end. */
    QEMUTimer xfer_timer;
    bool xfer_busy;
    bool xfer_in;
    int64_t xfer_end;
    uint8_t xfer_buf[ESP32P4_USJ_EP_SIZE];
    uint8_t xfer_len;
    /* What the host has for the chardev, and from it. */
    Fifo8 to_host;
    Fifo8 from_host;
    guint watch;
};

#endif
