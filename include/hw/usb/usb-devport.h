/*
 * USB device-port hosts: the host at the other end of a device controller's
 * cable
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A USB device controller in device mode is the device on its port; what
 * sits at the other end of the cable is one of two built-in hosts, both
 * full-speed hosts:
 *
 *  - the CDC-ACM bridge ("cdc-chardev" property of the controller): once
 *    the device's pull-up has been on for the 100 ms attach debounce it
 *    resets the bus for 10 ms, allows 10 ms of reset recovery, then sends
 *    an SOF every millisecond and enumerates the device at address 1. It
 *    reads the configuration descriptor, sets the configuration, gives the
 *    first CDC-ACM communication interface a line coding of 115200 8N1 and
 *    drives its DTR and RTS from whether the chardev is open, then moves
 *    data between the chardev and the bulk endpoints of the first CDC data
 *    interface. A NAKed transaction is retried when the device's software
 *    next arms a buffer, and at the latest in the next frame.
 *
 *  - the token port ("token-chardev" property): a raw USB host interface
 *    for external drivers and tests, connected while the chardev is. Each
 *    request is an 8-byte header followed by `len` bytes of data:
 *
 *      byte 0     op: 'S' SETUP (8 bytes of data), 'O' OUT, 'I' IN (len
 *                 is the most the host accepts), 'R' bus reset, 'F' SOF
 *                 (len is the frame number), 'P' port status
 *      byte 1     device address
 *      byte 2     endpoint number
 *      byte 3     data PID of SETUP and OUT data: 0 DATA0, 1 DATA1
 *      bytes 4-5  len, little-endian
 *      bytes 6-7  reserved, zero
 *
 *    and each gets a 4-byte reply, followed for an acknowledged IN by
 *    the data:
 *
 *      byte 0     handshake: 'A' ACK, 'N' NAK, 'S' STALL, 'T' no reply
 *      byte 1     IN: the data PID the device sent; 'P': port flags
 *                 (bit 0 the device's pull-up is connected, bit 1 the
 *                 device is suspended)
 *      bytes 2-3  IN: data length, little-endian
 *
 *    Transactions happen when they are requested: the token port sends
 *    SOFs and resets only when told to. This is the protocol of the
 *    RP2350's token port, so the same clients drive either machine.
 *
 * With neither, nothing is plugged into the port in device mode.
 */

#ifndef HW_USB_USB_DEVPORT_H
#define HW_USB_USB_DEVPORT_H

#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* What a transaction got from the device controller. */
typedef enum USBDevPortHandshake {
    USB_DEVPORT_ACK,
    USB_DEVPORT_NAK,
    USB_DEVPORT_STALL,
    /* No handshake: not addressed, endpoint disabled, or isochronous. */
    USB_DEVPORT_NORESP,
} USBDevPortHandshake;

/*
 * The device controller's side of the cable: each call is one
 * transaction, completed before it returns.
 */
typedef struct USBDevPortOps {
    /* The device's D+ pull-up is connected. */
    bool (*present)(void *opaque);
    /* The device controller has seen the bus suspend. */
    bool (*suspended)(void *opaque);
    /* A host is now (or is no longer) at the other end of the cable. */
    void (*host_attached)(void *opaque, bool attached);
    void (*bus_reset)(void *opaque, bool asserted);
    void (*sof)(void *opaque, uint16_t frame);
    USBDevPortHandshake (*setup)(void *opaque, uint8_t addr,
                                 const uint8_t *data);
    USBDevPortHandshake (*in)(void *opaque, uint8_t addr, uint8_t ep,
                              uint8_t *buf, uint16_t max, uint16_t *len,
                              int *pid);
    USBDevPortHandshake (*out)(void *opaque, uint8_t addr, uint8_t ep,
                               int pid, const uint8_t *data, uint16_t len);
} USBDevPortOps;

/* The CDC-ACM bridge's states. */
typedef enum USBDevPortBridgeState {
    USB_DEVPORT_DETACHED,
    USB_DEVPORT_DEBOUNCE,
    USB_DEVPORT_RESET,
    USB_DEVPORT_RECOVERY,
    USB_DEVPORT_ENUMERATE,
    USB_DEVPORT_RUNNING,
    USB_DEVPORT_FAILED,
} USBDevPortBridgeState;

#define USB_DEVPORT_CFG_MAX     512
#define USB_DEVPORT_TX_MAX      4096
#define USB_DEVPORT_TOKEN_MAX   (8 + 1024)

typedef struct USBDevPort {
    const USBDevPortOps *ops;
    void *opaque;
    const char *name;

    /* The controller's "cdc-chardev" and "token-chardev" properties. */
    CharFrontend cdc_chr;
    CharFrontend token_chr;
    bool token_open;

    uint32_t br_state;
    QEMUTimer *br_timer;
    /* When br_timer is due: the time of the bridge's next step. */
    int64_t br_due;
    QEMUTimer *br_frame_timer;
    uint16_t br_frame;
    int64_t br_frame_ns;
    uint8_t br_addr;
    uint8_t br_mps0;
    uint8_t br_step;
    uint8_t br_attempt;
    uint8_t br_config;
    int32_t br_comm_if;
    uint8_t br_in_ep;
    uint8_t br_out_ep;
    uint16_t br_in_mps;
    uint16_t br_out_mps;
    /* Data toggles of the bulk endpoints: bit 2n IN, 2n+1 OUT. */
    uint32_t br_toggle;
    /* DTR as the chardev wants it, and as last sent to the device. */
    bool br_dtr;
    bool br_dtr_line;
    bool br_dtr_sent;
    bool br_waiting;
    /* Bulk IN goes next, rather than OUT. */
    bool br_turn;
    /* The control transfer in progress. */
    uint8_t br_setup[8];
    uint8_t br_ctl_data[USB_DEVPORT_CFG_MAX];
    uint16_t br_ctl_len;
    uint16_t br_ctl_done;
    uint8_t br_ctl_stage;
    uint8_t br_ctl_toggle;
    /* Chardev data waiting for bulk OUT. */
    uint8_t br_tx[USB_DEVPORT_TX_MAX];
    uint32_t br_tx_len;

    uint8_t tok_buf[USB_DEVPORT_TOKEN_MAX];
    uint32_t tok_len;
} USBDevPort;

void usb_devport_init(USBDevPort *p, const char *name,
                      const USBDevPortOps *ops, void *opaque);
/* Fails if both chardevs are set: they are both hosts on the one port. */
bool usb_devport_realize(USBDevPort *p, Error **errp);
void usb_devport_reset(USBDevPort *p);
/* Whether a host is at the other end of the cable. */
bool usb_devport_host_attached(USBDevPort *p);
/* The device's presence on the bus changed. */
void usb_devport_changed(USBDevPort *p);
/* The device's software armed a buffer: a NAKed transaction may succeed. */
void usb_devport_kick(USBDevPort *p);
/*
 * The bridge's earliest event pending, if it is due no later than *@due:
 * set *@due, *@timer and *@run, which takes @p, for the device controller
 * to run it at its time (line_run_at()) once the timer is deleted, in
 * order with its own events, before a register access.
 */
bool usb_devport_next(USBDevPort *p, int64_t *due, QEMUTimer **timer,
                      void (**run)(void *opaque));

extern const VMStateDescription vmstate_usb_devport;

#define VMSTATE_USB_DEVPORT(_field, _state) \
    VMSTATE_STRUCT(_field, _state, 1, vmstate_usb_devport, USBDevPort)

#endif
