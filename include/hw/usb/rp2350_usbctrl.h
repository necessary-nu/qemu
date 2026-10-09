/*
 * RP2350 USB controller (USBCTRL) with its DPRAM
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_RP2350_USBCTRL_H
#define HW_USB_RP2350_USBCTRL_H

#include "hw/core/sysbus.h"
#include "hw/usb/usb.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_USBCTRL "rp2350-usbctrl"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350USBCtrlState, RP2350_USBCTRL)

/*
 * MMIO regions: the 4 KiB DPRAM (USBCTRL_DPRAM, no atomic aliases) and
 * the register block with its XOR/SET/CLR aliases (USBCTRL_REGS).
 */
#define RP2350_USBCTRL_MMIO_DPRAM 0
#define RP2350_USBCTRL_MMIO_REGS 1
#define RP2350_USBCTRL_DPRAM_SIZE 0x1000
#define RP2350_USBCTRL_REGS_SIZE 0x4000

#define RP2350_USBCTRL_ENDPOINTS 16

/*
 * Named GPIO inputs "overcurr-detect" and "vbus-detect", and output
 * "vbus-en": the controller's VBUS management signals, which the IO
 * muxing routes to GPIO pins.
 */
#define RP2350_USBCTRL_OVERCURR_DETECT "overcurr-detect"
#define RP2350_USBCTRL_VBUS_DETECT "vbus-detect"
#define RP2350_USBCTRL_VBUS_EN "vbus-en"

/*
 * The USB port. In host mode (MAIN_CTRL.HOST_NDEVICE) it is the root port
 * of a QEMU USB bus, so QEMU USB devices (-device usb-kbd and the like)
 * plug into it. In device mode the RP2350 is the device on the port, and
 * what sits at the other end of the cable is one of:
 *
 *  - the CDC-ACM bridge ("cdc-chardev" property): a built-in host that
 *    enumerates the device, configures it, raises DTR while the chardev
 *    is open and carries the first CDC-ACM data interface's bulk IN and
 *    OUT endpoints to and from the chardev;
 *
 *  - the token port ("token-chardev" property): a raw USB host interface
 *    for external drivers and tests, connected while the chardev is.
 *    Each request is an 8-byte header followed by `len` bytes of data:
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
 *    SOFs and resets only when told to.
 *
 * With neither, nothing is plugged into the port in device mode.
 */

/* What a device-side transaction got from the device controller. */
typedef enum RP2350USBHandshake {
    RP2350_USB_ACK,
    RP2350_USB_NAK,
    RP2350_USB_STALL,
    /* No handshake: not addressed, endpoint disabled, or isochronous. */
    RP2350_USB_NORESP,
} RP2350USBHandshake;

/* Built-in host on the device port. */
typedef enum RP2350USBBridgeState {
    BRIDGE_DETACHED,
    BRIDGE_DEBOUNCE,
    BRIDGE_RESET,
    BRIDGE_RECOVERY,
    BRIDGE_ENUMERATE,
    BRIDGE_RUNNING,
    BRIDGE_FAILED,
} RP2350USBBridgeState;

#define RP2350_USB_BRIDGE_CFG_MAX 512
#define RP2350_USB_BRIDGE_TX_MAX 4096
#define RP2350_USB_TOKEN_MAX (8 + 1024)

struct RP2350USBCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion dpram_mr;
    MemoryRegion regs_mr;
    qemu_irq irq;
    qemu_irq vbus_en;

    uint8_t dpram[RP2350_USBCTRL_DPRAM_SIZE];

    /* Registers, as stored; read-only fields are derived on read. */
    uint32_t addr_endp[RP2350_USBCTRL_ENDPOINTS];
    uint32_t main_ctrl;
    uint32_t sof_wr;
    uint32_t sof_rd;
    uint32_t sie_ctrl;
    uint32_t sie_status;
    uint32_t int_ep_ctrl;
    uint32_t buff_status;
    uint32_t buff_cpu_should_handle;
    uint32_t ep_abort;
    uint32_t ep_abort_done;
    uint32_t ep_stall_arm;
    uint32_t nak_poll;
    uint32_t nak_retry;
    uint32_t ep_status_stall_nak;
    uint32_t usb_muxing;
    uint32_t usb_pwr;
    uint32_t usbphy_direct;
    uint32_t usbphy_direct_override;
    uint32_t usbphy_trim;
    uint32_t linestate_tuning;
    uint32_t inte;
    uint32_t intf;
    uint32_t sof_timestamp_last;
    uint32_t ep_tx_error;
    uint32_t ep_rx_error;
    uint32_t dev_sm_watchdog;
    /*
     * INTR sources that are events rather than status bits: DEV_SOF,
     * HOST_SOF, DEV_SUSPEND, DEV_CONN_DIS, DEV_RESUME_FROM_HOST,
     * HOST_RESUME and HOST_CONN_DIS, in their INTR positions.
     */
    uint32_t intr_events;
    /*
     * A buffer that completed while its BUFF_STATUS bit was still set:
     * clearing the bit sets it again for this buffer.
     */
    uint32_t buff_status_pending;
    uint32_t buff_pending_sel;

    bool vbus_detect_in;
    bool overcurr_in;

    /* Device controller */
    /* Buffer select of each endpoint (bit 2n IN, 2n+1 OUT). */
    uint32_t dev_buf_sel;
    bool dev_connected;
    bool dev_suspended;
    /* Bus state as last seen, for the pull-up's connect/disconnect. */
    bool dev_present;
    QEMUTimer *dev_idle_timer;

    /* Host controller */
    USBBus bus;
    USBPort port;
    USBPacket packet;
    uint8_t host_buf[1024];
    QEMUTimer *host_frame_timer;
    int64_t host_frame_ns;
    QEMUTimer *host_timer;
    uint8_t host_speed;
    /* EPX transfer state */
    bool epx_active;
    uint8_t epx_phase;
    uint8_t epx_sel;
    bool epx_wait_sof;
    bool epx_wait_buf;
    int64_t epx_retry_ns;
    /* Interrupt endpoint polling: due slots and frames to the next poll. */
    uint32_t int_due;
    uint16_t int_countdown[RP2350_USBCTRL_ENDPOINTS];
    /* The transaction in flight, while a QEMU device completes it. */
    bool xact_async;
    uint8_t xact_slot;
    uint8_t xact_pid;
    uint8_t xact_addr;
    uint8_t xact_ep;
    uint8_t xact_sel;
    uint16_t xact_len;
    /* Device data toggles, per address: bit 2n IN, 2n+1 OUT of endpoint n. */
    uint32_t dev_toggle[128];
    /* The device stalled a SETUP's request: its next data or status. */
    bool ctrl_stall;
    uint8_t ctrl_stall_addr;

    /* Device-port hosts */
    CharFrontend cdc_chr;
    CharFrontend token_chr;
    bool token_open;
    bool cdc_open;

    uint32_t br_state;
    QEMUTimer *br_timer;
    QEMUTimer *br_frame_timer;
    uint16_t br_frame;
    int64_t br_frame_ns;
    uint8_t br_addr;
    uint8_t br_mps0;
    uint8_t br_step;
    uint8_t br_attempt;
    uint8_t br_config;
    int br_comm_if;
    uint8_t br_in_ep;
    uint8_t br_out_ep;
    uint16_t br_in_mps;
    uint16_t br_out_mps;
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
    uint8_t br_ctl_data[RP2350_USB_BRIDGE_CFG_MAX];
    uint16_t br_ctl_len;
    uint16_t br_ctl_done;
    uint8_t br_ctl_stage;
    uint8_t br_ctl_toggle;
    /* Chardev data waiting for bulk OUT. */
    uint8_t br_tx[RP2350_USB_BRIDGE_TX_MAX];
    uint32_t br_tx_len;

    uint8_t tok_buf[RP2350_USB_TOKEN_MAX];
    uint32_t tok_len;
};

/*
 * The device controller's side of the cable, for the hosts on the device
 * port: each call is one transaction, completed before it returns.
 */
bool rp2350_usbctrl_dev_present(RP2350USBCtrlState *s);
void rp2350_usbctrl_dev_bus_reset(RP2350USBCtrlState *s);
void rp2350_usbctrl_dev_sof(RP2350USBCtrlState *s, uint16_t frame);
RP2350USBHandshake rp2350_usbctrl_dev_setup(RP2350USBCtrlState *s,
                                            uint8_t addr, int pid,
                                            const uint8_t *data);
RP2350USBHandshake rp2350_usbctrl_dev_in(RP2350USBCtrlState *s, uint8_t addr,
                                         uint8_t ep, uint8_t *buf,
                                         uint16_t max, uint16_t *len,
                                         int *pid);
RP2350USBHandshake rp2350_usbctrl_dev_out(RP2350USBCtrlState *s,
                                          uint8_t addr, uint8_t ep, int pid,
                                          const uint8_t *data, uint16_t len);
/* Whether a host is at the other end of the cable in device mode. */
bool rp2350_usbctrl_host_attached(RP2350USBCtrlState *s);
void rp2350_usbctrl_update_dev_port(RP2350USBCtrlState *s);

/* The device-port hosts, in rp2350_usbctrl_bridge.c. */
void rp2350_usb_bridge_realize(RP2350USBCtrlState *s);
void rp2350_usb_bridge_reset(RP2350USBCtrlState *s);
/* The device's presence on the bus changed. */
void rp2350_usb_bridge_port_changed(RP2350USBCtrlState *s);
/* Software armed a device buffer: a NAKed transaction may now succeed. */
void rp2350_usb_bridge_kick(RP2350USBCtrlState *s);
extern const VMStateDescription vmstate_rp2350_usb_bridge;

#endif
