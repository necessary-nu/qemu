/*
 * RP2350 USB controller (USBCTRL) with its DPRAM
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "USB", and the pico-sdk usb.h and
 * usb_device_dpram.h register descriptions. The TinyUSB rp2040 device and
 * host ports are the reference software for the block.
 *
 * The controller is a full-speed device controller and a full/low-speed
 * host controller sharing one SIE, configured through registers at
 * USBCTRL_REGS and exchanging endpoint configuration, buffer control and
 * data with software through the 4 KiB DPRAM at USBCTRL_DPRAM.
 *
 * Device mode: the device controller answers each token from the host on
 * the port from the endpoint and buffer control words in DPRAM, as the
 * datasheet's IN, OUT and SETUP state machines describe. A buffer the
 * software has not handed over (AVAILABLE) is NAKed, so transfers
 * complete asynchronously as software arms buffers. What is at the host
 * end of the cable is in rp2350_usbctrl_bridge.c.
 *
 * Host mode: the port is the root port of a QEMU USB bus. The host
 * controller runs software-started transfers on EPX (SIE_CTRL.START_TRANS)
 * and polls the interrupt endpoint slots enabled in INT_EP_CTRL after each
 * SOF, turning each transaction into a USB packet to the addressed QEMU
 * device. QEMU devices do not model data toggles, so the controller keeps
 * each device endpoint's toggle as a device would, which lets it raise
 * DATA_SEQ_ERROR when software's PIDs go out of step.
 *
 * Transactions take bus time at 12 Mb/s (1.5 Mb/s for a low-speed
 * device), frames are 1 ms of QEMU_CLOCK_VIRTUAL, and a NAKed EPX
 * transaction is retried after NAK_POLL's delay.
 *
 * Not modelled: the PHY's direct drive (SIE_CTRL.DIRECT_*, USBPHY_DIRECT
 * overrides and the DP/DM pins as GPIO), the external PHY and digital-pad
 * muxing (the port is disconnected unless USB_MUXING.TO_PHY), isochronous
 * timing within a frame, and the line-level errors (CRC, bit stuffing,
 * RX overflow) that an emulated bus never produces. The device state
 * machine never stalls, so DEV_SM_WATCHDOG never fires.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/usb/rp2350_usbctrl.h"
#include "migration/vmstate.h"

#define A_ADDR_ENDP                 0x000
#define A_ADDR_ENDP15               0x03c
#define A_MAIN_CTRL                 0x040
#define A_SOF_WR                    0x044
#define A_SOF_RD                    0x048
#define A_SIE_CTRL                  0x04c
#define A_SIE_STATUS                0x050
#define A_INT_EP_CTRL               0x054
#define A_BUFF_STATUS               0x058
#define A_BUFF_CPU_SHOULD_HANDLE    0x05c
#define A_EP_ABORT                  0x060
#define A_EP_ABORT_DONE             0x064
#define A_EP_STALL_ARM              0x068
#define A_NAK_POLL                  0x06c
#define A_EP_STATUS_STALL_NAK       0x070
#define A_USB_MUXING                0x074
#define A_USB_PWR                   0x078
#define A_USBPHY_DIRECT             0x07c
#define A_USBPHY_DIRECT_OVERRIDE    0x080
#define A_USBPHY_TRIM               0x084
#define A_LINESTATE_TUNING          0x088
#define A_INTR                      0x08c
#define A_INTE                      0x090
#define A_INTF                      0x094
#define A_INTS                      0x098
#define A_SOF_TIMESTAMP_RAW         0x100
#define A_SOF_TIMESTAMP_LAST        0x104
#define A_SM_STATE                  0x108
#define A_EP_TX_ERROR               0x10c
#define A_EP_RX_ERROR               0x110
#define A_DEV_SM_WATCHDOG           0x114

#define ADDR_ENDP_MASK              0x000f007fu
#define ADDR_ENDPN_MASK             0x060f007fu
#define ADDR_ENDP_ADDRESS           0x7fu
#define ADDR_ENDP_ENDPOINT_SHIFT    16
#define ADDR_ENDP_INTEP_DIR         (1u << 25)

#define MAIN_CTRL_CONTROLLER_EN     (1u << 0)
#define MAIN_CTRL_HOST_NDEVICE      (1u << 1)
#define MAIN_CTRL_PHY_ISO           (1u << 2)
#define MAIN_CTRL_SIM_TIMING        (1u << 31)
#define MAIN_CTRL_MASK              0x80000007u
#define MAIN_CTRL_RESET             MAIN_CTRL_PHY_ISO

#define FRAME_MASK                  0x7ffu

#define SIE_CTRL_START_TRANS        (1u << 0)
#define SIE_CTRL_SEND_SETUP         (1u << 1)
#define SIE_CTRL_SEND_DATA          (1u << 2)
#define SIE_CTRL_RECEIVE_DATA       (1u << 3)
#define SIE_CTRL_STOP_TRANS         (1u << 4)
#define SIE_CTRL_PREAMBLE_EN        (1u << 6)
#define SIE_CTRL_SOF_SYNC           (1u << 8)
#define SIE_CTRL_SOF_EN             (1u << 9)
#define SIE_CTRL_KEEP_ALIVE_EN      (1u << 10)
#define SIE_CTRL_VBUS_EN            (1u << 11)
#define SIE_CTRL_RESUME             (1u << 12)
#define SIE_CTRL_RESET_BUS          (1u << 13)
#define SIE_CTRL_PULLDOWN_EN        (1u << 15)
#define SIE_CTRL_PULLUP_EN          (1u << 16)
#define SIE_CTRL_TRANSCEIVER_PD     (1u << 18)
#define SIE_CTRL_EP0_STOP_ON_SHORT  (1u << 19)
#define SIE_CTRL_DIRECT_EN          (1u << 26)
#define SIE_CTRL_EP0_INT_NAK        (1u << 27)
#define SIE_CTRL_EP0_INT_2BUF       (1u << 28)
#define SIE_CTRL_EP0_INT_1BUF       (1u << 29)
#define SIE_CTRL_EP0_DOUBLE_BUF     (1u << 30)
#define SIE_CTRL_EP0_INT_STALL      (1u << 31)
#define SIE_CTRL_SC                 (SIE_CTRL_START_TRANS | \
                                     SIE_CTRL_STOP_TRANS | \
                                     SIE_CTRL_RESUME | SIE_CTRL_RESET_BUS)
#define SIE_CTRL_MASK               0xff0fbf5fu
#define SIE_CTRL_RESET              SIE_CTRL_PULLDOWN_EN

#define SIE_STATUS_VBUS_DETECTED    (1u << 0)
#define SIE_STATUS_LINE_STATE_SHIFT 2
#define SIE_STATUS_SUSPENDED        (1u << 4)
#define SIE_STATUS_SPEED_SHIFT      8
#define SIE_STATUS_SPEED            (3u << 8)
#define SIE_STATUS_VBUS_OVER_CURR   (1u << 10)
#define SIE_STATUS_RESUME           (1u << 11)
#define SIE_STATUS_RX_SHORT_PACKET  (1u << 12)
#define SIE_STATUS_CONNECTED        (1u << 16)
#define SIE_STATUS_SETUP_REC        (1u << 17)
#define SIE_STATUS_TRANS_COMPLETE   (1u << 18)
#define SIE_STATUS_BUS_RESET        (1u << 19)
#define SIE_STATUS_ENDPOINT_ERROR   (1u << 23)
#define SIE_STATUS_CRC_ERROR        (1u << 24)
#define SIE_STATUS_BIT_STUFF_ERROR  (1u << 25)
#define SIE_STATUS_RX_OVERFLOW      (1u << 26)
#define SIE_STATUS_RX_TIMEOUT       (1u << 27)
#define SIE_STATUS_NAK_REC          (1u << 28)
#define SIE_STATUS_STALL_REC        (1u << 29)
#define SIE_STATUS_ACK_REC          (1u << 30)
#define SIE_STATUS_DATA_SEQ_ERROR   (1u << 31)
#define SIE_STATUS_WC               0xff8e1800u

#define INT_EP_CTRL_MASK            0x0000fffeu

#define NAK_POLL_DELAY_LS           0x000003ffu
#define NAK_POLL_RETRY_LO_SHIFT     10
#define NAK_POLL_DELAY_FS_SHIFT     16
#define NAK_POLL_STOP_EPX_ON_NAK    (1u << 26)
#define NAK_POLL_EPX_STOPPED_ON_NAK (1u << 27)
#define NAK_POLL_RETRY_HI_SHIFT     28
#define NAK_POLL_RW                 0x07ff03ffu
#define NAK_POLL_RESET              0x00100010u

#define USB_MUXING_TO_PHY           (1u << 0)
#define USB_MUXING_MASK             0x8000001fu
/* TO_EXTPHY, TO_DIGITAL_PAD and USBPHY_AS_GPIO */
#define USB_MUXING_UNMODELLED       0x00000016u
#define USB_MUXING_RESET            USB_MUXING_TO_PHY

#define USB_PWR_VBUS_EN             (1u << 0)
#define USB_PWR_VBUS_EN_OVERRIDE_EN (1u << 1)
#define USB_PWR_VBUS_DETECT         (1u << 2)
#define USB_PWR_VBUS_DETECT_OVERRIDE_EN (1u << 3)
#define USB_PWR_OVERCURR_DETECT     (1u << 4)
#define USB_PWR_OVERCURR_DETECT_EN  (1u << 5)
#define USB_PWR_MASK                0x3fu

/* USBPHY_DIRECT: RX_DD, RX_DP, RX_DM and the OVCN/OVV flags read back. */
#define USBPHY_DIRECT_RX_DD         (1u << 16)
#define USBPHY_DIRECT_RX_DP         (1u << 17)
#define USBPHY_DIRECT_RX_DM         (1u << 18)
#define USBPHY_DIRECT_RW            0x0380ff77u
#define USBPHY_DIRECT_OVERRIDE_MASK 0x00079fffu
#define USBPHY_TRIM_MASK            0x00001f1fu
#define USBPHY_TRIM_RESET           0x00001f1fu
#define LINESTATE_TUNING_MASK       0x00000fffu
#define LINESTATE_TUNING_RESET      0x000000f8u

#define INTR_HOST_CONN_DIS          (1u << 0)
#define INTR_HOST_RESUME            (1u << 1)
#define INTR_HOST_SOF               (1u << 2)
#define INTR_TRANS_COMPLETE         (1u << 3)
#define INTR_BUFF_STATUS            (1u << 4)
#define INTR_ERROR_DATA_SEQ         (1u << 5)
#define INTR_ERROR_RX_TIMEOUT       (1u << 6)
#define INTR_ERROR_RX_OVERFLOW      (1u << 7)
#define INTR_ERROR_BIT_STUFF        (1u << 8)
#define INTR_ERROR_CRC              (1u << 9)
#define INTR_STALL                  (1u << 10)
#define INTR_VBUS_DETECT            (1u << 11)
#define INTR_BUS_RESET              (1u << 12)
#define INTR_DEV_CONN_DIS           (1u << 13)
#define INTR_DEV_SUSPEND            (1u << 14)
#define INTR_DEV_RESUME_FROM_HOST   (1u << 15)
#define INTR_SETUP_REQ              (1u << 16)
#define INTR_DEV_SOF                (1u << 17)
#define INTR_ABORT_DONE             (1u << 18)
#define INTR_EP_STALL_NAK           (1u << 19)
#define INTR_RX_SHORT_PACKET        (1u << 20)
#define INTR_ENDPOINT_ERROR         (1u << 21)
#define INTR_DEV_SM_WATCHDOG_FIRED  (1u << 22)
#define INTR_EPX_STOPPED_ON_NAK     (1u << 23)
#define INTR_MASK                   0x00ffffffu

#define SOF_TIMESTAMP_MASK          0x001fffffu

#define DEV_SM_WATCHDOG_RW          0x000fffffu
#define DEV_SM_WATCHDOG_FIRED       (1u << 20)

#define DPRAM_SETUP                 0x000
#define DPRAM_EP_CTRL(n, out)       (0x008 + ((n) - 1) * 8 + (out) * 4)
#define DPRAM_BUF_CTRL(n, out)      (0x080 + (n) * 8 + (out) * 4)
#define DPRAM_EP0_BUF               0x100
#define DPRAM_EPX_BUF_CTRL          0x080
#define DPRAM_EPX_CTRL              0x100
#define DPRAM_BUF_CTRL_START        0x080
#define DPRAM_BUF_CTRL_END          0x100

/* Endpoint control words */
#define EP_CTRL_ENABLE              (1u << 31)
#define EP_CTRL_DOUBLE_BUFFERED     (1u << 30)
#define EP_CTRL_INT_PER_BUFF        (1u << 29)
#define EP_CTRL_INT_PER_DOUBLE_BUFF (1u << 28)
#define EP_CTRL_TYPE_SHIFT          26
#define EP_CTRL_INT_ON_STALL        (1u << 17)
#define EP_CTRL_INT_ON_NAK          (1u << 16)
#define EP_CTRL_HOST_INTERVAL_SHIFT 16
#define EP_CTRL_HOST_INTERVAL_MASK  0x3ffu
#define EP_CTRL_BUFFER_ADDRESS      0xffc0u

#define EP_TYPE_CONTROL             0
#define EP_TYPE_ISO                 1

/*
 * Buffer control words: the buffer 0 half, as also used for buffer 1 in
 * bits 31:16. RESET (buffer select) and STALL exist only in the low half;
 * bits 28:27 of the high half are the isochronous buffer 1 offset.
 */
#define BUF_FULL                    (1u << 15)
#define BUF_LAST                    (1u << 14)
#define BUF_PID                     (1u << 13)
#define BUF_RESET_SEL               (1u << 12)
#define BUF_STALL                   (1u << 11)
#define BUF_AVAILABLE               (1u << 10)
#define BUF_LEN                     0x3ffu
#define BUF_ISO_OFFSET_SHIFT        27

#define DPRAM_MASK                  (RP2350_USBCTRL_DPRAM_SIZE - 1)

/* Full-speed bit time, and the bytes of a transaction besides its data. */
#define FS_BIT_NS                   (NANOSECONDS_PER_SECOND / 12000000)
#define LS_BIT_NS                   (8 * FS_BIT_NS)
#define XACT_OVERHEAD_BYTES         13
#define FRAME_NS                    (NANOSECONDS_PER_SECOND / 1000)
/* Bus idle that suspends a device, and the 48 MHz PHY clock. */
#define SUSPEND_NS                  (3 * FRAME_NS)
#define PHY_CLK_HZ                  48000000

enum {
    EPX_PHASE_SETUP,
    EPX_PHASE_DATA,
};

#define XACT_EPX                    0

static uint32_t dpram_ld(RP2350USBCtrlState *s, uint32_t off)
{
    return ldl_le_p(&s->dpram[off & DPRAM_MASK & ~3u]);
}

static void dpram_st(RP2350USBCtrlState *s, uint32_t off, uint32_t v)
{
    stl_le_p(&s->dpram[off & DPRAM_MASK & ~3u], v);
}

static void dpram_copy_in(RP2350USBCtrlState *s, uint32_t off,
                          const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        s->dpram[(off + i) & DPRAM_MASK] = buf[i];
    }
}

static void dpram_copy_out(RP2350USBCtrlState *s, uint32_t off, uint8_t *buf,
                           uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        buf[i] = s->dpram[(off + i) & DPRAM_MASK];
    }
}

/*
 * The controller writes a buffer's status back as a 16-bit write to its
 * half of the buffer control word.
 */
static void buf_ctrl_write_half(RP2350USBCtrlState *s, uint32_t off, int sel,
                                uint32_t half)
{
    uint32_t bc = dpram_ld(s, off);

    if (sel) {
        bc = (bc & 0xffffu) | (half << 16);
    } else {
        bc = (bc & 0xffff0000u) | (half & 0xffffu);
    }
    dpram_st(s, off, bc);
}

static uint32_t buf_ctrl_half(uint32_t bc, int sel)
{
    return sel ? bc >> 16 : bc & 0xffffu;
}

/*
 * The DPRAM address of buffer `sel`: buffer 1 follows buffer 0 by 64
 * bytes, or for an isochronous endpoint by the offset in its buffer
 * control word.
 */
static uint32_t buf_addr(uint32_t ep_ctrl, uint32_t bc, int sel)
{
    uint32_t base = ep_ctrl & EP_CTRL_BUFFER_ADDRESS;
    int type = extract32(ep_ctrl, EP_CTRL_TYPE_SHIFT, 2);

    if (sel) {
        base += type == EP_TYPE_ISO ?
                128u << extract32(bc, BUF_ISO_OFFSET_SHIFT, 2) : 64u;
    }
    return base;
}

static bool vbus_detected(RP2350USBCtrlState *s)
{
    if (s->usb_pwr & USB_PWR_VBUS_DETECT_OVERRIDE_EN) {
        return s->usb_pwr & USB_PWR_VBUS_DETECT;
    }
    return s->vbus_detect_in;
}

static bool overcurrent(RP2350USBCtrlState *s)
{
    if (s->usb_pwr & USB_PWR_OVERCURR_DETECT_EN) {
        return s->usb_pwr & USB_PWR_OVERCURR_DETECT;
    }
    return s->overcurr_in;
}

/* The controller drives the PHY, which is connected to the port. */
static bool phy_live(RP2350USBCtrlState *s)
{
    return (s->main_ctrl & MAIN_CTRL_CONTROLLER_EN) &&
           !(s->main_ctrl & MAIN_CTRL_PHY_ISO) &&
           (s->usb_muxing & USB_MUXING_TO_PHY) &&
           !(s->sie_ctrl & SIE_CTRL_TRANSCEIVER_PD);
}

static bool host_mode(RP2350USBCtrlState *s)
{
    return s->main_ctrl & MAIN_CTRL_HOST_NDEVICE;
}

static bool host_active(RP2350USBCtrlState *s)
{
    return phy_live(s) && host_mode(s);
}

/* [spec:nuos:req:emu.usb] */
bool rp2350_usbctrl_dev_present(RP2350USBCtrlState *s)
{
    return phy_live(s) && !host_mode(s) && (s->sie_ctrl & SIE_CTRL_PULLUP_EN);
}

bool rp2350_usbctrl_host_attached(RP2350USBCtrlState *s)
{
    return qemu_chr_fe_backend_connected(&s->cdc_chr) || s->token_open;
}

/* The bus line state, LINE_STATE's encoding: 0 SE0, 1 J, 2 K. */
static uint32_t line_state(RP2350USBCtrlState *s)
{
    if (host_mode(s)) {
        return s->host_speed == 1 ? 2 : s->host_speed ? 1 : 0;
    }
    return rp2350_usbctrl_dev_present(s) && !s->dev_bus_reset ? 1 : 0;
}

static uint32_t sie_status(RP2350USBCtrlState *s)
{
    uint32_t v = s->sie_status & SIE_STATUS_WC;

    if (vbus_detected(s)) {
        v |= SIE_STATUS_VBUS_DETECTED;
    }
    v |= line_state(s) << SIE_STATUS_LINE_STATE_SHIFT;
    if (!host_mode(s) && s->dev_suspended) {
        v |= SIE_STATUS_SUSPENDED;
    }
    if (host_mode(s)) {
        v |= (uint32_t)s->host_speed << SIE_STATUS_SPEED_SHIFT;
    }
    if (overcurrent(s)) {
        v |= SIE_STATUS_VBUS_OVER_CURR;
    }
    if (!host_mode(s) && s->dev_connected) {
        v |= SIE_STATUS_CONNECTED;
    }
    return v;
}

/* [spec:nuos:req:emu.usb] */
static uint32_t usb_intr(RP2350USBCtrlState *s)
{
    uint32_t st = sie_status(s);
    uint32_t v = s->intr_events;
    static const struct {
        uint32_t status;
        uint32_t intr;
    } from_status[] = {
        { SIE_STATUS_ENDPOINT_ERROR, INTR_ENDPOINT_ERROR },
        { SIE_STATUS_RX_SHORT_PACKET, INTR_RX_SHORT_PACKET },
        { SIE_STATUS_SETUP_REC, INTR_SETUP_REQ },
        { SIE_STATUS_BUS_RESET, INTR_BUS_RESET },
        { SIE_STATUS_VBUS_DETECTED, INTR_VBUS_DETECT },
        { SIE_STATUS_STALL_REC, INTR_STALL },
        { SIE_STATUS_CRC_ERROR, INTR_ERROR_CRC },
        { SIE_STATUS_BIT_STUFF_ERROR, INTR_ERROR_BIT_STUFF },
        { SIE_STATUS_RX_OVERFLOW, INTR_ERROR_RX_OVERFLOW },
        { SIE_STATUS_RX_TIMEOUT, INTR_ERROR_RX_TIMEOUT },
        { SIE_STATUS_DATA_SEQ_ERROR, INTR_ERROR_DATA_SEQ },
        { SIE_STATUS_TRANS_COMPLETE, INTR_TRANS_COMPLETE },
    };
    int i;

    for (i = 0; i < ARRAY_SIZE(from_status); i++) {
        if (st & from_status[i].status) {
            v |= from_status[i].intr;
        }
    }
    if (s->nak_poll & NAK_POLL_EPX_STOPPED_ON_NAK) {
        v |= INTR_EPX_STOPPED_ON_NAK;
    }
    if (s->dev_sm_watchdog & DEV_SM_WATCHDOG_FIRED) {
        v |= INTR_DEV_SM_WATCHDOG_FIRED;
    }
    if (s->ep_status_stall_nak) {
        v |= INTR_EP_STALL_NAK;
    }
    if (s->ep_abort_done) {
        v |= INTR_ABORT_DONE;
    }
    if (s->buff_status) {
        v |= INTR_BUFF_STATUS;
    }
    return v;
}

static uint32_t usb_ints(RP2350USBCtrlState *s)
{
    return (usb_intr(s) | s->intf) & s->inte;
}

static void usb_update_irq(RP2350USBCtrlState *s)
{
    qemu_set_irq(s->irq, usb_ints(s) != 0);
}

static void usb_update_vbus_en(RP2350USBCtrlState *s)
{
    bool en = s->usb_pwr & USB_PWR_VBUS_EN_OVERRIDE_EN ?
              s->usb_pwr & USB_PWR_VBUS_EN : s->sie_ctrl & SIE_CTRL_VBUS_EN;

    qemu_set_irq(s->vbus_en, en);
}

/*
 * A buffer completed. If its endpoint's BUFF_STATUS bit is still set for
 * the other buffer, the completion waits behind it: clearing the bit sets
 * it again for this buffer. A buffer completing again before software
 * has cleared it is not counted twice.
 */
/* [spec:nuos:req:emu.usb] */
static void buff_status_raise(RP2350USBCtrlState *s, int bit, int sel)
{
    uint32_t m = 1u << bit;

    if (s->buff_status & m) {
        if (!!(s->buff_cpu_should_handle & m) == !!sel) {
            return;
        }
        s->buff_status_pending |= m;
        s->buff_pending_sel = (s->buff_pending_sel & ~m) | (sel ? m : 0);
        return;
    }
    s->buff_status |= m;
    s->buff_cpu_should_handle = (s->buff_cpu_should_handle & ~m) |
                                (sel ? m : 0);
}

static void buff_status_clear(RP2350USBCtrlState *s, uint32_t clear)
{
    uint32_t again = clear & s->buff_status & s->buff_status_pending;

    s->buff_status &= ~clear;
    s->buff_status |= again;
    s->buff_cpu_should_handle = (s->buff_cpu_should_handle & ~again) |
                                (s->buff_pending_sel & again);
    s->buff_status_pending &= ~again;
}

/*
 * Whether a completed buffer raises BUFF_STATUS: for every buffer, or for
 * every second one of a double-buffered endpoint and at the end of a
 * transfer.
 */
static bool buff_done_raises(uint32_t ep_ctrl, int sel, bool end)
{
    if (ep_ctrl & EP_CTRL_INT_PER_BUFF) {
        return true;
    }
    return (ep_ctrl & EP_CTRL_INT_PER_DOUBLE_BUFF) && (sel || end);
}

/* Device controller */

/*
 * EP0 has no endpoint control word: its buffers are fixed at 0x100 and
 * 0x140 and its configuration comes from SIE_CTRL.
 */
static uint32_t dev_ep_ctrl(RP2350USBCtrlState *s, int ep, bool out)
{
    uint32_t c;

    if (ep) {
        return dpram_ld(s, DPRAM_EP_CTRL(ep, out));
    }
    c = EP_CTRL_ENABLE | (EP_TYPE_CONTROL << EP_CTRL_TYPE_SHIFT) |
        DPRAM_EP0_BUF;
    if (s->sie_ctrl & SIE_CTRL_EP0_DOUBLE_BUF) {
        c |= EP_CTRL_DOUBLE_BUFFERED;
    }
    if (s->sie_ctrl & SIE_CTRL_EP0_INT_1BUF) {
        c |= EP_CTRL_INT_PER_BUFF;
    }
    if (s->sie_ctrl & SIE_CTRL_EP0_INT_2BUF) {
        c |= EP_CTRL_INT_PER_DOUBLE_BUFF;
    }
    if (s->sie_ctrl & SIE_CTRL_EP0_INT_NAK) {
        c |= EP_CTRL_INT_ON_NAK;
    }
    if (s->sie_ctrl & SIE_CTRL_EP0_INT_STALL) {
        c |= EP_CTRL_INT_ON_STALL;
    }
    return c;
}

static void dev_set_suspended(RP2350USBCtrlState *s, bool suspended)
{
    if (s->dev_suspended != suspended) {
        s->dev_suspended = suspended;
        s->intr_events |= INTR_DEV_SUSPEND;
    }
}

/*
 * Bus activity: wakes a suspended device and restarts the 3 ms of idle
 * after which it suspends. Activity other than a reset is the host's
 * resume.
 */
/* [spec:nuos:req:emu.usb] */
static void dev_bus_activity(RP2350USBCtrlState *s, bool reset)
{
    if (s->dev_suspended) {
        dev_set_suspended(s, false);
        if (!reset) {
            s->sie_status |= SIE_STATUS_RESUME;
            s->intr_events |= INTR_DEV_RESUME_FROM_HOST;
        }
    }
    timer_mod(s->dev_idle_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SUSPEND_NS);
}

/* [spec:nuos:req:emu.usb] */
static void dev_idle(void *opaque)
{
    RP2350USBCtrlState *s = opaque;

    if (rp2350_usbctrl_dev_present(s)) {
        dev_set_suspended(s, true);
        usb_update_irq(s);
    }
}

/*
 * The device's connection to the bus follows its pull-up and whether a
 * host is at the other end; it starts awake and suspends after 3 ms
 * without bus activity.
 */
/* [spec:nuos:req:emu.usb] */
void rp2350_usbctrl_update_dev_port(RP2350USBCtrlState *s)
{
    bool present = rp2350_usbctrl_dev_present(s);
    bool connected = present && rp2350_usbctrl_host_attached(s);

    if (connected != s->dev_connected) {
        s->dev_connected = connected;
        s->intr_events |= INTR_DEV_CONN_DIS;
    }
    if (present != s->dev_present) {
        s->dev_present = present;
        if (present) {
            timer_mod(s->dev_idle_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SUSPEND_NS);
        } else {
            timer_del(s->dev_idle_timer);
            s->dev_suspended = false;
            s->dev_bus_reset = false;
        }
        rp2350_usb_bridge_port_changed(s);
    }
    usb_update_irq(s);
}

/*
 * Any token on the bus is activity; the device answers only those
 * addressed to it.
 */
static bool dev_addressed(RP2350USBCtrlState *s, uint8_t addr)
{
    if (!rp2350_usbctrl_dev_present(s)) {
        return false;
    }
    dev_bus_activity(s, false);
    return addr == (s->addr_endp[0] & ADDR_ENDP_ADDRESS);
}

/* [spec:nuos:req:emu.usb] */
/*
 * The host drives SE0 for a bus reset: the device sees BUS_RESET as it
 * starts, and the bus is not idle, so not suspending, until it ends.
 */
void rp2350_usbctrl_dev_bus_reset(RP2350USBCtrlState *s, bool asserted)
{
    if (!rp2350_usbctrl_dev_present(s)) {
        return;
    }
    dev_bus_activity(s, true);
    s->dev_bus_reset = asserted;
    if (asserted) {
        timer_del(s->dev_idle_timer);
        s->sie_status |= SIE_STATUS_BUS_RESET;
        s->dev_buf_sel = 0;
    }
    usb_update_irq(s);
}

/* [spec:nuos:req:emu.usb] */
void rp2350_usbctrl_dev_sof(RP2350USBCtrlState *s, uint16_t frame)
{
    if (!rp2350_usbctrl_dev_present(s)) {
        return;
    }
    dev_bus_activity(s, false);
    s->sof_rd = frame & FRAME_MASK;
    s->sof_timestamp_last = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                     PHY_CLK_HZ, NANOSECONDS_PER_SECOND) &
                            SOF_TIMESTAMP_MASK;
    s->intr_events |= INTR_DEV_SOF;
    usb_update_irq(s);
}

/*
 * SETUP: always accepted into the first 8 bytes of DPRAM. Receiving one
 * disarms EP0's STALLs and restarts both EP0 directions at buffer 0.
 */
/* [spec:nuos:req:emu.usb] */
RP2350USBHandshake rp2350_usbctrl_dev_setup(RP2350USBCtrlState *s,
                                            uint8_t addr, int pid,
                                            const uint8_t *data)
{
    if (!dev_addressed(s, addr)) {
        usb_update_irq(s);
        return RP2350_USB_NORESP;
    }
    if (pid) {
        s->sie_status |= SIE_STATUS_DATA_SEQ_ERROR;
    }
    memcpy(&s->dpram[DPRAM_SETUP], data, 8);
    s->ep_stall_arm = 0;
    s->dev_buf_sel &= ~3u;
    s->sie_status |= SIE_STATUS_SETUP_REC;
    usb_update_irq(s);
    return RP2350_USB_ACK;
}

/*
 * Common token checks for IN and OUT: the endpoint must be enabled, and
 * an aborted endpoint NAKs. Returns the buffer control word's offset, or
 * -1 for no response.
 */
static int dev_token(RP2350USBCtrlState *s, uint8_t addr, uint8_t ep,
                     bool out, uint32_t *ep_ctrl)
{
    if (!dev_addressed(s, addr) || ep >= RP2350_USBCTRL_ENDPOINTS) {
        return -1;
    }
    *ep_ctrl = dev_ep_ctrl(s, ep, out);
    if (!(*ep_ctrl & EP_CTRL_ENABLE)) {
        return -1;
    }
    return DPRAM_BUF_CTRL(ep, out);
}

static RP2350USBHandshake dev_nak(RP2350USBCtrlState *s, int bit,
                                  uint32_t ep_ctrl)
{
    if (extract32(ep_ctrl, EP_CTRL_TYPE_SHIFT, 2) == EP_TYPE_ISO) {
        return RP2350_USB_NORESP;
    }
    if (ep_ctrl & EP_CTRL_INT_ON_NAK) {
        s->ep_status_stall_nak |= 1u << bit;
    }
    usb_update_irq(s);
    return RP2350_USB_NAK;
}

/*
 * STALL: the buffer control STALL bit, which for EP0 must also be armed
 * in EP_STALL_ARM.
 */
static bool dev_stalled(RP2350USBCtrlState *s, uint8_t ep, bool out,
                        uint32_t bc)
{
    if (!(bc & BUF_STALL)) {
        return false;
    }
    return ep || (s->ep_stall_arm & (out ? 2u : 1u));
}

static RP2350USBHandshake dev_stall(RP2350USBCtrlState *s, int bit,
                                    uint32_t ep_ctrl)
{
    if (ep_ctrl & EP_CTRL_INT_ON_STALL) {
        s->ep_status_stall_nak |= 1u << bit;
    }
    usb_update_irq(s);
    return RP2350_USB_STALL;
}

/*
 * A buffer is done: write its status back, raise TRANS_COMPLETE for the
 * last buffer of a transfer and BUFF_STATUS as configured, and move a
 * double-buffered endpoint to its other buffer.
 */
static void dev_buf_done(RP2350USBCtrlState *s, int bit, uint32_t ep_ctrl,
                         uint32_t off, int sel, uint32_t half, bool toggle)
{
    bool last = half & BUF_LAST;

    buf_ctrl_write_half(s, off, sel, half);
    if (last) {
        s->sie_status |= SIE_STATUS_TRANS_COMPLETE;
    }
    if (buff_done_raises(ep_ctrl, sel, last || !toggle)) {
        buff_status_raise(s, bit, sel);
    }
    if (toggle && (ep_ctrl & EP_CTRL_DOUBLE_BUFFERED)) {
        s->dev_buf_sel ^= 1u << bit;
    }
}

/*
 * The buffer the endpoint is on, after a RESET in its buffer control word
 * sends it back to buffer 0.
 */
static int dev_sel(RP2350USBCtrlState *s, int bit, uint32_t ep_ctrl,
                   uint32_t bc)
{
    if (bc & BUF_RESET_SEL) {
        s->dev_buf_sel &= ~(1u << bit);
    }
    if (!(ep_ctrl & EP_CTRL_DOUBLE_BUFFERED)) {
        return 0;
    }
    return (s->dev_buf_sel >> bit) & 1;
}

/* IN: send a full, available buffer; otherwise NAK. */
/* [spec:nuos:req:emu.usb] */
RP2350USBHandshake rp2350_usbctrl_dev_in(RP2350USBCtrlState *s, uint8_t addr,
                                         uint8_t ep, uint8_t *buf,
                                         uint16_t max, uint16_t *len,
                                         int *pid)
{
    int bit = ep * 2;
    uint32_t ep_ctrl, bc, half;
    int off, sel;

    *len = 0;
    *pid = 0;
    off = dev_token(s, addr, ep, false, &ep_ctrl);
    if (off < 0) {
        usb_update_irq(s);
        return RP2350_USB_NORESP;
    }
    if (s->ep_abort & (1u << bit)) {
        return dev_nak(s, bit, ep_ctrl);
    }
    bc = dpram_ld(s, off);
    if (dev_stalled(s, ep, false, bc)) {
        return dev_stall(s, bit, ep_ctrl);
    }
    sel = dev_sel(s, bit, ep_ctrl, bc);
    half = buf_ctrl_half(bc, sel);
    if (!(half & BUF_AVAILABLE) || !(half & BUF_FULL)) {
        return dev_nak(s, bit, ep_ctrl);
    }
    *len = MIN(half & BUF_LEN, max);
    *pid = !!(half & BUF_PID);
    dpram_copy_out(s, ep ? buf_addr(ep_ctrl, bc, sel) : DPRAM_EP0_BUF +
                   sel * 64, buf, *len);
    /* The host's ACK; an isochronous IN has none. */
    if (extract32(ep_ctrl, EP_CTRL_TYPE_SHIFT, 2) != EP_TYPE_ISO) {
        s->sie_status |= SIE_STATUS_ACK_REC;
    }
    dev_buf_done(s, bit, ep_ctrl, off, sel,
                 half & (BUF_LAST | BUF_PID | BUF_LEN), true);
    usb_update_irq(s);
    return extract32(ep_ctrl, EP_CTRL_TYPE_SHIFT, 2) == EP_TYPE_ISO ?
           RP2350_USB_NORESP : RP2350_USB_ACK;
}

/*
 * OUT: store the data in an available, empty buffer; otherwise NAK. A
 * DATA PID that does not match the buffer control word's is a sequence
 * error. A short packet does not move a double-buffered endpoint on, so
 * nothing more arrives until software rearms it (on EP0 only with
 * SIE_CTRL.EP0_STOP_ON_SHORT_PACKET).
 */
/* [spec:nuos:req:emu.usb] */
RP2350USBHandshake rp2350_usbctrl_dev_out(RP2350USBCtrlState *s,
                                          uint8_t addr, uint8_t ep, int pid,
                                          const uint8_t *data, uint16_t len)
{
    int bit = ep * 2 + 1;
    uint32_t ep_ctrl, bc, half, room;
    bool iso, short_packet;
    int off, sel;

    off = dev_token(s, addr, ep, true, &ep_ctrl);
    if (off < 0) {
        usb_update_irq(s);
        return RP2350_USB_NORESP;
    }
    iso = extract32(ep_ctrl, EP_CTRL_TYPE_SHIFT, 2) == EP_TYPE_ISO;
    if (s->ep_abort & (1u << bit)) {
        return dev_nak(s, bit, ep_ctrl);
    }
    bc = dpram_ld(s, off);
    if (dev_stalled(s, ep, true, bc)) {
        return dev_stall(s, bit, ep_ctrl);
    }
    sel = dev_sel(s, bit, ep_ctrl, bc);
    half = buf_ctrl_half(bc, sel);
    if (!iso && !!(half & BUF_PID) != !!pid) {
        s->sie_status |= SIE_STATUS_DATA_SEQ_ERROR;
    }
    if (!(half & BUF_AVAILABLE) || (half & BUF_FULL)) {
        return dev_nak(s, bit, ep_ctrl);
    }
    room = half & BUF_LEN;
    len = MIN(len, room);
    dpram_copy_in(s, ep ? buf_addr(ep_ctrl, bc, sel) : DPRAM_EP0_BUF +
                  sel * 64, data, len);
    short_packet = len < room;
    if (short_packet) {
        s->sie_status |= SIE_STATUS_RX_SHORT_PACKET;
    }
    dev_buf_done(s, bit, ep_ctrl, off, sel,
                 BUF_FULL | (half & (BUF_LAST | BUF_PID)) | len,
                 !short_packet ||
                 (ep == 0 && !(s->sie_ctrl & SIE_CTRL_EP0_STOP_ON_SHORT)));
    usb_update_irq(s);
    return iso ? RP2350_USB_NORESP : RP2350_USB_ACK;
}

/* Host controller */

static USBDevice *host_root(RP2350USBCtrlState *s)
{
    USBDevice *dev = s->port.dev;

    return dev && dev->attached ? dev : NULL;
}

static int64_t host_bit_ns(RP2350USBCtrlState *s)
{
    return s->host_speed == 1 ? LS_BIT_NS : FS_BIT_NS;
}

static int64_t host_xact_ns(RP2350USBCtrlState *s, uint32_t len)
{
    return (len + XACT_OVERHEAD_BYTES) * 8 * host_bit_ns(s);
}

static void host_kick(RP2350USBCtrlState *s, int64_t delay)
{
    int64_t when = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay;

    if (!timer_pending(s->host_timer) ||
        timer_expire_time_ns(s->host_timer) > when) {
        timer_mod(s->host_timer, when);
    }
}

/*
 * A connected device pulls DP (full speed) or DM (low speed) up against
 * the host's pull-downs; SPEED shows which, and HOST_CONN_DIS flags each
 * change.
 */
/* [spec:nuos:req:emu.usb] */
static void host_update_port(RP2350USBCtrlState *s)
{
    USBDevice *dev = host_root(s);
    uint8_t speed = 0;

    if (host_active(s) && (s->sie_ctrl & SIE_CTRL_PULLDOWN_EN) && dev) {
        speed = dev->speed == USB_SPEED_LOW ? 1 : 2;
    }
    if (speed != s->host_speed) {
        s->host_speed = speed;
        s->intr_events |= INTR_HOST_CONN_DIS;
    }
    if (host_active(s) &&
        (s->sie_ctrl & (SIE_CTRL_SOF_EN | SIE_CTRL_KEEP_ALIVE_EN))) {
        if (!timer_pending(s->host_frame_timer)) {
            s->host_frame_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                               FRAME_NS;
            timer_mod(s->host_frame_timer, s->host_frame_ns);
        }
    } else {
        timer_del(s->host_frame_timer);
    }
}

static void host_cancel(RP2350USBCtrlState *s)
{
    if (s->xact_async) {
        usb_cancel_packet(&s->packet);
        s->xact_async = false;
    }
}

static void host_reset_toggles(RP2350USBCtrlState *s)
{
    memset(s->dev_toggle, 0, sizeof(s->dev_toggle));
    s->ctrl_stall = false;
}

/*
 * The device data toggle state that standard requests reset: all of a
 * device's non-control endpoints on SET_CONFIGURATION and SET_INTERFACE,
 * one endpoint on CLEAR_FEATURE(ENDPOINT_HALT).
 */
static void host_setup_toggles(RP2350USBCtrlState *s, uint8_t addr,
                               const uint8_t *setup)
{
    uint8_t type = setup[0], req = setup[1];
    uint16_t value = setup[2] | (setup[3] << 8);
    uint16_t index = setup[4] | (setup[5] << 8);

    /* Both directions of EP0 continue with DATA1 after a SETUP. */
    s->dev_toggle[addr] |= 3u;
    if ((type & 0x60) != USB_TYPE_STANDARD) {
        return;
    }
    if ((type == (USB_DIR_OUT | USB_RECIP_DEVICE) &&
         req == USB_REQ_SET_CONFIGURATION) ||
        (type == (USB_DIR_OUT | USB_RECIP_INTERFACE) &&
         req == USB_REQ_SET_INTERFACE)) {
        s->dev_toggle[addr] &= 3u;
    } else if (type == (USB_DIR_OUT | USB_RECIP_ENDPOINT) &&
               req == USB_REQ_CLEAR_FEATURE && value == 0) {
        int ep = index & 0xf;

        if (ep) {
            s->dev_toggle[addr] &= ~(1u << (ep * 2 +
                                            !(index & USB_DIR_IN)));
        }
    } else if (type == (USB_DIR_OUT | USB_RECIP_DEVICE) &&
               req == USB_REQ_SET_ADDRESS && value < 128 && value != addr) {
        s->dev_toggle[value] = 0;
    }
}

static void host_xact_done(RP2350USBCtrlState *s, int status, int actual);

/*
 * Run one transaction on the bus: a USB packet to the addressed QEMU
 * device. The device's data toggle decides what it does with an OUT whose
 * PID is out of step (acknowledges it and drops the data) and which PID
 * its IN data carries.
 */
/* [spec:nuos:req:emu.usb] */
static void host_issue(RP2350USBCtrlState *s, int slot, int pid, uint8_t addr,
                       uint8_t ep, int sel, uint32_t buf, uint16_t len,
                       int data_pid)
{
    USBDevice *root = host_root(s);
    USBDevice *dev = NULL;
    USBEndpoint *uep;
    int tbit;

    s->xact_slot = slot;
    s->xact_pid = pid;
    s->xact_addr = addr;
    s->xact_ep = ep;
    s->xact_sel = sel;
    s->xact_len = len;
    timer_mod(s->host_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              host_xact_ns(s, pid == USB_TOKEN_IN ? 0 : len));

    /*
     * A device answers at address 0 once powered; QEMU's devices wait for
     * a bus reset first, which the port gives on the first transaction.
     */
    if (root && root->state == USB_STATE_ATTACHED && s->host_speed) {
        usb_device_reset(root);
    }
    if (s->host_speed) {
        dev = usb_find_device(&s->port, addr);
    }
    if (!dev) {
        host_xact_done(s, USB_RET_NODEV, 0);
        return;
    }
    tbit = 1u << (ep * 2 + (pid != USB_TOKEN_IN));
    if (pid == USB_TOKEN_SETUP) {
        s->ctrl_stall = false;
    } else if (ep == 0 && s->ctrl_stall && s->ctrl_stall_addr == addr) {
        host_xact_done(s, USB_RET_STALL, 0);
        return;
    }
    if (pid == USB_TOKEN_OUT &&
        !!(s->dev_toggle[addr] & tbit) != data_pid) {
        /* The device acknowledges a retransmission and drops it. */
        host_xact_done(s, USB_RET_SUCCESS, len);
        return;
    }
    if (pid != USB_TOKEN_IN) {
        dpram_copy_out(s, buf, s->host_buf, len);
    }
    uep = usb_ep_get(dev, pid == USB_TOKEN_SETUP ? USB_TOKEN_OUT : pid, ep);
    usb_packet_setup(&s->packet, pid, uep, 0, 0, false, false);
    usb_packet_addbuf(&s->packet, s->host_buf, len);
    usb_handle_packet(dev, &s->packet);
    if (s->packet.status == USB_RET_ASYNC) {
        s->xact_async = true;
        timer_del(s->host_timer);
        return;
    }
    host_xact_done(s, s->packet.status, s->packet.actual_length);
    if (pid == USB_TOKEN_IN) {
        timer_mod(s->host_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  host_xact_ns(s, s->packet.actual_length));
    }
}

static void host_epx_end(RP2350USBCtrlState *s)
{
    s->epx_active = false;
    s->epx_wait_buf = false;
}

static void host_nak_retry(RP2350USBCtrlState *s)
{
    uint32_t delay = s->host_speed == 1 ?
                     s->nak_poll & NAK_POLL_DELAY_LS :
                     extract32(s->nak_poll, NAK_POLL_DELAY_FS_SHIFT, 10);

    s->sie_status |= SIE_STATUS_NAK_REC;
    if (s->nak_poll & NAK_POLL_STOP_EPX_ON_NAK) {
        s->nak_poll |= NAK_POLL_EPX_STOPPED_ON_NAK;
        host_epx_end(s);
        return;
    }
    s->nak_retry = MIN(s->nak_retry + 1, 0x3ffu);
    s->epx_retry_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      (int64_t)delay * SCALE_US;
}

/*
 * The result of an EPX or interrupt endpoint transaction, as the host
 * controller's STATUS phase records it.
 */
/* [spec:nuos:req:emu.usb] */
static void host_xact_done(RP2350USBCtrlState *s, int status, int actual)
{
    int slot = s->xact_slot, sel = s->xact_sel;
    uint8_t addr = s->xact_addr, ep = s->xact_ep;
    bool in = s->xact_pid == USB_TOKEN_IN;
    uint32_t ep_ctrl, bc_off, bc, half;
    int tbit = 1u << (ep * 2 + !in);
    int bit;
    bool end;

    if (s->xact_pid == USB_TOKEN_SETUP) {
        if (status == USB_RET_SUCCESS || status == USB_RET_STALL) {
            /*
             * A device acknowledges every SETUP; QEMU's devices refuse
             * a request at once, which a device signals by stalling the
             * data or status stage that follows.
             */
            host_setup_toggles(s, addr, s->host_buf);
            if (status == USB_RET_STALL) {
                s->ctrl_stall = true;
                s->ctrl_stall_addr = addr;
            }
            s->sie_status |= SIE_STATUS_ACK_REC;
            if (s->sie_ctrl & (SIE_CTRL_SEND_DATA | SIE_CTRL_RECEIVE_DATA)) {
                s->epx_phase = EPX_PHASE_DATA;
                s->epx_sel = 0;
            } else {
                s->sie_status |= SIE_STATUS_TRANS_COMPLETE;
                host_epx_end(s);
            }
        } else if (status == USB_RET_NAK) {
            host_nak_retry(s);
        } else {
            s->sie_status |= SIE_STATUS_RX_TIMEOUT;
            host_epx_end(s);
        }
        usb_update_irq(s);
        return;
    }

    if (slot == XACT_EPX) {
        ep_ctrl = dpram_ld(s, DPRAM_EPX_CTRL);
        bc_off = DPRAM_EPX_BUF_CTRL;
        bit = 0;
    } else {
        ep_ctrl = dpram_ld(s, DPRAM_EP_CTRL(slot, 0)) &
                  ~(EP_CTRL_DOUBLE_BUFFERED | EP_CTRL_INT_PER_DOUBLE_BUFF);
        bc_off = DPRAM_BUF_CTRL(slot, 0);
        bit = slot * 2 + !in;
    }
    bc = dpram_ld(s, bc_off);
    half = buf_ctrl_half(bc, sel);

    switch (status) {
    case USB_RET_SUCCESS:
        if (in) {
            int pid = !!(s->dev_toggle[addr] & tbit);

            s->dev_toggle[addr] ^= tbit;
            if (pid != !!(half & BUF_PID)) {
                s->sie_status |= SIE_STATUS_DATA_SEQ_ERROR;
                if (slot == XACT_EPX) {
                    host_epx_end(s);
                }
                break;
            }
            dpram_copy_in(s, buf_addr(ep_ctrl, bc, sel), s->host_buf,
                          actual);
            end = actual < (half & BUF_LEN) || (half & BUF_LAST);
            if (actual < (half & BUF_LEN)) {
                s->sie_status |= SIE_STATUS_RX_SHORT_PACKET;
            }
            buf_ctrl_write_half(s, bc_off, sel, BUF_FULL |
                                (half & (BUF_LAST | BUF_PID)) | actual);
            if (slot == XACT_EPX && ((half & BUF_LAST) || actual == 0)) {
                s->sie_status |= SIE_STATUS_TRANS_COMPLETE;
            }
        } else {
            if (actual == s->xact_len &&
                !!(s->dev_toggle[addr] & tbit) == !!(half & BUF_PID)) {
                s->dev_toggle[addr] ^= tbit;
            }
            s->sie_status |= SIE_STATUS_ACK_REC;
            end = half & BUF_LAST;
            buf_ctrl_write_half(s, bc_off, sel,
                                half & (BUF_LAST | BUF_PID | BUF_LEN));
            if (slot == XACT_EPX && end) {
                s->sie_status |= SIE_STATUS_TRANS_COMPLETE;
            }
        }
        if (buff_done_raises(ep_ctrl, sel, end)) {
            buff_status_raise(s, bit, sel);
        }
        if (slot == XACT_EPX) {
            s->nak_retry = 0;
            if (end) {
                host_epx_end(s);
            } else if (ep_ctrl & EP_CTRL_DOUBLE_BUFFERED) {
                s->epx_sel ^= 1;
            }
        }
        break;
    case USB_RET_NAK:
        /* A polled interrupt endpoint's NAK leaves it for the next poll. */
        if (slot == XACT_EPX) {
            host_nak_retry(s);
        }
        break;
    case USB_RET_STALL:
        s->sie_status |= SIE_STATUS_STALL_REC;
        dpram_st(s, bc_off, (bc | BUF_STALL) &
                 ~(sel ? BUF_AVAILABLE << 16 : BUF_AVAILABLE));
        if (slot == XACT_EPX) {
            host_epx_end(s);
        }
        break;
    case USB_RET_BABBLE:
        s->sie_status |= SIE_STATUS_RX_OVERFLOW;
        if (slot == XACT_EPX) {
            host_epx_end(s);
        }
        break;
    default:
        s->sie_status |= SIE_STATUS_RX_TIMEOUT;
        if (slot == XACT_EPX) {
            host_epx_end(s);
        }
        break;
    }
    usb_update_irq(s);
}

/* One EPX transaction, if its buffer is ready. */
/* [spec:nuos:req:emu.usb] */
static void host_epx_step(RP2350USBCtrlState *s)
{
    uint8_t addr = s->addr_endp[0] & ADDR_ENDP_ADDRESS;
    uint8_t ep = extract32(s->addr_endp[0], ADDR_ENDP_ENDPOINT_SHIFT, 4);
    bool in = s->sie_ctrl & SIE_CTRL_RECEIVE_DATA;
    uint32_t ep_ctrl, bc, half;
    int sel = s->epx_sel;

    if (s->epx_phase == EPX_PHASE_SETUP) {
        host_issue(s, XACT_EPX, USB_TOKEN_SETUP, addr, 0, 0, DPRAM_SETUP, 8,
                   0);
        return;
    }
    ep_ctrl = dpram_ld(s, DPRAM_EPX_CTRL);
    bc = dpram_ld(s, DPRAM_EPX_BUF_CTRL);
    half = buf_ctrl_half(bc, sel);
    if (!(half & BUF_AVAILABLE) || !!(half & BUF_FULL) == in) {
        /* Waits in the CONTROL phase for software to hand it over. */
        s->epx_wait_buf = true;
        return;
    }
    host_issue(s, XACT_EPX, in ? USB_TOKEN_IN : USB_TOKEN_OUT, addr, ep, sel,
               buf_addr(ep_ctrl, bc, sel), half & BUF_LEN,
               !!(half & BUF_PID));
}

/*
 * Poll interrupt endpoint slot `i` if software has a buffer ready on it.
 * Returns whether a transaction went on the bus.
 */
/* [spec:nuos:req:emu.usb] */
static bool host_int_poll(RP2350USBCtrlState *s, int i)
{
    uint32_t ep_ctrl = dpram_ld(s, DPRAM_EP_CTRL(i, 0));
    uint32_t bc = dpram_ld(s, DPRAM_BUF_CTRL(i, 0));
    uint32_t ae = s->addr_endp[i];
    bool out = ae & ADDR_ENDP_INTEP_DIR;

    if (!(bc & BUF_AVAILABLE) || !!(bc & BUF_FULL) != out) {
        return false;
    }
    host_issue(s, i, out ? USB_TOKEN_OUT : USB_TOKEN_IN, ae & ADDR_ENDP_ADDRESS,
               extract32(ae, ADDR_ENDP_ENDPOINT_SHIFT, 4), 0,
               buf_addr(ep_ctrl, bc, 0), bc & BUF_LEN, !!(bc & BUF_PID));
    return true;
}

/*
 * The host SIE: one transaction at a time. Interrupt endpoints due after
 * an SOF go first, then the software-started EPX transfer.
 */
/* [spec:nuos:req:emu.usb] */
static void host_run(void *opaque)
{
    RP2350USBCtrlState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->xact_async || !host_active(s)) {
        return;
    }
    while (s->int_due) {
        int i = ctz32(s->int_due);

        s->int_due &= ~(1u << i);
        if ((s->int_ep_ctrl & (1u << i)) && host_int_poll(s, i)) {
            return;
        }
    }
    if (!s->epx_active || s->epx_wait_sof || s->epx_wait_buf) {
        return;
    }
    if (now < s->epx_retry_ns) {
        host_kick(s, s->epx_retry_ns - now);
        return;
    }
    host_epx_step(s);
}

/*
 * A frame: the SOF (or low-speed keep-alive) carries SOF_WR's frame
 * number, which then counts up; interrupt endpoints whose interval has
 * passed are polled.
 */
/* [spec:nuos:req:emu.usb] */
static void host_frame(void *opaque)
{
    RP2350USBCtrlState *s = opaque;
    int i;

    if (!host_active(s)) {
        return;
    }
    s->host_frame_ns += FRAME_NS;
    timer_mod(s->host_frame_timer, s->host_frame_ns);
    s->sof_rd = s->sof_wr;
    s->sof_wr = (s->sof_wr + 1) & FRAME_MASK;
    s->intr_events |= INTR_HOST_SOF;
    for (i = 1; i < RP2350_USBCTRL_ENDPOINTS; i++) {
        if (!(s->int_ep_ctrl & (1u << i))) {
            continue;
        }
        if (s->int_countdown[i]) {
            s->int_countdown[i]--;
            continue;
        }
        s->int_countdown[i] = extract32(dpram_ld(s, DPRAM_EP_CTRL(i, 0)),
                                        EP_CTRL_HOST_INTERVAL_SHIFT, 10);
        s->int_due |= 1u << i;
    }
    s->epx_wait_sof = false;
    usb_update_irq(s);
    host_kick(s, host_xact_ns(s, 0));
}

/* [spec:nuos:req:emu.usb] */
static void host_start_trans(RP2350USBCtrlState *s)
{
    if (s->xact_async && s->xact_slot == XACT_EPX) {
        host_cancel(s);
    }
    s->epx_active = s->sie_ctrl & (SIE_CTRL_SEND_SETUP | SIE_CTRL_SEND_DATA |
                                   SIE_CTRL_RECEIVE_DATA);
    s->epx_phase = s->sie_ctrl & SIE_CTRL_SEND_SETUP ? EPX_PHASE_SETUP
                                                      : EPX_PHASE_DATA;
    s->epx_sel = 0;
    s->epx_wait_buf = false;
    s->epx_wait_sof = s->sie_ctrl & SIE_CTRL_SOF_SYNC;
    s->epx_retry_ns = 0;
    s->nak_retry = 0;
    if (s->sie_ctrl & SIE_CTRL_PREAMBLE_EN) {
        qemu_log_mask(LOG_UNIMP, "rp2350-usbctrl: PRE tokens are not "
                      "modelled; low-speed devices behind a hub get no "
                      "preamble\n");
    }
    host_kick(s, host_xact_ns(s, 0));
}

/* [spec:nuos:req:emu.usb] */
static void host_bus_reset(RP2350USBCtrlState *s)
{
    USBDevice *dev = host_root(s);

    host_cancel(s);
    host_epx_end(s);
    host_reset_toggles(s);
    if (host_active(s) && dev) {
        usb_device_reset(dev);
    }
}

static void host_attach(USBPort *port)
{
    RP2350USBCtrlState *s = port->opaque;

    host_update_port(s);
    usb_update_irq(s);
}

static void host_detach(USBPort *port)
{
    RP2350USBCtrlState *s = port->opaque;

    host_cancel(s);
    host_reset_toggles(s);
    host_update_port(s);
    usb_update_irq(s);
}

static void host_child_detach(USBPort *port, USBDevice *child)
{
    RP2350USBCtrlState *s = port->opaque;

    if (s->xact_async && s->packet.ep && s->packet.ep->dev == child) {
        host_cancel(s);
        host_xact_done(s, USB_RET_NODEV, 0);
    }
}

/* A device's remote wakeup resumes a suspended bus. */
/* [spec:nuos:req:emu.usb] */
static void host_wakeup(USBPort *port)
{
    RP2350USBCtrlState *s = port->opaque;

    if (host_active(s) &&
        !(s->sie_ctrl & (SIE_CTRL_SOF_EN | SIE_CTRL_KEEP_ALIVE_EN))) {
        s->sie_status |= SIE_STATUS_RESUME;
        s->intr_events |= INTR_HOST_RESUME;
        usb_update_irq(s);
    }
}

static void host_complete(USBPort *port, USBPacket *p)
{
    RP2350USBCtrlState *s = port->opaque;

    if (p != &s->packet || !s->xact_async) {
        return;
    }
    s->xact_async = false;
    if (p->status == USB_RET_REMOVE_FROM_QUEUE) {
        return;
    }
    host_xact_done(s, p->status, p->actual_length);
    host_kick(s, host_xact_ns(s, p->actual_length));
}

static USBPortOps rp2350_usbctrl_port_ops = {
    .attach = host_attach,
    .detach = host_detach,
    .child_detach = host_child_detach,
    .wakeup = host_wakeup,
    .complete = host_complete,
};

static USBBusOps rp2350_usbctrl_bus_ops = {
};

/* Registers */

static void usb_update_ports(RP2350USBCtrlState *s)
{
    if (!host_active(s)) {
        host_cancel(s);
        host_epx_end(s);
    }
    host_update_port(s);
    rp2350_usbctrl_update_dev_port(s);
    usb_update_vbus_en(s);
    usb_update_irq(s);
}

/* [spec:nuos:req:emu.usb] */
static uint32_t usb_reg_read(RP2350USBCtrlState *s, hwaddr reg)
{
    uint32_t v;

    switch (reg) {
    case A_ADDR_ENDP ... A_ADDR_ENDP15:
        return s->addr_endp[reg / 4];
    case A_MAIN_CTRL:
        return s->main_ctrl;
    case A_SOF_WR:
        return 0;
    case A_SOF_RD:
        /* Reading the frame number acknowledges the SOF interrupts. */
        v = s->sof_rd;
        s->intr_events &= ~(INTR_DEV_SOF | INTR_HOST_SOF);
        usb_update_irq(s);
        return v;
    case A_SIE_CTRL:
        return s->sie_ctrl;
    case A_SIE_STATUS:
        return sie_status(s);
    case A_INT_EP_CTRL:
        return s->int_ep_ctrl;
    case A_BUFF_STATUS:
        return s->buff_status;
    case A_BUFF_CPU_SHOULD_HANDLE:
        return s->buff_cpu_should_handle;
    case A_EP_ABORT:
        return s->ep_abort;
    case A_EP_ABORT_DONE:
        return s->ep_abort_done;
    case A_EP_STALL_ARM:
        return s->ep_stall_arm;
    case A_NAK_POLL:
        return s->nak_poll |
               (extract32(s->nak_retry, 0, 6) << NAK_POLL_RETRY_LO_SHIFT) |
               (extract32(s->nak_retry, 6, 4) << NAK_POLL_RETRY_HI_SHIFT);
    case A_EP_STATUS_STALL_NAK:
        return s->ep_status_stall_nak;
    case A_USB_MUXING:
        return s->usb_muxing;
    case A_USB_PWR:
        return s->usb_pwr;
    case A_USBPHY_DIRECT:
        v = s->usbphy_direct;
        switch (line_state(s)) {
        case 1:
            v |= USBPHY_DIRECT_RX_DP | USBPHY_DIRECT_RX_DD;
            break;
        case 2:
            v |= USBPHY_DIRECT_RX_DM;
            break;
        default:
            break;
        }
        return v;
    case A_USBPHY_DIRECT_OVERRIDE:
        return s->usbphy_direct_override;
    case A_USBPHY_TRIM:
        return s->usbphy_trim;
    case A_LINESTATE_TUNING:
        return s->linestate_tuning;
    case A_INTR:
        return usb_intr(s);
    case A_INTE:
        return s->inte;
    case A_INTF:
        return s->intf;
    case A_INTS:
        return usb_ints(s);
    case A_SOF_TIMESTAMP_RAW:
        return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), PHY_CLK_HZ,
                        NANOSECONDS_PER_SECOND) & SOF_TIMESTAMP_MASK;
    case A_SOF_TIMESTAMP_LAST:
        return s->sof_timestamp_last;
    case A_SM_STATE:
        /* Every state machine is idle between transactions. */
        return 0;
    case A_EP_TX_ERROR:
        return s->ep_tx_error;
    case A_EP_RX_ERROR:
        return s->ep_rx_error;
    case A_DEV_SM_WATCHDOG:
        return s->dev_sm_watchdog;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-usbctrl: read of bad offset "
                      "0x%" HWADDR_PRIx "\n", reg);
        return 0;
    }
}

/*
 * A register write through the plain window or an atomic alias. The
 * aliases act on the read-write fields' stored value. Write-one-to-clear
 * flags are cleared by the ones in the written data through any alias,
 * as the datasheet's own examples clear SIE_STATUS bits through the
 * clear alias.
 */
/* [spec:nuos:req:emu.usb] */
static void usb_reg_write(RP2350USBCtrlState *s, hwaddr addr, uint32_t value)
{
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t old, v, start;

    switch (reg) {
    case A_ADDR_ENDP:
        s->addr_endp[0] = rp2350_atomic_apply(addr, s->addr_endp[0], value) &
                          ADDR_ENDP_MASK;
        break;
    case A_ADDR_ENDP + 4 ... A_ADDR_ENDP15:
        s->addr_endp[reg / 4] = rp2350_atomic_apply(addr,
                                                    s->addr_endp[reg / 4],
                                                    value) & ADDR_ENDPN_MASK;
        break;
    case A_MAIN_CTRL:
        s->main_ctrl = rp2350_atomic_apply(addr, s->main_ctrl, value) &
                       MAIN_CTRL_MASK;
        if (s->main_ctrl & MAIN_CTRL_SIM_TIMING) {
            qemu_log_mask(LOG_UNIMP, "rp2350-usbctrl: MAIN_CTRL.SIM_TIMING "
                          "has no effect\n");
        }
        usb_update_ports(s);
        break;
    case A_SOF_WR:
        s->sof_wr = rp2350_atomic_apply(addr, 0, value) & FRAME_MASK;
        break;
    case A_SOF_RD:
        break;
    case A_SIE_CTRL:
        old = s->sie_ctrl;
        v = rp2350_atomic_apply(addr, old, value) & SIE_CTRL_MASK;
        s->sie_ctrl = v & ~SIE_CTRL_SC;
        if (v & SIE_CTRL_DIRECT_EN) {
            qemu_log_mask(LOG_UNIMP, "rp2350-usbctrl: direct bus drive is "
                          "not modelled\n");
        }
        usb_update_ports(s);
        if (host_mode(s)) {
            if (v & SIE_CTRL_RESET_BUS) {
                host_bus_reset(s);
            }
            if (v & SIE_CTRL_STOP_TRANS) {
                host_cancel(s);
                host_epx_end(s);
            }
            if (v & SIE_CTRL_START_TRANS) {
                host_start_trans(s);
            }
        } else if ((v & SIE_CTRL_RESUME) && s->dev_suspended &&
                   s->dev_connected) {
            /* The host answers the device's remote wakeup with a resume. */
            dev_set_suspended(s, false);
        }
        break;
    case A_SIE_STATUS:
        start = value;
        s->sie_status &= ~(start & SIE_STATUS_WC);
        /*
         * Writing the read-only CONNECTED, SUSPENDED, SPEED and RESUME
         * fields acknowledges their interrupts.
         */
        if (start & SIE_STATUS_CONNECTED) {
            s->intr_events &= ~INTR_DEV_CONN_DIS;
        }
        if (start & SIE_STATUS_SUSPENDED) {
            s->intr_events &= ~INTR_DEV_SUSPEND;
        }
        if (start & SIE_STATUS_SPEED) {
            s->intr_events &= ~INTR_HOST_CONN_DIS;
        }
        if (start & SIE_STATUS_RESUME) {
            s->intr_events &= ~(INTR_DEV_RESUME_FROM_HOST | INTR_HOST_RESUME);
        }
        break;
    case A_INT_EP_CTRL:
        old = s->int_ep_ctrl;
        s->int_ep_ctrl = rp2350_atomic_apply(addr, old, value) &
                         INT_EP_CTRL_MASK;
        /* A newly enabled slot is polled after the next SOF. */
        for (v = s->int_ep_ctrl & ~old; v; v &= v - 1) {
            s->int_countdown[ctz32(v)] = 0;
        }
        s->int_due &= s->int_ep_ctrl;
        break;
    case A_BUFF_STATUS:
        buff_status_clear(s, value);
        break;
    case A_EP_ABORT:
        old = s->ep_abort;
        s->ep_abort = rp2350_atomic_apply(addr, old, value);
        /*
         * Transactions are never left half done, so an endpoint is idle
         * as soon as its abort is set.
         */
        s->ep_abort_done |= s->ep_abort & ~old;
        break;
    case A_EP_ABORT_DONE:
        s->ep_abort_done &= ~value;
        break;
    case A_EP_STALL_ARM:
        s->ep_stall_arm = rp2350_atomic_apply(addr, s->ep_stall_arm, value) &
                          3u;
        break;
    case A_NAK_POLL:
        v = rp2350_atomic_apply(addr, s->nak_poll, value);
        s->nak_poll = (v & NAK_POLL_RW) |
                      (s->nak_poll & NAK_POLL_EPX_STOPPED_ON_NAK &
                       ~value);
        break;
    case A_EP_STATUS_STALL_NAK:
        s->ep_status_stall_nak &= ~value;
        break;
    case A_USB_MUXING:
        s->usb_muxing = rp2350_atomic_apply(addr, s->usb_muxing, value) &
                        USB_MUXING_MASK;
        if (s->usb_muxing & USB_MUXING_UNMODELLED) {
            qemu_log_mask(LOG_UNIMP, "rp2350-usbctrl: USB_MUXING routes "
                          "other than TO_PHY are not modelled\n");
        }
        usb_update_ports(s);
        break;
    case A_USB_PWR:
        s->usb_pwr = rp2350_atomic_apply(addr, s->usb_pwr, value) &
                     USB_PWR_MASK;
        usb_update_ports(s);
        break;
    case A_USBPHY_DIRECT:
        s->usbphy_direct = rp2350_atomic_apply(addr, s->usbphy_direct,
                                               value) & USBPHY_DIRECT_RW;
        break;
    case A_USBPHY_DIRECT_OVERRIDE:
        s->usbphy_direct_override =
            rp2350_atomic_apply(addr, s->usbphy_direct_override, value) &
            USBPHY_DIRECT_OVERRIDE_MASK;
        if (s->usbphy_direct_override) {
            qemu_log_mask(LOG_UNIMP, "rp2350-usbctrl: USB PHY direct "
                          "overrides are not modelled\n");
        }
        break;
    case A_USBPHY_TRIM:
        s->usbphy_trim = rp2350_atomic_apply(addr, s->usbphy_trim, value) &
                         USBPHY_TRIM_MASK;
        break;
    case A_LINESTATE_TUNING:
        s->linestate_tuning = rp2350_atomic_apply(addr, s->linestate_tuning,
                                                  value) &
                              LINESTATE_TUNING_MASK;
        break;
    case A_INTE:
        s->inte = rp2350_atomic_apply(addr, s->inte, value) & INTR_MASK;
        break;
    case A_INTF:
        s->intf = rp2350_atomic_apply(addr, s->intf, value) & INTR_MASK;
        break;
    case A_INTR:
    case A_INTS:
    case A_SOF_TIMESTAMP_RAW:
    case A_SOF_TIMESTAMP_LAST:
    case A_SM_STATE:
        break;
    case A_EP_TX_ERROR:
        s->ep_tx_error &= ~value;
        break;
    case A_EP_RX_ERROR:
        s->ep_rx_error &= ~value;
        break;
    case A_DEV_SM_WATCHDOG:
        v = rp2350_atomic_apply(addr, s->dev_sm_watchdog, value);
        s->dev_sm_watchdog = (v & DEV_SM_WATCHDOG_RW) |
                             (s->dev_sm_watchdog & DEV_SM_WATCHDOG_FIRED &
                              ~value);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-usbctrl: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    usb_update_irq(s);
}

/*
 * The registers are 32 bits wide: a narrow write is replicated across the
 * bus and writes the whole register; a narrow read returns its lanes.
 */
static uint64_t rp2350_usbctrl_regs_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    RP2350USBCtrlState *s = opaque;
    uint32_t v = usb_reg_read(s, rp2350_atomic_reg(addr & ~3));

    return extract32(v, (addr & 3) * 8, size * 8);
}

static void rp2350_usbctrl_regs_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned size)
{
    RP2350USBCtrlState *s = opaque;
    uint32_t v = value;

    if (size == 1) {
        v = (v & 0xff) * 0x01010101u;
    } else if (size == 2) {
        v = (v & 0xffff) * 0x00010001u;
    }
    usb_reg_write(s, addr & ~3, v);
}

static const MemoryRegionOps rp2350_usbctrl_regs_ops = {
    .read = rp2350_usbctrl_regs_read,
    .write = rp2350_usbctrl_regs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * The DPRAM takes 8, 16 and 32-bit accesses and has no atomic aliases.
 * Software handing a buffer to the controller may let a waiting
 * transaction proceed.
 */
/* [spec:nuos:req:emu.usb] */
static uint64_t rp2350_usbctrl_dpram_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    RP2350USBCtrlState *s = opaque;

    return ldn_le_p(&s->dpram[addr], size);
}

/* [spec:nuos:req:emu.usb] */
static void rp2350_usbctrl_dpram_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned size)
{
    RP2350USBCtrlState *s = opaque;

    stn_le_p(&s->dpram[addr], size, value);
    if (addr < DPRAM_BUF_CTRL_START || addr >= DPRAM_BUF_CTRL_END) {
        return;
    }
    if (host_mode(s)) {
        if (s->epx_wait_buf && addr < DPRAM_EPX_BUF_CTRL + 4) {
            s->epx_wait_buf = false;
            host_kick(s, 0);
        }
    } else {
        rp2350_usb_bridge_kick(s);
    }
}

static const MemoryRegionOps rp2350_usbctrl_dpram_ops = {
    .read = rp2350_usbctrl_dpram_read,
    .write = rp2350_usbctrl_dpram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void rp2350_usbctrl_set_vbus_detect(void *opaque, int n, int level)
{
    RP2350USBCtrlState *s = opaque;

    s->vbus_detect_in = level > 0;
    usb_update_irq(s);
}

static void rp2350_usbctrl_set_overcurr(void *opaque, int n, int level)
{
    RP2350USBCtrlState *s = opaque;

    s->overcurr_in = level > 0;
    usb_update_irq(s);
}

static void rp2350_usbctrl_reset_enter(Object *obj, ResetType type)
{
    RP2350USBCtrlState *s = RP2350_USBCTRL(obj);

    host_cancel(s);
    timer_del(s->host_timer);
    timer_del(s->host_frame_timer);
    timer_del(s->dev_idle_timer);
}

/* [spec:nuos:req:emu.usb] */
static void rp2350_usbctrl_reset_hold(Object *obj, ResetType type)
{
    RP2350USBCtrlState *s = RP2350_USBCTRL(obj);

    memset(s->dpram, 0, sizeof(s->dpram));
    memset(s->addr_endp, 0, sizeof(s->addr_endp));
    s->main_ctrl = MAIN_CTRL_RESET;
    s->sof_wr = 0;
    s->sof_rd = 0;
    s->sie_ctrl = SIE_CTRL_RESET;
    s->sie_status = 0;
    s->int_ep_ctrl = 0;
    s->buff_status = 0;
    s->buff_cpu_should_handle = 0;
    s->buff_status_pending = 0;
    s->buff_pending_sel = 0;
    s->ep_abort = 0;
    s->ep_abort_done = 0;
    s->ep_stall_arm = 0;
    s->nak_poll = NAK_POLL_RESET;
    s->nak_retry = 0;
    s->ep_status_stall_nak = 0;
    s->usb_muxing = USB_MUXING_RESET;
    s->usb_pwr = 0;
    s->usbphy_direct = 0;
    s->usbphy_direct_override = 0;
    s->usbphy_trim = USBPHY_TRIM_RESET;
    s->linestate_tuning = LINESTATE_TUNING_RESET;
    s->inte = 0;
    s->intf = 0;
    s->sof_timestamp_last = 0;
    s->ep_tx_error = 0;
    s->ep_rx_error = 0;
    s->dev_sm_watchdog = 0;
    s->intr_events = 0;
    s->dev_buf_sel = 0;
    s->dev_connected = false;
    s->dev_suspended = false;
    s->dev_present = false;
    s->dev_bus_reset = false;
    s->host_speed = 0;
    s->epx_active = false;
    s->epx_phase = EPX_PHASE_SETUP;
    s->epx_sel = 0;
    s->epx_wait_sof = false;
    s->epx_wait_buf = false;
    s->epx_retry_ns = 0;
    s->int_due = 0;
    memset(s->int_countdown, 0, sizeof(s->int_countdown));
    host_reset_toggles(s);
    rp2350_usb_bridge_reset(s);
}

static void rp2350_usbctrl_reset_exit(Object *obj, ResetType type)
{
    RP2350USBCtrlState *s = RP2350_USBCTRL(obj);

    usb_update_vbus_en(s);
    usb_update_irq(s);
}

static void rp2350_usbctrl_init(Object *obj)
{
    RP2350USBCtrlState *s = RP2350_USBCTRL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->dpram_mr, obj, &rp2350_usbctrl_dpram_ops, s,
                          "rp2350-usbctrl-dpram", RP2350_USBCTRL_DPRAM_SIZE);
    sysbus_init_mmio(sbd, &s->dpram_mr);
    memory_region_init_io(&s->regs_mr, obj, &rp2350_usbctrl_regs_ops, s,
                          "rp2350-usbctrl-regs", RP2350_USBCTRL_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->regs_mr);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), rp2350_usbctrl_set_vbus_detect,
                            RP2350_USBCTRL_VBUS_DETECT, 1);
    qdev_init_gpio_in_named(DEVICE(obj), rp2350_usbctrl_set_overcurr,
                            RP2350_USBCTRL_OVERCURR_DETECT, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->vbus_en, RP2350_USBCTRL_VBUS_EN,
                             1);
}

static void rp2350_usbctrl_realize(DeviceState *dev, Error **errp)
{
    RP2350USBCtrlState *s = RP2350_USBCTRL(dev);

    if (qemu_chr_fe_backend_connected(&s->cdc_chr) &&
        qemu_chr_fe_backend_connected(&s->token_chr)) {
        error_setg(errp, "cdc-chardev and token-chardev are both hosts on "
                   "the one device port; set at most one");
        return;
    }
    usb_bus_new(&s->bus, sizeof(s->bus), &rp2350_usbctrl_bus_ops, dev);
    usb_register_port(&s->bus, &s->port, s, 0, &rp2350_usbctrl_port_ops,
                      USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL);
    usb_packet_init(&s->packet);
    s->host_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, host_run, s);
    s->host_frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, host_frame, s);
    s->dev_idle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dev_idle, s);
    rp2350_usb_bridge_realize(s);
}

static int rp2350_usbctrl_pre_save(void *opaque)
{
    RP2350USBCtrlState *s = opaque;

    /* A QEMU device's packet in flight cannot be migrated: redo it. */
    if (s->xact_async) {
        host_cancel(s);
        if (s->xact_slot == XACT_EPX) {
            host_kick(s, 0);
        }
    }
    return 0;
}

static const VMStateDescription vmstate_rp2350_usbctrl = {
    .name = TYPE_RP2350_USBCTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = rp2350_usbctrl_pre_save,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(dpram, RP2350USBCtrlState,
                            RP2350_USBCTRL_DPRAM_SIZE),
        VMSTATE_UINT32_ARRAY(addr_endp, RP2350USBCtrlState,
                             RP2350_USBCTRL_ENDPOINTS),
        VMSTATE_UINT32(main_ctrl, RP2350USBCtrlState),
        VMSTATE_UINT32(sof_wr, RP2350USBCtrlState),
        VMSTATE_UINT32(sof_rd, RP2350USBCtrlState),
        VMSTATE_UINT32(sie_ctrl, RP2350USBCtrlState),
        VMSTATE_UINT32(sie_status, RP2350USBCtrlState),
        VMSTATE_UINT32(int_ep_ctrl, RP2350USBCtrlState),
        VMSTATE_UINT32(buff_status, RP2350USBCtrlState),
        VMSTATE_UINT32(buff_cpu_should_handle, RP2350USBCtrlState),
        VMSTATE_UINT32(ep_abort, RP2350USBCtrlState),
        VMSTATE_UINT32(ep_abort_done, RP2350USBCtrlState),
        VMSTATE_UINT32(ep_stall_arm, RP2350USBCtrlState),
        VMSTATE_UINT32(nak_poll, RP2350USBCtrlState),
        VMSTATE_UINT32(nak_retry, RP2350USBCtrlState),
        VMSTATE_UINT32(ep_status_stall_nak, RP2350USBCtrlState),
        VMSTATE_UINT32(usb_muxing, RP2350USBCtrlState),
        VMSTATE_UINT32(usb_pwr, RP2350USBCtrlState),
        VMSTATE_UINT32(usbphy_direct, RP2350USBCtrlState),
        VMSTATE_UINT32(usbphy_direct_override, RP2350USBCtrlState),
        VMSTATE_UINT32(usbphy_trim, RP2350USBCtrlState),
        VMSTATE_UINT32(linestate_tuning, RP2350USBCtrlState),
        VMSTATE_UINT32(inte, RP2350USBCtrlState),
        VMSTATE_UINT32(intf, RP2350USBCtrlState),
        VMSTATE_UINT32(sof_timestamp_last, RP2350USBCtrlState),
        VMSTATE_UINT32(ep_tx_error, RP2350USBCtrlState),
        VMSTATE_UINT32(ep_rx_error, RP2350USBCtrlState),
        VMSTATE_UINT32(dev_sm_watchdog, RP2350USBCtrlState),
        VMSTATE_UINT32(intr_events, RP2350USBCtrlState),
        VMSTATE_UINT32(buff_status_pending, RP2350USBCtrlState),
        VMSTATE_UINT32(buff_pending_sel, RP2350USBCtrlState),
        VMSTATE_BOOL(vbus_detect_in, RP2350USBCtrlState),
        VMSTATE_BOOL(overcurr_in, RP2350USBCtrlState),
        VMSTATE_UINT32(dev_buf_sel, RP2350USBCtrlState),
        VMSTATE_BOOL(dev_connected, RP2350USBCtrlState),
        VMSTATE_BOOL(dev_suspended, RP2350USBCtrlState),
        VMSTATE_BOOL(dev_present, RP2350USBCtrlState),
        VMSTATE_BOOL(dev_bus_reset, RP2350USBCtrlState),
        VMSTATE_TIMER_PTR(dev_idle_timer, RP2350USBCtrlState),
        VMSTATE_TIMER_PTR(host_frame_timer, RP2350USBCtrlState),
        VMSTATE_INT64(host_frame_ns, RP2350USBCtrlState),
        VMSTATE_TIMER_PTR(host_timer, RP2350USBCtrlState),
        VMSTATE_UINT8(host_speed, RP2350USBCtrlState),
        VMSTATE_BOOL(epx_active, RP2350USBCtrlState),
        VMSTATE_UINT8(epx_phase, RP2350USBCtrlState),
        VMSTATE_UINT8(epx_sel, RP2350USBCtrlState),
        VMSTATE_BOOL(epx_wait_sof, RP2350USBCtrlState),
        VMSTATE_BOOL(epx_wait_buf, RP2350USBCtrlState),
        VMSTATE_INT64(epx_retry_ns, RP2350USBCtrlState),
        VMSTATE_UINT32(int_due, RP2350USBCtrlState),
        VMSTATE_UINT16_ARRAY(int_countdown, RP2350USBCtrlState,
                             RP2350_USBCTRL_ENDPOINTS),
        VMSTATE_UINT32_ARRAY(dev_toggle, RP2350USBCtrlState, 128),
        VMSTATE_BOOL(ctrl_stall, RP2350USBCtrlState),
        VMSTATE_UINT8(ctrl_stall_addr, RP2350USBCtrlState),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_rp2350_usb_bridge,
        NULL
    },
};

static const Property rp2350_usbctrl_properties[] = {
    DEFINE_PROP_CHR("cdc-chardev", RP2350USBCtrlState, cdc_chr),
    DEFINE_PROP_CHR("token-chardev", RP2350USBCtrlState, token_chr),
};

static void rp2350_usbctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_usbctrl_realize;
    dc->vmsd = &vmstate_rp2350_usbctrl;
    device_class_set_props(dc, rp2350_usbctrl_properties);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    rc->phases.enter = rp2350_usbctrl_reset_enter;
    rc->phases.hold = rp2350_usbctrl_reset_hold;
    rc->phases.exit = rp2350_usbctrl_reset_exit;
}

/* [spec:nuos:req:emu.usb] */
static const TypeInfo rp2350_usbctrl_info = {
    .name          = TYPE_RP2350_USBCTRL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350USBCtrlState),
    .instance_init = rp2350_usbctrl_init,
    .class_init    = rp2350_usbctrl_class_init,
};

static void rp2350_usbctrl_register_types(void)
{
    type_register_static(&rp2350_usbctrl_info);
}

type_init(rp2350_usbctrl_register_types)
