/*
 * RP2350 USB controller: the hosts at the other end of the device port
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * In device mode the RP2350's USB port needs a host. Two are built in;
 * see rp2350_usbctrl.h for how they are selected and for the token port's
 * protocol.
 *
 * The CDC-ACM bridge behaves as a USB 2.0 host would: once the device's
 * pull-up has been on for the 100 ms attach debounce it resets the bus
 * for 10 ms, allows 10 ms of reset recovery, then sends an SOF every
 * millisecond and enumerates the device at address 1. It reads the
 * configuration descriptor, sets the configuration, gives the first
 * CDC-ACM communication interface a line coding of 115200 8N1 and drives
 * its DTR and RTS from whether the chardev is open, then moves data
 * between the chardev and the bulk endpoints of the first CDC data
 * interface. Every transaction goes through the device controller as
 * the datasheet's state machines describe; a NAKed transaction is
 * retried when software next arms a buffer, and at the latest in the
 * next frame.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "hw/usb/rp2350_usbctrl.h"
#include "migration/vmstate.h"

#define BR_ADDR                 1
#define BR_MPS0_INITIAL         8
#define BR_DEBOUNCE_NS          (100 * SCALE_MS)
#define BR_RESET_NS             (10 * SCALE_MS)
#define BR_RECOVERY_NS          (10 * SCALE_MS)
#define BR_SET_ADDRESS_NS       (2 * SCALE_MS)
#define BR_FRAME_NS             SCALE_MS
#define BR_FS_BIT_NS            (NANOSECONDS_PER_SECOND / 12000000)
#define BR_KICK_NS              SCALE_US
#define BR_ATTEMPTS             3

/* Standard and CDC requests and descriptors used in enumeration. */
#define DESC_DEVICE             1
#define DESC_CONFIG             2
#define DESC_INTERFACE          4
#define DESC_ENDPOINT           5
#define CLASS_CDC_COMM          0x02
#define SUBCLASS_ACM            0x02
#define CLASS_CDC_DATA          0x0a
#define REQ_GET_DESCRIPTOR      6
#define REQ_SET_ADDRESS         5
#define REQ_SET_CONFIGURATION   9
#define CDC_SET_LINE_CODING     0x20
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_DTR                 1
#define CDC_RTS                 2
#define EP_XFER_BULK            2

enum {
    CTL_SETUP,
    CTL_DATA,
    CTL_STATUS,
    CTL_IDLE,
};

enum {
    STEP_GET_DEVICE8,
    STEP_SET_ADDRESS,
    STEP_GET_DEVICE,
    STEP_GET_CONFIG9,
    STEP_GET_CONFIG,
    STEP_SET_CONFIGURATION,
    STEP_SET_LINE_CODING,
    STEP_DONE,
};

/* What one transaction did: moved data, or has to wait for the device. */
typedef enum {
    BR_PROGRESS,
    BR_WAIT,
    BR_DONE,
    BR_FAIL,
} BrResult;

static int64_t br_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static int64_t br_xact_ns(uint32_t len)
{
    return (len + 13) * 8 * BR_FS_BIT_NS;
}

static bool br_ctl_in(RP2350USBCtrlState *s)
{
    return s->br_setup[0] & USB_DIR_IN;
}

static void br_ctl_start(RP2350USBCtrlState *s, uint8_t type, uint8_t req,
                         uint16_t value, uint16_t index, uint16_t len,
                         const uint8_t *data)
{
    s->br_setup[0] = type;
    s->br_setup[1] = req;
    s->br_setup[2] = value;
    s->br_setup[3] = value >> 8;
    s->br_setup[4] = index;
    s->br_setup[5] = index >> 8;
    s->br_setup[6] = len;
    s->br_setup[7] = len >> 8;
    s->br_ctl_len = MIN(len, RP2350_USB_BRIDGE_CFG_MAX);
    s->br_ctl_done = 0;
    s->br_ctl_stage = CTL_SETUP;
    if (data) {
        memcpy(s->br_ctl_data, data, s->br_ctl_len);
    }
}

/*
 * One transaction of a control transfer: SETUP, then data packets of up
 * to the device's EP0 packet size starting with DATA1, then the status
 * stage in the other direction, a zero-length DATA1 packet.
 */
static BrResult br_ctl_step(RP2350USBCtrlState *s, int64_t *ns)
{
    RP2350USBHandshake r;
    uint16_t len, want;
    int pid;

    switch (s->br_ctl_stage) {
    case CTL_SETUP:
        r = rp2350_usbctrl_dev_setup(s, s->br_addr, 0, s->br_setup);
        if (r != RP2350_USB_ACK) {
            return BR_FAIL;
        }
        s->br_ctl_toggle = 1;
        s->br_ctl_stage = s->br_ctl_len ? CTL_DATA : CTL_STATUS;
        *ns = br_xact_ns(8);
        return BR_PROGRESS;
    case CTL_DATA:
        want = MIN(s->br_mps0, s->br_ctl_len - s->br_ctl_done);
        if (br_ctl_in(s)) {
            r = rp2350_usbctrl_dev_in(s, s->br_addr, 0,
                                      s->br_ctl_data + s->br_ctl_done, want,
                                      &len, &pid);
        } else {
            len = want;
            r = rp2350_usbctrl_dev_out(s, s->br_addr, 0, s->br_ctl_toggle,
                                       s->br_ctl_data + s->br_ctl_done, len);
            pid = s->br_ctl_toggle;
        }
        if (r == RP2350_USB_NAK) {
            return BR_WAIT;
        }
        if (r != RP2350_USB_ACK) {
            return BR_FAIL;
        }
        *ns = br_xact_ns(len);
        /* Data with the wrong PID is a repeat of the last; drop it. */
        if (pid != s->br_ctl_toggle) {
            return BR_PROGRESS;
        }
        s->br_ctl_toggle ^= 1;
        s->br_ctl_done += len;
        if (s->br_ctl_done >= s->br_ctl_len ||
            (br_ctl_in(s) && len < s->br_mps0)) {
            s->br_ctl_stage = CTL_STATUS;
        }
        return BR_PROGRESS;
    case CTL_STATUS:
        if (br_ctl_in(s)) {
            r = rp2350_usbctrl_dev_out(s, s->br_addr, 0, 1, NULL, 0);
        } else {
            r = rp2350_usbctrl_dev_in(s, s->br_addr, 0, NULL, 0, &len, &pid);
        }
        if (r == RP2350_USB_NAK) {
            return BR_WAIT;
        }
        if (r != RP2350_USB_ACK) {
            return BR_FAIL;
        }
        s->br_ctl_stage = CTL_IDLE;
        *ns = br_xact_ns(0);
        return BR_DONE;
    default:
        return BR_DONE;
    }
}

/*
 * Find the CDC-ACM interfaces in the configuration descriptor: the first
 * communication interface, and the bulk endpoints of the first data
 * interface.
 */
static void br_parse_config(RP2350USBCtrlState *s)
{
    const uint8_t *d = s->br_ctl_data;
    uint32_t len = s->br_ctl_done, off = 0;
    int cur_class = -1;
    bool data_found = false;

    s->br_config = len >= 9 ? d[5] : 1;
    s->br_comm_if = -1;
    s->br_in_ep = 0;
    s->br_out_ep = 0;
    while (off + 2 <= len && d[off] >= 2 && off + d[off] <= len) {
        const uint8_t *desc = d + off;

        if (desc[1] == DESC_INTERFACE && desc[0] >= 9) {
            cur_class = desc[5];
            if (cur_class == CLASS_CDC_COMM && desc[6] == SUBCLASS_ACM &&
                s->br_comm_if < 0) {
                s->br_comm_if = desc[2];
            }
            if (cur_class == CLASS_CDC_DATA && s->br_in_ep && s->br_out_ep) {
                data_found = true;
            }
        } else if (desc[1] == DESC_ENDPOINT && desc[0] >= 7 &&
                   cur_class == CLASS_CDC_DATA && !data_found &&
                   (desc[3] & 3) == EP_XFER_BULK) {
            uint16_t mps = (desc[4] | (desc[5] << 8)) & 0x7ff;

            if (desc[2] & USB_DIR_IN) {
                if (!s->br_in_ep) {
                    s->br_in_ep = desc[2] & 0xf;
                    s->br_in_mps = mps;
                }
            } else if (!s->br_out_ep) {
                s->br_out_ep = desc[2] & 0xf;
                s->br_out_mps = mps;
            }
        }
        off += d[off];
    }
}

static void br_set_state(RP2350USBCtrlState *s, RP2350USBBridgeState state,
                         int64_t delay)
{
    s->br_state = state;
    s->br_waiting = false;
    timer_mod(s->br_timer, br_now() + delay);
}

/* Start enumeration step `step`. */
static void br_enum_step(RP2350USBCtrlState *s, int step)
{
    static const uint8_t line_coding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0,
                                            8 };

    s->br_step = step;
    switch (step) {
    case STEP_GET_DEVICE8:
        br_ctl_start(s, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_DEVICE << 8, 0,
                     8, NULL);
        break;
    case STEP_SET_ADDRESS:
        br_ctl_start(s, USB_DIR_OUT, REQ_SET_ADDRESS, BR_ADDR, 0, 0, NULL);
        break;
    case STEP_GET_DEVICE:
        br_ctl_start(s, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_DEVICE << 8, 0,
                     18, NULL);
        break;
    case STEP_GET_CONFIG9:
        br_ctl_start(s, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_CONFIG << 8, 0,
                     9, NULL);
        break;
    case STEP_GET_CONFIG:
        br_ctl_start(s, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_CONFIG << 8, 0,
                     MIN(s->br_ctl_data[2] | (s->br_ctl_data[3] << 8),
                         RP2350_USB_BRIDGE_CFG_MAX), NULL);
        break;
    case STEP_SET_CONFIGURATION:
        br_parse_config(s);
        br_ctl_start(s, USB_DIR_OUT, REQ_SET_CONFIGURATION, s->br_config, 0,
                     0, NULL);
        break;
    case STEP_SET_LINE_CODING:
        br_ctl_start(s, USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                     CDC_SET_LINE_CODING, 0, s->br_comm_if,
                     sizeof(line_coding), line_coding);
        break;
    default:
        break;
    }
}

/* The next step after `step` completes, and how long the host waits. */
static int br_enum_next(RP2350USBCtrlState *s, int64_t *delay)
{
    *delay = 0;
    switch (s->br_step) {
    case STEP_GET_DEVICE8:
        if (s->br_ctl_done >= 8 && s->br_ctl_data[7]) {
            s->br_mps0 = s->br_ctl_data[7];
        }
        return STEP_SET_ADDRESS;
    case STEP_SET_ADDRESS:
        s->br_addr = BR_ADDR;
        *delay = BR_SET_ADDRESS_NS;
        return STEP_GET_DEVICE;
    case STEP_GET_DEVICE:
        return STEP_GET_CONFIG9;
    case STEP_GET_CONFIG9:
        return STEP_GET_CONFIG;
    case STEP_GET_CONFIG:
        return STEP_SET_CONFIGURATION;
    case STEP_SET_CONFIGURATION:
        s->br_toggle = 0;
        if (!s->br_in_ep || !s->br_out_ep) {
            qemu_log_mask(LOG_UNIMP, "rp2350-usbctrl: the device has no "
                          "CDC-ACM data interface for the bridge\n");
            return STEP_DONE;
        }
        return s->br_comm_if >= 0 ? STEP_SET_LINE_CODING : STEP_DONE;
    default:
        return STEP_DONE;
    }
}

static void br_fail(RP2350USBCtrlState *s)
{
    if (++s->br_attempt < BR_ATTEMPTS) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-usbctrl: enumeration failed "
                      "at step %d; resetting the device\n", s->br_step);
        timer_del(s->br_frame_timer);
        br_set_state(s, BRIDGE_RESET, 0);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-usbctrl: enumeration failed at "
                  "step %d; giving up\n", s->br_step);
    timer_del(s->br_frame_timer);
    s->br_state = BRIDGE_FAILED;
}

/* One bulk transaction between the chardev and the CDC data endpoints. */
static BrResult br_bulk(RP2350USBCtrlState *s, int64_t *ns)
{
    uint8_t buf[1024];
    RP2350USBHandshake r;
    uint32_t bit;
    uint16_t len;
    int pid, i;

    for (i = 0; i < 2; i++) {
        bool in = s->br_turn ^ i;

        if (in) {
            bit = 1u << (s->br_in_ep * 2);
            r = rp2350_usbctrl_dev_in(s, s->br_addr, s->br_in_ep, buf,
                                      MIN(s->br_in_mps, sizeof(buf)), &len,
                                      &pid);
            if (r != RP2350_USB_ACK) {
                continue;
            }
            *ns = br_xact_ns(len);
            if (pid == !!(s->br_toggle & bit)) {
                s->br_toggle ^= bit;
                qemu_chr_fe_write_all(&s->cdc_chr, buf, len);
            }
        } else {
            if (!s->br_tx_len) {
                continue;
            }
            bit = 1u << (s->br_out_ep * 2 + 1);
            len = MIN(s->br_out_mps, s->br_tx_len);
            r = rp2350_usbctrl_dev_out(s, s->br_addr, s->br_out_ep,
                                       !!(s->br_toggle & bit), s->br_tx, len);
            if (r != RP2350_USB_ACK) {
                continue;
            }
            *ns = br_xact_ns(len);
            s->br_toggle ^= bit;
            s->br_tx_len -= len;
            memmove(s->br_tx, s->br_tx + len, s->br_tx_len);
            qemu_chr_fe_accept_input(&s->cdc_chr);
        }
        s->br_turn = !in;
        return BR_PROGRESS;
    }
    return BR_WAIT;
}

static void br_frame(void *opaque)
{
    RP2350USBCtrlState *s = opaque;

    s->br_frame_ns += BR_FRAME_NS;
    timer_mod(s->br_frame_timer, s->br_frame_ns);
    s->br_frame = (s->br_frame + 1) & 0x7ff;
    rp2350_usbctrl_dev_sof(s, s->br_frame);
    if (s->br_waiting) {
        s->br_waiting = false;
        timer_mod(s->br_timer, br_now() + br_xact_ns(0));
    }
}

/* The bridge's host state machine: bus states, then one transaction. */
/* [spec:nuos:req:emu.usb] */
static void br_run(void *opaque)
{
    RP2350USBCtrlState *s = opaque;
    int64_t ns = 0, delay;
    BrResult r;
    int next;

    if (!rp2350_usbctrl_dev_present(s)) {
        return;
    }
    switch (s->br_state) {
    case BRIDGE_DEBOUNCE:
        s->br_attempt = 0;
        br_set_state(s, BRIDGE_RESET, 0);
        return;
    case BRIDGE_RESET:
        timer_del(s->br_frame_timer);
        rp2350_usbctrl_dev_bus_reset(s, true);
        br_set_state(s, BRIDGE_RECOVERY, BR_RESET_NS);
        return;
    case BRIDGE_RECOVERY:
        /* SOFs start as the reset ends; requests after the recovery. */
        rp2350_usbctrl_dev_bus_reset(s, false);
        s->br_addr = 0;
        s->br_mps0 = BR_MPS0_INITIAL;
        s->br_dtr_sent = false;
        s->br_tx_len = 0;
        s->br_frame_ns = br_now();
        br_frame(s);
        br_enum_step(s, STEP_GET_DEVICE8);
        br_set_state(s, BRIDGE_ENUMERATE, BR_RECOVERY_NS + br_xact_ns(0));
        return;
    case BRIDGE_ENUMERATE:
        r = br_ctl_step(s, &ns);
        if (r == BR_FAIL) {
            br_fail(s);
            return;
        }
        if (r == BR_DONE) {
            next = br_enum_next(s, &delay);
            ns += delay;
            if (next == STEP_DONE) {
                s->br_state = BRIDGE_RUNNING;
                s->br_ctl_stage = CTL_IDLE;
                qemu_chr_fe_accept_input(&s->cdc_chr);
            } else {
                br_enum_step(s, next);
            }
        }
        break;
    case BRIDGE_RUNNING:
        if (s->br_ctl_stage == CTL_IDLE && s->br_comm_if >= 0 &&
            (!s->br_dtr_sent || s->br_dtr != s->br_dtr_line)) {
            s->br_dtr_line = s->br_dtr;
            s->br_dtr_sent = true;
            br_ctl_start(s, USB_DIR_OUT | USB_TYPE_CLASS |
                         USB_RECIP_INTERFACE, CDC_SET_CONTROL_LINE_STATE,
                         s->br_dtr ? CDC_DTR | CDC_RTS : 0, s->br_comm_if,
                         0, NULL);
        }
        if (s->br_ctl_stage != CTL_IDLE) {
            r = br_ctl_step(s, &ns);
            if (r == BR_FAIL) {
                s->br_ctl_stage = CTL_IDLE;
                r = BR_PROGRESS;
                ns = br_xact_ns(0);
            }
        } else if (s->br_in_ep && s->br_out_ep) {
            r = br_bulk(s, &ns);
        } else {
            return;
        }
        break;
    default:
        return;
    }
    if (r == BR_WAIT) {
        s->br_waiting = true;
        return;
    }
    timer_mod(s->br_timer, br_now() + ns);
}

void rp2350_usb_bridge_kick(RP2350USBCtrlState *s)
{
    int64_t when = br_now() + BR_KICK_NS;

    if (s->br_waiting) {
        s->br_waiting = false;
        if (!timer_pending(s->br_timer) ||
            timer_expire_time_ns(s->br_timer) > when) {
            timer_mod(s->br_timer, when);
        }
    }
}

/* [spec:nuos:req:emu.usb] */
void rp2350_usb_bridge_port_changed(RP2350USBCtrlState *s)
{
    timer_del(s->br_timer);
    timer_del(s->br_frame_timer);
    s->br_waiting = false;
    if (rp2350_usbctrl_dev_present(s) &&
        qemu_chr_fe_backend_connected(&s->cdc_chr)) {
        br_set_state(s, BRIDGE_DEBOUNCE, BR_DEBOUNCE_NS);
    } else {
        s->br_state = BRIDGE_DETACHED;
    }
}

static int br_can_receive(void *opaque)
{
    RP2350USBCtrlState *s = opaque;

    if (s->br_state != BRIDGE_RUNNING || !s->br_out_ep) {
        return 0;
    }
    return sizeof(s->br_tx) - s->br_tx_len;
}

static void br_receive(void *opaque, const uint8_t *buf, int size)
{
    RP2350USBCtrlState *s = opaque;

    size = MIN(size, (int)(sizeof(s->br_tx) - s->br_tx_len));
    memcpy(s->br_tx + s->br_tx_len, buf, size);
    s->br_tx_len += size;
    rp2350_usb_bridge_kick(s);
}

/* DTR follows whether something has the chardev open. */
static void br_event(void *opaque, QEMUChrEvent event)
{
    RP2350USBCtrlState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->br_dtr = true;
        break;
    case CHR_EVENT_CLOSED:
        s->br_dtr = false;
        break;
    default:
        return;
    }
    rp2350_usb_bridge_kick(s);
}

/* Token port */

static void tok_reply(RP2350USBCtrlState *s, char result, uint8_t b1,
                      const uint8_t *data, uint16_t len)
{
    uint8_t hdr[4] = { result, b1, len, len >> 8 };

    qemu_chr_fe_write_all(&s->token_chr, hdr, sizeof(hdr));
    if (len) {
        qemu_chr_fe_write_all(&s->token_chr, data, len);
    }
}

static char tok_handshake(RP2350USBHandshake r)
{
    switch (r) {
    case RP2350_USB_ACK:
        return 'A';
    case RP2350_USB_NAK:
        return 'N';
    case RP2350_USB_STALL:
        return 'S';
    default:
        return 'T';
    }
}

/* [spec:nuos:req:emu.usb] */
static void tok_execute(RP2350USBCtrlState *s, const uint8_t *req,
                        const uint8_t *data)
{
    uint8_t buf[1024];
    uint8_t op = req[0], addr = req[1], ep = req[2] & 0xf;
    int pid = req[3] & 1;
    uint16_t len = req[4] | (req[5] << 8), n;
    RP2350USBHandshake r;

    switch (op) {
    case 'S':
        if (len != 8) {
            tok_reply(s, 'T', 0, NULL, 0);
            return;
        }
        r = rp2350_usbctrl_dev_setup(s, addr, pid, data);
        tok_reply(s, tok_handshake(r), 0, NULL, 0);
        return;
    case 'O':
        r = rp2350_usbctrl_dev_out(s, addr, ep, pid, data, len);
        tok_reply(s, tok_handshake(r), 0, NULL, 0);
        return;
    case 'I':
        r = rp2350_usbctrl_dev_in(s, addr, ep, buf, MIN(len, sizeof(buf)),
                                  &n, &pid);
        if (r == RP2350_USB_ACK) {
            tok_reply(s, 'A', pid, buf, n);
        } else {
            tok_reply(s, tok_handshake(r), 0, NULL, 0);
        }
        return;
    case 'R':
        rp2350_usbctrl_dev_bus_reset(s, true);
        rp2350_usbctrl_dev_bus_reset(s, false);
        tok_reply(s, 'A', 0, NULL, 0);
        return;
    case 'F':
        rp2350_usbctrl_dev_sof(s, len);
        tok_reply(s, 'A', 0, NULL, 0);
        return;
    case 'P':
        tok_reply(s, 'A', (rp2350_usbctrl_dev_present(s) ? 1 : 0) |
                  (s->dev_suspended ? 2 : 0), NULL, 0);
        return;
    default:
        tok_reply(s, 'T', 0, NULL, 0);
        return;
    }
}

static int tok_can_receive(void *opaque)
{
    RP2350USBCtrlState *s = opaque;

    return sizeof(s->tok_buf) - s->tok_len;
}

static void tok_receive(void *opaque, const uint8_t *buf, int size)
{
    RP2350USBCtrlState *s = opaque;
    uint32_t need, len;

    size = MIN(size, (int)(sizeof(s->tok_buf) - s->tok_len));
    memcpy(s->tok_buf + s->tok_len, buf, size);
    s->tok_len += size;
    while (s->tok_len >= 8) {
        len = s->tok_buf[4] | (s->tok_buf[5] << 8);
        need = 8;
        if (s->tok_buf[0] == 'S' || s->tok_buf[0] == 'O') {
            need += MIN(len, 1024);
        }
        if (s->tok_len < need) {
            return;
        }
        if (need > 8 && len > 1024) {
            s->tok_buf[4] = 0;
            s->tok_buf[5] = 4;
        }
        tok_execute(s, s->tok_buf, s->tok_buf + 8);
        s->tok_len -= need;
        memmove(s->tok_buf, s->tok_buf + need, s->tok_len);
    }
}

/* The token port's host is attached while its chardev is connected. */
static void tok_event(void *opaque, QEMUChrEvent event)
{
    RP2350USBCtrlState *s = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        s->token_open = true;
        break;
    case CHR_EVENT_CLOSED:
        s->token_open = false;
        s->tok_len = 0;
        break;
    default:
        return;
    }
    rp2350_usbctrl_update_dev_port(s);
}

void rp2350_usb_bridge_realize(RP2350USBCtrlState *s)
{
    s->br_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, br_run, s);
    s->br_frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, br_frame, s);
    if (qemu_chr_fe_backend_connected(&s->cdc_chr)) {
        qemu_chr_fe_set_handlers(&s->cdc_chr, br_can_receive, br_receive,
                                 br_event, NULL, s, NULL, true);
        s->br_dtr = qemu_chr_fe_backend_open(&s->cdc_chr);
    }
    if (qemu_chr_fe_backend_connected(&s->token_chr)) {
        qemu_chr_fe_set_handlers(&s->token_chr, tok_can_receive, tok_receive,
                                 tok_event, NULL, s, NULL, true);
    }
}

void rp2350_usb_bridge_reset(RP2350USBCtrlState *s)
{
    timer_del(s->br_timer);
    timer_del(s->br_frame_timer);
    s->br_state = BRIDGE_DETACHED;
    s->br_waiting = false;
    s->br_ctl_stage = CTL_IDLE;
    s->br_tx_len = 0;
    s->br_frame = 0;
}

const VMStateDescription vmstate_rp2350_usb_bridge = {
    .name = TYPE_RP2350_USBCTRL "/bridge",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(br_state, RP2350USBCtrlState),
        VMSTATE_TIMER_PTR(br_timer, RP2350USBCtrlState),
        VMSTATE_TIMER_PTR(br_frame_timer, RP2350USBCtrlState),
        VMSTATE_UINT16(br_frame, RP2350USBCtrlState),
        VMSTATE_INT64(br_frame_ns, RP2350USBCtrlState),
        VMSTATE_UINT8(br_addr, RP2350USBCtrlState),
        VMSTATE_UINT8(br_mps0, RP2350USBCtrlState),
        VMSTATE_UINT8(br_step, RP2350USBCtrlState),
        VMSTATE_UINT8(br_attempt, RP2350USBCtrlState),
        VMSTATE_UINT8(br_config, RP2350USBCtrlState),
        VMSTATE_INT32(br_comm_if, RP2350USBCtrlState),
        VMSTATE_UINT8(br_in_ep, RP2350USBCtrlState),
        VMSTATE_UINT8(br_out_ep, RP2350USBCtrlState),
        VMSTATE_UINT16(br_in_mps, RP2350USBCtrlState),
        VMSTATE_UINT16(br_out_mps, RP2350USBCtrlState),
        VMSTATE_UINT32(br_toggle, RP2350USBCtrlState),
        VMSTATE_BOOL(br_dtr, RP2350USBCtrlState),
        VMSTATE_BOOL(br_dtr_line, RP2350USBCtrlState),
        VMSTATE_BOOL(br_dtr_sent, RP2350USBCtrlState),
        VMSTATE_BOOL(br_waiting, RP2350USBCtrlState),
        VMSTATE_BOOL(br_turn, RP2350USBCtrlState),
        VMSTATE_UINT8_ARRAY(br_setup, RP2350USBCtrlState, 8),
        VMSTATE_UINT8_ARRAY(br_ctl_data, RP2350USBCtrlState,
                            RP2350_USB_BRIDGE_CFG_MAX),
        VMSTATE_UINT16(br_ctl_len, RP2350USBCtrlState),
        VMSTATE_UINT16(br_ctl_done, RP2350USBCtrlState),
        VMSTATE_UINT8(br_ctl_stage, RP2350USBCtrlState),
        VMSTATE_UINT8(br_ctl_toggle, RP2350USBCtrlState),
        VMSTATE_UINT8_ARRAY(br_tx, RP2350USBCtrlState,
                            RP2350_USB_BRIDGE_TX_MAX),
        VMSTATE_UINT32(br_tx_len, RP2350USBCtrlState),
        VMSTATE_BOOL(token_open, RP2350USBCtrlState),
        VMSTATE_UINT8_ARRAY(tok_buf, RP2350USBCtrlState,
                            RP2350_USB_TOKEN_MAX),
        VMSTATE_UINT32(tok_len, RP2350USBCtrlState),
        VMSTATE_END_OF_LIST()
    },
};
