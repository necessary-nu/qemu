/*
 * USB device-port hosts: the CDC-ACM bridge and the token port
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * See usb-devport.h for what the hosts do and for the token port's
 * protocol. Every transaction goes through the device controller, which
 * answers it as its own state machines decide; the hosts only choose which
 * transactions to run and when, at full speed (12 Mb/s) with an SOF each
 * millisecond.
 *
 * The bridge's frames and steps each run at their time however late QEMU
 * runs their timers (line_run_at()), each timed from the one before, and
 * its device controller takes those due before a register access first
 * (usb_devport_next()), as on the chip software finds every transaction
 * of the host's that has come by then.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "hw/core/line-time.h"
#include "hw/usb/usb.h"
#include "hw/usb/usb-devport.h"
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

/* The present, or the time of the bridge's event being taken. */
static int64_t br_now(void)
{
    return line_time();
}

static void br_timer_at(USBDevPort *p, int64_t when)
{
    p->br_due = when;
    timer_mod(p->br_timer, when);
}

static int64_t br_xact_ns(uint32_t len)
{
    return (len + 13) * 8 * BR_FS_BIT_NS;
}

static bool br_present(USBDevPort *p)
{
    return p->ops->present(p->opaque);
}

static bool br_ctl_in(USBDevPort *p)
{
    return p->br_setup[0] & USB_DIR_IN;
}

static void br_ctl_start(USBDevPort *p, uint8_t type, uint8_t req,
                         uint16_t value, uint16_t index, uint16_t len,
                         const uint8_t *data)
{
    p->br_setup[0] = type;
    p->br_setup[1] = req;
    p->br_setup[2] = value;
    p->br_setup[3] = value >> 8;
    p->br_setup[4] = index;
    p->br_setup[5] = index >> 8;
    p->br_setup[6] = len;
    p->br_setup[7] = len >> 8;
    p->br_ctl_len = MIN(len, USB_DEVPORT_CFG_MAX);
    p->br_ctl_done = 0;
    p->br_ctl_stage = CTL_SETUP;
    if (data) {
        memcpy(p->br_ctl_data, data, p->br_ctl_len);
    }
}

/*
 * One transaction of a control transfer: SETUP, then data packets of up
 * to the device's EP0 packet size starting with DATA1, then the status
 * stage in the other direction, a zero-length DATA1 packet.
 */
static BrResult br_ctl_step(USBDevPort *p, int64_t *ns)
{
    USBDevPortHandshake r;
    uint16_t len, want;
    int pid;

    switch (p->br_ctl_stage) {
    case CTL_SETUP:
        r = p->ops->setup(p->opaque, p->br_addr, p->br_setup);
        if (r != USB_DEVPORT_ACK) {
            return BR_FAIL;
        }
        p->br_ctl_toggle = 1;
        p->br_ctl_stage = p->br_ctl_len ? CTL_DATA : CTL_STATUS;
        *ns = br_xact_ns(8);
        return BR_PROGRESS;
    case CTL_DATA:
        want = MIN(p->br_mps0, p->br_ctl_len - p->br_ctl_done);
        if (br_ctl_in(p)) {
            r = p->ops->in(p->opaque, p->br_addr, 0,
                           p->br_ctl_data + p->br_ctl_done, want, &len,
                           &pid);
        } else {
            len = want;
            r = p->ops->out(p->opaque, p->br_addr, 0, p->br_ctl_toggle,
                            p->br_ctl_data + p->br_ctl_done, len);
            pid = p->br_ctl_toggle;
        }
        if (r == USB_DEVPORT_NAK) {
            return BR_WAIT;
        }
        if (r != USB_DEVPORT_ACK) {
            return BR_FAIL;
        }
        *ns = br_xact_ns(len);
        /* Data with the wrong PID is a repeat of the last; drop it. */
        if (pid != p->br_ctl_toggle) {
            return BR_PROGRESS;
        }
        p->br_ctl_toggle ^= 1;
        p->br_ctl_done += len;
        if (p->br_ctl_done >= p->br_ctl_len ||
            (br_ctl_in(p) && len < p->br_mps0)) {
            p->br_ctl_stage = CTL_STATUS;
        }
        return BR_PROGRESS;
    case CTL_STATUS:
        if (br_ctl_in(p)) {
            r = p->ops->out(p->opaque, p->br_addr, 0, 1, NULL, 0);
        } else {
            r = p->ops->in(p->opaque, p->br_addr, 0, NULL, 0, &len, &pid);
        }
        if (r == USB_DEVPORT_NAK) {
            return BR_WAIT;
        }
        if (r != USB_DEVPORT_ACK) {
            return BR_FAIL;
        }
        p->br_ctl_stage = CTL_IDLE;
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
static void br_parse_config(USBDevPort *p)
{
    const uint8_t *d = p->br_ctl_data;
    uint32_t len = p->br_ctl_done, off = 0;
    int cur_class = -1;
    bool data_found = false;

    p->br_config = len >= 9 ? d[5] : 1;
    p->br_comm_if = -1;
    p->br_in_ep = 0;
    p->br_out_ep = 0;
    while (off + 2 <= len && d[off] >= 2 && off + d[off] <= len) {
        const uint8_t *desc = d + off;

        if (desc[1] == DESC_INTERFACE && desc[0] >= 9) {
            cur_class = desc[5];
            if (cur_class == CLASS_CDC_COMM && desc[6] == SUBCLASS_ACM &&
                p->br_comm_if < 0) {
                p->br_comm_if = desc[2];
            }
            if (cur_class == CLASS_CDC_DATA && p->br_in_ep && p->br_out_ep) {
                data_found = true;
            }
        } else if (desc[1] == DESC_ENDPOINT && desc[0] >= 7 &&
                   cur_class == CLASS_CDC_DATA && !data_found &&
                   (desc[3] & 3) == EP_XFER_BULK) {
            uint16_t mps = (desc[4] | (desc[5] << 8)) & 0x7ff;

            if (desc[2] & USB_DIR_IN) {
                if (!p->br_in_ep) {
                    p->br_in_ep = desc[2] & 0xf;
                    p->br_in_mps = mps;
                }
            } else if (!p->br_out_ep) {
                p->br_out_ep = desc[2] & 0xf;
                p->br_out_mps = mps;
            }
        }
        off += d[off];
    }
}

static void br_set_state(USBDevPort *p, USBDevPortBridgeState state,
                         int64_t delay)
{
    p->br_state = state;
    p->br_waiting = false;
    br_timer_at(p, br_now() + delay);
}

/* Start enumeration step `step`. */
static void br_enum_step(USBDevPort *p, int step)
{
    static const uint8_t line_coding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0,
                                            8 };

    p->br_step = step;
    switch (step) {
    case STEP_GET_DEVICE8:
        br_ctl_start(p, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_DEVICE << 8, 0,
                     8, NULL);
        break;
    case STEP_SET_ADDRESS:
        br_ctl_start(p, USB_DIR_OUT, REQ_SET_ADDRESS, BR_ADDR, 0, 0, NULL);
        break;
    case STEP_GET_DEVICE:
        br_ctl_start(p, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_DEVICE << 8, 0,
                     18, NULL);
        break;
    case STEP_GET_CONFIG9:
        br_ctl_start(p, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_CONFIG << 8, 0,
                     9, NULL);
        break;
    case STEP_GET_CONFIG:
        br_ctl_start(p, USB_DIR_IN, REQ_GET_DESCRIPTOR, DESC_CONFIG << 8, 0,
                     MIN(p->br_ctl_data[2] | (p->br_ctl_data[3] << 8),
                         USB_DEVPORT_CFG_MAX), NULL);
        break;
    case STEP_SET_CONFIGURATION:
        br_parse_config(p);
        br_ctl_start(p, USB_DIR_OUT, REQ_SET_CONFIGURATION, p->br_config, 0,
                     0, NULL);
        break;
    case STEP_SET_LINE_CODING:
        br_ctl_start(p, USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                     CDC_SET_LINE_CODING, 0, p->br_comm_if,
                     sizeof(line_coding), line_coding);
        break;
    default:
        break;
    }
}

/* The next step after the current one completes, and how long to wait. */
static int br_enum_next(USBDevPort *p, int64_t *delay)
{
    *delay = 0;
    switch (p->br_step) {
    case STEP_GET_DEVICE8:
        if (p->br_ctl_done >= 8 && p->br_ctl_data[7]) {
            p->br_mps0 = p->br_ctl_data[7];
        }
        return STEP_SET_ADDRESS;
    case STEP_SET_ADDRESS:
        p->br_addr = BR_ADDR;
        *delay = BR_SET_ADDRESS_NS;
        return STEP_GET_DEVICE;
    case STEP_GET_DEVICE:
        return STEP_GET_CONFIG9;
    case STEP_GET_CONFIG9:
        return STEP_GET_CONFIG;
    case STEP_GET_CONFIG:
        return STEP_SET_CONFIGURATION;
    case STEP_SET_CONFIGURATION:
        p->br_toggle = 0;
        if (!p->br_in_ep || !p->br_out_ep) {
            qemu_log_mask(LOG_UNIMP, "%s: the device has no CDC-ACM data "
                          "interface for the bridge\n", p->name);
            return STEP_DONE;
        }
        return p->br_comm_if >= 0 ? STEP_SET_LINE_CODING : STEP_DONE;
    default:
        return STEP_DONE;
    }
}

static void br_fail(USBDevPort *p)
{
    if (++p->br_attempt < BR_ATTEMPTS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: enumeration failed at step %d; "
                      "resetting the device\n", p->name, p->br_step);
        timer_del(p->br_frame_timer);
        br_set_state(p, USB_DEVPORT_RESET, 0);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: enumeration failed at step %d; "
                  "giving up\n", p->name, p->br_step);
    timer_del(p->br_frame_timer);
    p->br_state = USB_DEVPORT_FAILED;
}

/* One bulk transaction between the chardev and the CDC data endpoints. */
static BrResult br_bulk(USBDevPort *p, int64_t *ns)
{
    uint8_t buf[1024];
    USBDevPortHandshake r;
    uint32_t bit;
    uint16_t len;
    int pid, i;

    for (i = 0; i < 2; i++) {
        bool in = p->br_turn ^ i;

        if (in) {
            bit = 1u << (p->br_in_ep * 2);
            r = p->ops->in(p->opaque, p->br_addr, p->br_in_ep, buf,
                           MIN(p->br_in_mps, sizeof(buf)), &len, &pid);
            if (r != USB_DEVPORT_ACK) {
                continue;
            }
            *ns = br_xact_ns(len);
            if (pid == !!(p->br_toggle & bit)) {
                p->br_toggle ^= bit;
                qemu_chr_fe_write_all(&p->cdc_chr, buf, len);
            }
        } else {
            if (!p->br_tx_len) {
                continue;
            }
            bit = 1u << (p->br_out_ep * 2 + 1);
            len = MIN(p->br_out_mps, p->br_tx_len);
            r = p->ops->out(p->opaque, p->br_addr, p->br_out_ep,
                            !!(p->br_toggle & bit), p->br_tx, len);
            if (r != USB_DEVPORT_ACK) {
                continue;
            }
            *ns = br_xact_ns(len);
            p->br_toggle ^= bit;
            p->br_tx_len -= len;
            memmove(p->br_tx, p->br_tx + len, p->br_tx_len);
            qemu_chr_fe_accept_input(&p->cdc_chr);
        }
        p->br_turn = !in;
        return BR_PROGRESS;
    }
    return BR_WAIT;
}

static void br_frame(void *opaque)
{
    USBDevPort *p = opaque;

    p->br_frame_ns += BR_FRAME_NS;
    timer_mod(p->br_frame_timer, p->br_frame_ns);
    p->br_frame = (p->br_frame + 1) & 0x7ff;
    p->ops->sof(p->opaque, p->br_frame);
    if (p->br_waiting) {
        p->br_waiting = false;
        br_timer_at(p, br_now() + br_xact_ns(0));
    }
}

/* The bridge's host state machine: bus states, then one transaction. */
/* [spec:nuos:req:emu.esp32p4.usb] */
static void br_run(void *opaque)
{
    USBDevPort *p = opaque;
    int64_t ns = 0, delay;
    BrResult r;
    int next;

    if (!br_present(p)) {
        return;
    }
    switch (p->br_state) {
    case USB_DEVPORT_DEBOUNCE:
        p->br_attempt = 0;
        br_set_state(p, USB_DEVPORT_RESET, 0);
        return;
    case USB_DEVPORT_RESET:
        timer_del(p->br_frame_timer);
        p->ops->bus_reset(p->opaque, true);
        br_set_state(p, USB_DEVPORT_RECOVERY, BR_RESET_NS);
        return;
    case USB_DEVPORT_RECOVERY:
        /* SOFs start as the reset ends; requests after the recovery. */
        p->ops->bus_reset(p->opaque, false);
        p->br_addr = 0;
        p->br_mps0 = BR_MPS0_INITIAL;
        p->br_dtr_sent = false;
        p->br_tx_len = 0;
        p->br_frame_ns = br_now();
        br_frame(p);
        br_enum_step(p, STEP_GET_DEVICE8);
        br_set_state(p, USB_DEVPORT_ENUMERATE,
                     BR_RECOVERY_NS + br_xact_ns(0));
        return;
    case USB_DEVPORT_ENUMERATE:
        r = br_ctl_step(p, &ns);
        if (r == BR_FAIL) {
            br_fail(p);
            return;
        }
        if (r == BR_DONE) {
            /*
             * The next request goes in a later frame, after the device's
             * software has seen this transfer end.
             */
            next = br_enum_next(p, &delay);
            ns += MAX(delay, BR_FRAME_NS);
            if (next == STEP_DONE) {
                p->br_state = USB_DEVPORT_RUNNING;
                p->br_ctl_stage = CTL_IDLE;
                qemu_chr_fe_accept_input(&p->cdc_chr);
            } else {
                br_enum_step(p, next);
            }
        }
        break;
    case USB_DEVPORT_RUNNING:
        if (p->br_ctl_stage == CTL_IDLE && p->br_comm_if >= 0 &&
            (!p->br_dtr_sent || p->br_dtr != p->br_dtr_line)) {
            p->br_dtr_line = p->br_dtr;
            p->br_dtr_sent = true;
            br_ctl_start(p, USB_DIR_OUT | USB_TYPE_CLASS |
                         USB_RECIP_INTERFACE, CDC_SET_CONTROL_LINE_STATE,
                         p->br_dtr ? CDC_DTR | CDC_RTS : 0, p->br_comm_if,
                         0, NULL);
        }
        if (p->br_ctl_stage != CTL_IDLE) {
            r = br_ctl_step(p, &ns);
            if (r == BR_FAIL) {
                p->br_ctl_stage = CTL_IDLE;
                r = BR_PROGRESS;
                ns = br_xact_ns(0);
            } else if (r == BR_DONE) {
                ns += BR_FRAME_NS;
            }
        } else if (p->br_in_ep && p->br_out_ep) {
            r = br_bulk(p, &ns);
        } else {
            return;
        }
        break;
    default:
        return;
    }
    if (r == BR_WAIT) {
        p->br_waiting = true;
        return;
    }
    br_timer_at(p, br_now() + ns);
}

/* A frame or step runs at its time however late its timer runs. */
static void br_frame_cb(void *opaque)
{
    USBDevPort *p = opaque;

    line_run_at(p->br_frame_ns, br_frame, p);
}

static void br_run_cb(void *opaque)
{
    USBDevPort *p = opaque;

    line_run_at(p->br_due, br_run, p);
}

/* [spec:nuos:req:emu.esp32p4.usb] */
bool usb_devport_next(USBDevPort *p, int64_t *due, QEMUTimer **timer,
                      void (**run)(void *opaque))
{
    bool found = false;

    if (!p->br_timer) {
        return false;
    }
    if (timer_pending(p->br_frame_timer) && p->br_frame_ns <= *due) {
        *due = p->br_frame_ns;
        *timer = p->br_frame_timer;
        *run = br_frame;
        found = true;
    }
    if (timer_pending(p->br_timer) && p->br_due <= *due &&
        (!found || p->br_due < *due)) {
        *due = p->br_due;
        *timer = p->br_timer;
        *run = br_run;
        found = true;
    }
    return found;
}

void usb_devport_kick(USBDevPort *p)
{
    int64_t when = br_now() + BR_KICK_NS;

    if (p->br_waiting) {
        p->br_waiting = false;
        if (!timer_pending(p->br_timer) ||
            timer_expire_time_ns(p->br_timer) > when) {
            br_timer_at(p, when);
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.usb] */
void usb_devport_changed(USBDevPort *p)
{
    if (!p->br_timer) {
        return;
    }
    timer_del(p->br_timer);
    timer_del(p->br_frame_timer);
    p->br_waiting = false;
    if (br_present(p) && qemu_chr_fe_backend_connected(&p->cdc_chr)) {
        br_set_state(p, USB_DEVPORT_DEBOUNCE, BR_DEBOUNCE_NS);
    } else {
        p->br_state = USB_DEVPORT_DETACHED;
    }
}

bool usb_devport_host_attached(USBDevPort *p)
{
    return qemu_chr_fe_backend_connected(&p->cdc_chr) || p->token_open;
}

static int br_can_receive(void *opaque)
{
    USBDevPort *p = opaque;

    if (p->br_state != USB_DEVPORT_RUNNING || !p->br_out_ep) {
        return 0;
    }
    return sizeof(p->br_tx) - p->br_tx_len;
}

static void br_receive(void *opaque, const uint8_t *buf, int size)
{
    USBDevPort *p = opaque;

    size = MIN(size, (int)(sizeof(p->br_tx) - p->br_tx_len));
    memcpy(p->br_tx + p->br_tx_len, buf, size);
    p->br_tx_len += size;
    usb_devport_kick(p);
}

/* DTR follows whether something has the chardev open. */
static void br_event(void *opaque, QEMUChrEvent event)
{
    USBDevPort *p = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        p->br_dtr = true;
        break;
    case CHR_EVENT_CLOSED:
        p->br_dtr = false;
        break;
    default:
        return;
    }
    usb_devport_kick(p);
}

/* Token port */

static void tok_reply(USBDevPort *p, char result, uint8_t b1,
                      const uint8_t *data, uint16_t len)
{
    uint8_t hdr[4] = { result, b1, len, len >> 8 };

    qemu_chr_fe_write_all(&p->token_chr, hdr, sizeof(hdr));
    if (len) {
        qemu_chr_fe_write_all(&p->token_chr, data, len);
    }
}

static char tok_handshake(USBDevPortHandshake r)
{
    switch (r) {
    case USB_DEVPORT_ACK:
        return 'A';
    case USB_DEVPORT_NAK:
        return 'N';
    case USB_DEVPORT_STALL:
        return 'S';
    default:
        return 'T';
    }
}

/* [spec:nuos:req:emu.esp32p4.usb] */
static void tok_execute(USBDevPort *p, const uint8_t *req,
                        const uint8_t *data)
{
    uint8_t buf[1024];
    uint8_t op = req[0], addr = req[1], ep = req[2] & 0xf;
    int pid = req[3] & 1;
    uint16_t len = req[4] | (req[5] << 8), n;
    USBDevPortHandshake r;

    switch (op) {
    case 'S':
        if (len != 8) {
            tok_reply(p, 'T', 0, NULL, 0);
            return;
        }
        r = p->ops->setup(p->opaque, addr, data);
        tok_reply(p, tok_handshake(r), 0, NULL, 0);
        return;
    case 'O':
        r = p->ops->out(p->opaque, addr, ep, pid, data, len);
        tok_reply(p, tok_handshake(r), 0, NULL, 0);
        return;
    case 'I':
        r = p->ops->in(p->opaque, addr, ep, buf, MIN(len, sizeof(buf)), &n,
                       &pid);
        if (r == USB_DEVPORT_ACK) {
            tok_reply(p, 'A', pid, buf, n);
        } else {
            tok_reply(p, tok_handshake(r), 0, NULL, 0);
        }
        return;
    case 'R':
        p->ops->bus_reset(p->opaque, true);
        p->ops->bus_reset(p->opaque, false);
        tok_reply(p, 'A', 0, NULL, 0);
        return;
    case 'F':
        p->ops->sof(p->opaque, len);
        tok_reply(p, 'A', 0, NULL, 0);
        return;
    case 'P':
        tok_reply(p, 'A', (br_present(p) ? 1 : 0) |
                  (p->ops->suspended(p->opaque) ? 2 : 0), NULL, 0);
        return;
    default:
        tok_reply(p, 'T', 0, NULL, 0);
        return;
    }
}

static int tok_can_receive(void *opaque)
{
    USBDevPort *p = opaque;

    return sizeof(p->tok_buf) - p->tok_len;
}

static void tok_receive(void *opaque, const uint8_t *buf, int size)
{
    USBDevPort *p = opaque;
    uint32_t need, len;

    size = MIN(size, (int)(sizeof(p->tok_buf) - p->tok_len));
    memcpy(p->tok_buf + p->tok_len, buf, size);
    p->tok_len += size;
    while (p->tok_len >= 8) {
        len = p->tok_buf[4] | (p->tok_buf[5] << 8);
        need = 8;
        if (p->tok_buf[0] == 'S' || p->tok_buf[0] == 'O') {
            need += MIN(len, 1024);
        }
        if (p->tok_len < need) {
            return;
        }
        if (need > 8 && len > 1024) {
            p->tok_buf[4] = 0;
            p->tok_buf[5] = 4;
        }
        tok_execute(p, p->tok_buf, p->tok_buf + 8);
        p->tok_len -= need;
        memmove(p->tok_buf, p->tok_buf + need, p->tok_len);
    }
}

/* The token port's host is attached while its chardev is connected. */
static void tok_event(void *opaque, QEMUChrEvent event)
{
    USBDevPort *p = opaque;

    switch (event) {
    case CHR_EVENT_OPENED:
        p->token_open = true;
        break;
    case CHR_EVENT_CLOSED:
        p->token_open = false;
        p->tok_len = 0;
        break;
    default:
        return;
    }
    p->ops->host_attached(p->opaque, usb_devport_host_attached(p));
}

void usb_devport_init(USBDevPort *p, const char *name,
                      const USBDevPortOps *ops, void *opaque)
{
    p->name = name;
    p->ops = ops;
    p->opaque = opaque;
}

bool usb_devport_realize(USBDevPort *p, Error **errp)
{
    if (qemu_chr_fe_backend_connected(&p->cdc_chr) &&
        qemu_chr_fe_backend_connected(&p->token_chr)) {
        error_setg(errp, "%s: cdc-chardev and token-chardev are both hosts "
                   "on the one device port; set at most one", p->name);
        return false;
    }
    p->br_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, br_run_cb, p);
    p->br_frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, br_frame_cb, p);
    if (qemu_chr_fe_backend_connected(&p->cdc_chr)) {
        qemu_chr_fe_set_handlers(&p->cdc_chr, br_can_receive, br_receive,
                                 br_event, NULL, p, NULL, true);
        p->br_dtr = qemu_chr_fe_backend_open(&p->cdc_chr);
    }
    if (qemu_chr_fe_backend_connected(&p->token_chr)) {
        qemu_chr_fe_set_handlers(&p->token_chr, tok_can_receive, tok_receive,
                                 tok_event, NULL, p, NULL, true);
    }
    return true;
}

void usb_devport_reset(USBDevPort *p)
{
    timer_del(p->br_timer);
    timer_del(p->br_frame_timer);
    p->br_state = USB_DEVPORT_DETACHED;
    p->br_waiting = false;
    p->br_ctl_stage = CTL_IDLE;
    p->br_tx_len = 0;
    p->br_frame = 0;
}

const VMStateDescription vmstate_usb_devport = {
    .name = "usb-devport",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(br_state, USBDevPort),
        VMSTATE_TIMER_PTR(br_timer, USBDevPort),
        VMSTATE_INT64(br_due, USBDevPort),
        VMSTATE_TIMER_PTR(br_frame_timer, USBDevPort),
        VMSTATE_UINT16(br_frame, USBDevPort),
        VMSTATE_INT64(br_frame_ns, USBDevPort),
        VMSTATE_UINT8(br_addr, USBDevPort),
        VMSTATE_UINT8(br_mps0, USBDevPort),
        VMSTATE_UINT8(br_step, USBDevPort),
        VMSTATE_UINT8(br_attempt, USBDevPort),
        VMSTATE_UINT8(br_config, USBDevPort),
        VMSTATE_INT32(br_comm_if, USBDevPort),
        VMSTATE_UINT8(br_in_ep, USBDevPort),
        VMSTATE_UINT8(br_out_ep, USBDevPort),
        VMSTATE_UINT16(br_in_mps, USBDevPort),
        VMSTATE_UINT16(br_out_mps, USBDevPort),
        VMSTATE_UINT32(br_toggle, USBDevPort),
        VMSTATE_BOOL(br_dtr, USBDevPort),
        VMSTATE_BOOL(br_dtr_line, USBDevPort),
        VMSTATE_BOOL(br_dtr_sent, USBDevPort),
        VMSTATE_BOOL(br_waiting, USBDevPort),
        VMSTATE_BOOL(br_turn, USBDevPort),
        VMSTATE_UINT8_ARRAY(br_setup, USBDevPort, 8),
        VMSTATE_UINT8_ARRAY(br_ctl_data, USBDevPort, USB_DEVPORT_CFG_MAX),
        VMSTATE_UINT16(br_ctl_len, USBDevPort),
        VMSTATE_UINT16(br_ctl_done, USBDevPort),
        VMSTATE_UINT8(br_ctl_stage, USBDevPort),
        VMSTATE_UINT8(br_ctl_toggle, USBDevPort),
        VMSTATE_UINT8_ARRAY(br_tx, USBDevPort, USB_DEVPORT_TX_MAX),
        VMSTATE_UINT32(br_tx_len, USBDevPort),
        VMSTATE_BOOL(token_open, USBDevPort),
        VMSTATE_UINT8_ARRAY(tok_buf, USBDevPort, USB_DEVPORT_TOKEN_MAX),
        VMSTATE_UINT32(tok_len, USBDevPort),
        VMSTATE_END_OF_LIST()
    },
};
