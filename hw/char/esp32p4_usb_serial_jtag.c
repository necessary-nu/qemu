/*
 * ESP32-P4 USB Serial/JTAG controller: the CDC-ACM serial port
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM chapter 54 (USB Serial/JTAG Controller), with
 * ESP-IDF's rev3 usb_serial_jtag_reg.h and its usb_serial_jtag_ll.h.
 *
 * The controller is a USB full-speed device whose CDC-ACM function
 * software sees as two 64-byte endpoint buffers. The model puts a USB
 * host on the other end of its PHY, the chardev, behaving as a computer
 * with the board plugged in does (as the RP2350 USB model's CDC-ACM
 * bridge does, hw/usb/rp2350_usbctrl_bridge.c): once it sees the D+
 * pull-up it waits out the 100 ms attach debounce, resets the bus for
 * 10 ms (USB_BUS_RESET, which empties the endpoints), allows 10 ms of
 * reset recovery, then sends an SOF every millisecond (SOF, FRAM_NUM) and
 * enumerates the device in five frames, which the controller's hardware
 * answers by itself. Its CDC-ACM driver then sends SET_LINE_CODING
 * (115200 8N1: SET_LINE_CODE, SET_LINE_CODE_W0/W1), GET_LINE_CODING
 * (GET_LINE_CODE) and SET_CONTROL_LINE_STATE, with DTR and RTS set while
 * the chardev is open (DTR_CHG, RTS_CHG, CHIP_RST's DTR and RTS), a frame
 * apart, and again when the chardev opens or closes.
 *
 * While the chardev is open the host reads the bulk IN endpoint: a buffer
 * flushed by WR_DONE, or by its 64th byte, goes to the host in one
 * transaction, its bus time at 12 Mbit/s, after which the buffer is free
 * again (SERIAL_IN_EP_DATA_FREE, SERIAL_IN_EMPTY). The host polls with an
 * IN token (IN_TOKEN_REC_IN_EP1) on every transaction and once a frame
 * while it has nothing to read, and NAKs while the chardev cannot take
 * the data. Bytes from the chardev reach the bulk OUT endpoint in packets
 * of up to 64 bytes, one each time software has read the last one out
 * (SERIAL_OUT_RECV_PKT, SERIAL_OUT_EP_DATA_AVAIL).
 *
 * Every transaction and frame happens at its own virtual time: a register
 * access, and the chardev's traffic, first complete what fell due before
 * them, so that without icount software polling SERIAL_IN_EP_DATA_FREE
 * sees the IN buffer free once its transaction is over however late the
 * host runs the model's timers, as the boot ROM's 5 ms wait for it
 * (usb_serial_device_tx_one_char) needs.
 *
 * The host never changes DTR and RTS in the sequences of TRM table 54.3-2
 * that reset the chip or set the download flag, so neither happens. The
 * JTAG function (its endpoints, command processor and response capture)
 * has no host traffic and is not modelled; USB_JTAG_BRIDGE_EN, the pad
 * test mode, forcing the USB memory off and the external PHY are logged
 * as unimplemented. CONFIG_UPDATE is accepted; the configuration takes
 * effect as written.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/char/esp32p4_usb_serial_jtag.h"
#include "hw/gpio/esp32_line.h"
#include "migration/vmstate.h"

#define A_EP1               0x00
#define A_EP1_CONF          0x04
#define A_INT_RAW           0x08
#define A_INT_ST            0x0c
#define A_INT_ENA           0x10
#define A_INT_CLR           0x14
#define A_CONF0             0x18
#define A_TEST              0x1c
#define A_JFIFO_ST          0x20
#define A_FRAM_NUM          0x24
#define A_IN_EP0_ST         0x28
#define A_IN_EP1_ST         0x2c
#define A_IN_EP2_ST         0x30
#define A_IN_EP3_ST         0x34
#define A_OUT_EP0_ST        0x38
#define A_OUT_EP1_ST        0x3c
#define A_OUT_EP2_ST        0x40
#define A_MISC_CONF         0x44
#define A_MEM_CONF          0x48
#define A_CHIP_RST          0x4c
#define A_SET_LINE_CODE_W0  0x50
#define A_SET_LINE_CODE_W1  0x54
#define A_GET_LINE_CODE_W0  0x58
#define A_GET_LINE_CODE_W1  0x5c
#define A_CONFIG_UPDATE     0x60
#define A_SER_AFIFO_CONFIG  0x64
#define A_BUS_RESET_ST      0x68
#define A_ECO_LOW_48        0x6c
#define A_ECO_HIGH_48       0x70
#define A_ECO_CELL_CTRL_48  0x74
#define A_ECO_LOW_APB       0x78
#define A_ECO_HIGH_APB      0x7c
#define A_ECO_CELL_CTRL_APB 0x80
#define A_SRAM_CTRL         0x84
#define A_DATE              0x88

#define R(a)                ((a) / 4)

#define EP1_WR_DONE         BIT(0)
#define EP1_IN_DATA_FREE    BIT(1)
#define EP1_OUT_DATA_AVAIL  BIT(2)

#define INT_JTAG_IN_FLUSH   BIT(0)
#define INT_SOF             BIT(1)
#define INT_OUT_RECV_PKT    BIT(2)
#define INT_IN_EMPTY        BIT(3)
#define INT_IN_TOKEN_EP1    BIT(8)
#define INT_BUS_RESET       BIT(9)
#define INT_OUT_EP1_ZERO    BIT(10)
#define INT_RTS_CHG         BIT(12)
#define INT_DTR_CHG         BIT(13)
#define INT_GET_LINE_CODE   BIT(14)
#define INT_SET_LINE_CODE   BIT(15)
#define INT_MASK            0xffffu

#define CONF0_PHY_SEL       BIT(0)
#define CONF0_EXCHG_OVR     BIT(1)
#define CONF0_EXCHG_PINS    BIT(2)
#define CONF0_PULL_OVERRIDE BIT(8)
#define CONF0_DP_PULLUP     BIT(9)
#define CONF0_DM_PULLUP     BIT(11)
#define CONF0_PAD_ENABLE    BIT(14)
#define CONF0_JTAG_BRIDGE   BIT(15)

#define TEST_ENABLE         BIT(0)
#define TEST_RO_IDLE        0x30u

#define CHIP_RST_RTS        BIT(0)
#define CHIP_RST_DTR        BIT(1)

#define AFIFO_IN_RESET      (BIT(0) | BIT(1))
#define AFIFO_OUT_RESET     (BIT(2) | BIT(3))
#define AFIFO_OUT_REMPTY    BIT(4)

#define MEM_PD              BIT(0)

/* The JTAG FIFOs: both empty, their resets software's. */
#define JFIFO_EMPTY         0x44u
#define JFIFO_RESETS        0x300u

/* The host's timing (USB 2.0 sections 7.1.7.3 and 9.2.6) */
#define DEBOUNCE_NS         (100 * SCALE_MS)
#define BUS_RESET_NS        (10 * SCALE_MS)
#define RECOVERY_NS         (10 * SCALE_MS)
#define FRAME_NS            SCALE_MS
#define ENUM_FRAMES         5
#define FS_BIT_NS           (NANOSECONDS_PER_SECOND / 12000000)

/* The CDC-ACM driver's requests after configuration, a frame apart. */
enum {
    CDC_SET_LINE_CODING,
    CDC_GET_LINE_CODING,
    CDC_SET_CONTROL_LINE_STATE,
    CDC_DONE,
};

#define HOST_BUF            4096
#define HOST_IN_BUF         256

typedef struct UsjRegInfo {
    uint32_t reset;
    uint32_t rw;
} UsjRegInfo;

/* The registers software writes, their reset values and writable bits. */
static const UsjRegInfo usj_regs[ESP32P4_USJ_NREGS] = {
    [R(A_INT_ENA)]          = { 0x00000000, INT_MASK },
    [R(A_CONF0)]            = { 0x00004200, 0x0000ffff },
    [R(A_TEST)]             = { 0x00000000, 0x0000000f },
    [R(A_JFIFO_ST)]         = { 0x00000000, JFIFO_RESETS },
    [R(A_MISC_CONF)]        = { 0x00000000, 0x00000001 },
    [R(A_MEM_CONF)]         = { 0x00000002, 0x00000003 },
    [R(A_CHIP_RST)]         = { 0x00000000, 0x00000004 },
    [R(A_GET_LINE_CODE_W0)] = { 0x00000000, 0xffffffff },
    [R(A_GET_LINE_CODE_W1)] = { 0x00000000, 0x00ffffff },
    [R(A_SER_AFIFO_CONFIG)] = { 0x00000000, 0x0000000f },
    [R(A_ECO_LOW_48)]       = { 0x00000000, 0xffffffff },
    [R(A_ECO_HIGH_48)]      = { 0xffffffff, 0xffffffff },
    [R(A_ECO_CELL_CTRL_48)] = { 0x00000000, 0x00000002 },
    [R(A_ECO_LOW_APB)]      = { 0x00000000, 0xffffffff },
    [R(A_ECO_HIGH_APB)]     = { 0xffffffff, 0xffffffff },
    [R(A_ECO_CELL_CTRL_APB)] = { 0x00000000, 0x00000002 },
    [R(A_SRAM_CTRL)]        = { 0x00001320, 0x00003fff },
    [R(A_DATE)]             = { 0x02112010, 0xffffffff },
};

static void usj_host_update(ESP32P4UsjState *s);
static void usj_xfer_try(ESP32P4UsjState *s);
static void usj_catch_up(ESP32P4UsjState *s);

/* The present, or the time of the transaction or frame being taken. */
static int64_t now_ns(void)
{
    return esp32_line_time();
}

/* A bulk transaction's bus time: token, data with PID and CRC, handshake. */
static int64_t xact_ns(unsigned len)
{
    return (len + 13) * 8 * FS_BIT_NS;
}

static void usj_update_irq(ESP32P4UsjState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->regs[R(A_INT_ENA)]));
}

static bool usj_usb_running(ESP32P4UsjState *s)
{
    return clock_is_enabled(s->usb48);
}

/* ---- The PHY's pulls ---- */

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * [spec:nuos:req:emu.esp32p4.usb]
 * With USB_PAD_ENABLE the PHY has its pads, and pulls D+ up, as a
 * full-speed device attaches, or with PAD_PULL_OVERRIDE as DP_PULLUP and
 * DM_PULLUP say; D- is its first pad and D+ its second (GPIO24 and GPIO25
 * for PHY 0), the other way round with EXCHG_PINS. Which PHY is this
 * controller's is USB_WRAP's to route (hw/usb/esp32p4_usb.c).
 */
static void usj_pulls(ESP32P4UsjState *s, bool *dp, bool *dm)
{
    uint32_t c = s->regs[R(A_CONF0)];

    *dp = false;
    *dm = false;
    if (!(c & CONF0_PAD_ENABLE)) {
        return;
    }
    if (c & CONF0_PULL_OVERRIDE) {
        *dp = c & CONF0_DP_PULLUP;
        *dm = c & CONF0_DM_PULLUP;
    } else {
        *dp = true;
    }
}

static void usj_update_pulls(ESP32P4UsjState *s)
{
    uint32_t c = s->regs[R(A_CONF0)];
    bool dp, dm, swap = (c & CONF0_EXCHG_OVR) && (c & CONF0_EXCHG_PINS);

    usj_pulls(s, &dp, &dm);
    qemu_set_irq(s->pull_up[swap ? 1 : 0], dm);
    qemu_set_irq(s->pull_up[swap ? 0 : 1], dp);
    qemu_set_irq(s->pad_enable, !!(c & CONF0_PAD_ENABLE));
}

/* ---- Endpoints ---- */

static void usj_in_reset(ESP32P4UsjState *s)
{
    s->in_len = 0;
    s->in_flushed = false;
}

static void usj_out_reset(ESP32P4UsjState *s)
{
    s->out_len = 0;
    s->out_rd = 0;
}

static bool usj_out_avail(ESP32P4UsjState *s)
{
    return s->out_rd < s->out_len;
}

static uint32_t usj_ep1_conf(ESP32P4UsjState *s)
{
    return (s->in_flushed ? 0 : EP1_IN_DATA_FREE) |
           (usj_out_avail(s) ? EP1_OUT_DATA_AVAIL : 0);
}

/* ---- The host ---- */

/*
 * The host sees a full-speed device while D+ is pulled up and D- is not,
 * the cable in (a chardev given) and the PHY clocked.
 */
static bool usj_attached(ESP32P4UsjState *s)
{
    bool dp, dm;

    usj_pulls(s, &dp, &dm);
    return dp && !dm && qemu_chr_fe_backend_connected(&s->chr) &&
           usj_usb_running(s);
}

static bool usj_port_open(ESP32P4UsjState *s)
{
    return s->host_state == ESP32P4_USJ_CONFIGURED &&
           s->host_step == CDC_DONE && qemu_chr_fe_backend_open(&s->chr);
}

static void usj_host_detach(ESP32P4UsjState *s)
{
    s->host_state = ESP32P4_USJ_DETACHED;
    s->host_step = 0;
    timer_del(&s->host_timer);
    timer_del(&s->xfer_timer);
    s->xfer_busy = false;
    s->regs[R(A_BUS_RESET_ST)] = 1;
}

static void usj_host_enter(ESP32P4UsjState *s, uint8_t state, int64_t t,
                           int64_t len)
{
    s->host_state = state;
    s->host_step = 0;
    s->host_deadline = t + len;
}

/*
 * SET_CONTROL_LINE_STATE: DTR and RTS as the chardev is open; each change
 * raises DTR_CHG or RTS_CHG and shows in CHIP_RST.
 */
static void usj_set_line_state(ESP32P4UsjState *s)
{
    bool open = qemu_chr_fe_backend_open(&s->chr);

    s->line_state_pending = false;
    if (open != s->host_dtr) {
        s->host_dtr = open;
        s->int_raw |= INT_DTR_CHG;
    }
    if (open != s->host_rts) {
        s->host_rts = open;
        s->int_raw |= INT_RTS_CHG;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * A frame: its SOF, then the CDC-ACM driver's next request, and an IN
 * token on the bulk endpoint if the host reads it and nothing is flushed.
 */
static void usj_frame(ESP32P4UsjState *s)
{
    s->frame = (s->frame + 1) & 0x7ff;
    s->int_raw |= INT_SOF;
    if (s->host_state != ESP32P4_USJ_CONFIGURED) {
        return;
    }
    switch (s->host_step) {
    case CDC_SET_LINE_CODING:
        s->regs[R(A_SET_LINE_CODE_W0)] = 115200;
        s->regs[R(A_SET_LINE_CODE_W1)] = 8u << 16;
        s->int_raw |= INT_SET_LINE_CODE;
        s->host_step++;
        break;
    case CDC_GET_LINE_CODING:
        s->int_raw |= INT_GET_LINE_CODE;
        s->host_step++;
        break;
    case CDC_SET_CONTROL_LINE_STATE:
        usj_set_line_state(s);
        s->host_step++;
        break;
    default:
        if (s->line_state_pending) {
            usj_set_line_state(s);
        }
        if (usj_port_open(s) && !s->in_flushed && !s->xfer_busy) {
            s->int_raw |= INT_IN_TOKEN_EP1;
        }
        break;
    }
}

static void usj_host_schedule(ESP32P4UsjState *s)
{
    int64_t next;

    switch (s->host_state) {
    case ESP32P4_USJ_DETACHED:
        timer_del(&s->host_timer);
        return;
    case ESP32P4_USJ_DEBOUNCE:
    case ESP32P4_USJ_BUS_RESET:
        next = s->host_deadline;
        break;
    default:
        next = MIN(s->host_deadline, now_ns() + FRAME_NS);
        if (s->host_state == ESP32P4_USJ_CONFIGURED) {
            next = now_ns() + FRAME_NS;
        }
        break;
    }
    s->host_next = next;
    timer_mod_ns(&s->host_timer, next);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Bring the host's state up to the present: attach and detach, the bus
 * reset (USB_BUS_RESET, the endpoints emptied, BUS_RESET_ST low for its
 * 10 ms), recovery and enumeration.
 */
static void usj_host_update(ESP32P4UsjState *s)
{
    int64_t t = now_ns();

    if (!usj_attached(s)) {
        if (s->host_state != ESP32P4_USJ_DETACHED) {
            usj_host_detach(s);
        }
        return;
    }
    if (s->host_state == ESP32P4_USJ_DETACHED) {
        usj_host_enter(s, ESP32P4_USJ_DEBOUNCE, t, DEBOUNCE_NS);
    }
    while (s->host_state != ESP32P4_USJ_CONFIGURED &&
           t >= s->host_deadline) {
        int64_t at = s->host_deadline;

        switch (s->host_state) {
        case ESP32P4_USJ_DEBOUNCE:
            usj_host_enter(s, ESP32P4_USJ_BUS_RESET, at, BUS_RESET_NS);
            s->regs[R(A_BUS_RESET_ST)] = 0;
            usj_in_reset(s);
            usj_out_reset(s);
            s->host_dtr = false;
            s->host_rts = false;
            s->int_raw |= INT_BUS_RESET;
            break;
        case ESP32P4_USJ_BUS_RESET:
            s->regs[R(A_BUS_RESET_ST)] = 1;
            usj_host_enter(s, ESP32P4_USJ_RECOVERY, at, RECOVERY_NS);
            break;
        case ESP32P4_USJ_RECOVERY:
            usj_host_enter(s, ESP32P4_USJ_ENUMERATING, at,
                           ENUM_FRAMES * FRAME_NS);
            break;
        case ESP32P4_USJ_ENUMERATING:
            usj_host_enter(s, ESP32P4_USJ_CONFIGURED, at, 0);
            break;
        default:
            break;
        }
    }
    usj_host_schedule(s);
    usj_update_irq(s);
}

static void usj_host_step(void *opaque)
{
    ESP32P4UsjState *s = opaque;

    usj_host_update(s);
    if (s->host_state >= ESP32P4_USJ_RECOVERY) {
        usj_frame(s);
    }
    usj_xfer_try(s);
    usj_host_schedule(s);
    usj_update_irq(s);
}

/*
 * The host's next state change or frame, at its time however late its
 * timer runs: the frames keep to their millisecond (esp32_line_run_at()).
 */
static void usj_host_timer_cb(void *opaque)
{
    ESP32P4UsjState *s = opaque;

    esp32_line_run_at(s->host_next, usj_host_step, s);
}

/* ---- Bulk transactions ---- */

static gboolean usj_chr_writable(void *do_not_use, GIOCondition cond,
                                 void *opaque);

/* Hand the bytes the host has read to the chardev, as many as it takes. */
static void usj_to_host_flush(ESP32P4UsjState *s)
{
    while (!fifo8_is_empty(&s->to_host)) {
        uint32_t n;
        const uint8_t *buf = fifo8_peek_bufptr(&s->to_host,
                                               fifo8_num_used(&s->to_host),
                                               &n);
        int done = qemu_chr_fe_write(&s->chr, buf, n);

        if (done <= 0) {
            break;
        }
        fifo8_drop(&s->to_host, done);
    }
    if (!fifo8_is_empty(&s->to_host) && !s->watch) {
        s->watch = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                         usj_chr_writable, s);
        if (!s->watch) {
            fifo8_reset(&s->to_host);
        }
    }
}

static gboolean usj_chr_writable(void *do_not_use, GIOCondition cond,
                                 void *opaque)
{
    ESP32P4UsjState *s = opaque;

    s->watch = 0;
    usj_catch_up(s);
    usj_to_host_flush(s);
    usj_xfer_try(s);
    return G_SOURCE_REMOVE;
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Start the next bulk transaction at @t if the bus is free: the flushed IN
 * buffer to a host with room for it, else, once software has read the
 * last OUT packet, the host's next bytes as an OUT packet.
 */
static void usj_xfer_try_at(ESP32P4UsjState *s, int64_t t)
{
    if (s->xfer_busy || !usj_port_open(s) || !usj_usb_running(s)) {
        return;
    }
    if (s->in_flushed &&
        fifo8_num_free(&s->to_host) >= s->in_len) {
        s->xfer_busy = true;
        s->xfer_in = true;
        s->xfer_len = s->in_len;
        s->xfer_end = t + xact_ns(s->in_len);
        s->int_raw |= INT_IN_TOKEN_EP1;
    } else if (!usj_out_avail(s) && !fifo8_is_empty(&s->from_host)) {
        uint32_t n = MIN(fifo8_num_used(&s->from_host), ESP32P4_USJ_EP_SIZE);

        s->xfer_busy = true;
        s->xfer_in = false;
        s->xfer_len = n;
        for (uint32_t i = 0; i < n; i++) {
            s->xfer_buf[i] = fifo8_pop(&s->from_host);
        }
        s->xfer_end = t + xact_ns(n);
        qemu_chr_fe_accept_input(&s->chr);
    } else {
        return;
    }
    timer_mod_ns(&s->xfer_timer, s->xfer_end);
    usj_update_irq(s);
}

static void usj_xfer_try(ESP32P4UsjState *s)
{
    usj_xfer_try_at(s, now_ns());
}

/*
 * The transaction under way ends at xfer_end. The next starts then: every
 * change since that could start one (a flush, the host's bytes) came
 * through usj_catch_up(), which ends this one first.
 */
static void usj_xfer_end(ESP32P4UsjState *s)
{
    int64_t end = s->xfer_end;

    s->xfer_busy = false;
    if (s->xfer_in) {
        fifo8_push_all(&s->to_host, s->in_buf, s->xfer_len);
        usj_in_reset(s);
        s->int_raw |= INT_IN_EMPTY;
        usj_to_host_flush(s);
    } else {
        memcpy(s->out_buf, s->xfer_buf, s->xfer_len);
        s->out_len = s->xfer_len;
        s->out_rd = 0;
        s->int_raw |= s->xfer_len ? INT_OUT_RECV_PKT : INT_OUT_EP1_ZERO;
    }
    usj_xfer_try_at(s, end);
    usj_update_irq(s);
}

static void usj_xfer_timer_cb(void *opaque)
{
    ESP32P4UsjState *s = opaque;

    usj_catch_up(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.uart]
 * Complete every transaction, and run the host's frame, that fell due by
 * now, each at its own time, however late the host has run their timers.
 */
static void usj_catch_up(ESP32P4UsjState *s)
{
    int64_t now = now_ns();

    for (;;) {
        bool host = timer_pending(&s->host_timer) && s->host_next <= now;

        if (s->xfer_busy && s->xfer_end <= now &&
            (!host || s->xfer_end <= s->host_next)) {
            timer_del(&s->xfer_timer);
            usj_xfer_end(s);
        } else if (host) {
            timer_del(&s->host_timer);
            esp32_line_run_at(s->host_next, usj_host_step, s);
        } else {
            return;
        }
    }
}

static int usj_chr_can_receive(void *opaque)
{
    ESP32P4UsjState *s = opaque;

    return fifo8_num_free(&s->from_host);
}

static void usj_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    ESP32P4UsjState *s = opaque;

    usj_catch_up(s);
    for (int i = 0; i < size && !fifo8_is_full(&s->from_host); i++) {
        fifo8_push(&s->from_host, buf[i]);
    }
    usj_xfer_try(s);
}

/* The terminal program opened or closed the port: DTR and RTS follow. */
static void usj_chr_event(void *opaque, QEMUChrEvent event)
{
    ESP32P4UsjState *s = opaque;

    if (event == CHR_EVENT_OPENED || event == CHR_EVENT_CLOSED) {
        usj_catch_up(s);
        s->line_state_pending = true;
        usj_host_update(s);
        usj_xfer_try(s);
    }
}

/* ---- Registers ---- */

static uint32_t usj_out_ep1_st(ESP32P4UsjState *s)
{
    /* WR_ADDR counts the packet's two CRC bytes too. */
    uint32_t wr = s->out_len ? s->out_len + 2 : 0;

    return (wr << 2) | ((uint32_t)s->out_rd << 9) |
           ((uint32_t)s->out_len << 16);
}

static uint64_t usj_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4UsjState *s = opaque;
    uint32_t v = 0;

    if (addr >= ESP32P4_USJ_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_usb_serial_jtag: read of "
                      "reserved offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
    usj_catch_up(s);
    switch (addr) {
    case A_EP1:
        if (!usj_out_avail(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_usb_serial_jtag: read of "
                          "the empty CDC-ACM OUT endpoint\n");
            break;
        }
        v = s->out_buf[s->out_rd++];
        if (!usj_out_avail(s)) {
            usj_out_reset(s);
            usj_xfer_try(s);
        }
        break;
    case A_EP1_CONF:
        v = usj_ep1_conf(s);
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->regs[R(A_INT_ENA)];
        break;
    case A_INT_CLR:
    case A_CONFIG_UPDATE:
        v = 0;
        break;
    case A_TEST:
        v = (s->regs[R(addr)] & 0xf) | TEST_RO_IDLE;
        break;
    case A_JFIFO_ST:
        v = (s->regs[R(addr)] & JFIFO_RESETS) | JFIFO_EMPTY;
        break;
    case A_FRAM_NUM:
        v = s->frame;
        break;
    case A_IN_EP0_ST:
    case A_IN_EP2_ST:
    case A_IN_EP3_ST:
        v = 1;
        break;
    case A_IN_EP1_ST:
        v = 1 | ((uint32_t)s->in_len << 2);
        break;
    case A_OUT_EP0_ST:
    case A_OUT_EP2_ST:
        v = 0;
        break;
    case A_OUT_EP1_ST:
        v = usj_out_ep1_st(s);
        break;
    case A_CHIP_RST:
        v = (s->regs[R(addr)] & 0x4) | (s->host_rts ? CHIP_RST_RTS : 0) |
            (s->host_dtr ? CHIP_RST_DTR : 0);
        break;
    case A_SER_AFIFO_CONFIG:
        v = (s->regs[R(addr)] & 0xf) |
            (usj_out_avail(s) ? 0 : AFIFO_OUT_REMPTY);
        break;
    case A_ECO_CELL_CTRL_48:
    case A_ECO_CELL_CTRL_APB:
        /* RDN_RESULT follows RDN_ENA. */
        v = s->regs[R(addr)] | ((s->regs[R(addr)] >> 1) & 1);
        break;
    default:
        v = s->regs[R(addr)];
        break;
    }
    return v;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static void usj_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4UsjState *s = opaque;
    uint32_t v = value;
    unsigned r = R(addr);

    if (addr >= ESP32P4_USJ_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_usb_serial_jtag: write of "
                      "reserved offset 0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    usj_catch_up(s);
    switch (addr) {
    case A_EP1:
        if (s->in_flushed) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_usb_serial_jtag: write "
                          "to the CDC-ACM IN endpoint while it is not free\n");
            break;
        }
        s->in_buf[s->in_len++] = v & 0xff;
        if (s->in_len == ESP32P4_USJ_EP_SIZE) {
            /* The 64th byte flushes the buffer (TRM 54.3.2). */
            s->in_flushed = true;
            usj_xfer_try(s);
        }
        break;
    case A_EP1_CONF:
        if ((v & EP1_WR_DONE) && !s->in_flushed) {
            s->in_flushed = true;
            usj_xfer_try(s);
        }
        break;
    case A_INT_RAW:
        s->int_raw &= ~v;
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_MASK);
        break;
    case A_CONF0:
        s->regs[r] = v & usj_regs[r].rw;
        if (v & CONF0_PHY_SEL) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_usb_serial_jtag: the external "
                          "PHY (PHY_SEL) is not modelled\n");
        }
        if (v & CONF0_JTAG_BRIDGE) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_usb_serial_jtag: the JTAG "
                          "bridge to the GPIO matrix is not modelled\n");
        }
        usj_update_pulls(s);
        usj_host_update(s);
        break;
    case A_TEST:
        s->regs[r] = v & usj_regs[r].rw;
        if (v & TEST_ENABLE) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_usb_serial_jtag: the USB pad "
                          "test mode is not modelled\n");
        }
        break;
    case A_MEM_CONF:
        s->regs[r] = v & usj_regs[r].rw;
        if (v & MEM_PD) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_usb_serial_jtag: powering the "
                          "USB memory down is not modelled\n");
        }
        break;
    case A_SER_AFIFO_CONFIG:
        s->regs[r] = v & usj_regs[r].rw;
        if (v & AFIFO_IN_RESET) {
            usj_in_reset(s);
        }
        if (v & AFIFO_OUT_RESET) {
            usj_out_reset(s);
            usj_xfer_try(s);
        }
        break;
    case A_CONFIG_UPDATE:
    case A_INT_ST:
    case A_FRAM_NUM:
    case A_IN_EP0_ST ... A_OUT_EP2_ST:
    case A_SET_LINE_CODE_W0:
    case A_SET_LINE_CODE_W1:
    case A_BUS_RESET_ST:
        break;
    default:
        s->regs[r] = v & usj_regs[r].rw;
        break;
    }
    usj_update_irq(s);
}

static const MemoryRegionOps usj_ops = {
    .read = usj_read,
    .write = usj_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void usj_usb48_update(void *opaque, ClockEvent event)
{
    ESP32P4UsjState *s = opaque;

    usj_catch_up(s);
    usj_host_update(s);
    usj_xfer_try(s);
}

static void usj_reset_hold(Object *obj, ResetType type)
{
    ESP32P4UsjState *s = ESP32P4_USB_SERIAL_JTAG(obj);

    for (unsigned r = 0; r < ESP32P4_USJ_NREGS; r++) {
        s->regs[r] = usj_regs[r].reset;
    }
    s->regs[R(A_BUS_RESET_ST)] = 1;
    s->int_raw = INT_IN_EMPTY;
    usj_in_reset(s);
    usj_out_reset(s);
    s->frame = 0;
    s->host_dtr = false;
    s->host_rts = false;
    s->line_state_pending = false;
    usj_host_detach(s);
}

/*
 * The controller's reset drops its pull-up for a moment: the host sees
 * it leave and, with the pull-up back, plug in again.
 */
static void usj_reset_exit(Object *obj, ResetType type)
{
    ESP32P4UsjState *s = ESP32P4_USB_SERIAL_JTAG(obj);

    usj_update_pulls(s);
    usj_host_update(s);
    usj_update_irq(s);
}

static void usj_realize(DeviceState *dev, Error **errp)
{
    ESP32P4UsjState *s = ESP32P4_USB_SERIAL_JTAG(dev);

    qemu_chr_fe_set_handlers(&s->chr, usj_chr_can_receive, usj_chr_receive,
                             usj_chr_event, NULL, s, NULL, true);
}

static void usj_init(Object *obj)
{
    ESP32P4UsjState *s = ESP32P4_USB_SERIAL_JTAG(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &usj_ops, s,
                          "esp32p4.usb-serial-jtag", ESP32P4_USJ_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->pull_up, ESP32P4_USJ_PAD_PULL_UP, 2);
    qdev_init_gpio_out_named(dev, &s->pad_enable, ESP32P4_USJ_PAD_ENABLE, 1);
    s->usb48 = qdev_init_clock_in(dev, "usb48", usj_usb48_update, s,
                                  ClockUpdate);
    timer_init_ns(&s->host_timer, QEMU_CLOCK_VIRTUAL, usj_host_timer_cb, s);
    timer_init_ns(&s->xfer_timer, QEMU_CLOCK_VIRTUAL, usj_xfer_timer_cb, s);
    fifo8_create(&s->to_host, HOST_BUF);
    fifo8_create(&s->from_host, HOST_IN_BUF);
}

static void usj_finalize(Object *obj)
{
    ESP32P4UsjState *s = ESP32P4_USB_SERIAL_JTAG(obj);

    fifo8_destroy(&s->to_host);
    fifo8_destroy(&s->from_host);
}

static const VMStateDescription vmstate_esp32p4_usj = {
    .name = TYPE_ESP32P4_USB_SERIAL_JTAG,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(usb48, ESP32P4UsjState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4UsjState, ESP32P4_USJ_NREGS),
        VMSTATE_UINT32(int_raw, ESP32P4UsjState),
        VMSTATE_UINT8_ARRAY(in_buf, ESP32P4UsjState, ESP32P4_USJ_EP_SIZE),
        VMSTATE_UINT8(in_len, ESP32P4UsjState),
        VMSTATE_BOOL(in_flushed, ESP32P4UsjState),
        VMSTATE_UINT8_ARRAY(out_buf, ESP32P4UsjState, ESP32P4_USJ_EP_SIZE),
        VMSTATE_UINT8(out_len, ESP32P4UsjState),
        VMSTATE_UINT8(out_rd, ESP32P4UsjState),
        VMSTATE_TIMER(host_timer, ESP32P4UsjState),
        VMSTATE_UINT8(host_state, ESP32P4UsjState),
        VMSTATE_UINT32(host_step, ESP32P4UsjState),
        VMSTATE_INT64(host_deadline, ESP32P4UsjState),
        VMSTATE_INT64(host_next, ESP32P4UsjState),
        VMSTATE_UINT32(frame, ESP32P4UsjState),
        VMSTATE_BOOL(host_dtr, ESP32P4UsjState),
        VMSTATE_BOOL(host_rts, ESP32P4UsjState),
        VMSTATE_BOOL(line_state_pending, ESP32P4UsjState),
        VMSTATE_TIMER(xfer_timer, ESP32P4UsjState),
        VMSTATE_BOOL(xfer_busy, ESP32P4UsjState),
        VMSTATE_BOOL(xfer_in, ESP32P4UsjState),
        VMSTATE_INT64(xfer_end, ESP32P4UsjState),
        VMSTATE_UINT8_ARRAY(xfer_buf, ESP32P4UsjState, ESP32P4_USJ_EP_SIZE),
        VMSTATE_UINT8(xfer_len, ESP32P4UsjState),
        VMSTATE_FIFO8(to_host, ESP32P4UsjState),
        VMSTATE_FIFO8(from_host, ESP32P4UsjState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property usj_properties[] = {
    DEFINE_PROP_CHR("chardev", ESP32P4UsjState, chr),
};

static void usj_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = usj_realize;
    dc->vmsd = &vmstate_esp32p4_usj;
    device_class_set_props(dc, usj_properties);
    rc->phases.hold = usj_reset_hold;
    rc->phases.exit = usj_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.uart] */
static const TypeInfo usj_info = {
    .name = TYPE_ESP32P4_USB_SERIAL_JTAG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4UsjState),
    .instance_init = usj_init,
    .instance_finalize = usj_finalize,
    .class_init = usj_class_init,
};

static void usj_register_types(void)
{
    type_register_static(&usj_info);
}

type_init(usj_register_types)
