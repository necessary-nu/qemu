/*
 * A Goodix GT911 capacitive touch controller, as the ESP32-P4 board's
 * display modules carry it on the board's I2C wires
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: Goodix's GT911 datasheet (Rev.09) and GT911 Programming
 * Guide (Rev.10, firmware V1040 or later); ESP-IDF's
 * esp_lcd_touch_gt911 and Linux's goodix.c, which drive it.
 *
 * [spec:nuos:req:emu.esp32p4.touch]
 * The controller (-device gt911, or a display module's own):
 *
 *   - Its address. It answers 0x5D (0xBA/0xBB) or 0x14 (0x28/0x29) as
 *     its INT pin's level selects at RSTB's rise (the guide's 4.2: high
 *     0x14, low 0x5D): the "int" input's, from power-on "int-level"'s. It
 *     answers only its own address, and only from 5 ms after RSTB rises
 *     (the datasheet's T4, the host holding INT that long for the
 *     selection), powered (VDD, "vdd") and out of reset (RSTB, "rstb"):
 *     otherwise nothing acknowledges.
 *   - Its registers, through a 16-bit address, high byte first, that
 *     advances with each byte read or written (guide 2.1, 2.2); a read
 *     takes the address the last write left, after a repeated START or a
 *     STOP. 8040h to 8046h, the commands (0 reads coordinates, 5 sleep;
 *     the others, charge, gesture and HotKnot modes and ESD, are kept
 *     without effect, logged); 8047h to 80FEh the configuration, 80FFh
 *     its checksum (the two's complement of the bytes' sum), 8100h
 *     Config_Fresh; 8140h to 814Dh the product ID "911", the firmware
 *     version ("fw-version"), the X and Y resolution (the configuration's
 *     output maximum), the vendor ID ("vendor-id"); 814Eh the buffer
 *     status; 814Fh on, five touch points of eight bytes (track ID, X, Y,
 *     size, little-endian); 81A8h and 81A9h GT911_Status 00h, touch
 *     detection. Other addresses read 0; a write to a read-only one is
 *     ignored (logged).
 *   - Its configuration. Written registers take effect once Config_Fresh
 *     is written 1 with the checksum right, and Config_Version later than
 *     the one in effect, the same with other contents, or 0, which makes
 *     it 'A' (guide 3.2, 4.4); the chip then clears Config_Fresh. What
 *     takes effect is fixed in its flash ("Stationary Configuration",
 *     datasheet 8.4): it survives RSTB and VDD, the registers reloading it
 *     at each start. In effect from it: the X and Y output maxima, the
 *     touch number (1 to 5), Module_Switch1's X2Y, X2X and Y2Y (against
 *     the module's own), Refresh_Rate, x_threshold and y_threshold, and
 *     Low_Power_Control.
 *   - Touches. The QEMU pointer (absolute, the left button touching) and
 *     QEMU's multi-touch slots 0 to 4 on the panel's console are fingers
 *     on the panel, each mapped across it to 0 to the output maximum
 *     less one, taking the lowest free track ID as it comes down, with a
 *     fixed size of 24. The chip scans every 5 + N ms (Refresh_Rate), or
 *     40 ms after Low_Power_Control's seconds without a touch (Green
 *     mode, guide 6.1): a scan with fingers down reports them, in track
 *     ID order, up to the touch number (buffer status 80h | count), when
 *     a finger has moved more than its threshold since the last report
 *     (every scan with thresholds 0), the count has changed or none was
 *     reported; the first scan after the last finger lifts reports none
 *     (80h). While the host has not written 814Eh's buffer status back to
 *     0, scans report nothing new (guide 5: the chip pulses INT again
 *     instead). In sleep (command 5) it scans not, until RSTB.
 *
 * Not modelled: its INT output (the board's modules do not take it to
 * the connector), touch keys, large-area and palm detection, the
 * de-jitter counts and filters, gestures, HotKnot, the charge mode, ESD
 * checking and the INT wake-up from sleep.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/input/esp32p4_gt911.h"
#include "migration/vmstate.h"

#define ADDR_INT_LOW        0x5d
#define ADDR_INT_HIGH       0x14
#define BOOT_NS             (5 * SCALE_MS)
#define GREEN_SCAN_MS       40
#define POINT_SIZE          24

#define R_COMMAND           0x8040
#define R_COMMAND_CHECK     0x8046
#define R_CHKSUM            0x80ff
#define R_FRESH             0x8100
#define R_PRODUCT_ID        0x8140
#define R_STATUS            0x814e
#define R_POINTS            0x814f

/* Configuration offsets from 8047h. */
#define C_VERSION           0x00
#define C_XMAX              0x01
#define C_YMAX              0x03
#define C_TOUCHES           0x05
#define C_SWITCH1           0x06
#define C_LOW_POWER         0x0e
#define C_REFRESH           0x0f
#define C_XTHRESH           0x10
#define C_YTHRESH           0x11
#define C_CHKSUM            (R_CHKSUM - GT911_CFG_BASE)
#define C_FRESH             (R_FRESH - GT911_CFG_BASE)

#define SW1_X2Y             BIT(3)
#define SW1_X2X             BIT(6)
#define SW1_Y2Y             BIT(7)

#define STATUS_READY        0x80

/*
 * The module's stored configuration is not published; the model's has
 * the panel's 720 x 1280 as its output maximum, five touches, INT on the
 * falling edge (Waveshare's BSPs take INT as falling-edge), its axes as
 * the panel's, a 10 ms report period, thresholds 0 and 3 s to Green
 * mode, the rest 0.
 */
#define SW1_DEFAULT         0x05

static void cfg_seal(uint8_t *c)
{
    uint8_t sum = 0;

    for (int i = 0; i < C_CHKSUM; i++) {
        sum += c[i];
    }
    c[C_CHKSUM] = -sum;
}

static void cfg_default(uint8_t *c)
{
    memset(c, 0, GT911_CFG_LEN);
    c[C_VERSION] = 'A';
    stw_le_p(c + C_XMAX, 720);
    stw_le_p(c + C_YMAX, 1280);
    c[C_TOUCHES] = GT911_POINTS;
    c[C_SWITCH1] = SW1_DEFAULT;
    c[C_LOW_POWER] = 3;
    c[C_REFRESH] = 5;
    cfg_seal(c);
}

static int64_t gt911_now(void)
{
    return esp32_line_time();
}

static unsigned touch_number(GT911State *s)
{
    return MIN(MAX(s->flash[C_TOUCHES] & 0xf, 1), GT911_POINTS);
}

/* ---- Touches and scans ---- */

static bool gt911_touched(GT911State *s)
{
    for (int i = 0; i < ARRAY_SIZE(s->contact); i++) {
        if (s->contact[i].down) {
            return true;
        }
    }
    return false;
}

static int64_t scan_period(GT911State *s, int64_t now)
{
    int64_t idle = (int64_t)(s->flash[C_LOW_POWER] & 0xf) *
                   NANOSECONDS_PER_SECOND;

    if (!gt911_touched(s) && now - s->touch_ns >= idle) {
        return GREEN_SCAN_MS * SCALE_MS;
    }
    return (5 + (s->flash[C_REFRESH] & 0xf)) * SCALE_MS;
}

/* Contact @c where the chip reports it: X and Y in its output range. */
static void contact_coords(GT911State *s, const GT911Contact *c,
                           uint16_t *xo, uint16_t *yo)
{
    uint8_t d = s->flash[C_SWITCH1] ^ SW1_DEFAULT;
    uint32_t u = c->x, v = c->y;

    if (d & SW1_X2Y) {
        uint32_t t = u;

        u = v;
        v = t;
    }
    if (d & SW1_X2X) {
        u = INPUT_EVENT_ABS_MAX - u;
    }
    if (d & SW1_Y2Y) {
        v = INPUT_EVENT_ABS_MAX - v;
    }
    *xo = (u * lduw_le_p(s->flash + C_XMAX)) >> 15;
    *yo = (v * lduw_le_p(s->flash + C_YMAX)) >> 15;
}

/* [spec:nuos:req:emu.esp32p4.touch] A scan at @now, and what it reports. */
static void gt911_scan_once(GT911State *s, int64_t now)
{
    const GT911Contact *c[ARRAY_SIZE(s->contact)];
    uint16_t xo[GT911_POINTS], yo[GT911_POINTS];
    unsigned n = 0, max = touch_number(s);
    bool report;

    for (unsigned id = 0; id < ARRAY_SIZE(s->contact) && n < max; id++) {
        for (unsigned i = 0; i < ARRAY_SIZE(s->contact); i++) {
            if (s->contact[i].down && s->contact[i].track == id) {
                c[n++] = &s->contact[i];
                break;
            }
        }
    }
    if (n) {
        s->touch_ns = now;
    }
    if (s->status & STATUS_READY) {
        return;
    }
    if (!n) {
        if (s->reported) {
            s->status = STATUS_READY;
            s->reported = false;
            s->last_n = 0;
        }
        return;
    }
    report = !s->reported || n != s->last_n;
    for (unsigned i = 0; i < n; i++) {
        contact_coords(s, c[i], &xo[i], &yo[i]);
        if (abs(xo[i] - s->last_x[i]) > s->flash[C_XTHRESH] ||
            abs(yo[i] - s->last_y[i]) > s->flash[C_YTHRESH] ||
            !s->flash[C_XTHRESH] || !s->flash[C_YTHRESH]) {
            report = true;
        }
    }
    if (!report) {
        return;
    }
    memset(s->point, 0, sizeof(s->point));
    for (unsigned i = 0; i < n; i++) {
        uint8_t *p = s->point + 8 * i;

        p[0] = c[i]->track;
        stw_le_p(p + 1, xo[i]);
        stw_le_p(p + 3, yo[i]);
        stw_le_p(p + 5, POINT_SIZE);
        s->last_x[i] = xo[i];
        s->last_y[i] = yo[i];
    }
    s->status = STATUS_READY | n;
    s->reported = true;
    s->last_n = n;
}

/* Scan on while a finger is down or its lifting is still to report. */
static void gt911_kick(GT911State *s, int64_t now)
{
    if (!s->running || s->sleeping ||
        (!gt911_touched(s) && !s->reported)) {
        timer_del(s->scan);
        return;
    }
    if (!timer_pending(s->scan)) {
        s->scan_due_ns = now + scan_period(s, now);
        timer_mod(s->scan, s->scan_due_ns);
    }
}

static void gt911_scan_run(void *opaque)
{
    GT911State *s = opaque;
    int64_t t = s->scan_due_ns;

    gt911_scan_once(s, t);
    if (s->running && !s->sleeping && (gt911_touched(s) || s->reported)) {
        s->scan_due_ns = t + scan_period(s, t);
        timer_mod(s->scan, s->scan_due_ns);
    }
}

/* Each scan at its own time, however late its QEMU timer runs. */
static void gt911_scan_cb(void *opaque)
{
    GT911State *s = opaque;

    esp32_line_run_at(s->scan_due_ns, gt911_scan_run, s);
}

static void contact_set(GT911State *s, unsigned i, bool down)
{
    GT911Contact *c = &s->contact[i];

    if (down && !c->down) {
        uint8_t used = 0;

        for (unsigned k = 0; k < ARRAY_SIZE(s->contact); k++) {
            if (s->contact[k].down) {
                used |= BIT(s->contact[k].track);
            }
        }
        c->track = ctz32(~used);
    }
    c->down = down;
}

/* [spec:nuos:req:emu.esp32p4.touch] */
static void gt911_event(DeviceState *dev, QemuConsole *src,
                        QemuInputEvent *evt)
{
    GT911State *s = GT911(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS:
        if (evt->abs.axis == INPUT_AXIS_X) {
            s->ptr_x = evt->abs.value;
        } else if (evt->abs.axis == INPUT_AXIS_Y) {
            s->ptr_y = evt->abs.value;
        }
        if (s->contact[0].down) {
            s->contact[0].x = s->ptr_x;
            s->contact[0].y = s->ptr_y;
        }
        break;
    case INPUT_EVENT_KIND_BTN:
        if (evt->btn.button == INPUT_BUTTON_LEFT) {
            contact_set(s, 0, evt->btn.down);
            s->contact[0].x = s->ptr_x;
            s->contact[0].y = s->ptr_y;
        }
        break;
    case INPUT_EVENT_KIND_MTT: {
        GT911Contact *c;

        if (evt->mtt.slot < 0 || evt->mtt.slot >= GT911_POINTS) {
            break;
        }
        c = &s->contact[1 + evt->mtt.slot];
        switch (evt->mtt.type) {
        case INPUT_MULTI_TOUCH_TYPE_BEGIN:
            contact_set(s, 1 + evt->mtt.slot, true);
            break;
        case INPUT_MULTI_TOUCH_TYPE_END:
        case INPUT_MULTI_TOUCH_TYPE_CANCEL:
            contact_set(s, 1 + evt->mtt.slot, false);
            break;
        case INPUT_MULTI_TOUCH_TYPE_DATA:
            if (evt->mtt.axis == INPUT_AXIS_X) {
                c->x = evt->mtt.value;
            } else if (evt->mtt.axis == INPUT_AXIS_Y) {
                c->y = evt->mtt.value;
            }
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void gt911_sync(DeviceState *dev)
{
    GT911State *s = GT911(dev);

    gt911_kick(s, gt911_now());
}

static const QemuInputHandler gt911_input = {
    .name = "GT911 touch",
    .mask = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS |
            INPUT_EVENT_MASK_MTT,
    .event = gt911_event,
    .sync = gt911_sync,
};

void gt911_bind_console(GT911State *s, DeviceState *display)
{
    if (display->id) {
        qemu_input_handler_bind(s->input, display->id, 0, &error_abort);
    } else {
        qemu_input_handler_activate(s->input);
    }
}

/* ---- Power, reset and the address ---- */

/* [spec:nuos:req:emu.esp32p4.touch] RSTB rising, powered: it starts. */
static void gt911_start(GT911State *s, int64_t now)
{
    s->running = true;
    s->sleeping = false;
    s->addr = s->int_level ? ADDR_INT_HIGH : ADDR_INT_LOW;
    I2C_SLAVE(s)->address = s->addr;
    s->ready_ns = now + BOOT_NS;
    memcpy(s->cfg, s->flash, sizeof(s->cfg));
    memset(s->cmd, 0, sizeof(s->cmd));
    s->status = 0;
    memset(s->point, 0, sizeof(s->point));
    s->ptr = 0;
    s->addr_bytes = 0;
    s->reported = false;
    s->last_n = 0;
    s->touch_ns = now;
}

static void gt911_power_update(GT911State *s)
{
    bool run = s->vdd && s->rstb;
    int64_t now = gt911_now();

    if (!s->powered || run == s->running) {
        return;
    }
    if (run) {
        gt911_start(s, now);
        gt911_kick(s, now);
    } else {
        s->running = false;
        timer_del(s->scan);
    }
}

static void gt911_vdd(void *opaque, int n, int level)
{
    GT911State *s = opaque;

    s->vdd = level;
    gt911_power_update(s);
}

static void gt911_rstb(void *opaque, int n, int level)
{
    GT911State *s = opaque;

    s->rstb = level;
    gt911_power_update(s);
}

static void gt911_int(void *opaque, int n, int level)
{
    GT911State *s = opaque;

    s->int_level = level;
}

/* ---- The registers ---- */

/* [spec:nuos:req:emu.esp32p4.touch] Config_Fresh written 1. */
static void gt911_take_config(GT911State *s)
{
    uint8_t sum = 0, v = s->cfg[C_VERSION], cur = s->flash[C_VERSION];

    s->cfg[C_FRESH] = 0;
    for (int i = 0; i <= C_CHKSUM; i++) {
        sum += s->cfg[i];
    }
    if (sum) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: configuration checksum %02xh is "
                      "wrong: not taken\n", TYPE_GT911, s->cfg[C_CHKSUM]);
        return;
    }
    if (v && (v < cur || (v == cur && !memcmp(s->cfg, s->flash, C_FRESH)))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: configuration version %02xh not "
                      "later than %02xh: not taken\n", TYPE_GT911, v, cur);
        return;
    }
    if (!v) {
        s->cfg[C_VERSION] = 'A';
        cfg_seal(s->cfg);
    }
    memcpy(s->flash, s->cfg, sizeof(s->flash));
}

static uint8_t gt911_read(GT911State *s, uint16_t a)
{
    static const char product[4] = "911";

    switch (a) {
    case R_COMMAND ... R_COMMAND_CHECK:
        return s->cmd[a - R_COMMAND];
    case GT911_CFG_BASE ... R_FRESH:
        return s->cfg[a - GT911_CFG_BASE];
    case R_PRODUCT_ID ... R_PRODUCT_ID + 3:
        return product[a - R_PRODUCT_ID];
    case 0x8144:
    case 0x8145:
        return s->fw_version >> (8 * (a - 0x8144));
    case 0x8146:
    case 0x8147:
        return s->flash[C_XMAX + a - 0x8146];
    case 0x8148:
    case 0x8149:
        return s->flash[C_YMAX + a - 0x8148];
    case 0x814a:
        return s->vendor_id;
    case R_STATUS:
        return s->status;
    case R_POINTS ... R_POINTS + GT911_POINTS * 8 - 1:
        return s->point[a - R_POINTS];
    default:
        return 0;
    }
}

static void gt911_command(GT911State *s, uint8_t v)
{
    switch (v) {
    case 0x00:
        break;
    case 0x05:
        s->sleeping = true;
        timer_del(s->scan);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: command %02xh is kept without "
                      "effect\n", TYPE_GT911, v);
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.touch] */
static void gt911_write(GT911State *s, uint16_t a, uint8_t v)
{
    switch (a) {
    case R_COMMAND:
        s->cmd[0] = v;
        gt911_command(s, v);
        break;
    case R_COMMAND + 1 ... R_COMMAND_CHECK:
        s->cmd[a - R_COMMAND] = v;
        break;
    case GT911_CFG_BASE ... R_CHKSUM:
        s->cfg[a - GT911_CFG_BASE] = v;
        break;
    case R_FRESH:
        s->cfg[C_FRESH] = v;
        if (v == 1) {
            gt911_take_config(s);
        }
        break;
    case R_STATUS:
        s->status = v;
        gt911_kick(s, gt911_now());
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %02xh to %04xh, which "
                      "is read only or reserved\n", TYPE_GT911, v, a);
        break;
    }
}

static int gt911_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    GT911State *s = GT911(i2c);

    if (event == I2C_START_SEND) {
        s->addr_bytes = 0;
    }
    return 0;
}

static int gt911_send(I2CSlave *i2c, uint8_t data)
{
    GT911State *s = GT911(i2c);

    if (s->addr_bytes < 2) {
        s->ptr = s->addr_bytes++ ? (s->ptr & 0xff00) | data
                                 : (uint16_t)data << 8;
        return 0;
    }
    gt911_write(s, s->ptr++, data);
    return 0;
}

static uint8_t gt911_recv(I2CSlave *i2c)
{
    GT911State *s = GT911(i2c);

    return gt911_read(s, s->ptr++);
}

/*
 * [spec:nuos:req:emu.esp32p4.touch]
 * Its own address only, running and past its start.
 */
static bool gt911_match(I2CSlave *candidate, uint8_t address, bool broadcast,
                        I2CNodeList *current_devs)
{
    GT911State *s = GT911(candidate);
    I2CNode *node;

    if (!s->running || candidate->address != address) {
        return false;
    }
    if (gt911_now() < s->ready_ns) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: addressed %" PRId64 " us after "
                      "RSTB rose, before its 5 ms: no acknowledge\n",
                      TYPE_GT911,
                      (gt911_now() - (s->ready_ns - BOOT_NS)) / SCALE_US);
        return false;
    }
    node = g_new(struct I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

/* ---- The device ---- */

/* The module's own power-on, which the chip's resets do not reach. */
static void gt911_reset_hold(Object *obj, ResetType type)
{
    GT911State *s = GT911(obj);

    if (s->powered) {
        return;
    }
    s->powered = true;
    s->int_level = s->int_level_reset;
    cfg_default(s->flash);
    s->running = false;
    gt911_power_update(s);
}

static void gt911_realize(DeviceState *dev, Error **errp)
{
    GT911State *s = GT911(dev);

    s->input = qemu_input_handler_register(dev, &gt911_input);
    s->scan = timer_new_ns(QEMU_CLOCK_VIRTUAL, gt911_scan_cb, s);
}

static void gt911_unrealize(DeviceState *dev)
{
    GT911State *s = GT911(dev);

    qemu_input_handler_unregister(s->input);
    timer_free(s->scan);
}

static void gt911_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);

    qdev_init_gpio_in_named(dev, gt911_vdd, GT911_VDD, 1);
    qdev_init_gpio_in_named(dev, gt911_rstb, GT911_RSTB, 1);
    qdev_init_gpio_in_named(dev, gt911_int, GT911_INT, 1);
}

static const VMStateDescription vmstate_contact = {
    .name = "gt911-contact",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(down, GT911Contact),
        VMSTATE_UINT8(track, GT911Contact),
        VMSTATE_UINT16(x, GT911Contact),
        VMSTATE_UINT16(y, GT911Contact),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_gt911 = {
    .name = TYPE_GT911,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, GT911State),
        VMSTATE_TIMER_PTR(scan, GT911State),
        VMSTATE_BOOL(powered, GT911State),
        VMSTATE_BOOL(vdd, GT911State),
        VMSTATE_BOOL(rstb, GT911State),
        VMSTATE_BOOL(int_level, GT911State),
        VMSTATE_BOOL(running, GT911State),
        VMSTATE_BOOL(sleeping, GT911State),
        VMSTATE_UINT8(addr, GT911State),
        VMSTATE_INT64(ready_ns, GT911State),
        VMSTATE_INT64(touch_ns, GT911State),
        VMSTATE_INT64(scan_due_ns, GT911State),
        VMSTATE_UINT8_ARRAY(cfg, GT911State, GT911_CFG_LEN),
        VMSTATE_UINT8_ARRAY(flash, GT911State, GT911_CFG_LEN),
        VMSTATE_UINT8_ARRAY(cmd, GT911State, 7),
        VMSTATE_UINT8(status, GT911State),
        VMSTATE_UINT8_ARRAY(point, GT911State, GT911_POINTS * 8),
        VMSTATE_UINT16(ptr, GT911State),
        VMSTATE_UINT8(addr_bytes, GT911State),
        VMSTATE_STRUCT_ARRAY(contact, GT911State, 1 + GT911_POINTS, 1,
                             vmstate_contact, GT911Contact),
        VMSTATE_UINT16(ptr_x, GT911State),
        VMSTATE_UINT16(ptr_y, GT911State),
        VMSTATE_BOOL(reported, GT911State),
        VMSTATE_UINT8(last_n, GT911State),
        VMSTATE_UINT16_ARRAY(last_x, GT911State, GT911_POINTS),
        VMSTATE_UINT16_ARRAY(last_y, GT911State, GT911_POINTS),
        VMSTATE_END_OF_LIST()
    },
};

/* The values a module's chip leaves unpublished: "911" firmware 1060h. */
static const Property gt911_properties[] = {
    DEFINE_PROP_BOOL("int-level", GT911State, int_level_reset, false),
    DEFINE_PROP_UINT16("fw-version", GT911State, fw_version, 0x1060),
    DEFINE_PROP_UINT8("vendor-id", GT911State, vendor_id, 0),
};

static void gt911_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Goodix GT911 capacitive touch controller";
    dc->realize = gt911_realize;
    dc->unrealize = gt911_unrealize;
    dc->vmsd = &vmstate_gt911;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
    rc->phases.hold = gt911_reset_hold;
    sc->event = gt911_i2c_event;
    sc->send = gt911_send;
    sc->recv = gt911_recv;
    sc->match_and_add = gt911_match;
    device_class_set_props(dc, gt911_properties);
}

static const TypeInfo gt911_types[] = {
    {
        .name = TYPE_GT911,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(GT911State),
        .instance_init = gt911_init,
        .class_init = gt911_class_init,
    },
};

DEFINE_TYPES(gt911_types)
