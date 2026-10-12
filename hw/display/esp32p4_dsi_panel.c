/*
 * ESP32-P4 board: a command-level MIPI DSI panel on the DSI host's link
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: MIPI DSI 1.1 (the packets, error reporting, read responses)
 * and MIPI DCS's command set; the panels ESP-IDF's MIPI DSI examples drive,
 * EK79007 (1024 x 600, the ESP32-P4 Function EV board's) and ILI9881C
 * (800 x 1280), through esp-iot-solution's esp_lcd_ek79007 and
 * esp_lcd_ili9881c.
 *
 * The board's display: -device esp32p4-dsi-panel,width=W,height=H (default
 * 1024 x 600, an EK79007's), on the DSI host's bus, shown on a QEMU
 * graphic console of its size. It takes:
 *
 *   - Video. A V Sync Start starts a frame; each H Sync Start (or V Sync
 *     End) a line; the line's pixel packets (0x3E RGB888, 0x0E RGB565,
 *     0x1E packed and 0x2E loosely packed RGB666, MIPI DSI 1.1 8.8.20 to
 *     8.8.23) fill its frame memory's row from the left, rows below the
 *     panel's height and pixels past its width being dropped. HS packets
 *     must come on the data lanes the board wires ("lanes", default 2):
 *     on another count the panel cannot read them (it records an ECC
 *     multi-bit error and drops them). Null and blanking packets, H Sync
 *     End and EoTp only take time.
 *   - DCS commands (short and long writes, reads): nop, soft_reset,
 *     enter/exit_sleep_mode, enter_partial/normal_mode, enter/exit_invert
 *     _mode, set_display_on/off, set_column/page_address,
 *     write/read_memory_start/continue (the frame memory in
 *     set_pixel_format's DBI format, 16, 18 or 24 bits a pixel, laid out
 *     as the DSI pixel packets are: 0x0E's, 0x2E's, 0x3E's), set_tear_on/
 *     off (kept; TE is not signalled), set_address_mode (MADCTL: MY, MX
 *     and BGR as DCS defines them, applied to what is shown),
 *     enter/exit_idle_mode (8 colours), set_pixel_format, the brightness
 *     and CABC writes (kept), and the reads get_display_id,
 *     get_error_count_on_DSI, get_power_mode, get_address_mode,
 *     get_pixel_format, get_display_mode, get_signal_mode,
 *     get_diagnostic_result, get_scanline, get_display_brightness,
 *     get_control_display, get_power_save and the ID reads 0xDA to 0xDC
 *     ("id1".."id3"). Color Mode On/Off, Shut Down and Turn On Peripheral
 *     act as idle mode and display off/on.
 *   - Commands it does not know. Every other command code, which on the
 *     panels these model are manufacturer commands (EK79007's 0x80 to 0x86
 *     and 0xB2 lane setting, ILI9881C's page select 0xFF and its pages'
 *     registers), and the DCS commands not listed above (partial area,
 *     scrolling, gamma, tear scanline), is accepted without effect on the
 *     picture: its last parameters (up to 16 bytes) are kept by command
 *     code, without paging, and a DCS or generic read of that code returns
 *     them (a zero byte if none were written). Generic writes are taken
 *     the same way, their first byte the code. A command code is never
 *     an error: DSI reports only packets it cannot take.
 *   - Errors (MIPI DSI 1.1 8.9.5), reported at the next bus turnaround
 *     and then cleared: a data type it does not take (11), a virtual
 *     channel other than "vc" (12; the packet is ignored), a read request
 *     without the turnaround it needs (15), packets on the wrong lanes
 *     (9). After a turnaround it sends read data for a read (DCS short
 *     read responses of 1 or 2 bytes, a DCS long read response beyond,
 *     generic likewise, each up to Set Maximum Return Packet Size's
 *     count, 1 from reset), an Acknowledge and Error Report if it has
 *     errors (alone after a read it cannot answer), else the Acknowledge
 *     trigger. The Reset-Trigger resets it.
 *
 * From reset the panel is in sleep, its display on ("display-on", as the
 * EK79007, which ESP-IDF's driver never sends DISPON; off for one that
 * needs it, as the ILI9881C), MADCTL 0, 24-bit pixels, its frame memory
 * black. It shows its frame memory while out of sleep with its display on;
 * else black. It takes effect at once: the timing a panel's datasheet asks
 * between commands (5 ms after exit_sleep_mode, 120 ms before
 * enter_sleep_mode) is not checked. The chip's resets do not reach it:
 * only the board's power-on, and the Reset-Trigger and soft_reset, reset
 * it.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/esp32p4_mipi_dsi.h"
#include "hw/misc/esp32p4_image_source.h"
#include "migration/vmstate.h"
#include "ui/console.h"

/* Error report bits (MIPI DSI 1.1 table 20). */
#define ERR_ECC_MULTI       BIT(9)
#define ERR_DT              BIT(11)
#define ERR_VC              BIT(12)
#define ERR_PROTOCOL        BIT(15)

static void panel_dirty(ESP32P4DsiPanelState *s)
{
    s->dirty = true;
}

static void panel_reset_state(ESP32P4DsiPanelState *s)
{
    s->sleep_in = true;
    s->sleep_out_ns = 0;
    s->display_on = s->display_on_reset;
    s->inversion = false;
    s->idle = false;
    s->partial = false;
    s->shut_down = false;
    s->madctl = 0;
    s->colmod = 0x77;
    s->brightness = 0;
    s->ctrl_display = 0;
    s->cabc = 0;
    s->tear = 0;
    s->sc = 0;
    s->ec = s->width - 1;
    s->sp = 0;
    s->ep = s->height - 1;
    s->wx = s->wy = 0;
    s->rx = s->ry = 0;
    s->max_return = 1;
    s->errors = 0;
    s->video_row = 0;
    s->video_col = 0;
    s->in_frame = false;
    s->read_pending = false;
    s->error_count = 0;
    memset(s->vreg, 0, sizeof(s->vreg));
    memset(s->vreg_len, 0, sizeof(s->vreg_len));
    panel_dirty(s);
}

/* ---- Pixels ---- */

static uint32_t widen5(uint32_t c)
{
    return c << 3 | c >> 2;
}

static uint32_t widen6(uint32_t c)
{
    return c << 2 | c >> 4;
}

/*
 * Decode up to @n pixels of DSI pixel data type @dt from @b (@len bytes)
 * into @out, 0x00RRGGBB; how many it held.
 */
static uint32_t decode_pixels(uint8_t dt, const uint8_t *b, uint32_t len,
                              uint32_t *out, uint32_t n)
{
    uint32_t i;

    for (i = 0; i < n; i++) {
        uint32_t r, g, bl;

        switch (dt) {
        case 0x3e:
            if (3 * i + 3 > len) {
                return i;
            }
            r = b[3 * i];
            g = b[3 * i + 1];
            bl = b[3 * i + 2];
            break;
        case 0x0e: {
            uint16_t v;

            if (2 * i + 2 > len) {
                return i;
            }
            v = b[2 * i] | b[2 * i + 1] << 8;
            r = widen5(v & 0x1f);
            g = widen6((v >> 5) & 0x3f);
            bl = widen5(v >> 11);
            break;
        }
        case 0x2e:
            if (3 * i + 3 > len) {
                return i;
            }
            r = widen6(b[3 * i] >> 2);
            g = widen6(b[3 * i + 1] >> 2);
            bl = widen6(b[3 * i + 2] >> 2);
            break;
        case 0x1e: {
            uint32_t bit = 18 * i, v = 0;

            if (DIV_ROUND_UP(bit + 18, 8) > len) {
                return i;
            }
            for (int k = 0; k < 18; k++, bit++) {
                if (b[bit / 8] & BIT(bit % 8)) {
                    v |= BIT(k);
                }
            }
            r = widen6(v & 0x3f);
            g = widen6((v >> 6) & 0x3f);
            bl = widen6(v >> 12);
            break;
        }
        default:
            return 0;
        }
        out[i] = r << 16 | g << 8 | bl;
    }
    return i;
}

/* COLMOD's DBI format as a DSI pixel data type: 16, 18 or 24 bits. */
static uint8_t colmod_dt(ESP32P4DsiPanelState *s)
{
    switch (s->colmod & 7) {
    case 5:
        return 0x0e;
    case 6:
        return 0x2e;
    default:
        return 0x3e;
    }
}

static uint32_t encode_pixel(uint8_t dt, uint32_t px, uint8_t *out)
{
    uint32_t r = px >> 16 & 0xff, g = px >> 8 & 0xff, b = px & 0xff;

    switch (dt) {
    case 0x0e: {
        uint16_t v = r >> 3 | (g >> 2) << 5 | (b >> 3) << 11;

        out[0] = v;
        out[1] = v >> 8;
        return 2;
    }
    case 0x2e:
        out[0] = r & 0xfc;
        out[1] = g & 0xfc;
        out[2] = b & 0xfc;
        return 3;
    default:
        out[0] = r;
        out[1] = g;
        out[2] = b;
        return 3;
    }
}

/* ---- Frame memory, as DCS's write and read memory commands reach it ---- */

static void mem_advance(ESP32P4DsiPanelState *s, uint32_t *x, uint32_t *y)
{
    if (++*x > s->ec) {
        *x = s->sc;
        if (++*y > s->ep) {
            *y = s->sp;
        }
    }
}

static void mem_write(ESP32P4DsiPanelState *s, const uint8_t *b,
                      uint32_t len)
{
    uint8_t dt = colmod_dt(s);
    uint32_t bpp = dt == 0x0e ? 2 : 3;

    for (uint32_t i = 0; i + bpp <= len; i += bpp) {
        uint32_t px;

        decode_pixels(dt, b + i, bpp, &px, 1);
        if (s->wx < s->width && s->wy < s->height) {
            s->fb[s->wy * s->width + s->wx] = px;
        }
        mem_advance(s, &s->wx, &s->wy);
    }
    panel_dirty(s);
}

static uint32_t mem_read(ESP32P4DsiPanelState *s, uint8_t *out, uint32_t max)
{
    uint8_t dt = colmod_dt(s), tmp[3];
    uint32_t len = 0;

    while (len < max) {
        uint32_t px = s->rx < s->width && s->ry < s->height ?
                      s->fb[s->ry * s->width + s->rx] : 0;
        uint32_t n = encode_pixel(dt, px, tmp);

        if (len + n > max) {
            break;
        }
        memcpy(out + len, tmp, n);
        len += n;
        mem_advance(s, &s->rx, &s->ry);
    }
    return len;
}

/* ---- Commands ---- */

static bool dcs_known(uint8_t cmd)
{
    switch (cmd) {
    case 0x00: case 0x01: case 0x10: case 0x11: case 0x12: case 0x13:
    case 0x20: case 0x21: case 0x28: case 0x29: case 0x2a: case 0x2b:
    case 0x2c: case 0x3c: case 0x34: case 0x35: case 0x36: case 0x38:
    case 0x39: case 0x3a: case 0x51: case 0x53: case 0x55:
        return true;
    default:
        return false;
    }
}

static void vreg_store(ESP32P4DsiPanelState *s, uint8_t cmd,
                       const uint8_t *p, uint32_t n)
{
    n = MIN(n, ESP32P4_DSI_PANEL_VREG_LEN);
    memcpy(s->vreg[cmd], p, n);
    s->vreg_len[cmd] = n;
}

static void dcs_write(ESP32P4DsiPanelState *s, uint8_t cmd, const uint8_t *p,
                      uint32_t n, int64_t when)
{
    if (!dcs_known(cmd)) {
        vreg_store(s, cmd, p, n);
        return;
    }
    switch (cmd) {
    case 0x01:
        panel_reset_state(s);
        break;
    case 0x10:
        s->sleep_in = true;
        break;
    case 0x11:
        s->sleep_in = false;
        s->sleep_out_ns = when;
        break;
    case 0x12:
        s->partial = true;
        qemu_log_mask(LOG_UNIMP, "%s: partial mode shows the whole frame\n",
                      TYPE_ESP32P4_DSI_PANEL);
        break;
    case 0x13:
        s->partial = false;
        break;
    case 0x20:
    case 0x21:
        s->inversion = cmd == 0x21;
        break;
    case 0x28:
    case 0x29:
        s->display_on = cmd == 0x29;
        break;
    case 0x2a:
    case 0x2b:
        if (n >= 4) {
            uint16_t a = p[0] << 8 | p[1], e = p[2] << 8 | p[3];

            if (cmd == 0x2a) {
                s->sc = a;
                s->ec = MAX(e, a);
            } else {
                s->sp = a;
                s->ep = MAX(e, a);
            }
        }
        break;
    case 0x2c:
        s->wx = s->sc;
        s->wy = s->sp;
        mem_write(s, p, n);
        break;
    case 0x3c:
        mem_write(s, p, n);
        break;
    case 0x34:
        s->tear = 0;
        break;
    case 0x35:
        s->tear = 0x80 | (n ? (p[0] & 1) << 6 : 0);
        break;
    case 0x36:
        if (n) {
            s->madctl = p[0];
        }
        break;
    case 0x38:
    case 0x39:
        s->idle = cmd == 0x39;
        break;
    case 0x3a:
        if (n) {
            s->colmod = p[0];
        }
        break;
    case 0x51:
        if (n) {
            s->brightness = p[0];
        }
        break;
    case 0x53:
        if (n) {
            s->ctrl_display = p[0];
        }
        break;
    case 0x55:
        if (n) {
            s->cabc = p[0];
        }
        break;
    default:
        break;
    }
    panel_dirty(s);
}

/* The bytes a read of @cmd gives, up to @max; DCS's reads and the rest. */
static uint32_t dcs_read(ESP32P4DsiPanelState *s, uint8_t cmd, uint8_t *out,
                         uint32_t max, bool generic)
{
    switch (generic ? 0x100 : cmd) {
    case 0x04:
        out[0] = s->id[0];
        out[1] = s->id[1];
        out[2] = s->id[2];
        return MIN(max, 3);
    case 0x05:
        out[0] = MIN(s->error_count, 0x7f) | (s->error_count > 0x7f) << 7;
        s->error_count = 0;
        return 1;
    case 0x0a:
        out[0] = (!s->sleep_in) << 7 | s->idle << 6 | s->partial << 5 |
                 (!s->sleep_in) << 4 | (!s->partial) << 3 |
                 (s->display_on && !s->shut_down) << 2;
        return 1;
    case 0x0b:
        out[0] = s->madctl;
        return 1;
    case 0x0c:
        out[0] = s->colmod;
        return 1;
    case 0x0d:
        out[0] = s->inversion << 5;
        return 1;
    case 0x0e:
        out[0] = s->tear;
        return 1;
    case 0x0f:
        out[0] = 0;
        return 1;
    case 0x2e:
        s->rx = s->sc;
        s->ry = s->sp;
        return mem_read(s, out, max);
    case 0x3e:
        return mem_read(s, out, max);
    case 0x45:
        out[0] = s->video_row >> 8;
        out[1] = s->video_row;
        return MIN(max, 2);
    case 0x52:
        out[0] = s->brightness;
        return 1;
    case 0x54:
        out[0] = s->ctrl_display;
        return 1;
    case 0x56:
        out[0] = s->cabc;
        return 1;
    case 0xda:
    case 0xdb:
    case 0xdc:
        out[0] = s->id[cmd - 0xda];
        return 1;
    default:
        if (!s->vreg_len[cmd]) {
            out[0] = 0;
            return 1;
        }
        memcpy(out, s->vreg[cmd], MIN(max, s->vreg_len[cmd]));
        return MIN(max, s->vreg_len[cmd]);
    }
}

/* ---- The link ---- */

static void video_line_end(ESP32P4DsiPanelState *s)
{
    if (s->video_col) {
        s->video_row++;
        s->video_col = 0;
    }
}

static void video_pixels(ESP32P4DsiPanelState *s, const ESP32P4DsiPacket *p)
{
    uint32_t n, room;

    if (!s->in_frame || s->video_row >= s->height ||
        s->video_col >= s->width) {
        return;
    }
    room = s->width - s->video_col;
    n = decode_pixels(p->dt, p->payload, p->wc,
                      s->fb + s->video_row * s->width + s->video_col, room);
    s->video_col += n;
    if (s->video_row == s->height - 1 && s->video_col >= s->width) {
        s->frames++;
        panel_dirty(s);
    }
}

void esp32p4_dsi_panel_receive(ESP32P4DsiPanelState *s,
                               const ESP32P4DsiPacket *p)
{
    if (p->hs && p->lanes != s->lanes) {
        s->errors |= ERR_ECC_MULTI;
        s->error_count++;
        return;
    }
    if (s->read_pending) {
        /* The read request before was not followed by a turnaround. */
        s->errors |= ERR_PROTOCOL;
        s->read_pending = false;
    }
    if (p->vc != s->vc) {
        s->errors |= ERR_VC;
        s->error_count++;
        return;
    }
    switch (p->dt) {
    case 0x01:
        if (s->in_frame) {
            panel_dirty(s);
        }
        s->in_frame = true;
        s->video_row = 0;
        s->video_col = 0;
        break;
    case 0x11:
    case 0x21:
        video_line_end(s);
        break;
    case 0x31: case 0x08: case 0x09: case 0x19:
        break;
    case 0x3e: case 0x0e: case 0x1e: case 0x2e:
        video_pixels(s, p);
        break;
    case 0x02:
    case 0x12:
        s->idle = p->dt == 0x12;
        panel_dirty(s);
        break;
    case 0x22:
    case 0x32:
        s->shut_down = p->dt == 0x22;
        panel_dirty(s);
        break;
    case 0x03:
        break;
    case 0x13:
        vreg_store(s, p->wc & 0xff, NULL, 0);
        break;
    case 0x23: {
        uint8_t v = p->wc >> 8;

        vreg_store(s, p->wc & 0xff, &v, 1);
        break;
    }
    case 0x29:
        if (p->wc) {
            vreg_store(s, p->payload[0], p->payload + 1, p->wc - 1);
        }
        break;
    case 0x05:
        dcs_write(s, p->wc & 0xff, NULL, 0, p->end_ns);
        break;
    case 0x15: {
        uint8_t v = p->wc >> 8;

        dcs_write(s, p->wc & 0xff, &v, 1, p->end_ns);
        break;
    }
    case 0x39:
        if (p->wc) {
            dcs_write(s, p->payload[0], p->payload + 1, p->wc - 1, p->end_ns);
        }
        break;
    case 0x37:
        s->max_return = p->wc;
        break;
    case 0x04: case 0x14: case 0x24: case 0x06:
        if (!p->bta) {
            s->errors |= ERR_PROTOCOL;
            break;
        }
        s->read_pending = true;
        s->read_dt = p->dt;
        s->read_cmd = p->wc & 0xff;
        s->read_vc = p->vc;
        break;
    default:
        s->errors |= ERR_DT;
        s->error_count++;
        break;
    }
}

void esp32p4_dsi_panel_trigger(ESP32P4DsiPanelState *s, unsigned trigger,
                               int64_t when_ns)
{
    if (trigger & 1) {
        panel_reset_state(s);
    }
}

/* Append a packet to the answer: the header with its ECC, the payload. */
static void put_packet(GByteArray *a, uint8_t di, uint16_t wc,
                       const uint8_t *pld)
{
    uint32_t hdr = di | (uint32_t)wc << 8;
    uint8_t h[4] = { hdr, hdr >> 8, hdr >> 16, esp32p4_csi2_ecc(hdr) };

    g_byte_array_append(a, h, 4);
    if (pld) {
        uint16_t crc = esp32p4_csi2_crc16(pld, wc);
        uint8_t c[2] = { crc, crc >> 8 };

        g_byte_array_append(a, pld, wc);
        g_byte_array_append(a, c, 2);
    }
}

void esp32p4_dsi_panel_turnaround(ESP32P4DsiPanelState *s,
                                  ESP32P4DsiResponse *r, int64_t when_ns)
{
    GByteArray *a = g_byte_array_new();
    bool fatal = s->errors & (ERR_ECC_MULTI | ERR_DT | ERR_VC | ERR_PROTOCOL);

    r->none = false;
    r->ack = false;
    if (s->read_pending && !fatal) {
        uint8_t buf[512];
        uint32_t max = MIN(MAX(s->max_return, 1), sizeof(buf));
        bool generic = s->read_dt != 0x06;
        uint32_t n = dcs_read(s, s->read_cmd, buf, max, generic);
        uint8_t di = s->read_vc << 6;

        if (n <= 2) {
            put_packet(a, di | (generic ? 0x10 : 0x20) | n,
                       buf[0] | (n == 2 ? buf[1] << 8 : 0), NULL);
        } else {
            put_packet(a, di | (generic ? 0x1a : 0x1c), n, buf);
        }
    }
    s->read_pending = false;
    if (s->errors) {
        put_packet(a, 0x02, s->errors, NULL);
        s->errors = 0;
    }
    if (!a->len) {
        r->ack = true;
    }
    r->len = a->len;
    r->bytes = g_byte_array_free(a, false);
}

/* ---- The console ---- */

static void panel_invalidate(void *opaque)
{
    panel_dirty(opaque);
}

static bool panel_update(void *opaque)
{
    ESP32P4DsiPanelState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    bool blank = s->sleep_in || !s->display_on || s->shut_down;
    uint32_t stride;
    uint8_t *dst;

    if (!s->dirty || !surface || surface_bits_per_pixel(surface) != 32) {
        return true;
    }
    s->dirty = false;
    dst = surface_data(surface);
    stride = surface_stride(surface);
    for (uint32_t y = 0; y < s->height; y++) {
        uint32_t *row = (uint32_t *)(dst + y * stride);
        uint32_t sy = s->madctl & BIT(7) ? s->height - 1 - y : y;

        for (uint32_t x = 0; x < s->width; x++) {
            uint32_t sx = s->madctl & BIT(6) ? s->width - 1 - x : x;
            uint32_t px = s->fb[sy * s->width + sx];

            if (blank) {
                row[x] = 0;
                continue;
            }
            if (s->madctl & BIT(3)) {
                px = (px & 0xff) << 16 | (px & 0xff00) | (px >> 16 & 0xff);
            }
            if (s->inversion) {
                px ^= 0xffffff;
            }
            if (s->idle) {
                px = (px & 0x800000 ? 0xff0000 : 0) |
                     (px & 0x8000 ? 0xff00 : 0) | (px & 0x80 ? 0xff : 0);
            }
            row[x] = px;
        }
    }
    qemu_console_update(s->con, 0, 0, s->width, s->height);
    return true;
}

static const GraphicHwOps panel_gfx_ops = {
    .invalidate = panel_invalidate,
    .gfx_update = panel_update,
};

/* ---- The device ---- */

static void panel_realize(DeviceState *dev, Error **errp)
{
    ESP32P4DsiPanelState *s = ESP32P4_DSI_PANEL(dev);

    if (!s->width || s->width > 4096 || !s->height || s->height > 4096) {
        error_setg(errp, "%s: width and height must be 1 to 4096",
                   TYPE_ESP32P4_DSI_PANEL);
        return;
    }
    if (s->lanes < 1 || s->lanes > 4) {
        error_setg(errp, "%s: lanes must be 1 to 4", TYPE_ESP32P4_DSI_PANEL);
        return;
    }
    if (s->vc > 3) {
        error_setg(errp, "%s: vc must be 0 to 3", TYPE_ESP32P4_DSI_PANEL);
        return;
    }
    s->fb_words = s->width * s->height;
    s->fb = g_new0(uint32_t, s->fb_words);
    s->con = qemu_graphic_console_create(dev, 0, &panel_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
    panel_reset_state(s);
}

static void panel_unrealize(DeviceState *dev)
{
    ESP32P4DsiPanelState *s = ESP32P4_DSI_PANEL(dev);

    qemu_graphic_console_close(s->con);
    g_free(s->fb);
}

/* Only the board's power-on resets the panel; the chip's resets do not. */
static void panel_reset_hold(Object *obj, ResetType type)
{
    ESP32P4DsiPanelState *s = ESP32P4_DSI_PANEL(obj);

    if (!s->powered) {
        s->powered = true;
        panel_reset_state(s);
    }
}

static int panel_post_load(void *opaque, int version_id)
{
    ESP32P4DsiPanelState *s = opaque;

    panel_dirty(s);
    return 0;
}

static const VMStateDescription vmstate_panel = {
    .name = TYPE_ESP32P4_DSI_PANEL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = panel_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(fb_words, ESP32P4DsiPanelState),
        VMSTATE_VARRAY_UINT32_ALLOC(fb, ESP32P4DsiPanelState, fb_words, 0,
                                    vmstate_info_uint32, uint32_t),
        VMSTATE_BOOL(powered, ESP32P4DsiPanelState),
        VMSTATE_BOOL(sleep_in, ESP32P4DsiPanelState),
        VMSTATE_INT64(sleep_out_ns, ESP32P4DsiPanelState),
        VMSTATE_BOOL(display_on, ESP32P4DsiPanelState),
        VMSTATE_BOOL(inversion, ESP32P4DsiPanelState),
        VMSTATE_BOOL(idle, ESP32P4DsiPanelState),
        VMSTATE_BOOL(partial, ESP32P4DsiPanelState),
        VMSTATE_BOOL(shut_down, ESP32P4DsiPanelState),
        VMSTATE_UINT8(madctl, ESP32P4DsiPanelState),
        VMSTATE_UINT8(colmod, ESP32P4DsiPanelState),
        VMSTATE_UINT8(brightness, ESP32P4DsiPanelState),
        VMSTATE_UINT8(ctrl_display, ESP32P4DsiPanelState),
        VMSTATE_UINT8(cabc, ESP32P4DsiPanelState),
        VMSTATE_UINT8(tear, ESP32P4DsiPanelState),
        VMSTATE_UINT16(sc, ESP32P4DsiPanelState),
        VMSTATE_UINT16(ec, ESP32P4DsiPanelState),
        VMSTATE_UINT16(sp, ESP32P4DsiPanelState),
        VMSTATE_UINT16(ep, ESP32P4DsiPanelState),
        VMSTATE_UINT32(wx, ESP32P4DsiPanelState),
        VMSTATE_UINT32(wy, ESP32P4DsiPanelState),
        VMSTATE_UINT32(rx, ESP32P4DsiPanelState),
        VMSTATE_UINT32(ry, ESP32P4DsiPanelState),
        VMSTATE_UINT16(max_return, ESP32P4DsiPanelState),
        VMSTATE_UINT16(errors, ESP32P4DsiPanelState),
        VMSTATE_UINT32(error_count, ESP32P4DsiPanelState),
        VMSTATE_UINT32(video_row, ESP32P4DsiPanelState),
        VMSTATE_UINT32(video_col, ESP32P4DsiPanelState),
        VMSTATE_BOOL(in_frame, ESP32P4DsiPanelState),
        VMSTATE_UINT32(frames, ESP32P4DsiPanelState),
        VMSTATE_UINT8_2DARRAY(vreg, ESP32P4DsiPanelState,
                              ESP32P4_DSI_PANEL_VREGS,
                              ESP32P4_DSI_PANEL_VREG_LEN),
        VMSTATE_UINT8_ARRAY(vreg_len, ESP32P4DsiPanelState,
                            ESP32P4_DSI_PANEL_VREGS),
        VMSTATE_BOOL(read_pending, ESP32P4DsiPanelState),
        VMSTATE_UINT8(read_dt, ESP32P4DsiPanelState),
        VMSTATE_UINT8(read_cmd, ESP32P4DsiPanelState),
        VMSTATE_UINT8(read_vc, ESP32P4DsiPanelState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property panel_properties[] = {
    DEFINE_PROP_UINT32("width", ESP32P4DsiPanelState, width, 1024),
    DEFINE_PROP_UINT32("height", ESP32P4DsiPanelState, height, 600),
    DEFINE_PROP_UINT32("lanes", ESP32P4DsiPanelState, lanes, 2),
    DEFINE_PROP_UINT32("vc", ESP32P4DsiPanelState, vc, 0),
    DEFINE_PROP_BOOL("display-on", ESP32P4DsiPanelState, display_on_reset,
                     true),
    DEFINE_PROP_UINT8("id1", ESP32P4DsiPanelState, id[0], 0),
    DEFINE_PROP_UINT8("id2", ESP32P4DsiPanelState, id[1], 0),
    DEFINE_PROP_UINT8("id3", ESP32P4DsiPanelState, id[2], 0),
};

static void panel_init(Object *obj)
{
    ESP32P4DsiPanelState *s = ESP32P4_DSI_PANEL(obj);

    object_property_add_uint32_ptr(obj, "frames", &s->frames,
                                   OBJ_PROP_FLAG_READ);
}

static void panel_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = panel_realize;
    dc->unrealize = panel_unrealize;
    dc->vmsd = &vmstate_panel;
    dc->bus_type = TYPE_ESP32P4_DSI_BUS;
    dc->desc = "MIPI DSI panel on the ESP32-P4's DSI link";
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    rc->phases.hold = panel_reset_hold;
    device_class_set_props(dc, panel_properties);
}

/* [spec:nuos:req:emu.esp32p4.mipi] */
static const TypeInfo panel_info = {
    .name = TYPE_ESP32P4_DSI_PANEL,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(ESP32P4DsiPanelState),
    .instance_init = panel_init,
    .class_init = panel_class_init,
};

static void panel_register_types(void)
{
    type_register_static(&panel_info);
}
type_init(panel_register_types)
