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
 * esp_lcd_ili9881c; for "controller=hx8394", Himax's HX8394-F
 * preliminary datasheet V01 (December 2014), Waveshare's esp_lcd_hx8394
 * (2.1.0) and Linux's panel-himax-hx8394.c.
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
 *     picture: its last parameters (up to 128 bytes) are kept by command
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
 *
 * [spec:nuos:req:emu.esp32p4.dsi-panel]
 * An HX8394 ("controller=hx8394") is a video-mode controller with no frame
 * memory but its 1-bit idle GRAM, and takes only what its datasheet gives
 * it:
 *
 *   - Packets: those of its table 4.4 and the colour mode and peripheral
 *     packets after it (sync events, EoTp, DCS short and long writes, DCS
 *     read, Set Maximum Return Packet Size, null and blanking, the four
 *     pixel streams, 0x02, 0x12, 0x22, 0x32). Generic writes and reads
 *     are not among them: each is "DSI Data Type Not Recognized" (error
 *     11), and no access takes place (4.2.10 E). Read data always comes
 *     as a DCS Long Read Response (0x1C, table 4.6), however short.
 *   - Lanes. HS packets must come on the lane count SETMIPI's LAN_NUM
 *     sets (BAh's first parameter, bits 1:0, the count less one; 4.2 "The
 *     data lane number is selected by register RBAh"), 4 lanes from reset
 *     (its default 63h, table 5.18.2), and on no more lanes than the board
 *     wires: otherwise the panel cannot read them, as above. Commands in
 *     LP (escape mode on lane 0) need no lane setting, so software sets
 *     the count in LP before video, as ESP-IDF's DBI IO does.
 *   - Commands: the standard set of its table 5.18.1 (nop, swreset,
 *     RDDIDIF, RDNUMPE, the colour, status and mode reads 06h to 0Fh,
 *     slpin, slpout, noron, invoff/on, allpoff/on, gamset, dispoff/on,
 *     ramwr, teoff/on, madctl, idmoff/on, colmod, ramwrc, tesl, getscan,
 *     wrdisbv, rddisbv, wrctrld, rdctrld, wrcabc, rdcabc, wrcabcmb,
 *     rdcabcmb, rdabcsdr, wrimcol, rdimcol, the DDB reads and the ID reads
 *     DAh to DCh); "Undefined commands are treated as NOP" (its note 1):
 *     a write is ignored (logged), a read gets the Acknowledge and no
 *     data. The user define commands (B0h to FFh but DAh to DCh) are
 *     "available only set SETEXTC": before SETEXTC (B9h) takes FFh 83h 94h
 *     they are undefined, as RDNUMPE is (logged); after, their parameters
 *     are kept and read back, by bank for the six commands SET_BANK (BDh)
 *     banks (B1h, B2h, B8h, C1h, D3h, D8h), BAh setting the lane count,
 *     C3h (SETID) the IDs DAh to DCh read, the rest without effect on the
 *     picture. Unwritten, B9h reads 00h 00h 00h and BAh its defaults 63h
 *     03h 68h 63h; others a zero byte. RAMWR and RAMWRC are accepted
 *     without effect (logged): the panel shows only the video it is sent.
 *   - Status. RDDPM (0Ah) is D6 idle, D4 sleep out, D3 normal mode (always,
 *     partial mode not being "applicable for this project"), D2 display on
 *     (08h from reset); RDDMADCTL, RDDCOLMOD, RDDIM, RDDSM, RDDST as their
 *     sections give the bits, the ones "not applicable for this project"
 *     zero (RDDST's D26 among them, though MADCTL's B3 applies); RDDSDR's
 *     D7 and D6 invert at each Sleep Out from Sleep In (5.11). MADCTL keeps
 *     B3 (BGR), B1 (flip horizontal) and B0 (flip vertical), the rest "set
 *     to 0 internally", and swreset leaves it as it is. COLMOD (70h from
 *     reset) is kept and read back: in DSI video mode each pixel packet's
 *     data type gives its format.
 *   - The picture. Display off from reset (its RDDPM default 08h and
 *     DISPOFF's table; DISPON's table says on). Sleep Out takes 120 ms
 *     ("It takes 120msec to become Sleep Out mode"): the panel shows the
 *     video only from then, with its display on and not shut down; else
 *     black, as from Sleep In at once. ALLPOFF and ALLPON show black and
 *     white, display on or off, until NORON or each other ends them
 *     (neither takes effect with inversion on); idle mode 8 colours;
 *     inversion; MADCTL's flips and BGR.
 *   - Times its datasheet asks of the host are checked and logged, the
 *     command still taken (it does not say what a command too early
 *     does): 50 ms from RESX rising (or power on) to the first command
 *     (figure 5.28), 5 ms after swreset, slpout and slpin, 120 ms between
 *     slpin and slpout either way, and from slpout to dispon.
 *   - The Reset-Trigger resets it as RESX does; swreset to its S/W reset
 *     defaults (MADCTL kept, the extended commands disabled, the lane
 *     count 4 again).
 *
 * A panel whose module switches its supplies ("power-pins") has inputs
 * for them (esp32p4_mipi_dsi.h): it is powered while AVDD and IOVCC are
 * high, and runs while also RESX is high. Unpowered or held in reset it
 * receives nothing and does not answer a turnaround (it does not drive the
 * lane) and shows black; it comes up from its power-on (or RESX rising)
 * reset, its frame memory black. What it shows is lit by its backlight:
 * black with the backlight disabled, else each colour scaled by the PWM
 * duty over 255.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/esp32p4_mipi_dsi.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_image_source.h"
#include "migration/vmstate.h"
#include "ui/console.h"

/* Error report bits (MIPI DSI 1.1 table 20). */
#define ERR_ECC_MULTI       BIT(9)
#define ERR_DT              BIT(11)
#define ERR_VC              BIT(12)
#define ERR_PROTOCOL        BIT(15)

/* The HX8394-F's waits (figure 5.28, 5.18.4, 5.18.17 and 5.18.18). */
#define HX_RESET_WAIT_NS    (50 * SCALE_MS)
#define HX_CMD_WAIT_NS      (5 * SCALE_MS)
#define HX_SLEEP_NS         (120 * SCALE_MS)

enum {
    READY_RESET = 1,
    READY_SWRESET,
    READY_SLPOUT,
    READY_SLPIN,
};

/* The commands SET_BANK banks (5.19.11). */
static const uint8_t hx_banked[ESP32P4_DSI_PANEL_BANKED] = {
    0xb1, 0xb2, 0xb8, 0xc1, 0xd3, 0xd8,
};

static void panel_dirty(ESP32P4DsiPanelState *s)
{
    s->dirty = true;
}

/*
 * The defaults of a reset: @sw, the HX8394's swreset, which leaves
 * MADCTL; otherwise the power-on and H/W reset's.
 */
static void panel_reset_state(ESP32P4DsiPanelState *s, bool sw)
{
    uint8_t madctl = s->madctl;

    s->sleep_in = true;
    s->sleep_out_ns = 0;
    s->sleep_in_ns = INT64_MIN / 2;
    s->display_on = s->display_on_reset;
    s->inversion = false;
    s->idle = false;
    s->partial = false;
    s->shut_down = false;
    s->madctl = sw && s->hx8394 ? madctl : 0;
    s->colmod = s->hx8394 ? 0x70 : 0x77;
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
    memcpy(s->id_now, s->id, sizeof(s->id_now));
    s->extc = false;
    s->rx_lanes = 4;
    s->bank = 0;
    s->dsdr = 0;
    s->all_on = false;
    s->all_off = false;
    s->gamma = 0;
    s->imcol = 0;
    s->cabc_min = 0;
    if (s->hx8394) {
        static const uint8_t setmipi[4] = { 0x63, 0x03, 0x68, 0x63 };

        s->vreg_len[0xb9] = 3;
        memcpy(s->vreg[0xba], setmipi, sizeof(setmipi));
        s->vreg_len[0xba] = sizeof(setmipi);
    }
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

/* Where command @cmd's parameters are kept: by bank for those banked. */
static unsigned vreg_index(ESP32P4DsiPanelState *s, uint8_t cmd)
{
    if (s->hx8394 && s->bank) {
        for (unsigned k = 0; k < ESP32P4_DSI_PANEL_BANKED; k++) {
            if (hx_banked[k] == cmd) {
                return 256 + (s->bank - 1) * ESP32P4_DSI_PANEL_BANKED + k;
            }
        }
    }
    return cmd;
}

static void vreg_store(ESP32P4DsiPanelState *s, uint8_t cmd,
                       const uint8_t *p, uint32_t n)
{
    unsigned i = vreg_index(s, cmd);

    n = MIN(n, ESP32P4_DSI_PANEL_VREG_LEN);
    if (n) {
        memcpy(s->vreg[i], p, n);
    }
    s->vreg_len[i] = n;
}

/* The kept parameters of @cmd, up to @max; a zero byte if none. */
static uint32_t vreg_read(ESP32P4DsiPanelState *s, uint8_t cmd, uint8_t *out,
                          uint32_t max)
{
    unsigned i = vreg_index(s, cmd);

    if (!s->vreg_len[i]) {
        out[0] = 0;
        return 1;
    }
    memcpy(out, s->vreg[i], MIN(max, s->vreg_len[i]));
    return MIN(max, s->vreg_len[i]);
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
        panel_reset_state(s, true);
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
        return vreg_read(s, cmd, out, max);
    }
}

/* ---- The HX8394 ---- */

/* A user define command: B0h to FFh but the ID reads (5.18.1 note 2). */
static bool hx_user(uint8_t cmd)
{
    return cmd >= 0xb0 && (cmd < 0xda || cmd > 0xdc);
}

static bool hx_std_write(uint8_t cmd)
{
    switch (cmd) {
    case 0x00: case 0x01: case 0x10: case 0x11: case 0x13: case 0x20:
    case 0x21: case 0x22: case 0x23: case 0x26: case 0x28: case 0x29:
    case 0x2c: case 0x34: case 0x35: case 0x36: case 0x38: case 0x39:
    case 0x3a: case 0x3c: case 0x44: case 0x51: case 0x53: case 0x55:
    case 0x5e: case 0x80:
        return true;
    default:
        return false;
    }
}

/* The packets of its tables 4.4 and those described after it. */
static bool hx_dt_known(uint8_t dt)
{
    switch (dt) {
    case 0x01: case 0x11: case 0x21: case 0x31: case 0x08: case 0x05:
    case 0x15: case 0x06: case 0x37: case 0x09: case 0x19: case 0x39:
    case 0x0e: case 0x1e: case 0x2e: case 0x3e: case 0x02: case 0x12:
    case 0x22: case 0x32:
        return true;
    default:
        return false;
    }
}

static const char *const hx_ready_why[] = {
    [READY_RESET] = "RESX rose (50 ms asked)",
    [READY_SWRESET] = "swreset (5 ms asked)",
    [READY_SLPOUT] = "slpout (5 ms asked)",
    [READY_SLPIN] = "slpin (5 ms asked)",
};

/* [spec:nuos:req:emu.esp32p4.dsi-panel] A command too soon, logged. */
static void hx_check_ready(ESP32P4DsiPanelState *s, uint8_t cmd,
                           int64_t when)
{
    if (when < s->cmd_ready_ns && s->cmd_ready_why) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 command %02xh %" PRId64
                      " us after %s\n", TYPE_ESP32P4_DSI_PANEL, cmd,
                      (when - (s->cmd_ready_ns -
                               (s->cmd_ready_why == READY_RESET ?
                                HX_RESET_WAIT_NS : HX_CMD_WAIT_NS))) /
                      SCALE_US, hx_ready_why[s->cmd_ready_why]);
    }
}

static void hx_ready_after(ESP32P4DsiPanelState *s, int64_t when, int why)
{
    s->cmd_ready_ns = when + (why == READY_RESET ? HX_RESET_WAIT_NS :
                                                   HX_CMD_WAIT_NS);
    s->cmd_ready_why = why;
}

static void hx_user_write(ESP32P4DsiPanelState *s, uint8_t cmd,
                          const uint8_t *p, uint32_t n)
{
    if (cmd == 0xb9) {
        s->extc = n >= 3 && p[0] == 0xff && p[1] == 0x83 && p[2] == 0x94;
        vreg_store(s, cmd, p, n);
        return;
    }
    if (!s->extc) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 command %02xh before "
                      "SETEXTC enables it: ignored\n",
                      TYPE_ESP32P4_DSI_PANEL, cmd);
        return;
    }
    vreg_store(s, cmd, p, n);
    switch (cmd) {
    case 0xba:
        if (n) {
            s->rx_lanes = (p[0] & 3) + 1;
        }
        break;
    case 0xbd:
        if (n) {
            s->bank = p[0] & 3;
        }
        break;
    case 0xc3:
        memcpy(s->id_now, p, MIN(n, sizeof(s->id_now)));
        break;
    default:
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.dsi-panel] */
static void hx_write(ESP32P4DsiPanelState *s, uint8_t cmd, const uint8_t *p,
                     uint32_t n, int64_t when)
{
    if (hx_user(cmd)) {
        hx_user_write(s, cmd, p, n);
        return;
    }
    if (!hx_std_write(cmd)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 has no command %02xh: "
                      "taken as a NOP\n", TYPE_ESP32P4_DSI_PANEL, cmd);
        return;
    }
    switch (cmd) {
    case 0x01:
        panel_reset_state(s, true);
        hx_ready_after(s, when, READY_SWRESET);
        break;
    case 0x10:
        if (!s->sleep_in) {
            if (when < s->sleep_out_ns + HX_SLEEP_NS) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 slpin %" PRId64
                              " ms after slpout (120 ms asked)\n",
                              TYPE_ESP32P4_DSI_PANEL,
                              (when - s->sleep_out_ns) / SCALE_MS);
            }
            s->sleep_in = true;
            s->sleep_in_ns = when;
            hx_ready_after(s, when, READY_SLPIN);
        }
        break;
    case 0x11:
        if (s->sleep_in) {
            if (when < s->sleep_in_ns + HX_SLEEP_NS) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 slpout %" PRId64
                              " ms after slpin (120 ms asked)\n",
                              TYPE_ESP32P4_DSI_PANEL,
                              (when - s->sleep_in_ns) / SCALE_MS);
            }
            s->sleep_in = false;
            s->sleep_out_ns = when;
            s->dsdr ^= 0xc0;
            hx_ready_after(s, when, READY_SLPOUT);
        }
        break;
    case 0x20:
    case 0x21:
        s->inversion = cmd == 0x21;
        break;
    case 0x22:
    case 0x23:
        if (!s->inversion) {
            s->all_off = cmd == 0x22;
            s->all_on = cmd == 0x23;
        }
        break;
    case 0x13:
        s->all_off = false;
        s->all_on = false;
        break;
    case 0x26:
        if (n && p[0] && is_power_of_2(p[0])) {
            s->gamma = ctz32(p[0]);
        }
        break;
    case 0x28:
        s->display_on = false;
        break;
    case 0x29:
        if (!s->sleep_in && when < s->sleep_out_ns + HX_SLEEP_NS) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 dispon %" PRId64
                          " ms after slpout (120 ms asked)\n",
                          TYPE_ESP32P4_DSI_PANEL,
                          (when - s->sleep_out_ns) / SCALE_MS);
        }
        s->display_on = true;
        break;
    case 0x2c:
    case 0x3c:
        if (!s->ram_warned) {
            qemu_log_mask(LOG_UNIMP, "%s: HX8394 memory writes do not reach "
                          "the picture, which is the video's\n",
                          TYPE_ESP32P4_DSI_PANEL);
            s->ram_warned = true;
        }
        break;
    case 0x34:
        s->tear = 0;
        break;
    case 0x35:
        s->tear = 0x80 | (n ? (p[0] & 1) << 6 : 0);
        break;
    case 0x36:
        if (n) {
            s->madctl = p[0] & 0x0b;
        }
        break;
    case 0x38:
    case 0x39:
        s->idle = cmd == 0x39;
        break;
    case 0x3a:
        if (n) {
            s->colmod = p[0] & 0x70;
        }
        break;
    case 0x51:
        if (n) {
            s->brightness = p[0];
        }
        break;
    case 0x53:
        if (n) {
            s->ctrl_display = p[0] & 0x2c;
        }
        break;
    case 0x55:
        if (n) {
            s->cabc = p[0] & 3;
        }
        break;
    case 0x5e:
        if (n) {
            s->cabc_min = p[0];
        }
        break;
    case 0x80:
        if (n) {
            s->imcol = p[0] & 7;
        }
        break;
    default:
        vreg_store(s, cmd, p, n);
        break;
    }
    panel_dirty(s);
}

/* RDDST's four bytes (5.18.10). */
static uint32_t hx_status(ESP32P4DsiPanelState *s)
{
    return (uint32_t)!s->sleep_in << 31 | (s->madctl & BIT(1)) << 23 |
           (s->madctl & BIT(0)) << 23 | (s->colmod & 0x70) << 16 |
           s->idle << 19 | (!s->sleep_in) << 17 | 1u << 16 |
           s->inversion << 13 | s->all_on << 12 | s->all_off << 11 |
           (s->display_on && !s->shut_down) << 10 | (s->tear >> 7) << 9 |
           (s->gamma & 7) << 6 | ((s->tear >> 6) & 1) << 5;
}

/*
 * [spec:nuos:req:emu.esp32p4.dsi-panel]
 * The bytes the HX8394's read of @cmd gives, up to @max; -1 for a command
 * it does not have, which it takes as a NOP.
 */
static int hx_read(ESP32P4DsiPanelState *s, uint8_t cmd, uint8_t *out,
                   uint32_t max)
{
    switch (cmd) {
    case 0x04:
        memcpy(out, s->id_now, 3);
        return MIN(max, 3);
    case 0x05:
        if (!s->extc) {
            break;
        }
        out[0] = MIN(s->error_count, 0x7f) | (s->error_count > 0x7f) << 7;
        s->error_count = 0;
        return 1;
    case 0x06:
    case 0x07:
    case 0x08:
        qemu_log_mask(LOG_UNIMP, "%s: HX8394 colour reads give 0\n",
                      TYPE_ESP32P4_DSI_PANEL);
        out[0] = 0;
        return 1;
    case 0x09: {
        uint32_t v = hx_status(s);

        out[0] = v >> 24;
        out[1] = v >> 16;
        out[2] = v >> 8;
        out[3] = v;
        return MIN(max, 4);
    }
    case 0x0a:
        out[0] = s->idle << 6 | (!s->sleep_in) << 4 | 1 << 3 |
                 (s->display_on && !s->shut_down) << 2;
        return 1;
    case 0x0b:
        out[0] = s->madctl;
        return 1;
    case 0x0c:
        out[0] = s->colmod;
        return 1;
    case 0x0d:
        out[0] = s->inversion << 5 | s->all_on << 4 | s->all_off << 3 |
                 (s->gamma & 7);
        return 1;
    case 0x0e:
        out[0] = s->tear;
        return 1;
    case 0x0f:
        out[0] = s->dsdr;
        return 1;
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
    case 0x5f:
        out[0] = s->cabc_min;
        return 1;
    case 0x68:
        out[0] = 0;
        return 1;
    case 0x81:
        out[0] = s->imcol;
        return 1;
    case 0xa1:
    case 0xa8:
        qemu_log_mask(LOG_UNIMP, "%s: HX8394 DDB reads are not modelled\n",
                      TYPE_ESP32P4_DSI_PANEL);
        break;
    case 0xda:
    case 0xdb:
    case 0xdc:
        out[0] = s->id_now[cmd - 0xda];
        return 1;
    default:
        if (hx_user(cmd) && s->extc) {
            return vreg_read(s, cmd, out, max);
        }
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: HX8394 read of %02xh, which it %s: "
                  "no data\n", TYPE_ESP32P4_DSI_PANEL, cmd,
                  hx_user(cmd) ? "has only after SETEXTC" : "does not have");
    return -1;
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

static void panel_dcs_write(ESP32P4DsiPanelState *s, uint8_t cmd,
                            const uint8_t *p, uint32_t n, int64_t when)
{
    if (s->hx8394) {
        hx_check_ready(s, cmd, when);
        hx_write(s, cmd, p, n, when);
    } else {
        dcs_write(s, cmd, p, n, when);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.mipi]
 * [spec:nuos:req:emu.esp32p4.dsi-panel]
 */
void esp32p4_dsi_panel_receive(ESP32P4DsiPanelState *s,
                               const ESP32P4DsiPacket *p)
{
    uint32_t rx_lanes = s->hx8394 ? s->rx_lanes : s->lanes;

    if (!s->live) {
        return;
    }
    if (p->hs && (p->lanes != rx_lanes || rx_lanes > s->lanes)) {
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
    if (s->hx8394 && !hx_dt_known(p->dt)) {
        s->errors |= ERR_DT;
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
        panel_dcs_write(s, p->wc & 0xff, NULL, 0, p->end_ns);
        break;
    case 0x15: {
        uint8_t v = p->wc >> 8;

        panel_dcs_write(s, p->wc & 0xff, &v, 1, p->end_ns);
        break;
    }
    case 0x39:
        if (p->wc) {
            panel_dcs_write(s, p->payload[0], p->payload + 1, p->wc - 1,
                            p->end_ns);
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
        if (s->hx8394) {
            hx_check_ready(s, p->wc & 0xff, p->end_ns);
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
    if ((trigger & 1) && s->live) {
        panel_reset_state(s, false);
        if (s->hx8394) {
            hx_ready_after(s, when_ns, READY_RESET);
        }
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

/* [spec:nuos:req:emu.esp32p4.dsi-panel] */
void esp32p4_dsi_panel_turnaround(ESP32P4DsiPanelState *s,
                                  ESP32P4DsiResponse *r, int64_t when_ns)
{
    GByteArray *a;
    bool fatal = s->errors & (ERR_ECC_MULTI | ERR_DT | ERR_VC | ERR_PROTOCOL);

    r->len = 0;
    r->bytes = NULL;
    r->ack = false;
    if (!s->live) {
        /* Unpowered or in reset, it does not drive the lane. */
        r->none = true;
        return;
    }
    r->none = false;
    a = g_byte_array_new();
    if (s->read_pending && !fatal) {
        uint8_t buf[512];
        uint32_t max = MIN(MAX(s->max_return, 1), sizeof(buf));
        uint8_t di = s->read_vc << 6;

        if (s->hx8394) {
            int n = hx_read(s, s->read_cmd, buf, max);

            if (n > 0) {
                put_packet(a, di | 0x1c, n, buf);
            }
        } else {
            bool generic = s->read_dt != 0x06;
            uint32_t n = dcs_read(s, s->read_cmd, buf, max, generic);

            if (n <= 2) {
                put_packet(a, di | (generic ? 0x10 : 0x20) | n,
                           buf[0] | (n == 2 ? buf[1] << 8 : 0), NULL);
            } else {
                put_packet(a, di | (generic ? 0x1a : 0x1c), n, buf);
            }
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

/* ---- Supplies, reset and backlight ---- */

/*
 * [spec:nuos:req:emu.esp32p4.panel-control]
 * Follow the supplies and RESX: powering up, or RESX rising while
 * powered, is a power-on (H/W) reset; losing a supply loses the frame.
 */
static void panel_power_update(ESP32P4DsiPanelState *s)
{
    bool supplied = !s->power_pins || (s->avdd && s->iovcc);
    bool live = supplied && (!s->power_pins || s->resx);

    if (!s->powered) {
        return;
    }
    if (supplied != s->supplied) {
        memset(s->fb, 0, s->fb_words * sizeof(*s->fb));
    }
    if (live && !s->live) {
        panel_reset_state(s, false);
        if (s->hx8394) {
            hx_ready_after(s, esp32_line_time(), READY_RESET);
        }
    }
    s->supplied = supplied;
    s->live = live;
    panel_dirty(s);
}

static void panel_avdd(void *opaque, int n, int level)
{
    ESP32P4DsiPanelState *s = opaque;

    s->avdd = level;
    panel_power_update(s);
}

static void panel_iovcc(void *opaque, int n, int level)
{
    ESP32P4DsiPanelState *s = opaque;

    s->iovcc = level;
    panel_power_update(s);
}

static void panel_resx(void *opaque, int n, int level)
{
    ESP32P4DsiPanelState *s = opaque;

    s->resx = level;
    panel_power_update(s);
}

static void panel_bl_en(void *opaque, int n, int level)
{
    ESP32P4DsiPanelState *s = opaque;

    s->bl_en = level;
    panel_dirty(s);
}

static void panel_bl_pwm(void *opaque, int n, int level)
{
    ESP32P4DsiPanelState *s = opaque;

    s->bl_pwm = level;
    panel_dirty(s);
}

/* ---- The console ---- */

/*
 * Whether the panel shows a picture at @now, before its backlight: out
 * of sleep, its display on, or (an HX8394's) all pixels on or off, which
 * show "a status of the 'Display On/Off' register can be on or off".
 */
static bool panel_lit(ESP32P4DsiPanelState *s, int64_t now)
{
    if (!s->live || s->sleep_in || s->shut_down ||
        !(s->display_on || s->all_on || s->all_off)) {
        return false;
    }
    return !s->hx8394 || now >= s->sleep_out_ns + HX_SLEEP_NS;
}

static uint8_t panel_backlight(ESP32P4DsiPanelState *s)
{
    if (!s->power_pins) {
        return 255;
    }
    return s->bl_en ? s->bl_pwm : 0;
}

static uint32_t dim(uint32_t px, uint32_t bl)
{
    uint32_t out = 0;

    for (int sh = 0; sh < 24; sh += 8) {
        out |= (((px >> sh & 0xff) * bl + 127) / 255) << sh;
    }
    return out;
}

static void panel_invalidate(void *opaque)
{
    panel_dirty(opaque);
}

/*
 * [spec:nuos:req:emu.esp32p4.mipi]
 * [spec:nuos:req:emu.esp32p4.dsi-panel]
 */
static bool panel_update(void *opaque)
{
    ESP32P4DsiPanelState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    bool lit = panel_lit(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    uint8_t bl = panel_backlight(s);
    bool flip_y = s->madctl & (s->hx8394 ? BIT(0) : BIT(7));
    bool flip_x = s->madctl & (s->hx8394 ? BIT(1) : BIT(6));
    uint32_t stride;
    uint8_t *dst;

    if (lit != s->shown_lit || bl != s->shown_bl) {
        s->dirty = true;
    }
    if (!s->dirty || !surface || surface_bits_per_pixel(surface) != 32) {
        return true;
    }
    s->dirty = false;
    s->shown_lit = lit;
    s->shown_bl = bl;
    dst = surface_data(surface);
    stride = surface_stride(surface);
    for (uint32_t y = 0; y < s->height; y++) {
        uint32_t *row = (uint32_t *)(dst + y * stride);
        uint32_t sy = flip_y ? s->height - 1 - y : y;

        for (uint32_t x = 0; x < s->width; x++) {
            uint32_t sx = flip_x ? s->width - 1 - x : x;
            uint32_t px = s->fb[sy * s->width + sx];

            if (!lit || !bl) {
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
            if (s->all_off || s->all_on) {
                px = s->all_on ? 0xffffff : 0;
            } else if (!s->display_on) {
                px = 0;
            }
            row[x] = bl == 255 ? px : dim(px, bl);
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
    if (!s->controller || !strcmp(s->controller, "generic")) {
        s->hx8394 = false;
    } else if (!strcmp(s->controller, "hx8394")) {
        s->hx8394 = true;
        if (!s->id[0] && !s->id[1] && !s->id[2]) {
            /* SETID's defaults (table 5.18.2), unprogrammed OTP's. */
            s->id[0] = 0x83;
            s->id[1] = 0x94;
            s->id[2] = 0x0f;
        }
    } else {
        error_setg(errp, "%s: controller must be generic or hx8394",
                   TYPE_ESP32P4_DSI_PANEL);
        return;
    }
    s->fb_words = s->width * s->height;
    s->fb = g_new0(uint32_t, s->fb_words);
    s->con = qemu_graphic_console_create(dev, 0, &panel_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
    panel_reset_state(s, false);
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
        s->live = false;
        s->supplied = false;
        panel_power_update(s);
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
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = panel_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(fb_words, ESP32P4DsiPanelState),
        VMSTATE_VARRAY_UINT32_ALLOC(fb, ESP32P4DsiPanelState, fb_words, 0,
                                    vmstate_info_uint32, uint32_t),
        VMSTATE_BOOL(powered, ESP32P4DsiPanelState),
        VMSTATE_BOOL(avdd, ESP32P4DsiPanelState),
        VMSTATE_BOOL(iovcc, ESP32P4DsiPanelState),
        VMSTATE_BOOL(resx, ESP32P4DsiPanelState),
        VMSTATE_BOOL(bl_en, ESP32P4DsiPanelState),
        VMSTATE_UINT8(bl_pwm, ESP32P4DsiPanelState),
        VMSTATE_BOOL(supplied, ESP32P4DsiPanelState),
        VMSTATE_BOOL(live, ESP32P4DsiPanelState),
        VMSTATE_INT64(cmd_ready_ns, ESP32P4DsiPanelState),
        VMSTATE_UINT8(cmd_ready_why, ESP32P4DsiPanelState),
        VMSTATE_BOOL(sleep_in, ESP32P4DsiPanelState),
        VMSTATE_INT64(sleep_out_ns, ESP32P4DsiPanelState),
        VMSTATE_INT64(sleep_in_ns, ESP32P4DsiPanelState),
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
        VMSTATE_UINT8_ARRAY(id_now, ESP32P4DsiPanelState, 3),
        VMSTATE_BOOL(extc, ESP32P4DsiPanelState),
        VMSTATE_UINT8(rx_lanes, ESP32P4DsiPanelState),
        VMSTATE_UINT8(bank, ESP32P4DsiPanelState),
        VMSTATE_UINT8(dsdr, ESP32P4DsiPanelState),
        VMSTATE_BOOL(all_on, ESP32P4DsiPanelState),
        VMSTATE_BOOL(all_off, ESP32P4DsiPanelState),
        VMSTATE_UINT8(gamma, ESP32P4DsiPanelState),
        VMSTATE_UINT8(imcol, ESP32P4DsiPanelState),
        VMSTATE_UINT8(cabc_min, ESP32P4DsiPanelState),
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
    DEFINE_PROP_STRING("controller", ESP32P4DsiPanelState, controller),
    DEFINE_PROP_BOOL("power-pins", ESP32P4DsiPanelState, power_pins, false),
};

static void panel_init(Object *obj)
{
    ESP32P4DsiPanelState *s = ESP32P4_DSI_PANEL(obj);
    DeviceState *dev = DEVICE(obj);

    object_property_add_uint32_ptr(obj, "frames", &s->frames,
                                   OBJ_PROP_FLAG_READ);
    qdev_init_gpio_in_named(dev, panel_avdd, ESP32P4_DSI_PANEL_AVDD, 1);
    qdev_init_gpio_in_named(dev, panel_iovcc, ESP32P4_DSI_PANEL_IOVCC, 1);
    qdev_init_gpio_in_named(dev, panel_resx, ESP32P4_DSI_PANEL_RESX, 1);
    qdev_init_gpio_in_named(dev, panel_bl_en, ESP32P4_DSI_PANEL_BL_EN, 1);
    qdev_init_gpio_in_named(dev, panel_bl_pwm, ESP32P4_DSI_PANEL_BL_PWM, 1);
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

/* ---- The Waveshare 5-DSI-TOUCH-A ---- */

/*
 * [spec:nuos:req:emu.esp32p4.dsi-panel]
 * [spec:nuos:req:emu.esp32p4.panel-control]
 * [spec:nuos:req:emu.esp32p4.touch]
 * Waveshare's 5-DSI-TOUCH-A, plugged into the board's DSI connector:
 * -device waveshare-5-dsi-touch-a. Its panel is a 5" 720 x 1280 HX8394
 * on 2 data lanes (Waveshare's esp_lcd_hx8394 and Linux's
 * waveshare,5.0-dsi-touch-a); its control chip (esp32p4_panel_mcu.c) is
 * on the connector's I2C wires at 0x45, switching the panel's AVDD,
 * IOVCC, RESX and backlight (lines 0, 4, 1 and 2, and its PWM), as
 * Linux's vc4-kms-dsi-waveshare-panel-v2 overlay and gpio-waveshare-dsi.c
 * wire them. Its GT911 (hw/input/esp32p4_gt911.c) is on the same wires,
 * powered by the chip's VCC (line 8) and reset by its line 9, as the
 * Raspberry Pi kernel's waveshare-panel-regulator.c powers the touch
 * controller with them and its overlay resets the Goodix by line 9. Its
 * INT goes to the control chip, not the connector (the overlay's pull
 * request: "the mcu is connected to the INT pin"); what level the chip
 * holds it at through a reset is not documented: the model's is low
 * (-global gt911.int-level=on for high), selecting 0x5D, the address the
 * overlay gives the touch controller and the one Waveshare's ESP-IDF
 * BSPs try first. The touches are the QEMU pointer's on the panel's
 * console.
 */
static void ws5a_init(Object *obj)
{
    Waveshare5DsiTouchAState *m = WAVESHARE_5_DSI_TOUCH_A(obj);
    ESP32P4DsiPanelState *s = ESP32P4_DSI_PANEL(obj);

    s->width = 720;
    s->height = 1280;
    s->lanes = 2;
    s->display_on_reset = false;
    s->power_pins = true;
    s->controller = g_strdup("hx8394");
    object_initialize_child(obj, "mcu", &m->mcu, TYPE_WAVESHARE_PANEL_MCU);
    object_initialize_child(obj, "touch", &m->touch, TYPE_GT911);
}

static void ws5a_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    Waveshare5DsiTouchAState *m = WAVESHARE_5_DSI_TOUCH_A(dev);
    ESP32P4DsiHostState *host;
    DeviceState *mcu = DEVICE(&m->mcu), *touch = DEVICE(&m->touch);
    BusState *bus = qdev_get_parent_bus(dev);
    static const struct {
        int line;
        bool touch;
        const char *in;
    } wires[] = {
        { WAVESHARE_LINE_AVDD, false, ESP32P4_DSI_PANEL_AVDD },
        { WAVESHARE_LINE_IOVCC, false, ESP32P4_DSI_PANEL_IOVCC },
        { WAVESHARE_LINE_PANEL_RESET, false, ESP32P4_DSI_PANEL_RESX },
        { WAVESHARE_LINE_BL_ENABLE, false, ESP32P4_DSI_PANEL_BL_EN },
        { WAVESHARE_LINE_VCC, true, GT911_VDD },
        { WAVESHARE_LINE_TS_RESET, true, GT911_RSTB },
    };

    host = bus ? (ESP32P4DsiHostState *)object_dynamic_cast(
                     OBJECT(bus->parent), TYPE_ESP32P4_DSI_HOST) : NULL;
    if (!host || !host->connector_i2c) {
        error_setg(errp, "%s: the DSI connector has no I2C wires for its "
                   "control chip", TYPE_WAVESHARE_5_DSI_TOUCH_A);
        return;
    }
    panel_realize(dev, errp);
    if (*errp) {
        return;
    }
    for (int i = 0; i < ARRAY_SIZE(wires); i++) {
        qdev_connect_gpio_out_named(mcu, WAVESHARE_PANEL_MCU_LINE,
                                    wires[i].line,
                                    qdev_get_gpio_in_named(
                                        wires[i].touch ? touch : dev,
                                        wires[i].in, 0));
    }
    qdev_connect_gpio_out_named(mcu, WAVESHARE_PANEL_MCU_PWM, 0,
                                qdev_get_gpio_in_named(dev,
                                    ESP32P4_DSI_PANEL_BL_PWM, 0));
    if (!qdev_realize(mcu, BUS(host->connector_i2c), errp) ||
        !qdev_realize(touch, BUS(host->connector_i2c), errp)) {
        return;
    }
    gt911_bind_console(&m->touch, dev);
}

static void ws5a_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = ws5a_realize;
    dc->desc = "Waveshare 5-DSI-TOUCH-A: an HX8394 720x1280 panel, its "
               "control chip at I2C 0x45 and its GT911 touch controller";
}

/* [spec:nuos:req:emu.esp32p4.mipi] */
static const TypeInfo panel_types[] = {
    {
        .name = TYPE_ESP32P4_DSI_PANEL,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(ESP32P4DsiPanelState),
        .instance_init = panel_init,
        .class_init = panel_class_init,
    },
    {
        .name = TYPE_WAVESHARE_5_DSI_TOUCH_A,
        .parent = TYPE_ESP32P4_DSI_PANEL,
        .instance_size = sizeof(Waveshare5DsiTouchAState),
        .instance_init = ws5a_init,
        .class_init = ws5a_class_init,
    },
};

DEFINE_TYPES(panel_types)
