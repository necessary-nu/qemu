/*
 * i80 (MIPI DBI type B) LCD panels: ILI9341 and ST7789
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A panel module with an ILI9341 or ST7789 driver chip on an LCD
 * controller's i80 port (include/hw/display/lcd_panel.h), write only: its
 * RDX tied high. The chip keeps a 240x320 GRAM of 18-bit pixels, which
 * the panel shows continuously; the console shows the panel, "width" by
 * "height" pixels (default 240x320) from GRAM's top left. "bus-width" is
 * the module's interface, 8 (default) or 16 data lines (its IM pins).
 *
 * The chip takes the MIPI DCS commands that ESP-IDF's panel drivers send
 * (esp_lcd_panel_st7789.c and the ILI9341 component): a command byte on
 * D[7:0] with D/CX low, its parameters on D[7:0] with D/CX high, and
 * after RAMWR or RAMWRC pixel data:
 *
 *   NOP 0x00, SWRESET 0x01 (the registers to their reset values, GRAM
 *   kept), SLPIN 0x10 / SLPOUT 0x11, PTLON 0x12 / NORON 0x13, INVOFF 0x20
 *   / INVON 0x21, DISPOFF 0x28 / DISPON 0x29, CASET 0x2A and RASET 0x2B
 *   (the window, start and end, 16 bits each, big-endian), RAMWR 0x2C
 *   (pixels from the window's start), RAMWRC 0x3C (from where the last
 *   write stopped), MADCTL 0x36, COLMOD 0x3A, IDMOFF 0x38 / IDMON 0x39;
 *   the ST7789's RAMCTRL 0xB0 (its second parameter's bit 3 sends a
 *   16-bit pixel's low byte first on an 8-bit bus). Other commands (power,
 *   gamma, frame rate and so on) take their parameters and change nothing
 *   the console shows; the read commands send nothing, with no RDX.
 *
 * Pixels (COLMOD's low nibble): 5, 16 bits, two bytes on an 8-bit bus
 * (R[4:0] G[5:3], then G[2:0] B[4:0]) or one word on a 16-bit bus; 6, 18
 * bits, three bytes on an 8-bit bus, each colour in a byte's bits 7:2; 3,
 * 12 bits, three bytes for two pixels on an 8-bit bus or D[11:0] on a
 * 16-bit bus. A 5- or 4-bit colour goes into GRAM's 6 bits with its top
 * bits repeated below it, as the chips' datasheets give.
 *
 * Writes fill the window column by column, then row by row, wrapping to
 * its start. MADCTL: MX reverses the column counter (counter value c
 * addresses column N - 1 - c of the N it spans), MY the row counter, and
 * MV exchanges them, so that the column counter addresses GRAM rows; BGR
 * swaps red and blue on the panel; ML and MH, the refresh order, change
 * nothing the console shows. Interpretation: the datasheets name MX and
 * MY "column/row address order" and give no formula; MX and MY are taken
 * to act on the counters before MV's exchange.
 *
 * The panel shows black while it sleeps or is off (as a normally-black
 * panel with its backlight on); INVON inverts what it shows; idle mode
 * shows each colour's top bit only. Not modelled: partial mode and
 * vertical scrolling (logged), the tearing effect output, brightness, and
 * the chips' RESX pins (SWRESET is): GRAM's contents at power-on are 0.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/lcd_panel.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"

#define GRAM_W  240
#define GRAM_H  320

#define TYPE_MIPI_DBI_PANEL "mipi-dbi-panel"
OBJECT_DECLARE_TYPE(MipiDbiPanel, MipiDbiPanelClass, MIPI_DBI_PANEL)

enum {
    CMD_NOP = 0x00,
    CMD_SWRESET = 0x01,
    CMD_SLPIN = 0x10,
    CMD_SLPOUT = 0x11,
    CMD_PTLON = 0x12,
    CMD_NORON = 0x13,
    CMD_INVOFF = 0x20,
    CMD_INVON = 0x21,
    CMD_DISPOFF = 0x28,
    CMD_DISPON = 0x29,
    CMD_CASET = 0x2a,
    CMD_RASET = 0x2b,
    CMD_RAMWR = 0x2c,
    CMD_VSCRDEF = 0x33,
    CMD_MADCTL = 0x36,
    CMD_VSCSAD = 0x37,
    CMD_IDMOFF = 0x38,
    CMD_IDMON = 0x39,
    CMD_COLMOD = 0x3a,
    CMD_RAMWRC = 0x3c,
    CMD_ST7789_RAMCTRL = 0xb0,
};

#define MADCTL_MY       0x80
#define MADCTL_MX       0x40
#define MADCTL_MV       0x20
#define MADCTL_BGR      0x08

#define RAMCTRL_LITTLE_ENDIAN 0x08

#define MAX_PARAMS      16

struct MipiDbiPanelClass {
    LCDPanelClass parent_class;

    const char *name;
    bool st7789;
};

struct MipiDbiPanel {
    LCDPanel parent_obj;

    QemuConsole *con;
    uint32_t width;
    uint32_t height;
    uint32_t data_width;

    /* GRAM, 6 bits a colour: R in bits 17:12, G 11:6, B 5:0 */
    uint32_t gram[GRAM_H * GRAM_W];

    uint8_t madctl;
    uint8_t colmod;
    uint8_t ramctrl[2];
    bool sleeping;
    bool display_on;
    bool inverted;
    bool idle;
    uint16_t xs, xe, ys, ye;
    /* The address counters, in MADCTL's column and row space */
    uint16_t col, row;

    /* The command taking parameters or pixels, or -1 */
    int32_t cmd;
    uint32_t nparams;
    uint8_t params[MAX_PARAMS];
    /* A pixel's bytes so far */
    uint8_t pix[3];
    uint32_t npix;

    /* What the console has not been shown yet, in GRAM coordinates */
    bool full_redraw;
    int32_t dirty_x0, dirty_y0, dirty_x1, dirty_y1;
};

static void dbi_mark_dirty(MipiDbiPanel *s, int x, int y)
{
    s->dirty_x0 = MIN(s->dirty_x0, x);
    s->dirty_y0 = MIN(s->dirty_y0, y);
    s->dirty_x1 = MAX(s->dirty_x1, x + 1);
    s->dirty_y1 = MAX(s->dirty_y1, y + 1);
}

static void dbi_clear_dirty(MipiDbiPanel *s)
{
    s->full_redraw = false;
    s->dirty_x0 = s->dirty_y0 = INT32_MAX;
    s->dirty_x1 = s->dirty_y1 = 0;
}

static void dbi_reset_regs(MipiDbiPanel *s)
{
    s->madctl = 0;
    s->colmod = 0x66;
    s->ramctrl[0] = 0x00;
    s->ramctrl[1] = 0xf0;
    s->sleeping = true;
    s->display_on = false;
    s->inverted = false;
    s->idle = false;
    s->xs = 0;
    s->xe = GRAM_W - 1;
    s->ys = 0;
    s->ye = GRAM_H - 1;
    s->col = 0;
    s->row = 0;
    s->cmd = -1;
    s->nparams = 0;
    s->npix = 0;
    s->full_redraw = true;
}

/* The column and row counters' ranges, which MV exchanges. */
static unsigned dbi_cols(MipiDbiPanel *s)
{
    return s->madctl & MADCTL_MV ? GRAM_H : GRAM_W;
}

static unsigned dbi_rows(MipiDbiPanel *s)
{
    return s->madctl & MADCTL_MV ? GRAM_W : GRAM_H;
}

static void dbi_put_pixel(MipiDbiPanel *s, uint32_t rgb666)
{
    unsigned c = s->col, r = s->row, gx, gy;

    if (s->xs <= s->xe && s->ys <= s->ye && c < dbi_cols(s) &&
        r < dbi_rows(s)) {
        if (s->madctl & MADCTL_MX) {
            c = dbi_cols(s) - 1 - c;
        }
        if (s->madctl & MADCTL_MY) {
            r = dbi_rows(s) - 1 - r;
        }
        gx = s->madctl & MADCTL_MV ? r : c;
        gy = s->madctl & MADCTL_MV ? c : r;
        s->gram[gy * GRAM_W + gx] = rgb666;
        dbi_mark_dirty(s, gx, gy);
    }
    if (s->col >= s->xe) {
        s->col = s->xs;
        s->row = s->row >= s->ye ? s->ys : s->row + 1;
    } else {
        s->col++;
    }
}

/* A colour of @bits bits in GRAM's 6, its top bits repeated below it. */
static uint32_t dbi_to6(uint32_t v, unsigned bits)
{
    return (v << (6 - bits) | v >> (2 * bits - 6)) & 0x3f;
}

static uint32_t dbi_rgb565(uint32_t v)
{
    return dbi_to6(v >> 11 & 0x1f, 5) << 12 | (v >> 5 & 0x3f) << 6 |
           dbi_to6(v & 0x1f, 5);
}

static uint32_t dbi_rgb444(uint32_t v)
{
    return dbi_to6(v >> 8 & 0xf, 4) << 12 | dbi_to6(v >> 4 & 0xf, 4) << 6 |
           dbi_to6(v & 0xf, 4);
}

static void dbi_pixel_data(MipiDbiPanel *s, uint32_t word)
{
    unsigned fmt = s->colmod & 7;
    MipiDbiPanelClass *k = MIPI_DBI_PANEL_GET_CLASS(s);

    if (s->data_width == 16) {
        switch (fmt) {
        case 5:
            dbi_put_pixel(s, dbi_rgb565(word & 0xffff));
            return;
        case 3:
            dbi_put_pixel(s, dbi_rgb444(word & 0xfff));
            return;
        default:
            qemu_log_mask(LOG_UNIMP, "%s: COLMOD 0x%02x on a 16-bit bus is "
                          "not modelled\n", k->name, s->colmod);
            return;
        }
    }
    s->pix[s->npix++] = word;
    switch (fmt) {
    case 5:
        if (s->npix == 2) {
            bool le = k->st7789 && (s->ramctrl[1] & RAMCTRL_LITTLE_ENDIAN);

            dbi_put_pixel(s, dbi_rgb565(le ? s->pix[1] << 8 | s->pix[0] :
                                        s->pix[0] << 8 | s->pix[1]));
            s->npix = 0;
        }
        break;
    case 6:
        if (s->npix == 3) {
            dbi_put_pixel(s, (s->pix[0] >> 2) << 12 | (s->pix[1] >> 2) << 6 |
                             s->pix[2] >> 2);
            s->npix = 0;
        }
        break;
    case 3:
        if (s->npix == 3) {
            uint32_t v = s->pix[0] << 16 | s->pix[1] << 8 | s->pix[2];

            dbi_put_pixel(s, dbi_rgb444(v >> 12));
            dbi_put_pixel(s, dbi_rgb444(v & 0xfff));
            s->npix = 0;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: pixel data with COLMOD 0x%02x, "
                      "which names no interface format\n", k->name,
                      s->colmod);
        s->npix = 0;
        break;
    }
}

static unsigned dbi_param_count(MipiDbiPanel *s, uint8_t cmd)
{
    switch (cmd) {
    case CMD_CASET:
    case CMD_RASET:
        return 4;
    case CMD_MADCTL:
    case CMD_COLMOD:
        return 1;
    case CMD_ST7789_RAMCTRL:
        return MIPI_DBI_PANEL_GET_CLASS(s)->st7789 ? 2 : 0;
    default:
        return 0;
    }
}

/* A command's parameters are in: act on them. */
static void dbi_params_done(MipiDbiPanel *s)
{
    uint8_t *p = s->params;

    switch (s->cmd) {
    case CMD_CASET:
        s->xs = p[0] << 8 | p[1];
        s->xe = p[2] << 8 | p[3];
        break;
    case CMD_RASET:
        s->ys = p[0] << 8 | p[1];
        s->ye = p[2] << 8 | p[3];
        break;
    case CMD_MADCTL:
        if ((s->madctl ^ p[0]) & MADCTL_BGR) {
            s->full_redraw = true;
        }
        s->madctl = p[0];
        break;
    case CMD_COLMOD:
        s->colmod = p[0];
        break;
    case CMD_ST7789_RAMCTRL:
        s->ramctrl[0] = p[0];
        s->ramctrl[1] = p[1];
        break;
    }
}

static void dbi_command(MipiDbiPanel *s, uint8_t cmd)
{
    MipiDbiPanelClass *k = MIPI_DBI_PANEL_GET_CLASS(s);

    s->cmd = cmd;
    s->nparams = 0;
    s->npix = 0;
    switch (cmd) {
    case CMD_SWRESET:
        dbi_reset_regs(s);
        break;
    case CMD_SLPIN:
    case CMD_SLPOUT:
        s->sleeping = cmd == CMD_SLPIN;
        s->full_redraw = true;
        break;
    case CMD_DISPOFF:
    case CMD_DISPON:
        s->display_on = cmd == CMD_DISPON;
        s->full_redraw = true;
        break;
    case CMD_INVOFF:
    case CMD_INVON:
        s->inverted = cmd == CMD_INVON;
        s->full_redraw = true;
        break;
    case CMD_IDMOFF:
    case CMD_IDMON:
        s->idle = cmd == CMD_IDMON;
        s->full_redraw = true;
        break;
    case CMD_RAMWR:
        s->col = s->xs;
        s->row = s->ys;
        break;
    case CMD_PTLON:
    case CMD_VSCRDEF:
    case CMD_VSCSAD:
        qemu_log_mask(LOG_UNIMP, "%s: command 0x%02x (partial mode or "
                      "scrolling) is not modelled\n", k->name, cmd);
        break;
    default:
        break;
    }
}

static void dbi_write(LCDPanel *p, const LCDDbiRun *run)
{
    MipiDbiPanel *s = MIPI_DBI_PANEL(p);
    uint32_t i;

    for (i = 0; i < run->count; i++) {
        uint32_t w = run->words[i];

        if (!run->dc) {
            dbi_command(s, w & 0xff);
            continue;
        }
        if (s->cmd == CMD_RAMWR || s->cmd == CMD_RAMWRC) {
            dbi_pixel_data(s, w);
        } else if (s->cmd >= 0 && s->nparams < dbi_param_count(s, s->cmd)) {
            s->params[s->nparams++] = w;
            if (s->nparams == dbi_param_count(s, s->cmd)) {
                dbi_params_done(s);
            }
        }
    }
}

static unsigned dbi_bus_width(LCDPanel *p)
{
    return MIPI_DBI_PANEL(p)->data_width;
}

/* What the panel shows at GRAM's (@x, @y), as a console pixel. */
static uint32_t dbi_shown(MipiDbiPanel *s, unsigned x, unsigned y)
{
    uint32_t v = s->gram[y * GRAM_W + x], r, g, b;

    if (s->sleeping || !s->display_on) {
        return 0;
    }
    r = v >> 12 & 0x3f;
    g = v >> 6 & 0x3f;
    b = v & 0x3f;
    if (s->madctl & MADCTL_BGR) {
        uint32_t t = r;

        r = b;
        b = t;
    }
    if (s->idle) {
        r = r & 0x20 ? 0x3f : 0;
        g = g & 0x20 ? 0x3f : 0;
        b = b & 0x20 ? 0x3f : 0;
    }
    if (s->inverted) {
        r ^= 0x3f;
        g ^= 0x3f;
        b ^= 0x3f;
    }
    return rgb_to_pixel32(r << 2 | r >> 4, g << 2 | g >> 4, b << 2 | b >> 4);
}

static bool dbi_update_display(void *opaque)
{
    MipiDbiPanel *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    int x0, y0, x1, y1, x, y;
    uint8_t *data;
    int stride;

    if (!surface) {
        return true;
    }
    if (s->full_redraw) {
        x0 = y0 = 0;
        x1 = GRAM_W;
        y1 = GRAM_H;
    } else {
        x0 = s->dirty_x0;
        y0 = s->dirty_y0;
        x1 = s->dirty_x1;
        y1 = s->dirty_y1;
    }
    x1 = MIN(x1, (int)s->width);
    y1 = MIN(y1, (int)s->height);
    dbi_clear_dirty(s);
    if (x0 >= x1 || y0 >= y1) {
        return true;
    }
    data = surface_data(surface);
    stride = surface_stride(surface);
    for (y = y0; y < y1; y++) {
        uint32_t *line = (uint32_t *)(data + y * stride);

        for (x = x0; x < x1; x++) {
            line[x] = dbi_shown(s, x, y);
        }
    }
    qemu_console_update(s->con, x0, y0, x1 - x0, y1 - y0);
    return true;
}

static void dbi_invalidate_display(void *opaque)
{
    MIPI_DBI_PANEL(opaque)->full_redraw = true;
}

static const GraphicHwOps dbi_ops = {
    .invalidate = dbi_invalidate_display,
    .gfx_update = dbi_update_display,
};

static void dbi_realize(DeviceState *dev, Error **errp)
{
    MipiDbiPanel *s = MIPI_DBI_PANEL(dev);

    if (!s->width || s->width > GRAM_W || !s->height ||
        s->height > GRAM_H) {
        error_setg(errp, "%s: the panel is at most %ux%u pixels, its GRAM",
                   MIPI_DBI_PANEL_GET_CLASS(s)->name, GRAM_W, GRAM_H);
        return;
    }
    if (s->data_width != 8 && s->data_width != 16) {
        error_setg(errp, "%s: bus-width is 8 or 16",
                   MIPI_DBI_PANEL_GET_CLASS(s)->name);
        return;
    }
    s->con = qemu_graphic_console_create(dev, 0, &dbi_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
}

static void dbi_reset(DeviceState *dev)
{
    MipiDbiPanel *s = MIPI_DBI_PANEL(dev);

    dbi_reset_regs(s);
    dbi_clear_dirty(s);
    s->full_redraw = true;
}

static int dbi_post_load(void *opaque, int version_id)
{
    MipiDbiPanel *s = opaque;

    if (s->nparams > MAX_PARAMS || s->npix > 3) {
        return -EINVAL;
    }
    s->full_redraw = true;
    return 0;
}

static const VMStateDescription vmstate_mipi_dbi_panel = {
    .name = TYPE_MIPI_DBI_PANEL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dbi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(gram, MipiDbiPanel, GRAM_H * GRAM_W),
        VMSTATE_UINT8(madctl, MipiDbiPanel),
        VMSTATE_UINT8(colmod, MipiDbiPanel),
        VMSTATE_UINT8_ARRAY(ramctrl, MipiDbiPanel, 2),
        VMSTATE_BOOL(sleeping, MipiDbiPanel),
        VMSTATE_BOOL(display_on, MipiDbiPanel),
        VMSTATE_BOOL(inverted, MipiDbiPanel),
        VMSTATE_BOOL(idle, MipiDbiPanel),
        VMSTATE_UINT16(xs, MipiDbiPanel),
        VMSTATE_UINT16(xe, MipiDbiPanel),
        VMSTATE_UINT16(ys, MipiDbiPanel),
        VMSTATE_UINT16(ye, MipiDbiPanel),
        VMSTATE_UINT16(col, MipiDbiPanel),
        VMSTATE_UINT16(row, MipiDbiPanel),
        VMSTATE_INT32(cmd, MipiDbiPanel),
        VMSTATE_UINT32(nparams, MipiDbiPanel),
        VMSTATE_UINT8_ARRAY(params, MipiDbiPanel, MAX_PARAMS),
        VMSTATE_UINT8_ARRAY(pix, MipiDbiPanel, 3),
        VMSTATE_UINT32(npix, MipiDbiPanel),
        VMSTATE_END_OF_LIST()
    },
};

static const Property dbi_properties[] = {
    DEFINE_PROP_UINT32("width", MipiDbiPanel, width, GRAM_W),
    DEFINE_PROP_UINT32("height", MipiDbiPanel, height, GRAM_H),
    DEFINE_PROP_UINT32("bus-width", MipiDbiPanel, data_width, 8),
};

static void dbi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    LCDPanelClass *lc = LCD_PANEL_CLASS(klass);

    dc->realize = dbi_realize;
    device_class_set_legacy_reset(dc, dbi_reset);
    dc->vmsd = &vmstate_mipi_dbi_panel;
    device_class_set_props(dc, dbi_properties);
    lc->bus_width = dbi_bus_width;
    lc->dbi_write = dbi_write;
}

static void ili9341_class_init(ObjectClass *klass, const void *data)
{
    MipiDbiPanelClass *k = MIPI_DBI_PANEL_CLASS(klass);

    DEVICE_CLASS(klass)->desc = "ILI9341 i80 LCD panel, 240x320";
    k->name = "ili9341";
}

static void st7789_class_init(ObjectClass *klass, const void *data)
{
    MipiDbiPanelClass *k = MIPI_DBI_PANEL_CLASS(klass);

    DEVICE_CLASS(klass)->desc = "ST7789 i80 LCD panel, up to 240x320";
    k->name = "st7789";
    k->st7789 = true;
}

static const TypeInfo mipi_dbi_panel_types[] = {
    {
        .name = TYPE_MIPI_DBI_PANEL,
        .parent = TYPE_LCD_PANEL,
        .instance_size = sizeof(MipiDbiPanel),
        .class_size = sizeof(MipiDbiPanelClass),
        .class_init = dbi_class_init,
        .abstract = true,
    },
    {
        .name = "ili9341",
        .parent = TYPE_MIPI_DBI_PANEL,
        .class_init = ili9341_class_init,
    },
    {
        .name = "st7789",
        .parent = TYPE_MIPI_DBI_PANEL,
        .class_init = st7789_class_init,
    },
};

DEFINE_TYPES(mipi_dbi_panel_types)
