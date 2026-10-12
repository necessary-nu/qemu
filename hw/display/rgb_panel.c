/*
 * RGB (DPI) LCD panel
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A TFT panel on an LCD controller's RGB port (include/hw/display/
 * lcd_panel.h), "width" by "height" pixels (default 800x480), in DE mode:
 * it starts a frame at VSYNC's active edge (low, or high with
 * "vsync-active-high") and takes each DE pulse (DE active high, or low
 * with "de-active-low") as the next row from the top, its pixels from the
 * left at one a PCLK. "bus-width" is its data interface: 16 (default),
 * RGB565 on D[15:0] (red in D[15:11]); 24, RGB888 (red in D[23:16]); or 8,
 * serial RGB, a pixel's red, green and blue on D[7:0] in three PCLKs.
 * Pixels beyond the panel's width in a row, and rows beyond its height,
 * fall off its edge; a row cut short leaves the rest of it as it was.
 *
 * The panel has no frame memory of its own: it shows each frame as the
 * controller scans it out, and the console shows a frame once its last
 * row has come (or, cut short, at the next VSYNC), as a whole. "frames"
 * counts the frames shown.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/lcd_panel.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"

#define TYPE_RGB_PANEL "rgb-panel"
OBJECT_DECLARE_SIMPLE_TYPE(RgbPanel, RGB_PANEL)

#define RGB_PANEL_MAX_SIDE 4096

struct RgbPanel {
    LCDPanel parent_obj;

    QemuConsole *con;
    uint32_t width;
    uint32_t height;
    uint32_t data_width;
    bool vsync_high;
    bool de_low;

    /* The frame coming in, and the one shown, as console pixels */
    uint32_t npixels;
    uint32_t *back;
    uint32_t *front;
    /* The next row; whether a VSYNC has started a frame; rows since */
    uint32_t row;
    bool in_frame;
    bool rows_pending;
    uint32_t frames;
    bool de_logged;
};

static void rgb_panel_show(RgbPanel *s)
{
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint32_t y;

    if (!surface) {
        return;
    }
    for (y = 0; y < s->height; y++) {
        memcpy(surface_data(surface) + y * surface_stride(surface),
               s->front + y * s->width, s->width * sizeof(uint32_t));
    }
    qemu_console_update_full(s->con);
}

static void rgb_panel_present(RgbPanel *s)
{
    memcpy(s->front, s->back, s->npixels * sizeof(uint32_t));
    s->rows_pending = false;
    s->frames++;
    rgb_panel_show(s);
}

static void rgb_panel_vsync(LCDPanel *p, bool level, int64_t when_ns)
{
    RgbPanel *s = RGB_PANEL(p);

    if (level != s->vsync_high) {
        return;
    }
    if (s->rows_pending) {
        rgb_panel_present(s);
    }
    s->row = 0;
    s->in_frame = true;
}

static void rgb_panel_line(LCDPanel *p, const LCDDpiLine *line)
{
    RgbPanel *s = RGB_PANEL(p);
    uint32_t *out, x, i;

    if (line->de_level == s->de_low) {
        if (!s->de_logged) {
            qemu_log_mask(LOG_GUEST_ERROR, "rgb-panel: DE reaches the panel "
                          "inverted: it samples the blanking, which the "
                          "model does not carry, and shows nothing\n");
            s->de_logged = true;
        }
        return;
    }
    if (!s->in_frame || s->row >= s->height) {
        return;
    }
    out = s->back + s->row * s->width;
    switch (s->data_width) {
    case 8:
        for (x = 0, i = 0; x < s->width && i + 2 < line->count; x++, i += 3) {
            out[x] = rgb_to_pixel32(line->words[i] & 0xff,
                                    line->words[i + 1] & 0xff,
                                    line->words[i + 2] & 0xff);
        }
        break;
    case 24:
        for (x = 0; x < s->width && x < line->count; x++) {
            uint32_t w = line->words[x];

            out[x] = rgb_to_pixel32(w >> 16 & 0xff, w >> 8 & 0xff, w & 0xff);
        }
        break;
    default:
        for (x = 0; x < s->width && x < line->count; x++) {
            uint32_t w = line->words[x], r = w >> 11 & 0x1f;
            uint32_t g = w >> 5 & 0x3f, b = w & 0x1f;

            out[x] = rgb_to_pixel32(r << 3 | r >> 2, g << 2 | g >> 4,
                                    b << 3 | b >> 2);
        }
        break;
    }
    s->rows_pending = true;
    if (++s->row == s->height) {
        rgb_panel_present(s);
        s->in_frame = false;
    }
}

static unsigned rgb_panel_bus_width(LCDPanel *p)
{
    return RGB_PANEL(p)->data_width;
}

static bool rgb_panel_update_display(void *opaque)
{
    return true;
}

static void rgb_panel_invalidate_display(void *opaque)
{
    rgb_panel_show(opaque);
}

static const GraphicHwOps rgb_panel_ops = {
    .invalidate = rgb_panel_invalidate_display,
    .gfx_update = rgb_panel_update_display,
};

static void rgb_panel_realize(DeviceState *dev, Error **errp)
{
    RgbPanel *s = RGB_PANEL(dev);

    if (!s->width || s->width > RGB_PANEL_MAX_SIDE || !s->height ||
        s->height > RGB_PANEL_MAX_SIDE) {
        error_setg(errp, "rgb-panel: width and height are 1 to %u",
                   RGB_PANEL_MAX_SIDE);
        return;
    }
    if (s->data_width != 8 && s->data_width != 16 && s->data_width != 24) {
        error_setg(errp, "rgb-panel: bus-width is 8, 16 or 24");
        return;
    }
    s->npixels = s->width * s->height;
    s->back = g_new0(uint32_t, s->npixels);
    s->front = g_new0(uint32_t, s->npixels);
    s->con = qemu_graphic_console_create(dev, 0, &rgb_panel_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
}

static void rgb_panel_unrealize(DeviceState *dev)
{
    RgbPanel *s = RGB_PANEL(dev);

    qemu_graphic_console_close(s->con);
    g_free(s->back);
    g_free(s->front);
}

static void rgb_panel_reset(DeviceState *dev)
{
    RgbPanel *s = RGB_PANEL(dev);

    s->row = 0;
    s->in_frame = false;
    s->rows_pending = false;
}

static int rgb_panel_post_load(void *opaque, int version_id)
{
    RgbPanel *s = opaque;

    rgb_panel_show(s);
    return 0;
}

static const VMStateDescription vmstate_rgb_panel = {
    .name = TYPE_RGB_PANEL,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rgb_panel_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_EQUAL(npixels, RgbPanel),
        VMSTATE_VARRAY_UINT32(back, RgbPanel, npixels, 0,
                              vmstate_info_uint32, uint32_t),
        VMSTATE_VARRAY_UINT32(front, RgbPanel, npixels, 0,
                              vmstate_info_uint32, uint32_t),
        VMSTATE_UINT32(row, RgbPanel),
        VMSTATE_BOOL(in_frame, RgbPanel),
        VMSTATE_BOOL(rows_pending, RgbPanel),
        VMSTATE_UINT32(frames, RgbPanel),
        VMSTATE_END_OF_LIST()
    },
};

static void rgb_panel_get_frames(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    uint32_t frames = RGB_PANEL(obj)->frames;

    visit_type_uint32(v, name, &frames, errp);
}

static void rgb_panel_init(Object *obj)
{
    object_property_add(obj, "frames", "uint32", rgb_panel_get_frames,
                        NULL, NULL, NULL);
}

static const Property rgb_panel_properties[] = {
    DEFINE_PROP_UINT32("width", RgbPanel, width, 800),
    DEFINE_PROP_UINT32("height", RgbPanel, height, 480),
    DEFINE_PROP_UINT32("bus-width", RgbPanel, data_width, 16),
    DEFINE_PROP_BOOL("vsync-active-high", RgbPanel, vsync_high, false),
    DEFINE_PROP_BOOL("de-active-low", RgbPanel, de_low, false),
};

static void rgb_panel_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    LCDPanelClass *lc = LCD_PANEL_CLASS(klass);

    dc->desc = "RGB (DPI) LCD panel, DE mode";
    dc->realize = rgb_panel_realize;
    dc->unrealize = rgb_panel_unrealize;
    device_class_set_legacy_reset(dc, rgb_panel_reset);
    dc->vmsd = &vmstate_rgb_panel;
    device_class_set_props(dc, rgb_panel_properties);
    lc->bus_width = rgb_panel_bus_width;
    lc->dpi_vsync = rgb_panel_vsync;
    lc->dpi_line = rgb_panel_line;
}

static const TypeInfo rgb_panel_types[] = {
    {
        .name = TYPE_RGB_PANEL,
        .parent = TYPE_LCD_PANEL,
        .instance_size = sizeof(RgbPanel),
        .instance_init = rgb_panel_init,
        .class_init = rgb_panel_class_init,
    },
};

DEFINE_TYPES(rgb_panel_types)
