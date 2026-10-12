/*
 * Parallel LCD panels on an LCD controller's port: the bus and the panels
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_LCD_PANEL_H
#define HW_DISPLAY_LCD_PANEL_H

#include "hw/core/qdev.h"
#include "qom/object.h"

/*
 * A board's parallel LCD panels
 * =============================
 *
 * An LCD controller with a parallel port (the ESP32-P4's LCD_CAM) owns an
 * "lcd-bus", and the board's panels are devices on it:
 *
 *   -device st7789,bus=lcd           an i80 panel with its own GRAM
 *   -device rgb-panel,bus=lcd,width=800,height=480
 *
 * The bus carries what crosses the panel's pins, a run of bus cycles at a
 * time rather than edge by edge: the pins change at tens of MHz. The
 * controller works out what each panel's pins see from the board wiring
 * (the pins the matrix routes its signals to) and calls the panel's class
 * hooks at the time of each run's last cycle, esp32_line_time() where the
 * controller times its lines that way.
 *
 * i80 (Intel 8080, MIPI DBI type B, write only: the panel's RDX tied
 * high). Each WR strobe while CSX is active latches D[n-1:0] with D/CX
 * telling a command (low) from data (high). dbi_write() gives a run of
 * strobes with CSX active and D/CX steady: @words[k] is the panel's
 * D[bus_width - 1:0] at strobe k. Strobes while CSX is inactive reach no
 * hook.
 *
 * RGB (DPI). The panel samples D[n-1:0] at each PCLK while DE is active,
 * a line of pixels per DE pulse, and starts a frame at VSYNC's active
 * edge (DE mode: HSYNC and the porches are not used). dpi_vsync() gives
 * each VSYNC edge as the panel's pin sees it; dpi_line() each DE pulse,
 * as DE goes inactive, with the level DE had while the words were sent:
 * a panel whose DE polarity is the other way samples the blanking, not
 * these.
 *
 * The property "cs-pad" is the pad the board wires the panel's CSX to:
 * the panel is selected while that pad is low. With none (-1, the
 * default) the board ties CSX low, or wires it to wherever the controller
 * routes its chip select, if it does.
 */

#define TYPE_LCD_BUS "lcd-bus"
OBJECT_DECLARE_SIMPLE_TYPE(LCDBus, LCD_BUS)

#define TYPE_LCD_PANEL "lcd-panel"
OBJECT_DECLARE_TYPE(LCDPanel, LCDPanelClass, LCD_PANEL)

struct LCDBus {
    BusState parent_obj;
};

struct LCDPanel {
    DeviceState parent_obj;

    /* Board wiring: the pad CSX is on, or -1 */
    int32_t cs_pad;
};

typedef struct LCDDbiRun {
    /* D/CX: false for a command, true for data */
    bool dc;
    const uint32_t *words;
    uint32_t count;
    /* The first and last strobes' times */
    int64_t first_ns;
    int64_t last_ns;
} LCDDbiRun;

typedef struct LCDDpiLine {
    const uint32_t *words;
    uint32_t count;
    /* DE's level while the words were sent, as the panel's pin saw it */
    bool de_level;
    int64_t de_ns;
    int64_t de_end_ns;
} LCDDpiLine;

struct LCDPanelClass {
    DeviceClass parent_class;

    /* The data lines the panel takes, D0 upwards */
    unsigned (*bus_width)(LCDPanel *p);
    void (*dbi_write)(LCDPanel *p, const LCDDbiRun *run);
    void (*dpi_vsync)(LCDPanel *p, bool level, int64_t when_ns);
    void (*dpi_line)(LCDPanel *p, const LCDDpiLine *line);
};

LCDBus *lcd_bus_create(DeviceState *parent, const char *name);

/* Call @fn for each panel on @bus. */
void lcd_bus_foreach(LCDBus *bus, void (*fn)(LCDPanel *p, void *opaque),
                     void *opaque);

#endif
