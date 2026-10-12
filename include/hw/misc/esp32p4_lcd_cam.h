/*
 * ESP32-P4 LCD and camera controller (LCD_CAM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LCD_CAM_H
#define HW_MISC_ESP32P4_LCD_CAM_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/display/lcd_panel.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/misc/esp32p4_image_source.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LCD_CAM "esp32p4-lcd-cam"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LcdCamState, ESP32P4_LCD_CAM)

#define ESP32P4_LCD_CAM_BASE    0x500dc000
#define ESP32P4_LCD_CAM_SIZE    0x1000

/*
 * How LCD_CAM connects
 * ====================
 *
 *   ESP32P4_LCD_CAM_SIG_OUT, "sig-out" (output, ESP32P4_LCD_CAM_OUTS
 *     lines): the LCD's output signals, for the GPIO matrix's "sig-out"
 *     lines of the numbers esp32p4_lcd_cam_out_signal() gives: CS, DC,
 *     PCLK, HSYNC, VSYNC, DE (H_ENABLE), CAM_CLK and LCD_DATA_OUT0..23.
 *     CS, DC, HSYNC, VSYNC and DE change at their edges' times; the data
 *     lines take the last word of each i80 transaction and of each RGB
 *     frame, which the bus holds after it; PCLK holds its idle level and
 *     CAM_CLK stays low. Those two clocks, at up to tens of MHz, and the
 *     words on the data lines reach the board's panels as runs of bus
 *     cycles (include/hw/display/lcd_panel.h), and CAM_CLK the board's
 *     sensor as a Clock (below), not as edges on the pads.
 *   ESP32P4_LCD_CAM_SIG_IN, "sig-in" (input, ESP32P4_LCD_CAM_INS lines):
 *     the camera's input signals as the matrix delivers them (PCLK,
 *     VSYNC, HSYNC, DE, CAM_DATA_IN0..15, esp32p4_lcd_cam_in_signal()),
 *     whose levels the camera takes for the inputs no sensor signal
 *     reaches; the sensor's stream comes through the image source API.
 *   Clock inputs "lcd-clk" (HP_SYS_CLKRST's LCD_CLK) and "cam-clk" (its
 *     CAM_CLK, the camera module's clock and its master clock output to a
 *     sensor: a sensor whose XCLK the board wires to it takes the same
 *     clock).
 *   Links "gdma" (GDMA-AXI, which serves both directions as peripheral
 *     ESP32P4_GDMA_AXI_LCD_CAM), "gpio" (the matrix, for the routing)
 *     and "camera" (the board's DVP sensor, or none).
 *   The bus "lcd" carries the board's panels (lcd_panel.h).
 *   The interrupt is sysbus IRQ 0, LCD_CAM_INTR.
 */
#define ESP32P4_LCD_CAM_SIG_OUT "sig-out"
#define ESP32P4_LCD_CAM_SIG_IN  "sig-in"

enum {
    ESP32P4_LCD_CAM_OUT_CS,
    ESP32P4_LCD_CAM_OUT_DC,
    ESP32P4_LCD_CAM_OUT_PCLK,
    ESP32P4_LCD_CAM_OUT_HSYNC,
    ESP32P4_LCD_CAM_OUT_VSYNC,
    ESP32P4_LCD_CAM_OUT_DE,
    ESP32P4_LCD_CAM_OUT_CAM_CLK,
    ESP32P4_LCD_CAM_OUT_DATA0,
    ESP32P4_LCD_CAM_OUTS = ESP32P4_LCD_CAM_OUT_DATA0 + 24,
};

enum {
    ESP32P4_LCD_CAM_IN_PCLK,
    ESP32P4_LCD_CAM_IN_VSYNC,
    ESP32P4_LCD_CAM_IN_HSYNC,
    ESP32P4_LCD_CAM_IN_DE,
    ESP32P4_LCD_CAM_IN_DATA0,
    ESP32P4_LCD_CAM_INS = ESP32P4_LCD_CAM_IN_DATA0 + 16,
};

/* The GPIO matrix signal numbers of the outputs and inputs above. */
unsigned esp32p4_lcd_cam_out_signal(unsigned n);
unsigned esp32p4_lcd_cam_in_signal(unsigned n);

/* Words of a run of LCD cycles for the panels; words of an RGB line. */
#define ESP32P4_LCD_CAM_RUN     256
#define ESP32P4_LCD_CAM_LINE    4096
/* The LCD's queue of words ready to go out, and its delayed pad edges */
#define ESP32P4_LCD_CAM_QUEUE   512
#define ESP32P4_LCD_CAM_EDGES   8
/* The LCD clock domain's register copies, and the camera's */
#define ESP32P4_LCD_CAM_LCD_REGS 11
#define ESP32P4_LCD_CAM_CAM_REGS 3

/*
 * The data format path of one direction (TRM 40.3.5, 40.3.6), its
 * stages' partial groups of bytes: a byte stream in, a byte stream out
 * (hw/misc/esp32p4_lcd_cam_conv.c).
 */
typedef struct ESP32P4LcdCamConv {
    /* The register in effect, and the direction's own settings */
    uint32_t cfg;
    bool lcd;
    bool wire24;
    uint32_t line_pixels;
    /* 8-bit mode with 8BITS_DATA_INV: the first byte of a pair */
    uint8_t swap;
    bool swap_held;
    /* Bytes taken towards the next input group, and the line position */
    uint8_t in[6];
    uint32_t in_n;
    uint32_t pixel;
    uint32_t line;
    /* Pixels converted towards the next output group, Y U V or R G B */
    uint8_t pend[4][3];
    uint32_t pend_n;
    /* YUV420: the chroma the line before gave, a value a pixel pair */
    uint8_t chroma[ESP32P4_LCD_CAM_LINE / 2];
} ESP32P4LcdCamConv;

/* An RGB frame's shape, in PCLK cycles and lines, as it started. */
typedef struct ESP32P4LcdCamFrame {
    uint32_t line_cycles;
    uint32_t lines;
    uint32_t active_start;
    uint32_t active_cycles;
    uint32_t active_line;
    uint32_t active_lines;
    uint32_t hsync_pos;
    uint32_t hsync_cycles;
    uint32_t vsync_lines;
    bool hsync_blank;
} ESP32P4LcdCamFrame;

/* A pad edge due later, for an output its LCD_DLY_MODE delays. */
typedef struct ESP32P4LcdCamEdge {
    int64_t ns;
    uint8_t out;
    uint8_t level;
} ESP32P4LcdCamEdge;

struct ESP32P4LcdCamState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq sig_out[ESP32P4_LCD_CAM_OUTS];
    Clock *lcd_clk;
    Clock *cam_clk;
    ESP32P4GdmaState *gdma;
    ESP32P4GpioState *gpio;
    ESP32P4ImageSource *camera;
    LCDBus *bus;
    QEMUTimer lcd_timer;
    QEMUTimer cam_timer;
    Notifier route_notifier;
    Notifier line_source;

    /* Registers, as software wrote them (the APB side) */
    uint32_t lcd_clock;
    uint32_t cam_ctrl;
    uint32_t cam_ctrl1;
    uint32_t cam_rgb_yuv;
    uint32_t lcd_rgb_yuv;
    uint32_t lcd_user;
    uint32_t lcd_misc;
    uint32_t lcd_ctrl;
    uint32_t lcd_ctrl1;
    uint32_t lcd_ctrl2;
    uint32_t lcd_cmd_val[2];
    uint32_t lcd_dly_mode[2];
    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t date;

    /*
     * The LCD and camera clock domains' copies, which LCD_UPDATE and
     * CAM_UPDATE take: the LCD's LCD_CLOCK, LCD_RGB_YUV, LCD_USER,
     * LCD_MISC, LCD_CTRL, LCD_CTRL1, LCD_CTRL2, the two command values and
     * the two delay modes; the camera's CAM_CTRL, CAM_CTRL1, CAM_RGB_YUV.
     */
    uint32_t lcd_act[ESP32P4_LCD_CAM_LCD_REGS];
    uint32_t cam_act[ESP32P4_LCD_CAM_CAM_REGS];

    /*
     * LCD_CLK, counted exactly: its cycles since @lcd_count's origin are
     * cycles @lcd_count_base on of the controller's own count.
     */
    ESP32P4ClkCount lcd_count;
    uint64_t lcd_count_base;

    /*
     * The LCD engine. A transaction's or frame's PCLK cycle p starts at
     * LCD_CLK cycle @lcd_pos0 + p * @lcd_mo of the count, with the
     * configuration (@lcd_cfg) LCD_UPDATE's copy had as it started.
     */
    uint8_t lcd_state;
    uint64_t lcd_pos0;
    uint32_t lcd_mo;
    uint32_t lcd_cfg[ESP32P4_LCD_CAM_LCD_REGS];
    /* The next cycle position whose events are still to take */
    uint64_t lcd_pos;
    /* i80: each phase's first cycle, the phase in progress, DOUT's state */
    uint64_t i80_phase[6];
    uint8_t i80_ph;
    uint32_t i80_dout_sent;
    bool i80_stalled;
    /*
     * RGB: the frame's shape and count; the data line in progress, its
     * active cycles given words so far, whether it waits for GDMA and
     * whether it has underrun.
     */
    ESP32P4LcdCamFrame frame;
    uint32_t rgb_frames;
    uint32_t rgb_line;
    uint32_t rgb_fill;
    bool rgb_in_line;
    bool rgb_starved;
    bool rgb_underrun;
    /* A GDMA tx_ready that came during the engine's own run */
    bool tx_ready_pending;
    int64_t tx_ready_ns;
    /* The levels the LCD's outputs have, and the data lines' */
    bool out_level[ESP32P4_LCD_CAM_OUT_DATA0];
    uint32_t data_out;
    ESP32P4LcdCamEdge edges[ESP32P4_LCD_CAM_EDGES];
    uint32_t nedges;
    /* The last word out, which the lines hold in an underrun */
    uint32_t last_word;
    /*
     * Words ready to go out (the async TX FIFO): from @q_head, @q_num of
     * them; the @q_eof-th of those (1 the first) ends a GDMA frame, 0
     * for none.
     */
    uint32_t q[ESP32P4_LCD_CAM_QUEUE];
    uint32_t q_head;
    uint32_t q_num;
    uint32_t q_eof;
    /* Stage one's group, stage three's swizzle group and wire word */
    uint8_t s1[3];
    uint32_t s1_n;
    uint8_t s3_sw[3];
    uint32_t s3_sw_n;
    uint8_t s3_wire[3];
    uint32_t s3_wire_n;
    ESP32P4LcdCamConv lcd_conv;
    /* A run of words for the panels, and the RGB line's */
    uint32_t run[ESP32P4_LCD_CAM_RUN];
    uint32_t run_n;
    int64_t run_first_ns;
    uint32_t line[ESP32P4_LCD_CAM_LINE];
    bool in_lcd_run;

    /* The camera: lines counted for CAM_HS_INT, bytes for the EOF */
    uint32_t cam_lines;
    uint32_t cam_frame_lines;
    uint32_t cam_bytes;
    /* Whether the sensor's VSYNC is active */
    bool cam_vsync;
    int64_t cam_vsync_due;
    /* The async RX FIFO: bytes waiting for GDMA, and an EOF after them */
    uint8_t cam_fifo[16];
    uint32_t cam_fifo_n;
    bool cam_fifo_eof;
    bool cam_overflow_logged;
    ESP32P4LcdCamConv cam_conv;
    /* The levels of the camera's input lines from the matrix */
    uint32_t in_level;

    /*
     * The routing, as the matrix last had it: the first pad each LCD
     * output reaches (-1 for none) and whether the path inverts it; the
     * pad each camera input takes (-1 for none) and whether it inverts.
     */
    int8_t out_pad[ESP32P4_LCD_CAM_OUTS];
    bool out_inv[ESP32P4_LCD_CAM_OUTS];
    int8_t in_pad[ESP32P4_LCD_CAM_INS];
    bool in_inv[ESP32P4_LCD_CAM_INS];
    bool route_valid;
    /* Board mismatches already logged, a bit each */
    uint32_t logged;
};

#endif
