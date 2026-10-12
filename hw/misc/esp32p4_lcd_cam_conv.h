/*
 * ESP32-P4 LCD_CAM's YUV-RGB converters
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LCD_CAM_CONV_H
#define HW_MISC_ESP32P4_LCD_CAM_CONV_H

#include "hw/misc/esp32p4_lcd_cam.h"

/*
 * A converter takes a byte stream (the LCD's stage one output, the
 * camera's received bytes) and gives one, through @emit. Its register
 * value is the LCD's LCD_RGB_YUV or the camera's CAM_RGB_YUV, @lcd
 * telling which; @wire24 is the LCD's 24-bit output (LCD_WIRE_MODE 2),
 * and @line_pixels the pixels a line has, for YUV420's line parity where
 * the caller does not mark lines itself (0 for none).
 */
typedef void ESP32P4LcdCamEmit(void *opaque, const uint8_t *bytes,
                               unsigned n);

void esp32p4_lcd_cam_conv_setup(ESP32P4LcdCamConv *c, uint32_t cfg,
                                bool lcd, bool wire24, uint32_t line_pixels);
/* Whether the converter changes the stream at all. */
bool esp32p4_lcd_cam_conv_on(const ESP32P4LcdCamConv *c);
/* Whether the settings name a conversion the TRM lists. */
bool esp32p4_lcd_cam_conv_valid(const ESP32P4LcdCamConv *c);
/* Drop what is partly taken, and start a frame. */
void esp32p4_lcd_cam_conv_reset(ESP32P4LcdCamConv *c);
void esp32p4_lcd_cam_conv_new_line(ESP32P4LcdCamConv *c);
void esp32p4_lcd_cam_conv_feed(ESP32P4LcdCamConv *c, const uint8_t *bytes,
                               unsigned n, ESP32P4LcdCamEmit *emit,
                               void *opaque);
/* Complete a partly taken group with zero bytes. */
void esp32p4_lcd_cam_conv_flush(ESP32P4LcdCamConv *c,
                                ESP32P4LcdCamEmit *emit, void *opaque);

extern const VMStateDescription vmstate_esp32p4_lcd_cam_conv;

#endif
