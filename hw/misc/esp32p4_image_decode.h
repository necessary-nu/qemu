/*
 * ESP32-P4 camera sensor: still pictures from PPM, PGM and PNG files
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_IMAGE_DECODE_H
#define HW_MISC_ESP32P4_IMAGE_DECODE_H

/* A pixel, each sample scaled to 16 bits. */
typedef struct ESP32P4Rgb16 {
    uint16_t r;
    uint16_t g;
    uint16_t b;
} ESP32P4Rgb16;

typedef struct ESP32P4Picture {
    uint32_t width;
    uint32_t height;
    /* width * height pixels, row by row from the top left */
    ESP32P4Rgb16 *pixels;
} ESP32P4Picture;

/*
 * Read a binary PPM (P6) or PGM (P5), any maxval, or a PNG of any colour
 * type and bit depth, interlaced or not; alpha and transparency are
 * ignored. The file's first bytes decide which.
 */
bool esp32p4_picture_load(const char *path, ESP32P4Picture *pic,
                          Error **errp);

/* @in resampled bilinearly to @width by @height, pixel centres aligned. */
void esp32p4_picture_scale(const ESP32P4Picture *in, uint32_t width,
                           uint32_t height, ESP32P4Picture *out);

void esp32p4_picture_free(ESP32P4Picture *pic);

#endif
