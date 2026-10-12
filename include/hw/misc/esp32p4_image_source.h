/*
 * ESP32-P4 camera sensor: a host image source for the camera inputs
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_IMAGE_SOURCE_H
#define HW_MISC_ESP32P4_IMAGE_SOURCE_H

#include "hw/core/irq.h"
#include "hw/core/qdev.h"
#include "qemu/host-utils.h"
#include "qemu/timer.h"
#include "qom/object.h"

/*
 * The board's camera sensor
 * =========================
 *
 * An esp32p4-image-source stands for a camera sensor on the board,
 * streaming frames in virtual time to whichever of the chip's camera
 * inputs the board wires it to: LCD_CAM's camera port (DVP, a parallel
 * bus of PCLK, VSYNC, HSYNC, DE and up to 16 data lines through the GPIO
 * matrix; the ISP's DVP input shares those signals) or MIPI CSI (a
 * D-PHY clock lane and one or two data lanes carrying CSI-2 packets).
 * It is a user-creatable object, wired to the machine by a machine
 * property:
 *
 *   -object esp32p4-image-source,id=cam0,format=raw10,width=1280,
 *           height=720,fps=30,image=photo.png
 *   -M esp32p4,csi-camera=cam0       (or dvp-camera=cam0)
 *
 * What it shows: "file", a file of whole frames, one after another, in
 * the declared format's byte layout (see "Formats"), played in a loop;
 * "image", a still picture (binary PPM or PGM, or PNG), scaled to the
 * frame's size and converted to the format (a Bayer mosaic for the RAW
 * formats); or, with neither, "pattern", a moving test pattern: "bars"
 * (the default), colour bars scrolling left, or "checker", a
 * checkerboard moving down and right, each with the frame number in the
 * top left corner (see "The test patterns").
 *
 * Formats
 * -------
 *
 * "format" names the pixels the sensor puts out, each with its CSI-2
 * data type; a line's bytes ("line_bytes") are laid out as CSI-2 sends
 * them, which is also the layout of a "file"'s frames:
 *
 *   raw8      0x2A  1 byte a pixel
 *   raw10     0x2B  4 pixels in 5 bytes: each pixel's bits 9:2, then
 *                   one byte of their bits 1:0, the first pixel's lowest
 *   raw12     0x2C  2 pixels in 3 bytes: bits 11:4 of each, then one
 *                   byte of their bits 3:0, the first pixel's lowest
 *   rgb565    0x22  2 bytes, little-endian: G[2:0] B[4:0], R[4:0] G[5:3]
 *                   ("rgb565le" too)
 *   rgb565be  0x22  2 bytes, big-endian
 *   rgb888    0x24  R, G, B
 *   bgr888    0x24  B, G, R: CSI-2's own RGB888 order
 *   yuv422    0x1E  U0 Y0 V0 Y1 for each pair of pixels ("uyvy" too),
 *                   CSI-2's YUV422 8-bit order
 *   yuyv, vyuy, yvyu 0x1E  the other orders a DVP sensor can give
 *
 * The RAW formats are the sensor's colour filter array, "bayer" naming
 * its first two lines' first two pixels (rggb, grbg, gbrg or bggr;
 * default rggb). A still picture or a pattern is converted from RGB:
 * each sample to n bits as round(v * (2^n - 1) / 65535) from 16 bits;
 * YUV with "yuv-matrix" (bt601, the default, or bt709) and "yuv-range"
 * (full, the default, or limited), U and V the rounded mean of the
 * pair's two pixels. Widths: raw10 a multiple of 4; raw12 and the YUV
 * formats of 2.
 *
 * Timing
 * ------
 *
 * A frame is "vts" lines of "hts" pixel periods each, at the pixel clock;
 * "width" by "height" of them carry pixels and the rest is blanking:
 *
 *   hts = width + hblank         vts = height + vblank
 *   pixel clock = fps * hts * vts (or "pixel-clock", and then
 *                 fps = pixel clock / (hts * vts))
 *
 * "hblank" (default width / 4, at least 16) and "vblank" (default
 * height / 16 rounded up, at least 3), or "hts" and "vts" in their place,
 * give the blanking; "fps" (default 30) or "pixel-clock" (Hz) the rate.
 * Line l of a frame starts with an HSYNC pulse "hsync" pixel periods long
 * (default hblank / 8, at least 1), and its pixels follow "hbp" pixel
 * periods later (default hblank / 4); the frame starts with a VSYNC
 * pulse "vsync" lines long (default vblank / 10, at least 1), and its
 * first line of pixels is "vbp" lines later (default vblank / 2). As on
 * a sensor, these are times on the sensor's own clock, an oscillator of
 * its own: it does not follow the chip's clocks (an XCLK from the chip
 * is not modelled).
 *
 * Every edge is at an exact time: clock edge n after the stream started
 * is at the first whole ns at or after n periods, and every time in the
 * structures below is such an edge, so a receiver never accumulates
 * rounding. The stream's first frame starts as the stream starts.
 *
 * DVP
 * ---
 *
 * On DVP the clock is PCLK, one sample of the data lines D[15:0] a
 * cycle: a RAW pixel is one sample, its value on D[n-1:0]; the other
 * formats go a byte a PCLK ("dvp-width=8", the default), their bytes in
 * line_bytes order, or two bytes a PCLK ("dvp-width=16", RGB565 and YUV
 * only), the earlier byte on D[7:0]. PCLK runs at the pixel clock times
 * PCLKs a pixel, and the horizontal timing above, in pixel periods, is
 * in PCLKs times that. VSYNC is active for the first "vsync" lines of
 * each frame, HSYNC for the first "hsync" pixel periods of every line,
 * and DE (HREF) while the samples of a line of pixels are on the bus;
 * "vsync-active-low", "hsync-active-low" and "de-active-low" give their
 * polarities (default active high).
 *
 * The data reach receivers through the API below, a line at a time,
 * never as pad levels: the bus changes at up to tens of MHz, too fast to
 * carry edge by edge. Which pads the board wires the sensor's signals to
 * is "pclk-pad", "vsync-pad", "hsync-pad", "de-pad" and "data-pads" (D0
 * upwards, a list such as "10,11,12,13,14,15,16,17"; -1 for an unwired
 * line); a receiver (LCD_CAM's camera port, the ISP's DVP input) reads
 * esp32p4_image_source_dvp_pad() and the GPIO matrix's routing
 * (esp32p4_gpio_sig_in_pad()) to see which sensor signal reaches each of
 * its CAM_* inputs. A signal with no pad given reaches the receiver
 * through whatever pad the matrix routes to it, as if the board wired
 * the sensor there. With its pad given, VSYNC, HSYNC and DE are also
 * driven onto that pad at their edges' times while the sensor is
 * powered (so a GPIO interrupt, PCNT or ETM can watch them), and
 * released while it is powered down or held in reset.
 *
 * MIPI CSI
 * --------
 *
 * On CSI each frame is a frame start short packet (FS), at the frame's
 * start; each line of pixels is a long packet of line_bytes, its data
 * type the format's, at the start of its line, between line start and
 * line end short packets (LS, LE) with "line-sync"; and a frame end short
 * packet (FE) after the last line. Each packet is a high-speed burst of
 * its own on "lanes" data lanes (default 2; 1 to 4) at "lane-rate" bits
 * a second on each lane (default: the lowest multiple of 10 Mbit/s from
 * 80 Mbit/s that keeps the line's bursts within 80% of a line), the
 * packet's bytes dealt to the lanes in turn. The bursts follow D-PHY 1.1
 * with T_LPX 50 ns, T_HS-PREPARE 50 ns + 5 UI, T_HS-ZERO 100 ns + 6 UI,
 * the 8-UI sync sequence, the bytes, T_HS-TRAIL max(8 UI, 60 ns + 4 UI)
 * and at least T_HS-EXIT 100 ns in LP-11 before the next. The packets'
 * virtual channel is "vc" (0 to 3); FS and FE carry frame numbers 1 to
 * 65535 then 1 again, LS and LE line numbers from 1 in each frame.
 * "clock-continuous" (default on) says whether the clock lane stays in
 * high speed between bursts.
 *
 * Streaming
 * ---------
 *
 * A sensor streams once software has configured it over SCCB (I2C);
 * SCCB is not modelled. The stream runs while "streaming" is on (the
 * default: the sensor streams from power-on), the sensor is powered
 * ("pwdn-pad", the board's PWDN line, active high unless
 * "pwdn-active-low") and out of reset ("reset-pad", its RESETB line,
 * active low unless "reset-active-high"). So a guest starts and stops
 * the stream as esp_cam_sensor's drivers drive their pwdn_pin and
 * reset_pin, with the GPIO pads given; "streaming", settable with
 * qom-set while the machine runs, and esp32p4_image_source_set_streaming()
 * below (for an SCCB model) are its software stream on and off. A
 * software stream off ends the stream after the frame in progress, as
 * a sensor's standby does; power-down and reset end it at once. Each
 * start begins a new frame at once, its timing counted from then.
 *
 * The test patterns
 * -----------------
 *
 * "bars" are the eight 100% colour bars (white, yellow, cyan, green,
 * magenta, red, blue, black) across the width, scrolled left by step =
 * width / 64 pixels (at least 1) a frame: pixel x of frame f shows bar
 * ((x + f * step) mod width) * 8 / width.
 * "checker" is white and black squares of "cell" pixels (default 16),
 * moving one pixel down and right a frame. Over either, frame number f
 * (counted since the source was created) is drawn in its top left corner
 * as 16 squares of "cell" pixels, bit 15 first, white for a 1 and black
 * for a 0, as many as fit in the width.
 */

#define TYPE_ESP32P4_IMAGE_SOURCE "esp32p4-image-source"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ImageSource, ESP32P4_IMAGE_SOURCE)

#define TYPE_ESP32P4_IMAGE_PROBE "esp32p4-image-probe"

typedef enum ESP32P4ImageFormat {
    ESP32P4_IMAGE_RAW8,
    ESP32P4_IMAGE_RAW10,
    ESP32P4_IMAGE_RAW12,
    ESP32P4_IMAGE_RGB565,
    ESP32P4_IMAGE_RGB565BE,
    ESP32P4_IMAGE_RGB888,
    ESP32P4_IMAGE_BGR888,
    ESP32P4_IMAGE_UYVY,
    ESP32P4_IMAGE_YUYV,
    ESP32P4_IMAGE_VYUY,
    ESP32P4_IMAGE_YVYU,
    ESP32P4_IMAGE_FORMATS,
} ESP32P4ImageFormat;

/* The colour of the first two lines' first two pixels, in that order. */
typedef enum ESP32P4ImageBayer {
    ESP32P4_IMAGE_BAYER_RGGB,
    ESP32P4_IMAGE_BAYER_GRBG,
    ESP32P4_IMAGE_BAYER_GBRG,
    ESP32P4_IMAGE_BAYER_BGGR,
} ESP32P4ImageBayer;

/* The CSI-2 data types the source sends */
#define ESP32P4_CSI2_DT_FS      0x00
#define ESP32P4_CSI2_DT_FE      0x01
#define ESP32P4_CSI2_DT_LS      0x02
#define ESP32P4_CSI2_DT_LE      0x03
#define ESP32P4_CSI2_DT_YUV422  0x1e
#define ESP32P4_CSI2_DT_RGB565  0x22
#define ESP32P4_CSI2_DT_RGB888  0x24
#define ESP32P4_CSI2_DT_RAW8    0x2a
#define ESP32P4_CSI2_DT_RAW10   0x2b
#define ESP32P4_CSI2_DT_RAW12   0x2c

typedef enum ESP32P4ImagePort {
    ESP32P4_IMAGE_PORT_NONE,
    ESP32P4_IMAGE_PORT_DVP,
    ESP32P4_IMAGE_PORT_CSI,
} ESP32P4ImagePort;

/* The sensor's DVP signals, for esp32p4_image_source_dvp_pad(). */
typedef enum ESP32P4DvpSignal {
    ESP32P4_DVP_PCLK,
    ESP32P4_DVP_VSYNC,
    ESP32P4_DVP_HSYNC,
    ESP32P4_DVP_DE,
    ESP32P4_DVP_D0,
    ESP32P4_DVP_SIGNALS = ESP32P4_DVP_D0 + 16,
} ESP32P4DvpSignal;

/*
 * The stream's format and timing, fixed once the source is bound to its
 * port. Horizontal times are in cycles of the line's clock: PCLK on DVP,
 * the pixel clock on CSI.
 */
typedef struct ESP32P4ImageTiming {
    ESP32P4ImagePort port;
    ESP32P4ImageFormat format;
    ESP32P4ImageBayer bayer;
    /* Pixels a line and lines a frame */
    uint32_t width;
    uint32_t height;
    /* The format's CSI-2 data type, and a line's bytes in its layout */
    uint8_t data_type;
    uint32_t line_bytes;
    /* The pixel clock, and the line's clock: PCLK on DVP */
    uint64_t pixel_hz;
    uint64_t clock_hz;
    /* Clocks a line; HSYNC's length; the first pixel's clock, and pixels' */
    uint32_t line_clocks;
    uint32_t hsync_clocks;
    uint32_t active_start;
    uint32_t active_clocks;
    /* Lines a frame; VSYNC's length; the first line of pixels */
    uint32_t frame_lines;
    uint32_t vsync_lines;
    uint32_t active_line;

    /* DVP: PCLKs a pixel, the data lines the sensor drives, polarities */
    uint32_t dvp_clocks;
    uint32_t dvp_bits;
    bool vsync_active_low;
    bool hsync_active_low;
    bool de_active_low;

    /* CSI */
    uint32_t lanes;
    uint64_t lane_bps;
    uint8_t vc;
    bool line_sync;
    bool clock_continuous;
} ESP32P4ImageTiming;

/*
 * A DVP line of pixels, given to the receivers as DE goes inactive:
 * samples[k] is on D[15:0] for PCLK cycle clock + k, from that cycle's
 * edge to the next (esp32p4_dvp_sample_ns()). The pointer is valid
 * during the call only.
 */
typedef struct ESP32P4DvpLine {
    /* Frames since the source was created, and the line, 0 the first */
    uint32_t frame;
    uint32_t line;
    const uint16_t *samples;
    uint32_t count;
    /* This line's HSYNC active and inactive edges, and DE's */
    int64_t hsync_ns;
    int64_t hsync_end_ns;
    int64_t de_ns;
    int64_t de_end_ns;
    /* The stream's start, PCLK 0; the first sample's PCLK; PCLK */
    int64_t epoch_ns;
    uint64_t clock;
    uint64_t pclk_hz;
} ESP32P4DvpLine;

/*
 * A CSI-2 packet, given to the receivers as its last byte arrives. A
 * short packet (FS, FE, LS, LE) has its 16-bit data field in wc and no
 * payload; a long packet wc bytes of payload and their CRC. ecc is the
 * header's ECC byte as sent. The burst: the data lanes leave LP-11 at
 * sot_ns, the packet's first byte (after the sync sequence) starts at
 * data_ns, its last byte has arrived at end_ns, and the lanes are back
 * in LP-11 at eot_ns. The pointer is valid during the call only.
 */
typedef struct ESP32P4CsiPacket {
    uint32_t frame;
    /* The line of pixels, 0 the first; 0 for FS and FE */
    uint32_t line;
    uint8_t vc;
    uint8_t dt;
    uint16_t wc;
    uint8_t ecc;
    const uint8_t *payload;
    uint16_t crc;
    int64_t sot_ns;
    int64_t data_ns;
    int64_t end_ns;
    int64_t eot_ns;
} ESP32P4CsiPacket;

/*
 * A receiver's callbacks, each called at its event's virtual time, with
 * that time as esp32_line_time() (hw/gpio/esp32_line.h): a callback runs
 * late only when QEMU's timer does, and then sees the event's own time.
 */
typedef struct ESP32P4ImageSinkOps {
    /* The stream starts (on) or stops (off) at @when_ns. */
    void (*stream)(void *opaque, bool on, int64_t when_ns);
    /* DVP: VSYNC's active (start of @frame) or inactive edge */
    void (*vsync)(void *opaque, uint32_t frame, bool active, int64_t when_ns);
    /* DVP: a line of pixels, as DE goes inactive */
    void (*dvp_line)(void *opaque, const ESP32P4DvpLine *line);
    /* CSI: a packet, as its last byte arrives */
    void (*csi_packet)(void *opaque, const ESP32P4CsiPacket *packet);
} ESP32P4ImageSinkOps;

/*
 * The board wires the sensor to @port: the machine does, from its
 * dvp-camera or csi-camera property, before receivers attach. A sensor is
 * wired to one port; this checks that the format and timing work on it
 * and, if the sensor is streaming, starts the stream now.
 */
bool esp32p4_image_source_bind(ESP32P4ImageSource *s, ESP32P4ImagePort port,
                               Error **errp);

/*
 * A receiver on the port the source is bound to listens from now on.
 * Several can listen to one sensor: the DVP signals reach LCD_CAM's
 * camera port and the ISP's DVP input alike.
 */
void esp32p4_image_source_attach(ESP32P4ImageSource *s,
                                 const ESP32P4ImageSinkOps *ops,
                                 void *opaque);
void esp32p4_image_source_detach(ESP32P4ImageSource *s, void *opaque);

const ESP32P4ImageTiming *esp32p4_image_source_timing(ESP32P4ImageSource *s);

/* Whether the stream is running now. */
bool esp32p4_image_source_streaming(ESP32P4ImageSource *s);

/*
 * The sensor's software stream on and off, for an SCCB model: off ends
 * the stream after the frame in progress, on starts it now.
 */
void esp32p4_image_source_set_streaming(ESP32P4ImageSource *s, bool on);

/*
 * The time of clock @clock of line @line (0 to frame_lines - 1, blanking
 * included) of frame @frame of the stream running now: line @line's
 * HSYNC is at clock 0, its first pixel at active_start. @clock may run
 * past the line into the following ones.
 */
int64_t esp32p4_image_source_clock_ns(ESP32P4ImageSource *s, uint32_t frame,
                                      uint32_t line, uint64_t clock);

/* The pad the board wires DVP signal @sig to, or -1. */
int esp32p4_image_source_dvp_pad(ESP32P4ImageSource *s, ESP32P4DvpSignal sig);

/*
 * Board wiring, for the machine: drive VSYNC, HSYNC and DE onto their
 * pads, and watch PWDN and RESETB, through the GPIO block's "pad-in",
 * "pad-release" and "pad" lines.
 */
void esp32p4_image_source_connect_pads(ESP32P4ImageSource *s,
                                       DeviceState *gpio);

/* The time of sample @k of @line. */
static inline int64_t esp32p4_dvp_sample_ns(const ESP32P4DvpLine *line,
                                            uint32_t k)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, line->clock + k, NANOSECONDS_PER_SECOND);
    /* Rounded up: the first whole ns the edge has come by. */
    if (divu128(&lo, &hi, line->pclk_hz)) {
        lo++;
    }
    return line->epoch_ns + lo;
}

/*
 * The CSI-2 packet header's ECC over its 24 bits (data identifier in
 * bits 7:0, then the 16-bit word count or data field), and the payload's
 * CRC-16 (x^16 + x^12 + x^5 + 1, seed 0xFFFF, bits least significant
 * first), as the receivers check them.
 */
uint8_t esp32p4_csi2_ecc(uint32_t header);
uint16_t esp32p4_csi2_crc16(const uint8_t *data, size_t len);

#endif
