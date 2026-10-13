/*
 * Everest Semiconductor ES8311 low power mono audio codec
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_ES8311_H
#define HW_AUDIO_ES8311_H

#include "hw/i2c/i2c.h"
#include "hw/core/clock.h"
#include "qemu/audio.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ES8311 "es8311"
OBJECT_DECLARE_SIMPLE_TYPE(ES8311State, ES8311)

/*
 * How a board connects the codec
 * ===============================
 *
 *   - Its I2C control port (CCLK, CDATA): an i2c-slave at 0x18, or 0x19
 *     with property "ce" (the CE pin strapped high).
 *   - Clock input "mclk": the clock on its MCLK pin (stopped: none).
 *   - Its serial port's pins, as levels timed the way the ESP32 boards'
 *     lines are (hw/gpio/esp32_line.h): inputs "sclk", "lrck" and
 *     "dsdin"; outputs "sclk-out", "lrck-out" and "asdout", each with its
 *     enable, "sclk-oe", "lrck-oe" and "asdout-oe" (low: the pin is
 *     released).
 *     SCLK and LRCK are outputs in master mode only. Output "pull-up":
 *     high while the codec's own pull-ups on SCLK and LRCK are on
 *     (PULLUP_SE clear, the reset state).
 *   - A clock source that does not toggle the pins edge by edge (an SoC's
 *     I2S master modelled a frame at a time) runs the serial port with
 *     es8311_sclk_run(), es8311_sclk_fall() and es8311_sclk_rise() below.
 *   - Input "out-path": high while the board passes the analog output,
 *     OUTP-OUTN, to its speaker (an amplifier's enable).
 *   - The audio backend (property "audiodev"): playback is what the
 *     board's speaker plays, the analog output while "out-path" is high
 *     and silence otherwise; capture is the analog input, MIC1P-MIC1N.
 *     Both are mono, at the sample rate of LRCK, as a fraction of the
 *     converters' full scale (the DAC's 0.9 * AVDD / 3.3 Vrms output, the
 *     ADC's AVDD / 3.3 Vrms input).
 */
#define ES8311_ADDR             0x18

#define ES8311_SCLK             "sclk"
#define ES8311_LRCK             "lrck"
#define ES8311_DSDIN            "dsdin"
#define ES8311_SCLK_OUT         "sclk-out"
#define ES8311_LRCK_OUT         "lrck-out"
#define ES8311_ASDOUT           "asdout"
#define ES8311_SCLK_OE          "sclk-oe"
#define ES8311_LRCK_OE          "lrck-oe"
#define ES8311_ASDOUT_OE        "asdout-oe"
#define ES8311_PULL_UP          "pull-up"
#define ES8311_OUT_PATH         "out-path"

#define ES8311_NREGS            0x100
/* Frames buffered between the converters and the audio backend */
#define ES8311_AUDIO_FRAMES     8192

/* One direction of the serial data port, SDP_IN or SDP_OUT */
typedef struct ES8311Sdp {
    /* LRCK as last sampled, and whether it has been */
    bool lrck;
    bool synced;
    /* Cycles until the next word's first bit (-1: none due), its channel */
    int32_t pending;
    uint8_t pending_chan;
    /* The word being shifted: its channel, its bits so far, the word */
    bool active;
    uint8_t chan;
    uint8_t bits;
    uint32_t word;
} ES8311Sdp;

/* A gain that moves in 0.25 dB steps: a volume, the ALC's, the DRC's */
typedef struct ES8311Gain {
    /* In 0.25 dB steps from -95.5 dB */
    int32_t q;
    /* Periods since the ramp's last step */
    uint32_t ramp;
    /*
     * The level control: periods into its window, the window's peak so
     * far, and the way the last window decided to move (-1, 0 or 1).
     */
    uint32_t win;
    uint32_t peak;
    int32_t dir;
} ES8311Gain;

struct ES8311State {
    I2CSlave parent_obj;

    bool ce;
    uint32_t dvdd_mv;
    AudioBackend *audio_be;
    Clock *mclk;
    qemu_irq sclk_out, lrck_out, asdout_out;
    qemu_irq sclk_oe, lrck_oe, asdout_oe;
    qemu_irq pull_up;

    uint8_t regs[ES8311_NREGS];

    /* The control port: the register addressed; the next byte addresses */
    uint8_t ptr;
    bool ptr_next;

    /* The pins as their inputs give them, and what the codec drives */
    bool pin_sclk;
    bool pin_lrck;
    bool pin_dsdin;
    bool drv_sclk, drv_lrck, drv_asdout;
    bool oe_sclk, oe_lrck, oe_asdout;
    bool out_path;

    /* A run from a clock source that does not toggle the pins */
    bool in_run;
    uint32_t run_cycles;
    uint32_t run_cycle;
    int64_t run_start_ns;
    int64_t run_end_ns;
    bool run_inv;
    bool run_last_out;

    /* The serial port: its two directions, the words in and to go out */
    ES8311Sdp sdp_in;
    ES8311Sdp sdp_out;
    uint32_t rx_word[2];
    uint32_t tx_word[2];
    /* SCLK cycles since the frame started, and in the frame before */
    uint32_t frame_cycles;
    uint32_t cycles_per_frame;

    /*
     * LRCK's rate as the codec measures it: the last frame's start and the
     * period before it, the start of the agreeing periods being averaged
     * and their count, and the rate taken, in mHz (0 while unknown).
     */
    int64_t frame_ns;
    int64_t last_period;
    int64_t avg_start_ns;
    uint32_t avg_frames;
    uint64_t lrck_mhz;
    QEMUTimer lost_timer;

    /* The chip state machine: its state (FLAG_CSM_CHIP), periods left */
    uint8_t csm;
    uint32_t csm_frames;

    /* The converters: working, their gains, the ADC's sample, automute */
    bool dac_ok;
    bool adc_ok;
    ES8311Gain dac_gain;
    ES8311Gain adc_gain;
    int32_t adc_sample;
    uint32_t automute_frames;
    bool automuted;

    /*
     * Master mode: SCLK's edges counted from the anchor, every DIV_BCLK
     * divider half-periods of MCLK, the frame in SCLK periods, and the
     * MCLK period they were timed with.
     */
    bool master;
    int64_t mst_anchor_ns;
    uint64_t mst_edge;
    uint32_t mst_div;
    uint32_t mst_frame;
    uint64_t mst_mclk_period;
    QEMUTimer mst_timer;
    Notifier line_source;
    bool line_source_added;

    /* Samples on their way to and from the audio backend */
    SWVoiceOut *voice_out;
    SWVoiceIn *voice_in;
    int32_t out_buf[ES8311_AUDIO_FRAMES];
    uint32_t out_head;
    uint32_t out_num;
    int32_t in_buf[ES8311_AUDIO_FRAMES];
    uint32_t in_head;
    uint32_t in_num;
    bool in_primed;
    bool in_on;
    uint32_t out_freq;

    /* Guest errors already logged, so as to log each once */
    uint32_t logged;
};

/*
 * A run of @ncycles SCLK cycles that a clock source puts through the
 * codec from @start_ns to @end_ns, evenly spaced, without toggling the
 * pins (@inverted: SCLK reaches the pin inverted). For each cycle, the
 * source calls es8311_sclk_fall() with LRCK as the cycle's falling edge
 * leaves it, which returns the bit on ASDOUT for the cycle (*driven
 * false while the codec does not drive the pin), then es8311_sclk_rise()
 * with LRCK and DSDIN as the rising edge samples them. es8311_sclk_end()
 * ends the run, leaving ASDOUT at its last bit.
 */
void es8311_sclk_run(ES8311State *s, uint32_t ncycles, int64_t start_ns,
                     int64_t end_ns, bool inverted);
bool es8311_sclk_fall(ES8311State *s, bool lrck, bool *driven);
void es8311_sclk_rise(ES8311State *s, bool lrck, bool dsdin);
void es8311_sclk_end(ES8311State *s);

/* Whether the codec generates SCLK and LRCK (master mode, MSC set). */
bool es8311_is_master(ES8311State *s);

#endif
