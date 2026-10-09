/*
 * ESP32 I2S controller, with its DMA engine and LCD and camera modes
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_ESP32_I2S_H
#define HW_AUDIO_ESP32_I2S_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/audio.h"
#include "qemu/timer.h"
#include "system/memory.h"

typedef struct Esp32GpioState Esp32GpioState;
typedef struct Esp32ApbCtrlState Esp32ApbCtrlState;
typedef struct Esp32SensState Esp32SensState;

#define TYPE_ESP32_I2S "esp32.i2s"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32I2sState, ESP32_I2S)

#define ESP32_I2S_COUNT         2
#define ESP32_I2S_REG_SIZE      0x1000
/* Each direction's FIFO: 64 entries of 32 bits (TRM 22.1) */
#define ESP32_I2S_FIFO_DEPTH    64
#define ESP32_I2S_DATA_OUT_COUNT 24
#define ESP32_I2S_DATA_IN_COUNT 16
/* Frames buffered between the controller and the audio backend */
#define ESP32_I2S_AUDIO_FRAMES  4096

/* The FIFO_RD register, whose read pops the receive FIFO */
#define ESP32_I2S_FIFO_RD_OFFSET 0x04

/*
 * GPIO matrix signals. The controller drives the output lines named
 * ESP32_I2S_SIG_OUT and listens on the input lines named ESP32_I2S_SIG_IN,
 * indexed as below; esp32_i2s_out_signal() and esp32_i2s_in_signal() give
 * the GPIO matrix signal number of each line.
 */
#define ESP32_I2S_SIG_OUT "esp32-i2s-sig-out"
#define ESP32_I2S_SIG_IN  "esp32-i2s-sig-in"

typedef enum Esp32I2sOutLine {
    ESP32_I2S_OUT_O_BCK,        /* I2SnO_BCK_out: transmitter bit clock */
    ESP32_I2S_OUT_O_WS,         /* I2SnO_WS_out: transmitter word select */
    ESP32_I2S_OUT_I_BCK,        /* I2SnI_BCK_out: receiver bit clock */
    ESP32_I2S_OUT_I_WS,         /* I2SnI_WS_out: receiver word select */
    ESP32_I2S_OUT_DATA0,        /* I2SnO_DATA_out0..23 */
    ESP32_I2S_OUT_COUNT = ESP32_I2S_OUT_DATA0 + ESP32_I2S_DATA_OUT_COUNT,
} Esp32I2sOutLine;

typedef enum Esp32I2sInLine {
    ESP32_I2S_IN_O_BCK,         /* I2SnO_BCK_in: slave transmitter clock */
    ESP32_I2S_IN_O_WS,          /* I2SnO_WS_in */
    ESP32_I2S_IN_I_BCK,         /* I2SnI_BCK_in: slave receiver clock */
    ESP32_I2S_IN_I_WS,          /* I2SnI_WS_in, the camera's PCLK */
    ESP32_I2S_IN_DATA0,         /* I2SnI_Data_in0..15 */
    ESP32_I2S_IN_H_SYNC = ESP32_I2S_IN_DATA0 + ESP32_I2S_DATA_IN_COUNT,
    ESP32_I2S_IN_V_SYNC,
    ESP32_I2S_IN_H_ENABLE,
    ESP32_I2S_IN_COUNT,
} Esp32I2sInLine;

unsigned esp32_i2s_out_signal(unsigned id, Esp32I2sOutLine line);
unsigned esp32_i2s_in_signal(unsigned id, Esp32I2sInLine line);

typedef struct Esp32I2sFifo {
    uint32_t data[ESP32_I2S_FIFO_DEPTH];
    /*
     * Transmit FIFO entries carry the model's markers of the events their
     * departure raises: the last word of an eof descriptor, and of the last
     * descriptor of the list, with that descriptor and its buffer.
     */
    uint8_t tag[ESP32_I2S_FIFO_DEPTH];
    uint32_t tag_dscr[ESP32_I2S_FIFO_DEPTH];
    uint32_t tag_buf[ESP32_I2S_FIFO_DEPTH];
    uint32_t head;
    uint32_t num;
} Esp32I2sFifo;

/* One direction of the DMA engine, walking a descriptor list */
typedef struct Esp32I2sLink {
    bool active;
    bool parked;
    /* The descriptor in hand: its address, words, and bytes moved */
    bool have_desc;
    uint32_t dscr;
    uint32_t dw0;
    uint32_t buf;
    uint32_t next;
    uint32_t pos;
    /* The descriptor to fetch when none is in hand */
    uint32_t fetch;
    /* The last descriptor finished, from whose next RESTART continues */
    bool have_last;
    uint32_t last;
    /* Inlink: out of descriptors, waiting for RESTART */
    bool exhausted;
    bool empty_flagged;
} Esp32I2sLink;

/* A channel slot being shifted out or in, one bit per BCK cycle */
typedef struct Esp32I2sSlot {
    bool active;
    uint8_t ch;
    uint32_t pos;
    uint32_t val;
    /* The cycle at which a slot that has been replaced lets go of the line */
    uint32_t limit;
} Esp32I2sSlot;

/* The serializer of the transmitter or the deserializer of the receiver */
typedef struct Esp32I2sSerial {
    bool started;
    uint8_t prev_ws;
    bool pulse;
    uint8_t next_ch;
    Esp32I2sSlot cur;
    Esp32I2sSlot prev;
} Esp32I2sSerial;

struct Esp32I2sState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq sig_out[ESP32_I2S_OUT_COUNT];
    Clock *apb_clk;
    Clock *f160m_clk;
    Clock *apll_clk;
    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    Esp32GpioState *gpio;
    Esp32ApbCtrlState *apb_ctrl;
    /* SENS, with the SAR ADCs and DACs of I2S0's ADC and DAC modes */
    Esp32SensState *sens;
    /* The other controller, which shares the GPIO matrix with this one */
    Esp32I2sState *peer;
    uint8_t id;
    AudioBackend *audio_be;
    SWVoiceOut *voice_out;
    SWVoiceIn *voice_in;

    /* Registers */
    uint32_t conf;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t timing;
    uint32_t fifo_conf;
    uint32_t rx_eof_num;
    uint32_t single_data;
    uint32_t conf_chan;
    uint32_t out_link;
    uint32_t in_link;
    uint32_t out_eof_des_addr;
    uint32_t in_eof_des_addr;
    uint32_t out_eof_bfr_des_addr;
    uint32_t ahb_test;
    uint32_t inlink_dscr;
    uint32_t inlink_dscr_bf0;
    uint32_t inlink_dscr_bf1;
    uint32_t outlink_dscr;
    uint32_t outlink_dscr_bf0;
    uint32_t outlink_dscr_bf1;
    uint32_t lc_conf;
    uint32_t outfifo_push;
    uint32_t infifo_pop;
    uint32_t lc_hung_conf;
    uint32_t cvsd_conf[3];
    uint32_t plc_conf[3];
    uint32_t esco_conf0;
    uint32_t sco_conf0;
    uint32_t conf1;
    uint32_t pd_conf;
    uint32_t conf2;
    uint32_t clkm_conf;
    uint32_t sample_rate_conf;
    uint32_t pdm_conf;
    uint32_t pdm_freq_conf;
    uint32_t date;

    Esp32I2sFifo tx_fifo;
    Esp32I2sFifo rx_fifo;
    Esp32I2sLink out;
    Esp32I2sLink in;
    /* Words the inlink has stored since the last receive EOF */
    uint32_t rx_eof_count;

    /*
     * Transmitter: the frame being sent, a 16-bit sample left over from a
     * FIFO word in the 16-bit single-channel mode, and the timing of the
     * clock it generates as master: events at anchor + (n + 1) periods.
     */
    bool tx_stopped;
    uint32_t tx_frame[2];
    bool tx_half_valid;
    uint32_t tx_half;
    int64_t tx_anchor;
    uint64_t tx_events;
    QEMUTimer tx_timer;
    Esp32I2sSerial tx_ser;
    uint32_t data_out_level;

    /*
     * Receiver: the first channel of the frame being assembled, the half
     * word of the 16-bit single-channel mode, and the previous sample of
     * the parallel modes.
     */
    bool rx_have_first;
    uint32_t rx_first;
    bool rx_half_valid;
    uint32_t rx_half;
    uint32_t rx_prev_sample;
    int64_t rx_anchor;
    uint64_t rx_events;
    QEMUTimer rx_timer;
    Esp32I2sSerial rx_ser;

    /* Levels of the input signals, as the GPIO matrix delivers them */
    uint32_t in_level;

    /* FIFO timeouts */
    QEMUTimer tx_hung_timer;
    QEMUTimer rx_hung_timer;
    bool tx_hung_fired;
    bool rx_hung_fired;

    /* Frames on their way to and from the audio backend */
    int32_t audio_out[ESP32_I2S_AUDIO_FRAMES][2];
    uint32_t audio_out_head;
    uint32_t audio_out_num;
    int32_t audio_in[ESP32_I2S_AUDIO_FRAMES][2];
    uint32_t audio_in_head;
    uint32_t audio_in_num;
    int audio_out_freq;
    int audio_out_bits;
    int audio_in_freq;
    int audio_in_bits;

    /*
     * What the transmitter and receiver are doing, and the timing and
     * channel width their configuration gives, as of their last change.
     */
    uint8_t tx_mode_cur;
    uint8_t rx_mode_cur;
    double tx_period;
    double rx_period;
    uint32_t tx_offset;
    uint32_t rx_offset;
    uint32_t tx_bits;
    uint32_t rx_bits;

    bool in_kick;
    bool kick_again;
    bool logged_pdm_in;
};

#endif
