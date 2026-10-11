/*
 * ESP32-P4 I2S controllers, I2S0 to I2S2, and the LP I2S with its voice
 * activity detection
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_ESP32P4_I2S_H
#define HW_AUDIO_ESP32P4_I2S_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "qemu/audio.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

typedef struct ESP32P4GdmaState ESP32P4GdmaState;
typedef struct ESP32P4GpioState ESP32P4GpioState;

#define TYPE_ESP32P4_I2S "esp32p4-i2s"
#define TYPE_ESP32P4_LP_I2S "esp32p4-lp-i2s"
OBJECT_DECLARE_TYPE(ESP32P4I2sState, ESP32P4I2sClass, ESP32P4_I2S)

#define ESP32P4_I2S0_BASE       0x500c6000
#define ESP32P4_I2S_SIZE        0x1000
#define ESP32P4_LP_I2S_BASE     0x50125000
#define ESP32P4_LP_I2S_SIZE     0x1000

/* I2S0 to I2S2, then the LP I2S: the controllers sharing the pads. */
#define ESP32P4_I2S_HP_COUNT    3
#define ESP32P4_I2S_COUNT       4
#define ESP32P4_I2S_LP_ID       3

/*
 * How the machine connects a controller
 * =====================================
 *
 *   - Clock inputs: "apb", the bus clock (I2Sn_APB_CLK, or LPPERI's
 *     LP_I2S gate), which clocks the registers and the FIFO timeout;
 *     "tx" and "rx", I2Sn_TX_CLK and I2Sn_RX_CLK (HP_SYS_CLKRST's source
 *     mux, N + b/a divider and gate; the LP I2S's LP_I2S_RX_CLK from
 *     LPPERI), from which a master divides BCK and on which a slave
 *     samples; "mst", I2Sn_MCLK_out (HP_SYS_CLKRST_I2Sn_MST_CLK_SEL's
 *     choice of the two). The LP I2S has no "tx" or "mst".
 *   - Clock output "pad-mclk": the clock reaching the controller's
 *     I2Sn_MCLK_in signal through the pads, from another controller's
 *     I2Sn_MCLK_out, for HP_SYS_CLKRST's "pad-i2sn-mclk-in".
 *   - Its pad signals: the named GPIO output "sig-out" and input
 *     "sig-in", lines ESP32P4I2sOutLine and ESP32P4I2sInLine, which the
 *     machine connects to the GPIO matrix's (LP matrix's for the LP I2S)
 *     signals esp32p4_i2s_out_signal() and esp32p4_i2s_in_signal() give.
 *   - The interrupt, sysbus IRQ 0: I2Sn_INTR or LP_I2S_INTR.
 *   - Links: "gdma", GDMA-AHB, which serves I2S0 to I2S2 as its
 *     peripherals 3 to 5 (the LP I2S has no DMA); "gpio", the GPIO
 *     matrix, whose routing tells which controller's outputs reach which
 *     inputs; "i2s0" to "i2s2" and "lp-i2s", every controller on the
 *     pads, this one included.
 *   - ETM: named GPIO output "etm-event", lines RX_DONE, TX_DONE,
 *     X_WORDS_RECEIVED, X_WORDS_SENT (I2Sn_EVT_* in that order), and
 *     input "etm-task", lines START_RX, START_TX, STOP_RX, STOP_TX
 *     (I2Sn_TASK_*), each acting on a rising edge. The LP I2S has neither.
 *   - The LP I2S's named GPIO output "wakeup": LP_I2S_WAKEUP, high while
 *     the VAD is in its voice activity status, for the PMU.
 *   - The audio backend (property "audiodev"): a codec on the bus.
 */
#define ESP32P4_I2S_SIG_OUT     "sig-out"
#define ESP32P4_I2S_SIG_IN      "sig-in"
#define ESP32P4_I2S_ETM_EVENT   "etm-event"
#define ESP32P4_I2S_ETM_TASK    "etm-task"
#define ESP32P4_I2S_WAKEUP      "wakeup"

typedef enum ESP32P4I2sOutLine {
    ESP32P4_I2S_OUT_O_BCK,      /* I2SnO_BCK_out */
    ESP32P4_I2S_OUT_O_WS,       /* I2SnO_WS_out */
    ESP32P4_I2S_OUT_O_SD,       /* I2SnO_SD_out */
    ESP32P4_I2S_OUT_O_SD1,      /* I2S0O_SD1_out, I2S0 only */
    ESP32P4_I2S_OUT_I_BCK,      /* I2SnI_BCK_out */
    ESP32P4_I2S_OUT_I_WS,       /* I2SnI_WS_out */
    ESP32P4_I2S_OUT_MCLK,       /* I2Sn_MCLK_out */
    ESP32P4_I2S_OUT_COUNT,
} ESP32P4I2sOutLine;

typedef enum ESP32P4I2sInLine {
    ESP32P4_I2S_IN_O_BCK,       /* I2SnO_BCK_in */
    ESP32P4_I2S_IN_O_WS,        /* I2SnO_WS_in */
    ESP32P4_I2S_IN_I_BCK,       /* I2SnI_BCK_in */
    ESP32P4_I2S_IN_I_WS,        /* I2SnI_WS_in */
    ESP32P4_I2S_IN_I_SD,        /* I2SnI_SD_in */
    ESP32P4_I2S_IN_I_SD1,       /* I2S0I_SD1_in to SD3_in, I2S0 only */
    ESP32P4_I2S_IN_I_SD2,
    ESP32P4_I2S_IN_I_SD3,
    ESP32P4_I2S_IN_MCLK,        /* I2Sn_MCLK_in */
    ESP32P4_I2S_IN_COUNT,
} ESP32P4I2sInLine;

/* The matrix signal of a line, or -1 where the controller has none. */
int esp32p4_i2s_out_signal(unsigned id, ESP32P4I2sOutLine line);
int esp32p4_i2s_in_signal(unsigned id, ESP32P4I2sInLine line);

enum {
    ESP32P4_I2S_EVT_RX_DONE,
    ESP32P4_I2S_EVT_TX_DONE,
    ESP32P4_I2S_EVT_X_WORDS_RECEIVED,
    ESP32P4_I2S_EVT_X_WORDS_SENT,
    ESP32P4_I2S_ETM_EVENTS,
};

enum {
    ESP32P4_I2S_TASK_START_RX,
    ESP32P4_I2S_TASK_START_TX,
    ESP32P4_I2S_TASK_STOP_RX,
    ESP32P4_I2S_TASK_STOP_TX,
    ESP32P4_I2S_ETM_TASKS,
};

/* Each direction's FIFO: 64 words of 32 bits (TRM 49.4). */
#define ESP32P4_I2S_FIFO_BYTES  256
/* TDM channels, and PDM-to-PCM channels (two on each of four lines). */
#define ESP32P4_I2S_SLOTS       16
#define ESP32P4_I2S_PDM_CHANS   8
/* The LP I2S memory: 256 words of 32 bits (TRM 50.3). */
#define ESP32P4_LP_I2S_MEM_WORDS 256
/* A VAD frame: 256 samples of 16 bits (TRM 44.2). */
#define ESP32P4_LP_I2S_VAD_FRAME 256
/* Frames buffered between a controller and the audio backend. */
#define ESP32P4_I2S_AUDIO_FRAMES 4096

typedef struct ESP32P4I2sFifo {
    uint8_t data[ESP32P4_I2S_FIFO_BYTES];
    /* The receive FIFO's marks of the bytes that end an RX_EOF_NUM run. */
    uint8_t eof[ESP32P4_I2S_FIFO_BYTES];
    uint32_t head;
    uint32_t num;
} ESP32P4I2sFifo;

/*
 * A unit's frame on the serial line: the BCK cycles since its sync and
 * its channel data, sent or as received. The frame before the current one
 * keeps the line for the cycle by which MSB_SHIFT delays its last bit.
 */
typedef struct ESP32P4I2sFrame {
    bool active;
    uint32_t pos;
    uint32_t limit;
    uint32_t slot[ESP32P4_I2S_SLOTS];
} ESP32P4I2sFrame;

/* One channel of a PDM converter, or of the PDM codec on the bus. */
typedef struct ESP32P4I2sPdmChan {
    /* Sigma-delta: the integrator; decimator: three integrators, combs. */
    int64_t integ[3];
    int64_t comb[3];
    /* The two high-pass stages' last input and output. */
    int32_t hp_x[2];
    int32_t hp_y[2];
    /* The sample being modulated, or the raw PDM word being shifted. */
    int32_t pcm;
    uint32_t word;
} ESP32P4I2sPdmChan;

/* The transmitter or the receiver. */
typedef struct ESP32P4I2sUnit {
    /*
     * The configuration as *_UPDATE last carried it into the unit's
     * clock domain: CONF, CONF1, TDM_CTRL, the PDM converter's one or two
     * registers and CONF_SINGLE_DATA.
     */
    uint32_t conf;
    uint32_t conf1;
    uint32_t tdm_ctrl;
    uint32_t pdm_conf;
    uint32_t pdm_conf1;
    uint32_t single_data;
    bool update_pending;

    /* What the unit is doing, from its START and its configuration. */
    uint8_t mode;
    /*
     * A master transmitter stopped for want of data with TX_STOP_EN, or
     * a receiver suspended by RX_STOP_MODE until RX_RESET.
     */
    bool stalled;
    /* The transmitter ran out of data: TX_DONE raised for this run-out. */
    bool drained;

    bool started;
    uint8_t prev_ws;
    ESP32P4I2sFrame cur;
    ESP32P4I2sFrame prev;
    /* The last frame sent, sent again when data runs out. */
    uint32_t last[ESP32P4_I2S_SLOTS];
    /* The last channel data sent, for TX_CHAN_EQUAL. */
    uint32_t last_chan;
    /* The audio backend's frame, for a receiver it feeds. */
    uint32_t codec[2];

    /* The PDM bit within the current sample, and the converters. */
    uint32_t pdm_bit;
    ESP32P4I2sPdmChan pdm[ESP32P4_I2S_PDM_CHANS];
    ESP32P4I2sPdmChan codec_pdm[2];

    /* Receiver: data stored since the last RX_EOF_NUM run ended. */
    uint32_t eof_count;
    /* Channel data moved since START, for the ETM's word events. */
    uint32_t words;

    /*
     * A master's clock: frames (or PDM samples) end at anchor plus
     * clock_ticks_to_ns() of (events + 1) frames' unit clock cycles.
     */
    int64_t anchor;
    uint64_t events;
    uint64_t frame_ticks;
    QEMUTimer timer;

    /* The FIFO timeout. */
    QEMUTimer hung_timer;
    bool hung_fired;

    /*
     * The level the unit last drove on its data line, and, for a slave on
     * the pads' BCK, the bit for its next falling edge.
     */
    bool sd_level;
    bool sd_next;
    bool logged_mismatch;
} ESP32P4I2sUnit;

/* The VAD's state and observation registers (TRM 44.4). */
typedef struct ESP32P4I2sVad {
    bool flag;
    bool energy_enough;
    uint32_t init_frames;
    uint32_t speech_count;
    uint32_t silent_count;
    uint32_t noise_amp;
    uint32_t threshold;
    int32_t offset;
    uint64_t energy;
    uint16_t max_signal[3];
    /* Samples stored since the last frame the VAD took. */
    uint32_t samples;
} ESP32P4I2sVad;

struct ESP32P4I2sState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq sig_out[ESP32P4_I2S_OUT_COUNT];
    qemu_irq etm_event[ESP32P4_I2S_ETM_EVENTS];
    qemu_irq wakeup;
    Clock *apb_clk;
    Clock *tx_clk;
    Clock *rx_clk;
    Clock *mst_clk;
    Clock *pad_mclk;
    ESP32P4GdmaState *gdma;
    ESP32P4GpioState *gpio;
    ESP32P4I2sState *bus[ESP32P4_I2S_COUNT];
    uint8_t id;
    Notifier route_notifier;
    bool mclk_busy;
    AudioBackend *audio_be;
    SWVoiceOut *voice_out;
    SWVoiceIn *voice_in;

    /* Registers, as software wrote them. */
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t rx_conf;
    uint32_t tx_conf;
    uint32_t rx_conf1;
    uint32_t tx_conf1;
    uint32_t tx_pcm2pdm_conf;
    uint32_t tx_pcm2pdm_conf1;
    uint32_t rx_pdm2pcm_conf;
    uint32_t rx_tdm_ctrl;
    uint32_t tx_tdm_ctrl;
    uint32_t rx_timing;
    uint32_t tx_timing;
    uint32_t lc_hung_conf;
    uint32_t rx_eof_num;
    uint32_t single_data;
    uint32_t etm_conf;
    uint32_t tx_fifo_cnt;
    uint32_t tx_bck_cnt;
    uint32_t clk_gate;
    uint32_t date;
    /* The LP I2S's own. */
    uint32_t vad_conf;
    uint32_t rx_mem_conf;
    uint32_t eco_low;
    uint32_t eco_high;
    uint32_t eco_conf;
    uint32_t vad_param[9];

    ESP32P4I2sUnit tx;
    ESP32P4I2sUnit rx;
    ESP32P4I2sFifo tx_fifo;
    ESP32P4I2sFifo rx_fifo;

    /* The LP I2S memory, a circular buffer of words. */
    uint32_t mem[ESP32P4_LP_I2S_MEM_WORDS];
    uint32_t mem_rd;
    uint32_t mem_num;
    uint32_t mem_wr;
    /* A 16-bit sample waiting for the other half of its word. */
    bool mem_half_valid;
    uint16_t mem_half;
    ESP32P4I2sVad vad;

    /* The levels of the input signals, as the matrix delivers them. */
    uint32_t in_level;

    /* Frames on their way to and from the audio backend. */
    int32_t audio_out[ESP32P4_I2S_AUDIO_FRAMES][2];
    uint32_t audio_out_head;
    uint32_t audio_out_num;
    int32_t audio_in[ESP32P4_I2S_AUDIO_FRAMES][2];
    uint32_t audio_in_head;
    uint32_t audio_in_num;
    int audio_out_freq;
    int audio_out_bits;
    int audio_in_freq;
    int audio_in_bits;

    /*
     * A master's period is running through the units on its clock; the
     * ETM tasks that come meanwhile wait for its end.
     */
    bool in_run;
    uint8_t etm_deferred;

    bool logged_conv;
    bool logged_slave_clk;
};

struct ESP32P4I2sClass {
    SysBusDeviceClass parent_class;

    bool lp;
};

#endif
