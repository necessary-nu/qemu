/*
 * ESP32-P4 I2S controllers, I2S0 to I2S2, and the LP I2S with its voice
 * activity detection
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 TRM chapters 49 "I2S Controller", 50 "LP I2S
 * Controller" and 44 "Voice Activity Detection", with the register layout
 * and reset values of ESP-IDF's rev3 soc headers (i2s_reg.h, lp_i2s_reg.h)
 * and the way ESP-IDF's i2s_ll.h, lp_i2s_ll.h and esp_driver_i2s drive
 * the blocks.
 *
 * Each HP controller has a transmitter and a receiver (the TX and RX
 * units), each with a FIFO of 64 words between it and GDMA-AHB, which
 * serves it as peripheral 3, 4 or 5. The LP I2S has a receiver only,
 * which stores into the LP I2S memory, where the CPUs read it and the VAD
 * looks for speech.
 *
 * Configuration. A unit works from its registers as TX_UPDATE or
 * RX_UPDATE last carried them into its clock domain (TRM 49.9, 49.13):
 * CONF (all but the reset, start and update bits), CONF1, TDM_CTRL, the
 * PDM converter's registers and CONF_SINGLE_DATA. The update takes effect
 * and the bit clears at once while the unit's clock runs, and waits for
 * the clock while it is stopped. SIG_LOOPBACK, a routing bit, acts at
 * once.
 *
 * Clocks. I2Sn_TX_CLK and I2Sn_RX_CLK come from HP_SYS_CLKRST (its source
 * mux and N + b/a divider). A master's BCK is that clock divided by
 * BCK_DIV_NUM + 1; a TDM frame, one WS period, is 2 * (HALF_SAMPLE_BITS +
 * 1) BCK cycles. In PDM mode one BCK cycle is one period of the PDM clock
 * on the WS line, carrying a left bit while WS is at its idle level and a
 * right bit in the other half: the TRM gives the PDM clock as BCK in its
 * converters' sections and as BCK / 2 in its PDM mode sections; the model
 * takes the former, with which ESP-IDF's PDM sample rates agree. A slave
 * needs its own unit clock running.
 *
 * The serial line is modelled BCK cycle by BCK cycle (TRM 49.5): in TDM
 * mode a master's WS is at WS_IDLE_POL for TDM_WS_WIDTH + 1 cycles from
 * the start of each frame and at the other level for the rest, which
 * gives the Philips, MSB-aligned and PCM short and long frame standards
 * from the settings ESP-IDF makes for each; a slave starts its frame where
 * WS changes to its idle level. Channel n of TDM_TOT_CHAN_NUM + 1 is the
 * TDM_CHAN_BITS + 1 bits from cycle n * (TDM_CHAN_BITS + 1) of the frame,
 * MSB first, one cycle later with MSB_SHIFT. A master runs a frame of
 * cycles at a time, at its end, through every unit on its clock: itself;
 * the other unit of its controller with SIG_LOOPBACK, which shares BCK
 * and WS; and any slave unit of any controller whose BCK and WS inputs the
 * GPIO matrices connect to its BCK and WS outputs. A receiver takes the
 * bits of the transmitter on the same clock that the matrices connect to
 * its data input, or that input's level. A slave clocked by no master of
 * the chip follows the edges of its BCK input from the pads.
 *
 * Data (TRM 49.9.1, 49.10.2). The transmitter reads each channel's valid
 * data, BITS_MOD + 1 bits (32 for 24 with 24_FILL_EN) in whole bytes,
 * little-endian or with BIG_ENDIAN byte-reversed, passes it through the
 * A-law or u-law unit unless PCM_BYPASS, fits it to the channel's bits,
 * left- or right-aligned, and reverses its bits with BIT_ORDER. The
 * receiver does the reverse. A-law and u-law are G.711's, on a 16-bit
 * linear sample in the low half of the 32-bit value the units work on:
 * compression puts the 8-bit code in the low byte, expansion gives the
 * sample sign-extended. A disabled TX channel sends the channel before it
 * with TX_CHAN_EQUAL, CONF_SINGLE_DATA otherwise, and with
 * TX_TDM_SKIP_MSK_EN still consumes its data; a disabled RX channel is not
 * stored. TX_MONO and RX_MONO act in PDM mode only (TRM table 49.9-3):
 * in TDM mode ESP-IDF's mono modes are the channel enables and
 * TX_CHAN_EQUAL.
 *
 * A transmitter takes a whole frame's data from its FIFO at the frame's
 * start. Without enough (TRM 49.8.1), a master with TX_STOP_EN stops its
 * clocks until data comes, a slave with TX_STOP_EN sends zeros, and a
 * unit without it sends the last frame again; TX_DONE marks the running
 * out. The receiver ends a run of (RX_EOF_NUM + 1) channel data, the
 * register's (BITS_MOD + 1) * (EOF_NUM + 1) bits, with GDMA's EOF and
 * RX_DONE, and RX_STOP_MODE 1 then suspends it until RX_RESET, as mode 2
 * does when its FIFO is full; without mode 2 data that finds the FIFO full
 * is lost. ESP-IDF writes a byte count to RX_EOF_NUM; the model follows
 * the register's description.
 *
 * PDM (TRM 49.9.2.2, 49.10.1.2). Raw PDM sends and receives each
 * channel's data bits as they are, a channel's BITS_MOD + 1 bits per
 * sample, the channels mapped by TX_CHAN_MOD and TX_WS_IDLE_POL (table
 * 49.9-4) and fetched as TX_MONO and TX_MONO_FST_VLD say (table 49.9-3).
 * I2S0's PCM-to-PDM converter takes a 16-bit sample per 64 *
 * TX_PDM_SINC_OSR2 BCK cycles; its two high-pass stages are first-order
 * IIRs whose pole is (504 + TX_IIR_HP_MULT12_n) / 512, its four scaling
 * shifts multiply, and a first-order sigma-delta modulator, holding each
 * sample, makes the bits; in DAC mode one channel goes out per WS period,
 * the right on SD1 with DAC_2OUT. I2S0's PDM-to-PCM converter decimates
 * each of up to eight channels (two on each of SD to SD3) by 64, or 128
 * with RX_PDM_SINC_DSR_16_EN, through a third-order CIC, through the
 * same high-pass stages unless RX_PDM_HP_BYPASS, scaled by
 * RX_PDM2PCM_AMPLIFY_NUM. The silicon's interpolation and decimation
 * filters, prescale and dither are not documented: these stand for them
 * (logged once), so a sample comes back close to, not equal to, what was
 * sent.
 *
 * The LP I2S receives 16-bit channels (BITS_MOD fixed at 15) in TDM (up to
 * two channels) or PDM mode, two samples to a word, the first in the low
 * half, into its 256-word memory, a circular buffer that overwrites its
 * oldest word when full and that software reads, oldest first, through the
 * window at 0xC00 (LP_I2S_RAM_BASE), where writes store words for the
 * VAD's manual mode. The VAD chapter's "2 KB buffer of four frames"
 * disagrees with the LP I2S chapter's 256 words and with RX_MEM_FIFO_CNT's
 * nine bits; the model takes 256 words, two VAD frames.
 *
 * The VAD (TRM 44.4) takes each 256 samples stored (with VAD_EN) or, on
 * VAD_FORCE_START, the last 256 stored. A frame whose sum of squares is
 * below PARAM_MIN_ENERGY * 256 is not speech (ENERGY_ENOUGH clear). The
 * first PARAM_INIT_FRAME_NUM frames only learn the noise level. The FFT,
 * LTSD and band energy stages are not documented: a frame that passes the
 * energy check is speech when its RMS exceeds twice the learned noise
 * level, which outside speech follows each frame's RMS through
 * PARAM_NOISE_AMP_UP and _DOWN (Q15 smoothing). The speech and silence
 * counters and the state machine are the TRM's; in the speech state the
 * noise level learns again once SPEECH_COUNT exceeds
 * PARAM_MAX_SPEECH_COUNT. VAD_FLAG is LP_I2S_WAKEUP.
 *
 * ETM (TRM 49.11). The events carry the time of what raises them (see
 * esp32_line.h): a master's period runs as of its end's time, however
 * late the QEMU timer that runs it. The tasks act at their events' time;
 * those that come while a period runs wait for its end and are then
 * taken in the order of their events' times, each at its own.
 *
 * Devices on the board whose serial audio pins are pads (the board's
 * codec, esp32p4_i2s_add_peer()) are clocked through the pads as the
 * controllers clock each other: a TDM master whose BCK output the
 * matrices route to a device's SCLK pad runs each of its periods through
 * it, cycle by cycle, LRCK being the pad's level or, routed there, the
 * master's WS, and the device's data input the bit of the transmitter on
 * the same clock whose data output reaches its pad, or the pad's level;
 * a receiver whose data input the matrices take from the device's data
 * output pad receives the device's bits, or the pad's level where the
 * device does not drive it. The pads' inversions apply. A device clocked
 * by a PDM master takes no part (logged): the PDM clock on WS is no LRCK.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/audio/esp32p4_i2s.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "migration/vmstate.h"

/* HP I2S registers */
#define A_INT_RAW               0x0c
#define A_INT_ST                0x10
#define A_INT_ENA               0x14
#define A_INT_CLR               0x18
#define A_RX_CONF               0x20
#define A_TX_CONF               0x24
#define A_RX_CONF1              0x28
#define A_TX_CONF1              0x2c
#define A_TX_PCM2PDM_CONF       0x40
#define A_TX_PCM2PDM_CONF1      0x44
#define A_RX_PDM2PCM_CONF       0x48
#define A_RX_TDM_CTRL           0x50
#define A_TX_TDM_CTRL           0x54
#define A_RX_TIMING             0x58
#define A_TX_TIMING             0x5c
#define A_LC_HUNG_CONF          0x60
#define A_RXEOF_NUM             0x64
#define A_CONF_SIGLE_DATA       0x68
#define A_STATE                 0x6c
#define A_ETM_CONF              0x70
#define A_FIFO_CNT              0x74
#define A_BCK_CNT               0x78
#define A_CLK_GATE              0x7c
#define A_DATE                  0x80

/* LP I2S registers */
#define A_LP_VAD_CONF           0x00
#define A_LP_VAD_RESULT         0x04
#define A_LP_RX_MEM_CONF        0x08
#define A_LP_INT_RAW            0x0c
#define A_LP_INT_ST             0x10
#define A_LP_INT_ENA            0x14
#define A_LP_INT_CLR            0x18
#define A_LP_RX_CONF            0x20
#define A_LP_RX_CONF1           0x28
#define A_LP_RX_TDM_CTRL        0x50
#define A_LP_RX_TIMING          0x58
#define A_LP_LC_HUNG_CONF       0x60
#define A_LP_RXEOF_NUM          0x64
#define A_LP_CONF_SIGLE_DATA    0x68
#define A_LP_RX_PDM_CONF        0x70
#define A_LP_ECO_LOW            0x74
#define A_LP_ECO_HIGH           0x78
#define A_LP_ECO_CONF           0x7c
#define A_LP_VAD_PARAM0         0x80
#define A_LP_VAD_PARAM8         0xa0
#define A_LP_VAD_OB0            0xb0
#define A_LP_VAD_OB8            0xd0
#define A_LP_CLK_GATE           0xf8
#define A_LP_DATE               0xfc
/* LP_I2S_RAM_BASE, the memory's window */
#define A_LP_RAM                0xc00
#define A_LP_RAM_END            0x1000

/* HP interrupts */
#define INT_RX_DONE             (1u << 0)
#define INT_TX_DONE             (1u << 1)
#define INT_RX_HUNG             (1u << 2)
#define INT_TX_HUNG             (1u << 3)
#define INT_MASK                0xfu

/* LP interrupts */
#define LP_INT_RX_DONE          (1u << 0)
#define LP_INT_RX_HUNG          (1u << 1)
#define LP_INT_FIFOMEM_UDF      (1u << 2)
#define LP_INT_VAD_DONE         (1u << 3)
#define LP_INT_VAD_RESET_DONE   (1u << 4)
#define LP_INT_MEM_THRESHOLD    (1u << 5)
#define LP_INT_MASK             0x3fu

/* RX_CONF and TX_CONF, the bits both have in the same place */
#define CONF_RESET              (1u << 0)
#define CONF_FIFO_RESET         (1u << 1)
#define CONF_START              (1u << 2)
#define CONF_SLAVE_MOD          (1u << 3)
#define CONF_UPDATE             (1u << 8)
#define CONF_MONO_FST_VLD       (1u << 9)
#define CONF_PCM_CONF(v)        (((v) >> 10) & 3)
#define CONF_PCM_BYPASS         (1u << 12)
#define CONF_LEFT_ALIGN         (1u << 15)
#define CONF_24_FILL_EN         (1u << 16)
#define CONF_WS_IDLE_POL        (1u << 17)
#define CONF_BIT_ORDER          (1u << 18)
#define CONF_TDM_EN             (1u << 19)
#define CONF_PDM_EN             (1u << 20)
#define CONF_BIG_ENDIAN         (1u << 7)
#define CONF_BCK_DIV_NUM(v)     (((v) >> 21) & 0x3f)

/* RX_CONF */
#define RX_CONF_STOP_MODE(v)    (((v) >> 4) & 3)
#define RX_CONF_MONO            (1u << 6)
#define RX_CONF_MSB_SHIFT       (1u << 13)
#define RX_CONF_MASK            0x07ffbffcu
#define RX_CONF_RESET_VAL       0x00c0b600u

/* TX_CONF */
#define TX_CONF_STOP_EN         (1u << 4)
#define TX_CONF_CHAN_EQUAL      (1u << 5)
#define TX_CONF_MONO            (1u << 6)
#define TX_CONF_MSB_SHIFT       (1u << 13)
#define TX_CONF_CHAN_MOD(v)     (((v) >> 27) & 7)
#define TX_CONF_SIG_LOOPBACK    (1u << 30)
#define TX_CONF_MASK            0x7ffffffcu
#define TX_CONF_RESET_VAL       0x00c0f210u

/* RX_CONF1 and TX_CONF1 */
#define CONF1_TDM_WS_WIDTH(v)   ((v) & 0x1ff)
#define CONF1_BITS_MOD(v)       (((v) >> 14) & 0x1f)
#define CONF1_HALF_SAMPLE(v)    (((v) >> 19) & 0xff)
#define CONF1_TDM_CHAN_BITS(v)  (((v) >> 27) & 0x1f)
#define CONF1_MASK              0xffffc1ffu
#define CONF1_RESET_VAL         0x787bc000u

/* TX_PCM2PDM_CONF and TX_PCM2PDM_CONF1 */
#define P2D_SINC_OSR2(v)        (((v) >> 1) & 0xf)
#define P2D_PRESCALE(v)         (((v) >> 5) & 0xff)
#define P2D_HP_IN_SHIFT(v)      (((v) >> 13) & 3)
#define P2D_LP_IN_SHIFT(v)      (((v) >> 15) & 3)
#define P2D_SINC_IN_SHIFT(v)    (((v) >> 17) & 3)
#define P2D_SD_IN_SHIFT(v)      (((v) >> 19) & 3)
#define P2D_DAC_2OUT_EN         (1u << 23)
#define P2D_DAC_MODE_EN         (1u << 24)
#define P2D_CONV_EN             (1u << 25)
#define P2D_MASK                0x03fffffeu
#define P2D_RESET_VAL           0x004aa004u
#define P2D1_MULT12_5(v)        (((v) >> 20) & 7)
#define P2D1_MULT12_0(v)        (((v) >> 23) & 7)
#define P2D1_MASK               0x03ffffffu
#define P2D1_RESET_VAL          0x03f783c0u

/* RX_PDM2PCM_CONF (LP: RX_PDM_CONF) */
#define D2P_EN                  (1u << 19)
#define D2P_DSR_16_EN           (1u << 20)
#define D2P_AMPLIFY(v)          (((v) >> 21) & 0xf)
#define D2P_HP_BYPASS           (1u << 25)
#define D2P_MULT12_5(v)         (((v) >> 26) & 7)
#define D2P_MULT12_0(v)         (((v) >> 29) & 7)
#define D2P_MASK                0xfff80000u
#define D2P_RESET_VAL           0xf8200000u

/* RX_TDM_CTRL and TX_TDM_CTRL */
#define TDM_CHAN_EN(v)          ((v) & 0xffff)
#define TDM_TOT_CHAN_NUM(v)     (((v) >> 16) & 0xf)
#define TDM_SKIP_MSK_EN         (1u << 20)
#define RX_TDM_MASK             0x000fffffu
#define TX_TDM_MASK             0x001fffffu
#define TDM_RESET_VAL           0x0000ffffu

#define RX_TIMING_MASK          0x33333333u
#define TX_TIMING_MASK          0x33330033u

/* LC_HUNG_CONF */
#define HUNG_TIMEOUT(v)         ((v) & 0xff)
#define HUNG_SHIFT(v)           (((v) >> 8) & 7)
#define HUNG_ENA                (1u << 11)
#define HUNG_MASK               0x00000fffu
#define HUNG_RESET_VAL          0x00000810u
/* The tick counter wraps at 88000 >> SHIFT bus clock cycles. */
#define HUNG_TICK_CYCLES        88000

#define RXEOF_NUM_MASK          0xfffu
#define RXEOF_NUM_RESET_VAL     0x40u

#define STATE_TX_IDLE           (1u << 0)

/* ETM_CONF */
#define ETM_TX_WORD_NUM(v)      ((v) & 0x3ff)
#define ETM_RX_WORD_NUM(v)      (((v) >> 10) & 0x3ff)
#define ETM_CONF_MASK           0x000fffffu
#define ETM_CONF_RESET_VAL      0x00010040u

/* FIFO_CNT and BCK_CNT */
#define CNT_MASK                0x7fffffffu
#define CNT_RST                 (1u << 31)

#define CLK_GATE_MASK           0x1u
#define DATE_MASK               0x0fffffffu
#define DATE_RESET_VAL          0x02303240u

/* LP_I2S_VAD_CONF */
#define VAD_EN                  (1u << 0)
#define VAD_RESET               (1u << 1)
#define VAD_FORCE_START         (1u << 2)
/* LP_I2S_RX_MEM_CONF */
#define MEM_THRESHOLD(v)        (((v) >> 9) & 0xff)
#define MEM_CONF_MASK           0x0001fe00u
#define MEM_CONF_RESET_VAL      0x00007e00u
/* LP_I2S_RX_CONF */
#define LP_RX_CONF_FIFOMEM_RESET (1u << 4)
#define LP_RX_CONF_MONO         (1u << 5)
#define LP_RX_CONF_STOP_MODE(v) (((v) >> 13) & 3)
#define LP_RX_CONF_MASK         0x001fffacu
#define LP_RX_CONF_RESET_VAL    0x00009600u
/* LP_I2S_RX_CONF1 */
#define LP_CONF1_TDM_WS_WIDTH(v) ((v) & 0x7f)
#define LP_CONF1_BCK_DIV_NUM(v) (((v) >> 7) & 0x3f)
#define LP_CONF1_BITS_MOD(v)    (((v) >> 13) & 0x1f)
#define LP_CONF1_HALF_SAMPLE(v) (((v) >> 18) & 0x3f)
#define LP_CONF1_CHAN_BITS(v)   (((v) >> 24) & 0x1f)
#define LP_CONF1_MSB_SHIFT      (1u << 29)
#define LP_CONF1_MASK           0x3fffffffu
#define LP_CONF1_RESET_VAL      0x2f3de300u
#define LP_TDM_MASK             0x000f0003u
#define LP_TDM_RESET_VAL        0x00000003u
#define LP_TIMING_MASK          0x33330003u
#define LP_ECO_CONF_MASK        0x1u
#define LP_CLK_GATE_MASK        0xfu
#define LP_CLK_GATE_RESET_VAL   0xau
#define LP_DATE_RESET_VAL       0x02305040u

/* LP_I2S_VAD_PARAMn */
#define VP0_MIN_ENERGY(v)       ((v) & 0xffff)
#define VP0_INIT_FRAME_NUM(v)   (((v) >> 16) & 0x1ff)
#define VP0_MASK                0x01ffffffu
#define VP1_MIN_SPEECH(v)       ((v) & 0xf)
#define VP1_MAX_SPEECH(v)       (((v) >> 4) & 0x7f)
#define VP1_HANGOVER_SPEECH(v)  (((v) >> 11) & 0x1f)
#define VP1_HANGOVER_SILENT(v)  (((v) >> 16) & 0xff)
#define VP1_SKIP_BAND_ENERGY    (1u << 31)
#define VP2_NOISE_AMP_DOWN(v)   ((v) & 0xffff)
#define VP2_NOISE_AMP_UP(v)     (((v) >> 16) & 0xffff)

static const uint32_t vad_param_reset[9] = {
    0x00c81388, 0x281e1e43, 0x7eb86666, 0x7d717fdf, 0x799a6666,
    0x7c287d71, 0xb4007d00, 0x01487eb8, 0x20005040,
};

/* What a unit is doing */
enum {
    MODE_OFF,
    MODE_TDM,
    MODE_PDM,
};

/* A frame's data: 16-bit PCM from the PDM-to-PCM converter */
#define PCM_BITS                16

/* Bound on the events one timer callback runs to catch up */
#define MAX_CATCH_UP            65536

/* GDMA-AHB's peripheral numbers of I2S0 to I2S2 */
static const unsigned i2s_gdma_periph[ESP32P4_I2S_HP_COUNT] = {
    ESP32P4_GDMA_AHB_I2S0, ESP32P4_GDMA_AHB_I2S1, ESP32P4_GDMA_AHB_I2S2,
};

/* The matrix signals (TRM table 10.12-1, 10.13-1) */
static const int8_t out_sig[ESP32P4_I2S_COUNT][ESP32P4_I2S_OUT_COUNT] = {
    [0] = {
        [ESP32P4_I2S_OUT_O_BCK] = ESP32P4_SIG_I2S0_O_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_O_WS] = ESP32P4_SIG_I2S0_O_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_O_SD] = ESP32P4_SIG_I2S0_O_SD_PAD_OUT,
        [ESP32P4_I2S_OUT_O_SD1] = ESP32P4_SIG_I2S0_O_SD1_PAD_OUT,
        [ESP32P4_I2S_OUT_I_BCK] = ESP32P4_SIG_I2S0_I_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_I_WS] = ESP32P4_SIG_I2S0_I_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_MCLK] = ESP32P4_SIG_I2S0_MCLK_PAD_OUT,
    },
    [1] = {
        [ESP32P4_I2S_OUT_O_BCK] = ESP32P4_SIG_I2S1_O_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_O_WS] = ESP32P4_SIG_I2S1_O_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_O_SD] = ESP32P4_SIG_I2S1_O_SD_PAD_OUT,
        [ESP32P4_I2S_OUT_O_SD1] = -1,
        [ESP32P4_I2S_OUT_I_BCK] = ESP32P4_SIG_I2S1_I_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_I_WS] = ESP32P4_SIG_I2S1_I_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_MCLK] = ESP32P4_SIG_I2S1_MCLK_PAD_OUT,
    },
    [2] = {
        [ESP32P4_I2S_OUT_O_BCK] = ESP32P4_SIG_I2S2_O_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_O_WS] = ESP32P4_SIG_I2S2_O_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_O_SD] = ESP32P4_SIG_I2S2_O_SD_PAD_OUT,
        [ESP32P4_I2S_OUT_O_SD1] = -1,
        [ESP32P4_I2S_OUT_I_BCK] = ESP32P4_SIG_I2S2_I_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_I_WS] = ESP32P4_SIG_I2S2_I_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_MCLK] = ESP32P4_SIG_I2S2_MCLK_PAD_OUT,
    },
    [ESP32P4_I2S_LP_ID] = {
        [ESP32P4_I2S_OUT_O_BCK] = -1,
        [ESP32P4_I2S_OUT_O_WS] = -1,
        [ESP32P4_I2S_OUT_O_SD] = -1,
        [ESP32P4_I2S_OUT_O_SD1] = -1,
        [ESP32P4_I2S_OUT_I_BCK] = ESP32P4_LP_SIG_I2S_I_BCK_PAD_OUT,
        [ESP32P4_I2S_OUT_I_WS] = ESP32P4_LP_SIG_I2S_I_WS_PAD_OUT,
        [ESP32P4_I2S_OUT_MCLK] = -1,
    },
};

static const int8_t in_sig[ESP32P4_I2S_COUNT][ESP32P4_I2S_IN_COUNT] = {
    [0] = {
        [ESP32P4_I2S_IN_O_BCK] = ESP32P4_SIG_I2S0_O_BCK_PAD_IN,
        [ESP32P4_I2S_IN_O_WS] = ESP32P4_SIG_I2S0_O_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_BCK] = ESP32P4_SIG_I2S0_I_BCK_PAD_IN,
        [ESP32P4_I2S_IN_I_WS] = ESP32P4_SIG_I2S0_I_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_SD] = ESP32P4_SIG_I2S0_I_SD_PAD_IN,
        [ESP32P4_I2S_IN_I_SD1] = ESP32P4_SIG_I2S0_I_SD1_PAD_IN,
        [ESP32P4_I2S_IN_I_SD2] = ESP32P4_SIG_I2S0_I_SD2_PAD_IN,
        [ESP32P4_I2S_IN_I_SD3] = ESP32P4_SIG_I2S0_I_SD3_PAD_IN,
        [ESP32P4_I2S_IN_MCLK] = ESP32P4_SIG_I2S0_MCLK_PAD_IN,
    },
    [1] = {
        [ESP32P4_I2S_IN_O_BCK] = ESP32P4_SIG_I2S1_O_BCK_PAD_IN,
        [ESP32P4_I2S_IN_O_WS] = ESP32P4_SIG_I2S1_O_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_BCK] = ESP32P4_SIG_I2S1_I_BCK_PAD_IN,
        [ESP32P4_I2S_IN_I_WS] = ESP32P4_SIG_I2S1_I_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_SD] = ESP32P4_SIG_I2S1_I_SD_PAD_IN,
        [ESP32P4_I2S_IN_I_SD1] = -1,
        [ESP32P4_I2S_IN_I_SD2] = -1,
        [ESP32P4_I2S_IN_I_SD3] = -1,
        [ESP32P4_I2S_IN_MCLK] = ESP32P4_SIG_I2S1_MCLK_PAD_IN,
    },
    [2] = {
        [ESP32P4_I2S_IN_O_BCK] = ESP32P4_SIG_I2S2_O_BCK_PAD_IN,
        [ESP32P4_I2S_IN_O_WS] = ESP32P4_SIG_I2S2_O_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_BCK] = ESP32P4_SIG_I2S2_I_BCK_PAD_IN,
        [ESP32P4_I2S_IN_I_WS] = ESP32P4_SIG_I2S2_I_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_SD] = ESP32P4_SIG_I2S2_I_SD_PAD_IN,
        [ESP32P4_I2S_IN_I_SD1] = -1,
        [ESP32P4_I2S_IN_I_SD2] = -1,
        [ESP32P4_I2S_IN_I_SD3] = -1,
        [ESP32P4_I2S_IN_MCLK] = ESP32P4_SIG_I2S2_MCLK_PAD_IN,
    },
    [ESP32P4_I2S_LP_ID] = {
        [ESP32P4_I2S_IN_O_BCK] = -1,
        [ESP32P4_I2S_IN_O_WS] = -1,
        [ESP32P4_I2S_IN_I_BCK] = ESP32P4_LP_SIG_I2S_I_BCK_PAD_IN,
        [ESP32P4_I2S_IN_I_WS] = ESP32P4_LP_SIG_I2S_I_WS_PAD_IN,
        [ESP32P4_I2S_IN_I_SD] = ESP32P4_LP_SIG_I2S_I_SD_PAD_IN,
        [ESP32P4_I2S_IN_I_SD1] = -1,
        [ESP32P4_I2S_IN_I_SD2] = -1,
        [ESP32P4_I2S_IN_I_SD3] = -1,
        [ESP32P4_I2S_IN_MCLK] = -1,
    },
};

int esp32p4_i2s_out_signal(unsigned id, ESP32P4I2sOutLine line)
{
    assert(id < ESP32P4_I2S_COUNT && line < ESP32P4_I2S_OUT_COUNT);
    return out_sig[id][line];
}

int esp32p4_i2s_in_signal(unsigned id, ESP32P4I2sInLine line)
{
    assert(id < ESP32P4_I2S_COUNT && line < ESP32P4_I2S_IN_COUNT);
    return in_sig[id][line];
}

static bool i2s_is_lp(ESP32P4I2sState *s)
{
    return ESP32P4_I2S_GET_CLASS(s)->lp;
}

static const char *i2s_name(ESP32P4I2sState *s)
{
    static const char *const names[ESP32P4_I2S_COUNT] = {
        "I2S0", "I2S1", "I2S2", "LP I2S",
    };

    return names[s->id];
}

static void i2s_update_units(ESP32P4I2sState *s, bool reanchor);
static int64_t unit_event_time(ESP32P4I2sState *s, ESP32P4I2sUnit *u,
                               uint64_t n);
static void i2s_tx_resume(ESP32P4I2sState *s);
static void i2s_rx_flush(ESP32P4I2sState *s);
static void i2s_hung_update(ESP32P4I2sState *s, ESP32P4I2sUnit *u,
                            bool progress);

/* ---- Interrupts and ETM events ---- */

static void i2s_update_irq(ESP32P4I2sState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static void i2s_raise(ESP32P4I2sState *s, uint32_t mask)
{
    s->int_raw |= mask;
    i2s_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Pulse ETM event @evt, carrying the time of what raises it
 * (esp32_line_time()): the end of the period being run, or the access.
 */
static void i2s_event(ESP32P4I2sState *s, unsigned evt)
{
    if (!i2s_is_lp(s)) {
        int64_t t = esp32_line_time();

        esp32_line_set_at(s->etm_event[evt], 1, t);
        esp32_line_set_at(s->etm_event[evt], 0, t);
    }
}

/* ---- Clocks ---- */

static bool i2s_bus_on(ESP32P4I2sState *s)
{
    return clock_is_enabled(s->apb_clk);
}

static Clock *unit_clk(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    return u == &s->tx ? s->tx_clk : s->rx_clk;
}

/* ---- The unit configuration ---- */

/* A unit's configuration, as its synchronised registers give it. */
typedef struct I2sCfg {
    bool tdm;
    bool pdm;
    bool slave;
    bool msb_shift;
    bool idle_pol;
    bool left_align;
    bool bit_order;
    bool big_endian;
    bool fill24;
    bool pcm_bypass;
    bool mono;
    bool fst_vld;
    bool chan_equal;
    bool skip_msk;
    bool stop_en;
    uint8_t pcm_conf;
    uint8_t chan_mod;
    uint8_t stop_mode;
    uint32_t bck_div;       /* M, BCK_DIV_NUM + 1 */
    uint32_t hs;            /* HALF_SAMPLE_BITS + 1 */
    uint32_t ws_w;          /* TDM_WS_WIDTH + 1 */
    uint32_t cb;            /* TDM_CHAN_BITS + 1 */
    uint32_t n;             /* TDM_TOT_CHAN_NUM + 1 */
    uint32_t chan_en;
    uint32_t bits;          /* the valid data's width */
    uint32_t bytes;         /* and its bytes in memory */
    uint32_t single;
    /* PDM */
    bool conv;
    bool dac_mode;
    bool dac_2out;
    bool dsr16;
    bool hp_bypass;
    uint32_t osr2;
    int scale_shift;
    uint32_t amplify;
    uint32_t mult0;
    uint32_t mult5;
} I2sCfg;

static void unit_cfg(ESP32P4I2sState *s, ESP32P4I2sUnit *u, I2sCfg *c)
{
    bool tx = u == &s->tx;
    uint32_t v = u->conf, v1 = u->conf1;
    uint32_t bits_mod;

    memset(c, 0, sizeof(*c));
    c->slave = v & CONF_SLAVE_MOD;
    c->idle_pol = v & CONF_WS_IDLE_POL;
    c->left_align = v & CONF_LEFT_ALIGN;
    c->bit_order = v & CONF_BIT_ORDER;
    c->big_endian = v & CONF_BIG_ENDIAN;
    c->fill24 = v & CONF_24_FILL_EN;
    c->pcm_bypass = v & CONF_PCM_BYPASS;
    c->pcm_conf = CONF_PCM_CONF(v);
    c->fst_vld = v & CONF_MONO_FST_VLD;
    c->tdm = v & CONF_TDM_EN;
    c->pdm = v & CONF_PDM_EN;
    c->chan_en = TDM_CHAN_EN(u->tdm_ctrl);
    c->n = TDM_TOT_CHAN_NUM(u->tdm_ctrl) + 1;
    c->single = u->single_data;
    if (i2s_is_lp(s)) {
        c->mono = v & LP_RX_CONF_MONO;
        c->stop_mode = LP_RX_CONF_STOP_MODE(v);
        c->msb_shift = v1 & LP_CONF1_MSB_SHIFT;
        c->bck_div = LP_CONF1_BCK_DIV_NUM(v1) + 1;
        c->hs = LP_CONF1_HALF_SAMPLE(v1) + 1;
        c->ws_w = LP_CONF1_TDM_WS_WIDTH(v1) + 1;
        c->cb = LP_CONF1_CHAN_BITS(v1) + 1;
        bits_mod = LP_CONF1_BITS_MOD(v1);
        c->chan_en &= 3;
    } else {
        c->bck_div = CONF_BCK_DIV_NUM(v) + 1;
        c->hs = CONF1_HALF_SAMPLE(v1) + 1;
        c->ws_w = CONF1_TDM_WS_WIDTH(v1) + 1;
        c->cb = CONF1_TDM_CHAN_BITS(v1) + 1;
        bits_mod = CONF1_BITS_MOD(v1);
        if (tx) {
            c->mono = v & TX_CONF_MONO;
            c->msb_shift = v & TX_CONF_MSB_SHIFT;
            c->stop_en = v & TX_CONF_STOP_EN;
            c->chan_equal = v & TX_CONF_CHAN_EQUAL;
            c->chan_mod = TX_CONF_CHAN_MOD(v);
            c->skip_msk = u->tdm_ctrl & TDM_SKIP_MSK_EN;
        } else {
            c->mono = v & RX_CONF_MONO;
            c->msb_shift = v & RX_CONF_MSB_SHIFT;
            c->stop_mode = RX_CONF_STOP_MODE(v);
        }
    }
    c->bits = bits_mod + 1;
    if (bits_mod == 23 && c->fill24) {
        c->bits = 32;
    }
    c->bytes = DIV_ROUND_UP(c->bits, 8);
    if (tx) {
        c->conv = u->pdm_conf & P2D_CONV_EN;
        c->dac_mode = u->pdm_conf & P2D_DAC_MODE_EN;
        c->dac_2out = u->pdm_conf & P2D_DAC_2OUT_EN;
        c->osr2 = P2D_SINC_OSR2(u->pdm_conf);
        c->scale_shift = (int)P2D_HP_IN_SHIFT(u->pdm_conf) - 1 +
                         (int)P2D_LP_IN_SHIFT(u->pdm_conf) - 1 +
                         (int)P2D_SINC_IN_SHIFT(u->pdm_conf) - 1 +
                         (int)P2D_SD_IN_SHIFT(u->pdm_conf) - 1;
        c->mult0 = P2D1_MULT12_0(u->pdm_conf1);
        c->mult5 = P2D1_MULT12_5(u->pdm_conf1);
    } else {
        c->conv = u->pdm_conf & D2P_EN;
        c->dsr16 = u->pdm_conf & D2P_DSR_16_EN;
        c->hp_bypass = u->pdm_conf & D2P_HP_BYPASS;
        c->amplify = D2P_AMPLIFY(u->pdm_conf);
        c->mult0 = D2P_MULT12_0(u->pdm_conf);
        c->mult5 = D2P_MULT12_5(u->pdm_conf);
    }
}

/*
 * The BCK cycles of a master's period: a TDM frame, or a PDM sample: the
 * PCM-to-PDM converter's 64 * OSR2, the PDM-to-PCM's 64 or 128, a raw
 * channel word's bits.
 */
static uint32_t unit_period_cycles(ESP32P4I2sUnit *u, const I2sCfg *c,
                                   bool tx)
{
    if (c->tdm) {
        return 2 * c->hs;
    }
    if (c->conv) {
        if (tx) {
            return 64 * MAX(c->osr2, 1);
        }
        return c->dsr16 ? 128 : 64;
    }
    return c->bits;
}

/* ---- A-law and u-law (ITU-T G.711) ---- */

static uint8_t alaw_encode(int16_t pcm)
{
    static const int seg_end[8] = {
        0x1f, 0x3f, 0x7f, 0xff, 0x1ff, 0x3ff, 0x7ff, 0xfff,
    };
    int v = pcm >> 3;
    uint8_t mask;
    int seg = 0;
    uint8_t a;

    if (v >= 0) {
        mask = 0xd5;
    } else {
        mask = 0x55;
        v = -v - 1;
    }
    while (seg < 8 && v > seg_end[seg]) {
        seg++;
    }
    if (seg >= 8) {
        return 0x7f ^ mask;
    }
    a = seg << 4;
    a |= (seg < 2 ? v >> 1 : v >> seg) & 0xf;
    return a ^ mask;
}

static int16_t alaw_decode(uint8_t a)
{
    int t, seg;

    a ^= 0x55;
    t = (a & 0xf) << 4;
    seg = (a >> 4) & 7;
    switch (seg) {
    case 0:
        t += 8;
        break;
    case 1:
        t += 0x108;
        break;
    default:
        t += 0x108;
        t <<= seg - 1;
        break;
    }
    return (a & 0x80) ? t : -t;
}

static uint8_t ulaw_encode(int16_t pcm)
{
    static const int seg_end[8] = {
        0x3f, 0x7f, 0xff, 0x1ff, 0x3ff, 0x7ff, 0xfff, 0x1fff,
    };
    int v = pcm >> 2;
    uint8_t mask;
    int seg = 0;

    if (v < 0) {
        v = -v;
        mask = 0x7f;
    } else {
        mask = 0xff;
    }
    if (v > 8159) {
        v = 8159;
    }
    v += 0x84 >> 2;
    while (seg < 8 && v > seg_end[seg]) {
        seg++;
    }
    if (seg >= 8) {
        return 0x7f ^ mask;
    }
    return ((seg << 4) | ((v >> (seg + 1)) & 0xf)) ^ mask;
}

static int16_t ulaw_decode(uint8_t u)
{
    int t;

    u = ~u;
    t = ((u & 0xf) << 3) + 0x84;
    t <<= (u >> 4) & 7;
    return (u & 0x80) ? 0x84 - t : t - 0x84;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The compress/decompress units (TRM 49.9.1.3): PCM_CONF 0 and 2 expand
 * an A-law or u-law code in the low byte into a 16-bit sample, 1 and 3
 * compress the low 16 bits into a code.
 */
static uint32_t i2s_pcm(const I2sCfg *c, uint32_t v)
{
    if (c->pcm_bypass) {
        return v;
    }
    switch (c->pcm_conf) {
    case 0:
        return (uint32_t)(int32_t)alaw_decode(v);
    case 1:
        return alaw_encode((int16_t)v);
    case 2:
        return (uint32_t)(int32_t)ulaw_decode(v);
    default:
        return ulaw_encode((int16_t)v);
    }
}

static uint32_t mask_bits(uint32_t bits)
{
    return bits >= 32 ? UINT32_MAX : (1u << bits) - 1;
}

/* Reverse the low @bits bits of @v. */
static uint32_t reverse_bits(uint32_t v, uint32_t bits)
{
    return revbit32(v) >> (32 - bits);
}

/* Reverse the order of the low @bytes bytes of @v. */
static uint32_t reverse_bytes(uint32_t v, uint32_t bytes)
{
    return bswap32(v) >> (8 * (4 - bytes));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Channel valid data, as read from memory, to the bits a channel sends
 * (TRM figure 49.9-1): A-law/u-law, then the channel's TDM_CHAN_BITS + 1
 * bits, the valid data left- or right-aligned or, wider than the channel,
 * its low bits; then BIT_ORDER reverses the valid bits.
 */
static uint32_t tx_format(const I2sCfg *c, uint32_t d)
{
    uint32_t v = i2s_pcm(c, d & mask_bits(c->bits)) & mask_bits(c->bits);
    uint32_t w = MIN(c->bits, c->cb);

    if (c->cb > c->bits && c->left_align) {
        v <<= c->cb - c->bits;
        if (c->bit_order) {
            v = reverse_bits(v >> (c->cb - w), w) << (c->cb - w);
        }
    } else {
        v &= mask_bits(c->cb);
        if (c->bit_order) {
            v = reverse_bits(v, w);
        }
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A received channel's bits, first bit highest, to the data stored (TRM
 * 49.10.2): BIT_ORDER, then the storage width, the high bits with
 * LEFT_ALIGN or the low ones of a wider channel, zero-filled for a
 * narrower one; then BIG_ENDIAN, then A-law/u-law.
 */
static uint32_t rx_format(const I2sCfg *c, uint32_t raw, uint32_t cb)
{
    uint32_t v = raw & mask_bits(cb);

    if (c->bit_order) {
        v = reverse_bits(v, cb);
    }
    if (c->bits < cb) {
        v = c->left_align ? v >> (cb - c->bits) : v & mask_bits(c->bits);
    }
    if (c->big_endian) {
        v = reverse_bytes(v, c->bytes);
    }
    return i2s_pcm(c, v) & mask_bits(c->bytes * 8);
}

/* ---- FIFOs ---- */

static void fifo_reset(ESP32P4I2sFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static uint32_t fifo_room(ESP32P4I2sFifo *f)
{
    return ESP32P4_I2S_FIFO_BYTES - f->num;
}

static void fifo_put(ESP32P4I2sFifo *f, uint8_t b, bool eof)
{
    uint32_t i = (f->head + f->num) % ESP32P4_I2S_FIFO_BYTES;

    f->data[i] = b;
    f->eof[i] = eof;
    f->num++;
}

static uint8_t fifo_get(ESP32P4I2sFifo *f, bool *eof)
{
    uint32_t i = f->head;

    f->head = (f->head + 1) % ESP32P4_I2S_FIFO_BYTES;
    f->num--;
    if (eof) {
        *eof = f->eof[i];
    }
    return f->data[i];
}

/* ---- DMA ---- */

/* Fill the transmit FIFO from GDMA as far as it has room. */
static void i2s_tx_fill(ESP32P4I2sState *s)
{
    unsigned periph;
    uint8_t buf[ESP32P4_I2S_FIFO_BYTES];
    size_t n;
    bool eof;

    if (i2s_is_lp(s) || !s->gdma || !i2s_bus_on(s)) {
        return;
    }
    periph = i2s_gdma_periph[s->id];
    while (fifo_room(&s->tx_fifo)) {
        n = esp32p4_gdma_pull(s->gdma, periph, buf, fifo_room(&s->tx_fifo),
                              &eof);
        if (n == 0 && !eof) {
            break;
        }
        for (size_t i = 0; i < n; i++) {
            fifo_put(&s->tx_fifo, buf[i], false);
        }
    }
    i2s_hung_update(s, &s->tx, false);
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static void i2s_gdma_tx_ready(void *opaque)
{
    ESP32P4I2sState *s = opaque;

    i2s_tx_fill(s);
    i2s_tx_resume(s);
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static void i2s_gdma_rx_ready(void *opaque)
{
    ESP32P4I2sState *s = opaque;

    i2s_rx_flush(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Move received bytes on to GDMA as far as it takes them, each byte that
 * ends an RX_EOF_NUM run with its EOF.
 */
static void i2s_rx_flush(ESP32P4I2sState *s)
{
    unsigned periph;
    uint8_t buf[ESP32P4_I2S_FIFO_BYTES];
    uint32_t n = 0, i;
    size_t took;
    bool eof = false;

    if (i2s_is_lp(s) || !s->gdma || !i2s_bus_on(s)) {
        return;
    }
    periph = i2s_gdma_periph[s->id];
    while (s->rx_fifo.num) {
        /* The bytes up to and including the next EOF mark */
        n = 0;
        eof = false;
        for (i = 0; i < s->rx_fifo.num && !eof; i++) {
            uint32_t k = (s->rx_fifo.head + i) % ESP32P4_I2S_FIFO_BYTES;

            buf[n++] = s->rx_fifo.data[k];
            eof = s->rx_fifo.eof[k];
        }
        took = esp32p4_gdma_push(s->gdma, periph, buf, n,
                                 eof ? ESP32P4_GDMA_EOF : 0);
        for (i = 0; i < took; i++) {
            fifo_get(&s->rx_fifo, NULL);
        }
        if (took < n) {
            break;
        }
    }
}

/* ---- The FIFO timeout ---- */

/* When the timeout's cycles of the bus clock from @from have run. */
static int64_t i2s_hung_end(ESP32P4I2sState *s, int64_t from)
{
    uint64_t cycles = (uint64_t)HUNG_TIMEOUT(s->lc_hung_conf) *
                      (HUNG_TICK_CYCLES >> HUNG_SHIFT(s->lc_hung_conf));
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->apb_clk, from);

    return esp32p4_clk_edges_after(&e, from, cycles);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * LC_HUNG_CONF times a unit out while it is stuck (TRM 49.12, 50.9): a
 * started transmitter with data in its FIFO that it does not send (a
 * slave with no BCK), a started receiver that receives nothing. The tick
 * counter counts the bus clock to 88000 >> SHIFT and the hung counter
 * fires at TIMEOUT ticks; it fires once per stall, and restarts with each
 * channel data the unit moves.
 */
static void i2s_hung_update(ESP32P4I2sState *s, ESP32P4I2sUnit *u,
                            bool progress)
{
    bool tx = u == &s->tx;
    bool stuck = u->mode != MODE_OFF && !u->stalled &&
                 (!tx || s->tx_fifo.num != 0);

    if (!stuck || !(s->lc_hung_conf & HUNG_ENA) || !i2s_bus_on(s) ||
        HUNG_TIMEOUT(s->lc_hung_conf) == 0) {
        timer_del(&u->hung_timer);
        u->hung_fired = false;
        return;
    }
    if (progress) {
        timer_del(&u->hung_timer);
        u->hung_fired = false;
    }
    if (!u->hung_fired && !timer_pending(&u->hung_timer)) {
        timer_mod_ns(&u->hung_timer,
                     i2s_hung_end(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    }
}

static void i2s_tx_hung_cb(void *opaque)
{
    ESP32P4I2sState *s = opaque;

    s->tx.hung_fired = true;
    i2s_raise(s, INT_TX_HUNG);
}

static void i2s_rx_hung_cb(void *opaque)
{
    ESP32P4I2sState *s = opaque;

    s->rx.hung_fired = true;
    i2s_raise(s, i2s_is_lp(s) ? LP_INT_RX_HUNG : INT_RX_HUNG);
}

/* ---- The PDM converters ---- */

/*
 * One of the converters' high-pass stages, a first-order IIR whose pole is
 * (504 + mult) / 512 (TRM registers 49.6, 49.7).
 */
static int32_t hp_stage(ESP32P4I2sPdmChan *p, int k, uint32_t mult,
                        int32_t x)
{
    int64_t y = ((int64_t)(504 + mult) *
                 ((int64_t)p->hp_y[k] + x - p->hp_x[k])) >> 9;

    p->hp_x[k] = x;
    p->hp_y[k] = (int32_t)y;
    return (int32_t)y;
}

static int32_t clamp16(int64_t v)
{
    return MIN(MAX(v, INT16_MIN), INT16_MAX);
}

/* The PCM-to-PDM converter takes a sample (TRM 49.9.2.2). */
static void pdm_mod_load(ESP32P4I2sPdmChan *p, const I2sCfg *c, int16_t x)
{
    int64_t v = hp_stage(p, 0, c->mult0, x);

    v = hp_stage(p, 1, c->mult5, (int32_t)v);
    v = c->scale_shift >= 0 ? v << c->scale_shift : v >> -c->scale_shift;
    p->pcm = clamp16(v);
}

/* One bit of a first-order sigma-delta modulator, holding the sample. */
static bool pdm_mod_bit(ESP32P4I2sPdmChan *p)
{
    bool bit = p->integ[0] >= 0;

    p->integ[0] += p->pcm - (bit ? 32768 : -32768);
    return bit;
}

/* One bit into a channel's third-order CIC decimator. */
static void pdm_cic_bit(ESP32P4I2sPdmChan *p, bool bit)
{
    uint64_t i0 = p->integ[0], i1 = p->integ[1], i2 = p->integ[2];

    i0 += bit ? 1 : (uint64_t)-1;
    i1 += i0;
    i2 += i1;
    p->integ[0] = i0;
    p->integ[1] = i1;
    p->integ[2] = i2;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The PDM-to-PCM converter's sample for a channel: the CIC's combs, the
 * high-pass stages unless RX_PDM_HP_BYPASS, RX_PDM2PCM_AMPLIFY_NUM (0
 * mutes), as 16 bits.
 */
static uint32_t pdm_cic_out(ESP32P4I2sPdmChan *p, const I2sCfg *c,
                            uint32_t r)
{
    uint64_t x = p->integ[2], d;
    int64_t v;

    for (int k = 0; k < 3; k++) {
        d = x - (uint64_t)p->comb[k];
        p->comb[k] = x;
        x = d;
    }
    v = ((int64_t)x * 32768) / ((int64_t)r * r * r);
    if (!c->hp_bypass) {
        v = hp_stage(p, 0, c->mult0, clamp16(v));
        v = hp_stage(p, 1, c->mult5, (int32_t)v);
    }
    return (uint16_t)clamp16(v * c->amplify);
}

/* ---- The transmitter ---- */

/* Whether the transmit FIFO, topped up from GDMA, holds @bytes. */
static bool tx_has(ESP32P4I2sState *s, uint32_t bytes)
{
    if (s->tx_fifo.num < bytes) {
        i2s_tx_fill(s);
    }
    return s->tx_fifo.num >= bytes;
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static uint32_t tx_take(ESP32P4I2sState *s, const I2sCfg *c)
{
    uint32_t v = 0;

    for (uint32_t i = 0; i < c->bytes; i++) {
        v |= (uint32_t)fifo_get(&s->tx_fifo, NULL) << (8 * i);
    }
    if (c->big_endian) {
        v = reverse_bytes(v, c->bytes);
    }
    s->tx_fifo_cnt = (s->tx_fifo_cnt + 1) & CNT_MASK;
    if (++s->tx.words == ETM_TX_WORD_NUM(s->etm_conf)) {
        i2s_event(s, ESP32P4_I2S_EVT_X_WORDS_SENT);
    }
    i2s_hung_update(s, &s->tx, true);
    i2s_tx_fill(s);
    return v;
}

/* The channel data a frame or PDM sample takes from the FIFO. */
static uint32_t tx_need(const I2sCfg *c)
{
    uint32_t n = 0;

    if (c->pdm) {
        return c->mono ? 1 : 2;
    }
    for (uint32_t k = 0; k < MIN(c->n, ESP32P4_I2S_SLOTS); k++) {
        if (c->skip_msk || (c->chan_en & BIT(k))) {
            n++;
        }
    }
    return n;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The transmitter has run out of data (TRM 49.8.1): TX_DONE, once per
 * running out.
 */
static void tx_drained(ESP32P4I2sState *s)
{
    if (!s->tx.drained) {
        s->tx.drained = true;
        i2s_raise(s, INT_TX_DONE);
        i2s_event(s, ESP32P4_I2S_EVT_TX_DONE);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A TDM frame's channels (TRM 49.9.2.1). Returns false if a master with
 * TX_STOP_EN must stop for want of data.
 */
static bool tdm_tx_load(ESP32P4I2sState *s, const I2sCfg *c, bool master,
                        uint32_t *slot)
{
    ESP32P4I2sUnit *u = &s->tx;
    uint32_t need = tx_need(c);

    if (!tx_has(s, need * c->bytes)) {
        tx_drained(s);
        if (c->stop_en) {
            if (master) {
                u->stalled = true;
                return false;
            }
            memset(slot, 0, sizeof(uint32_t) * ESP32P4_I2S_SLOTS);
            return true;
        }
        memcpy(slot, u->last, sizeof(u->last));
        return true;
    }
    u->drained = false;
    for (uint32_t k = 0; k < ESP32P4_I2S_SLOTS; k++) {
        uint32_t v = 0;

        if (k < c->n) {
            if (c->chan_en & BIT(k)) {
                v = tx_format(c, tx_take(s, c));
            } else {
                if (c->skip_msk) {
                    tx_take(s, c);
                }
                v = c->chan_equal ? u->last_chan : tx_format(c, c->single);
            }
            u->last_chan = v;
        }
        slot[k] = v;
    }
    memcpy(u->last, slot, sizeof(u->last));
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Whether a frame starts at this BCK cycle: a master's own frame boundary;
 * for a slave, WS changing to WS_IDLE_POL, its first cycle with WS there
 * counting as a change.
 */
static bool frame_sync(ESP32P4I2sUnit *u, const I2sCfg *c, bool ws,
                       int boundary)
{
    bool sync;

    if (boundary >= 0) {
        sync = boundary;
    } else {
        sync = ws == c->idle_pol && (!u->started || u->prev_ws != ws);
    }
    u->started = true;
    u->prev_ws = ws;
    return sync;
}

/* The previous frame keeps the line for the MSB_SHIFT cycle it may need. */
static void frame_new(ESP32P4I2sUnit *u, const I2sCfg *c)
{
    if (c->msb_shift && u->cur.active) {
        u->prev = u->cur;
        u->prev.limit = u->prev.pos + 1;
    } else {
        u->prev.active = false;
    }
    memset(&u->cur, 0, sizeof(u->cur));
    u->cur.active = true;
    u->cur.limit = UINT32_MAX;
}

/* tdm_tx_cycle()'s mark of a master's last cycle before it stops. */
#define TX_LAST_CYCLE   2

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * One BCK cycle of a TDM transmitter whose WS is ws: the bit it drives.
 * boundary is the master's frame boundary for the master itself, -1 for a
 * slave. A master that must stop for want of data at a frame boundary
 * still gives the cycle by which MSB_SHIFT delays the last frame's last
 * bit, and marks it TX_LAST_CYCLE.
 */
static int tdm_tx_cycle(ESP32P4I2sState *s, const I2sCfg *c, bool ws,
                        int boundary)
{
    ESP32P4I2sUnit *u = &s->tx;
    ESP32P4I2sFrame *f[2] = { &u->prev, &u->cur };
    uint32_t shift = c->msb_shift, span = c->n * c->cb;
    int last = 0;
    bool bit = false;

    if (frame_sync(u, c, ws, boundary)) {
        uint32_t slot[ESP32P4_I2S_SLOTS];

        if (!tdm_tx_load(s, c, boundary > 0, slot)) {
            if (!shift || !u->cur.active) {
                return -1;
            }
            frame_new(u, c);
            u->cur.active = false;
            last = TX_LAST_CYCLE;
        } else {
            frame_new(u, c);
            memcpy(u->cur.slot, slot, sizeof(slot));
        }
    }
    for (int i = 0; i < 2; i++) {
        int64_t p;

        if (!f[i]->active) {
            continue;
        }
        if (f[i]->pos >= f[i]->limit) {
            f[i]->active = false;
            continue;
        }
        p = (int64_t)f[i]->pos - shift;
        if (p >= 0 && p < span) {
            uint32_t k = p / c->cb, b = p % c->cb;

            bit = (f[i]->slot[k] >> (c->cb - 1 - b)) & 1;
        }
        f[i]->pos++;
    }
    s->tx_bck_cnt = (s->tx_bck_cnt + 1) & CNT_MASK;
    return bit | last;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The channel mapping of PDM transmission (TRM table 49.9-4): which of the
 * fetched data, 0 or 1, or CONF_SINGLE_DATA (2) each half of the WS period
 * carries, by TX_CHAN_MOD and TX_WS_IDLE_POL.
 */
static void pdm_tx_map(const I2sCfg *c, int src[2])
{
    static const int8_t map[5][2][2] = {
        { { 0, 1 }, { 0, 1 } },
        { { 0, 0 }, { 1, 1 } },
        { { 1, 1 }, { 0, 0 } },
        { { 2, 1 }, { 0, 2 } },
        { { 0, 2 }, { 2, 1 } },
    };
    unsigned mod = c->chan_mod;

    if (mod > 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: reserved TX_CHAN_MOD "
                      "%u\n", mod);
        mod = 0;
    }
    src[0] = map[mod][c->idle_pol][0];
    src[1] = map[mod][c->idle_pol][1];
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A PDM sample's data (TRM table 49.9-3): both channels' in stereo, one
 * with TX_MONO, into the first or the second by TX_MONO_FST_VLD. Returns
 * false if a master with TX_STOP_EN must stop for want of data.
 */
static bool pdm_tx_load(ESP32P4I2sState *s, const I2sCfg *c, bool master,
                        uint32_t d[3])
{
    ESP32P4I2sUnit *u = &s->tx;

    d[0] = d[1] = 0;
    d[2] = c->single;
    if (!tx_has(s, tx_need(c) * c->bytes)) {
        tx_drained(s);
        if (c->stop_en) {
            if (master) {
                u->stalled = true;
                return false;
            }
            return true;
        }
        d[0] = u->last[0];
        d[1] = u->last[1];
        return true;
    }
    u->drained = false;
    if (c->mono) {
        d[c->fst_vld ? 0 : 1] = tx_take(s, c);
    } else {
        d[0] = tx_take(s, c);
        d[1] = tx_take(s, c);
    }
    u->last[0] = d[0];
    u->last[1] = d[1];
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * One BCK cycle, one PDM clock period, of a PDM transmitter: bits[line][h]
 * for the halves of the WS period on SD and SD1. Returns false if a
 * master stops for want of data.
 */
static bool pdm_tx_cycle(ESP32P4I2sState *s, const I2sCfg *c, bool master,
                         uint8_t bits[2][2])
{
    ESP32P4I2sUnit *u = &s->tx;
    uint32_t k = unit_period_cycles(u, c, true);
    uint8_t ch[2];

    if (u->pdm_bit == 0) {
        uint32_t d[3];
        int src[2];

        if (!pdm_tx_load(s, c, master, d)) {
            return false;
        }
        pdm_tx_map(c, src);
        for (int h = 0; h < 2; h++) {
            if (c->conv) {
                pdm_mod_load(&u->pdm[h], c, (int16_t)d[src[h]]);
            } else {
                u->pdm[h].word = d[src[h]] & mask_bits(c->bits);
            }
        }
    }
    for (int h = 0; h < 2; h++) {
        if (c->conv) {
            ch[h] = pdm_mod_bit(&u->pdm[h]);
        } else {
            uint32_t b = c->bit_order ? u->pdm_bit : k - 1 - u->pdm_bit;

            ch[h] = (u->pdm[h].word >> b) & 1;
        }
    }
    if (c->conv && c->dac_mode) {
        bits[0][0] = bits[0][1] = ch[0];
        bits[1][0] = bits[1][1] = c->dac_2out ? ch[1] : 0;
    } else {
        bits[0][0] = ch[0];
        bits[0][1] = ch[1];
        bits[1][0] = bits[1][1] = 0;
    }
    if (++u->pdm_bit >= k) {
        u->pdm_bit = 0;
    }
    s->tx_bck_cnt = (s->tx_bck_cnt + 1) & CNT_MASK;
    return true;
}

/* ---- The receiver ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The LP I2S memory takes a 16-bit sample (TRM 50.8.3): two to a word,
 * the first in the low half. A full memory overwrites its oldest word.
 * RX_MEM_THRESHOLD_INT marks more than RX_MEM_THRESHOLD words.
 */
static void lp_mem_store(ESP32P4I2sState *s, uint32_t word)
{
    s->mem[s->mem_wr] = word;
    s->mem_wr = (s->mem_wr + 1) % ESP32P4_LP_I2S_MEM_WORDS;
    if (s->mem_num == ESP32P4_LP_I2S_MEM_WORDS) {
        s->mem_rd = (s->mem_rd + 1) % ESP32P4_LP_I2S_MEM_WORDS;
    } else {
        s->mem_num++;
    }
    if (s->mem_num > MEM_THRESHOLD(s->rx_mem_conf)) {
        i2s_raise(s, LP_INT_MEM_THRESHOLD);
    }
}

static void vad_frame(ESP32P4I2sState *s);

static void lp_rx_store(ESP32P4I2sState *s, uint16_t sample)
{
    if (!s->mem_half_valid) {
        s->mem_half = sample;
        s->mem_half_valid = true;
    } else {
        lp_mem_store(s, s->mem_half | ((uint32_t)sample << 16));
        s->mem_half_valid = false;
    }
    if (s->vad_conf & VAD_EN) {
        if (++s->vad.samples >= ESP32P4_LP_I2S_VAD_FRAME) {
            s->vad.samples = 0;
            vad_frame(s);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The receiver stores a channel's data: into the FIFO, on to GDMA, or
 * into the LP I2S memory. RX_EOF_NUM + 1 of them end a run: GDMA's EOF,
 * RX_DONE, and with RX_STOP_MODE 1 the receiver suspends. Data that finds
 * the FIFO full is lost, and RX_STOP_MODE 2 suspends the receiver.
 */
static void rx_store(ESP32P4I2sState *s, const I2sCfg *c, uint32_t v)
{
    ESP32P4I2sUnit *u = &s->rx;
    bool eof;

    if (u->stalled) {
        return;
    }
    if (!i2s_is_lp(s) && fifo_room(&s->rx_fifo) < c->bytes) {
        i2s_rx_flush(s);
        if (fifo_room(&s->rx_fifo) < c->bytes) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s receive FIFO "
                          "overflow\n", i2s_name(s));
            if (c->stop_mode == 2) {
                u->stalled = true;
            }
            return;
        }
    }
    eof = ++u->eof_count >= (s->rx_eof_num & RXEOF_NUM_MASK) + 1;
    if (eof) {
        u->eof_count = 0;
    }
    if (i2s_is_lp(s)) {
        lp_rx_store(s, v);
    } else {
        for (uint32_t i = 0; i < c->bytes; i++) {
            fifo_put(&s->rx_fifo, v >> (8 * i), eof && i == c->bytes - 1);
        }
        i2s_rx_flush(s);
    }
    if (++u->words == ETM_RX_WORD_NUM(s->etm_conf) && !i2s_is_lp(s)) {
        i2s_event(s, ESP32P4_I2S_EVT_X_WORDS_RECEIVED);
    }
    i2s_hung_update(s, u, true);
    if (eof) {
        i2s_raise(s, i2s_is_lp(s) ? LP_INT_RX_DONE : INT_RX_DONE);
        i2s_event(s, ESP32P4_I2S_EVT_RX_DONE);
        if (c->stop_mode == 1) {
            u->stalled = true;
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A received TDM channel: stored if its RX_TDM_CHANn_EN is set (TRM
 * 49.10.1.1).
 */
static void tdm_rx_slot(ESP32P4I2sState *s, const I2sCfg *c, uint32_t k,
                        uint32_t raw)
{
    if (k < ESP32P4_I2S_SLOTS && (c->chan_en & BIT(k))) {
        rx_store(s, c, rx_format(c, raw, c->cb));
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * One BCK cycle of a TDM receiver: WS ws, data bit sd.
 */
static void tdm_rx_cycle(ESP32P4I2sState *s, const I2sCfg *c, bool ws,
                         bool sd, int boundary)
{
    ESP32P4I2sUnit *u = &s->rx;
    ESP32P4I2sFrame *f[2] = { &u->prev, &u->cur };
    uint32_t shift = c->msb_shift, span = c->n * c->cb;

    if (frame_sync(u, c, ws, boundary)) {
        frame_new(u, c);
    }
    for (int i = 0; i < 2; i++) {
        int64_t p;

        if (!f[i]->active) {
            continue;
        }
        if (f[i]->pos >= f[i]->limit) {
            f[i]->active = false;
            continue;
        }
        p = (int64_t)f[i]->pos - shift;
        if (p >= 0 && p < span) {
            uint32_t k = p / c->cb, b = p % c->cb;

            f[i]->slot[k] |= (uint32_t)sd << (c->cb - 1 - b);
            if (b == c->cb - 1) {
                tdm_rx_slot(s, c, k, f[i]->slot[k]);
            }
        }
        f[i]->pos++;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * One BCK cycle of a PDM receiver: bits[line][h], the halves of the WS
 * period on SD to SD3. Each PDM sample, the enabled channels (left then
 * right of each line) are stored (TRM 49.10.1.2, table 49.10-1): raw, a
 * BITS_MOD + 1 bit word; through I2S0's PDM-to-PCM converter, 16 bits.
 */
static void pdm_rx_cycle(ESP32P4I2sState *s, const I2sCfg *c,
                         uint8_t bits[4][2])
{
    ESP32P4I2sUnit *u = &s->rx;
    uint32_t k = unit_period_cycles(u, c, false);
    unsigned nch = c->conv ? ESP32P4_I2S_PDM_CHANS : 2;

    for (unsigned ch = 0; ch < nch; ch++) {
        bool bit = bits[ch / 2][ch % 2];

        if (c->conv) {
            pdm_cic_bit(&u->pdm[ch], bit);
        } else {
            u->pdm[ch].word = (u->pdm[ch].word << 1) | bit;
        }
    }
    if (++u->pdm_bit < k) {
        return;
    }
    u->pdm_bit = 0;
    for (unsigned ch = 0; ch < nch; ch++) {
        uint32_t v;

        if (c->conv) {
            v = pdm_cic_out(&u->pdm[ch], c, k);
            if (c->chan_en & BIT(ch)) {
                rx_store(s, c, rx_format(c, v, PCM_BITS));
            }
        } else {
            v = u->pdm[ch].word;
            u->pdm[ch].word = 0;
            if (c->chan_en & BIT(ch)) {
                rx_store(s, c, rx_format(c, v, c->bits));
            }
        }
    }
}

/* ---- The bus between units ---- */

static bool i2s_loopback(ESP32P4I2sState *s)
{
    return !i2s_is_lp(s) && (s->tx_conf & TX_CONF_SIG_LOOPBACK);
}

/*
 * The output line of a controller that drives pad @pad, on the pad or on
 * another pad of the board wire it is on (the GPIO block's pad-wires), as
 * no driver from outside the chip or pad hold overrides.
 */
static bool i2s_pad_source(ESP32P4I2sState *c, int pad,
                           ESP32P4I2sState **src, unsigned *src_line)
{
    ESP32P4GpioState *g = c->gpio;
    uint64_t pads = BIT_ULL(pad);

    if (g->pad_wire[pad]) {
        for (unsigned p = 0; p < ESP32P4_GPIO_PADS; p++) {
            if (g->pad_wire[p] == g->pad_wire[pad]) {
                pads |= BIT_ULL(p);
            }
        }
    }
    if (pads & (g->ext_high | g->ext_low | g->held)) {
        return false;
    }
    for (unsigned k = 0; k < ESP32P4_I2S_COUNT; k++) {
        ESP32P4I2sState *d = k == c->id ? c : c->bus[k];

        if (!d) {
            continue;
        }
        for (unsigned l = 0; l < ESP32P4_I2S_OUT_COUNT; l++) {
            int o = out_sig[k][l];
            uint64_t hit;

            if (o < 0) {
                continue;
            }
            if (k == ESP32P4_I2S_LP_ID) {
                hit = esp32p4_gpio_lp_sig_out_pads(g, o) & pads;
            } else {
                hit = esp32p4_gpio_sig_out_pads(g, o) & pads;
            }
            if (hit) {
                *src = d;
                *src_line = l;
                return true;
            }
        }
    }
    return false;
}

/*
 * The output line of a controller that drives input line @line of @c
 * through the pads, with whether the way inverts it: through the HP
 * matrix's own query; else, between the HP and LP matrices or over a
 * board wire, by the pads (with no inversion).
 */
static bool i2s_route(ESP32P4I2sState *c, unsigned line,
                      ESP32P4I2sState **src, unsigned *src_line, bool *inv)
{
    int sig = in_sig[c->id][line];
    unsigned out;
    int pad;

    *inv = false;
    if (sig < 0 || !c->gpio) {
        return false;
    }
    if (!i2s_is_lp(c)) {
        if (esp32p4_gpio_sig_in_source(c->gpio, sig, &out, inv)) {
            for (unsigned k = 0; k < ESP32P4_I2S_HP_COUNT; k++) {
                ESP32P4I2sState *d = k == c->id ? c : c->bus[k];

                for (unsigned l = 0; l < ESP32P4_I2S_OUT_COUNT; l++) {
                    if (out_sig[k][l] == (int)out && d) {
                        *src = d;
                        *src_line = l;
                        return true;
                    }
                }
            }
            return false;
        }
        *inv = false;
        pad = esp32p4_gpio_sig_in_pad(c->gpio, sig);
    } else {
        pad = esp32p4_gpio_lp_sig_in_pad(c->gpio, sig);
    }
    return pad >= 0 && i2s_pad_source(c, pad, src, src_line);
}

/* The BCK and WS lines a unit drives as master, or follows as slave. */
static void unit_clock_lines(ESP32P4I2sState *c, bool tx, unsigned *bck,
                             unsigned *ws, bool in)
{
    bool o = tx || i2s_loopback(c);

    if (in) {
        *bck = o ? ESP32P4_I2S_IN_O_BCK : ESP32P4_I2S_IN_I_BCK;
        *ws = o ? ESP32P4_I2S_IN_O_WS : ESP32P4_I2S_IN_I_WS;
    } else {
        *bck = o ? ESP32P4_I2S_OUT_O_BCK : ESP32P4_I2S_OUT_I_BCK;
        *ws = o ? ESP32P4_I2S_OUT_O_WS : ESP32P4_I2S_OUT_I_WS;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Whether slave unit (c, tx) runs on the clock of master unit (m, m_tx):
 * the other unit of the same controller with SIG_LOOPBACK, which shares
 * BCK and WS, or a slave whose BCK and WS inputs the pads connect to the
 * master's BCK and WS outputs, WS alone in PDM mode. *inv says whether WS
 * is inverted on the way.
 */
static bool unit_clocked_by(ESP32P4I2sState *c, bool tx, ESP32P4I2sState *m,
                            bool m_tx, bool pdm, bool *inv)
{
    unsigned bck_in, ws_in, bck_out, ws_out, l1, l2;
    ESP32P4I2sState *s1, *s2;
    bool i1;

    *inv = false;
    if (c == m && tx != m_tx) {
        return i2s_loopback(c);
    }
    unit_clock_lines(c, tx, &bck_in, &ws_in, true);
    unit_clock_lines(m, m_tx, &bck_out, &ws_out, false);
    if (!i2s_route(c, ws_in, &s2, &l2, inv) || s2 != m || l2 != ws_out) {
        return false;
    }
    /* A PDM slave takes the PDM clock on WS alone (TRM 49.5.4). */
    return pdm || (i2s_route(c, bck_in, &s1, &l1, &i1) && s1 == m &&
                   l1 == bck_out);
}

typedef struct BusUnit {
    ESP32P4I2sState *c;
    I2sCfg cfg;
    bool inv;
    /* Receivers: each data line's transmitter (index) and its line, or -1 */
    int src[4];
    uint8_t src_line[4];
    bool src_inv[4];
    bool level[4];
    /* Receivers: each data line's device on the pads (index), or -1 */
    int peer[4];
    bool peer_inv[4];
    /* Transmitters: this cycle's bits */
    uint8_t bits[2][2];
} BusUnit;

/* A device on the pads that a master's period clocks */
typedef struct BusPeer {
    const ESP32P4I2sPeer *p;
    /* LRCK: the master's WS (and whether inverted), or the pad's level */
    bool lrck_ws;
    bool lrck_inv;
    bool lrck_level;
    /* The data input: a transmitter's line (index, line), or the level */
    int din_src;
    uint8_t din_line;
    bool din_inv;
    bool din_level;
    /* This cycle's LRCK and data output, and whether the device drives it */
    bool lrck;
    bool dout;
    bool driven;
} BusPeer;

static const unsigned rx_sd_lines[4] = {
    ESP32P4_I2S_IN_I_SD, ESP32P4_I2S_IN_I_SD1,
    ESP32P4_I2S_IN_I_SD2, ESP32P4_I2S_IN_I_SD3,
};

static bool i2s_in(ESP32P4I2sState *s, unsigned line)
{
    return (s->in_level >> line) & 1;
}

static void i2s_drive(ESP32P4I2sState *s, unsigned line, bool level)
{
    qemu_set_irq(s->sig_out[line], level);
}

/*
 * The master stopped its clocks: when they start again its frame starts
 * afresh, and every unit on them takes its first WS at the idle level as
 * a frame's start, as WS rests away from it while the clocks are stopped.
 */
static void bus_stopped(BusUnit *tx, unsigned ntx, BusUnit *rx, unsigned nrx)
{
    for (unsigned t = 0; t < ntx; t++) {
        tx[t].c->tx.started = false;
    }
    for (unsigned j = 0; j < nrx; j++) {
        rx[j].c->rx.started = false;
    }
}

/*
 * The pad input line @line of @c takes its level from, through its
 * matrix, and whether the matrix inverts it on the way in (-1: none).
 */
static int i2s_in_pad(ESP32P4I2sState *c, unsigned line, bool *inv)
{
    int sig = in_sig[c->id][line];

    *inv = false;
    if (sig < 0 || !c->gpio) {
        return -1;
    }
    if (i2s_is_lp(c)) {
        *inv = esp32p4_gpio_lp_sig_in_inverted(c->gpio, sig);
        return esp32p4_gpio_lp_sig_in_pad(c->gpio, sig);
    }
    *inv = esp32p4_gpio_sig_in_inverted(c->gpio, sig);
    return esp32p4_gpio_sig_in_pad(c->gpio, sig);
}

/* Whether pad @pad inverts the output signal it carries. */
static bool i2s_pad_out_inv(ESP32P4I2sState *c, int pad)
{
    bool inv = false;

    if (esp32p4_gpio_pad_out_sig(c->gpio, pad, &inv) < 0) {
        esp32p4_gpio_lp_pad_out_sig(c->gpio, pad, &inv);
    }
    return inv;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Whether master unit (m, m_tx) clocks device @p on the pads: its BCK on
 * the device's SCLK pad. If so, *bp says where its LRCK and data input
 * come from, among @ntx transmitters on the clock.
 */
static bool peer_clocked(ESP32P4I2sState *m, bool m_tx,
                         const ESP32P4I2sPeer *p, BusUnit *tx, unsigned ntx,
                         BusPeer *bp)
{
    ESP32P4GpioState *g = m->gpio;
    ESP32P4I2sState *src;
    unsigned bck_out, ws_out, line;

    unit_clock_lines(m, m_tx, &bck_out, &ws_out, false);
    if (p->sclk_pad < 0 ||
        !i2s_pad_source(m, p->sclk_pad, &src, &line) || src != m ||
        line != bck_out) {
        return false;
    }
    *bp = (BusPeer) { .p = p, .din_src = -1 };
    if (p->lrck_pad >= 0 && i2s_pad_source(m, p->lrck_pad, &src, &line) &&
        src == m && line == ws_out) {
        bp->lrck_ws = true;
        bp->lrck_inv = i2s_pad_out_inv(m, p->lrck_pad);
    } else if (p->lrck_pad >= 0) {
        bp->lrck_level = esp32p4_gpio_pad_level(g, p->lrck_pad);
    }
    if (p->din_pad >= 0 && i2s_pad_source(m, p->din_pad, &src, &line) &&
        (line == ESP32P4_I2S_OUT_O_SD || line == ESP32P4_I2S_OUT_O_SD1)) {
        for (unsigned i = 0; i < ntx; i++) {
            if (tx[i].c == src) {
                bp->din_src = i;
                bp->din_line = line == ESP32P4_I2S_OUT_O_SD1;
                bp->din_inv = i2s_pad_out_inv(m, p->din_pad);
            }
        }
    }
    if (bp->din_src < 0 && p->din_pad >= 0) {
        bp->din_level = esp32p4_gpio_pad_level(g, p->din_pad);
    }
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A master unit's period, ncycles BCK cycles, through every unit and
 * every device on the pads on its clock. Transmitters drive their bits; a
 * device takes LRCK and its data input as the pads carry them, and gives
 * its data output; a receiver takes the bits of the transmitter on the
 * same clock that the pads connect to each of its data inputs, or of the
 * device whose output pad its input takes, or that input's level.
 */
static void i2s_bus_run(ESP32P4I2sState *m, bool m_tx)
{
    BusUnit tx[ESP32P4_I2S_COUNT], rx[ESP32P4_I2S_COUNT + 1];
    BusPeer peers[ESP32P4_I2S_PEERS];
    unsigned ntx = 0, nrx = 0, npeers = 0;
    ESP32P4I2sUnit *mu = m_tx ? &m->tx : &m->rx;
    I2sCfg mc;
    uint32_t ncycles;
    bool pdm, stopped = false;

    unit_cfg(m, mu, &mc);
    pdm = mc.pdm;
    ncycles = unit_period_cycles(mu, &mc, m_tx);

    if (m_tx) {
        tx[ntx++] = (BusUnit) { .c = m, .cfg = mc };
    } else {
        rx[nrx++] = (BusUnit) { .c = m, .cfg = mc };
    }
    for (unsigned k = 0; k < ESP32P4_I2S_COUNT; k++) {
        ESP32P4I2sState *c = k == m->id ? m : m->bus[k];
        bool inv;

        if (!c) {
            continue;
        }
        for (int t = 0; t < 2; t++) {
            ESP32P4I2sUnit *u = t ? &c->tx : &c->rx;
            BusUnit b = { .c = c };

            if ((c == m && t == m_tx) || u->mode == MODE_OFF ||
                (t && i2s_is_lp(c))) {
                continue;
            }
            unit_cfg(c, u, &b.cfg);
            if (!b.cfg.slave && !(c == m && i2s_loopback(c))) {
                continue;
            }
            if (!unit_clocked_by(c, t, m, m_tx, pdm, &inv)) {
                continue;
            }
            if (b.cfg.pdm != pdm) {
                if (!u->logged_mismatch) {
                    qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s %s, in "
                                  "%s mode, is clocked by a %s master\n",
                                  i2s_name(c), t ? "TX" : "RX",
                                  b.cfg.pdm ? "PDM" : "TDM",
                                  pdm ? "PDM" : "TDM");
                    u->logged_mismatch = true;
                }
                continue;
            }
            /* Periods: the slave's clock times 8 against BCK's */
            if (b.cfg.slave && !c->logged_slave_clk &&
                clock_get(unit_clk(c, u)) * 8 >
                clock_get(unit_clk(m, mu)) * mc.bck_div) {
                /* TRM 49.6: a slave's unit clock must be 8 times BCK. */
                qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s %s's clock "
                              "is below 8 times its master's BCK\n",
                              i2s_name(c), t ? "TX" : "RX");
                c->logged_slave_clk = true;
            }
            b.inv = inv;
            if (t) {
                tx[ntx++] = b;
            } else {
                rx[nrx++] = b;
            }
        }
    }
    for (unsigned k = 0; k < m->npeers && m->gpio; k++) {
        if (!peer_clocked(m, m_tx, &m->peers[k], tx, ntx, &peers[npeers])) {
            continue;
        }
        if (pdm) {
            if (!m->logged_peer_pdm) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s's PDM clock "
                              "reaches the SCLK of a device on the pads, "
                              "which takes no PDM\n", i2s_name(m));
                m->logged_peer_pdm = true;
            }
            continue;
        }
        npeers++;
    }
    for (unsigned j = 0; j < nrx; j++) {
        BusUnit *r = &rx[j];

        for (unsigned l = 0; l < 4; l++) {
            ESP32P4I2sState *src;
            unsigned sl;
            bool inv;
            int pad;

            r->src[l] = -1;
            r->peer[l] = -1;
            r->level[l] = in_sig[r->c->id][rx_sd_lines[l]] >= 0 &&
                          i2s_in(r->c, rx_sd_lines[l]);
            pad = i2s_in_pad(r->c, rx_sd_lines[l], &inv);
            for (unsigned k = 0; k < npeers && pad >= 0; k++) {
                if (peers[k].p->dout_pad == pad) {
                    r->peer[l] = k;
                    r->peer_inv[l] = inv;
                }
            }
            if (r->peer[l] >= 0 ||
                !i2s_route(r->c, rx_sd_lines[l], &src, &sl,
                           &r->src_inv[l])) {
                continue;
            }
            for (unsigned i = 0; i < ntx; i++) {
                if (tx[i].c == src && (sl == ESP32P4_I2S_OUT_O_SD ||
                                       sl == ESP32P4_I2S_OUT_O_SD1)) {
                    r->src[l] = i;
                    r->src_line[l] = sl == ESP32P4_I2S_OUT_O_SD1;
                }
            }
        }
    }
    for (unsigned k = 0; k < npeers; k++) {
        const ESP32P4I2sPeer *p = peers[k].p;
        int64_t end = esp32_line_time();
        int64_t start = mu->events ? unit_event_time(m, mu, mu->events - 1)
                                   : end;

        p->ops->begin(p->opaque, ncycles, start, end,
                      i2s_pad_out_inv(m, p->sclk_pad));
    }

    for (uint32_t i = 0; i < ncycles && !stopped; i++) {
        if (!pdm) {
            bool ws = i < mc.ws_w ? mc.idle_pol : !mc.idle_pol;
            bool stop = false;

            for (unsigned t = 0; t < ntx; t++) {
                bool master = tx[t].c == m && m_tx;
                int bit = tdm_tx_cycle(tx[t].c, &tx[t].cfg, ws ^ tx[t].inv,
                                       master ? i == 0 : -1);

                if (bit < 0) {
                    stopped = true;
                    break;
                }
                stop |= bit & TX_LAST_CYCLE;
                tx[t].bits[0][0] = bit & 1;
            }
            if (stopped) {
                break;
            }
            for (unsigned k = 0; k < npeers; k++) {
                BusPeer *bp = &peers[k];

                bp->lrck = bp->lrck_ws ? ws ^ bp->lrck_inv : bp->lrck_level;
                bp->dout = bp->p->ops->fall(bp->p->opaque, bp->lrck,
                                            &bp->driven);
            }
            for (unsigned j = 0; j < nrx; j++) {
                bool sd = rx[j].level[0];
                bool master = rx[j].c == m && !m_tx;

                if (rx[j].peer[0] >= 0) {
                    BusPeer *bp = &peers[rx[j].peer[0]];

                    if (bp->driven) {
                        sd = bp->dout ^ rx[j].peer_inv[0];
                    }
                } else if (rx[j].src[0] >= 0) {
                    sd = tx[rx[j].src[0]].bits[rx[j].src_line[0]][0] ^
                         rx[j].src_inv[0];
                }
                tdm_rx_cycle(rx[j].c, &rx[j].cfg, ws ^ rx[j].inv, sd,
                             master ? i == 0 : -1);
            }
            for (unsigned k = 0; k < npeers; k++) {
                BusPeer *bp = &peers[k];
                bool din = bp->din_level;

                if (bp->din_src >= 0) {
                    din = tx[bp->din_src].bits[bp->din_line][0] ^
                          bp->din_inv;
                }
                bp->p->ops->rise(bp->p->opaque, bp->lrck, din);
            }
            stopped = stop;
        } else {
            for (unsigned t = 0; t < ntx; t++) {
                bool master = tx[t].c == m && m_tx;

                if (!pdm_tx_cycle(tx[t].c, &tx[t].cfg, master,
                                  tx[t].bits)) {
                    stopped = true;
                    break;
                }
            }
            if (stopped) {
                break;
            }
            for (unsigned j = 0; j < nrx; j++) {
                uint8_t in[4][2];

                for (unsigned l = 0; l < 4; l++) {
                    for (unsigned h = 0; h < 2; h++) {
                        bool b = rx[j].level[l];

                        if (rx[j].src[l] >= 0) {
                            b = tx[rx[j].src[l]].bits[rx[j].src_line[l]][h] ^
                                rx[j].src_inv[l];
                        }
                        in[l][h] = b;
                    }
                }
                pdm_rx_cycle(rx[j].c, &rx[j].cfg, in);
            }
        }
    }
    for (unsigned k = 0; k < npeers; k++) {
        peers[k].p->ops->end(peers[k].p->opaque);
    }
    if (stopped) {
        bus_stopped(tx, ntx, rx, nrx);
        return;
    }
    for (unsigned t = 0; t < ntx; t++) {
        ESP32P4I2sState *c = tx[t].c;
        bool last = tx[t].bits[0][pdm];

        /* The data line rests at the period's last bit. */
        if (c->tx.sd_level != last) {
            c->tx.sd_level = last;
            i2s_drive(c, ESP32P4_I2S_OUT_O_SD, last);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A device on the board on the pads, for every controller that may clock
 * it to know.
 */
void esp32p4_i2s_add_peer(ESP32P4I2sState *s, const ESP32P4I2sPeer *peer)
{
    assert(s->npeers < ESP32P4_I2S_PEERS);
    s->peers[s->npeers++] = *peer;
}

/* ---- Unit timing ---- */

/*
 * The time of a master's event n: the first nanosecond its unit clock has
 * made n frames' cycles from the anchor, at the clock's exact edges.
 */
static int64_t unit_event_time(ESP32P4I2sState *s, ESP32P4I2sUnit *u,
                               uint64_t n)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(unit_clk(s, u), u->anchor);

    return esp32p4_clk_edges_after(&e, u->anchor, n * u->frame_ticks);
}

/* Whether the unit is a master running on its own clock. */
static bool unit_own_clock(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    I2sCfg c;

    if (u->mode == MODE_OFF || u->stalled) {
        return false;
    }
    unit_cfg(s, u, &c);
    if (c.slave) {
        return false;
    }
    if (u == &s->rx && i2s_loopback(s) && s->tx.mode != MODE_OFF) {
        I2sCfg t;

        unit_cfg(s, &s->tx, &t);
        return t.slave;
    }
    return true;
}

static void unit_arm(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    timer_del(&u->timer);
    if (unit_own_clock(s, u) && u->frame_ticks &&
        clock_is_enabled(unit_clk(s, u))) {
        timer_mod_ns(&u->timer, unit_event_time(s, u, u->events + 1));
    }
}

static void i2s_etm_apply(ESP32P4I2sState *s, unsigned n);

/* A master's period run, or an ETM task applied, at a time of its own. */
typedef struct I2sAt {
    ESP32P4I2sState *s;
    unsigned n;
} I2sAt;

static void i2s_bus_run_at(void *opaque)
{
    I2sAt *a = opaque;

    i2s_bus_run(a->s, a->n);
}

static void i2s_etm_apply_at(void *opaque)
{
    I2sAt *a = opaque;

    i2s_etm_apply(a->s, a->n);
}

/*
 * Apply the ETM tasks that came while a period ran, in the order of their
 * events' times, each at its own.
 */
static void i2s_etm_deferred_apply(ESP32P4I2sState *c)
{
    while (c->etm_deferred) {
        I2sAt a = { .s = c, .n = ctz32(c->etm_deferred) };
        uint32_t rest = c->etm_deferred & ~BIT(a.n);

        while (rest) {
            unsigned m = ctz32(rest);

            if (c->etm_deferred_ns[m] < c->etm_deferred_ns[a.n]) {
                a.n = m;
            }
            rest &= ~BIT(m);
        }
        c->etm_deferred &= ~BIT(a.n);
        esp32_line_run_at(c->etm_deferred_ns[a.n], i2s_etm_apply_at, &a);
    }
}

/* Whether the transmit FIFO holds a whole frame's data. */
static bool tx_frame_ready(ESP32P4I2sState *s)
{
    I2sCfg c;

    unit_cfg(s, &s->tx, &c);
    return tx_has(s, tx_need(&c) * c.bytes);
}

static void unit_timer_cb(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned runs = 0;
    bool tx = u == &s->tx;

    for (unsigned k = 0; k < ESP32P4_I2S_COUNT; k++) {
        if (s->bus[k]) {
            s->bus[k]->in_run = true;
        }
    }
    s->in_run = true;
    while (unit_own_clock(s, u) && u->frame_ticks &&
           unit_event_time(s, u, u->events + 1) <= now) {
        if (tx && unit_event_time(s, u, u->events + 2) <= now &&
            !tx_frame_ready(s)) {
            /*
             * Behind time, with the FIFO short of a frame: GDMA, whose
             * transfers wait for this to return, cannot have kept up. The
             * clock slips rather than running out of data it would have.
             */
            u->anchor = now;
            u->events = 0;
            break;
        }
        u->events++;
        esp32_line_run_at(unit_event_time(s, u, u->events), i2s_bus_run_at,
                          &(I2sAt) { .s = s, .n = tx });
        if (++runs == MAX_CATCH_UP) {
            /* Too far behind to catch up: drop the backlog */
            u->anchor = now;
            u->events = 0;
            break;
        }
    }
    unit_arm(s, u);
    s->in_run = false;
    for (unsigned k = 0; k < ESP32P4_I2S_COUNT; k++) {
        ESP32P4I2sState *c = s->bus[k];

        if (!c) {
            continue;
        }
        c->in_run = false;
        i2s_etm_deferred_apply(c);
    }
    i2s_update_irq(s);
}

static void i2s_tx_timer_cb(void *opaque)
{
    ESP32P4I2sState *s = opaque;

    unit_timer_cb(s, &s->tx);
}

static void i2s_rx_timer_cb(void *opaque)
{
    ESP32P4I2sState *s = opaque;

    unit_timer_cb(s, &s->rx);
}

/* A master transmitter that stopped for want of data goes on. */
static void i2s_tx_resume(ESP32P4I2sState *s)
{
    I2sCfg c;

    if (!s->tx.stalled || s->tx.mode == MODE_OFF) {
        return;
    }
    unit_cfg(s, &s->tx, &c);
    if (!tx_has(s, tx_need(&c) * c.bytes)) {
        return;
    }
    s->tx.stalled = false;
    s->tx.anchor = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->tx.events = 0;
    unit_arm(s, &s->tx);
    i2s_hung_update(s, &s->tx, false);
}

/* ---- Unit control ---- */

static void unit_serial_reset(ESP32P4I2sUnit *u)
{
    u->logged_mismatch = false;
    u->started = false;
    u->prev_ws = 0;
    memset(&u->cur, 0, sizeof(u->cur));
    memset(&u->prev, 0, sizeof(u->prev));
    u->pdm_bit = 0;
    memset(u->pdm, 0, sizeof(u->pdm));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * What a unit does: nothing until START, with its bus clock and its own
 * clock running, in TDM or PDM mode, exactly one of TDM_EN and PDM_EN set
 * (TRM 49.5); the PDM converters are I2S0's.
 */
static unsigned unit_mode(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    bool tx = u == &s->tx;
    uint32_t live = tx ? s->tx_conf : s->rx_conf;
    I2sCfg c;

    if (!(live & CONF_START) || !i2s_bus_on(s) ||
        !clock_is_enabled(unit_clk(s, u)) || (tx && i2s_is_lp(s))) {
        return MODE_OFF;
    }
    unit_cfg(s, u, &c);
    if (c.tdm == c.pdm) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s %s starts with "
                      "%s of TDM_EN and PDM_EN set\n", i2s_name(s),
                      tx ? "TX" : "RX", c.tdm ? "both" : "neither");
        return MODE_OFF;
    }
    if (c.pdm && c.conv && s->id != 0 && !i2s_is_lp(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s has no %s "
                      "converter\n", i2s_name(s),
                      tx ? "PCM-to-PDM" : "PDM-to-PCM");
        return MODE_OFF;
    }
    return c.tdm ? MODE_TDM : MODE_PDM;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Start, stop or retime the units after a change of their configuration,
 * their START or their clocks. A unit starting begins afresh; with
 * reanchor, a running master restarts its clock's timing from now.
 */
static void i2s_update_unit(ESP32P4I2sState *s, ESP32P4I2sUnit *u,
                            bool reanchor, int64_t now)
{
    bool tx = u == &s->tx;
    unsigned mode = unit_mode(s, u);
    bool restart = mode != u->mode;
    I2sCfg c;

    u->mode = mode;
    unit_cfg(s, u, &c);
    u->frame_ticks = mode == MODE_OFF ? 0 :
                     (uint64_t)unit_period_cycles(u, &c, tx) * c.bck_div;
    if (restart) {
        unit_serial_reset(u);
        u->stalled = false;
        u->drained = false;
        u->words = 0;
        if (!tx) {
            u->eof_count = 0;
        }
        if (mode != MODE_OFF) {
            if (c.bck_div <= 2 && !c.slave) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s %s "
                              "BCK_DIV_NUM %u is below 2\n", i2s_name(s),
                              tx ? "TX" : "RX", c.bck_div - 1);
            }
            if (c.pdm && c.conv && !s->logged_conv) {
                qemu_log_mask(LOG_UNIMP, "esp32p4_i2s: the PDM converters' "
                              "filters, prescale and dither are not "
                              "documented; a first-order modulator and a "
                              "third-order CIC stand for them\n");
                s->logged_conv = true;
            }
            if (c.tdm && c.n * c.cb > 2 * c.hs && !c.slave) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s %s channels "
                              "(%u of %u bits) overrun its frame of %u "
                              "BCK cycles\n", i2s_name(s), tx ? "TX" : "RX",
                              c.n, c.cb, 2 * c.hs);
            }
            if (i2s_is_lp(s) && c.bits != 16) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: LP I2S "
                              "RX_BITS_MOD must be 15\n");
            }
        } else if (tx && s->tx.sd_level) {
            /* A stopped slave sends zeros. */
            s->tx.sd_level = false;
            i2s_drive(s, ESP32P4_I2S_OUT_O_SD, false);
        }
    }
    if (restart || reanchor) {
        u->anchor = now;
        u->events = 0;
        unit_arm(s, u);
    }
    i2s_hung_update(s, u, restart);
}

static void i2s_update_units(ESP32P4I2sState *s, bool reanchor)
{
    int64_t now = esp32_line_time();

    if (!i2s_is_lp(s)) {
        i2s_update_unit(s, &s->tx, reanchor, now);
    }
    i2s_update_unit(s, &s->rx, reanchor, now);
    /* The receiver runs on its own clock unless its transmitter's. */
    unit_arm(s, &s->rx);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * TX_UPDATE and RX_UPDATE carry the unit's configuration into its clock
 * domain, which takes a running clock; the bit clears when done.
 */
static void unit_sync(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    bool tx = u == &s->tx;

    if (!u->update_pending || !clock_is_enabled(unit_clk(s, u))) {
        return;
    }
    u->update_pending = false;
    if (tx) {
        u->conf = s->tx_conf;
        u->conf1 = s->tx_conf1;
        u->tdm_ctrl = s->tx_tdm_ctrl;
        u->pdm_conf = s->tx_pcm2pdm_conf;
        u->pdm_conf1 = s->tx_pcm2pdm_conf1;
        s->tx_conf &= ~CONF_UPDATE;
    } else {
        u->conf = s->rx_conf;
        u->conf1 = s->rx_conf1;
        u->tdm_ctrl = s->rx_tdm_ctrl;
        u->pdm_conf = s->rx_pdm2pcm_conf;
        s->rx_conf &= ~CONF_UPDATE;
    }
    u->single_data = s->single_data;
}

/* The reset bits: the unit's serial state, or its FIFO. */
static void unit_reset(ESP32P4I2sState *s, ESP32P4I2sUnit *u)
{
    unit_serial_reset(u);
    u->stalled = false;
    u->drained = false;
    u->eof_count = 0;
    memset(u->last, 0, sizeof(u->last));
    u->last_chan = 0;
}

/* ---- Pads ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * An input signal changes. A TDM slave that no master of the chip clocks
 * follows the edges of its BCK input. A receiver samples WS and its data
 * input on each rising edge. A transmitter drives a bit on each falling
 * edge, from the WS it sampled on the rising edge before: the bit one
 * cycle on from that WS, which is the bit the frame puts there with
 * MSB_SHIFT; without it (MSB alignment) the first bit of a frame comes a
 * cycle after its WS edge. With SIG_LOOPBACK both units follow the
 * transmitter's BCK and WS inputs.
 */
static void i2s_sig_in(void *opaque, int n, int level)
{
    ESP32P4I2sState *s = opaque;
    bool old = i2s_in(s, n);
    unsigned bck, ws;
    I2sCfg c;

    s->in_level = deposit32(s->in_level, n, 1, !!level);
    if (old == !!level || n == ESP32P4_I2S_IN_MCLK || s->in_run) {
        return;
    }
    if (!i2s_is_lp(s) && s->tx.mode == MODE_TDM) {
        unit_cfg(s, &s->tx, &c);
        unit_clock_lines(s, true, &bck, &ws, true);
        if (c.slave && n == bck && level) {
            int bit;

            c.msb_shift = false;
            bit = tdm_tx_cycle(s, &c, i2s_in(s, ws), -1);
            s->tx.sd_next = bit > 0 && (bit & 1);
        } else if (c.slave && n == bck && s->tx.sd_next != s->tx.sd_level) {
            s->tx.sd_level = s->tx.sd_next;
            i2s_drive(s, ESP32P4_I2S_OUT_O_SD, s->tx.sd_level);
        }
    }
    if (s->rx.mode == MODE_TDM) {
        unit_cfg(s, &s->rx, &c);
        unit_clock_lines(s, false, &bck, &ws, true);
        if (c.slave && n == bck && level) {
            tdm_rx_cycle(s, &c, i2s_in(s, ws),
                         i2s_in(s, ESP32P4_I2S_IN_I_SD), -1);
        }
    }
    i2s_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The clock on I2Sn_MCLK_in: another controller's I2Sn_MCLK_out that the
 * pads connect to it, for HP_SYS_CLKRST's PAD_I2Sn_MCLK, at that clock's
 * exact period and with its edges, which the controller gives the clock
 * tree as the TYPE_ESP32P4_CLK_SOURCE of "pad-mclk". A controller's own
 * MCLK_out fed back is no clock: the loop has no source.
 */
static void i2s_update_pad_mclk(ESP32P4I2sState *s)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_stopped();
    ESP32P4I2sState *src;
    unsigned line;
    bool inv;

    if (i2s_is_lp(s) || s->mclk_busy) {
        return;
    }
    if (i2s_route(s, ESP32P4_I2S_IN_MCLK, &src, &line, &inv) &&
        line == ESP32P4_I2S_OUT_MCLK && src != s) {
        e = esp32p4_clk_edges_or_at(src->mst_clk,
                                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (!esp32p4_clk_edges_eq(&s->pad_mclk_edges, &e)) {
        ESP32P4ClkEdges old = s->pad_mclk_edges;

        s->pad_mclk_edges = e;
        s->mclk_busy = true;
        esp32p4_clk_update(s->pad_mclk, &old, &e);
        s->mclk_busy = false;
    }
}

static bool esp32p4_i2s_exact_edges(Object *obj, Clock *out,
                                    ESP32P4ClkEdges *e)
{
    ESP32P4I2sState *s = ESP32P4_I2S(obj);

    if (out != s->pad_mclk) {
        return false;
    }
    *e = s->pad_mclk_edges;
    return true;
}

static void i2s_route_changed(Notifier *n, void *data)
{
    ESP32P4I2sState *s = container_of(n, ESP32P4I2sState, route_notifier);

    i2s_update_pad_mclk(s);
}

/* ---- The VAD ---- */

static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0;

    for (int b = 31; b >= 0; b--) {
        uint64_t t = r | (1ull << b);

        if (t * t <= v) {
            r = t;
        }
    }
    return r;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The VAD takes a frame, the last 256 samples stored (TRM 44.4): the
 * energy check, noise learning over the initial frames, the speech
 * decision, the speech and silence counters and the state machine;
 * VAD_DONE at the end.
 */
static void vad_frame(ESP32P4I2sState *s)
{
    ESP32P4I2sVad *v = &s->vad;
    uint32_t p0 = s->vad_param[0], p1 = s->vad_param[1];
    uint32_t p2 = s->vad_param[2];
    int16_t x[ESP32P4_LP_I2S_VAD_FRAME];
    uint32_t words = ESP32P4_LP_I2S_VAD_FRAME / 2;
    uint32_t w = (s->mem_wr + ESP32P4_LP_I2S_MEM_WORDS - words) %
                 ESP32P4_LP_I2S_MEM_WORDS;
    uint64_t energy = 0;
    int64_t sum = 0;
    uint32_t peak = 0, amp, up, down;
    bool speech = false;

    for (uint32_t i = 0; i < words; i++) {
        uint32_t word = s->mem[(w + i) % ESP32P4_LP_I2S_MEM_WORDS];

        x[2 * i] = word;
        x[2 * i + 1] = word >> 16;
    }
    for (uint32_t i = 0; i < ESP32P4_LP_I2S_VAD_FRAME; i++) {
        energy += (int64_t)x[i] * x[i];
        sum += x[i];
        peak = MAX(peak, (uint32_t)abs(x[i]));
    }
    v->energy = energy;
    v->offset = sum / ESP32P4_LP_I2S_VAD_FRAME;
    v->max_signal[2] = v->max_signal[1];
    v->max_signal[1] = v->max_signal[0];
    v->max_signal[0] = MIN(peak, 0xffff);
    v->energy_enough = energy >= (uint64_t)VP0_MIN_ENERGY(p0) *
                                 ESP32P4_LP_I2S_VAD_FRAME;
    amp = isqrt64(energy / ESP32P4_LP_I2S_VAD_FRAME);

    if (v->init_frames < VP0_INIT_FRAME_NUM(p0)) {
        /* The noise level: the mean of the initial frames' RMS. */
        v->noise_amp = (uint32_t)(((uint64_t)v->noise_amp * v->init_frames +
                                   amp) / (v->init_frames + 1));
        v->init_frames++;
    } else {
        speech = v->energy_enough && amp > v->threshold;
        if (!v->flag) {
            if (speech) {
                v->speech_count = MIN(v->speech_count + 1, 0xff);
            } else if (v->speech_count) {
                v->speech_count--;
            }
            if (v->speech_count > VP1_HANGOVER_SPEECH(p1)) {
                v->flag = true;
                v->silent_count = 0;
            }
        } else if (speech) {
            v->speech_count = MIN(v->speech_count + 1, 0xff);
            if (v->silent_count) {
                v->silent_count--;
            }
        } else {
            v->silent_count = MIN(v->silent_count + 1, 0xff);
            if (v->speech_count > VP1_MIN_SPEECH(p1)) {
                if (v->silent_count > VP1_HANGOVER_SILENT(p1)) {
                    v->flag = false;
                    v->speech_count = 0;
                    v->silent_count = 0;
                }
            } else {
                v->flag = false;
                v->speech_count = 0;
            }
        }
        if (!v->flag || v->speech_count > VP1_MAX_SPEECH(p1)) {
            up = VP2_NOISE_AMP_UP(p2);
            down = VP2_NOISE_AMP_DOWN(p2);
            if (amp > v->noise_amp) {
                v->noise_amp = ((uint64_t)v->noise_amp * up +
                                (uint64_t)amp * (32768 - MIN(up, 32768))) >>
                               15;
            } else {
                v->noise_amp = ((uint64_t)v->noise_amp * down +
                                (uint64_t)amp * (32768 - MIN(down, 32768))) >>
                               15;
            }
        }
    }
    v->threshold = 2 * v->noise_amp;
    qemu_set_irq(s->wakeup, v->flag);
    i2s_raise(s, LP_INT_VAD_DONE);
}

static void vad_reset(ESP32P4I2sState *s)
{
    memset(&s->vad, 0, sizeof(s->vad));
    qemu_set_irq(s->wakeup, 0);
}

/* ---- Registers ---- */

static uint32_t i2s_state_reg(ESP32P4I2sState *s)
{
    return s->tx.mode == MODE_OFF || s->tx.stalled ? STATE_TX_IDLE : 0;
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static uint64_t i2s_hp_read(ESP32P4I2sState *s, hwaddr addr)
{
    switch (addr) {
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_RX_CONF:
        return s->rx_conf;
    case A_TX_CONF:
        return s->tx_conf;
    case A_RX_CONF1:
        return s->rx_conf1;
    case A_TX_CONF1:
        return s->tx_conf1;
    case A_TX_PCM2PDM_CONF:
        return s->tx_pcm2pdm_conf;
    case A_TX_PCM2PDM_CONF1:
        return s->tx_pcm2pdm_conf1;
    case A_RX_PDM2PCM_CONF:
        return s->rx_pdm2pcm_conf;
    case A_RX_TDM_CTRL:
        return s->rx_tdm_ctrl;
    case A_TX_TDM_CTRL:
        return s->tx_tdm_ctrl;
    case A_RX_TIMING:
        return s->rx_timing;
    case A_TX_TIMING:
        return s->tx_timing;
    case A_LC_HUNG_CONF:
        return s->lc_hung_conf;
    case A_RXEOF_NUM:
        return s->rx_eof_num;
    case A_CONF_SIGLE_DATA:
        return s->single_data;
    case A_STATE:
        return i2s_state_reg(s);
    case A_ETM_CONF:
        return s->etm_conf;
    case A_FIFO_CNT:
        return s->tx_fifo_cnt;
    case A_BCK_CNT:
        return s->tx_bck_cnt;
    case A_CLK_GATE:
        return s->clk_gate;
    case A_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s read of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", i2s_name(s), addr);
        return 0;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * CONF's reset bits act as written (WT, TRM 49.7); START acts at once;
 * UPDATE carries the rest into the unit (TRM 49.9).
 */
static void i2s_write_conf(ESP32P4I2sState *s, ESP32P4I2sUnit *u,
                           uint32_t *reg, uint32_t mask, uint32_t val)
{
    bool tx = u == &s->tx;

    *reg = (*reg & CONF_UPDATE) | (val & mask);
    if (val & CONF_FIFO_RESET) {
        if (tx) {
            fifo_reset(&s->tx_fifo);
        } else if (!i2s_is_lp(s)) {
            fifo_reset(&s->rx_fifo);
        } else {
            s->mem_half_valid = false;
        }
    }
    if (val & CONF_RESET) {
        unit_reset(s, u);
    }
    if (i2s_is_lp(s) && (val & LP_RX_CONF_FIFOMEM_RESET)) {
        s->mem_rd = s->mem_wr = s->mem_num = 0;
        s->mem_half_valid = false;
    }
    if (val & CONF_UPDATE) {
        *reg |= CONF_UPDATE;
        u->update_pending = true;
        unit_sync(s, u);
    }
    /* A reset unit's clock starts its frames afresh. */
    i2s_update_units(s, val & CONF_RESET);
}

static void i2s_etm_apply(ESP32P4I2sState *s, unsigned n)
{
    uint32_t *reg;

    switch (n) {
    case ESP32P4_I2S_TASK_START_RX:
    case ESP32P4_I2S_TASK_STOP_RX:
        reg = &s->rx_conf;
        break;
    default:
        reg = &s->tx_conf;
        break;
    }
    /*
     * [spec:nuos:req:emu.esp32p4.i2s]
     * I2Sn_TASK_START_* sets the unit's START and I2Sn_TASK_STOP_* clears
     * it (TRM 49.11): the bits the TRM calls R/W/SC.
     */
    if (n == ESP32P4_I2S_TASK_START_RX || n == ESP32P4_I2S_TASK_START_TX) {
        *reg |= CONF_START;
    } else {
        *reg &= ~CONF_START;
    }
    i2s_update_units(s, false);
    i2s_update_irq(s);
}

static void i2s_etm_task(void *opaque, int n, int level)
{
    ESP32P4I2sState *s = opaque;

    if (!level) {
        return;
    }
    if (s->in_run) {
        if (!(s->etm_deferred & BIT(n))) {
            s->etm_deferred |= BIT(n);
            s->etm_deferred_ns[n] = esp32_line_time();
        }
        return;
    }
    i2s_etm_apply(s, n);
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static void i2s_hp_write(ESP32P4I2sState *s, hwaddr addr, uint32_t val)
{
    switch (addr) {
    case A_INT_RAW:
        /* WTC: writing 1 clears */
        s->int_raw &= ~(val & INT_MASK);
        break;
    case A_INT_ENA:
        s->int_ena = val & INT_MASK;
        break;
    case A_INT_CLR:
        s->int_raw &= ~(val & INT_MASK);
        break;
    case A_RX_CONF:
        i2s_write_conf(s, &s->rx, &s->rx_conf, RX_CONF_MASK & ~CONF_UPDATE,
                       val);
        break;
    case A_TX_CONF:
        i2s_write_conf(s, &s->tx, &s->tx_conf, TX_CONF_MASK & ~CONF_UPDATE,
                       val);
        break;
    case A_RX_CONF1:
        s->rx_conf1 = val & CONF1_MASK;
        break;
    case A_TX_CONF1:
        s->tx_conf1 = val & CONF1_MASK;
        break;
    case A_TX_PCM2PDM_CONF:
        s->tx_pcm2pdm_conf = val & P2D_MASK;
        if (P2D_PRESCALE(val) && (val & P2D_CONV_EN)) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_i2s: TX_PDM_PRESCALE is not "
                          "modelled\n");
        }
        break;
    case A_TX_PCM2PDM_CONF1:
        s->tx_pcm2pdm_conf1 = val & P2D1_MASK;
        break;
    case A_RX_PDM2PCM_CONF:
        s->rx_pdm2pcm_conf = val & D2P_MASK;
        break;
    case A_RX_TDM_CTRL:
        s->rx_tdm_ctrl = val & RX_TDM_MASK;
        break;
    case A_TX_TDM_CTRL:
        s->tx_tdm_ctrl = val & TX_TDM_MASK;
        break;
    case A_RX_TIMING:
        s->rx_timing = val & RX_TIMING_MASK;
        break;
    case A_TX_TIMING:
        s->tx_timing = val & TX_TIMING_MASK;
        break;
    case A_LC_HUNG_CONF:
        s->lc_hung_conf = val & HUNG_MASK;
        timer_del(&s->tx.hung_timer);
        timer_del(&s->rx.hung_timer);
        i2s_hung_update(s, &s->tx, false);
        i2s_hung_update(s, &s->rx, false);
        break;
    case A_RXEOF_NUM:
        s->rx_eof_num = val & RXEOF_NUM_MASK;
        break;
    case A_CONF_SIGLE_DATA:
        s->single_data = val;
        break;
    case A_ETM_CONF:
        s->etm_conf = val & ETM_CONF_MASK;
        break;
    case A_FIFO_CNT:
        if (val & CNT_RST) {
            s->tx_fifo_cnt = 0;
        }
        break;
    case A_BCK_CNT:
        if (val & CNT_RST) {
            s->tx_bck_cnt = 0;
        }
        break;
    case A_CLK_GATE:
        s->clk_gate = val & CLK_GATE_MASK;
        break;
    case A_DATE:
        s->date = val & DATE_MASK;
        break;
    case A_INT_ST:
    case A_STATE:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s write to read-only "
                      "offset 0x%" HWADDR_PRIx "\n", i2s_name(s), addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: %s write to reserved "
                      "offset 0x%" HWADDR_PRIx "\n", i2s_name(s), addr);
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The CPU reads the LP I2S memory's oldest word (TRM 50.8.3). An empty
 * memory reads the word at its read pointer again and raises
 * RX_FIFOMEM_UDF.
 */
static uint32_t lp_mem_read(ESP32P4I2sState *s)
{
    uint32_t v = s->mem[s->mem_rd];

    if (s->mem_num == 0) {
        i2s_raise(s, LP_INT_FIFOMEM_UDF);
        return v;
    }
    s->mem_rd = (s->mem_rd + 1) % ESP32P4_LP_I2S_MEM_WORDS;
    s->mem_num--;
    return v;
}

static uint32_t lp_vad_ob(ESP32P4I2sState *s, unsigned n)
{
    ESP32P4I2sVad *v = &s->vad;

    switch (n) {
    case 0:
        return MIN(v->speech_count, 0xff) | MIN(v->silent_count, 0xff) << 8 |
               (uint32_t)v->max_signal[0] << 16;
    case 1:
        return v->max_signal[1] | (uint32_t)v->max_signal[2] << 16;
    case 2:
        return v->noise_amp;
    case 5:
        return v->offset;
    case 6:
        return v->threshold;
    case 7:
        return v->energy;
    case 8:
        return v->energy >> 32;
    default:
        /* NOISE_MEAN_OB and NOISE_STD_OB: the LTSD's, not modelled */
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static uint64_t i2s_lp_read(ESP32P4I2sState *s, hwaddr addr)
{
    if (addr >= A_LP_RAM && addr < A_LP_RAM_END) {
        return lp_mem_read(s);
    }
    switch (addr) {
    case A_LP_VAD_CONF:
        return s->vad_conf;
    case A_LP_VAD_RESULT:
        return s->vad.flag | s->vad.energy_enough << 1;
    case A_LP_RX_MEM_CONF:
        return (s->rx_mem_conf & MEM_CONF_MASK) | MIN(s->mem_num, 0x1ff);
    case A_LP_INT_RAW:
        return s->int_raw;
    case A_LP_INT_ST:
        return s->int_raw & s->int_ena;
    case A_LP_INT_ENA:
        return s->int_ena;
    case A_LP_INT_CLR:
        return 0;
    case A_LP_RX_CONF:
        return s->rx_conf;
    case A_LP_RX_CONF1:
        return s->rx_conf1;
    case A_LP_RX_TDM_CTRL:
        return s->rx_tdm_ctrl;
    case A_LP_RX_TIMING:
        return s->rx_timing;
    case A_LP_LC_HUNG_CONF:
        return s->lc_hung_conf;
    case A_LP_RXEOF_NUM:
        return s->rx_eof_num;
    case A_LP_CONF_SIGLE_DATA:
        return s->single_data;
    case A_LP_RX_PDM_CONF:
        return s->rx_pdm2pcm_conf;
    case A_LP_ECO_LOW:
        return s->eco_low;
    case A_LP_ECO_HIGH:
        return s->eco_high;
    case A_LP_ECO_CONF:
        return s->eco_conf;
    case A_LP_VAD_PARAM0 ... A_LP_VAD_PARAM8:
        return s->vad_param[(addr - A_LP_VAD_PARAM0) / 4];
    case A_LP_VAD_OB0 ... A_LP_VAD_OB8:
        return lp_vad_ob(s, (addr - A_LP_VAD_OB0) / 4);
    case A_LP_CLK_GATE:
        return s->clk_gate;
    case A_LP_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: LP I2S read of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static void i2s_lp_write(ESP32P4I2sState *s, hwaddr addr, uint32_t val)
{
    if (addr >= A_LP_RAM && addr < A_LP_RAM_END) {
        /* The VAD's manual mode: software stores the voice data. */
        lp_mem_store(s, val);
        return;
    }
    switch (addr) {
    case A_LP_VAD_CONF:
        if (!(s->vad_conf & VAD_EN) && (val & VAD_EN)) {
            qemu_log_mask(LOG_UNIMP, "esp32p4_i2s: the VAD's FFT, LTSD and "
                          "band energy stages are not modelled: a frame "
                          "that passes the energy check is speech when its "
                          "RMS exceeds twice the noise level, and "
                          "NOISE_MEAN_OB and NOISE_STD_OB read 0\n");
            s->vad.samples = 0;
        }
        s->vad_conf = val & VAD_EN;
        if (val & VAD_RESET) {
            vad_reset(s);
            i2s_raise(s, LP_INT_VAD_RESET_DONE);
        }
        if (val & VAD_FORCE_START) {
            vad_frame(s);
        }
        break;
    case A_LP_RX_MEM_CONF:
        s->rx_mem_conf = val & MEM_CONF_MASK;
        break;
    case A_LP_INT_RAW:
        s->int_raw &= ~(val & LP_INT_MASK);
        break;
    case A_LP_INT_ENA:
        s->int_ena = val & LP_INT_MASK;
        break;
    case A_LP_INT_CLR:
        s->int_raw &= ~(val & LP_INT_MASK);
        break;
    case A_LP_RX_CONF:
        i2s_write_conf(s, &s->rx, &s->rx_conf,
                       LP_RX_CONF_MASK & ~CONF_UPDATE, val);
        break;
    case A_LP_RX_CONF1:
        s->rx_conf1 = val & LP_CONF1_MASK;
        break;
    case A_LP_RX_TDM_CTRL:
        s->rx_tdm_ctrl = val & LP_TDM_MASK;
        break;
    case A_LP_RX_TIMING:
        s->rx_timing = val & LP_TIMING_MASK;
        break;
    case A_LP_LC_HUNG_CONF:
        s->lc_hung_conf = val & HUNG_MASK;
        timer_del(&s->rx.hung_timer);
        i2s_hung_update(s, &s->rx, false);
        break;
    case A_LP_RXEOF_NUM:
        s->rx_eof_num = val & RXEOF_NUM_MASK;
        break;
    case A_LP_CONF_SIGLE_DATA:
        s->single_data = val;
        break;
    case A_LP_RX_PDM_CONF:
        s->rx_pdm2pcm_conf = val & D2P_MASK;
        break;
    case A_LP_ECO_LOW:
        s->eco_low = val;
        break;
    case A_LP_ECO_HIGH:
        s->eco_high = val;
        break;
    case A_LP_ECO_CONF:
        s->eco_conf = val & LP_ECO_CONF_MASK;
        break;
    case A_LP_VAD_PARAM0:
        s->vad_param[0] = val & VP0_MASK;
        break;
    case A_LP_VAD_PARAM0 + 4 ... A_LP_VAD_PARAM8:
        s->vad_param[(addr - A_LP_VAD_PARAM0) / 4] = val;
        break;
    case A_LP_CLK_GATE:
        s->clk_gate = val & LP_CLK_GATE_MASK;
        break;
    case A_LP_DATE:
        s->date = val & DATE_MASK;
        break;
    case A_LP_VAD_RESULT:
    case A_LP_INT_ST:
    case A_LP_VAD_OB0 ... A_LP_VAD_OB8:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: LP I2S write to "
                      "read-only offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4_i2s: LP I2S write to "
                      "reserved offset 0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static uint64_t esp32p4_i2s_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4I2sState *s = opaque;

    return i2s_is_lp(s) ? i2s_lp_read(s, addr) : i2s_hp_read(s, addr);
}

static void esp32p4_i2s_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4I2sState *s = opaque;

    if (i2s_is_lp(s)) {
        i2s_lp_write(s, addr, value);
    } else {
        i2s_hp_write(s, addr, value);
    }
    i2s_update_irq(s);
}

static const MemoryRegionOps esp32p4_i2s_ops = {
    .read = esp32p4_i2s_read,
    .write = esp32p4_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---- Device ---- */

/*
 * A clock changed: pending updates complete on a unit clock that runs,
 * units start or stop with their clocks, and a unit whose own clock
 * changed restarts its timing.
 */
static void i2s_clock_changed(ESP32P4I2sState *s, ESP32P4I2sUnit *changed)
{
    int64_t now = esp32_line_time();

    if (!i2s_is_lp(s)) {
        unit_sync(s, &s->tx);
        i2s_update_unit(s, &s->tx, changed == &s->tx, now);
    }
    unit_sync(s, &s->rx);
    i2s_update_unit(s, &s->rx, changed == &s->rx, now);
    unit_arm(s, &s->rx);
    i2s_tx_fill(s);
}

static void esp32p4_i2s_apb_clock(void *opaque, ClockEvent event)
{
    i2s_clock_changed(opaque, NULL);
}

static void esp32p4_i2s_tx_clock(void *opaque, ClockEvent event)
{
    ESP32P4I2sState *s = opaque;

    i2s_clock_changed(s, &s->tx);
}

static void esp32p4_i2s_rx_clock(void *opaque, ClockEvent event)
{
    ESP32P4I2sState *s = opaque;

    i2s_clock_changed(s, &s->rx);
}

static void esp32p4_i2s_mst_clock(void *opaque, ClockEvent event)
{
    ESP32P4I2sState *s = opaque;

    for (unsigned k = 0; k < ESP32P4_I2S_HP_COUNT; k++) {
        if (s->bus[k] && s->bus[k] != s) {
            i2s_update_pad_mclk(s->bus[k]);
        }
    }
}

static void esp32p4_i2s_reset_hold(Object *obj, ResetType type)
{
    ESP32P4I2sState *s = ESP32P4_I2S(obj);
    bool lp = i2s_is_lp(s);

    s->int_raw = 0;
    s->int_ena = 0;
    s->rx_conf = lp ? LP_RX_CONF_RESET_VAL : RX_CONF_RESET_VAL;
    s->tx_conf = lp ? 0 : TX_CONF_RESET_VAL;
    s->rx_conf1 = lp ? LP_CONF1_RESET_VAL : CONF1_RESET_VAL;
    s->tx_conf1 = lp ? 0 : CONF1_RESET_VAL;
    s->tx_pcm2pdm_conf = lp ? 0 : P2D_RESET_VAL;
    s->tx_pcm2pdm_conf1 = lp ? 0 : P2D1_RESET_VAL;
    s->rx_pdm2pcm_conf = D2P_RESET_VAL;
    s->rx_tdm_ctrl = lp ? LP_TDM_RESET_VAL : TDM_RESET_VAL;
    s->tx_tdm_ctrl = lp ? 0 : TDM_RESET_VAL;
    s->rx_timing = 0;
    s->tx_timing = 0;
    s->lc_hung_conf = HUNG_RESET_VAL;
    s->rx_eof_num = RXEOF_NUM_RESET_VAL;
    s->single_data = 0;
    s->etm_conf = lp ? 0 : ETM_CONF_RESET_VAL;
    s->tx_fifo_cnt = 0;
    s->tx_bck_cnt = 0;
    s->clk_gate = lp ? LP_CLK_GATE_RESET_VAL : 0;
    s->date = lp ? LP_DATE_RESET_VAL : DATE_RESET_VAL;
    s->vad_conf = 0;
    s->rx_mem_conf = MEM_CONF_RESET_VAL;
    s->eco_low = 0;
    s->eco_high = UINT32_MAX;
    s->eco_conf = 0;
    memcpy(s->vad_param, vad_param_reset, sizeof(s->vad_param));

    for (int t = 0; t < 2; t++) {
        ESP32P4I2sUnit *u = t ? &s->tx : &s->rx;

        timer_del(&u->timer);
        timer_del(&u->hung_timer);
        unit_serial_reset(u);
        u->conf = t ? s->tx_conf : s->rx_conf;
        u->conf1 = t ? s->tx_conf1 : s->rx_conf1;
        u->tdm_ctrl = t ? s->tx_tdm_ctrl : s->rx_tdm_ctrl;
        u->pdm_conf = t ? s->tx_pcm2pdm_conf : s->rx_pdm2pcm_conf;
        u->pdm_conf1 = t ? s->tx_pcm2pdm_conf1 : 0;
        u->single_data = 0;
        u->update_pending = false;
        u->mode = MODE_OFF;
        u->stalled = false;
        u->drained = false;
        memset(u->last, 0, sizeof(u->last));
        u->last_chan = 0;
        u->eof_count = 0;
        u->words = 0;
        u->anchor = 0;
        u->events = 0;
        u->frame_ticks = 0;
        u->hung_fired = false;
        u->sd_level = false;
        u->sd_next = false;
    }
    fifo_reset(&s->tx_fifo);
    fifo_reset(&s->rx_fifo);
    memset(s->mem, 0, sizeof(s->mem));
    s->mem_rd = s->mem_wr = s->mem_num = 0;
    s->mem_half_valid = false;
    s->mem_half = 0;
    memset(&s->vad, 0, sizeof(s->vad));
    s->logged_conv = false;
    s->logged_slave_clk = false;
    s->logged_peer_pdm = false;
}

static void esp32p4_i2s_reset_exit(Object *obj, ResetType type)
{
    ESP32P4I2sState *s = ESP32P4_I2S(obj);

    for (unsigned i = 0; i < ESP32P4_I2S_OUT_COUNT; i++) {
        qemu_set_irq(s->sig_out[i], 0);
    }
    qemu_set_irq(s->wakeup, 0);
    i2s_update_irq(s);
}

static const ESP32P4GdmaPeriphOps i2s_gdma_ops = {
    .tx_ready = i2s_gdma_tx_ready,
    .rx_ready = i2s_gdma_rx_ready,
};

static void esp32p4_i2s_realize(DeviceState *dev, Error **errp)
{
    ESP32P4I2sState *s = ESP32P4_I2S(dev);
    bool lp = i2s_is_lp(s);

    if (lp) {
        s->id = ESP32P4_I2S_LP_ID;
    } else if (s->id >= ESP32P4_I2S_HP_COUNT) {
        error_setg(errp, "esp32p4_i2s: id must be 0, 1 or 2");
        return;
    }
    if (!lp && s->gdma) {
        esp32p4_gdma_attach(s->gdma, i2s_gdma_periph[s->id], &i2s_gdma_ops,
                            s);
    }
    if (s->gpio) {
        s->route_notifier.notify = i2s_route_changed;
        esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
    }
}

static void esp32p4_i2s_init(Object *obj)
{
    ESP32P4I2sState *s = ESP32P4_I2S(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    bool lp = i2s_is_lp(s);

    memory_region_init_io(&s->iomem, obj, &esp32p4_i2s_ops, s,
                          lp ? TYPE_ESP32P4_LP_I2S : TYPE_ESP32P4_I2S,
                          lp ? ESP32P4_LP_I2S_SIZE : ESP32P4_I2S_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, s->sig_out, ESP32P4_I2S_SIG_OUT,
                             ESP32P4_I2S_OUT_COUNT);
    qdev_init_gpio_in_named(dev, i2s_sig_in, ESP32P4_I2S_SIG_IN,
                            ESP32P4_I2S_IN_COUNT);
    s->apb_clk = qdev_init_clock_in(dev, "apb", esp32p4_i2s_apb_clock, s,
                                    ClockUpdate);
    s->rx_clk = qdev_init_clock_in(dev, "rx", esp32p4_i2s_rx_clock, s,
                                   ClockUpdate);
    if (lp) {
        qdev_init_gpio_out_named(dev, &s->wakeup, ESP32P4_I2S_WAKEUP, 1);
        s->tx_clk = clock_new(obj, "tx");
        s->mst_clk = clock_new(obj, "mst");
        s->pad_mclk = clock_new(obj, "pad-mclk");
    } else {
        s->tx_clk = qdev_init_clock_in(dev, "tx", esp32p4_i2s_tx_clock, s,
                                       ClockUpdate);
        s->mst_clk = qdev_init_clock_in(dev, "mst", esp32p4_i2s_mst_clock, s,
                                        ClockUpdate);
        s->pad_mclk = qdev_init_clock_out(dev, "pad-mclk");
        s->pad_mclk_edges = esp32p4_clk_edges_stopped();
        qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_I2S_ETM_EVENT,
                                 ESP32P4_I2S_ETM_EVENTS);
        qdev_init_gpio_in_named(dev, i2s_etm_task, ESP32P4_I2S_ETM_TASK,
                                ESP32P4_I2S_ETM_TASKS);
    }
    timer_init_ns(&s->tx.timer, QEMU_CLOCK_VIRTUAL, i2s_tx_timer_cb, s);
    timer_init_ns(&s->rx.timer, QEMU_CLOCK_VIRTUAL, i2s_rx_timer_cb, s);
    timer_init_ns(&s->tx.hung_timer, QEMU_CLOCK_VIRTUAL, i2s_tx_hung_cb, s);
    timer_init_ns(&s->rx.hung_timer, QEMU_CLOCK_VIRTUAL, i2s_rx_hung_cb, s);
}

static const VMStateDescription vmstate_esp32p4_i2s_fifo = {
    .name = "esp32p4-i2s.fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, ESP32P4I2sFifo, ESP32P4_I2S_FIFO_BYTES),
        VMSTATE_UINT8_ARRAY(eof, ESP32P4I2sFifo, ESP32P4_I2S_FIFO_BYTES),
        VMSTATE_UINT32(head, ESP32P4I2sFifo),
        VMSTATE_UINT32(num, ESP32P4I2sFifo),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_i2s_frame = {
    .name = "esp32p4-i2s.frame",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, ESP32P4I2sFrame),
        VMSTATE_UINT32(pos, ESP32P4I2sFrame),
        VMSTATE_UINT32(limit, ESP32P4I2sFrame),
        VMSTATE_UINT32_ARRAY(slot, ESP32P4I2sFrame, ESP32P4_I2S_SLOTS),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_i2s_pdm = {
    .name = "esp32p4-i2s.pdm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64_ARRAY(integ, ESP32P4I2sPdmChan, 3),
        VMSTATE_INT64_ARRAY(comb, ESP32P4I2sPdmChan, 3),
        VMSTATE_INT32_ARRAY(hp_x, ESP32P4I2sPdmChan, 2),
        VMSTATE_INT32_ARRAY(hp_y, ESP32P4I2sPdmChan, 2),
        VMSTATE_INT32(pcm, ESP32P4I2sPdmChan),
        VMSTATE_UINT32(word, ESP32P4I2sPdmChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_i2s_unit = {
    .name = "esp32p4-i2s.unit",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(conf, ESP32P4I2sUnit),
        VMSTATE_UINT32(conf1, ESP32P4I2sUnit),
        VMSTATE_UINT32(tdm_ctrl, ESP32P4I2sUnit),
        VMSTATE_UINT32(pdm_conf, ESP32P4I2sUnit),
        VMSTATE_UINT32(pdm_conf1, ESP32P4I2sUnit),
        VMSTATE_UINT32(single_data, ESP32P4I2sUnit),
        VMSTATE_BOOL(update_pending, ESP32P4I2sUnit),
        VMSTATE_UINT8(mode, ESP32P4I2sUnit),
        VMSTATE_BOOL(stalled, ESP32P4I2sUnit),
        VMSTATE_BOOL(drained, ESP32P4I2sUnit),
        VMSTATE_BOOL(started, ESP32P4I2sUnit),
        VMSTATE_UINT8(prev_ws, ESP32P4I2sUnit),
        VMSTATE_STRUCT(cur, ESP32P4I2sUnit, 1, vmstate_esp32p4_i2s_frame,
                       ESP32P4I2sFrame),
        VMSTATE_STRUCT(prev, ESP32P4I2sUnit, 1, vmstate_esp32p4_i2s_frame,
                       ESP32P4I2sFrame),
        VMSTATE_UINT32_ARRAY(last, ESP32P4I2sUnit, ESP32P4_I2S_SLOTS),
        VMSTATE_UINT32(last_chan, ESP32P4I2sUnit),
        VMSTATE_UINT32(pdm_bit, ESP32P4I2sUnit),
        VMSTATE_STRUCT_ARRAY(pdm, ESP32P4I2sUnit, ESP32P4_I2S_PDM_CHANS, 1,
                             vmstate_esp32p4_i2s_pdm, ESP32P4I2sPdmChan),
        VMSTATE_UINT32(eof_count, ESP32P4I2sUnit),
        VMSTATE_UINT32(words, ESP32P4I2sUnit),
        VMSTATE_INT64(anchor, ESP32P4I2sUnit),
        VMSTATE_UINT64(events, ESP32P4I2sUnit),
        VMSTATE_UINT64(frame_ticks, ESP32P4I2sUnit),
        VMSTATE_TIMER(timer, ESP32P4I2sUnit),
        VMSTATE_TIMER(hung_timer, ESP32P4I2sUnit),
        VMSTATE_BOOL(hung_fired, ESP32P4I2sUnit),
        VMSTATE_BOOL(sd_level, ESP32P4I2sUnit),
        VMSTATE_BOOL(sd_next, ESP32P4I2sUnit),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_i2s_vad = {
    .name = "esp32p4-i2s.vad",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(flag, ESP32P4I2sVad),
        VMSTATE_BOOL(energy_enough, ESP32P4I2sVad),
        VMSTATE_UINT32(init_frames, ESP32P4I2sVad),
        VMSTATE_UINT32(speech_count, ESP32P4I2sVad),
        VMSTATE_UINT32(silent_count, ESP32P4I2sVad),
        VMSTATE_UINT32(noise_amp, ESP32P4I2sVad),
        VMSTATE_UINT32(threshold, ESP32P4I2sVad),
        VMSTATE_INT32(offset, ESP32P4I2sVad),
        VMSTATE_UINT64(energy, ESP32P4I2sVad),
        VMSTATE_UINT16_ARRAY(max_signal, ESP32P4I2sVad, 3),
        VMSTATE_UINT32(samples, ESP32P4I2sVad),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_i2s = {
    .name = TYPE_ESP32P4_I2S,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(apb_clk, ESP32P4I2sState),
        VMSTATE_CLOCK(tx_clk, ESP32P4I2sState),
        VMSTATE_CLOCK(rx_clk, ESP32P4I2sState),
        VMSTATE_CLOCK(mst_clk, ESP32P4I2sState),
        VMSTATE_ESP32P4_CLK_EDGES(pad_mclk_edges, ESP32P4I2sState),
        VMSTATE_UINT32(int_raw, ESP32P4I2sState),
        VMSTATE_UINT32(int_ena, ESP32P4I2sState),
        VMSTATE_UINT32(rx_conf, ESP32P4I2sState),
        VMSTATE_UINT32(tx_conf, ESP32P4I2sState),
        VMSTATE_UINT32(rx_conf1, ESP32P4I2sState),
        VMSTATE_UINT32(tx_conf1, ESP32P4I2sState),
        VMSTATE_UINT32(tx_pcm2pdm_conf, ESP32P4I2sState),
        VMSTATE_UINT32(tx_pcm2pdm_conf1, ESP32P4I2sState),
        VMSTATE_UINT32(rx_pdm2pcm_conf, ESP32P4I2sState),
        VMSTATE_UINT32(rx_tdm_ctrl, ESP32P4I2sState),
        VMSTATE_UINT32(tx_tdm_ctrl, ESP32P4I2sState),
        VMSTATE_UINT32(rx_timing, ESP32P4I2sState),
        VMSTATE_UINT32(tx_timing, ESP32P4I2sState),
        VMSTATE_UINT32(lc_hung_conf, ESP32P4I2sState),
        VMSTATE_UINT32(rx_eof_num, ESP32P4I2sState),
        VMSTATE_UINT32(single_data, ESP32P4I2sState),
        VMSTATE_UINT32(etm_conf, ESP32P4I2sState),
        VMSTATE_UINT32(tx_fifo_cnt, ESP32P4I2sState),
        VMSTATE_UINT32(tx_bck_cnt, ESP32P4I2sState),
        VMSTATE_UINT32(clk_gate, ESP32P4I2sState),
        VMSTATE_UINT32(date, ESP32P4I2sState),
        VMSTATE_UINT32(vad_conf, ESP32P4I2sState),
        VMSTATE_UINT32(rx_mem_conf, ESP32P4I2sState),
        VMSTATE_UINT32(eco_low, ESP32P4I2sState),
        VMSTATE_UINT32(eco_high, ESP32P4I2sState),
        VMSTATE_UINT32(eco_conf, ESP32P4I2sState),
        VMSTATE_UINT32_ARRAY(vad_param, ESP32P4I2sState, 9),
        VMSTATE_STRUCT(tx, ESP32P4I2sState, 1, vmstate_esp32p4_i2s_unit,
                       ESP32P4I2sUnit),
        VMSTATE_STRUCT(rx, ESP32P4I2sState, 1, vmstate_esp32p4_i2s_unit,
                       ESP32P4I2sUnit),
        VMSTATE_STRUCT(tx_fifo, ESP32P4I2sState, 1, vmstate_esp32p4_i2s_fifo,
                       ESP32P4I2sFifo),
        VMSTATE_STRUCT(rx_fifo, ESP32P4I2sState, 1, vmstate_esp32p4_i2s_fifo,
                       ESP32P4I2sFifo),
        VMSTATE_UINT32_ARRAY(mem, ESP32P4I2sState, ESP32P4_LP_I2S_MEM_WORDS),
        VMSTATE_UINT32(mem_rd, ESP32P4I2sState),
        VMSTATE_UINT32(mem_num, ESP32P4I2sState),
        VMSTATE_UINT32(mem_wr, ESP32P4I2sState),
        VMSTATE_BOOL(mem_half_valid, ESP32P4I2sState),
        VMSTATE_UINT16(mem_half, ESP32P4I2sState),
        VMSTATE_STRUCT(vad, ESP32P4I2sState, 1, vmstate_esp32p4_i2s_vad,
                       ESP32P4I2sVad),
        VMSTATE_UINT32(in_level, ESP32P4I2sState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32p4_i2s_properties[] = {
    DEFINE_PROP_UINT8("id", ESP32P4I2sState, id, 0),
    DEFINE_PROP_LINK("gdma", ESP32P4I2sState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
    DEFINE_PROP_LINK("gpio", ESP32P4I2sState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
    DEFINE_PROP_LINK("i2s0", ESP32P4I2sState, bus[0], TYPE_ESP32P4_I2S,
                     ESP32P4I2sState *),
    DEFINE_PROP_LINK("i2s1", ESP32P4I2sState, bus[1], TYPE_ESP32P4_I2S,
                     ESP32P4I2sState *),
    DEFINE_PROP_LINK("i2s2", ESP32P4I2sState, bus[2], TYPE_ESP32P4_I2S,
                     ESP32P4I2sState *),
    DEFINE_PROP_LINK("lp-i2s", ESP32P4I2sState, bus[3], TYPE_ESP32P4_I2S,
                     ESP32P4I2sState *),
};

static void esp32p4_i2s_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    ESP32P4ClkSourceClass *csc = ESP32P4_CLK_SOURCE_CLASS(klass);

    rc->phases.hold = esp32p4_i2s_reset_hold;
    rc->phases.exit = esp32p4_i2s_reset_exit;
    dc->realize = esp32p4_i2s_realize;
    dc->vmsd = &vmstate_esp32p4_i2s;
    device_class_set_props(dc, esp32p4_i2s_properties);
    csc->exact_edges = esp32p4_i2s_exact_edges;
}

static void esp32p4_lp_i2s_class_init(ObjectClass *klass, const void *data)
{
    ESP32P4_I2S_CLASS(klass)->lp = true;
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
static const TypeInfo esp32p4_i2s_types[] = {
    {
        .name = TYPE_ESP32P4_I2S,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4I2sState),
        .instance_init = esp32p4_i2s_init,
        .class_size = sizeof(ESP32P4I2sClass),
        .class_init = esp32p4_i2s_class_init,
        .interfaces = (const InterfaceInfo[]) {
            { TYPE_ESP32P4_CLK_SOURCE },
            { }
        },
    },
    {
        .name = TYPE_ESP32P4_LP_I2S,
        .parent = TYPE_ESP32P4_I2S,
        .class_init = esp32p4_lp_i2s_class_init,
    },
};

DEFINE_TYPES(esp32p4_i2s_types)
