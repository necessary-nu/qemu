/*
 * Everest Semiconductor ES8311 low power mono audio codec
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: Everest's ES8311 datasheet (revision 10.0, January 2021) and
 * user guide (revision 1.11, September 2018), and the way Espressif's
 * esp_codec_dev and esp-bsp es8311 drivers and Linux's es8311 driver
 * program it.
 *
 * Control port. The I2C address is 0011 00x, x the CE pin. A write
 * transfer is the register address and its data; a read writes the
 * register address, then reads after a repeated START. The documents show
 * one data byte per transfer, and Linux's driver makes single transfers
 * only: the model does not advance the address, so further bytes of a
 * write go to the same register and further bytes of a read return it
 * again. It does not answer the general call. Registers 0x00 to 0x45 and
 * 0xFA hold the eight bits written; 0xFC to 0xFF are read-only; other
 * addresses read 0 and ignore writes (logged). INI_REG (0xFA bit 0)
 * holds the others at their defaults while it is set.
 *
 * Resets and power. RST_DIG, RST_CMG, RST_MST, RST_ADC_DIG and
 * RST_DAC_DIG hold their blocks in reset while set. The chip state
 * machine runs with CSM_ON set and RST_DIG clear: unless SEQ_DIS, it
 * powers up through stages A, B and C (PWRUP_A to _C), counted in LRCK
 * periods, before the converters work. The documents give only each
 * stage's range ends ("0~31: 21us~232ms (LRCK=48KHz)", about 1 to 11136
 * periods for A, 1 to 4992 for B, 1 to 11232 for C); the model takes the
 * lengths geometrically between them, the field's top value or more
 * giving the top. FLAG_CSM_CHIP reads S0 while the machine is off, S1 to
 * S3 in the stages and S6 once powered up (the datasheet names S6 and S7
 * without saying what they are). PDN_VREF is taken by its name, as both
 * Espressif's and Linux's drivers use it, 1 powering the references down
 * (the datasheet's text has it the other way round). The DAC's output
 * needs PDN_ANA, PDN_IBIASGEN, PDN_VREF, PDN_DACVREFGEN and PDN_DAC
 * clear and VMIDSEL non-zero; the ADC's PDN_ANA, PDN_IBIASGEN, PDN_VREF,
 * PDN_ADCBIASGEN, PDN_ADCVREFGEN, PDN_PGA, PDN_MOD and RST_MOD clear and
 * VMIDSEL non-zero. VMID's charge time is not modelled.
 *
 * Clocks (user guide chapter 8, datasheet register 0x02's note). The
 * internal master clock is the MCLK pin (MCLK_ON) or SCLK (MCLK_SEL),
 * divided by DIV_PRE + 1 and multiplied by MULT_PRE's 1 to 8; the ADC's
 * and DAC's clocks divide it by DIV_CLKADC + 1 and DIV_CLKDAC + 1. A
 * converter works only with: its digital and analog clocks on
 * (CLKDAC_ON and ANACLKDAC_ON, or the ADC's), BCLK_ON, its block and the
 * clock manager out of reset; the divider's output at most 10 MHz (4.5
 * MHz with DVDD at 1.8 V, property "dvdd-mv") and, before the x4 and x8
 * multipliers, more than 1 MHz (0.5 MHz); its clock at most 35 MHz (13
 * MHz); MCLK at most 49.2 MHz, SCLK 26 MHz, LRCK 200 kHz; and its clock
 * at least 256 times LRCK (the ADC's: 240 with its equaliser bypassed; 128
 * in double speed mode). The oversampling rates bound the clock too: an
 * ADC_OSR below 15 (16 in double speed) is "not use", and the decimator
 * takes 16 (double speed 8) of its clock's cycles per OSR step of a
 * sample (user guide 10.6: OSR = clock / LRCK / 8, the register counting
 * in twos); a DAC_OSR below 15 is "not use", and the model takes the
 * modulator to need two clock cycles per step, its 4 * DAC_OSR steps a
 * sample (Espressif's driver runs DAC_OSR 32 at 256 fs). The ratio need
 * not be a whole multiple (the vendor's table has USB clocks at 375 and
 * 272.1). A converter whose clocks fall short gives silence, logged once
 * per cause: its DAC output, or its ADC data on ASDOUT. How far the ADC's
 * amplitude falls with a lower ADC_OSR is not documented, and is not
 * modelled.
 *
 * Serial port (datasheet chapter 6, user guide chapter 7). DSDIN is
 * sampled on SCLK's rising edges and ASDOUT changes on its falling edges,
 * both inverted with BCLK_INV; LRCK is sampled with them. I2S puts a
 * channel's MSB one cycle after LRCK changes, left while LRCK is low;
 * left-justified with the change, left while LRCK is high; SDP_*_LRP swaps
 * the channels. DSP/PCM starts a frame where LRCK rises, its left word
 * then the right, the MSB on the second rising edge after (mode A) or,
 * with LRP, the first (mode B). A word is SDP_*_WL bits, MSB first: a word
 * cut short by the next one's start keeps the bits it has (the rest 0), and
 * bits past its length are ignored, so I2S and left-justified data of a
 * shorter or longer slot comes through, as on any MSB-first codec, while a
 * format or polarity mismatch garbles it. The reserved format (2) and word
 * lengths (5 to 7) move no data. Without BCLK_ON the port is stopped;
 * SDP_IN_MUTE and SDP_OUT_MUTE send zeros; TRI_ADCDAT releases ASDOUT.
 * ADCDAT_SEL chooses ASDOUT's left and right words from the ADC's output
 * and the words DSDIN carried (DACL, DACR), which pass unchanged; 7 sends
 * zeros. A frame starts where SDP_OUT's format starts its left word: the
 * ADC's sample goes out in the frame after it is taken, and a DAC sample
 * is converted as its word completes, a frame's latency each way (the
 * filters' group delay is not documented).
 *
 * The DAC. SDP_IN_SEL's channel (or the ADC's output with ADC2DAC_SEL),
 * 24 bits, through DAC_VOLUME (-95.5 dB to +32 dB in 0.5 dB steps,
 * moving 0.25 dB per 2^(DAC_RAMPRATE + 1) LRCK periods unless
 * DAC_RAMPRATE is 0, starting from -95.5 dB as the DAC's digital block
 * leaves reset), DAC_INV, and DAC_DSMMUTE or DAC_DEMMUTE muting it.
 * With DRC_EN, DAC_VOLUME is the highest gain: at the end of each window
 * of 2^(DRC_WINSIZE + 1) periods, a peak over DRC_MAXLEVEL turns the gain
 * down and one under DRC_MINLEVEL up, 0.25 dB per ramp step (per window
 * with no ramp) until the next window decides.
 *
 * The ADC. The analog input (LINSEL selecting MIC1P-MIC1N; else none)
 * through the PGA (0 to 30 dB in 3 dB steps), clipping at full scale;
 * 24 bits; ADC_SCALE (0 to 42 dB in 6 dB steps); ADC_INV; ADC_VOLUME (or,
 * with ALC_EN, the ALC as the DRC above, with ADC_VOLUME its highest gain,
 * ALC_WINSIZE, the ALC levels and ADC_RAMPRATE); then automute: a level
 * that stays under ADC_AUTOMUTE_NG (taken before the volume) for 2048 *
 * (ADC_AUTOMUTE_WS + 1) samples brings the output down by ADC_AUTOMUTE_VOL
 * * 4 dB and sets FLAG_ADCAM, until a sample goes over it.
 *
 * Not modelled (logged where software enables them): the equalisers,
 * whose coefficients' format is not documented (they pass the signal
 * unchanged); the ADC's high-pass filter and DAC_OFFSET; DSM dither and
 * the mute targets; the low power and bias settings (0x0F to 0x11) and
 * HPSW; a digital microphone on MIC1P (DMIC_ON): the model has none and
 * the ADC records silence.
 *
 * Master mode (MSC, with RST_MST and RST_CMG clear and BCLK_ON): SCLK and
 * LRCK are generated from the internal master clock's source, the MCLK
 * pin (MCLK_SEL must be clear: SCLK cannot clock itself): SCLK divides it
 * by DIV_BCLK's divider and LRCK by DIV_LRCK + 1, as the datasheet's note
 * has it ("MCLK here is MCLK pin"; the user guide's diagram puts the
 * dividers after the pre-divider and multiplier). LRCK changes on SCLK's
 * falling edges, at the whole number of SCLK periods closest to its
 * division, with half the frame each level for I2S and left-justified and a
 * one-period pulse for DSP/PCM (SDP_OUT_FMT, SDP_OUT_LRP); DIV_LRCK 0 holds
 * it high. BCLK_CON stops SCLK outside the words. TRI_BLRCK releases both
 * pins. The edges are timed from MCLK's period, a frame at a time, each
 * edge at its own time (esp32_line_run_at()).
 *
 * Audio streams. The playback stream gets one sample per LRCK period, the
 * capture stream gives one: their rate is LRCK's, as the codec measures it
 * over its frames. QEMU's audio timer moves samples between the codec's
 * buffers and the backend on the virtual clock, so with a backend paced by
 * virtual time (wav, none) the streams are exact under icount; a host
 * sound device plays at the host's real time, and the codec's buffers
 * absorb the difference (an empty capture buffer gives silence and fills
 * again before the ADC takes from it; a full playback buffer drops its
 * oldest samples). The codec never holds up its serial port for the
 * backend.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/audio/es8311.h"
#include "hw/gpio/esp32_line.h"
#include "migration/vmstate.h"

/* Registers */
#define R_RESET             0x00
#define R_CLK1              0x01
#define R_CLK2              0x02
#define R_CLK3              0x03
#define R_CLK4              0x04
#define R_CLK5              0x05
#define R_CLK6              0x06
#define R_CLK7              0x07
#define R_CLK8              0x08
#define R_SDPIN             0x09
#define R_SDPOUT            0x0a
#define R_SYS0B             0x0b
#define R_SYS0C             0x0c
#define R_SYS0D             0x0d
#define R_SYS0E             0x0e
#define R_SYS12             0x12
#define R_SYS14             0x14
#define R_ADC15             0x15
#define R_ADC16             0x16
#define R_ADC17             0x17
#define R_ADC18             0x18
#define R_ADC19             0x19
#define R_ADC1A             0x1a
#define R_ADC1B             0x1b
#define R_ADC1C             0x1c
#define R_DAC31             0x31
#define R_DAC32             0x32
#define R_DAC33             0x33
#define R_DAC34             0x34
#define R_DAC35             0x35
#define R_DAC37             0x37
#define R_GPIO44            0x44
#define R_GP45              0x45
#define R_LAST              0x45
#define R_I2C               0xfa
#define R_FLAG              0xfc
#define R_CHIP_ID1          0xfd
#define R_CHIP_ID2          0xfe
#define R_CHIP_VER          0xff

/* 0x00 */
#define CSM_ON              BIT(7)
#define MSC                 BIT(6)
#define SEQ_DIS             BIT(5)
#define RST_DIG             BIT(4)
#define RST_CMG             BIT(3)
#define RST_MST             BIT(2)
#define RST_ADC_DIG         BIT(1)
#define RST_DAC_DIG         BIT(0)
/* 0x01 */
#define MCLK_SEL            BIT(7)
#define MCLK_ON             BIT(5)
#define BCLK_ON             BIT(4)
#define CLKADC_ON           BIT(3)
#define CLKDAC_ON           BIT(2)
#define ANACLKADC_ON        BIT(1)
#define ANACLKDAC_ON        BIT(0)
/* 0x02 */
#define DIV_PRE(v)          (((v) >> 5) & 7)
#define MULT_PRE(v)         (((v) >> 3) & 3)
/* 0x03, 0x04 */
#define ADC_FSMODE          BIT(6)
#define ADC_OSR(v)          ((v) & 0x3f)
#define DAC_OSR(v)          ((v) & 0x7f)
/* 0x05 */
#define DIV_CLKADC(v)       (((v) >> 4) & 0xf)
#define DIV_CLKDAC(v)       ((v) & 0xf)
/* 0x06 */
#define BCLK_CON            BIT(6)
#define BCLK_INV            BIT(5)
#define DIV_BCLK(v)         ((v) & 0x1f)
/* 0x07 */
#define TRI_BLRCK           BIT(5)
#define TRI_ADCDAT          BIT(4)
#define DIV_LRCK_H(v)       ((v) & 0xf)
/* 0x09, 0x0a */
#define SDP_IN_SEL          BIT(7)
#define SDP_MUTE            BIT(6)
#define SDP_LRP             BIT(5)
#define SDP_WL(v)           (((v) >> 2) & 7)
#define SDP_FMT(v)          ((v) & 3)
#define FMT_I2S             0
#define FMT_LJ              1
#define FMT_DSP             3
/* 0x0b, 0x0c */
#define PWRUP_A(b)          ((b) >> 3)
#define PWRUP_B(b, c)       ((((b) & 7) << 1) | ((c) >> 7))
#define PWRUP_C(c)          ((c) & 0x7f)
/* 0x0d */
#define PDN_ANA             BIT(7)
#define PDN_IBIASGEN        BIT(6)
#define PDN_ADCBIASGEN      BIT(5)
#define PDN_ADCVREFGEN      BIT(4)
#define PDN_DACVREFGEN      BIT(3)
#define PDN_VREF            BIT(2)
#define VMIDSEL(v)          ((v) & 3)
/* 0x0e */
#define PDN_PGA             BIT(6)
#define PDN_MOD             BIT(5)
#define RST_MOD             BIT(4)
/* 0x12 */
#define PDN_DAC             BIT(1)
/* 0x14 */
#define DMIC_ON             BIT(6)
#define LINSEL(v)           (((v) >> 4) & 3)
#define PGAGAIN(v)          ((v) & 0xf)
/* 0x15, 0x37 */
#define RAMPRATE(v)         (((v) >> 4) & 0xf)
/* 0x16 */
#define ADC_INV             BIT(4)
#define ADC_SCALE(v)        ((v) & 7)
/* 0x18 */
#define ALC_EN              BIT(7)
#define AUTOMUTE_EN         BIT(6)
#define WINSIZE(v)          ((v) & 0xf)
/* 0x19, 0x35 */
#define MAXLEVEL(v)         (((v) >> 4) & 0xf)
#define MINLEVEL(v)         ((v) & 0xf)
/* 0x1a */
#define AUTOMUTE_WS(v)      (((v) >> 4) & 0xf)
#define AUTOMUTE_NG(v)      ((v) & 0xf)
/* 0x1b */
#define AUTOMUTE_VOL(v)     (((v) >> 5) & 7)
/* 0x1c */
#define ADC_EQBYPASS        BIT(6)
#define ADC_HPF             BIT(5)
/* 0x31 */
#define DAC_DSMMUTE         BIT(6)
#define DAC_DEMMUTE         BIT(5)
#define DAC_INV             BIT(4)
/* 0x34 */
#define DRC_EN              BIT(7)
/* 0x37 */
#define DAC_EQBYPASS        BIT(3)
/* 0x44 */
#define ADC2DAC_SEL         BIT(7)
#define ADCDAT_SEL(v)       (((v) >> 4) & 7)
/* 0x45 */
#define PULLUP_SE           BIT(0)
/* 0xfa */
#define INI_REG             BIT(0)
/* 0xfc */
#define FLAG_ADCAM          BIT(1)

#define CHIP_ID1            0x83
#define CHIP_ID2            0x11
#define CHIP_VER            0x00

/* The chip state machine's states, as FLAG_CSM_CHIP reads them */
#define CSM_S0              0
#define CSM_S1              1
#define CSM_S2              2
#define CSM_S3              3
#define CSM_S6              6

enum {
    LEFT,
    RIGHT,
};

/* The 24-bit converters' full scale */
#define FS24                ((1 << 23) - 1)

/* Guest errors logged once each, bits of ES8311State.logged */
enum {
    LOG_DAC_CLOCK = 1 << 0,
    LOG_ADC_CLOCK = 1 << 1,
    LOG_SDP_IN = 1 << 2,
    LOG_SDP_OUT = 1 << 3,
    LOG_EQ = 1 << 4,
    LOG_HPF = 1 << 5,
    LOG_OFFSET = 1 << 6,
    LOG_DMIC = 1 << 7,
    LOG_LINSEL = 1 << 8,
    LOG_MASTER = 1 << 9,
    LOG_MASTER_LRCK = 1 << 10,
    LOG_LAG = 1 << 11,
};

/* Register reset values (datasheet chapter 8) */
static const uint8_t es8311_reset_regs[R_LAST + 1] = {
    [0x00] = 0x1f, [0x03] = 0x10, [0x04] = 0x10, [0x06] = 0x03,
    [0x08] = 0xff, [0x0c] = 0x20, [0x0d] = 0xfc, [0x0e] = 0x6a,
    [0x10] = 0x13, [0x11] = 0x7c, [0x12] = 0x02, [0x13] = 0x40,
    [0x14] = 0x10, [0x16] = 0x04, [0x1b] = 0x0c, [0x1c] = 0x4c,
    [0x37] = 0x08,
};

/*
 * Gains in 0.25 dB steps from -95.5 dB (index 0) to +42 dB (550), as
 * multipliers with 24 fraction bits. 0 dB is index 382.
 */
#define GAIN_0DB            382
#define GAIN_MAX            550
static const uint32_t gain_q24[GAIN_MAX + 1] = {
    282, 290, 298, 307, 316, 325, 335, 345, 355, 365, 376, 387, 398, 409, 421,
    434, 446, 459, 473, 487, 501, 515, 531, 546, 562, 578, 595, 613, 631, 649,
    668, 687, 707, 728, 749, 771, 794, 817, 841, 865, 891, 917, 943, 971, 999,
    1029, 1059, 1089, 1121, 1154, 1188, 1222, 1258, 1295, 1333, 1372, 1412,
    1453, 1495, 1539, 1584, 1630, 1678, 1727, 1777, 1829, 1882, 1937, 1994,
    2052, 2112, 2174, 2237, 2303, 2370, 2439, 2510, 2584, 2659, 2737, 2817,
    2899, 2983, 3071, 3160, 3253, 3347, 3445, 3546, 3649, 3756, 3866, 3979,
    4095, 4214, 4337, 4464, 4594, 4728, 4867, 5009, 5155, 5305, 5460, 5620,
    5784, 5953, 6127, 6306, 6490, 6679, 6874, 7075, 7281, 7494, 7713, 7938,
    8170, 8409, 8654, 8907, 9167, 9435, 9710, 9994, 10285, 10586, 10895, 11213,
    11540, 11877, 12224, 12581, 12949, 13327, 13716, 14116, 14528, 14953,
    15389, 15839, 16301, 16777, 17267, 17771, 18290, 18824, 19374, 19940,
    20522, 21121, 21738, 22373, 23026, 23698, 24390, 25103, 25836, 26590,
    27367, 28166, 28988, 29835, 30706, 31602, 32525, 33475, 34452, 35458,
    36494, 37560, 38656, 39785, 40947, 42142, 43373, 44640, 45943, 47285,
    48665, 50086, 51549, 53054, 54603, 56198, 57839, 59528, 61266, 63055,
    64896, 66791, 68742, 70749, 72815, 74941, 77129, 79382, 81700, 84085,
    86541, 89068, 91668, 94345, 97100, 99936, 102854, 105857, 108948, 112130,
    115404, 118774, 122242, 125811, 129485, 133266, 137158, 141163, 145285,
    149527, 153893, 158387, 163012, 167772, 172671, 177713, 182903, 188243,
    193740, 199398, 205220, 211213, 217380, 223728, 230261, 236984, 243905,
    251027, 258357, 265901, 273665, 281657, 289881, 298346, 307058, 316024,
    325252, 334749, 344524, 354585, 364939, 375595, 386563, 397850, 409468,
    421425, 433730, 446396, 459431, 472846, 486654, 500864, 515490, 530542,
    546034, 561979, 578389, 595278, 612661, 630551, 648963, 667913, 687416,
    707489, 728148, 749411, 771294, 793816, 816996, 840853, 865406, 890676,
    916685, 943452, 971001, 999355, 1028537, 1058571, 1089482, 1121295,
    1154037, 1187736, 1222418, 1258114, 1294851, 1332662, 1371576, 1411627,
    1452847, 1495271, 1538934, 1583871, 1630121, 1677722, 1726712, 1777133,
    1829026, 1882435, 1937403, 1993976, 2052201, 2112126, 2173802, 2237278,
    2302607, 2369845, 2439045, 2510267, 2583568, 2659010, 2736654, 2816566,
    2898811, 2983458, 3070576, 3160239, 3252519, 3347495, 3445243, 3545846,
    3649387, 3755951, 3865627, 3978505, 4094679, 4214246, 4337304, 4463956,
    4594306, 4728462, 4866536, 5008641, 5154896, 5305422, 5460343, 5619788,
    5783888, 5952781, 6126605, 6305505, 6489629, 6679130, 6874164, 7074893,
    7281484, 7494107, 7712939, 7938161, 8169959, 8408526, 8654060, 8906763,
    9166845, 9434522, 9710015, 9993552, 10285369, 10585708, 10894816, 11212950,
    11540374, 11877359, 12224184, 12581137, 12948513, 13326616, 13715761,
    14116268, 14528471, 14952709, 15389336, 15838713, 16301212, 16777216,
    17267120, 17771329, 18290261, 18824346, 19374027, 19939759, 20522010,
    21121264, 21738016, 22372777, 23026074, 23698447, 24390455, 25102669,
    25835680, 26590095, 27366540, 28165658, 28988110, 29834578, 30705763,
    31602388, 32525194, 33474947, 34452433, 35458462, 36493868, 37559508,
    38656266, 39785049, 40946793, 42142461, 43373043, 44639559, 45943058,
    47284619, 48665355, 50086409, 51548959, 53054215, 54603426, 56197875,
    57838883, 59527809, 61266052, 63055053, 64896294, 66791300, 68741641,
    70748933, 72814839, 74941071, 77129390, 79381609, 81699594, 84085265,
    86540599, 89067630, 91668452, 94345219, 97100149, 99935524, 102853694,
    105857077, 108948159, 112129503, 115403743, 118773593, 122241845,
    125811371, 129485130, 133266164, 137157606, 141162680, 145284705,
    149527095, 153893365, 158387132, 163012120, 167772160, 172671196,
    177713285, 182902607, 188243460, 193740268, 199397586, 205220100,
    211212636, 217380156, 223727771, 230260740, 236984475, 243904547,
    251026688, 258356800, 265900954, 273665402, 281656576, 289881097,
    298345778, 307057632, 316023876, 325251940, 334749468, 344524329,
    354584621, 364938679, 375595081, 386562655, 397850489, 409467933,
    421424612, 433730433, 446395590, 459430577, 472846192, 486653550,
    500864090, 515489586, 530542154, 546034264, 561978753, 578388828,
    595278087, 612660521, 630550531, 648962939, 667912999, 687416411,
    707489332, 728148394, 749410711, 771293899, 793816087, 816995935,
    840852648, 865405988, 890676299, 916684517, 943452188, 971001489,
    999355245, 1028536944, 1058570766, 1089481590, 1121295027, 1154037433,
    1187735934, 1222418450, 1258113713, 1294851296, 1332661637, 1371576059,
    1411626803, 1452847050, 1495270950, 1538933650, 1583871324, 1630121202,
    1677721600, 1726711955, 1777132855, 1829026071, 1882434596, 1937402679,
    1993975859, 2052201005, 2112126356,
};

/* ALC_MAXLEVEL and _MINLEVEL, DRC's: -30.1 dBFS to -6.0 dBFS, 24-bit */
static const uint32_t level24[16] = {
    262235, 523227, 782870, 1043975, 1314286, 1580119, 1835227, 2107123,
    2364231, 2622345, 2875345, 3152753, 3417353, 3661760, 3923647, 4204263,
};

/* ADC_AUTOMUTE_NG: -96 dB to -54 dB in 6 dB steps, then to -30 in 3 */
static const uint32_t noise_gate24[16] = {
    133, 265, 529, 1056, 2107, 4204, 8389, 16737,
    23642, 33396, 47173, 66633, 94122, 132950, 187798, 265271,
};

/* The power-up stages' lengths in LRCK periods, by field value */
static const uint16_t pwrup_frames[3][32] = {
    { 1, 1, 2, 2, 3, 4, 6, 8, 11, 15, 20, 27, 37, 50, 67, 91, 123, 166,
      224, 302, 408, 551, 745, 1006, 1358, 1834, 2478, 3346, 4520, 6104,
      8245, 11136 },
    { 1, 1, 2, 2, 3, 4, 5, 7, 9, 12, 16, 21, 27, 36, 47, 62, 81, 107,
      140, 185, 243, 320, 421, 554, 730, 960, 1264, 1664, 2190, 2882,
      3793, 4992 },
    { 1, 1, 2, 2, 3, 5, 6, 8, 11, 15, 20, 27, 37, 50, 67, 91, 123, 166,
      225, 304, 410, 554, 749, 1012, 1367, 1847, 2495, 3371, 4555, 6154,
      8314, 11232 },
};

/* DIV_BCLK 20 to 31's dividers */
static const uint8_t bclk_div_high[12] = {
    22, 24, 25, 30, 32, 33, 34, 36, 44, 48, 66, 72,
};

static void es8311_update(ES8311State *s);

static int32_t sat24(int64_t v)
{
    return MIN(MAX(v, -FS24 - 1), FS24);
}

static int32_t gain_apply(int32_t v, int q)
{
    return sat24(((int64_t)v * gain_q24[MIN(MAX(q, 0), GAIN_MAX)]) >> 24);
}

static void log_once(ES8311State *s, uint32_t bit, int mask, const char *fmt,
                     ...) G_GNUC_PRINTF(4, 5);

static void log_once(ES8311State *s, uint32_t bit, int mask, const char *fmt,
                     ...)
{
    va_list ap;
    g_autofree char *msg = NULL;

    if (s->logged & bit) {
        return;
    }
    s->logged |= bit;
    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    qemu_log_mask(mask, "es8311: %s\n", msg);
}

/* ---- Pins ---- */

static void pin_drive(qemu_irq out, qemu_irq oe, bool *cur_level,
                      bool *cur_oe, bool level, bool enable)
{
    if (*cur_oe != enable) {
        *cur_oe = enable;
        if (enable) {
            *cur_level = level;
            qemu_set_irq(out, level);
        }
        qemu_set_irq(oe, enable);
        return;
    }
    if (enable && *cur_level != level) {
        *cur_level = level;
        qemu_set_irq(out, level);
    }
}

static void drive_asdout(ES8311State *s, bool level)
{
    pin_drive(s->asdout_out, s->asdout_oe, &s->drv_asdout, &s->oe_asdout,
              level, !(s->regs[R_CLK7] & TRI_ADCDAT));
}

/* ---- The audio backend ---- */

static void es8311_audio_out_cb(void *opaque, int free)
{
    ES8311State *s = opaque;
    uint8_t buf[4096];

    while (s->out_num && free >= 4) {
        int n = MIN(MIN(s->out_num, free / 4), (int)sizeof(buf) / 4);
        size_t done;

        for (int i = 0; i < n; i++) {
            stl_le_p(buf + 4 * i,
                     s->out_buf[(s->out_head + i) % ES8311_AUDIO_FRAMES]);
        }
        done = audio_be_write(s->audio_be, s->voice_out, buf, 4 * n) / 4;
        if (done == 0) {
            break;
        }
        s->out_head = (s->out_head + done) % ES8311_AUDIO_FRAMES;
        s->out_num -= done;
        free -= 4 * done;
    }
}

/*
 * The microphone's samples: the capture stream is stereo, and the mono
 * microphone takes the mean of its channels (QEMU's mixer gives a mono
 * source in both).
 */
static void es8311_audio_in_cb(void *opaque, int avail)
{
    ES8311State *s = opaque;
    uint8_t buf[4096];

    while (avail >= 8) {
        int n = MIN(avail, (int)sizeof(buf)) / 8;
        size_t got = audio_be_read(s->audio_be, s->voice_in, buf, 8 * n) / 8;

        if (got == 0) {
            break;
        }
        for (size_t i = 0; i < got; i++) {
            int64_t l = (int32_t)ldl_le_p(buf + 8 * i);
            int64_t r = (int32_t)ldl_le_p(buf + 8 * i + 4);

            if (s->in_num == ES8311_AUDIO_FRAMES) {
                s->in_head = (s->in_head + 1) % ES8311_AUDIO_FRAMES;
                s->in_num--;
            }
            s->in_buf[(s->in_head + s->in_num) % ES8311_AUDIO_FRAMES] =
                (int32_t)((l + r) / 2);
            s->in_num++;
        }
        avail -= 8 * got;
    }
}

/*
 * The streams at LRCK's rate (@freq, Hz) while the codec has frames:
 * the board's speaker and microphone go on whether or not the converters
 * use them. Without frames (@freq 0) the microphone's stream stops; the
 * speaker's plays out what it has.
 */
static void es8311_voices(ES8311State *s, uint32_t freq)
{
    struct audsettings out_as = {
        .freq = freq,
        .nchannels = 1,
        .fmt = AUDIO_FORMAT_S32,
        .big_endian = false,
    };
    struct audsettings in_as = out_as;

    if (!s->audio_be) {
        return;
    }
    in_as.nchannels = 2;
    if (freq && freq != s->out_freq) {
        s->voice_out = audio_be_open_out(s->audio_be, s->voice_out,
                                         "es8311.out", s,
                                         es8311_audio_out_cb, &out_as);
        s->voice_in = audio_be_open_in(s->audio_be, s->voice_in,
                                       "es8311.in", s, es8311_audio_in_cb,
                                       &in_as);
        s->out_num = 0;
        s->out_freq = freq;
        if (s->voice_out) {
            audio_be_set_active_out(s->audio_be, s->voice_out, true);
        }
        /* The reopened capture stream starts afresh. */
        s->in_on = false;
    }
    if (s->voice_in && s->in_on != (freq != 0)) {
        s->in_on = freq != 0;
        s->in_num = 0;
        s->in_primed = false;
        audio_be_set_active_in(s->audio_be, s->voice_in, s->in_on);
    }
}

/* A sample for the speaker, 24-bit. */
static void speaker_put(ES8311State *s, int32_t v)
{
    if (!s->voice_out) {
        return;
    }
    if (s->out_num == ES8311_AUDIO_FRAMES) {
        s->out_head = (s->out_head + 1) % ES8311_AUDIO_FRAMES;
        s->out_num--;
    }
    s->out_buf[(s->out_head + s->out_num) % ES8311_AUDIO_FRAMES] =
        (int32_t)((uint32_t)v << 8);
    s->out_num++;
}

/*
 * The microphone's sample for this frame, 24-bit; silence while the
 * capture buffer refills to a twentieth of a second.
 */
static int32_t mic_take(ES8311State *s)
{
    int32_t v;
    if (!s->voice_in || !s->in_on) {
        return 0;
    }
    if (!s->in_primed) {
        if (s->in_num < MAX(s->out_freq / 20, 1)) {
            return 0;
        }
        s->in_primed = true;
    }
    if (s->in_num == 0) {
        s->in_primed = false;
        return 0;
    }
    v = s->in_buf[s->in_head];
    s->in_head = (s->in_head + 1) % ES8311_AUDIO_FRAMES;
    s->in_num--;
    return v >> 8;
}

/* ---- Clocks ---- */

static double clock_hz(Clock *c)
{
    uint64_t p = clock_get(c);

    return p ? 1e9 * 4294967296.0 / (double)p : 0;
}

/* The DIV_BCLK divider */
static unsigned bclk_div(ES8311State *s)
{
    unsigned d = DIV_BCLK(s->regs[R_CLK6]);

    return d < 20 ? d + 1 : bclk_div_high[d - 20];
}

static unsigned lrck_div(ES8311State *s)
{
    return ((unsigned)DIV_LRCK_H(s->regs[R_CLK7]) << 8 | s->regs[R_CLK8]) +
           1;
}

static double lrck_hz(ES8311State *s)
{
    return s->lrck_mhz / 1000.0;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Whether the clock manager gives a converter what it needs, the DAC or
 * the ADC: the source, the dividers' limits and the ratio of its clock to
 * LRCK. @why says what falls short.
 */
static bool converter_clocked(ES8311State *s, bool dac, const char **why)
{
    uint8_t r0 = s->regs[R_RESET], c1 = s->regs[R_CLK1];
    uint8_t c2 = s->regs[R_CLK2];
    bool low_v = s->dvdd_mv < 2500;
    double lrck = lrck_hz(s), sclk = lrck * s->cycles_per_frame;
    double src, pre, dig, conv, ratio, need;
    unsigned osr;

    *why = NULL;
    if ((r0 & (RST_DIG | RST_CMG)) || !(c1 & BCLK_ON) || lrck <= 0) {
        return false;
    }
    if (dac ? (r0 & RST_DAC_DIG) || !(c1 & CLKDAC_ON) ||
              !(c1 & ANACLKDAC_ON)
            : (r0 & RST_ADC_DIG) || !(c1 & CLKADC_ON) ||
              !(c1 & ANACLKADC_ON)) {
        return false;
    }
    if (c1 & MCLK_SEL) {
        src = sclk;
    } else {
        src = (c1 & MCLK_ON) ? clock_hz(s->mclk) : 0;
    }
    if (src <= 0) {
        *why = (c1 & MCLK_SEL) ? "no clock on SCLK" : "no clock on MCLK";
        return false;
    }
    if (lrck > 200e3 || sclk > 26e6 || (!(c1 & MCLK_SEL) && src > 49.2e6)) {
        *why = "LRCK, SCLK or MCLK over its maximum";
        return false;
    }
    pre = src / (DIV_PRE(c2) + 1);
    if (pre > (low_v ? 4.5e6 : 10e6)) {
        *why = "the pre-divider's output over its maximum";
        return false;
    }
    if (MULT_PRE(c2) >= 2 && pre <= (low_v ? 0.5e6 : 1e6)) {
        *why = "the pre-divider's output too slow for MULT_PRE";
        return false;
    }
    dig = pre * (1 << MULT_PRE(c2));
    conv = dig / ((dac ? DIV_CLKDAC(s->regs[R_CLK5])
                       : DIV_CLKADC(s->regs[R_CLK5])) + 1);
    if (conv > (low_v ? 13e6 : 35e6)) {
        *why = "the converter's clock over its maximum";
        return false;
    }
    ratio = conv / lrck;
    if (dac) {
        osr = DAC_OSR(s->regs[R_CLK4]);
        need = MAX(256.0, 8.0 * osr);
        if (osr < 15) {
            *why = "DAC_OSR below 15";
            return false;
        }
    } else if (s->regs[R_CLK3] & ADC_FSMODE) {
        osr = ADC_OSR(s->regs[R_CLK3]);
        need = MAX(128.0, 8.0 * osr);
        if (osr < 16) {
            *why = "ADC_OSR below 16 in double speed mode";
            return false;
        }
    } else {
        osr = ADC_OSR(s->regs[R_CLK3]);
        need = MAX((s->regs[R_ADC1C] & ADC_EQBYPASS) ? 240.0 : 256.0,
                   16.0 * osr);
        if (osr < 15) {
            *why = "ADC_OSR below 15";
            return false;
        }
    }
    /* A thousandth's slack for LRCK as measured */
    if (ratio < need * 0.999) {
        *why = "its clock too slow for LRCK";
        return false;
    }
    return true;
}

static bool dac_powered(ES8311State *s)
{
    uint8_t d = s->regs[R_SYS0D];

    return !(d & (PDN_ANA | PDN_IBIASGEN | PDN_VREF | PDN_DACVREFGEN)) &&
           VMIDSEL(d) && !(s->regs[R_SYS12] & PDN_DAC);
}

static bool adc_powered(ES8311State *s)
{
    uint8_t d = s->regs[R_SYS0D], e = s->regs[R_SYS0E];

    return !(d & (PDN_ANA | PDN_IBIASGEN | PDN_VREF | PDN_ADCBIASGEN |
                  PDN_ADCVREFGEN)) &&
           VMIDSEL(d) && !(e & (PDN_PGA | PDN_MOD | RST_MOD));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Whether each converter works: powered up by the state machine, its
 * analog part powered and its clocks as it needs them. A shortfall in the
 * clocks of a converter that is otherwise set up is logged once.
 */
static void converters_update(ES8311State *s)
{
    const char *why;
    bool on = s->csm == CSM_S6;

    s->dac_ok = on && dac_powered(s) && converter_clocked(s, true, &why);
    if (!s->dac_ok && why && on && dac_powered(s)) {
        log_once(s, LOG_DAC_CLOCK, LOG_GUEST_ERROR,
                 "the DAC is silent: %s", why);
    } else if (s->dac_ok) {
        s->logged &= ~LOG_DAC_CLOCK;
    }
    s->adc_ok = on && adc_powered(s) && converter_clocked(s, false, &why);
    if (!s->adc_ok && why && on && adc_powered(s)) {
        log_once(s, LOG_ADC_CLOCK, LOG_GUEST_ERROR,
                 "the ADC is silent: %s", why);
    } else if (s->adc_ok) {
        s->logged &= ~LOG_ADC_CLOCK;
    }
    if (s->adc_ok || s->dac_ok) {
        if (s->dac_ok && !(s->regs[R_DAC37] & DAC_EQBYPASS)) {
            log_once(s, LOG_EQ, LOG_UNIMP, "the DAC equaliser is not "
                     "modelled (its coefficients' format is not "
                     "documented): it passes the signal unchanged");
        }
        if (s->adc_ok && !(s->regs[R_ADC1C] & ADC_EQBYPASS)) {
            log_once(s, LOG_EQ, LOG_UNIMP, "the ADC equaliser is not "
                     "modelled (its coefficients' format is not "
                     "documented): it passes the signal unchanged");
        }
        if (s->adc_ok && (s->regs[R_ADC1C] & ADC_HPF)) {
            log_once(s, LOG_HPF, LOG_UNIMP, "the ADC's high-pass filter "
                     "is not modelled");
        }
        if (s->dac_ok && s->regs[R_DAC33]) {
            log_once(s, LOG_OFFSET, LOG_UNIMP, "DAC_OFFSET is not "
                     "modelled");
        }
    }
}

/* ---- The chip state machine ---- */

static void csm_enter(ES8311State *s, uint8_t state)
{
    uint8_t b = s->regs[R_SYS0B], c = s->regs[R_SYS0C];

    s->csm = state;
    switch (state) {
    case CSM_S1:
        s->csm_frames = pwrup_frames[0][MIN(PWRUP_A(b), 31)];
        break;
    case CSM_S2:
        s->csm_frames = pwrup_frames[1][MIN(PWRUP_B(b, c), 31)];
        break;
    case CSM_S3:
        s->csm_frames = pwrup_frames[2][MIN(PWRUP_C(c), 31)];
        break;
    default:
        s->csm_frames = 0;
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * CSM_ON and RST_DIG: the machine powers down at once, and powers up
 * through its stages, or at once with SEQ_DIS.
 */
static void csm_update(ES8311State *s)
{
    uint8_t r0 = s->regs[R_RESET];

    if (!(r0 & CSM_ON) || (r0 & RST_DIG)) {
        csm_enter(s, CSM_S0);
    } else if (s->csm == CSM_S0) {
        csm_enter(s, (r0 & SEQ_DIS) ? CSM_S6 : CSM_S1);
    }
}

/* A frame goes by: the power-up stages count LRCK periods. */
static void csm_frame(ES8311State *s)
{
    if (s->csm < CSM_S1 || s->csm > CSM_S3) {
        return;
    }
    if (s->csm_frames > 1) {
        s->csm_frames--;
        return;
    }
    csm_enter(s, s->csm == CSM_S3 ? CSM_S6 : s->csm + 1);
    converters_update(s);
}

/* ---- Gains ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A gain one period on: towards the volume (@vol_q) at the ramp rate
 * (2^(@rate + 1) periods a 0.25 dB step, or at once for @rate 0), or,
 * with the level control on (@ctl), the way its last window decided, up
 * to the volume, a step a ramp period (a window with no ramp).
 */
static void gain_step(ES8311Gain *g, int vol_q, unsigned rate, bool ctl)
{
    uint32_t ramp_len = rate ? 1u << (rate + 1) : 0;

    if (!ctl) {
        g->win = 0;
        g->peak = 0;
        g->dir = 0;
        if (!ramp_len) {
            g->q = vol_q;
        } else if (g->q != vol_q && ++g->ramp >= ramp_len) {
            g->ramp = 0;
            g->q += g->q < vol_q ? 1 : -1;
        }
        return;
    }
    if (ramp_len && g->dir && ++g->ramp >= ramp_len) {
        g->ramp = 0;
        g->q += g->dir;
    }
    g->q = MIN(MAX(g->q, 0), vol_q);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The level control's detector, the period's output @v: at the end of a
 * window of 2^(@winsize + 1) periods, a peak over MAXLEVEL turns the gain
 * down and one under MINLEVEL up (@lvl_reg's levels).
 */
static void gain_detect(ES8311Gain *g, bool ctl, uint8_t lvl_reg,
                        unsigned winsize, unsigned rate, int32_t v)
{
    if (!ctl) {
        return;
    }
    g->peak = MAX(g->peak, (uint32_t)ABS(v));
    if (++g->win < (1u << (winsize + 1))) {
        return;
    }
    if (g->peak > level24[MAXLEVEL(lvl_reg)]) {
        g->dir = -1;
    } else if (g->peak < level24[MINLEVEL(lvl_reg)]) {
        g->dir = 1;
    } else {
        g->dir = 0;
    }
    g->win = 0;
    g->peak = 0;
    if (!rate) {
        g->q += g->dir;
    }
}

/* ---- The converters ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The ADC takes the frame's analog input: through the PGA, the 24-bit
 * conversion, ADC_SCALE, ADC_INV, the volume or ALC and automute, for
 * ASDOUT in the next frame.
 */
static void adc_convert(ES8311State *s)
{
    int64_t m = mic_take(s);
    uint8_t r14 = s->regs[R_SYS14], r16 = s->regs[R_ADC16];
    uint8_t r18 = s->regs[R_ADC18];
    int32_t v;

    if (!s->adc_ok) {
        s->adc_sample = 0;
        return;
    }
    if (r14 & DMIC_ON) {
        log_once(s, LOG_DMIC, LOG_UNIMP, "DMIC_ON: no digital microphone "
                 "is modelled on MIC1P; the ADC records silence");
        m = 0;
    } else if (LINSEL(r14) != 1) {
        if (LINSEL(r14)) {
            log_once(s, LOG_LINSEL, LOG_GUEST_ERROR, "LINSEL %u selects "
                     "MIC2, which the ES8311 does not have",
                     LINSEL(r14));
        }
        m = 0;
    }
    /* The PGA, 3 dB a step to 30 dB, clipping at the ADC's full scale */
    m = (m * gain_q24[GAIN_0DB + 12 * MIN(PGAGAIN(r14), 10)]) >> 24;
    v = sat24(m);
    v = gain_apply(v, GAIN_0DB + 24 * ADC_SCALE(r16));
    if (r16 & ADC_INV) {
        v = sat24(-(int64_t)v);
    }
    /* Automute looks at the signal before the volume. */
    if (r18 & AUTOMUTE_EN) {
        uint8_t r1a = s->regs[R_ADC1A];

        if ((uint32_t)ABS(v) > noise_gate24[AUTOMUTE_NG(r1a)]) {
            s->automute_frames = 0;
            s->automuted = false;
        } else if (!s->automuted &&
                   ++s->automute_frames >= 2048u * (AUTOMUTE_WS(r1a) + 1)) {
            s->automuted = true;
        }
    } else {
        s->automute_frames = 0;
        s->automuted = false;
    }
    gain_step(&s->adc_gain, 2 * s->regs[R_ADC17],
              RAMPRATE(s->regs[R_ADC15]), r18 & ALC_EN);
    v = gain_apply(v, s->adc_gain.q);
    gain_detect(&s->adc_gain, r18 & ALC_EN, s->regs[R_ADC19], WINSIZE(r18),
                RAMPRATE(s->regs[R_ADC15]), v);
    if (s->automuted) {
        v = gain_apply(v, GAIN_0DB -
                          16 * AUTOMUTE_VOL(s->regs[R_ADC1B]));
    }
    s->adc_sample = v;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The DAC converts a sample: the word DSDIN carried (MSB-aligned), or the
 * ADC's output, through the volume or DRC, DAC_INV and the mutes, to the
 * speaker while the board's output path is on.
 */
static void dac_convert(ES8311State *s, uint32_t word)
{
    uint8_t r31 = s->regs[R_DAC31], r34 = s->regs[R_DAC34];
    int32_t v;

    if (s->regs[R_GPIO44] & ADC2DAC_SEL) {
        v = s->adc_sample;
    } else {
        v = (int32_t)word >> 8;
    }
    if (s->regs[R_SDPIN] & SDP_MUTE) {
        v = 0;
    }
    gain_step(&s->dac_gain, 2 * s->regs[R_DAC32],
              RAMPRATE(s->regs[R_DAC37]), r34 & DRC_EN);
    v = gain_apply(v, s->dac_gain.q);
    gain_detect(&s->dac_gain, r34 & DRC_EN, s->regs[R_DAC35], WINSIZE(r34),
                RAMPRATE(s->regs[R_DAC37]), v);
    if (r31 & DAC_INV) {
        v = sat24(-(int64_t)v);
    }
    if ((r31 & (DAC_DSMMUTE | DAC_DEMMUTE)) || !s->dac_ok) {
        v = 0;
    }
    speaker_put(s, s->out_path ? v : 0);
}

/* ---- The serial port ---- */

static unsigned sdp_wl(uint8_t reg)
{
    static const uint8_t wl[8] = { 24, 20, 18, 16, 32, 0, 0, 0 };

    return wl[SDP_WL(reg)];
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * LRCK as this cycle's edge samples it, for one direction of the port
 * and its register: whether a word is due, after how many cycles and for
 * which channel. Returns true where a frame starts (its left word due).
 */
static bool sdp_sync(ES8311Sdp *d, uint8_t reg, bool lrck)
{
    unsigned fmt = SDP_FMT(reg);
    bool lrp = reg & SDP_LRP, rising;

    if (d->synced && lrck == d->lrck) {
        return false;
    }
    /* The first cycle seen counts as a change to its level. */
    rising = lrck && (!d->synced || !d->lrck);
    d->synced = true;
    d->lrck = lrck;
    switch (fmt) {
    case FMT_I2S:
    case FMT_LJ:
        d->pending = fmt == FMT_I2S ? 1 : 0;
        d->pending_chan = (lrck ^ (fmt == FMT_LJ) ^ lrp) ? RIGHT : LEFT;
        return d->pending_chan == LEFT;
    case FMT_DSP:
        if (!rising) {
            return false;
        }
        d->pending = lrp ? 0 : 1;
        d->pending_chan = LEFT;
        return true;
    default:
        d->pending = -1;
        return rising;
    }
}

static void sdp_start_word(ES8311Sdp *d, uint8_t chan, uint32_t word)
{
    d->active = true;
    d->chan = chan;
    d->bits = 0;
    d->word = word;
}

/* A word in from DSDIN is complete, or cut short by the next. */
static void rx_word_done(ES8311State *s)
{
    ES8311Sdp *d = &s->sdp_in;
    uint8_t chan = (s->regs[R_SDPIN] & SDP_IN_SEL) ? RIGHT : LEFT;

    d->active = false;
    s->rx_word[d->chan] = d->word;
    if (d->chan == chan) {
        dac_convert(s, d->word);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A rising edge of the port's bit clock: DSDIN's bit for the word in.
 */
static void sdp_rise(ES8311State *s, bool lrck, bool bit)
{
    ES8311Sdp *d = &s->sdp_in;
    uint8_t reg = s->regs[R_SDPIN];
    unsigned wl = sdp_wl(reg);

    sdp_sync(d, reg, lrck);
    if (!wl || SDP_FMT(reg) == 2) {
        log_once(s, LOG_SDP_IN, LOG_GUEST_ERROR, "SDP_IN 0x%02x: a "
                 "reserved format or word length, no data", reg);
        d->active = false;
        return;
    }
    if (d->pending == 0) {
        if (d->active) {
            rx_word_done(s);
        }
        sdp_start_word(d, d->pending_chan, 0);
    }
    if (d->pending >= 0) {
        d->pending--;
    }
    if (!d->active) {
        return;
    }
    d->word |= (uint32_t)bit << (31 - d->bits);
    if (++d->bits == wl) {
        uint8_t chan = d->chan;

        rx_word_done(s);
        if (SDP_FMT(reg) == FMT_DSP && chan == LEFT) {
            sdp_start_word(d, RIGHT, 0);
        }
    }
}

/* The words for ASDOUT's channels in the frame starting (ADCDAT_SEL). */
static void tx_words_load(ES8311State *s)
{
    uint32_t adc = (uint32_t)s->adc_sample << 8;
    uint32_t dacl = s->rx_word[LEFT], dacr = s->rx_word[RIGHT];
    uint32_t w[8][2] = {
        { adc, adc }, { adc, 0 }, { 0, adc }, { 0, 0 },
        { dacl, adc }, { adc, dacr }, { dacl, dacr }, { 0, 0 },
    };
    unsigned sel = ADCDAT_SEL(s->regs[R_GPIO44]);

    s->tx_word[LEFT] = w[sel][LEFT];
    s->tx_word[RIGHT] = w[sel][RIGHT];
}

static void lrck_lost(ES8311State *s);

/* Whether two periods agree within a thousandth */
static bool periods_agree(int64_t a, int64_t b)
{
    return a * 1000 <= b * 1001 && b * 1000 <= a * 1001;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * LRCK's rate from the frames' starts: taken from the first two periods
 * that agree, then over runs of 64 agreeing periods, to the hertz. A
 * period that disagrees with the one before (a clock started, stopped or
 * retimed, a stray edge) starts the run afresh.
 */
static uint64_t lrck_measure(ES8311State *s, int64_t now)
{
    int64_t period = now - s->frame_ns;
    uint64_t mhz = 0, m;

    if (s->frame_ns == INT64_MIN || period <= 0) {
        s->frame_ns = now;
        s->last_period = 0;
        return 0;
    }
    s->frame_ns = now;
    if (!periods_agree(period, s->last_period)) {
        s->last_period = period;
        s->avg_start_ns = now;
        s->avg_frames = 0;
        return 0;
    }
    s->last_period = period;
    s->avg_frames++;
    if (!s->lrck_mhz) {
        mhz = 1000000000000ull / (uint64_t)period;
    }
    if (s->avg_frames == 64) {
        m = 64000000000000ull / (uint64_t)(now - s->avg_start_ns);
        if ((m + 500) / 1000 != (s->lrck_mhz + 500) / 1000) {
            mhz = m;
        }
        s->avg_frames = 0;
        s->avg_start_ns = now;
    }
    return mhz;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A frame starts at @now: LRCK's rate, the state machine's count, the
 * ADC's sample for ASDOUT and its next.
 */
static void frame_start(ES8311State *s, int64_t now)
{
    uint64_t mhz = lrck_measure(s, now);

    s->cycles_per_frame = s->frame_cycles;
    s->frame_cycles = 0;
    if (mhz) {
        s->lrck_mhz = mhz;
        es8311_voices(s, (uint32_t)((mhz + 500) / 1000));
        converters_update(s);
    }
    /* LRCK is taken as lost after four periods (and 20 ms) without one. */
    timer_mod_ns(&s->lost_timer, now + 20 * SCALE_MS +
                 (s->lrck_mhz ? 4000000000000ull / s->lrck_mhz : 0));
    csm_frame(s);
    tx_words_load(s);
    adc_convert(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A falling edge of the port's bit clock: ASDOUT's bit for the cycle.
 */
static bool sdp_fall(ES8311State *s, bool lrck, int64_t now)
{
    ES8311Sdp *d = &s->sdp_out;
    uint8_t reg = s->regs[R_SDPOUT];
    unsigned wl = sdp_wl(reg);
    bool bit = false;

    if (sdp_sync(d, reg, lrck)) {
        frame_start(s, now);
    }
    s->frame_cycles++;
    if (!wl || SDP_FMT(reg) == 2) {
        log_once(s, LOG_SDP_OUT, LOG_GUEST_ERROR, "SDP_OUT 0x%02x: a "
                 "reserved format or word length, no data", reg);
        d->active = false;
        return false;
    }
    if (d->pending == 0) {
        sdp_start_word(d, d->pending_chan, s->tx_word[d->pending_chan]);
    }
    if (d->pending >= 0) {
        d->pending--;
    }
    if (d->active) {
        bit = d->word >> 31;
        d->word <<= 1;
        if (++d->bits == wl) {
            d->active = false;
            if (SDP_FMT(reg) == FMT_DSP && d->chan == LEFT) {
                sdp_start_word(d, RIGHT, s->tx_word[RIGHT]);
            }
        }
    }
    return bit && !(reg & SDP_MUTE);
}

/* Whether the serial port runs: out of reset, its bit clock on. */
static bool sdp_running(ES8311State *s)
{
    return !(s->regs[R_RESET] & RST_DIG) && (s->regs[R_CLK1] & BCLK_ON);
}

static void sdp_reset(ES8311State *s)
{
    memset(&s->sdp_in, 0, sizeof(s->sdp_in));
    memset(&s->sdp_out, 0, sizeof(s->sdp_out));
    s->sdp_in.pending = -1;
    s->sdp_out.pending = -1;
}

/* ---- Runs from a clock source that does not toggle the pins ---- */

/* [spec:nuos:req:emu.esp32p4.i2s] */
void es8311_sclk_run(ES8311State *s, uint32_t ncycles, int64_t start_ns,
                     int64_t end_ns, bool inverted)
{
    s->in_run = true;
    s->run_cycles = MAX(ncycles, 1);
    s->run_cycle = 0;
    s->run_start_ns = start_ns;
    s->run_end_ns = end_ns;
    s->run_inv = inverted ^ !!(s->regs[R_CLK6] & BCLK_INV);
}

/* The middle of the run's current cycle */
static int64_t run_cycle_ns(ES8311State *s)
{
    return s->run_start_ns + (s->run_end_ns - s->run_start_ns) *
           (2 * (int64_t)s->run_cycle + 1) / (2 * (int64_t)s->run_cycles);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A cycle's falling edge on the pin. With the clock inverted the codec's
 * own falling edge is the pin's rising edge in mid-cycle, as the master's
 * receiver samples: its bit reaches the receiver a cycle late.
 */
bool es8311_sclk_fall(ES8311State *s, bool lrck, bool *driven)
{
    bool bit = false, out;

    *driven = !(s->regs[R_CLK7] & TRI_ADCDAT);
    if (!s->in_run || s->master || !sdp_running(s)) {
        return false;
    }
    bit = sdp_fall(s, lrck, run_cycle_ns(s));
    out = s->run_inv ? s->run_last_out : bit;
    s->run_last_out = bit;
    return out;
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
void es8311_sclk_rise(ES8311State *s, bool lrck, bool dsdin)
{
    if (s->in_run && !s->master && sdp_running(s)) {
        sdp_rise(s, lrck, dsdin);
    }
    s->run_cycle++;
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
void es8311_sclk_end(ES8311State *s)
{
    if (!s->in_run) {
        return;
    }
    s->in_run = false;
    drive_asdout(s, s->run_last_out);
}

bool es8311_is_master(ES8311State *s)
{
    return s->master;
}

/* ---- The pins: a slave clocked edge by edge ---- */

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * SCLK's pin changes: a slave's port runs on its edges, inverted with
 * BCLK_INV, sampling the pins as they are at the edge.
 */
static void es8311_sclk_in(void *opaque, int n, int level)
{
    ES8311State *s = opaque;
    bool rising;

    if (s->pin_sclk == !!level) {
        return;
    }
    s->pin_sclk = level;
    if (s->master || s->in_run || !sdp_running(s)) {
        return;
    }
    rising = !!level ^ !!(s->regs[R_CLK6] & BCLK_INV);
    if (rising) {
        sdp_rise(s, s->pin_lrck, s->pin_dsdin);
    } else {
        drive_asdout(s, sdp_fall(s, s->pin_lrck, esp32_line_time()));
    }
}

static void es8311_lrck_in(void *opaque, int n, int level)
{
    ES8311State *s = opaque;

    s->pin_lrck = level;
}

static void es8311_dsdin_in(void *opaque, int n, int level)
{
    ES8311State *s = opaque;

    s->pin_dsdin = level;
}

static void es8311_out_path(void *opaque, int n, int level)
{
    ES8311State *s = opaque;

    s->out_path = level;
}

/* ---- Master mode ---- */

/* The frame in SCLK periods, the nearest whole number to its division. */
static uint32_t mst_frame_cycles(ES8311State *s)
{
    unsigned div = bclk_div(s), l = lrck_div(s);

    if (l % div) {
        log_once(s, LOG_MASTER_LRCK, LOG_GUEST_ERROR, "DIV_LRCK + 1 (%u) "
                 "is no whole number of SCLK periods (%u MCLK cycles)",
                 l, div);
    }
    return MAX((l + div / 2) / div, 2);
}

/* LRCK in cycle @cycle of a frame of @frame cycles */
static bool mst_lrck_at(ES8311State *s, uint32_t cycle, uint32_t frame)
{
    uint8_t reg = s->regs[R_SDPOUT];
    bool left_high;

    if (lrck_div(s) == 1) {
        return true;
    }
    if (SDP_FMT(reg) == FMT_DSP) {
        return cycle == 0;
    }
    left_high = (SDP_FMT(reg) == FMT_LJ) ^ !!(reg & SDP_LRP);
    return (cycle < frame / 2) ? left_high : !left_high;
}

/* With BCLK_CON, SCLK runs only in the cycles that carry a word's bits. */
static bool mst_cycle_clocked(ES8311State *s, uint32_t cycle, uint32_t frame)
{
    uint8_t reg = s->regs[R_SDPOUT];
    unsigned wl = sdp_wl(reg), delay;

    if (!(s->regs[R_CLK6] & BCLK_CON)) {
        return true;
    }
    if (SDP_FMT(reg) == FMT_DSP) {
        delay = (reg & SDP_LRP) ? 0 : 1;
        return cycle >= delay && cycle < delay + 2 * wl;
    }
    delay = SDP_FMT(reg) == FMT_I2S ? 1 : 0;
    cycle %= MAX(frame / 2, 1);
    return cycle >= delay && cycle < delay + wl;
}

static bool mst_running(ES8311State *s)
{
    uint8_t r0 = s->regs[R_RESET], c1 = s->regs[R_CLK1];

    return s->master && !(r0 & (RST_MST | RST_CMG | RST_DIG)) &&
           (c1 & BCLK_ON) && (c1 & MCLK_ON) && !(c1 & MCLK_SEL) &&
           clock_get(s->mclk) != 0;
}

/*
 * The time of edge @n from the anchor: the edges are SCLK's, every half
 * period, each DIV_BCLK's divider half-periods of MCLK apart.
 */
static int64_t mst_edge_time(ES8311State *s, uint64_t n)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, n * s->mst_div, s->mst_mclk_period);
    return s->mst_anchor_ns + (int64_t)((hi << 31) | (lo >> 33));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Edge @n of the master's SCLK, at its time: even, a falling edge, where
 * LRCK changes and ASDOUT takes its bit; odd, a rising edge, where DSDIN
 * is sampled. BCLK_INV inverts the pin, BCLK_CON keeps it low between
 * words, TRI_BLRCK releases both pins.
 */
static void mst_edge(ES8311State *s, uint64_t n)
{
    uint32_t frame = s->mst_frame, cycle = (n / 2) % frame;
    bool tri = s->regs[R_CLK7] & TRI_BLRCK;
    bool inv = s->regs[R_CLK6] & BCLK_INV;
    bool lrck = mst_lrck_at(s, cycle, frame);

    if (!(n & 1)) {
        pin_drive(s->lrck_out, s->lrck_oe, &s->drv_lrck, &s->oe_lrck, lrck,
                  !tri);
        pin_drive(s->sclk_out, s->sclk_oe, &s->drv_sclk, &s->oe_sclk, inv,
                  !tri);
        drive_asdout(s, sdp_fall(s, lrck, esp32_line_time()));
    } else {
        pin_drive(s->sclk_out, s->sclk_oe, &s->drv_sclk, &s->oe_sclk,
                  mst_cycle_clocked(s, cycle, frame) ^ inv, !tri);
        sdp_rise(s, lrck, s->pin_dsdin);
    }
}

typedef struct MstEdge {
    ES8311State *s;
    uint64_t n;
} MstEdge;

static void mst_edge_at(void *opaque)
{
    MstEdge *e = opaque;

    mst_edge(e->s, e->n);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Run the master's edges due by @now, each at its own time, and arm the
 * timer for the end of the frame after. On the host's clock an edge
 * LINE_MAX_LAG_NS behind the present (hw/core/line-time.h), which the
 * host cannot follow, restarts the clocks' timing at the present, the
 * frames between not run.
 */
static void mst_catch_up(ES8311State *s, int64_t now)
{
    uint64_t per_frame = 2 * (uint64_t)s->mst_frame;

    for (;;) {
        int64_t t = mst_edge_time(s, s->mst_edge);

        if (t > now) {
            break;
        }
        if (line_lags(t)) {
            log_once(s, LOG_LAG, LOG_UNIMP, "the host is too slow for the "
                     "master clocks: their timing restarts at the present");
            s->mst_anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->mst_edge = 0;
            continue;
        }
        esp32_line_run_at(t, mst_edge_at,
                          &(MstEdge) { .s = s, .n = s->mst_edge });
        s->mst_edge++;
    }
    /* Rebase the count on a frame's start well before it could overflow. */
    if (s->mst_edge >= (1ull << 24) && s->mst_edge % per_frame == 0) {
        s->mst_anchor_ns = mst_edge_time(s, s->mst_edge);
        s->mst_edge = 0;
    }
    timer_mod_ns(&s->mst_timer,
                 mst_edge_time(s, (s->mst_edge / per_frame + 1) *
                                  per_frame));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * The timer takes the frame it was set for, up to the next frame's start,
 * however late it runs, and no later one: re-armed for the next in the
 * past, it runs again among every other device's timers in order of
 * time, so that on the host's clock a master the host held up catches up
 * frame by frame in step with the audio backend's timer, and the buffers
 * between them neither run dry nor overflow. Under icount it runs at that
 * time.
 */
static void es8311_mst_timer(void *opaque)
{
    ES8311State *s = opaque;
    uint64_t per_frame = 2 * (uint64_t)s->mst_frame;

    mst_catch_up(s, mst_edge_time(s, (s->mst_edge / per_frame + 1) *
                                     per_frame));
}

/* Software samples the pads: the master's edges up to the present. */
static void es8311_line_sync(Notifier *n, void *data)
{
    ES8311State *s = container_of(n, ES8311State, line_source);

    if (timer_pending(&s->mst_timer)) {
        mst_catch_up(s, esp32_line_time());
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * Master mode starts or stops with MSC, its resets, its clocks and MCLK.
 * A change of its dividers or of MCLK takes effect at the next edge,
 * which starts a frame.
 */
static void master_update(ES8311State *s)
{
    bool tri = s->regs[R_CLK7] & TRI_BLRCK;
    uint32_t div, frame;
    uint64_t mclk;

    s->master = s->regs[R_RESET] & MSC;
    if (s->master && (s->regs[R_CLK1] & MCLK_SEL)) {
        log_once(s, LOG_MASTER, LOG_GUEST_ERROR, "master mode with "
                 "MCLK_SEL: SCLK cannot clock itself");
    }
    if (s->master && !s->line_source_added) {
        s->line_source.notify = es8311_line_sync;
        esp32_line_add_source(&s->line_source);
        s->line_source_added = true;
    }
    if (!s->master) {
        timer_del(&s->mst_timer);
        pin_drive(s->sclk_out, s->sclk_oe, &s->drv_sclk, &s->oe_sclk,
                  false, false);
        pin_drive(s->lrck_out, s->lrck_oe, &s->drv_lrck, &s->oe_lrck,
                  false, false);
        return;
    }
    if (!mst_running(s)) {
        /* The pins hold their levels, released with TRI_BLRCK. */
        timer_del(&s->mst_timer);
        pin_drive(s->sclk_out, s->sclk_oe, &s->drv_sclk, &s->oe_sclk,
                  s->drv_sclk, !tri);
        pin_drive(s->lrck_out, s->lrck_oe, &s->drv_lrck, &s->oe_lrck,
                  s->drv_lrck, !tri);
        return;
    }
    div = bclk_div(s);
    frame = mst_frame_cycles(s);
    mclk = clock_get(s->mclk);
    if (timer_pending(&s->mst_timer) && div == s->mst_div &&
        frame == s->mst_frame && mclk == s->mst_mclk_period) {
        pin_drive(s->sclk_out, s->sclk_oe, &s->drv_sclk, &s->oe_sclk,
                  s->drv_sclk, !tri);
        pin_drive(s->lrck_out, s->lrck_oe, &s->drv_lrck, &s->oe_lrck,
                  s->drv_lrck, !tri);
        return;
    }
    s->mst_anchor_ns = timer_pending(&s->mst_timer) ?
                       mst_edge_time(s, s->mst_edge) : esp32_line_time();
    s->mst_edge = 0;
    s->mst_div = div;
    s->mst_frame = frame;
    s->mst_mclk_period = mclk;
    mst_catch_up(s, esp32_line_time());
}

static void es8311_mclk_changed(void *opaque, ClockEvent event)
{
    ES8311State *s = opaque;

    es8311_update(s);
}

/* ---- Registers ---- */

static void lrck_lost(ES8311State *s)
{
    s->lrck_mhz = 0;
    s->frame_ns = INT64_MIN;
    s->last_period = 0;
    s->avg_frames = 0;
    s->cycles_per_frame = 0;
    es8311_voices(s, 0);
    converters_update(s);
}

static void es8311_lost_timer(void *opaque)
{
    lrck_lost(opaque);
}

/* The DAC's or the ADC's digital block in reset: its gain starts low. */
static void gains_reset(ES8311State *s)
{
    uint8_t r0 = s->regs[R_RESET];

    if (r0 & (RST_DIG | RST_DAC_DIG)) {
        memset(&s->dac_gain, 0, sizeof(s->dac_gain));
    }
    if (r0 & (RST_DIG | RST_ADC_DIG)) {
        memset(&s->adc_gain, 0, sizeof(s->adc_gain));
        s->automute_frames = 0;
        s->automuted = false;
        s->adc_sample = 0;
    }
}

/* Everything that follows from the registers and the clocks. */
static void es8311_update(ES8311State *s)
{
    csm_update(s);
    gains_reset(s);
    if (s->regs[R_RESET] & RST_DIG) {
        sdp_reset(s);
    }
    converters_update(s);
    qemu_set_irq(s->pull_up, !(s->regs[R_GP45] & PULLUP_SE));
    drive_asdout(s, s->drv_asdout);
    master_update(s);
}

static void regs_reset(ES8311State *s)
{
    uint8_t i2c = s->regs[R_I2C];

    memset(s->regs, 0, sizeof(s->regs));
    memcpy(s->regs, es8311_reset_regs, sizeof(es8311_reset_regs));
    s->regs[R_I2C] = i2c;
}

static bool reg_defined(uint8_t reg)
{
    return reg <= R_LAST || reg == R_I2C || reg >= R_FLAG;
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static uint8_t es8311_read_reg(ES8311State *s, uint8_t reg)
{
    switch (reg) {
    case R_FLAG:
        return (s->csm << 4) | (s->automuted ? FLAG_ADCAM : 0);
    case R_CHIP_ID1:
        return CHIP_ID1;
    case R_CHIP_ID2:
        return CHIP_ID2;
    case R_CHIP_VER:
        return CHIP_VER;
    default:
        if (!reg_defined(reg)) {
            qemu_log_mask(LOG_GUEST_ERROR, "es8311: read of undefined "
                          "register 0x%02x\n", reg);
            return 0;
        }
        return s->regs[reg];
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void es8311_write_reg(ES8311State *s, uint8_t reg, uint8_t val)
{
    if (!reg_defined(reg) || reg >= R_FLAG) {
        qemu_log_mask(LOG_GUEST_ERROR, "es8311: write of 0x%02x to %s "
                      "register 0x%02x\n", val,
                      reg_defined(reg) ? "read-only" : "undefined", reg);
        return;
    }
    if (reg == R_I2C) {
        s->regs[R_I2C] = val;
        if (val & INI_REG) {
            regs_reset(s);
        }
    } else if (s->regs[R_I2C] & INI_REG) {
        /* Held at their defaults */
        return;
    } else {
        s->regs[reg] = val;
    }
    es8311_update(s);
}

/* ---- The I2C control port ---- */

static int es8311_event(I2CSlave *i2c, enum i2c_event event)
{
    ES8311State *s = ES8311(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->ptr_next = true;
        break;
    default:
        break;
    }
    return 0;
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void es8311_access_catch_up(ES8311State *s);

static int es8311_send(I2CSlave *i2c, uint8_t data)
{
    ES8311State *s = ES8311(i2c);

    es8311_access_catch_up(s);
    if (s->ptr_next) {
        s->ptr = data;
        s->ptr_next = false;
    } else {
        es8311_write_reg(s, s->ptr, data);
    }
    return 0;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The codec answers its own address only: not the general call, which
 * its documents do not give it.
 */
static bool es8311_match(I2CSlave *candidate, uint8_t address,
                         bool broadcast, I2CNodeList *current_devs)
{
    I2CNode *node;

    if (candidate->address != address) {
        return false;
    }
    node = g_new(struct I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
/*
 * [spec:nuos:req:emu.esp32p4.i2s]
 * A register access on the I2C bus, at its bit's time: as the codec's
 * master clocks, its LRCK periods due by then come first, which their
 * timer, late without icount, has yet to take, so that FLAG's state
 * machine and automute read as they are then. Only a timer that has come
 * is run.
 */
static void es8311_access_catch_up(ES8311State *s)
{
    int64_t now = esp32_line_time();

    if (timer_pending(&s->mst_timer) &&
        timer_expire_time_ns(&s->mst_timer) <= now) {
        timer_del(&s->mst_timer);
        mst_catch_up(s, now);
    }
}

static uint8_t es8311_recv(I2CSlave *i2c)
{
    ES8311State *s = ES8311(i2c);

    es8311_access_catch_up(s);
    return es8311_read_reg(s, s->ptr);
}

/* ---- Device ---- */

static void es8311_reset_hold(Object *obj, ResetType type)
{
    ES8311State *s = ES8311(obj);

    s->regs[R_I2C] = 0;
    regs_reset(s);
    s->ptr = 0;
    s->ptr_next = false;
    s->in_run = false;
    s->run_last_out = false;
    sdp_reset(s);
    memset(s->rx_word, 0, sizeof(s->rx_word));
    memset(s->tx_word, 0, sizeof(s->tx_word));
    s->frame_cycles = 0;
    s->cycles_per_frame = 0;
    s->frame_ns = INT64_MIN;
    s->last_period = 0;
    s->avg_frames = 0;
    s->avg_start_ns = 0;
    s->lrck_mhz = 0;
    timer_del(&s->lost_timer);
    s->csm = CSM_S0;
    s->csm_frames = 0;
    s->dac_ok = false;
    s->adc_ok = false;
    memset(&s->dac_gain, 0, sizeof(s->dac_gain));
    memset(&s->adc_gain, 0, sizeof(s->adc_gain));
    s->adc_sample = 0;
    s->automute_frames = 0;
    s->automuted = false;
    s->master = false;
    timer_del(&s->mst_timer);
    s->mst_anchor_ns = 0;
    s->mst_edge = 0;
    s->mst_div = 0;
    s->mst_frame = 0;
    s->mst_mclk_period = 0;
    s->out_head = s->out_num = 0;
    s->in_head = s->in_num = 0;
    s->in_primed = false;
    s->logged = 0;
    if (s->voice_in && s->in_on) {
        audio_be_set_active_in(s->audio_be, s->voice_in, false);
    }
    s->in_on = false;
}

static void es8311_reset_exit(Object *obj, ResetType type)
{
    ES8311State *s = ES8311(obj);

    /* ASDOUT is driven low out of reset; SCLK and LRCK are inputs. */
    s->oe_asdout = false;
    s->oe_sclk = true;
    s->oe_lrck = true;
    pin_drive(s->sclk_out, s->sclk_oe, &s->drv_sclk, &s->oe_sclk, false,
              false);
    pin_drive(s->lrck_out, s->lrck_oe, &s->drv_lrck, &s->oe_lrck, false,
              false);
    s->drv_asdout = false;
    drive_asdout(s, false);
    qemu_set_irq(s->pull_up, 1);
    es8311_voices(s, 0);
}

static void es8311_realize(DeviceState *dev, Error **errp)
{
    ES8311State *s = ES8311(dev);

    I2C_SLAVE(dev)->address = ES8311_ADDR | s->ce;
}

static void es8311_init(Object *obj)
{
    ES8311State *s = ES8311(obj);
    DeviceState *dev = DEVICE(obj);

    s->mclk = qdev_init_clock_in(dev, "mclk", es8311_mclk_changed, s,
                                 ClockUpdate);
    qdev_init_gpio_in_named(dev, es8311_sclk_in, ES8311_SCLK, 1);
    qdev_init_gpio_in_named(dev, es8311_lrck_in, ES8311_LRCK, 1);
    qdev_init_gpio_in_named(dev, es8311_dsdin_in, ES8311_DSDIN, 1);
    qdev_init_gpio_in_named(dev, es8311_out_path, ES8311_OUT_PATH, 1);
    qdev_init_gpio_out_named(dev, &s->sclk_out, ES8311_SCLK_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->lrck_out, ES8311_LRCK_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->asdout_out, ES8311_ASDOUT, 1);
    qdev_init_gpio_out_named(dev, &s->sclk_oe, ES8311_SCLK_OE, 1);
    qdev_init_gpio_out_named(dev, &s->lrck_oe, ES8311_LRCK_OE, 1);
    qdev_init_gpio_out_named(dev, &s->asdout_oe, ES8311_ASDOUT_OE, 1);
    qdev_init_gpio_out_named(dev, &s->pull_up, ES8311_PULL_UP, 1);
    timer_init_ns(&s->lost_timer, QEMU_CLOCK_VIRTUAL, es8311_lost_timer, s);
    timer_init_ns(&s->mst_timer, QEMU_CLOCK_VIRTUAL, es8311_mst_timer, s);
    s->frame_ns = INT64_MIN;
}

static int es8311_post_load(void *opaque, int version_id)
{
    ES8311State *s = opaque;
    uint32_t freq = s->out_freq;
    bool in_on = s->in_on;

    s->out_freq = 0;
    s->in_on = false;
    if (freq) {
        es8311_voices(s, freq);
        if (!in_on) {
            es8311_voices(s, 0);
        }
    }
    return 0;
}

static const VMStateDescription vmstate_es8311_sdp = {
    .name = "es8311.sdp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(lrck, ES8311Sdp),
        VMSTATE_BOOL(synced, ES8311Sdp),
        VMSTATE_INT32(pending, ES8311Sdp),
        VMSTATE_UINT8(pending_chan, ES8311Sdp),
        VMSTATE_BOOL(active, ES8311Sdp),
        VMSTATE_UINT8(chan, ES8311Sdp),
        VMSTATE_UINT8(bits, ES8311Sdp),
        VMSTATE_UINT32(word, ES8311Sdp),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_es8311_gain = {
    .name = "es8311.gain",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT32(q, ES8311Gain),
        VMSTATE_UINT32(ramp, ES8311Gain),
        VMSTATE_UINT32(win, ES8311Gain),
        VMSTATE_UINT32(peak, ES8311Gain),
        VMSTATE_INT32(dir, ES8311Gain),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_es8311 = {
    .name = TYPE_ES8311,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = es8311_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, ES8311State),
        VMSTATE_CLOCK(mclk, ES8311State),
        VMSTATE_UINT8_ARRAY(regs, ES8311State, ES8311_NREGS),
        VMSTATE_UINT8(ptr, ES8311State),
        VMSTATE_BOOL(ptr_next, ES8311State),
        VMSTATE_BOOL(pin_sclk, ES8311State),
        VMSTATE_BOOL(pin_lrck, ES8311State),
        VMSTATE_BOOL(pin_dsdin, ES8311State),
        VMSTATE_BOOL(drv_sclk, ES8311State),
        VMSTATE_BOOL(drv_lrck, ES8311State),
        VMSTATE_BOOL(drv_asdout, ES8311State),
        VMSTATE_BOOL(oe_sclk, ES8311State),
        VMSTATE_BOOL(oe_lrck, ES8311State),
        VMSTATE_BOOL(oe_asdout, ES8311State),
        VMSTATE_BOOL(out_path, ES8311State),
        VMSTATE_BOOL(run_last_out, ES8311State),
        VMSTATE_STRUCT(sdp_in, ES8311State, 1, vmstate_es8311_sdp,
                       ES8311Sdp),
        VMSTATE_STRUCT(sdp_out, ES8311State, 1, vmstate_es8311_sdp,
                       ES8311Sdp),
        VMSTATE_UINT32_ARRAY(rx_word, ES8311State, 2),
        VMSTATE_UINT32_ARRAY(tx_word, ES8311State, 2),
        VMSTATE_UINT32(frame_cycles, ES8311State),
        VMSTATE_UINT32(cycles_per_frame, ES8311State),
        VMSTATE_INT64(frame_ns, ES8311State),
        VMSTATE_INT64(avg_start_ns, ES8311State),
        VMSTATE_INT64(last_period, ES8311State),
        VMSTATE_UINT32(avg_frames, ES8311State),
        VMSTATE_UINT64(lrck_mhz, ES8311State),
        VMSTATE_TIMER(lost_timer, ES8311State),
        VMSTATE_UINT8(csm, ES8311State),
        VMSTATE_UINT32(csm_frames, ES8311State),
        VMSTATE_BOOL(dac_ok, ES8311State),
        VMSTATE_BOOL(adc_ok, ES8311State),
        VMSTATE_STRUCT(dac_gain, ES8311State, 1, vmstate_es8311_gain,
                       ES8311Gain),
        VMSTATE_STRUCT(adc_gain, ES8311State, 1, vmstate_es8311_gain,
                       ES8311Gain),
        VMSTATE_INT32(adc_sample, ES8311State),
        VMSTATE_UINT32(automute_frames, ES8311State),
        VMSTATE_BOOL(automuted, ES8311State),
        VMSTATE_BOOL(master, ES8311State),
        VMSTATE_INT64(mst_anchor_ns, ES8311State),
        VMSTATE_UINT64(mst_edge, ES8311State),
        VMSTATE_UINT32(mst_div, ES8311State),
        VMSTATE_UINT32(mst_frame, ES8311State),
        VMSTATE_UINT64(mst_mclk_period, ES8311State),
        VMSTATE_TIMER(mst_timer, ES8311State),
        VMSTATE_INT32_ARRAY(out_buf, ES8311State, ES8311_AUDIO_FRAMES),
        VMSTATE_UINT32(out_head, ES8311State),
        VMSTATE_UINT32(out_num, ES8311State),
        VMSTATE_INT32_ARRAY(in_buf, ES8311State, ES8311_AUDIO_FRAMES),
        VMSTATE_UINT32(in_head, ES8311State),
        VMSTATE_UINT32(in_num, ES8311State),
        VMSTATE_BOOL(in_primed, ES8311State),
        VMSTATE_BOOL(in_on, ES8311State),
        VMSTATE_UINT32(out_freq, ES8311State),
        VMSTATE_UINT32(logged, ES8311State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property es8311_properties[] = {
    DEFINE_PROP_BOOL("ce", ES8311State, ce, false),
    DEFINE_PROP_UINT32("dvdd-mv", ES8311State, dvdd_mv, 3300),
    DEFINE_AUDIO_PROPERTIES(ES8311State, audio_be),
};

static void es8311_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Everest ES8311 mono audio codec";
    dc->realize = es8311_realize;
    dc->vmsd = &vmstate_es8311;
    rc->phases.hold = es8311_reset_hold;
    rc->phases.exit = es8311_reset_exit;
    sc->event = es8311_event;
    sc->send = es8311_send;
    sc->recv = es8311_recv;
    sc->match_and_add = es8311_match;
    device_class_set_props(dc, es8311_properties);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

/* [spec:nuos:req:emu.esp32p4.i2s] */
/* [spec:nuos:req:emu.esp32p4.i2c] */
static const TypeInfo es8311_types[] = {
    {
        .name = TYPE_ES8311,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(ES8311State),
        .instance_init = es8311_init,
        .class_init = es8311_class_init,
    },
};

DEFINE_TYPES(es8311_types)
