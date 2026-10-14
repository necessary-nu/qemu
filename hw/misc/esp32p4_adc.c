/*
 * ESP32-P4 SAR ADCs: the HP and LP ADC controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 65 (ADC Controller); ESP-IDF's adc_reg.h and lp_adc_reg.h for v3, the
 * authority for the registers, their reset values and access types,
 * regi2c_saradc.h for the SARs' analog registers, its hal adc_ll.h,
 * adc_hal.c and adc_oneshot_hal.c and esp_adc's calibration
 * (adc_cali_curve_fitting.c, curve_fitting_coefficients.c and
 * esp_efuse_rtc_calib.c for the P4), which show how software drives the
 * block.
 *
 * Two 12-bit SAR ADCs: SAR ADC1 converts GPIO16 to GPIO23 (channels 0 to
 * 7) and SAR ADC2 GPIO49 to GPIO54 (channels 0 to 5). Each SAR's input
 * multiplexer has eight positions; ADC2's channels are at positions 2 to
 * 7, as ESP-IDF's adc_ll.h selects them in SAR2_EN_PAD, SAR2_ATTEN and
 * the pattern tables (the TRM numbers them 0 to 5, which ESP-IDF on
 * silicon contradicts), and the filters and monitors name position p of
 * ADC u as 8u + p (TRM: 0 to 7 for ADC1, 10 to 15 for ADC2). Each SAR is
 * driven by one of two controllers:
 *
 * - The HP controller (registers at 0x500DE000, HPADC_CLK): a timer
 *   counting TIMER_TARGET cycles of the SAR clock, HPADC_CLK /
 *   (SAR_CLK_DIV + 1), raises a sampling request, while TIMER_EN and
 *   TIMER_SEL are set, or, with START_FORCE, a rising START does; so does
 *   the ETM task ADC_TASK_START0, which starts the timer as TIMER_EN
 *   does, and ADC_TASK_STOP0 stops it (events ADC_EVT_STARTED0 and
 *   _STOPPED0). WORK_MODE gives the request to the FSM SAR_SEL names (0),
 *   to both (1), or to FSM1 and FSM2 in turn, FSM1 first (2). An FSM
 *   converts the entry of its pattern table its pointer is at (channel in
 *   bits 5:2, attenuation in bits 1:0; entry 0 in bits 23:18 of TAB1),
 *   then moves the pointer on, from 0 to SARn_PATT_LEN and round again;
 *   SARn_PATT_P_CLEAR holds it at 0. A conversion takes 25 cycles of the
 *   SAR clock (TRM 65.5.3) and, while the FSM powers the SAR
 *   (XPD_SARn_FORCE not forcing it), XPD_WAIT and RSTB_WAIT cycles of
 *   HPADC_CLK before; an FSM still converting drops a request. Its
 *   result, inverted by SARn_INV, goes through the filter that names its
 *   channel (filter 0 if both do): out = ((k - 1) out + in) / k rounded,
 *   k = 2^FILTER_FACTOR, 0 for none, starting from the first sample after
 *   FILTER_RESET; then past the threshold monitors (THRESx_EN, and
 *   THRESx_CHANNEL or, with THRES_ALL_EN, every channel), which raise
 *   THRESx_HIGH_INT and ADC_EVT_EQ_ABOVE_THRESHx above THRESx_HIGH, and
 *   THRESx_LOW_INT and ADC_EVT_EQ_BELOW_THRESHx below THRESx_LOW; then
 *   into SARn_DATA_STATUS and, with
 *   APB_ADC_TRANS, to GDMA-AHB (peripheral 8) as a 32-bit word: the data
 *   in bits 11:0, the multiplexer position in bits 16:13 and, with
 *   DATA_SAR_SEL, the SAR in bit 17 (ESP-IDF's adc_digi_output_data_t),
 *   the APB_ADC_EOF_NUM-th since the last closing the DMA frame. Each
 *   sample raises SARn_DONE_INT and ADC_EVT_CONV_CMPLT0, and with
 *   MEAS_NUM_LIMIT the timer stops after MAX_MEAS_NUM. APB_ADC_RESET_FSM
 *   restarts the pointers and counters and drops the conversions under
 *   way. The model takes the TRM's SAR_CLK_DIV as the SAR clock's
 *   divider; the clock tree's HPADCn_SARCLK dividers
 *   (HP_SYS_CLKRST_ADC_SARn_CLK_DIV_NUM), which ESP-IDF leaves alone,
 *   time nothing. So ESP-IDF's sample rate, HPADC_CLK / 2 / TIMER_TARGET
 *   at SAR_CLK_DIV 1, holds, and its fastest, 83333 Hz, leaves time for
 *   each 25-cycle conversion.
 *
 * - The LP controller (registers at 0x50127000, LPADC_CLK): a rising
 *   SARn_MEAS_START_SAR, with MEASn_START_FORCE, or, without it, the ETM
 *   task ADC_TASK_SAMPLEn, converts the position SARn_EN_PAD selects (with
 *   EN_PAD_FORCE; no position without it, the P4 having no ULP to select
 *   one), at its SARn_ATTEN attenuation, in 25 cycles of LPADCn_SARCLK,
 *   the clock tree's LP SAR clock divided by SARn_CLK_DIV. The result,
 *   inverted by SARn_DATA_INV, lands in MEASn_DATA_SAR with
 *   MEASn_DONE_SAR, which the start clears, raising
 *   COCPU_SARADCn_INT (while SARn_INT_EN) and ADC_EVT_CONV_CMPLT0. With
 *   no position selected the SAR converts its input as calibration
 *   leaves it, at channel 0's attenuation, as ESP-IDF's self-calibration
 *   relies on. Its automatic monitor (ADCn_HW_READ_EN_I) samples the same
 *   way every ADCn_HW_READ_RATE_I cycles of LPADC_CLK and, with
 *   SARn_WAKEUP_EN, raises COCPU_SARADCn_WAKE_INT when the data is above
 *   SARn_WAKEUP_TH_HIGH or below _TH_LOW (WAKEUP_MODE 0) or rose or fell
 *   by more than them since the last sample (1), recording which in
 *   WAKEUP_OVER_UPPER_TH; its samples land in MEASn_DATA_SAR without
 *   DONE_SAR or the done interrupt. A start the controller cannot serve,
 *   its SAR given to the HP controller, raises COCPU_SARADCn_ERROR_INT.
 *
 * The SARs: SAR1_DIG_FORCE gives ADC1 to the HP controller, else the LP
 * controller has it. ADC2 is shared by arbitration: the LP controller may
 * always convert, the HP controller unless SAR2_RTC_FORCE gives ADC2 to
 * the LP side alone (ESP-IDF clears it for DMA and for the LP core); a
 * request finding the SAR converting for the other controller waits, the
 * LP controller's first. A SAR its controller's power control forces off
 * (2 in XPD_SARn_FORCE or FORCE_XPD_SARn) converts nothing: the reader
 * takes 0. A request for a SAR the controller does not have is refused
 * and logged.
 *
 * The analog conversion follows the chip's calibration, so that ESP-IDF's
 * calibration gives back the pad's voltage: the eFuses record, per SAR
 * and attenuation, the SAR's offset (ADCn_AVE_INITCODE_ATTENa, plus 1400)
 * and the code it gives at a reference voltage (2300 or 2350 +
 * ADCn_HI_DOUT_ATTENa at 600, 800, 1200 and 2300 mV), and per channel its
 * offset from the average at 0 dB, scaled by 4 - atten
 * (ADCn_CHc_ATTEN0_INITCODE_DIFF); and ESP-IDF's curve fitting removes a
 * reading error linear in the voltage. The model's code for V mV is
 * ESP-IDF's curve inverted: ((V - K0) / (1 - K1)) * HI_DOUT / Vref, plus
 * the channel's offset, plus the SAR's offset less the initial code
 * software has set in the analog block's ANA_SENSOR registers (0 and 1
 * for ADC1, 3 and 4 for ADC2), clamped to 0 to 4095. ESP-IDF sets the
 * eFuse's offset, or finds it, with ADCn_ENCAL_GND connecting the SAR to
 * ground, as the smallest initial code reading 0. The attenuation's
 * reference (ADCn_DREF) and sample cycles are taken as ESP-IDF sets them.
 *
 * The pads' voltages are the properties adc1-ch<n>-mv and adc2-ch<n>-mv; a
 * pad a digital driver holds, the chip's own or the board's, is at 3.3 V
 * or 0 V.
 *
 * Not modelled, and logged: the legacy paths to I2S (TIMER_SEL clear,
 * DATA_TO_I2S); the power detector's controls (SAR2_PWDET_*), as the P4
 * has no radio. Kept as written: the arbiter's priorities and forces
 * (ARB_CTRL), WAIT_ARB_CYCLE, CALI, the ECO registers,
 * SARn_EN_PAD_FORCE_ENABLE and WAKEUP_SEL, which select nothing the
 * model shows; the
 * controller's FSM state, MEAS2_CNTL_STATE, whose encoding is not
 * published, reads 0. ADC_EVT_RESULT_DONE0 is not described and not
 * raised.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_adc.h"
#include "hw/misc/esp32p4_ana.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "migration/vmstate.h"

/* HP controller registers */
#define A_CTRL                  0x000
#define A_CTRL2                 0x004
#define A_FILTER_CTRL1          0x008
#define A_FSM_WAIT              0x00c
#define A_SAR_PATT_TAB(u, n)    (0x018 + 0x10 * (u) + 4 * (n))
#define A_ARB_CTRL              0x038
#define A_FILTER_CTRL0          0x03c
#define A_SAR1_DATA_STATUS      0x040
#define A_THRES_CTRLX(x)        (0x044 + 4 * (x))
#define A_THRES_CTRL            0x04c
#define A_INT_ENA               0x050
#define A_INT_RAW               0x054
#define A_INT_ST                0x058
#define A_INT_CLR               0x05c
#define A_DMA_CONF              0x060
#define A_SAR2_DATA_STATUS      0x064
#define A_CALI                  0x068
#define A_RND_ECO_LOW           0x06c
#define A_RND_ECO_HIGH          0x070
#define A_RND_ECO_CS            0x074
#define A_CTRL_DATE             0x3fc

/* CTRL */
#define CTRL_START_FORCE        BIT(0)
#define CTRL_START              BIT(1)
#define CTRL_WORK_MODE(v)       extract32((v), 2, 2)
#define CTRL_SAR_SEL            BIT(4)
#define CTRL_SAR_CLK_GATED      BIT(5)
#define CTRL_SAR_CLK_DIV(v)     extract32((v), 6, 8)
#define CTRL_PATT_LEN(v, u)     extract32((v), 14 + 4 * (u), 4)
#define CTRL_PATT_P_CLEAR(u)    BIT(22 + (u))
#define CTRL_DATA_SAR_SEL       BIT(24)
#define CTRL_DATA_TO_I2S        BIT(25)
#define CTRL_XPD_SAR_FORCE(v, u) extract32((v), 26 + 2 * (u), 2)
/* CTRL2 */
#define CTRL2_MEAS_NUM_LIMIT    BIT(0)
#define CTRL2_MAX_MEAS_NUM(v)   extract32((v), 1, 8)
#define CTRL2_SAR_INV(u)        BIT(9 + (u))
#define CTRL2_TIMER_SEL         BIT(11)
#define CTRL2_TIMER_TARGET(v)   extract32((v), 12, 12)
#define CTRL2_TIMER_EN          BIT(24)
/* FILTER_CTRL1, FILTER_CTRL0 */
#define FILTER_FACTOR(v, x)     extract32((v), 29 - 3 * (x), 3)
#define FILTER_CHANNEL(v, x)    extract32((v), 19 - 5 * (x), 5)
#define FILTER_RESET            BIT(31)
/* FSM_WAIT */
#define FSM_XPD_WAIT(v)         extract32((v), 0, 8)
#define FSM_RSTB_WAIT(v)        extract32((v), 8, 8)
/* THRESx_CTRL, THRES_CTRL */
#define THRES_CHANNEL(v)        extract32((v), 0, 5)
#define THRES_HIGH(v)           extract32((v), 5, 13)
#define THRES_LOW(v)            extract32((v), 18, 13)
#define THRES_ALL_EN            BIT(27)
#define THRES_EN(x)             BIT(31 - (x))
/* INT_* */
#define INT_THRES_LOW(x)        BIT(27 - (x))
#define INT_THRES_HIGH(x)       BIT(29 - (x))
#define INT_DONE(u)             BIT(31 - (u))
#define HP_INT_ALL              0xfc000000u
/* DMA_CONF */
#define DMA_EOF_NUM(v)          extract32((v), 0, 16)
#define DMA_RESET_FSM           BIT(30)
#define DMA_TRANS               BIT(31)
/* The data status registers hold bits 16:0 of the sample's DMA word. */
#define DATA_STATUS_MASK        0x1ffffu

/* LP controller registers */
#define A_READER_CTRL(u)        ((u) ? 0x024 : 0x000)
#define A_MEAS_CTRL2(u)         ((u) ? 0x030 : 0x00c)
#define A_MEAS_MUX(u)           ((u) ? 0x034 : 0x010)
#define A_ATTEN(u)              ((u) ? 0x038 : 0x014)
#define A_MEAS2_CTRL1           0x02c
#define A_FORCE_WPD_SAR         0x03c
#define A_LP_INT_RAW            0x048
#define A_LP_INT_ENA            0x04c
#define A_LP_INT_ST             0x050
#define A_LP_INT_CLR            0x054
#define A_LP_INT_ENA_W1TS       0x058
#define A_LP_INT_ENA_W1TC       0x05c
#define A_WAKEUP(u)             (0x060 + 4 * (u))
#define A_WAKEUP_SEL            0x068
#define A_HW_WAKEUP(u)          (0x06c + 4 * (u))

/* READERn_CTRL */
#define READER_CLK_DIV(v)       extract32((v), 0, 8)
#define READER_DATA_INV(u)      BIT((u) ? 29 : 28)
#define READER_INT_EN(u)        BIT((u) ? 30 : 29)
/* MEASn_CTRL2 */
#define MEAS_DATA_MASK          0xffffu
#define MEAS_DONE               BIT(16)
#define MEAS_START_SAR          BIT(17)
#define MEAS_START_FORCE        BIT(18)
#define MEAS_EN_PAD(v)          extract32((v), 19, 12)
#define MEAS_EN_PAD_FORCE       BIT(31)
/* MEASn_MUX: ADC1's SAR1_DIG_FORCE, ADC2's SAR2_RTC_FORCE */
#define MUX_FORCE               BIT(31)
#define MUX_PWDET_CCT_MASK      (7u << 28)
/* MEAS2_CTRL1 */
#define MEAS2_PWDET_CAL_EN      BIT(3)
#define MEAS2_PKDET_CAL_EN      BIT(4)
/* FORCE_WPD_SAR */
#define FORCE_XPD_SAR(v, u)     extract32((v), 2 * (u), 2)
/* LP INT_* */
#define LP_INT_DONE(u)          BIT(u)
#define LP_INT_ERROR(u)         BIT(2 + (u))
#define LP_INT_WAKE(u)          BIT(4 + (u))
#define LP_INT_ALL              0x3fu
/* WAKEUPn */
#define WAKEUP_TH_LOW(v)        extract32((v), 0, 12)
#define WAKEUP_TH_HIGH(v)       extract32((v), 14, 12)
#define WAKEUP_OVER_UPPER_TH    BIT(29)
#define WAKEUP_EN               BIT(30)
#define WAKEUP_MODE             BIT(31)
/* SARn_HW_WAKEUP */
#define HW_READ_EN              BIT(0)
#define HW_READ_RATE(v)         extract32((v), 1, 16)

/* A power control's settings: 2'b11 forced on, 2'b10 forced off. */
#define XPD_FORCED              2
#define XPD_FORCED_OFF          2

/* The SAR's analog registers in the ANA_SENSOR block (regi2c_saradc.h). */
#define ANA_INITIAL_CODE_LOW(u) ((u) ? 3 : 0)
#define ANA_INITIAL_CODE_HIGH(u) ((u) ? 4 : 1)
#define ANA_ENCAL               7
#define ANA_ENCAL_GND(u)        BIT((u) ? 7 : 5)

/* A conversion: 25 SAR clock cycles (TRM 65.5.3). */
#define SAR_CYCLES              25
#define CODE_MAX                4095
/* A pad's digital high level, VDD_IO. */
#define PAD_VDD_MV              3300

#define HP_REG(s, a)            ((s)->hp_regs[(a) / 4])
#define LP_REG(s, a)            ((s)->lp_regs[(a) / 4])

typedef struct AdcReg {
    uint32_t reset;
    uint32_t rw;
    bool valid;
} AdcReg;

#define REG(a, r, w)    [(a) / 4] = { .reset = (r), .rw = (w), .valid = true }

static const AdcReg hp_reg_info[ESP32P4_ADC_HP_NREGS] = {
    REG(A_CTRL, 0x403fc120, 0xffffffff),
    REG(A_CTRL2, 0x0000a1fe, 0x01ffffff),
    REG(A_FILTER_CTRL1, 0, 0xfc000000),
    REG(A_FSM_WAIT, 0x00ff0808, 0x00ffffff),
    REG(A_SAR_PATT_TAB(0, 0), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(0, 1), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(0, 2), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(0, 3), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(1, 0), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(1, 1), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(1, 2), 0, 0x00ffffff),
    REG(A_SAR_PATT_TAB(1, 3), 0, 0x00ffffff),
    REG(A_ARB_CTRL, 0x00000900, 0x00001ffc),
    REG(A_FILTER_CTRL0, 0x006b4000, 0x80ffc000),
    REG(A_SAR1_DATA_STATUS, 0, 0),
    REG(A_THRES_CTRLX(0), 0x0003ffed, 0x7fffffff),
    REG(A_THRES_CTRLX(1), 0x0003ffed, 0x7fffffff),
    REG(A_THRES_CTRL, 0, 0xf8000000),
    REG(A_INT_ENA, 0, HP_INT_ALL),
    REG(A_INT_RAW, 0, 0),
    REG(A_INT_ST, 0, 0),
    REG(A_INT_CLR, 0, 0),
    REG(A_DMA_CONF, 0x000000ff, 0xc000ffff),
    REG(A_SAR2_DATA_STATUS, 0, 0),
    REG(A_CALI, 0x00008000, 0x0001ffff),
    REG(A_RND_ECO_LOW, 0, 0xffffffff),
    REG(A_RND_ECO_HIGH, 0xffffffff, 0xffffffff),
    REG(A_RND_ECO_CS, 0, 0x00000001),
    REG(A_CTRL_DATE, 0x02212260, 0xffffffff),
};

static const AdcReg lp_reg_info[ESP32P4_ADC_LP_NREGS] = {
    REG(A_READER_CTRL(0), 0x20000002, 0xf00000ff),
    REG(A_MEAS_CTRL2(0), 0, 0xfffe0000),
    REG(A_MEAS_MUX(0), 0, 0x80000000),
    REG(A_ATTEN(0), 0xffffffff, 0xffffffff),
    REG(A_READER_CTRL(1), 0x40010002, 0x780300ff),
    REG(A_MEAS2_CTRL1, 0, 0x00000038),
    REG(A_MEAS_CTRL2(1), 0, 0xfffe0000),
    REG(A_MEAS_MUX(1), 0, 0xf0000000),
    REG(A_ATTEN(1), 0xffffffff, 0xffffffff),
    REG(A_FORCE_WPD_SAR, 0, 0x0000000f),
    REG(A_LP_INT_RAW, 0, 0),
    REG(A_LP_INT_ENA, 0, LP_INT_ALL),
    REG(A_LP_INT_ST, 0, 0),
    REG(A_LP_INT_CLR, 0, 0),
    REG(A_LP_INT_ENA_W1TS, 0, 0),
    REG(A_LP_INT_ENA_W1TC, 0, 0),
    REG(A_WAKEUP(0), 0x03ffc000, 0xc3ffcfff),
    REG(A_WAKEUP(1), 0x03ffc000, 0xc3ffcfff),
    REG(A_WAKEUP_SEL, 0, 0x00000001),
    REG(A_HW_WAKEUP(0), 0x000000c8, 0x0001ffff),
    REG(A_HW_WAKEUP(1), 0x000000c8, 0x0001ffff),
};

/* Each channel's pad. */
static const uint8_t adc_pad[ESP32P4_ADC_UNITS][ESP32P4_ADC1_CHANNELS] = {
    { 16, 17, 18, 19, 20, 21, 22, 23 },
    { 49, 50, 51, 52, 53, 54 },
};
static const unsigned adc_channels[ESP32P4_ADC_UNITS] = {
    ESP32P4_ADC1_CHANNELS, ESP32P4_ADC2_CHANNELS,
};
/* The multiplexer position of each SAR's channel 0. */
static const unsigned adc_pos0[ESP32P4_ADC_UNITS] = { 0, 2 };

/*
 * ESP-IDF's curve fitting for the P4 (calibration version 1): the reading
 * error, -K0 + K1 v mV, of each SAR and attenuation.
 */
static const double adc_err_k0[ESP32P4_ADC_UNITS][4] = {
    { 0.7170501832480995, 0.9960085535084866, 1.4711053224678996,
      2.8811493455181565 },
    { 0.4900967548489932, 0.7296214814536025, 1.0991620450220592,
      2.4421401024626730 },
};
static const double adc_err_k1[ESP32P4_ADC_UNITS][4] = {
    { 0.0010598497992115, 0.0015840076608145, 0.0001594266424857,
      0.0010082311568625 },
    { 0.0005037402667913, 0.0011021577596635, 0.0011623930881896,
      0.0009458501263393 },
};
/* The calibration's reference voltages, and the codes they give less HI_DOUT */
static const uint16_t adc_cal_mv[4] = { 600, 800, 1200, 2300 };
static const uint16_t adc_cal_code[4] = { 2300, 2300, 2350, 2350 };
#define ADC_INITCODE_BASE       1400

/* The eFuse fields (ESP-IDF's esp_efuse_table_v3.0.csv). */
#define EFUSE_BLOCK2            2
#define EFUSE_BLOCK10           10
#define EF_AVE_INITCODE(u, a)   (128 + 40 * (u) + 10 * (a))
#define EF_ADC1_HI_DOUT(a)      (208 + 10 * (a))
#define EF_ADC2_HI_DOUT(a)      (10 * (a))
#define EF_CH_DIFF(u, c)        (40 + 32 * (u) + 4 * (c))

static void adc_hp_update_irq(ESP32P4AdcState *s);
static void adc_lp_update_irq(ESP32P4AdcState *s);
static void adc_sar_free(ESP32P4AdcState *s, unsigned u, int64_t when);

static void adc_pulse(qemu_irq line, int64_t when)
{
    esp32_line_set_at(line, 1, when);
    esp32_line_set_at(line, 0, when);
}

/* ---- The analog side ---- */

static uint32_t adc_efuse_field(ESP32P4AdcState *s, unsigned block,
                                unsigned bit, unsigned width)
{
    uint64_t v = esp32p4_efuse_word(s->efuse, block, bit / 32);

    if (bit % 32 + width > 32) {
        v |= (uint64_t)esp32p4_efuse_word(s->efuse, block, bit / 32 + 1) << 32;
    }
    return extract64(v, bit % 32, width);
}

/* ESP-IDF's RTC_CALIB_GET_SIGNED_VAL: sign and magnitude. */
static int adc_sign_mag(uint32_t v, unsigned sign_bit)
{
    return v & BIT(sign_bit) ? -(int)(v & ~BIT(sign_bit)) : (int)v;
}

static int adc_channel_of(unsigned u, unsigned pos)
{
    int ch = (int)pos - (int)adc_pos0[u];

    return ch >= 0 && ch < adc_channels[u] ? ch : -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The voltage of channel @ch's pad: the supply or ground while a digital
 * driver holds it, else the voltage set from QEMU.
 */
static uint32_t adc_channel_mv(ESP32P4AdcState *s, unsigned u, unsigned ch)
{
    bool level;

    if (s->gpio && esp32p4_gpio_pad_driven(s->gpio, adc_pad[u][ch], &level)) {
        return level ? PAD_VDD_MV : 0;
    }
    return s->pad_mv[u * ESP32P4_ADC1_CHANNELS + ch];
}

uint32_t esp32p4_adc_pad_mv(ESP32P4AdcState *s, unsigned gpio)
{
    bool level;

    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        for (unsigned ch = 0; ch < adc_channels[u]; ch++) {
            if (adc_pad[u][ch] == gpio) {
                return adc_channel_mv(s, u, ch);
            }
        }
    }
    if (s->gpio && esp32p4_gpio_pad_driven(s->gpio, gpio, &level)) {
        return level ? PAD_VDD_MV : 0;
    }
    return 0;
}

static uint8_t adc_ana_reg(ESP32P4AdcState *s, unsigned reg)
{
    return s->ana->regs[ESP32P4_ANA_SLAVE_SAR][reg];
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The code SAR @u converts at multiplexer position @pos and attenuation
 * @atten: the input's voltage through the chip's calibrated transfer
 * function (see the top of the file), offset by the initial code the
 * analog registers hold. ADCn_ENCAL_GND connects the SAR to ground.
 */
static uint32_t adc_convert(ESP32P4AdcState *s, unsigned u, int pos,
                            unsigned atten)
{
    bool gnd = adc_ana_reg(s, ANA_ENCAL) & ANA_ENCAL_GND(u);
    int ch = pos >= 0 ? adc_channel_of(u, pos) : -1;
    unsigned hi_bit = u ? EF_ADC2_HI_DOUT(atten) : EF_ADC1_HI_DOUT(atten);
    unsigned hi_block = u ? EFUSE_BLOCK10 : EFUSE_BLOCK2;
    int digi = adc_cal_code[atten] +
        adc_sign_mag(adc_efuse_field(s, hi_block, hi_bit, 10), 9);
    int offset = ADC_INITCODE_BASE +
        adc_efuse_field(s, EFUSE_BLOCK2, EF_AVE_INITCODE(u, atten), 10);
    int initial = adc_ana_reg(s, ANA_INITIAL_CODE_LOW(u)) |
        (adc_ana_reg(s, ANA_INITIAL_CODE_HIGH(u)) & 0xf) << 8;
    double mv = 0, v1, ideal = 0;
    int64_t code;

    if (!gnd && ch >= 0) {
        mv = adc_channel_mv(s, u, ch);
    } else if (!gnd && pos >= 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ADC%u converts multiplexer "
                      "position %d, which has no pad\n", TYPE_ESP32P4_ADC,
                      u + 1, pos);
    } else if (!gnd) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ADC%u converts with no pad "
                      "selected\n", TYPE_ESP32P4_ADC, u + 1);
    }
    v1 = (mv - adc_err_k0[u][atten]) / (1.0 - adc_err_k1[u][atten]);
    if (v1 > 0 && digi > 0) {
        ideal = v1 * digi / adc_cal_mv[atten];
    }
    code = llround(ideal) + offset - initial;
    if (!gnd && ch >= 0) {
        code += adc_sign_mag(adc_efuse_field(s, EFUSE_BLOCK10,
                                             EF_CH_DIFF(u, ch), 4), 3) *
                (4 - (int)atten);
    }
    return MIN(MAX(code, 0), CODE_MAX);
}

/* ---- The HP controllers ---- */

static bool adc_hp_granted(ESP32P4AdcState *s, unsigned u)
{
    bool force = LP_REG(s, A_MEAS_MUX(u)) & MUX_FORCE;

    /* ADC1's SAR1_DIG_FORCE gives it to the HP side; ADC2's RTC_FORCE not. */
    return u ? !force : force;
}

static bool adc_lp_granted(ESP32P4AdcState *s, unsigned u)
{
    return u || !(LP_REG(s, A_MEAS_MUX(0)) & MUX_FORCE);
}

/* Whether the HP controllers' clock runs: HPADC_CLK, ungated, not reset. */
static bool adc_hp_clocked(ESP32P4AdcState *s)
{
    return clock_is_enabled(s->clk) && !s->hp_in_reset &&
           (HP_REG(s, A_CTRL) & CTRL_SAR_CLK_GATED);
}

static unsigned adc_sar_div(ESP32P4AdcState *s)
{
    return CTRL_SAR_CLK_DIV(HP_REG(s, A_CTRL)) + 1;
}

static void adc_hp_update_irq(ESP32P4AdcState *s)
{
    qemu_set_irq(s->hp_irq,
                 !!(HP_REG(s, A_INT_RAW) & HP_REG(s, A_INT_ENA)));
}

static void adc_hp_raise(ESP32P4AdcState *s, uint32_t bits)
{
    HP_REG(s, A_INT_RAW) |= bits;
    adc_hp_update_irq(s);
}

/* The sampling timer: armed while the controller samples by its timer. */
static bool adc_hp_timer_ticks(ESP32P4AdcState *s)
{
    uint32_t ctrl2 = HP_REG(s, A_CTRL2);

    return s->hp_running && adc_hp_clocked(s) &&
           (ctrl2 & CTRL2_TIMER_SEL) &&
           !(HP_REG(s, A_CTRL) & CTRL_START_FORCE) &&
           CTRL2_TIMER_TARGET(ctrl2);
}

static void adc_hp_timer_arm(ESP32P4AdcState *s, int64_t from)
{
    uint32_t target = CTRL2_TIMER_TARGET(HP_REG(s, A_CTRL2));

    if (!adc_hp_timer_ticks(s)) {
        esp32p4_clkwait_stop(&s->hp_timer);
        return;
    }
    esp32p4_clkwait_start(&s->hp_timer, from,
                          (uint64_t)target * adc_sar_div(s));
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_hp_set_running(ESP32P4AdcState *s, bool run, int64_t when)
{
    if (run == s->hp_running) {
        return;
    }
    s->hp_running = run;
    if (run) {
        s->meas_count = 0;
        s->hp_denied_logged = false;
        if (!(HP_REG(s, A_CTRL2) & CTRL2_TIMER_SEL)) {
            qemu_log_mask(LOG_UNIMP, "%s: sampling on I2S's WS (TIMER_SEL "
                          "clear) is not modelled\n", TYPE_ESP32P4_ADC);
        }
        adc_hp_timer_arm(s, when);
        adc_pulse(s->etm_event[ESP32P4_ADC_EVT_STARTED0], when);
    } else {
        esp32p4_clkwait_stop(&s->hp_timer);
        adc_pulse(s->etm_event[ESP32P4_ADC_EVT_STOPPED0], when);
    }
}

/* The cycles of HPADC_CLK an HP conversion on SAR @u takes. */
static uint64_t adc_hp_conv_cycles(ESP32P4AdcState *s, unsigned u)
{
    uint64_t cycles = (uint64_t)SAR_CYCLES * adc_sar_div(s);
    uint32_t fsm = HP_REG(s, A_FSM_WAIT);

    if (!(CTRL_XPD_SAR_FORCE(HP_REG(s, A_CTRL), u) & XPD_FORCED)) {
        cycles += FSM_XPD_WAIT(fsm) + FSM_RSTB_WAIT(fsm);
    }
    return cycles;
}

/* The pattern table entry FSM @u's pointer is at. */
static uint8_t adc_hp_entry(ESP32P4AdcState *s, unsigned u)
{
    unsigned ptr = s->unit[u].patt_ptr;
    uint32_t tab = HP_REG(s, A_SAR_PATT_TAB(u, ptr / 4));

    return extract32(tab, 18 - 6 * (ptr % 4), 6);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_hp_conv_begin(ESP32P4AdcState *s, unsigned u, int64_t when)
{
    ESP32P4AdcUnit *un = &s->unit[u];
    uint8_t entry = adc_hp_entry(s, u);
    unsigned pos = extract32(entry, 2, 4);
    unsigned atten = extract32(entry, 0, 2);

    un->owner = ESP32P4_ADC_BY_HP;
    un->hp_pending = false;
    un->hp_pos = pos;
    if (CTRL_XPD_SAR_FORCE(HP_REG(s, A_CTRL), u) == XPD_FORCED_OFF) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the HP controller samples ADC%u "
                      "while XPD_SAR%u_FORCE holds it powered down\n",
                      TYPE_ESP32P4_ADC, u + 1, u + 1);
        un->code = 0;
    } else if (pos >= ESP32P4_ADC_MUX_POS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ADC%u's pattern entry selects "
                      "channel %u, beyond its multiplexer\n",
                      TYPE_ESP32P4_ADC, u + 1, pos);
        un->code = 0;
    } else {
        un->code = adc_convert(s, u, pos, atten);
    }
    esp32p4_clkwait_start(&un->hp_conv, when, adc_hp_conv_cycles(s, u));
}

/* A sampling request reaches FSM @u. */
static void adc_hp_request(ESP32P4AdcState *s, unsigned u, int64_t when)
{
    ESP32P4AdcUnit *un = &s->unit[u];

    if (!adc_hp_granted(s, u)) {
        if (!s->hp_denied_logged) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: the HP controller samples "
                          "ADC%u, which %s gives to the LP controller\n",
                          TYPE_ESP32P4_ADC, u + 1,
                          u ? "SAR2_RTC_FORCE" : "SAR1_DIG_FORCE clear");
            s->hp_denied_logged = true;
        }
        return;
    }
    if (un->owner == ESP32P4_ADC_BY_HP || un->hp_pending) {
        /* The FSM is still converting: it does not take the request. */
        return;
    }
    if (un->owner == ESP32P4_ADC_BY_LP || un->lp_pending) {
        un->hp_pending = true;
        return;
    }
    adc_hp_conv_begin(s, u, when);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_hp_sample_request(ESP32P4AdcState *s, int64_t when)
{
    uint32_t ctrl = HP_REG(s, A_CTRL);

    switch (CTRL_WORK_MODE(ctrl)) {
    case 0:
        adc_hp_request(s, !!(ctrl & CTRL_SAR_SEL), when);
        break;
    case 1:
        adc_hp_request(s, 0, when);
        adc_hp_request(s, 1, when);
        break;
    case 2:
        adc_hp_request(s, s->alt_unit, when);
        s->alt_unit ^= 1;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: WORK_MODE 3 is reserved; the "
                      "request is dropped\n", TYPE_ESP32P4_ADC);
        break;
    }
}

static void adc_hp_timer_cb(void *opaque)
{
    ESP32P4AdcState *s = opaque;
    int64_t when = esp32p4_clkwait_expire(&s->hp_timer);

    adc_hp_timer_arm(s, when);
    adc_hp_sample_request(s, when);
}

/* The filter that takes samples of filter channel @id, or -1. */
static int adc_filter_of(ESP32P4AdcState *s, unsigned id)
{
    uint32_t c0 = HP_REG(s, A_FILTER_CTRL0), c1 = HP_REG(s, A_FILTER_CTRL1);

    for (int x = 0; x < ESP32P4_ADC_FILTERS; x++) {
        if (FILTER_FACTOR(c1, x) && FILTER_CHANNEL(c0, x) == id) {
            return x;
        }
    }
    return -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * Filter @x's output for input @in: ((k - 1) out + in) / k, rounded,
 * from the first sample after its reset.
 */
static uint32_t adc_filter(ESP32P4AdcState *s, int x, uint32_t in)
{
    unsigned factor = FILTER_FACTOR(HP_REG(s, A_FILTER_CTRL1), x);
    uint64_t k = 1ull << factor;

    if (HP_REG(s, A_FILTER_CTRL0) & FILTER_RESET) {
        return in;
    }
    if (!s->filter_valid[x]) {
        s->filter_val[x] = in;
        s->filter_valid[x] = true;
    } else {
        s->filter_val[x] = ((k - 1) * s->filter_val[x] + in + k / 2) / k;
    }
    return s->filter_val[x];
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_hp_monitor(ESP32P4AdcState *s, unsigned id, uint32_t data,
                           int64_t when)
{
    uint32_t ctrl = HP_REG(s, A_THRES_CTRL);

    for (int x = 0; x < ESP32P4_ADC_MONITORS; x++) {
        uint32_t t = HP_REG(s, A_THRES_CTRLX(x));

        if (!(ctrl & THRES_EN(x)) ||
            (!(ctrl & THRES_ALL_EN) && THRES_CHANNEL(t) != id)) {
            continue;
        }
        if (data > THRES_HIGH(t)) {
            adc_hp_raise(s, INT_THRES_HIGH(x));
            adc_pulse(s->etm_event[ESP32P4_ADC_EVT_EQ_ABOVE_THRESH0 + x],
                      when);
        }
        if (data < THRES_LOW(t)) {
            adc_hp_raise(s, INT_THRES_LOW(x));
            adc_pulse(s->etm_event[ESP32P4_ADC_EVT_EQ_BELOW_THRESH0 + x],
                      when);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * [spec:nuos:req:emu.esp32p4.dma]
 * FSM @u's sample is read: filter, monitors, data status, DMA, the
 * interrupts and the pointer.
 */
static void adc_hp_finish(ESP32P4AdcState *s, unsigned u, int64_t when)
{
    ESP32P4AdcUnit *un = &s->unit[u];
    uint32_t ctrl = HP_REG(s, A_CTRL);
    uint32_t dma = HP_REG(s, A_DMA_CONF);
    unsigned id = 8 * u + un->hp_pos;
    uint32_t data = un->code;
    uint32_t word;
    uint8_t buf[4];
    int x;

    if (HP_REG(s, A_CTRL2) & CTRL2_SAR_INV(u)) {
        data = ~data & CODE_MAX;
    }
    x = adc_filter_of(s, id);
    if (x >= 0) {
        data = adc_filter(s, x, data);
    }
    adc_hp_monitor(s, id, data, when);
    word = data | (un->hp_pos & 0xf) << 13;
    if (u && (ctrl & CTRL_DATA_SAR_SEL)) {
        word |= BIT(17);
    }
    HP_REG(s, u ? A_SAR2_DATA_STATUS : A_SAR1_DATA_STATUS) =
        word & DATA_STATUS_MASK;
    if (dma & DMA_TRANS) {
        bool eof;

        s->eof_count++;
        eof = DMA_EOF_NUM(dma) && s->eof_count >= DMA_EOF_NUM(dma);
        if (eof) {
            s->eof_count = 0;
        }
        stl_le_p(buf, word);
        if (esp32p4_gdma_rx_room(s->gdma, ESP32P4_GDMA_AHB_ADC) >= 4) {
            esp32p4_gdma_push(s->gdma, ESP32P4_GDMA_AHB_ADC, buf, 4,
                              eof ? ESP32P4_GDMA_EOF : 0);
        }
    }
    adc_hp_raise(s, INT_DONE(u));
    adc_pulse(s->etm_event[ESP32P4_ADC_EVT_CONV_CMPLT0], when);

    if (!(ctrl & CTRL_PATT_P_CLEAR(u))) {
        un->patt_ptr = (un->patt_ptr + 1) % (CTRL_PATT_LEN(ctrl, u) + 1);
    }
    s->meas_count++;
    if ((HP_REG(s, A_CTRL2) & CTRL2_MEAS_NUM_LIMIT) &&
        s->meas_count >= CTRL2_MAX_MEAS_NUM(HP_REG(s, A_CTRL2))) {
        adc_hp_set_running(s, false, when);
    }
}

static void adc_hp_conv_done(ESP32P4AdcState *s, unsigned u)
{
    int64_t when = esp32p4_clkwait_expire(&s->unit[u].hp_conv);

    s->unit[u].owner = ESP32P4_ADC_IDLE;
    adc_hp_finish(s, u, when);
    adc_sar_free(s, u, when);
}

static void adc_hp_conv0_cb(void *opaque)
{
    adc_hp_conv_done(opaque, 0);
}

static void adc_hp_conv1_cb(void *opaque)
{
    adc_hp_conv_done(opaque, 1);
}

/* Drop the HP side's conversions: a reset of its FSMs. */
static void adc_hp_abort(ESP32P4AdcState *s, int64_t when)
{
    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        ESP32P4AdcUnit *un = &s->unit[u];

        un->hp_pending = false;
        un->patt_ptr = 0;
        if (un->owner == ESP32P4_ADC_BY_HP) {
            esp32p4_clkwait_stop(&un->hp_conv);
            un->owner = ESP32P4_ADC_IDLE;
            adc_sar_free(s, u, when);
        }
    }
    s->alt_unit = 0;
    s->eof_count = 0;
    s->meas_count = 0;
}

/* ---- The LP controllers ---- */

static void adc_lp_update_irq(ESP32P4AdcState *s)
{
    qemu_set_irq(s->lp_irq,
                 !!(LP_REG(s, A_LP_INT_RAW) & LP_REG(s, A_LP_INT_ENA)));
}

static void adc_lp_raise(ESP32P4AdcState *s, uint32_t bits)
{
    LP_REG(s, A_LP_INT_RAW) |= bits;
    adc_lp_update_irq(s);
}

/* The position SARn_EN_PAD selects, or -1 for none. */
static int adc_lp_pos(ESP32P4AdcState *s, unsigned u)
{
    uint32_t meas = LP_REG(s, A_MEAS_CTRL2(u));
    uint32_t pads = MEAS_EN_PAD(meas);

    if (!(meas & MEAS_EN_PAD_FORCE) || !pads) {
        return -1;
    }
    if (pads & (pads - 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SAR%u_EN_PAD selects several "
                      "positions (0x%x); position %d is converted\n",
                      TYPE_ESP32P4_ADC, u + 1, pads, ctz32(pads));
    }
    return ctz32(pads);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_lp_conv_begin(ESP32P4AdcState *s, unsigned u, bool hw,
                              int64_t when)
{
    ESP32P4AdcUnit *un = &s->unit[u];
    int pos = adc_lp_pos(s, u);
    unsigned apos = pos >= 0 ? pos : adc_pos0[u];
    unsigned atten = extract32(LP_REG(s, A_ATTEN(u)), 2 * (apos % 16), 2);
    unsigned div = READER_CLK_DIV(LP_REG(s, A_READER_CTRL(u)));

    un->owner = ESP32P4_ADC_BY_LP;
    un->lp_pending = false;
    un->lp_hw = hw;
    if (FORCE_XPD_SAR(LP_REG(s, A_FORCE_WPD_SAR), u) == XPD_FORCED_OFF) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the LP controller samples ADC%u "
                      "while FORCE_XPD_SAR%u holds it powered down\n",
                      TYPE_ESP32P4_ADC, u + 1, u + 1);
        un->code = 0;
    } else if (pos >= ESP32P4_ADC_MUX_POS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: SAR%u_EN_PAD selects position "
                      "%d, beyond the multiplexer\n", TYPE_ESP32P4_ADC,
                      u + 1, pos);
        un->code = 0;
    } else {
        un->code = adc_convert(s, u, pos, atten);
    }
    esp32p4_clkwait_start(&un->lp_conv, when,
                          (uint64_t)SAR_CYCLES * MAX(div, 1));
}

/*
 * A start of LP controller @u, by software or the ETM (@hw false) or its
 * automatic monitor (@hw true).
 */
static void adc_lp_start(ESP32P4AdcState *s, unsigned u, bool hw,
                         int64_t when)
{
    ESP32P4AdcUnit *un = &s->unit[u];

    if (!adc_lp_granted(s, u)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the LP controller samples ADC1, "
                      "which SAR1_DIG_FORCE gives to the HP controller\n",
                      TYPE_ESP32P4_ADC);
        adc_lp_raise(s, LP_INT_ERROR(u));
        return;
    }
    if (!hw) {
        LP_REG(s, A_MEAS_CTRL2(u)) &= ~MEAS_DONE;
    }
    if (un->owner == ESP32P4_ADC_BY_LP || un->lp_pending) {
        /* The controller is converting: the start finds it busy. */
        return;
    }
    if (un->owner == ESP32P4_ADC_BY_HP) {
        un->lp_pending = true;
        un->lp_pending_hw = hw;
        return;
    }
    adc_lp_conv_begin(s, u, hw, when);
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The automatic monitor's sample: wake on leaving the thresholds' band,
 * or on a change beyond them.
 */
static void adc_lp_monitor(ESP32P4AdcState *s, unsigned u, uint32_t data)
{
    ESP32P4AdcUnit *un = &s->unit[u];
    uint32_t *w = &LP_REG(s, A_WAKEUP(u));
    uint32_t hi = WAKEUP_TH_HIGH(*w), lo = WAKEUP_TH_LOW(*w);
    bool upper = false, wake = false;

    if (*w & WAKEUP_MODE) {
        if (un->hw_prev_valid) {
            if (data > un->hw_prev && data - un->hw_prev > hi) {
                wake = upper = true;
            } else if (data < un->hw_prev && un->hw_prev - data > lo) {
                wake = true;
            }
        }
    } else if (data > hi) {
        wake = upper = true;
    } else if (data < lo) {
        wake = true;
    }
    un->hw_prev = data;
    un->hw_prev_valid = true;
    if (wake && (*w & WAKEUP_EN)) {
        *w = (*w & ~WAKEUP_OVER_UPPER_TH) | (upper ? WAKEUP_OVER_UPPER_TH : 0);
        adc_lp_raise(s, LP_INT_WAKE(u));
    }
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_lp_conv_done(ESP32P4AdcState *s, unsigned u)
{
    ESP32P4AdcUnit *un = &s->unit[u];
    int64_t when = esp32p4_clkwait_expire(&un->lp_conv);
    uint32_t data = un->code;
    uint32_t *meas = &LP_REG(s, A_MEAS_CTRL2(u));

    un->owner = ESP32P4_ADC_IDLE;
    if (LP_REG(s, A_READER_CTRL(u)) & READER_DATA_INV(u)) {
        data = ~data & CODE_MAX;
    }
    *meas = (*meas & ~MEAS_DATA_MASK) | data;
    if (un->lp_hw) {
        adc_lp_monitor(s, u, data);
    } else {
        *meas |= MEAS_DONE;
        if (LP_REG(s, A_READER_CTRL(u)) & READER_INT_EN(u)) {
            adc_lp_raise(s, LP_INT_DONE(u));
        }
    }
    adc_pulse(s->etm_event[ESP32P4_ADC_EVT_CONV_CMPLT0], when);
    adc_sar_free(s, u, when);
}

static void adc_lp_conv0_cb(void *opaque)
{
    adc_lp_conv_done(opaque, 0);
}

static void adc_lp_conv1_cb(void *opaque)
{
    adc_lp_conv_done(opaque, 1);
}

static void adc_hw_arm(ESP32P4AdcState *s, unsigned u, int64_t from)
{
    uint32_t hw = LP_REG(s, A_HW_WAKEUP(u));

    if (s->lp_in_reset || !(hw & HW_READ_EN) || !HW_READ_RATE(hw)) {
        esp32p4_clkwait_stop(&s->unit[u].hw_timer);
        return;
    }
    esp32p4_clkwait_start(&s->unit[u].hw_timer, from, HW_READ_RATE(hw));
}

static void adc_hw_timer(ESP32P4AdcState *s, unsigned u)
{
    int64_t when = esp32p4_clkwait_expire(&s->unit[u].hw_timer);

    adc_hw_arm(s, u, when);
    adc_lp_start(s, u, true, when);
}

static void adc_hw0_cb(void *opaque)
{
    adc_hw_timer(opaque, 0);
}

static void adc_hw1_cb(void *opaque)
{
    adc_hw_timer(opaque, 1);
}

/* SAR @u has finished: serve the waiting request, the LP side's first. */
static void adc_sar_free(ESP32P4AdcState *s, unsigned u, int64_t when)
{
    ESP32P4AdcUnit *un = &s->unit[u];

    if (un->owner != ESP32P4_ADC_IDLE) {
        return;
    }
    if (un->lp_pending) {
        adc_lp_conv_begin(s, u, un->lp_pending_hw, when);
    } else if (un->hp_pending) {
        adc_hp_conv_begin(s, u, when);
    }
}

static void adc_lp_abort(ESP32P4AdcState *s, int64_t when)
{
    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        ESP32P4AdcUnit *un = &s->unit[u];

        un->lp_pending = false;
        un->hw_prev_valid = false;
        esp32p4_clkwait_stop(&un->hw_timer);
        if (un->owner == ESP32P4_ADC_BY_LP) {
            esp32p4_clkwait_stop(&un->lp_conv);
            un->owner = ESP32P4_ADC_IDLE;
            adc_sar_free(s, u, when);
        }
    }
}

/* ---- Catching up ---- */

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * Take, in time order and each at its own time, the sampling timer's
 * ticks, the conversions' ends and the LP timers' ticks due by @now,
 * which without icount their QEMU timers can run well after: a register
 * access, and GDMA's, first find the controllers as they are then, a
 * oneshot's MEAS_DONE and data in place, the HP controller's INT_RAW
 * raised and the samples in GDMA's channel. An input (an ETM task, a
 * reset, a clock change) takes those due before it.
 */
static void adc_catch_up(ESP32P4AdcState *s, int64_t now)
{
    ESP32P4ClkWait *waits[] = {
        &s->hp_timer,
        &s->unit[0].hp_conv, &s->unit[1].hp_conv,
        &s->unit[0].lp_conv, &s->unit[1].lp_conv,
        &s->unit[0].hw_timer, &s->unit[1].hw_timer,
    };

    esp32p4_clkwaits_catch_up(waits, ARRAY_SIZE(waits), now);
}

/* ---- Registers ---- */

static uint64_t adc_hp_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4AdcState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    adc_catch_up(s, esp32_line_time());
    if (reg >= ESP32P4_ADC_HP_NREGS * 4 || !hp_reg_info[reg / 4].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved HP offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ADC, addr);
        return 0;
    }
    switch (reg) {
    case A_INT_ST:
        v = HP_REG(s, A_INT_RAW) & HP_REG(s, A_INT_ENA);
        break;
    case A_INT_CLR:
        v = 0;
        break;
    default:
        v = s->hp_regs[reg / 4];
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_hp_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    ESP32P4AdcState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    const AdcReg *info;
    uint32_t *r, old;

    adc_catch_up(s, now);
    if (reg >= ESP32P4_ADC_HP_NREGS * 4 || !hp_reg_info[reg / 4].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved HP offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ADC, addr);
        return;
    }
    if (s->hp_in_reset) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to HP offset 0x%"
                      HWADDR_PRIx " while RST_EN_ADC holds it in reset\n",
                      TYPE_ESP32P4_ADC, addr);
        return;
    }
    info = &hp_reg_info[reg / 4];
    r = &s->hp_regs[reg / 4];
    old = *r;

    switch (reg) {
    case A_INT_RAW:
    case A_INT_CLR:
        HP_REG(s, A_INT_RAW) &= ~(v & HP_INT_ALL);
        adc_hp_update_irq(s);
        return;
    case A_INT_ST:
    case A_SAR1_DATA_STATUS:
    case A_SAR2_DATA_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only HP offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ADC, addr);
        return;
    default:
        break;
    }
    *r = (*r & ~(info->rw & lanes)) | (v & info->rw);

    switch (reg) {
    case A_CTRL:
        if ((*r & CTRL_DATA_TO_I2S) && !(old & CTRL_DATA_TO_I2S)) {
            qemu_log_mask(LOG_UNIMP, "%s: the path to I2S (DATA_TO_I2S) is "
                          "not modelled\n", TYPE_ESP32P4_ADC);
        }
        for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
            if ((*r & CTRL_PATT_P_CLEAR(u)) ||
                s->unit[u].patt_ptr > CTRL_PATT_LEN(*r, u)) {
                s->unit[u].patt_ptr = 0;
            }
        }
        if ((*r & CTRL_START_FORCE) && (*r & CTRL_START) &&
            !(old & CTRL_START) && adc_hp_clocked(s)) {
            adc_hp_sample_request(s, now);
        }
        if ((*r ^ old) & (CTRL_START_FORCE | CTRL_SAR_CLK_GATED |
                          (0xffu << 6))) {
            esp32p4_clkwait_fold(&s->hp_timer, now);
            adc_hp_timer_arm(s, now);
        }
        break;
    case A_CTRL2:
        if ((*r & CTRL2_TIMER_EN) != (old & CTRL2_TIMER_EN)) {
            adc_hp_set_running(s, *r & CTRL2_TIMER_EN, now);
        } else if ((*r ^ old) & (CTRL2_TIMER_SEL | (0xfffu << 12))) {
            adc_hp_timer_arm(s, now);
        }
        break;
    case A_FILTER_CTRL0:
    case A_FILTER_CTRL1:
        for (int x = 0; x < ESP32P4_ADC_FILTERS; x++) {
            if ((HP_REG(s, A_FILTER_CTRL0) & FILTER_RESET) ||
                FILTER_FACTOR(HP_REG(s, A_FILTER_CTRL1), x) == 0) {
                s->filter_valid[x] = false;
            }
        }
        break;
    case A_INT_ENA:
        adc_hp_update_irq(s);
        break;
    case A_DMA_CONF:
        if (*r & DMA_RESET_FSM) {
            adc_hp_abort(s, now);
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps adc_hp_ops = {
    .read = adc_hp_read,
    .write = adc_hp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t adc_lp_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4AdcState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    adc_catch_up(s, esp32_line_time());
    if (reg >= ESP32P4_ADC_LP_NREGS * 4 || !lp_reg_info[reg / 4].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved LP offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ADC, addr);
        return 0;
    }
    switch (reg) {
    case A_LP_INT_ST:
        v = LP_REG(s, A_LP_INT_RAW) & LP_REG(s, A_LP_INT_ENA);
        break;
    case A_LP_INT_CLR:
    case A_LP_INT_ENA_W1TS:
    case A_LP_INT_ENA_W1TC:
        v = 0;
        break;
    default:
        v = s->lp_regs[reg / 4];
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_lp_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    ESP32P4AdcState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    const AdcReg *info;
    uint32_t *r, old;

    adc_catch_up(s, now);
    if (reg >= ESP32P4_ADC_LP_NREGS * 4 || !lp_reg_info[reg / 4].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved LP offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ADC, addr);
        return;
    }
    if (s->lp_in_reset) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to LP offset 0x%"
                      HWADDR_PRIx " while RST_EN_LP_ADC holds it in reset\n",
                      TYPE_ESP32P4_ADC, addr);
        return;
    }
    info = &lp_reg_info[reg / 4];
    r = &s->lp_regs[reg / 4];
    old = *r;

    switch (reg) {
    case A_LP_INT_RAW:
    case A_LP_INT_CLR:
        LP_REG(s, A_LP_INT_RAW) &= ~(v & LP_INT_ALL);
        adc_lp_update_irq(s);
        return;
    case A_LP_INT_ENA_W1TS:
        LP_REG(s, A_LP_INT_ENA) |= v & LP_INT_ALL;
        adc_lp_update_irq(s);
        return;
    case A_LP_INT_ENA_W1TC:
        LP_REG(s, A_LP_INT_ENA) &= ~(v & LP_INT_ALL);
        adc_lp_update_irq(s);
        return;
    case A_LP_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only LP offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ADC, addr);
        return;
    default:
        break;
    }
    *r = (*r & ~(info->rw & lanes)) | (v & info->rw);

    switch (reg) {
    case A_MEAS_CTRL2(0):
    case A_MEAS_CTRL2(1): {
        unsigned u = reg == A_MEAS_CTRL2(1);

        if ((*r & MEAS_START_SAR) && !(old & MEAS_START_SAR) &&
            (*r & MEAS_START_FORCE)) {
            adc_lp_start(s, u, false, now);
        }
        break;
    }
    case A_MEAS_MUX(1):
        if (*r & MUX_PWDET_CCT_MASK & ~old) {
            qemu_log_mask(LOG_UNIMP, "%s: the power detector (SAR2_PWDET_CCT)"
                          " is not modelled\n", TYPE_ESP32P4_ADC);
        }
        break;
    case A_MEAS2_CTRL1:
        if (*r & (MEAS2_PWDET_CAL_EN | MEAS2_PKDET_CAL_EN)) {
            qemu_log_mask(LOG_UNIMP, "%s: the power and peak detectors' "
                          "calibration is not modelled\n", TYPE_ESP32P4_ADC);
        }
        break;
    case A_LP_INT_ENA:
        adc_lp_update_irq(s);
        break;
    case A_WAKEUP(0):
    case A_WAKEUP(1):
        if ((*r ^ old) & WAKEUP_MODE) {
            s->unit[reg == A_WAKEUP(1)].hw_prev_valid = false;
        }
        break;
    case A_HW_WAKEUP(0):
    case A_HW_WAKEUP(1): {
        unsigned u = reg == A_HW_WAKEUP(1);

        if ((*r ^ old) & HW_READ_EN) {
            s->unit[u].hw_prev_valid = false;
            adc_hw_arm(s, u, now);
        } else if (*r != old) {
            esp32p4_clkwait_fold(&s->unit[u].hw_timer, now);
            adc_hw_arm(s, u, now);
        }
        break;
    }
    default:
        break;
    }
}

static const MemoryRegionOps adc_lp_ops = {
    .read = adc_lp_read,
    .write = adc_lp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ---- Inputs ---- */

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * [spec:nuos:req:emu.esp32p4.etm]
 * The ETM tasks, at their events' time.
 */
static void adc_etm_task(void *opaque, int n, int level)
{
    ESP32P4AdcState *s = opaque;
    int64_t when = esp32_line_time();

    if (!level) {
        return;
    }
    adc_catch_up(s, when - 1);
    switch (n) {
    case ESP32P4_ADC_TASK_SAMPLE0:
    case ESP32P4_ADC_TASK_SAMPLE1: {
        unsigned u = n - ESP32P4_ADC_TASK_SAMPLE0;

        if (!s->lp_in_reset &&
            !(LP_REG(s, A_MEAS_CTRL2(u)) & MEAS_START_FORCE)) {
            adc_lp_start(s, u, false, when);
        }
        break;
    }
    case ESP32P4_ADC_TASK_START0:
        if (adc_hp_clocked(s)) {
            adc_hp_set_running(s, true, when);
        }
        break;
    case ESP32P4_ADC_TASK_STOP0:
        adc_hp_set_running(s, false, when);
        break;
    }
}

static void adc_hp_reset_regs(ESP32P4AdcState *s)
{
    for (int i = 0; i < ESP32P4_ADC_HP_NREGS; i++) {
        s->hp_regs[i] = hp_reg_info[i].reset;
    }
    for (int x = 0; x < ESP32P4_ADC_FILTERS; x++) {
        s->filter_valid[x] = false;
        s->filter_val[x] = 0;
    }
    s->hp_running = false;
    esp32p4_clkwait_stop(&s->hp_timer);
    s->hp_denied_logged = false;
}

static void adc_lp_reset_regs(ESP32P4AdcState *s)
{
    for (int i = 0; i < ESP32P4_ADC_LP_NREGS; i++) {
        s->lp_regs[i] = lp_reg_info[i].reset;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * HP_SYS_CLKRST_RST_EN_ADC: the HP controllers reset, and held so.
 */
static void adc_set_hp_reset(void *opaque, int n, int level)
{
    ESP32P4AdcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->hp_in_reset == !!level) {
        return;
    }
    adc_catch_up(s, now - 1);
    adc_hp_abort(s, now);
    adc_hp_reset_regs(s);
    s->hp_in_reset = level;
    adc_hp_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * LPPERI_RST_EN_LP_ADC: the LP controllers reset, and held so.
 */
static void adc_set_lp_reset(void *opaque, int n, int level)
{
    ESP32P4AdcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->lp_in_reset == !!level) {
        return;
    }
    adc_catch_up(s, now - 1);
    adc_lp_abort(s, now);
    adc_lp_reset_regs(s);
    s->lp_in_reset = level;
    adc_lp_update_irq(s);
}

static void adc_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4AdcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        adc_catch_up(s, esp32_line_time() - 1);
        esp32p4_clkwait_fold(&s->hp_timer, now);
        for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
            esp32p4_clkwait_fold(&s->unit[u].hp_conv, now);
        }
        return;
    }
    esp32p4_clkwait_resched(&s->hp_timer);
    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        esp32p4_clkwait_resched(&s->unit[u].hp_conv);
    }
    if (!adc_hp_timer_ticks(s)) {
        esp32p4_clkwait_stop(&s->hp_timer);
    } else if (!esp32p4_clkwait_active(&s->hp_timer)) {
        adc_hp_timer_arm(s, now);
    }
}

static void adc_lp_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4AdcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        adc_catch_up(s, esp32_line_time() - 1);
    }
    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        if (event == ClockPreUpdate) {
            esp32p4_clkwait_fold(&s->unit[u].hw_timer, now);
        } else {
            esp32p4_clkwait_resched(&s->unit[u].hw_timer);
        }
    }
}

static void adc_lp_sar_clk_update(ESP32P4AdcState *s, unsigned u,
                                  ClockEvent event)
{
    if (event == ClockPreUpdate) {
        adc_catch_up(s, esp32_line_time() - 1);
        esp32p4_clkwait_fold(&s->unit[u].lp_conv,
                             qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    } else {
        esp32p4_clkwait_resched(&s->unit[u].lp_conv);
    }
}

static void adc_lp_sar1_update(void *opaque, ClockEvent event)
{
    adc_lp_sar_clk_update(opaque, 0, event);
}

static void adc_lp_sar2_update(void *opaque, ClockEvent event)
{
    adc_lp_sar_clk_update(opaque, 1, event);
}

/* The GDMA channel's readiness: the ADC does not wait for the DMA. */
static void adc_dma_ready(void *opaque)
{
}

static void adc_dma_sync(void *opaque, int64_t when)
{
    adc_catch_up(opaque, when);
}

static const ESP32P4GdmaPeriphOps adc_dma_ops = {
    .tx_ready = adc_dma_ready,
    .rx_ready = adc_dma_ready,
    .sync = adc_dma_sync,
};

/* ---- Analog inputs ---- */

static void adc_get_mv(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    uint32_t *mv = opaque;

    visit_type_uint32(v, name, mv, errp);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void adc_set_mv(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    ESP32P4AdcState *s = ESP32P4_ADC(obj);
    uint32_t *mv = opaque;
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value > PAD_VDD_MV) {
        error_setg(errp, "%s: %u mV is above the pads' 3.3 V supply", name,
                   value);
        return;
    }
    *mv = value;
    if (DEVICE(obj)->realized) {
        qemu_irq_pulse(s->pad_change);
    }
}

/* ---- Device ---- */

static void esp32p4_adc_reset_hold(Object *obj, ResetType type)
{
    ESP32P4AdcState *s = ESP32P4_ADC(obj);

    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        ESP32P4AdcUnit *un = &s->unit[u];

        esp32p4_clkwait_stop(&un->hp_conv);
        esp32p4_clkwait_stop(&un->lp_conv);
        esp32p4_clkwait_stop(&un->hw_timer);
        un->owner = ESP32P4_ADC_IDLE;
        un->code = 0;
        un->patt_ptr = 0;
        un->hp_pos = 0;
        un->hp_pending = false;
        un->lp_pending = false;
        un->lp_pending_hw = false;
        un->lp_hw = false;
        un->hw_prev = 0;
        un->hw_prev_valid = false;
    }
    adc_hp_reset_regs(s);
    adc_lp_reset_regs(s);
    s->meas_count = 0;
    s->alt_unit = 0;
    s->eof_count = 0;
}

static void esp32p4_adc_reset_exit(Object *obj, ResetType type)
{
    ESP32P4AdcState *s = ESP32P4_ADC(obj);

    adc_hp_update_irq(s);
    adc_lp_update_irq(s);
}

static void esp32p4_adc_init(Object *obj)
{
    ESP32P4AdcState *s = ESP32P4_ADC(obj);
    DeviceState *dev = DEVICE(obj);
    static QEMUTimerCB *const hp_conv_cb[ESP32P4_ADC_UNITS] = {
        adc_hp_conv0_cb, adc_hp_conv1_cb,
    };
    static QEMUTimerCB *const lp_conv_cb[ESP32P4_ADC_UNITS] = {
        adc_lp_conv0_cb, adc_lp_conv1_cb,
    };
    static QEMUTimerCB *const hw_cb[ESP32P4_ADC_UNITS] = {
        adc_hw0_cb, adc_hw1_cb,
    };

    memory_region_init_io(&s->hp_iomem, obj, &adc_hp_ops, s,
                          "esp32p4.adc", ESP32P4_ADC_SIZE);
    memory_region_init_io(&s->lp_iomem, obj, &adc_lp_ops, s,
                          "esp32p4.lp-adc", ESP32P4_ADC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->hp_iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->lp_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->hp_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->lp_irq);
    qdev_init_gpio_in_named(dev, adc_set_hp_reset, ESP32P4_ADC_HP_RESET, 1);
    qdev_init_gpio_in_named(dev, adc_set_lp_reset, ESP32P4_ADC_LP_RESET, 1);
    qdev_init_gpio_in_named(dev, adc_etm_task, ESP32P4_ADC_ETM_TASK,
                            ESP32P4_ADC_ETM_TASKS);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_ADC_ETM_EVENT,
                             ESP32P4_ADC_ETM_EVENTS);
    qdev_init_gpio_out_named(dev, &s->pad_change, ESP32P4_ADC_PAD_CHANGE, 1);

    s->clk = qdev_init_clock_in(dev, "adc", adc_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    s->lp_clk = qdev_init_clock_in(dev, "lp-adc", adc_lp_clk_update, s,
                                   ClockPreUpdate | ClockUpdate);
    s->lp_sar_clk[0] = qdev_init_clock_in(dev, "lp-adc-sar1",
                                          adc_lp_sar1_update, s,
                                          ClockPreUpdate | ClockUpdate);
    s->lp_sar_clk[1] = qdev_init_clock_in(dev, "lp-adc-sar2",
                                          adc_lp_sar2_update, s,
                                          ClockPreUpdate | ClockUpdate);
    esp32p4_clkwait_init(&s->hp_timer, s->clk, adc_hp_timer_cb, s);
    for (unsigned u = 0; u < ESP32P4_ADC_UNITS; u++) {
        esp32p4_clkwait_init(&s->unit[u].hp_conv, s->clk,
                             hp_conv_cb[u], s);
        esp32p4_clkwait_init(&s->unit[u].lp_conv, s->lp_sar_clk[u],
                             lp_conv_cb[u], s);
        esp32p4_clkwait_init(&s->unit[u].hw_timer, s->lp_clk,
                             hw_cb[u], s);
        for (unsigned ch = 0; ch < adc_channels[u]; ch++) {
            g_autofree char *name = g_strdup_printf("adc%u-ch%u-mv", u + 1,
                                                    ch);
            g_autofree char *desc = g_strdup_printf(
                "The voltage on ADC%u channel %u's pad, GPIO%u, in mV",
                u + 1, ch, adc_pad[u][ch]);

            object_property_add(obj, name, "uint32", adc_get_mv, adc_set_mv,
                                NULL,
                                &s->pad_mv[u * ESP32P4_ADC1_CHANNELS + ch]);
            object_property_set_description(obj, name, desc);
        }
    }
}

static void esp32p4_adc_realize(DeviceState *dev, Error **errp)
{
    ESP32P4AdcState *s = ESP32P4_ADC(dev);

    if (!s->gdma || !s->ana || !s->efuse) {
        error_setg(errp, "%s: the gdma, ana and efuse links must be set",
                   TYPE_ESP32P4_ADC);
        return;
    }
    esp32p4_gdma_attach(s->gdma, ESP32P4_GDMA_AHB_ADC, &adc_dma_ops, s);
}

static const VMStateDescription vmstate_esp32p4_adc_unit = {
    .name = "esp32p4-adc-unit",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(owner, ESP32P4AdcUnit),
        VMSTATE_UINT32(code, ESP32P4AdcUnit),
        VMSTATE_UINT8(patt_ptr, ESP32P4AdcUnit),
        VMSTATE_UINT8(hp_pos, ESP32P4AdcUnit),
        VMSTATE_BOOL(hp_pending, ESP32P4AdcUnit),
        VMSTATE_BOOL(lp_pending, ESP32P4AdcUnit),
        VMSTATE_BOOL(lp_pending_hw, ESP32P4AdcUnit),
        VMSTATE_BOOL(lp_hw, ESP32P4AdcUnit),
        VMSTATE_UINT32(hw_prev, ESP32P4AdcUnit),
        VMSTATE_BOOL(hw_prev_valid, ESP32P4AdcUnit),
        VMSTATE_ESP32P4_CLKWAIT(hp_conv, ESP32P4AdcUnit),
        VMSTATE_ESP32P4_CLKWAIT(lp_conv, ESP32P4AdcUnit),
        VMSTATE_ESP32P4_CLKWAIT(hw_timer, ESP32P4AdcUnit),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_adc = {
    .name = TYPE_ESP32P4_ADC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(hp_regs, ESP32P4AdcState, ESP32P4_ADC_HP_NREGS),
        VMSTATE_UINT32_ARRAY(lp_regs, ESP32P4AdcState, ESP32P4_ADC_LP_NREGS),
        VMSTATE_BOOL(hp_in_reset, ESP32P4AdcState),
        VMSTATE_BOOL(lp_in_reset, ESP32P4AdcState),
        VMSTATE_STRUCT_ARRAY(unit, ESP32P4AdcState, ESP32P4_ADC_UNITS, 1,
                             vmstate_esp32p4_adc_unit, ESP32P4AdcUnit),
        VMSTATE_BOOL(hp_running, ESP32P4AdcState),
        VMSTATE_UINT32(meas_count, ESP32P4AdcState),
        VMSTATE_UINT8(alt_unit, ESP32P4AdcState),
        VMSTATE_UINT32(eof_count, ESP32P4AdcState),
        VMSTATE_ESP32P4_CLKWAIT(hp_timer, ESP32P4AdcState),
        VMSTATE_UINT32_ARRAY(filter_val, ESP32P4AdcState,
                             ESP32P4_ADC_FILTERS),
        VMSTATE_BOOL_ARRAY(filter_valid, ESP32P4AdcState,
                           ESP32P4_ADC_FILTERS),
        VMSTATE_BOOL(hp_denied_logged, ESP32P4AdcState),
        VMSTATE_UINT32_ARRAY(pad_mv, ESP32P4AdcState, ESP32P4_ADC_CHANNELS),
        VMSTATE_CLOCK(clk, ESP32P4AdcState),
        VMSTATE_CLOCK(lp_clk, ESP32P4AdcState),
        VMSTATE_ARRAY_CLOCK(lp_sar_clk, ESP32P4AdcState, ESP32P4_ADC_UNITS),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32p4_adc_properties[] = {
    DEFINE_PROP_LINK("gdma", ESP32P4AdcState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
    DEFINE_PROP_LINK("ana", ESP32P4AdcState, ana, TYPE_ESP32P4_ANA,
                     ESP32P4AnaState *),
    DEFINE_PROP_LINK("efuse", ESP32P4AdcState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
    DEFINE_PROP_LINK("gpio", ESP32P4AdcState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
};

static void esp32p4_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_adc_reset_hold;
    rc->phases.exit = esp32p4_adc_reset_exit;
    dc->realize = esp32p4_adc_realize;
    dc->vmsd = &vmstate_esp32p4_adc;
    device_class_set_props(dc, esp32p4_adc_properties);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static const TypeInfo esp32p4_adc_info = {
    .name = TYPE_ESP32P4_ADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4AdcState),
    .instance_init = esp32p4_adc_init,
    .class_init = esp32p4_adc_class_init,
};

static void esp32p4_adc_register_types(void)
{
    type_register_static(&esp32p4_adc_info);
}

type_init(esp32p4_adc_register_types)
