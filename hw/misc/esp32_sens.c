/*
 * ESP32 SENS: the SAR ADCs, DACs, touch, Hall and temperature sensors, and
 * the ULP coprocessor's timer settings, start and RTC I2C interface
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM chapter 31 "On-Chip Sensors and Analog Signal
 * Processing" and section 22.5.3 "ADC/DAC mode"; ESP-IDF's soc/esp32
 * sens_reg.h, rtc_io_reg.h and syscon_reg.h for the registers, their reset
 * values and access types, and hal/esp32 adc_ll.h, dac_ll.h and
 * touch_sensor_ll.h for how software drives them.
 *
 * SENS is in the RTC domain. It holds:
 *
 * - The RTC controllers of SAR ADC1 (8 channels) and ADC2 (10 channels):
 *   a rising MEASn_START_SAR, with MEASn_START_FORCE, converts the channel
 *   SARn_EN_PAD selects at the pad's attenuation (SENS_SAR_ATTENn) and the
 *   width SARn_BIT_WIDTH gives, taking (SAMPLE_CYCLE + width + 1) *
 *   CLK_DIV cycles of RTC_FAST_CLK. The converter's raw output is the
 *   complement of the code; SARn_DATA_INV un-inverts it, as ESP-IDF sets
 *   it to.
 * - The DIG controllers' scan, whose configuration is in SYSCON: each WS
 *   edge of I2S0 in its ADC mode converts the next pattern table entry and
 *   hands I2S0 the result in the type I or type II DMA format.
 * - ADC2's arbitration: SAR2_PWDET_FORCE gives ADC2 to the Wi-Fi power
 *   detector, SAR2_DIG_FORCE with SYSCON's SAR2_MUX to the DIG controller,
 *   and otherwise the RTC controller has it. A controller without the ADC
 *   cannot convert: its start is ignored.
 * - The Hall sensor, powered by RTCIO_XPD_HALL with XPD_HALL_FORCE, which
 *   adds its output, of a sign set by RTCIO_HALL_PHASE, to SENSOR_VP (ADC1
 *   channel 0) and takes it from SENSOR_VN (channel 3).
 * - The DACs' sources: the cosine generator (DAC_CW_ENn), RTCIO's
 *   PDACn_DAC, or I2S0's DMA (DAC_DIG_FORCE). A DAC powered by
 *   PDACn_XPD_DAC with PDACn_DAC_XPD_FORCE drives its pad, GPIO25 or
 *   GPIO26, at VDD3P3_RTC * code / 255.
 * - The touch FSM: started by software (TOUCH_START_FORCE, TOUCH_START_EN)
 *   or by its timer every TOUCH_SLEEP_CYCLES of RTC_SLOW_CLK while
 *   RTC_CNTL_TOUCH_SLP_TIMER_EN is set, it powers the pads up for
 *   TOUCH_XPD_WAIT cycles of RTC_FAST_CLK, then counts each working pad's
 *   charge and discharge cycles for TOUCH_MEAS_DELAY cycles.
 *
 * SENS is clocked by RTC_FAST_CLK (TRM 7.2.6, "RTC_FAST_CLK is used to
 * clock the On-chip Sensor module", and 31.2.3, "The sensor is operated by
 * RTC_FAST_CLK, which normally runs at 8 MHz"). The registers' "8 MHz
 * cycles" are RTC_FAST_CLK cycles at the 8 MHz ESP-IDF runs it at:
 * RC_FAST_CLK undivided. The cosine generator runs from it too (ESP-IDF's
 * DAC_COSINE_CLK_SRC_RTC_FAST; the TRM's dig_clk_rtc). Every wait counts
 * cycles of the clock as it currently runs: a change of rate takes effect
 * from the moment of the change, and a stopped clock holds the wait and
 * the cosine generator's phase until it runs again.
 *
 * The analog inputs are QOM properties of the device: each ADC channel's
 * pad voltage, adc<u>-ch<n>-mv, which a digitally driven pad overrides with
 * its supply or ground level; the magnetic field at the Hall sensor,
 * hall-field-ut; and each touch pad's capacitance, touch<n>-ff. The DACs'
 * outputs are the read-only properties dac1-mv and dac2-mv.
 *
 * - The RTC controllers' sharing with the ULP coprocessor (TRM "ULP
 *   Coprocessor"): without MEASn_START_FORCE the ULP's ADC instruction
 *   starts the conversion, and without SARn_EN_PAD_FORCE the pad is the one
 *   the ULP's last ADC instruction selected.
 * - The temperature sensor: powered by TSENS_POWER_UP with
 *   TSENS_POWER_UP_FORCE, or by the ULP's TSENS instruction, it reports the
 *   die temperature in degrees Fahrenheit (ESP-IDF's temperatureRead()
 *   converts with (code - 32) / 1.8) in TSENS_OUT.
 * - The ULP's timer periods (ULP_CP_SLEEP_CYCn), its entry point and its
 *   software start (SENS_SAR_START_FORCE), which the ULP device reads.
 * - The RTC I2C controller's slave addresses (SENS_I2C_SLAVE_ADDRn) and the
 *   interface the ULP's I2C instructions drive, which SENS_SAR_I2C_CTRL
 *   takes over with SAR_I2C_START_FORCE; the byte read and the done flag
 *   land in SENS_SAR_SLAVE_ADDR4.
 *
 * The die temperature is the QOM property tsens-temp-mc, in millidegrees.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "migration/vmstate.h"
#include "hw/misc/esp32_sens.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/misc/esp32_apb_ctrl.h"
#include "hw/gpio/esp32_rtcio.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/i2c/esp32_rtc_i2c.h"

/* VDD3P3_RTC, the DACs' reference and a driven pad's high level */
#define VDD_MV                  3300
/* The SAR ADCs' reference voltage */
#define ADC_VREF_MV             1100
/*
 * The Hall sensor's output: microvolts per microtesla on each of VP and
 * VN. ESP32 documents no sensitivity; this one makes a 10 mT field read
 * about 300 in ESP-IDF's hall_sensor_read() at 12 bits and 0 dB.
 */
#define HALL_UV_PER_UT          2

/*
 * The touch pads' charge current: 1 uA per TOUCH_PADn_DAC (slope) step.
 * ESP32 documents no currents; with it ESP-IDF's default configuration
 * reads a 10 pF pad at about 1200.
 */
#define TOUCH_UA_PER_SLOPE      1
#define TOUCH_DEFAULT_FF        10000

/* SYSCON SARADC_CTRL and SARADC_CTRL2 fields */
#define SARADC_SAR2_MUX         (1u << 2)
#define SARADC_WORK_MODE(v)     (((v) >> 3) & 3)
#define SARADC_SAR_SEL          (1u << 5)
#define SARADC_SAR1_PATT_LEN(v) (((v) >> 15) & 0xf)
#define SARADC_SAR2_PATT_LEN(v) (((v) >> 19) & 0xf)
#define SARADC_SAR1_PATT_CLR    (1u << 23)
#define SARADC_SAR2_PATT_CLR    (1u << 24)
#define SARADC_DATA_SAR_SEL     (1u << 25)
#define SARADC2_SAR1_INV        (1u << 9)
#define SARADC2_SAR2_INV        (1u << 10)

/* RTCIO fields */
#define RTCIO_XPD_HALL          (1u << 31)
#define RTCIO_HALL_PHASE        (1u << 30)
#define RTCIO_PDAC_DAC(v)       (((v) >> 19) & 0xff)
#define RTCIO_PDAC_XPD_DAC      (1u << 18)
#define RTCIO_PDAC_XPD_FORCE    (1u << 10)
#define RTCIO_TOUCH_DREFH(v)    (((v) >> 29) & 3)
#define RTCIO_TOUCH_DREFL(v)    (((v) >> 27) & 3)
#define RTCIO_TOUCH_DRANGE(v)   (((v) >> 25) & 3)
#define RTCIO_TOUCH_SLOPE(v)    (((v) >> 23) & 7)
#define RTCIO_TOUCH_XPD         (1u << 20)

/* Each ADC channel's pad (TRM table 31.3-1) */
static const uint8_t adc_pad[ESP32_ADC_UNITS][ESP32_ADC2_CHANNELS] = {
    { 36, 37, 38, 39, 32, 33, 34, 35 },
    { 4, 0, 2, 15, 13, 12, 14, 27, 25, 26 },
};
static const unsigned adc_channels[ESP32_ADC_UNITS] = {
    ESP32_ADC1_CHANNELS, ESP32_ADC2_CHANNELS
};
/* SENSOR_VP and SENSOR_VN, ADC1's channels for the Hall sensor */
#define HALL_VP_CHANNEL         0
#define HALL_VN_CHANNEL         3

/*
 * Full scale, in mV, at each attenuation: 0 dB, 2.5 dB, 6 dB and 11 dB
 * above the reference.
 */
static const uint32_t adc_full_scale_mv[4] = { 1100, 1467, 2195, 3900 };

/* The DAC pads, GPIO25 and GPIO26 */
static const uint8_t dac_pad[ESP32_DAC_CHANNELS] = { 25, 26 };

/*
 * Touch channel n of SENS and RTCIO's TOUCH_PADn measures touch pad
 * touch_pad_of[n]: channels 8 and 9 are wired to pads T9 and T8, which
 * ESP-IDF's touch_sensor_ll.h swaps back.
 */
static const uint8_t touch_pad_of[ESP32_TOUCH_PADS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 9, 8
};

typedef struct RegInfo {
    uint32_t reset;
    uint32_t writable;
} RegInfo;

#define R(reg) [A_##reg / 4]

static const RegInfo reg_info[ESP32_SENS_REG_COUNT] = {
    R(SENS_SAR_READ_CTRL) = { 0x00070902, 0x1fffffff },
    R(SENS_SAR_MEAS_WAIT1) = { 0x000a000a, 0xffffffff },
    R(SENS_SAR_MEAS_WAIT2) = { 0x0020000a, 0x0fffffff },
    R(SENS_SAR_MEAS_CTRL) = { 0x0707338f, 0xffffffff },
    [A_SENS_ULP_CP_SLEEP_CYC0 / 4 + 0] = { 200, 0xffffffff },
    [A_SENS_ULP_CP_SLEEP_CYC0 / 4 + 1] = { 100, 0xffffffff },
    [A_SENS_ULP_CP_SLEEP_CYC0 / 4 + 2] = { 50, 0xffffffff },
    [A_SENS_ULP_CP_SLEEP_CYC0 / 4 + 3] = { 40, 0xffffffff },
    [A_SENS_ULP_CP_SLEEP_CYC0 / 4 + 4] = { 20, 0xffffffff },
    R(SENS_SAR_START_FORCE) = { 0x0000000f, 0x01ffffff },
    R(SENS_SAR_MEM_WR_CTRL) = { 0x00100200, 0x003fffff },
    R(SENS_SAR_ATTEN1) = { 0xffffffff, 0xffffffff },
    R(SENS_SAR_ATTEN2) = { 0xffffffff, 0xffffffff },
    R(SENS_SAR_SLAVE_ADDR1) = { 0, 0x003fffff },
    R(SENS_SAR_SLAVE_ADDR2) = { 0, 0x003fffff },
    R(SENS_SAR_SLAVE_ADDR3) = { 0, 0x003fffff },
    R(SENS_SAR_SLAVE_ADDR4) = { 0, 0x003fffff },
    R(SENS_SAR_TSENS_CTRL) = { 0x00066002, 0x07ffffff },
    R(SENS_SAR_I2C_CTRL) = { 0, 0x3fffffff },
    R(SENS_SAR_MEAS_START1) = { 0, 0xfffe0000 },
    R(SENS_SAR_TOUCH_CTRL1) = { 0x02041000, 0x0fffffff },
    [A_SENS_SAR_TOUCH_THRES1 / 4 ... A_SENS_SAR_TOUCH_THRES5 / 4] = {
        0, 0xffffffff },
    R(SENS_SAR_TOUCH_CTRL2) = { 0x00400800, 0x3ffff800 },
    R(SENS_SAR_TOUCH_ENABLE) = { 0x3fffffff, 0x3fffffff },
    R(SENS_SAR_READ_CTRL2) = { 0x00070902, 0x3fffffff },
    R(SENS_SAR_MEAS_START2) = { 0, 0xfffe0000 },
    R(SENS_SAR_DAC_CTRL1) = { 0, 0x03ffffff },
    R(SENS_SAR_DAC_CTRL2) = { 0x03000000, 0x03ffffff },
    R(SENS_SAR_MEAS_CTRL2) = { 0x00000003, 0x0007ffff },
    R(SENS_SAR_NOUSE) = { 0, 0xffffffff },
    R(SENS_SARDATE) = { 0x01605180, 0x0fffffff },
};

#define REG(s, name) ((s)->regs[A_##name / 4])

static inline bool bit32(uint32_t v, unsigned n)
{
    return (v >> n) & 1;
}

static uint32_t rtcio_reg(Esp32SensState *s, hwaddr addr)
{
    return s->rtcio->regs[addr / 4];
}

/* ---- Waits counted in clock cycles ---- */

/* When w ends, with clk running at its current rate */
static int64_t wait_end_ns(Esp32SensWait *w, Clock *clk)
{
    return w->base_ns + MAX(clock_ticks_to_ns(clk, w->left), 1);
}

/* Arm w's timer for its remaining cycles at clk's current rate */
static void wait_schedule(Esp32SensWait *w, Clock *clk)
{
    if (!clock_is_enabled(clk)) {
        timer_del(&w->timer);
        return;
    }
    timer_mod(&w->timer, wait_end_ns(w, clk));
}

/*
 * Wait for cycles cycles of clk from from_ns: now, or the end of the wait
 * before it, which a late timer callback must not stretch.
 */
static void wait_start(Esp32SensWait *w, Clock *clk, int64_t from_ns,
                       uint64_t cycles)
{
    w->base_ns = from_ns;
    w->left = cycles;
    wait_schedule(w, clk);
}

/* clk is about to change: take off the cycles it has made since base_ns */
static void wait_fold(Esp32SensWait *w, Clock *clk, int64_t now)
{
    uint64_t done = clock_ns_to_ticks(clk, now - w->base_ns);

    w->left -= MIN(done, w->left);
    w->base_ns = now;
}

/* ---- The DACs ---- */

static uint32_t cw_phase_at(Esp32SensState *s, int64_t now)
{
    uint32_t ctrl1 = REG(s, SENS_SAR_DAC_CTRL1);
    uint64_t ticks;

    if (!FIELD_EX32(ctrl1, SENS_SAR_DAC_CTRL1, SW_TONE_EN)) {
        return s->cw_phase;
    }
    ticks = clock_ns_to_ticks(s->fast_clk, now - s->cw_anchor_ns);
    return (s->cw_phase +
            ticks * FIELD_EX32(ctrl1, SENS_SAR_DAC_CTRL1, SW_FSTEP)) & 0xffff;
}

/*
 * Fold the generator's progress into its phase before a change of step or
 * of RTC_FAST_CLK
 */
static void cw_reanchor(Esp32SensState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->cw_phase = cw_phase_at(s, now);
    s->cw_anchor_ns = now;
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * The cosine generator's output for DAC ch (TRM 31.4.4): a signed cosine
 * of amplitude 127 at RTC_FAST_CLK * SW_FSTEP / 65536, one SW_FSTEP phase
 * step per RTC_FAST_CLK cycle, scaled by DAC_SCALEn,
 * offset by the signed DAC_DCn and saturated, then made a DAC code by
 * DAC_INVn: 0 as is, 1 inverted, 2 with the MSB inverted (offset binary),
 * 3 with the other bits inverted (offset binary, negated).
 */
static uint8_t cw_code(Esp32SensState *s, unsigned ch)
{
    uint32_t ctrl1 = REG(s, SENS_SAR_DAC_CTRL1);
    uint32_t ctrl2 = REG(s, SENS_SAR_DAC_CTRL2);
    unsigned scale = ch ? FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, SCALE2) :
                          FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, SCALE1);
    unsigned inv = ch ? FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, INV2) :
                        FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, INV1);
    int dc = (int8_t)(ch ? FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, DC2) :
                           FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, DC1));
    int x = 0;
    uint8_t code;

    if (FIELD_EX32(ctrl1, SENS_SAR_DAC_CTRL1, SW_TONE_EN)) {
        uint32_t phase = cw_phase_at(s,
                                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

        x = (int)lround(127.0 * cos(2.0 * M_PI * phase / 65536.0));
        x >>= scale;
    }
    x = MAX(-128, MIN(127, x + dc));
    code = (uint8_t)x;
    switch (inv) {
    case 1:
        return ~code;
    case 2:
        return code ^ 0x80;
    case 3:
        return code ^ 0x7f;
    default:
        return code;
    }
}

static bool dac_powered(Esp32SensState *s, unsigned ch)
{
    uint32_t pad = rtcio_reg(s, ch ? A_RTCIO_PAD_DAC2 : A_RTCIO_PAD_DAC1);

    return (pad & RTCIO_PDAC_XPD_FORCE) && (pad & RTCIO_PDAC_XPD_DAC);
}

static uint8_t dac_code(Esp32SensState *s, unsigned ch)
{
    uint32_t ctrl2 = REG(s, SENS_SAR_DAC_CTRL2);
    bool cw = ch ? FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, CW_EN2) :
                   FIELD_EX32(ctrl2, SENS_SAR_DAC_CTRL2, CW_EN1);

    if (esp32_sens_dac_dma_enabled(s)) {
        return s->dac_dma[ch];
    }
    if (cw) {
        return cw_code(s, ch);
    }
    return RTCIO_PDAC_DAC(rtcio_reg(s, ch ? A_RTCIO_PAD_DAC2 :
                                            A_RTCIO_PAD_DAC1));
}

/* [spec:nuos:req:emu.esp32.analog] DAC ch's output, in uV; 0 if off */
static uint32_t dac_uv(Esp32SensState *s, unsigned ch)
{
    if (!dac_powered(s, ch)) {
        return 0;
    }
    return (uint64_t)dac_code(s, ch) * VDD_MV * 1000 / 255;
}

bool esp32_sens_dac_dma_enabled(Esp32SensState *s)
{
    return FIELD_EX32(REG(s, SENS_SAR_DAC_CTRL1), SENS_SAR_DAC_CTRL1,
                      DAC_DIG_FORCE);
}

void esp32_sens_dac_dma(Esp32SensState *s, unsigned ch, uint8_t code)
{
    assert(ch < ESP32_DAC_CHANNELS);
    s->dac_dma[ch] = code;
}

/* ---- Pad voltages ---- */

/*
 * [spec:nuos:req:emu.esp32.analog]
 * The voltage on an ADC channel's pad, in uV: a powered DAC's output on
 * its pad; else the supply or ground where the pad is driven digitally;
 * else the voltage set from QEMU. The powered Hall sensor adds its output
 * to SENSOR_VP and takes it from SENSOR_VN, reversed by HALL_PHASE.
 */
static int64_t pad_uv(Esp32SensState *s, unsigned unit, unsigned ch)
{
    unsigned pad = adc_pad[unit][ch];
    uint32_t ctrl1 = REG(s, SENS_SAR_TOUCH_CTRL1);
    uint32_t hall = rtcio_reg(s, A_RTCIO_HALL_SENS);
    int64_t uv = (int64_t)s->adc_mv[unit][ch] * 1000;
    bool level;

    for (unsigned d = 0; d < ESP32_DAC_CHANNELS; d++) {
        if (pad == dac_pad[d] && dac_powered(s, d)) {
            return dac_uv(s, d);
        }
    }
    if (esp32_gpio_pad_driven(s->gpio, pad, &level)) {
        return level ? VDD_MV * 1000 : 0;
    }
    if (unit == 0 && (ch == HALL_VP_CHANNEL || ch == HALL_VN_CHANNEL)) {
        bool on = FIELD_EX32(ctrl1, SENS_SAR_TOUCH_CTRL1, XPD_HALL_FORCE) &&
                  (hall & RTCIO_XPD_HALL);
        bool phase = FIELD_EX32(ctrl1, SENS_SAR_TOUCH_CTRL1,
                                HALL_PHASE_FORCE) &&
                     (hall & RTCIO_HALL_PHASE);
        int64_t out = (int64_t)s->hall_ut * HALL_UV_PER_UT;

        if (on) {
            out = phase ? out : -out;
            uv += ch == HALL_VP_CHANNEL ? out : -out;
        }
    }
    return MAX(uv, 0);
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * Convert channel ch of ADC unit at attenuation atten to a bits-wide code,
 * saturating at full scale.
 */
static uint32_t adc_convert(Esp32SensState *s, unsigned unit, unsigned ch,
                            unsigned atten, unsigned bits)
{
    uint32_t max = (1u << bits) - 1;
    uint64_t fs_uv = (uint64_t)adc_full_scale_mv[atten & 3] * 1000;
    uint64_t uv = pad_uv(s, unit, ch);

    if (uv >= fs_uv) {
        return max;
    }
    return (uv * max + fs_uv / 2) / fs_uv;
}

/* ---- The RTC controllers ---- */

static hwaddr meas_start_reg(unsigned unit)
{
    return unit ? A_SENS_SAR_MEAS_START2 : A_SENS_SAR_MEAS_START1;
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * Which controller has ADC2: PWDET (Wi-Fi's power detector) with
 * SAR2_PWDET_FORCE; with SAR2_DIG_FORCE the DIG controller if SYSCON's
 * SAR2_MUX selects it, else PWDET; otherwise the RTC controller.
 */
typedef enum AdcOwner {
    ADC_OWNER_RTC,
    ADC_OWNER_DIG,
    ADC_OWNER_PWDET,
} AdcOwner;

static AdcOwner adc_owner(Esp32SensState *s, unsigned unit)
{
    uint32_t ctrl2 = REG(s, SENS_SAR_READ_CTRL2);
    uint32_t saradc = s->apb_ctrl->regs[R_APB_CTRL_SARADC_CTRL];

    if (unit == 0) {
        return FIELD_EX32(REG(s, SENS_SAR_READ_CTRL), SENS_SAR_READ_CTRL,
                          DIG_FORCE) ? ADC_OWNER_DIG : ADC_OWNER_RTC;
    }
    if (FIELD_EX32(ctrl2, SENS_SAR_READ_CTRL2, PWDET_FORCE)) {
        return ADC_OWNER_PWDET;
    }
    if (FIELD_EX32(ctrl2, SENS_SAR_READ_CTRL2, DIG_FORCE)) {
        return (saradc & SARADC_SAR2_MUX) ? ADC_OWNER_DIG : ADC_OWNER_PWDET;
    }
    return ADC_OWNER_RTC;
}

static void update_meas_status(Esp32SensState *s)
{
    REG(s, SENS_SAR_SLAVE_ADDR1) =
        FIELD_DP32(REG(s, SENS_SAR_SLAVE_ADDR1), SENS_SAR_SLAVE_ADDR1,
                   MEAS_STATUS, s->adc_busy[0] | (s->adc_busy[1] << 1));
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * [spec:nuos:req:emu.esp32.ulp]
 * Whether the RTC controller of ADC unit can convert: it must have the ADC
 * and not be stopped by SARn_STOP.
 */
static bool rtc_adc_usable(Esp32SensState *s, unsigned unit)
{
    uint32_t force = REG(s, SENS_SAR_START_FORCE);
    AdcOwner owner = adc_owner(s, unit);

    if (owner != ADC_OWNER_RTC) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: RTC controller of ADC%u started while %s "
                      "controller has the ADC; nothing is converted\n",
                      unit + 1,
                      owner == ADC_OWNER_DIG ? "the DIG" : "the PWDET");
        return false;
    }
    if (bit32(force, unit ? R_SENS_SAR_START_FORCE_SAR2_STOP_SHIFT :
                            R_SENS_SAR_START_FORCE_SAR1_STOP_SHIFT)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ADC%u started while SAR%u_STOP is set\n",
                      unit + 1, unit + 1);
        return false;
    }
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * [spec:nuos:req:emu.esp32.ulp]
 * The code the RTC controller of ADC unit converts now: the pad
 * SARn_EN_PAD selects with SARn_EN_PAD_FORCE, else the one the ULP
 * selected, at its attenuation and the width SARn_BIT_WIDTH gives, raw
 * (inverted) unless SARn_DATA_INV.
 */
static uint32_t rtc_adc_code(Esp32SensState *s, unsigned unit)
{
    uint32_t start = s->regs[meas_start_reg(unit) / 4];
    uint32_t read = unit ? REG(s, SENS_SAR_READ_CTRL2) :
                           REG(s, SENS_SAR_READ_CTRL);
    uint32_t force = REG(s, SENS_SAR_START_FORCE);
    uint32_t atten_reg = unit ? REG(s, SENS_SAR_ATTEN2) :
                                REG(s, SENS_SAR_ATTEN1);
    uint32_t pads, code, mask;
    unsigned bits, ch;

    if (FIELD_EX32(start, SENS_SAR_MEAS_START1, EN_PAD_FORCE)) {
        pads = FIELD_EX32(start, SENS_SAR_MEAS_START1, EN_PAD);
    } else {
        pads = s->ulp_pads[unit];
    }
    pads &= (1u << adc_channels[unit]) - 1;

    bits = 9 + (unit ? FIELD_EX32(force, SENS_SAR_START_FORCE,
                                  SAR2_BIT_WIDTH) :
                       FIELD_EX32(force, SENS_SAR_START_FORCE,
                                  SAR1_BIT_WIDTH));
    mask = (1u << bits) - 1;
    if (!pads) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ADC%u converts with no pad selected\n",
                      unit + 1);
        code = 0;
    } else {
        if (pads & (pads - 1)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_sens: ADC%u has several pads selected "
                          "(0x%x); channel %u is converted\n", unit + 1,
                          pads, ctz32(pads));
        }
        ch = ctz32(pads);
        code = adc_convert(s, unit, ch, (atten_reg >> (2 * ch)) & 3, bits);
    }
    if (!(unit ? FIELD_EX32(read, SENS_SAR_READ_CTRL2, DATA_INV) :
                 FIELD_EX32(read, SENS_SAR_READ_CTRL, DATA_INV))) {
        code = ~code & mask;
    }
    return code;
}

/* The conversion is under way: deliver code after cycles of RTC_FAST_CLK */
static void rtc_adc_begin(Esp32SensState *s, unsigned unit, uint32_t code,
                          uint32_t cycles)
{
    s->adc_result[unit] = code;
    s->adc_busy[unit] = true;
    s->regs[meas_start_reg(unit) / 4] &= ~R_SENS_SAR_MEAS_START1_DONE_MASK;
    update_meas_status(s);
    wait_start(&s->adc_wait[unit], s->fast_clk,
               qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), cycles);
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * Software starts a conversion of the RTC controller of ADC unit: sample
 * the pad now, deliver the result after (SAMPLE_CYCLE + width + 1) *
 * CLK_DIV cycles.
 */
static void rtc_adc_start(Esp32SensState *s, unsigned unit)
{
    uint32_t read = unit ? REG(s, SENS_SAR_READ_CTRL2) :
                           REG(s, SENS_SAR_READ_CTRL);
    uint32_t force = REG(s, SENS_SAR_START_FORCE);
    uint32_t div, bits;

    if (!rtc_adc_usable(s, unit)) {
        return;
    }
    bits = 9 + (unit ? FIELD_EX32(force, SENS_SAR_START_FORCE,
                                  SAR2_BIT_WIDTH) :
                       FIELD_EX32(force, SENS_SAR_START_FORCE,
                                  SAR1_BIT_WIDTH));
    div = FIELD_EX32(read, SENS_SAR_READ_CTRL, CLK_DIV);
    if (div == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ADC%u's SAR clock divider is 0\n",
                      unit + 1);
        div = 1;
    }
    rtc_adc_begin(s, unit, rtc_adc_code(s, unit),
                  (FIELD_EX32(read, SENS_SAR_READ_CTRL, SAMPLE_CYCLE) +
                   bits + 1) * div);
}

uint32_t esp32_sens_ulp_adc(Esp32SensState *s, unsigned unit, unsigned mux,
                            uint32_t *cycles)
{
    uint32_t start = s->regs[meas_start_reg(unit) / 4];
    uint32_t read = unit ? REG(s, SENS_SAR_READ_CTRL2) :
                           REG(s, SENS_SAR_READ_CTRL);
    uint32_t code;

    /* 23 + the amplifier's three waits + the sampling (ESP-IDF's ADC) */
    *cycles = 23 +
        MAX(1, FIELD_EX32(REG(s, SENS_SAR_MEAS_WAIT1), SENS_SAR_MEAS_WAIT1,
                          AMP_WAIT1)) +
        MAX(1, FIELD_EX32(REG(s, SENS_SAR_MEAS_WAIT1), SENS_SAR_MEAS_WAIT1,
                          AMP_WAIT2)) +
        MAX(1, FIELD_EX32(REG(s, SENS_SAR_MEAS_WAIT2), SENS_SAR_MEAS_WAIT2,
                          AMP_WAIT3)) +
        FIELD_EX32(read, SENS_SAR_READ_CTRL, SAMPLE_CYCLE) +
        FIELD_EX32(read, SENS_SAR_READ_CTRL, SAMPLE_BIT);

    if (mux == 0 || mux > adc_channels[unit]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ULP ADC instruction selects pad %u of "
                      "ADC%u, which has pads 1 to %u\n", mux, unit + 1,
                      adc_channels[unit]);
        s->ulp_pads[unit] = 0;
    } else {
        s->ulp_pads[unit] = 1u << (mux - 1);
    }
    if (FIELD_EX32(start, SENS_SAR_MEAS_START1, START_FORCE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ULP ADC instruction while "
                      "MEAS%u_START_FORCE gives software the start; ADC%u "
                      "does not convert\n", unit + 1, unit + 1);
        return FIELD_EX32(start, SENS_SAR_MEAS_START1, DATA);
    }
    if (!rtc_adc_usable(s, unit)) {
        return FIELD_EX32(start, SENS_SAR_MEAS_START1, DATA);
    }
    code = rtc_adc_code(s, unit);
    rtc_adc_begin(s, unit, code, *cycles);
    return code;
}

static void rtc_adc_done(Esp32SensState *s, unsigned unit)
{
    uint32_t *r = &s->regs[meas_start_reg(unit) / 4];

    *r = FIELD_DP32(*r, SENS_SAR_MEAS_START1, DATA, s->adc_result[unit]);
    *r |= R_SENS_SAR_MEAS_START1_DONE_MASK;
    s->adc_busy[unit] = false;
    update_meas_status(s);
}

static void adc1_timer_cb(void *opaque)
{
    rtc_adc_done(opaque, 0);
}

static void adc2_timer_cb(void *opaque)
{
    rtc_adc_done(opaque, 1);
}

/* ---- The DIG controllers ---- */

static void dig_clear_pointers(Esp32SensState *s)
{
    uint32_t ctrl = s->apb_ctrl->regs[R_APB_CTRL_SARADC_CTRL];

    if (ctrl & SARADC_SAR1_PATT_CLR) {
        s->patt_ptr[0] = 0;
    }
    if (ctrl & SARADC_SAR2_PATT_CLR) {
        s->patt_ptr[1] = 0;
    }
}

static void esp32_sens_saradc_ctrl(void *opaque, int n, int level)
{
    if (level) {
        dig_clear_pointers(ESP32_SENS(opaque));
    }
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * One conversion of the DIG controller of ADC unit: the pattern table
 * entry at its pointer (channel, width, attenuation; TRM table 31.3-3),
 * as a 16-bit DMA word of type I (channel, 12-bit data) or, with
 * DATA_SAR_SEL, type II (unit, channel, 11-bit data).
 */
static uint16_t dig_convert(Esp32SensState *s, unsigned unit)
{
    uint32_t ctrl = s->apb_ctrl->regs[R_APB_CTRL_SARADC_CTRL];
    uint32_t ctrl2 = s->apb_ctrl->regs[R_APB_CTRL_SARADC_CTRL2];
    unsigned len = (unit ? SARADC_SAR2_PATT_LEN(ctrl) :
                           SARADC_SAR1_PATT_LEN(ctrl)) + 1;
    unsigned tab = (unit ? R_APB_CTRL_SARADC_SAR2_PATT_TAB1 :
                           R_APB_CTRL_SARADC_SAR1_PATT_TAB1);
    unsigned p = s->patt_ptr[unit] % len;
    uint8_t item = s->apb_ctrl->regs[tab + p / 4] >> (24 - 8 * (p % 4));
    unsigned ch = item >> 4;
    unsigned bits = 9 + ((item >> 2) & 3);
    uint32_t code = 0;
    bool inv = ctrl2 & (unit ? SARADC2_SAR2_INV : SARADC2_SAR1_INV);

    s->patt_ptr[unit] = (p + 1) % len;
    if (adc_owner(s, unit) != ADC_OWNER_DIG) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: DIG controller scans ADC%u, which another "
                      "controller has\n", unit + 1);
    } else if (ch >= adc_channels[unit]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ADC%u pattern entry %u names channel %u, "
                      "which ADC%u does not have\n", unit + 1, p, ch,
                      unit + 1);
    } else {
        code = adc_convert(s, unit, ch, item & 3, bits);
    }
    if (!inv) {
        code = ~code & ((1u << bits) - 1);
    }
    if (ctrl & SARADC_DATA_SAR_SEL) {
        if (bits == 12) {
            code >>= 1;
        }
        return (unit << 15) | ((ch & 0xf) << 11) | (code & 0x7ff);
    }
    return ((ch & 0xf) << 12) | (code & 0xfff);
}

unsigned esp32_sens_dig_sample(Esp32SensState *s, uint16_t out[2])
{
    uint32_t ctrl = s->apb_ctrl->regs[R_APB_CTRL_SARADC_CTRL];

    dig_clear_pointers(s);
    switch (SARADC_WORK_MODE(ctrl)) {
    case 0:
        out[0] = dig_convert(s, (ctrl & SARADC_SAR_SEL) ? 1 : 0);
        return 1;
    case 1:
        out[0] = dig_convert(s, 0);
        out[1] = dig_convert(s, 1);
        return 2;
    case 2:
        out[0] = dig_convert(s, s->alt_unit);
        s->alt_unit ^= 1;
        return 1;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: reserved SARADC_WORK_MODE 3\n");
        out[0] = dig_convert(s, 0);
        return 1;
    }
}

/* ---- The touch sensor ---- */

/*
 * [spec:nuos:req:emu.esp32.analog]
 * The count of touch channel n over one measurement: the pad charges and
 * discharges between DREFL and DREFH (RTCIO_TOUCH_CFG) at the current its
 * slope sets, so it completes t * I / (2 C dV) cycles in the measurement's
 * t = TOUCH_MEAS_DELAY cycles of RTC_FAST_CLK, meas_ns nanoseconds. The
 * pads oscillate on their own: a measurement RTC_FAST_CLK stretches, by
 * running slower or stopping, counts for longer.
 */
static uint32_t touch_count(Esp32SensState *s, unsigned n, uint64_t meas_ns)
{
    static const uint32_t atten_mv[4] = { 1500, 1000, 500, 0 };
    uint32_t cfg = rtcio_reg(s, A_RTCIO_TOUCH_CFG);
    uint32_t pad = rtcio_reg(s, A_RTCIO_TOUCH_PAD0 + 4 * n);
    int32_t vh = 2400 + 100 * RTCIO_TOUCH_DREFH(cfg) -
                 atten_mv[RTCIO_TOUCH_DRANGE(cfg)];
    int32_t vl = 500 + 100 * RTCIO_TOUCH_DREFL(cfg);
    uint64_t c_ff = s->touch_ff[touch_pad_of[n]];
    uint64_t ua = RTCIO_TOUCH_SLOPE(pad) * TOUCH_UA_PER_SLOPE;
    uint64_t count;

    if (!(pad & RTCIO_TOUCH_XPD) || ua == 0) {
        return 0;
    }
    if (vh <= vl) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: touch DREFH %d mV is not above DREFL %d "
                      "mV\n", vh, vl);
        return 0;
    }
    if (c_ff == 0) {
        return 0xffff;
    }
    /* meas_ns 1e-9 s * ua 1e-6 A / (2 * c_ff 1e-15 F * dV 1e-3 V) */
    count = muldiv64(meas_ns, ua * 500, c_ff * (vh - vl));
    return MIN(count, 0xffff);
}

static bool touch_wakeup_cond(Esp32SensState *s)
{
    uint32_t st = FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL2),
                             SENS_SAR_TOUCH_CTRL2, MEAS_EN);
    uint32_t en = REG(s, SENS_SAR_TOUCH_ENABLE);
    bool set1 = st & FIELD_EX32(en, SENS_SAR_TOUCH_ENABLE, OUTEN1);
    bool set2 = st & FIELD_EX32(en, SENS_SAR_TOUCH_ENABLE, OUTEN2);

    return set1 && (FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL1),
                               SENS_SAR_TOUCH_CTRL1, OUT_1EN) || set2);
}

static void touch_update_wakeup(Esp32SensState *s)
{
    bool level = touch_wakeup_cond(s);

    if (level != s->touch_wakeup_level) {
        s->touch_wakeup_level = level;
        qemu_set_irq(s->touch_wakeup, level);
    }
}

/* The clock the FSM's current phase counts: RTC_SLOW_CLK while it sleeps */
static Clock *touch_clk(Esp32SensState *s)
{
    return s->touch_phase == ESP32_TOUCH_SLEEP ? s->slow_clk : s->fast_clk;
}

/* Enter phase at from_ns for cycles of its clock */
static void touch_enter(Esp32SensState *s, Esp32TouchPhase phase,
                        int64_t from_ns, uint64_t cycles)
{
    s->touch_phase = phase;
    wait_start(&s->touch_wait, touch_clk(s), from_ns, cycles);
}

static bool touch_timer_mode(Esp32SensState *s)
{
    return s->touch_timer_en &&
           !FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL2), SENS_SAR_TOUCH_CTRL2,
                       START_FORCE);
}

static void touch_sleep(Esp32SensState *s, int64_t from_ns)
{
    uint32_t cycles = FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL2),
                                 SENS_SAR_TOUCH_CTRL2, SLEEP_CYCLES);

    touch_enter(s, ESP32_TOUCH_SLEEP, from_ns, cycles);
}

/* [spec:nuos:req:emu.esp32.analog] A measurement starts: power the pads */
static void touch_start(Esp32SensState *s, int64_t from_ns)
{
    uint32_t wait = FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL1),
                               SENS_SAR_TOUCH_CTRL1, XPD_WAIT);

    if (!FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL2), SENS_SAR_TOUCH_CTRL2,
                    START_FSM_EN)) {
        qemu_log_mask(LOG_UNIMP,
                      "esp32_sens: touch pads driven by RTCIO's "
                      "TOUCH_PADn_START and XPD (TOUCH_START_FSM_EN clear) "
                      "are not modelled\n");
        s->touch_phase = ESP32_TOUCH_IDLE;
        return;
    }
    REG(s, SENS_SAR_TOUCH_CTRL2) &= ~R_SENS_SAR_TOUCH_CTRL2_MEAS_DONE_MASK;
    touch_enter(s, ESP32_TOUCH_XPD_WAIT, from_ns, wait);
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * A measurement ends: each working channel's count goes to its
 * TOUCH_MEAS_OUTn, and a channel below its threshold (above it, with
 * TOUCH_OUT_SEL) is touched, which TOUCH_MEAS_EN records until cleared.
 * The TOUCH interrupt is raised when SET1 has a touched pad and, without
 * TOUCH_OUT_1EN, SET2 has one too; the wakeup request follows the same
 * condition.
 */
static void touch_finish(Esp32SensState *s, int64_t end_ns)
{
    uint32_t worken = FIELD_EX32(REG(s, SENS_SAR_TOUCH_ENABLE),
                                 SENS_SAR_TOUCH_ENABLE, WORKEN);
    bool above = FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL1),
                            SENS_SAR_TOUCH_CTRL1, OUT_SEL);
    uint64_t meas_ns = end_ns - s->touch_meas_ns;
    uint32_t touched = 0;

    for (unsigned n = 0; n < ESP32_TOUCH_PADS; n++) {
        unsigned shift = (n & 1) ? 0 : 16;
        uint32_t *out = &s->regs[A_SENS_SAR_TOUCH_OUT1 / 4 + n / 2];
        uint32_t thr = (s->regs[A_SENS_SAR_TOUCH_THRES1 / 4 + n / 2] >>
                        shift) & 0xffff;
        uint32_t count;

        if (!bit32(worken, n)) {
            continue;
        }
        count = touch_count(s, n, meas_ns);
        *out = deposit32(*out, shift, 16, count);
        if (above ? count > thr : count < thr) {
            touched |= 1u << n;
        }
    }
    REG(s, SENS_SAR_TOUCH_CTRL2) |= touched |
                                    R_SENS_SAR_TOUCH_CTRL2_MEAS_DONE_MASK;
    if (touch_wakeup_cond(s)) {
        qemu_irq_pulse(s->touch_int);
    }
    touch_update_wakeup(s);
}

static void touch_timer_cb(void *opaque)
{
    Esp32SensState *s = opaque;
    uint32_t delay = FIELD_EX32(REG(s, SENS_SAR_TOUCH_CTRL1),
                                SENS_SAR_TOUCH_CTRL1, MEAS_DELAY);
    int64_t end = wait_end_ns(&s->touch_wait, touch_clk(s));

    switch (s->touch_phase) {
    case ESP32_TOUCH_XPD_WAIT:
        s->touch_meas_ns = end;
        touch_enter(s, ESP32_TOUCH_MEASURE, end, delay);
        break;
    case ESP32_TOUCH_MEASURE:
        touch_finish(s, end);
        if (touch_timer_mode(s)) {
            touch_sleep(s, end);
        } else {
            s->touch_phase = ESP32_TOUCH_IDLE;
        }
        break;
    case ESP32_TOUCH_SLEEP:
        touch_start(s, end);
        break;
    default:
        break;
    }
}

/* The FSM's start source changed: start or stop its timer */
static void touch_update_mode(Esp32SensState *s)
{
    if (touch_timer_mode(s)) {
        if (s->touch_phase == ESP32_TOUCH_IDLE) {
            touch_sleep(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
    } else if (s->touch_phase == ESP32_TOUCH_SLEEP) {
        timer_del(&s->touch_wait.timer);
        s->touch_phase = ESP32_TOUCH_IDLE;
    }
}

static void esp32_sens_touch_timer(void *opaque, int n, int level)
{
    Esp32SensState *s = ESP32_SENS(opaque);

    s->touch_timer_en = level;
    touch_update_mode(s);
}

/* ---- Clock changes ---- */

/*
 * [spec:nuos:req:emu.esp32.analog]
 * [spec:nuos:req:emu.esp32.clock-gating]
 * RTC_FAST_CLK or RTC_SLOW_CLK is about to change (ClockPreUpdate): the
 * waits counting it, and the cosine generator on RTC_FAST_CLK, keep the
 * cycles made at the old rate. Once it has changed (ClockUpdate) the
 * waits run on at the new rate, or hold while it is stopped.
 */
static void esp32_sens_clk_event(Esp32SensState *s, Clock *clk,
                                 ClockEvent event)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    Esp32SensWait *waits[ESP32_ADC_UNITS + 2];
    unsigned n = 0;

    if (clk == s->fast_clk) {
        for (unsigned u = 0; u < ESP32_ADC_UNITS; u++) {
            if (s->adc_busy[u]) {
                waits[n++] = &s->adc_wait[u];
            }
        }
        if (s->i2c_busy) {
            waits[n++] = &s->i2c_wait;
        }
        if (event == ClockPreUpdate) {
            cw_reanchor(s);
        }
    }
    if (s->touch_phase != ESP32_TOUCH_IDLE && touch_clk(s) == clk) {
        waits[n++] = &s->touch_wait;
    }
    for (unsigned i = 0; i < n; i++) {
        if (event == ClockPreUpdate) {
            wait_fold(waits[i], clk, now);
        } else {
            wait_schedule(waits[i], clk);
        }
    }
}

static void esp32_sens_fast_clk_update(void *opaque, ClockEvent event)
{
    Esp32SensState *s = opaque;

    esp32_sens_clk_event(s, s->fast_clk, event);
}

static void esp32_sens_slow_clk_update(void *opaque, ClockEvent event)
{
    Esp32SensState *s = opaque;

    esp32_sens_clk_event(s, s->slow_clk, event);
}

/* ---- The temperature sensor ---- */

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * The temperature sensor's code: the die temperature in degrees
 * Fahrenheit, saturated to 8 bits.
 */
static uint32_t tsens_code(Esp32SensState *s)
{
    int64_t f_milli = (int64_t)s->tsens_mc * 9 / 5 + 32000;

    return MAX(0, MIN(255, (f_milli + 500) / 1000));
}

static void tsens_latch(Esp32SensState *s, uint32_t code)
{
    uint32_t *r = &REG(s, SENS_SAR_SLAVE_ADDR3);

    *r = FIELD_DP32(*r, SENS_SAR_SLAVE_ADDR3, TSENS_OUT, code);
    *r = FIELD_DP32(*r, SENS_SAR_SLAVE_ADDR3, TSENS_RDY_OUT, 1);
}

uint32_t esp32_sens_ulp_tsens(Esp32SensState *s, uint32_t wait,
                              uint32_t *cycles)
{
    uint32_t ctrl = REG(s, SENS_SAR_TSENS_CTRL);
    uint32_t code;

    /*
     * ESP32 documents no TSENS timing: the sensor powers up for
     * TSENS_XPD_WAIT cycles, then measures for the instruction's wait.
     */
    *cycles = 2 + FIELD_EX32(ctrl, SENS_SAR_TSENS_CTRL, XPD_WAIT) + wait;
    if (FIELD_EX32(ctrl, SENS_SAR_TSENS_CTRL, POWER_UP_FORCE) &&
        !FIELD_EX32(ctrl, SENS_SAR_TSENS_CTRL, POWER_UP)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: ULP TSENS instruction with the "
                      "temperature sensor forced off\n");
        return FIELD_EX32(REG(s, SENS_SAR_SLAVE_ADDR3), SENS_SAR_SLAVE_ADDR3,
                          TSENS_OUT);
    }
    code = tsens_code(s);
    tsens_latch(s, code);
    return code;
}

/* ---- The RTC I2C interface ---- */

static void i2c_timer_cb(void *opaque)
{
    Esp32SensState *s = opaque;

    s->i2c_busy = false;
    REG(s, SENS_SAR_SLAVE_ADDR4) |= R_SENS_SAR_SLAVE_ADDR4_I2C_DONE_MASK;
}

uint32_t esp32_sens_rtc_i2c(Esp32SensState *s, uint32_t ctrl,
                            uint32_t *cycles)
{
    unsigned sel = extract32(ctrl, 22, 4);
    unsigned high = extract32(ctrl, 19, 3), low = extract32(ctrl, 16, 3);
    bool write = extract32(ctrl, 27, 1);
    uint32_t mask, addr, *a4 = &REG(s, SENS_SAR_SLAVE_ADDR4);
    uint8_t rdata;

    if (sel > 7) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: RTC I2C slave address %u of 0-7\n", sel);
        sel &= 7;
    }
    /* SENS_I2C_SLAVE_ADDR(2n) in [21:11] and (2n+1) in [10:0] */
    addr = extract32(s->regs[A_SENS_SAR_SLAVE_ADDR1 / 4 + sel / 2],
                     (sel & 1) ? 0 : 11, 11);
    if (addr > 0x7f) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: RTC I2C slave address 0x%x is not a "
                      "7-bit address\n", addr);
    }
    if (high < low) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: RTC I2C bit range [%u:%u] is empty\n",
                      high, low);
        mask = 0;
    } else {
        mask = MAKE_64BIT_MASK(low, high - low + 1);
    }

    *a4 &= ~R_SENS_SAR_SLAVE_ADDR4_I2C_DONE_MASK;
    *cycles = esp32_rtc_i2c_transfer(s->rtc_i2c, addr & 0x7f,
                                     extract32(ctrl, 0, 8), write,
                                     extract32(ctrl, 8, 8) & mask, &rdata);
    rdata = write ? 0 : rdata & mask;
    *a4 = FIELD_DP32(*a4, SENS_SAR_SLAVE_ADDR4, I2C_RDATA, rdata);
    s->i2c_busy = true;
    wait_start(&s->i2c_wait, s->fast_clk,
               qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), *cycles);
    return rdata;
}

/* ---- Registers ---- */

static uint64_t esp32_sens_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32SensState *s = ESP32_SENS(opaque);
    unsigned idx = addr / 4;

    if (addr >= ESP32_SENS_REG_COUNT * 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: read of reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
    switch (addr) {
    case A_SENS_SAR_READ_STATUS1:
    case A_SENS_SAR_READ_STATUS2:
    case A_SENS_SAR_TOUCH_OUT1 ... A_SENS_SAR_TOUCH_OUT5:
    case A_SENS_SAR_MEAS_START1:
    case A_SENS_SAR_MEAS_START2:
    case A_SENS_SAR_SLAVE_ADDR1 ... A_SENS_SAR_SLAVE_ADDR4:
    case A_SENS_SAR_TOUCH_CTRL2:
        return s->regs[idx];
    default:
        if (!reg_info[idx].writable) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_sens: read of reserved offset 0x%"
                          HWADDR_PRIx "\n", addr);
            return 0;
        }
        return s->regs[idx];
    }
}

static void esp32_sens_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
    Esp32SensState *s = ESP32_SENS(opaque);
    unsigned idx = addr / 4;
    uint32_t v = value;
    uint32_t old, writable;

    if (addr >= ESP32_SENS_REG_COUNT * 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: write to reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return;
    }
    old = s->regs[idx];
    writable = reg_info[idx].writable;

    switch (addr) {
    case A_SENS_SAR_READ_STATUS1:
    case A_SENS_SAR_READ_STATUS2:
    case A_SENS_SAR_TOUCH_OUT1 ... A_SENS_SAR_TOUCH_OUT5:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_sens: write to read-only register 0x%"
                      HWADDR_PRIx "\n", addr);
        return;

    case A_SENS_SAR_MEAS_START1:
    case A_SENS_SAR_MEAS_START2:
        s->regs[idx] = (v & writable) | (old & ~writable);
        if (FIELD_EX32(v, SENS_SAR_MEAS_START1, START_FORCE) &&
            FIELD_EX32(v & ~old, SENS_SAR_MEAS_START1, START_SAR)) {
            rtc_adc_start(s, addr == A_SENS_SAR_MEAS_START2);
        } else if (!FIELD_EX32(v, SENS_SAR_MEAS_START1, START_FORCE) &&
                   FIELD_EX32(v & ~old, SENS_SAR_MEAS_START1, START_SAR)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_sens: MEAS_START_SAR without "
                          "MEAS_START_FORCE, which leaves the start to the "
                          "ULP coprocessor, is ignored\n");
        }
        return;

    case A_SENS_SAR_SLAVE_ADDR1 ... A_SENS_SAR_SLAVE_ADDR4:
        s->regs[idx] = (v & writable) | (old & ~writable);
        return;

    case A_SENS_SAR_MEM_WR_CTRL:
        s->regs[idx] = v & writable & ~R_SENS_SAR_MEM_WR_CTRL_OFFST_CLR_MASK;
        return;

    case A_SENS_SAR_START_FORCE:
        s->regs[idx] = v & writable;
        qemu_set_irq(s->ulp_start,
                     FIELD_EX32(v, SENS_SAR_START_FORCE,
                                ULP_CP_FORCE_START_TOP) &&
                     FIELD_EX32(v, SENS_SAR_START_FORCE, ULP_CP_START_TOP));
        if (FIELD_EX32(v, SENS_SAR_START_FORCE, SAR2_EN_TEST)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_sens: ADC2's test input (SAR2_EN_TEST) is "
                          "not modelled\n");
        }
        return;

    case A_SENS_SAR_I2C_CTRL:
        s->regs[idx] = v & writable;
        if (FIELD_EX32(v, SENS_SAR_I2C_CTRL, START_FORCE) &&
            FIELD_EX32(v & ~old, SENS_SAR_I2C_CTRL, START)) {
            uint32_t cycles;

            if (s->i2c_busy) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "esp32_sens: SAR_I2C_START while an RTC I2C "
                              "transaction is in progress is ignored\n");
                return;
            }
            esp32_sens_rtc_i2c(s, FIELD_EX32(v, SENS_SAR_I2C_CTRL, CTRL),
                               &cycles);
        }
        return;

    case A_SENS_SAR_TSENS_CTRL:
        s->regs[idx] = v & writable;
        /*
         * [spec:nuos:req:emu.esp32.ulp]
         * Software's reading: TSENS_DUMP_OUT rising with the sensor
         * powered by TSENS_POWER_UP_FORCE and TSENS_POWER_UP.
         */
        if (FIELD_EX32(v & ~old, SENS_SAR_TSENS_CTRL, DUMP_OUT)) {
            if (FIELD_EX32(v, SENS_SAR_TSENS_CTRL, POWER_UP_FORCE) &&
                FIELD_EX32(v, SENS_SAR_TSENS_CTRL, POWER_UP)) {
                tsens_latch(s, tsens_code(s));
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "esp32_sens: TSENS_DUMP_OUT with the "
                              "temperature sensor not powered up\n");
            }
        }
        return;

    case A_SENS_SAR_TOUCH_CTRL2:
        s->regs[idx] = (v & writable) | (old & ~writable);
        if (FIELD_EX32(v, SENS_SAR_TOUCH_CTRL2, MEAS_EN_CLR)) {
            s->regs[idx] &= ~R_SENS_SAR_TOUCH_CTRL2_MEAS_EN_MASK;
        }
        if (FIELD_EX32(v, SENS_SAR_TOUCH_CTRL2, START_FORCE) &&
            FIELD_EX32(v & ~old, SENS_SAR_TOUCH_CTRL2, START_EN) &&
            (s->touch_phase == ESP32_TOUCH_IDLE ||
             s->touch_phase == ESP32_TOUCH_SLEEP)) {
            touch_start(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        touch_update_mode(s);
        touch_update_wakeup(s);
        return;

    case A_SENS_SAR_TOUCH_CTRL1:
    case A_SENS_SAR_TOUCH_ENABLE:
        s->regs[idx] = v & writable;
        touch_update_wakeup(s);
        return;

    case A_SENS_SAR_DAC_CTRL1:
        cw_reanchor(s);
        s->regs[idx] = v & writable;
        return;

    default:
        if (!writable) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_sens: write to reserved offset 0x%"
                          HWADDR_PRIx "\n", addr);
            return;
        }
        s->regs[idx] = v & writable;
        return;
    }
}

static const MemoryRegionOps esp32_sens_ops = {
    .read = esp32_sens_read,
    .write = esp32_sens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---- QOM ---- */

static void esp32_sens_get_dac(Object *obj, Visitor *v, const char *name,
                               void *opaque, Error **errp)
{
    Esp32SensState *s = ESP32_SENS(obj);
    /* An unrealized device has no RTCIO to power its DACs */
    uint32_t mv = s->rtcio ? dac_uv(s, (uintptr_t)opaque) / 1000 : 0;

    visit_type_uint32(v, name, &mv, errp);
}

static void esp32_sens_get_hall(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    visit_type_int32(v, name, &ESP32_SENS(obj)->hall_ut, errp);
}

static void esp32_sens_set_hall(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    visit_type_int32(v, name, &ESP32_SENS(obj)->hall_ut, errp);
}

static void esp32_sens_get_tsens(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    visit_type_int32(v, name, &ESP32_SENS(obj)->tsens_mc, errp);
}

static void esp32_sens_set_tsens(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    visit_type_int32(v, name, &ESP32_SENS(obj)->tsens_mc, errp);
}

static void esp32_sens_reset_hold(Object *obj, ResetType type)
{
    Esp32SensState *s = ESP32_SENS(obj);

    for (unsigned i = 0; i < ESP32_SENS_REG_COUNT; i++) {
        s->regs[i] = reg_info[i].reset;
    }
    for (unsigned u = 0; u < ESP32_ADC_UNITS; u++) {
        timer_del(&s->adc_wait[u].timer);
        s->adc_busy[u] = false;
        s->adc_result[u] = 0;
        s->patt_ptr[u] = 0;
    }
    s->alt_unit = 0;
    memset(s->ulp_pads, 0, sizeof(s->ulp_pads));
    timer_del(&s->i2c_wait.timer);
    s->i2c_busy = false;
    timer_del(&s->touch_wait.timer);
    s->touch_phase = ESP32_TOUCH_IDLE;
    s->cw_anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->cw_phase = 0;
    memset(s->dac_dma, 0, sizeof(s->dac_dma));
}

static void esp32_sens_reset_exit(Object *obj, ResetType type)
{
    Esp32SensState *s = ESP32_SENS(obj);

    s->touch_wakeup_level = false;
    qemu_set_irq(s->touch_wakeup, 0);
    qemu_set_irq(s->ulp_start, 0);
    touch_update_mode(s);
}

static void esp32_sens_realize(DeviceState *dev, Error **errp)
{
    Esp32SensState *s = ESP32_SENS(dev);

    if (!s->rtc_cntl || !s->rtcio || !s->gpio || !s->apb_ctrl ||
        !s->rtc_i2c) {
        error_setg(errp, "esp32_sens: rtc-cntl, rtcio, gpio, apb-ctrl and "
                   "rtc-i2c links must be set");
    }
}

static void esp32_sens_init(Object *obj)
{
    Esp32SensState *s = ESP32_SENS(obj);
    DeviceState *dev = DEVICE(obj);
    char name[24];

    memory_region_init_io(&s->iomem, obj, &esp32_sens_ops, s,
                          TYPE_ESP32_SENS, ESP32_SENS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    timer_init_ns(&s->adc_wait[0].timer, QEMU_CLOCK_VIRTUAL, adc1_timer_cb,
                  s);
    timer_init_ns(&s->adc_wait[1].timer, QEMU_CLOCK_VIRTUAL, adc2_timer_cb,
                  s);
    timer_init_ns(&s->touch_wait.timer, QEMU_CLOCK_VIRTUAL, touch_timer_cb,
                  s);
    timer_init_ns(&s->i2c_wait.timer, QEMU_CLOCK_VIRTUAL, i2c_timer_cb, s);
    s->fast_clk = qdev_init_clock_in(dev, ESP32_SENS_FAST_CLK,
                                     esp32_sens_fast_clk_update, s,
                                     ClockPreUpdate | ClockUpdate);
    s->slow_clk = qdev_init_clock_in(dev, ESP32_SENS_SLOW_CLK,
                                     esp32_sens_slow_clk_update, s,
                                     ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_out_named(dev, &s->ulp_start, ESP32_SENS_ULP_START, 1);
    qdev_init_gpio_out_named(dev, &s->touch_int, ESP32_SENS_TOUCH_INT, 1);
    qdev_init_gpio_out_named(dev, &s->touch_wakeup, ESP32_SENS_TOUCH_WAKEUP,
                             1);
    qdev_init_gpio_in_named(dev, esp32_sens_touch_timer,
                            ESP32_SENS_TOUCH_TIMER_IN, 1);
    qdev_init_gpio_in_named(dev, esp32_sens_saradc_ctrl,
                            ESP32_SENS_SARADC_CTRL_IN, 1);

    for (unsigned u = 0; u < ESP32_ADC_UNITS; u++) {
        for (unsigned ch = 0; ch < adc_channels[u]; ch++) {
            snprintf(name, sizeof(name), "adc%u-ch%u-mv", u + 1, ch);
            object_property_add_uint32_ptr(obj, name, &s->adc_mv[u][ch],
                                           OBJ_PROP_FLAG_READWRITE);
        }
    }
    for (unsigned n = 0; n < ESP32_TOUCH_PADS; n++) {
        s->touch_ff[n] = TOUCH_DEFAULT_FF;
        snprintf(name, sizeof(name), "touch%u-ff", n);
        object_property_add_uint32_ptr(obj, name, &s->touch_ff[n],
                                       OBJ_PROP_FLAG_READWRITE);
    }
    object_property_add(obj, "hall-field-ut", "int32", esp32_sens_get_hall,
                        esp32_sens_set_hall, NULL, NULL);
    s->tsens_mc = 25000;
    object_property_add(obj, "tsens-temp-mc", "int32", esp32_sens_get_tsens,
                        esp32_sens_set_tsens, NULL, NULL);
    for (uintptr_t d = 0; d < ESP32_DAC_CHANNELS; d++) {
        snprintf(name, sizeof(name), "dac%u-mv", (unsigned)d + 1);
        object_property_add(obj, name, "uint32", esp32_sens_get_dac, NULL,
                            NULL, (void *)d);
    }
}

static const Property esp32_sens_properties[] = {
    DEFINE_PROP_LINK("rtc-cntl", Esp32SensState, rtc_cntl,
                     TYPE_ESP32_RTC_CNTL, Esp32RtcCntlState *),
    DEFINE_PROP_LINK("rtcio", Esp32SensState, rtcio, TYPE_ESP32_RTCIO,
                     Esp32RtcIoState *),
    DEFINE_PROP_LINK("gpio", Esp32SensState, gpio, TYPE_ESP32_GPIO,
                     Esp32GpioState *),
    DEFINE_PROP_LINK("apb-ctrl", Esp32SensState, apb_ctrl,
                     TYPE_ESP32_APB_CTRL, Esp32ApbCtrlState *),
    DEFINE_PROP_LINK("rtc-i2c", Esp32SensState, rtc_i2c,
                     TYPE_ESP32_RTC_I2C, Esp32RtcI2cState *),
};

static const VMStateDescription vmstate_esp32_sens_wait = {
    .name = TYPE_ESP32_SENS "/wait",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, Esp32SensWait),
        VMSTATE_INT64(base_ns, Esp32SensWait),
        VMSTATE_UINT64(left, Esp32SensWait),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32_sens = {
    .name = TYPE_ESP32_SENS,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(fast_clk, Esp32SensState),
        VMSTATE_CLOCK(slow_clk, Esp32SensState),
        VMSTATE_UINT32_ARRAY(regs, Esp32SensState, ESP32_SENS_REG_COUNT),
        VMSTATE_UINT32_ARRAY(ulp_pads, Esp32SensState, ESP32_ADC_UNITS),
        VMSTATE_STRUCT(i2c_wait, Esp32SensState, 1, vmstate_esp32_sens_wait,
                       Esp32SensWait),
        VMSTATE_BOOL(i2c_busy, Esp32SensState),
        VMSTATE_STRUCT_ARRAY(adc_wait, Esp32SensState, ESP32_ADC_UNITS, 1,
                             vmstate_esp32_sens_wait, Esp32SensWait),
        VMSTATE_BOOL_ARRAY(adc_busy, Esp32SensState, ESP32_ADC_UNITS),
        VMSTATE_UINT32_ARRAY(adc_result, Esp32SensState, ESP32_ADC_UNITS),
        VMSTATE_UINT8_ARRAY(patt_ptr, Esp32SensState, ESP32_ADC_UNITS),
        VMSTATE_UINT8(alt_unit, Esp32SensState),
        VMSTATE_STRUCT(touch_wait, Esp32SensState, 1,
                       vmstate_esp32_sens_wait, Esp32SensWait),
        VMSTATE_INT64(touch_meas_ns, Esp32SensState),
        VMSTATE_UINT32(touch_phase, Esp32SensState),
        VMSTATE_BOOL(touch_timer_en, Esp32SensState),
        VMSTATE_BOOL(touch_wakeup_level, Esp32SensState),
        VMSTATE_INT64(cw_anchor_ns, Esp32SensState),
        VMSTATE_UINT32(cw_phase, Esp32SensState),
        VMSTATE_UINT8_ARRAY(dac_dma, Esp32SensState, ESP32_DAC_CHANNELS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32_sens_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_sens_reset_hold;
    rc->phases.exit = esp32_sens_reset_exit;
    dc->realize = esp32_sens_realize;
    dc->vmsd = &vmstate_esp32_sens;
    device_class_set_props(dc, esp32_sens_properties);
}

/* [spec:nuos:req:emu.esp32.analog] */
/* [spec:nuos:req:emu.esp32.ulp] */
static const TypeInfo esp32_sens_info = {
    .name = TYPE_ESP32_SENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SensState),
    .instance_init = esp32_sens_init,
    .class_init = esp32_sens_class_init,
};

static void esp32_sens_register_types(void)
{
    type_register_static(&esp32_sens_info);
}

type_init(esp32_sens_register_types)
