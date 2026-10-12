/*
 * ESP32-P4 capacitive touch sensor (LP_TOUCH)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 63 (Touch Sensor); ESP-IDF's touch_reg.h, lp_analog_peri_reg.h and
 * pmu_reg.h for v3, the authority for the registers, their reset values
 * and access types; its hal touch_sensor_ll.h, touch_sens_hal.c and
 * esp_driver_touch_sens (hw_ver3), which show how software drives the
 * sensor.
 *
 * The sensor's configuration is LP_ANALOG_PERI's TOUCH_* registers, its
 * timer PMU_TOUCH_PWR_CNTL, and its status, data and interrupts RTC_TOUCH,
 * this device's registers.
 *
 * Measuring. A START measures one channel, at each frequency mode: mode
 * 0 alone, or, with FREQ_SCAN_EN, modes 0 to FREQ_SCAN_CNT_LIMIT - 1.
 * The FSM, clocked by LP_DYN_FAST_CLK, powers the channel up and waits
 * TOUCH_XPD_WAIT cycles, as it does again between the modes (TRM
 * 63.4.2.4), then counts LP_DYN_FAST_CLK cycles until the pad's
 * charge-discharge pulses, divided by DIV_NUMf + 1 (as ESP-IDF's
 * touch_ll_set_clock_div() has it; the TRM's description of the field
 * describes no division), number TOUCH_MEAS_NUMf (TOUCH_APPROACH_MEAS
 * _NUMf for a proximity channel). The pulses' period is the pad's
 * capacitance over the driver's current, which the TRM does not give: the
 * model takes 8 uA a step of TOUCH_FREQf_DRV_HS, plus one, and 1 uA a
 * step of _DRV_LS, so that a 10 pF pad gives ESP-IDF's sample
 * configurations counts of some thousands; TOUCH_FREQf_DBIAS and the RC
 * filter (DRES_LPF, DCAP_LPF) change nothing. With TOUCH_EN_CAL the sensor
 * measures its own capacitor, TOUCH_DCAP_CAL 20 fF steps, instead of the
 * pad. Without TOUCH_OUT_GATE no pulse is counted, nor on a channel the
 * software start leaves unpowered (TOUCH_XPD); with TOUCH_TIMEOUT_EN the
 * measurement stops when the count reaches TOUCH_TIMEOUT_NUM, raising
 * TOUCH_TIMEOUT_INT, and gives no data and no done. The TRM's two
 * counting methods (TOUCH_OUT_SEL) count the same here.
 *
 * Which channel: with TOUCH_FSM_EN, the next channel of
 * TOUCH_SCAN_PAD_MAP, round, skipping channel 0, which has no pad, and,
 * with TOUCH_SHIELD_PAD_EN, the shield channels TOUCH_BUFSEL drives;
 * without it, the channel TOUCH_START selects (the lowest if several),
 * powered by TOUCH_XPD. In LP_SLEEP (the PMU's "lp-sleep"), with
 * TOUCH_SLP_PAD naming a channel, only that channel, as the sleep
 * channel, with its own data and thresholds (TRM 63.4.5). The START comes
 * from software, a rising TOUCH_START_EN with TOUCH_START_FORCE, or from
 * the touch timer without it. The timer, counting LP_DYN_SLOW_CLK while
 * PMU_TOUCH_SLEEP_TIMER_EN is set, waits PMU_TOUCH_WAIT_CYCLES, starts a
 * measurement, waits for its done (the FSM's, or with TOUCH_DONE_FORCE a
 * rising TOUCH_DONE_EN, or PMU_TOUCH_FORCE_DONE, which also stops the
 * measurement it started), then waits PMU_TOUCH_SLEEP_CYCLES, and again.
 * TOUCH_MEAS_DONE clears as a measurement starts and sets as it ends;
 * TOUCH_SCAN_CURR names the channel measured or last measured.
 *
 * Data. Each channel has, per mode, its raw count, its smoothed data
 * (TOUCH_SMOOTH_LVL's IIR filter, table 63.4-1) and its benchmark: the
 * first measurement after a reset or TOUCH_CHANNEL_CLR sets both; then,
 * with TOUCH_FILTER_EN, while the channel is not touched, the benchmark
 * follows the raw count (TOUCH_FILTER_MODE's IIR filter or jitter step,
 * table 63.4-2) while the smoothed data stays within the noise thresholds
 * of it, TOUCH_NOISE_THRES above (4/8, 3/8, 2/8 or 8/8 of the channel's
 * threshold, as ESP-IDF's denoise levels number them) and TOUCH_NN_THRES
 * below; above, it holds; below for TOUCH_NN_LIMIT measurements, it is
 * reset to the smoothed data, unless TOUCH_NN_DISUPDATE_BENCHMARK_EN.
 * TOUCH_BYPASS_NOISE_THRES and _NN_THRES lift the limits. Without the
 * filter the smoothed data is the raw count and the benchmark holds.
 * Software can set a benchmark (TOUCH_UPDATE_BENCHMARK_SW), raising
 * TOUCH_BENCHMARK_UPDATE_INT. RTC_TOUCH_PADn_DATA reads channel n's raw
 * count (TOUCH_DATA_SEL 0 or 1), benchmark (2) or smoothed data (3) of
 * mode TOUCH_FREQ_SEL; the sleep channel's (TOUCH_SLP_DATA) of mode
 * TOUCH_FREQ_SEL - 1, as ESP-IDF's touch_ll reads them.
 *
 * Detection (TRM 63.4.3.2). After all modes, a mode is touched where its
 * smoothed data exceeds the benchmark by more than its threshold
 * (TOUCH_PADn_THf; TOUCH_SLP_THf for the sleep channel) plus the
 * hysteresis (TOUCH_HYSTERESIS: 4/32, 3/32, 2/32 of the threshold, or
 * none), released where by less than the threshold less the hysteresis.
 * The channel becomes touched when FREQ_SCAN_CNT_RISE modes or more are
 * touched for TOUCH_DEBOUNCE_LIMIT measurements running, and released
 * when too few are left for that as long, raising TOUCH_ACTIVE_INT and
 * TOUCH_INACTIVE_INT and setting its RTC_TOUCH_PAD_ACTIVE bit while
 * TOUCH_OUTEN enables its output; TOUCH_STATUS_CLR clears the touches.
 * RTC_TOUCH_CHN_TMP_STATUS shows each channel's last comparison before
 * debouncing. A touched channel with its output enabled, or a touched
 * sleep channel, wakes the chip (the PMU's source 11).
 *
 * Proximity (TRM 63.4.4): a channel TOUCH_APPROACH_PADx names (the sleep
 * channel with TOUCH_SLP_APPROACH_EN) sums its counts; after
 * TOUCH_APPROACH_LIMIT measurements each mode's sum, at most 0xFFFF,
 * becomes its benchmark, where ESP-IDF reads it, raising
 * TOUCH_APPROACH_LOOP_DONE_INT as that mode completes, and the channel is
 * touched where FREQ_SCAN_CNT_RISE modes' sums exceed their thresholds.
 *
 * Interrupts: TOUCH_DONE_INT as each mode's measurement completes;
 * TOUCH_SCAN_DONE_INT as the last channel of TOUCH_SCAN_PAD_MAP
 * completes, as ESP-IDF's on_scan_done has it, or each software or
 * sleep-channel measurement.
 *
 * Every timed event (the timer's phases, a channel's power-up wait and
 * a measurement's end or timeout) happens at its own time, however late
 * the QEMU timer that runs it: it runs as of that time
 * (esp32_line_run_at()), so that the lines it changes, the PMU's wakeup
 * among them, carry it, and what it times from there it times from that.
 * Register accesses (RTC_TOUCH's, LP_ANALOG_PERI's TOUCH_* and the touch
 * timer's PMU_TOUCH_PWR_CNTL), clock changes, the PMU's sleep and the
 * reset first take the events that fell due before them, in time order.
 *
 * Not modelled, and without effect here: the shield's drive
 * (TOUCH_BUF_DRV), the guard ring (TOUCH_OUT_RING), whose water rejection
 * ESP-IDF does in software, idle channels' connection, and
 * TOUCH_OUT_RESET.
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
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_lp_ana.h"
#include "hw/misc/esp32p4_pmu.h"
#include "hw/misc/esp32p4_touch.h"
#include "migration/vmstate.h"

/* RTC_TOUCH */
#define A_INT_RAW               0x000
#define A_INT_ST                0x004
#define A_INT_ENA               0x008
#define A_INT_CLR               0x00c
#define A_CHN_STATUS            0x010
#define A_STATUS(n)             (0x014 + 4 * (n))
#define A_SLP_STATUS            0x050
#define A_APRCH_STATUS          0x054
#define A_PARA_STATUS           0x058
#define A_CHN_TMP_STATUS        0x05c
#define A_DATE                  0x100

#define INT_SCAN_DONE           BIT(0)
#define INT_DONE                BIT(1)
#define INT_ACTIVE              BIT(2)
#define INT_INACTIVE            BIT(3)
#define INT_TIMEOUT             BIT(4)
#define INT_APPROACH_LOOP_DONE  BIT(5)
#define INT_BENCHMARK_UPDATE    BIT(6)
#define INT_ALL                 0x7fu
#define CHN_MEAS_DONE           BIT(15)
#define DATE_CLK_EN             BIT(31)

/* LP_ANALOG_PERI's TOUCH_* registers */
#define A_APPROACH_MEAS_NUM     0x0fc
#define A_SCAN_CTRL1            0x100
#define A_SCAN_CTRL2            0x104
#define A_WORK                  0x108
#define A_WORK_MEAS_NUM         0x10c
#define A_FILTER1               0x110
#define A_FILTER2               0x114
#define A_FILTER3               0x118
#define A_SLP0                  0x11c
#define A_SLP1                  0x120
#define A_CLR                   0x124
#define A_APPROACH              0x128
#define A_FREQ_PARA(f)          (0x12c + 4 * (f))
#define A_ANA_PARA              0x138
#define A_MUX0                  0x13c
#define A_MUX1                  0x140
#define A_PAD_TH(n, f)          (0x144 + 12 * (n) + 4 * (f))
#define A_CTRL                  0x1fc

/* Per mode f: MEAS_NUMf in bits 29:20, 19:10, 9:0; DIV_NUMf 24:22 ... */
#define MEAS_NUM(v, f)          extract32((v), 20 - 10 * (f), 10)
#define WORK_DIV_NUM(v, f)      extract32((v), 22 - 3 * (f), 3)
#define WORK_OUT_GATE           BIT(27)
#define SCAN1_SHIELD_PAD_EN     BIT(0)
#define SCAN1_PAD_MAP(v)        extract32((v), 2, 15)
#define SCAN1_XPD_WAIT(v)       extract32((v), 17, 15)
#define SCAN2_TIMEOUT_NUM(v)    extract32((v), 6, 16)
#define SCAN2_TIMEOUT_EN        BIT(22)
#define SCAN2_FREQ_SCAN_EN      BIT(27)
#define SCAN2_FREQ_CNT_LIMIT(v) extract32((v), 28, 2)
#define F1_NN_DISUPDATE         BIT(0)
#define F1_HYSTERESIS(v)        extract32((v), 1, 2)
#define F1_NN_THRES(v)          extract32((v), 3, 2)
#define F1_NOISE_THRES(v)       extract32((v), 5, 2)
#define F1_SMOOTH_LVL(v)        extract32((v), 7, 2)
#define F1_JITTER_STEP(v)       extract32((v), 9, 4)
#define F1_FILTER_MODE(v)       extract32((v), 13, 3)
#define F1_FILTER_EN            BIT(16)
#define F1_NN_LIMIT(v)          extract32((v), 17, 4)
#define F1_APPROACH_LIMIT(v)    extract32((v), 21, 8)
#define F1_DEBOUNCE_LIMIT(v)    extract32((v), 29, 3)
#define F2_OUTEN(v)             extract32((v), 15, 15)
#define F2_BYPASS_NOISE         BIT(30)
#define F2_BYPASS_NN            BIT(31)
#define F3_BENCHMARK_SW(v)      extract32((v), 0, 16)
#define F3_UPDATE_BENCHMARK_SW  BIT(16)
#define SLP0_TH0(v)             extract32((v), 0, 16)
#define SLP0_CHANNEL_CLR        BIT(16)
#define SLP0_PAD(v)             extract32((v), 17, 4)
#define SLP1_TH(v, f)           extract32((v), (f) == 1 ? 16 : 0, 16)
#define CLR_CHANNEL_CLR(v)      extract32((v), 0, 15)
#define CLR_STATUS_CLR          BIT(15)
#define APPROACH_PAD(v, x)      extract32((v), 4 * (x), 4)
#define APPROACH_SLP_EN         BIT(12)
#define PARA_DRV_LS(v)          extract32((v), 9, 4)
#define PARA_DRV_HS(v)          extract32((v), 13, 5)
#define ANA_EN_CAL              BIT(3)
#define ANA_DCAP_CAL(v)         extract32((v), 4, 7)
#define MUX0_DATA_SEL(v)        extract32((v), 8, 2)
#define MUX0_FREQ_SEL(v)        extract32((v), 10, 2)
#define MUX0_BUFSEL(v)          extract32((v), 12, 15)
#define MUX0_DONE_EN            BIT(27)
#define MUX0_DONE_FORCE         BIT(28)
#define MUX0_FSM_EN             BIT(29)
#define MUX0_START_EN           BIT(30)
#define MUX0_START_FORCE        BIT(31)
#define MUX1_START(v)           extract32((v), 0, 15)
#define MUX1_XPD(v)             extract32((v), 15, 15)
#define PAD_TH(v)               extract32((v), 16, 16)
#define CTRL_BM_FREQ_SEL(v)     extract32((v), 0, 2)
#define CTRL_BM_PAD_SEL(v)      extract32((v), 2, 4)
#define CTRL_CNT_RISE(v)        extract32((v), 6, 2)

/* PMU_TOUCH_PWR_CNTL */
#define PWR_WAIT_CYCLES(v)      extract32((v), 5, 9)
#define PWR_SLEEP_CYCLES(v)     extract32((v), 14, 16)
#define PWR_FORCE_DONE          BIT(30)
#define PWR_SLEEP_TIMER_EN      BIT(31)

/* The data's fractional bits. */
#define FRAC                    8
#define DATA_MAX                0xffffu
/* The driver's current per DRV_HS and DRV_LS step, in uA. */
#define DRV_HS_UA               8.0
#define DRV_LS_UA               1.0
#define DCAP_CAL_FF             20
#define DEFAULT_PAD_FF          10000

static uint32_t touch_ana(ESP32P4TouchState *s, hwaddr offset)
{
    return esp32p4_lp_ana_touch_reg(s->lp_ana, offset);
}

static void touch_update_irq(ESP32P4TouchState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

static void touch_raise(ESP32P4TouchState *s, uint32_t bits)
{
    s->int_raw |= bits;
    touch_update_irq(s);
}

static unsigned touch_slp_pad(ESP32P4TouchState *s)
{
    return SLP0_PAD(touch_ana(s, A_SLP0));
}

static bool touch_valid_chan(unsigned ch)
{
    return ch >= ESP32P4_TOUCH_FIRST && ch < ESP32P4_TOUCH_SLOTS;
}

/* The channels whose touches the sensor outputs: PAD_ACTIVE. */
static uint32_t touch_pad_active(ESP32P4TouchState *s)
{
    uint32_t outen = F2_OUTEN(touch_ana(s, A_FILTER2));
    uint32_t active = 0;
    unsigned slp = touch_slp_pad(s);

    for (unsigned ch = 0; ch < ESP32P4_TOUCH_SLOTS; ch++) {
        if (s->chan[ch].active && (outen & BIT(ch))) {
            active |= BIT(ch);
        }
    }
    if (s->slp.active && touch_valid_chan(slp)) {
        active |= BIT(slp);
    }
    return active;
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_update_wakeup(ESP32P4TouchState *s)
{
    bool level = touch_pad_active(s) != 0;

    if (level != s->wakeup_level) {
        s->wakeup_level = level;
        qemu_set_irq(s->wakeup, level);
    }
}

static ESP32P4TouchChan *touch_cur(ESP32P4TouchState *s)
{
    return s->cur_slp ? &s->slp : &s->chan[s->cur_ch];
}

/* The proximity channel slot @ch is, or -1. */
static int touch_prox_of(ESP32P4TouchState *s, unsigned ch, bool slp)
{
    uint32_t ap = touch_ana(s, A_APPROACH);

    if (slp) {
        return (ap & APPROACH_SLP_EN) ? 0 : -1;
    }
    for (int x = 0; x < ESP32P4_TOUCH_PROX; x++) {
        if (APPROACH_PAD(ap, x) == ch) {
            return x;
        }
    }
    return -1;
}

static uint32_t touch_threshold(ESP32P4TouchState *s, unsigned ch, bool slp,
                                unsigned f)
{
    if (slp) {
        return f ? SLP1_TH(touch_ana(s, A_SLP1), f)
                 : SLP0_TH0(touch_ana(s, A_SLP0));
    }
    return PAD_TH(touch_ana(s, A_PAD_TH(ch, f)));
}

static unsigned touch_nfreq(ESP32P4TouchState *s)
{
    uint32_t c2 = touch_ana(s, A_SCAN_CTRL2);

    if (!(c2 & SCAN2_FREQ_SCAN_EN)) {
        return 1;
    }
    return MIN(MAX(SCAN2_FREQ_CNT_LIMIT(c2), 1), ESP32P4_TOUCH_FREQS);
}

static unsigned touch_rise(ESP32P4TouchState *s, unsigned nf)
{
    return MIN(MAX(CTRL_CNT_RISE(touch_ana(s, A_CTRL)), 1), nf);
}

/* ---- Detection ---- */

/* The noise thresholds' fractions of a channel's threshold, in eighths. */
static const uint8_t touch_noise_eighths[4] = { 4, 3, 2, 8 };
/* The hysteresis's, in 32nds. */
static const uint8_t touch_hys_32nds[4] = { 4, 3, 2, 0 };

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * Mode @f of channel @c measured @raw: its smoothed data and benchmark.
 */
static void touch_filter(ESP32P4TouchState *s, ESP32P4TouchChan *c,
                         unsigned f, uint32_t raw, uint32_t thr)
{
    uint32_t f1 = touch_ana(s, A_FILTER1), f2 = touch_ana(s, A_FILTER2);
    int64_t x = (int64_t)raw << FRAC;
    int64_t diff;

    c->raw[f] = raw;
    if (!c->bench_valid[f]) {
        c->smooth[f] = x;
        c->bench[f] = x;
        c->bench_valid[f] = true;
        return;
    }
    if (!(f1 & F1_FILTER_EN)) {
        c->smooth[f] = x;
        return;
    }
    if (F1_SMOOTH_LVL(f1)) {
        c->smooth[f] += (x - (int64_t)c->smooth[f]) >> F1_SMOOTH_LVL(f1);
    } else {
        c->smooth[f] = x;
    }
    if (c->active) {
        return;
    }
    diff = ((int64_t)c->smooth[f] - c->bench[f]) >> FRAC;
    if (!(f2 & F2_BYPASS_NOISE) &&
        diff > (int64_t)thr * touch_noise_eighths[F1_NOISE_THRES(f1)] / 8) {
        return;
    }
    if (!(f2 & F2_BYPASS_NN) &&
        -diff > (int64_t)thr * touch_noise_eighths[F1_NN_THRES(f1)] / 8) {
        c->nn_cnt = MIN(c->nn_cnt + 1, 15);
        if (c->nn_cnt >= MAX(F1_NN_LIMIT(f1), 1)) {
            if (!(f1 & F1_NN_DISUPDATE)) {
                c->bench[f] = c->smooth[f];
            }
            c->nn_cnt = 0;
        }
        return;
    }
    c->nn_cnt = 0;
    if (F1_FILTER_MODE(f1) == 7) {
        int64_t step = (int64_t)F1_JITTER_STEP(f1) << FRAC;
        int64_t d = x - c->bench[f];

        c->bench[f] += d > 0 ? MIN(d, step) : MAX(d, -step);
    } else {
        c->bench[f] += (x - (int64_t)c->bench[f]) >>
                       (2 + F1_FILTER_MODE(f1));
    }
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_set_active(ESP32P4TouchState *s, ESP32P4TouchChan *c,
                             unsigned ch, bool slp, bool active)
{
    bool out = slp || (F2_OUTEN(touch_ana(s, A_FILTER2)) & BIT(ch));

    if (active == c->active) {
        return;
    }
    c->active = active;
    if (out) {
        touch_raise(s, active ? INT_ACTIVE : INT_INACTIVE);
    }
    touch_update_wakeup(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The channel measured at all modes: the touch decision (see the top of
 * the file).
 */
static void touch_detect(ESP32P4TouchState *s, ESP32P4TouchChan *c,
                         unsigned ch, bool slp)
{
    uint32_t f1 = touch_ana(s, A_FILTER1);
    unsigned nf = s->nfreq, rise = touch_rise(s, nf);
    unsigned hits = 0, rels = 0;
    unsigned limit = MAX(F1_DEBOUNCE_LIMIT(f1), 1);
    bool hit, rel;

    for (unsigned f = 0; f < nf; f++) {
        int64_t thr = touch_threshold(s, ch, slp, f);
        int64_t hys = thr * touch_hys_32nds[F1_HYSTERESIS(f1)] / 32;
        int64_t diff = ((int64_t)c->smooth[f] >> FRAC) -
                       ((int64_t)c->bench[f] >> FRAC);

        hits += diff > thr + hys;
        rels += diff < thr - hys;
    }
    hit = hits >= rise;
    rel = rels > nf - rise;
    if (!slp) {
        s->tmp_active = deposit32(s->tmp_active, ch, 1, hit);
        s->tmp_inactive = deposit32(s->tmp_inactive, ch, 1, rel);
    }
    if (c->active ? rel : hit) {
        c->debounce = MIN(c->debounce + 1, 7);
        if (c->debounce >= limit) {
            c->debounce = 0;
            touch_set_active(s, c, ch, slp, !c->active);
        }
    } else {
        c->debounce = 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_proximity(ESP32P4TouchState *s, ESP32P4TouchChan *c,
                            unsigned ch, bool slp, unsigned f, uint32_t raw)
{
    uint32_t limit = MAX(F1_APPROACH_LIMIT(touch_ana(s, A_FILTER1)), 1);

    c->raw[f] = raw;
    c->smooth[f] = raw << FRAC;
    c->prox_sum[f] = MIN((uint64_t)c->prox_sum[f] + raw, UINT32_MAX);
    if (f == 0) {
        c->prox_cnt++;
    }
    if (c->prox_cnt < limit) {
        return;
    }
    c->bench[f] = MIN(c->prox_sum[f], DATA_MAX) << FRAC;
    c->bench_valid[f] = true;
    c->prox_sum[f] = 0;
    touch_raise(s, INT_APPROACH_LOOP_DONE);
    if (f == s->nfreq - 1) {
        unsigned hits = 0;

        c->prox_cnt = 0;
        for (unsigned g = 0; g < s->nfreq; g++) {
            hits += (c->bench[g] >> FRAC) > touch_threshold(s, ch, slp, g);
        }
        touch_set_active(s, c, ch, slp, hits >= touch_rise(s, s->nfreq));
    }
}

/* ---- The FSM ---- */

static bool touch_counting(ESP32P4TouchState *s)
{
    return s->fsm_phase == ESP32P4_TOUCH_FSM_MEAS;
}

/* The cycles of LP_DYN_FAST_CLK counted up to @t. */
static uint64_t touch_count_at(ESP32P4TouchState *s, int64_t t)
{
    return s->meas_count + esp32p4_clk_count_at(&s->meas_edges, t);
}


/* Arm the measurement's timer: at its end, or at its timeout if sooner. */
static void touch_meas_arm(ESP32P4TouchState *s)
{
    uint32_t c2 = touch_ana(s, A_SCAN_CTRL2);
    int64_t due = s->meas_end_ns;

    if ((c2 & SCAN2_TIMEOUT_EN) && clock_is_enabled(s->fast)) {
        uint64_t limit = SCAN2_TIMEOUT_NUM(c2);
        int64_t t = s->meas_edges.origin_ns;

        if (s->meas_count < limit) {
            t = esp32p4_clk_count_time(&s->meas_edges,
                                       limit - s->meas_count);
        }
        due = MIN(due, t);
    }
    s->meas_due_ns = due;
    if (due == INT64_MAX) {
        timer_del(&s->meas_timer);
    } else {
        timer_mod(&s->meas_timer, due);
    }
}

static void touch_abort(ESP32P4TouchState *s)
{
    esp32p4_clkwait_stop(&s->xpd_wait);
    timer_del(&s->meas_timer);
    s->fsm_phase = ESP32P4_TOUCH_FSM_IDLE;
}

static void touch_timer_set(ESP32P4TouchState *s, int phase, uint32_t cycles,
                            int64_t when)
{
    s->timer_phase = phase;
    if (phase == ESP32P4_TOUCH_TIMER_WAIT ||
        phase == ESP32P4_TOUCH_TIMER_SLEEP) {
        esp32p4_clkwait_start(&s->timer_wait, when, cycles);
    } else {
        esp32p4_clkwait_stop(&s->timer_wait);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The touch timer's done: it sleeps PMU_TOUCH_SLEEP_CYCLES.
 */
static void touch_timer_done(ESP32P4TouchState *s, int64_t when)
{
    if (s->timer_phase == ESP32P4_TOUCH_TIMER_MEAS) {
        touch_timer_set(s, ESP32P4_TOUCH_TIMER_SLEEP,
                        PWR_SLEEP_CYCLES(s->pwr_cntl), when);
    }
}

/* Whether the channel the FSM measures is powered up. */
static bool touch_powered(ESP32P4TouchState *s)
{
    if (s->cur_slp || s->cur_scan) {
        return true;
    }
    return MUX1_XPD(touch_ana(s, A_MUX1)) & BIT(s->cur_ch);
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * Measure the current mode: count LP_DYN_FAST_CLK until the divided
 * pulses number MEAS_NUM (see the top of the file).
 */
static void touch_meas_begin(ESP32P4TouchState *s, int64_t when)
{
    unsigned f = s->cur_freq;
    uint32_t para = touch_ana(s, A_FREQ_PARA(f));
    uint32_t work = touch_ana(s, A_WORK);
    uint32_t ana = touch_ana(s, A_ANA_PARA);
    bool prox = touch_prox_of(s, s->cur_ch, s->cur_slp) >= 0;
    uint32_t n = MEAS_NUM(touch_ana(s, prox ? A_APPROACH_MEAS_NUM
                                            : A_WORK_MEAS_NUM), f);
    uint32_t div = WORK_DIV_NUM(work, f) + 1;
    double cap = (ana & ANA_EN_CAL) ? ANA_DCAP_CAL(ana) * DCAP_CAL_FF
                                    : s->cap_ff[s->cur_ch];
    double ua = DRV_HS_UA * (PARA_DRV_HS(para) + 1) +
                DRV_LS_UA * PARA_DRV_LS(para);

    s->cur_para = para;
    s->fsm_phase = ESP32P4_TOUCH_FSM_MEAS;
    esp32p4_clk_count_start(&s->meas_edges, s->fast, when);
    s->meas_count = 0;
    if (!(work & WORK_OUT_GATE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TOUCH_OUT_GATE stops the touch "
                      "clock: channel %u's measurement counts no pulse\n",
                      TYPE_ESP32P4_TOUCH, s->cur_ch);
        s->meas_end_ns = INT64_MAX;
    } else if (!touch_powered(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %u is measured with its "
                      "TOUCH_XPD bit clear: no pulse comes\n",
                      TYPE_ESP32P4_TOUCH, s->cur_ch);
        s->meas_end_ns = INT64_MAX;
    } else {
        s->meas_end_ns = when + llround((double)n * div * cap / ua);
    }
    touch_meas_arm(s);
}

static void touch_xpd_run(void *opaque)
{
    ESP32P4TouchState *s = opaque;

    touch_meas_begin(s, esp32p4_clkwait_expire(&s->xpd_wait));
}

/* Power the channel up for the next mode: TOUCH_XPD_WAIT, then measure. */
static void touch_next(ESP32P4TouchState *s, int64_t when)
{
    uint32_t wait = SCAN1_XPD_WAIT(touch_ana(s, A_SCAN_CTRL1));

    s->fsm_phase = ESP32P4_TOUCH_FSM_XPD;
    if (wait) {
        esp32p4_clkwait_start(&s->xpd_wait, when, wait);
    } else {
        touch_meas_begin(s, when);
    }
}

/* The channels the FSM scans. */
static uint32_t touch_scan_map(ESP32P4TouchState *s)
{
    uint32_t map = SCAN1_PAD_MAP(touch_ana(s, A_SCAN_CTRL1)) & ~1u;

    if (touch_ana(s, A_SCAN_CTRL1) & SCAN1_SHIELD_PAD_EN) {
        map &= ~MUX0_BUFSEL(touch_ana(s, A_MUX0));
    }
    return map;
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_finish(ESP32P4TouchState *s, int64_t when)
{
    uint32_t map = touch_scan_map(s);

    s->fsm_phase = ESP32P4_TOUCH_FSM_IDLE;
    s->meas_done = true;
    if (touch_prox_of(s, s->cur_ch, s->cur_slp) < 0) {
        touch_detect(s, touch_cur(s), s->cur_ch, s->cur_slp);
    }
    if (!s->cur_scan || !map || s->cur_ch == 31 - clz32(map)) {
        touch_raise(s, INT_SCAN_DONE);
    }
    if (!(touch_ana(s, A_MUX0) & MUX0_DONE_FORCE)) {
        touch_timer_done(s, when);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * The counter has reached its end: a mode measured, or the timeout.
 */
static void touch_meas_run(void *opaque)
{
    ESP32P4TouchState *s = opaque;
    int64_t when = s->meas_due_ns;
    uint64_t count;
    ESP32P4TouchChan *c = touch_cur(s);
    unsigned f = s->cur_freq;
    uint32_t raw;

    count = touch_count_at(s, when);
    if (when < s->meas_end_ns) {
        s->fsm_phase = ESP32P4_TOUCH_FSM_IDLE;
        touch_raise(s, INT_TIMEOUT);
        return;
    }
    raw = MIN(count, DATA_MAX);
    if (touch_prox_of(s, s->cur_ch, s->cur_slp) >= 0) {
        touch_proximity(s, c, s->cur_ch, s->cur_slp, f, raw);
    } else {
        touch_filter(s, c, f, raw,
                     touch_threshold(s, s->cur_ch, s->cur_slp, f));
    }
    touch_raise(s, INT_DONE);
    if (f + 1 < s->nfreq) {
        s->cur_freq++;
        touch_next(s, when);
    } else {
        touch_finish(s, when);
    }
}

/* The next channel of the scan after the last one, or -1. */
static int touch_next_scan(ESP32P4TouchState *s)
{
    uint32_t map = touch_scan_map(s);

    for (unsigned i = 1; i <= ESP32P4_TOUCH_SLOTS; i++) {
        unsigned ch = (s->scan_last + i) % ESP32P4_TOUCH_SLOTS;

        if (map & BIT(ch)) {
            return ch;
        }
    }
    return -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * A START, from software or the touch timer: measure the channel it
 * selects (see the top of the file).
 */
static void touch_start(ESP32P4TouchState *s, bool by_timer, int64_t when)
{
    uint32_t mux0 = touch_ana(s, A_MUX0);
    unsigned slp = touch_slp_pad(s);
    int ch = -1;

    if (s->in_reset) {
        return;
    }
    if (s->fsm_phase != ESP32P4_TOUCH_FSM_IDLE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: a START while channel %u is "
                      "measured is ignored\n", TYPE_ESP32P4_TOUCH,
                      s->cur_ch);
        return;
    }
    s->cur_slp = false;
    s->cur_scan = false;
    if (s->sleeping && touch_valid_chan(slp)) {
        ch = slp;
        s->cur_slp = true;
    } else if (mux0 & MUX0_FSM_EN) {
        ch = touch_next_scan(s);
        s->cur_scan = true;
    } else {
        uint32_t mask = MUX1_START(touch_ana(s, A_MUX1)) & ~1u;

        if (mask & (mask - 1)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TOUCH_START selects several "
                          "channels (0x%x); channel %d is measured\n",
                          TYPE_ESP32P4_TOUCH, mask, ctz32(mask));
        }
        ch = mask ? ctz32(mask) : -1;
    }
    if (ch < 0) {
        if (!by_timer) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: a START with no channel to "
                          "measure\n", TYPE_ESP32P4_TOUCH);
        }
        touch_timer_done(s, when);
        return;
    }
    if (s->cur_scan) {
        s->scan_last = ch;
    }
    s->by_timer = by_timer;
    s->cur_ch = ch;
    s->scan_curr = ch;
    s->meas_done = false;
    s->cur_freq = 0;
    s->nfreq = touch_nfreq(s);
    touch_next(s, when);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_timer_run(void *opaque)
{
    ESP32P4TouchState *s = opaque;
    int64_t when = esp32p4_clkwait_expire(&s->timer_wait);

    switch (s->timer_phase) {
    case ESP32P4_TOUCH_TIMER_WAIT:
        s->timer_phase = ESP32P4_TOUCH_TIMER_MEAS;
        if (!(touch_ana(s, A_MUX0) & MUX0_START_FORCE)) {
            touch_start(s, true, when);
        }
        if (s->pwr_cntl & PWR_FORCE_DONE) {
            touch_timer_done(s, when);
        }
        break;
    case ESP32P4_TOUCH_TIMER_SLEEP:
        touch_timer_set(s, ESP32P4_TOUCH_TIMER_WAIT,
                        PWR_WAIT_CYCLES(s->pwr_cntl), when);
        break;
    default:
        break;
    }
}

/* Take the timed event @fn at @when, its changes catching up nothing. */
static void touch_run_event(ESP32P4TouchState *s, int64_t when,
                            void (*fn)(void *opaque))
{
    bool was = s->in_event;

    s->in_event = true;
    esp32_line_run_at(when, fn, s);
    s->in_event = was;
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * Take the events that fell due by @now, in time order, each at its own
 * time, before a change at @now.
 */
static void touch_catch_up(ESP32P4TouchState *s, int64_t now)
{
    if (s->in_event) {
        return;
    }
    for (;;) {
        QEMUTimer *timer = NULL;
        void (*run)(void *opaque) = NULL;
        int64_t due = INT64_MAX, t;

        if (timer_pending(&s->meas_timer) && s->meas_due_ns < due) {
            timer = &s->meas_timer;
            due = s->meas_due_ns;
            run = touch_meas_run;
        }
        t = esp32p4_clkwait_end(&s->xpd_wait);
        if (timer_pending(&s->xpd_wait.timer) && t < due) {
            timer = &s->xpd_wait.timer;
            due = t;
            run = touch_xpd_run;
        }
        t = esp32p4_clkwait_end(&s->timer_wait);
        if (timer_pending(&s->timer_wait.timer) && t < due) {
            timer = &s->timer_wait.timer;
            due = t;
            run = touch_timer_run;
        }
        if (!timer || due > now) {
            return;
        }
        timer_del(timer);
        touch_run_event(s, due, run);
    }
}

void esp32p4_touch_sync(ESP32P4TouchState *s)
{
    touch_catch_up(s, esp32_line_time());
}

/* An event falls due: take it at its time, which a late timer has passed. */
static void touch_meas_cb(void *opaque)
{
    ESP32P4TouchState *s = opaque;

    touch_run_event(s, s->meas_due_ns, touch_meas_run);
}

static void touch_xpd_done(void *opaque)
{
    ESP32P4TouchState *s = opaque;

    touch_run_event(s, esp32p4_clkwait_end(&s->xpd_wait), touch_xpd_run);
}

static void touch_timer_cb(void *opaque)
{
    ESP32P4TouchState *s = opaque;

    touch_run_event(s, esp32p4_clkwait_end(&s->timer_wait), touch_timer_run);
}

/*
 * [spec:nuos:req:emu.esp32p4.analog]
 * PMU_TOUCH_PWR_CNTL written: start or stop the timer, or force its done,
 * stopping the measurement it started.
 */
static void touch_timer_update(void *opaque, int n, int level)
{
    ESP32P4TouchState *s = opaque;
    uint32_t old = s->pwr_cntl;
    int64_t now = esp32_line_time();

    if (!level) {
        return;
    }
    touch_catch_up(s, now);
    s->pwr_cntl = esp32p4_pmu_touch_pwr_cntl(s->pmu);
    if ((s->pwr_cntl & PWR_SLEEP_TIMER_EN) && !(old & PWR_SLEEP_TIMER_EN)) {
        touch_timer_set(s, ESP32P4_TOUCH_TIMER_WAIT,
                        PWR_WAIT_CYCLES(s->pwr_cntl), now);
    } else if (!(s->pwr_cntl & PWR_SLEEP_TIMER_EN)) {
        touch_timer_set(s, ESP32P4_TOUCH_TIMER_OFF, 0, now);
    }
    if ((s->pwr_cntl & PWR_FORCE_DONE) &&
        s->timer_phase == ESP32P4_TOUCH_TIMER_MEAS) {
        if (s->by_timer && s->fsm_phase != ESP32P4_TOUCH_FSM_IDLE) {
            touch_abort(s);
        }
        touch_timer_done(s, now);
    }
}

/* ---- Registers ---- */

static void touch_clear_chan(ESP32P4TouchChan *c)
{
    bool active = c->active;

    *c = (ESP32P4TouchChan) { .active = active };
}

/* [spec:nuos:req:emu.esp32p4.analog] */
void esp32p4_touch_ana_write(ESP32P4TouchState *s, hwaddr offset,
                             uint32_t old, uint32_t val, uint32_t wt)
{
    int64_t now = esp32_line_time();

    switch (offset) {
    case A_MUX0:
        if ((val & MUX0_START_FORCE) && (val & MUX0_START_EN) &&
            !(old & MUX0_START_EN)) {
            touch_start(s, false, now);
        }
        if ((val & MUX0_DONE_FORCE) && (val & MUX0_DONE_EN) &&
            !(old & MUX0_DONE_EN)) {
            touch_timer_done(s, now);
        }
        break;
    case A_FILTER3:
        if (wt & F3_UPDATE_BENCHMARK_SW) {
            uint32_t ctrl = touch_ana(s, A_CTRL);
            unsigned pad = CTRL_BM_PAD_SEL(ctrl);
            unsigned f = CTRL_BM_FREQ_SEL(ctrl);

            if (pad >= ESP32P4_TOUCH_SLOTS || f >= ESP32P4_TOUCH_FREQS) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: the benchmark update "
                              "names channel %u, mode %u, which do not "
                              "exist\n", TYPE_ESP32P4_TOUCH, pad, f);
                break;
            }
            s->chan[pad].bench[f] = F3_BENCHMARK_SW(val) << FRAC;
            s->chan[pad].bench_valid[f] = true;
            touch_raise(s, INT_BENCHMARK_UPDATE);
        }
        break;
    case A_SLP0:
        if (wt & SLP0_CHANNEL_CLR) {
            touch_clear_chan(&s->slp);
        }
        touch_update_wakeup(s);
        break;
    case A_CLR:
        for (unsigned ch = 0; ch < ESP32P4_TOUCH_SLOTS; ch++) {
            if (CLR_CHANNEL_CLR(wt) & BIT(ch)) {
                touch_clear_chan(&s->chan[ch]);
            }
        }
        if (wt & CLR_STATUS_CLR) {
            for (unsigned ch = 0; ch < ESP32P4_TOUCH_SLOTS; ch++) {
                s->chan[ch].active = false;
                s->chan[ch].debounce = 0;
            }
            s->slp.active = false;
            s->slp.debounce = 0;
        }
        touch_update_wakeup(s);
        break;
    case A_SCAN_CTRL2:
        if (s->fsm_phase == ESP32P4_TOUCH_FSM_MEAS) {
            touch_meas_arm(s);
        }
        break;
    case A_FILTER2:
        touch_update_wakeup(s);
        break;
    default:
        break;
    }
}

void esp32p4_touch_ana_reset(ESP32P4TouchState *s)
{
    touch_update_wakeup(s);
}

static uint32_t touch_data(ESP32P4TouchChan *c, unsigned sel, int f)
{
    if (f < 0 || f >= ESP32P4_TOUCH_FREQS) {
        return 0;
    }
    switch (sel) {
    case 2:
        return MIN(c->bench[f] >> FRAC, DATA_MAX);
    case 3:
        return MIN(c->smooth[f] >> FRAC, DATA_MAX);
    default:
        return c->raw[f];
    }
}

static uint32_t touch_status(ESP32P4TouchChan *c, unsigned sel, int f)
{
    return touch_data(c, sel, f) | (uint32_t)c->debounce << 16 |
           (uint32_t)c->nn_cnt << 19;
}

static uint64_t touch_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4TouchState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t mux0 = touch_ana(s, A_MUX0);
    unsigned sel = MUX0_DATA_SEL(mux0), fsel = MUX0_FREQ_SEL(mux0);
    uint32_t ap = touch_ana(s, A_APPROACH);
    uint32_t v = 0;

    touch_catch_up(s, esp32_line_time());
    if (reg >= A_STATUS(0) && reg < A_STATUS(ESP32P4_TOUCH_SLOTS)) {
        v = touch_status(&s->chan[(reg - A_STATUS(0)) / 4], sel, fsel);
        return extract32(v, (addr & 3) * 8, size * 8);
    }
    switch (reg) {
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_INT_CLR:
        v = 0;
        break;
    case A_CHN_STATUS:
        v = touch_pad_active(s) | (s->meas_done ? CHN_MEAS_DONE : 0) |
            (uint32_t)s->scan_curr << 16;
        break;
    case A_SLP_STATUS:
        v = touch_status(&s->slp, sel, (int)fsel - 1);
        break;
    case A_APRCH_STATUS:
        for (int x = 0; x < ESP32P4_TOUCH_PROX; x++) {
            unsigned ch = APPROACH_PAD(ap, x);

            if (ch < ESP32P4_TOUCH_SLOTS) {
                v |= MIN(s->chan[ch].prox_cnt, 0xff) << (16 - 8 * x);
            }
        }
        v |= MIN(s->slp.prox_cnt, 0xff) << 24;
        break;
    case A_PARA_STATUS:
        v = extract32(s->cur_para, 0, 23) | (uint32_t)s->cur_freq << 23;
        break;
    case A_CHN_TMP_STATUS:
        v = s->tmp_inactive | (uint32_t)s->tmp_active << 15;
        break;
    case A_DATE:
        v = s->date;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TOUCH, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    ESP32P4TouchState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;

    touch_catch_up(s, esp32_line_time());
    if (s->in_reset) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to offset 0x%" HWADDR_PRIx
                      " while RST_EN_LP_TOUCH holds the sensor in reset\n",
                      TYPE_ESP32P4_TOUCH, addr);
        return;
    }
    switch (reg) {
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_ALL);
        touch_update_irq(s);
        break;
    case A_INT_ENA:
        s->int_ena = (s->int_ena & ~(lanes & INT_ALL)) | (v & INT_ALL);
        touch_update_irq(s);
        break;
    case A_DATE:
        s->date = (s->date & ~(lanes & DATE_CLK_EN)) | (v & DATE_CLK_EN);
        break;
    case A_INT_ST:
    case A_CHN_STATUS:
    case A_SLP_STATUS:
    case A_APRCH_STATUS:
    case A_PARA_STATUS:
    case A_CHN_TMP_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TOUCH, addr);
        break;
    default:
        if (reg >= A_STATUS(0) && reg < A_STATUS(ESP32P4_TOUCH_SLOTS)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset "
                          "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_TOUCH, addr);
            break;
        }
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_TOUCH, addr);
        break;
    }
}

static const MemoryRegionOps touch_ops = {
    .read = touch_read,
    .write = touch_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ---- Inputs ---- */

/* The sensor's own state: its FSM, data, touches and interrupts. */
static void touch_reset_state(ESP32P4TouchState *s)
{
    touch_abort(s);
    for (unsigned ch = 0; ch < ESP32P4_TOUCH_SLOTS; ch++) {
        s->chan[ch] = (ESP32P4TouchChan) { 0 };
    }
    s->slp = (ESP32P4TouchChan) { 0 };
    s->int_raw = 0;
    s->int_ena = 0;
    s->date = 0;
    s->meas_done = false;
    s->scan_curr = ESP32P4_TOUCH_NULL;
    s->scan_last = 0;
    s->tmp_active = 0;
    s->tmp_inactive = 0;
    s->cur_para = 0;
    s->cur_ch = 0;
    s->cur_freq = 0;
    s->nfreq = 1;
    s->cur_slp = false;
    s->cur_scan = false;
    s->by_timer = false;
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * LPPERI_RST_EN_LP_TOUCH: the sensor reset, and held so.
 */
static void touch_set_reset(void *opaque, int n, int level)
{
    ESP32P4TouchState *s = opaque;

    if (s->in_reset == !!level) {
        return;
    }
    touch_catch_up(s, esp32_line_time());
    touch_reset_state(s);
    s->in_reset = level;
    touch_update_irq(s);
    touch_update_wakeup(s);
}

static void touch_set_sleep(void *opaque, int n, int level)
{
    ESP32P4TouchState *s = opaque;

    touch_catch_up(s, esp32_line_time());
    s->sleeping = level;
}

static void touch_fast_update(void *opaque, ClockEvent event)
{
    ESP32P4TouchState *s = opaque;
    int64_t now = esp32_line_time();

    if (event == ClockPreUpdate) {
        touch_catch_up(s, now);
        esp32p4_clkwait_fold(&s->xpd_wait, now);
        if (touch_counting(s)) {
            s->meas_count = touch_count_at(s, now);
            if (now > s->meas_edges.origin_ns) {
                esp32p4_clk_count_rebase(&s->meas_edges, now);
            }
        }
        return;
    }
    esp32p4_clkwait_resched(&s->xpd_wait);
    esp32p4_clk_count_start(&s->meas_edges, s->fast,
                            s->meas_edges.origin_ns);
    if (touch_counting(s)) {
        touch_meas_arm(s);
    }
}

static void touch_slow_update(void *opaque, ClockEvent event)
{
    ESP32P4TouchState *s = opaque;

    if (event == ClockPreUpdate) {
        touch_catch_up(s, esp32_line_time());
        esp32p4_clkwait_fold(&s->timer_wait, esp32_line_time());
    } else {
        esp32p4_clkwait_resched(&s->timer_wait);
    }
}

static void touch_get_ff(Object *obj, Visitor *v, const char *name,
                         void *opaque, Error **errp)
{
    visit_type_uint32(v, name, opaque, errp);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static void touch_set_ff(Object *obj, Visitor *v, const char *name,
                         void *opaque, Error **errp)
{
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value > 10000000) {
        error_setg(errp, "%s: %u fF is above the 10 uF a pad could carry",
                   name, value);
        return;
    }
    *(uint32_t *)opaque = value;
}

/* ---- Device ---- */

static void esp32p4_touch_reset_hold(Object *obj, ResetType type)
{
    ESP32P4TouchState *s = ESP32P4_TOUCH(obj);

    touch_reset_state(s);
    s->timer_phase = ESP32P4_TOUCH_TIMER_OFF;
    esp32p4_clkwait_stop(&s->timer_wait);
    s->pwr_cntl = 0;
    s->in_reset = false;
    s->in_event = false;
}

static void esp32p4_touch_reset_exit(Object *obj, ResetType type)
{
    ESP32P4TouchState *s = ESP32P4_TOUCH(obj);

    touch_update_irq(s);
    s->wakeup_level = false;
    qemu_set_irq(s->wakeup, 0);
}

static void esp32p4_touch_init(Object *obj)
{
    ESP32P4TouchState *s = ESP32P4_TOUCH(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &touch_ops, s, "esp32p4.lp-touch",
                          ESP32P4_TOUCH_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(dev, touch_set_reset, ESP32P4_TOUCH_RESET, 1);
    qdev_init_gpio_in_named(dev, touch_set_sleep, ESP32P4_TOUCH_SLEEP, 1);
    qdev_init_gpio_in_named(dev, touch_timer_update,
                            ESP32P4_TOUCH_TIMER_UPDATE, 1);
    qdev_init_gpio_out_named(dev, &s->wakeup, ESP32P4_TOUCH_WAKEUP, 1);
    s->fast = qdev_init_clock_in(dev, "fast", touch_fast_update, s,
                                 ClockPreUpdate | ClockUpdate);
    s->slow = qdev_init_clock_in(dev, "slow", touch_slow_update, s,
                                 ClockPreUpdate | ClockUpdate);
    esp32p4_clkwait_init(&s->xpd_wait, s->fast, touch_xpd_done, s);
    esp32p4_clkwait_init(&s->timer_wait, s->slow, touch_timer_cb, s);
    timer_init_ns(&s->meas_timer, QEMU_CLOCK_VIRTUAL, touch_meas_cb, s);

    for (unsigned ch = ESP32P4_TOUCH_FIRST; ch < ESP32P4_TOUCH_SLOTS; ch++) {
        g_autofree char *name = g_strdup_printf("ch%u-ff", ch);
        g_autofree char *desc = g_strdup_printf(
            "The capacitance of touch pad T%u, GPIO%u, in fF", ch, ch + 1);

        s->cap_ff[ch] = DEFAULT_PAD_FF;
        object_property_add(obj, name, "uint32", touch_get_ff, touch_set_ff,
                            NULL, &s->cap_ff[ch]);
        object_property_set_description(obj, name, desc);
    }
}

static void esp32p4_touch_realize(DeviceState *dev, Error **errp)
{
    ESP32P4TouchState *s = ESP32P4_TOUCH(dev);

    if (!s->lp_ana || !s->pmu) {
        error_setg(errp, "%s: the lp-ana and pmu links must be set",
                   TYPE_ESP32P4_TOUCH);
    }
}

static const VMStateDescription vmstate_esp32p4_touch_chan = {
    .name = "esp32p4-touch-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(raw, ESP32P4TouchChan, ESP32P4_TOUCH_FREQS),
        VMSTATE_UINT32_ARRAY(smooth, ESP32P4TouchChan, ESP32P4_TOUCH_FREQS),
        VMSTATE_UINT32_ARRAY(bench, ESP32P4TouchChan, ESP32P4_TOUCH_FREQS),
        VMSTATE_BOOL_ARRAY(bench_valid, ESP32P4TouchChan,
                           ESP32P4_TOUCH_FREQS),
        VMSTATE_UINT32_ARRAY(prox_sum, ESP32P4TouchChan,
                             ESP32P4_TOUCH_FREQS),
        VMSTATE_UINT32(prox_cnt, ESP32P4TouchChan),
        VMSTATE_UINT8(nn_cnt, ESP32P4TouchChan),
        VMSTATE_UINT8(debounce, ESP32P4TouchChan),
        VMSTATE_BOOL(active, ESP32P4TouchChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32p4_touch = {
    .name = TYPE_ESP32P4_TOUCH,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_raw, ESP32P4TouchState),
        VMSTATE_UINT32(int_ena, ESP32P4TouchState),
        VMSTATE_UINT32(date, ESP32P4TouchState),
        VMSTATE_BOOL(meas_done, ESP32P4TouchState),
        VMSTATE_UINT8(scan_curr, ESP32P4TouchState),
        VMSTATE_UINT16(tmp_active, ESP32P4TouchState),
        VMSTATE_UINT16(tmp_inactive, ESP32P4TouchState),
        VMSTATE_UINT32(cur_para, ESP32P4TouchState),
        VMSTATE_STRUCT_ARRAY(chan, ESP32P4TouchState, ESP32P4_TOUCH_SLOTS, 1,
                             vmstate_esp32p4_touch_chan, ESP32P4TouchChan),
        VMSTATE_STRUCT(slp, ESP32P4TouchState, 1, vmstate_esp32p4_touch_chan,
                       ESP32P4TouchChan),
        VMSTATE_UINT8(fsm_phase, ESP32P4TouchState),
        VMSTATE_UINT8(cur_ch, ESP32P4TouchState),
        VMSTATE_BOOL(cur_slp, ESP32P4TouchState),
        VMSTATE_BOOL(cur_scan, ESP32P4TouchState),
        VMSTATE_BOOL(by_timer, ESP32P4TouchState),
        VMSTATE_UINT8(cur_freq, ESP32P4TouchState),
        VMSTATE_UINT8(nfreq, ESP32P4TouchState),
        VMSTATE_UINT8(scan_last, ESP32P4TouchState),
        VMSTATE_ESP32P4_CLKWAIT(xpd_wait, ESP32P4TouchState),
        VMSTATE_TIMER(meas_timer, ESP32P4TouchState),
        VMSTATE_ESP32P4_CLK_COUNT(meas_edges, ESP32P4TouchState),
        VMSTATE_UINT64(meas_count, ESP32P4TouchState),
        VMSTATE_INT64(meas_end_ns, ESP32P4TouchState),
        VMSTATE_INT64(meas_due_ns, ESP32P4TouchState),
        VMSTATE_UINT8(timer_phase, ESP32P4TouchState),
        VMSTATE_ESP32P4_CLKWAIT(timer_wait, ESP32P4TouchState),
        VMSTATE_UINT32(pwr_cntl, ESP32P4TouchState),
        VMSTATE_BOOL(sleeping, ESP32P4TouchState),
        VMSTATE_BOOL(in_reset, ESP32P4TouchState),
        VMSTATE_BOOL(wakeup_level, ESP32P4TouchState),
        VMSTATE_UINT32_ARRAY(cap_ff, ESP32P4TouchState, ESP32P4_TOUCH_SLOTS),
        VMSTATE_CLOCK(fast, ESP32P4TouchState),
        VMSTATE_CLOCK(slow, ESP32P4TouchState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32p4_touch_properties[] = {
    DEFINE_PROP_LINK("lp-ana", ESP32P4TouchState, lp_ana, TYPE_ESP32P4_LP_ANA,
                     ESP32P4LPAnaState *),
    DEFINE_PROP_LINK("pmu", ESP32P4TouchState, pmu, TYPE_ESP32P4_PMU,
                     ESP32P4PMUState *),
};

static void esp32p4_touch_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_touch_reset_hold;
    rc->phases.exit = esp32p4_touch_reset_exit;
    dc->realize = esp32p4_touch_realize;
    dc->vmsd = &vmstate_esp32p4_touch;
    device_class_set_props(dc, esp32p4_touch_properties);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static const TypeInfo esp32p4_touch_info = {
    .name = TYPE_ESP32P4_TOUCH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4TouchState),
    .instance_init = esp32p4_touch_init,
    .class_init = esp32p4_touch_class_init,
};

static void esp32p4_touch_register_types(void)
{
    type_register_static(&esp32p4_touch_info);
}

type_init(esp32p4_touch_register_types)
