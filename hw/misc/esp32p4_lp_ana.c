/*
 * ESP32-P4 LP analog peripherals (LP_ANALOG_PERI): the brown-out detector,
 * the glitch detectors and the power supply detector
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapters
 * 25 (Brown-out Detector) and 26 (Power Supply Detector), 21.2.2.2 (FIB
 * control) and table 11.1-1 (reset causes); ESP-IDF's
 * lp_analog_peri_reg.h for v3, which gives the registers' layout, access
 * and reset values, regi2c_brownout.h, and its hal brownout_ll.h and
 * vbat_ll.h and power_supply/brownout.c, which show how software drives
 * the detector.
 *
 * The supplies are the board's: VDD_ANA and VDD_BAT in millivolts, the
 * "vdda-mv" and "vbat-mv" properties, which a board or a test changes as
 * the machine runs. Their comparators take their thresholds from the
 * analog block's BIAS registers, which software sets through the analog
 * I2C master: VDD_BAT's (BIAS register 8), VDD_ANA's (9) and the battery
 * charger's (10), each an under-voltage threshold in bits 4:2 and a
 * release threshold in bits 7:5, 2.0 V + 0.1 V per code (2.2 V for the
 * charger's). A supply is under-voltage once below its under-voltage
 * threshold and until back at or above the higher of the two. The
 * charger's comparator runs only while BIAS register 10's bit 0 (ESP-IDF's
 * OR_FORCE_PU_VBAT_CHARGER) powers it.
 *
 * The brown-out detector (TRM figure 25.3-1) sees a brown-out while it is
 * enabled, by LP_SYSTEM's FIB control ("bod-en"), and a source
 * LP_ANA_BOD_SOURCE_ENA selects is under-voltage: VDD_ANA (bit 2, the
 * default) or VDD_BAT (bit 1), the latter only while the PMU's
 * BOD_SOURCE_SEL lets it through ("vbat-sel").
 *
 *   - Mode 0: while BOD_MODE0_INTR_ENA is set, and BOD_MODE0_CNT_CLR not,
 *     a brown-out runs the counter, which counts LP_DYN_FAST_CLK and
 *     restarts from 0 each time the brown-out ends. At INTR_WAIT counts it
 *     raises BOD_MODE0_INT (INT_RAW and LP_INT_RAW bit 31), which stays
 *     raised, clearing it or not, while the count stays there; at
 *     RESET_WAIT counts, with RESET_ENA, it resets the chip (RESET_SEL 0)
 *     or the system (1), with reset cause 0x0F. CLOSE_FLASH_ENA's suspend
 *     of the flash is logged as not modelled; PD_RF_ENA powers down the
 *     radio, which the P4 does not have.
 *   - Mode 1: a brown-out resets the system, with cause 0x0F, a cycle of
 *     LP_DYN_FAST_CLK later, while it is enabled: by the analog default,
 *     which has it enabled, while FIB_ENABLE bit 1 is set (its reset
 *     value), or by BOD_MODE1_RESET_ENA once software clears that bit, as
 *     ESP-IDF's brownout_ll_ana_reset_enable() does.
 *
 * VDD_BAT's under-voltage filters (TRM figure 25.4-1), VDDBAT_BOD_CNTL's
 * on its own comparator and VDDBAT_CHARGE_CNTL's on the charger's, count
 * LP_DYN_FAST_CLK (the TRM does not name their clock): from 0 while the
 * supply is under-voltage, up to UNDERVOLTAGE_TARGET, where the flag sets
 * and the under-voltage interrupt rises; with the flag set, down while it
 * is good, to UPVOLTAGE_TARGET, where the flag clears, the up-voltage
 * interrupt rises and the count returns to 0, or back up while it is not.
 * CNT_CLR holds the count at 0. VDDBAT_UNDERVOLTAGE_FLAG is the PMU's
 * VBAT_UNDERVOLTAGE wakeup. Charging the battery (the CHARGER bits) is
 * logged as not modelled.
 *
 * VGOOD_EVENT_RECORD, whose layout the TRM does not give, records the
 * under-voltage of VDD_BAT (bit 1) and VDD_ANA (bit 2), the bits by which
 * BOD_SOURCE_ENA selects them, until VBAT_EVENT_RECORD_CLR's bit for it is
 * written. DETMODE_SEL is kept as written.
 *
 * The glitch detectors: a glitch on VDD ("power-glitch") resets the system
 * while PG_GLITCH_CNTL's POWER_GLITCH_RESET_ENA is set, and one on the
 * crystal's clock ("clock-glitch") while CK_GLITCH_CNTL's
 * CK_GLITCH_RESET_ENA is, both with cause 0x13. The power supply detector
 * (PSDET, chapter 26), which has no registers, resets the system with
 * cause 0x1B when a glitch on the HP system regulator ("hp-supply-glitch")
 * lasts 50 ns, while the eFuse EFUSE_PVT_GLITCH_EN enables it; its
 * EFUSE_PVT_GLITCH_MODE, which the TRM does not describe, is logged as not
 * modelled.
 *
 * The interrupts: INT_* drive LP_ANAPERI_INTR (source 8), LP_INT_* the LP
 * core's interrupt; the raw bits clear when written with 1 and through
 * the clear registers.
 *
 * The touch sensor's registers, 0xFC to 0x1FC, are kept as written for
 * the touch sensor, which is not modelled; writing them is logged as
 * such. The block is in the LP always-on domain: a system reset resets
 * it, as LP_CLKRST_RST_EN_LP_ANAPERI does.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_lp_ana.h"
#include "migration/vmstate.h"
#include "system/runstate.h"

#define A_BOD_MODE0_CNTL        0x000
#define A_BOD_MODE1_CNTL        0x004
#define A_VDD_SOURCE_CNTL       0x008
#define A_VDDBAT_BOD_CNTL       0x00c
#define A_VDDBAT_CHARGE_CNTL    0x010
#define A_CK_GLITCH_CNTL        0x014
#define A_PG_GLITCH_CNTL        0x018
#define A_FIB_ENABLE            0x01c
#define A_INT_RAW               0x020
#define A_INT_ST                0x024
#define A_INT_ENA               0x028
#define A_INT_CLR               0x02c
#define A_LP_INT_RAW            0x030
#define A_LP_INT_ST             0x034
#define A_LP_INT_ENA            0x038
#define A_LP_INT_CLR            0x03c
#define A_TOUCH_FIRST           0x0fc
#define A_TOUCH_LAST            0x1fc
#define A_DATE                  0x3fc

#define R(a)                    ((a) / 4)

/* BOD_MODE0_CNTL */
#define MODE0_CLOSE_FLASH_ENA   BIT(6)
#define MODE0_INTR_WAIT(v)      extract32((v), 8, 10)
#define MODE0_RESET_WAIT(v)     extract32((v), 18, 10)
#define MODE0_CNT_CLR           BIT(28)
#define MODE0_INTR_ENA          BIT(29)
#define MODE0_RESET_SEL         BIT(30)
#define MODE0_RESET_ENA         BIT(31)
/* BOD_MODE1_CNTL */
#define MODE1_RESET_ENA         BIT(31)
/* VDD_SOURCE_CNTL */
#define VGOOD_EVENT_RECORD_SHIFT 8
#define VBAT_EVENT_RECORD_CLR_SHIFT 16
#define BOD_SOURCE_ENA(v)       extract32((v), 24, 8)
#define SOURCE_VBAT             BIT(1)
#define SOURCE_VDDA             BIT(2)
/* VDDBAT_BOD_CNTL and VDDBAT_CHARGE_CNTL */
#define FILTER_FLAG             BIT(0)
#define FILTER_CHARGER          BIT(10)
#define FILTER_CNT_CLR          BIT(11)
#define FILTER_UP_TARGET(v)     extract32((v), 12, 10)
#define FILTER_UNDER_TARGET(v)  extract32((v), 22, 10)
/* CK_GLITCH_CNTL, PG_GLITCH_CNTL */
#define GLITCH_RESET_ENA        BIT(31)
/* FIB_ENABLE: bit 1 leaves mode 1's reset enable to the analog default. */
#define FIB_BOD                 BIT(1)
/* INT_* and LP_INT_* */
#define INT_CHARGE_UPVOLTAGE    BIT(27)
#define INT_CHARGE_UNDERVOLTAGE BIT(28)
#define INT_VBAT_UPVOLTAGE      BIT(29)
#define INT_VBAT_UNDERVOLTAGE   BIT(30)
#define INT_BOD_MODE0           BIT(31)
#define INT_MASK                0xf8000000
#define LP_INT_MASK             INT_BOD_MODE0

/* The thresholds' scales, millivolts at code 0 and per code (TRM 25.4). */
#define BOD_BASE_MV             2000
#define CHARGER_BASE_MV         2200
#define THRESHOLD_STEP_MV       100
/* BIAS register 10's power-up of the charger comparator. */
#define BIAS_CHARGER_PU         BIT(0)

/* The power supply detector catches a glitch of this length or more. */
#define PSDET_GLITCH_NS         50
/* EFUSE_PVT_GLITCH_EN and _MODE: eFuse block 0, word 5. */
#define EFUSE_PVT_WORD          5
#define EFUSE_PVT_GLITCH_EN     BIT(2)
#define EFUSE_PVT_GLITCH_MODE(v) extract32((v), 22, 2)

#define DEFAULT_SUPPLY_MV       3300

typedef struct {
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
    uint32_t ro;
    uint32_t wtc;
    bool valid;
} LPAnaReg;

#define REG(a, r, w, t, o, c) \
    [R(a)] = { .reset = (r), .rw = (w), .wt = (t), .ro = (o), .wtc = (c), \
               .valid = true }

/* From lp_analog_peri_reg.h: reset, R/W, WT, RO and R/WTC/SS bits. */
static const LPAnaReg lp_ana_regs[ESP32P4_LP_ANA_NREGS] = {
    REG(0x000, 0x0ffc0100, 0xffffffc0, 0, 0, 0),
    REG(0x004, 0, 0x80000000, 0, 0, 0),
    REG(0x008, 0x040000ff, 0xff0000ff, 0x00ff0000, 0x0000ff00, 0),
    REG(0x00c, 0xffc00000, 0xfffffc00, 0, 0x00000001, 0),
    REG(0x010, 0xffc00000, 0xfffffc00, 0, 0x00000001, 0),
    REG(0x014, 0, 0x80000000, 0, 0, 0),
    REG(0x018, 0, 0x80000000, 0, 0, 0),
    REG(0x01c, 0xffffffff, 0xffffffff, 0, 0, 0),
    REG(0x020, 0, 0, 0, 0, INT_MASK),
    REG(0x024, 0, 0, 0, INT_MASK, 0),
    REG(0x028, 0, INT_MASK, 0, 0, 0),
    REG(0x02c, 0, 0, INT_MASK, 0, 0),
    REG(0x030, 0, 0, 0, 0, LP_INT_MASK),
    REG(0x034, 0, 0, 0, LP_INT_MASK, 0),
    REG(0x038, 0, LP_INT_MASK, 0, 0, 0),
    REG(0x03c, 0, 0, LP_INT_MASK, 0, 0),
    REG(0x0fc, 0x06419064, 0x3fffffff, 0, 0, 0),
    REG(0x100, 0x00080000, 0xffffffff, 0, 0, 0),
    REG(0x104, 0x37bfffc0, 0x3fffffc0, 0, 0, 0),
    REG(0x108, 0, 0x0bff0000, 0x04000000, 0, 0),
    REG(0x10c, 0x06419064, 0x3fffffff, 0, 0, 0),
    REG(0x110, 0x6a0a0200, 0xffffffff, 0, 0, 0),
    REG(0x114, 0x1fff8000, 0xffff8000, 0, 0, 0),
    REG(0x118, 0, 0x0000ffff, 0x00010000, 0, 0),
    REG(0x11c, 0x001e0000, 0x001effff, 0x00010000, 0, 0),
    REG(0x120, 0, 0xffffffff, 0, 0, 0),
    REG(0x124, 0, 0, 0x0000ffff, 0, 0),
    REG(0x128, 0x00000fff, 0x00001fff, 0, 0, 0),
    REG(0x12c, 0, 0x007fffff, 0, 0, 0),
    REG(0x130, 0, 0x007fffff, 0, 0, 0),
    REG(0x134, 0, 0x007fffff, 0, 0, 0),
    REG(0x138, 0, 0x000007ff, 0, 0, 0),
    REG(0x13c, 0x20000000, 0xffffff00, 0, 0, 0),
    REG(0x140, 0, 0x3fffffff, 0, 0, 0),
    /* TOUCH_PAD0_TH0 to TOUCH_PAD14_TH2: see esp32p4_lp_ana_reg_info(). */
    REG(0x1fc, 0x00000040, 0x000000ff, 0, 0, 0),
    REG(0x3fc, 0x00250220, 0xffffffff, 0, 0, 0),
};

/* The touch pads' thresholds, 0x144 to 0x1F4, three a pad. */
#define A_TOUCH_PAD_TH_FIRST    0x144
#define A_TOUCH_PAD_TH_LAST     0x1f4
static const LPAnaReg lp_ana_touch_th = {
    .reset = 0, .rw = 0xffff0000, .valid = true,
};

static const LPAnaReg *esp32p4_lp_ana_reg_info(hwaddr reg)
{
    if (reg >= A_TOUCH_PAD_TH_FIRST && reg <= A_TOUCH_PAD_TH_LAST) {
        return &lp_ana_touch_th;
    }
    if (reg >= ESP32P4_LP_ANA_SIZE || !lp_ana_regs[R(reg)].valid) {
        return NULL;
    }
    return &lp_ana_regs[R(reg)];
}

/* The least time in which @clk makes @n cycles, capped for timer_mod(). */
static int64_t lp_ana_cycles_ns(Clock *clk, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(clk, n);

    while (ns < INT64_MAX / 4 && clock_ns_to_ticks(clk, ns) < n) {
        ns++;
    }
    return MIN(ns, INT64_MAX / 4);
}

static uint64_t lp_ana_cycles_since(ESP32P4LPAnaState *s, int64_t from,
                                    int64_t now)
{
    if (!clock_is_enabled(s->fast) || now <= from) {
        return 0;
    }
    return clock_ns_to_ticks(s->fast, now - from);
}

/* A threshold of BIAS register @reg: its release one or under-voltage. */
static uint32_t lp_ana_threshold_mv(ESP32P4LPAnaState *s, uint8_t reg,
                                    bool release, uint32_t base)
{
    uint8_t v = 0;

    esp32p4_ana_read(s->ana, ESP32P4_ANA_SLAVE_BIAS, reg, &v);
    return base + THRESHOLD_STEP_MV * extract32(v, release ? 5 : 2, 3);
}

/* A comparator: under-voltage, with hysteresis between the thresholds. */
static bool lp_ana_compare(ESP32P4LPAnaState *s, uint8_t reg, uint32_t mv,
                           bool low, uint32_t base)
{
    uint32_t under = lp_ana_threshold_mv(s, reg, false, base);
    uint32_t release = lp_ana_threshold_mv(s, reg, true, base);

    return low ? mv < MAX(under, release) : mv < under;
}

static void lp_ana_update_irqs(ESP32P4LPAnaState *s)
{
    uint32_t *r = s->regs;
    bool level;

    qemu_set_irq(s->irq, (r[R(A_INT_RAW)] & r[R(A_INT_ENA)]) != 0);
    qemu_set_irq(s->lp_int, (r[R(A_LP_INT_RAW)] & r[R(A_LP_INT_ENA)]) != 0);
    level = s->brownout;
    if (level != s->brownout_level) {
        s->brownout_level = level;
        qemu_set_irq(s->brownout_out, level);
    }
    level = s->filter[ESP32P4_LP_ANA_FILTER_VBAT].flag;
    if (level != s->vbat_undervoltage_level) {
        s->vbat_undervoltage_level = level;
        qemu_set_irq(s->vbat_undervoltage_out, level);
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_reset_request(ESP32P4LPAnaState *s, bool chip,
                                 uint8_t cause)
{
    if (chip) {
        esp32p4_clkrst_set_chip_reset_cause(s->clkrst, cause);
    } else {
        esp32p4_clkrst_set_system_reset_cause(s->clkrst, cause);
    }
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

/* The register of filter @f. */
static uint32_t lp_ana_filter_cntl(ESP32P4LPAnaState *s, int f)
{
    return s->regs[R(f == ESP32P4_LP_ANA_FILTER_VBAT ? A_VDDBAT_BOD_CNTL
                                                      : A_VDDBAT_CHARGE_CNTL)];
}

/* Bring filter @f's count up to @now. */
static void lp_ana_filter_sync(ESP32P4LPAnaState *s, int f, int64_t now)
{
    ESP32P4LPAnaFilter *fl = &s->filter[f];
    uint32_t cntl = lp_ana_filter_cntl(s, f);
    uint64_t n = lp_ana_cycles_since(s, fl->sync_ns, now);

    if (fl->dir > 0) {
        fl->count = MIN(fl->count + n, FILTER_UNDER_TARGET(cntl));
    } else if (fl->dir < 0 && fl->count > FILTER_UP_TARGET(cntl)) {
        fl->count -= MIN(n, fl->count - FILTER_UP_TARGET(cntl));
    }
    fl->sync_ns = now;
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * Filter @f after a sync, its supply under-voltage (@low) or not: set its
 * direction, fire the targets it has reached and time the next.
 */
static void lp_ana_filter_eval(ESP32P4LPAnaState *s, int f, bool low)
{
    ESP32P4LPAnaFilter *fl = &s->filter[f];
    uint32_t cntl = lp_ana_filter_cntl(s, f);
    uint32_t under = FILTER_UNDER_TARGET(cntl);
    uint32_t up = FILTER_UP_TARGET(cntl);
    uint32_t under_int = f == ESP32P4_LP_ANA_FILTER_VBAT ?
                         INT_VBAT_UNDERVOLTAGE : INT_CHARGE_UNDERVOLTAGE;
    uint32_t up_int = f == ESP32P4_LP_ANA_FILTER_VBAT ?
                      INT_VBAT_UPVOLTAGE : INT_CHARGE_UPVOLTAGE;
    uint64_t ahead = 0;

    timer_del(&fl->timer);
    if (cntl & FILTER_CNT_CLR) {
        fl->count = 0;
        fl->dir = 0;
        return;
    }
    if (!fl->flag) {
        if (!low) {
            fl->count = 0;
            fl->dir = 0;
            return;
        }
        if (fl->count >= under) {
            fl->flag = true;
            fl->dir = 0;
            s->regs[R(A_INT_RAW)] |= under_int;
            return;
        }
        fl->dir = 1;
        ahead = under - fl->count;
    } else if (low) {
        fl->dir = fl->count < under ? 1 : 0;
        ahead = fl->dir ? under - fl->count : 0;
    } else {
        if (fl->count <= up) {
            fl->flag = false;
            fl->count = 0;
            fl->dir = 0;
            s->regs[R(A_INT_RAW)] |= up_int;
            return;
        }
        fl->dir = -1;
        ahead = fl->count - up;
    }
    if (fl->dir && clock_is_enabled(s->fast)) {
        timer_mod(&fl->timer, fl->sync_ns + lp_ana_cycles_ns(s->fast, ahead));
    }
}

/* The mode-0 count at @now. */
static uint64_t lp_ana_bod_count(ESP32P4LPAnaState *s, int64_t now)
{
    return s->bod_counting ?
           s->bod_base + lp_ana_cycles_since(s, s->bod_start_ns, now) : 0;
}

/* Whether mode 1's reset is enabled: by the analog default or software. */
static bool lp_ana_mode1_enabled(ESP32P4LPAnaState *s)
{
    return (s->regs[R(A_FIB_ENABLE)] & FIB_BOD) ||
           (s->regs[R(A_BOD_MODE1_CNTL)] & MODE1_RESET_ENA);
}

/*
 * [spec:nuos:req:emu.esp32p4.low-power]
 * Re-evaluate the comparators, the brown-out and its counters at @now.
 */
static void lp_ana_update(ESP32P4LPAnaState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t mode0 = s->regs[R(A_BOD_MODE0_CNTL)];
    uint32_t src = BOD_SOURCE_ENA(s->regs[R(A_VDD_SOURCE_CNTL)]);
    uint8_t charger_pu = 0;
    bool counting, level, was, reset_rises;
    uint64_t count, next;
    int f;

    for (f = 0; f < ESP32P4_LP_ANA_FILTERS; f++) {
        lp_ana_filter_sync(s, f, now);
    }

    was = s->vdda_low;
    s->vdda_low = lp_ana_compare(s, ESP32P4_ANA_BIAS_DREF_VDDA, s->vdda_mv,
                                 s->vdda_low, BOD_BASE_MV);
    if (s->vdda_low && !was) {
        s->regs[R(A_VDD_SOURCE_CNTL)] |= SOURCE_VDDA <<
                                         VGOOD_EVENT_RECORD_SHIFT;
    }
    was = s->vbat_low;
    s->vbat_low = lp_ana_compare(s, ESP32P4_ANA_BIAS_DREF_VBAT, s->vbat_mv,
                                 s->vbat_low, BOD_BASE_MV);
    if (s->vbat_low && !was) {
        s->regs[R(A_VDD_SOURCE_CNTL)] |= SOURCE_VBAT <<
                                         VGOOD_EVENT_RECORD_SHIFT;
    }
    esp32p4_ana_read(s->ana, ESP32P4_ANA_SLAVE_BIAS,
                     ESP32P4_ANA_BIAS_DREF_VBAT_CHARGER, &charger_pu);
    s->charger_low = (charger_pu & BIAS_CHARGER_PU) &&
                     lp_ana_compare(s, ESP32P4_ANA_BIAS_DREF_VBAT_CHARGER,
                                    s->vbat_mv, s->charger_low,
                                    CHARGER_BASE_MV);

    s->brownout = s->bod_en &&
                  (((src & SOURCE_VDDA) && s->vdda_low) ||
                   ((src & SOURCE_VBAT) && s->vbat_low && s->vbat_sel));

    /* Mode 0: the counter and its comparators. */
    counting = s->brownout && (mode0 & MODE0_INTR_ENA) &&
               !(mode0 & MODE0_CNT_CLR);
    if (counting && !s->bod_counting) {
        s->bod_base = 0;
        s->bod_start_ns = now;
    }
    s->bod_counting = counting;
    count = lp_ana_bod_count(s, now);
    level = counting && count >= MODE0_INTR_WAIT(mode0);
    if (level && !s->bod_int) {
        s->regs[R(A_INT_RAW)] |= INT_BOD_MODE0;
        s->regs[R(A_LP_INT_RAW)] |= INT_BOD_MODE0;
        if (mode0 & MODE0_CLOSE_FLASH_ENA) {
            qemu_log_mask(LOG_UNIMP, "%s: the brown-out's suspend of the "
                          "flash (BOD_MODE0_CLOSE_FLASH_ENA) is not "
                          "modelled\n", TYPE_ESP32P4_LP_ANA);
        }
    }
    s->bod_int = level;
    level = counting && (mode0 & MODE0_RESET_ENA) &&
            count >= MODE0_RESET_WAIT(mode0);
    reset_rises = level && !s->bod_reset;
    s->bod_reset = level;
    timer_del(&s->bod_timer);
    if (counting && clock_is_enabled(s->fast)) {
        next = UINT64_MAX;
        if (!s->bod_int) {
            next = MODE0_INTR_WAIT(mode0);
        }
        if ((mode0 & MODE0_RESET_ENA) && !s->bod_reset) {
            next = MIN(next, MODE0_RESET_WAIT(mode0));
        }
        if (next != UINT64_MAX) {
            timer_mod(&s->bod_timer,
                      now + lp_ana_cycles_ns(s->fast, next - count));
        }
    }

    /* Mode 1. */
    if (s->brownout && lp_ana_mode1_enabled(s)) {
        if (!timer_pending(&s->mode1_timer) && clock_is_enabled(s->fast)) {
            timer_mod(&s->mode1_timer, now + lp_ana_cycles_ns(s->fast, 1));
        }
    } else {
        timer_del(&s->mode1_timer);
    }

    lp_ana_filter_eval(s, ESP32P4_LP_ANA_FILTER_VBAT, s->vbat_low);
    lp_ana_filter_eval(s, ESP32P4_LP_ANA_FILTER_CHARGER, s->charger_low);
    lp_ana_update_irqs(s);

    if (reset_rises) {
        lp_ana_reset_request(s, !(mode0 & MODE0_RESET_SEL),
                             ESP32P4_RESET_CAUSE_BROWN_OUT);
    }
}

static void lp_ana_bod_cb(void *opaque)
{
    lp_ana_update(opaque);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_mode1_cb(void *opaque)
{
    ESP32P4LPAnaState *s = opaque;

    if (s->brownout && lp_ana_mode1_enabled(s)) {
        lp_ana_reset_request(s, false, ESP32P4_RESET_CAUSE_BROWN_OUT);
    }
}

static void lp_ana_filter_cb(void *opaque)
{
    ESP32P4LPAnaFilter *fl = opaque;

    lp_ana_update(fl->s);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_psdet_cb(void *opaque)
{
    ESP32P4LPAnaState *s = opaque;

    lp_ana_reset_request(s, false, ESP32P4_RESET_CAUSE_PSDET);
}

static uint32_t lp_ana_value(ESP32P4LPAnaState *s, hwaddr reg)
{
    uint32_t *r = s->regs;

    switch (reg) {
    case A_INT_ST:
        return r[R(A_INT_RAW)] & r[R(A_INT_ENA)];
    case A_LP_INT_ST:
        return r[R(A_LP_INT_RAW)] & r[R(A_LP_INT_ENA)];
    case A_VDDBAT_BOD_CNTL:
        return (r[R(reg)] & ~FILTER_FLAG) |
               (s->filter[ESP32P4_LP_ANA_FILTER_VBAT].flag ? FILTER_FLAG : 0);
    case A_VDDBAT_CHARGE_CNTL:
        return (r[R(reg)] & ~FILTER_FLAG) |
               (s->filter[ESP32P4_LP_ANA_FILTER_CHARGER].flag ?
                FILTER_FLAG : 0);
    default:
        return r[R(reg)];
    }
}

static uint64_t lp_ana_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LPAnaState *s = opaque;
    hwaddr reg = addr & ~3;

    if (!esp32p4_lp_ana_reg_info(reg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_ANA,
                      size, addr);
        return 0;
    }
    return extract32(lp_ana_value(s, reg), (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    ESP32P4LPAnaState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;
    const LPAnaReg *info = esp32p4_lp_ana_reg_info(reg);
    uint32_t *r;
    uint32_t wt;

    if (!info) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_LP_ANA, size, addr, value);
        return;
    }
    if (!(info->rw | info->wt | info->wtc)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_ANA, addr);
        return;
    }
    r = &s->regs[R(reg)];
    *r = (*r & ~(info->rw & lanes)) | (v & info->rw);
    *r &= ~(v & info->wtc);
    wt = v & info->wt;

    if (reg >= A_TOUCH_FIRST && reg <= A_TOUCH_LAST) {
        qemu_log_mask(LOG_UNIMP, "%s: the touch sensor is not modelled "
                      "(write of 0x%" PRIx64 " at 0x%" HWADDR_PRIx ")\n",
                      TYPE_ESP32P4_LP_ANA, value, addr);
        return;
    }
    switch (reg) {
    case A_VDD_SOURCE_CNTL:
        *r &= ~(extract32(wt, VBAT_EVENT_RECORD_CLR_SHIFT, 8) <<
                VGOOD_EVENT_RECORD_SHIFT);
        break;
    case A_VDDBAT_BOD_CNTL:
    case A_VDDBAT_CHARGE_CNTL:
        if (v & FILTER_CHARGER) {
            qemu_log_mask(LOG_UNIMP, "%s: charging the battery is not "
                          "modelled\n", TYPE_ESP32P4_LP_ANA);
        }
        break;
    case A_INT_CLR:
        s->regs[R(A_INT_RAW)] &= ~wt;
        break;
    case A_LP_INT_CLR:
        s->regs[R(A_LP_INT_RAW)] &= ~wt;
        break;
    default:
        break;
    }
    /* The mode-0 interrupt is a level: clearing it while it holds is moot. */
    if (s->bod_int) {
        s->regs[R(A_INT_RAW)] |= INT_BOD_MODE0;
        s->regs[R(A_LP_INT_RAW)] |= INT_BOD_MODE0;
    }
    lp_ana_update(s);
}

static const MemoryRegionOps lp_ana_ops = {
    .read = lp_ana_read,
    .write = lp_ana_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void lp_ana_fast_update(void *opaque, ClockEvent event)
{
    ESP32P4LPAnaState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int f;

    if (event == ClockPreUpdate) {
        for (f = 0; f < ESP32P4_LP_ANA_FILTERS; f++) {
            lp_ana_filter_sync(s, f, now);
        }
        s->bod_base = lp_ana_bod_count(s, now);
        s->bod_start_ns = now;
        return;
    }
    lp_ana_update(s);
}

static void lp_ana_set_bod_en(void *opaque, int n, int level)
{
    ESP32P4LPAnaState *s = opaque;

    s->bod_en = level;
    lp_ana_update(s);
}

static void lp_ana_set_vbat_sel(void *opaque, int n, int level)
{
    ESP32P4LPAnaState *s = opaque;

    s->vbat_sel = level;
    lp_ana_update(s);
}

static void lp_ana_bias_update(void *opaque, int n, int level)
{
    if (level) {
        lp_ana_update(opaque);
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_power_glitch(void *opaque, int n, int level)
{
    ESP32P4LPAnaState *s = opaque;

    if (level && (s->regs[R(A_PG_GLITCH_CNTL)] & GLITCH_RESET_ENA)) {
        lp_ana_reset_request(s, false, ESP32P4_RESET_CAUSE_GLITCH);
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_clock_glitch(void *opaque, int n, int level)
{
    ESP32P4LPAnaState *s = opaque;

    if (level && (s->regs[R(A_CK_GLITCH_CNTL)] & GLITCH_RESET_ENA)) {
        lp_ana_reset_request(s, false, ESP32P4_RESET_CAUSE_GLITCH);
    }
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_hp_supply_glitch(void *opaque, int n, int level)
{
    ESP32P4LPAnaState *s = opaque;
    uint32_t w = esp32p4_efuse_word(s->efuse, 0, EFUSE_PVT_WORD);

    s->hp_glitch = level;
    if (!level || !(w & EFUSE_PVT_GLITCH_EN)) {
        timer_del(&s->psdet_timer);
        return;
    }
    if (EFUSE_PVT_GLITCH_MODE(w)) {
        qemu_log_mask(LOG_UNIMP, "%s: EFUSE_PVT_GLITCH_MODE %u is not "
                      "modelled; the detector acts as with mode 0\n",
                      TYPE_ESP32P4_LP_ANA, EFUSE_PVT_GLITCH_MODE(w));
    }
    timer_mod(&s->psdet_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              PSDET_GLITCH_NS);
}

static void lp_ana_get_mv(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    uint32_t *mv = opaque;

    visit_type_uint32(v, name, mv, errp);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void lp_ana_set_mv(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    ESP32P4LPAnaState *s = ESP32P4_LP_ANA(obj);
    uint32_t *mv = opaque;
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    *mv = value;
    if (DEVICE(obj)->realized) {
        lp_ana_update(s);
    }
}

static void esp32p4_lp_ana_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPAnaState *s = ESP32P4_LP_ANA(obj);
    int i;

    for (i = 0; i < ESP32P4_LP_ANA_NREGS; i++) {
        const LPAnaReg *info = esp32p4_lp_ana_reg_info(i * 4);

        s->regs[i] = info ? info->reset : 0;
    }
    s->vdda_low = false;
    s->vbat_low = false;
    s->charger_low = false;
    s->brownout = false;
    s->bod_counting = false;
    s->bod_base = 0;
    s->bod_start_ns = 0;
    s->bod_int = false;
    s->bod_reset = false;
    timer_del(&s->bod_timer);
    timer_del(&s->mode1_timer);
    timer_del(&s->psdet_timer);
    for (i = 0; i < ESP32P4_LP_ANA_FILTERS; i++) {
        timer_del(&s->filter[i].timer);
        s->filter[i].count = 0;
        s->filter[i].dir = 0;
        s->filter[i].flag = false;
        s->filter[i].sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static void esp32p4_lp_ana_reset_exit(Object *obj, ResetType type)
{
    lp_ana_update(ESP32P4_LP_ANA(obj));
}

static void esp32p4_lp_ana_init(Object *obj)
{
    ESP32P4LPAnaState *s = ESP32P4_LP_ANA(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &lp_ana_ops, s,
                          "esp32p4.lp-analog-peri", ESP32P4_LP_ANA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->fast = qdev_init_clock_in(dev, "fast", lp_ana_fast_update, s,
                                 ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_out_named(dev, &s->lp_int, ESP32P4_LP_ANA_LP_INT, 1);
    qdev_init_gpio_out_named(dev, &s->brownout_out, ESP32P4_LP_ANA_BROWNOUT,
                             1);
    qdev_init_gpio_out_named(dev, &s->vbat_undervoltage_out,
                             ESP32P4_LP_ANA_VBAT_UNDERVOLTAGE, 1);
    qdev_init_gpio_in_named(dev, lp_ana_set_bod_en, ESP32P4_LP_ANA_BOD_EN, 1);
    qdev_init_gpio_in_named(dev, lp_ana_set_vbat_sel, ESP32P4_LP_ANA_VBAT_SEL,
                            1);
    qdev_init_gpio_in_named(dev, lp_ana_bias_update,
                            ESP32P4_LP_ANA_BIAS_UPDATE, 1);
    qdev_init_gpio_in_named(dev, lp_ana_power_glitch,
                            ESP32P4_LP_ANA_POWER_GLITCH, 1);
    qdev_init_gpio_in_named(dev, lp_ana_clock_glitch,
                            ESP32P4_LP_ANA_CLOCK_GLITCH, 1);
    qdev_init_gpio_in_named(dev, lp_ana_hp_supply_glitch,
                            ESP32P4_LP_ANA_HP_SUPPLY_GLITCH, 1);
    timer_init_ns(&s->bod_timer, QEMU_CLOCK_VIRTUAL, lp_ana_bod_cb, s);
    timer_init_ns(&s->mode1_timer, QEMU_CLOCK_VIRTUAL, lp_ana_mode1_cb, s);
    timer_init_ns(&s->psdet_timer, QEMU_CLOCK_VIRTUAL, lp_ana_psdet_cb, s);
    for (i = 0; i < ESP32P4_LP_ANA_FILTERS; i++) {
        s->filter[i].s = s;
        timer_init_ns(&s->filter[i].timer, QEMU_CLOCK_VIRTUAL,
                      lp_ana_filter_cb, &s->filter[i]);
    }

    s->vdda_mv = DEFAULT_SUPPLY_MV;
    s->vbat_mv = DEFAULT_SUPPLY_MV;
    /* LP_SYSTEM's analog FIB state enables the detector. */
    s->bod_en = true;
    object_property_add(obj, "vdda-mv", "uint32", lp_ana_get_mv,
                        lp_ana_set_mv, NULL, &s->vdda_mv);
    object_property_add(obj, "vbat-mv", "uint32", lp_ana_get_mv,
                        lp_ana_set_mv, NULL, &s->vbat_mv);
}

static void esp32p4_lp_ana_realize(DeviceState *dev, Error **errp)
{
    ESP32P4LPAnaState *s = ESP32P4_LP_ANA(dev);

    if (!s->clkrst || !s->ana || !s->efuse) {
        error_setg(errp, "%s: the \"clkrst\", \"ana\" and \"efuse\" links "
                   "must be set", TYPE_ESP32P4_LP_ANA);
    }
}

static void esp32p4_lp_ana_finalize(Object *obj)
{
    ESP32P4LPAnaState *s = ESP32P4_LP_ANA(obj);
    int i;

    timer_del(&s->bod_timer);
    timer_del(&s->mode1_timer);
    timer_del(&s->psdet_timer);
    for (i = 0; i < ESP32P4_LP_ANA_FILTERS; i++) {
        timer_del(&s->filter[i].timer);
    }
}

static const VMStateDescription vmstate_esp32p4_lp_ana_filter = {
    .name = TYPE_ESP32P4_LP_ANA "/filter",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4LPAnaFilter),
        VMSTATE_UINT32(count, ESP32P4LPAnaFilter),
        VMSTATE_INT64(sync_ns, ESP32P4LPAnaFilter),
        VMSTATE_INT32(dir, ESP32P4LPAnaFilter),
        VMSTATE_BOOL(flag, ESP32P4LPAnaFilter),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_lp_ana = {
    .name = TYPE_ESP32P4_LP_ANA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(fast, ESP32P4LPAnaState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4LPAnaState, ESP32P4_LP_ANA_NREGS),
        VMSTATE_UINT32(vdda_mv, ESP32P4LPAnaState),
        VMSTATE_UINT32(vbat_mv, ESP32P4LPAnaState),
        VMSTATE_BOOL(bod_en, ESP32P4LPAnaState),
        VMSTATE_BOOL(vbat_sel, ESP32P4LPAnaState),
        VMSTATE_BOOL(vdda_low, ESP32P4LPAnaState),
        VMSTATE_BOOL(vbat_low, ESP32P4LPAnaState),
        VMSTATE_BOOL(charger_low, ESP32P4LPAnaState),
        VMSTATE_BOOL(brownout, ESP32P4LPAnaState),
        VMSTATE_BOOL(bod_counting, ESP32P4LPAnaState),
        VMSTATE_UINT64(bod_base, ESP32P4LPAnaState),
        VMSTATE_INT64(bod_start_ns, ESP32P4LPAnaState),
        VMSTATE_TIMER(bod_timer, ESP32P4LPAnaState),
        VMSTATE_BOOL(bod_int, ESP32P4LPAnaState),
        VMSTATE_BOOL(bod_reset, ESP32P4LPAnaState),
        VMSTATE_TIMER(mode1_timer, ESP32P4LPAnaState),
        VMSTATE_STRUCT_ARRAY(filter, ESP32P4LPAnaState,
                             ESP32P4_LP_ANA_FILTERS, 1,
                             vmstate_esp32p4_lp_ana_filter,
                             ESP32P4LPAnaFilter),
        VMSTATE_TIMER(psdet_timer, ESP32P4LPAnaState),
        VMSTATE_BOOL(hp_glitch, ESP32P4LPAnaState),
        VMSTATE_BOOL(brownout_level, ESP32P4LPAnaState),
        VMSTATE_BOOL(vbat_undervoltage_level, ESP32P4LPAnaState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_lp_ana_properties[] = {
    DEFINE_PROP_LINK("clkrst", ESP32P4LPAnaState, clkrst,
                     TYPE_ESP32P4_CLKRST, ESP32P4ClkRstState *),
    DEFINE_PROP_LINK("ana", ESP32P4LPAnaState, ana, TYPE_ESP32P4_ANA,
                     ESP32P4AnaState *),
    DEFINE_PROP_LINK("efuse", ESP32P4LPAnaState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
};

static void esp32p4_lp_ana_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_lp_ana_reset_hold;
    rc->phases.exit = esp32p4_lp_ana_reset_exit;
    dc->realize = esp32p4_lp_ana_realize;
    dc->vmsd = &vmstate_esp32p4_lp_ana;
    device_class_set_props(dc, esp32p4_lp_ana_properties);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static const TypeInfo esp32p4_lp_ana_types[] = {
    {
        .name          = TYPE_ESP32P4_LP_ANA,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4LPAnaState),
        .instance_init = esp32p4_lp_ana_init,
        .instance_finalize = esp32p4_lp_ana_finalize,
        .class_init    = esp32p4_lp_ana_class_init,
    },
};

DEFINE_TYPES(esp32p4_lp_ana_types)
