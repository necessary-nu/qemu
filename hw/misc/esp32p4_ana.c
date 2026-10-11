/*
 * ESP32-P4 analog blocks on the analog I2C bus: the PLLs, BIAS, DIG_REG
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 11 (Reset and Clock) and chapter 48 (Analog I2C Controller); ESP-IDF's
 * soc/esp32p4 regi2c_cpll.h, regi2c_syspll.h, regi2c_mpll.h,
 * regi2c_apll.h, regi2c_bias.h, regi2c_dig_reg.h and regi2c_saradc.h,
 * hal/esp32p4 clk_tree_ll.h and regi2c_impl.c, and
 * esp_hw_support/port/esp32p4/rtc_clk.c, which show how software drives
 * them.
 *
 * The analog blocks the analog I2C master reaches, each an I2C slave
 * holding a file of 8-bit configuration registers: the CPU PLL (0x67),
 * the system PLL (0x66), the MSPI PLL (0x63), the audio PLL (PLLA, 0x6F),
 * the SDIO PLL (0x62), the analog sensors (0x69), BIAS (0x6A) and
 * DIG_REG (0x6D). Each slave has sixteen registers, which hold every
 * register ESP-IDF names; a transaction to a register beyond them is not
 * acknowledged.
 *
 * The PLLs multiply XTAL_CLK, at the frequencies their registers give:
 *
 *   CPLL_CLK = XTAL * OC_DIV_7_0 / (OC_REF_DIV + 1)   (v1.0 silicon on)
 *   SPLL_CLK = XTAL * OC_DIV_7_0 / (OC_REF_DIV + 1)   (the same layout)
 *   MPLL_CLK = XTAL * (DIV + 1) / (REF_DIV + 1)
 *   APLL_CLK = XTAL * (4 + SDM2 + SDM1 / 256 + SDM0 / 65536)
 *              / (2 (OR_OUTPUT_DIV + 2))
 *   SDIO_PLL0..2_CLK = 200 MHz
 *
 * The APLL's sigma-delta modulator adds the fraction, SDM1 and SDM0, only
 * while it runs: SDM_RSTB set and SDM_STOP clear. The SDIO PLL's register
 * layout is not published; it runs at the 200 MHz ESP-IDF gives it. Each
 * output's period is kept exactly (esp32p4_clk_period.h) for the clock
 * tree, which divides it; its Clock has it rounded down to a whole unit.
 *
 * Each PLL runs while the PMU powers it up ("xpd"). After power-up or a
 * change of its dividers it relocks, its output stopped meanwhile, and
 * the CPU and system PLLs report the lock in OR_LOCK. Its calibration runs
 * while HP_SYS_CLKRST_*_CAL_STOP is clear, from power-up, from the
 * control's clearing and from a change of the dividers, and reports its
 * end in HP_SYS_CLKRST_*_CAL_END and, for the CPU, system and audio PLLs,
 * in OR_CAL_END; software can also run it from the slave, by clearing
 * IR_CAL_RSTB, which resets it, and raising IR_CAL_START. The TRM gives
 * neither time: a PLL locks 10 us after power-up or a change, and a
 * calibration ends 20 us after it starts. A chip reset finds the powered
 * PLLs locked, as the PMU releases the CPUs only once they are stable.
 *
 * The register values after power-up are not published. The model takes
 * the PLLs to come up at their nominal frequencies, the CPU PLL at the
 * 360 MHz ESP-IDF keeps it at, the system PLL at 480 MHz and the MSPI PLL
 * at the 500 MHz its dividers in HP_SYS_CLKRST assume, with OC_DCHGP and
 * the reference selections ESP-IDF programs; the audio PLL comes up with
 * its coefficients clear, and everything else clear. The results of the
 * calibrations, OR_CAL_CAP, OR_CAL_UDF and OR_CAL_OVF, read 0.
 *
 * The PMU powers the PLLs ("xpd"). Until it drives them the CPU and
 * system PLLs are powered, as the PMU's power-on has them (ESP-IDF relies
 * on the system PLL without powering it up), and the audio, MSPI and SDIO
 * PLLs, which ESP-IDF powers up through the PMU, are not.
 *
 * BIAS holds the brown-out detector's thresholds, which the detector
 * (esp32p4_lp_ana.c) takes from it; their power-up values are the 2.4 V
 * TRM 25.1 gives as the default.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32p4_ana.h"
#include "migration/vmstate.h"

#define PLL_LOCK_NS (10 * SCALE_US)
#define PLL_CAL_NS  (20 * SCALE_US)

/* The CPU and system PLLs' registers (regi2c_cpll.h, regi2c_syspll.h). */
#define XPLL_IR_CAL         1
#define XPLL_IR_CAL_RSTB    BIT(5)
#define XPLL_IR_CAL_START   BIT(6)
#define XPLL_OC_REF_DIV     2
#define XPLL_OC_DIV_7_0     3
#define XPLL_OR             8
#define XPLL_OR_CAL_END     BIT(6)
#define XPLL_OR_LOCK        BIT(7)

/* The MSPI PLL's (regi2c_mpll.h). */
#define MPLL_IR_CAL         1
#define MPLL_IR_CAL_RSTB    BIT(5)
#define MPLL_DIV            2

/* The audio PLL's (regi2c_apll.h). */
#define APLL_IR_CAL         0
#define APLL_IR_CAL_RSTB    BIT(4)
#define APLL_IR_CAL_START   BIT(5)
#define APLL_OR             3
#define APLL_OR_CAL_END     BIT(7)
#define APLL_OR_OUTPUT_DIV  4
#define APLL_SDM            5
#define APLL_SDM_STOP       BIT(5)
#define APLL_SDM_RSTB       BIT(6)
#define APLL_DSDM2          7
#define APLL_DSDM1          8
#define APLL_DSDM0          9

/* The APLL's multiplier works from 350 to 500 MHz (clk_tree_ll.h). */
#define APLL_VCO_MIN_HZ     350000000ull
#define APLL_VCO_MAX_HZ     500000000ull

#define SDIO_PLL_MULT       5

const char *const esp32p4_ana_pll_names[ESP32P4_ANA_NUM_PLLS] = {
    [ESP32P4_ANA_PLL_APLL] = "apll",
    [ESP32P4_ANA_PLL_CPLL] = "cpll",
    [ESP32P4_ANA_PLL_SDIO] = "sdio-pll",
    [ESP32P4_ANA_PLL_SPLL] = "spll",
    [ESP32P4_ANA_PLL_MPLL] = "mpll",
};

static const struct {
    uint8_t addr;
    const char *name;
    /* The PLL whose registers these are, or -1. */
    int pll;
    uint8_t reset[ESP32P4_ANA_SLAVE_REGS];
} esp32p4_ana_slaves[ESP32P4_ANA_NUM_SLAVES] = {
    [ESP32P4_ANA_SLAVE_SDIO_PLL] = {
        .addr = 0x62, .name = "SDIO_PLL", .pll = ESP32P4_ANA_PLL_SDIO,
    },
    [ESP32P4_ANA_SLAVE_MSPI] = {
        .addr = 0x63, .name = "MSPI", .pll = ESP32P4_ANA_PLL_MPLL,
        .reset = {
            [MPLL_IR_CAL] = MPLL_IR_CAL_RSTB,
            /* DIV 24, REF_DIV 1: 500 MHz. */
            [MPLL_DIV] = (24 << 3) | 1,
        },
    },
    [ESP32P4_ANA_SLAVE_SYS_PLL] = {
        .addr = 0x66, .name = "SYS_PLL", .pll = ESP32P4_ANA_PLL_SPLL,
        .reset = {
            [XPLL_IR_CAL] = XPLL_IR_CAL_RSTB,
            [XPLL_OC_REF_DIV] = 5 << 4,
            [XPLL_OC_DIV_7_0] = 12,
            [6] = (1 << 6) | (3 << 4) | 3,
        },
    },
    [ESP32P4_ANA_SLAVE_CPU_PLL] = {
        .addr = 0x67, .name = "CPU_PLL", .pll = ESP32P4_ANA_PLL_CPLL,
        .reset = {
            [XPLL_IR_CAL] = XPLL_IR_CAL_RSTB,
            [XPLL_OC_REF_DIV] = 5 << 4,
            [XPLL_OC_DIV_7_0] = 9,
            [6] = (1 << 6) | (3 << 4) | 3,
        },
    },
    [ESP32P4_ANA_SLAVE_SAR] = {
        .addr = 0x69, .name = "ANA_SENSOR", .pll = -1,
    },
    [ESP32P4_ANA_SLAVE_BIAS] = {
        .addr = 0x6a, .name = "BIAS", .pll = -1,
        .reset = {
            /*
             * The brown-out detector's thresholds for VDD_BAT and VDD_ANA,
             * the under-voltage one in bits 4:2 and the release one in bits
             * 7:5 (TRM 25.4), at the 2.4 V (code 4) TRM 25.1 gives.
             */
            [ESP32P4_ANA_BIAS_DREF_VBAT] = (4 << 5) | (4 << 2),
            [ESP32P4_ANA_BIAS_DREF_VDDA] = (4 << 5) | (4 << 2),
        },
    },
    [ESP32P4_ANA_SLAVE_DIG_REG] = {
        .addr = 0x6d, .name = "DIG_REG", .pll = -1,
    },
    [ESP32P4_ANA_SLAVE_PLLA] = {
        .addr = 0x6f, .name = "PLLA", .pll = ESP32P4_ANA_PLL_APLL,
        .reset = {
            [APLL_IR_CAL] = APLL_IR_CAL_RSTB,
        },
    },
};

static const int esp32p4_ana_pll_slave[ESP32P4_ANA_NUM_PLLS] = {
    [ESP32P4_ANA_PLL_APLL] = ESP32P4_ANA_SLAVE_PLLA,
    [ESP32P4_ANA_PLL_CPLL] = ESP32P4_ANA_SLAVE_CPU_PLL,
    [ESP32P4_ANA_PLL_SDIO] = ESP32P4_ANA_SLAVE_SDIO_PLL,
    [ESP32P4_ANA_PLL_SPLL] = ESP32P4_ANA_SLAVE_SYS_PLL,
    [ESP32P4_ANA_PLL_MPLL] = ESP32P4_ANA_SLAVE_MSPI,
};

/* Whether each PLL is powered until the PMU, which powers it, says. */
static const bool esp32p4_ana_xpd_default[ESP32P4_ANA_NUM_PLLS] = {
    [ESP32P4_ANA_PLL_CPLL] = true,
    [ESP32P4_ANA_PLL_SPLL] = true,
};

int esp32p4_ana_slave(uint8_t addr)
{
    int i;

    for (i = 0; i < ESP32P4_ANA_NUM_SLAVES; i++) {
        if (esp32p4_ana_slaves[i].addr == addr) {
            return i;
        }
    }
    return -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * The period the PLL's registers configure, from XTAL_CLK's, exactly; or
 * stopped.
 */
static ESP32P4ClkPeriod esp32p4_ana_target(ESP32P4AnaState *s, int pll)
{
    const uint8_t *r = s->regs[esp32p4_ana_pll_slave[pll]];
    ESP32P4ClkPeriod xtal = esp32p4_clk_exact(s->xtal);
    uint32_t div, ref, odiv, num;

    switch (pll) {
    case ESP32P4_ANA_PLL_CPLL:
    case ESP32P4_ANA_PLL_SPLL:
        div = r[XPLL_OC_DIV_7_0];
        ref = extract32(r[XPLL_OC_REF_DIV], 0, 4);
        return esp32p4_clk_period_scale(xtal, ref + 1, div);
    case ESP32P4_ANA_PLL_MPLL:
        div = extract32(r[MPLL_DIV], 3, 5);
        ref = extract32(r[MPLL_DIV], 0, 3);
        return esp32p4_clk_period_scale(xtal, ref + 1, div + 1);
    case ESP32P4_ANA_PLL_APLL:
        odiv = extract32(r[APLL_OR_OUTPUT_DIV], 0, 5);
        num = (4 + extract32(r[APLL_DSDM2], 0, 6)) << 16;
        if ((r[APLL_SDM] & APLL_SDM_RSTB) && !(r[APLL_SDM] & APLL_SDM_STOP)) {
            num += (r[APLL_DSDM1] << 8) | r[APLL_DSDM0];
        }
        return esp32p4_clk_period_scale(xtal, 2 * (odiv + 2) * 65536ull,
                                        num);
    case ESP32P4_ANA_PLL_SDIO:
    default:
        return esp32p4_clk_period_scale(xtal, 1, SDIO_PLL_MULT);
    }
}

/* The output's period: the target's while powered up and locked. */
static ESP32P4ClkPeriod esp32p4_ana_out_period(ESP32P4AnaState *s, int pll)
{
    if (s->xpd[pll] && s->locked[pll]) {
        return esp32p4_ana_target(s, pll);
    }
    return esp32p4_clk_period_stopped();
}

static void esp32p4_ana_update_out(ESP32P4AnaState *s, int pll)
{
    s->out_period[pll] = esp32p4_ana_out_period(s, pll);
    clock_update(s->out[pll], esp32p4_clk_period_clock(s->out_period[pll]));
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static bool esp32p4_ana_exact_period(Object *obj, Clock *out,
                                     ESP32P4ClkPeriod *p)
{
    ESP32P4AnaState *s = ESP32P4_ANA(obj);
    int pll;

    for (pll = 0; pll < ESP32P4_ANA_NUM_PLLS; pll++) {
        if (s->out[pll] == out) {
            *p = s->out_period[pll];
            return true;
        }
    }
    return false;
}

static void esp32p4_ana_set_cal_end(ESP32P4AnaState *s, int pll, bool end)
{
    s->cal_end[pll] = end;
    qemu_set_irq(s->cal_end_out[pll], end);
}

static void esp32p4_ana_check_apll(ESP32P4AnaState *s)
{
    const uint8_t *r = s->regs[ESP32P4_ANA_SLAVE_PLLA];
    uint64_t vco = (uint64_t)clock_get_hz(s->xtal) *
                   (4 + extract32(r[APLL_DSDM2], 0, 6));

    if (vco < APLL_VCO_MIN_HZ || vco > APLL_VCO_MAX_HZ) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: the APLL multiplier's output, %" PRIu64 " Hz, is "
                      "outside its %llu-%llu Hz range\n", TYPE_ESP32P4_ANA,
                      vco, APLL_VCO_MIN_HZ, APLL_VCO_MAX_HZ);
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_ana_start_cal(ESP32P4AnaState *s, int pll)
{
    if (!s->xpd[pll]) {
        return;
    }
    if (pll == ESP32P4_ANA_PLL_APLL) {
        esp32p4_ana_check_apll(s);
    }
    esp32p4_ana_set_cal_end(s, pll, false);
    timer_mod(s->cal_timer[pll],
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + PLL_CAL_NS);
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static void esp32p4_ana_relock(ESP32P4AnaState *s, int pll)
{
    s->locked[pll] = false;
    timer_del(s->lock_timer[pll]);
    if (s->xpd[pll]) {
        timer_mod(s->lock_timer[pll],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + PLL_LOCK_NS);
    }
    esp32p4_ana_update_out(s, pll);
}

static void esp32p4_ana_lock_done(void *opaque)
{
    ESP32P4AnaPllTimer *t = opaque;

    t->s->locked[t->pll] = true;
    esp32p4_ana_update_out(t->s, t->pll);
}

static void esp32p4_ana_cal_done(void *opaque)
{
    ESP32P4AnaPllTimer *t = opaque;

    esp32p4_ana_set_cal_end(t->s, t->pll, true);
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * The PMU powers a PLL up or down. Powered up, it locks and, unless
 * stopped, calibrates; powered down, it stops and loses both.
 */
static void esp32p4_ana_set_xpd(void *opaque, int pll, int level)
{
    ESP32P4AnaState *s = opaque;

    if (s->xpd[pll] == !!level) {
        return;
    }
    s->xpd[pll] = level;
    if (level) {
        esp32p4_ana_relock(s, pll);
        if (!s->cal_stop[pll]) {
            esp32p4_ana_start_cal(s, pll);
        }
    } else {
        timer_del(s->cal_timer[pll]);
        esp32p4_ana_set_cal_end(s, pll, false);
        esp32p4_ana_relock(s, pll);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * HP_SYS_CLKRST_*_CAL_STOP: clearing it starts a calibration; setting it
 * stops one in progress, which then does not end.
 */
static void esp32p4_ana_set_cal_stop(void *opaque, int pll, int level)
{
    ESP32P4AnaState *s = opaque;

    if (s->cal_stop[pll] == !!level) {
        return;
    }
    s->cal_stop[pll] = level;
    if (level) {
        timer_del(s->cal_timer[pll]);
    } else {
        esp32p4_ana_start_cal(s, pll);
    }
}

/* The register of @slave that reports its PLL's state, with its value. */
static bool esp32p4_ana_status(ESP32P4AnaState *s, int slave, uint8_t reg,
                               uint8_t *value)
{
    int pll = esp32p4_ana_slaves[slave].pll;

    switch (slave) {
    case ESP32P4_ANA_SLAVE_CPU_PLL:
    case ESP32P4_ANA_SLAVE_SYS_PLL:
        if (reg != XPLL_OR) {
            return false;
        }
        *value = (s->cal_end[pll] ? XPLL_OR_CAL_END : 0) |
                 (s->xpd[pll] && s->locked[pll] ? XPLL_OR_LOCK : 0);
        return true;
    case ESP32P4_ANA_SLAVE_PLLA:
        if (reg != APLL_OR) {
            return false;
        }
        *value = s->cal_end[pll] ? APLL_OR_CAL_END : 0;
        return true;
    default:
        return false;
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
bool esp32p4_ana_read(ESP32P4AnaState *s, int slave, uint8_t reg,
                      uint8_t *data)
{
    if (slave < 0 || slave >= ESP32P4_ANA_NUM_SLAVES ||
        reg >= ESP32P4_ANA_SLAVE_REGS) {
        return false;
    }
    if (!esp32p4_ana_status(s, slave, reg, data)) {
        *data = s->regs[slave][reg];
    }
    return true;
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A write to a slave's register. The PLLs' status registers ignore it. A
 * PLL whose frequency it changes relocks and, unless stopped,
 * recalibrates; a PLL's own calibration controls start and reset its
 * calibration.
 */
bool esp32p4_ana_write(ESP32P4AnaState *s, int slave, uint8_t reg,
                       uint8_t data)
{
    int pll;
    uint8_t old, scratch;
    ESP32P4ClkPeriod before;

    if (slave < 0 || slave >= ESP32P4_ANA_NUM_SLAVES ||
        reg >= ESP32P4_ANA_SLAVE_REGS) {
        return false;
    }
    if (esp32p4_ana_status(s, slave, reg, &scratch)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to %s's read-only "
                      "register %u\n", TYPE_ESP32P4_ANA,
                      esp32p4_ana_slaves[slave].name, reg);
        return true;
    }
    pll = esp32p4_ana_slaves[slave].pll;
    if (pll < 0) {
        s->regs[slave][reg] = data;
        if (slave == ESP32P4_ANA_SLAVE_BIAS) {
            qemu_irq_pulse(s->bias_update);
        } else if (slave == ESP32P4_ANA_SLAVE_SAR) {
            qemu_irq_pulse(s->sar_update);
        }
        return true;
    }

    before = esp32p4_ana_target(s, pll);
    old = s->regs[slave][reg];
    s->regs[slave][reg] = data;

    switch (slave) {
    case ESP32P4_ANA_SLAVE_CPU_PLL:
    case ESP32P4_ANA_SLAVE_SYS_PLL:
        if (reg == XPLL_IR_CAL) {
            if (!(data & XPLL_IR_CAL_RSTB)) {
                timer_del(s->cal_timer[pll]);
                esp32p4_ana_set_cal_end(s, pll, false);
            } else if (data & ~old & XPLL_IR_CAL_START) {
                esp32p4_ana_start_cal(s, pll);
            }
        }
        break;
    case ESP32P4_ANA_SLAVE_MSPI:
        if (reg == MPLL_IR_CAL && !(data & MPLL_IR_CAL_RSTB)) {
            timer_del(s->cal_timer[pll]);
            esp32p4_ana_set_cal_end(s, pll, false);
        }
        break;
    case ESP32P4_ANA_SLAVE_PLLA:
        if (reg == APLL_IR_CAL) {
            if (!(data & APLL_IR_CAL_RSTB)) {
                timer_del(s->cal_timer[pll]);
                esp32p4_ana_set_cal_end(s, pll, false);
            } else if (data & ~old & APLL_IR_CAL_START) {
                esp32p4_ana_start_cal(s, pll);
            }
        }
        break;
    default:
        break;
    }

    if (!esp32p4_clk_period_eq(esp32p4_ana_target(s, pll), before)) {
        esp32p4_ana_relock(s, pll);
        if (!s->cal_stop[pll]) {
            esp32p4_ana_start_cal(s, pll);
        }
    }
    return true;
}

static void esp32p4_ana_xtal_update(void *opaque, ClockEvent event)
{
    ESP32P4AnaState *s = opaque;
    int pll;

    for (pll = 0; pll < ESP32P4_ANA_NUM_PLLS; pll++) {
        esp32p4_ana_update_out(s, pll);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.clocks]
 * A chip reset: the registers take their power-up values, and the
 * powered PLLs run locked, calibrating unless stopped.
 */
static void esp32p4_ana_reset_hold(Object *obj, ResetType type)
{
    ESP32P4AnaState *s = ESP32P4_ANA(obj);
    int i;

    for (i = 0; i < ESP32P4_ANA_NUM_SLAVES; i++) {
        memcpy(s->regs[i], esp32p4_ana_slaves[i].reset,
               ESP32P4_ANA_SLAVE_REGS);
    }
    for (i = 0; i < ESP32P4_ANA_NUM_PLLS; i++) {
        timer_del(s->lock_timer[i]);
        timer_del(s->cal_timer[i]);
        s->locked[i] = s->xpd[i];
        esp32p4_ana_set_cal_end(s, i, false);
        if (!s->cal_stop[i]) {
            esp32p4_ana_start_cal(s, i);
        }
        esp32p4_ana_update_out(s, i);
    }
}

static void esp32p4_ana_init(Object *obj)
{
    ESP32P4AnaState *s = ESP32P4_ANA(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    s->xtal = qdev_init_clock_in(dev, "xtal", esp32p4_ana_xtal_update, s,
                                 ClockUpdate);
    for (i = 0; i < ESP32P4_ANA_NUM_PLLS; i++) {
        s->out[i] = qdev_init_clock_out(dev, esp32p4_ana_pll_names[i]);
        s->timer_ctx[i].s = s;
        s->timer_ctx[i].pll = i;
        s->lock_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        esp32p4_ana_lock_done,
                                        &s->timer_ctx[i]);
        s->cal_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       esp32p4_ana_cal_done,
                                       &s->timer_ctx[i]);
        s->xpd[i] = esp32p4_ana_xpd_default[i];
    }
    qdev_init_gpio_in_named(dev, esp32p4_ana_set_xpd, ESP32P4_ANA_XPD,
                            ESP32P4_ANA_NUM_PLLS);
    qdev_init_gpio_in_named(dev, esp32p4_ana_set_cal_stop,
                            ESP32P4_ANA_CAL_STOP, ESP32P4_ANA_NUM_PLLS);
    qdev_init_gpio_out_named(dev, s->cal_end_out, ESP32P4_ANA_CAL_END,
                             ESP32P4_ANA_NUM_PLLS);
    qdev_init_gpio_out_named(dev, &s->bias_update, ESP32P4_ANA_BIAS_UPDATE,
                             1);
    qdev_init_gpio_out_named(dev, &s->sar_update, ESP32P4_ANA_SAR_UPDATE, 1);
}

static int esp32p4_ana_post_load(void *opaque, int version_id)
{
    ESP32P4AnaState *s = opaque;
    int i;

    for (i = 0; i < ESP32P4_ANA_NUM_PLLS; i++) {
        s->out_period[i] = esp32p4_ana_out_period(s, i);
        clock_set(s->out[i], esp32p4_clk_period_clock(s->out_period[i]));
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_ana = {
    .name = TYPE_ESP32P4_ANA,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32p4_ana_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(xtal, ESP32P4AnaState),
        VMSTATE_UINT8_2DARRAY(regs, ESP32P4AnaState, ESP32P4_ANA_NUM_SLAVES,
                              ESP32P4_ANA_SLAVE_REGS),
        VMSTATE_BOOL_ARRAY(xpd, ESP32P4AnaState, ESP32P4_ANA_NUM_PLLS),
        VMSTATE_BOOL_ARRAY(cal_stop, ESP32P4AnaState, ESP32P4_ANA_NUM_PLLS),
        VMSTATE_BOOL_ARRAY(locked, ESP32P4AnaState, ESP32P4_ANA_NUM_PLLS),
        VMSTATE_BOOL_ARRAY(cal_end, ESP32P4AnaState, ESP32P4_ANA_NUM_PLLS),
        VMSTATE_TIMER_PTR_ARRAY(lock_timer, ESP32P4AnaState,
                                ESP32P4_ANA_NUM_PLLS),
        VMSTATE_TIMER_PTR_ARRAY(cal_timer, ESP32P4AnaState,
                                ESP32P4_ANA_NUM_PLLS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_ana_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    ESP32P4ClkSourceClass *csc = ESP32P4_CLK_SOURCE_CLASS(klass);

    rc->phases.hold = esp32p4_ana_reset_hold;
    dc->vmsd = &vmstate_esp32p4_ana;
    csc->exact_period = esp32p4_ana_exact_period;
}

/* [spec:nuos:req:emu.esp32p4.clocks] */
static const TypeInfo esp32p4_ana_info = {
    .name          = TYPE_ESP32P4_ANA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4AnaState),
    .instance_init = esp32p4_ana_init,
    .class_init    = esp32p4_ana_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { TYPE_ESP32P4_CLK_SOURCE },
        { }
    },
};

static void esp32p4_ana_register_types(void)
{
    type_register_static(&esp32p4_ana_info);
}
type_init(esp32p4_ana_register_types)
