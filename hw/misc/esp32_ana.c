/*
 * ESP32 analog block: the internal analog I2C bus to the BBPLL and APLL
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM "Reset and Clock" (PLL_CLK, APLL_CLK); ESP-IDF's
 * soc/esp32 regi2c_bbpll.h, regi2c_apll.h and regi2c_defs.h,
 * hal/esp32 clk_tree_ll.h and esp_hw_support/port/esp32/rtc_clk.c; and the
 * ESP32 ROM's I2C routines (rom_i2c_readReg at 0x40004148 and the
 * functions behind it), which show the master's register interface.
 *
 * The analog blocks hang off an internal I2C bus. Software reaches it
 * through a master in this block: writing a host register starts a
 * transaction to the slave in SLAVE, at register ADDR; with WR set it
 * writes DATA, otherwise the byte read lands in DATA. BUSY is set until
 * the transaction is over. The model completes a transaction within the
 * register write, so BUSY always reads 0.
 *
 * Two slaves are modelled: the BBPLL (0x66), whose configuration selects
 * PLL_CLK's 320 MHz or 480 MHz, and the APLL (0x6d), whose sigma-delta
 * coefficients and output divider give
 *   APLL_CLK = XTAL * (4 + SDM2 + SDM1 / 256 + SDM0 / 65536) / (2 (ODIV + 2)).
 * The bus is powered down by RTC_CNTL_BIAS_I2C_FORCE_PD; the BBPLL's slave
 * by RTC_CNTL_BBPLL_I2C_FORCE_PD; and ANA_CONFIG disconnects each PLL's
 * slave. A transaction to a slave that is powered down, disconnected or
 * absent is not acknowledged: nothing is written, and a read returns 0.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/misc/esp32_ana.h"
#include "hw/misc/esp32_rtc_cntl.h"

/* BBPLL registers (soc/regi2c_bbpll.h) */
#define BBPLL_IR_CAL            0
#define BBPLL_IR_CAL_RSTB       (1u << 5)
#define BBPLL_IR_CAL_START      (1u << 6)
#define BBPLL_OC_LREF           2
#define BBPLL_OC_DIV_7_0        3
#define BBPLL_OR_LOCK           6
#define BBPLL_OR_LOCK_BOTH      0x03
#define BBPLL_OR_CAL            7
#define BBPLL_OR_CAL_END        (1u << 6)
#define BBPLL_ENDIV5            11
#define BBPLL_ENDIV5_BIT        (1u << 7)
#define BBPLL_REG_COUNT         13

/* APLL registers (soc/regi2c_apll.h) */
#define APLL_IR_CAL             0
#define APLL_IR_CAL_RSTB        (1u << 4)
#define APLL_IR_CAL_START       (1u << 5)
#define APLL_OR_CAL             3
#define APLL_OR_CAL_END         (1u << 7)
#define APLL_OR_OUTPUT_DIV      4
#define APLL_OUTPUT_DIV_MASK    0x1f
#define APLL_SDM                5
#define APLL_SDM_STOP           (1u << 5)
#define APLL_SDM_RSTB           (1u << 6)
#define APLL_DSDM2              7
#define APLL_DSDM2_MASK         0x3f
#define APLL_DSDM1              8
#define APLL_DSDM0              9
#define APLL_REG_COUNT          10

/* The APLL multiplier's working range (hal/esp32 clk_tree_ll.h) */
#define APLL_VCO_MIN_HZ         350000000ull
#define APLL_VCO_MAX_HZ         500000000ull

#define PLL_320M_HZ             320000000
#define PLL_480M_HZ             480000000

/*
 * The BBPLL's registers after power-on. ESP-IDF's clk_tree_ll.h gives
 * IR_CAL_DELAY, IR_CAL_EXT_CAP, OC_ENB_FCAL, OC_ENB_VCON and BBADC_CAL as
 * their reset values; the dividers are not documented and are taken to
 * hold the configuration for 320 MHz from the 40 MHz crystal, the one
 * ESP-IDF itself programs for an 80 MHz or 160 MHz CPU_CLK.
 */
static const uint8_t bbpll_reset[ESP32_ANA_BANK_SIZE] = {
    [BBPLL_IR_CAL] = 0x18,
    [1] = 0x20,
    [BBPLL_OC_LREF] = 0x00,
    [BBPLL_OC_DIV_7_0] = 32,
    [4] = 0x9a,
    [5] = 0xc6,
    [9] = 0x84,
    [BBPLL_ENDIV5] = 0x43,
};

/*
 * The divider settings with which the BBPLL locks, per crystal frequency
 * (ESP-IDF's clk_ll_bbpll_set_config): OC_LREF (LREF, DIV_10_8, DIV_REF)
 * and OC_DIV_7_0, for 320 MHz and for 480 MHz.
 */
static const struct {
    uint32_t xtal_hz;
    uint8_t lref;
    uint8_t div_320, div_480;
} bbpll_configs[] = {
    { 40000000, 0x00, 32, 28 },
    { 26000000, 0xcc, 224, 144 },
    { 24000000, 0xcb, 224, 144 },
};

static uint32_t rtc_reg(Esp32AnaState *s, hwaddr addr)
{
    return s->rtc_cntl->regs[addr / 4];
}

static bool bbpll_powered(Esp32AnaState *s)
{
    return !FIELD_EX32(rtc_reg(s, A_RTC_CNTL_OPTIONS0), RTC_CNTL_OPTIONS0,
                       BBPLL_FORCE_PD);
}

/*
 * The APLL has no automatic power control: it runs only while software
 * forces it up, and RTC_CNTL_PLLA_FORCE_PD, set after reset, wins.
 */
static bool apll_powered(Esp32AnaState *s)
{
    uint32_t ana = rtc_reg(s, A_RTC_CNTL_ANA_CONF);

    return FIELD_EX32(ana, RTC_CNTL_ANA_CONF, PLLA_FORCE_PU) &&
           !FIELD_EX32(ana, RTC_CNTL_ANA_CONF, PLLA_FORCE_PD);
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * PLL_CLK: ENDIV5 selects the BBPLL's 480 MHz configuration, clear its
 * 320 MHz one.
 */
uint32_t esp32_ana_pll_hz(Esp32AnaState *s)
{
    if (!bbpll_powered(s)) {
        return 0;
    }
    return (s->bbpll[BBPLL_ENDIV5] & BBPLL_ENDIV5_BIT) ? PLL_480M_HZ :
                                                        PLL_320M_HZ;
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * The PLL locks with the dividers ESP-IDF programs for the crystal; other
 * divider settings are not characterised, and are taken to give the
 * frequency ENDIV5 selects. Software reprograms the dividers one register
 * at a time, so they are only checked while PLL_CLK is in use.
 */
void esp32_ana_check_pll(Esp32AnaState *s)
{
    bool is_480 = s->bbpll[BBPLL_ENDIV5] & BBPLL_ENDIV5_BIT;
    uint32_t xtal = s->rtc_cntl->xtal_apb_freq;
    uint32_t cfg;
    bool known = false;

    if (!bbpll_powered(s)) {
        return;
    }
    for (int i = 0; i < ARRAY_SIZE(bbpll_configs); i++) {
        if (bbpll_configs[i].xtal_hz == xtal &&
            bbpll_configs[i].lref == s->bbpll[BBPLL_OC_LREF] &&
            (is_480 ? bbpll_configs[i].div_480 : bbpll_configs[i].div_320) ==
            s->bbpll[BBPLL_OC_DIV_7_0]) {
            known = true;
        }
    }
    cfg = (is_480 << 16) | (s->bbpll[BBPLL_OC_LREF] << 8) |
          s->bbpll[BBPLL_OC_DIV_7_0];
    if (!known && cfg != s->logged_bbpll_cfg) {
        qemu_log_mask(LOG_UNIMP,
                      "esp32_ana: BBPLL dividers OC_LREF 0x%02x DIV_7_0 %u "
                      "for %u MHz are not characterised for a %u Hz "
                      "crystal; PLL_CLK taken as %u MHz\n",
                      s->bbpll[BBPLL_OC_LREF], s->bbpll[BBPLL_OC_DIV_7_0],
                      is_480 ? 480 : 320, xtal, is_480 ? 480 : 320);
        s->logged_bbpll_cfg = cfg;
    }
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * APLL_CLK from its coefficients. The sigma-delta modulator adds the
 * fractional part, SDM1 and SDM0, only while it runs: out of reset
 * (SDM_RSTB set) and not stopped.
 */
uint32_t esp32_ana_apll_hz(Esp32AnaState *s)
{
    uint64_t xtal = s->rtc_cntl->xtal_apb_freq;
    uint32_t odiv = s->apll[APLL_OR_OUTPUT_DIV] & APLL_OUTPUT_DIV_MASK;
    uint32_t sdm2 = s->apll[APLL_DSDM2] & APLL_DSDM2_MASK;
    uint32_t frac = 0;
    uint64_t vco_x65536;

    if (!apll_powered(s)) {
        return 0;
    }
    if ((s->apll[APLL_SDM] & APLL_SDM_RSTB) &&
        !(s->apll[APLL_SDM] & APLL_SDM_STOP)) {
        frac = (s->apll[APLL_DSDM1] << 8) | s->apll[APLL_DSDM0];
    }
    vco_x65536 = xtal * (((4 + sdm2) << 16) + frac);
    return vco_x65536 / (65536ull * 2 * (odiv + 2));
}

/*
 * The multiplier must be in its working range for the APLL to calibrate;
 * the coefficients are checked when software starts the calibration, as
 * ESP-IDF does after programming them.
 */
static void apll_check_range(Esp32AnaState *s)
{
    uint64_t xtal = s->rtc_cntl->xtal_apb_freq;
    uint32_t sdm2 = s->apll[APLL_DSDM2] & APLL_DSDM2_MASK;
    uint64_t vco = xtal * (4 + sdm2);

    if (vco < APLL_VCO_MIN_HZ || vco > APLL_VCO_MAX_HZ) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ana: APLL multiplier output %" PRIu64 " Hz is "
                      "outside its %llu-%llu Hz range\n", vco,
                      APLL_VCO_MIN_HZ, APLL_VCO_MAX_HZ);
    }
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * A write to one of a PLL's registers. The OR_ registers are the PLL's
 * outputs and ignore writes. Starting a calibration with IR_CAL_START,
 * IR_CAL_RSTB set, completes it at once (OR_CAL_END); clearing
 * IR_CAL_RSTB resets it.
 */
static void bbpll_write(Esp32AnaState *s, unsigned reg, uint8_t v)
{
    uint8_t old = s->bbpll[reg];

    switch (reg) {
    case BBPLL_OR_LOCK:
    case BBPLL_OR_CAL:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ana: write to the BBPLL's read-only register "
                      "%u\n", reg);
        return;
    case BBPLL_IR_CAL:
        s->bbpll[reg] = v;
        if (!(v & BBPLL_IR_CAL_RSTB)) {
            s->bbpll[BBPLL_OR_CAL] &= ~BBPLL_OR_CAL_END;
        } else if ((v & ~old) & BBPLL_IR_CAL_START) {
            s->bbpll[BBPLL_OR_CAL] |= BBPLL_OR_CAL_END;
        }
        return;
    default:
        s->bbpll[reg] = v;
        break;
    }
    if (reg == BBPLL_OC_LREF || reg == BBPLL_OC_DIV_7_0 ||
        reg == BBPLL_ENDIV5) {
        qemu_irq_pulse(s->clk_update);
    }
}

static void apll_write(Esp32AnaState *s, unsigned reg, uint8_t v)
{
    uint8_t old = s->apll[reg];

    switch (reg) {
    case APLL_OR_CAL:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ana: write to the APLL's read-only register "
                      "%u\n", reg);
        return;
    case APLL_IR_CAL:
        s->apll[reg] = v;
        if (!(v & APLL_IR_CAL_RSTB)) {
            s->apll[APLL_OR_CAL] &= ~APLL_OR_CAL_END;
        } else if ((v & ~old) & APLL_IR_CAL_START) {
            apll_check_range(s);
            s->apll[APLL_OR_CAL] |= APLL_OR_CAL_END;
        }
        return;
    default:
        s->apll[reg] = v;
        break;
    }
    qemu_irq_pulse(s->clk_update);
}

static uint8_t bbpll_read(Esp32AnaState *s, unsigned reg)
{
    if (reg == BBPLL_OR_LOCK) {
        return bbpll_powered(s) ? BBPLL_OR_LOCK_BOTH : 0;
    }
    return s->bbpll[reg];
}

/*
 * [spec:nuos:req:emu.esp32.analog]
 * One transaction of the I2C master, as the host register value v asks.
 * Returns the register's value after it.
 */
static uint32_t esp32_ana_transfer(Esp32AnaState *s, unsigned host,
                                   uint32_t v)
{
    unsigned slave = FIELD_EX32(v, ANA_I2C_HOST0, SLAVE);
    unsigned reg = FIELD_EX32(v, ANA_I2C_HOST0, ADDR);
    uint8_t data = FIELD_EX32(v, ANA_I2C_HOST0, DATA);
    bool wr = FIELD_EX32(v, ANA_I2C_HOST0, WR);
    uint32_t opt = rtc_reg(s, A_RTC_CNTL_OPTIONS0);
    bool acked;
    unsigned count;

    v &= ~R_ANA_I2C_HOST0_BUSY_MASK;
    switch (slave) {
    case ESP32_ANA_SLAVE_BBPLL:
        acked = !FIELD_EX32(opt, RTC_CNTL_OPTIONS0, BBPLL_I2C_FORCE_PD) &&
                !FIELD_EX32(s->ana_config, ANA_CONFIG, I2C_BBPLL_PD);
        count = BBPLL_REG_COUNT;
        break;
    case ESP32_ANA_SLAVE_APLL:
        acked = !FIELD_EX32(s->ana_config, ANA_CONFIG, I2C_APLL_PD);
        count = APLL_REG_COUNT;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "esp32_ana: host %u: analog I2C slave 0x%02x is not "
                      "modelled\n", host, slave);
        acked = false;
        count = 0;
        break;
    }
    if (FIELD_EX32(opt, RTC_CNTL_OPTIONS0, BIAS_I2C_FORCE_PD)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ana: host %u: the analog I2C bus is powered down "
                      "(RTC_CNTL_BIAS_I2C_FORCE_PD)\n", host);
        acked = false;
    } else if (count && !acked) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ana: host %u: analog I2C slave 0x%02x is "
                      "powered down or disconnected\n", host, slave);
    }
    if (acked && reg >= count) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ana: host %u: slave 0x%02x has no register "
                      "%u\n", host, slave, reg);
        acked = false;
    }

    if (wr) {
        if (acked && slave == ESP32_ANA_SLAVE_BBPLL) {
            bbpll_write(s, reg, data);
        } else if (acked) {
            apll_write(s, reg, data);
        }
        return v;
    }
    if (!acked) {
        data = 0;
    } else if (slave == ESP32_ANA_SLAVE_BBPLL) {
        data = bbpll_read(s, reg);
    } else {
        data = s->apll[reg];
    }
    return FIELD_DP32(v, ANA_I2C_HOST0, DATA, data);
}

static uint64_t esp32_ana_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32AnaState *s = ESP32_ANA(opaque);

    if (addr < ESP32_ANA_HOST_COUNT * 4) {
        return s->host[addr / 4];
    }
    if (addr == A_ANA_CONFIG) {
        return s->ana_config;
    }
    qemu_log_mask(LOG_UNIMP,
                  "esp32_ana: read of unmodelled analog register 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void esp32_ana_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Esp32AnaState *s = ESP32_ANA(opaque);
    uint32_t old;

    if (addr < ESP32_ANA_HOST_COUNT * 4) {
        s->host[addr / 4] = esp32_ana_transfer(s, addr / 4,
                                               value & 0x01ffffff);
        return;
    }
    if (addr == A_ANA_CONFIG) {
        old = s->ana_config;
        s->ana_config = value;
        if ((old ^ value) & (R_ANA_CONFIG_I2C_APLL_PD_MASK |
                             R_ANA_CONFIG_I2C_BBPLL_PD_MASK)) {
            qemu_irq_pulse(s->clk_update);
        }
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "esp32_ana: write of unmodelled analog register 0x%"
                  HWADDR_PRIx "\n", addr);
}

static const MemoryRegionOps esp32_ana_ops = {
    .read = esp32_ana_read,
    .write = esp32_ana_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_ana_reset_hold(Object *obj, ResetType type)
{
    Esp32AnaState *s = ESP32_ANA(obj);

    memset(s->host, 0, sizeof(s->host));
    s->ana_config = 0;
    memcpy(s->bbpll, bbpll_reset, sizeof(s->bbpll));
    memset(s->apll, 0, sizeof(s->apll));
    s->logged_bbpll_cfg = 0;
}

static void esp32_ana_reset_exit(Object *obj, ResetType type)
{
    qemu_irq_pulse(ESP32_ANA(obj)->clk_update);
}

static void esp32_ana_realize(DeviceState *dev, Error **errp)
{
    Esp32AnaState *s = ESP32_ANA(dev);

    if (!s->rtc_cntl) {
        error_setg(errp, "esp32_ana: rtc-cntl link not set");
    }
}

static void esp32_ana_init(Object *obj)
{
    Esp32AnaState *s = ESP32_ANA(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_ana_ops, s, TYPE_ESP32_ANA,
                          ESP32_ANA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->clk_update,
                             ESP32_ANA_CLK_UPDATE_GPIO, 1);
}

static const Property esp32_ana_properties[] = {
    DEFINE_PROP_LINK("rtc-cntl", Esp32AnaState, rtc_cntl,
                     TYPE_ESP32_RTC_CNTL, struct Esp32RtcCntlState *),
};

static const VMStateDescription vmstate_esp32_ana = {
    .name = TYPE_ESP32_ANA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(host, Esp32AnaState, ESP32_ANA_HOST_COUNT),
        VMSTATE_UINT32(ana_config, Esp32AnaState),
        VMSTATE_UINT8_ARRAY(bbpll, Esp32AnaState, ESP32_ANA_BANK_SIZE),
        VMSTATE_UINT8_ARRAY(apll, Esp32AnaState, ESP32_ANA_BANK_SIZE),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32_ana_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_ana_reset_hold;
    rc->phases.exit = esp32_ana_reset_exit;
    dc->realize = esp32_ana_realize;
    dc->vmsd = &vmstate_esp32_ana;
    device_class_set_props(dc, esp32_ana_properties);
}

/* [spec:nuos:req:emu.esp32.analog] */
static const TypeInfo esp32_ana_info = {
    .name = TYPE_ESP32_ANA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32AnaState),
    .instance_init = esp32_ana_init,
    .class_init = esp32_ana_class_init,
};

static void esp32_ana_register_types(void)
{
    type_register_static(&esp32_ana_info);
}

type_init(esp32_ana_register_types)
