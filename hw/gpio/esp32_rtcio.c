/*
 * ESP32 RTCIO: the RTC IO MUX, which controls the 18 RTC-capable pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "RTC IO MUX for Low Power and Analog I/O" and the
 * RTCIO registers of "IO_MUX and GPIO Matrix"; ESP-IDF's soc/rtc_io_reg.h
 * and soc/rtc_io_periph.c.
 *
 * Each RTC pad has a MUX_SEL bit. Clear, the pad belongs to the IO_MUX and
 * the GPIO matrix; set, the RTC IO MUX controls its input enable, output
 * and pulls (TRM register 6.53, MUX_SEL). In RTC function 0 (FUN_SEL 0)
 * the pad is RTC GPIO n: RTC_GPIO_OUT and RTC_GPIO_ENABLE drive it, with
 * PAD_DRIVER making it open drain, and RTC_GPIO_IN reads it. The pads are
 * in the RTC domain: a deep sleep, which resets the digital domain, leaves
 * them as they are.
 *
 * In RTC function 1 (FUN_SEL 3) the pads TOUCH_PAD0 to TOUCH_PAD3 (RTC GPIO
 * 10 to 13) carry the RTC I2C controller's SCL and SDA, on the pads
 * RTCIO_SAR_I2C_SCL_SEL and RTCIO_SAR_I2C_SDA_SEL choose (TRM table 6.11-1).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_rtcio.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "migration/vmstate.h"

const uint8_t esp32_rtcio_gpio[ESP32_RTCIO_PAD_COUNT] = {
    36, 37, 38, 39, 34, 35, 25, 26, 33, 32, 4, 0, 2, 15, 13, 12, 14, 27,
};

#define NO_BIT 0xff

/*
 * Where each RTC GPIO's controls are: its pad register and the bits in it,
 * and its bit in RTC_CNTL_HOLD_FORCE (ESP-IDF's rtc_io_desc[]). The sensor
 * and ADC pads (RTC GPIO 0-5) are input only and have no pulls.
 */
typedef struct RtcPadDesc {
    uint8_t reg;
    uint8_t mux;
    uint8_t fun_sel;
    uint8_t fun_ie;
    uint8_t rue;
    uint8_t rde;
    uint8_t slp_sel;
    uint8_t hold;
    uint8_t hold_force;
} RtcPadDesc;

#define TOUCH_PAD(n) \
    { A_RTCIO_TOUCH_PAD0 + 4 * (n), 19, 17, 13, 27, 28, 16, 31, 8 + (n) }

static const RtcPadDesc pad_desc[ESP32_RTCIO_PAD_COUNT] = {
    { A_RTCIO_SENSOR_PADS, 27, 22, 19, NO_BIT, NO_BIT, 21, 31, 4 },
    { A_RTCIO_SENSOR_PADS, 26, 17, 14, NO_BIT, NO_BIT, 16, 30, 5 },
    { A_RTCIO_SENSOR_PADS, 25, 12, 9, NO_BIT, NO_BIT, 11, 29, 6 },
    { A_RTCIO_SENSOR_PADS, 24, 7, 4, NO_BIT, NO_BIT, 6, 28, 7 },
    { A_RTCIO_ADC_PAD, 29, 26, 23, NO_BIT, NO_BIT, 25, 31, 0 },
    { A_RTCIO_ADC_PAD, 28, 21, 18, NO_BIT, NO_BIT, 20, 30, 1 },
    { A_RTCIO_PAD_DAC1, 17, 15, 11, 27, 28, 14, 29, 2 },
    { A_RTCIO_PAD_DAC2, 17, 15, 11, 27, 28, 14, 29, 3 },
    { A_RTCIO_XTAL_32K_PAD, 18, 15, 11, 27, 28, 14, 29, 17 },
    { A_RTCIO_XTAL_32K_PAD, 17, 9, 5, 22, 23, 8, 24, 16 },
    TOUCH_PAD(0), TOUCH_PAD(1), TOUCH_PAD(2), TOUCH_PAD(3),
    TOUCH_PAD(4), TOUCH_PAD(5), TOUCH_PAD(6), TOUCH_PAD(7),
};

#define FUN_SEL_RTC_GPIO 0
/* RTC function 1: the RTC I2C controller's SCL or SDA, on RTC GPIO 10-13 */
#define FUN_SEL_RTC_FUNC1 3
/* SCL on RTC GPIO 10 + 2 * SCL_SEL, SDA on RTC GPIO 11 + 2 * SDA_SEL */
#define RTC_I2C_SCL_PAD0  10
#define RTC_I2C_SDA_PAD0  11

/* RTC_GPIO_PINn_INT_TYPE */
enum {
    INT_TYPE_DISABLED,
    INT_TYPE_RISING,
    INT_TYPE_FALLING,
    INT_TYPE_ANY_EDGE,
    INT_TYPE_LOW,
    INT_TYPE_HIGH,
};

#define RTC_GPIO_MASK (((1u << ESP32_RTCIO_PAD_COUNT) - 1) << \
                       ESP32_RTCIO_GPIO_SHIFT)
#define PIN_WRITABLE  (R_RTCIO_RTC_GPIO_PIN0_WAKEUP_ENABLE_MASK | \
                       R_RTCIO_RTC_GPIO_PIN0_INT_TYPE_MASK | \
                       R_RTCIO_RTC_GPIO_PIN0_PAD_DRIVER_MASK)

/* Reset values, and the bits software can write, of the plain registers */
typedef struct RegInfo {
    uint32_t reset;
    uint32_t writable;
} RegInfo;

static const RegInfo reg_info[ESP32_RTCIO_REG_COUNT] = {
    [A_RTCIO_DEBUG_SEL / 4] = { 0, 0x03ffffff },
    [A_RTCIO_DIG_PAD_HOLD / 4] = { 0, 0xffffffff },
    [A_RTCIO_HALL_SENS / 4] = { 0, 0xc0000000 },
    [A_RTCIO_SENSOR_PADS / 4] = { 0, 0xfffffff0 },
    [A_RTCIO_ADC_PAD / 4] = { 0, 0xfffc0000 },
    [A_RTCIO_PAD_DAC1 / 4] = { 0x80000000, 0xfffffc00 },
    [A_RTCIO_PAD_DAC2 / 4] = { 0x80000000, 0xfffffc00 },
    [A_RTCIO_XTAL_32K_PAD / 4] = { 0x84100010, 0xfffffffe },
    [A_RTCIO_TOUCH_CFG / 4] = { 0x66000000, 0xff800000 },
    /* DRV 2, DAC 4, and the pulls the pads' IO_MUX reset state has */
    [A_RTCIO_TOUCH_PAD0 / 4 + 0] = { 0x52000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 1] = { 0x4a000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 2] = { 0x52000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 3] = { 0x4a000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 4] = { 0x52000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 5] = { 0x52000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 6] = { 0x4a000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD0 / 4 + 7] = { 0x42000000, 0xfbfff000 },
    [A_RTCIO_TOUCH_PAD8 / 4] = { 0x02000000, 0x03f80000 },
    [A_RTCIO_TOUCH_PAD9 / 4] = { 0x02000000, 0x03f80000 },
    [A_RTCIO_EXT_WAKEUP0 / 4] = { 0, 0xf8000000 },
    [A_RTCIO_XTL_EXT_CTR / 4] = { 0, 0xf8000000 },
    [A_RTCIO_SAR_I2C_IO / 4] = { 0, 0xff800000 },
    [A_RTCIO_DATE / 4] = { 0x01603160, 0x0fffffff },
};

static inline bool bit32(uint32_t v, unsigned n)
{
    return (v >> n) & 1;
}

static uint32_t pad_reg(Esp32RtcIoState *s, unsigned n)
{
    return s->regs[pad_desc[n].reg / 4];
}

static uint32_t rtc_gpio_bits(Esp32RtcIoState *s, hwaddr reg)
{
    return s->regs[reg / 4] >> ESP32_RTCIO_GPIO_SHIFT;
}

static bool pad_held(Esp32RtcIoState *s, unsigned n)
{
    const RtcPadDesc *d = &pad_desc[n];

    return bit32(pad_reg(s, n), d->hold) ||
           bit32(esp32_rtc_cntl_hold_force(s->rtc_cntl), d->hold_force) ||
           esp32_rtc_cntl_dg_pad_force_hold(s->rtc_cntl);
}

/* The RTC GPIOs carrying the RTC I2C controller's SCL and SDA, if any */
static unsigned i2c_scl_pad(Esp32RtcIoState *s)
{
    return RTC_I2C_SCL_PAD0 +
           2 * (FIELD_EX32(s->regs[A_RTCIO_SAR_I2C_IO / 4], RTCIO_SAR_I2C_IO,
                           SCL_SEL) & 1);
}

static unsigned i2c_sda_pad(Esp32RtcIoState *s)
{
    return RTC_I2C_SDA_PAD0 +
           2 * (FIELD_EX32(s->regs[A_RTCIO_SAR_I2C_IO / 4], RTCIO_SAR_I2C_IO,
                           SDA_SEL) & 1);
}

static bool pad_in_func1(Esp32RtcIoState *s, unsigned n)
{
    uint32_t reg = pad_reg(s, n);

    return bit32(reg, pad_desc[n].mux) &&
           ((reg >> pad_desc[n].fun_sel) & 3) == FUN_SEL_RTC_FUNC1;
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * Give the RTC I2C controller its lines as the selected pads read them;
 * a line no pad carries reads high.
 */
static void esp32_rtcio_update_i2c_in(Esp32RtcIoState *s, bool resync)
{
    unsigned scl = i2c_scl_pad(s), sda = i2c_sda_pad(s);
    bool scl_level = !pad_in_func1(s, scl) || bit32(s->pad_in, scl);
    bool sda_level = !pad_in_func1(s, sda) || bit32(s->pad_in, sda);

    if (resync || scl_level != s->i2c_scl_level) {
        s->i2c_scl_level = scl_level;
        qemu_set_irq(s->i2c_scl_in, scl_level);
    }
    if (resync || sda_level != s->i2c_sda_level) {
        s->i2c_sda_level = sda_level;
        qemu_set_irq(s->i2c_sda_in, sda_level);
    }
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * Work out each pad's control from the registers. A held pad keeps the
 * control it had when the hold took effect (TRM 6.7): whatever the
 * registers say afterwards, until the hold is released.
 */
static void esp32_rtcio_update(Esp32RtcIoState *s)
{
    uint32_t out = rtc_gpio_bits(s, A_RTCIO_RTC_GPIO_OUT);
    uint32_t ena = rtc_gpio_bits(s, A_RTCIO_RTC_GPIO_ENABLE);
    uint32_t old_mux = s->ctl_mux, old_out = s->ctl_out;
    uint32_t old_oe = s->ctl_oe, old_pu = s->ctl_pu;
    uint32_t old_pd = s->ctl_pd, old_ie = s->ctl_ie;
    bool resync = s->resync;

    s->resync = false;
    for (unsigned n = 0; n < ESP32_RTCIO_PAD_COUNT; n++) {
        const RtcPadDesc *d = &pad_desc[n];
        uint32_t reg = pad_reg(s, n);
        uint32_t m = 1u << n;
        bool mux = bit32(reg, d->mux);
        unsigned fun = (reg >> d->fun_sel) & 3;
        uint32_t pin = s->regs[A_RTCIO_RTC_GPIO_PIN0 / 4 + n];
        bool val = bit32(out, n), oe = false;

        if (pad_held(s, n)) {
            if (!(s->held & m) && !mux) {
                qemu_log_mask(LOG_UNIMP,
                              "esp32_rtcio: hold of RTC GPIO %u while the "
                              "IO_MUX controls it is not modelled\n", n);
            }
            s->held |= m;
            continue;
        }
        s->held &= ~m;

        if (fun == FUN_SEL_RTC_GPIO) {
            oe = bit32(ena, n);
            if (FIELD_EX32(pin, RTCIO_RTC_GPIO_PIN0, PAD_DRIVER) && val) {
                /* Open drain: a high output releases the pad */
                oe = false;
            }
        } else if (fun == FUN_SEL_RTC_FUNC1 && n == i2c_scl_pad(s)) {
            /* [spec:nuos:req:emu.esp32.ulp] */
            val = s->i2c_scl_out;
            oe = s->i2c_scl_oe;
        } else if (fun == FUN_SEL_RTC_FUNC1 && n == i2c_sda_pad(s)) {
            val = s->i2c_sda_out;
            oe = s->i2c_sda_oe;
        }
        s->ctl_mux = (s->ctl_mux & ~m) | (mux ? m : 0);
        s->ctl_out = (s->ctl_out & ~m) | (val ? m : 0);
        s->ctl_oe = (s->ctl_oe & ~m) | (oe ? m : 0);
        s->ctl_ie = (s->ctl_ie & ~m) | (bit32(reg, d->fun_ie) ? m : 0);
        s->ctl_pu &= ~m;
        s->ctl_pd &= ~m;
        if (d->rue != NO_BIT) {
            s->ctl_pu |= bit32(reg, d->rue) ? m : 0;
            s->ctl_pd |= bit32(reg, d->rde) ? m : 0;
        }
    }

    for (unsigned n = 0; n < ESP32_RTCIO_PAD_COUNT; n++) {
        uint32_t m = 1u << n;

        if (resync || ((old_out ^ s->ctl_out) & m)) {
            qemu_set_irq(s->pad_out[n], bit32(s->ctl_out, n));
        }
        if (resync || ((old_oe ^ s->ctl_oe) & m)) {
            qemu_set_irq(s->pad_oe[n], bit32(s->ctl_oe, n));
        }
        if (resync || ((old_pu ^ s->ctl_pu) & m)) {
            qemu_set_irq(s->pad_pu[n], bit32(s->ctl_pu, n));
        }
        if (resync || ((old_pd ^ s->ctl_pd) & m)) {
            qemu_set_irq(s->pad_pd[n], bit32(s->ctl_pd, n));
        }
        if (resync || ((old_ie ^ s->ctl_ie) & m)) {
            qemu_set_irq(s->pad_ie[n], bit32(s->ctl_ie, n));
        }
        if (resync || ((old_mux ^ s->ctl_mux) & m)) {
            qemu_set_irq(s->pad_mux[n], bit32(s->ctl_mux, n));
        }
    }
    esp32_rtcio_update_i2c_in(s, resync);
    esp32_rtc_cntl_rtcio_changed(s->rtc_cntl);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * Latch RTC GPIO interrupt status from an input change, by each pad's
 * RTC_GPIO_PINn_INT_TYPE. ESP32 routes no CPU interrupt from it; the status
 * is for software and the ULP to read.
 */
static void esp32_rtcio_latch_status(Esp32RtcIoState *s, uint32_t old_in)
{
    uint32_t in = s->pad_in;
    uint32_t edges = in ^ old_in;
    uint32_t set = 0;

    for (unsigned n = 0; n < ESP32_RTCIO_PAD_COUNT; n++) {
        uint32_t pin = s->regs[A_RTCIO_RTC_GPIO_PIN0 / 4 + n];
        bool hit;

        switch (FIELD_EX32(pin, RTCIO_RTC_GPIO_PIN0, INT_TYPE)) {
        case INT_TYPE_RISING:
            hit = bit32(edges & in, n);
            break;
        case INT_TYPE_FALLING:
            hit = bit32(edges & ~in, n);
            break;
        case INT_TYPE_ANY_EDGE:
            hit = bit32(edges, n);
            break;
        case INT_TYPE_LOW:
            hit = !bit32(in, n);
            break;
        case INT_TYPE_HIGH:
            hit = bit32(in, n);
            break;
        default:
            hit = false;
            break;
        }
        set |= hit ? 1u << n : 0;
    }
    s->regs[A_RTCIO_RTC_GPIO_STATUS / 4] |= set << ESP32_RTCIO_GPIO_SHIFT;
}

uint32_t esp32_rtcio_inputs(Esp32RtcIoState *s)
{
    return s->pad_in;
}

unsigned esp32_rtcio_ext0_sel(Esp32RtcIoState *s)
{
    return FIELD_EX32(s->regs[A_RTCIO_EXT_WAKEUP0 / 4], RTCIO_EXT_WAKEUP0,
                      SEL);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * The GPIO wakeup source's RTC half: an RTC GPIO with WAKEUP_ENABLE set
 * whose input is at the level its INT_TYPE names. Only the level types
 * wake the chip.
 */
bool esp32_rtcio_gpio_wakeup(Esp32RtcIoState *s)
{
    for (unsigned n = 0; n < ESP32_RTCIO_PAD_COUNT; n++) {
        uint32_t pin = s->regs[A_RTCIO_RTC_GPIO_PIN0 / 4 + n];

        if (!FIELD_EX32(pin, RTCIO_RTC_GPIO_PIN0, WAKEUP_ENABLE)) {
            continue;
        }
        switch (FIELD_EX32(pin, RTCIO_RTC_GPIO_PIN0, INT_TYPE)) {
        case INT_TYPE_LOW:
            if (!bit32(s->pad_in, n)) {
                return true;
            }
            break;
        case INT_TYPE_HIGH:
            if (bit32(s->pad_in, n)) {
                return true;
            }
            break;
        default:
            break;
        }
    }
    return false;
}

void esp32_rtcio_hold_changed(Esp32RtcIoState *s)
{
    esp32_rtcio_update(s);
}

static uint64_t esp32_rtcio_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);

    switch (addr) {
    case A_RTCIO_RTC_GPIO_OUT_W1TS:
    case A_RTCIO_RTC_GPIO_OUT_W1TC:
    case A_RTCIO_RTC_GPIO_ENABLE_W1TS:
    case A_RTCIO_RTC_GPIO_ENABLE_W1TC:
    case A_RTCIO_RTC_GPIO_STATUS_W1TS:
    case A_RTCIO_RTC_GPIO_STATUS_W1TC:
        return 0;
    case A_RTCIO_RTC_GPIO_IN:
        return s->pad_in << ESP32_RTCIO_GPIO_SHIFT;
    default:
        if (addr > A_RTCIO_DATE) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_rtcio: read of reserved offset 0x%"
                          HWADDR_PRIx "\n", addr);
            return 0;
        }
        return s->regs[addr / 4];
    }
}

/* Report RTC functions a pad has newly been given that are not modelled. */
static void esp32_rtcio_check_fun_sel(Esp32RtcIoState *s, uint32_t old,
                                      hwaddr addr)
{
    uint32_t reg = s->regs[addr / 4];

    for (unsigned n = 0; n < ESP32_RTCIO_PAD_COUNT; n++) {
        const RtcPadDesc *d = &pad_desc[n];
        unsigned fun = (reg >> d->fun_sel) & 3;
        uint32_t sel_mask = (3u << d->fun_sel) | (1u << d->mux) |
                            (1u << d->slp_sel);

        if (d->reg != addr || !bit32(reg, d->mux) ||
            !((old ^ reg) & sel_mask)) {
            continue;
        }
        if (fun == FUN_SEL_RTC_FUNC1 &&
            (n < RTC_I2C_SCL_PAD0 || n > RTC_I2C_SDA_PAD0 + 2)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_rtcio: RTC GPIO %u has no RTC function 1\n",
                          n);
        } else if (fun != FUN_SEL_RTC_GPIO && fun != FUN_SEL_RTC_FUNC1) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_rtcio: RTC GPIO %u: reserved FUN_SEL %u\n",
                          n, fun);
        }
        if (bit32(reg, d->slp_sel)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_rtcio: RTC GPIO %u: sleep-mode pad control "
                          "(SLP_SEL) is not modelled\n", n);
        }
    }
}

static void esp32_rtcio_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);
    uint32_t *r = &s->regs[addr / 4];
    uint32_t v = value & RTC_GPIO_MASK;
    uint32_t old;

    switch (addr) {
    case A_RTCIO_RTC_GPIO_OUT:
    case A_RTCIO_RTC_GPIO_ENABLE:
    case A_RTCIO_RTC_GPIO_STATUS:
        *r = v;
        break;
    case A_RTCIO_RTC_GPIO_OUT_W1TS:
    case A_RTCIO_RTC_GPIO_ENABLE_W1TS:
    case A_RTCIO_RTC_GPIO_STATUS_W1TS:
        s->regs[(addr - 4) / 4] |= v;
        break;
    case A_RTCIO_RTC_GPIO_OUT_W1TC:
    case A_RTCIO_RTC_GPIO_ENABLE_W1TC:
    case A_RTCIO_RTC_GPIO_STATUS_W1TC:
        s->regs[(addr - 8) / 4] &= ~v;
        break;
    case A_RTCIO_RTC_GPIO_IN:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_rtcio: write to read-only RTC_GPIO_IN\n");
        return;
    case A_RTCIO_RTC_GPIO_PIN0 ... A_RTCIO_RTC_GPIO_PIN17:
        *r = value & PIN_WRITABLE;
        break;
    case A_RTCIO_DIG_PAD_HOLD:
        if (value) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_rtcio: holding digital pads "
                          "(RTCIO_DIG_PAD_HOLD 0x%08" PRIx64 ") is not "
                          "modelled\n", value);
        }
        *r = value;
        break;
    case A_RTCIO_HALL_SENS:
    case A_RTCIO_TOUCH_CFG:
    case A_RTCIO_TOUCH_PAD8:
    case A_RTCIO_TOUCH_PAD9:
    case A_RTCIO_XTL_EXT_CTR:
    case A_RTCIO_SAR_I2C_IO:
        /*
         * The Hall sensor, touch and the RTC I2C pads are the analog and
         * ULP blocks' business; their fields are stored for them.
         */
        *r = value & reg_info[addr / 4].writable;
        break;
    default:
        if (addr > A_RTCIO_DATE) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_rtcio: write to reserved offset 0x%"
                          HWADDR_PRIx "\n", addr);
            return;
        }
        old = *r;
        *r = value & reg_info[addr / 4].writable;
        esp32_rtcio_check_fun_sel(s, old, addr);
        break;
    }
    esp32_rtcio_update(s);
}

static const MemoryRegionOps esp32_rtcio_ops = {
    .read = esp32_rtcio_read,
    .write = esp32_rtcio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_rtcio_pad_in(void *opaque, int n, int level)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);
    uint32_t old = s->pad_in;

    s->pad_in = (s->pad_in & ~(1u << n)) | ((level ? 1u : 0) << n);
    if (s->pad_in != old) {
        esp32_rtcio_latch_status(s, old);
        esp32_rtcio_update_i2c_in(s, false);
        esp32_rtc_cntl_rtcio_changed(s->rtc_cntl);
    }
}

static void esp32_rtcio_i2c_line(Esp32RtcIoState *s, bool *line, int level)
{
    if (*line != (level != 0)) {
        *line = level != 0;
        esp32_rtcio_update(s);
    }
}

static void esp32_rtcio_i2c_scl_out(void *opaque, int n, int level)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);

    esp32_rtcio_i2c_line(s, &s->i2c_scl_out, level);
}

static void esp32_rtcio_i2c_scl_oe(void *opaque, int n, int level)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);

    esp32_rtcio_i2c_line(s, &s->i2c_scl_oe, level);
}

static void esp32_rtcio_i2c_sda_out(void *opaque, int n, int level)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);

    esp32_rtcio_i2c_line(s, &s->i2c_sda_out, level);
}

static void esp32_rtcio_i2c_sda_oe(void *opaque, int n, int level)
{
    Esp32RtcIoState *s = ESP32_RTCIO(opaque);

    esp32_rtcio_i2c_line(s, &s->i2c_sda_oe, level);
}

static void esp32_rtcio_reset_hold(Object *obj, ResetType type)
{
    Esp32RtcIoState *s = ESP32_RTCIO(obj);

    for (unsigned i = 0; i < ESP32_RTCIO_REG_COUNT; i++) {
        s->regs[i] = reg_info[i].reset;
    }
    s->held = 0;
    s->resync = true;
}

static void esp32_rtcio_reset_exit(Object *obj, ResetType type)
{
    esp32_rtcio_update(ESP32_RTCIO(obj));
}

static void esp32_rtcio_realize(DeviceState *dev, Error **errp)
{
    Esp32RtcIoState *s = ESP32_RTCIO(dev);

    if (!s->rtc_cntl) {
        error_setg(errp, "esp32_rtcio: rtc-cntl link not set");
    }
}

static void esp32_rtcio_init(Object *obj)
{
    Esp32RtcIoState *s = ESP32_RTCIO(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rtcio_ops, s,
                          TYPE_ESP32_RTCIO, ESP32_RTCIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(dev, s->pad_mux, ESP32_RTCIO_PAD_MUX,
                             ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_out_named(dev, s->pad_out, ESP32_RTCIO_PAD_OUT,
                             ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_out_named(dev, s->pad_oe, ESP32_RTCIO_PAD_OE,
                             ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_out_named(dev, s->pad_pu, ESP32_RTCIO_PAD_PU,
                             ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_out_named(dev, s->pad_pd, ESP32_RTCIO_PAD_PD,
                             ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_out_named(dev, s->pad_ie, ESP32_RTCIO_PAD_IE,
                             ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_in_named(dev, esp32_rtcio_pad_in, ESP32_RTCIO_PAD_IN,
                            ESP32_RTCIO_PAD_COUNT);
    qdev_init_gpio_in_named(dev, esp32_rtcio_i2c_scl_out,
                            ESP32_RTCIO_I2C_SCL_OUT, 1);
    qdev_init_gpio_in_named(dev, esp32_rtcio_i2c_scl_oe,
                            ESP32_RTCIO_I2C_SCL_OE, 1);
    qdev_init_gpio_in_named(dev, esp32_rtcio_i2c_sda_out,
                            ESP32_RTCIO_I2C_SDA_OUT, 1);
    qdev_init_gpio_in_named(dev, esp32_rtcio_i2c_sda_oe,
                            ESP32_RTCIO_I2C_SDA_OE, 1);
    qdev_init_gpio_out_named(dev, &s->i2c_scl_in, ESP32_RTCIO_I2C_SCL_IN, 1);
    qdev_init_gpio_out_named(dev, &s->i2c_sda_in, ESP32_RTCIO_I2C_SDA_IN, 1);
}

static const Property esp32_rtcio_properties[] = {
    DEFINE_PROP_LINK("rtc-cntl", Esp32RtcIoState, rtc_cntl,
                     TYPE_ESP32_RTC_CNTL, Esp32RtcCntlState *),
};

static const VMStateDescription vmstate_esp32_rtcio = {
    .name = TYPE_ESP32_RTCIO,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Esp32RtcIoState, ESP32_RTCIO_REG_COUNT),
        VMSTATE_BOOL(i2c_scl_out, Esp32RtcIoState),
        VMSTATE_BOOL(i2c_scl_oe, Esp32RtcIoState),
        VMSTATE_BOOL(i2c_sda_out, Esp32RtcIoState),
        VMSTATE_BOOL(i2c_sda_oe, Esp32RtcIoState),
        VMSTATE_BOOL(i2c_scl_level, Esp32RtcIoState),
        VMSTATE_BOOL(i2c_sda_level, Esp32RtcIoState),
        VMSTATE_UINT32(pad_in, Esp32RtcIoState),
        VMSTATE_UINT32(ctl_mux, Esp32RtcIoState),
        VMSTATE_UINT32(ctl_out, Esp32RtcIoState),
        VMSTATE_UINT32(ctl_oe, Esp32RtcIoState),
        VMSTATE_UINT32(ctl_pu, Esp32RtcIoState),
        VMSTATE_UINT32(ctl_pd, Esp32RtcIoState),
        VMSTATE_UINT32(ctl_ie, Esp32RtcIoState),
        VMSTATE_UINT32(held, Esp32RtcIoState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_rtcio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_rtcio_reset_hold;
    rc->phases.exit = esp32_rtcio_reset_exit;
    dc->realize = esp32_rtcio_realize;
    dc->vmsd = &vmstate_esp32_rtcio;
    device_class_set_props(dc, esp32_rtcio_properties);
}

/* [spec:nuos:req:emu.esp32.rtc] */
static const TypeInfo esp32_rtcio_info = {
    .name = TYPE_ESP32_RTCIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RtcIoState),
    .instance_init = esp32_rtcio_init,
    .class_init = esp32_rtcio_class_init,
};

static void esp32_rtcio_register_types(void)
{
    type_register_static(&esp32_rtcio_info);
}

type_init(esp32_rtcio_register_types)
