/*
 * ESP32 GPIO matrix, IO_MUX and sigma-delta modulators
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * Reference: ESP32 TRM, "IO_MUX and GPIO Matrix", and ESP-IDF's
 * soc/gpio_reg.h, soc/gpio_sd_reg.h, soc/io_mux_reg.h and soc/gpio_sig_map.h.
 *
 * Each pad's level comes from, in order: a driver outside the chip; the
 * chip's own output driver, when its output enable is set (an open-drain pad
 * drives only low); its pull-up; its pull-down; otherwise it floats, which
 * reads low. With its input enabled (FUN_IE) the pad's level is its input,
 * read in GPIO_IN/GPIO_IN1, watched by its interrupt, and routed to
 * peripheral input signals. Everything is recomputed whenever a register, a
 * peripheral signal or an outside driver changes, and whenever software
 * samples the pads, so that the sigma-delta modulators' bitstreams are read
 * at the virtual time of the access.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"

REG32(GPIO_BT_SELECT, 0x0000)
REG32(GPIO_OUT, 0x0004)
REG32(GPIO_OUT_W1TS, 0x0008)
REG32(GPIO_OUT_W1TC, 0x000c)
REG32(GPIO_OUT1, 0x0010)
REG32(GPIO_OUT1_W1TS, 0x0014)
REG32(GPIO_OUT1_W1TC, 0x0018)
REG32(GPIO_SDIO_SELECT, 0x001c)
REG32(GPIO_ENABLE, 0x0020)
REG32(GPIO_ENABLE_W1TS, 0x0024)
REG32(GPIO_ENABLE_W1TC, 0x0028)
REG32(GPIO_ENABLE1, 0x002c)
REG32(GPIO_ENABLE1_W1TS, 0x0030)
REG32(GPIO_ENABLE1_W1TC, 0x0034)
REG32(GPIO_IN, 0x003c)
REG32(GPIO_IN1, 0x0040)
REG32(GPIO_STATUS, 0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004c)
REG32(GPIO_STATUS1, 0x0050)
REG32(GPIO_STATUS1_W1TS, 0x0054)
REG32(GPIO_STATUS1_W1TC, 0x0058)
REG32(GPIO_ACPU_INT, 0x0060)
REG32(GPIO_ACPU_NMI_INT, 0x0064)
REG32(GPIO_PCPU_INT, 0x0068)
REG32(GPIO_PCPU_NMI_INT, 0x006c)
REG32(GPIO_CPUSDIO_INT, 0x0070)
REG32(GPIO_ACPU_INT1, 0x0074)
REG32(GPIO_ACPU_NMI_INT1, 0x0078)
REG32(GPIO_PCPU_INT1, 0x007c)
REG32(GPIO_PCPU_NMI_INT1, 0x0080)
REG32(GPIO_CPUSDIO_INT1, 0x0084)
REG32(GPIO_PIN0, 0x0088)
    FIELD(GPIO_PIN0, PAD_DRIVER, 2, 1)
    FIELD(GPIO_PIN0, INT_TYPE, 7, 3)
    FIELD(GPIO_PIN0, WAKEUP_ENABLE, 10, 1)
    FIELD(GPIO_PIN0, CONFIG, 11, 2)
    FIELD(GPIO_PIN0, INT_ENA, 13, 5)
REG32(GPIO_CALI_CONF, 0x0128)
REG32(GPIO_CALI_DATA, 0x012c)
REG32(GPIO_FUNC0_IN_SEL_CFG, 0x0130)
    FIELD(GPIO_FUNC0_IN_SEL_CFG, IN_SEL, 0, 6)
    FIELD(GPIO_FUNC0_IN_SEL_CFG, IN_INV_SEL, 6, 1)
    FIELD(GPIO_FUNC0_IN_SEL_CFG, SIG_IN_SEL, 7, 1)
REG32(GPIO_FUNC0_OUT_SEL_CFG, 0x0530)
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, OUT_SEL, 0, 9)
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, OUT_INV_SEL, 9, 1)
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, OEN_SEL, 10, 1)
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, OEN_INV_SEL, 11, 1)
REG32(GPIO_SIGMADELTA0, 0x0f00)
    FIELD(GPIO_SIGMADELTA0, IN, 0, 8)
    FIELD(GPIO_SIGMADELTA0, PRESCALE, 8, 8)
REG32(GPIO_SIGMADELTA_CG, 0x0f20)
REG32(GPIO_SIGMADELTA_MISC, 0x0f24)
REG32(GPIO_SIGMADELTA_VERSION, 0x0f28)

REG32(IO_MUX_PIN_CTRL, 0x0000)
REG32(IO_MUX_GPIO36, 0x0004)
    FIELD(IO_MUX_GPIO36, MCU_OE, 0, 1)
    FIELD(IO_MUX_GPIO36, SLP_SEL, 1, 1)
    FIELD(IO_MUX_GPIO36, MCU_WPD, 2, 1)
    FIELD(IO_MUX_GPIO36, MCU_WPU, 3, 1)
    FIELD(IO_MUX_GPIO36, MCU_IE, 4, 1)
    FIELD(IO_MUX_GPIO36, MCU_DRV, 5, 2)
    FIELD(IO_MUX_GPIO36, FUN_WPD, 7, 1)
    FIELD(IO_MUX_GPIO36, FUN_WPU, 8, 1)
    FIELD(IO_MUX_GPIO36, FUN_IE, 9, 1)
    FIELD(IO_MUX_GPIO36, FUN_DRV, 10, 2)
    FIELD(IO_MUX_GPIO36, MCU_SEL, 12, 3)

#define GPIO_PIN_STRIDE         4
#define GPIO_PIN_WRITABLE       0x3ff84
#define GPIO_FUNC_IN_WRITABLE   0xff
#define GPIO_FUNC_OUT_WRITABLE  0xfff
#define GPIO_SDM_WRITABLE       0xffff
#define GPIO_SD_CLK_EN          (1u << 31)
#define GPIO_SPI_SWAP           (1u << 31)
#define GPIO_SD_DATE_RESET      0x1506190
#define GPIO_SD_DATE_MASK       0x0fffffff
#define GPIO_PIN_CTRL_WRITABLE  0xfff
#define IO_MUX_PIN_WRITABLE     0x7fff
#define IO_MUX_NO_PULL_DRV      (R_IO_MUX_GPIO36_FUN_WPD_MASK | \
                                 R_IO_MUX_GPIO36_FUN_WPU_MASK | \
                                 R_IO_MUX_GPIO36_FUN_DRV_MASK)
#define IO_MUX_LAST_REG         0x8c

/* GPIO_FUNCn_IN_SEL values that are not pads */
#define GPIO_IN_SEL_CONST_LOW   0x30
#define GPIO_IN_SEL_CONST_HIGH  0x38

/*
 * Signals 224-228 (sig_in_func224-228) loop back inside the matrix: input
 * signal n is output signal n, so a pad can be routed to another pad.
 */
#define SIG_LOOPBACK_FIRST      224
#define SIG_LOOPBACK_LAST       228

/* GPIO_PINn_INT_TYPE */
enum {
    INT_TYPE_DISABLED,
    INT_TYPE_RISING,
    INT_TYPE_FALLING,
    INT_TYPE_ANY_EDGE,
    INT_TYPE_LOW,
    INT_TYPE_HIGH,
};

/* GPIO_PINn_INT_ENA bits, and the status register views they select */
enum {
    INT_ENA_APP,
    INT_ENA_APP_NMI,
    INT_ENA_PRO,
    INT_ENA_PRO_NMI,
    INT_ENA_SDIO,
    INT_ENA_COUNT,
};

/* Pads that exist: GPIO0-19, 21-23, 25-27 and 32-39 */
#define PAD_VALID       0x000000ff0eefffffULL
/* Pads with an output driver: all but the input-only GPIO34-39 */
#define PAD_OUTPUT      0x000000030eefffffULL
#define HIGH_PADS_MASK  0xff

/* The GPIO each IO_MUX_x_REG configures, from offset 0x04 on */
static const uint8_t iomux_reg_gpio[] = {
    36, 37, 38, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13, 15, 2, 0, 4,
    16, 17, 9, 10, 11, 6, 7, 8, 5, 18, 19, 20, 21, 22, 3, 1, 23,
};

/* The "Reset" column of TRM table 6.10-1 */
enum {
    PAD_RST_NONE,
    PAD_RST_IE,
    PAD_RST_IE_WPD,
    PAD_RST_IE_WPU,
};

static const uint8_t pad_reset[ESP32_GPIO_PIN_COUNT] = {
    [0] = PAD_RST_IE_WPU, [1] = PAD_RST_IE_WPU, [2] = PAD_RST_IE_WPD,
    [3] = PAD_RST_IE_WPU, [4] = PAD_RST_IE_WPD, [5] = PAD_RST_IE_WPU,
    [6] = PAD_RST_IE_WPU, [7] = PAD_RST_IE_WPU, [8] = PAD_RST_IE_WPU,
    [9] = PAD_RST_IE_WPU, [10] = PAD_RST_IE_WPU, [11] = PAD_RST_IE_WPU,
    [12] = PAD_RST_IE_WPD, [13] = PAD_RST_IE_WPD, [14] = PAD_RST_IE_WPU,
    [15] = PAD_RST_IE_WPU, [16] = PAD_RST_IE, [17] = PAD_RST_IE,
    [18] = PAD_RST_IE, [19] = PAD_RST_IE, [21] = PAD_RST_IE,
    [22] = PAD_RST_IE, [23] = PAD_RST_IE,
};

/*
 * IO_MUX functions (TRM table 6.10-1). Function 2, and function 0 where the
 * TRM names it GPIOn, is the GPIO matrix. A function that carries a GPIO
 * matrix signal connects the pad to that signal directly ("Same input
 * signal from IO_MUX core" in table 6.9-1); the other peripheral functions
 * have lines of their own.
 */
typedef enum IomuxKind {
    IOMUX_NONE,
    IOMUX_GPIO,
    IOMUX_SIG,
    IOMUX_DIRECT,
} IomuxKind;

typedef struct IomuxFunc {
    uint8_t kind;
    int16_t sig_in;
    int16_t sig_out;
} IomuxFunc;

#define F_NONE      { IOMUX_NONE, -1, -1 }
#define F_GPIO      { IOMUX_GPIO, -1, -1 }
#define F_DIRECT    { IOMUX_DIRECT, -1, -1 }
#define F_IO(s)     { IOMUX_SIG, (s), (s) }
#define F_IN(s)     { IOMUX_SIG, (s), -1 }
#define F_OUT(s)    { IOMUX_SIG, -1, (s) }

static const IomuxFunc iomux_funcs[ESP32_GPIO_PIN_COUNT]
                                  [ESP32_IOMUX_FUNC_COUNT] = {
    /* GPIO0: GPIO0, CLK_OUT1, GPIO0, -, -, EMAC_TX_CLK */
    [0] = { F_GPIO, F_DIRECT, F_GPIO, F_NONE, F_NONE, F_DIRECT },
    /* U0TXD: U0TXD, CLK_OUT3, GPIO1, -, -, EMAC_RXD2 */
    [1] = { F_OUT(14), F_DIRECT, F_GPIO, F_NONE, F_NONE, F_DIRECT },
    /* GPIO2: GPIO2, HSPIWP, GPIO2, HS2_DATA0, SD_DATA0, - */
    [2] = { F_GPIO, F_IO(13), F_GPIO, F_DIRECT, F_DIRECT, F_NONE },
    /* U0RXD: U0RXD, CLK_OUT2, GPIO3, -, -, - */
    [3] = { F_IN(14), F_DIRECT, F_GPIO, F_NONE, F_NONE, F_NONE },
    /* GPIO4: GPIO4, HSPIHD, GPIO4, HS2_DATA1, SD_DATA1, EMAC_TX_ER */
    [4] = { F_GPIO, F_IO(12), F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* GPIO5: GPIO5, VSPICS0, GPIO5, HS1_DATA6, -, EMAC_RX_CLK */
    [5] = { F_GPIO, F_IO(68), F_GPIO, F_DIRECT, F_NONE, F_DIRECT },
    /* SD_CLK: SD_CLK, SPICLK, GPIO6, HS1_CLK, U1CTS, - */
    [6] = { F_DIRECT, F_IO(0), F_GPIO, F_DIRECT, F_IN(18), F_NONE },
    /* SD_DATA_0: SD_DATA0, SPIQ, GPIO7, HS1_DATA0, U2RTS, - */
    [7] = { F_DIRECT, F_IO(1), F_GPIO, F_DIRECT, F_OUT(199), F_NONE },
    /* SD_DATA_1: SD_DATA1, SPID, GPIO8, HS1_DATA1, U2CTS, - */
    [8] = { F_DIRECT, F_IO(2), F_GPIO, F_DIRECT, F_IN(199), F_NONE },
    /* SD_DATA_2: SD_DATA2, SPIHD, GPIO9, HS1_DATA2, U1RXD, - */
    [9] = { F_DIRECT, F_IO(3), F_GPIO, F_DIRECT, F_IN(17), F_NONE },
    /* SD_DATA_3: SD_DATA3, SPIWP, GPIO10, HS1_DATA3, U1TXD, - */
    [10] = { F_DIRECT, F_IO(4), F_GPIO, F_DIRECT, F_OUT(17), F_NONE },
    /* SD_CMD: SD_CMD, SPICS0, GPIO11, HS1_CMD, U1RTS, - */
    [11] = { F_DIRECT, F_IO(5), F_GPIO, F_DIRECT, F_OUT(18), F_NONE },
    /* MTDI: MTDI, HSPIQ, GPIO12, HS2_DATA2, SD_DATA2, EMAC_TXD3 */
    [12] = { F_DIRECT, F_IO(9), F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* MTCK: MTCK, HSPID, GPIO13, HS2_DATA3, SD_DATA3, EMAC_RX_ER */
    [13] = { F_DIRECT, F_IO(10), F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* MTMS: MTMS, HSPICLK, GPIO14, HS2_CLK, SD_CLK, EMAC_TXD2 */
    [14] = { F_DIRECT, F_IO(8), F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* MTDO: MTDO, HSPICS0, GPIO15, HS2_CMD, SD_CMD, EMAC_RXD3 */
    [15] = { F_DIRECT, F_IO(11), F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* GPIO16: GPIO16, -, GPIO16, HS1_DATA4, U2RXD, EMAC_CLK_OUT */
    [16] = { F_GPIO, F_NONE, F_GPIO, F_DIRECT, F_IN(198), F_DIRECT },
    /* GPIO17: GPIO17, -, GPIO17, HS1_DATA5, U2TXD, EMAC_CLK_180 */
    [17] = { F_GPIO, F_NONE, F_GPIO, F_DIRECT, F_OUT(198), F_DIRECT },
    /* GPIO18: GPIO18, VSPICLK, GPIO18, HS1_DATA7, -, - */
    [18] = { F_GPIO, F_IO(63), F_GPIO, F_DIRECT, F_NONE, F_NONE },
    /* GPIO19: GPIO19, VSPIQ, GPIO19, U0CTS, -, EMAC_TXD0 */
    [19] = { F_GPIO, F_IO(64), F_GPIO, F_IN(15), F_NONE, F_DIRECT },
    /* GPIO21: GPIO21, VSPIHD, GPIO21, -, -, EMAC_TX_EN */
    [21] = { F_GPIO, F_IO(66), F_GPIO, F_NONE, F_NONE, F_DIRECT },
    /* GPIO22: GPIO22, VSPIWP, GPIO22, U0RTS, -, EMAC_TXD1 */
    [22] = { F_GPIO, F_IO(67), F_GPIO, F_OUT(15), F_NONE, F_DIRECT },
    /* GPIO23: GPIO23, VSPID, GPIO23, HS1_STROBE, -, - */
    [23] = { F_GPIO, F_IO(65), F_GPIO, F_DIRECT, F_NONE, F_NONE },
    /* GPIO25-27: GPIOn, -, GPIOn, -, -, EMAC_RXD0 / RXD1 / RX_DV */
    [25] = { F_GPIO, F_NONE, F_GPIO, F_NONE, F_NONE, F_DIRECT },
    [26] = { F_GPIO, F_NONE, F_GPIO, F_NONE, F_NONE, F_DIRECT },
    [27] = { F_GPIO, F_NONE, F_GPIO, F_NONE, F_NONE, F_DIRECT },
    [32 ... 39] = { F_GPIO, F_NONE, F_GPIO, F_NONE, F_NONE, F_NONE },
};

static const IomuxFunc iomux_none = F_NONE;

/*
 * The "Default Value If Unassigned" column of TRM table 6.9-1: the level an
 * input signal sees when it bypasses the GPIO matrix and no pad's IO_MUX
 * function carries it.
 */
static bool sig_in_default(unsigned sig)
{
    switch (sig) {
    case 29: /* I2CEXT0_SCL_in */
    case 30: /* I2CEXT0_SDA_in */
    case 94: /* twai_rx */
    case 95: /* I2CEXT1_SCL_in */
    case 96: /* I2CEXT1_SDA_in */
    case 115: /* pwm2_flta */
    case 116: /* pwm2_fltb */
    case 120: /* pwm3_flta */
    case 121: /* pwm3_fltb */
        return true;
    default:
        return false;
    }
}

/*
 * Output signals of peripherals that hold them high out of reset: the
 * UARTs' TXD, RTS and DTR, the SPI chip selects and the I2C lines. They
 * start high so that a peripheral which does not drive the line yet does
 * not pull its pad low.
 */
static bool sig_out_idle(unsigned sig)
{
    switch (sig) {
    case 5 ... 7:       /* SPICS0-2 */
    case 11:            /* HSPICS0 */
    case 14 ... 18:     /* U0TXD, U0RTS, U0DTR, U1TXD, U1RTS */
    case 29 ... 30:     /* I2CEXT0_SCL, I2CEXT0_SDA */
    case 61 ... 62:     /* HSPICS1-2 */
    case 68 ... 70:     /* VSPICS0-2 */
    case 95 ... 96:     /* I2CEXT1_SCL, I2CEXT1_SDA */
    case 198 ... 199:   /* U2TXD, U2RTS */
        return true;
    default:
        return false;
    }
}

static inline bool bit64(uint64_t v, unsigned n)
{
    return (v >> n) & 1;
}

static uint64_t gpio_out_all(Esp32GpioState *s)
{
    return s->out | ((uint64_t)s->out1 << 32);
}

static uint64_t gpio_enable_all(Esp32GpioState *s)
{
    return s->enable | ((uint64_t)s->enable1 << 32);
}

static uint64_t gpio_status_all(Esp32GpioState *s)
{
    return s->status | ((uint64_t)s->status1 << 32);
}

static const IomuxFunc *pad_func(Esp32GpioState *s, unsigned n,
                                 unsigned *func)
{
    unsigned f = FIELD_EX32(s->iomux[n], IO_MUX_GPIO36, MCU_SEL);

    *func = f;
    if (f >= ESP32_IOMUX_FUNC_COUNT) {
        return &iomux_none;
    }
    return &iomux_funcs[n][f];
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * The sigma-delta modulators are first-order: on every tick of
 * APB_CLK / (PRESCALE + 1) a modulator adds its duty, as an offset binary
 * 0-255 (GPIO_SDn_IN as a signed -128..127, plus 128), to an 8-bit
 * accumulator, and outputs 1 for the tick when the sum carries. The output's
 * density is (IN + 128) / 256. The accumulator advances lazily, by the
 * whole ticks elapsed since the modulator's settings last changed.
 */
static void sdm_advance(Esp32GpioState *s, unsigned ch, int64_t now)
{
    uint32_t reg = s->sdm_reg[ch];
    uint32_t div = FIELD_EX32(reg, GPIO_SIGMADELTA0, PRESCALE) + 1;
    uint32_t d = FIELD_EX32(reg, GPIO_SIGMADELTA0, IN) ^ 0x80;
    uint64_t total, n;
    uint32_t a;

    if (now <= s->sdm_anchor_ns[ch]) {
        return;
    }
    total = clock_ns_to_ticks(s->apb_clk, now - s->sdm_anchor_ns[ch]) / div;
    if (total <= s->sdm_ticks[ch]) {
        return;
    }
    n = total - s->sdm_ticks[ch];
    s->sdm_ticks[ch] = total;
    /* The accumulator before the last of the n ticks, modulo 256 */
    a = (s->sdm_acc[ch] + ((n - 1) & 0xff) * d) & 0xff;
    s->sdm_out[ch] = a + d > 0xff;
    s->sdm_acc[ch] = (a + d) & 0xff;
}

/* Bring a modulator up to now and restart its tick count from here. */
static void sdm_reanchor(Esp32GpioState *s, unsigned ch, int64_t now)
{
    sdm_advance(s, ch, now);
    s->sdm_anchor_ns[ch] = now;
    s->sdm_ticks[ch] = 0;
}

static void esp32_gpio_apb_update(void *opaque, ClockEvent event)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (unsigned ch = 0; ch < ESP32_GPIO_SDM_COUNT; ch++) {
        sdm_reanchor(s, ch, now);
    }
}

/* GPIO_STATUS bits of the pads whose GPIO_PINn_INT_ENA has bit k set */
static uint64_t int_view(Esp32GpioState *s, unsigned k)
{
    uint64_t ena = 0;

    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        if (FIELD_EX32(s->pin[n], GPIO_PIN0, INT_ENA) & (1u << k)) {
            ena |= 1ULL << n;
        }
    }
    return gpio_status_all(s) & ena;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Resolve every pad's level and input. Output: in the GPIO matrix function
 * GPIO_FUNCn_OUT_SEL picks a peripheral output signal, or GPIO_OUT and
 * GPIO_ENABLE for 256, and OUT_INV_SEL inverts it; in a peripheral's IO_MUX
 * function the peripheral drives the pad itself. Either way OEN_SEL takes
 * the output enable from GPIO_ENABLE instead of the peripheral and
 * OEN_INV_SEL inverts it, as ESP-IDF's gpio_ll_iomux_out() relies on.
 */
static void esp32_gpio_resolve(Esp32GpioState *s, uint64_t *pad_out,
                               uint64_t *in_out)
{
    uint64_t out_all = gpio_out_all(s);
    uint64_t enable_all = gpio_enable_all(s);
    uint64_t pad = 0, in = 0;

    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        uint32_t mux = s->iomux[n];
        uint32_t cfg = s->func_out_sel[n];
        bool val = false, oe = false, level, pull_up, ie;
        unsigned f;
        const IomuxFunc *fn;

        if (!bit64(PAD_VALID, n)) {
            continue;
        }
        fn = pad_func(s, n, &f);
        if (bit64(s->rtc_mux, n)) {
            /* [spec:nuos:req:emu.esp32.rtc] The RTC IO MUX has the pad */
            val = bit64(s->rtc_out, n);
            oe = bit64(PAD_OUTPUT, n) && bit64(s->rtc_oe, n);
            pull_up = bit64(s->rtc_pu, n);
            ie = bit64(s->rtc_ie, n);
        } else {
            pull_up = FIELD_EX32(mux, IO_MUX_GPIO36, FUN_WPU);
            ie = FIELD_EX32(mux, IO_MUX_GPIO36, FUN_IE);
        }
        if (bit64(PAD_OUTPUT, n) && !bit64(s->rtc_mux, n)) {
            bool oe_periph = false;
            unsigned sel, idx;

            switch (fn->kind) {
            case IOMUX_GPIO:
                sel = FIELD_EX32(cfg, GPIO_FUNC0_OUT_SEL_CFG, OUT_SEL);
                if (sel == ESP32_SIG_GPIO_OUT) {
                    val = bit64(out_all, n);
                    oe_periph = bit64(enable_all, n);
                } else if (sel < ESP32_GPIO_SIG_COUNT) {
                    val = s->sig_out[sel];
                    oe_periph = s->sig_oe[sel];
                }
                val ^= FIELD_EX32(cfg, GPIO_FUNC0_OUT_SEL_CFG, OUT_INV_SEL);
                break;
            case IOMUX_SIG:
                if (fn->sig_out >= 0) {
                    val = s->sig_out[fn->sig_out];
                    oe_periph = s->sig_oe[fn->sig_out];
                }
                break;
            case IOMUX_DIRECT:
                idx = n * ESP32_IOMUX_FUNC_COUNT + f;
                val = s->iomux_func_out[idx];
                oe_periph = s->iomux_func_oe[idx];
                break;
            default:
                break;
            }
            oe = FIELD_EX32(cfg, GPIO_FUNC0_OUT_SEL_CFG, OEN_SEL) ?
                 bit64(enable_all, n) : oe_periph;
            oe ^= FIELD_EX32(cfg, GPIO_FUNC0_OUT_SEL_CFG, OEN_INV_SEL);
        }
        if (!bit64(s->rtc_mux, n) &&
            FIELD_EX32(s->pin[n], GPIO_PIN0, PAD_DRIVER) && val) {
            /* Open drain: a high output releases the pad */
            oe = false;
        }

        if (bit64(s->ext_driven, n)) {
            level = bit64(s->ext_level, n);
        } else if (oe) {
            level = val;
        } else {
            /* Pulled up, or pulled down, or floating, which reads low */
            level = pull_up;
        }
        pad |= (uint64_t)level << n;
        if (ie) {
            in |= (uint64_t)level << n;
        }
    }
    *pad_out = pad;
    *in_out = in;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Latch pad interrupts into GPIO_STATUS. Edges are those of the pad's input
 * (GPIO_IN) between successive resolutions; level types set the status bit
 * for as long as the level holds, so clearing it while the level persists
 * sets it again.
 */
static void esp32_gpio_latch_interrupts(Esp32GpioState *s, uint64_t in)
{
    uint64_t edges = in ^ s->in_level;
    uint64_t set = 0;

    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        bool hit;

        switch (FIELD_EX32(s->pin[n], GPIO_PIN0, INT_TYPE)) {
        case INT_TYPE_RISING:
            hit = bit64(edges & in, n);
            break;
        case INT_TYPE_FALLING:
            hit = bit64(edges & ~in, n);
            break;
        case INT_TYPE_ANY_EDGE:
            hit = bit64(edges, n);
            break;
        case INT_TYPE_LOW:
            hit = bit64(PAD_VALID & ~in, n);
            break;
        case INT_TYPE_HIGH:
            hit = bit64(in, n);
            break;
        default:
            hit = false;
            break;
        }
        if (hit && bit64(PAD_VALID, n)) {
            set |= 1ULL << n;
        }
    }
    s->in_level = in;
    s->status |= (uint32_t)set;
    s->status1 |= (uint32_t)(set >> 32);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * The level of input signal sig. Through the matrix (SIG_IN_SEL) it is the
 * selected pad's input, or a constant, inverted by IN_INV_SEL. Bypassing
 * the matrix it is the input of the pad whose IO_MUX function carries sig,
 * or the signal's default when none does.
 */
static bool esp32_gpio_sig_in(Esp32GpioState *s, unsigned sig, uint64_t in,
                              const int8_t *iomux_src)
{
    uint32_t cfg = s->func_in_sel[sig];
    unsigned sel;
    bool v;

    if (!FIELD_EX32(cfg, GPIO_FUNC0_IN_SEL_CFG, SIG_IN_SEL)) {
        if (iomux_src[sig] >= 0) {
            return bit64(in, iomux_src[sig]);
        }
        return sig_in_default(sig);
    }
    sel = FIELD_EX32(cfg, GPIO_FUNC0_IN_SEL_CFG, IN_SEL);
    if (sel < ESP32_GPIO_PIN_COUNT) {
        v = bit64(in, sel);
    } else {
        v = sel == GPIO_IN_SEL_CONST_HIGH;
    }
    return v ^ FIELD_EX32(cfg, GPIO_FUNC0_IN_SEL_CFG, IN_INV_SEL);
}

/*
 * [spec:nuos:req:emu.esp32.rtc]
 * The GPIO wakeup from light sleep: a pad with GPIO_PINn_WAKEUP_ENABLE
 * whose input is at the level of its INT_TYPE, which must be a level type.
 */
static bool esp32_gpio_wakeup(Esp32GpioState *s, uint64_t in)
{
    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        uint32_t pin = s->pin[n];

        if (!bit64(PAD_VALID, n) ||
            !FIELD_EX32(pin, GPIO_PIN0, WAKEUP_ENABLE)) {
            continue;
        }
        switch (FIELD_EX32(pin, GPIO_PIN0, INT_TYPE)) {
        case INT_TYPE_LOW:
            if (!bit64(in, n)) {
                return true;
            }
            break;
        case INT_TYPE_HIGH:
            if (bit64(in, n)) {
                return true;
            }
            break;
        default:
            break;
        }
    }
    return false;
}

static void esp32_gpio_propagate(Esp32GpioState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int8_t iomux_src[ESP32_GPIO_SIG_COUNT];
    uint64_t pad, in, changed;
    bool resync = s->resync;
    bool irq[ESP32_GPIO_IRQ_COUNT];
    bool wakeup;

    s->resync = false;
    for (unsigned ch = 0; ch < ESP32_GPIO_SDM_COUNT; ch++) {
        sdm_advance(s, ch, now);
        s->sig_out[ESP32_SIG_GPIO_SD0_OUT + ch] = s->sdm_out[ch];
    }

    esp32_gpio_resolve(s, &pad, &in);
    esp32_gpio_latch_interrupts(s, in);
    changed = pad ^ s->pad_level;
    s->pad_level = pad;

    memset(iomux_src, -1, sizeof(iomux_src));
    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        unsigned f;
        const IomuxFunc *fn = pad_func(s, n, &f);

        if (bit64(PAD_VALID, n) && fn->kind == IOMUX_SIG &&
            fn->sig_in >= 0 && iomux_src[fn->sig_in] < 0) {
            iomux_src[fn->sig_in] = n;
        }
    }

    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        if (resync || bit64(changed, n)) {
            qemu_set_irq(s->pad_out[n], bit64(pad, n));
        }
    }
    for (unsigned sig = 0; sig < ESP32_GPIO_SIG_COUNT; sig++) {
        bool v = esp32_gpio_sig_in(s, sig, in, iomux_src);

        if (resync || v != s->sig_in[sig]) {
            s->sig_in[sig] = v;
            qemu_set_irq(s->sig_in_out[sig], v);
        }
        if (sig >= SIG_LOOPBACK_FIRST && sig <= SIG_LOOPBACK_LAST &&
            s->sig_out[sig] != v) {
            s->sig_out[sig] = v;
            s->update_pending = true;
        }
    }
    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        unsigned f;
        const IomuxFunc *fn = pad_func(s, n, &f);

        for (unsigned i = 0; i < ESP32_IOMUX_FUNC_COUNT; i++) {
            unsigned idx = n * ESP32_IOMUX_FUNC_COUNT + i;
            bool v = bit64(PAD_VALID, n) && fn->kind == IOMUX_DIRECT &&
                     f == i && bit64(in, n);

            if (resync || v != s->iomux_in[idx]) {
                s->iomux_in[idx] = v;
                qemu_set_irq(s->iomux_in_out[idx], v);
            }
        }
    }

    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        if (resync || bit64(in ^ s->rtc_in_level, n)) {
            qemu_set_irq(s->rtc_in_out[n], bit64(in, n));
        }
    }
    s->rtc_in_level = in;
    wakeup = esp32_gpio_wakeup(s, in);
    if (resync || wakeup != s->wakeup_level) {
        s->wakeup_level = wakeup;
        qemu_set_irq(s->wakeup_out, wakeup);
    }

    irq[ESP32_GPIO_IRQ_PRO] = int_view(s, INT_ENA_PRO) != 0;
    irq[ESP32_GPIO_IRQ_PRO_NMI] = int_view(s, INT_ENA_PRO_NMI) != 0;
    irq[ESP32_GPIO_IRQ_APP] = int_view(s, INT_ENA_APP) != 0;
    irq[ESP32_GPIO_IRQ_APP_NMI] = int_view(s, INT_ENA_APP_NMI) != 0;
    for (unsigned i = 0; i < ESP32_GPIO_IRQ_COUNT; i++) {
        if (resync || irq[i] != s->irq_level[i]) {
            s->irq_level[i] = irq[i];
            qemu_set_irq(s->irq[i], irq[i]);
        }
    }
}

/*
 * Bound on re-resolving the pads when peripherals answer an input change
 * with an output change, as a loop through the matrix that inverts itself
 * would otherwise never settle.
 */
#define GPIO_SETTLE_LIMIT 16

static void esp32_gpio_update(Esp32GpioState *s)
{
    unsigned rounds = 0;

    if (s->updating) {
        s->update_pending = true;
        return;
    }
    s->updating = true;
    do {
        s->update_pending = false;
        esp32_gpio_propagate(s);
        if (++rounds == GPIO_SETTLE_LIMIT && s->update_pending) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_gpio: signal routing oscillates through the "
                          "GPIO matrix\n");
            break;
        }
    } while (s->update_pending);
    s->update_pending = false;
    s->updating = false;
}

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    switch (addr) {
    case A_GPIO_BT_SELECT:
        return s->bt_select;
    case A_GPIO_OUT:
        return s->out;
    case A_GPIO_OUT1:
        return s->out1;
    case A_GPIO_SDIO_SELECT:
        return s->sdio_select;
    case A_GPIO_ENABLE:
        return s->enable;
    case A_GPIO_ENABLE1:
        return s->enable1;
    case A_GPIO_OUT_W1TS:
    case A_GPIO_OUT_W1TC:
    case A_GPIO_OUT1_W1TS:
    case A_GPIO_OUT1_W1TC:
    case A_GPIO_ENABLE_W1TS:
    case A_GPIO_ENABLE_W1TC:
    case A_GPIO_ENABLE1_W1TS:
    case A_GPIO_ENABLE1_W1TC:
    case A_GPIO_STATUS_W1TS:
    case A_GPIO_STATUS_W1TC:
    case A_GPIO_STATUS1_W1TS:
    case A_GPIO_STATUS1_W1TC:
        return 0;
    case A_GPIO_STRAP:
        return s->strap_mode & 0xffff;
    case A_GPIO_IN:
        esp32_gpio_update(s);
        return (uint32_t)s->in_level;
    case A_GPIO_IN1:
        esp32_gpio_update(s);
        return (uint32_t)(s->in_level >> 32);
    case A_GPIO_STATUS:
        esp32_gpio_update(s);
        return s->status;
    case A_GPIO_STATUS1:
        esp32_gpio_update(s);
        return s->status1;
    case A_GPIO_ACPU_INT:
    case A_GPIO_ACPU_NMI_INT:
    case A_GPIO_PCPU_INT:
    case A_GPIO_PCPU_NMI_INT:
        esp32_gpio_update(s);
        return (uint32_t)int_view(s, (addr - A_GPIO_ACPU_INT) / 4);
    case A_GPIO_CPUSDIO_INT:
        esp32_gpio_update(s);
        return (uint32_t)int_view(s, INT_ENA_SDIO);
    case A_GPIO_ACPU_INT1:
    case A_GPIO_ACPU_NMI_INT1:
    case A_GPIO_PCPU_INT1:
    case A_GPIO_PCPU_NMI_INT1:
        esp32_gpio_update(s);
        return (uint32_t)(int_view(s, (addr - A_GPIO_ACPU_INT1) / 4) >> 32);
    case A_GPIO_CPUSDIO_INT1:
        esp32_gpio_update(s);
        return (uint32_t)(int_view(s, INT_ENA_SDIO) >> 32);
    case A_GPIO_PIN0 ... A_GPIO_PIN0 + 39 * GPIO_PIN_STRIDE:
        return s->pin[(addr - A_GPIO_PIN0) / GPIO_PIN_STRIDE];
    case A_GPIO_CALI_CONF:
        return s->cali_conf;
    case A_GPIO_CALI_DATA:
        qemu_log_mask(LOG_UNIMP,
                      "esp32_gpio: GPIO_CALI_DATA not implemented\n");
        return 0;
    case A_GPIO_FUNC0_IN_SEL_CFG ... A_GPIO_FUNC0_IN_SEL_CFG + 255 * 4:
        return s->func_in_sel[(addr - A_GPIO_FUNC0_IN_SEL_CFG) / 4];
    case A_GPIO_FUNC0_OUT_SEL_CFG ... A_GPIO_FUNC0_OUT_SEL_CFG + 39 * 4:
        return s->func_out_sel[(addr - A_GPIO_FUNC0_OUT_SEL_CFG) / 4];
    case A_GPIO_SIGMADELTA0 ... A_GPIO_SIGMADELTA0 + 7 * 4:
        return s->sdm_reg[(addr - A_GPIO_SIGMADELTA0) / 4];
    case A_GPIO_SIGMADELTA_CG:
        return s->sdm_cg;
    case A_GPIO_SIGMADELTA_MISC:
        return s->sdm_misc;
    case A_GPIO_SIGMADELTA_VERSION:
        return s->sdm_version;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_gpio: read from reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void esp32_gpio_check_in_sel(unsigned sig, uint32_t cfg)
{
    unsigned sel = FIELD_EX32(cfg, GPIO_FUNC0_IN_SEL_CFG, IN_SEL);

    if (!FIELD_EX32(cfg, GPIO_FUNC0_IN_SEL_CFG, SIG_IN_SEL)) {
        return;
    }
    if (sel < ESP32_GPIO_PIN_COUNT ? !bit64(PAD_VALID, sel) :
        sel != GPIO_IN_SEL_CONST_LOW && sel != GPIO_IN_SEL_CONST_HIGH) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_gpio: input signal %u routed from 0x%x, which "
                      "is neither a pad nor a constant; it reads low\n",
                      sig, sel);
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * GPIO matrix and sigma-delta register writes.
 */
static void esp32_gpio_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = value;
    unsigned i;

    switch (addr) {
    case A_GPIO_BT_SELECT:
        s->bt_select = v;
        break;
    case A_GPIO_OUT:
        s->out = v;
        break;
    case A_GPIO_OUT_W1TS:
        s->out |= v;
        break;
    case A_GPIO_OUT_W1TC:
        s->out &= ~v;
        break;
    case A_GPIO_OUT1:
        s->out1 = v & HIGH_PADS_MASK;
        break;
    case A_GPIO_OUT1_W1TS:
        s->out1 |= v & HIGH_PADS_MASK;
        break;
    case A_GPIO_OUT1_W1TC:
        s->out1 &= ~v;
        break;
    case A_GPIO_SDIO_SELECT:
        s->sdio_select = v & 0xff;
        break;
    case A_GPIO_ENABLE:
        s->enable = v;
        break;
    case A_GPIO_ENABLE_W1TS:
        s->enable |= v;
        break;
    case A_GPIO_ENABLE_W1TC:
        s->enable &= ~v;
        break;
    case A_GPIO_ENABLE1:
        s->enable1 = v & HIGH_PADS_MASK;
        break;
    case A_GPIO_ENABLE1_W1TS:
        s->enable1 |= v & HIGH_PADS_MASK;
        break;
    case A_GPIO_ENABLE1_W1TC:
        s->enable1 &= ~v;
        break;
    case A_GPIO_STATUS:
        s->status = v;
        break;
    case A_GPIO_STATUS_W1TS:
        s->status |= v;
        break;
    case A_GPIO_STATUS_W1TC:
        s->status &= ~v;
        break;
    case A_GPIO_STATUS1:
        s->status1 = v & HIGH_PADS_MASK;
        break;
    case A_GPIO_STATUS1_W1TS:
        s->status1 |= v & HIGH_PADS_MASK;
        break;
    case A_GPIO_STATUS1_W1TC:
        s->status1 &= ~v;
        break;
    case A_GPIO_PIN0 ... A_GPIO_PIN0 + 39 * GPIO_PIN_STRIDE:
        i = (addr - A_GPIO_PIN0) / GPIO_PIN_STRIDE;
        s->pin[i] = v & GPIO_PIN_WRITABLE;
        if (FIELD_EX32(v, GPIO_PIN0, WAKEUP_ENABLE)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_gpio: GPIO%u wakeup: light sleep is not "
                          "modelled\n", i);
        }
        break;
    case A_GPIO_CALI_CONF:
        s->cali_conf = v & 0x800003ff;
        if (v & (1u << 31)) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_gpio: GPIO_CALI_START not implemented\n");
        }
        break;
    case A_GPIO_FUNC0_IN_SEL_CFG ... A_GPIO_FUNC0_IN_SEL_CFG + 255 * 4:
        i = (addr - A_GPIO_FUNC0_IN_SEL_CFG) / 4;
        s->func_in_sel[i] = v & GPIO_FUNC_IN_WRITABLE;
        esp32_gpio_check_in_sel(i, s->func_in_sel[i]);
        break;
    case A_GPIO_FUNC0_OUT_SEL_CFG ... A_GPIO_FUNC0_OUT_SEL_CFG + 39 * 4:
        i = (addr - A_GPIO_FUNC0_OUT_SEL_CFG) / 4;
        s->func_out_sel[i] = v & GPIO_FUNC_OUT_WRITABLE;
        if (FIELD_EX32(v, GPIO_FUNC0_OUT_SEL_CFG, OUT_SEL) >
            ESP32_SIG_GPIO_OUT) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_gpio: GPIO%u output from signal %u, which "
                          "does not exist; it drives low\n", i,
                          (unsigned)FIELD_EX32(v, GPIO_FUNC0_OUT_SEL_CFG,
                                               OUT_SEL));
        }
        break;
    case A_GPIO_SIGMADELTA0 ... A_GPIO_SIGMADELTA0 + 7 * 4:
        i = (addr - A_GPIO_SIGMADELTA0) / 4;
        sdm_reanchor(s, i, now);
        s->sdm_reg[i] = v & GPIO_SDM_WRITABLE;
        break;
    case A_GPIO_SIGMADELTA_CG:
        /*
         * ESP-IDF never sets SD_CLK_EN on the ESP32 ("the clk enable
         * register does not exist") and the modulators run regardless, so
         * the bit is kept but gates nothing.
         */
        s->sdm_cg = v & GPIO_SD_CLK_EN;
        break;
    case A_GPIO_SIGMADELTA_MISC:
        s->sdm_misc = v & GPIO_SPI_SWAP;
        if (v & GPIO_SPI_SWAP) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32_gpio: GPIO_SPI_SWAP not implemented\n");
        }
        break;
    case A_GPIO_SIGMADELTA_VERSION:
        s->sdm_version = v & GPIO_SD_DATE_MASK;
        break;
    case A_GPIO_STRAP:
    case A_GPIO_IN:
    case A_GPIO_IN1:
    case A_GPIO_ACPU_INT ... A_GPIO_CPUSDIO_INT1:
    case A_GPIO_CALI_DATA:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_gpio: write to read-only register 0x%"
                      HWADDR_PRIx "\n", addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_gpio: write to reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return;
    }
    esp32_gpio_update(s);
}

static const MemoryRegionOps esp32_gpio_ops = {
    .read = esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static uint32_t iomux_writable(unsigned gpio)
{
    /* GPIO34-39 have no output driver and no pulls */
    if (gpio >= 34) {
        return IO_MUX_PIN_WRITABLE & ~IO_MUX_NO_PULL_DRV;
    }
    return IO_MUX_PIN_WRITABLE;
}

static uint64_t esp32_iomux_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    if (addr < 4) {
        return s->pin_ctrl;
    }
    if (addr <= IO_MUX_LAST_REG) {
        return s->iomux[iomux_reg_gpio[addr / 4 - 1]];
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32_iomux: read from reserved offset 0x%" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * IO_MUX register writes. MCU_SEL, FUN_IE and the pulls take effect at
 * once; the drive strength only sets how hard a pad is driven, which the
 * model does not resolve, and the MCU_* (SLP_SEL) settings apply only in
 * light sleep.
 */
static void esp32_iomux_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    unsigned gpio;

    if (addr < 4) {
        s->pin_ctrl = value & GPIO_PIN_CTRL_WRITABLE;
        qemu_log_mask(LOG_UNIMP,
                      "esp32_iomux: clock outputs on CLK_OUT1-3 not "
                      "implemented\n");
        return;
    }
    if (addr > IO_MUX_LAST_REG) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_iomux: write to reserved offset 0x%" HWADDR_PRIx
                      "\n", addr);
        return;
    }
    gpio = iomux_reg_gpio[addr / 4 - 1];
    s->iomux[gpio] = value & iomux_writable(gpio);
    if (FIELD_EX32(value, IO_MUX_GPIO36, SLP_SEL)) {
        qemu_log_mask(LOG_UNIMP,
                      "esp32_iomux: GPIO%u sleep configuration: light "
                      "sleep is not modelled\n", gpio);
    }
    esp32_gpio_update(s);
}

static const MemoryRegionOps esp32_iomux_ops = {
    .read = esp32_iomux_read,
    .write = esp32_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32_gpio_pad_in(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    s->ext_driven |= 1ULL << n;
    if (level) {
        s->ext_level |= 1ULL << n;
    } else {
        s->ext_level &= ~(1ULL << n);
    }
    esp32_gpio_update(s);
}

static void esp32_gpio_pad_release(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    if (level) {
        s->ext_driven &= ~(1ULL << n);
        esp32_gpio_update(s);
    }
}

static void esp32_gpio_set_sig_out(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    s->sig_out[n] = level != 0;
    esp32_gpio_update(s);
}

static void esp32_gpio_set_sig_oe(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    s->sig_oe[n] = level != 0;
    esp32_gpio_update(s);
}

static void esp32_gpio_set_iomux_out(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    s->iomux_func_out[n] = level != 0;
    esp32_gpio_update(s);
}

static void esp32_gpio_set_iomux_oe(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    s->iomux_func_oe[n] = level != 0;
    esp32_gpio_update(s);
}

static void esp32_gpio_set_rtc(uint64_t *mask, Esp32GpioState *s, int n,
                               int level)
{
    *mask = deposit64(*mask, n, 1, level != 0);
    esp32_gpio_update(s);
}

static void esp32_gpio_set_rtc_mux(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    esp32_gpio_set_rtc(&s->rtc_mux, s, n, level);
}

static void esp32_gpio_set_rtc_out(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    esp32_gpio_set_rtc(&s->rtc_out, s, n, level);
}

static void esp32_gpio_set_rtc_oe(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    esp32_gpio_set_rtc(&s->rtc_oe, s, n, level);
}

static void esp32_gpio_set_rtc_pu(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    esp32_gpio_set_rtc(&s->rtc_pu, s, n, level);
}

static void esp32_gpio_set_rtc_pd(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    esp32_gpio_set_rtc(&s->rtc_pd, s, n, level);
}

static void esp32_gpio_set_rtc_ie(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    esp32_gpio_set_rtc(&s->rtc_ie, s, n, level);
}

/*
 * The TRM gives no reset value ("x") for the GPIO matrix registers; ESP-IDF
 * reads none of them before writing. They reset to 0, but GPIO_FUNCn_OUT_SEL
 * resets to 256, as on the ESP32's successors, so that a pad switched to its
 * GPIO function is a plain GPIO, undriven until GPIO_ENABLE is set. The
 * IO_MUX and sigma-delta reset values are the TRM's and ESP-IDF's.
 */
static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->bt_select = 0;
    s->sdio_select = 0;
    s->out = 0;
    s->out1 = 0;
    s->enable = 0;
    s->enable1 = 0;
    s->status = 0;
    s->status1 = 0;
    s->cali_conf = 0;
    memset(s->pin, 0, sizeof(s->pin));
    memset(s->func_in_sel, 0, sizeof(s->func_in_sel));
    for (unsigned n = 0; n < ESP32_GPIO_PIN_COUNT; n++) {
        uint32_t mux = 0;

        s->func_out_sel[n] = ESP32_SIG_GPIO_OUT;
        if (n < 34) {
            mux = FIELD_DP32(mux, IO_MUX_GPIO36, FUN_DRV, 2);
        }
        switch (pad_reset[n]) {
        case PAD_RST_IE_WPU:
            mux = FIELD_DP32(mux, IO_MUX_GPIO36, FUN_WPU, 1);
            mux = FIELD_DP32(mux, IO_MUX_GPIO36, FUN_IE, 1);
            break;
        case PAD_RST_IE_WPD:
            mux = FIELD_DP32(mux, IO_MUX_GPIO36, FUN_WPD, 1);
            mux = FIELD_DP32(mux, IO_MUX_GPIO36, FUN_IE, 1);
            break;
        case PAD_RST_IE:
            mux = FIELD_DP32(mux, IO_MUX_GPIO36, FUN_IE, 1);
            break;
        default:
            break;
        }
        s->iomux[n] = mux;
    }
    s->pin_ctrl = 0;
    for (unsigned ch = 0; ch < ESP32_GPIO_SDM_COUNT; ch++) {
        s->sdm_reg[ch] = FIELD_DP32(0, GPIO_SIGMADELTA0, PRESCALE, 0xff);
        s->sdm_anchor_ns[ch] = now;
        s->sdm_ticks[ch] = 0;
        s->sdm_acc[ch] = 0;
        s->sdm_out[ch] = 0;
    }
    s->sdm_cg = 0;
    s->sdm_misc = 0;
    s->sdm_version = GPIO_SD_DATE_RESET;
    s->resync = true;
}

static void esp32_gpio_reset_exit(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);

    /* Interrupt types are all disabled, so this latches no interrupt. */
    esp32_gpio_update(s);
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_gpio_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    memory_region_init_io(&s->iomux_iomem, obj, &esp32_iomux_ops, s,
                          TYPE_ESP32_GPIO ".iomux", 0x1000);
    sysbus_init_mmio(sbd, &s->iomux_iomem);
    for (unsigned i = 0; i < ESP32_GPIO_IRQ_COUNT; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    s->apb_clk = qdev_init_clock_in(dev, "apb", esp32_gpio_apb_update, s,
                                    ClockPreUpdate);

    qdev_init_gpio_in_named(dev, esp32_gpio_pad_in, ESP32_GPIO_PAD_IN,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_pad_release,
                            ESP32_GPIO_PAD_RELEASE, ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_out_named(dev, s->pad_out, ESP32_GPIO_PAD_OUT,
                             ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_sig_out, ESP32_GPIO_SIG_OUT,
                            ESP32_GPIO_SIG_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_sig_oe, ESP32_GPIO_SIG_OE,
                            ESP32_GPIO_SIG_COUNT);
    qdev_init_gpio_out_named(dev, s->sig_in_out, ESP32_GPIO_SIG_IN,
                             ESP32_GPIO_SIG_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_iomux_out,
                            ESP32_GPIO_IOMUX_OUT,
                            ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_iomux_oe,
                            ESP32_GPIO_IOMUX_OE,
                            ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT);
    qdev_init_gpio_out_named(dev, s->iomux_in_out, ESP32_GPIO_IOMUX_IN,
                             ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_rtc_mux, ESP32_GPIO_RTC_MUX,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_rtc_out, ESP32_GPIO_RTC_OUT,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_rtc_oe, ESP32_GPIO_RTC_OE,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_rtc_pu, ESP32_GPIO_RTC_PU,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_rtc_pd, ESP32_GPIO_RTC_PD,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_in_named(dev, esp32_gpio_set_rtc_ie, ESP32_GPIO_RTC_IE,
                            ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_out_named(dev, s->rtc_in_out, ESP32_GPIO_RTC_IN,
                             ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_out_named(dev, &s->wakeup_out, ESP32_GPIO_WAKEUP, 1);

    /*
     * The signals peripherals drive are theirs, not reset with this block:
     * they start at the level the peripherals hold out of reset. The RTC
     * pad controls are the RTC domain's, which a reset of the digital
     * domain leaves as they are.
     */
    for (unsigned sig = 0; sig < ESP32_GPIO_SIG_COUNT; sig++) {
        s->sig_out[sig] = sig_out_idle(sig);
        s->sig_oe[sig] = 1;
    }
}

static const Property esp32_gpio_properties[] = {
    /* The boot-mode strapping pins as latched at reset (GPIO_STRAP) */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode,
                       ESP32_STRAP_MODE_FLASH_BOOT),
};

static const VMStateDescription vmstate_esp32_gpio = {
    .name = TYPE_ESP32_GPIO,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(apb_clk, Esp32GpioState),
        VMSTATE_UINT32(bt_select, Esp32GpioState),
        VMSTATE_UINT32(sdio_select, Esp32GpioState),
        VMSTATE_UINT32(out, Esp32GpioState),
        VMSTATE_UINT32(out1, Esp32GpioState),
        VMSTATE_UINT32(enable, Esp32GpioState),
        VMSTATE_UINT32(enable1, Esp32GpioState),
        VMSTATE_UINT32(status, Esp32GpioState),
        VMSTATE_UINT32(status1, Esp32GpioState),
        VMSTATE_UINT32_ARRAY(pin, Esp32GpioState, ESP32_GPIO_PIN_COUNT),
        VMSTATE_UINT32(cali_conf, Esp32GpioState),
        VMSTATE_UINT32_ARRAY(func_in_sel, Esp32GpioState,
                             ESP32_GPIO_SIG_COUNT),
        VMSTATE_UINT32_ARRAY(func_out_sel, Esp32GpioState,
                             ESP32_GPIO_PIN_COUNT),
        VMSTATE_UINT32(pin_ctrl, Esp32GpioState),
        VMSTATE_UINT32_ARRAY(iomux, Esp32GpioState, ESP32_GPIO_PIN_COUNT),
        VMSTATE_UINT32_ARRAY(sdm_reg, Esp32GpioState, ESP32_GPIO_SDM_COUNT),
        VMSTATE_UINT32(sdm_cg, Esp32GpioState),
        VMSTATE_UINT32(sdm_misc, Esp32GpioState),
        VMSTATE_UINT32(sdm_version, Esp32GpioState),
        VMSTATE_INT64_ARRAY(sdm_anchor_ns, Esp32GpioState,
                            ESP32_GPIO_SDM_COUNT),
        VMSTATE_UINT64_ARRAY(sdm_ticks, Esp32GpioState, ESP32_GPIO_SDM_COUNT),
        VMSTATE_UINT32_ARRAY(sdm_acc, Esp32GpioState, ESP32_GPIO_SDM_COUNT),
        VMSTATE_UINT8_ARRAY(sdm_out, Esp32GpioState, ESP32_GPIO_SDM_COUNT),
        VMSTATE_UINT64(ext_driven, Esp32GpioState),
        VMSTATE_UINT64(ext_level, Esp32GpioState),
        VMSTATE_UINT8_ARRAY(sig_out, Esp32GpioState, ESP32_GPIO_SIG_COUNT),
        VMSTATE_UINT8_ARRAY(sig_oe, Esp32GpioState, ESP32_GPIO_SIG_COUNT),
        VMSTATE_UINT8_ARRAY(iomux_func_out, Esp32GpioState,
                            ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT),
        VMSTATE_UINT8_ARRAY(iomux_func_oe, Esp32GpioState,
                            ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT),
        VMSTATE_UINT64(pad_level, Esp32GpioState),
        VMSTATE_UINT64(in_level, Esp32GpioState),
        VMSTATE_UINT8_ARRAY(sig_in, Esp32GpioState, ESP32_GPIO_SIG_COUNT),
        VMSTATE_UINT8_ARRAY(iomux_in, Esp32GpioState,
                            ESP32_GPIO_PIN_COUNT * ESP32_IOMUX_FUNC_COUNT),
        VMSTATE_UINT8_ARRAY(irq_level, Esp32GpioState, ESP32_GPIO_IRQ_COUNT),
        VMSTATE_UINT64_V(rtc_mux, Esp32GpioState, 2),
        VMSTATE_UINT64_V(rtc_out, Esp32GpioState, 2),
        VMSTATE_UINT64_V(rtc_oe, Esp32GpioState, 2),
        VMSTATE_UINT64_V(rtc_pu, Esp32GpioState, 2),
        VMSTATE_UINT64_V(rtc_pd, Esp32GpioState, 2),
        VMSTATE_UINT64_V(rtc_ie, Esp32GpioState, 2),
        VMSTATE_UINT64_V(rtc_in_level, Esp32GpioState, 2),
        VMSTATE_BOOL_V(wakeup_level, Esp32GpioState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_gpio_reset_hold;
    rc->phases.exit = esp32_gpio_reset_exit;
    dc->realize = esp32_gpio_realize;
    dc->vmsd = &vmstate_esp32_gpio;
    device_class_set_props(dc, esp32_gpio_properties);
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
