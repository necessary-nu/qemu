/*
 * ESP32-P4 GPIO matrix, IO MUX, GPIO_EXT, LP GPIO matrix and LP IO MUX
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 10 (GPIO Matrix and IO MUX), and ESP-IDF's v3 register headers
 * gpio_reg.h, gpio_ext_reg.h, io_mux_reg.h, lp_gpio_reg.h and
 * lp_iomux_reg.h, which are the authority for offsets, fields and reset
 * values where the TRM differs: GPIO_FUNCn_IN_SEL_CFG_REG is at
 * 0x158 + 4 * n (the TRM's 0x15C + 4 * n counts from signal 1), exists
 * only for the 222 input signals (none for 0, 46, 67, 72, 73, 79, 81, 82,
 * 84, 85, 87, 88, 115, 116, 119 to 125, 138, 139, 157 and 204 to 213),
 * and resets to a constant input of the signal's default level (63 for a
 * default of 1, 62 for 0), which the TRM gives as 63 throughout;
 * GPIO_DATE resets to 0x241226; GPIO_BT_SELECT, GPIO_SEND_SEQ,
 * GPIO_RECIVE_SEQ, GPIO_BISTIN_SEL, GPIO_BIST_CTRL (with BIST_MODE) and
 * LP_IOMUX_EXT_WAKEUP0_SEL, which the TRM leaves out, are there; and the
 * GPIO55 and GPIO56 slots exist in every per-pad register file.
 *
 * Pads. Each of GPIO0 to GPIO54 takes its level from, in order: a driver
 * outside the chip; the chip's output driver while its output enable is
 * set (an open-drain pad, GPIO_PINn_PAD_DRIVER or LP_GPIO_PINn_PAD_DRIVER,
 * drives only low); a pull-up; otherwise it is low, pulled down or
 * floating, which reads low. Its input buffer delivers that level while
 * its input is enabled, and low otherwise. GPIO0 to GPIO15 belong to the
 * HP IO MUX unless LP_IOMUX_PADn_MUX_SEL hands them to the LP IO MUX,
 * whose register then gives the pad's function, pulls, input enable and
 * open drain, its output coming from the LP GPIO matrix; either way the
 * one input buffer feeds both matrices, so GPIO_IN and LP_GPIO_IN both
 * see the pad. The drive strengths and hysteresis only shape the pad's
 * analogue behaviour, which a digital model does not resolve; they are
 * kept. IO_MUX_GPIOn_I3C_RUE_EN is a further pull-up, on GPIO32 and GPIO33
 * only.
 *
 * Input chains. Through the HP GPIO matrix a pad's input passes its pin
 * filter (IO_MUX_GPIOn_FILTER_EN), then the glitch filter channel whose
 * GPIO_EXT_FILTER_CHn_INPUT_IO_NUM names it, if one is enabled, and the
 * result is GPIO_IN, the pad's interrupt and wakeup, and its level as
 * the matrix routes it to input signals. The pin filter passes a level
 * once the input has held it for two cycles of the HP IO MUX operating
 * clock. A glitch filter samples its input on every such cycle and takes
 * the input's level once WINDOW_THRES + 1 of the last WINDOW_WIDTH + 1
 * samples differ from its output, its window then starting afresh. The
 * GPIO SYNC stages (GPIO_PINn_SYNC1/2_BYPASS) delay a level by at most two
 * of those cycles and do not change it; the model does not delay it. The
 * LP GPIO matrix has only pin filters (LP_IOMUX_PADn_FILTER_EN), clocked
 * by the LP IO MUX's clock. A filter's clock stopped, it holds its
 * output. An IO MUX function that carries a peripheral signal directly
 * takes the pad's input unfiltered. A filter's output changes at the
 * clock edge that decides it: everything the change reaches, an ETM
 * event among them, sees that time (see esp32_line.h), however late the
 * QEMU timer that runs it, and every change from outside first runs the
 * filter changes due before it, in order.
 *
 * Interrupts. GPIO_STATUS latches each pad's interrupt type
 * (GPIO_PINn_INT_TYPE) on its filtered input: edges as they happen, and
 * a level type for as long as the level holds, so that clearing it while
 * the level holds sets it again. GPIO_INTR0 to GPIO_INTR3 are each the OR
 * of the status bits of the pads whose GPIO_PINn_INT_ENA has bit 0, 1, 3
 * or 4 set (bit 2 is the TRM's "invalid" one), each a source of the
 * interrupt matrix, which routes it to either HP core. LP_GPIO_STATUS
 * latches in the same way on the LP chain, and LP_GPIO_INTR is the OR of
 * all its bits. GPIO_STATUS_NEXT and LP_GPIO_STATUS_NEXT show the
 * interrupt condition itself: a level type's level, or an edge type's
 * one-cycle pulse, which software never sees.
 *
 * Sigma-delta modulators. Each of the eight takes a tick of the HP IO MUX
 * operating clock divided by GPIO_EXT_SDn_PRESCALE + 1, while
 * GPIO_EXT_SD_FUNCTION_CLK_EN is set, and is output signal 72 + n. The
 * TRM calls the modulators second-order, H(z) = X(z)z^-1 +
 * E(z)(1 - z^-1)^2, but gives neither their structure nor their
 * integrators' widths and saturation, without which a 1-bit second-order
 * loop is unstable towards full scale; the model is a first-order
 * accumulator, whose pulse density is exactly the TRM's
 * (SDn_IN + 128) / 256 at every input, though its bit pattern shapes the
 * quantisation noise less steeply than the hardware's.
 *
 * Everything is recomputed whenever a register, a peripheral signal, an
 * outside driver or a filter changes, and whenever software samples the
 * pads, so that the modulators' bitstreams are read at the virtual time
 * of the access. A recomputation drives every line it changes at one
 * time (see esp32_line.h): that of the change it propagates, and for a
 * register access the moment the access began, so that every block an
 * edge reaches, through the matrix, the IO MUX or the ETM, sees it at the
 * same moment.
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/registerfields.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "migration/vmstate.h"

/* HP GPIO matrix */
#define A_BT_SELECT         0x000
#define A_OUT               0x004
#define A_OUT_W1TS          0x008
#define A_OUT_W1TC          0x00c
#define A_OUT1              0x010
#define A_OUT1_W1TS         0x014
#define A_OUT1_W1TC         0x018
#define A_ENABLE            0x020
#define A_ENABLE_W1TS       0x024
#define A_ENABLE_W1TC       0x028
#define A_ENABLE1           0x02c
#define A_ENABLE1_W1TS      0x030
#define A_ENABLE1_W1TC      0x034
#define A_STRAP             0x038
#define A_IN                0x03c
#define A_IN1               0x040
#define A_STATUS            0x044
#define A_STATUS_W1TS       0x048
#define A_STATUS_W1TC       0x04c
#define A_STATUS1           0x050
#define A_STATUS1_W1TS      0x054
#define A_STATUS1_W1TC      0x058
#define A_INTR_0            0x05c
#define A_INTR1_0           0x060
#define A_INTR_1            0x064
#define A_INTR1_1           0x068
#define A_STATUS_NEXT       0x06c
#define A_STATUS_NEXT1      0x070
#define A_PIN0              0x074
#define A_PIN_LAST          (A_PIN0 + 4 * (ESP32P4_GPIO_SLOTS - 1))
#define A_FUNC0_IN_SEL      0x158
#define A_FUNC_IN_SEL_LAST  (A_FUNC0_IN_SEL + 4 * (ESP32P4_GPIO_SIGS - 1))
#define A_FUNC0_OUT_SEL     0x558
#define A_FUNC_OUT_SEL_LAST (A_FUNC0_OUT_SEL + 4 * (ESP32P4_GPIO_SLOTS - 1))
#define A_INTR_2            0x63c
#define A_INTR1_2           0x640
#define A_INTR_3            0x644
#define A_INTR1_3           0x648
#define A_CLOCK_GATE        0x64c
#define A_INT_RAW           0x700
#define A_INT_ST            0x704
#define A_INT_ENA           0x708
#define A_INT_CLR           0x70c
#define A_ZERO_DET0         0x710
#define A_ZERO_DET1         0x714
#define A_SEND_SEQ          0x718
#define A_RECIVE_SEQ        0x71c
#define A_BISTIN_SEL        0x720
#define A_BIST_CTRL         0x724
#define A_DATE              0x7fc

/* GPIO_EXT, in the same region from 0xF00 */
#define A_SIGMADELTA0       0xf00
#define A_SIGMADELTA_LAST   (A_SIGMADELTA0 + 4 * (ESP32P4_GPIO_SDMS - 1))
#define A_SIGMADELTA_MISC   0xf24
#define A_GLITCH0           0xf30
#define A_GLITCH_LAST       (A_GLITCH0 + 4 * (ESP32P4_GPIO_GLITCH_FILTERS - 1))
#define A_ETM_EVENT0        0xf60
#define A_ETM_EVENT_LAST    (A_ETM_EVENT0 + 4 * (ESP32P4_GPIO_ETM_CHANNELS - 1))
#define A_ETM_TASK0         0xfa0
#define A_ETM_TASK_LAST     0xfd4
#define A_EXT_VERSION       0xffc

/* HP IO MUX */
#define A_IOMUX_GPIO0       0x004
#define A_IOMUX_LAST        (A_IOMUX_GPIO0 + 4 * (ESP32P4_GPIO_SLOTS - 1))
#define A_IOMUX_DATE        0x104

/* LP GPIO matrix */
#define A_LP_CLK_EN         0x000
#define A_LP_VER_DATE       0x004
#define A_LP_OUT            0x008
#define A_LP_OUT_W1TS       0x00c
#define A_LP_OUT_W1TC       0x010
#define A_LP_ENABLE         0x014
#define A_LP_ENABLE_W1TS    0x018
#define A_LP_ENABLE_W1TC    0x01c
#define A_LP_STATUS         0x020
#define A_LP_STATUS_W1TS    0x024
#define A_LP_STATUS_W1TC    0x028
#define A_LP_STATUS_NEXT    0x02c
#define A_LP_IN             0x030
#define A_LP_PIN0           0x034
#define A_LP_PIN_LAST       (A_LP_PIN0 + 4 * (ESP32P4_GPIO_LP_PADS - 1))
#define A_LP_FUNC0_IN_SEL   0x074
/* LP input signals 0 to 13 have a register. */
#define LP_IN_SIGS          14
#define A_LP_FUNC_IN_LAST   (A_LP_FUNC0_IN_SEL + 4 * (LP_IN_SIGS - 1))
#define A_LP_FUNC0_OUT_SEL  0x0f4
#define A_LP_FUNC_OUT_LAST  (A_LP_FUNC0_OUT_SEL + \
                             4 * (ESP32P4_GPIO_LP_PADS - 1))

/* LP IO MUX */
#define A_LPIO_CLK_EN       0x000
#define A_LPIO_VER_DATE     0x004
#define A_LPIO_PAD0         0x008
#define A_LPIO_PAD_LAST     (A_LPIO_PAD0 + 4 * (ESP32P4_GPIO_LP_PADS - 1))
#define A_LPIO_EXT_WAKEUP0  0x048
#define A_LPIO_PAD_HOLD     0x04c
#define A_LPIO_PAD_HYS      0x050

REG32(PIN, 0)
    FIELD(PIN, SYNC2_BYPASS, 0, 2)
    FIELD(PIN, PAD_DRIVER, 2, 1)
    FIELD(PIN, SYNC1_BYPASS, 3, 2)
    FIELD(PIN, INT_TYPE, 7, 3)
    FIELD(PIN, WAKEUP_ENABLE, 10, 1)
    FIELD(PIN, CONFIG, 11, 2)
    FIELD(PIN, INT_ENA, 13, 5)
REG32(FUNC_IN, 0)
    FIELD(FUNC_IN, IN_SEL, 0, 6)
    FIELD(FUNC_IN, IN_INV_SEL, 6, 1)
    FIELD(FUNC_IN, SIG_IN_SEL, 7, 1)
REG32(FUNC_OUT, 0)
    FIELD(FUNC_OUT, OUT_SEL, 0, 9)
    FIELD(FUNC_OUT, OUT_INV_SEL, 9, 1)
    FIELD(FUNC_OUT, OEN_SEL, 10, 1)
    FIELD(FUNC_OUT, OEN_INV_SEL, 11, 1)
REG32(IOMUX, 0)
    FIELD(IOMUX, MCU_OE, 0, 1)
    FIELD(IOMUX, SLP_SEL, 1, 1)
    FIELD(IOMUX, MCU_WPD, 2, 1)
    FIELD(IOMUX, MCU_WPU, 3, 1)
    FIELD(IOMUX, MCU_IE, 4, 1)
    FIELD(IOMUX, MCU_DRV, 5, 2)
    FIELD(IOMUX, FUN_WPD, 7, 1)
    FIELD(IOMUX, FUN_WPU, 8, 1)
    FIELD(IOMUX, FUN_IE, 9, 1)
    FIELD(IOMUX, FUN_DRV, 10, 2)
    FIELD(IOMUX, MCU_SEL, 12, 3)
    FIELD(IOMUX, FILTER_EN, 15, 1)
    FIELD(IOMUX, I3C_RUE_EN, 16, 1)
    FIELD(IOMUX, I3C_RU, 17, 2)
REG32(SDM, 0)
    FIELD(SDM, IN, 0, 8)
    FIELD(SDM, PRESCALE, 8, 8)
REG32(SDM_MISC, 0)
    FIELD(SDM_MISC, FUNCTION_CLK_EN, 30, 1)
    FIELD(SDM_MISC, SPI_SWAP, 31, 1)
REG32(GLITCH, 0)
    FIELD(GLITCH, EN, 0, 1)
    FIELD(GLITCH, INPUT_IO_NUM, 1, 6)
    FIELD(GLITCH, WINDOW_THRES, 7, 6)
    FIELD(GLITCH, WINDOW_WIDTH, 13, 6)
REG32(ETM_EVENT, 0)
    FIELD(ETM_EVENT, SEL, 0, 6)
    FIELD(ETM_EVENT, EN, 7, 1)
REG32(LP_PIN, 0)
    FIELD(LP_PIN, WAKEUP_ENABLE, 0, 1)
    FIELD(LP_PIN, INT_TYPE, 1, 3)
    FIELD(LP_PIN, PAD_DRIVER, 4, 1)
    FIELD(LP_PIN, EDGE_WAKEUP_CLR, 5, 1)
REG32(LP_FUNC_IN, 0)
    FIELD(LP_FUNC_IN, IN_INV_SEL, 0, 1)
    FIELD(LP_FUNC_IN, SIG_IN_SEL, 1, 1)
    FIELD(LP_FUNC_IN, IN_SEL, 2, 6)
REG32(LP_FUNC_OUT, 0)
    FIELD(LP_FUNC_OUT, OE_INV_SEL, 0, 1)
    FIELD(LP_FUNC_OUT, OE_SEL, 1, 1)
    FIELD(LP_FUNC_OUT, OUT_INV_SEL, 2, 1)
    FIELD(LP_FUNC_OUT, OUT_SEL, 3, 6)
REG32(LP_PAD, 0)
    FIELD(LP_PAD, DRV, 0, 2)
    FIELD(LP_PAD, RDE, 2, 1)
    FIELD(LP_PAD, RUE, 3, 1)
    FIELD(LP_PAD, MUX_SEL, 4, 1)
    FIELD(LP_PAD, FUN_SEL, 5, 2)
    FIELD(LP_PAD, SLP_SEL, 7, 1)
    FIELD(LP_PAD, SLP_IE, 8, 1)
    FIELD(LP_PAD, SLP_OE, 9, 1)
    FIELD(LP_PAD, FUN_IE, 10, 1)
    FIELD(LP_PAD, FILTER_EN, 11, 1)

/* The writable bits and reset values the headers give. */
#define PIN_WRITABLE        0x3ff9f
#define FUNC_IN_WRITABLE    0xff
#define FUNC_OUT_WRITABLE   0xfff
#define FUNC_OUT_RESET      ESP32P4_SIG_GPIO_OUT
#define HIGH_WORD_MASK      0x1ffffff
#define IOMUX_WRITABLE      0x7ffff
#define IOMUX_DRV_RESET     2
#define SDM_WRITABLE        0xffff
#define SDM_RESET           0xff00
#define SDM_MISC_WRITABLE   0xc0000000
#define GLITCH_WRITABLE     0x7ffff
#define ETM_EVENT_WRITABLE  0xbf
#define ETM_TASK_WRITABLE   0x0f0f0f0f
#define INT_BITS            0xff
/*
 * GPIO_INT_RAW's bits for comparator n's rising, falling and either edge
 * of COMP_OUT; esp32p4_gpio_set_ana_comp() has why the rising edge's is
 * bit 3n, which the TRM names COMP<n>_NEG_INT.
 */
#define INT_COMP_RISE(n)    BIT(3 * (n))
#define INT_COMP_FALL(n)    BIT(3 * (n) + 1)
#define INT_COMP_ANY(n)     BIT(3 * (n) + 2)
#define BISTIN_SEL_WRITABLE 0xf
#define BISTIN_SEL_RESET    0xf
#define BIST_PAD_OE         BIT(0)
#define BIST_START          BIT(1)
#define BIST_MODE           BIT(2)
#define SEND_SEQ_RESET      0x12345678
#define DATE_MASK           0x0fffffff
#define GPIO_DATE_RESET     0x241226
#define IOMUX_DATE_RESET    0x201222
#define EXT_VERSION_RESET   0x2203050
#define LP_VER_DATE_RESET   0x230323
#define LPIO_VER_DATE_RESET 0x230313
#define LP_MASK             0xffff
#define LP_PIN_WRITABLE     0x1f
#define LP_FUNC_IN_WRITABLE 0xff
#define LP_FUNC_OUT_WRITABLE 0x1ff
#define LP_FUNC_OUT_RESET   (ESP32P4_LP_SIG_GPIO_OUT << 3)
#define LP_PAD_WRITABLE     0xfff
#define LP_EXT_WAKEUP0_WRITABLE 0x3ff

/* GPIO_FUNCn_IN_SEL values that are not pads */
#define IN_SEL_CONST_LOW    62
#define IN_SEL_CONST_HIGH   63
/* LP_GPIO_FUNCn_IN_SEL: 32 to 47 a constant low, 48 to 63 high */
#define LP_IN_SEL_CONST     0x20
#define LP_IN_SEL_HIGH      0x10

/* Matrix output signals the matrix drives itself */
#define SIG_SDM0            ESP32P4_SIG_GPIO_SD0_OUT
#define SIG_CONST0          ESP32P4_SIG_CONSTANT0_PAD_OUT
#define SIG_CONST1          ESP32P4_SIG_CONSTANT1_PAD_OUT
#define SIG_ANA_COMP0       ESP32P4_SIG_ANA_COMP0_OUT
/* Output signal n of these is input signal n (TRM 10.5.1) */
#define SIG_LOOPBACK_FIRST  250
#define SIG_LOOPBACK_LAST   255

/* LP signals the LP IO MUX carries: the LP UART's, on GPIO14 and GPIO15 */
#define LP_SIG_UART_TXD     ESP32P4_LP_SIG_UART_TXD_PAD_OUT
#define LP_SIG_UART_RXD     ESP32P4_LP_SIG_UART_RXD_PAD_IN
#define LP_PAD_UART_TXD     14
#define LP_PAD_UART_RXD     15

#define PAD_MASK            MAKE_64BIT_MASK(0, ESP32P4_GPIO_PADS)
#define LP_PAD_MASK         MAKE_64BIT_MASK(0, ESP32P4_GPIO_LP_PADS)

/* GPIO_PINn_INT_TYPE and LP_GPIO_PINn_INT_TYPE */
enum {
    INT_TYPE_DISABLED,
    INT_TYPE_RISING,
    INT_TYPE_FALLING,
    INT_TYPE_ANY_EDGE,
    INT_TYPE_LOW,
    INT_TYPE_HIGH,
};

/*
 * GPIO_PINn_INT_ENA bits feeding GPIO_INTR0 to _3: bits 13, 14, 16 and
 * 17 of the register (TRM register 10.28 and table 10.18-1).
 */
static const uint8_t int_ena_bit[4] = { 0, 1, 3, 4 };

/*
 * The input signals with a GPIO_FUNCn_IN_SEL_CFG register: all but those
 * the v3 register header leaves out, which have no input.
 */
static bool sig_has_input(unsigned sig)
{
    switch (sig) {
    case 0:
    case 46:
    case 67:
    case 72 ... 73:
    case 79:
    case 81 ... 82:
    case 84 ... 85:
    case 87 ... 88:
    case 115 ... 116:
    case 119 ... 125:
    case 138 ... 139:
    case 157:
    case 204 ... 213:
        return false;
    default:
        return sig < ESP32P4_GPIO_SIGS;
    }
}

/*
 * The "Default Value" column of TRM table 10.12-1: the level an input
 * signal sees when it bypasses the matrix and no pad's IO MUX function
 * carries it, which is also the constant its GPIO_FUNCn_IN_SEL selects
 * out of reset.
 */
static bool sig_in_default(unsigned sig)
{
    switch (sig) {
    case 1 ... 9:       /* SD card CMD and DATA0-7 */
    case 68 ... 71:     /* I2C0 and I2C1 SCL and SDA */
    case 80:            /* TWAI0 RX */
    case 83:            /* TWAI1 RX */
    case 86:            /* TWAI2 RX */
    case 128 ... 129:   /* SD card INT_N */
    case 134 ... 137:   /* I3C master and slave SCL and SDA */
        return true;
    default:
        return false;
    }
}

/*
 * Output signals of peripherals that hold them high out of reset, which
 * start high so that a peripheral not yet driving the line does not pull
 * its pad low: the UARTs' TXD, RTS and DTR, the SPI chip selects, the
 * I2C and I3C lines, the TWAIs' TX (recessive), the LCD's CS and
 * PARLIO's TX CS.
 */
static bool sig_out_idle(unsigned sig)
{
    switch (sig) {
    case 10 ... 24:     /* UART0-4 TXD, RTS, DTR */
    case 45 ... 46:     /* SPI3 CS2, CS1 */
    case 52:            /* SPI3 CS */
    case 62 ... 67:     /* SPI2 CS, CS1-5 */
    case 68 ... 71:     /* I2C0 and I2C1 SCL and SDA */
    case 80:            /* TWAI0 TX */
    case 83:            /* TWAI1 TX */
    case 86:            /* TWAI2 TX */
    case 134 ... 137:   /* I3C master and slave SCL and SDA */
    case 144:           /* LCD CS */
    case 242:           /* PARLIO TX CS */
        return true;
    default:
        return false;
    }
}

/* Table 10.13-1's defaults: LP I2C SCL and SDA, LP UART CTSN and DSRN. */
static bool lp_sig_in_default(unsigned sig)
{
    return sig == 0 || sig == 1 || sig == 3 || sig == 4;
}

/* LP I2C SCL and SDA, LP UART TXD, RTSN and DTRN, and LP SPI CS */
static bool lp_sig_out_idle(unsigned sig)
{
    return sig <= 4 || sig == 6;
}

/*
 * HP IO MUX functions (TRM table 10.14-1 and ESP-IDF's io_mux_reg.h,
 * which adds function 4, the debug pads). Function 1, and function 0
 * where the table names it GPIOn, is the GPIO matrix. A function that
 * carries a matrix signal connects the pad to that signal directly
 * ("Direct Input/Output via HP IO MUX" in table 10.12-1); the other
 * peripheral functions have lines of their own.
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

/* Pads whose functions 0 and 1 are both the GPIO matrix and no more */
#define F_PLAIN     { F_GPIO, F_GPIO, F_NONE, F_NONE, F_NONE }

static const IomuxFunc iomux_funcs[ESP32P4_GPIO_PADS]
                                  [ESP32P4_IOMUX_FUNCS] = {
    [0] = F_PLAIN,
    [1] = F_PLAIN,
    /* MTCK, MTDI, MTMS, MTDO */
    [2] = { F_DIRECT, F_GPIO, F_NONE, F_NONE, F_NONE },
    [3] = { F_DIRECT, F_GPIO, F_NONE, F_NONE, F_NONE },
    [4] = { F_DIRECT, F_GPIO, F_NONE, F_NONE, F_NONE },
    [5] = { F_DIRECT, F_GPIO, F_NONE, F_NONE, F_NONE },
    /* -, SPI2_HOLD */
    [6] = { F_GPIO, F_GPIO, F_NONE, F_IO(56), F_NONE },
    /* -, SPI2_CS */
    [7] = { F_GPIO, F_GPIO, F_NONE, F_IO(62), F_NONE },
    /* UART0_RTS, SPI2_D */
    [8] = { F_GPIO, F_GPIO, F_OUT(11), F_IO(55), F_NONE },
    /* UART0_CTS, SPI2_CK */
    [9] = { F_GPIO, F_GPIO, F_IN(11), F_IO(53), F_NONE },
    /* UART1_TXD, SPI2_Q */
    [10] = { F_GPIO, F_GPIO, F_OUT(13), F_IO(54), F_NONE },
    /* UART1_RXD, SPI2_WP */
    [11] = { F_GPIO, F_GPIO, F_IN(13), F_IO(57), F_NONE },
    /* UART1_RTS */
    [12] = { F_GPIO, F_GPIO, F_OUT(14), F_NONE, F_NONE },
    /* UART1_CTS */
    [13] = { F_GPIO, F_GPIO, F_IN(14), F_NONE, F_NONE },
    [14 ... 21] = F_PLAIN,
    /* DBG_PSRAM_CK */
    [22] = { F_GPIO, F_GPIO, F_NONE, F_NONE, F_DIRECT },
    /* REF_50M_CLK, DBG_PSRAM_CS */
    [23] = { F_GPIO, F_GPIO, F_NONE, F_DIRECT, F_DIRECT },
    [24 ... 27] = F_PLAIN,
    /* SPI2_CS, EMAC_PHY_RXDV, DBG_PSRAM_D */
    [28] = { F_GPIO, F_GPIO, F_IO(62), F_IN(178), F_DIRECT },
    /* SPI2_D, EMAC_PHY_RXD0, DBG_PSRAM_Q */
    [29] = { F_GPIO, F_GPIO, F_IO(55), F_IN(179), F_DIRECT },
    /* SPI2_CK, EMAC_PHY_RXD1, DBG_PSRAM_WP */
    [30] = { F_GPIO, F_GPIO, F_IO(53), F_IN(180), F_DIRECT },
    /* SPI2_Q, EMAC_PHY_RXER, DBG_PSRAM_HOLD */
    [31] = { F_GPIO, F_GPIO, F_IO(54), F_IN(183), F_DIRECT },
    /* SPI2_HOLD, EMAC_RMII_CLK, DBG_PSRAM_DQ4 */
    [32] = { F_GPIO, F_GPIO, F_IO(56), F_DIRECT, F_DIRECT },
    /* SPI2_WP, EMAC_PHY_TXEN, DBG_PSRAM_DQ5 */
    [33] = { F_GPIO, F_GPIO, F_IO(57), F_OUT(178), F_DIRECT },
    /* SPI2_IO4, EMAC_PHY_TXD0, DBG_PSRAM_DQ6 */
    [34] = { F_GPIO, F_GPIO, F_IO(58), F_OUT(179), F_DIRECT },
    /* SPI2_IO5, EMAC_PHY_TXD1, DBG_PSRAM_DQ7 */
    [35] = { F_GPIO, F_GPIO, F_IO(59), F_OUT(180), F_DIRECT },
    /* SPI2_IO6, EMAC_PHY_TXER, DBG_PSRAM_DQS_0 */
    [36] = { F_GPIO, F_GPIO, F_IO(60), F_OUT(183), F_DIRECT },
    /* UART0_TXD, GPIO37, SPI2_IO7 */
    [37] = { F_OUT(10), F_GPIO, F_IO(61), F_NONE, F_NONE },
    /* UART0_RXD, GPIO38, SPI2_DQS */
    [38] = { F_IN(10), F_GPIO, F_OUT(44), F_NONE, F_NONE },
    /* SD1_CDATA0, GPIO39, BIST, REF_50M_CLK, DBG_PSRAM_DQ8 */
    [39] = { F_DIRECT, F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* SD1_CDATA1, GPIO40, BIST, EMAC_PHY_TXEN, DBG_PSRAM_DQ9 */
    [40] = { F_DIRECT, F_GPIO, F_DIRECT, F_OUT(178), F_DIRECT },
    /* SD1_CDATA2, GPIO41, BIST, EMAC_PHY_TXD0, DBG_PSRAM_DQ10 */
    [41] = { F_DIRECT, F_GPIO, F_DIRECT, F_OUT(179), F_DIRECT },
    /* SD1_CDATA3, GPIO42, BIST, EMAC_PHY_TXD1, DBG_PSRAM_DQ11 */
    [42] = { F_DIRECT, F_GPIO, F_DIRECT, F_OUT(180), F_DIRECT },
    /* SD1_CCLK, GPIO43, BIST, EMAC_PHY_TXER, DBG_PSRAM_DQ12 */
    [43] = { F_DIRECT, F_GPIO, F_DIRECT, F_OUT(183), F_DIRECT },
    /* SD1_CCMD, GPIO44, BIST, EMAC_RMII_CLK, DBG_PSRAM_DQ13 */
    [44] = { F_DIRECT, F_GPIO, F_DIRECT, F_DIRECT, F_DIRECT },
    /* SD1_CDATA4, GPIO45, BIST, EMAC_PHY_RXDV, DBG_PSRAM_DQ14 */
    [45] = { F_DIRECT, F_GPIO, F_DIRECT, F_IN(178), F_DIRECT },
    /* SD1_CDATA5, GPIO46, BIST, EMAC_PHY_RXD0, DBG_PSRAM_DQ15 */
    [46] = { F_DIRECT, F_GPIO, F_DIRECT, F_IN(179), F_DIRECT },
    /* SD1_CDATA6, GPIO47, BIST, EMAC_PHY_RXD1, DBG_PSRAM_DQS_1 */
    [47] = { F_DIRECT, F_GPIO, F_DIRECT, F_IN(180), F_DIRECT },
    /* SD1_CDATA7, GPIO48, BIST, EMAC_PHY_RXER */
    [48] = { F_DIRECT, F_GPIO, F_DIRECT, F_IN(183), F_NONE },
    /* -, EMAC_PHY_TXEN, DBG_FLASH_CS */
    [49] = { F_GPIO, F_GPIO, F_NONE, F_OUT(178), F_DIRECT },
    /* -, EMAC_RMII_CLK, DBG_FLASH_Q */
    [50] = { F_GPIO, F_GPIO, F_NONE, F_DIRECT, F_DIRECT },
    /* -, EMAC_PHY_RXDV, DBG_FLASH_WP */
    [51] = { F_GPIO, F_GPIO, F_NONE, F_IN(178), F_DIRECT },
    /* -, EMAC_PHY_RXD0, DBG_FLASH_HOLD */
    [52] = { F_GPIO, F_GPIO, F_NONE, F_IN(179), F_DIRECT },
    /* -, EMAC_PHY_RXD1, DBG_FLASH_CK */
    [53] = { F_GPIO, F_GPIO, F_NONE, F_IN(180), F_DIRECT },
    /* -, EMAC_PHY_RXER, DBG_FLASH_D */
    [54] = { F_GPIO, F_GPIO, F_NONE, F_IN(183), F_DIRECT },
};

static const IomuxFunc iomux_none = F_NONE;

/*
 * The "Reset" and "DRV" columns of table 10.14-1: GPIO2 (MTCK) with its
 * input enabled and pulled up, as on a chip with EFUSE_DIS_PAD_JTAG
 * clear; GPIO3 and GPIO4 (MTDI, MTMS) with their inputs enabled; GPIO24
 * and GPIO25, the USB Serial/JTAG PHY's pads, at drive strength 3, GPIO25
 * pulled up by the PHY rather than by its IO MUX register. Every other
 * pad starts at drive strength 2 with its input disabled.
 */
static uint32_t iomux_reset_value(unsigned n)
{
    uint32_t v = FIELD_DP32(0, IOMUX, FUN_DRV, IOMUX_DRV_RESET);

    switch (n) {
    case 2:
        v = FIELD_DP32(v, IOMUX, FUN_WPU, 1);
        /* fall through */
    case 3:
    case 4:
        v = FIELD_DP32(v, IOMUX, FUN_IE, 1);
        break;
    case 24:
    case 25:
        v = FIELD_DP32(v, IOMUX, FUN_DRV, 3);
        break;
    default:
        break;
    }
    return v;
}

static inline bool bit64(uint64_t v, unsigned n)
{
    return (v >> n) & 1;
}

static uint64_t gpio_out_all(ESP32P4GpioState *s)
{
    return s->out[0] | ((uint64_t)s->out[1] << 32);
}

static uint64_t gpio_enable_all(ESP32P4GpioState *s)
{
    return s->enable[0] | ((uint64_t)s->enable[1] << 32);
}

static uint64_t gpio_status_all(ESP32P4GpioState *s)
{
    return s->status[0] | ((uint64_t)s->status[1] << 32);
}

/* Whether the LP IO MUX has pad n (LP_IOMUX_PADn_MUX_SEL). */
static bool lp_owned(ESP32P4GpioState *s, unsigned n)
{
    return n < ESP32P4_GPIO_LP_PADS &&
           FIELD_EX32(s->lp_pad[n], LP_PAD, MUX_SEL);
}

static const IomuxFunc *hp_func(ESP32P4GpioState *s, unsigned n,
                                unsigned *func)
{
    unsigned f = FIELD_EX32(s->iomux[n], IOMUX, MCU_SEL);

    *func = f;
    if (n >= ESP32P4_GPIO_PADS || f >= ESP32P4_IOMUX_FUNCS) {
        return &iomux_none;
    }
    return &iomux_funcs[n][f];
}

/* Whether a pad's LP IO MUX function is the LP GPIO matrix. */
static bool lp_func_is_matrix(ESP32P4GpioState *s, unsigned n)
{
    unsigned f = FIELD_EX32(s->lp_pad[n], LP_PAD, FUN_SEL);

    return f == 1 || (f == 0 && n < LP_PAD_UART_TXD);
}

/* The level of HP output signal sig as the matrix sees it. */
static bool sig_out_level(ESP32P4GpioState *s, unsigned sig)
{
    if (sig >= SIG_SDM0 && sig < SIG_SDM0 + ESP32P4_GPIO_SDMS) {
        return s->sdm_out[sig - SIG_SDM0];
    }
    if (sig >= SIG_ANA_COMP0 && sig < SIG_ANA_COMP0 + ESP32P4_GPIO_ANA_COMPS) {
        return s->ana_comp[sig - SIG_ANA_COMP0];
    }
    switch (sig) {
    case SIG_CONST0:
        return false;
    case SIG_CONST1:
        return true;
    default:
        return s->sig_out[sig];
    }
}

/* The output enable of HP output signal sig: 1'd1 for the matrix's own. */
static bool sig_oe_level(ESP32P4GpioState *s, unsigned sig)
{
    if ((sig >= SIG_SDM0 && sig < SIG_SDM0 + ESP32P4_GPIO_SDMS) ||
        (sig >= SIG_ANA_COMP0 &&
         sig < SIG_ANA_COMP0 + ESP32P4_GPIO_ANA_COMPS) ||
        sig == SIG_CONST0 || sig == SIG_CONST1 ||
        sig >= SIG_LOOPBACK_FIRST) {
        return true;
    }
    return s->sig_oe[sig];
}

/*
 * What controls a pad: its output driver's level and enable, its pulls
 * and its input enable.
 */
typedef struct PadCtl {
    bool val;
    bool oe;
    bool od;
    bool pu;
    bool ie;
} PadCtl;

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The LP IO MUX's drive of LP pad n: through the LP GPIO matrix
 * LP_GPIO_FUNCn_OUT_SEL picks an LP output signal, or LP_GPIO_OUT and
 * LP_GPIO_ENABLE for 32, the inversions and the enable's source as in the
 * HP matrix; in function 0 GPIO14 is the LP UART's TXD.
 */
static int lp_pad_drive(ESP32P4GpioState *s, unsigned n, bool *val_out,
                        bool *oe_out)
{
    uint32_t cfg = s->lp_func_out_sel[n];
    unsigned sel = FIELD_EX32(cfg, LP_FUNC_OUT, OUT_SEL);
    bool val = false, oe_p = false, oe = false;

    if (lp_func_is_matrix(s, n)) {
        if (sel == ESP32P4_LP_SIG_GPIO_OUT) {
            val = bit64(s->lp_out, n);
            oe_p = bit64(s->lp_enable, n);
        } else if (sel < ESP32P4_GPIO_LP_SIGS) {
            val = s->lp_sig_out[sel];
            oe_p = s->lp_sig_oe[sel];
        }
        val ^= FIELD_EX32(cfg, LP_FUNC_OUT, OUT_INV_SEL);
        oe = FIELD_EX32(cfg, LP_FUNC_OUT, OE_SEL) ?
             bit64(s->lp_enable, n) : oe_p;
        oe ^= FIELD_EX32(cfg, LP_FUNC_OUT, OE_INV_SEL);
    } else if (n == LP_PAD_UART_TXD &&
               FIELD_EX32(s->lp_pad[n], LP_PAD, FUN_SEL) == 0) {
        val = s->lp_sig_out[LP_SIG_UART_TXD];
        oe = s->lp_sig_oe[LP_SIG_UART_TXD];
    }
    *val_out = val;
    *oe_out = oe;
    return -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Drive of pad n and the HP matrix signal it carries (-1 for none). In
 * the HP GPIO matrix function GPIO_FUNCn_OUT_SEL picks an output signal,
 * or GPIO_OUT and GPIO_ENABLE for 256; OUT_INV_SEL inverts the level, and
 * OEN_SEL takes the enable from GPIO_ENABLE rather than the peripheral,
 * OEN_INV_SEL inverting it. In an IO MUX function the peripheral drives
 * the pad itself, enable and all (ESP-IDF's gpio_ll: "otherwise the oe
 * can only be controlled by peripheral").
 */
static int pad_drive(ESP32P4GpioState *s, unsigned n, bool *val_out,
                     bool *oe_out)
{
    bool val = false, oe_p = false, oe = false;
    uint32_t cfg;
    const IomuxFunc *fn;
    unsigned f, sel;
    int sig = -1;

    *val_out = false;
    *oe_out = false;
    if (n >= ESP32P4_GPIO_PADS) {
        return -1;
    }
    if (lp_owned(s, n)) {
        return lp_pad_drive(s, n, val_out, oe_out);
    }

    fn = hp_func(s, n, &f);
    cfg = s->func_out_sel[n];
    switch (fn->kind) {
    case IOMUX_GPIO:
        sel = FIELD_EX32(cfg, FUNC_OUT, OUT_SEL);
        if (sel == ESP32P4_SIG_GPIO_OUT) {
            val = bit64(gpio_out_all(s), n);
            oe_p = bit64(gpio_enable_all(s), n);
        } else if (sel < ESP32P4_GPIO_SIGS) {
            val = sig_out_level(s, sel);
            oe_p = sig_oe_level(s, sel);
            sig = sel;
        }
        val ^= FIELD_EX32(cfg, FUNC_OUT, OUT_INV_SEL);
        oe = FIELD_EX32(cfg, FUNC_OUT, OEN_SEL) ?
             bit64(gpio_enable_all(s), n) : oe_p;
        oe ^= FIELD_EX32(cfg, FUNC_OUT, OEN_INV_SEL);
        break;
    case IOMUX_SIG:
        if (fn->sig_out >= 0) {
            val = sig_out_level(s, fn->sig_out);
            oe = sig_oe_level(s, fn->sig_out);
            sig = fn->sig_out;
        }
        break;
    case IOMUX_DIRECT:
        val = s->iomux_func_out[ESP32P4_GPIO_IOMUX_LINE(n, f)];
        oe = s->iomux_func_oe[ESP32P4_GPIO_IOMUX_LINE(n, f)];
        break;
    default:
        break;
    }
    *val_out = val;
    *oe_out = oe;
    return sig;
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Everything that controls pad n but its hold, with the sleep
 * configuration of TRM table 10.8-1 in light sleep: IO_MUX_GPIOn_MCU_*
 * (or LP_IOMUX_PADn_SLP_IE and _OE) in place of the normal input enable,
 * output enable and pulls, for a pad with SLP_SEL set.
 */
static void pad_ctl(ESP32P4GpioState *s, unsigned n, PadCtl *c)
{
    uint32_t mux = s->iomux[n];
    bool i3c_pu;

    pad_drive(s, n, &c->val, &c->oe);
    if (lp_owned(s, n)) {
        uint32_t pad = s->lp_pad[n];

        c->od = FIELD_EX32(s->lp_pin[n], LP_PIN, PAD_DRIVER);
        c->pu = FIELD_EX32(pad, LP_PAD, RUE) || bit64(s->ext_pull_up, n);
        c->ie = FIELD_EX32(pad, LP_PAD, FUN_IE);
        if (s->light_sleep && FIELD_EX32(pad, LP_PAD, SLP_SEL)) {
            c->ie = FIELD_EX32(pad, LP_PAD, SLP_IE);
            c->oe = FIELD_EX32(pad, LP_PAD, SLP_OE);
        }
        return;
    }

    i3c_pu = (n == 32 || n == 33) && FIELD_EX32(mux, IOMUX, I3C_RUE_EN);
    c->od = FIELD_EX32(s->pin[n], PIN, PAD_DRIVER);
    c->pu = FIELD_EX32(mux, IOMUX, FUN_WPU) || i3c_pu ||
            bit64(s->ext_pull_up, n);
    c->ie = FIELD_EX32(mux, IOMUX, FUN_IE);
    if (s->light_sleep && FIELD_EX32(mux, IOMUX, SLP_SEL)) {
        c->pu = FIELD_EX32(mux, IOMUX, MCU_WPU) || i3c_pu ||
                bit64(s->ext_pull_up, n);
        c->ie = FIELD_EX32(mux, IOMUX, MCU_IE);
        c->oe = FIELD_EX32(mux, IOMUX, MCU_OE);
    }
}

/* Whether pad n's own driver drives it, and *level at what. */
static bool pad_driven_by_chip(ESP32P4GpioState *s, unsigned n, bool *level)
{
    PadCtl c;

    if (bit64(s->held, n)) {
        *level = bit64(s->hold_val, n);
        return bit64(s->hold_oe, n);
    }
    pad_ctl(s, n, &c);
    *level = c.val;
    /* Open drain: a high output releases the pad. */
    return c.oe && !(c.od && c.val);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Resolve every pad's level and input buffer. A pad that has just been
 * held latches what it has; a held pad keeps that whatever its registers
 * and signals do (TRM 10.9). Pads the board wires together ("pad-wires")
 * share one level: low while any of them is driven low, from outside or
 * by its own driver; else high while any is driven high or pulled up.
 */
static void resolve(ESP32P4GpioState *s, uint64_t *pad_out,
                    uint64_t *raw_out, uint64_t *drv_high_out,
                    uint64_t *drv_low_out)
{
    uint64_t pad = 0, raw = 0, ie_mask = 0;
    uint64_t drv_low = 0, drv_high = 0, pulled = 0;
    uint64_t held = (s->ext_hold | s->lp_pad_hold) & PAD_MASK;

    if (s->hold_all[ESP32P4_GPIO_HOLD_ALL_HP]) {
        held |= PAD_MASK & ~LP_PAD_MASK;
    }
    if (s->hold_all[ESP32P4_GPIO_HOLD_ALL_LP]) {
        held |= LP_PAD_MASK;
    }

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        bool drive, val, pu, ie;

        if (bit64(held, n) && !bit64(s->held, n)) {
            PadCtl c;

            pad_ctl(s, n, &c);
            s->hold_val = deposit64(s->hold_val, n, 1, c.val);
            s->hold_oe = deposit64(s->hold_oe, n, 1,
                                   c.oe && !(c.od && c.val));
            s->hold_pu = deposit64(s->hold_pu, n, 1, c.pu);
            s->hold_ie = deposit64(s->hold_ie, n, 1, c.ie);
        }
        if (bit64(held, n)) {
            val = bit64(s->hold_val, n);
            drive = bit64(s->hold_oe, n);
            pu = bit64(s->hold_pu, n);
            ie = bit64(s->hold_ie, n);
        } else {
            PadCtl c;

            pad_ctl(s, n, &c);
            val = c.val;
            drive = c.oe && !(c.od && c.val);
            pu = c.pu;
            ie = c.ie;
        }

        if (bit64(s->ext_driven, n)) {
            drive = true;
            val = bit64(s->ext_level, n);
        }
        if (drive) {
            if (val) {
                drv_high |= 1ULL << n;
            } else {
                drv_low |= 1ULL << n;
            }
        } else if (pu) {
            pulled |= 1ULL << n;
        }
        if (ie) {
            ie_mask |= 1ULL << n;
        }
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        uint64_t net = 1ULL << n;
        bool level;

        if (s->pad_wire[n]) {
            net = 0;
            for (unsigned m = 0; m < ESP32P4_GPIO_PADS; m++) {
                if (s->pad_wire[m] == s->pad_wire[n]) {
                    net |= 1ULL << m;
                }
            }
        }
        /*
         * An outside driver wins over the pad's own; on a wire, a low
         * driver wins. Undriven, a pull-up holds the pad high; pulled
         * down or floating, it reads low.
         */
        if (!s->pad_wire[n] && bit64(s->ext_driven, n)) {
            level = bit64(s->ext_level, n);
        } else if (drv_low & net) {
            level = false;
        } else {
            level = (drv_high | pulled) & net;
        }
        pad |= (uint64_t)level << n;
    }
    raw = pad & ie_mask;
    s->held = held;
    *pad_out = pad;
    *raw_out = raw;
    *drv_high_out = drv_high;
    *drv_low_out = drv_low;
}

/* Filters */

/* A filter's window, and the samples that must differ to change it. */
typedef struct FilterParams {
    unsigned width;
    unsigned thres;
} FilterParams;

/* The pin filters: two cycles at the new level. */
static const FilterParams pin_filter_params = { 1, 1 };

static FilterParams glitch_params(ESP32P4GpioState *s, unsigned ch)
{
    FilterParams p = {
        FIELD_EX32(s->glitch[ch], GLITCH, WINDOW_WIDTH),
        FIELD_EX32(s->glitch[ch], GLITCH, WINDOW_THRES),
    };

    return p;
}

/* The pad a glitch filter channel filters, or -1. */
static int glitch_pad(ESP32P4GpioState *s, unsigned ch)
{
    unsigned n = FIELD_EX32(s->glitch[ch], GLITCH, INPUT_IO_NUM);

    if (!FIELD_EX32(s->glitch[ch], GLITCH, EN) || n >= ESP32P4_GPIO_PADS) {
        return -1;
    }
    return n;
}

/* The enabled glitch filter channel of pad n, the lowest-numbered. */
static int pad_glitch(ESP32P4GpioState *s, unsigned n)
{
    for (unsigned ch = 0; ch < ESP32P4_GPIO_GLITCH_FILTERS; ch++) {
        if (glitch_pad(s, ch) == (int)n) {
            return ch;
        }
    }
    return -1;
}

/* Cycles of a clock from its anchor to t. */
static uint64_t clk_cycles(ESP32P4ClkCount *edges, int64_t t)
{
    return esp32p4_clk_count_at(edges, t);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Take one sample: record whether it differs from the output, and once
 * thres + 1 of the last width + 1 samples have, take the input's level.
 */
static bool filter_sample(ESP32P4GpioFilter *f, FilterParams p)
{
    uint64_t mask = MAKE_64BIT_MASK(0, p.width + 1);

    f->window = ((f->window << 1) | (f->in != f->out)) & mask;
    if (ctpop64(f->window) >= p.thres + 1) {
        f->out = f->in;
        f->window = 0;
        return true;
    }
    return false;
}

/*
 * Take the samples up to cycle k, the input having been constant since
 * the last. After width + 1 samples of a constant input the filter has
 * settled: its output has taken the input's level, or, with a threshold
 * above its width, never will; so it needs no more than twice that.
 */
static void filter_advance(ESP32P4GpioFilter *f, FilterParams p, uint64_t k)
{
    uint64_t n;

    if (k <= f->tick) {
        return;
    }
    n = MIN(k - f->tick, 2 * (uint64_t)(p.width + 2));
    while (n--) {
        filter_sample(f, p);
    }
    f->tick = k;
}

/* Start a filter at its input's level, with nothing sampled. */
static void filter_start(ESP32P4GpioFilter *f, bool level, uint64_t k)
{
    f->in = level;
    f->out = level;
    f->window = 0;
    f->tick = k;
}

/* Feed a filter its input as it changes at cycle k. */
static void filter_input(ESP32P4GpioFilter *f, FilterParams p, bool level,
                         uint64_t k)
{
    if (level == f->in) {
        return;
    }
    filter_advance(f, p, k);
    f->in = level;
}

/*
 * The cycle at which the filter's output will change if its input holds,
 * or 0 if it will not.
 */
static uint64_t filter_due(ESP32P4GpioFilter *f, FilterParams p)
{
    ESP32P4GpioFilter t = *f;

    if (f->in == f->out) {
        return 0;
    }
    for (unsigned i = 1; i <= p.width + 1; i++) {
        if (filter_sample(&t, p)) {
            return f->tick + i;
        }
    }
    return 0;
}

static bool hp_filter_on(ESP32P4GpioState *s, unsigned n)
{
    return FIELD_EX32(s->iomux[n], IOMUX, FILTER_EN);
}

static bool lp_filter_on(ESP32P4GpioState *s, unsigned n)
{
    return FIELD_EX32(s->lp_pad[n], LP_PAD, FILTER_EN);
}

/* Take every enabled filter's samples up to time t. */
static void filters_advance(ESP32P4GpioState *s, int64_t t)
{
    uint64_t k = clk_cycles(&s->iomux_edges, t);
    uint64_t lk = clk_cycles(&s->lp_edges, t);

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        if (hp_filter_on(s, n)) {
            filter_advance(&s->pin_filter[n], pin_filter_params, k);
        }
    }
    for (unsigned ch = 0; ch < ESP32P4_GPIO_GLITCH_FILTERS; ch++) {
        if (glitch_pad(s, ch) >= 0) {
            filter_advance(&s->glitch_filter[ch], glitch_params(s, ch), k);
        }
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        if (lp_filter_on(s, n)) {
            filter_advance(&s->lp_pin_filter[n], pin_filter_params, lk);
        }
    }
}

/* The earlier of *next and when filter f's output change is due. */
static void filter_next(int64_t *next, ESP32P4ClkCount *edges,
                        ESP32P4GpioFilter *f, FilterParams p)
{
    uint64_t k = filter_due(f, p);

    if (k) {
        *next = MIN(*next, esp32p4_clk_count_time(edges, k));
    }
}

/* When the earliest filter output change falls due, or INT64_MAX. */
static int64_t filters_next(ESP32P4GpioState *s)
{
    int64_t next = INT64_MAX;

    if (clock_is_enabled(s->iomux_clk)) {
        for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
            if (hp_filter_on(s, n)) {
                filter_next(&next, &s->iomux_edges, &s->pin_filter[n],
                            pin_filter_params);
            }
        }
        for (unsigned ch = 0; ch < ESP32P4_GPIO_GLITCH_FILTERS; ch++) {
            if (glitch_pad(s, ch) >= 0) {
                filter_next(&next, &s->iomux_edges, &s->glitch_filter[ch],
                            glitch_params(s, ch));
            }
        }
    }
    if (clock_is_enabled(s->lp_clk)) {
        for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
            if (lp_filter_on(s, n)) {
                filter_next(&next, &s->lp_edges, &s->lp_pin_filter[n],
                            pin_filter_params);
            }
        }
    }
    return next;
}

/* Arm the timer for the earliest filter output change due. */
static void filters_schedule(ESP32P4GpioState *s)
{
    int64_t next = filters_next(s);

    if (next == INT64_MAX) {
        timer_del(&s->filter_timer);
    } else {
        timer_mod(&s->filter_timer, next);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Run the input chains on the pads' input buffers at time t: the HP pin
 * filters, then the glitch filters, into the HP chain; the LP pin filters
 * into the LP chain.
 */
static void run_chains(ESP32P4GpioState *s, uint64_t raw, int64_t t,
                       uint64_t *pf_out, uint64_t *hp_out, uint32_t *lp_out)
{
    uint64_t k = clk_cycles(&s->iomux_edges, t);
    uint64_t lk = clk_cycles(&s->lp_edges, t);
    uint64_t pf = 0, hp;
    uint32_t lp = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        bool v = bit64(raw, n);

        if (hp_filter_on(s, n)) {
            filter_input(&s->pin_filter[n], pin_filter_params, v, k);
            v = s->pin_filter[n].out;
        }
        pf |= (uint64_t)v << n;
    }
    hp = pf;
    for (unsigned ch = 0; ch < ESP32P4_GPIO_GLITCH_FILTERS; ch++) {
        int n = glitch_pad(s, ch);

        if (n < 0) {
            continue;
        }
        filter_input(&s->glitch_filter[ch], glitch_params(s, ch),
                     bit64(pf, n), k);
        if (pad_glitch(s, n) == (int)ch) {
            hp = deposit64(hp, n, 1, s->glitch_filter[ch].out);
        }
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        bool v = bit64(raw, n);

        if (lp_filter_on(s, n)) {
            filter_input(&s->lp_pin_filter[n], pin_filter_params, v, lk);
            v = s->lp_pin_filter[n].out;
        }
        lp |= (uint32_t)v << n;
    }
    *pf_out = pf;
    *hp_out = hp;
    *lp_out = lp;
}

/* Sigma-delta modulators */

static bool sdm_clock_on(ESP32P4GpioState *s)
{
    return FIELD_EX32(s->sdm_misc, SDM_MISC, FUNCTION_CLK_EN) &&
           clock_is_enabled(s->iomux_clk);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * On every tick of the IO MUX operating clock divided by PRESCALE + 1 a
 * modulator adds its duty, as an offset binary 0-255 (SDn_IN as a signed
 * -128..127, plus 128), to an 8-bit accumulator, and outputs 1 for the
 * tick when the sum carries: a density of (SDn_IN + 128) / 256. The
 * accumulator advances lazily, by the whole ticks elapsed since the
 * modulator's settings or clock last changed.
 */
static void sdm_advance(ESP32P4GpioState *s, unsigned ch, int64_t now)
{
    uint32_t reg = s->sdm[ch];
    uint32_t div = FIELD_EX32(reg, SDM, PRESCALE) + 1;
    uint32_t d = FIELD_EX32(reg, SDM, IN) ^ 0x80;
    ESP32P4ClkCount edges = s->iomux_edges;
    uint64_t total, n;
    uint32_t a;

    if (!sdm_clock_on(s) || now <= s->sdm_anchor_ns[ch]) {
        return;
    }
    /* The clock's edges since the anchor. */
    esp32p4_clk_count_rebase(&edges, s->sdm_anchor_ns[ch]);
    total = esp32p4_clk_count_at(&edges, now) / div;
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

/* Bring a modulator up to now and count its ticks afresh from here. */
static void sdm_reanchor(ESP32P4GpioState *s, unsigned ch, int64_t now)
{
    sdm_advance(s, ch, now);
    s->sdm_anchor_ns[ch] = now;
    s->sdm_ticks[ch] = 0;
}

static void sdm_reanchor_all(ESP32P4GpioState *s, int64_t now)
{
    for (unsigned ch = 0; ch < ESP32P4_GPIO_SDMS; ch++) {
        sdm_reanchor(s, ch, now);
    }
}

/* Interrupts */

/* GPIO_STATUS bits of the pads whose INT_ENA has bit k */
static uint64_t int_view(ESP32P4GpioState *s, unsigned k)
{
    uint64_t ena = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_SLOTS; n++) {
        if (FIELD_EX32(s->pin[n], PIN, INT_ENA) & BIT(k)) {
            ena |= 1ULL << n;
        }
    }
    return gpio_status_all(s) & ena;
}

/* Whether input level in, as it was prev, makes interrupt type type. */
static bool int_hit(unsigned type, bool prev, bool in)
{
    switch (type) {
    case INT_TYPE_RISING:
        return !prev && in;
    case INT_TYPE_FALLING:
        return prev && !in;
    case INT_TYPE_ANY_EDGE:
        return prev != in;
    case INT_TYPE_LOW:
        return !in;
    case INT_TYPE_HIGH:
        return in;
    default:
        return false;
    }
}

/* The interrupt condition: a level type's level; edges are momentary. */
static bool int_next(unsigned type, bool in)
{
    return (type == INT_TYPE_LOW && !in) || (type == INT_TYPE_HIGH && in);
}

static uint64_t status_next(ESP32P4GpioState *s)
{
    uint64_t v = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        if (int_next(FIELD_EX32(s->pin[n], PIN, INT_TYPE),
                     bit64(s->hp_in, n))) {
            v |= 1ULL << n;
        }
    }
    return v;
}

static uint32_t lp_status_next(ESP32P4GpioState *s)
{
    uint32_t v = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        if (int_next(FIELD_EX32(s->lp_pin[n], LP_PIN, INT_TYPE),
                     bit64(s->lp_in, n))) {
            v |= 1u << n;
        }
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Latch pad interrupts into GPIO_STATUS and LP_GPIO_STATUS from each
 * chain's input between successive resolutions, and the LP edge wakeups
 * of the pads with LP_GPIO_PINn_WAKEUP_ENABLE.
 */
static void latch_interrupts(ESP32P4GpioState *s, uint64_t hp, uint32_t lp)
{
    uint64_t set = 0;
    uint32_t lp_set = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        if (int_hit(FIELD_EX32(s->pin[n], PIN, INT_TYPE),
                    bit64(s->hp_in, n), bit64(hp, n))) {
            set |= 1ULL << n;
        }
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        uint32_t pin = s->lp_pin[n];
        unsigned type = FIELD_EX32(pin, LP_PIN, INT_TYPE);
        bool prev = bit64(s->lp_in, n), in = bit64(lp, n);

        if (int_hit(type, prev, in)) {
            lp_set |= 1u << n;
            if (FIELD_EX32(pin, LP_PIN, WAKEUP_ENABLE) &&
                type >= INT_TYPE_RISING && type <= INT_TYPE_ANY_EDGE) {
                s->lp_wakeup_edge |= 1u << n;
            }
        }
    }
    s->status[0] |= (uint32_t)set;
    s->status[1] |= (uint32_t)(set >> 32);
    s->lp_status |= lp_set;
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The HP GPIO wakeup from light sleep (table 10.4-1), on a level type;
 * and the LP GPIO wakeup (table 10.4-2), on a level type's level or a
 * latched edge.
 */
static bool hp_wakeup(ESP32P4GpioState *s)
{
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        uint32_t pin = s->pin[n];

        if (FIELD_EX32(pin, PIN, WAKEUP_ENABLE) &&
            int_next(FIELD_EX32(pin, PIN, INT_TYPE), bit64(s->hp_in, n))) {
            return true;
        }
    }
    return false;
}

static bool lp_wakeup(ESP32P4GpioState *s)
{
    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        uint32_t pin = s->lp_pin[n];

        if (!FIELD_EX32(pin, LP_PIN, WAKEUP_ENABLE)) {
            continue;
        }
        if (int_next(FIELD_EX32(pin, LP_PIN, INT_TYPE), bit64(s->lp_in, n)) ||
            bit64(s->lp_wakeup_edge, n)) {
            return true;
        }
    }
    return false;
}

/* Input signals */

/*
 * The pad whose HP IO MUX function carries input signal sig directly, the
 * lowest-numbered if more than one does, or -1.
 */
static int iomux_in_pad(ESP32P4GpioState *s, unsigned sig)
{
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        unsigned f;
        const IomuxFunc *fn = hp_func(s, n, &f);

        if (!lp_owned(s, n) && fn->kind == IOMUX_SIG && fn->sig_in == sig) {
            return n;
        }
    }
    return -1;
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The level of HP input signal sig. Through the matrix (SIG_IN_SEL) it is
 * the selected pad's HP chain, or a constant, inverted by IN_INV_SEL.
 * Bypassing the matrix it is the unfiltered input of the pad whose IO MUX
 * function carries sig, or the signal's default when none does.
 */
static bool sig_in_level(ESP32P4GpioState *s, unsigned sig, uint64_t raw,
                         uint64_t hp, const int8_t *iomux_src)
{
    uint32_t cfg = s->func_in_sel[sig];
    unsigned sel;
    bool v;

    if (!sig_has_input(sig)) {
        return false;
    }
    if (!FIELD_EX32(cfg, FUNC_IN, SIG_IN_SEL)) {
        if (iomux_src[sig] >= 0) {
            return bit64(raw, iomux_src[sig]);
        }
        return sig_in_default(sig);
    }
    sel = FIELD_EX32(cfg, FUNC_IN, IN_SEL);
    if (sel < ESP32P4_GPIO_PADS) {
        v = bit64(hp, sel);
    } else {
        v = sel == IN_SEL_CONST_HIGH;
    }
    return v ^ FIELD_EX32(cfg, FUNC_IN, IN_INV_SEL);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The level of LP input signal sig: through the LP matrix the selected
 * pad's LP chain or a constant, inverted by IN_INV_SEL; bypassing it, the
 * LP UART's RXD on GPIO15 in function 0, or the signal's default.
 */
static bool lp_sig_in_level(ESP32P4GpioState *s, unsigned sig, uint64_t raw,
                            uint32_t lp)
{
    uint32_t cfg = s->lp_func_in_sel[sig];
    unsigned sel;
    bool v;

    if (sig >= LP_IN_SIGS) {
        return false;
    }
    if (!FIELD_EX32(cfg, LP_FUNC_IN, SIG_IN_SEL)) {
        if (sig == LP_SIG_UART_RXD && lp_owned(s, LP_PAD_UART_RXD) &&
            FIELD_EX32(s->lp_pad[LP_PAD_UART_RXD], LP_PAD, FUN_SEL) == 0) {
            return bit64(raw, LP_PAD_UART_RXD);
        }
        return lp_sig_in_default(sig);
    }
    sel = FIELD_EX32(cfg, LP_FUNC_IN, IN_SEL);
    if (sel < ESP32P4_GPIO_LP_PADS) {
        v = bit64(lp, sel);
    } else if (sel & LP_IN_SEL_CONST) {
        v = sel & LP_IN_SEL_HIGH;
    } else {
        v = false;
    }
    return v ^ FIELD_EX32(cfg, LP_FUNC_IN, IN_INV_SEL);
}

/* Pulse ETM event line @line, of an edge at time @t. */
static void etm_pulse(qemu_irq line, int64_t t)
{
    esp32_line_set_at(line, 1, t);
    esp32_line_set_at(line, 0, t);
}

/*
 * [spec:nuos:req:emu.esp32p4.etm]
 * ETM events, on the edges of the pin-filtered inputs at time @t.
 */
static void etm_events(ESP32P4GpioState *s, uint64_t pf, int64_t t)
{
    uint64_t edges = pf ^ s->pf_in;

    for (unsigned ch = 0; ch < ESP32P4_GPIO_ETM_CHANNELS; ch++) {
        uint32_t cfg = s->etm_event_cfg[ch];
        unsigned n = FIELD_EX32(cfg, ETM_EVENT, SEL);

        if (!FIELD_EX32(cfg, ETM_EVENT, EN) || n >= ESP32P4_GPIO_PADS ||
            !bit64(edges, n)) {
            continue;
        }
        etm_pulse(s->etm_event[ESP32P4_GPIO_ETM_LINE(ch,
                      bit64(pf, n) ? ESP32P4_GPIO_ETM_RISE
                                   : ESP32P4_GPIO_ETM_FALL)], t);
        etm_pulse(s->etm_event[ESP32P4_GPIO_ETM_LINE(ch,
                      ESP32P4_GPIO_ETM_ANY)], t);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * One resolution: the modulators up to now, the pads, the input chains at
 * the time of the change being propagated, the interrupts they latch, and
 * every output that changed.
 */
static void propagate(ESP32P4GpioState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t t = esp32_line_time();
    int8_t iomux_src[ESP32P4_GPIO_SIGS];
    uint64_t pad, raw, pf, hp, changed, drv_high, drv_low;
    uint32_t lp;
    bool resync = s->resync;
    bool irq[ESP32P4_GPIO_IRQS], wakeup;

    s->resync = false;
    for (unsigned ch = 0; ch < ESP32P4_GPIO_SDMS; ch++) {
        sdm_advance(s, ch, now);
    }

    resolve(s, &pad, &raw, &drv_high, &drv_low);
    run_chains(s, raw, t, &pf, &hp, &lp);
    etm_events(s, pf, t);
    latch_interrupts(s, hp, lp);
    changed = pad ^ s->pad_level;
    s->pad_level = pad;
    s->raw_in = raw;
    s->pf_in = pf;
    s->hp_in = hp;
    s->lp_in = lp;

    memset(iomux_src, -1, sizeof(iomux_src));
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        unsigned f;
        const IomuxFunc *fn = hp_func(s, n, &f);

        if (!lp_owned(s, n) && fn->kind == IOMUX_SIG && fn->sig_in >= 0 &&
            iomux_src[fn->sig_in] < 0) {
            iomux_src[fn->sig_in] = n;
        }
    }

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        if (resync || bit64(changed, n)) {
            qemu_set_irq(s->pad_out[n], bit64(pad, n));
        }
    }
    changed = (drv_high ^ s->drive_high) | (drv_low ^ s->drive_low);
    s->drive_high = drv_high;
    s->drive_low = drv_low;
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        if (resync || bit64(changed, n)) {
            qemu_set_irq(s->drive_high_out[n], bit64(drv_high, n));
            qemu_set_irq(s->drive_low_out[n], bit64(drv_low, n));
        }
    }
    for (unsigned sig = 0; sig < ESP32P4_GPIO_SIGS; sig++) {
        bool v = sig_in_level(s, sig, raw, hp, iomux_src);

        if (resync || v != s->sig_in[sig]) {
            s->sig_in[sig] = v;
            qemu_set_irq(s->sig_in_out[sig], v);
        }
        if (sig >= SIG_LOOPBACK_FIRST && s->sig_out[sig] != v) {
            s->sig_out[sig] = v;
            s->update_pending = true;
        }
    }
    for (unsigned sig = 0; sig < ESP32P4_GPIO_LP_SIGS; sig++) {
        bool v = lp_sig_in_level(s, sig, raw, lp);

        if (resync || v != s->lp_sig_in[sig]) {
            s->lp_sig_in[sig] = v;
            qemu_set_irq(s->lp_sig_in_out[sig], v);
        }
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        unsigned f;
        const IomuxFunc *fn = hp_func(s, n, &f);

        for (unsigned i = 0; i < ESP32P4_IOMUX_FUNCS; i++) {
            unsigned idx = ESP32P4_GPIO_IOMUX_LINE(n, i);
            bool v = !lp_owned(s, n) && fn->kind == IOMUX_DIRECT && f == i &&
                     bit64(raw, n);

            if (resync || v != s->iomux_in[idx]) {
                s->iomux_in[idx] = v;
                qemu_set_irq(s->iomux_in_out[idx], v);
            }
        }
    }

    wakeup = hp_wakeup(s);
    if (resync || wakeup != s->wakeup_level) {
        s->wakeup_level = wakeup;
        qemu_set_irq(s->wakeup_out, wakeup);
    }
    wakeup = lp_wakeup(s);
    if (resync || wakeup != s->lp_wakeup_level) {
        s->lp_wakeup_level = wakeup;
        qemu_set_irq(s->lp_wakeup_out, wakeup);
    }

    for (unsigned k = 0; k < 4; k++) {
        irq[ESP32P4_GPIO_IRQ_INTR0 + k] = int_view(s, int_ena_bit[k]) != 0;
    }
    irq[ESP32P4_GPIO_IRQ_PAD_COMP] = (s->int_raw & s->int_ena) != 0;
    irq[ESP32P4_GPIO_IRQ_LP] = (s->lp_status & LP_MASK) != 0;
    for (unsigned i = 0; i < ESP32P4_GPIO_IRQS; i++) {
        if (resync || irq[i] != s->irq_level[i]) {
            s->irq_level[i] = irq[i];
            qemu_set_irq(s->irq[i], irq[i]);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.boot]
 * GPIO_STRAP: the strapping as latched at reset, its boot mode bits
 * (GPIO35 and GPIO36) those of joint download boot while the boot mode
 * override holds a chip strapped for SPI boot there.
 */
static uint32_t esp32p4_gpio_strap(ESP32P4GpioState *s)
{
    if (s->force_download && (s->strap & ESP32P4_GPIO_STRAP_SPI_BOOT)) {
        return (s->strap & ~ESP32P4_GPIO_STRAP_SPI_BOOT) |
               ESP32P4_GPIO_STRAP_JOINT_DOWNLOAD;
    }
    return s->strap;
}

/* [spec:nuos:req:emu.esp32p4.boot] */
static void esp32p4_gpio_update_boot_mode(ESP32P4GpioState *s)
{
    qemu_set_irq(s->spi_boot_out,
                 !!(esp32p4_gpio_strap(s) & ESP32P4_GPIO_STRAP_SPI_BOOT));
}

/*
 * Bound on re-resolving the pads when peripherals answer an input change
 * with an output change, as a loop through the matrix that inverts itself
 * would otherwise never settle.
 */
#define GPIO_SETTLE_LIMIT 16

static void esp32p4_gpio_settle(void *opaque)
{
    ESP32P4GpioState *s = opaque;
    unsigned rounds = 0;

    do {
        s->update_pending = false;
        propagate(s);
        if (++rounds == GPIO_SETTLE_LIMIT && s->update_pending) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: signal routing oscillates through the GPIO "
                          "matrix\n", TYPE_ESP32P4_GPIO);
            break;
        }
    } while (s->update_pending);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Resolve the pads until the routing settles. Every line this drives
 * carries one time, the change's (esp32_line_time()): a stamped change's
 * own, and for one that carries none, such as a property or a clock, the
 * moment the resolution began. Every receiver of an edge, and every block
 * the edge reaches through others, times it alike, rather than each
 * reading the clock as it happens to run.
 */
static void esp32p4_gpio_update(ESP32P4GpioState *s)
{
    if (s->updating) {
        s->update_pending = true;
        return;
    }
    s->updating = true;
    esp32_line_run_at(esp32_line_time(), esp32p4_gpio_settle, s);
    s->update_pending = false;
    s->updating = false;
    filters_schedule(s);
}

static void notify_route(ESP32P4GpioState *s)
{
    notifier_list_notify(&s->route_notifiers, s);
}

/* A filter output change, at the time it falls due, esp32_line_time(). */
static void filters_change(void *opaque)
{
    ESP32P4GpioState *s = opaque;

    filters_advance(s, esp32_line_time());
    esp32p4_gpio_update(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * Make the filter output changes due by t, each at its own time. Each
 * moves the filters' next change past its own; should one not, the
 * filters take their samples up to t at once.
 */
static void filters_run(ESP32P4GpioState *s, int64_t t)
{
    int64_t due, done = INT64_MIN;

    while ((due = filters_next(s)) <= t) {
        if (due <= done) {
            filters_advance(s, t);
            esp32p4_gpio_update(s);
            return;
        }
        done = due;
        esp32_line_run_at(due, filters_change, s);
    }
}

/*
 * A change from outside at time t: first the filter output changes due
 * by then, which a QEMU timer running late has not yet made. Outside a
 * resolution the filter timer is armed for the earliest change due.
 */
static void filters_catch_up(ESP32P4GpioState *s, int64_t t)
{
    if (!s->updating && timer_pending(&s->filter_timer) &&
        timer_expire_time_ns(&s->filter_timer) <= t) {
        filters_run(s, t);
    }
}

/* Bring the line sources up to now, then resolve the pads. */
static void esp32p4_gpio_sample(ESP32P4GpioState *s)
{
    esp32_line_sync();
    esp32p4_gpio_update(s);
}

/* The modulators as a line source, for blocks that sample the pads. */
static void esp32p4_gpio_line_sync(Notifier *n, void *data)
{
    ESP32P4GpioState *s = container_of(n, ESP32P4GpioState, line_source);

    if (sdm_clock_on(s)) {
        filters_catch_up(s, esp32_line_time());
        esp32p4_gpio_update(s);
    }
}

static void esp32p4_gpio_filter_timer(void *opaque)
{
    ESP32P4GpioState *s = opaque;

    filters_run(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    filters_schedule(s);
}

/* Routing queries */

/* [spec:nuos:req:emu.esp32p4.gpio] */
uint64_t esp32p4_gpio_sig_out_pads(ESP32P4GpioState *s, unsigned sig)
{
    uint64_t pads = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        bool val, oe;

        if (pad_drive(s, n, &val, &oe) == (int)sig && oe) {
            pads |= 1ULL << n;
        }
    }
    return pads;
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
int esp32p4_gpio_sig_in_pad(ESP32P4GpioState *s, unsigned sig)
{
    uint32_t cfg;
    int pad = -1;
    PadCtl c;

    if (!sig_has_input(sig)) {
        return -1;
    }
    cfg = s->func_in_sel[sig];
    if (FIELD_EX32(cfg, FUNC_IN, SIG_IN_SEL)) {
        unsigned sel = FIELD_EX32(cfg, FUNC_IN, IN_SEL);

        if (sel < ESP32P4_GPIO_PADS) {
            pad = sel;
        }
    } else {
        pad = iomux_in_pad(s, sig);
    }
    if (pad < 0) {
        return -1;
    }
    pad_ctl(s, pad, &c);
    return c.ie ? pad : -1;
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
uint32_t esp32p4_gpio_lp_sig_out_pads(ESP32P4GpioState *s, unsigned sig)
{
    uint32_t pads = 0;

    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        uint32_t cfg = s->lp_func_out_sel[n];
        bool val, oe;

        if (!lp_owned(s, n)) {
            continue;
        }
        pad_drive(s, n, &val, &oe);
        if (!oe) {
            continue;
        }
        if (lp_func_is_matrix(s, n) ?
            FIELD_EX32(cfg, LP_FUNC_OUT, OUT_SEL) == sig :
            n == LP_PAD_UART_TXD && sig == LP_SIG_UART_TXD) {
            pads |= 1u << n;
        }
    }
    return pads;
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
int esp32p4_gpio_lp_sig_in_pad(ESP32P4GpioState *s, unsigned sig)
{
    uint32_t cfg;
    int pad = -1;
    PadCtl c;

    if (sig >= LP_IN_SIGS) {
        return -1;
    }
    cfg = s->lp_func_in_sel[sig];
    if (FIELD_EX32(cfg, LP_FUNC_IN, SIG_IN_SEL)) {
        unsigned sel = FIELD_EX32(cfg, LP_FUNC_IN, IN_SEL);

        if (sel < ESP32P4_GPIO_LP_PADS) {
            pad = sel;
        }
    } else if (sig == LP_SIG_UART_RXD && lp_owned(s, LP_PAD_UART_RXD) &&
               FIELD_EX32(s->lp_pad[LP_PAD_UART_RXD], LP_PAD, FUN_SEL) == 0) {
        pad = LP_PAD_UART_RXD;
    }
    if (pad < 0) {
        return -1;
    }
    pad_ctl(s, pad, &c);
    return c.ie ? pad : -1;
}

void esp32p4_gpio_add_route_notifier(ESP32P4GpioState *s, Notifier *n)
{
    notifier_list_add(&s->route_notifiers, n);
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
bool esp32p4_gpio_pad_driven(ESP32P4GpioState *s, unsigned n, bool *level)
{
    if (n >= ESP32P4_GPIO_PADS) {
        return false;
    }
    if (bit64(s->ext_driven, n)) {
        *level = bit64(s->ext_level, n);
        return true;
    }
    return pad_driven_by_chip(s, n, level);
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
bool esp32p4_gpio_iomux_in_selected(ESP32P4GpioState *s, unsigned n,
                                    unsigned func)
{
    unsigned f;
    PadCtl c;

    if (n >= ESP32P4_GPIO_PADS || lp_owned(s, n)) {
        return false;
    }
    hp_func(s, n, &f);
    if (f != func) {
        return false;
    }
    pad_ctl(s, n, &c);
    return c.ie;
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
bool esp32p4_gpio_sig_in_source(ESP32P4GpioState *s, unsigned sig,
                                unsigned *src, bool *inverted)
{
    uint32_t cfg, out_cfg;
    bool inv = false, val, oe;
    int pad = -1, out_sig;
    unsigned f;
    const IomuxFunc *fn;
    PadCtl c;

    if (!sig_has_input(sig)) {
        return false;
    }
    cfg = s->func_in_sel[sig];
    if (FIELD_EX32(cfg, FUNC_IN, SIG_IN_SEL)) {
        unsigned sel = FIELD_EX32(cfg, FUNC_IN, IN_SEL);

        if (sel < ESP32P4_GPIO_PADS) {
            pad = sel;
        }
        inv = FIELD_EX32(cfg, FUNC_IN, IN_INV_SEL);
    } else {
        pad = iomux_in_pad(s, sig);
    }
    if (pad < 0 || lp_owned(s, pad) || bit64(s->ext_driven, pad) ||
        bit64(s->held, pad) || s->pad_wire[pad]) {
        return false;
    }
    pad_ctl(s, pad, &c);
    if (!c.ie || c.od) {
        return false;
    }
    out_sig = pad_drive(s, pad, &val, &oe);
    if (out_sig < 0 || !oe) {
        return false;
    }
    out_cfg = s->func_out_sel[pad];
    fn = hp_func(s, pad, &f);
    if (fn->kind == IOMUX_GPIO) {
        inv ^= FIELD_EX32(out_cfg, FUNC_OUT, OUT_INV_SEL);
    }
    *src = out_sig;
    *inverted = inv;
    return true;
}

/* Register blocks */

/* GPIO_INTR_k and GPIO_INTR1_k */
static uint32_t intr_word(ESP32P4GpioState *s, unsigned k, unsigned word)
{
    uint64_t v = int_view(s, int_ena_bit[k]);

    return word ? (uint32_t)(v >> 32) : (uint32_t)v;
}

static bool gpio_read_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t *v)
{
    switch (addr) {
    case A_BT_SELECT:
        *v = s->bt_select;
        return true;
    case A_OUT:
        *v = s->out[0];
        return true;
    case A_OUT1:
        *v = s->out[1];
        return true;
    case A_ENABLE:
        *v = s->enable[0];
        return true;
    case A_ENABLE1:
        *v = s->enable[1];
        return true;
    case A_OUT_W1TS:
    case A_OUT_W1TC:
    case A_OUT1_W1TS:
    case A_OUT1_W1TC:
    case A_ENABLE_W1TS:
    case A_ENABLE_W1TC:
    case A_ENABLE1_W1TS:
    case A_ENABLE1_W1TC:
    case A_STATUS_W1TS:
    case A_STATUS_W1TC:
    case A_STATUS1_W1TS:
    case A_STATUS1_W1TC:
    case A_INT_CLR:
        *v = 0;
        return true;
    case A_STRAP:
        *v = esp32p4_gpio_strap(s);
        return true;
    case A_IN:
        esp32p4_gpio_sample(s);
        *v = (uint32_t)s->hp_in;
        return true;
    case A_IN1:
        esp32p4_gpio_sample(s);
        *v = (uint32_t)(s->hp_in >> 32);
        return true;
    case A_STATUS:
        esp32p4_gpio_sample(s);
        *v = s->status[0];
        return true;
    case A_STATUS1:
        esp32p4_gpio_sample(s);
        *v = s->status[1];
        return true;
    case A_INTR_0:
    case A_INTR1_0:
    case A_INTR_1:
    case A_INTR1_1:
        esp32p4_gpio_sample(s);
        *v = intr_word(s, (addr - A_INTR_0) / 8, ((addr - A_INTR_0) / 4) & 1);
        return true;
    case A_INTR_2:
    case A_INTR1_2:
    case A_INTR_3:
    case A_INTR1_3:
        esp32p4_gpio_sample(s);
        *v = intr_word(s, 2 + (addr - A_INTR_2) / 8,
                       ((addr - A_INTR_2) / 4) & 1);
        return true;
    case A_STATUS_NEXT:
        esp32p4_gpio_sample(s);
        *v = (uint32_t)status_next(s);
        return true;
    case A_STATUS_NEXT1:
        esp32p4_gpio_sample(s);
        *v = (uint32_t)(status_next(s) >> 32);
        return true;
    case A_PIN0 ... A_PIN_LAST:
        *v = s->pin[(addr - A_PIN0) / 4];
        return true;
    case A_FUNC0_IN_SEL ... A_FUNC_IN_SEL_LAST:
        if (!sig_has_input((addr - A_FUNC0_IN_SEL) / 4)) {
            return false;
        }
        *v = s->func_in_sel[(addr - A_FUNC0_IN_SEL) / 4];
        return true;
    case A_FUNC0_OUT_SEL ... A_FUNC_OUT_SEL_LAST:
        *v = s->func_out_sel[(addr - A_FUNC0_OUT_SEL) / 4];
        return true;
    case A_CLOCK_GATE:
        *v = s->clock_gate;
        return true;
    case A_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_ZERO_DET0:
    case A_ZERO_DET1:
        *v = s->zero_det[(addr - A_ZERO_DET0) / 4];
        return true;
    case A_SEND_SEQ:
        *v = s->send_seq;
        return true;
    case A_RECIVE_SEQ:
        *v = 0;
        return true;
    case A_BISTIN_SEL:
        *v = s->bistin_sel;
        return true;
    case A_BIST_CTRL:
        *v = s->bist_ctrl;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    case A_SIGMADELTA0 ... A_SIGMADELTA_LAST:
        *v = s->sdm[(addr - A_SIGMADELTA0) / 4];
        return true;
    case A_SIGMADELTA_MISC:
        *v = s->sdm_misc;
        return true;
    case A_GLITCH0 ... A_GLITCH_LAST:
        *v = s->glitch[(addr - A_GLITCH0) / 4];
        return true;
    case A_ETM_EVENT0 ... A_ETM_EVENT_LAST:
        *v = s->etm_event_cfg[(addr - A_ETM_EVENT0) / 4];
        return true;
    case A_ETM_TASK0 ... A_ETM_TASK_LAST:
        *v = s->etm_task_cfg[(addr - A_ETM_TASK0) / 4];
        return true;
    case A_EXT_VERSION:
        *v = s->ext_version;
        return true;
    default:
        return false;
    }
}

static void check_in_sel(unsigned sig, uint32_t cfg)
{
    unsigned sel = FIELD_EX32(cfg, FUNC_IN, IN_SEL);

    if (FIELD_EX32(cfg, FUNC_IN, SIG_IN_SEL) && sel >= ESP32P4_GPIO_PADS &&
        sel != IN_SEL_CONST_LOW && sel != IN_SEL_CONST_HIGH) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: input signal %u routed from %u, which is neither "
                      "a pad nor a constant; it reads low\n",
                      TYPE_ESP32P4_GPIO, sig, sel);
    }
}

/* Start a pad's pin filter at its input as it is enabled. */
static void pin_filter_enabled(ESP32P4GpioState *s, unsigned n)
{
    filter_start(&s->pin_filter[n], bit64(s->raw_in, n),
                 clk_cycles(&s->iomux_edges,
                            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
}

static void glitch_written(ESP32P4GpioState *s, unsigned ch, uint32_t old)
{
    uint32_t v = s->glitch[ch];
    FilterParams p = glitch_params(s, ch);
    int n = FIELD_EX32(v, GLITCH, INPUT_IO_NUM);

    if (!FIELD_EX32(v, GLITCH, EN)) {
        return;
    }
    if (n >= ESP32P4_GPIO_PADS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: glitch filter %u on GPIO%d, which is not a pad\n",
                      TYPE_ESP32P4_GPIO, ch, n);
    }
    if (p.width == 63) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: glitch filter %u's window width 63 is reserved\n",
                      TYPE_ESP32P4_GPIO, ch);
    }
    if (p.thres > p.width) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: glitch filter %u's threshold %u exceeds its "
                      "window %u; it passes nothing\n",
                      TYPE_ESP32P4_GPIO, ch, p.thres, p.width);
    }
    if (v != old && n < ESP32P4_GPIO_PADS) {
        filter_start(&s->glitch_filter[ch], bit64(s->pf_in, n),
                     clk_cycles(&s->iomux_edges,
                                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
    }
}

/* Set the bits of *reg in mask to value. */
static void put(uint32_t *reg, uint32_t value, uint32_t mask)
{
    *reg = (*reg & ~mask) | (value & mask);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * GPIO matrix and GPIO_EXT register writes, of the bits in mask.
 */
static void gpio_write_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t v,
                           uint32_t mask)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool route = false;
    uint32_t old;
    unsigned i;

    switch (addr) {
    case A_BT_SELECT:
        put(&s->bt_select, v, mask);
        break;
    case A_OUT:
        put(&s->out[0], v, mask);
        break;
    case A_OUT_W1TS:
        s->out[0] |= v & mask;
        break;
    case A_OUT_W1TC:
        s->out[0] &= ~(v & mask);
        break;
    case A_OUT1:
        put(&s->out[1], v, mask & HIGH_WORD_MASK);
        break;
    case A_OUT1_W1TS:
        s->out[1] |= v & mask & HIGH_WORD_MASK;
        break;
    case A_OUT1_W1TC:
        s->out[1] &= ~(v & mask);
        break;
    case A_ENABLE:
        put(&s->enable[0], v, mask);
        route = true;
        break;
    case A_ENABLE_W1TS:
        s->enable[0] |= v & mask;
        route = true;
        break;
    case A_ENABLE_W1TC:
        s->enable[0] &= ~(v & mask);
        route = true;
        break;
    case A_ENABLE1:
        put(&s->enable[1], v, mask & HIGH_WORD_MASK);
        route = true;
        break;
    case A_ENABLE1_W1TS:
        s->enable[1] |= v & mask & HIGH_WORD_MASK;
        route = true;
        break;
    case A_ENABLE1_W1TC:
        s->enable[1] &= ~(v & mask);
        route = true;
        break;
    case A_STATUS:
        put(&s->status[0], v, mask);
        break;
    case A_STATUS_W1TS:
        s->status[0] |= v & mask;
        break;
    case A_STATUS_W1TC:
        s->status[0] &= ~(v & mask);
        break;
    case A_STATUS1:
        put(&s->status[1], v, mask & HIGH_WORD_MASK);
        break;
    case A_STATUS1_W1TS:
        s->status[1] |= v & mask & HIGH_WORD_MASK;
        break;
    case A_STATUS1_W1TC:
        s->status[1] &= ~(v & mask);
        break;
    case A_PIN0 ... A_PIN_LAST:
        put(&s->pin[(addr - A_PIN0) / 4], v, mask & PIN_WRITABLE);
        break;
    case A_FUNC0_IN_SEL ... A_FUNC_IN_SEL_LAST:
        i = (addr - A_FUNC0_IN_SEL) / 4;
        if (!sig_has_input(i)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: write to GPIO_FUNC%u_IN_SEL_CFG, which does "
                          "not exist: signal %u has no input\n",
                          TYPE_ESP32P4_GPIO, i, i);
            return;
        }
        put(&s->func_in_sel[i], v, mask & FUNC_IN_WRITABLE);
        check_in_sel(i, s->func_in_sel[i]);
        route = true;
        break;
    case A_FUNC0_OUT_SEL ... A_FUNC_OUT_SEL_LAST:
        i = (addr - A_FUNC0_OUT_SEL) / 4;
        put(&s->func_out_sel[i], v, mask & FUNC_OUT_WRITABLE);
        if (FIELD_EX32(s->func_out_sel[i], FUNC_OUT, OUT_SEL) >
            ESP32P4_SIG_GPIO_OUT) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: GPIO%u output from signal %u, which does not "
                          "exist; it drives low\n", TYPE_ESP32P4_GPIO, i,
                          (unsigned)FIELD_EX32(s->func_out_sel[i], FUNC_OUT,
                                               OUT_SEL));
        }
        route = true;
        break;
    case A_CLOCK_GATE:
        /* The register file's clock gate forced on: nothing to see. */
        put(&s->clock_gate, v, mask & 1);
        break;
    case A_INT_RAW:
        s->int_raw &= ~(v & mask & INT_BITS);
        break;
    case A_INT_ENA:
        put(&s->int_ena, v, mask & INT_BITS);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & mask & INT_BITS);
        break;
    case A_ZERO_DET0:
    case A_ZERO_DET1:
        put(&s->zero_det[(addr - A_ZERO_DET0) / 4], v, mask);
        break;
    case A_SEND_SEQ:
        put(&s->send_seq, v, mask);
        break;
    case A_BISTIN_SEL:
        put(&s->bistin_sel, v, mask & BISTIN_SEL_WRITABLE);
        break;
    case A_BIST_CTRL:
        put(&s->bist_ctrl, v, mask & (BIST_PAD_OE | BIST_MODE));
        if (v & mask & BIST_START) {
            qemu_log_mask(LOG_UNIMP, "%s: the SDIO pads' BIST is not "
                          "modelled\n", TYPE_ESP32P4_GPIO);
        }
        break;
    case A_DATE:
        put(&s->date, v, mask & DATE_MASK);
        break;
    case A_SIGMADELTA0 ... A_SIGMADELTA_LAST:
        i = (addr - A_SIGMADELTA0) / 4;
        sdm_reanchor(s, i, now);
        put(&s->sdm[i], v, mask & SDM_WRITABLE);
        break;
    case A_SIGMADELTA_MISC:
        old = s->sdm_misc;
        if ((v & mask & R_SDM_MISC_FUNCTION_CLK_EN_MASK) !=
            (old & mask & R_SDM_MISC_FUNCTION_CLK_EN_MASK)) {
            sdm_reanchor_all(s, now);
        }
        put(&s->sdm_misc, v, mask & SDM_MISC_WRITABLE);
        if (FIELD_EX32(s->sdm_misc, SDM_MISC, SPI_SWAP)) {
            qemu_log_mask(LOG_UNIMP, "%s: GPIO_EXT_SPI_SWAP, which the TRM "
                          "reserves, is not modelled\n", TYPE_ESP32P4_GPIO);
        }
        break;
    case A_GLITCH0 ... A_GLITCH_LAST:
        i = (addr - A_GLITCH0) / 4;
        old = s->glitch[i];
        /* Bring the filter up to now under its old settings. */
        filters_advance(s, now);
        put(&s->glitch[i], v, mask & GLITCH_WRITABLE);
        glitch_written(s, i, old);
        break;
    case A_ETM_EVENT0 ... A_ETM_EVENT_LAST:
        i = (addr - A_ETM_EVENT0) / 4;
        put(&s->etm_event_cfg[i], v, mask & ETM_EVENT_WRITABLE);
        if (FIELD_EX32(s->etm_event_cfg[i], ETM_EVENT, EN) &&
            FIELD_EX32(s->etm_event_cfg[i], ETM_EVENT, SEL) >=
            ESP32P4_GPIO_PADS) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: ETM event channel %u on a GPIO that is not a "
                          "pad\n", TYPE_ESP32P4_GPIO, i);
        }
        break;
    case A_ETM_TASK0 ... A_ETM_TASK_LAST:
        put(&s->etm_task_cfg[(addr - A_ETM_TASK0) / 4], v,
            mask & ETM_TASK_WRITABLE);
        break;
    case A_EXT_VERSION:
        put(&s->ext_version, v, mask & DATE_MASK);
        break;
    case A_STRAP:
    case A_IN:
    case A_IN1:
    case A_INTR_0 ... A_STATUS_NEXT1:
    case A_INTR_2 ... A_INTR1_3:
    case A_INT_ST:
    case A_RECIVE_SEQ:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to read-only register 0x%" HWADDR_PRIx "\n",
                      TYPE_ESP32P4_GPIO, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to 0x%" HWADDR_PRIx ", which has no "
                      "register\n", TYPE_ESP32P4_GPIO, addr);
        return;
    }
    esp32p4_gpio_update(s);
    if (route) {
        notify_route(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * HP IO MUX register writes. MCU_SEL, FUN_IE, the pulls and FILTER_EN
 * take effect at once; the MCU_* settings in light sleep only.
 */
static void iomux_write_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t v,
                            uint32_t mask)
{
    unsigned n;
    uint32_t old;

    switch (addr) {
    case A_IOMUX_GPIO0 ... A_IOMUX_LAST:
        n = (addr - A_IOMUX_GPIO0) / 4;
        old = s->iomux[n];
        if (n < ESP32P4_GPIO_PADS && hp_filter_on(s, n)) {
            filters_advance(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        put(&s->iomux[n], v, mask & IOMUX_WRITABLE);
        if (FIELD_EX32(s->iomux[n], IOMUX, MCU_SEL) >= ESP32P4_IOMUX_FUNCS &&
            FIELD_EX32(s->iomux[n], IOMUX, MCU_SEL) !=
            FIELD_EX32(old, IOMUX, MCU_SEL)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: GPIO%u's IO MUX function %u does not exist; "
                          "the pad is disconnected\n", TYPE_ESP32P4_GPIO, n,
                          (unsigned)FIELD_EX32(s->iomux[n], IOMUX, MCU_SEL));
        }
        if (n < ESP32P4_GPIO_PADS && hp_filter_on(s, n) &&
            !FIELD_EX32(old, IOMUX, FILTER_EN)) {
            pin_filter_enabled(s, n);
        }
        break;
    case A_IOMUX_DATE:
        put(&s->iomux_date, v, mask & DATE_MASK);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: IO MUX write to 0x%" HWADDR_PRIx ", which has no "
                      "register\n", TYPE_ESP32P4_GPIO, addr);
        return;
    }
    esp32p4_gpio_update(s);
    notify_route(s);
}

static bool iomux_read_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t *v)
{
    switch (addr) {
    case A_IOMUX_GPIO0 ... A_IOMUX_LAST:
        *v = s->iomux[(addr - A_IOMUX_GPIO0) / 4];
        return true;
    case A_IOMUX_DATE:
        *v = s->iomux_date;
        return true;
    default:
        return false;
    }
}

static bool lp_gpio_read_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t *v)
{
    switch (addr) {
    case A_LP_CLK_EN:
        *v = s->lp_clk_en;
        return true;
    case A_LP_VER_DATE:
        *v = s->lp_ver_date;
        return true;
    case A_LP_OUT:
        *v = s->lp_out;
        return true;
    case A_LP_ENABLE:
        *v = s->lp_enable;
        return true;
    case A_LP_STATUS:
        esp32p4_gpio_sample(s);
        *v = s->lp_status;
        return true;
    case A_LP_OUT_W1TS:
    case A_LP_OUT_W1TC:
    case A_LP_ENABLE_W1TS:
    case A_LP_ENABLE_W1TC:
    case A_LP_STATUS_W1TS:
    case A_LP_STATUS_W1TC:
        *v = 0;
        return true;
    case A_LP_STATUS_NEXT:
        esp32p4_gpio_sample(s);
        *v = lp_status_next(s);
        return true;
    case A_LP_IN:
        esp32p4_gpio_sample(s);
        *v = s->lp_in;
        return true;
    case A_LP_PIN0 ... A_LP_PIN_LAST:
        *v = s->lp_pin[(addr - A_LP_PIN0) / 4];
        return true;
    case A_LP_FUNC0_IN_SEL ... A_LP_FUNC_IN_LAST:
        *v = s->lp_func_in_sel[(addr - A_LP_FUNC0_IN_SEL) / 4];
        return true;
    case A_LP_FUNC0_OUT_SEL ... A_LP_FUNC_OUT_LAST:
        *v = s->lp_func_out_sel[(addr - A_LP_FUNC0_OUT_SEL) / 4];
        return true;
    default:
        return false;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * LP GPIO matrix register writes.
 */
static void lp_gpio_write_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t v,
                              uint32_t mask)
{
    bool route = false;
    unsigned i, sel;

    switch (addr) {
    case A_LP_CLK_EN:
        /* The register file's clock gate forced on: nothing to see. */
        put(&s->lp_clk_en, v, mask & 1);
        return;
    case A_LP_VER_DATE:
        put(&s->lp_ver_date, v, mask & DATE_MASK);
        return;
    case A_LP_OUT:
        put(&s->lp_out, v, mask & LP_MASK);
        break;
    case A_LP_OUT_W1TS:
        s->lp_out |= v & mask & LP_MASK;
        break;
    case A_LP_OUT_W1TC:
        s->lp_out &= ~(v & mask);
        break;
    case A_LP_ENABLE:
        put(&s->lp_enable, v, mask & LP_MASK);
        route = true;
        break;
    case A_LP_ENABLE_W1TS:
        s->lp_enable |= v & mask & LP_MASK;
        route = true;
        break;
    case A_LP_ENABLE_W1TC:
        s->lp_enable &= ~(v & mask);
        route = true;
        break;
    case A_LP_STATUS:
        put(&s->lp_status, v, mask & LP_MASK);
        break;
    case A_LP_STATUS_W1TS:
        s->lp_status |= v & mask & LP_MASK;
        break;
    case A_LP_STATUS_W1TC:
        s->lp_status &= ~(v & mask);
        break;
    case A_LP_PIN0 ... A_LP_PIN_LAST:
        i = (addr - A_LP_PIN0) / 4;
        if (v & mask & R_LP_PIN_EDGE_WAKEUP_CLR_MASK) {
            s->lp_wakeup_edge &= ~BIT(i);
        }
        put(&s->lp_pin[i], v, mask & LP_PIN_WRITABLE);
        route = true;
        break;
    case A_LP_FUNC0_IN_SEL ... A_LP_FUNC_IN_LAST:
        i = (addr - A_LP_FUNC0_IN_SEL) / 4;
        put(&s->lp_func_in_sel[i], v, mask & LP_FUNC_IN_WRITABLE);
        sel = FIELD_EX32(s->lp_func_in_sel[i], LP_FUNC_IN, IN_SEL);
        if (FIELD_EX32(s->lp_func_in_sel[i], LP_FUNC_IN, SIG_IN_SEL) &&
            sel >= ESP32P4_GPIO_LP_PADS && !(sel & LP_IN_SEL_CONST)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: LP input signal %u routed from %u, which is "
                          "neither an LP pad nor a constant; it reads low\n",
                          TYPE_ESP32P4_GPIO, i, sel);
        }
        route = true;
        break;
    case A_LP_FUNC0_OUT_SEL ... A_LP_FUNC_OUT_LAST:
        i = (addr - A_LP_FUNC0_OUT_SEL) / 4;
        put(&s->lp_func_out_sel[i], v, mask & LP_FUNC_OUT_WRITABLE);
        if (FIELD_EX32(s->lp_func_out_sel[i], LP_FUNC_OUT, OUT_SEL) >
            ESP32P4_LP_SIG_GPIO_OUT) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: LP GPIO%u output from signal %u, which does "
                          "not exist; it drives low\n", TYPE_ESP32P4_GPIO, i,
                          (unsigned)FIELD_EX32(s->lp_func_out_sel[i],
                                               LP_FUNC_OUT, OUT_SEL));
        }
        route = true;
        break;
    case A_LP_STATUS_NEXT:
    case A_LP_IN:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to read-only LP GPIO register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_GPIO, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: LP GPIO write to 0x%" HWADDR_PRIx ", which has no "
                      "register\n", TYPE_ESP32P4_GPIO, addr);
        return;
    }
    esp32p4_gpio_update(s);
    if (route) {
        notify_route(s);
    }
}

static bool lp_iomux_read_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t *v)
{
    switch (addr) {
    case A_LPIO_CLK_EN:
        *v = s->lp_iomux_clk_en;
        return true;
    case A_LPIO_VER_DATE:
        *v = s->lp_iomux_ver_date;
        return true;
    case A_LPIO_PAD0 ... A_LPIO_PAD_LAST:
        *v = s->lp_pad[(addr - A_LPIO_PAD0) / 4];
        return true;
    case A_LPIO_EXT_WAKEUP0:
        *v = s->lp_ext_wakeup0_sel;
        return true;
    case A_LPIO_PAD_HOLD:
        *v = s->lp_pad_hold;
        return true;
    case A_LPIO_PAD_HYS:
        *v = s->lp_pad_hys;
        return true;
    default:
        return false;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * LP IO MUX register writes. MUX_SEL hands a pad between the HP and LP IO
 * MUXes at once; LP_PAD_HOLD holds the LP pads.
 */
static void lp_iomux_write_reg(ESP32P4GpioState *s, hwaddr addr, uint32_t v,
                               uint32_t mask)
{
    uint32_t old;
    unsigned n;

    switch (addr) {
    case A_LPIO_CLK_EN:
        /* The register file's clock gate forced on: nothing to see. */
        put(&s->lp_iomux_clk_en, v, mask & 1);
        return;
    case A_LPIO_VER_DATE:
        put(&s->lp_iomux_ver_date, v, mask & DATE_MASK);
        return;
    case A_LPIO_PAD0 ... A_LPIO_PAD_LAST:
        n = (addr - A_LPIO_PAD0) / 4;
        old = s->lp_pad[n];
        if (lp_filter_on(s, n)) {
            filters_advance(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        put(&s->lp_pad[n], v, mask & LP_PAD_WRITABLE);
        if (lp_filter_on(s, n) && !FIELD_EX32(old, LP_PAD, FILTER_EN)) {
            filter_start(&s->lp_pin_filter[n], bit64(s->raw_in, n),
                         clk_cycles(&s->lp_edges,
                                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)));
        }
        if (FIELD_EX32(s->lp_pad[n], LP_PAD, FUN_SEL) >= 2) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: GPIO%u's LP IO MUX function %u does not "
                          "exist; the pad is disconnected\n",
                          TYPE_ESP32P4_GPIO, n,
                          (unsigned)FIELD_EX32(s->lp_pad[n], LP_PAD,
                                               FUN_SEL));
        }
        break;
    case A_LPIO_EXT_WAKEUP0:
        /* The PMU's EXT0 wakeup pad and XTAL control pad select */
        put(&s->lp_ext_wakeup0_sel, v, mask & LP_EXT_WAKEUP0_WRITABLE);
        return;
    case A_LPIO_PAD_HOLD:
        put(&s->lp_pad_hold, v, mask & LP_MASK);
        break;
    case A_LPIO_PAD_HYS:
        put(&s->lp_pad_hys, v, mask & LP_MASK);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: LP IO MUX write to 0x%" HWADDR_PRIx ", which has "
                      "no register\n", TYPE_ESP32P4_GPIO, addr);
        return;
    }
    esp32p4_gpio_update(s);
    notify_route(s);
}

typedef bool (*RegRead)(ESP32P4GpioState *s, hwaddr addr, uint32_t *v);
typedef void (*RegWrite)(ESP32P4GpioState *s, hwaddr addr, uint32_t v,
                         uint32_t mask);

static const struct {
    const char *name;
    RegRead read;
    RegWrite write;
    unsigned reset;
} blocks[] = {
    [ESP32P4_GPIO_MMIO_GPIO] = {
        "GPIO", gpio_read_reg, gpio_write_reg, ESP32P4_GPIO_RESET_HP },
    [ESP32P4_GPIO_MMIO_IO_MUX] = {
        "IO MUX", iomux_read_reg, iomux_write_reg, ESP32P4_GPIO_RESET_HP },
    [ESP32P4_GPIO_MMIO_LP_GPIO] = {
        "LP GPIO", lp_gpio_read_reg, lp_gpio_write_reg,
        ESP32P4_GPIO_RESET_LP },
    [ESP32P4_GPIO_MMIO_LP_IOMUX] = {
        "LP IO MUX", lp_iomux_read_reg, lp_iomux_write_reg,
        ESP32P4_GPIO_RESET_LP },
};

/* A register access under way, which acts at one moment throughout. */
typedef struct GpioAccess {
    ESP32P4GpioRegion *r;
    hwaddr addr;
    uint32_t value;
    uint32_t mask;
} GpioAccess;

static void esp32p4_gpio_do_read(void *opaque)
{
    GpioAccess *a = opaque;
    ESP32P4GpioRegion *r = a->r;

    if (!blocks[r->block].read(r->s, a->addr & ~3, &a->value)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: %s read at 0x%" HWADDR_PRIx ", which has no "
                      "register\n", TYPE_ESP32P4_GPIO, blocks[r->block].name,
                      a->addr);
        a->value = 0;
    }
}

static void esp32p4_gpio_do_write(void *opaque)
{
    GpioAccess *a = opaque;

    blocks[a->r->block].write(a->r->s, a->addr & ~3, a->value, a->mask);
}

/*
 * Bring the line sources up to the present, before a write: a source
 * whose edges due before the write are not yet driven (its timer running
 * late) would otherwise drive them on reacting to the write's change, from
 * inside the resolution of that change, which would propagate them at the
 * write's time rather than their own.
 */
static void esp32p4_gpio_sync_sources(void *opaque)
{
    esp32_line_sync();
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The other offsets of each block have no register: they read as zero and
 * ignore writes. Byte and halfword accesses read and write their bytes of
 * a register. An access acts at the moment it began: the pad changes it
 * makes (GPIO_OUT, GPIO_ENABLE, the IO MUX, the routing) and the samples
 * of the line sources it takes all carry that time (see esp32_line.h), so
 * that every block an edge reaches sees it at the same moment however its
 * handlers are ordered, and without icount however long they take. A
 * write first brings the line sources up to that moment, so that the
 * edges they made before it reach their receivers, each at its own time,
 * before the write's own changes do.
 */
static uint64_t esp32p4_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    GpioAccess a = { .r = opaque, .addr = addr };
    int64_t t = esp32_line_time();

    filters_catch_up(a.r->s, t);
    esp32_line_run_at(t, esp32p4_gpio_do_read, &a);
    return extract32(a.value, (addr & 3) * 8, size * 8);
}

static void esp32p4_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    unsigned shift = (addr & 3) * 8;
    GpioAccess a = {
        .r = opaque, .addr = addr, .value = value << shift,
        .mask = MAKE_64BIT_MASK(shift, size * 8),
    };
    int64_t t = esp32_line_time();

    esp32_line_run_at(t, esp32p4_gpio_sync_sources, NULL);
    filters_catch_up(a.r->s, t);
    if (a.r->s->in_reset[blocks[a.r->block].reset]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to the %s block at 0x%" HWADDR_PRIx
                      " while it is held in reset\n", TYPE_ESP32P4_GPIO,
                      blocks[a.r->block].name, addr);
        return;
    }
    esp32_line_run_at(t, esp32p4_gpio_do_write, &a);
}

static const MemoryRegionOps esp32p4_gpio_ops = {
    .read = esp32p4_gpio_read,
    .write = esp32p4_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* Inputs */

static void esp32p4_gpio_pad_in(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->ext_driven |= 1ULL << n;
    s->ext_level = deposit64(s->ext_level, n, 1, level != 0);
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_pad_release(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    if (level) {
        filters_catch_up(s, esp32_line_time());
        s->ext_driven &= ~(1ULL << n);
        esp32p4_gpio_update(s);
    }
}

static void esp32p4_gpio_set_sig_out(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    if (s->sig_out[n] == (level != 0)) {
        return;
    }
    filters_catch_up(s, esp32_line_time());
    s->sig_out[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_sig_oe(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->sig_oe[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_lp_sig_out(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    if (s->lp_sig_out[n] == (level != 0)) {
        return;
    }
    filters_catch_up(s, esp32_line_time());
    s->lp_sig_out[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_lp_sig_oe(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->lp_sig_oe[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_iomux_out(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->iomux_func_out[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_iomux_oe(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->iomux_func_oe[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_pull_up(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->ext_pull_up = deposit64(s->ext_pull_up, n, 1, level != 0);
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_hold(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->ext_hold = deposit64(s->ext_hold, n, 1, level != 0);
    esp32p4_gpio_update(s);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_gpio_set_hold_all(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    s->hold_all[n] = level != 0;
    esp32p4_gpio_update(s);
}

static void esp32p4_gpio_set_light_sleep(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    filters_catch_up(s, esp32_line_time());
    s->light_sleep = level != 0;
    esp32p4_gpio_update(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.boot]
 * LP_SYSTEM_FORCE_DOWNLOAD_BOOT, unless EFUSE_DIS_FORCE_DOWNLOAD disables
 * it, changes the boot mode of a chip strapped for SPI boot to joint
 * download boot.
 */
static void esp32p4_gpio_set_force_download(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    s->force_download = level;
    esp32p4_gpio_update_boot_mode(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * [spec:nuos:req:emu.esp32p4.etm]
 * An analog comparator's output COMP_OUT, high while its source is above
 * its reference. A rising edge, the source crossing above the reference,
 * latches GPIO_INT_RAW bit 3n and pulses the ETM event
 * GPIO_EVT_ZERO_DET_POS<n>; a falling edge latches bit 3n + 1 and pulses
 * GPIO_EVT_ZERO_DET_NEG<n>; either latches bit 3n + 2, COMP<n>_ALL_INT_RAW.
 * None of this happens within GPIO_ZERO_DET<n>_FILTER_CNT IO MUX operating
 * clock cycles of the comparator's last interrupt. An edge is at its
 * line's time, which the event carries.
 *
 * The sources disagree on which of bits 3n and 3n + 1 is the rising edge.
 * The TRM's GPIO_INT_RAW_REG (Register 10.39) and the field names in
 * ESP-IDF's v3 gpio_reg.h and gpio_struct.h make bit 3n COMP<n>_NEG_INT,
 * which the TRM's Table 66.6-1 raises on a falling COMP_OUT, and bit
 * 3n + 1 COMP<n>_POS_INT, on a rising one. The descriptions of those same
 * fields say the reverse (bit 3n: "analog comparator pos edge interrupt
 * raw"), and so does ESP-IDF's driver, the software that runs on silicon:
 * the ESP32-P4's hal/ana_cmpr_ll.h has ANALOG_CMPR_LL_POS_CROSS_INTR_MASK
 * at bit 3n and _NEG_ at 3n + 1. ESP-IDF changed the masks to that in
 * commit cb7b9850c8 ("Fix swapped POS/NEG cross interrupt masks on
 * ESP32-C5/P4/C61"), which says its test case, which arms one direction,
 * drives the source pad across a 0.5 VDD_IO_6 internal reference and
 * expects the callback for that direction only, failed on ESP32-P4
 * hardware with the bits the names give. The model follows the field
 * descriptions and ESP-IDF's driver. The ETM events raise no such
 * conflict: the TRM's 66.5 and ESP-IDF (ANA_CMPR_EVENT_POS_CROSS, the
 * source becoming higher than the reference, is ZERO_DET_POS<n>) agree,
 * and ESP-IDF's ETM test, which starts a timer on ZERO_DET_NEG<n> and
 * stops it on ZERO_DET_POS<n> around a low pulse of the source, passes on
 * ESP32-P4 hardware (commit 529b6bf97c enabled it there). Neither commit
 * says which chip revision its test ran on, hence the check below.
 *
 * To be confirmed on the ESP32-P4-WIFI6-POE-ETH board: power comparator 1
 * against its internal reference at 0.5 VDD_IO_6 (LP_SYSTEM_REG_PAD_COMP1
 * = 0x15), set GPIO_ZERO_DET1_FILTER_CNT to a few cycles, drive its source
 * pad GPIO54 low, clear GPIO_INT_RAW, drive the pad high and read
 * GPIO_INT_RAW, clear it, drive the pad low and read it again. The model
 * reads 0x28 and then 0x30; the field names would have 0x30 and then
 * 0x28. The chip's own GPIO output can drive the pad, as in ESP-IDF's
 * test, or a supply on the header.
 */
static void esp32p4_gpio_set_ana_comp(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;
    int64_t now = esp32_line_time();
    int64_t quiet;
    qemu_irq etm = NULL;

    if (s->ana_comp[n] == (level != 0)) {
        return;
    }
    filters_catch_up(s, now);
    s->ana_comp[n] = level != 0;
    quiet = esp32p4_clk_count_running(&s->iomux_edges) ?
            esp32p4_clk_cycles_ns(s->iomux_edges.edges.period,
                                  s->zero_det[n]) : 0;
    if (s->ana_comp_int_ns[n] == INT64_MIN ||
        now - s->ana_comp_int_ns[n] >= quiet) {
        s->int_raw |= (level ? INT_COMP_RISE(n) : INT_COMP_FALL(n)) |
                      INT_COMP_ANY(n);
        s->ana_comp_int_ns[n] = now;
        etm = s->etm_zero_det[2 * n + (level ? 0 : 1)];
    }
    esp32p4_gpio_update(s);
    if (etm) {
        etm_pulse(etm, now);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * A GPIO ETM task: set, clear or toggle GPIO_OUT of every pad whose
 * GPIO_EXT_ETM_TASK_GPIOn_EN is set and whose _SEL names the channel, at
 * the event's time, which the task's pulse carries (esp32_line_time()).
 */
static void esp32p4_gpio_etm_task(void *opaque, int line, int level)
{
    ESP32P4GpioState *s = opaque;
    unsigned ch = line / 3, kind = line % 3;
    uint64_t pads = 0, out;

    if (!level) {
        return;
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
        uint32_t cfg = extract32(s->etm_task_cfg[n / 4], (n % 4) * 8, 4);

        if ((cfg & 1) && (cfg >> 1) == ch) {
            pads |= 1ULL << n;
        }
    }
    if (!pads) {
        return;
    }
    filters_catch_up(s, esp32_line_time());
    out = gpio_out_all(s);
    switch (kind) {
    case ESP32P4_GPIO_ETM_SET:
        out |= pads;
        break;
    case ESP32P4_GPIO_ETM_CLEAR:
        out &= ~pads;
        break;
    default:
        out ^= pads;
        break;
    }
    s->out[0] = (uint32_t)out;
    s->out[1] = (uint32_t)(out >> 32) & HIGH_WORD_MASK;
    esp32p4_gpio_update(s);
}

/* Resets */

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The HP GPIO matrix, IO MUX and GPIO_EXT registers take their reset
 * values: every pad at its table 10.14-1 configuration, every output from
 * GPIO_OUT, every input signal bypassing the matrix with its default as
 * its matrix source, filters, modulators and interrupts off.
 */
static void hp_reset(ESP32P4GpioState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->bt_select = 0;
    memset(s->out, 0, sizeof(s->out));
    memset(s->enable, 0, sizeof(s->enable));
    memset(s->status, 0, sizeof(s->status));
    memset(s->pin, 0, sizeof(s->pin));
    for (unsigned sig = 0; sig < ESP32P4_GPIO_SIGS; sig++) {
        s->func_in_sel[sig] = !sig_has_input(sig) ? 0 :
                              sig_in_default(sig) ? IN_SEL_CONST_HIGH
                                                  : IN_SEL_CONST_LOW;
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_SLOTS; n++) {
        s->func_out_sel[n] = FUNC_OUT_RESET;
        s->iomux[n] = iomux_reset_value(n);
    }
    s->clock_gate = 1;
    s->int_raw = 0;
    s->int_ena = INT_BITS;
    for (unsigned n = 0; n < ESP32P4_GPIO_ANA_COMPS; n++) {
        s->zero_det[n] = UINT32_MAX;
        s->ana_comp_int_ns[n] = INT64_MIN;
    }
    s->send_seq = SEND_SEQ_RESET;
    s->bistin_sel = BISTIN_SEL_RESET;
    s->bist_ctrl = BIST_PAD_OE;
    s->date = GPIO_DATE_RESET;
    s->iomux_date = IOMUX_DATE_RESET;
    for (unsigned ch = 0; ch < ESP32P4_GPIO_SDMS; ch++) {
        s->sdm[ch] = SDM_RESET;
        s->sdm_anchor_ns[ch] = now;
        s->sdm_ticks[ch] = 0;
        s->sdm_acc[ch] = 0;
        s->sdm_out[ch] = 0;
    }
    s->sdm_misc = 0;
    memset(s->glitch, 0, sizeof(s->glitch));
    memset(s->etm_event_cfg, 0, sizeof(s->etm_event_cfg));
    memset(s->etm_task_cfg, 0, sizeof(s->etm_task_cfg));
    s->ext_version = EXT_VERSION_RESET;
    memset(s->pin_filter, 0, sizeof(s->pin_filter));
    memset(s->glitch_filter, 0, sizeof(s->glitch_filter));
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The LP GPIO matrix and LP IO MUX registers take their reset values:
 * every LP pad with the HP IO MUX, the LP outputs from LP_GPIO_OUT, the
 * LP input signals bypassing the LP matrix with their defaults as their
 * matrix sources.
 */
static void lp_reset(ESP32P4GpioState *s)
{
    s->lp_clk_en = 1;
    s->lp_ver_date = LP_VER_DATE_RESET;
    s->lp_out = 0;
    s->lp_enable = 0;
    s->lp_status = 0;
    memset(s->lp_pin, 0, sizeof(s->lp_pin));
    memset(s->lp_func_in_sel, 0, sizeof(s->lp_func_in_sel));
    for (unsigned sig = 0; sig < LP_IN_SIGS; sig++) {
        s->lp_func_in_sel[sig] = FIELD_DP32(0, LP_FUNC_IN, IN_SEL,
                                            LP_IN_SEL_CONST |
                                            (lp_sig_in_default(sig) ?
                                             LP_IN_SEL_HIGH : 0));
    }
    for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
        s->lp_func_out_sel[n] = LP_FUNC_OUT_RESET;
        s->lp_pad[n] = FIELD_DP32(0, LP_PAD, DRV, IOMUX_DRV_RESET);
    }
    s->lp_iomux_clk_en = 1;
    s->lp_iomux_ver_date = LPIO_VER_DATE_RESET;
    s->lp_ext_wakeup0_sel = 0;
    s->lp_pad_hold = 0;
    s->lp_pad_hys = 0;
    s->lp_wakeup_edge = 0;
    memset(s->lp_pin_filter, 0, sizeof(s->lp_pin_filter));
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * A block reset line: asserting it resets the block's registers and holds
 * them there, ignoring writes; releasing it lets them run from their
 * reset values.
 */
static void esp32p4_gpio_set_reset(void *opaque, int n, int level)
{
    ESP32P4GpioState *s = opaque;

    if (s->in_reset[n] == (level != 0)) {
        return;
    }
    s->in_reset[n] = level != 0;
    if (level) {
        if (n == ESP32P4_GPIO_RESET_HP) {
            hp_reset(s);
        } else {
            lp_reset(s);
        }
        esp32p4_gpio_update(s);
        notify_route(s);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * A chip reset: both blocks' registers, and GPIO_STRAP latching the
 * strapping pins, GPIO38 to GPIO34 as bits 0 to 4, each from its driver
 * outside the chip if it has one, otherwise from the board's strapping
 * (the strap-mode property). The signals peripherals drive are theirs,
 * not reset with this block, and the pads' holds are taken afresh.
 */
static void esp32p4_gpio_reset_hold(Object *obj, ResetType type)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);

    hp_reset(s);
    lp_reset(s);
    s->strap = 0;
    for (unsigned i = 0; i < ESP32P4_GPIO_STRAP_PINS; i++) {
        unsigned pad = 38 - i;
        bool v = bit64(s->ext_driven, pad) ? bit64(s->ext_level, pad)
                                           : extract32(s->strap_mode, i, 1);

        s->strap |= (uint32_t)v << i;
    }
    s->held = 0;
    s->resync = true;
}

static void esp32p4_gpio_reset_exit(Object *obj, ResetType type)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);

    esp32p4_gpio_update_boot_mode(s);

    /* Interrupt types are all disabled, so this latches no interrupt. */
    esp32p4_gpio_update(s);
    notify_route(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The IO MUX operating clock changes: the filters and modulators run up
 * to now at its old frequency, then count afresh at the new one.
 */
static void esp32p4_gpio_iomux_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4GpioState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        filters_catch_up(s, now);
        filters_advance(s, now);
        sdm_reanchor_all(s, now);
        for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
            s->pin_filter[n].tick = 0;
        }
        for (unsigned ch = 0; ch < ESP32P4_GPIO_GLITCH_FILTERS; ch++) {
            s->glitch_filter[ch].tick = 0;
        }
        esp32p4_clk_count_rebase(&s->iomux_edges, now);
    } else {
        esp32p4_clk_count_start(&s->iomux_edges, s->iomux_clk, now);
        esp32p4_gpio_update(s);
    }
}

static void esp32p4_gpio_lp_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4GpioState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        filters_catch_up(s, now);
        filters_advance(s, now);
        for (unsigned n = 0; n < ESP32P4_GPIO_LP_PADS; n++) {
            s->lp_pin_filter[n].tick = 0;
        }
        esp32p4_clk_count_rebase(&s->lp_edges, now);
    } else {
        esp32p4_clk_count_start(&s->lp_edges, s->lp_clk, now);
        esp32p4_gpio_update(s);
    }
}

/* Properties */

static void esp32p4_gpio_get_mask(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);
    uint64_t value = strcmp(name, "pad-ext-driven") ? s->ext_level
                                                     : s->ext_driven;

    visit_type_uint64(v, name, &value, errp);
}

/* A board's wiring: pads driven from outside, and at what levels. */
static void esp32p4_gpio_set_mask(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);
    uint64_t value;

    if (!visit_type_uint64(v, name, &value, errp)) {
        return;
    }
    if (value & ~PAD_MASK) {
        error_setg(errp, "%s has bits for GPIOs that are not pads", name);
        return;
    }
    if (strcmp(name, "pad-ext-driven")) {
        s->ext_level = value;
    } else {
        s->ext_driven = value;
    }
    if (DEVICE(obj)->realized) {
        esp32p4_gpio_update(s);
    }
}

static char *esp32p4_gpio_get_wires(Object *obj, Error **errp)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);
    GString *str = g_string_new(NULL);

    for (unsigned w = 1; w <= ESP32P4_GPIO_PADS; w++) {
        bool first = true;

        for (unsigned n = 0; n < ESP32P4_GPIO_PADS; n++) {
            if (s->pad_wire[n] != w) {
                continue;
            }
            if (first && str->len) {
                g_string_append_c(str, ',');
            }
            g_string_append_printf(str, first ? "%u" : ":%u", n);
            first = false;
        }
    }
    return g_string_free(str, false);
}

/*
 * [spec:nuos:req:emu.esp32p4.gpio]
 * The board's wires between pads: comma-separated groups of pads joined
 * by colons, each group one wire, e.g. "7:9,8:10".
 */
static void esp32p4_gpio_set_wires(Object *obj, const char *value,
                                   Error **errp)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);
    uint8_t wire[ESP32P4_GPIO_PADS] = { 0 };
    g_auto(GStrv) groups = g_strsplit(value, ",", -1);
    unsigned w = 0;

    for (unsigned g = 0; groups[g]; g++) {
        g_auto(GStrv) pads = g_strsplit(groups[g], ":", -1);
        unsigned count = 0;

        if (!*groups[g]) {
            continue;
        }
        w++;
        for (unsigned i = 0; pads[i]; i++) {
            uint64_t n;

            if (qemu_strtou64(pads[i], NULL, 10, &n) ||
                n >= ESP32P4_GPIO_PADS) {
                error_setg(errp, "pad-wires: '%s' is not a pad", pads[i]);
                return;
            }
            if (wire[n]) {
                error_setg(errp, "pad-wires: GPIO%" PRIu64 " is on two "
                           "wires", n);
                return;
            }
            wire[n] = w;
            count++;
        }
        if (count < 2) {
            error_setg(errp, "pad-wires: a wire joins two pads or more");
            return;
        }
    }
    memcpy(s->pad_wire, wire, sizeof(wire));
    if (DEVICE(obj)->realized) {
        esp32p4_gpio_update(s);
    }
}

static void esp32p4_gpio_init(Object *obj)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);
    static const char *const names[] = {
        "esp32p4.gpio", "esp32p4.io-mux", "esp32p4.lp-gpio",
        "esp32p4.lp-iomux",
    };
    static const uint64_t sizes[] = {
        ESP32P4_GPIO_SIZE, ESP32P4_IO_MUX_SIZE, ESP32P4_LP_GPIO_SIZE,
        ESP32P4_LP_IOMUX_SIZE,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(s->iomem); i++) {
        s->region[i].s = s;
        s->region[i].block = i;
        memory_region_init_io(&s->iomem[i], obj, &esp32p4_gpio_ops,
                              &s->region[i], names[i], sizes[i]);
        sysbus_init_mmio(sbd, &s->iomem[i]);
    }
    for (unsigned i = 0; i < ESP32P4_GPIO_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    s->iomux_clk = qdev_init_clock_in(dev, "iomux",
                                      esp32p4_gpio_iomux_clk_update, s,
                                      ClockPreUpdate | ClockUpdate);
    s->lp_clk = qdev_init_clock_in(dev, "lp", esp32p4_gpio_lp_clk_update, s,
                                   ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->filter_timer, QEMU_CLOCK_VIRTUAL,
                  esp32p4_gpio_filter_timer, s);
    notifier_list_init(&s->route_notifiers);
    s->line_source.notify = esp32p4_gpio_line_sync;

    qdev_init_gpio_in_named(dev, esp32p4_gpio_pad_in, ESP32P4_GPIO_PAD_IN,
                            ESP32P4_GPIO_PADS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_pad_release,
                            ESP32P4_GPIO_PAD_RELEASE, ESP32P4_GPIO_PADS);
    qdev_init_gpio_out_named(dev, s->pad_out, ESP32P4_GPIO_PAD_OUT,
                             ESP32P4_GPIO_PADS);
    qdev_init_gpio_out_named(dev, s->drive_high_out,
                             ESP32P4_GPIO_PAD_DRIVE_HIGH, ESP32P4_GPIO_PADS);
    qdev_init_gpio_out_named(dev, s->drive_low_out,
                             ESP32P4_GPIO_PAD_DRIVE_LOW, ESP32P4_GPIO_PADS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_sig_out,
                            ESP32P4_GPIO_SIG_OUT, ESP32P4_GPIO_SIGS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_sig_oe,
                            ESP32P4_GPIO_SIG_OE, ESP32P4_GPIO_SIGS);
    qdev_init_gpio_out_named(dev, s->sig_in_out, ESP32P4_GPIO_SIG_IN,
                             ESP32P4_GPIO_SIGS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_lp_sig_out,
                            ESP32P4_GPIO_LP_SIG_OUT, ESP32P4_GPIO_LP_SIGS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_lp_sig_oe,
                            ESP32P4_GPIO_LP_SIG_OE, ESP32P4_GPIO_LP_SIGS);
    qdev_init_gpio_out_named(dev, s->lp_sig_in_out, ESP32P4_GPIO_LP_SIG_IN,
                             ESP32P4_GPIO_LP_SIGS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_iomux_out,
                            ESP32P4_GPIO_IOMUX_OUT, ESP32P4_GPIO_IOMUX_LINES);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_iomux_oe,
                            ESP32P4_GPIO_IOMUX_OE, ESP32P4_GPIO_IOMUX_LINES);
    qdev_init_gpio_out_named(dev, s->iomux_in_out, ESP32P4_GPIO_IOMUX_IN,
                             ESP32P4_GPIO_IOMUX_LINES);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_reset, ESP32P4_GPIO_RESET,
                            ESP32P4_GPIO_RESETS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_pull_up,
                            ESP32P4_GPIO_PAD_PULL_UP, ESP32P4_GPIO_PADS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_hold, ESP32P4_GPIO_PAD_HOLD,
                            ESP32P4_GPIO_PADS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_light_sleep,
                            ESP32P4_GPIO_LIGHT_SLEEP, 1);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_hold_all,
                            ESP32P4_GPIO_HOLD_ALL, ESP32P4_GPIO_HOLD_ALLS);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_ana_comp,
                            ESP32P4_GPIO_ANA_COMP, ESP32P4_GPIO_ANA_COMPS);
    qdev_init_gpio_out_named(dev, &s->wakeup_out, ESP32P4_GPIO_WAKEUP, 1);
    qdev_init_gpio_out_named(dev, &s->lp_wakeup_out, ESP32P4_GPIO_LP_WAKEUP,
                             1);
    qdev_init_gpio_out_named(dev, s->etm_event, ESP32P4_GPIO_ETM_EVENT,
                             ESP32P4_GPIO_ETM_LINES);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_etm_task, ESP32P4_GPIO_ETM_TASK,
                            ESP32P4_GPIO_ETM_LINES);
    qdev_init_gpio_out_named(dev, s->etm_zero_det, ESP32P4_GPIO_ETM_ZERO_DET,
                             ESP32P4_GPIO_ETM_ZERO_DET_LINES);
    qdev_init_gpio_in_named(dev, esp32p4_gpio_set_force_download,
                            ESP32P4_GPIO_FORCE_DOWNLOAD, 1);
    qdev_init_gpio_out_named(dev, &s->spi_boot_out, ESP32P4_GPIO_SPI_BOOT, 1);

    /*
     * The signals peripherals drive are theirs, not reset with this block:
     * they start at the level the peripherals hold out of reset.
     */
    for (unsigned sig = 0; sig < ESP32P4_GPIO_SIGS; sig++) {
        s->sig_out[sig] = sig_out_idle(sig);
        s->sig_oe[sig] = 1;
    }
    for (unsigned sig = 0; sig < ESP32P4_GPIO_LP_SIGS; sig++) {
        s->lp_sig_out[sig] = lp_sig_out_idle(sig);
        s->lp_sig_oe[sig] = 1;
    }
    /* The USB Serial/JTAG PHY's D+ pull-up, on out of reset */
    s->ext_pull_up = 1ULL << 25;
}

static void esp32p4_gpio_realize(DeviceState *dev, Error **errp)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(dev);

    if (s->strap_mode >> ESP32P4_GPIO_STRAP_PINS) {
        error_setg(errp, "strap-mode 0x%x has bits beyond GPIO34 to GPIO38's "
                   "five", s->strap_mode);
        return;
    }
    esp32_line_add_source(&s->line_source);
}

static void esp32p4_gpio_unrealize(DeviceState *dev)
{
    ESP32P4GpioState *s = ESP32P4_GPIO(dev);

    notifier_remove(&s->line_source);
    timer_del(&s->filter_timer);
}

static const Property esp32p4_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap-mode", ESP32P4GpioState, strap_mode,
                       ESP32P4_GPIO_STRAP_SPI_BOOT),
};

static const VMStateDescription vmstate_esp32p4_gpio_filter = {
    .name = TYPE_ESP32P4_GPIO "/filter",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(in, ESP32P4GpioFilter),
        VMSTATE_BOOL(out, ESP32P4GpioFilter),
        VMSTATE_UINT64(window, ESP32P4GpioFilter),
        VMSTATE_UINT64(tick, ESP32P4GpioFilter),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_gpio = {
    .name = TYPE_ESP32P4_GPIO,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(iomux_clk, ESP32P4GpioState),
        VMSTATE_CLOCK(lp_clk, ESP32P4GpioState),
        VMSTATE_TIMER(filter_timer, ESP32P4GpioState),
        VMSTATE_UINT32(bt_select, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(out, ESP32P4GpioState, 2),
        VMSTATE_UINT32_ARRAY(enable, ESP32P4GpioState, 2),
        VMSTATE_UINT32(strap, ESP32P4GpioState),
        VMSTATE_BOOL(force_download, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(status, ESP32P4GpioState, 2),
        VMSTATE_UINT32_ARRAY(pin, ESP32P4GpioState, ESP32P4_GPIO_SLOTS),
        VMSTATE_UINT32_ARRAY(func_in_sel, ESP32P4GpioState,
                             ESP32P4_GPIO_SIGS),
        VMSTATE_UINT32_ARRAY(func_out_sel, ESP32P4GpioState,
                             ESP32P4_GPIO_SLOTS),
        VMSTATE_UINT32(clock_gate, ESP32P4GpioState),
        VMSTATE_UINT32(int_raw, ESP32P4GpioState),
        VMSTATE_UINT32(int_ena, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(zero_det, ESP32P4GpioState,
                             ESP32P4_GPIO_ANA_COMPS),
        VMSTATE_UINT32(send_seq, ESP32P4GpioState),
        VMSTATE_UINT32(bistin_sel, ESP32P4GpioState),
        VMSTATE_UINT32(bist_ctrl, ESP32P4GpioState),
        VMSTATE_UINT32(date, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(iomux, ESP32P4GpioState, ESP32P4_GPIO_SLOTS),
        VMSTATE_UINT32(iomux_date, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(sdm, ESP32P4GpioState, ESP32P4_GPIO_SDMS),
        VMSTATE_UINT32(sdm_misc, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(glitch, ESP32P4GpioState,
                             ESP32P4_GPIO_GLITCH_FILTERS),
        VMSTATE_UINT32_ARRAY(etm_event_cfg, ESP32P4GpioState,
                             ESP32P4_GPIO_ETM_CHANNELS),
        VMSTATE_UINT32_ARRAY(etm_task_cfg, ESP32P4GpioState, 14),
        VMSTATE_UINT32(ext_version, ESP32P4GpioState),
        VMSTATE_UINT32(lp_clk_en, ESP32P4GpioState),
        VMSTATE_UINT32(lp_ver_date, ESP32P4GpioState),
        VMSTATE_UINT32(lp_out, ESP32P4GpioState),
        VMSTATE_UINT32(lp_enable, ESP32P4GpioState),
        VMSTATE_UINT32(lp_status, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(lp_pin, ESP32P4GpioState, ESP32P4_GPIO_LP_PADS),
        VMSTATE_UINT32_ARRAY(lp_func_in_sel, ESP32P4GpioState,
                             ESP32P4_GPIO_LP_SIGS),
        VMSTATE_UINT32_ARRAY(lp_func_out_sel, ESP32P4GpioState,
                             ESP32P4_GPIO_LP_PADS),
        VMSTATE_UINT32(lp_iomux_clk_en, ESP32P4GpioState),
        VMSTATE_UINT32(lp_iomux_ver_date, ESP32P4GpioState),
        VMSTATE_UINT32_ARRAY(lp_pad, ESP32P4GpioState, ESP32P4_GPIO_LP_PADS),
        VMSTATE_UINT32(lp_ext_wakeup0_sel, ESP32P4GpioState),
        VMSTATE_UINT32(lp_pad_hold, ESP32P4GpioState),
        VMSTATE_UINT32(lp_pad_hys, ESP32P4GpioState),
        VMSTATE_BOOL_ARRAY(in_reset, ESP32P4GpioState, ESP32P4_GPIO_RESETS),
        VMSTATE_INT64_ARRAY(sdm_anchor_ns, ESP32P4GpioState,
                            ESP32P4_GPIO_SDMS),
        VMSTATE_UINT64_ARRAY(sdm_ticks, ESP32P4GpioState, ESP32P4_GPIO_SDMS),
        VMSTATE_UINT32_ARRAY(sdm_acc, ESP32P4GpioState, ESP32P4_GPIO_SDMS),
        VMSTATE_UINT8_ARRAY(sdm_out, ESP32P4GpioState, ESP32P4_GPIO_SDMS),
        VMSTATE_STRUCT_ARRAY(pin_filter, ESP32P4GpioState, ESP32P4_GPIO_PADS,
                             1, vmstate_esp32p4_gpio_filter,
                             ESP32P4GpioFilter),
        VMSTATE_STRUCT_ARRAY(glitch_filter, ESP32P4GpioState,
                             ESP32P4_GPIO_GLITCH_FILTERS, 1,
                             vmstate_esp32p4_gpio_filter, ESP32P4GpioFilter),
        VMSTATE_STRUCT_ARRAY(lp_pin_filter, ESP32P4GpioState,
                             ESP32P4_GPIO_LP_PADS, 1,
                             vmstate_esp32p4_gpio_filter, ESP32P4GpioFilter),
        VMSTATE_ESP32P4_CLK_COUNT(iomux_edges, ESP32P4GpioState),
        VMSTATE_ESP32P4_CLK_COUNT(lp_edges, ESP32P4GpioState),
        VMSTATE_UINT8_ARRAY(ana_comp, ESP32P4GpioState,
                            ESP32P4_GPIO_ANA_COMPS),
        VMSTATE_INT64_ARRAY(ana_comp_int_ns, ESP32P4GpioState,
                            ESP32P4_GPIO_ANA_COMPS),
        VMSTATE_UINT64(ext_driven, ESP32P4GpioState),
        VMSTATE_UINT64(ext_level, ESP32P4GpioState),
        VMSTATE_UINT64(ext_pull_up, ESP32P4GpioState),
        VMSTATE_UINT8_ARRAY(pad_wire, ESP32P4GpioState, ESP32P4_GPIO_PADS),
        VMSTATE_UINT64(ext_hold, ESP32P4GpioState),
        VMSTATE_BOOL_ARRAY(hold_all, ESP32P4GpioState,
                           ESP32P4_GPIO_HOLD_ALLS),
        VMSTATE_BOOL(light_sleep, ESP32P4GpioState),
        VMSTATE_UINT8_ARRAY(sig_out, ESP32P4GpioState, ESP32P4_GPIO_SIGS),
        VMSTATE_UINT8_ARRAY(sig_oe, ESP32P4GpioState, ESP32P4_GPIO_SIGS),
        VMSTATE_UINT8_ARRAY(lp_sig_out, ESP32P4GpioState,
                            ESP32P4_GPIO_LP_SIGS),
        VMSTATE_UINT8_ARRAY(lp_sig_oe, ESP32P4GpioState,
                            ESP32P4_GPIO_LP_SIGS),
        VMSTATE_UINT8_ARRAY(iomux_func_out, ESP32P4GpioState,
                            ESP32P4_GPIO_IOMUX_LINES),
        VMSTATE_UINT8_ARRAY(iomux_func_oe, ESP32P4GpioState,
                            ESP32P4_GPIO_IOMUX_LINES),
        VMSTATE_UINT64(held, ESP32P4GpioState),
        VMSTATE_UINT64(hold_val, ESP32P4GpioState),
        VMSTATE_UINT64(hold_oe, ESP32P4GpioState),
        VMSTATE_UINT64(hold_pu, ESP32P4GpioState),
        VMSTATE_UINT64(hold_ie, ESP32P4GpioState),
        VMSTATE_UINT32(lp_wakeup_edge, ESP32P4GpioState),
        VMSTATE_UINT64(pad_level, ESP32P4GpioState),
        VMSTATE_UINT64(drive_high, ESP32P4GpioState),
        VMSTATE_UINT64(drive_low, ESP32P4GpioState),
        VMSTATE_UINT64(raw_in, ESP32P4GpioState),
        VMSTATE_UINT64(pf_in, ESP32P4GpioState),
        VMSTATE_UINT64(hp_in, ESP32P4GpioState),
        VMSTATE_UINT32(lp_in, ESP32P4GpioState),
        VMSTATE_UINT8_ARRAY(sig_in, ESP32P4GpioState, ESP32P4_GPIO_SIGS),
        VMSTATE_UINT8_ARRAY(lp_sig_in, ESP32P4GpioState,
                            ESP32P4_GPIO_LP_SIGS),
        VMSTATE_UINT8_ARRAY(iomux_in, ESP32P4GpioState,
                            ESP32P4_GPIO_IOMUX_LINES),
        VMSTATE_UINT8_ARRAY(irq_level, ESP32P4GpioState, ESP32P4_GPIO_IRQS),
        VMSTATE_BOOL(wakeup_level, ESP32P4GpioState),
        VMSTATE_BOOL(lp_wakeup_level, ESP32P4GpioState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_gpio_reset_hold;
    rc->phases.exit = esp32p4_gpio_reset_exit;
    dc->realize = esp32p4_gpio_realize;
    dc->unrealize = esp32p4_gpio_unrealize;
    dc->vmsd = &vmstate_esp32p4_gpio;
    device_class_set_props(dc, esp32p4_gpio_properties);
    object_class_property_add(klass, "pad-ext-driven", "uint64",
                              esp32p4_gpio_get_mask, esp32p4_gpio_set_mask,
                              NULL, NULL);
    object_class_property_set_description(klass, "pad-ext-driven",
        "The pads driven from outside the chip, bit n for GPIOn");
    object_class_property_add(klass, "pad-ext-level", "uint64",
                              esp32p4_gpio_get_mask, esp32p4_gpio_set_mask,
                              NULL, NULL);
    object_class_property_set_description(klass, "pad-ext-level",
        "The levels outside drivers hold the pads at, bit n for GPIOn");
    object_class_property_add_str(klass, "pad-wires",
                                  esp32p4_gpio_get_wires,
                                  esp32p4_gpio_set_wires);
    object_class_property_set_description(klass, "pad-wires",
        "Pads the board wires together: groups of GPIO numbers joined by "
        "':', separated by ',' (e.g. \"7:9,8:10\")");
}

/* [spec:nuos:req:emu.esp32p4.gpio] */
static const TypeInfo esp32p4_gpio_info = {
    .name = TYPE_ESP32P4_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4GpioState),
    .instance_init = esp32p4_gpio_init,
    .class_init = esp32p4_gpio_class_init,
};

static void esp32p4_gpio_register_types(void)
{
    type_register_static(&esp32p4_gpio_info);
}

type_init(esp32p4_gpio_register_types)
