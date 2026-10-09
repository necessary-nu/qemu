/*
 * RP2350 GPIO muxing and pads (IO_BANK0, IO_QSPI, PADS_BANK0, PADS_QSPI)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "GPIO". Function select tables follow the
 * pico-sdk io_bank0.h and io_qspi.h FUNCSEL values.
 *
 * Each pin is modelled as the datasheet's chain:
 *
 *   FUNCSEL source -> OUTOVER/OEOVER -> pad (OD, isolation latches) -> pin
 *   pin -> pad IE -> INOVER -> SIO, PIO and the selected peripheral input
 *                 -> IRQOVER -> level/edge detection -> INTR, INTE/INTF/INTS
 *
 * Pad drive strength, slew rate, the Schmitt trigger and VOLTAGE_SELECT
 * have no logical effect and are register storage only.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/rp2350_gpio.h"
#include "hw/misc/rp2350_atomic.h"
#include "migration/vmstate.h"

#define STATUS_OUTTOPAD  (1u << 9)
#define STATUS_OETOPAD   (1u << 13)
#define STATUS_INFROMPAD (1u << 17)
#define STATUS_IRQTOPROC (1u << 26)

#define CTRL_MASK        0x3003f01fu
#define CTRL_RESET       0x1fu
#define CTRL_FUNCSEL(v)  ((v) & 0x1f)
#define CTRL_OUTOVER(v)  extract32(v, 12, 2)
#define CTRL_OEOVER(v)   extract32(v, 14, 2)
#define CTRL_INOVER(v)   extract32(v, 16, 2)
#define CTRL_IRQOVER(v)  extract32(v, 28, 2)
#define FUNCSEL_SIO      5
#define FUNCSEL_NULL     0x1f

/* Pad control registers */
#define PAD_SLEWFAST     (1u << 0)
#define PAD_SCHMITT      (1u << 1)
#define PAD_PDE          (1u << 2)
#define PAD_PUE          (1u << 3)
#define PAD_DRIVE_4MA    (1u << 4)
#define PAD_IE           (1u << 6)
#define PAD_OD           (1u << 7)
#define PAD_ISO          (1u << 8)
#define PAD_MASK         0x1ffu
#define PAD_BANK0_RESET  (PAD_ISO | PAD_DRIVE_4MA | PAD_PDE | PAD_SCHMITT)
#define PAD_QSPI_PD_RESET (PAD_ISO | PAD_IE | PAD_DRIVE_4MA | PAD_PDE | \
                           PAD_SCHMITT)
#define PAD_QSPI_PU_RESET (PAD_ISO | PAD_IE | PAD_DRIVE_4MA | PAD_PUE | \
                           PAD_SCHMITT)
#define PAD_SWD_RESET    (PAD_IE | PAD_DRIVE_4MA | PAD_PUE | PAD_SCHMITT)

#define A_PADS_VOLTAGE_SELECT 0x00
#define A_PADS_BANK0_GPIO0    0x04
#define A_PADS_SWCLK          0xc4
#define A_PADS_SWD            0xc8

/* Pad isolation latch contents */
#define LATCH_OE  (1u << 0)
#define LATCH_OUT (1u << 1)
#define LATCH_IE  (1u << 2)
#define LATCH_PUE (1u << 3)
#define LATCH_PDE (1u << 4)

/* Interrupt bits per pin, in each 4-bit field. */
#define INT_LEVEL_LOW  (1u << 0)
#define INT_LEVEL_HIGH (1u << 1)
#define INT_EDGE_LOW   (1u << 2)
#define INT_EDGE_HIGH  (1u << 3)
#define INT_EDGES      0xccccccccu

#define IRQSUMMARY_BASE 0x200

/*
 * The two IO register blocks share a layout: STATUS/CTRL pairs per pin,
 * then the interrupt summary, INTR, and per destination INTE, INTF and
 * INTS arrays.
 */
typedef struct RP2350GPIOBank {
    const char *name;
    int first_pin;
    int pins;
    /* Interrupt register words, and IRQSUMMARY words per summary. */
    int words;
    int summary_words;
    hwaddr intr;
    hwaddr dest_base;
    hwaddr dest_stride;
} RP2350GPIOBank;

static const RP2350GPIOBank rp2350_gpio_banks[2] = {
    {
        .name = "rp2350-io-bank0",
        .first_pin = 0,
        .pins = RP2350_GPIO_BANK0_PINS,
        .words = RP2350_GPIO_BANK0_PINS / 8,
        .summary_words = 2,
        .intr = 0x230,
        .dest_base = 0x248,
        .dest_stride = 0x48,
    },
    {
        .name = "rp2350-io-qspi",
        .first_pin = RP2350_GPIO_BANK0_PINS,
        .pins = RP2350_GPIO_QSPI_PINS,
        .words = 1,
        .summary_words = 1,
        .intr = 0x218,
        .dest_base = 0x21c,
        .dest_stride = 0x0c,
    },
};

/* PADS_QSPI register order differs from IO_QSPI's. */
static const int rp2350_qspi_pad_pin[] = {
    RP2350_GPIO_QSPI_SCLK, RP2350_GPIO_QSPI_SD0, RP2350_GPIO_QSPI_SD1,
    RP2350_GPIO_QSPI_SD2, RP2350_GPIO_QSPI_SD3, RP2350_GPIO_QSPI_SS,
};

static const struct {
    const char *name;
    int signals;
} rp2350_gpio_ports[RP2350_GPIO_NUM_PORTS] = {
    [RP2350_GPIO_PORT_SPI0] = { "spi0", RP2350_GPIO_SPI_SIGNALS },
    [RP2350_GPIO_PORT_SPI1] = { "spi1", RP2350_GPIO_SPI_SIGNALS },
    [RP2350_GPIO_PORT_UART0] = { "uart0", RP2350_GPIO_UART_SIGNALS },
    [RP2350_GPIO_PORT_UART1] = { "uart1", RP2350_GPIO_UART_SIGNALS },
    [RP2350_GPIO_PORT_I2C0] = { "i2c0", RP2350_GPIO_I2C_SIGNALS },
    [RP2350_GPIO_PORT_I2C1] = { "i2c1", RP2350_GPIO_I2C_SIGNALS },
    [RP2350_GPIO_PORT_PWM] = { "pwm", RP2350_GPIO_PWM_SIGNALS },
    [RP2350_GPIO_PORT_SIO] = { "sio", RP2350_GPIO_SIO_BITS },
    [RP2350_GPIO_PORT_PIO0] = { "pio0", RP2350_GPIO_PIO_SIGNALS },
    [RP2350_GPIO_PORT_PIO1] = { "pio1", RP2350_GPIO_PIO_SIGNALS },
    [RP2350_GPIO_PORT_PIO2] = { "pio2", RP2350_GPIO_PIO_SIGNALS },
    [RP2350_GPIO_PORT_HSTX] = { "hstx", RP2350_GPIO_HSTX_SIGNALS },
    [RP2350_GPIO_PORT_QMI] = { "qmi", RP2350_GPIO_QMI_SIGNALS },
    [RP2350_GPIO_PORT_JTAG] = { "jtag", RP2350_GPIO_JTAG_SIGNALS },
    [RP2350_GPIO_PORT_TRACE] = { "trace", RP2350_GPIO_TRACE_SIGNALS },
    [RP2350_GPIO_PORT_USB] = { "usb", RP2350_GPIO_USB_SIGNALS },
    [RP2350_GPIO_PORT_CLOCKS] = { "clocks", RP2350_GPIO_CLOCKS_SIGNALS },
};

/* First signal of each port in the flat signal numbering. */
static int rp2350_gpio_port_base[RP2350_GPIO_NUM_PORTS];

/* The signal each pin's FUNCSEL value selects, or -1 for none. */
static int16_t rp2350_gpio_fn[RP2350_GPIO_PINS][32];

const char *rp2350_gpio_port_name(RP2350GPIOPort port)
{
    return rp2350_gpio_ports[port].name;
}

int rp2350_gpio_port_signals(RP2350GPIOPort port)
{
    return rp2350_gpio_ports[port].signals;
}

static int sig(RP2350GPIOPort port, int n)
{
    assert(n >= 0 && n < rp2350_gpio_ports[port].signals);
    return rp2350_gpio_port_base[port] + n;
}

/* The bank 0 function select table (datasheet "Function select"). */
/* [spec:nuos:req:emu.gpio] */
static void rp2350_gpio_bank0_functions(int p, int16_t *fn)
{
    static const struct {
        int pin;
        RP2350GPIOPort port;
        int n;
    } f9[] = {
        { 0, RP2350_GPIO_PORT_QMI, RP2350_GPIO_QMI_CS1N },
        { 1, RP2350_GPIO_PORT_TRACE, RP2350_GPIO_TRACE_CLK },
        { 2, RP2350_GPIO_PORT_TRACE, RP2350_GPIO_TRACE_DATA0 },
        { 3, RP2350_GPIO_PORT_TRACE, RP2350_GPIO_TRACE_DATA1 },
        { 4, RP2350_GPIO_PORT_TRACE, RP2350_GPIO_TRACE_DATA2 },
        { 5, RP2350_GPIO_PORT_TRACE, RP2350_GPIO_TRACE_DATA3 },
        { 8, RP2350_GPIO_PORT_QMI, RP2350_GPIO_QMI_CS1N },
        { 12, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPIN0 },
        { 13, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPOUT0 },
        { 14, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPIN1 },
        { 15, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPOUT1 },
        { 19, RP2350_GPIO_PORT_QMI, RP2350_GPIO_QMI_CS1N },
        { 20, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPIN0 },
        { 21, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPOUT0 },
        { 22, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPIN1 },
        { 23, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPOUT1 },
        { 24, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPOUT2 },
        { 25, RP2350_GPIO_PORT_CLOCKS, RP2350_GPIO_CLOCKS_GPOUT3 },
        { 47, RP2350_GPIO_PORT_QMI, RP2350_GPIO_QMI_CS1N },
    };
    RP2350GPIOPort uart = ((p + 4) / 8) % 2 ? RP2350_GPIO_PORT_UART1
                                            : RP2350_GPIO_PORT_UART0;
    int slice = p < 32 ? (p / 2) % 8 : 8 + ((p - 32) / 2) % 4;
    int i;

    if (p < 4) {
        fn[0] = sig(RP2350_GPIO_PORT_JTAG, p);
    } else if (p >= RP2350_GPIO_HSTX_FIRST_PIN &&
               p < RP2350_GPIO_HSTX_FIRST_PIN + RP2350_GPIO_HSTX_SIGNALS) {
        fn[0] = sig(RP2350_GPIO_PORT_HSTX, p - RP2350_GPIO_HSTX_FIRST_PIN);
    }
    fn[1] = sig((p / 8) % 2 ? RP2350_GPIO_PORT_SPI1 : RP2350_GPIO_PORT_SPI0,
                p % 4);
    fn[2] = sig(uart, p % 4);
    fn[3] = sig((p / 2) % 2 ? RP2350_GPIO_PORT_I2C1 : RP2350_GPIO_PORT_I2C0,
                p % 2);
    fn[4] = sig(RP2350_GPIO_PORT_PWM, RP2350_GPIO_PWM(slice, p % 2));
    fn[5] = sig(RP2350_GPIO_PORT_SIO, p);
    fn[6] = sig(RP2350_GPIO_PORT_PIO0, p);
    fn[7] = sig(RP2350_GPIO_PORT_PIO1, p);
    fn[8] = sig(RP2350_GPIO_PORT_PIO2, p);
    for (i = 0; i < ARRAY_SIZE(f9); i++) {
        if (f9[i].pin == p) {
            fn[9] = sig(f9[i].port, f9[i].n);
        }
    }
    /* USB overcurrent detect, VBUS detect and VBUS enable, in rotation. */
    fn[10] = sig(RP2350_GPIO_PORT_USB, p % 3);
    /* The alternative UART TX and RX on the CTS and RTS pins. */
    if (p % 4 >= 2) {
        fn[11] = sig(uart, p % 4 == 2 ? RP2350_GPIO_UART_TX
                                      : RP2350_GPIO_UART_RX);
    }
}

/* The QSPI bank function select table; q is the IO_QSPI pin index. */
/* [spec:nuos:req:emu.gpio] */
static void rp2350_gpio_qspi_functions(int q, int16_t *fn)
{
    RP2350GPIOPort uart = q < 4 ? RP2350_GPIO_PORT_UART1
                                : RP2350_GPIO_PORT_UART0;

    /* SCLK, SS, SD0-3 map onto the QMI's SCK, CS0N, SD0-3. */
    if (q >= 2) {
        fn[0] = sig(RP2350_GPIO_PORT_QMI, q - 2);
    }
    fn[2] = sig(uart, q % 4);
    fn[3] = sig((q / 2) % 2 ? RP2350_GPIO_PORT_I2C1 : RP2350_GPIO_PORT_I2C0,
                q % 2);
    fn[5] = sig(RP2350_GPIO_PORT_SIO,
                rp2350_gpio_sio_bit(RP2350_GPIO_BANK0_PINS + q));
    if (q % 4 >= 2) {
        fn[11] = sig(uart, q % 4 == 2 ? RP2350_GPIO_UART_TX
                                      : RP2350_GPIO_UART_RX);
    }
}

static void rp2350_gpio_init_tables(void)
{
    int port, base = 0, p, f;

    for (port = 0; port < RP2350_GPIO_NUM_PORTS; port++) {
        rp2350_gpio_port_base[port] = base;
        base += rp2350_gpio_ports[port].signals;
    }
    assert(base == RP2350_GPIO_SIGNALS);

    for (p = 0; p < RP2350_GPIO_PINS; p++) {
        for (f = 0; f < 32; f++) {
            rp2350_gpio_fn[p][f] = -1;
        }
        if (p < RP2350_GPIO_BANK0_PINS) {
            rp2350_gpio_bank0_functions(p, rp2350_gpio_fn[p]);
        } else {
            rp2350_gpio_qspi_functions(p - RP2350_GPIO_BANK0_PINS,
                                       rp2350_gpio_fn[p]);
        }
    }
}

qemu_irq rp2350_gpio_out_line(RP2350GPIOState *s, RP2350GPIOPort port, int n)
{
    g_autofree char *name = g_strdup_printf("%s-out",
                                            rp2350_gpio_port_name(port));

    return qdev_get_gpio_in_named(DEVICE(s), name, n);
}

qemu_irq rp2350_gpio_oe_line(RP2350GPIOState *s, RP2350GPIOPort port, int n)
{
    g_autofree char *name = g_strdup_printf("%s-oe",
                                            rp2350_gpio_port_name(port));

    return qdev_get_gpio_in_named(DEVICE(s), name, n);
}

void rp2350_gpio_connect_in(RP2350GPIOState *s, RP2350GPIOPort port, int n,
                            qemu_irq irq)
{
    g_autofree char *name = g_strdup_printf("%s-in",
                                            rp2350_gpio_port_name(port));

    qdev_connect_gpio_out_named(DEVICE(s), name, n, irq);
}

/* CTRL override fields: 0 pass, 1 invert, 2 force low, 3 force high. */
static int over(int mode, int v)
{
    switch (mode) {
    case 0:
        return v;
    case 1:
        return !v;
    case 2:
        return 0;
    default:
        return 1;
    }
}

static bool pin_has_pad_reg(int p)
{
    return p != RP2350_GPIO_USB_DP && p != RP2350_GPIO_USB_DM;
}

static bool pin_bonded(RP2350GPIOState *s, int p)
{
    return p >= RP2350_GPIO_BANK0_PINS || p < s->bonded_gpios;
}

/*
 * The pad controls the IO muxing presents to pin p. The USB DP/DM pins
 * have no PADS register; as GPIOs their input is always enabled and they
 * have no pulls or isolation.
 */
static uint8_t pad_controls(RP2350GPIOState *s, int p, int out, int oe)
{
    uint32_t pad = pin_has_pad_reg(p) ? s->pad[p] : PAD_IE;
    uint8_t c = 0;

    if (oe && !(pad & PAD_OD)) {
        c |= LATCH_OE;
    }
    if (out) {
        c |= LATCH_OUT;
    }
    if (pad & PAD_IE) {
        c |= LATCH_IE;
    }
    if (pad & PAD_PUE) {
        c |= LATCH_PUE;
    }
    if (pad & PAD_PDE) {
        c |= LATCH_PDE;
    }
    return c;
}

static bool pad_isolated(RP2350GPIOState *s, int p)
{
    return pin_has_pad_reg(p) && (s->pad[p] & PAD_ISO);
}

/*
 * The level on the pin. The chip's own driver wins over an external one;
 * an undriven pin follows its pull, and with both pulls (bus keeper) or
 * neither (floating) it holds its last level.
 */
static int pad_level(RP2350GPIOState *s, int p, uint8_t c)
{
    if (c & LATCH_OE) {
        return !!(c & LATCH_OUT);
    }
    if (s->ext[p] != RP2350_GPIO_EXT_NONE && pin_bonded(s, p)) {
        return s->ext[p];
    }
    switch (c & (LATCH_PUE | LATCH_PDE)) {
    case LATCH_PUE:
        return 1;
    case LATCH_PDE:
        return 0;
    default:
        return s->pad_level[p];
    }
}

static bool pin_ns(RP2350GPIOState *s, int p)
{
    return (s->nsmask >> rp2350_gpio_sio_bit(p)) & 1;
}

static void rp2350_gpio_set_irqs(RP2350GPIOState *s)
{
    bool pending[RP2350_GPIO_CORES][RP2350_GPIO_CORE_IRQS] = {};
    bool wake = false;
    int p, d;

    for (p = 0; p < RP2350_GPIO_PINS; p++) {
        int w = p / 8, shift = 4 * (p % 8);
        uint32_t intr = s->intr_edge[w] |
                        (s->irq_level[p] ? INT_LEVEL_HIGH : INT_LEVEL_LOW)
                        << shift;
        int line = (p < RP2350_GPIO_BANK0_PINS ? RP2350_GPIO_IRQ_BANK0
                                               : RP2350_GPIO_IRQ_QSPI) +
                   pin_ns(s, p);

        for (d = 0; d < RP2350_GPIO_DESTS; d++) {
            uint32_t ints = (intr | s->intf[d][w]) & s->inte[d][w];

            if (!((ints >> shift) & 0xf)) {
                continue;
            }
            if (d == RP2350_GPIO_DEST_DORMANT_WAKE) {
                wake = true;
            } else {
                pending[d][line] = true;
            }
        }
    }

    for (d = 0; d < RP2350_GPIO_CORES; d++) {
        for (p = 0; p < RP2350_GPIO_CORE_IRQS; p++) {
            qemu_set_irq(s->irq[d][p], pending[d][p]);
        }
    }
    qemu_set_irq(s->dormant_wake, wake);
}

/*
 * Propagate every pin: outputs through the muxing and pads, pad levels
 * back to the peripherals, and interrupt detection. With `force`, every
 * peripheral input is delivered whether or not it changed.
 */
/* [spec:nuos:req:emu.gpio] */
static void rp2350_gpio_propagate(RP2350GPIOState *s, bool force)
{
    uint8_t in[RP2350_GPIO_SIGNALS] = {};
    uint8_t to_peri[RP2350_GPIO_PINS];
    int p, i, port;

    for (p = 0; p < RP2350_GPIO_PINS; p++) {
        uint32_t ctrl = s->ctrl[p];
        int f = rp2350_gpio_fn[p][CTRL_FUNCSEL(ctrl)];
        int out = f >= 0 ? s->src_out[f] : 0;
        int oe = f >= 0 ? s->src_oe[f] : 0;
        uint8_t c;
        int level, irq;

        out = over(CTRL_OUTOVER(ctrl), out);
        oe = over(CTRL_OEOVER(ctrl), oe);
        s->out_to_pad[p] = out;
        s->oe_to_pad[p] = oe;

        /*
         * While isolated, the pad keeps the controls latched when
         * isolation began; the input path from the pad is not isolated.
         */
        if (!pad_isolated(s, p)) {
            s->latch[p] = pad_controls(s, p, out, oe);
        }
        c = s->latch[p];
        level = pad_level(s, p, c);
        s->pad_level[p] = level;

        s->in_from_pad[p] = (c & LATCH_IE) ? level : 0;
        to_peri[p] = over(CTRL_INOVER(ctrl), s->in_from_pad[p]);
        if (f >= 0) {
            in[f] |= to_peri[p];
        }

        irq = over(CTRL_IRQOVER(ctrl), s->in_from_pad[p]);
        if (irq != s->irq_level[p]) {
            s->intr_edge[p / 8] |= (irq ? INT_EDGE_HIGH : INT_EDGE_LOW)
                                   << (4 * (p % 8));
            s->irq_level[p] = irq;
        }
    }

    /* SIO and PIO see every pin, selected or not. */
    for (p = 0; p < RP2350_GPIO_PINS; p++) {
        in[sig(RP2350_GPIO_PORT_SIO, rp2350_gpio_sio_bit(p))] = to_peri[p];
    }
    for (port = RP2350_GPIO_PORT_PIO0; port <= RP2350_GPIO_PORT_PIO2;
         port++) {
        for (p = 0; p < RP2350_GPIO_BANK0_PINS; p++) {
            in[sig(port, p)] = to_peri[p];
        }
    }

    for (i = 0; i < RP2350_GPIO_SIGNALS; i++) {
        if (force || in[i] != s->peri_in[i]) {
            s->peri_in[i] = in[i];
            qemu_set_irq(s->peri_in_irq[i], in[i]);
        }
    }

    rp2350_gpio_set_irqs(s);
}

/*
 * Delivering a pin input can make a peripheral change an output at once;
 * such a change is applied once the current pass has finished.
 */
static void rp2350_gpio_update_force(RP2350GPIOState *s, bool force)
{
    if (s->updating) {
        s->update_pending = true;
        return;
    }
    s->updating = true;
    do {
        s->update_pending = false;
        rp2350_gpio_propagate(s, force);
        force = false;
    } while (s->update_pending);
    s->updating = false;
}

static void rp2350_gpio_update(RP2350GPIOState *s)
{
    rp2350_gpio_update_force(s, false);
}

void rp2350_gpio_set_nsmask(RP2350GPIOState *s, uint64_t mask)
{
    s->nsmask = mask;
    rp2350_gpio_update(s);
}

static void rp2350_gpio_set_src_out(void *opaque, int n, int level)
{
    RP2350GPIOPortRef *ref = opaque;

    ref->s->src_out[ref->base + n] = level != 0;
    rp2350_gpio_update(ref->s);
}

static void rp2350_gpio_set_src_oe(void *opaque, int n, int level)
{
    RP2350GPIOPortRef *ref = opaque;

    ref->s->src_oe[ref->base + n] = level != 0;
    rp2350_gpio_update(ref->s);
}

static void rp2350_gpio_set_pad_in(void *opaque, int n, int level)
{
    RP2350GPIOState *s = opaque;

    s->ext[n] = level < 0 ? RP2350_GPIO_EXT_NONE : level != 0;
    rp2350_gpio_update(s);
}

/*
 * Accesses from Non-secure code reach only the pins ACCESSCTRL grants to
 * Non-secure; the rest read zero and ignore writes.
 */
static bool access_ns(MemTxAttrs attrs)
{
    return !attrs.secure && !attrs.unspecified;
}

static bool pin_accessible(RP2350GPIOState *s, int p, MemTxAttrs attrs)
{
    return !access_ns(attrs) || pin_ns(s, p);
}

/* The bits of interrupt word w that the access may see. */
static uint32_t int_word_mask(RP2350GPIOState *s, int w, MemTxAttrs attrs)
{
    uint32_t mask = 0;
    int i;

    for (i = 0; i < 8; i++) {
        if (pin_accessible(s, w * 8 + i, attrs)) {
            mask |= 0xfu << (4 * i);
        }
    }
    return mask;
}

static uint32_t intr_word(RP2350GPIOState *s, int w)
{
    uint32_t v = s->intr_edge[w];
    int i;

    for (i = 0; i < 8; i++) {
        v |= (s->irq_level[w * 8 + i] ? INT_LEVEL_HIGH : INT_LEVEL_LOW)
             << (4 * i);
    }
    return v;
}

static uint32_t ints_word(RP2350GPIOState *s, int d, int w)
{
    return (intr_word(s, w) | s->intf[d][w]) & s->inte[d][w];
}

/*
 * IRQSUMMARY: one bit per pin with any interrupt asserted to destination
 * d, in the Secure or Non-secure summary by the pin's security.
 */
static uint32_t irqsummary(RP2350GPIOState *s, const RP2350GPIOBank *b, int d,
                           bool ns, int half, MemTxAttrs attrs)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < 32 && half * 32 + i < b->pins; i++) {
        int p = b->first_pin + half * 32 + i;

        if (pin_ns(s, p) == ns && pin_accessible(s, p, attrs) &&
            (ints_word(s, d, p / 8) >> (4 * (p % 8))) & 0xf) {
            v |= 1u << i;
        }
    }
    return v;
}

static uint32_t pin_status(RP2350GPIOState *s, int p)
{
    uint32_t v = 0;

    if (s->out_to_pad[p]) {
        v |= STATUS_OUTTOPAD;
    }
    if (s->oe_to_pad[p]) {
        v |= STATUS_OETOPAD;
    }
    if (s->in_from_pad[p]) {
        v |= STATUS_INFROMPAD;
    }
    if (s->irq_level[p]) {
        v |= STATUS_IRQTOPROC;
    }
    return v;
}

/*
 * Decode an interrupt register offset of bank b: the INTR word, or a
 * destination's INTE (kind 0), INTF (1) or INTS (2) word.
 */
static bool decode_int_reg(const RP2350GPIOBank *b, hwaddr reg, int *dest,
                           int *kind, int *word)
{
    hwaddr end = b->dest_base + RP2350_GPIO_DESTS * b->dest_stride;
    hwaddr off;

    if (reg >= b->intr && reg < b->intr + 4 * b->words) {
        *dest = -1;
        *word = b->first_pin / 8 + (reg - b->intr) / 4;
        return true;
    }
    if (reg >= b->dest_base && reg < end) {
        *dest = (reg - b->dest_base) / b->dest_stride;
        off = (reg - b->dest_base) % b->dest_stride;
        *kind = off / (4 * b->words);
        *word = b->first_pin / 8 + (off % (4 * b->words)) / 4;
        return true;
    }
    return false;
}

static MemTxResult rp2350_io_read(void *opaque, hwaddr addr, uint64_t *data,
                                  unsigned size, MemTxAttrs attrs,
                                  const RP2350GPIOBank *b)
{
    RP2350GPIOState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    hwaddr sum_end = IRQSUMMARY_BASE +
                     4 * b->summary_words * 2 * RP2350_GPIO_DESTS;
    int dest, kind, w;

    *data = 0;
    if (reg < 8 * b->pins) {
        int p = b->first_pin + reg / 8;

        if (pin_accessible(s, p, attrs)) {
            *data = reg & 4 ? s->ctrl[p] : pin_status(s, p);
        }
    } else if (reg >= IRQSUMMARY_BASE && reg < sum_end) {
        int n = (reg - IRQSUMMARY_BASE) / 4;
        int half = n % b->summary_words;

        n /= b->summary_words;
        *data = irqsummary(s, b, n / 2, n % 2, half, attrs);
    } else if (decode_int_reg(b, reg, &dest, &kind, &w)) {
        uint32_t mask = int_word_mask(s, w, attrs);

        if (dest < 0) {
            *data = intr_word(s, w) & mask;
        } else if (kind == 0) {
            *data = s->inte[dest][w] & mask;
        } else if (kind == 1) {
            *data = s->intf[dest][w] & mask;
        } else {
            *data = ints_word(s, dest, w) & mask;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of bad offset 0x%"
                      HWADDR_PRIx "\n", b->name, addr);
    }
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.gpio] */
static MemTxResult rp2350_io_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size, MemTxAttrs attrs,
                                   const RP2350GPIOBank *b)
{
    RP2350GPIOState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    hwaddr sum_end = IRQSUMMARY_BASE +
                     4 * b->summary_words * 2 * RP2350_GPIO_DESTS;
    int dest, kind, w;

    if (reg < 8 * b->pins) {
        int p = b->first_pin + reg / 8;

        /* STATUS is read-only. */
        if ((reg & 4) && pin_accessible(s, p, attrs)) {
            s->ctrl[p] = rp2350_atomic_apply(addr, s->ctrl[p], value) &
                         CTRL_MASK;
        }
    } else if (reg >= IRQSUMMARY_BASE && reg < sum_end) {
        /* Read-only. */
    } else if (decode_int_reg(b, reg, &dest, &kind, &w)) {
        uint32_t mask = int_word_mask(s, w, attrs);
        uint32_t *r;

        if (dest < 0) {
            /*
             * Writing 1 clears a latched edge. The atomic aliases act
             * per written bit: a SET alias write writes 1s, so it clears;
             * CLR and XOR alias writes never write a 1 to a set bit.
             */
            switch (addr / RP2350_ATOMIC_ALIAS_SIZE) {
            case 0:
            case 2:
                s->intr_edge[w] &= ~(value & mask & INT_EDGES);
                break;
            default:
                break;
            }
        } else if (kind < 2) {
            r = kind ? &s->intf[dest][w] : &s->inte[dest][w];
            *r = (*r & ~mask) |
                 (rp2350_atomic_apply(addr, *r, value) & mask);
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to bad offset 0x%"
                      HWADDR_PRIx "\n", b->name, addr);
        return MEMTX_OK;
    }
    rp2350_gpio_update(s);
    return MEMTX_OK;
}

static MemTxResult rp2350_io_bank0_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    return rp2350_io_read(opaque, addr, data, size, attrs,
                          &rp2350_gpio_banks[0]);
}

static MemTxResult rp2350_io_bank0_write(void *opaque, hwaddr addr,
                                         uint64_t value, unsigned size,
                                         MemTxAttrs attrs)
{
    return rp2350_io_write(opaque, addr, value, size, attrs,
                           &rp2350_gpio_banks[0]);
}

static MemTxResult rp2350_io_qspi_read(void *opaque, hwaddr addr,
                                       uint64_t *data, unsigned size,
                                       MemTxAttrs attrs)
{
    return rp2350_io_read(opaque, addr, data, size, attrs,
                          &rp2350_gpio_banks[1]);
}

static MemTxResult rp2350_io_qspi_write(void *opaque, hwaddr addr,
                                        uint64_t value, unsigned size,
                                        MemTxAttrs attrs)
{
    return rp2350_io_write(opaque, addr, value, size, attrs,
                           &rp2350_gpio_banks[1]);
}

/*
 * A pads register: the pin it controls (-1 for none), or the storage for
 * the per-bank and SWD registers.
 */
static uint32_t *pads_reg(RP2350GPIOState *s, bool qspi, hwaddr reg,
                          int *pin, uint32_t *mask)
{
    *pin = -1;
    *mask = PAD_MASK;
    if (reg == A_PADS_VOLTAGE_SELECT) {
        *mask = 1;
        return &s->voltage_select[qspi];
    }
    if (qspi) {
        if (reg < 4 + 4 * ARRAY_SIZE(rp2350_qspi_pad_pin)) {
            *pin = rp2350_qspi_pad_pin[reg / 4 - 1];
            return &s->pad[*pin];
        }
        return NULL;
    }
    if (reg < A_PADS_BANK0_GPIO0 + 4 * RP2350_GPIO_BANK0_PINS) {
        *pin = (reg - A_PADS_BANK0_GPIO0) / 4;
        return &s->pad[*pin];
    }
    if (reg == A_PADS_SWCLK) {
        return &s->pad_swclk;
    }
    if (reg == A_PADS_SWD) {
        return &s->pad_swd;
    }
    return NULL;
}

static MemTxResult rp2350_pads_read(void *opaque, hwaddr addr, uint64_t *data,
                                    MemTxAttrs attrs, bool qspi)
{
    RP2350GPIOState *s = opaque;
    uint32_t mask, *r;
    int pin;

    r = pads_reg(s, qspi, rp2350_atomic_reg(addr), &pin, &mask);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pads-%s: read of bad offset "
                      "0x%" HWADDR_PRIx "\n", qspi ? "qspi" : "bank0", addr);
        *data = 0;
    } else if (pin >= 0 && !pin_accessible(s, pin, attrs)) {
        *data = 0;
    } else {
        *data = *r;
    }
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.gpio] */
static MemTxResult rp2350_pads_write(void *opaque, hwaddr addr,
                                     uint64_t value, MemTxAttrs attrs,
                                     bool qspi)
{
    RP2350GPIOState *s = opaque;
    uint32_t mask, *r;
    int pin;

    r = pads_reg(s, qspi, rp2350_atomic_reg(addr), &pin, &mask);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-pads-%s: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", qspi ? "qspi" : "bank0", addr);
        return MEMTX_OK;
    }
    if (pin >= 0 && !pin_accessible(s, pin, attrs)) {
        return MEMTX_OK;
    }
    *r = rp2350_atomic_apply(addr, *r, value) & mask;
    if (pin >= 0) {
        rp2350_gpio_update(s);
    }
    return MEMTX_OK;
}

static MemTxResult rp2350_pads_bank0_read(void *opaque, hwaddr addr,
                                          uint64_t *data, unsigned size,
                                          MemTxAttrs attrs)
{
    return rp2350_pads_read(opaque, addr, data, attrs, false);
}

static MemTxResult rp2350_pads_bank0_write(void *opaque, hwaddr addr,
                                           uint64_t value, unsigned size,
                                           MemTxAttrs attrs)
{
    return rp2350_pads_write(opaque, addr, value, attrs, false);
}

static MemTxResult rp2350_pads_qspi_read(void *opaque, hwaddr addr,
                                         uint64_t *data, unsigned size,
                                         MemTxAttrs attrs)
{
    return rp2350_pads_read(opaque, addr, data, attrs, true);
}

static MemTxResult rp2350_pads_qspi_write(void *opaque, hwaddr addr,
                                          uint64_t value, unsigned size,
                                          MemTxAttrs attrs)
{
    return rp2350_pads_write(opaque, addr, value, attrs, true);
}

#define RP2350_GPIO_OPS(rd, wr) {                    \
    .read_with_attrs = rd,                           \
    .write_with_attrs = wr,                          \
    .endianness = DEVICE_LITTLE_ENDIAN,              \
    .valid.min_access_size = 4,                      \
    .valid.max_access_size = 4,                      \
}

static const MemoryRegionOps rp2350_io_bank0_ops =
    RP2350_GPIO_OPS(rp2350_io_bank0_read, rp2350_io_bank0_write);
static const MemoryRegionOps rp2350_io_qspi_ops =
    RP2350_GPIO_OPS(rp2350_io_qspi_read, rp2350_io_qspi_write);
static const MemoryRegionOps rp2350_pads_bank0_ops =
    RP2350_GPIO_OPS(rp2350_pads_bank0_read, rp2350_pads_bank0_write);
static const MemoryRegionOps rp2350_pads_qspi_ops =
    RP2350_GPIO_OPS(rp2350_pads_qspi_read, rp2350_pads_qspi_write);

/*
 * Reset is the chip-level (always-on domain) reset: registers, the pad
 * isolation latches, which take the reset values of the controls they
 * hold, and the edge detectors. External drive and peripheral sources
 * belong to the board and the peripherals.
 */
static void rp2350_gpio_hold_reset(Object *obj, ResetType type)
{
    RP2350GPIOState *s = RP2350_GPIO(obj);
    int p;

    for (p = 0; p < RP2350_GPIO_PINS; p++) {
        s->ctrl[p] = CTRL_RESET;
        s->pad[p] = 0;
    }
    for (p = 0; p < RP2350_GPIO_BANK0_PINS; p++) {
        s->pad[p] = PAD_BANK0_RESET;
    }
    s->pad[RP2350_GPIO_QSPI_SCLK] = PAD_QSPI_PD_RESET;
    s->pad[RP2350_GPIO_QSPI_SD0] = PAD_QSPI_PD_RESET;
    s->pad[RP2350_GPIO_QSPI_SD1] = PAD_QSPI_PD_RESET;
    s->pad[RP2350_GPIO_QSPI_SD2] = PAD_QSPI_PU_RESET;
    s->pad[RP2350_GPIO_QSPI_SD3] = PAD_QSPI_PU_RESET;
    s->pad[RP2350_GPIO_QSPI_SS] = PAD_QSPI_PU_RESET;
    s->voltage_select[0] = 0;
    s->voltage_select[1] = 0;
    s->pad_swclk = PAD_SWD_RESET;
    s->pad_swd = PAD_SWD_RESET;

    memset(s->intr_edge, 0, sizeof(s->intr_edge));
    memset(s->inte, 0, sizeof(s->inte));
    memset(s->intf, 0, sizeof(s->intf));
    memset(s->irq_level, 0, sizeof(s->irq_level));
    memset(s->pad_level, 0, sizeof(s->pad_level));
    s->nsmask = 0;

    for (p = 0; p < RP2350_GPIO_PINS; p++) {
        /* FUNCSEL is null, so nothing drives the pad. */
        s->latch[p] = pad_controls(s, p, 0, 0);
    }
}

static void rp2350_gpio_exit_reset(Object *obj, ResetType type)
{
    rp2350_gpio_update_force(RP2350_GPIO(obj), true);
}

static void rp2350_gpio_init(Object *obj)
{
    RP2350GPIOState *s = RP2350_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int port, core, i;

    memory_region_init_io(&s->io_bank0, obj, &rp2350_io_bank0_ops, s,
                          "rp2350-io-bank0", RP2350_ATOMIC_REGION_SIZE);
    memory_region_init_io(&s->io_qspi, obj, &rp2350_io_qspi_ops, s,
                          "rp2350-io-qspi", RP2350_ATOMIC_REGION_SIZE);
    memory_region_init_io(&s->pads_bank0, obj, &rp2350_pads_bank0_ops, s,
                          "rp2350-pads-bank0", RP2350_ATOMIC_REGION_SIZE);
    memory_region_init_io(&s->pads_qspi, obj, &rp2350_pads_qspi_ops, s,
                          "rp2350-pads-qspi", RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(sbd, &s->io_bank0);
    sysbus_init_mmio(sbd, &s->io_qspi);
    sysbus_init_mmio(sbd, &s->pads_bank0);
    sysbus_init_mmio(sbd, &s->pads_qspi);

    for (core = 0; core < RP2350_GPIO_CORES; core++) {
        for (i = 0; i < RP2350_GPIO_CORE_IRQS; i++) {
            sysbus_init_irq(sbd, &s->irq[core][i]);
        }
    }
    qdev_init_gpio_out_named(dev, &s->dormant_wake, RP2350_GPIO_DORMANT_WAKE,
                             1);

    for (port = 0; port < RP2350_GPIO_NUM_PORTS; port++) {
        const char *name = rp2350_gpio_ports[port].name;
        int n = rp2350_gpio_ports[port].signals;
        int base = rp2350_gpio_port_base[port];
        g_autofree char *out = g_strdup_printf("%s-out", name);
        g_autofree char *oe = g_strdup_printf("%s-oe", name);
        g_autofree char *in = g_strdup_printf("%s-in", name);

        s->port_ref[port].s = s;
        s->port_ref[port].base = base;
        qdev_init_gpio_in_named_with_opaque(dev, rp2350_gpio_set_src_out,
                                            &s->port_ref[port], out, n);
        qdev_init_gpio_in_named_with_opaque(dev, rp2350_gpio_set_src_oe,
                                            &s->port_ref[port], oe, n);
        qdev_init_gpio_out_named(dev, &s->peri_in_irq[base], in, n);
    }

    qdev_init_gpio_in_named(dev, rp2350_gpio_set_pad_in, RP2350_GPIO_PAD_IN,
                            RP2350_GPIO_PINS);
    memset(s->ext, RP2350_GPIO_EXT_NONE, sizeof(s->ext));
}

static void rp2350_gpio_realize(DeviceState *dev, Error **errp)
{
    RP2350GPIOState *s = RP2350_GPIO(dev);

    if (s->bonded_gpios > RP2350_GPIO_BANK0_PINS) {
        error_setg(errp, "bonded-gpios must be at most %d",
                   RP2350_GPIO_BANK0_PINS);
    }
}

static const VMStateDescription vmstate_rp2350_gpio = {
    .name = TYPE_RP2350_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ctrl, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT32_ARRAY(pad, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT32_ARRAY(voltage_select, RP2350GPIOState, 2),
        VMSTATE_UINT32(pad_swclk, RP2350GPIOState),
        VMSTATE_UINT32(pad_swd, RP2350GPIOState),
        VMSTATE_UINT32_ARRAY(intr_edge, RP2350GPIOState,
                             RP2350_GPIO_INT_WORDS),
        VMSTATE_UINT32_2DARRAY(inte, RP2350GPIOState, RP2350_GPIO_DESTS,
                               RP2350_GPIO_INT_WORDS),
        VMSTATE_UINT32_2DARRAY(intf, RP2350GPIOState, RP2350_GPIO_DESTS,
                               RP2350_GPIO_INT_WORDS),
        VMSTATE_UINT8_ARRAY(latch, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT8_ARRAY(pad_level, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT8_ARRAY(ext, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT8_ARRAY(irq_level, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT8_ARRAY(src_out, RP2350GPIOState, RP2350_GPIO_SIGNALS),
        VMSTATE_UINT8_ARRAY(src_oe, RP2350GPIOState, RP2350_GPIO_SIGNALS),
        VMSTATE_UINT8_ARRAY(peri_in, RP2350GPIOState, RP2350_GPIO_SIGNALS),
        VMSTATE_UINT64(nsmask, RP2350GPIOState),
        VMSTATE_UINT8_ARRAY(out_to_pad, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT8_ARRAY(oe_to_pad, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_UINT8_ARRAY(in_from_pad, RP2350GPIOState, RP2350_GPIO_PINS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_gpio_properties[] = {
    /* GPIOs bonded out to pins: 30 on the QFN-60 RP2350A, 48 on QFN-80. */
    DEFINE_PROP_UINT32("bonded-gpios", RP2350GPIOState, bonded_gpios,
                       RP2350_GPIO_QFN60_PINS),
};

static void rp2350_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rp2350_gpio_init_tables();
    dc->realize = rp2350_gpio_realize;
    device_class_set_props(dc, rp2350_gpio_properties);
    rc->phases.hold = rp2350_gpio_hold_reset;
    rc->phases.exit = rp2350_gpio_exit_reset;
    dc->vmsd = &vmstate_rp2350_gpio;
}

/* [spec:nuos:req:emu.gpio] */
static const TypeInfo rp2350_gpio_info = {
    .name          = TYPE_RP2350_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350GPIOState),
    .instance_init = rp2350_gpio_init,
    .class_init    = rp2350_gpio_class_init,
};

static void rp2350_gpio_register_types(void)
{
    type_register_static(&rp2350_gpio_info);
}
type_init(rp2350_gpio_register_types)
