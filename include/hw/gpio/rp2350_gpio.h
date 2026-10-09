/*
 * RP2350 GPIO muxing and pads (IO_BANK0, IO_QSPI, PADS_BANK0, PADS_QSPI)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_RP2350_GPIO_H
#define HW_GPIO_RP2350_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_GPIO "rp2350-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350GPIOState, RP2350_GPIO)

/*
 * Pins. Bank 0 has registers for 48 GPIOs on every package; the QFN-60
 * RP2350A bonds out only GPIOs 0-29, and the "bonded-gpios" property says
 * how many have a pin. The QSPI bank follows bank 0, in IO_QSPI register
 * order.
 */
#define RP2350_GPIO_BANK0_PINS 48
#define RP2350_GPIO_QSPI_PINS 8
#define RP2350_GPIO_PINS (RP2350_GPIO_BANK0_PINS + RP2350_GPIO_QSPI_PINS)
#define RP2350_GPIO_QFN60_PINS 30

enum {
    RP2350_GPIO_USB_DP = RP2350_GPIO_BANK0_PINS,
    RP2350_GPIO_USB_DM,
    RP2350_GPIO_QSPI_SCLK,
    RP2350_GPIO_QSPI_SS,
    RP2350_GPIO_QSPI_SD0,
    RP2350_GPIO_QSPI_SD1,
    RP2350_GPIO_QSPI_SD2,
    RP2350_GPIO_QSPI_SD3,
};

/*
 * SIO, the GPIO coprocessor and ACCESSCTRL GPIO_NSMASK number pins as one
 * 64-bit vector: bank 0 GPIOs in bits 0-47, the QSPI bank in bits 56-63.
 */
#define RP2350_GPIO_SIO_BITS 64

static inline int rp2350_gpio_sio_bit(int pin)
{
    return pin < RP2350_GPIO_BANK0_PINS ? pin : pin + 8;
}

/*
 * Peripheral ports.
 *
 * Each block whose signals the IO muxing can route to a pin is a port with
 * a fixed number of signals, numbered by the enums below. For port <name>,
 * the device has three named GPIO arrays, one line per signal:
 *
 *   "<name>-out"  input:  the level the peripheral drives on the signal
 *   "<name>-oe"   input:  the peripheral's output enable for the signal
 *   "<name>-in"   output: the level the signal's pins deliver to the
 *                         peripheral
 *
 * A pin drives out and oe of the signal its FUNCSEL selects (both 0 for
 * an unconnected signal), through the CTRL overrides and its pad. A
 * signal's input is the OR of the pin inputs of every pin that selects it,
 * as the datasheet specifies, except for SIO and PIO: their inputs are
 * connected to every pin whether selected or not. Use the helpers below
 * or qdev_get_gpio_in_named()/qdev_connect_gpio_out_named() with the
 * names from rp2350_gpio_port_name().
 *
 * Source levels are the peripherals' state, so they are not reset with
 * this device; a peripheral sets its own on reset.
 */
typedef enum RP2350GPIOPort {
    RP2350_GPIO_PORT_SPI0,
    RP2350_GPIO_PORT_SPI1,
    RP2350_GPIO_PORT_UART0,
    RP2350_GPIO_PORT_UART1,
    RP2350_GPIO_PORT_I2C0,
    RP2350_GPIO_PORT_I2C1,
    RP2350_GPIO_PORT_PWM,
    RP2350_GPIO_PORT_SIO,
    RP2350_GPIO_PORT_PIO0,
    RP2350_GPIO_PORT_PIO1,
    RP2350_GPIO_PORT_PIO2,
    RP2350_GPIO_PORT_HSTX,
    RP2350_GPIO_PORT_QMI,
    RP2350_GPIO_PORT_JTAG,
    RP2350_GPIO_PORT_TRACE,
    RP2350_GPIO_PORT_USB,
    RP2350_GPIO_PORT_CLOCKS,
    RP2350_GPIO_NUM_PORTS,
} RP2350GPIOPort;

/* SPI0, SPI1 */
enum {
    RP2350_GPIO_SPI_RX,
    RP2350_GPIO_SPI_CSN,
    RP2350_GPIO_SPI_SCK,
    RP2350_GPIO_SPI_TX,
    RP2350_GPIO_SPI_SIGNALS,
};

/* UART0, UART1 */
enum {
    RP2350_GPIO_UART_TX,
    RP2350_GPIO_UART_RX,
    RP2350_GPIO_UART_CTS,
    RP2350_GPIO_UART_RTS,
    RP2350_GPIO_UART_SIGNALS,
};

/* I2C0, I2C1 */
enum {
    RP2350_GPIO_I2C_SDA,
    RP2350_GPIO_I2C_SCL,
    RP2350_GPIO_I2C_SIGNALS,
};

/* PWM: channel A (0) or B (1) of slices 0-11. */
#define RP2350_GPIO_PWM_SLICES 12
#define RP2350_GPIO_PWM(slice, chan) ((slice) * 2 + (chan))
#define RP2350_GPIO_PWM_SIGNALS (RP2350_GPIO_PWM_SLICES * 2)

/* SIO: numbered by rp2350_gpio_sio_bit(). PIO0-2: by bank 0 GPIO. */
#define RP2350_GPIO_PIO_SIGNALS RP2350_GPIO_BANK0_PINS

/* HSTX: output bits 0-7, on GPIOs 12-19. */
#define RP2350_GPIO_HSTX_FIRST_PIN 12
#define RP2350_GPIO_HSTX_SIGNALS 8

/* QMI (XIP): chip select 0 and data on the QSPI bank, CS1 on bank 0. */
enum {
    RP2350_GPIO_QMI_SCK,
    RP2350_GPIO_QMI_CS0N,
    RP2350_GPIO_QMI_SD0,
    RP2350_GPIO_QMI_SD1,
    RP2350_GPIO_QMI_SD2,
    RP2350_GPIO_QMI_SD3,
    RP2350_GPIO_QMI_CS1N,
    RP2350_GPIO_QMI_SIGNALS,
};

/* JTAG on GPIOs 0-3 */
enum {
    RP2350_GPIO_JTAG_TCK,
    RP2350_GPIO_JTAG_TMS,
    RP2350_GPIO_JTAG_TDI,
    RP2350_GPIO_JTAG_TDO,
    RP2350_GPIO_JTAG_SIGNALS,
};

/* CoreSight trace port */
enum {
    RP2350_GPIO_TRACE_CLK,
    RP2350_GPIO_TRACE_DATA0,
    RP2350_GPIO_TRACE_DATA1,
    RP2350_GPIO_TRACE_DATA2,
    RP2350_GPIO_TRACE_DATA3,
    RP2350_GPIO_TRACE_SIGNALS,
};

/* USB VBUS management */
enum {
    RP2350_GPIO_USB_OVERCURR_DETECT,
    RP2350_GPIO_USB_VBUS_DETECT,
    RP2350_GPIO_USB_VBUS_EN,
    RP2350_GPIO_USB_SIGNALS,
};

/* CLOCKS general-purpose clock inputs and outputs */
enum {
    RP2350_GPIO_CLOCKS_GPIN0,
    RP2350_GPIO_CLOCKS_GPIN1,
    RP2350_GPIO_CLOCKS_GPOUT0,
    RP2350_GPIO_CLOCKS_GPOUT1,
    RP2350_GPIO_CLOCKS_GPOUT2,
    RP2350_GPIO_CLOCKS_GPOUT3,
    RP2350_GPIO_CLOCKS_SIGNALS,
};

/* Total peripheral signals across all ports. */
#define RP2350_GPIO_SIGNALS                                              \
    (2 * RP2350_GPIO_SPI_SIGNALS + 2 * RP2350_GPIO_UART_SIGNALS +       \
     2 * RP2350_GPIO_I2C_SIGNALS + RP2350_GPIO_PWM_SIGNALS +            \
     RP2350_GPIO_SIO_BITS + 3 * RP2350_GPIO_PIO_SIGNALS +               \
     RP2350_GPIO_HSTX_SIGNALS + RP2350_GPIO_QMI_SIGNALS +               \
     RP2350_GPIO_JTAG_SIGNALS + RP2350_GPIO_TRACE_SIGNALS +             \
     RP2350_GPIO_USB_SIGNALS + RP2350_GPIO_CLOCKS_SIGNALS)

/*
 * Interrupt outputs (sysbus IRQs), per core: core * RP2350_GPIO_CORE_IRQS
 * plus one of these. Each reaches only its own core's NVIC.
 */
enum {
    RP2350_GPIO_IRQ_BANK0,
    RP2350_GPIO_IRQ_BANK0_NS,
    RP2350_GPIO_IRQ_QSPI,
    RP2350_GPIO_IRQ_QSPI_NS,
    RP2350_GPIO_CORE_IRQS,
};
#define RP2350_GPIO_CORES 2

/*
 * Named GPIO output "dormant-wake": the OR of every bank's DORMANT_WAKE
 * interrupts, Secure and Non-secure, which wakes XOSC or ROSC from
 * dormant mode.
 */
#define RP2350_GPIO_DORMANT_WAKE "dormant-wake"

/*
 * Named GPIO input array "pad-in", one line per pin: drives the pin from
 * outside the chip. Level 0 or 1 drives the pin low or high; a negative
 * level stops driving it, leaving it to its pulls. A pin the chip drives
 * itself (pad output enabled) reads its own output. Lines for pins the
 * package does not bond out have no effect.
 */
#define RP2350_GPIO_PAD_IN "pad-in"
#define RP2350_GPIO_EXT_NONE 0xff

/* Interrupt destinations, in register order. */
enum {
    RP2350_GPIO_DEST_PROC0,
    RP2350_GPIO_DEST_PROC1,
    RP2350_GPIO_DEST_DORMANT_WAKE,
    RP2350_GPIO_DESTS,
};

/* Interrupt registers hold 4 bits per pin, 8 pins per word. */
#define RP2350_GPIO_INT_WORDS (RP2350_GPIO_PINS / 8)

typedef struct RP2350GPIOPortRef {
    RP2350GPIOState *s;
    int base;
} RP2350GPIOPortRef;

struct RP2350GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion io_bank0;
    MemoryRegion io_qspi;
    MemoryRegion pads_bank0;
    MemoryRegion pads_qspi;

    uint32_t bonded_gpios;

    /* IO_BANK0/IO_QSPI GPIOn_CTRL */
    uint32_t ctrl[RP2350_GPIO_PINS];
    /* PADS registers by pin; USB DP/DM have none. */
    uint32_t pad[RP2350_GPIO_PINS];
    uint32_t voltage_select[2];
    uint32_t pad_swclk;
    uint32_t pad_swd;

    /*
     * Latched edge events (INTR), and the enables and forces of each
     * destination, in the register layout: word w covers pins 8w-8w+7,
     * so words 0-5 are IO_BANK0's and word 6 is IO_QSPI's.
     */
    uint32_t intr_edge[RP2350_GPIO_INT_WORDS];
    uint32_t inte[RP2350_GPIO_DESTS][RP2350_GPIO_INT_WORDS];
    uint32_t intf[RP2350_GPIO_DESTS][RP2350_GPIO_INT_WORDS];

    /* Pad isolation latches (PAD_LATCH_* bits). */
    uint8_t latch[RP2350_GPIO_PINS];
    /* The level on each pin; held by the bus keeper or a floating pin. */
    uint8_t pad_level[RP2350_GPIO_PINS];
    /* External drive on each pin: 0, 1, or RP2350_GPIO_EXT_NONE. */
    uint8_t ext[RP2350_GPIO_PINS];
    /* Interrupt signal per pin after IRQOVER, for edge detection. */
    uint8_t irq_level[RP2350_GPIO_PINS];

    /* Peripheral sources and the inputs last delivered to them. */
    uint8_t src_out[RP2350_GPIO_SIGNALS];
    uint8_t src_oe[RP2350_GPIO_SIGNALS];
    uint8_t peri_in[RP2350_GPIO_SIGNALS];
    qemu_irq peri_in_irq[RP2350_GPIO_SIGNALS];
    RP2350GPIOPortRef port_ref[RP2350_GPIO_NUM_PORTS];

    /* ACCESSCTRL GPIO_NSMASK1:0, in the SIO layout. */
    uint64_t nsmask;

    /* Derived per pin by each update, for the STATUS registers. */
    uint8_t out_to_pad[RP2350_GPIO_PINS];
    uint8_t oe_to_pad[RP2350_GPIO_PINS];
    uint8_t in_from_pad[RP2350_GPIO_PINS];

    qemu_irq irq[RP2350_GPIO_CORES][RP2350_GPIO_CORE_IRQS];
    qemu_irq dormant_wake;

    bool updating;
    bool update_pending;
};

/* The name of a port, as used in its "<name>-out/-oe/-in" GPIO arrays. */
const char *rp2350_gpio_port_name(RP2350GPIOPort port);

/* The number of signals of a port. */
int rp2350_gpio_port_signals(RP2350GPIOPort port);

/* The inputs a peripheral drives signal `n` of `port` through. */
qemu_irq rp2350_gpio_out_line(RP2350GPIOState *s, RP2350GPIOPort port, int n);
qemu_irq rp2350_gpio_oe_line(RP2350GPIOState *s, RP2350GPIOPort port, int n);

/* Deliver the pin input of signal `n` of `port` to `irq`. */
void rp2350_gpio_connect_in(RP2350GPIOState *s, RP2350GPIOPort port, int n,
                            qemu_irq irq);

/*
 * Set the ACCESSCTRL GPIO_NSMASK1:0 pair (SIO layout). A pin whose bit is
 * set is Non-secure accessible: its interrupts go to the _NS outputs and
 * Non-secure code may access its IO and pad registers; Non-secure accesses
 * to other pins' registers read zero and are ignored. Resets to 0.
 */
void rp2350_gpio_set_nsmask(RP2350GPIOState *s, uint64_t mask);

#endif
