/*
 * A board-level serial terminal on an ESP32 UART's pads
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The chardev behind -serial is not wired into the UART: it is a terminal
 * (a USB-serial bridge on a dev board) attached to the pads the UART's
 * lines leave the chip on by default. It receives frames from the TXD
 * pad's level and sends the chardev's bytes as frames on the RXD pad, so
 * the bytes only get through where software routes the UART to those pads.
 * Like a terminal set up to match the chip, it uses the frame format and
 * baud rate the UART is configured for at each frame; the UART's own line
 * inversions it does not follow.
 *
 * While its chardev is connected the terminal drives its TX pad, idle
 * high, as a bridge's TX output does.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/irq.h"
#include "hw/char/esp32_uart_terminal.h"
#include "hw/gpio/esp32_gpio.h"
#include "migration/vmstate.h"

static gboolean term_out_ready(void *do_not_use, GIOCondition cond,
                               void *opaque);

/* Hand the bytes waiting to the chardev, as many as it takes. */
static void term_flush(Esp32UartTerminalState *s)
{
    while (!fifo8_is_empty(&s->out)) {
        uint32_t n;
        const uint8_t *buf = fifo8_peek_bufptr(&s->out,
                                               fifo8_num_used(&s->out), &n);
        int done = qemu_chr_fe_write(&s->chr, buf, n);

        if (done <= 0) {
            break;
        }
        fifo8_drop(&s->out, done);
    }
    if (!fifo8_is_empty(&s->out) && !s->out_watch) {
        s->out_watch = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                             term_out_ready, s);
        if (!s->out_watch) {
            /* A backend that can't be waited on loses what it refused */
            fifo8_reset(&s->out);
        }
    }
}

static gboolean term_out_ready(void *do_not_use, GIOCondition cond,
                               void *opaque)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(opaque);

    s->out_watch = 0;
    term_flush(s);
    return G_SOURCE_REMOVE;
}

/* [spec:nuos:req:emu.esp32.gpio] A frame received from the chip's TXD. */
static void term_rx_deliver(void *opaque, unsigned data, unsigned flags)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(opaque);

    if ((flags & ESP32_UART_RX_BREAK) ||
        !qemu_chr_fe_backend_connected(&s->chr)) {
        return;
    }
    if (fifo8_is_full(&s->out)) {
        return;
    }
    fifo8_push(&s->out, data);
    term_flush(s);
}

static bool term_format(void *opaque, Esp32UartFrameFmt *fmt)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(opaque);

    return esp32_uart_line_format(s->uart, fmt);
}

static void term_rx_in(void *opaque, int n, int level)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(opaque);

    esp32_uart_rx_line(&s->rx, level != 0, esp32_line_time());
}

static int64_t term_tx_time(Esp32UartTerminalState *s, unsigned half_bits)
{
    return s->tx_start_ns + esp32_uart_mul_shr(half_bits, s->tx_q32, 33);
}

static bool term_tx_level(Esp32UartTerminalState *s, unsigned bit)
{
    return bit < s->tx_nbits ? (s->tx_levels >> bit) & 1 : 1;
}

static unsigned term_tx_next_change(Esp32UartTerminalState *s)
{
    unsigned b;

    for (b = s->tx_bit + 1; b <= s->tx_nbits; b++) {
        if (term_tx_level(s, b) != term_tx_level(s, b - 1)) {
            break;
        }
    }
    return b;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Put every bit whose time has passed on the TX pad, stamped with its own
 * time; when the frame's stop bits are over, take the next byte.
 */
static void term_tx_advance(Esp32UartTerminalState *s);

/* Start sending the next byte waiting, at start_ns, if the UART can run. */
static bool term_tx_start(Esp32UartTerminalState *s, int64_t start_ns)
{
    Esp32UartFrameFmt fmt;
    bool was_full = fifo8_is_full(&s->in);

    if (fifo8_is_empty(&s->in) || !esp32_uart_line_format(s->uart, &fmt)) {
        return false;
    }
    s->tx_nbits = esp32_uart_frame_levels(&fmt, fifo8_pop(&s->in),
                                          &s->tx_levels);
    if (was_full) {
        qemu_chr_fe_accept_input(&s->chr);
    }
    s->tx_half = s->tx_nbits * 2 + fmt.stop_half_bits;
    s->tx_q32 = fmt.bit_q32;
    s->tx_start_ns = start_ns;
    s->tx_bit = 0;
    s->tx_busy = true;
    esp32_line_set_at(s->tx_out, 0, start_ns);
    return true;
}

static void term_tx_advance(Esp32UartTerminalState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned b;
    int64_t t;

    while (s->tx_busy) {
        b = term_tx_next_change(s);
        if (b <= s->tx_nbits) {
            t = term_tx_time(s, 2 * b);
            if (t > now) {
                break;
            }
            s->tx_bit = b;
            esp32_line_set_at(s->tx_out, term_tx_level(s, b), t);
            continue;
        }
        t = term_tx_time(s, s->tx_half);
        if (t > now) {
            break;
        }
        s->tx_busy = false;
        /* The next byte follows the stop bits without a gap */
        term_tx_start(s, t);
    }
    if (s->tx_busy) {
        b = term_tx_next_change(s);
        timer_mod_ns(&s->tx_timer, b <= s->tx_nbits ?
                     term_tx_time(s, 2 * b) : term_tx_time(s, s->tx_half));
    } else {
        timer_del(&s->tx_timer);
    }
}

static void term_tx_timer_cb(void *opaque)
{
    term_tx_advance(ESP32_UART_TERMINAL(opaque));
}

static int term_can_receive(void *opaque)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(opaque);

    return fifo8_num_free(&s->in);
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Bytes from the chardev wait in the terminal's FIFO and go out as frames,
 * back to back, at the UART's baud rate.
 */
static void term_receive(void *opaque, const uint8_t *buf, int size)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(opaque);

    for (int i = 0; i < size && !fifo8_is_full(&s->in); i++) {
        fifo8_push(&s->in, buf[i]);
    }
    if (!s->tx_busy) {
        term_tx_start(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    term_tx_advance(s);
}

static void term_event(void *opaque, QEMUChrEvent event)
{
}

static void term_format_changed(Notifier *n, void *data)
{
    Esp32UartTerminalState *s = container_of(n, Esp32UartTerminalState,
                                             format_notifier);

    if (!s->tx_busy &&
        term_tx_start(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL))) {
        term_tx_advance(s);
    }
}

static void term_line_sync(Notifier *n, void *data)
{
    term_tx_advance(container_of(n, Esp32UartTerminalState, line_source));
}

static void esp32_uart_terminal_realize(DeviceState *dev, Error **errp)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(dev);

    if (!s->uart) {
        error_setg(errp, "esp32.uart-terminal: 'uart' link not set");
        return;
    }
    esp32_uart_add_format_notifier(s->uart, &s->format_notifier);
    esp32_line_add_source(&s->line_source);
    qemu_chr_fe_set_handlers(&s->chr, term_can_receive, term_receive,
                             term_event, NULL, s, NULL, true);
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_set_irq(s->tx_out, 1);
    }
}

static void esp32_uart_terminal_init(Object *obj)
{
    Esp32UartTerminalState *s = ESP32_UART_TERMINAL(obj);
    DeviceState *dev = DEVICE(obj);

    fifo8_create(&s->out, ESP32_UART_TERMINAL_BUF);
    fifo8_create(&s->in, ESP32_UART_TERMINAL_IN_BUF);
    esp32_uart_rx_init(&s->rx, term_format, term_rx_deliver, s);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, term_tx_timer_cb, s);
    qdev_init_gpio_in_named(dev, term_rx_in, ESP32_UART_TERMINAL_RX, 1);
    qdev_init_gpio_out_named(dev, &s->tx_out, ESP32_UART_TERMINAL_TX, 1);
    object_property_add_link(obj, "uart", TYPE_ESP32_UART,
                             (Object **)&s->uart,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    s->format_notifier.notify = term_format_changed;
    s->line_source.notify = term_line_sync;
}

static const Property esp32_uart_terminal_properties[] = {
    DEFINE_PROP_CHR("chardev", Esp32UartTerminalState, chr),
};

static const VMStateDescription vmstate_esp32_uart_terminal = {
    .name = TYPE_ESP32_UART_TERMINAL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(rx, Esp32UartTerminalState, 1, vmstate_esp32_uart_rx,
                       Esp32UartRx),
        VMSTATE_FIFO8(out, Esp32UartTerminalState),
        VMSTATE_FIFO8(in, Esp32UartTerminalState),
        VMSTATE_TIMER(tx_timer, Esp32UartTerminalState),
        VMSTATE_BOOL(tx_busy, Esp32UartTerminalState),
        VMSTATE_UINT16(tx_levels, Esp32UartTerminalState),
        VMSTATE_UINT8(tx_nbits, Esp32UartTerminalState),
        VMSTATE_UINT8(tx_half, Esp32UartTerminalState),
        VMSTATE_UINT8(tx_bit, Esp32UartTerminalState),
        VMSTATE_INT64(tx_start_ns, Esp32UartTerminalState),
        VMSTATE_UINT64(tx_q32, Esp32UartTerminalState),
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_uart_terminal_class_init(ObjectClass *klass,
                                           const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32_uart_terminal_realize;
    dc->vmsd = &vmstate_esp32_uart_terminal;
    device_class_set_props(dc, esp32_uart_terminal_properties);
    dc->user_creatable = false;
}

/* [spec:nuos:req:emu.esp32.gpio] */
static const TypeInfo esp32_uart_terminal_info = {
    .name = TYPE_ESP32_UART_TERMINAL,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32UartTerminalState),
    .instance_init = esp32_uart_terminal_init,
    .class_init = esp32_uart_terminal_class_init,
};

static void esp32_uart_terminal_register_types(void)
{
    type_register_static(&esp32_uart_terminal_info);
}

type_init(esp32_uart_terminal_register_types)
