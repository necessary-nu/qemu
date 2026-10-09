/*
 * Asynchronous serial lines: edge timing and a host serial adapter
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The adapter is the far end of a UART's pins: it samples the chip's TX
 * line in the middle of each bit, as a UART receiver does, and drives
 * the chip's RX line one bit at a time. Its line timers keep to the bit
 * grid of the frame they belong to, however late they run.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/char/uart-line.h"
#include "chardev/char-serial.h"
#include "migration/vmstate.h"

/* The deadline of the line change being delivered, or -1 outside one. */
static int64_t uart_line_edge_time = -1;

int64_t uart_line_now(void)
{
    if (uart_line_edge_time >= 0) {
        return uart_line_edge_time;
    }
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

void uart_line_set(qemu_irq irq, int level, int64_t when)
{
    int64_t outer = uart_line_edge_time;

    uart_line_edge_time = when;
    qemu_set_irq(irq, level);
    uart_line_edge_time = outer;
}

/* 8N1: start, eight data bits, stop. */
#define FRAME_BITS 10
#define BREAK_BITS (2 * FRAME_BITS)

/* Time from a frame's start edge to `halves` half bits later. */
static int64_t uart_line_offset(UARTLineState *s, unsigned halves)
{
    return muldiv64(halves, NANOSECONDS_PER_SECOND, 2 * s->baud);
}

static void uart_line_rx_sample(void *opaque)
{
    UARTLineState *s = opaque;
    int level = s->rx_level;

    if (s->rx_bit == 0) {
        if (level) {
            /* A glitch, not a start bit. */
            s->rx_bit = -1;
            return;
        }
    } else if (s->rx_bit < FRAME_BITS - 1) {
        s->rx_data |= level << (s->rx_bit - 1);
        s->rx_zero &= !level;
    } else {
        uint8_t ch = s->rx_data;

        s->rx_bit = -1;
        if (level) {
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
        } else if (s->rx_zero) {
            int on = 1;

            s->rx_break = true;
            qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK, &on);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: framing error: the UART "
                          "is not sending 8N1 at %" PRIu32 " baud\n",
                          object_get_canonical_path(OBJECT(s)), s->baud);
        }
        return;
    }
    s->rx_bit++;
    s->rx_next = s->rx_start + uart_line_offset(s, 2 * s->rx_bit + 1);
    timer_mod(s->rx_timer, s->rx_next);
}

static void uart_line_rx_in(void *opaque, int n, int level)
{
    UARTLineState *s = opaque;
    int64_t now = uart_line_now();
    uint8_t v = level != 0;

    /* An undriven line reads high through the adapter's pull-up. */
    if (level < 0) {
        v = 1;
    }
    if (v == s->rx_level) {
        return;
    }
    s->rx_level = v;
    if (v) {
        if (s->rx_break) {
            int off = 0;

            s->rx_break = false;
            qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK, &off);
        }
        return;
    }
    if (s->rx_bit >= 0 || s->rx_break) {
        return;
    }
    s->rx_bit = 0;
    s->rx_data = 0;
    s->rx_zero = true;
    s->rx_start = now;
    s->rx_next = now + uart_line_offset(s, 1);
    timer_mod(s->rx_timer, s->rx_next);
}

static void uart_line_set_txd(UARTLineState *s, int level, int64_t when)
{
    if (s->txd != level) {
        s->txd = level;
        uart_line_set(s->tx, level, when);
    }
}

/* Schedule the next change of level after the current bit. */
static void uart_line_tx_schedule(UARTLineState *s)
{
    int bit = s->tx_bit + 1;

    while (bit < s->tx_len &&
           ((s->tx_frame >> bit) & 1) == ((s->tx_frame >> s->tx_bit) & 1)) {
        bit++;
    }
    s->tx_bit = bit - 1;
    s->tx_next = s->tx_start + uart_line_offset(s, 2 * bit);
    timer_mod(s->tx_timer, s->tx_next);
}

static void uart_line_tx_start(UARTLineState *s, int64_t now)
{
    if (s->tx_bit >= 0) {
        return;
    }
    if (s->tx_break) {
        s->tx_break = false;
        s->tx_frame = 0;
        s->tx_len = BREAK_BITS;
    } else if (!fifo8_is_empty(&s->tx_queue)) {
        s->tx_frame = (fifo8_pop(&s->tx_queue) << 1) | (1u << 9);
        s->tx_len = FRAME_BITS;
        qemu_chr_fe_accept_input(&s->chr);
    } else {
        return;
    }
    s->tx_start = now;
    s->tx_bit = 0;
    uart_line_set_txd(s, 0, now);
    uart_line_tx_schedule(s);
}

static void uart_line_tx_tick(void *opaque)
{
    UARTLineState *s = opaque;
    int64_t now = s->tx_next;

    s->tx_bit++;
    if (s->tx_bit >= s->tx_len) {
        s->tx_bit = -1;
        uart_line_set_txd(s, 1, now);
        uart_line_tx_start(s, now);
        return;
    }
    uart_line_set_txd(s, (s->tx_frame >> s->tx_bit) & 1, now);
    uart_line_tx_schedule(s);
}

static int uart_line_can_receive(void *opaque)
{
    UARTLineState *s = opaque;

    return fifo8_num_free(&s->tx_queue);
}

static void uart_line_receive(void *opaque, const uint8_t *buf, int size)
{
    UARTLineState *s = opaque;
    int i;

    for (i = 0; i < size && !fifo8_is_full(&s->tx_queue); i++) {
        fifo8_push(&s->tx_queue, buf[i]);
    }
    uart_line_tx_start(s, uart_line_now());
}

static void uart_line_event(void *opaque, QEMUChrEvent event)
{
    UARTLineState *s = opaque;

    if (event == CHR_EVENT_BREAK) {
        s->tx_break = true;
        uart_line_tx_start(s, uart_line_now());
    }
}

static void uart_line_reset_hold(Object *obj, ResetType type)
{
    UARTLineState *s = UART_LINE(obj);

    timer_del(s->rx_timer);
    timer_del(s->tx_timer);
    s->rx_bit = -1;
    s->rx_break = false;
    s->tx_bit = -1;
    s->tx_break = false;
    fifo8_reset(&s->tx_queue);
}

static void uart_line_reset_exit(Object *obj, ResetType type)
{
    UARTLineState *s = UART_LINE(obj);

    s->txd = 1;
    qemu_set_irq(s->tx, 1);
}

static void uart_line_init(Object *obj)
{
    UARTLineState *s = UART_LINE(obj);

    qdev_init_gpio_in_named(DEVICE(obj), uart_line_rx_in, UART_LINE_RX, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->tx, UART_LINE_TX, 1);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_line_rx_sample, s);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_line_tx_tick, s);
    fifo8_create(&s->tx_queue, UART_LINE_TX_QUEUE);
    s->rx_level = 1;
    s->txd = 1;
}

static void uart_line_finalize(Object *obj)
{
    UARTLineState *s = UART_LINE(obj);

    timer_free(s->rx_timer);
    timer_free(s->tx_timer);
    fifo8_destroy(&s->tx_queue);
}

static void uart_line_realize(DeviceState *dev, Error **errp)
{
    UARTLineState *s = UART_LINE(dev);

    if (s->baud == 0) {
        error_setg(errp, "baud must not be zero");
        return;
    }
    qemu_chr_fe_set_handlers(&s->chr, uart_line_can_receive,
                             uart_line_receive, uart_line_event, NULL, s,
                             NULL, true);
}

static const VMStateDescription vmstate_uart_line = {
    .name = TYPE_UART_LINE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(rx_timer, UARTLineState),
        VMSTATE_INT64(rx_start, UARTLineState),
        VMSTATE_INT64(rx_next, UARTLineState),
        VMSTATE_INT32(rx_bit, UARTLineState),
        VMSTATE_UINT32(rx_data, UARTLineState),
        VMSTATE_UINT8(rx_level, UARTLineState),
        VMSTATE_BOOL(rx_zero, UARTLineState),
        VMSTATE_BOOL(rx_break, UARTLineState),
        VMSTATE_TIMER_PTR(tx_timer, UARTLineState),
        VMSTATE_FIFO8(tx_queue, UARTLineState),
        VMSTATE_INT64(tx_start, UARTLineState),
        VMSTATE_INT64(tx_next, UARTLineState),
        VMSTATE_UINT32(tx_frame, UARTLineState),
        VMSTATE_INT32(tx_len, UARTLineState),
        VMSTATE_INT32(tx_bit, UARTLineState),
        VMSTATE_UINT8(txd, UARTLineState),
        VMSTATE_BOOL(tx_break, UARTLineState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property uart_line_properties[] = {
    DEFINE_PROP_CHR("chardev", UARTLineState, chr),
    DEFINE_PROP_UINT32("baud", UARTLineState, baud, 115200),
};

static void uart_line_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = uart_line_realize;
    dc->vmsd = &vmstate_uart_line;
    device_class_set_props(dc, uart_line_properties);
    rc->phases.hold = uart_line_reset_hold;
    rc->phases.exit = uart_line_reset_exit;
}

/* [spec:nuos:req:emu.uart] */
static const TypeInfo uart_line_info = {
    .name              = TYPE_UART_LINE,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(UARTLineState),
    .instance_init     = uart_line_init,
    .instance_finalize = uart_line_finalize,
    .class_init        = uart_line_class_init,
};

static void uart_line_register_types(void)
{
    type_register_static(&uart_line_info);
}

type_init(uart_line_register_types)
