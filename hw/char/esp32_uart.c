/*
 * ESP32 UART emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * The QEMU model of nRF51 UART by Julia Suvorova was used as a template.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/system.h"
#include "chardev/char-fe.h"
#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/qdev-clock.h"
#include "hw/char/esp32_uart.h"
#include "trace.h"


static gboolean uart_tx_chr_ready(void *do_not_use, GIOCondition cond,
                                  void *opaque);
static void uart_receive(void *opaque, const uint8_t *buf, int size);
static void uart_tx_start(ESP32UARTState *s);
static void uart_tx_advance(ESP32UARTState *s);


/* The UART only operates while the SoC lets its clocks run. */
static bool uart_running(ESP32UARTState *s)
{
    return clock_is_enabled(s->apb_clk);
}

static bool uart_uses_ref_tick(ESP32UARTState *s)
{
    return !FIELD_EX32(s->reg[R_UART_CONF0], UART_CONF0, TICK_REF_ALWAYS_ON);
}

static Clock *uart_baud_clk(ESP32UARTState *s)
{
    return uart_uses_ref_tick(s) ? s->ref_tick_clk : s->apb_clk;
}

/* The baud divider in 1/16ths: CLKDIV + CLKDIV_FRAG / 16. */
static uint64_t uart_div16(ESP32UARTState *s)
{
    return ((uint64_t)FIELD_EX32(s->reg[R_UART_CLKDIV], UART_CLKDIV, CLKDIV)
            << 4) + FIELD_EX32(s->reg[R_UART_CLKDIV], UART_CLKDIV, CLKDIV_FRAG);
}

/* Bits per frame, in half bits: start, data, parity and stop bits. */
static unsigned uart_frame_half_bits(ESP32UARTState *s)
{
    uint32_t conf0 = s->reg[R_UART_CONF0];
    static const unsigned stop_half_bits[] = { 2, 2, 3, 4 };
    unsigned bits = 1 + 5 + FIELD_EX32(conf0, UART_CONF0, BIT_NUM) +
                    FIELD_EX32(conf0, UART_CONF0, PARITY_EN);

    return bits * 2 + stop_half_bits[FIELD_EX32(conf0, UART_CONF0,
                                                STOP_BIT_NUM)];
}

/* Frame time at the current baud clock and divider; 0 if it can't run. */
static uint64_t uart_frame_ns(ESP32UARTState *s)
{
    Clock *clk = uart_baud_clk(s);
    uint64_t div16 = uart_div16(s);

    if (!uart_running(s) || !clock_is_enabled(clk) || div16 == 0) {
        return 0;
    }
    return clock_ticks_to_ns(clk, div16 * uart_frame_half_bits(s)) / 32;
}

/*
 * [spec:nuos:req:emu.esp32.clock-gating]
 * Baud rate = baud clock * 16 / (CLKDIV * 16 + CLKDIV_FRAG), with the baud
 * clock at its current frequency. 0 when the UART is stopped or the
 * divider is 0.
 */
static void uart_update_baud(ESP32UARTState *s)
{
    Clock *clk = uart_baud_clk(s);
    uint64_t div16 = uart_div16(s);

    if (!uart_running(s) || !clock_is_enabled(clk) || div16 == 0) {
        s->baud_rate = 0;
    } else {
        s->baud_rate = muldiv64(clock_get_hz(clk), 16, div16);
    }
}

/* Stop the frame in flight, keeping how much of it is left to send. */
static void uart_tx_suspend(ESP32UARTState *s)
{
    uint64_t frame_ns = uart_frame_ns(s);

    if (!s->tx_timed) {
        return;
    }
    if (frame_ns) {
        int64_t left = MAX(s->tx_end_ns -
                           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), 0);
        s->tx_frame_left = MIN(muldiv64(left, 65536, frame_ns), 65536);
    }
    timer_del(&s->tx_timer);
    s->tx_timed = false;
}

/* Continue the frame in flight at the current frame time. */
static void uart_tx_resume(ESP32UARTState *s)
{
    uint64_t frame_ns = uart_frame_ns(s);

    if (!s->tx_busy || s->tx_wait_chr || s->tx_timed || !frame_ns) {
        return;
    }
    s->tx_end_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                   muldiv64(frame_ns, s->tx_frame_left, 65536);
    timer_mod_ns(&s->tx_timer, s->tx_end_ns);
    s->tx_timed = true;
}

/* [spec:nuos:req:emu.esp32.clock-gating] */
static void uart_clk_update(void *opaque, ClockEvent event)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    if (event == ClockPreUpdate) {
        uart_tx_advance(s);
        uart_tx_suspend(s);
        return;
    }
    uart_update_baud(s);
    uart_tx_resume(s);
    uart_tx_start(s);
    if (uart_running(s)) {
        qemu_chr_fe_accept_input(&s->chr);
    } else {
        timer_del(&s->rx_timeout_timer);
    }
}


void esp32_uart_update_irq(ESP32UARTState *s)
{
    bool irq = false;

    uint32_t tx_empty_raw = (fifo8_num_used(&s->tx_fifo) <= s->tx_empty_threshold);
    uint32_t rx_full_raw = (fifo8_num_used(&s->rx_fifo) >= s->rx_full_threshold);
    uint32_t tx_done_raw = (fifo8_num_used(&s->tx_fifo) == 0) && !s->tx_busy;
    uint32_t rxfifo_tout_raw = (s->rxfifo_tout) ? 1 : 0;

    uint32_t int_raw = s->reg[R_UART_INT_RAW];
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, RXFIFO_FULL, rx_full_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, TXFIFO_EMPTY, tx_empty_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, TX_DONE, tx_done_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, RXFIFO_TOUT, rxfifo_tout_raw);
    s->reg[R_UART_INT_RAW] = int_raw;

    uint32_t int_st = s->reg[R_UART_INT_RAW] & s->reg[R_UART_INT_ENA];
    irq = int_st != 0;
    s->reg[R_UART_INT_ST] = int_st;

    qemu_set_irq(s->irq, irq);
}


void esp32_uart_set_rx_timeout(ESP32UARTState *s)
{
    if (s->rx_tout_ena && s->baud_rate) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t thres = s->rx_tout_thres;
        /*
         * The timeout counter runs from APB_CLK: with REF_TICK as the baud
         * clock, the threshold counts bit times scaled by REF_TICK / APB_CLK.
         */
        if (uart_uses_ref_tick(s)) {
            thres = muldiv64(thres, clock_get_hz(s->ref_tick_clk),
                             MAX(clock_get_hz(s->apb_clk), 1));
        }
        int64_t rx_timeout_ns = now +
            thres * NANOSECONDS_PER_SECOND / s->baud_rate;
        /* If throttling is done, make sure timeout doesn't happen before more data
         * is allowed to come. Offset it by 1ms.
         */
        if (rx_timeout_ns <= s->throttle_timer.expire_time) {
            rx_timeout_ns = s->throttle_timer.expire_time + 10000000;
        }
        timer_mod_ns(&s->rx_timeout_timer, rx_timeout_ns);
    } else {
        timer_del(&s->rx_timeout_timer);
        s->rxfifo_tout = false;
    }
}


static uint64_t uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    uint64_t r = 0;

    uart_tx_advance(s);
    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            r = 0xEE;
            error_report("esp_uart: read UART FIFO while it is empty");
        } else {
            r = fifo8_pop(&s->rx_fifo);
            esp32_uart_update_irq(s);
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;

    case A_UART_STATUS:
        r = FIELD_DP32(r, UART_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
        r = FIELD_DP32(r, UART_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
        /* TX_STRT while a frame is on the line, TX_IDLE otherwise */
        r = FIELD_DP32(r, UART_STATUS, ST_UTX_OUT, s->tx_busy ? 1 : 0);
        break;

    case A_UART_LOWPULSE:
    case A_UART_HIGHPULSE:
        r = 337;  /* FIXME: this should depend on the APB frequency */
        break;
    case A_UART_MEM_CONF:
        r = FIELD_DP32(r, UART_MEM_CONF, RX_SIZE, (unsigned char)(UART_FIFO_LENGTH/128));
        r = FIELD_DP32(r, UART_MEM_CONF, TX_SIZE,  (unsigned char)(UART_FIFO_LENGTH/128));
        break;
    case A_UART_MEM_RX_STATUS: {
        uint32_t fifo_size = fifo8_num_used(&s->rx_fifo);
        /* The software only cares about the differene between WR_ADDR and RD_ADDR;
         * to keep things simpler, set RD_ADDR to 0 and WR_ADDR to the number of bytes
         * in the FIFO. 128 is a special case — write and read pointers should be
         * the same in this case.
         */
        r = FIELD_DP32(0, UART_MEM_RX_STATUS, WR_ADDR, (fifo_size == 128) ? 0 : fifo_size);
        }
        break;
    case A_UART_DATE:
        r = 0x15122500;
        break;
    default:
        r = s->reg[addr / 4];
        break;
    }

    return r;
}


static void uart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    uart_tx_advance(s);
    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            error_report("esp_uart: write to UART FIFO while it is full");
        } else {
            fifo8_push(&s->tx_fifo, (uint8_t) (value & 0xff));
            uart_tx_start(s);
        }
        break;

    case A_UART_INT_CLR:
        s->reg[R_UART_INT_ST] &= ~((uint32_t) value);
        s->reg[addr / 4] = value;
        if (value & R_UART_INT_CLR_RXFIFO_TOUT_MASK) {
            s->rxfifo_tout = false;
        }
        break;

    case A_UART_INT_ENA:
        s->reg[addr / 4] = value;
        break;

    case A_UART_CLKDIV:
    case A_UART_CONF0:
        uart_tx_suspend(s);
        s->reg[addr / 4] = value;
        uart_update_baud(s);
        uart_tx_resume(s);
        uart_tx_start(s);
        break;

    case A_UART_AUTOBAUD:
        /* If autobaud is enabled, pretend that sufficient number of edges on the RXD line
         * have been received instantly. Autobaud is only used in the ROM bootloader,
         * and it doesn't care if the result is ready immediately.
         */
        if (FIELD_EX32(value, UART_AUTOBAUD, EN)) {
            s->reg[R_UART_RXD_CNT] = 0x3FF;
        } else {
            s->reg[R_UART_RXD_CNT] = 0;
        }
        s->reg[addr / 4] = value;
        break;

    case A_UART_INT_RAW:
    case A_UART_INT_ST:
    case A_UART_STATUS:
        /* no-op */
        break;

    case A_UART_CONF1:
        s->reg[addr / 4] = value;
        s->tx_empty_threshold = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TXFIFO_EMPTY_THRD);
        s->rx_full_threshold = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, RXFIFO_FULL_THRD);
        /* On the ESP32, rx_tout_thres is in units of (bit_time * 8).
         * Note this is different on later chips.
         */
        s->rx_tout_thres = 8 * FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TOUT_THRD);
        s->rx_tout_ena = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TOUT_EN) != 0;
        esp32_uart_set_rx_timeout(s);
        esp32_uart_update_irq(s);
        break;

    default:
        if (addr > sizeof(s->reg)) {
            error_report("esp_uart: write to addr=0x%x out of bounds\n", (uint32_t) addr);
        } else {
            s->reg[addr / 4] = value;
        }
        break;

    }
    esp32_uart_update_irq(s);
}


/* Take the next byte from the TX FIFO into the shift register at start_ns. */
static void uart_tx_start_at(ESP32UARTState *s, int64_t start_ns,
                             uint64_t frame_ns)
{
    s->tx_shift = fifo8_pop(&s->tx_fifo);
    s->tx_busy = true;
    s->tx_frame_left = 65536;
    s->tx_end_ns = start_ns + frame_ns;
    timer_mod_ns(&s->tx_timer, s->tx_end_ns);
    s->tx_timed = true;
    /* A UHCI feeding this UART can refill the slot just freed. */
    notifier_list_notify(&s->dma_notifiers, s);
}

/* Start a frame now if the line is idle, a byte waits and the UART runs. */
static void uart_tx_start(ESP32UARTState *s)
{
    uint64_t frame_ns = uart_frame_ns(s);

    if (s->tx_busy || fifo8_is_empty(&s->tx_fifo) || !frame_ns) {
        return;
    }
    uart_tx_start_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), frame_ns);
    esp32_uart_update_irq(s);
}

/*
 * The frame has been sent: hand the byte to the backend. A backend that
 * can't take it holds the line until it can, as flow control would; then
 * this returns false.
 */
static bool uart_tx_emit(ESP32UARTState *s)
{
    if (qemu_chr_fe_backend_open(&s->chr) &&
        qemu_chr_fe_write(&s->chr, &s->tx_shift, 1) != 1) {
        if (!s->tx_watch_handle) {
            s->tx_watch_handle = qemu_chr_fe_add_watch(&s->chr,
                                                       G_IO_OUT | G_IO_HUP,
                                                       uart_tx_chr_ready, s);
        }
        if (s->tx_watch_handle) {
            s->tx_wait_chr = true;
            return false;
        }
        /* A backend that can't be waited on loses the byte. */
    }
    return true;
}

/*
 * Bring the transmitter up to the present: every frame whose time has
 * passed has been sent, and the next byte followed it onto the line
 * without a gap. Register accesses call this first, so what software sees
 * does not depend on how promptly the timer callback runs.
 */
static void uart_tx_advance(ESP32UARTState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    while (s->tx_timed && now >= s->tx_end_ns) {
        int64_t end = s->tx_end_ns;
        uint64_t frame_ns;

        timer_del(&s->tx_timer);
        s->tx_timed = false;
        if (!uart_tx_emit(s)) {
            break;
        }
        s->tx_busy = false;
        frame_ns = uart_frame_ns(s);
        if (!fifo8_is_empty(&s->tx_fifo) && frame_ns) {
            uart_tx_start_at(s, end, frame_ns);
        }
    }
    esp32_uart_update_irq(s);
}

static void uart_tx_timer_cb(void *opaque)
{
    uart_tx_advance(ESP32_UART(opaque));
}

static gboolean uart_tx_chr_ready(void *do_not_use, GIOCondition cond,
                                  void *opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    s->tx_watch_handle = 0;
    if (s->tx_wait_chr) {
        s->tx_wait_chr = false;
        if (uart_tx_emit(s)) {
            s->tx_busy = false;
            uart_tx_start(s);
            esp32_uart_update_irq(s);
        }
    }
    return G_SOURCE_REMOVE;
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    if (size == 0) {
        return;
    }

    /* If we can receive anything: cancel any pending RX timeout timer,
     * and clear the receive timeout flag.
     */
    if (fifo8_num_free(&s->rx_fifo) > 0) {
        timer_del(&s->rx_timeout_timer);
        s->rxfifo_tout = false;
    }

    /* Move the data into the FIFO */
    for (int i = 0; i < size && fifo8_num_free(&s->rx_fifo) > 0; i++) {
        fifo8_push(&s->rx_fifo, buf[i]);
    }

    /* A UHCI attached to this UART takes the data from the RX FIFO. */
    s->in_receive = true;
    notifier_list_notify(&s->dma_notifiers, s);
    s->in_receive = false;

    /* Receive throttling: some applications (in particular the ESP32 ROM bootloader)
     * may work incorrectly if the data comes in much faster than what UART baud rate
     * would allow. This code adds a delay every UART_FIFO_LENGTH bytes, to make the
     * average data rate match the configured baud rate.
     * This doesn't need to be very precise, so only add the delay if the FIFO is full
     * (which most likely means that more data will come).
     */
    if (fifo8_is_full(&s->rx_fifo)) {
        s->throttle_rx = true;
        const int bits_per_symbol = 10;
        int64_t throttle_time_ns = (int64_t) UART_FIFO_LENGTH * bits_per_symbol * NANOSECONDS_PER_SECOND / s->baud_rate;
        timer_mod_ns(&s->throttle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     throttle_time_ns);
    }

    esp32_uart_set_rx_timeout(s);
    esp32_uart_update_irq(s);
}

/* [spec:nuos:req:emu.esp32.uhci] */
void esp32_uart_add_dma_notifier(ESP32UARTState *s, Notifier *n)
{
    notifier_list_add(&s->dma_notifiers, n);
}

/* [spec:nuos:req:emu.esp32.uhci] */
unsigned esp32_uart_dma_tx_free(ESP32UARTState *s)
{
    return fifo8_num_free(&s->tx_fifo);
}

/* [spec:nuos:req:emu.esp32.uhci] */
void esp32_uart_dma_tx_push(ESP32UARTState *s, uint8_t byte)
{
    uart_tx_advance(s);
    if (fifo8_num_free(&s->tx_fifo) == 0) {
        return;
    }
    fifo8_push(&s->tx_fifo, byte);
    uart_tx_start(s);
    esp32_uart_update_irq(s);
}

/* [spec:nuos:req:emu.esp32.uhci] */
unsigned esp32_uart_dma_rx_count(ESP32UARTState *s)
{
    return fifo8_num_used(&s->rx_fifo);
}

/*
 * [spec:nuos:req:emu.esp32.uhci]
 * Take a byte from the RX FIFO for the UHCI. While the backend is
 * delivering, it asks again for room once it is done.
 */
uint8_t esp32_uart_dma_rx_pop(ESP32UARTState *s)
{
    uint8_t byte;

    if (fifo8_is_empty(&s->rx_fifo)) {
        return 0;
    }
    byte = fifo8_pop(&s->rx_fifo);
    esp32_uart_update_irq(s);
    if (!s->in_receive) {
        qemu_chr_fe_accept_input(&s->chr);
    }
    return byte;
}

static int uart_can_receive(void *opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    if (s->throttle_rx || !uart_running(s) || !s->baud_rate) {
        return 0;
    }
    return fifo8_num_free(&s->rx_fifo);
}

static void uart_event(void *opaque, QEMUChrEvent event)
{
    /* TODO: handle UART break */
}


static void uart_throttle_timer_cb(void* opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    s->throttle_rx = false;
    qemu_chr_fe_accept_input(&s->chr);
}

static void uart_rx_timeout_timer_cb(void* opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    s->rxfifo_tout = true;
    esp32_uart_update_irq(s);
}

static void esp32_uart_reset_hold(Object *obj, ResetType type)
{
    ESP32UARTState *s = ESP32_UART(obj);

    memset(s->reg, 0, sizeof(s->reg));
    s->reg[R_UART_RXD_CNT] = 0;
    s->reg[R_UART_INT_ST] = 0;
    s->reg[R_UART_INT_RAW] = 0;
    s->reg[R_UART_INT_ENA] = 0;
    s->reg[R_UART_AUTOBAUD] = 0;
    /* Default baud rate divider after reset */
    s->reg[R_UART_CLKDIV] = FIELD_DP32(0, UART_CLKDIV, CLKDIV, 0x2B6);
    s->reg[R_UART_CONF0] = UART_CONF0_RESET;
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    if (s->tx_watch_handle) {
        g_source_remove(s->tx_watch_handle);
        s->tx_watch_handle = 0;
    }
    timer_del(&s->tx_timer);
    s->tx_timed = false;
    s->tx_busy = false;
    s->tx_wait_chr = false;
    s->tx_frame_left = 0;
    timer_del(&s->rx_timeout_timer);
    s->rxfifo_tout = false;
    uart_update_baud(s);
    timer_del(&s->throttle_timer);
    s->throttle_rx = false;
    s->rx_tout_ena = false;
    s->tx_empty_threshold = 0;
    s->rx_full_threshold = 0;
    s->rx_tout_thres = 0;
    qemu_irq_lower(s->irq);
}


static void esp32_uart_realize(DeviceState *dev, Error **errp)
{
    ESP32UARTState *s = ESP32_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, uart_can_receive, uart_receive,
                             uart_event, NULL, s, NULL, true);
}


static void esp32_uart_init(Object *obj)
{
    ESP32UARTState *s = ESP32_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    ESP32UARTClass *class = ESP32_UART_GET_CLASS(obj);

    s->uart_ops = (MemoryRegionOps) {
        .read =  class->uart_read,
        .write = class->uart_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    };

    memory_region_init_io(&s->iomem, obj, &s->uart_ops, s,
                          TYPE_ESP32_UART, UART_REG_CNT * sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    fifo8_create(&s->tx_fifo, UART_FIFO_LENGTH);
    fifo8_create(&s->rx_fifo, UART_FIFO_LENGTH);
    notifier_list_init(&s->dma_notifiers);
    timer_init_ns(&s->throttle_timer, QEMU_CLOCK_VIRTUAL, uart_throttle_timer_cb, s);
    timer_init_ns(&s->rx_timeout_timer, QEMU_CLOCK_VIRTUAL, uart_rx_timeout_timer_cb, s);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, uart_tx_timer_cb, s);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb", uart_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->ref_tick_clk = qdev_init_clock_in(DEVICE(obj), "ref_tick",
                                         uart_clk_update, s,
                                         ClockPreUpdate | ClockUpdate);
}


static const Property esp32_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", ESP32UARTState, chr),
};

static void esp32_uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32UARTClass *class = ESP32_UART_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    /* Populate the virtual attributes and methods here (if any) */
    class->uart_write = uart_write;
    class->uart_read = uart_read;

    rc->phases.hold = esp32_uart_reset_hold;
    dc->realize = esp32_uart_realize;
    device_class_set_props(dc, esp32_uart_properties);
}

static const TypeInfo esp32_uart_info = {
    .name = TYPE_ESP32_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32UARTState),
    .instance_init = esp32_uart_init,
    .class_init = esp32_uart_class_init,
    .class_size = sizeof(ESP32UARTClass)
};

static void esp32_uart_register_types(void)
{
    type_register_static(&esp32_uart_info);
}

type_init(esp32_uart_register_types)
