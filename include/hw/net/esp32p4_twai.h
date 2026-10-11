/*
 * ESP32-P4 Two-Wire Automotive Interface controllers TWAI0 to TWAI2
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ESP32P4_TWAI_H
#define HW_NET_ESP32P4_TWAI_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "net/can_emu.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_TWAI "esp32p4-twai"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4TwaiState, ESP32P4_TWAI)

#define ESP32P4_TWAI_NUM        3
#define ESP32P4_TWAI0_BASE      0x500d7000
#define ESP32P4_TWAI_STRIDE     0x1000
#define ESP32P4_TWAI_SIZE       0x1000

/*
 * How the controllers connect
 * ===========================
 *
 * The device models all three controllers, so that those on one bus
 * arbitrate and acknowledge each other bit by bit. Controller n is MMIO
 * region n and sysbus IRQ n (TWAIn_INTR, interrupt source 40 + n), takes
 * its function clock TWAIn_CLK on clock input "twai<n>" and its reset,
 * HP_SYS_CLKRST_RST_EN_CANn, on line n of ESP32P4_TWAI_RESET. Its
 * registers are behind TWAIn_APB_CLK, which the machine gates.
 *
 * Its pad signals are named GPIOs, line n each: ESP32P4_TWAI_TX,
 * _BUS_OFF, _CLKOUT and _STANDBY out (twai_tx, twai_bus_off_on,
 * twai_clkout and twai_standby) and ESP32P4_TWAI_RX in (twai_rx).
 *
 * Links: "gpio", the GPIO matrix, whose routing of twai_tx and twai_rx
 * says what each controller hears and drives; and "canbus0" to
 * "canbus2", the QEMU CAN bus (-object can-bus) standing for the bus
 * behind a transceiver on the pads of each controller. Controllers on
 * one can-bus share that bus with each other and with the bus's other
 * clients, such as can-host-socketcan.
 */
#define ESP32P4_TWAI_RESET      "reset"
#define ESP32P4_TWAI_TX         "tx"
#define ESP32P4_TWAI_BUS_OFF    "bus-off-on"
#define ESP32P4_TWAI_CLKOUT     "clkout"
#define ESP32P4_TWAI_STANDBY    "standby"
#define ESP32P4_TWAI_RX         "rx"

/* The 64-byte receive FIFO and the most messages it counts. */
#define ESP32P4_TWAI_FIFO_SIZE  64
#define ESP32P4_TWAI_MAX_MSGS   64

/*
 * The buses a controller can be on, which index ESP32P4TwaiState.ep and
 * .seg_idle_at: one per QEMU CAN bus, numbered by the lowest controller
 * on it, and one per controller whose twai_rx hears only its own twai_tx.
 */
#define ESP32P4_TWAI_SEGS       (2 * ESP32P4_TWAI_NUM)

/* The most timed actions one frame on a bus takes. */
#define ESP32P4_TWAI_ACTIONS    48

typedef struct ESP32P4TwaiAction {
    /* The bit boundary, counted from the start of SOF, it happens at. */
    int32_t boundary;
    uint8_t kind;
    int8_t ctrl;
    uint16_t arg;
} ESP32P4TwaiAction;

/* A frame, from SOF until the bus is idle again, on one bus. */
typedef struct ESP32P4TwaiEpisode {
    ESP32P4TwaiState *s;
    int seg;
    /* A frame is on the bus. */
    bool active;
    /* A start is scheduled at start_at. */
    bool start_pending;
    int64_t start_at;
    int64_t t_sof;
    /*
     * The bit time of the winning transmitter, exactly, and in
     * picoseconds, rounded down, for receivers at other bit times.
     */
    ESP32P4ClkPeriod bit;
    int64_t bit_ps;
    /* The controllers that took part from SOF, and those since gone. */
    uint8_t members;
    uint8_t departed;
    /* Those error passive at SOF, who signal errors with passive flags. */
    uint8_t passive;
    /* The receivers that acknowledge the frame. */
    uint8_t ackers;
    /* The bit an active error flag first destroys the frame at. */
    int32_t destroy_at;
    /*
     * The contenders, and the bit each lost arbitration at, or -1, with
     * its ARB_LOST_CAP value.
     */
    uint8_t contenders;
    int16_t lost_bit[ESP32P4_TWAI_NUM];
    uint8_t lost_alc[ESP32P4_TWAI_NUM];
    /*
     * A contender's bit error where its frame differed from another's
     * after arbitration, or -1, with its error code.
     */
    int16_t coll_bit[ESP32P4_TWAI_NUM];
    uint8_t coll_ecc[ESP32P4_TWAI_NUM];
    /* The contenders whose identical frames survived to the end. */
    uint8_t survivors;
    /* The controller whose frame is on the bus, and the frame. */
    int8_t winner;
    uint8_t frame[13];
    /* The bit the winner stopped driving at, or -1. */
    int16_t tx_stop;
    /* 0 until the ACK slot, then 1 if acknowledged, 2 if not. */
    uint8_t ack_state;
    /* The last action done, as (boundary << 8) | order. */
    int64_t done_key;
    uint32_t nact;
    ESP32P4TwaiAction act[ESP32P4_TWAI_ACTIONS];
    int64_t idle_at;
    QEMUTimer *timer;
} ESP32P4TwaiEpisode;

typedef struct ESP32P4TwaiCtrl {
    ESP32P4TwaiState *s;
    int idx;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq tx_out;
    qemu_irq bus_off_out;
    qemu_irq clkout_out;
    qemu_irq standby_out;
    Clock *clk;
    CanBusState *canbus;
    CanBusClientState client;

    /* Registers, as software sees them. */
    uint32_t mode;
    uint32_t status;
    uint32_t ir;
    uint32_t ier;
    uint32_t btr0;
    uint32_t btr1;
    uint32_t alc;
    uint32_t ecc;
    uint32_t ewl;
    uint32_t rec;
    uint32_t tec;
    uint8_t acr[4];
    uint8_t amr[4];
    uint8_t txbuf[13];
    uint32_t cdr;
    uint32_t sw_standby;
    uint32_t hw_cfg;
    uint32_t hw_standby_cnt;
    uint32_t idle_intr_cnt;
    uint32_t eco_cfg;
    uint32_t ts_prescaler;
    uint32_t ts_cfg;

    /* The receive FIFO: its bytes and the messages in it. */
    uint8_t fifo[ESP32P4_TWAI_FIFO_SIZE];
    uint32_t fifo_rd;
    uint32_t fifo_used;
    uint32_t rmc;
    uint32_t msg_rd;
    uint8_t msg_len[ESP32P4_TWAI_MAX_MSGS];
    bool msg_valid[ESP32P4_TWAI_MAX_MSGS];
    uint32_t msg_ts[ESP32P4_TWAI_MAX_MSGS];
    bool doi_armed;

    /* The capture registers hold their value until read. */
    bool alc_held;
    bool ecc_held;

    /* The transmission software requested, and when. */
    bool tx_pending;
    int64_t tx_req_at;
    bool tx_single_shot;
    bool tx_self_rx;

    /* Error state: 0 active, 1 passive, 2 bus-off. */
    uint8_t err_state;
    /*
     * Counter writes in reset mode, acted on as reset mode ends: a TEC
     * of 255 forces bus-off, and a TEC write during bus-off ends it.
     */
    bool counters_written;
    bool busoff_clear;

    /* When the controller has seen 11 recessive bits since joining. */
    int64_t sync_at;
    /* When it may next start a frame (Suspend Transmission). */
    int64_t ready_at;
    /*
     * Bus-off recovery: counting runs of 11 recessive bits from mark,
     * recov_runs of them counted so far.
     */
    bool recovering;
    bool recov_paused;
    int64_t recov_mark;
    uint64_t recov_runs;
    QEMUTimer *recov_timer;

    /*
     * A transmission that cannot reach any bus, failing bit by bit: its
     * next step at local_t0, local_bits bit times after local_origin.
     */
    bool local_active;
    uint8_t local_phase;
    uint32_t local_bit;
    int64_t local_t0;
    int64_t local_origin;
    uint64_t local_bits;
    QEMUTimer *local_timer;

    /* What twai_rx hears and whether twai_tx reaches a pad. */
    uint8_t view;
    bool drives;
    bool rx_level;
    bool in_reset;

    /* TWAIn_CLK's exact period, as it last changed. */
    ESP32P4ClkPeriod clk_period;

    /* The timestamp counter: its count at ts_base_ns. */
    uint64_t ts_count_base;
    int64_t ts_base_ns;
    QEMUTimer *ts_timer;

    /* The transceiver standby signal and the bus idle it counts. */
    bool standby;
    int64_t idle_since;
    QEMUTimer *standby_timer;

    bool clkout_level;
} ESP32P4TwaiCtrl;

struct ESP32P4TwaiState {
    SysBusDevice parent_obj;

    ESP32P4TwaiCtrl ctrl[ESP32P4_TWAI_NUM];
    ESP32P4TwaiEpisode ep[ESP32P4_TWAI_SEGS];
    /* When each bus last went idle. */
    int64_t seg_idle_at[ESP32P4_TWAI_SEGS];
    /* The bus number of each controller's QEMU CAN bus. */
    int bus_id[ESP32P4_TWAI_NUM];

    ESP32P4GpioState *gpio;
    Notifier route_notifier;
    Notifier line_source;
    /* A frame of ours is going out to the QEMU CAN bus. */
    bool sending;
};

#endif
