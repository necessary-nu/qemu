/*
 * DesignWare I2C Module.
 *
 * Copyright 2021 Google LLC
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: Synopsys DW_apb_i2c v2.03a as documented in the RP2350
 * Datasheet, section 12.2 (I2C).
 *
 * Controller transfers. Each command taken from the TX FIFO becomes a
 * plan of steps on the wire: an optional STOP (a direction change or
 * RESTART with IC_RESTART_EN clear), an optional START BYTE, the address
 * byte(s) behind a START or RESTART, the data byte and an optional STOP.
 * A step takes the time its SCL periods take at ic_clk, from Table 1105
 * of the datasheet:
 *
 *   SCL high = HCNT + SPKLEN + 7, SCL low = LCNT + 1 (one bit is both)
 *   START: tHD;STA = HCNT
 *   RESTART: SCL low, tSU;STA (SS: SS_SCL_LCNT, FS: FS_SCL_HCNT), tHD;STA
 *   STOP: SCL low, tSU;STO = HCNT, then tBUF = LCNT
 *   byte: 9 bits (8 data bits and the acknowledge)
 *
 * The step's effect on the QEMU I2C bus (start, send, receive, end) is
 * applied when the step completes, which is when hardware samples the
 * acknowledge. The command is popped from the TX FIFO when its first
 * step begins, and received bytes enter the RX FIFO as their byte ends.
 *
 * The bus is modelled at byte level: SDA and SCL are not toggled bit by
 * bit, so arbitration (ARB_LOST) and high-speed mode cannot occur. The
 * device holds SCL low while a controller waits for a command or the RX
 * FIFO, and while a target waits for software, and waits for SCL and
 * SDA to be high before a START.
 *
 * Target transfers. The target logic answers on the bus through
 * TYPE_DESIGNWARE_I2C_TARGET slaves. QEMU's slave interface completes
 * every byte at once, so a target can only stretch SCL toward a
 * controller that asks first: a DW_apb_i2c controller does, before each
 * data byte. Other QEMU bus masters read 0xff from a target with an
 * empty TX FIFO, and bytes they send to a full RX FIFO are lost.
 */

#include "qemu/osdep.h"

#include "hw/i2c/designware_i2c.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"

/* Register offsets. */
#define A_CON               0x00
#define A_TAR               0x04
#define A_SAR               0x08
#define A_DATA_CMD          0x10
#define A_SS_SCL_HCNT       0x14
#define A_SS_SCL_LCNT       0x18
#define A_FS_SCL_HCNT       0x1c
#define A_FS_SCL_LCNT       0x20
#define A_INTR_STAT         0x2c
#define A_INTR_MASK         0x30
#define A_RAW_INTR_STAT     0x34
#define A_RX_TL             0x38
#define A_TX_TL             0x3c
#define A_CLR_INTR          0x40
#define A_CLR_RX_UNDER      0x44
#define A_CLR_RX_OVER       0x48
#define A_CLR_TX_OVER       0x4c
#define A_CLR_RD_REQ        0x50
#define A_CLR_TX_ABRT       0x54
#define A_CLR_RX_DONE       0x58
#define A_CLR_ACTIVITY      0x5c
#define A_CLR_STOP_DET      0x60
#define A_CLR_START_DET     0x64
#define A_CLR_GEN_CALL      0x68
#define A_ENABLE            0x6c
#define A_STATUS            0x70
#define A_TXFLR             0x74
#define A_RXFLR             0x78
#define A_SDA_HOLD          0x7c
#define A_TX_ABRT_SOURCE    0x80
#define A_SLV_DATA_NACK     0x84
#define A_DMA_CR            0x88
#define A_DMA_TDLR          0x8c
#define A_DMA_RDLR          0x90
#define A_SDA_SETUP         0x94
#define A_ACK_GENERAL_CALL  0x98
#define A_ENABLE_STATUS     0x9c
#define A_FS_SPKLEN         0xa0
#define A_CLR_RESTART_DET   0xa8
#define A_SMBUS_INTR_MASK   0xcc
#define A_COMP_PARAM_1      0xf4
#define A_COMP_VERSION      0xf8
#define A_COMP_TYPE         0xfc

#define COMP_TYPE           0x44570140

#define CON_MASTER_MODE             BIT(0)
#define CON_SPEED_SHIFT             1
#define CON_SPEED_MASK              (3u << CON_SPEED_SHIFT)
#define CON_10BITADDR_SLAVE         BIT(3)
#define CON_10BITADDR_MASTER        BIT(4)
#define CON_RESTART_EN              BIT(5)
#define CON_SLAVE_DISABLE           BIT(6)
#define CON_STOP_DET_IFADDRESSED    BIT(7)
#define CON_TX_EMPTY_CTRL           BIT(8)
#define CON_RX_FIFO_FULL_HLD_CTRL   BIT(9)
#define CON_WRITABLE                0x3ffu

#define SPEED_STANDARD              1

#define TAR_ADDR_MASK               0x3ffu
#define TAR_GC_OR_START             BIT(10)
#define TAR_SPECIAL                 BIT(11)

#define DATA_CMD_DAT_MASK           0xffu
#define DATA_CMD_CMD                BIT(8)
#define DATA_CMD_STOP               BIT(9)
#define DATA_CMD_RESTART            BIT(10)
#define DATA_CMD_FIRST_DATA_BYTE    BIT(11)
#define DATA_CMD_WRITABLE           0x7ffu

#define INTR_RX_UNDER               BIT(0)
#define INTR_RX_OVER                BIT(1)
#define INTR_RX_FULL                BIT(2)
#define INTR_TX_OVER                BIT(3)
#define INTR_TX_EMPTY               BIT(4)
#define INTR_RD_REQ                 BIT(5)
#define INTR_TX_ABRT                BIT(6)
#define INTR_RX_DONE                BIT(7)
#define INTR_ACTIVITY               BIT(8)
#define INTR_STOP_DET               BIT(9)
#define INTR_START_DET              BIT(10)
#define INTR_GEN_CALL               BIT(11)
#define INTR_RESTART_DET            BIT(12)
#define INTR_ALL                    0x1fffu

#define ENABLE_ENABLE               BIT(0)
#define ENABLE_ABORT                BIT(1)
#define ENABLE_TX_CMD_BLOCK         BIT(2)

#define STATUS_ACTIVITY             BIT(0)
#define STATUS_TFNF                 BIT(1)
#define STATUS_TFE                  BIT(2)
#define STATUS_RFNE                 BIT(3)
#define STATUS_RFF                  BIT(4)
#define STATUS_MST_ACTIVITY         BIT(5)
#define STATUS_SLV_ACTIVITY         BIT(6)

#define ABRT_7B_ADDR_NOACK          BIT(0)
#define ABRT_10ADDR1_NOACK          BIT(1)
#define ABRT_10ADDR2_NOACK          BIT(2)
#define ABRT_TXDATA_NOACK           BIT(3)
#define ABRT_GCALL_NOACK            BIT(4)
#define ABRT_GCALL_READ             BIT(5)
#define ABRT_SBYTE_NORSTRT          BIT(9)
#define ABRT_10B_RD_NORSTRT         BIT(10)
#define ABRT_MASTER_DIS             BIT(11)
#define ABRT_SLVFLUSH_TXFIFO        BIT(13)
#define ABRT_SLVRD_INTX             BIT(15)
#define ABRT_USER_ABRT              BIT(16)
#define ABRT_TX_FLUSH_CNT_SHIFT     23

#define DMA_CR_RDMAE                BIT(0)
#define DMA_CR_TDMAE                BIT(1)

#define ENABLE_STATUS_IC_EN         BIT(0)
#define ENABLE_STATUS_SLV_DIS_BUSY  BIT(1)
#define ENABLE_STATUS_SLV_RX_LOST   BIT(2)

#define TEN_BIT_PREFIX              0x78

/* The IP's default ic_clk period, for an unconnected "clk" input. */
#define DEFAULT_IC_CLK_NS           100

#define FIFO_DEPTH                  DESIGNWARE_I2C_TX_FIFO_SIZE

static void dw_i2c_kick(DesignWareI2CState *s);

/* FIFOs. */

static uint32_t tx_count(DesignWareI2CState *s)
{
    return s->tx_num;
}

static uint32_t rx_count(DesignWareI2CState *s)
{
    return s->rx_num;
}

static uint32_t tx_peek(DesignWareI2CState *s)
{
    return s->tx_fifo[s->tx_head];
}

static uint32_t tx_pop(DesignWareI2CState *s)
{
    uint32_t v = s->tx_fifo[s->tx_head];

    s->tx_head = (s->tx_head + 1) % FIFO_DEPTH;
    s->tx_num--;
    return v;
}

static void tx_push(DesignWareI2CState *s, uint32_t v)
{
    s->tx_fifo[(s->tx_head + s->tx_num) % FIFO_DEPTH] = v;
    s->tx_num++;
}

static uint32_t rx_pop(DesignWareI2CState *s)
{
    uint32_t v = s->rx_fifo[s->rx_head];

    s->rx_head = (s->rx_head + 1) % FIFO_DEPTH;
    s->rx_num--;
    return v;
}

static void rx_push(DesignWareI2CState *s, uint32_t v)
{
    s->rx_fifo[(s->rx_head + s->rx_num) % FIFO_DEPTH] = v;
    s->rx_num++;
}

static bool dw_master_mode(DesignWareI2CState *s)
{
    return s->con & CON_MASTER_MODE;
}

static bool dw_slave_enabled(DesignWareI2CState *s)
{
    return (s->enable & ENABLE_ENABLE) && !(s->con & CON_SLAVE_DISABLE);
}

static bool dw_mst_active(DesignWareI2CState *s)
{
    return s->mst_owned || s->mst_stepping || s->plan_pos < s->plan_len;
}

/*
 * ic_en: set while IC_ENABLE.ENABLE is, and after it is cleared until
 * the controller and target state machines are idle.
 */
static bool dw_ic_en(DesignWareI2CState *s)
{
    return (s->enable & ENABLE_ENABLE) || dw_mst_active(s) || s->slv_active;
}

/*
 * TX_EMPTY: the TX FIFO is at or below IC_TX_TL and, with TX_EMPTY_CTRL,
 * the last command popped has left the shift register. With the block
 * disabled the FIFO is held empty, so this follows activity.
 */
static uint32_t dw_raw_intr(DesignWareI2CState *s)
{
    uint32_t raw = s->raw_intr;

    if (dw_ic_en(s) && tx_count(s) <= s->tx_tl &&
        (!(s->con & CON_TX_EMPTY_CTRL) || !s->mst_shifting)) {
        raw |= INTR_TX_EMPTY;
    }
    if (rx_count(s) > s->rx_tl) {
        raw |= INTR_RX_FULL;
    }
    return raw;
}

/*
 * Recompute the interrupt, DMA requests and SCL drive, and apply what
 * happens when ic_en falls: the overrun and underrun interrupts,
 * GEN_CALL and ACTIVITY clear, and so does TX_FLUSH_CNT.
 */
static void dw_i2c_update(DesignWareI2CState *s)
{
    bool ic_en = dw_ic_en(s);
    bool hold;

    if (s->ic_en && !ic_en) {
        s->raw_intr &= ~(INTR_TX_OVER | INTR_RX_OVER | INTR_RX_UNDER |
                         INTR_GEN_CALL | INTR_ACTIVITY);
        s->tx_abrt_source &= MAKE_64BIT_MASK(0, ABRT_TX_FLUSH_CNT_SHIFT);
    }
    s->ic_en = ic_en;

    qemu_set_irq(s->irq, !!(dw_raw_intr(s) & s->intr_mask));

    /*
     * A DMA controller may answer a request from within qemu_set_irq(),
     * moving data through the FIFOs: each request is worked out from the
     * FIFOs as the one before it left them.
     */
    qemu_set_irq(s->dma_tx_req,
                 (s->dma_cr & DMA_CR_TDMAE) && (s->enable & ENABLE_ENABLE) &&
                 tx_count(s) <= s->dma_tdlr);
    qemu_set_irq(s->dma_rx_req,
                 (s->dma_cr & DMA_CR_RDMAE) && rx_count(s) >= s->dma_rdlr + 1);

    /*
     * The controller holds SCL low between commands and while the RX
     * FIFO is full; it lets go while it has a step to perform. The
     * target holds SCL low while it waits for software.
     */
    hold = (s->mst_owned && !s->mst_stepping &&
            (s->plan_pos >= s->plan_len || s->mst_rx_stall)) || s->slv_hold;
    if (hold != s->scl_hold) {
        s->scl_hold = hold;
        qemu_set_irq(s->scl_oe, hold);
    }
}

static uint32_t dw_flush_tx(DesignWareI2CState *s)
{
    uint32_t n = tx_count(s);

    s->tx_head = s->tx_num = 0;
    /* A plan for a command still in the FIFO goes with it. */
    if (!s->plan_popped) {
        s->plan_len = s->plan_pos = 0;
    }
    return n;
}

static void dw_flush_rx(DesignWareI2CState *s)
{
    s->rx_head = s->rx_num = 0;
    s->mst_rx_stall = false;
}

static void dw_push_rx(DesignWareI2CState *s, uint32_t data)
{
    if (!(s->enable & ENABLE_ENABLE)) {
        /* The FIFO is held in reset: the byte is lost. */
        return;
    }
    if (rx_count(s) == FIFO_DEPTH) {
        s->raw_intr |= INTR_RX_OVER;
        return;
    }
    rx_push(s, data);
}

/*
 * A transmit abort: the source is recorded, and the TX and RX FIFOs are
 * flushed and held flushed until TX_ABRT is cleared.
 */
static void dw_tx_abort(DesignWareI2CState *s, uint32_t source)
{
    uint32_t flushed = dw_flush_tx(s);

    dw_flush_rx(s);
    s->raw_intr |= INTR_TX_ABRT;
    s->tx_abrt_source |= source;
    s->tx_abrt_source = deposit32(s->tx_abrt_source, ABRT_TX_FLUSH_CNT_SHIFT,
                                  32 - ABRT_TX_FLUSH_CNT_SHIFT, flushed);
}

/* Timing. */

static uint32_t dw_speed(DesignWareI2CState *s)
{
    return (s->con & CON_SPEED_MASK) >> CON_SPEED_SHIFT;
}

static uint32_t dw_hcnt(DesignWareI2CState *s)
{
    return dw_speed(s) == SPEED_STANDARD ? s->ss_scl_hcnt : s->fs_scl_hcnt;
}

static uint32_t dw_lcnt(DesignWareI2CState *s)
{
    return dw_speed(s) == SPEED_STANDARD ? s->ss_scl_lcnt : s->fs_scl_lcnt;
}

static uint64_t dw_bit_cycles(DesignWareI2CState *s)
{
    return (dw_hcnt(s) + s->fs_spklen + 7) + (dw_lcnt(s) + 1);
}

static uint64_t dw_restart_cycles(DesignWareI2CState *s)
{
    uint32_t su_sta = dw_speed(s) == SPEED_STANDARD ? s->ss_scl_lcnt
                                                    : s->fs_scl_hcnt;

    return dw_lcnt(s) + 1 + su_sta + dw_hcnt(s);
}

static uint64_t dw_step_cycles(DesignWareI2CState *s, DesignWareI2CStep step)
{
    uint64_t byte = 9 * dw_bit_cycles(s);

    switch (step) {
    case DW_I2C_STEP_STOP:
        return dw_lcnt(s) + 1 + dw_hcnt(s) + dw_lcnt(s);
    case DW_I2C_STEP_SBYTE:
        return dw_hcnt(s) + byte;
    case DW_I2C_STEP_ADDR:
        return (s->mst_owned ? dw_restart_cycles(s) : dw_hcnt(s)) + byte;
    case DW_I2C_STEP_ADDR_R:
        return dw_restart_cycles(s) + byte;
    default:
        return byte;
    }
}

static int64_t dw_cycles_ns(DesignWareI2CState *s, uint64_t cycles)
{
    if (clock_get(s->clk)) {
        return clock_ticks_to_ns(s->clk, cycles);
    }
    return cycles * DEFAULT_IC_CLK_NS;
}

/* Target logic. */

/*
 * Release SCL after waiting for software, and let the controller that is
 * addressing the target go on if it is a DesignWare one.
 */
static void dw_slv_release(DesignWareI2CState *s)
{
    DeviceState *parent;

    if (!s->slv_hold) {
        return;
    }
    s->slv_hold = false;
    dw_i2c_update(s);
    if (s->slv_bus) {
        parent = BUS(s->slv_bus)->parent;
        if (object_dynamic_cast(OBJECT(parent), TYPE_DESIGNWARE_I2C)) {
            dw_i2c_kick(DESIGNWARE_I2C(parent));
        }
    }
}

/*
 * Whether the target can take part in the next data byte, asked by a
 * controller before it clocks the byte. A target that has to wait for
 * software holds SCL low: for a read with an empty TX FIFO it raises
 * RD_REQ, for a write it waits for room in the RX FIFO.
 */
static bool dw_slv_ready(DesignWareI2CState *s, bool read)
{
    if (!s->slv_active || !(s->enable & ENABLE_ENABLE)) {
        return true;
    }
    if (read) {
        if (tx_count(s)) {
            return true;
        }
        if (!s->slv_hold) {
            s->slv_hold = true;
            s->raw_intr |= INTR_RD_REQ;
            dw_i2c_update(s);
        }
        return false;
    }
    if (rx_count(s) < FIFO_DEPTH) {
        return true;
    }
    if (!s->slv_hold) {
        s->slv_hold = true;
        dw_i2c_update(s);
    }
    return false;
}

/* [spec:nuos:req:emu.i2c] */
static bool dw_slv_match(DesignWareI2CState *s, uint8_t address,
                         bool broadcast)
{
    if (!dw_slave_enabled(s)) {
        return false;
    }
    s->raw_intr |= INTR_START_DET;
    if (broadcast) {
        s->slv_gc_match = s->ack_general_call & 1;
        dw_i2c_update(s);
        return s->slv_gc_match;
    }
    dw_i2c_update(s);
    if (s->con & CON_10BITADDR_SLAVE) {
        return address == (TEN_BIT_PREFIX | ((s->sar >> 8) & 3));
    }
    return address == (s->sar & 0x7f);
}

/* [spec:nuos:req:emu.i2c] */
static int dw_slv_start(DesignWareI2CState *s, I2CBus *bus, bool read)
{
    bool restart = s->slv_active;
    bool ten = s->con & CON_10BITADDR_SLAVE;

    s->raw_intr |= INTR_START_DET | INTR_ACTIVITY;
    s->slv_bus = bus;
    if (restart) {
        s->raw_intr |= INTR_RESTART_DET;
    } else {
        s->slv_active = true;
        s->slv_data = false;
        s->slv_gc = s->slv_gc_match;
        s->slv_10bit = ten && !s->slv_gc ? 1 : 0;
        if (s->slv_gc) {
            s->raw_intr |= INTR_GEN_CALL;
        }
    }
    s->slv_gc_match = false;
    s->slv_first = true;
    s->slv_read = read;
    if (!read) {
        if (ten && restart && !s->slv_gc) {
            /* A repeated START in 10-bit mode sends both bytes again. */
            s->slv_10bit = 1;
        }
        dw_i2c_update(s);
        return 0;
    }
    /*
     * A read: a general call cannot be one, and in 10-bit mode only a
     * RESTART after both address bytes matched addresses this target.
     */
    if (s->slv_gc || (ten && !(restart && s->slv_10bit == 2))) {
        dw_i2c_update(s);
        return 1;
    }
    s->raw_intr |= INTR_RD_REQ;
    if (!tx_count(s)) {
        s->slv_hold = true;
    }
    dw_i2c_update(s);
    return 0;
}

/* [spec:nuos:req:emu.i2c] */
static void dw_slv_stop(DesignWareI2CState *s)
{
    if (!s->slv_active) {
        return;
    }
    if (!(s->slv_gc && (s->con & CON_STOP_DET_IFADDRESSED))) {
        s->raw_intr |= INTR_STOP_DET;
    }
    s->slv_active = false;
    s->slv_hold = false;
    s->slv_bus = NULL;
    dw_i2c_update(s);
}

/*
 * The controller did not acknowledge the last byte it read: the transfer
 * is done, and what software left in the TX FIFO is flushed.
 */
/* [spec:nuos:req:emu.i2c] */
static void dw_slv_nack(DesignWareI2CState *s)
{
    if (!s->slv_active || !s->slv_read) {
        return;
    }
    s->raw_intr |= INTR_RX_DONE;
    if (tx_count(s)) {
        dw_tx_abort(s, ABRT_SLVFLUSH_TXFIFO);
    }
    dw_i2c_update(s);
}

/* A byte from the controller: 0 to acknowledge it. */
/* [spec:nuos:req:emu.i2c] */
static int dw_slv_send(DesignWareI2CState *s, uint8_t data)
{
    if (!s->slv_active) {
        return 1;
    }
    if (!(s->enable & ENABLE_ENABLE)) {
        /* Disabled during the transfer: the target NACKs from here on. */
        s->enable_status_slv |= ENABLE_STATUS_SLV_DIS_BUSY;
        if (s->slv_10bit != 1) {
            s->enable_status_slv |= ENABLE_STATUS_SLV_RX_LOST;
        }
        return 1;
    }
    if (s->slv_10bit == 1) {
        if (data != (s->sar & 0xff)) {
            s->slv_active = false;
            s->slv_bus = NULL;
            dw_i2c_update(s);
            return 1;
        }
        s->slv_10bit = 2;
        return 0;
    }
    if (s->slv_data_nack_only & 1) {
        return 1;
    }
    s->slv_data = true;
    if (rx_count(s) == FIFO_DEPTH && (s->con & CON_RX_FIFO_FULL_HLD_CTRL)) {
        qemu_log_mask(LOG_UNIMP, "%s: the controller cannot be held off "
                      "while the RX FIFO is full; byte lost\n",
                      DEVICE(s)->canonical_path);
        return 0;
    }
    dw_push_rx(s, data | (s->slv_first ? DATA_CMD_FIRST_DATA_BYTE : 0));
    s->slv_first = false;
    dw_i2c_update(s);
    return 0;
}

/* A byte for the controller. */
/* [spec:nuos:req:emu.i2c] */
static uint8_t dw_slv_recv(DesignWareI2CState *s)
{
    uint32_t cmd;

    if (!s->slv_active || !(s->enable & ENABLE_ENABLE)) {
        if (s->slv_active) {
            s->enable_status_slv |= ENABLE_STATUS_SLV_DIS_BUSY;
        }
        return 0xff;
    }
    if (!tx_count(s)) {
        s->raw_intr |= INTR_RD_REQ;
        dw_i2c_update(s);
        qemu_log_mask(LOG_UNIMP, "%s: the controller cannot be held off "
                      "until software supplies data; it reads 0xff\n",
                      DEVICE(s)->canonical_path);
        return 0xff;
    }
    cmd = tx_pop(s);
    if (cmd & DATA_CMD_CMD) {
        dw_tx_abort(s, ABRT_SLVRD_INTX);
        dw_i2c_update(s);
        return 0xff;
    }
    dw_i2c_update(s);
    return cmd & DATA_CMD_DAT_MASK;
}

/* Controller. */

static void dw_mst_plan_add(DesignWareI2CState *s, DesignWareI2CStep step)
{
    assert(s->plan_len < DW_I2C_MAX_STEPS);
    s->plan[s->plan_len++] = step;
}

/* Replace what is left of the plan with a STOP, if the bus is ours. */
static void dw_mst_plan_stop(DesignWareI2CState *s)
{
    s->plan_len = s->plan_pos = 0;
    s->plan_popped = true;
    if (s->mst_owned) {
        dw_mst_plan_add(s, DW_I2C_STEP_STOP);
    }
}

/* [spec:nuos:req:emu.i2c] */
static void dw_mst_abort(DesignWareI2CState *s, uint32_t source)
{
    s->mst_shifting = false;
    dw_tx_abort(s, source);
    dw_mst_plan_stop(s);
}

/*
 * Plan the command at the head of the TX FIFO. Commands that cannot be
 * carried out with the current settings abort here.
 */
/* [spec:nuos:req:emu.i2c] */
static bool dw_mst_plan(DesignWareI2CState *s, uint32_t cmd)
{
    bool read = cmd & DATA_CMD_CMD;
    bool ten = s->con & CON_10BITADDR_MASTER;
    bool restart_en = s->con & CON_RESTART_EN;
    bool special = s->tar & TAR_SPECIAL;
    bool gcall = special && !(s->tar & TAR_GC_OR_START);
    bool sbyte = special && (s->tar & TAR_GC_OR_START);
    bool addr = true;

    s->plan_len = s->plan_pos = 0;
    s->plan_popped = false;
    s->mst_cmd = cmd;

    if (gcall && read) {
        dw_mst_abort(s, ABRT_GCALL_READ);
        return false;
    }
    if (s->mst_owned) {
        if ((cmd & DATA_CMD_RESTART) || read != s->mst_read) {
            if (!restart_en) {
                dw_mst_plan_add(s, DW_I2C_STEP_STOP);
            }
        } else {
            addr = false;
        }
    }
    if (addr) {
        if (ten && read && !restart_en && !gcall) {
            dw_mst_abort(s, ABRT_10B_RD_NORSTRT);
            return false;
        }
        if (sbyte && !restart_en) {
            dw_mst_abort(s, ABRT_SBYTE_NORSTRT);
            return false;
        }
        if (sbyte) {
            dw_mst_plan_add(s, DW_I2C_STEP_SBYTE);
        }
        dw_mst_plan_add(s, DW_I2C_STEP_ADDR);
        if (ten && !gcall) {
            dw_mst_plan_add(s, DW_I2C_STEP_ADDR2);
            if (read) {
                dw_mst_plan_add(s, DW_I2C_STEP_ADDR_R);
            }
        }
    }
    dw_mst_plan_add(s, DW_I2C_STEP_DATA);
    if (cmd & DATA_CMD_STOP) {
        dw_mst_plan_add(s, DW_I2C_STEP_STOP);
    }
    return true;
}

/* Whether every DesignWare target addressed on the bus is ready. */
static bool dw_mst_targets_ready(DesignWareI2CState *s, bool read)
{
    I2CNode *node;

    QLIST_FOREACH(node, &s->bus->current_devs, next) {
        DesignWareI2CTarget *t = (DesignWareI2CTarget *)
            object_dynamic_cast(OBJECT(node->elt),
                                TYPE_DESIGNWARE_I2C_TARGET);

        if (t && t->controller && !dw_slv_ready(t->controller, read)) {
            return false;
        }
    }
    return true;
}

/*
 * Begin the next step of the plan when the bus allows: a START needs an
 * idle bus, with SCL and SDA high and no other QEMU bus master; other
 * steps need SCL high, so wait while another device stretches it. A
 * data byte also waits for room in the RX FIFO when RX_FIFO_FULL_HLD_CTRL
 * says so.
 */
/* [spec:nuos:req:emu.i2c] */
static void dw_mst_begin_step(DesignWareI2CState *s)
{
    DesignWareI2CStep step = s->plan[s->plan_pos];
    bool start = step == DW_I2C_STEP_SBYTE || step == DW_I2C_STEP_ADDR ||
                 step == DW_I2C_STEP_ADDR_R;
    bool read = s->mst_cmd & DATA_CMD_CMD;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (step == DW_I2C_STEP_DATA && read && rx_count(s) == FIFO_DEPTH &&
        (s->con & CON_RX_FIFO_FULL_HLD_CTRL)) {
        s->mst_rx_stall = true;
        dw_i2c_update(s);
        return;
    }
    s->mst_rx_stall = false;
    dw_i2c_update(s);
    if (!s->mst_owned) {
        if (!s->scl_in || !s->sda_in) {
            return;
        }
        if (i2c_bus_busy(s->bus)) {
            s->mst_polling = true;
            timer_mod(s->timer, now + dw_cycles_ns(s, dw_bit_cycles(s)));
            return;
        }
    } else if (!s->scl_in) {
        return;
    }
    if (step == DW_I2C_STEP_DATA && !dw_mst_targets_ready(s, read)) {
        return;
    }

    if (!s->plan_popped) {
        tx_pop(s);
        s->plan_popped = true;
        s->mst_shifting = true;
    }
    if (start) {
        s->raw_intr |= INTR_START_DET | INTR_ACTIVITY;
    }
    s->mst_stepping = true;
    timer_mod(s->timer, now + dw_cycles_ns(s, dw_step_cycles(s, step)));
    dw_i2c_update(s);
}

static void dw_mst_end_xfer(DesignWareI2CState *s)
{
    if (s->mst_xfer_open) {
        if (s->mst_read && s->mst_read_done) {
            i2c_nack(s->bus);
        }
        i2c_end_transfer(s->bus);
        s->mst_xfer_open = false;
    }
}

/* Finish an abort requested through IC_ENABLE.ABORT. */
static void dw_mst_user_abort_done(DesignWareI2CState *s)
{
    s->mst_user_abort = false;
    s->enable &= ~ENABLE_ABORT;
    dw_tx_abort(s, ABRT_USER_ABRT);
}

/* Apply a completed step to the QEMU bus. */
/* [spec:nuos:req:emu.i2c] */
static void dw_mst_do_step(DesignWareI2CState *s, DesignWareI2CStep step)
{
    bool ten = s->con & CON_10BITADDR_MASTER;
    bool gcall = (s->tar & TAR_SPECIAL) && !(s->tar & TAR_GC_OR_START);
    bool read = s->mst_cmd & DATA_CMD_CMD;
    uint8_t addr;

    switch (step) {
    case DW_I2C_STEP_STOP:
        dw_mst_end_xfer(s);
        s->mst_owned = false;
        s->raw_intr |= INTR_STOP_DET;
        i2c_schedule_pending_master(s->bus);
        if (s->mst_user_abort) {
            dw_mst_user_abort_done(s);
        }
        break;
    case DW_I2C_STEP_SBYTE:
        /* START, 0000 0001 and an acknowledge no device may give. */
        s->mst_owned = true;
        break;
    case DW_I2C_STEP_ADDR:
        if (gcall) {
            addr = 0;
        } else if (ten) {
            addr = TEN_BIT_PREFIX | ((s->tar >> 8) & 3);
        } else {
            addr = s->tar & 0x7f;
        }
        read = read && !(ten && !gcall);
        if (s->mst_xfer_open) {
            if (s->mst_read && s->mst_read_done) {
                i2c_nack(s->bus);
            }
            /*
             * QEMU's bus does not re-address on a repeated START: a
             * different target sees the previous one released instead.
             */
            if (addr != s->mst_addr) {
                i2c_end_transfer(s->bus);
                s->mst_xfer_open = false;
            }
        }
        s->mst_owned = true;
        if (i2c_start_transfer(s->bus, addr, read)) {
            if (s->mst_xfer_open) {
                i2c_end_transfer(s->bus);
                s->mst_xfer_open = false;
            }
            dw_mst_abort(s, gcall ? ABRT_GCALL_NOACK :
                            ten ? ABRT_10ADDR1_NOACK : ABRT_7B_ADDR_NOACK);
            return;
        }
        s->mst_xfer_open = true;
        s->mst_addr = addr;
        s->mst_read = read;
        s->mst_read_done = false;
        s->mst_first_data = true;
        break;
    case DW_I2C_STEP_ADDR2:
        if (i2c_send(s->bus, s->tar & 0xff)) {
            dw_mst_end_xfer(s);
            dw_mst_abort(s, ABRT_10ADDR2_NOACK);
            return;
        }
        break;
    case DW_I2C_STEP_ADDR_R:
        if (i2c_start_transfer(s->bus, s->mst_addr, true)) {
            dw_mst_end_xfer(s);
            dw_mst_abort(s, ABRT_10ADDR1_NOACK);
            return;
        }
        s->mst_read = true;
        s->mst_read_done = false;
        s->mst_first_data = true;
        break;
    case DW_I2C_STEP_DATA:
        s->mst_shifting = false;
        if (read) {
            uint8_t data = i2c_recv(s->bus);

            s->mst_read_done = true;
            dw_push_rx(s, data | (s->mst_first_data ?
                                  DATA_CMD_FIRST_DATA_BYTE : 0));
        } else if (i2c_send(s->bus, s->mst_cmd & DATA_CMD_DAT_MASK)) {
            dw_mst_end_xfer(s);
            dw_mst_abort(s, ABRT_TXDATA_NOACK);
            return;
        }
        s->mst_first_data = false;
        break;
    }
    s->plan_pos++;
    if (s->plan_pos >= s->plan_len) {
        s->plan_len = s->plan_pos = 0;
    }
}

/* Decide what the controller does next, from the state it is in. */
/* [spec:nuos:req:emu.i2c] */
static void dw_mst_advance(DesignWareI2CState *s)
{
    uint32_t cmd;

    if (s->mst_stepping || s->mst_polling) {
        return;
    }
    /* An abort ends the transfer at this step; the FIFO goes after it. */
    if ((s->enable & ENABLE_ABORT) && !s->mst_user_abort) {
        if (s->mst_owned) {
            s->mst_user_abort = true;
            s->mst_shifting = false;
            dw_mst_plan_stop(s);
        } else {
            s->plan_len = s->plan_pos = 0;
            dw_mst_user_abort_done(s);
            return;
        }
    }
    if (s->plan_pos < s->plan_len) {
        dw_mst_begin_step(s);
        return;
    }
    if (!(s->enable & ENABLE_ENABLE)) {
        /* Disabled while receiving: the transfer ends after its byte. */
        if (s->mst_owned && s->mst_read) {
            dw_mst_plan_stop(s);
            dw_mst_begin_step(s);
        }
        return;
    }
    if (!tx_count(s) || (s->enable & ENABLE_TX_CMD_BLOCK)) {
        return;
    }
    if (!dw_master_mode(s)) {
        if (s->con & CON_SLAVE_DISABLE) {
            dw_tx_abort(s, ABRT_MASTER_DIS);
        }
        return;
    }
    cmd = tx_peek(s);
    if (dw_mst_plan(s, cmd) || s->plan_len) {
        dw_mst_begin_step(s);
    }
}

/*
 * Move the controller on. Changes it causes can come back here through
 * the SCL wire; they are taken up in the same call.
 */
static void dw_i2c_kick(DesignWareI2CState *s)
{
    if (s->mst_kicking) {
        s->mst_kick_again = true;
        return;
    }
    s->mst_kicking = true;
    do {
        s->mst_kick_again = false;
        dw_mst_advance(s);
        dw_i2c_update(s);
    } while (s->mst_kick_again);
    s->mst_kicking = false;
}

static void dw_i2c_timer(void *opaque)
{
    DesignWareI2CState *s = opaque;

    if (s->mst_polling) {
        s->mst_polling = false;
    } else if (s->mst_stepping) {
        s->mst_stepping = false;
        dw_mst_do_step(s, s->plan[s->plan_pos]);
    }
    dw_i2c_kick(s);
}

static void dw_i2c_scl_in(void *opaque, int n, int level)
{
    DesignWareI2CState *s = opaque;

    s->scl_in = level;
    dw_i2c_kick(s);
}

static void dw_i2c_sda_in(void *opaque, int n, int level)
{
    DesignWareI2CState *s = opaque;

    s->sda_in = level;
    dw_i2c_kick(s);
}

/* Registers. */

static uint32_t dw_status(DesignWareI2CState *s)
{
    uint32_t v = 0;
    bool mst = dw_mst_active(s);

    if (mst || s->slv_active) {
        v |= STATUS_ACTIVITY;
    }
    if (tx_count(s) < FIFO_DEPTH) {
        v |= STATUS_TFNF;
    }
    if (!tx_count(s)) {
        v |= STATUS_TFE;
    }
    if (rx_count(s)) {
        v |= STATUS_RFNE;
    }
    if (rx_count(s) == FIFO_DEPTH) {
        v |= STATUS_RFF;
    }
    if (mst) {
        v |= STATUS_MST_ACTIVITY;
    }
    if (s->slv_active) {
        v |= STATUS_SLV_ACTIVITY;
    }
    return v;
}

/* The ABRT_SBYTE_NORSTRT source persists until its cause is removed. */
static void dw_clear_abrt_source(DesignWareI2CState *s)
{
    uint32_t keep = 0;

    if (!(s->con & CON_RESTART_EN) && (s->tar & TAR_SPECIAL) &&
        (s->tar & TAR_GC_OR_START)) {
        keep = s->tx_abrt_source & ABRT_SBYTE_NORSTRT;
    }
    s->tx_abrt_source = keep;
}

static uint32_t dw_clear_intr(DesignWareI2CState *s, uint32_t bits)
{
    uint32_t was = s->raw_intr;

    if (dw_mst_active(s) || s->slv_active) {
        /* ACTIVITY stays set while the block is busy on the bus. */
        bits &= ~INTR_ACTIVITY;
    }
    s->raw_intr &= ~bits;
    if (bits & INTR_TX_ABRT) {
        dw_clear_abrt_source(s);
    }
    dw_i2c_kick(s);
    return was;
}

static uint32_t dw_data_cmd_read(DesignWareI2CState *s)
{
    uint32_t v;

    if (!rx_count(s)) {
        s->raw_intr |= INTR_RX_UNDER;
        dw_i2c_update(s);
        return 0;
    }
    v = rx_pop(s);
    if (s->slv_hold && !s->slv_read) {
        dw_slv_release(s);
    }
    dw_i2c_kick(s);
    return v;
}

static void dw_data_cmd_write(DesignWareI2CState *s, uint32_t value)
{
    if (!(s->enable & ENABLE_ENABLE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: IC_DATA_CMD written while "
                      "disabled; the command is lost\n",
                      DEVICE(s)->canonical_path);
        return;
    }
    if (s->raw_intr & INTR_TX_ABRT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: IC_DATA_CMD written with "
                      "TX_ABRT set; the TX FIFO is held flushed\n",
                      DEVICE(s)->canonical_path);
        return;
    }
    if (tx_count(s) == FIFO_DEPTH) {
        s->raw_intr |= INTR_TX_OVER;
        dw_i2c_update(s);
        return;
    }
    if (!dw_master_mode(s) && (value & DATA_CMD_CMD) && s->slv_active &&
        s->slv_read) {
        /* A read command in answer to a read request. */
        dw_tx_abort(s, ABRT_SLVRD_INTX);
        dw_i2c_update(s);
        return;
    }
    tx_push(s, value & DATA_CMD_WRITABLE);
    if (s->slv_hold && s->slv_read) {
        dw_slv_release(s);
    }
    dw_i2c_kick(s);
}

static void dw_enable_write(DesignWareI2CState *s, uint32_t value)
{
    uint32_t old = s->enable;

    value &= ENABLE_ENABLE | ENABLE_ABORT | ENABLE_TX_CMD_BLOCK;
    /* ABORT can only be set while enabled, and only hardware clears it. */
    if (!(old & ENABLE_ENABLE)) {
        value &= ~ENABLE_ABORT;
    }
    value |= old & ENABLE_ABORT;

    if (!(old & ENABLE_ENABLE) && (value & ENABLE_ENABLE)) {
        s->enable_status_slv = 0;
        if (dw_master_mode(s) && !(s->con & CON_SLAVE_DISABLE)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: enabled with both "
                          "MASTER_MODE set and IC_SLAVE_DISABLE clear\n",
                          DEVICE(s)->canonical_path);
        }
    } else if ((old & ENABLE_ENABLE) && !(value & ENABLE_ENABLE)) {
        /* The FIFOs are flushed and held in reset. */
        dw_flush_tx(s);
        dw_flush_rx(s);
        if (s->slv_active) {
            s->enable_status_slv |= ENABLE_STATUS_SLV_DIS_BUSY;
            if (s->slv_data) {
                s->enable_status_slv |= ENABLE_STATUS_SLV_RX_LOST;
            }
        }
    }
    s->enable = value;
    /* A target disabled while it holds SCL lets the transfer go on. */
    dw_slv_release(s);
    dw_i2c_kick(s);
}

/* Registers writable only while IC_ENABLE.ENABLE is clear. */
static bool dw_check_disabled(DesignWareI2CState *s, const char *name)
{
    if (s->enable & ENABLE_ENABLE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s written while enabled; "
                      "ignored\n", DEVICE(s)->canonical_path, name);
        return false;
    }
    return true;
}

/* [spec:nuos:req:emu.i2c] */
static uint64_t dw_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    DesignWareI2CState *s = opaque;
    DesignWareI2CClass *dc = DESIGNWARE_I2C_GET_CLASS(s);

    switch (addr) {
    case A_CON:
        return s->con;
    case A_TAR:
        return s->tar;
    case A_SAR:
        return s->sar;
    case A_DATA_CMD:
        return dw_data_cmd_read(s);
    case A_SS_SCL_HCNT:
        return s->ss_scl_hcnt;
    case A_SS_SCL_LCNT:
        return s->ss_scl_lcnt;
    case A_FS_SCL_HCNT:
        return s->fs_scl_hcnt;
    case A_FS_SCL_LCNT:
        return s->fs_scl_lcnt;
    case A_INTR_STAT:
        return dw_raw_intr(s) & s->intr_mask;
    case A_INTR_MASK:
        return s->intr_mask;
    case A_RAW_INTR_STAT:
        return dw_raw_intr(s);
    case A_RX_TL:
        return s->rx_tl;
    case A_TX_TL:
        return s->tx_tl;
    case A_CLR_INTR:
        dw_clear_intr(s, INTR_ALL & ~(INTR_TX_EMPTY | INTR_RX_FULL));
        return 0;
    case A_CLR_RX_UNDER:
        dw_clear_intr(s, INTR_RX_UNDER);
        return 0;
    case A_CLR_RX_OVER:
        dw_clear_intr(s, INTR_RX_OVER);
        return 0;
    case A_CLR_TX_OVER:
        dw_clear_intr(s, INTR_TX_OVER);
        return 0;
    case A_CLR_RD_REQ:
        dw_clear_intr(s, INTR_RD_REQ);
        return 0;
    case A_CLR_TX_ABRT:
        dw_clear_intr(s, INTR_TX_ABRT);
        return 0;
    case A_CLR_RX_DONE:
        dw_clear_intr(s, INTR_RX_DONE);
        return 0;
    case A_CLR_ACTIVITY:
        return !!(dw_clear_intr(s, INTR_ACTIVITY) & INTR_ACTIVITY);
    case A_CLR_STOP_DET:
        dw_clear_intr(s, INTR_STOP_DET);
        return 0;
    case A_CLR_START_DET:
        dw_clear_intr(s, INTR_START_DET);
        return 0;
    case A_CLR_GEN_CALL:
        dw_clear_intr(s, INTR_GEN_CALL);
        return 0;
    case A_CLR_RESTART_DET:
        dw_clear_intr(s, INTR_RESTART_DET);
        return 0;
    case A_ENABLE:
        return s->enable;
    case A_STATUS:
        return dw_status(s);
    case A_TXFLR:
        return tx_count(s);
    case A_RXFLR:
        return rx_count(s);
    case A_SDA_HOLD:
        return s->sda_hold;
    case A_TX_ABRT_SOURCE:
        return s->tx_abrt_source;
    case A_SLV_DATA_NACK:
        return s->slv_data_nack_only;
    case A_DMA_CR:
        return s->dma_cr;
    case A_DMA_TDLR:
        return s->dma_tdlr;
    case A_DMA_RDLR:
        return s->dma_rdlr;
    case A_SDA_SETUP:
        return s->sda_setup;
    case A_ACK_GENERAL_CALL:
        return s->ack_general_call;
    case A_ENABLE_STATUS:
        if (s->enable & ENABLE_ENABLE) {
            return ENABLE_STATUS_IC_EN;
        }
        return (dw_ic_en(s) ? ENABLE_STATUS_IC_EN : 0) | s->enable_status_slv;
    case A_FS_SPKLEN:
        return s->fs_spklen;
    case A_SMBUS_INTR_MASK:
        if (dc->has_smbus_intr_mask) {
            return s->smbus_intr_mask;
        }
        break;
    case A_COMP_PARAM_1:
        return dc->comp_param_1;
    case A_COMP_VERSION:
        return dc->comp_version;
    case A_COMP_TYPE:
        return COMP_TYPE;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%03"
                  HWADDR_PRIx "\n", DEVICE(s)->canonical_path, addr);
    return 0;
}

/* [spec:nuos:req:emu.i2c] */
static void dw_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    DesignWareI2CState *s = opaque;
    DesignWareI2CClass *dc = DESIGNWARE_I2C_GET_CLASS(s);
    uint32_t speed;

    switch (addr) {
    case A_CON:
        if (dw_check_disabled(s, "IC_CON")) {
            s->con = value & CON_WRITABLE;
            speed = (s->con & CON_SPEED_MASK) >> CON_SPEED_SHIFT;
            /* Out-of-range speeds become the maximum speed mode. */
            if (speed < SPEED_STANDARD || speed > dc->max_speed) {
                s->con = deposit32(s->con, CON_SPEED_SHIFT, 2,
                                   dc->max_speed);
            }
        }
        break;
    case A_TAR:
        if (dw_check_disabled(s, "IC_TAR")) {
            s->tar = value & dc->tar_mask;
        }
        break;
    case A_SAR:
        if (dw_check_disabled(s, "IC_SAR")) {
            s->sar = value & TAR_ADDR_MASK;
        }
        break;
    case A_DATA_CMD:
        dw_data_cmd_write(s, value);
        return;
    case A_SS_SCL_HCNT:
        if (dw_check_disabled(s, "IC_SS_SCL_HCNT")) {
            s->ss_scl_hcnt = MAX(value & 0xffff, 6);
        }
        break;
    case A_SS_SCL_LCNT:
        if (dw_check_disabled(s, "IC_SS_SCL_LCNT")) {
            s->ss_scl_lcnt = MAX(value & 0xffff, 8);
        }
        break;
    case A_FS_SCL_HCNT:
        if (dw_check_disabled(s, "IC_FS_SCL_HCNT")) {
            s->fs_scl_hcnt = MAX(value & 0xffff, 6);
        }
        break;
    case A_FS_SCL_LCNT:
        if (dw_check_disabled(s, "IC_FS_SCL_LCNT")) {
            s->fs_scl_lcnt = MAX(value & 0xffff, 8);
        }
        break;
    case A_INTR_MASK:
        s->intr_mask = value & INTR_ALL;
        break;
    case A_RX_TL:
        s->rx_tl = MIN(value & 0xff, FIFO_DEPTH - 1);
        break;
    case A_TX_TL:
        s->tx_tl = MIN(value & 0xff, FIFO_DEPTH - 1);
        break;
    case A_ENABLE:
        dw_enable_write(s, value);
        return;
    case A_SDA_HOLD:
        if (dw_check_disabled(s, "IC_SDA_HOLD")) {
            s->sda_hold = value & 0xffffff;
        }
        break;
    case A_SLV_DATA_NACK:
        if (dw_check_disabled(s, "IC_SLV_DATA_NACK_ONLY") &&
            !s->slv_active) {
            s->slv_data_nack_only = value & 1;
        }
        break;
    case A_DMA_CR:
        s->dma_cr = value & (DMA_CR_RDMAE | DMA_CR_TDMAE);
        break;
    case A_DMA_TDLR:
        s->dma_tdlr = value & 0xf;
        break;
    case A_DMA_RDLR:
        s->dma_rdlr = value & 0xf;
        break;
    case A_SDA_SETUP:
        if (dw_check_disabled(s, "IC_SDA_SETUP")) {
            s->sda_setup = value & 0xff;
        }
        break;
    case A_ACK_GENERAL_CALL:
        s->ack_general_call = value & 1;
        break;
    case A_FS_SPKLEN:
        if (dw_check_disabled(s, "IC_FS_SPKLEN")) {
            s->fs_spklen = MAX(value & 0xff, 1);
        }
        break;
    case A_SMBUS_INTR_MASK:
        if (dc->has_smbus_intr_mask) {
            /* No SMBus interrupts are implemented; Linux sets the mask. */
            s->smbus_intr_mask = value & 0x7ff;
            break;
        }
        /* fall through */
    default:
        if (addr != A_INTR_STAT && addr != A_RAW_INTR_STAT &&
            addr != A_STATUS && addr != A_TXFLR && addr != A_RXFLR &&
            addr != A_TX_ABRT_SOURCE && addr != A_ENABLE_STATUS &&
            !(addr >= A_CLR_INTR && addr <= A_CLR_GEN_CALL) &&
            addr != A_CLR_RESTART_DET && addr < A_COMP_PARAM_1) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset "
                          "0x%03" HWADDR_PRIx "\n",
                          DEVICE(s)->canonical_path, addr);
        }
        return;
    }
    dw_i2c_kick(s);
}

static const MemoryRegionOps designware_i2c_ops = {
    .read = dw_i2c_read,
    .write = dw_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void designware_i2c_enter_reset(Object *obj, ResetType type)
{
    DesignWareI2CState *s = DESIGNWARE_I2C(obj);
    DesignWareI2CClass *dc = DESIGNWARE_I2C_GET_CLASS(s);

    timer_del(s->timer);
    s->con = dc->con_reset;
    s->tar = dc->tar_reset;
    s->sar = dc->sar_reset;
    s->ss_scl_hcnt = dc->ss_scl_hcnt_reset;
    s->ss_scl_lcnt = dc->ss_scl_lcnt_reset;
    s->fs_scl_hcnt = dc->fs_scl_hcnt_reset;
    s->fs_scl_lcnt = dc->fs_scl_lcnt_reset;
    s->intr_mask = 0x8ff;
    s->raw_intr = 0;
    s->rx_tl = 0;
    s->tx_tl = 0;
    s->enable = 0;
    s->sda_hold = 1;
    s->tx_abrt_source = 0;
    s->slv_data_nack_only = 0;
    s->dma_cr = 0;
    s->dma_tdlr = 0;
    s->dma_rdlr = 0;
    s->sda_setup = 0x64;
    s->ack_general_call = 1;
    s->enable_status_slv = 0;
    s->fs_spklen = dc->fs_spklen_reset;
    s->smbus_intr_mask = 0x7ff;
    s->tx_head = s->tx_num = 0;
    s->rx_head = s->rx_num = 0;
    s->ic_en = false;

    s->mst_owned = false;
    s->mst_read = false;
    s->mst_read_done = false;
    s->mst_first_data = false;
    s->mst_shifting = false;
    s->mst_stepping = false;
    s->mst_polling = false;
    s->mst_user_abort = false;
    s->mst_rx_stall = false;
    s->mst_addr = 0;
    s->mst_cmd = 0;
    s->plan_len = s->plan_pos = 0;
    s->plan_popped = false;

    s->slv_active = false;
    s->slv_read = false;
    s->slv_gc = false;
    s->slv_gc_match = false;
    s->slv_first = false;
    s->slv_data = false;
    s->slv_hold = false;
    s->slv_10bit = 0;
    s->slv_bus = NULL;
}

static void designware_i2c_hold_reset(Object *obj, ResetType type)
{
    DesignWareI2CState *s = DESIGNWARE_I2C(obj);

    /* A transfer cut short by the reset leaves the bus. */
    if (s->mst_xfer_open) {
        i2c_end_transfer(s->bus);
        s->mst_xfer_open = false;
    }
    qemu_irq_lower(s->irq);
    qemu_irq_lower(s->dma_tx_req);
    qemu_irq_lower(s->dma_rx_req);
    s->scl_hold = false;
    qemu_irq_lower(s->scl_oe);
}

static int designware_i2c_post_load(void *opaque, int version_id)
{
    DesignWareI2CState *s = opaque;

    if (s->tx_num > FIFO_DEPTH || s->tx_head >= FIFO_DEPTH ||
        s->rx_num > FIFO_DEPTH || s->rx_head >= FIFO_DEPTH ||
        s->plan_len > DW_I2C_MAX_STEPS || s->plan_pos > s->plan_len) {
        return -EINVAL;
    }
    s->slv_bus = NULL;
    return 0;
}

static const VMStateDescription vmstate_designware_i2c = {
    .name = TYPE_DESIGNWARE_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = designware_i2c_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(con, DesignWareI2CState),
        VMSTATE_UINT32(tar, DesignWareI2CState),
        VMSTATE_UINT32(sar, DesignWareI2CState),
        VMSTATE_UINT32(ss_scl_hcnt, DesignWareI2CState),
        VMSTATE_UINT32(ss_scl_lcnt, DesignWareI2CState),
        VMSTATE_UINT32(fs_scl_hcnt, DesignWareI2CState),
        VMSTATE_UINT32(fs_scl_lcnt, DesignWareI2CState),
        VMSTATE_UINT32(intr_mask, DesignWareI2CState),
        VMSTATE_UINT32(raw_intr, DesignWareI2CState),
        VMSTATE_UINT32(rx_tl, DesignWareI2CState),
        VMSTATE_UINT32(tx_tl, DesignWareI2CState),
        VMSTATE_UINT32(enable, DesignWareI2CState),
        VMSTATE_UINT32(sda_hold, DesignWareI2CState),
        VMSTATE_UINT32(tx_abrt_source, DesignWareI2CState),
        VMSTATE_UINT32(slv_data_nack_only, DesignWareI2CState),
        VMSTATE_UINT32(dma_cr, DesignWareI2CState),
        VMSTATE_UINT32(dma_tdlr, DesignWareI2CState),
        VMSTATE_UINT32(dma_rdlr, DesignWareI2CState),
        VMSTATE_UINT32(sda_setup, DesignWareI2CState),
        VMSTATE_UINT32(ack_general_call, DesignWareI2CState),
        VMSTATE_UINT32(enable_status_slv, DesignWareI2CState),
        VMSTATE_UINT32(fs_spklen, DesignWareI2CState),
        VMSTATE_UINT32(smbus_intr_mask, DesignWareI2CState),
        VMSTATE_UINT32_ARRAY(tx_fifo, DesignWareI2CState,
                             DESIGNWARE_I2C_TX_FIFO_SIZE),
        VMSTATE_UINT8(tx_head, DesignWareI2CState),
        VMSTATE_UINT8(tx_num, DesignWareI2CState),
        VMSTATE_UINT32_ARRAY(rx_fifo, DesignWareI2CState,
                             DESIGNWARE_I2C_RX_FIFO_SIZE),
        VMSTATE_UINT8(rx_head, DesignWareI2CState),
        VMSTATE_UINT8(rx_num, DesignWareI2CState),
        VMSTATE_BOOL(scl_in, DesignWareI2CState),
        VMSTATE_BOOL(sda_in, DesignWareI2CState),
        VMSTATE_BOOL(scl_hold, DesignWareI2CState),
        VMSTATE_BOOL(ic_en, DesignWareI2CState),
        VMSTATE_TIMER_PTR(timer, DesignWareI2CState),
        VMSTATE_BOOL(mst_owned, DesignWareI2CState),
        VMSTATE_BOOL(mst_xfer_open, DesignWareI2CState),
        VMSTATE_BOOL(mst_read, DesignWareI2CState),
        VMSTATE_BOOL(mst_read_done, DesignWareI2CState),
        VMSTATE_BOOL(mst_first_data, DesignWareI2CState),
        VMSTATE_BOOL(mst_shifting, DesignWareI2CState),
        VMSTATE_BOOL(mst_stepping, DesignWareI2CState),
        VMSTATE_BOOL(mst_polling, DesignWareI2CState),
        VMSTATE_BOOL(mst_user_abort, DesignWareI2CState),
        VMSTATE_BOOL(mst_rx_stall, DesignWareI2CState),
        VMSTATE_UINT8(mst_addr, DesignWareI2CState),
        VMSTATE_UINT32(mst_cmd, DesignWareI2CState),
        VMSTATE_UINT8_ARRAY(plan, DesignWareI2CState, DW_I2C_MAX_STEPS),
        VMSTATE_UINT8(plan_len, DesignWareI2CState),
        VMSTATE_UINT8(plan_pos, DesignWareI2CState),
        VMSTATE_BOOL(plan_popped, DesignWareI2CState),
        VMSTATE_BOOL(slv_active, DesignWareI2CState),
        VMSTATE_BOOL(slv_read, DesignWareI2CState),
        VMSTATE_BOOL(slv_gc, DesignWareI2CState),
        VMSTATE_BOOL(slv_gc_match, DesignWareI2CState),
        VMSTATE_BOOL(slv_first, DesignWareI2CState),
        VMSTATE_BOOL(slv_data, DesignWareI2CState),
        VMSTATE_BOOL(slv_hold, DesignWareI2CState),
        VMSTATE_UINT8(slv_10bit, DesignWareI2CState),
        VMSTATE_END_OF_LIST(),
    },
};

static void designware_i2c_instance_init(Object *obj)
{
    DesignWareI2CState *s = DESIGNWARE_I2C(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dw_i2c_timer, s);
    /* Unconnected wires are pulled up. */
    s->scl_in = true;
    s->sda_in = true;

    object_initialize_child(obj, "target", &s->target,
                            TYPE_DESIGNWARE_I2C_TARGET);
    s->target.controller = s;

    memory_region_init_io(&s->iomem, obj, &designware_i2c_ops, s,
                          TYPE_DESIGNWARE_I2C, 4 * KiB);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->dma_tx_req, DESIGNWARE_I2C_DMA_TX_REQ,
                             1);
    qdev_init_gpio_out_named(dev, &s->dma_rx_req, DESIGNWARE_I2C_DMA_RX_REQ,
                             1);
    qdev_init_gpio_out_named(dev, &s->scl_oe, DESIGNWARE_I2C_SCL_OE, 1);
    qdev_init_gpio_in_named(dev, dw_i2c_scl_in, DESIGNWARE_I2C_SCL_IN, 1);
    qdev_init_gpio_in_named(dev, dw_i2c_sda_in, DESIGNWARE_I2C_SDA_IN, 1);
    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
}

static void designware_i2c_realize(DeviceState *dev, Error **errp)
{
    DesignWareI2CState *s = DESIGNWARE_I2C(dev);

    s->bus = i2c_init_bus(dev, s->bus_name ? s->bus_name : "i2c-bus");
    qdev_realize(DEVICE(&s->target), BUS(s->bus), errp);
}

static void designware_i2c_finalize(Object *obj)
{
    DesignWareI2CState *s = DESIGNWARE_I2C(obj);

    timer_free(s->timer);
}

static const Property designware_i2c_properties[] = {
    DEFINE_PROP_STRING("bus-name", DesignWareI2CState, bus_name),
};

static void designware_i2c_class_init(ObjectClass *klass, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    DesignWareI2CClass *dwc = DESIGNWARE_I2C_CLASS(klass);

    dc->desc = "Designware I2C";
    dc->vmsd = &vmstate_designware_i2c;
    dc->realize = designware_i2c_realize;
    device_class_set_props(dc, designware_i2c_properties);
    rc->phases.enter = designware_i2c_enter_reset;
    rc->phases.hold = designware_i2c_hold_reset;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);

    dwc->con_reset = 0x7d;
    dwc->tar_reset = 0x1055;
    dwc->tar_mask = 0x1fff;
    dwc->sar_reset = 0x55;
    dwc->ss_scl_hcnt_reset = 0x190;
    dwc->ss_scl_lcnt_reset = 0x1d6;
    dwc->fs_scl_hcnt_reset = 0x3c;
    dwc->fs_scl_lcnt_reset = 0x82;
    dwc->fs_spklen_reset = 2;
    dwc->max_speed = 3;
    /* 32-bit APB, high speed, combined interrupt, no DMA, 16 deep. */
    dwc->comp_param_1 = (2 << 0) | (3 << 2) | BIT(5) | BIT(7) |
                        ((DESIGNWARE_I2C_RX_FIFO_SIZE - 1) << 8) |
                        ((DESIGNWARE_I2C_TX_FIFO_SIZE - 1) << 16);
    dwc->comp_version = 0x3132302a;
    dwc->has_smbus_intr_mask = true;
}

/* Target slaves. */

static DesignWareI2CState *dw_target_ctrl(I2CSlave *slave)
{
    return DESIGNWARE_I2C_TARGET(slave)->controller;
}

static I2CBus *dw_target_bus(I2CSlave *slave)
{
    return I2C_BUS(qdev_get_parent_bus(DEVICE(slave)));
}

static bool dw_target_match_and_add(I2CSlave *candidate, uint8_t address,
                                    bool broadcast, I2CNodeList *current_devs)
{
    I2CNode *node;

    if (!dw_slv_match(dw_target_ctrl(candidate), address, broadcast)) {
        return false;
    }
    node = g_new(struct I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

static int dw_target_event(I2CSlave *slave, enum i2c_event event)
{
    DesignWareI2CState *s = dw_target_ctrl(slave);

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_SEND_ASYNC:
        return dw_slv_start(s, dw_target_bus(slave), false);
    case I2C_START_RECV:
        return dw_slv_start(s, dw_target_bus(slave), true);
    case I2C_FINISH:
        dw_slv_stop(s);
        return 0;
    case I2C_NACK:
        dw_slv_nack(s);
        return 0;
    }
    return 0;
}

static int dw_target_send(I2CSlave *slave, uint8_t data)
{
    return dw_slv_send(dw_target_ctrl(slave), data);
}

static void dw_target_send_async(I2CSlave *slave, uint8_t data)
{
    dw_slv_send(dw_target_ctrl(slave), data);
    i2c_ack(dw_target_bus(slave));
}

static uint8_t dw_target_recv(I2CSlave *slave)
{
    return dw_slv_recv(dw_target_ctrl(slave));
}

static void dw_target_realize(DeviceState *dev, Error **errp)
{
    if (!DESIGNWARE_I2C_TARGET(dev)->controller) {
        error_setg(errp, "the controller link must be set");
    }
}

/*
 * "controller": the controller whose target logic this is. The link is
 * weak: the controller's own target is its child, and a strong link back
 * would hold a reference the controller never gives (it sets the field
 * directly) and make a cycle of the two.
 */
static void dw_target_instance_init(Object *obj)
{
    DesignWareI2CTarget *t = DESIGNWARE_I2C_TARGET(obj);

    object_property_add_link(obj, "controller", TYPE_DESIGNWARE_I2C,
                             (Object **)&t->controller,
                             qdev_prop_allow_set_link_before_realize, 0);
}

static const VMStateDescription vmstate_dw_target = {
    .name = TYPE_DESIGNWARE_I2C_TARGET,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, DesignWareI2CTarget),
        VMSTATE_END_OF_LIST(),
    },
};

/* [spec:nuos:req:emu.i2c] */
static void dw_target_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->desc = "DesignWare I2C target logic on an I2C bus";
    dc->realize = dw_target_realize;
    dc->vmsd = &vmstate_dw_target;
    sc->match_and_add = dw_target_match_and_add;
    sc->event = dw_target_event;
    sc->send = dw_target_send;
    sc->send_async = dw_target_send_async;
    sc->recv = dw_target_recv;
}

static const TypeInfo designware_i2c_types[] = {
    {
        .name = TYPE_DESIGNWARE_I2C,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(DesignWareI2CState),
        .class_size = sizeof(DesignWareI2CClass),
        .class_init = designware_i2c_class_init,
        .instance_init = designware_i2c_instance_init,
        .instance_finalize = designware_i2c_finalize,
    },
    {
        .name = TYPE_DESIGNWARE_I2C_TARGET,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(DesignWareI2CTarget),
        .instance_init = dw_target_instance_init,
        .class_init = dw_target_class_init,
    },
};
DEFINE_TYPES(designware_i2c_types);
