/*
 * ESP32-P4 I2C controllers: I2C0, I2C1 and the LP I2C
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 47 (I2C Controller); ESP-IDF's i2c_reg.h, i2c_struct.h, lp_i2c_reg.h and
 * lp_i2c_struct.h for v3, and hal/esp32p4 i2c_ll.h, esp_driver_i2c and
 * ulp lp_core_i2c.c, which show how software drives the controllers.
 *
 * The controller is modelled at the level of its SCL and SDA lines (see
 * the header). Times are counted in cycles of I2C_SCLK:
 *
 *   - A master bit: SCL is pulled low; SDA takes the bit SDA_HOLD_TIME + 1
 *     cycles later; SCL is released SCL_LOW_PERIOD + 1 cycles after it
 *     fell; the controller waits SCL_WAIT_HIGH_PERIOD + 1 cycles, plus
 *     SCL_FILTER_THRES with the SCL filter on, for SCL to be high, and
 *     longer while something holds it low; SDA is sampled
 *     SDA_SAMPLE_TIME + 1 cycles after SCL is seen high; SCL falls
 *     SCL_HIGH_PERIOD + 1 cycles after the wait. Unstretched, a bit takes
 *     LOW + WAIT_HIGH + HIGH + 3 (+ FILTER_THRES) cycles, the TRM's
 *     formula for f_SCL.
 *   - START: with both lines high, SDA falls SCL_RSTART_SETUP_TIME + 1
 *     cycles after SCL is seen high, SCL SCL_START_HOLD_TIME + 1 after
 *     that. STOP: SDA is pulled low in SCL's low period, and released
 *     SCL_STOP_SETUP_TIME + 1 cycles after SCL is seen high; the bus is
 *     free SCL_STOP_HOLD_TIME + 1 cycles after that.
 *   - The SCL and SDA filters pass a level once it has held for
 *     FILTER_THRES cycles, so shorter pulses are lost and every edge
 *     arrives that much later.
 *   - The SCL timeout (I2C_TO) fires after 2^TIME_OUT_VALUE cycles with
 *     SCL unchanged while the controller is in a transfer or waiting to
 *     start one, as the register description and ESP-IDF's
 *     i2c_ll_calculate_timeout_us_to_reg_val() have it (TRM 47.4.8 gives
 *     2^(VALUE+1) - 1, its interrupt list 2^VALUE); a master's own END
 *     and a slave's own stretch do not count. The SCL_FSM and
 *     SCL_MAIN_FSM timeouts fire after 2^(N+1) - 1 cycles of a master
 *     waiting on the bus.
 *
 * Choices where the TRM is silent or contradicts itself:
 *
 *   - The command opcodes are 1 WRITE, 2 STOP, 3 READ, 4 END and 6
 *     RSTART (TRM 47.4.9, ESP-IDF); others are skipped as invalid.
 *   - A master that receives a byte with its RX FIFO already full drops
 *     it (RXFIFO_OVF) and answers it with I2C_RX_FULL_ACK_LEVEL; a slave
 *     does the same, and with SCL stretching enabled holds SCL low
 *     (stretch cause 2) after the byte that fills the FIFO, answering it
 *     with RX_FULL_ACK_LEVEL if the FIFO is still full once released.
 *   - After a NACK the master raises NACK_INT, puts a STOP on the bus
 *     (TRANS_COMPLETE_INT, as for any STOP it detects) and abandons the
 *     command list. A slave raises TRANS_COMPLETE_INT for the STOP that
 *     ends a transfer it was addressed in.
 *   - A slave in manual start mode (SLV_TX_AUTO_START_EN clear) that is
 *     addressed for a read without TRANS_START since its last read
 *     acknowledges the address and sends nothing ("undefined" in TRM
 *     47.4.14).
 *   - The watermark interrupts follow their conditions: clearing one
 *     while its condition holds sets it again.
 *   - LP_I2C_CLK_CONF, which the TRM omits, divides the LP I2C's
 *     function clock by SCLK_DIV_NUM + 1 + SCLK_DIV_A / SCLK_DIV_B and
 *     stops it while SCLK_ACTIVE is clear.
 *
 * Not modelled: I2C_SAMPLE_SCL_LEVEL, sampling SDA in SCL's low phase
 * (logged); I2C_CLK_EN only gates the register clock and has no effect
 * software can see.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/i2c/esp32p4_i2c.h"
#include "migration/vmstate.h"

/* Register word indices */
enum {
    R_SCL_LOW_PERIOD = 0x00 / 4,
    R_CTR = 0x04 / 4,
    R_SR = 0x08 / 4,
    R_TO = 0x0c / 4,
    R_SLAVE_ADDR = 0x10 / 4,
    R_FIFO_ST = 0x14 / 4,
    R_FIFO_CONF = 0x18 / 4,
    R_DATA = 0x1c / 4,
    R_INT_RAW = 0x20 / 4,
    R_INT_CLR = 0x24 / 4,
    R_INT_ENA = 0x28 / 4,
    R_INT_STATUS = 0x2c / 4,
    R_SDA_HOLD = 0x30 / 4,
    R_SDA_SAMPLE = 0x34 / 4,
    R_SCL_HIGH_PERIOD = 0x38 / 4,
    R_SCL_START_HOLD = 0x40 / 4,
    R_SCL_RSTART_SETUP = 0x44 / 4,
    R_SCL_STOP_HOLD = 0x48 / 4,
    R_SCL_STOP_SETUP = 0x4c / 4,
    R_FILTER_CFG = 0x50 / 4,
    R_CLK_CONF = 0x54 / 4,
    R_COMD0 = 0x58 / 4,
    R_SCL_ST_TIME_OUT = 0x78 / 4,
    R_SCL_MAIN_ST_TIME_OUT = 0x7c / 4,
    R_SCL_SP_CONF = 0x80 / 4,
    R_SCL_STRETCH_CONF = 0x84 / 4,
    R_DATE = 0xf8 / 4,
};

#define A_DATA          0x1c
#define A_TXRAM         0x100
#define A_RXRAM         0x180

/* I2C_CTR */
#define CTR_SDA_FORCE_OUT       BIT(0)
#define CTR_SCL_FORCE_OUT       BIT(1)
#define CTR_SAMPLE_SCL_LEVEL    BIT(2)
#define CTR_RX_FULL_ACK_LEVEL   BIT(3)
#define CTR_MS_MODE             BIT(4)
#define CTR_TRANS_START         BIT(5)
#define CTR_TX_LSB_FIRST        BIT(6)
#define CTR_RX_LSB_FIRST        BIT(7)
#define CTR_CLK_EN              BIT(8)
#define CTR_ARBITRATION_EN      BIT(9)
#define CTR_FSM_RST             BIT(10)
#define CTR_CONF_UPGATE         BIT(11)
#define CTR_SLV_TX_AUTO_START   BIT(12)
#define CTR_ADDR_10BIT_RW_CHECK BIT(13)
#define CTR_ADDR_BROADCASTING   BIT(14)

/* I2C_TO */
#define TO_VALUE(v)             extract32((v), 0, 5)
#define TO_EN                   BIT(5)

/* I2C_SLAVE_ADDR */
#define SLAVE_ADDR_10BIT_EN     BIT(31)

/* I2C_FIFO_CONF */
#define FIFO_NONFIFO_EN         BIT(10)
#define FIFO_ADDR_CFG_EN        BIT(11)
#define FIFO_RX_RST             BIT(12)
#define FIFO_TX_RST             BIT(13)
#define FIFO_PRT_EN             BIT(14)

/* I2C_FILTER_CFG */
#define FILTER_SCL_THRES(v)     extract32((v), 0, 4)
#define FILTER_SDA_THRES(v)     extract32((v), 4, 4)
#define FILTER_SCL_EN           BIT(8)
#define FILTER_SDA_EN           BIT(9)

/* LP_I2C_CLK_CONF */
#define CLK_DIV_NUM(v)          extract32((v), 0, 8)
#define CLK_DIV_A(v)            extract32((v), 8, 6)
#define CLK_DIV_B(v)            extract32((v), 14, 6)
#define CLK_SCLK_SEL            BIT(20)
#define CLK_SCLK_ACTIVE         BIT(21)

/* I2C_COMDn */
#define CMD_BYTE_NUM(v)         extract32((v), 0, 8)
#define CMD_ACK_CHECK_EN        BIT(8)
#define CMD_ACK_EXP             BIT(9)
#define CMD_ACK_VALUE           BIT(10)
#define CMD_OPCODE(v)           extract32((v), 11, 3)
#define CMD_DONE                BIT(31)

enum {
    OP_WRITE = 1,
    OP_STOP = 2,
    OP_READ = 3,
    OP_END = 4,
    OP_RSTART = 6,
};

/* I2C_SCL_SP_CONF */
#define SP_RST_SLV_EN           BIT(0)
#define SP_RST_SLV_NUM(v)       extract32((v), 1, 5)
#define SP_SCL_PD_EN            BIT(6)
#define SP_SDA_PD_EN            BIT(7)

/* I2C_SCL_STRETCH_CONF */
#define STRETCH_PROTECT(v)      extract32((v), 0, 10)
#define STRETCH_EN              BIT(10)
#define STRETCH_CLR             BIT(11)
#define STRETCH_ACK_CTL_EN      BIT(12)
#define STRETCH_ACK_LVL         BIT(13)

/* Interrupts */
#define INT_RXFIFO_WM           BIT(0)
#define INT_TXFIFO_WM           BIT(1)
#define INT_RXFIFO_OVF          BIT(2)
#define INT_END_DETECT          BIT(3)
#define INT_BYTE_TRANS_DONE     BIT(4)
#define INT_ARBITRATION_LOST    BIT(5)
#define INT_MST_TXFIFO_UDF      BIT(6)
#define INT_TRANS_COMPLETE      BIT(7)
#define INT_TIME_OUT            BIT(8)
#define INT_TRANS_START         BIT(9)
#define INT_NACK                BIT(10)
#define INT_TXFIFO_OVF          BIT(11)
#define INT_RXFIFO_UDF          BIT(12)
#define INT_SCL_ST_TO           BIT(13)
#define INT_SCL_MAIN_ST_TO      BIT(14)
#define INT_DET_START           BIT(15)
#define INT_SLAVE_STRETCH       BIT(16)
#define INT_GENERAL_CALL        BIT(17)
#define INT_SLAVE_ADDR_UNMATCH  BIT(18)

#define HP_INT_MASK             0x7ffff
#define LP_INT_MASK             0xffff

/* SR state fields */
enum {
    MAIN_IDLE,
    MAIN_ADDR_SHIFT,
    MAIN_ACK_ADDR,
    MAIN_RX_DATA,
    MAIN_TX_DATA,
    MAIN_SEND_ACK,
    MAIN_WAIT_ACK,
};

enum {
    SCLST_IDLE,
    SCLST_START,
    SCLST_NEGEDGE,
    SCLST_LOW,
    SCLST_POSEDGE,
    SCLST_HIGH,
    SCLST_STOP,
};

/* Stretch causes, I2C_STRETCH_CAUSE */
enum {
    STRETCH_ADDR_MATCH,
    STRETCH_TX_EMPTY,
    STRETCH_RX_FULL,
    STRETCH_SENDING_ACK,
};

#define LINE_SCL 0
#define LINE_SDA 1

/* Master FSM steps, run by the FSM timer */
enum {
    ST_NONE,
    ST_START_SDA,
    ST_START_SCL,
    ST_RS_SDA,
    ST_RS_SCL,
    ST_RS_HIGH,
    ST_BIT_SDA,
    ST_BIT_SCL,
    ST_BIT_HIGH,
    ST_BIT_SAMPLE,
    ST_BIT_FALL,
    ST_STOP_SDA,
    ST_STOP_SCL,
    ST_STOP_HIGH,
    ST_STOP_REL,
    ST_STOP_IDLE,
    ST_END_SDA,
    ST_CLR_LOW,
    ST_CLR_HIGH,
    ST_CLR_NEXT,
    /* Waits on the lines, which an input edge ends. */
    ST_WAIT_IDLE,
    ST_WAIT_RS_HIGH,
    ST_WAIT_BIT_HIGH,
    ST_WAIT_STOP_HIGH,
};

/* Slave states */
enum {
    SLV_IDLE,
    SLV_ADDR,
    SLV_ADDR2,
    SLV_RX,
    SLV_TX,
    SLV_IGNORE,
};

/* What the slave timer does */
enum {
    SLV_ACT_NONE,
    SLV_ACT_SDA,
    SLV_ACT_RELEASE_SCL,
};

/* Which timeouts are armed */
#define TO_KIND_SCL     BIT(0)
#define TO_KIND_FSM     BIT(1)

/* A register field the controller works from: the synchronised copy. */
#define ACT(s, r)       ((s)->act[(r)])

static const struct {
    uint32_t reset_hp;
    uint32_t reset_lp;
    uint32_t rw_hp;
    uint32_t rw_lp;
    /* Fields that I2C_CONF_UPGATE copies to the I2C_SCLK domain. */
    uint32_t sync_hp;
    uint32_t sync_lp;
} esp32p4_i2c_regs[ESP32P4_I2C_NREGS] = {
    [R_SCL_LOW_PERIOD] = { 0, 0, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_CTR] = { 0x208, 0x208, 0x73df, 0x3cc, 0x72df, 0x2cc },
    [R_TO] = { 0x10, 0x10, 0x3f, 0x3f, 0x3f, 0x3f },
    [R_SLAVE_ADDR] = { 0, 0, 0x80007fff, 0, 0x80007fff, 0 },
    [R_FIFO_CONF] = { 0x408b, 0x4046, 0x7fff, 0x75ef, FIFO_ADDR_CFG_EN, 0 },
    [R_INT_ENA] = { 0, 0, HP_INT_MASK, LP_INT_MASK, 0, 0 },
    [R_SDA_HOLD] = { 0, 0, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_SDA_SAMPLE] = { 0, 0, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_SCL_HIGH_PERIOD] = { 0, 0, 0xffff, 0xffff, 0xffff, 0xffff },
    [R_SCL_START_HOLD] = { 8, 8, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_SCL_RSTART_SETUP] = { 8, 8, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_SCL_STOP_HOLD] = { 8, 8, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_SCL_STOP_SETUP] = { 8, 8, 0x1ff, 0x1ff, 0x1ff, 0x1ff },
    [R_FILTER_CFG] = { 0x300, 0x300, 0x3ff, 0x3ff, 0x3ff, 0x3ff },
    [R_CLK_CONF] = { 0, 0x200000, 0, 0x3fffff, 0, 0 },
    [R_COMD0 + 0] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 1] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 2] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 3] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 4] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 5] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 6] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_COMD0 + 7] = { 0, 0, 0x80003fff, 0x80003fff, 0, 0 },
    [R_SCL_ST_TIME_OUT] = { 0x10, 0x10, 0x1f, 0x1f, 0x1f, 0x1f },
    [R_SCL_MAIN_ST_TIME_OUT] = { 0x10, 0x10, 0x1f, 0x1f, 0x1f, 0x1f },
    [R_SCL_SP_CONF] = { 0, 0, 0xff, 0xff, 0xff, 0xff },
    [R_SCL_STRETCH_CONF] = { 0, 0, 0x37ff, 0, 0x37ff, 0 },
    [R_DATE] = { 0x02201172, 0x02404120, 0xffffffff, 0xffffffff, 0, 0 },
};

static void esp32p4_i2c_fsm_step(ESP32P4I2CState *s, unsigned step);
static void esp32p4_i2c_mst_next_cmd(ESP32P4I2CState *s);
static void esp32p4_i2c_to_update(ESP32P4I2CState *s);
static void esp32p4_i2c_mst_idle(ESP32P4I2CState *s);

static bool esp32p4_i2c_is_lp(ESP32P4I2CState *s)
{
    return ESP32P4_I2C_GET_CLASS(s)->lp;
}

static const char *esp32p4_i2c_name(ESP32P4I2CState *s)
{
    return esp32p4_i2c_is_lp(s) ? TYPE_ESP32P4_LP_I2C : TYPE_ESP32P4_I2C;
}

static unsigned esp32p4_i2c_depth(ESP32P4I2CState *s)
{
    return esp32p4_i2c_is_lp(s) ? 16 : 32;
}

static uint32_t esp32p4_i2c_int_mask(ESP32P4I2CState *s)
{
    return esp32p4_i2c_is_lp(s) ? LP_INT_MASK : HP_INT_MASK;
}

/* The CTR bits the controller works from: synchronised or direct. */
static uint32_t esp32p4_i2c_ctr(ESP32P4I2CState *s)
{
    uint32_t sync = esp32p4_i2c_is_lp(s) ? esp32p4_i2c_regs[R_CTR].sync_lp
                                         : esp32p4_i2c_regs[R_CTR].sync_hp;
    uint32_t synced = s->act[R_CTR] & sync;

    return synced | (s->regs[R_CTR] & ~sync);
}

static bool esp32p4_i2c_master(ESP32P4I2CState *s)
{
    return esp32p4_i2c_is_lp(s) || (esp32p4_i2c_ctr(s) & CTR_MS_MODE);
}

static bool esp32p4_i2c_fifo_mode(ESP32P4I2CState *s)
{
    return !(s->regs[R_FIFO_CONF] & FIFO_NONFIFO_EN);
}

static bool esp32p4_i2c_prt(ESP32P4I2CState *s)
{
    return s->regs[R_FIFO_CONF] & FIFO_PRT_EN;
}

/* Time */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The period of I2C_SCLK, exactly (esp32p4_clk_period.h); stopped while it
 * is. The LP controller divides its function clock by LP_I2C_CLK_CONF.
 */
static ESP32P4ClkPeriod esp32p4_i2c_period(ESP32P4I2CState *s)
{
    ESP32P4ClkPeriod period = esp32p4_clk_exact(s->sclk);
    uint32_t conf, num, a, b;

    if (!esp32p4_clk_period_running(period) || !esp32p4_i2c_is_lp(s)) {
        return period;
    }
    conf = s->regs[R_CLK_CONF];
    if (!(conf & CLK_SCLK_ACTIVE)) {
        return esp32p4_clk_period_stopped();
    }
    num = CLK_DIV_NUM(conf) + 1;
    a = CLK_DIV_A(conf);
    b = CLK_DIV_B(conf);
    if (!b || !a) {
        return esp32p4_clk_period_scale(period, num, 1);
    }
    return esp32p4_clk_period_scale(period, (uint64_t)num * b + a, b);
}

static bool esp32p4_i2c_runs(ESP32P4I2CState *s)
{
    return esp32p4_clk_period_running(esp32p4_i2c_period(s));
}

/*
 * The edges I2C_SCLK has at @now: its own, or the LP controller's divided
 * clock's, which counts them afresh after a change of the function
 * clock's edges or of the division, and keeps its edges while neither
 * changes.
 */
static ESP32P4ClkEdges esp32p4_i2c_edges(ESP32P4I2CState *s, int64_t now)
{
    ESP32P4ClkPeriod period = esp32p4_i2c_period(s);
    ESP32P4ClkEdges sclk = esp32p4_clk_edges_or_at(s->sclk, now);

    if (!esp32p4_i2c_is_lp(s) || !esp32p4_clk_period_running(period)) {
        return esp32p4_clk_period_eq(period, sclk.period) ?
               sclk : esp32p4_clk_edges_stopped();
    }
    if (esp32p4_clk_period_eq(period, s->tp_edges.period) &&
        esp32p4_clk_edges_eq(&sclk, &s->tp_sclk)) {
        return s->tp_edges;
    }
    return esp32p4_clk_edges_restart(period, &sclk, now);
}

/*
 * The first ns by which @n cycles of I2C_SCLK, its edges, have come after
 * @from; the clock must be running.
 */
static int64_t esp32p4_i2c_after(ESP32P4I2CState *s, int64_t from,
                                 uint64_t n)
{
    return MIN(esp32p4_clk_edges_after(&s->tp_edges, from, n),
               INT64_MAX / 2);
}

/*
 * The master's instants: @tp's cycles of I2C_SCLK, its edges as the
 * controller took them as the clock last changed, after its origin,
 * exactly; at the first nanosecond they have come.
 */
static ESP32P4I2CTp esp32p4_i2c_tp(int64_t t)
{
    return (ESP32P4I2CTp) { .origin = t };
}

static ESP32P4I2CTp esp32p4_i2c_tp_after(ESP32P4I2CTp tp, uint64_t n)
{
    tp.cyc += n;
    return tp;
}

static int64_t esp32p4_i2c_tp_ns_at(const ESP32P4ClkEdges *e,
                                    ESP32P4I2CTp tp)
{
    if (!tp.cyc) {
        return tp.origin;
    }
    return MIN(esp32p4_clk_edges_after(e, tp.origin, tp.cyc), INT64_MAX / 2);
}

static int64_t esp32p4_i2c_tp_ns(ESP32P4I2CState *s, ESP32P4I2CTp tp)
{
    return esp32p4_i2c_tp_ns_at(&s->tp_edges, tp);
}

static int64_t esp32p4_i2c_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* Interrupts */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The watermark interrupts follow the FIFO levels while FIFO_PRT_EN is
 * set; the rest are latched until cleared.
 */
static void esp32p4_i2c_update_irq(ESP32P4I2CState *s)
{
    uint32_t conf = s->regs[R_FIFO_CONF];
    unsigned rx_thr, tx_thr;
    bool level;

    if (esp32p4_i2c_prt(s)) {
        if (esp32p4_i2c_is_lp(s)) {
            rx_thr = extract32(conf, 0, 4);
            tx_thr = extract32(conf, 5, 4);
        } else {
            rx_thr = extract32(conf, 0, 5);
            tx_thr = extract32(conf, 5, 5);
        }
        if (s->rx_cnt > rx_thr) {
            s->int_raw |= INT_RXFIFO_WM;
        }
        if (s->tx_cnt < tx_thr) {
            s->int_raw |= INT_TXFIFO_WM;
        }
    }
    s->int_raw &= esp32p4_i2c_int_mask(s);
    level = (s->int_raw & s->regs[R_INT_ENA]) != 0;
    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->irq, level);
    }
}

/* FIFOs */

static void esp32p4_i2c_tx_reset(ESP32P4I2CState *s)
{
    s->tx_rd = s->tx_wr = s->tx_cnt = 0;
}

static void esp32p4_i2c_rx_reset(ESP32P4I2CState *s)
{
    s->rx_rd = s->rx_wr = s->rx_cnt = 0;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The next byte to send: from the TX FIFO, or in non-FIFO mode from TX
 * RAM at the read pointer. A master that finds the FIFO empty raises
 * MST_TXFIFO_UDF and sends what the RAM holds there.
 */
static uint8_t esp32p4_i2c_tx_pop(ESP32P4I2CState *s, bool *empty)
{
    unsigned depth = esp32p4_i2c_depth(s);
    uint8_t b = s->txram[s->tx_rd];

    *empty = false;
    if (esp32p4_i2c_fifo_mode(s)) {
        if (!s->tx_cnt) {
            *empty = true;
            return b;
        }
        s->tx_cnt--;
    }
    s->tx_rd = (s->tx_rd + 1) % depth;
    esp32p4_i2c_update_irq(s);
    return b;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Store a received byte in RX RAM at the write pointer. Returns false if
 * the FIFO was full: the byte is lost and RXFIFO_OVF raised.
 */
static bool esp32p4_i2c_rx_push(ESP32P4I2CState *s, uint8_t b)
{
    unsigned depth = esp32p4_i2c_depth(s);

    if (esp32p4_i2c_fifo_mode(s) || esp32p4_i2c_prt(s)) {
        if (s->rx_cnt >= depth) {
            if (esp32p4_i2c_prt(s)) {
                s->int_raw |= INT_RXFIFO_OVF;
            }
            esp32p4_i2c_update_irq(s);
            return false;
        }
        s->rx_cnt++;
    }
    s->rxram[s->rx_wr] = b;
    s->rx_wr = (s->rx_wr + 1) % depth;
    esp32p4_i2c_update_irq(s);
    return true;
}

static bool esp32p4_i2c_rx_full(ESP32P4I2CState *s)
{
    return s->rx_cnt >= esp32p4_i2c_depth(s);
}

/* Lines */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Drive SCL and SDA (1 releases a line, 0 pulls it low) at @when: open
 * drain through the output enable, or with FORCE_OUT set through the
 * level alone, which SCL_PD_EN and SDA_PD_EN then force low.
 */
static void esp32p4_i2c_drive(ESP32P4I2CState *s, bool scl, bool sda,
                              int64_t when)
{
    uint32_t ctr = esp32p4_i2c_ctr(s);
    uint32_t sp = ACT(s, R_SCL_SP_CONF);
    bool lp = esp32p4_i2c_is_lp(s);
    bool scl_force = !lp && (ctr & CTR_SCL_FORCE_OUT);
    bool sda_force = !lp && (ctr & CTR_SDA_FORCE_OUT);
    bool scl_lvl = scl && !(scl_force && (sp & SP_SCL_PD_EN));
    bool sda_lvl = sda && !(sda_force && (sp & SP_SDA_PD_EN));

    ESP32P4I2CTp tp = when == s->step_ns ? s->cur_tp : esp32p4_i2c_tp(when);

    if (scl != s->scl_drv) {
        s->drv_ns[LINE_SCL] = when;
        s->drv_tp[LINE_SCL] = tp;
    }
    if (sda != s->sda_drv) {
        s->drv_ns[LINE_SDA] = when;
        s->drv_tp[LINE_SDA] = tp;
    }
    s->scl_drv = scl;
    s->sda_drv = sda;
    esp32_line_set_at(s->scl_out, scl_lvl, when);
    esp32_line_set_at(s->scl_oe, scl_force || !scl_lvl, when);
    esp32_line_set_at(s->sda_out, sda_lvl, when);
    esp32_line_set_at(s->sda_oe, sda_force || !sda_lvl, when);
}

static void esp32p4_i2c_scl(ESP32P4I2CState *s, bool level, int64_t when)
{
    esp32p4_i2c_drive(s, level, s->sda_drv, when);
}

static void esp32p4_i2c_sda(ESP32P4I2CState *s, bool level, int64_t when)
{
    esp32p4_i2c_drive(s, s->scl_drv, level, when);
}

static bool esp32p4_i2c_filt(ESP32P4I2CState *s, int n)
{
    return s->line[n].filt;
}

/* The filter threshold of line @n in cycles; 0 with the filter off. */
static unsigned esp32p4_i2c_thres(ESP32P4I2CState *s, int n)
{
    uint32_t f = ACT(s, R_FILTER_CFG);

    if (n == LINE_SCL) {
        return (f & FILTER_SCL_EN) ? FILTER_SCL_THRES(f) : 0;
    }
    return (f & FILTER_SDA_EN) ? FILTER_SDA_THRES(f) : 0;
}

/* QEMU bus devices */

static void esp32p4_i2c_dev_end(ESP32P4I2CState *s)
{
    if (s->dev_active) {
        i2c_end_transfer(s->bus);
        s->dev_active = false;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Address the bus's devices with the byte after a START. A repeated
 * START to the address already in the transfer continues it, as QEMU's
 * I2C devices expect of a write-then-read; any other address ends it
 * and selects afresh: a general call (address 0) is no device's address.
 * A NACK ends the core's transfer at once, as the STOP after it would:
 * QEMU's core leaves to the caller a continued transfer whose device
 * refuses the START, and a general call no device takes, its broadcast
 * flag set, which would make every later START a broadcast. Returns true
 * for a NACK, which the master answers as for any address (NACK_INT
 * where ack_check_en expects an ACK, then a STOP).
 */
static bool esp32p4_i2c_dev_start(ESP32P4I2CState *s, uint8_t addr, bool rd)
{
    int ret;

    if (!s->connected) {
        esp32p4_i2c_dev_end(s);
        return true;
    }
    if (s->dev_active && s->dev_addr != addr) {
        esp32p4_i2c_dev_end(s);
    }
    ret = i2c_start_transfer(s->bus, addr, rd);
    if (ret) {
        i2c_end_transfer(s->bus);
    }
    s->dev_active = ret == 0;
    s->dev_addr = addr;
    return ret != 0;
}


/* Master: scheduling */

/* Run @step at the instant @tp. */
static void esp32p4_i2c_fsm_at(ESP32P4I2CState *s, ESP32P4I2CTp tp,
                               unsigned step)
{
    s->step = step;
    s->step_tp = tp;
    s->step_mark = esp32p4_i2c_tp_ns(s, tp);
    s->step_left = 0;
    if (esp32p4_clk_edges_running(&s->tp_edges)) {
        timer_mod(&s->fsm_timer, s->step_mark);
    } else {
        timer_del(&s->fsm_timer);
    }
}

/*
 * Run @step @cycles of I2C_SCLK after the step now running, exactly: the
 * steps that follow each other keep to the clock however many they are.
 */
static void esp32p4_i2c_fsm_after(ESP32P4I2CState *s, uint64_t cycles,
                                  unsigned step)
{
    if (!esp32p4_clk_edges_running(&s->tp_edges)) {
        s->step = step;
        s->step_mark = s->step_ns;
        s->step_left = cycles;
        timer_del(&s->fsm_timer);
        return;
    }
    esp32p4_i2c_fsm_at(s, esp32p4_i2c_tp_after(s->cur_tp, cycles), step);
}

static bool esp32p4_i2c_fsm_waiting(ESP32P4I2CState *s)
{
    return s->step >= ST_WAIT_IDLE;
}

/* Wait on the lines: an input edge ends the wait. */
static void esp32p4_i2c_fsm_wait(ESP32P4I2CState *s, unsigned step)
{
    timer_del(&s->fsm_timer);
    s->step = step;
    s->wait_since = s->step_ns;
    esp32p4_i2c_to_update(s);
}

static void esp32p4_i2c_fsm_cancel(ESP32P4I2CState *s)
{
    s->step = ST_NONE;
    timer_del(&s->fsm_timer);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * A clock change rescales the cycles left to the pending step; a stopped
 * clock holds it until the clock runs again. The master's other instants
 * are taken at the whole nanoseconds they came to, at the old period.
 */
static void esp32p4_i2c_fsm_rescale(ESP32P4I2CState *s)
{
    int64_t now = esp32p4_i2c_now();
    ESP32P4ClkEdges old = s->tp_edges;
    ESP32P4ClkEdges edges = esp32p4_i2c_edges(s, now);
    unsigned i;

    if (esp32p4_clk_edges_eq(&old, &edges)) {
        return;
    }
    s->cur_tp = esp32p4_i2c_tp(esp32p4_i2c_tp_ns_at(&old, s->cur_tp));
    s->fall_tp = esp32p4_i2c_tp(esp32p4_i2c_tp_ns_at(&old, s->fall_tp));
    for (i = 0; i < 2; i++) {
        s->drv_tp[i] = esp32p4_i2c_tp(s->drv_ns[i]);
        s->line[i].edge_tp = esp32p4_i2c_tp(s->line[i].edge_ns);
        s->line[i].pend_tp = esp32p4_i2c_tp(
            esp32p4_i2c_tp_ns_at(&old, s->line[i].pend_tp));
    }
    s->tp_edges = edges;
    s->tp_sclk = esp32p4_clk_edges_or_at(s->sclk, now);
    if (s->step == ST_NONE || esp32p4_i2c_fsm_waiting(s)) {
        return;
    }
    if (esp32p4_clk_edges_running(&old)) {
        s->step_left = 0;
        if (s->step_mark > now) {
            s->step_left = esp32p4_clk_edges_between(&old, now,
                                                     s->step_mark);
        }
        s->step_mark = MIN(s->step_mark, now);
    }
    if (!esp32p4_clk_edges_running(&edges)) {
        timer_del(&s->fsm_timer);
        return;
    }
    s->step_tp = esp32p4_i2c_tp_after(esp32p4_i2c_tp(s->step_left ? now
                                                     : s->step_mark),
                                      s->step_left);
    s->step_mark = esp32p4_i2c_tp_ns(s, s->step_tp);
    s->step_left = 0;
    timer_mod(&s->fsm_timer, s->step_mark);
}

static void esp32p4_i2c_fsm_timer(void *opaque)
{
    ESP32P4I2CState *s = opaque;
    unsigned step = s->step;

    if (step == ST_NONE || esp32p4_i2c_fsm_waiting(s) ||
        !esp32p4_clk_edges_running(&s->tp_edges)) {
        return;
    }
    s->step = ST_NONE;
    s->step_ns = s->step_mark;
    s->cur_tp = s->step_tp;
    esp32p4_i2c_fsm_step(s, step);
}

/* Master: abandoning a transfer */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Stop the master where it is, release both lines and end the bus
 * devices' transfer: I2C_FSM_RST, a timeout or lost arbitration. The
 * controller takes the bus to be idle again, though no STOP was seen.
 */
static void esp32p4_i2c_mst_abort(ESP32P4I2CState *s, int64_t when)
{
    esp32p4_i2c_fsm_cancel(s);
    s->bus_active = false;
    esp32p4_i2c_dev_end(s);
    s->mst_busy = false;
    s->mst_suspended = false;
    s->mst_in_txn = false;
    s->nack_stop = false;
    s->start_pending = false;
    s->main_state = MAIN_IDLE;
    s->scl_state = SCLST_IDLE;
    esp32p4_i2c_drive(s, true, true, when);
    esp32p4_i2c_mst_idle(s);
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_arb_lost(ESP32P4I2CState *s)
{
    s->arb_lost = true;
    s->int_raw |= INT_ARBITRATION_LOST;
    esp32p4_i2c_mst_abort(s, s->step_ns);
    esp32p4_i2c_update_irq(s);
}

/* Master: START, repeated START */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * A START from an idle bus: once both lines are high and no other
 * master's transfer is in progress, SDA falls SCL_RSTART_SETUP_TIME + 1
 * cycles later. Until then the master waits on the lines.
 */
static void esp32p4_i2c_mst_start(ESP32P4I2CState *s)
{
    if (!esp32p4_i2c_filt(s, LINE_SCL) || !esp32p4_i2c_filt(s, LINE_SDA) ||
        s->bus_active) {
        esp32p4_i2c_fsm_wait(s, ST_WAIT_IDLE);
        return;
    }
    s->scl_state = SCLST_START;
    esp32p4_i2c_fsm_after(s, ACT(s, R_SCL_RSTART_SETUP) + 1, ST_START_SDA);
    esp32p4_i2c_to_update(s);
}

/* @tp, or the step now running if that is later. */
static ESP32P4I2CTp esp32p4_i2c_not_before_now(ESP32P4I2CState *s,
                                                ESP32P4I2CTp tp)
{
    return esp32p4_i2c_tp_ns(s, tp) < s->step_ns ? s->cur_tp : tp;
}

/* SCL seen high at @rise in a repeated START: SDA falls after setup. */
static void esp32p4_i2c_mst_rs_high(ESP32P4I2CState *s, ESP32P4I2CTp rise)
{
    ESP32P4I2CTp due = esp32p4_i2c_tp_after(rise,
                                            ACT(s, R_SCL_RSTART_SETUP) + 1);

    s->scl_state = SCLST_START;
    esp32p4_i2c_fsm_at(s, esp32p4_i2c_not_before_now(s, due), ST_START_SDA);
}

/* SCL seen high at @rise before a STOP: SDA rises after the setup. */
static void esp32p4_i2c_mst_stop_high(ESP32P4I2CState *s, ESP32P4I2CTp rise)
{
    ESP32P4I2CTp due = esp32p4_i2c_tp_after(rise,
                                            ACT(s, R_SCL_STOP_SETUP) + 1);

    s->scl_state = SCLST_STOP;
    esp32p4_i2c_fsm_at(s, esp32p4_i2c_not_before_now(s, due), ST_STOP_REL);
}

/* Master: bits */

/* The level the master drives on SDA in bit s->bit of the byte. */
static bool esp32p4_i2c_mst_bit_out(ESP32P4I2CState *s)
{
    bool lsb = esp32p4_i2c_ctr(s) & CTR_TX_LSB_FIRST;

    if (s->bit < 8) {
        if (s->reading) {
            return true;
        }
        return lsb ? extract32(s->cur_byte, s->bit, 1)
                   : extract32(s->cur_byte, 7 - s->bit, 1);
    }
    return s->reading ? s->mst_ack : true;
}

/* Begin bit s->bit, SCL having just fallen at s->step_ns. */
static void esp32p4_i2c_mst_bit_begin(ESP32P4I2CState *s)
{
    s->bit_out = esp32p4_i2c_mst_bit_out(s);
    s->scl_state = SCLST_LOW;
    if (s->bit < 8) {
        s->main_state = s->reading ? MAIN_RX_DATA :
                        s->addr_next ? MAIN_ADDR_SHIFT : MAIN_TX_DATA;
    } else {
        s->main_state = s->reading ? MAIN_SEND_ACK :
                        s->addr_next ? MAIN_ACK_ADDR : MAIN_WAIT_ACK;
    }
    esp32p4_i2c_fsm_after(s, ACT(s, R_SDA_HOLD) + 1, ST_BIT_SDA);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Begin a byte of a WRITE or READ command, SCL having just fallen: the
 * byte to send comes from the TX FIFO; for a READ, the addressed bus
 * device answers with its byte now.
 */
static void esp32p4_i2c_mst_byte_begin(ESP32P4I2CState *s)
{
    bool empty;

    s->bit = 0;
    if (s->reading) {
        s->cur_byte = 0;
        s->dev_byte = 0xff;
        if (s->dev_active && s->dev_read) {
            s->dev_byte = i2c_recv(s->bus);
        }
    } else {
        s->cur_byte = esp32p4_i2c_tx_pop(s, &empty);
        if (empty && esp32p4_i2c_prt(s)) {
            s->int_raw |= INT_MST_TXFIFO_UDF;
            esp32p4_i2c_update_irq(s);
        }
    }
    esp32p4_i2c_mst_bit_begin(s);
}

/*
 * SCL seen high: SDA is sampled SDA_SAMPLE_TIME + 1 cycles after the
 * edge (@rise) and SCL falls SCL_HIGH_PERIOD + 1 cycles after the high
 * period began (@high).
 */
static void esp32p4_i2c_mst_bit_high(ESP32P4I2CState *s, ESP32P4I2CTp high,
                                     ESP32P4I2CTp rise)
{
    ESP32P4I2CTp sample = esp32p4_i2c_tp_after(rise,
                                               ACT(s, R_SDA_SAMPLE) + 1);
    uint32_t len = extract32(ACT(s, R_SCL_HIGH_PERIOD), 0, 9);

    s->scl_state = SCLST_HIGH;
    s->fall_tp = esp32p4_i2c_tp_after(high, len + 1);
    s->t_fall_due = esp32p4_i2c_tp_ns(s, s->fall_tp);
    sample = esp32p4_i2c_not_before_now(s, sample);
    if (esp32p4_i2c_tp_ns(s, sample) < s->t_fall_due) {
        s->sample_at_fall = false;
        esp32p4_i2c_fsm_at(s, sample, ST_BIT_SAMPLE);
    } else {
        s->sample_at_fall = true;
        esp32p4_i2c_fsm_at(s, s->fall_tp, ST_BIT_FALL);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Sample SDA in bit s->bit: the line as the SDA filter passes it, ANDed
 * with what an addressed bus device drives. The ninth bit of a byte the
 * master sends is the receiver's acknowledge: for the address byte after
 * a START the bus's devices are addressed here, for the others the
 * addressed device takes the byte. A master driving SDA high that finds
 * it low has lost arbitration.
 */
static void esp32p4_i2c_mst_sample(ESP32P4I2CState *s)
{
    bool lvl = esp32p4_i2c_filt(s, LINE_SDA);
    bool arb = esp32p4_i2c_ctr(s) & CTR_ARBITRATION_EN;
    bool lsb = esp32p4_i2c_ctr(s) & CTR_TX_LSB_FIRST;
    uint8_t wire;

    if (s->bit < 8) {
        if (s->reading) {
            lvl &= extract32(s->dev_byte, 7 - s->bit, 1);
            s->cur_byte = (s->cur_byte << 1) | lvl;
            return;
        }
        if (arb && s->bit_out && !lvl) {
            esp32p4_i2c_arb_lost(s);
        }
        return;
    }
    if (s->reading) {
        if (arb && s->bit_out && !lvl) {
            esp32p4_i2c_arb_lost(s);
        }
        return;
    }
    /* The byte as it went on the wire, MSB first. */
    wire = lsb ? revbit8(s->cur_byte) : s->cur_byte;
    if (s->addr_next) {
        s->dev_read = wire & 1;
        s->dev_nack = esp32p4_i2c_dev_start(s, wire >> 1, wire & 1);
    } else if (s->dev_active && !s->dev_read) {
        s->dev_nack = i2c_send(s->bus, wire) != 0;
    } else {
        s->dev_nack = true;
    }
    lvl &= s->dev_nack;
    s->resp_rec = lvl;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Bit s->bit is over, SCL having just fallen. After the eighth bit a
 * received byte goes to the RX FIFO, and the master answers it with the
 * READ command's ACK value, or with RX_FULL_ACK_LEVEL if it had no room;
 * after the ninth the byte is done: a NACK where WRITE expected an ACK
 * (ack_check_en) raises NACK_INT and ends the transfer with a STOP.
 */
static void esp32p4_i2c_mst_bit_end(ESP32P4I2CState *s)
{
    uint32_t cmd = s->regs[R_COMD0 + s->cmd_idx];
    uint32_t ctr = esp32p4_i2c_ctr(s);
    uint8_t b;

    if (s->bit < 7) {
        s->bit++;
        esp32p4_i2c_mst_bit_begin(s);
        return;
    }
    if (s->bit == 7) {
        if (s->reading) {
            b = (ctr & CTR_RX_LSB_FIRST) ? revbit8(s->cur_byte) : s->cur_byte;
            s->mst_ack = (cmd & CMD_ACK_VALUE) != 0;
            if (!esp32p4_i2c_rx_push(s, b)) {
                s->mst_ack = (ctr & CTR_RX_FULL_ACK_LEVEL) != 0;
            }
        }
        s->bit = 8;
        esp32p4_i2c_mst_bit_begin(s);
        return;
    }

    s->int_raw |= INT_BYTE_TRANS_DONE;
    esp32p4_i2c_update_irq(s);
    if (s->reading) {
        /* A NACKed transmitter stops driving SDA: what follows reads 1s. */
        if (s->mst_ack && s->dev_active && s->dev_read) {
            i2c_nack(s->bus);
            s->dev_read = false;
        }
    } else {
        s->addr_next = false;
        if ((cmd & CMD_ACK_CHECK_EN) &&
            s->resp_rec != !!(cmd & CMD_ACK_EXP)) {
            s->int_raw |= INT_NACK;
            esp32p4_i2c_update_irq(s);
            s->nack_stop = true;
            esp32p4_i2c_fsm_after(s, ACT(s, R_SDA_HOLD) + 1, ST_STOP_SDA);
            return;
        }
    }
    if (--s->byte_left) {
        esp32p4_i2c_mst_byte_begin(s);
        return;
    }
    s->regs[R_COMD0 + s->cmd_idx] |= CMD_DONE;
    s->cmd_idx++;
    esp32p4_i2c_mst_next_cmd(s);
}

/* Master: the command list */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Run command s->cmd_idx, with SCL low after a byte or a START (or high
 * on an idle bus for the first RSTART).
 */
static void esp32p4_i2c_mst_next_cmd(ESP32P4I2CState *s)
{
    uint32_t cmd;

    for (;;) {
        if (s->cmd_idx >= ESP32P4_I2C_CMDS) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: the command list has no "
                          "STOP or END\n", esp32p4_i2c_name(s));
            s->mst_busy = false;
            s->mst_suspended = s->mst_in_txn;
            esp32p4_i2c_mst_idle(s);
            return;
        }
        cmd = s->regs[R_COMD0 + s->cmd_idx];
        switch (CMD_OPCODE(cmd)) {
        case OP_RSTART:
            if (s->mst_in_txn) {
                s->scl_state = SCLST_LOW;
                esp32p4_i2c_fsm_after(s, ACT(s, R_SDA_HOLD) + 1, ST_RS_SDA);
            } else {
                esp32p4_i2c_mst_start(s);
            }
            return;
        case OP_WRITE:
        case OP_READ:
            if (!CMD_BYTE_NUM(cmd)) {
                s->regs[R_COMD0 + s->cmd_idx] |= CMD_DONE;
                s->cmd_idx++;
                continue;
            }
            if (!s->mst_in_txn) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: COMD%u: %s without a "
                              "START\n", esp32p4_i2c_name(s), s->cmd_idx,
                              CMD_OPCODE(cmd) == OP_READ ? "READ" : "WRITE");
                s->mst_busy = false;
                esp32p4_i2c_mst_idle(s);
                return;
            }
            s->reading = CMD_OPCODE(cmd) == OP_READ;
            s->byte_left = CMD_BYTE_NUM(cmd);
            esp32p4_i2c_mst_byte_begin(s);
            return;
        case OP_STOP:
            s->scl_state = SCLST_LOW;
            esp32p4_i2c_fsm_after(s, ACT(s, R_SDA_HOLD) + 1, ST_STOP_SDA);
            return;
        case OP_END:
            esp32p4_i2c_fsm_after(s, ACT(s, R_SDA_HOLD) + 1, ST_END_SDA);
            return;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: COMD%u: invalid opcode %u\n",
                          esp32p4_i2c_name(s), s->cmd_idx, CMD_OPCODE(cmd));
            s->cmd_idx++;
            continue;
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_mst_trans_start(ESP32P4I2CState *s)
{
    if (s->step == ST_STOP_IDLE) {
        /* After the STOP, once the bus free time is over. */
        s->start_pending = true;
        return;
    }
    if (s->mst_busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TRANS_START while the master is "
                      "running its command list\n", esp32p4_i2c_name(s));
        return;
    }
    s->mst_busy = true;
    s->mst_suspended = false;
    s->cmd_idx = 0;
    s->step_ns = esp32p4_i2c_now();
    s->cur_tp = esp32p4_i2c_tp(s->step_ns);
    esp32p4_i2c_mst_next_cmd(s);
    esp32p4_i2c_to_update(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Send SCL_RST_SLV_NUM pulses on SCL with SDA released (I2C_SCL_SP_CONF),
 * for a slave stuck holding SDA low, once the master is idle; the
 * controller clears SCL_RST_SLV_EN when they are done.
 */
static void esp32p4_i2c_mst_clear_bus(ESP32P4I2CState *s)
{
    if (!esp32p4_i2c_master(s) || s->mst_busy || s->mst_suspended ||
        !(ACT(s, R_SCL_SP_CONF) & SP_RST_SLV_EN)) {
        return;
    }
    s->rst_pulses = SP_RST_SLV_NUM(ACT(s, R_SCL_SP_CONF));
    s->step_ns = esp32p4_i2c_now();
    s->cur_tp = esp32p4_i2c_tp(s->step_ns);
    if (!s->rst_pulses) {
        s->regs[R_SCL_SP_CONF] &= ~SP_RST_SLV_EN;
        s->act[R_SCL_SP_CONF] &= ~SP_RST_SLV_EN;
        return;
    }
    s->mst_busy = true;
    esp32p4_i2c_sda(s, true, s->step_ns);
    esp32p4_i2c_fsm_after(s, 0, ST_CLR_LOW);
}

/* The master has nothing to do: run what waited for it. */
static void esp32p4_i2c_mst_idle(ESP32P4I2CState *s)
{
    if (!s->mst_busy && !s->mst_suspended) {
        s->main_state = MAIN_IDLE;
        s->scl_state = SCLST_IDLE;
        esp32p4_i2c_mst_clear_bus(s);
    }
    esp32p4_i2c_to_update(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The master's steps, each at its time s->step_ns.
 */
static void esp32p4_i2c_fsm_step(ESP32P4I2CState *s, unsigned step)
{
    uint32_t low = ACT(s, R_SCL_LOW_PERIOD);
    uint32_t hold = ACT(s, R_SDA_HOLD);
    uint32_t hp = ACT(s, R_SCL_HIGH_PERIOD);
    uint32_t wait = extract32(hp, 9, 7) + 1 +
                    esp32p4_i2c_thres(s, LINE_SCL);
    uint32_t scl_low = low > hold ? low - hold : 0;
    int64_t t = s->step_ns;

    switch (step) {
    case ST_START_SDA:
        if (!esp32p4_i2c_filt(s, LINE_SDA) || !esp32p4_i2c_filt(s, LINE_SCL)) {
            if (s->mst_in_txn) {
                esp32p4_i2c_arb_lost(s);
            } else {
                esp32p4_i2c_mst_start(s);
            }
            return;
        }
        esp32p4_i2c_sda(s, false, t);
        s->mst_in_txn = true;
        s->addr_next = true;
        s->chk_sda = true;
        s->chk_scl = true;
        s->connected = false;
        s->arb_lost = false;
        s->int_raw |= INT_TRANS_START;
        esp32p4_i2c_update_irq(s);
        esp32p4_i2c_fsm_after(s, ACT(s, R_SCL_START_HOLD) + 1, ST_START_SCL);
        esp32p4_i2c_to_update(s);
        break;
    case ST_START_SCL:
        esp32p4_i2c_scl(s, false, t);
        s->scl_state = SCLST_NEGEDGE;
        s->regs[R_COMD0 + s->cmd_idx] |= CMD_DONE;
        s->cmd_idx++;
        esp32p4_i2c_mst_next_cmd(s);
        break;
    case ST_RS_SDA:
        esp32p4_i2c_sda(s, true, t);
        esp32p4_i2c_fsm_after(s, scl_low, ST_RS_SCL);
        break;
    case ST_RS_SCL:
        esp32p4_i2c_scl(s, true, t);
        s->scl_state = SCLST_POSEDGE;
        s->t_rel = t;
        esp32p4_i2c_fsm_after(s, wait, ST_RS_HIGH);
        break;
    case ST_RS_HIGH:
        if (!esp32p4_i2c_filt(s, LINE_SCL)) {
            esp32p4_i2c_fsm_wait(s, ST_WAIT_RS_HIGH);
            return;
        }
        esp32p4_i2c_mst_rs_high(s, s->line[LINE_SCL].edge_tp);
        break;
    case ST_BIT_SDA:
        if (s->chk_sda) {
            s->connected_sda = !esp32p4_i2c_filt(s, LINE_SDA);
            s->chk_sda = false;
        }
        esp32p4_i2c_sda(s, s->bit_out, t);
        esp32p4_i2c_fsm_after(s, scl_low, ST_BIT_SCL);
        break;
    case ST_BIT_SCL:
        if (s->chk_scl) {
            s->connected = s->connected_sda &&
                           !esp32p4_i2c_filt(s, LINE_SCL);
            s->chk_scl = false;
        }
        esp32p4_i2c_scl(s, true, t);
        s->scl_state = SCLST_POSEDGE;
        s->t_rel = t;
        esp32p4_i2c_fsm_after(s, wait, ST_BIT_HIGH);
        break;
    case ST_BIT_HIGH:
        if (!esp32p4_i2c_filt(s, LINE_SCL)) {
            esp32p4_i2c_fsm_wait(s, ST_WAIT_BIT_HIGH);
            return;
        }
        esp32p4_i2c_mst_bit_high(s, s->cur_tp, s->line[LINE_SCL].edge_tp);
        break;
    case ST_BIT_SAMPLE:
        esp32p4_i2c_mst_sample(s);
        if (s->mst_busy) {
            esp32p4_i2c_fsm_at(s, s->fall_tp, ST_BIT_FALL);
        }
        break;
    case ST_BIT_FALL:
        if (s->sample_at_fall) {
            esp32p4_i2c_mst_sample(s);
            if (!s->mst_busy) {
                return;
            }
        }
        esp32p4_i2c_scl(s, false, t);
        s->scl_state = SCLST_NEGEDGE;
        esp32p4_i2c_mst_bit_end(s);
        break;
    case ST_STOP_SDA:
        esp32p4_i2c_sda(s, false, t);
        esp32p4_i2c_fsm_after(s, s->scl_drv ? 0 : scl_low, ST_STOP_SCL);
        break;
    case ST_STOP_SCL:
        esp32p4_i2c_scl(s, true, t);
        s->scl_state = SCLST_POSEDGE;
        s->t_rel = t;
        esp32p4_i2c_fsm_after(s, wait, ST_STOP_HIGH);
        break;
    case ST_STOP_HIGH:
        if (!esp32p4_i2c_filt(s, LINE_SCL)) {
            esp32p4_i2c_fsm_wait(s, ST_WAIT_STOP_HIGH);
            return;
        }
        esp32p4_i2c_mst_stop_high(s, s->line[LINE_SCL].edge_tp);
        break;
    case ST_STOP_REL:
        esp32p4_i2c_sda(s, true, t);
        esp32p4_i2c_dev_end(s);
        if (!s->nack_stop) {
            s->regs[R_COMD0 + s->cmd_idx] |= CMD_DONE;
        }
        s->mst_in_txn = false;
        s->int_raw |= INT_TRANS_COMPLETE;
        esp32p4_i2c_update_irq(s);
        esp32p4_i2c_fsm_after(s, ACT(s, R_SCL_STOP_HOLD) + 1, ST_STOP_IDLE);
        esp32p4_i2c_to_update(s);
        break;
    case ST_STOP_IDLE:
        s->nack_stop = false;
        s->mst_busy = false;
        if (s->start_pending) {
            s->start_pending = false;
            esp32p4_i2c_mst_trans_start(s);
            break;
        }
        esp32p4_i2c_mst_idle(s);
        break;
    case ST_END_SDA:
        /* SCL stays low; the master is ready for TRANS_START. */
        esp32p4_i2c_sda(s, true, t);
        s->regs[R_COMD0 + s->cmd_idx] |= CMD_DONE;
        s->int_raw |= INT_END_DETECT;
        esp32p4_i2c_update_irq(s);
        s->mst_busy = false;
        s->mst_suspended = true;
        s->main_state = MAIN_IDLE;
        esp32p4_i2c_to_update(s);
        break;
    case ST_CLR_LOW:
        esp32p4_i2c_scl(s, false, t);
        esp32p4_i2c_fsm_after(s, low + 1, ST_CLR_HIGH);
        break;
    case ST_CLR_HIGH:
        esp32p4_i2c_scl(s, true, t);
        esp32p4_i2c_fsm_after(s, wait + extract32(hp, 0, 9) + 1,
                              ST_CLR_NEXT);
        break;
    case ST_CLR_NEXT:
        if (--s->rst_pulses) {
            esp32p4_i2c_fsm_after(s, 0, ST_CLR_LOW);
            break;
        }
        s->regs[R_SCL_SP_CONF] &= ~SP_RST_SLV_EN;
        s->act[R_SCL_SP_CONF] &= ~SP_RST_SLV_EN;
        s->mst_busy = false;
        esp32p4_i2c_mst_idle(s);
        break;
    default:
        g_assert_not_reached();
    }
}

/*
 * An edge on the lines at @when ends the master's wait: SCL rising for a
 * bit, repeated START or STOP; any change for an idle bus.
 */
static void esp32p4_i2c_mst_edge(ESP32P4I2CState *s, int n, int64_t when)
{
    unsigned step = s->step;

    if (!esp32p4_i2c_fsm_waiting(s)) {
        return;
    }
    if (step != ST_WAIT_IDLE &&
        (n != LINE_SCL || !esp32p4_i2c_filt(s, LINE_SCL))) {
        return;
    }
    s->step = ST_NONE;
    s->step_ns = when;
    s->cur_tp = s->line[n].edge_tp;
    switch (step) {
    case ST_WAIT_IDLE:
        esp32p4_i2c_mst_start(s);
        break;
    case ST_WAIT_RS_HIGH:
        esp32p4_i2c_mst_rs_high(s, s->cur_tp);
        break;
    case ST_WAIT_BIT_HIGH:
        esp32p4_i2c_mst_bit_high(s, s->cur_tp, s->cur_tp);
        break;
    case ST_WAIT_STOP_HIGH:
        esp32p4_i2c_mst_stop_high(s, s->cur_tp);
        break;
    }
}

/* Slave */

static bool esp32p4_i2c_slave_mode(ESP32P4I2CState *s)
{
    return !esp32p4_i2c_master(s) && esp32p4_i2c_runs(s);
}

static void esp32p4_i2c_slv_timer_update(ESP32P4I2CState *s)
{
    int64_t due = MIN(s->slv_sda_due, s->slv_scl_due);

    if (due == INT64_MAX) {
        timer_del(&s->slv_timer);
    } else {
        timer_mod(&s->slv_timer, due);
    }
}

/* Drive SDA to @lvl at @when. */
static void esp32p4_i2c_slv_sda_at(ESP32P4I2CState *s, bool lvl,
                                   int64_t when)
{
    s->slv_sda_lvl = lvl;
    s->slv_sda_due = when;
    esp32p4_i2c_slv_timer_update(s);
}

static void esp32p4_i2c_slv_scl_at(ESP32P4I2CState *s, bool lvl,
                                   int64_t when)
{
    s->slv_scl_lvl = lvl;
    s->slv_scl_due = when;
    esp32p4_i2c_slv_timer_update(s);
}

static void esp32p4_i2c_slv_timer(void *opaque)
{
    ESP32P4I2CState *s = opaque;
    int64_t now = esp32p4_i2c_now();

    while (MIN(s->slv_sda_due, s->slv_scl_due) <= now) {
        if (s->slv_sda_due <= s->slv_scl_due) {
            int64_t when = s->slv_sda_due;

            s->slv_sda_due = INT64_MAX;
            esp32p4_i2c_sda(s, s->slv_sda_lvl, when);
        } else {
            int64_t when = s->slv_scl_due;

            s->slv_scl_due = INT64_MAX;
            esp32p4_i2c_scl(s, s->slv_scl_lvl, when);
        }
    }
    esp32p4_i2c_slv_timer_update(s);
}

/* Forget the transfer: release both lines, wait for a START. */
static void esp32p4_i2c_slv_reset(ESP32P4I2CState *s, int64_t when)
{
    s->slv_state = SLV_IDLE;
    s->slv_matched = false;
    s->slv_10bit = false;
    s->slv_stretch = false;
    s->slv_stretch_req = 0;
    s->slv_sda_due = INT64_MAX;
    s->slv_scl_due = INT64_MAX;
    timer_del(&s->slv_timer);
    if (!s->scl_drv || !s->sda_drv) {
        esp32p4_i2c_drive(s, true, true, when);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The byte to send as a slave: from the TX FIFO, from TX RAM at the
 * offset the master gave in dual address mode, or in non-FIFO mode at
 * the read pointer.
 */
static uint8_t esp32p4_i2c_slv_tx_fetch(ESP32P4I2CState *s)
{
    unsigned depth = esp32p4_i2c_depth(s);
    bool empty;
    uint8_t b;

    if (!esp32p4_i2c_fifo_mode(s) && (ACT(s, R_FIFO_CONF) & FIFO_ADDR_CFG_EN)) {
        b = s->txram[s->slave_rw_point % depth];
        s->slave_rw_point++;
        return b;
    }
    return esp32p4_i2c_tx_pop(s, &empty);
}

/* Drive the bit of the byte being sent that s->slv_bits counts. */
static void esp32p4_i2c_slv_tx_bit(ESP32P4I2CState *s, int64_t when)
{
    bool lsb = esp32p4_i2c_ctr(s) & CTR_TX_LSB_FIRST;
    unsigned i = s->slv_bits;

    esp32p4_i2c_slv_sda_at(s, lsb ? extract32(s->slv_byte, i, 1)
                                  : extract32(s->slv_byte, 7 - i, 1),
                           when);
    s->slv_bits++;
}

static void esp32p4_i2c_slv_stretch(ESP32P4I2CState *s, uint8_t cause,
                                    bool tx, int64_t when)
{
    s->slv_stretch = true;
    s->slv_stretch_tx = tx;
    s->stretch_cause = cause;
    esp32p4_i2c_slv_scl_at(s, false, when);
    s->int_raw |= INT_SLAVE_STRETCH;
    esp32p4_i2c_update_irq(s);
    esp32p4_i2c_to_update(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Start sending a byte, SCL having fallen at @when after an acknowledge.
 * With SCL stretching on, the slave holds SCL low when the master has
 * just addressed it for a read (cause 0) and when its TX FIFO is empty
 * (cause 1), until software sets SLAVE_SCL_STRETCH_CLR.
 */
static void esp32p4_i2c_slv_tx_begin(ESP32P4I2CState *s, bool first,
                                     int64_t when)
{
    bool stretch = ACT(s, R_SCL_STRETCH_CONF) & STRETCH_EN;
    uint32_t ctr = esp32p4_i2c_ctr(s);

    s->slv_state = SLV_TX;
    s->slv_bits = 0;
    if (first && !(ctr & CTR_SLV_TX_AUTO_START) && !s->slv_armed) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: addressed for a read with "
                      "neither SLV_TX_AUTO_START_EN nor TRANS_START\n",
                      esp32p4_i2c_name(s));
        s->slv_state = SLV_IGNORE;
        esp32p4_i2c_slv_sda_at(s, true, when);
        return;
    }
    if (stretch && first) {
        esp32p4_i2c_slv_stretch(s, STRETCH_ADDR_MATCH, true, when);
        return;
    }
    if (stretch && esp32p4_i2c_fifo_mode(s) && !s->tx_cnt) {
        esp32p4_i2c_slv_stretch(s, STRETCH_TX_EMPTY, true, when);
        return;
    }
    s->slv_byte = esp32p4_i2c_slv_tx_fetch(s);
    esp32p4_i2c_slv_tx_bit(s, esp32p4_i2c_after(s, when,
                                                ACT(s, R_SDA_HOLD) + 1));
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The acknowledge for a received data byte, as the slave sends it once
 * any stretch is over: RX_FULL_ACK_LEVEL while the RX FIFO is full,
 * otherwise SLAVE_BYTE_ACK_LVL with SLAVE_BYTE_ACK_CTL_EN, otherwise an
 * ACK.
 */
static bool esp32p4_i2c_slv_rx_ack(ESP32P4I2CState *s)
{
    uint32_t st = ACT(s, R_SCL_STRETCH_CONF);

    if (esp32p4_i2c_rx_full(s) && esp32p4_i2c_fifo_mode(s)) {
        return esp32p4_i2c_ctr(s) & CTR_RX_FULL_ACK_LEVEL;
    }
    if (st & STRETCH_ACK_CTL_EN) {
        return st & STRETCH_ACK_LVL;
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * A byte has come in (the eighth SCL rise): the address, compared with
 * I2C_SLAVE_ADDR (7-bit, 10-bit, or the general call with
 * ADDR_BROADCASTING_EN), or data for the RX FIFO, or in dual address
 * mode the RAM offset and then data at it. Decide the acknowledge and
 * what follows it.
 */
static void esp32p4_i2c_slv_byte(ESP32P4I2CState *s)
{
    uint32_t sa = ACT(s, R_SLAVE_ADDR);
    uint32_t ctr = esp32p4_i2c_ctr(s);
    uint32_t st = ACT(s, R_SCL_STRETCH_CONF);
    unsigned depth = esp32p4_i2c_depth(s);
    uint8_t b = s->slv_byte;
    bool rw = b & 1;
    bool ok;

    s->slv_ack = true;
    s->slv_next = SLV_IGNORE;
    switch (s->slv_state) {
    case SLV_ADDR:
        if (sa & SLAVE_ADDR_10BIT_EN) {
            if ((b >> 1) != extract32(sa, 0, 7)) {
                break;
            }
            if (rw && s->slv_10bit) {
                s->slv_ack = false;
                s->slv_matched = true;
                s->slave_rw = true;
                s->slave_addressed = true;
                s->slv_next = SLV_TX;
                return;
            }
            if (rw && (ctr & CTR_ADDR_10BIT_RW_CHECK)) {
                return;
            }
            s->slv_ack = false;
            s->slv_next = SLV_ADDR2;
            return;
        }
        if ((b >> 1) == extract32(sa, 0, 7)) {
            s->slv_ack = false;
            s->slv_matched = true;
            s->slave_rw = rw;
            s->slave_addressed = true;
            s->slv_next = rw ? SLV_TX : SLV_RX;
            return;
        }
        if (b == 0 && (ctr & CTR_ADDR_BROADCASTING)) {
            s->slv_ack = false;
            s->slv_matched = true;
            s->slave_rw = false;
            s->slave_addressed = true;
            s->slv_next = SLV_RX;
            s->int_raw |= INT_GENERAL_CALL;
            esp32p4_i2c_update_irq(s);
            return;
        }
        break;
    case SLV_ADDR2:
        if (b == extract32(sa, 7, 8)) {
            s->slv_ack = false;
            s->slv_matched = true;
            s->slv_10bit = true;
            s->slave_rw = false;
            s->slave_addressed = true;
            s->slv_next = SLV_RX;
            return;
        }
        break;
    case SLV_RX:
        s->slv_next = SLV_RX;
        s->int_raw |= INT_BYTE_TRANS_DONE;
        if (s->slv_first_data) {
            s->slv_first_data = false;
            s->slave_rw_point = b;
            s->slv_ack = false;
            esp32p4_i2c_update_irq(s);
            return;
        }
        if (ctr & CTR_RX_LSB_FIRST) {
            b = revbit8(b);
        }
        if (!esp32p4_i2c_fifo_mode(s) &&
            (ACT(s, R_FIFO_CONF) & FIFO_ADDR_CFG_EN)) {
            s->rxram[s->slave_rw_point % depth] = b;
            s->slave_rw_point++;
            s->slv_ack = false;
            esp32p4_i2c_update_irq(s);
            return;
        }
        ok = esp32p4_i2c_rx_push(s, b);
        if (!ok) {
            s->slv_ack = ctr & CTR_RX_FULL_ACK_LEVEL;
            return;
        }
        if ((st & STRETCH_EN) && esp32p4_i2c_rx_full(s)) {
            s->slv_stretch_req = STRETCH_RX_FULL + 1;
        } else if ((st & STRETCH_EN) && (st & STRETCH_ACK_CTL_EN)) {
            s->slv_stretch_req = STRETCH_SENDING_ACK + 1;
        }
        s->slv_ack = esp32p4_i2c_slv_rx_ack(s);
        return;
    default:
        return;
    }
    s->slave_addressed = false;
    s->int_raw |= INT_SLAVE_ADDR_UNMATCH;
    esp32p4_i2c_update_irq(s);
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_slv_start(ESP32P4I2CState *s, int64_t when)
{
    s->slv_state = SLV_ADDR;
    s->slv_bits = 0;
    s->slv_byte = 0;
    s->slv_stretch_req = 0;
    s->slv_first_data = ACT(s, R_FIFO_CONF) & FIFO_ADDR_CFG_EN;
    s->slv_sda_due = INT64_MAX;
    esp32p4_i2c_slv_timer_update(s);
    if (!s->sda_drv) {
        esp32p4_i2c_sda(s, true, when);
    }
    s->main_state = MAIN_ADDR_SHIFT;
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_slv_stop(ESP32P4I2CState *s, int64_t when)
{
    if (s->slv_matched) {
        s->int_raw |= INT_TRANS_COMPLETE;
        if (s->slave_rw) {
            s->slv_armed = false;
        }
        esp32p4_i2c_update_irq(s);
    }
    esp32p4_i2c_slv_reset(s, when);
    s->main_state = MAIN_IDLE;
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_slv_scl_rise(ESP32P4I2CState *s, int64_t when)
{
    switch (s->slv_state) {
    case SLV_ADDR:
    case SLV_ADDR2:
    case SLV_RX:
        if (s->slv_bits < 8) {
            s->slv_byte = (s->slv_byte << 1) |
                          esp32p4_i2c_filt(s, LINE_SDA);
            if (++s->slv_bits == 8) {
                esp32p4_i2c_slv_byte(s);
            }
        }
        break;
    case SLV_TX:
        if (s->slv_bits == 9) {
            s->slv_master_nack = esp32p4_i2c_filt(s, LINE_SDA);
            s->resp_rec = s->slv_master_nack;
        }
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * SCL has fallen at @when: drive the next bit SDA_HOLD_TIME + 1 cycles
 * later, the acknowledge after a received byte, or stretch.
 */
static void esp32p4_i2c_slv_scl_fall(ESP32P4I2CState *s, int64_t when)
{
    int64_t hold = esp32p4_i2c_after(s, when, ACT(s, R_SDA_HOLD) + 1);
    uint8_t req;

    switch (s->slv_state) {
    case SLV_ADDR:
    case SLV_ADDR2:
    case SLV_RX:
        if (s->slv_bits == 8) {
            s->slv_bits = 9;
            s->main_state = MAIN_SEND_ACK;
            req = s->slv_stretch_req;
            s->slv_stretch_req = 0;
            if (req) {
                esp32p4_i2c_slv_stretch(s, req - 1, false, when);
                break;
            }
            esp32p4_i2c_slv_sda_at(s, s->slv_ack, hold);
        } else if (s->slv_bits == 9) {
            s->slv_bits = 0;
            s->slv_byte = 0;
            if (s->slv_next == SLV_TX) {
                esp32p4_i2c_slv_tx_begin(s, true, when);
                break;
            }
            esp32p4_i2c_slv_sda_at(s, true, hold);
            s->slv_state = s->slv_next;
            s->main_state = s->slv_state == SLV_RX ? MAIN_RX_DATA :
                            s->slv_state == SLV_ADDR2 ? MAIN_ADDR_SHIFT :
                            MAIN_IDLE;
        }
        break;
    case SLV_TX:
        if (s->slv_bits < 8) {
            esp32p4_i2c_slv_tx_bit(s, hold);
            s->main_state = MAIN_TX_DATA;
        } else if (s->slv_bits == 8) {
            esp32p4_i2c_slv_sda_at(s, true, hold);
            s->slv_bits = 9;
            s->main_state = MAIN_WAIT_ACK;
            s->int_raw |= INT_BYTE_TRANS_DONE;
            esp32p4_i2c_update_irq(s);
        } else if (s->slv_master_nack) {
            s->int_raw |= INT_NACK;
            esp32p4_i2c_update_irq(s);
            s->slv_state = SLV_IGNORE;
            s->main_state = MAIN_IDLE;
        } else {
            esp32p4_i2c_slv_tx_begin(s, false, when);
        }
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * SLAVE_SCL_STRETCH_CLR: put the bit the stretch held back on SDA and
 * release SCL STRETCH_PROTECT_NUM cycles later.
 */
static void esp32p4_i2c_slv_stretch_clr(ESP32P4I2CState *s)
{
    int64_t now = esp32p4_i2c_now();
    uint32_t protect = STRETCH_PROTECT(ACT(s, R_SCL_STRETCH_CONF));

    if (!s->slv_stretch || !esp32p4_i2c_runs(s)) {
        return;
    }
    s->slv_stretch = false;
    if (s->slv_stretch_tx) {
        s->slv_byte = esp32p4_i2c_slv_tx_fetch(s);
        s->slv_bits = 0;
        esp32p4_i2c_slv_tx_bit(s, now);
    } else {
        s->slv_ack = esp32p4_i2c_slv_rx_ack(s);
        esp32p4_i2c_slv_sda_at(s, s->slv_ack, now);
    }
    esp32p4_i2c_slv_scl_at(s, true, esp32p4_i2c_after(s, now, protect));
    esp32p4_i2c_to_update(s);
}

/* Lines */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * An edge has passed the filter on line @n at @when: START and STOP
 * conditions on SDA while SCL is high, and the master's or slave's
 * reaction.
 */
static void esp32p4_i2c_edge(ESP32P4I2CState *s, int n, bool level,
                             int64_t when)
{
    bool slave = esp32p4_i2c_slave_mode(s);

    if (n == LINE_SDA && esp32p4_i2c_filt(s, LINE_SCL)) {
        if (!level) {
            s->bus_active = true;
            s->int_raw |= INT_DET_START;
            esp32p4_i2c_update_irq(s);
            if (slave) {
                esp32p4_i2c_slv_start(s, when);
            }
        } else {
            s->bus_active = false;
            if (slave) {
                esp32p4_i2c_slv_stop(s, when);
            }
        }
    } else if (n == LINE_SCL && slave && s->bus_active) {
        if (level) {
            esp32p4_i2c_slv_scl_rise(s, when);
        } else {
            esp32p4_i2c_slv_scl_fall(s, when);
        }
    }
    if (esp32p4_i2c_master(s)) {
        esp32p4_i2c_mst_edge(s, n, when);
    }
    esp32p4_i2c_to_update(s);
}

static void esp32p4_i2c_accept(ESP32P4I2CState *s, int n, int64_t when,
                               ESP32P4I2CTp tp)
{
    ESP32P4I2CLine *l = &s->line[n];

    l->filt = l->raw;
    l->edge_ns = when;
    l->edge_tp = tp;
    l->pend_ns = INT64_MAX;
    esp32p4_i2c_edge(s, n, l->filt, when);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The SCL and SDA filters: a new level passes once it has held for
 * FILTER_THRES cycles of I2C_SCLK; a pulse shorter than that is lost.
 * With I2C_SCLK stopped the controller does nothing, and the levels pass
 * as they are. A change the controller made itself is counted from the
 * exact instant of the step that made it.
 */
static void esp32p4_i2c_in(ESP32P4I2CState *s, int n, int level)
{
    ESP32P4I2CLine *l = &s->line[n];
    int64_t when = esp32_line_time();
    ESP32P4I2CTp tp = when == s->drv_ns[n] ? s->drv_tp[n]
                                           : esp32p4_i2c_tp(when);
    unsigned thres;

    if (l->raw == !!level) {
        return;
    }
    l->raw = level;
    if (!esp32p4_i2c_runs(s)) {
        timer_del(&s->flt_timer[n]);
        l->filt = l->raw;
        l->edge_ns = when;
        l->edge_tp = esp32p4_i2c_tp(when);
        l->pend_ns = INT64_MAX;
        return;
    }
    if (l->raw == l->filt) {
        timer_del(&s->flt_timer[n]);
        l->pend_ns = INT64_MAX;
        return;
    }
    thres = esp32p4_i2c_thres(s, n);
    if (!thres) {
        esp32p4_i2c_accept(s, n, when, tp);
        return;
    }
    l->pend_tp = esp32p4_i2c_tp_after(tp, thres);
    l->pend_ns = MAX(esp32p4_i2c_tp_ns(s, l->pend_tp), when);
    timer_mod(&s->flt_timer[n], l->pend_ns);
}

static void esp32p4_i2c_scl_in(void *opaque, int n, int level)
{
    esp32p4_i2c_in(opaque, LINE_SCL, level);
}

static void esp32p4_i2c_sda_in(void *opaque, int n, int level)
{
    esp32p4_i2c_in(opaque, LINE_SDA, level);
}

static void esp32p4_i2c_flt(ESP32P4I2CState *s, int n)
{
    ESP32P4I2CLine *l = &s->line[n];

    if (l->pend_ns != INT64_MAX && l->raw != l->filt) {
        esp32p4_i2c_accept(s, n, l->pend_ns, l->pend_tp);
    }
}

static void esp32p4_i2c_flt_scl(void *opaque)
{
    esp32p4_i2c_flt(opaque, LINE_SCL);
}

static void esp32p4_i2c_flt_sda(void *opaque)
{
    esp32p4_i2c_flt(opaque, LINE_SDA);
}

/* Timeouts */

/*
 * Whether the SCL timeout runs: a master in its transfer (but for its
 * own END) or waiting for the bus to start one; a slave in a transfer
 * that is not holding SCL low itself.
 */
static bool esp32p4_i2c_to_cond(ESP32P4I2CState *s)
{
    if (esp32p4_i2c_master(s)) {
        return (s->mst_busy && s->mst_in_txn) || s->step == ST_WAIT_IDLE;
    }
    return s->bus_active && s->slv_state != SLV_IDLE &&
           s->slv_state != SLV_IGNORE && !s->slv_stretch;
}

/* When 2^@n cycles of I2C_SCLK, less @less, after @from have come. */
static int64_t esp32p4_i2c_pow2_after(ESP32P4I2CState *s, int64_t from,
                                      unsigned n, uint64_t less)
{
    return esp32p4_i2c_after(s, from, (1ull << n) - less);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Arm the timeouts that apply now: SCL unchanged for 2^TIME_OUT_VALUE
 * cycles with TIME_OUT_EN, and a master waiting on the lines for
 * 2^(SCL_ST_TO_I2C + 1) - 1 or 2^(SCL_MAIN_ST_TO_I2C + 1) - 1 cycles.
 */
static void esp32p4_i2c_to_update(ESP32P4I2CState *s)
{
    int64_t now = esp32p4_i2c_now();
    bool cond = esp32p4_i2c_to_cond(s);
    uint32_t to = ACT(s, R_TO);
    int64_t due;

    if (cond && !s->to_cond) {
        s->to_since = now;
    }
    s->to_cond = cond;
    s->to_scl_due = s->to_st_due = s->to_main_due = INT64_MAX;
    if (!esp32p4_i2c_runs(s)) {
        timer_del(&s->to_timer);
        return;
    }
    if (cond && (to & TO_EN)) {
        s->to_scl_due = esp32p4_i2c_pow2_after(
            s, MAX(s->line[LINE_SCL].edge_ns, s->to_since), TO_VALUE(to), 0);
    }
    if (esp32p4_i2c_master(s) && esp32p4_i2c_fsm_waiting(s)) {
        s->to_st_due = esp32p4_i2c_pow2_after(
            s, s->wait_since, ACT(s, R_SCL_ST_TIME_OUT) + 1, 1);
        s->to_main_due = esp32p4_i2c_pow2_after(
            s, s->wait_since, ACT(s, R_SCL_MAIN_ST_TIME_OUT) + 1, 1);
    }
    due = MIN(s->to_scl_due, MIN(s->to_st_due, s->to_main_due));
    if (due == INT64_MAX) {
        timer_del(&s->to_timer);
    } else {
        timer_mod(&s->to_timer, due);
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_to_timer(void *opaque)
{
    ESP32P4I2CState *s = opaque;
    int64_t now = esp32p4_i2c_now();
    int64_t when = INT64_MAX;
    uint32_t bits = 0;

    if (s->to_scl_due <= now) {
        bits |= INT_TIME_OUT;
        when = MIN(when, s->to_scl_due);
    }
    if (s->to_st_due <= now) {
        bits |= INT_SCL_ST_TO;
        when = MIN(when, s->to_st_due);
    }
    if (s->to_main_due <= now) {
        bits |= INT_SCL_MAIN_ST_TO;
        when = MIN(when, s->to_main_due);
    }
    if (!bits) {
        esp32p4_i2c_to_update(s);
        return;
    }
    s->int_raw |= bits;
    if (esp32p4_i2c_master(s)) {
        esp32p4_i2c_mst_abort(s, when);
    } else {
        esp32p4_i2c_slv_reset(s, when);
    }
    s->to_cond = false;
    esp32p4_i2c_update_irq(s);
    esp32p4_i2c_to_update(s);
}

/* Configuration */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * I2C_CONF_UPGATE: the I2C_SCLK domain takes the synchronised fields
 * (TRM table 47.4-1); with I2C_SCLK stopped, once it runs again.
 */
static void esp32p4_i2c_upgate(ESP32P4I2CState *s)
{
    bool lp = esp32p4_i2c_is_lp(s);
    bool was_master = esp32p4_i2c_master(s);
    int64_t now = esp32p4_i2c_now();
    unsigned i;

    if (!esp32p4_i2c_runs(s)) {
        s->upgate_pending = true;
        return;
    }
    s->upgate_pending = false;
    for (i = 0; i < ESP32P4_I2C_NREGS; i++) {
        uint32_t sync = lp ? esp32p4_i2c_regs[i].sync_lp
                           : esp32p4_i2c_regs[i].sync_hp;

        s->act[i] = (s->act[i] & ~sync) | (s->regs[i] & sync);
    }
    if (esp32p4_i2c_ctr(s) & CTR_SAMPLE_SCL_LEVEL) {
        qemu_log_mask(LOG_UNIMP, "%s: SAMPLE_SCL_LEVEL (sampling SDA in "
                      "SCL's low phase) is not modelled\n",
                      esp32p4_i2c_name(s));
    }
    if (was_master && !esp32p4_i2c_master(s)) {
        esp32p4_i2c_mst_abort(s, now);
        s->slv_state = SLV_IDLE;
    } else if (!was_master && esp32p4_i2c_master(s)) {
        esp32p4_i2c_slv_reset(s, now);
    }
    esp32p4_i2c_drive(s, s->scl_drv, s->sda_drv, now);
    esp32p4_i2c_mst_clear_bus(s);
    esp32p4_i2c_to_update(s);
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_fsm_rst(ESP32P4I2CState *s)
{
    int64_t now = esp32p4_i2c_now();

    if (esp32p4_i2c_master(s)) {
        esp32p4_i2c_mst_abort(s, now);
    } else {
        esp32p4_i2c_slv_reset(s, now);
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_trans_start(ESP32P4I2CState *s)
{
    if (!esp32p4_i2c_master(s)) {
        s->slv_armed = true;
        return;
    }
    if (!esp32p4_i2c_runs(s)) {
        s->start_pending = true;
        return;
    }
    esp32p4_i2c_mst_trans_start(s);
}

static void esp32p4_i2c_clock_update(void *opaque, ClockEvent event)
{
    ESP32P4I2CState *s = opaque;

    esp32p4_i2c_fsm_rescale(s);
    if (esp32p4_i2c_runs(s)) {
        if (s->upgate_pending) {
            esp32p4_i2c_upgate(s);
        }
        if (s->start_pending) {
            s->start_pending = false;
            esp32p4_i2c_trans_start(s);
        }
    }
    esp32p4_i2c_to_update(s);
}

/* Registers */

static uint32_t esp32p4_i2c_sr(ESP32P4I2CState *s)
{
    bool lp = esp32p4_i2c_is_lp(s);
    bool busy = s->bus_active || s->mst_in_txn || s->mst_busy;
    uint32_t v = 0;

    v = deposit32(v, 0, 1, s->resp_rec);
    v = deposit32(v, 3, 1, s->arb_lost);
    v = deposit32(v, 4, 1, busy);
    v = deposit32(v, 8, lp ? 5 : 6, s->rx_cnt);
    v = deposit32(v, 18, lp ? 5 : 6, s->tx_cnt);
    v = deposit32(v, 24, 3, s->main_state);
    v = deposit32(v, 28, 3, s->scl_state);
    if (!lp) {
        v = deposit32(v, 1, 1, s->slave_rw);
        v = deposit32(v, 5, 1, s->slave_addressed);
        v = deposit32(v, 14, 2, s->stretch_cause);
    }
    return v;
}

static uint32_t esp32p4_i2c_fifo_st(ESP32P4I2CState *s)
{
    unsigned w = esp32p4_i2c_is_lp(s) ? 4 : 5;
    uint32_t v = 0;

    v = deposit32(v, 0, w, s->rx_rd);
    v = deposit32(v, 5, w, s->rx_wr);
    v = deposit32(v, 10, w, s->tx_rd);
    v = deposit32(v, 15, w, s->tx_wr);
    if (!esp32p4_i2c_is_lp(s)) {
        v = deposit32(v, 22, 8, s->slave_rw_point);
    }
    return v;
}

static bool esp32p4_i2c_reg_exists(ESP32P4I2CState *s, unsigned idx)
{
    switch (idx) {
    case R_SR:
    case R_FIFO_ST:
    case R_DATA:
    case R_INT_RAW:
    case R_INT_CLR:
    case R_INT_STATUS:
        return true;
    default:
        return esp32p4_i2c_is_lp(s) ? esp32p4_i2c_regs[idx].rw_lp
                                    : esp32p4_i2c_regs[idx].rw_hp;
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static uint32_t esp32p4_i2c_read32(ESP32P4I2CState *s, hwaddr a)
{
    unsigned depth = esp32p4_i2c_depth(s);
    unsigned idx = a / 4;
    uint8_t b;

    if (a >= A_TXRAM && a < A_TXRAM + depth * 4) {
        return s->txram[(a - A_TXRAM) / 4];
    }
    if (a >= A_RXRAM && a < A_RXRAM + depth * 4) {
        return s->rxram[(a - A_RXRAM) / 4];
    }
    if (a >= 0x100 || !esp32p4_i2c_reg_exists(s, idx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved address "
                      "0x%" HWADDR_PRIx "\n", esp32p4_i2c_name(s), a);
        return 0;
    }
    switch (idx) {
    case R_SR:
        return esp32p4_i2c_sr(s);
    case R_FIFO_ST:
        return esp32p4_i2c_fifo_st(s);
    case R_DATA:
        b = s->rxram[s->rx_rd];
        if (!esp32p4_i2c_fifo_mode(s)) {
            return b;
        }
        if (!s->rx_cnt) {
            if (esp32p4_i2c_prt(s)) {
                s->int_raw |= INT_RXFIFO_UDF;
                esp32p4_i2c_update_irq(s);
            }
            return b;
        }
        s->rx_cnt--;
        s->rx_rd = (s->rx_rd + 1) % depth;
        esp32p4_i2c_update_irq(s);
        return b;
    case R_INT_RAW:
        return s->int_raw;
    case R_INT_CLR:
        return 0;
    case R_INT_STATUS:
        return s->int_raw & s->regs[R_INT_ENA];
    default:
        return s->regs[idx];
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_write32(ESP32P4I2CState *s, hwaddr a, uint32_t v,
                                uint32_t mask)
{
    bool lp = esp32p4_i2c_is_lp(s);
    unsigned depth = esp32p4_i2c_depth(s);
    unsigned idx = a / 4;
    uint32_t rw, act;

    if (a >= A_TXRAM && a < A_TXRAM + depth * 4) {
        if (esp32p4_i2c_fifo_mode(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TX RAM written directly in "
                          "FIFO mode\n", esp32p4_i2c_name(s));
            return;
        }
        if (mask & 0xff) {
            s->txram[(a - A_TXRAM) / 4] = v;
        }
        return;
    }
    if (a >= 0x100 || !esp32p4_i2c_reg_exists(s, idx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to %s address "
                      "0x%" HWADDR_PRIx "\n", esp32p4_i2c_name(s),
                      (a >= A_RXRAM && a < A_RXRAM + depth * 4) ?
                      "read-only" : "reserved", a);
        return;
    }
    rw = lp ? esp32p4_i2c_regs[idx].rw_lp : esp32p4_i2c_regs[idx].rw_hp;
    switch (idx) {
    case R_SR:
    case R_FIFO_ST:
    case R_INT_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                      "0x%" HWADDR_PRIx "\n", esp32p4_i2c_name(s), a);
        return;
    case R_DATA:
        if (!(mask & 0xff)) {
            return;
        }
        if (!esp32p4_i2c_fifo_mode(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DATA written in non-FIFO "
                          "mode\n", esp32p4_i2c_name(s));
            return;
        }
        if (s->tx_cnt >= depth) {
            if (esp32p4_i2c_prt(s)) {
                s->int_raw |= INT_TXFIFO_OVF;
            }
        } else {
            s->txram[s->tx_wr] = v;
            s->tx_wr = (s->tx_wr + 1) % depth;
            s->tx_cnt++;
        }
        break;
    case R_INT_RAW:
    case R_INT_CLR:
        s->int_raw &= ~(v & mask);
        break;
    case R_CTR:
        act = v & mask & (CTR_TRANS_START | CTR_FSM_RST | CTR_CONF_UPGATE);
        s->regs[idx] = (s->regs[idx] & ~(mask & rw)) | (v & mask & rw);
        if (act & CTR_CONF_UPGATE) {
            esp32p4_i2c_upgate(s);
        }
        if (act & CTR_FSM_RST) {
            esp32p4_i2c_fsm_rst(s);
        }
        if (act & CTR_TRANS_START) {
            esp32p4_i2c_trans_start(s);
        }
        break;
    case R_FIFO_CONF:
        s->regs[idx] = (s->regs[idx] & ~(mask & rw)) | (v & mask & rw);
        if (s->regs[idx] & FIFO_RX_RST) {
            esp32p4_i2c_rx_reset(s);
        }
        if (s->regs[idx] & FIFO_TX_RST) {
            esp32p4_i2c_tx_reset(s);
        }
        break;
    case R_SCL_STRETCH_CONF:
        s->regs[idx] = (s->regs[idx] & ~(mask & rw)) | (v & mask & rw);
        if (v & mask & STRETCH_CLR) {
            esp32p4_i2c_slv_stretch_clr(s);
        }
        break;
    case R_CLK_CONF:
        s->regs[idx] = (s->regs[idx] & ~(mask & rw)) | (v & mask & rw);
        esp32p4_i2c_clock_update(s, ClockUpdate);
        break;
    default:
        s->regs[idx] = (s->regs[idx] & ~(mask & rw)) | (v & mask & rw);
        break;
    }
    esp32p4_i2c_update_irq(s);
}

static uint64_t esp32p4_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4I2CState *s = opaque;

    return extract32(esp32p4_i2c_read32(s, addr & ~3), (addr & 3) * 8,
                     size * 8);
}

static void esp32p4_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    ESP32P4I2CState *s = opaque;
    unsigned shift = (addr & 3) * 8;

    esp32p4_i2c_write32(s, addr & ~3, (uint32_t)value << shift,
                        MAKE_64BIT_MASK(shift, size * 8));
}

static const MemoryRegionOps esp32p4_i2c_ops = {
    .read = esp32p4_i2c_read,
    .write = esp32p4_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* Device */

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i2c_reset_hold(Object *obj, ResetType type)
{
    ESP32P4I2CState *s = ESP32P4_I2C(obj);
    bool lp = esp32p4_i2c_is_lp(s);
    unsigned i;

    timer_del(&s->fsm_timer);
    timer_del(&s->slv_timer);
    timer_del(&s->to_timer);
    for (i = 0; i < 2; i++) {
        timer_del(&s->flt_timer[i]);
        s->line[i].filt = s->line[i].raw;
        s->line[i].pend_ns = INT64_MAX;
        s->drv_ns[i] = INT64_MIN;
    }
    if (s->dev_active) {
        i2c_end_transfer(s->bus);
    }
    for (i = 0; i < ESP32P4_I2C_NREGS; i++) {
        s->regs[i] = lp ? esp32p4_i2c_regs[i].reset_lp
                        : esp32p4_i2c_regs[i].reset_hp;
        s->act[i] = s->regs[i];
    }
    s->upgate_pending = false;
    memset(s->txram, 0, sizeof(s->txram));
    memset(s->rxram, 0, sizeof(s->rxram));
    esp32p4_i2c_tx_reset(s);
    esp32p4_i2c_rx_reset(s);
    s->int_raw = 0;
    s->resp_rec = false;
    s->slave_rw = false;
    s->arb_lost = false;
    s->slave_addressed = false;
    s->stretch_cause = STRETCH_SENDING_ACK;
    s->main_state = MAIN_IDLE;
    s->scl_state = SCLST_IDLE;
    s->slave_rw_point = 0;
    s->bus_active = false;
    s->step = ST_NONE;
    s->step_left = 0;
    s->tp_edges = esp32p4_clk_edges_stopped();
    s->tp_edges = esp32p4_i2c_edges(s, esp32p4_i2c_now());
    s->tp_sclk = esp32p4_clk_edges_or_at(s->sclk, esp32p4_i2c_now());
    s->mst_busy = false;
    s->mst_suspended = false;
    s->mst_in_txn = false;
    s->cmd_idx = 0;
    s->nack_stop = false;
    s->connected = false;
    s->dev_active = false;
    s->start_pending = false;
    s->rst_pulses = 0;
    s->slv_state = SLV_IDLE;
    s->slv_matched = false;
    s->slv_10bit = false;
    s->slv_armed = false;
    s->slv_stretch = false;
    s->slv_stretch_req = 0;
    s->slv_sda_due = INT64_MAX;
    s->slv_scl_due = INT64_MAX;
    s->to_cond = false;
    s->to_scl_due = s->to_st_due = s->to_main_due = INT64_MAX;
    s->scl_drv = true;
    s->sda_drv = true;
}

/* Both lines released, and the TX watermark interrupt's reset level. */
static void esp32p4_i2c_reset_exit(Object *obj, ResetType type)
{
    ESP32P4I2CState *s = ESP32P4_I2C(obj);

    esp32p4_i2c_drive(s, true, true, esp32p4_i2c_now());
    esp32p4_i2c_update_irq(s);
}

static void esp32p4_i2c_init(Object *obj)
{
    ESP32P4I2CState *s = ESP32P4_I2C(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_i2c_ops, s,
                          "esp32p4.i2c", ESP32P4_I2C_BLOCK_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, &s->scl_out, ESP32P4_I2C_SCL_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->scl_oe, ESP32P4_I2C_SCL_OE, 1);
    qdev_init_gpio_out_named(dev, &s->sda_out, ESP32P4_I2C_SDA_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->sda_oe, ESP32P4_I2C_SDA_OE, 1);
    qdev_init_gpio_in_named(dev, esp32p4_i2c_scl_in, ESP32P4_I2C_SCL_IN, 1);
    qdev_init_gpio_in_named(dev, esp32p4_i2c_sda_in, ESP32P4_I2C_SDA_IN, 1);
    s->sclk = qdev_init_clock_in(dev, "sclk", esp32p4_i2c_clock_update, s,
                                 ClockUpdate);
    timer_init_ns(&s->fsm_timer, QEMU_CLOCK_VIRTUAL, esp32p4_i2c_fsm_timer,
                  s);
    timer_init_ns(&s->slv_timer, QEMU_CLOCK_VIRTUAL, esp32p4_i2c_slv_timer,
                  s);
    timer_init_ns(&s->to_timer, QEMU_CLOCK_VIRTUAL, esp32p4_i2c_to_timer, s);
    timer_init_ns(&s->flt_timer[LINE_SCL], QEMU_CLOCK_VIRTUAL,
                  esp32p4_i2c_flt_scl, s);
    timer_init_ns(&s->flt_timer[LINE_SDA], QEMU_CLOCK_VIRTUAL,
                  esp32p4_i2c_flt_sda, s);
    /* An input the matrix does not route reads high (table 10.12-1). */
    s->line[LINE_SCL].raw = s->line[LINE_SCL].filt = true;
    s->line[LINE_SDA].raw = s->line[LINE_SDA].filt = true;
}

static void esp32p4_i2c_realize(DeviceState *dev, Error **errp)
{
    ESP32P4I2CState *s = ESP32P4_I2C(dev);

    s->bus = i2c_init_bus(dev, s->bus_name ? s->bus_name : "i2c");
}

static const VMStateDescription vmstate_esp32p4_i2c_line = {
    .name = "esp32p4-i2c-line",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(raw, ESP32P4I2CLine),
        VMSTATE_BOOL(filt, ESP32P4I2CLine),
        VMSTATE_INT64(edge_ns, ESP32P4I2CLine),
        VMSTATE_INT64(edge_tp.origin, ESP32P4I2CLine),
        VMSTATE_UINT64(edge_tp.cyc, ESP32P4I2CLine),
        VMSTATE_INT64(pend_ns, ESP32P4I2CLine),
        VMSTATE_INT64(pend_tp.origin, ESP32P4I2CLine),
        VMSTATE_UINT64(pend_tp.cyc, ESP32P4I2CLine),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_i2c = {
    .name = TYPE_ESP32P4_I2C,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(sclk, ESP32P4I2CState),
        VMSTATE_TIMER(fsm_timer, ESP32P4I2CState),
        VMSTATE_TIMER(slv_timer, ESP32P4I2CState),
        VMSTATE_TIMER(to_timer, ESP32P4I2CState),
        VMSTATE_TIMER_ARRAY(flt_timer, ESP32P4I2CState, 2),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4I2CState, ESP32P4_I2C_NREGS),
        VMSTATE_UINT32_ARRAY(act, ESP32P4I2CState, ESP32P4_I2C_NREGS),
        VMSTATE_BOOL(upgate_pending, ESP32P4I2CState),
        VMSTATE_UINT8_ARRAY(txram, ESP32P4I2CState, ESP32P4_I2C_RAM_MAX),
        VMSTATE_UINT8_ARRAY(rxram, ESP32P4I2CState, ESP32P4_I2C_RAM_MAX),
        VMSTATE_UINT8(tx_rd, ESP32P4I2CState),
        VMSTATE_UINT8(tx_wr, ESP32P4I2CState),
        VMSTATE_UINT8(tx_cnt, ESP32P4I2CState),
        VMSTATE_UINT8(rx_rd, ESP32P4I2CState),
        VMSTATE_UINT8(rx_wr, ESP32P4I2CState),
        VMSTATE_UINT8(rx_cnt, ESP32P4I2CState),
        VMSTATE_UINT32(int_raw, ESP32P4I2CState),
        VMSTATE_BOOL(irq_level, ESP32P4I2CState),
        VMSTATE_BOOL(resp_rec, ESP32P4I2CState),
        VMSTATE_BOOL(slave_rw, ESP32P4I2CState),
        VMSTATE_BOOL(arb_lost, ESP32P4I2CState),
        VMSTATE_BOOL(slave_addressed, ESP32P4I2CState),
        VMSTATE_UINT8(stretch_cause, ESP32P4I2CState),
        VMSTATE_UINT8(main_state, ESP32P4I2CState),
        VMSTATE_UINT8(scl_state, ESP32P4I2CState),
        VMSTATE_UINT8(slave_rw_point, ESP32P4I2CState),
        VMSTATE_STRUCT_ARRAY(line, ESP32P4I2CState, 2, 1,
                             vmstate_esp32p4_i2c_line, ESP32P4I2CLine),
        VMSTATE_BOOL(scl_drv, ESP32P4I2CState),
        VMSTATE_BOOL(sda_drv, ESP32P4I2CState),
        VMSTATE_BOOL(bus_active, ESP32P4I2CState),
        VMSTATE_UINT32(step, ESP32P4I2CState),
        VMSTATE_UINT64(step_left, ESP32P4I2CState),
        VMSTATE_INT64(step_mark, ESP32P4I2CState),
        VMSTATE_ESP32P4_CLK_EDGES(tp_edges, ESP32P4I2CState),
        VMSTATE_ESP32P4_CLK_EDGES(tp_sclk, ESP32P4I2CState),
        VMSTATE_INT64(step_tp.origin, ESP32P4I2CState),
        VMSTATE_UINT64(step_tp.cyc, ESP32P4I2CState),
        VMSTATE_INT64(step_ns, ESP32P4I2CState),
        VMSTATE_INT64(cur_tp.origin, ESP32P4I2CState),
        VMSTATE_UINT64(cur_tp.cyc, ESP32P4I2CState),
        VMSTATE_INT64_ARRAY(drv_ns, ESP32P4I2CState, 2),
        VMSTATE_INT64(drv_tp[0].origin, ESP32P4I2CState),
        VMSTATE_UINT64(drv_tp[0].cyc, ESP32P4I2CState),
        VMSTATE_INT64(drv_tp[1].origin, ESP32P4I2CState),
        VMSTATE_UINT64(drv_tp[1].cyc, ESP32P4I2CState),
        VMSTATE_BOOL(mst_busy, ESP32P4I2CState),
        VMSTATE_BOOL(mst_suspended, ESP32P4I2CState),
        VMSTATE_BOOL(mst_in_txn, ESP32P4I2CState),
        VMSTATE_UINT8(cmd_idx, ESP32P4I2CState),
        VMSTATE_UINT16(byte_left, ESP32P4I2CState),
        VMSTATE_UINT8(bit, ESP32P4I2CState),
        VMSTATE_UINT8(cur_byte, ESP32P4I2CState),
        VMSTATE_BOOL(reading, ESP32P4I2CState),
        VMSTATE_BOOL(bit_out, ESP32P4I2CState),
        VMSTATE_BOOL(mst_ack, ESP32P4I2CState),
        VMSTATE_BOOL(addr_next, ESP32P4I2CState),
        VMSTATE_BOOL(nack_stop, ESP32P4I2CState),
        VMSTATE_BOOL(sample_at_fall, ESP32P4I2CState),
        VMSTATE_BOOL(chk_sda, ESP32P4I2CState),
        VMSTATE_BOOL(chk_scl, ESP32P4I2CState),
        VMSTATE_BOOL(connected_sda, ESP32P4I2CState),
        VMSTATE_BOOL(connected, ESP32P4I2CState),
        VMSTATE_BOOL(dev_active, ESP32P4I2CState),
        VMSTATE_BOOL(dev_read, ESP32P4I2CState),
        VMSTATE_BOOL(dev_nack, ESP32P4I2CState),
        VMSTATE_UINT8(dev_addr, ESP32P4I2CState),
        VMSTATE_UINT8(dev_byte, ESP32P4I2CState),
        VMSTATE_INT64(t_rel, ESP32P4I2CState),
        VMSTATE_INT64(t_fall_due, ESP32P4I2CState),
        VMSTATE_INT64(fall_tp.origin, ESP32P4I2CState),
        VMSTATE_UINT64(fall_tp.cyc, ESP32P4I2CState),
        VMSTATE_UINT8(rst_pulses, ESP32P4I2CState),
        VMSTATE_BOOL(start_pending, ESP32P4I2CState),
        VMSTATE_UINT8(slv_state, ESP32P4I2CState),
        VMSTATE_UINT8(slv_bits, ESP32P4I2CState),
        VMSTATE_UINT8(slv_byte, ESP32P4I2CState),
        VMSTATE_UINT8(slv_next, ESP32P4I2CState),
        VMSTATE_BOOL(slv_ack, ESP32P4I2CState),
        VMSTATE_BOOL(slv_matched, ESP32P4I2CState),
        VMSTATE_BOOL(slv_10bit, ESP32P4I2CState),
        VMSTATE_BOOL(slv_first_data, ESP32P4I2CState),
        VMSTATE_BOOL(slv_armed, ESP32P4I2CState),
        VMSTATE_BOOL(slv_master_nack, ESP32P4I2CState),
        VMSTATE_UINT8(slv_stretch_req, ESP32P4I2CState),
        VMSTATE_BOOL(slv_stretch, ESP32P4I2CState),
        VMSTATE_BOOL(slv_stretch_tx, ESP32P4I2CState),
        VMSTATE_INT64(slv_sda_due, ESP32P4I2CState),
        VMSTATE_BOOL(slv_sda_lvl, ESP32P4I2CState),
        VMSTATE_INT64(slv_scl_due, ESP32P4I2CState),
        VMSTATE_BOOL(slv_scl_lvl, ESP32P4I2CState),
        VMSTATE_BOOL(to_cond, ESP32P4I2CState),
        VMSTATE_INT64(to_since, ESP32P4I2CState),
        VMSTATE_INT64(wait_since, ESP32P4I2CState),
        VMSTATE_INT64(to_scl_due, ESP32P4I2CState),
        VMSTATE_INT64(to_st_due, ESP32P4I2CState),
        VMSTATE_INT64(to_main_due, ESP32P4I2CState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_i2c_properties[] = {
    DEFINE_PROP_STRING("bus-name", ESP32P4I2CState, bus_name),
};

static void esp32p4_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_i2c_reset_hold;
    rc->phases.exit = esp32p4_i2c_reset_exit;
    dc->realize = esp32p4_i2c_realize;
    dc->vmsd = &vmstate_esp32p4_i2c;
    dc->desc = "ESP32-P4 I2C controller";
    device_class_set_props(dc, esp32p4_i2c_properties);
}

static void esp32p4_lp_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    ESP32P4_I2C_CLASS(klass)->lp = true;
    dc->desc = "ESP32-P4 LP I2C controller";
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static const TypeInfo esp32p4_i2c_types[] = {
    {
        .name          = TYPE_ESP32P4_I2C,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4I2CState),
        .instance_init = esp32p4_i2c_init,
        .class_size    = sizeof(ESP32P4I2CClass),
        .class_init    = esp32p4_i2c_class_init,
    },
    {
        .name          = TYPE_ESP32P4_LP_I2C,
        .parent        = TYPE_ESP32P4_I2C,
        .class_init    = esp32p4_lp_i2c_class_init,
    },
};

DEFINE_TYPES(esp32p4_i2c_types)
