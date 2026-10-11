/*
 * ESP32-P4 I3C master and slave controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: the TRM's I3C chapter is "to be added later"; the registers
 * are ESP-IDF's i3c_mst_reg.h, i3c_mst_mem_reg.h and i3c_slv_reg.h for
 * v3, and how software drives them its esp_hal_i3c (i3c_master_ll.h,
 * i3c_slave_ll.h) and esp_driver_i3c. The master is a Synopsys
 * DesignWare MIPI I3C controller behind Espressif's own register map:
 * command, response, data and IBI buffers and the device address and
 * characteristic tables (the DAT at 0xC0, the DCT at 0x100) share its 4
 * KiB with the registers. The slave is a Silvaco I3C target, the subset
 * of its registers the header lists.
 *
 * The master runs the commands software writes to COMMAND_BUF_PORT, two
 * words each, from TRANS_START: regular transfers (private, legacy I2C
 * and CCCs, with their data in the TX and RX buffers), immediate
 * transfers (up to four bytes in the command) and address assignment
 * (ENTDAA and SETDASA, recording each device found in the DCT). Each
 * header and byte takes the time of its bits in I3C_MST_CLK cycles:
 * (OD_LOW + 1) + (OD_HIGH + 1) open drain for the header after a START
 * and the ENTDAA arbitration, (PP_LOW + 1) + (PP_HIGH + 1) push-pull
 * for the rest (SDR1 to SDR4 with the longer low periods of
 * SCL_EXT_LOW_TIME), and the FM or FM+ counts for legacy I2C devices; a
 * START takes START_HOLD + 1 cycles, a repeated START PP_LOW + 1 +
 * RSTART_SETUP + 1 + START_HOLD + 1, a STOP PP_LOW + 1 + STOP_SETUP + 1,
 * and the bus is free STOP_HOLD + 1 or BUS_FREE_TIME cycles after it,
 * whichever is longer. A transfer that needs data the TX buffer lacks,
 * or room the RX buffer lacks, holds SCL low until it has it.
 *
 * Choices where the documentation is silent:
 *
 *   - A command that fails (a NACK, error 4 for the broadcast address,
 *     5 for a device's address, 9 for an I2C device's data) ends with a
 *     STOP, pushes its response whatever ROC says, raises TRANSFER_ERR
 *     and discards the commands after it; the DesignWare core would halt
 *     for software to resume, which this register map has no bit for.
 *   - The threshold interrupts (TX and RX data, IBI status, command
 *     buffer empty, response ready) follow their conditions; the others
 *     are latched until INT_CLR.
 *   - The IBI data buffer holds the payload packed four bytes a word.
 *   - In ENTDAA the address byte goes to the target as QEMU's I3C core
 *     takes it, the 7-bit address itself; the DCT records it as on the
 *     wire, shifted up with its parity bit.
 *
 * With DMA_TX_EN and DMA_RX_EN the data buffers are GDMA-AHB's, as the
 * header describes.
 *
 * Not modelled: HDR modes, the RX bit and byte order bits, and the buffer
 * timeouts of TIME_OUT_VALUE (logged).
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/i3c/esp32p4_i3c.h"
#include "migration/vmstate.h"

/* Master registers, word indices */
enum {
    R_DEVICE_CTRL = 0x00 / 4,
    R_COMMAND_BUF_PORT = 0x08 / 4,
    R_RESPONSE_BUF_PORT = 0x0c / 4,
    R_RX_DATA_PORT = 0x10 / 4,
    R_TX_DATA_PORT = 0x14 / 4,
    R_IBI_STATUS_BUF = 0x18 / 4,
    R_BUFFER_THLD_CTRL = 0x1c / 4,
    R_DATA_BUFFER_THLD_CTRL = 0x20 / 4,
    R_IBI_NOTIFY_CTRL = 0x24 / 4,
    R_IBI_SIR_REQ_PAYLOAD = 0x28 / 4,
    R_IBI_SIR_REQ_REJECT = 0x2c / 4,
    R_INT_CLR = 0x30 / 4,
    R_INT_RAW = 0x34 / 4,
    R_INT_ST = 0x38 / 4,
    R_INT_ST_ENA = 0x3c / 4,
    R_IBI_DATA_BUF = 0x40 / 4,
    R_RESET_CTRL = 0x44 / 4,
    R_BUFFER_STATUS_LEVEL = 0x48 / 4,
    R_DATA_BUFFER_STATUS_LEVEL = 0x4c / 4,
    R_PRESENT_STATE0 = 0x50 / 4,
    R_PRESENT_STATE1 = 0x54 / 4,
    R_DEVICE_TABLE = 0x58 / 4,
    R_TIME_OUT_VALUE = 0x5c / 4,
    R_OD_TIME = 0x60 / 4,
    R_PP_TIME = 0x64 / 4,
    R_FM_TIME = 0x68 / 4,
    R_FMP_TIME = 0x6c / 4,
    R_EXT_LOW_TIME = 0x70 / 4,
    R_SDA_SAMPLE_TIME = 0x74 / 4,
    R_SDA_HOLD_TIME = 0x78 / 4,
    R_SCL_START_HOLD = 0x7c / 4,
    R_SCL_RSTART_SETUP = 0x80 / 4,
    R_SCL_STOP_HOLD = 0x84 / 4,
    R_SCL_STOP_SETUP = 0x88 / 4,
    R_BUS_FREE_TIME = 0x90 / 4,
    R_TERMN_T_EXT_LOW_TIME = 0x94 / 4,
    R_VER_ID = 0xa0 / 4,
    R_VER_TYPE = 0xa4 / 4,
    R_FPGA_DEBUG_PROBE = 0xac / 4,
    R_RND_ECO_CS = 0xb0 / 4,
    R_RND_ECO_LOW = 0xb4 / 4,
    R_RND_ECO_HIGH = 0xb8 / 4,
};

#define A_DAT           0xc0
#define A_DCT           0x100

#define CTRL_BA_INCLUDE         BIT(1)
#define CTRL_TRANS_START        BIT(2)
#define CTRL_AUTO_DIS_IBI       BIT(5)
#define CTRL_DMA_RX_EN          BIT(6)
#define CTRL_DMA_TX_EN          BIT(7)
#define CTRL_MULTI_SLV_CCC      BIT(8)
#define CTRL_RX_BIT_ORDER       BIT(9)
#define CTRL_RX_BYTE_ORDER      BIT(10)
#define CTRL_SCL_PULLUP_FORCE   BIT(11)
#define CTRL_SCL_OE_FORCE       BIT(12)
#define CTRL_SDA_PP_RD_PULLUP   BIT(13)
#define CTRL_SDA_PP_WR_PULLUP   BIT(15)
#define CTRL_BYTE_CNT_UNLATCH   BIT(16)

/* IBI_NOTIFY_CTRL */
#define NOTIFY_SIR_REJECTED     BIT(2)

/* Interrupts */
#define INT_TX_DATA_BUF_THLD    BIT(0)
#define INT_RX_DATA_BUF_THLD    BIT(1)
#define INT_IBI_STATUS_THLD     BIT(2)
#define INT_CMD_BUF_EMPTY_THLD  BIT(3)
#define INT_RESP_READY          BIT(4)
#define INT_NXT_CMD_REQ_ERR     BIT(5)
#define INT_TRANSFER_ERR        BIT(6)
#define INT_TRANSFER_COMPLETE   BIT(7)
#define INT_COMMAND_DONE        BIT(8)
#define INT_DETECT_START        BIT(9)
#define INT_RESP_BUF_OVF        BIT(10)
#define INT_IBI_DATA_BUF_OVF    BIT(11)
#define INT_IBI_STATUS_BUF_OVF  BIT(12)
#define INT_IBI_HANDLE_DONE     BIT(13)
#define INT_IBI_DETECT          BIT(14)
#define INT_CMD_CCC_MISMATCH    BIT(15)
#define INT_MASK                0xffff
#define INT_LEVELS              (INT_TX_DATA_BUF_THLD | \
                                 INT_RX_DATA_BUF_THLD | \
                                 INT_IBI_STATUS_THLD | \
                                 INT_CMD_BUF_EMPTY_THLD | INT_RESP_READY)

/* RESET_CTRL */
#define RST_CORE_SOFT           BIT(0)
#define RST_CMD_BUF             BIT(1)
#define RST_RESP_BUF            BIT(2)
#define RST_TX_DATA_BUF         BIT(3)
#define RST_RX_DATA_BUF         BIT(4)
#define RST_IBI_DATA_BUF        BIT(5)
#define RST_IBI_STATUS_BUF      BIT(6)

/* Command descriptors (i3c_master_ll.h) */
#define CMD_ATTR(l)             extract32((l), 0, 3)
#define CMD_TID(l)              extract32((l), 3, 4)
#define CMD_CCC(l)              extract32((l), 7, 8)
#define CMD_CP(l)               extract32((l), 15, 1)
#define CMD_DEV_INDX(l)         extract32((l), 16, 5)
#define CMD_DEV_CNT(l)          extract32((l), 26, 4)
#define CMD_BC(l)               extract32((l), 23, 3)
#define CMD_MODE(l)             extract32((l), 26, 3)
#define CMD_RNW(l)              extract32((l), 29, 1)
#define CMD_ROC(l)              extract32((l), 30, 1)
#define CMD_TOC(l)              extract32((l), 31, 1)
#define CMD_DL(h)               extract32((h), 16, 16)

enum {
    ATTR_REGULAR,
    ATTR_IMMEDIATE,
    ATTR_ADDR_ASSIGN,
};

/* Device address table entries */
#define DAT_STATIC(v)           extract32((v), 0, 7)
#define DAT_DYNAMIC(v)          extract32((v), 16, 7)
#define DAT_PARITY(v)           extract32((v), 23, 1)
#define DAT_DNRC(v)             extract32((v), 29, 2)
#define DAT_I2C(v)              extract32((v), 31, 1)

/* Response errors */
enum {
    ERR_NONE = 0,
    ERR_BROADCAST_NACK = 4,
    ERR_ADDR_NACK = 5,
    ERR_I2C_DATA_NACK = 9,
};

#define CCC_ENTDAA              0x07
#define CCC_SETDASA             0x87
#define CCC_DISEC_DIRECT        0x81
#define DISEC_INT               0x01

/* The master's phases, each ended by the timer */
enum {
    PH_NONE,
    PH_START,
    PH_RS_SCL,
    PH_RS_SDA,
    PH_RS_END,
    PH_HDR_BCAST,
    PH_CCC,
    PH_HDR_DIRECT,
    PH_DATA,
    PH_DAA_HDR,
    PH_DAA_ID,
    PH_DAA_ADDR,
    PH_DASA_HDR,
    PH_DASA_ADDR,
    PH_STOP_SCL,
    PH_STOP_SDA,
    PH_IBI,
    /* Waits: for TX data, or RX room, while SCL is held low */
    PH_WAIT_TX,
    PH_WAIT_RX,
};

/* What follows a repeated START */
enum {
    NEXT_CMD,
    NEXT_DIRECT,
    NEXT_DAA,
    NEXT_DASA,
};

/* Bit timings */
enum {
    BIT_OD,
    BIT_PP,
    BIT_I2C,
};

static const struct {
    uint32_t reset;
    uint32_t rw;
} esp32p4_i3c_mst_regs[ESP32P4_I3C_MST_NREGS] = {
    [R_DEVICE_CTRL] = { 0x1020, 0x3fffe },
    [R_BUFFER_THLD_CTRL] = { 0x41041, 0x1c71cf },
    [R_DATA_BUFFER_THLD_CTRL] = { 0x9, 0x3f },
    [R_IBI_NOTIFY_CTRL] = { 0, NOTIFY_SIR_REJECTED },
    [R_IBI_SIR_REQ_PAYLOAD] = { 0, 0xffffffff },
    [R_IBI_SIR_REQ_REJECT] = { 0, 0xffffffff },
    [R_INT_ST_ENA] = { 0, INT_MASK },
    [R_RESET_CTRL] = { 0, 0x7e },
    [R_DEVICE_TABLE] = { 0, 0xff },
    [R_TIME_OUT_VALUE] = { 0x410410, 0xffffff },
    [R_OD_TIME] = { 0x00050019, 0xffffffff },
    [R_PP_TIME] = { 0x00050005, 0x00ff00ff },
    [R_FM_TIME] = { 0x004b00a3, 0xffffffff },
    [R_FMP_TIME] = { 0x0021003f, 0x00ffffff },
    [R_EXT_LOW_TIME] = { 0, 0xffffffff },
    [R_SDA_SAMPLE_TIME] = { 0, 0x3fff },
    [R_SDA_HOLD_TIME] = { 1, 0x3fff },
    [R_SCL_START_HOLD] = { 8, 0x7ff },
    [R_SCL_RSTART_SETUP] = { 8, 0x1ff },
    [R_SCL_STOP_HOLD] = { 8, 0x1ff },
    [R_SCL_STOP_SETUP] = { 8, 0x1ff },
    [R_BUS_FREE_TIME] = { 5, 0xffff },
    [R_TERMN_T_EXT_LOW_TIME] = { 2, 0xff },
    [R_VER_ID] = { 0x20230504, 0xffffffff },
    [R_VER_TYPE] = { 0, 0xffffffff },
    [R_FPGA_DEBUG_PROBE] = { 1, 0xffffffff },
    [R_RND_ECO_CS] = { 0, 0x1 },
    [R_RND_ECO_LOW] = { 0, 0xffffffff },
    [R_RND_ECO_HIGH] = { 0xffff, 0xffffffff },
};

static void esp32p4_i3c_mst_next(ESP32P4I3CMstState *s);
static void esp32p4_i3c_mst_phase_end(ESP32P4I3CMstState *s, int64_t t);

static int64_t esp32p4_i3c_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* @n cycles of @period (Clock units) in ns */
static int64_t esp32p4_i3c_cyc(uint64_t period, uint64_t n)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, n, period);
    lo = (lo >> 32) | (hi << 32);
    return (hi >> 32 || lo > INT64_MAX / 2) ? INT64_MAX / 2 : lo;
}

/* Master: interrupts and buffers */

static unsigned esp32p4_i3c_data_thld(unsigned v)
{
    static const uint8_t thld[] = { 2, 4, 8, 16, 31, 31, 31, 31 };

    return thld[v & 7];
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The threshold interrupts follow the buffers' levels; the others are
 * latched in int_raw.
 */
static uint32_t esp32p4_i3c_mst_raw(ESP32P4I3CMstState *s)
{
    uint32_t thld = s->regs[R_BUFFER_THLD_CTRL];
    uint32_t dthld = s->regs[R_DATA_BUFFER_THLD_CTRL];
    uint32_t raw = s->int_raw & ~INT_LEVELS;

    if (fifo32_num_free(&s->tx) >= esp32p4_i3c_data_thld(dthld)) {
        raw |= INT_TX_DATA_BUF_THLD;
    }
    if (fifo32_num_used(&s->rx) >=
        esp32p4_i3c_data_thld(extract32(dthld, 3, 3))) {
        raw |= INT_RX_DATA_BUF_THLD;
    }
    if (fifo32_num_used(&s->ibi_status) >=
        MAX(extract32(thld, 18, 3), 1)) {
        raw |= INT_IBI_STATUS_THLD;
    }
    if (fifo32_num_free(&s->cmd) >= MAX(extract32(thld, 0, 4), 1)) {
        raw |= INT_CMD_BUF_EMPTY_THLD;
    }
    if (fifo32_num_used(&s->resp) >= MAX(extract32(thld, 6, 3), 1)) {
        raw |= INT_RESP_READY;
    }
    return raw;
}

static void esp32p4_i3c_mst_update_irq(ESP32P4I3CMstState *s)
{
    bool level = esp32p4_i3c_mst_raw(s) & s->regs[R_INT_ST_ENA];

    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->irq, level);
    }
}

static void esp32p4_i3c_mst_raise(ESP32P4I3CMstState *s, uint32_t bits)
{
    s->int_raw |= bits;
    esp32p4_i3c_mst_update_irq(s);
}

/* Master: lines */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Drive SCL and SDA (1 releases a line): SCL push-pull while
 * SCL_OE_FORCE_EN is set, SDA open drain, so that a target can pull it
 * for an IBI.
 */
static void esp32p4_i3c_mst_drive(ESP32P4I3CMstState *s, bool scl, bool sda,
                                  int64_t when)
{
    bool scl_force = s->regs[R_DEVICE_CTRL] & CTRL_SCL_OE_FORCE;

    s->scl_drv = scl;
    s->sda_drv = sda;
    esp32_line_set_at(s->scl_out, scl, when);
    esp32_line_set_at(s->scl_oe, scl_force || !scl, when);
    esp32_line_set_at(s->sda_out, sda, when);
    esp32_line_set_at(s->sda_oe, !sda, when);
    esp32_line_set_at(s->scl_pullup,
                      !!(s->regs[R_DEVICE_CTRL] & CTRL_SCL_PULLUP_FORCE),
                      when);
    esp32_line_set_at(s->sda_pullup, !s->in_txn, when);
}

static void esp32p4_i3c_mst_scl_in(void *opaque, int n, int level)
{
    ESP32P4I3CMstState *s = opaque;

    s->scl_in = level;
}

/* SDA falling while SCL is high: a START, the master's or a target's. */
static void esp32p4_i3c_mst_sda_in(void *opaque, int n, int level)
{
    ESP32P4I3CMstState *s = opaque;

    if (s->sda_in && !level && s->scl_in && clock_get(s->clk)) {
        esp32p4_i3c_mst_raise(s, INT_DETECT_START);
    }
    s->sda_in = level;
}

/* Master: timing */

static uint64_t esp32p4_i3c_mst_bit(ESP32P4I3CMstState *s, int kind)
{
    uint32_t mode = CMD_MODE(s->cmd_l);
    uint32_t v, low, high;

    switch (kind) {
    case BIT_OD:
        v = s->regs[R_OD_TIME];
        return (extract32(v, 0, 16) + 1) + (extract32(v, 16, 16) + 1);
    case BIT_I2C:
        v = s->regs[mode == 1 ? R_FMP_TIME : R_FM_TIME];
        return (extract32(v, 0, 16) + 1) +
               (extract32(v, 16, mode == 1 ? 8 : 16) + 1);
    default:
        v = s->regs[R_PP_TIME];
        low = extract32(v, 0, 8);
        high = extract32(v, 16, 8);
        if (CMD_ATTR(s->cmd_l) != ATTR_ADDR_ASSIGN && mode >= 1 &&
            mode <= 4) {
            low = MAX(low, extract32(s->regs[R_EXT_LOW_TIME],
                                     8 * (mode - 1), 8));
        }
        return (low + 1) + (high + 1);
    }
}

/* Bits of a byte, header or data, to or from the addressed device */
static int esp32p4_i3c_mst_data_kind(ESP32P4I3CMstState *s)
{
    return s->dev_i2c ? BIT_I2C : BIT_PP;
}

/*
 * End the current phase @cycles of I3C_MST_CLK after @from (a time),
 * moving to @phase; with the clock stopped the phase waits for it.
 */
static void esp32p4_i3c_mst_at(ESP32P4I3CMstState *s, int64_t from,
                               uint64_t cycles, uint32_t phase)
{
    uint64_t period = clock_get(s->clk);

    s->phase = phase;
    s->phase_period = period;
    if (!period) {
        s->phase_left = cycles;
        timer_del(&s->timer);
        return;
    }
    s->phase_left = 0;
    s->phase_due = from + esp32p4_i3c_cyc(period, cycles);
    timer_mod(&s->timer, s->phase_due);
}

static void esp32p4_i3c_mst_after(ESP32P4I3CMstState *s, uint64_t cycles,
                                  uint32_t phase)
{
    esp32p4_i3c_mst_at(s, s->phase_due, cycles, phase);
}

static void esp32p4_i3c_mst_timer(void *opaque)
{
    ESP32P4I3CMstState *s = opaque;

    if (s->phase == PH_NONE || s->phase >= PH_WAIT_TX || !s->phase_period) {
        return;
    }
    esp32p4_i3c_mst_phase_end(s, s->phase_due);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * I3C_MST_CLK changing rescales what is left of the phase; stopped, it
 * holds it.
 */
static void esp32p4_i3c_mst_clock_update(void *opaque, ClockEvent event)
{
    ESP32P4I3CMstState *s = opaque;
    uint64_t period = clock_get(s->clk);
    int64_t now = esp32p4_i3c_now();
    uint64_t lo, hi;

    if (s->phase == PH_NONE || s->phase >= PH_WAIT_TX) {
        return;
    }
    if (s->phase_period) {
        s->phase_left = 0;
        if (s->phase_due > now) {
            mulu64(&lo, &hi, s->phase_due - now, 1ull << 32);
            divu128(&lo, &hi, s->phase_period);
            s->phase_left = hi ? UINT64_MAX : lo;
        }
    }
    s->phase_period = period;
    if (!period) {
        timer_del(&s->timer);
        return;
    }
    s->phase_due = now + esp32p4_i3c_cyc(period, s->phase_left);
    s->phase_left = 0;
    timer_mod(&s->timer, s->phase_due);
}

/* Master: the bus */

static uint8_t esp32p4_i3c_mst_dev_addr(ESP32P4I3CMstState *s, unsigned dev)
{
    uint32_t dat = s->dat[dev % ESP32P4_I3C_DEVS];

    return DAT_I2C(dat) ? DAT_STATIC(dat) : DAT_DYNAMIC(dat);
}

/* Address a device: false for a NACK. */
static bool esp32p4_i3c_mst_start_dev(ESP32P4I3CMstState *s, uint8_t addr,
                                      bool rnw, bool i2c)
{
    int ret;

    if (!s->connected) {
        return false;
    }
    if (i2c) {
        ret = legacy_i2c_start_transfer(s->bus, addr, rnw);
    } else {
        ret = i3c_start_transfer(s->bus, addr, rnw);
    }
    s->dev_active = true;
    s->dev_i2c = i2c;
    return ret == 0;
}

static void esp32p4_i3c_mst_end_dev(ESP32P4I3CMstState *s)
{
    if (s->dev_active) {
        legacy_i2c_end_transfer(s->bus);
        i3c_end_transfer(s->bus);
        s->dev_active = false;
    }
}

/* Master: DMA */

static bool esp32p4_i3c_mst_dma(ESP32P4I3CMstState *s, uint32_t en)
{
    return s->gdma && (s->regs[R_DEVICE_CTRL] & en);
}

static void esp32p4_i3c_mst_resume(ESP32P4I3CMstState *s);

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * GDMA fills the TX buffer a word at a time, the last word of a frame
 * as far as the frame goes.
 */
static void esp32p4_i3c_mst_dma_tx(ESP32P4I3CMstState *s)
{
    uint8_t buf[4];
    bool eof;
    size_t n, i;

    if (!esp32p4_i3c_mst_dma(s, CTRL_DMA_TX_EN)) {
        return;
    }
    while (!fifo32_is_full(&s->tx)) {
        n = esp32p4_gdma_pull(s->gdma, ESP32P4_GDMA_AHB_I3C, buf,
                              4 - s->dma_tx_bytes, &eof);
        for (i = 0; i < n; i++) {
            s->dma_tx_word |= (uint32_t)buf[i] << (8 * s->dma_tx_bytes++);
        }
        if (s->dma_tx_bytes == 4 || (eof && s->dma_tx_bytes)) {
            fifo32_push(&s->tx, s->dma_tx_word);
            s->dma_tx_word = 0;
            s->dma_tx_bytes = 0;
        }
        if (!n) {
            break;
        }
    }
    esp32p4_i3c_mst_update_irq(s);
    esp32p4_i3c_mst_resume(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * GDMA empties the RX buffer a word at a time as its channel has room,
 * and is told where a read ends.
 */
static void esp32p4_i3c_mst_dma_rx(ESP32P4I3CMstState *s)
{
    uint8_t buf[4];
    bool last;

    if (!esp32p4_i3c_mst_dma(s, CTRL_DMA_RX_EN)) {
        return;
    }
    while (!fifo32_is_empty(&s->rx) &&
           esp32p4_gdma_rx_room(s->gdma, ESP32P4_GDMA_AHB_I3C) >= 4) {
        if (s->dma_rx_hold && fifo32_num_used(&s->rx) == 1) {
            break;
        }
        last = s->dma_rx_eof && fifo32_num_used(&s->rx) == 1;
        stl_le_p(buf, fifo32_pop(&s->rx));
        esp32p4_gdma_push(s->gdma, ESP32P4_GDMA_AHB_I3C, buf, 4,
                          last ? ESP32P4_GDMA_EOF : 0);
        if (last) {
            s->dma_rx_eof = false;
        }
    }
    if (s->dma_rx_eof && fifo32_is_empty(&s->rx) &&
        esp32p4_gdma_rx_room(s->gdma, ESP32P4_GDMA_AHB_I3C)) {
        esp32p4_gdma_push(s->gdma, ESP32P4_GDMA_AHB_I3C, NULL, 0,
                          ESP32P4_GDMA_EOF);
        s->dma_rx_eof = false;
    }
    esp32p4_i3c_mst_update_irq(s);
    esp32p4_i3c_mst_resume(s);
}

static void esp32p4_i3c_mst_dma_tx_ready(void *opaque)
{
    esp32p4_i3c_mst_dma_tx(opaque);
}

static void esp32p4_i3c_mst_dma_rx_ready(void *opaque)
{
    esp32p4_i3c_mst_dma_rx(opaque);
}

static const ESP32P4GdmaPeriphOps esp32p4_i3c_mst_dma_ops = {
    .tx_ready = esp32p4_i3c_mst_dma_tx_ready,
    .rx_ready = esp32p4_i3c_mst_dma_rx_ready,
};

/* Master: commands */

static void esp32p4_i3c_mst_rx_byte(ESP32P4I3CMstState *s, uint8_t b)
{
    s->rx_word |= (uint32_t)b << (8 * s->rx_bytes);
    if (++s->rx_bytes == 4) {
        fifo32_push(&s->rx, s->rx_word);
        s->rx_word = 0;
        s->rx_bytes = 0;
        esp32p4_i3c_mst_dma_rx(s);
    }
}

static void esp32p4_i3c_mst_rx_flush(ESP32P4I3CMstState *s)
{
    if (s->rx_bytes) {
        fifo32_push(&s->rx, s->rx_word);
        s->rx_word = 0;
        s->rx_bytes = 0;
    }
}

/* The next byte to send of a write: from the command or the TX buffer. */
static uint8_t esp32p4_i3c_mst_tx_byte(ESP32P4I3CMstState *s)
{
    uint8_t b;

    if (CMD_ATTR(s->cmd_l) == ATTR_IMMEDIATE) {
        return extract32(s->cmd_h, 8 * s->done, 8);
    }
    if (!s->tx_bytes) {
        s->tx_word = fifo32_pop(&s->tx);
        s->tx_bytes = 4;
        esp32p4_i3c_mst_dma_tx(s);
    }
    b = s->tx_word;
    s->tx_word >>= 8;
    s->tx_bytes--;
    return b;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The command is over: its response, when ROC asks for one or it
 * failed; COMMAND_DONE; and the bus left with a STOP, or held for the
 * next command's repeated START.
 */
static void esp32p4_i3c_mst_cmd_end(ESP32P4I3CMstState *s)
{
    uint32_t attr = CMD_ATTR(s->cmd_l);
    bool rnw = CMD_RNW(s->cmd_l) && attr == ATTR_REGULAR;
    uint32_t dl;

    esp32p4_i3c_mst_rx_flush(s);
    if (rnw) {
        s->dma_rx_hold = false;
        s->dma_rx_eof = true;
    }
    esp32p4_i3c_mst_dma_rx(s);
    /* An unfinished TX word is the command's; the next starts afresh. */
    s->tx_bytes = 0;
    if (CMD_ROC(s->cmd_l) || s->err) {
        if (attr == ATTR_ADDR_ASSIGN) {
            dl = s->left;
        } else if (rnw) {
            dl = s->done;
        } else {
            dl = s->left;
        }
        if (fifo32_is_full(&s->resp)) {
            s->int_raw |= INT_RESP_BUF_OVF;
        } else {
            fifo32_push(&s->resp, (dl & 0xffff) |
                        (CMD_TID(s->cmd_l) << 24) | ((uint32_t)s->err << 28));
        }
    }
    s->int_raw |= INT_COMMAND_DONE;
    if (s->err) {
        s->int_raw |= INT_TRANSFER_ERR;
        fifo32_reset(&s->cmd);
    }
    esp32p4_i3c_mst_update_irq(s);
    if (!s->err && !CMD_TOC(s->cmd_l)) {
        if (fifo32_num_used(&s->cmd) >= 2) {
            esp32p4_i3c_mst_next(s);
            return;
        }
        esp32p4_i3c_mst_raise(s, INT_NXT_CMD_REQ_ERR);
    }
    esp32p4_i3c_mst_drive(s, false, false, s->phase_due);
    esp32p4_i3c_mst_after(s, extract32(s->regs[R_PP_TIME], 0, 8) + 1,
                          PH_STOP_SCL);
}

static void esp32p4_i3c_mst_fail(ESP32P4I3CMstState *s, uint8_t err)
{
    s->err = err;
    esp32p4_i3c_mst_cmd_end(s);
}

/* A repeated START, then @next. */
static void esp32p4_i3c_mst_rstart(ESP32P4I3CMstState *s, int next)
{
    s->rs_next = next;
    esp32p4_i3c_mst_drive(s, false, true, s->phase_due);
    esp32p4_i3c_mst_after(s, extract32(s->regs[R_PP_TIME], 0, 8) + 1,
                          PH_RS_SCL);
}

/* The data bytes of a regular or immediate transfer, one by one. */
static void esp32p4_i3c_mst_data(ESP32P4I3CMstState *s)
{
    bool rnw = CMD_RNW(s->cmd_l) && CMD_ATTR(s->cmd_l) == ATTR_REGULAR;

    if (!s->left) {
        esp32p4_i3c_mst_cmd_end(s);
        return;
    }
    if (rnw && fifo32_is_full(&s->rx)) {
        s->phase = PH_WAIT_RX;
        return;
    }
    if (!rnw && CMD_ATTR(s->cmd_l) == ATTR_REGULAR && !s->tx_bytes &&
        fifo32_is_empty(&s->tx)) {
        s->phase = PH_WAIT_TX;
        return;
    }
    esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s,
                          esp32p4_i3c_mst_data_kind(s)), PH_DATA);
}

/*
 * After a START or repeated START, the header the command starts with:
 * the broadcast address for a CCC, and with BA_INCLUDE for a private
 * transfer to an I3C device after a START, where a target's IBI can win
 * the arbitration of the header.
 */
static void esp32p4_i3c_mst_header(ESP32P4I3CMstState *s, bool after_start)
{
    uint32_t attr = CMD_ATTR(s->cmd_l);
    bool bcast = attr == ATTR_ADDR_ASSIGN || CMD_CP(s->cmd_l) ||
                 (after_start && !s->dev_i2c &&
                  (s->regs[R_DEVICE_CTRL] & CTRL_BA_INCLUDE));

    if (bcast) {
        esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, BIT_OD),
                              PH_HDR_BCAST);
        return;
    }
    esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, s->dev_i2c ? BIT_I2C :
                          after_start ? BIT_OD : BIT_PP), PH_HDR_DIRECT);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Begin the next command in the command buffer: with a START, after the
 * bus free time, or with a repeated START when the last one kept the bus.
 */
static void esp32p4_i3c_mst_next(ESP32P4I3CMstState *s)
{
    uint32_t attr;
    int64_t from;

    s->cmd_l = fifo32_pop(&s->cmd);
    s->cmd_h = fifo32_pop(&s->cmd);
    esp32p4_i3c_mst_update_irq(s);
    attr = CMD_ATTR(s->cmd_l);
    s->err = ERR_NONE;
    s->done = 0;
    s->byte_cnt = 0;
    s->dev = CMD_DEV_INDX(s->cmd_l);
    s->dev_i2c = attr != ATTR_ADDR_ASSIGN &&
                 DAT_I2C(s->dat[s->dev % ESP32P4_I3C_DEVS]);
    s->retries = 0;
    switch (attr) {
    case ATTR_REGULAR:
        s->left = CMD_DL(s->cmd_h);
        s->dma_rx_hold = CMD_RNW(s->cmd_l);
        break;
    case ATTR_IMMEDIATE:
        s->left = MIN(CMD_BC(s->cmd_l), 4);
        if (CMD_RNW(s->cmd_l)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: an immediate command "
                          "cannot read\n", TYPE_ESP32P4_I3C_MST);
            s->left = 0;
        }
        break;
    case ATTR_ADDR_ASSIGN:
        s->left = CMD_DEV_CNT(s->cmd_l);
        if (CMD_CCC(s->cmd_l) != CCC_ENTDAA &&
            CMD_CCC(s->cmd_l) != CCC_SETDASA) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: address assignment with "
                          "CCC 0x%02x\n", TYPE_ESP32P4_I3C_MST,
                          CMD_CCC(s->cmd_l));
            esp32p4_i3c_mst_raise(s, INT_CMD_CCC_MISMATCH);
            s->phase_due = esp32p4_i3c_now();
            s->in_txn = true;
            esp32p4_i3c_mst_fail(s, ERR_BROADCAST_NACK);
            return;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: command attribute %u\n",
                      TYPE_ESP32P4_I3C_MST, attr);
        s->left = 0;
        break;
    }
    if (CMD_MODE(s->cmd_l) >= 5 && attr != ATTR_ADDR_ASSIGN &&
        !s->dev_i2c) {
        qemu_log_mask(LOG_UNIMP, "%s: HDR transfers are not modelled\n",
                      TYPE_ESP32P4_I3C_MST);
    }
    if (s->in_txn) {
        esp32p4_i3c_mst_rstart(s, NEXT_CMD);
        return;
    }
    /* A START once the bus has been free long enough */
    from = MAX(esp32p4_i3c_now(), s->free_at);
    s->in_txn = true;
    s->connected = false;
    esp32p4_i3c_mst_drive(s, true, false, from);
    s->phase_due = from;
    esp32p4_i3c_mst_after(s, extract32(s->regs[R_SCL_START_HOLD], 0, 9) + 1,
                          PH_START);
}

/* Master: IBIs */

static void esp32p4_i3c_mst_ibi_report(ESP32P4I3CMstState *s,
                                       ESP32P4I3CIbi *ibi)
{
    unsigned words = DIV_ROUND_UP(ibi->len, 4);
    unsigned i;

    if (!ibi->valid) {
        return;
    }
    ibi->valid = false;
    if (ibi->notify) {
        if (fifo32_is_full(&s->ibi_status)) {
            s->int_raw |= INT_IBI_STATUS_BUF_OVF;
        } else {
            fifo32_push(&s->ibi_status, ibi->len | ((uint32_t)ibi->id << 8) |
                        ((uint32_t)!ibi->acked << 28));
        }
        for (i = 0; i < words; i++) {
            if (fifo32_is_full(&s->ibi_data)) {
                s->int_raw |= INT_IBI_DATA_BUF_OVF;
                break;
            }
            fifo32_push(&s->ibi_data, ldl_le_p(&ibi->data[4 * i]));
        }
    }
    s->int_raw |= INT_IBI_HANDLE_DONE;
    esp32p4_i3c_mst_update_irq(s);
}

/* Start reporting a pending IBI if the master is idle. */
static void esp32p4_i3c_mst_ibi_run(ESP32P4I3CMstState *s)
{
    uint64_t bits;

    if (s->busy || !s->ibi_pend.valid || !clock_get(s->clk)) {
        return;
    }
    s->busy = true;
    s->phase_due = MAX(esp32p4_i3c_now(), s->free_at);
    /* The header, the payload and the STOP */
    bits = 9 * esp32p4_i3c_mst_bit(s, BIT_OD) +
           9 * s->ibi_pend.len * esp32p4_i3c_mst_bit(s, BIT_PP) +
           (extract32(s->regs[R_SCL_STOP_SETUP], 0, 9) + 1);
    if (s->ibi_pend.disec_addr) {
        bits += 4 * 9 * esp32p4_i3c_mst_bit(s, BIT_PP);
    }
    esp32p4_i3c_mst_after(s, bits, PH_IBI);
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * A target asks for an IBI: hot-join and controller role requests are
 * refused; a target interrupt is refused when IBI_SIR_REQ_REJECT has its
 * address's bit, at ((addr >> 5) + (addr & 0x1f)) % 32, and its payload
 * taken when IBI_SIR_REQ_PAYLOAD has it. A master not on the wires hears
 * nothing.
 */
static int esp32p4_i3c_bus_ibi_handle(I3CBus *bus, uint8_t addr,
                                      bool is_recv)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(BUS(bus)->parent);
    unsigned bit = ((addr >> 5) + (addr & 0x1f)) % 32;
    bool reject = extract32(s->regs[R_IBI_SIR_REQ_REJECT], bit, 1);
    ESP32P4I3CIbi *ibi = &s->ibi_in;

    if (!s->connected || !clock_get(s->clk)) {
        return -1;
    }
    memset(ibi, 0, sizeof(*ibi));
    esp32p4_i3c_mst_raise(s, INT_IBI_DETECT);
    if (addr == I3C_HJ_ADDR || !is_recv) {
        qemu_log_mask(LOG_UNIMP, "%s: hot-join and controller role requests "
                      "are refused\n", TYPE_ESP32P4_I3C_MST);
        return -1;
    }
    ibi->valid = true;
    ibi->id = (addr << 1) | 1;
    ibi->acked = !reject;
    ibi->notify = !reject ||
                  (s->regs[R_IBI_NOTIFY_CTRL] & NOTIFY_SIR_REJECTED);
    if (reject && (s->regs[R_DEVICE_CTRL] & CTRL_AUTO_DIS_IBI)) {
        ibi->disec_addr = addr;
    }
    /* Whether the payload is wanted: len counts it, 0xff for none */
    ibi->len = extract32(s->regs[R_IBI_SIR_REQ_PAYLOAD], bit, 1) && !reject
               ? 0 : 0xff;
    return reject ? -1 : 0;
}

static int esp32p4_i3c_bus_ibi_recv(I3CBus *bus, uint8_t data)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(BUS(bus)->parent);
    ESP32P4I3CIbi *ibi = &s->ibi_in;

    if (!ibi->valid || ibi->len == 0xff ||
        ibi->len >= sizeof(ibi->data)) {
        return -1;
    }
    ibi->data[ibi->len++] = data;
    return 0;
}

static int esp32p4_i3c_bus_ibi_finish(I3CBus *bus)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(BUS(bus)->parent);
    ESP32P4I3CIbi *ibi = &s->ibi_in;

    if (!ibi->valid) {
        return 0;
    }
    if (ibi->len == 0xff) {
        ibi->len = 0;
    }
    /* Auto-disable: DISEC with DISINT to the refused target */
    if (ibi->disec_addr) {
        i3c_start_transfer(s->bus, I3C_BROADCAST, false);
        i3c_send_byte(s->bus, CCC_DISEC_DIRECT);
        i3c_start_transfer(s->bus, ibi->disec_addr, false);
        i3c_send_byte(s->bus, DISEC_INT);
        i3c_end_transfer(s->bus);
    }
    esp32p4_i3c_mst_ibi_report(s, &s->ibi_pend);
    s->ibi_pend = *ibi;
    ibi->valid = false;
    esp32p4_i3c_mst_ibi_run(s);
    return 0;
}

/* Master: phases */

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The bus is free: the next command, a pending IBI, or TRANSFER_COMPLETE.
 */
static void esp32p4_i3c_mst_idle(ESP32P4I3CMstState *s)
{
    s->phase = PH_NONE;
    if (fifo32_num_used(&s->cmd) >= 2 && s->busy) {
        esp32p4_i3c_mst_next(s);
        return;
    }
    if (s->busy) {
        s->busy = false;
        esp32p4_i3c_mst_raise(s, INT_TRANSFER_COMPLETE);
    }
    esp32p4_i3c_mst_ibi_run(s);
}

static void esp32p4_i3c_mst_phase_end(ESP32P4I3CMstState *s, int64_t t)
{
    uint32_t attr = CMD_ATTR(s->cmd_l);
    uint32_t phase = s->phase;
    bool rnw = CMD_RNW(s->cmd_l);
    uint32_t dat = s->dat[s->dev % ESP32P4_I3C_DEVS];
    uint8_t id[I3C_ENTDAA_SIZE], b;
    uint32_t n;
    uint64_t pid;
    unsigned dct;

    s->phase = PH_NONE;
    switch (phase) {
    case PH_START:
        esp32p4_i3c_mst_drive(s, false, false, t);
        /* Both lines read back low: the master is on its pads. */
        s->connected = !s->sda_in && !s->scl_in;
        esp32p4_i3c_mst_header(s, true);
        break;
    case PH_RS_SCL:
        esp32p4_i3c_mst_drive(s, true, true, t);
        esp32p4_i3c_mst_after(s, extract32(s->regs[R_SCL_RSTART_SETUP],
                                           0, 9) + 1, PH_RS_SDA);
        break;
    case PH_RS_SDA:
        esp32p4_i3c_mst_drive(s, true, false, t);
        esp32p4_i3c_mst_after(s, extract32(s->regs[R_SCL_START_HOLD],
                                           0, 9) + 1, PH_RS_END);
        break;
    case PH_RS_END:
        esp32p4_i3c_mst_drive(s, false, false, t);
        switch (s->rs_next) {
        case NEXT_DAA:
            esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, BIT_OD),
                                  PH_DAA_HDR);
            break;
        case NEXT_DASA:
            esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, BIT_PP),
                                  PH_DASA_HDR);
            break;
        case NEXT_DIRECT:
            esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s,
                                  s->dev_i2c ? BIT_I2C : BIT_PP),
                                  PH_HDR_DIRECT);
            break;
        default:
            esp32p4_i3c_mst_header(s, false);
            break;
        }
        break;
    case PH_HDR_BCAST:
        if (!s->connected ||
            i3c_start_transfer(s->bus, I3C_BROADCAST, false)) {
            s->dev_active = s->connected;
            esp32p4_i3c_mst_fail(s, ERR_BROADCAST_NACK);
            break;
        }
        s->dev_active = true;
        if (attr == ATTR_ADDR_ASSIGN || CMD_CP(s->cmd_l)) {
            esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, BIT_PP),
                                  PH_CCC);
        } else {
            esp32p4_i3c_mst_rstart(s, NEXT_DIRECT);
        }
        break;
    case PH_CCC:
        i3c_send_byte(s->bus, CMD_CCC(s->cmd_l));
        if (attr == ATTR_ADDR_ASSIGN) {
            if (!s->left) {
                esp32p4_i3c_mst_cmd_end(s);
            } else {
                esp32p4_i3c_mst_rstart(s, CMD_CCC(s->cmd_l) == CCC_ENTDAA ?
                                       NEXT_DAA : NEXT_DASA);
            }
        } else if (CMD_CCC(s->cmd_l) & 0x80) {
            esp32p4_i3c_mst_rstart(s, NEXT_DIRECT);
        } else {
            esp32p4_i3c_mst_data(s);
        }
        break;
    case PH_HDR_DIRECT:
        if (!esp32p4_i3c_mst_start_dev(s, esp32p4_i3c_mst_dev_addr(s, s->dev),
                                       rnw && attr == ATTR_REGULAR,
                                       s->dev_i2c)) {
            /* DNRC: retry the address that many times */
            if (s->retries < DAT_DNRC(dat)) {
                s->retries++;
                esp32p4_i3c_mst_rstart(s, NEXT_DIRECT);
                break;
            }
            esp32p4_i3c_mst_fail(s, ERR_ADDR_NACK);
            break;
        }
        esp32p4_i3c_mst_data(s);
        break;
    case PH_DATA:
        if (rnw && attr == ATTR_REGULAR) {
            if (s->dev_i2c) {
                b = legacy_i2c_recv(s->bus);
                if (s->left == 1) {
                    legacy_i2c_nack(s->bus);
                }
                n = 1;
            } else if (i3c_recv(s->bus, &b, 1, &n)) {
                n = 0;
            }
            if (!n) {
                /* The target ended the read early. */
                esp32p4_i3c_mst_cmd_end(s);
                break;
            }
            esp32p4_i3c_mst_rx_byte(s, b);
        } else {
            b = esp32p4_i3c_mst_tx_byte(s);
            if (s->dev_i2c) {
                if (legacy_i2c_send(s->bus, b)) {
                    s->done++;
                    s->left--;
                    s->byte_cnt++;
                    esp32p4_i3c_mst_fail(s, ERR_I2C_DATA_NACK);
                    break;
                }
            } else {
                i3c_send_byte(s->bus, b);
            }
        }
        s->done++;
        s->left--;
        s->byte_cnt++;
        esp32p4_i3c_mst_update_irq(s);
        esp32p4_i3c_mst_data(s);
        break;
    case PH_DAA_HDR:
        if (i3c_start_transfer(s->bus, I3C_BROADCAST, true)) {
            esp32p4_i3c_mst_fail(s, ERR_ADDR_NACK);
            break;
        }
        esp32p4_i3c_mst_after(s, 64 * esp32p4_i3c_mst_bit(s, BIT_OD),
                              PH_DAA_ID);
        break;
    case PH_DAA_ID:
        memset(id, 0, sizeof(id));
        i3c_recv(s->bus, id, I3C_ENTDAA_SIZE, &n);
        pid = ((uint64_t)id[0] << 40) | ((uint64_t)id[1] << 32) |
              ((uint32_t)id[2] << 24) | ((uint32_t)id[3] << 16) |
              ((uint32_t)id[4] << 8) | id[5];
        dct = s->present_dct % ESP32P4_I3C_DEVS;
        s->dct[dct][0] = pid >> 16;
        s->dct[dct][1] = pid & 0xffff;
        s->dct[dct][2] = id[6] | ((uint32_t)id[7] << 8);
        s->dct[dct][3] = (DAT_DYNAMIC(dat) << 1) | DAT_PARITY(dat);
        esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, BIT_OD),
                              PH_DAA_ADDR);
        break;
    case PH_DAA_ADDR:
        i3c_send_byte(s->bus, DAT_DYNAMIC(dat));
        s->present_dct = (s->present_dct + 1) % ESP32P4_I3C_DEVS;
        s->present_dat = s->dev % ESP32P4_I3C_DEVS;
        s->dev++;
        if (--s->left) {
            esp32p4_i3c_mst_rstart(s, NEXT_DAA);
        } else {
            esp32p4_i3c_mst_cmd_end(s);
        }
        break;
    case PH_DASA_HDR:
        if (!esp32p4_i3c_mst_start_dev(s, DAT_STATIC(dat), false, false)) {
            esp32p4_i3c_mst_fail(s, ERR_ADDR_NACK);
            break;
        }
        esp32p4_i3c_mst_after(s, 9 * esp32p4_i3c_mst_bit(s, BIT_PP),
                              PH_DASA_ADDR);
        break;
    case PH_DASA_ADDR:
        i3c_send_byte(s->bus, DAT_DYNAMIC(dat) << 1);
        s->present_dat = s->dev % ESP32P4_I3C_DEVS;
        s->dev++;
        if (--s->left) {
            esp32p4_i3c_mst_rstart(s, NEXT_DASA);
        } else {
            esp32p4_i3c_mst_cmd_end(s);
        }
        break;
    case PH_STOP_SCL:
        esp32p4_i3c_mst_drive(s, true, false, t);
        esp32p4_i3c_mst_after(s, extract32(s->regs[R_SCL_STOP_SETUP], 0, 9) + 1,
                              PH_STOP_SDA);
        break;
    case PH_STOP_SDA:
        esp32p4_i3c_mst_end_dev(s);
        s->in_txn = false;
        esp32p4_i3c_mst_drive(s, true, true, t);
        n = MAX(extract32(s->regs[R_SCL_STOP_HOLD], 0, 9) + 1,
                extract32(s->regs[R_BUS_FREE_TIME], 0, 16));
        s->free_at = t + esp32p4_i3c_cyc(s->phase_period, n);
        esp32p4_i3c_mst_idle(s);
        break;
    case PH_IBI:
        s->busy = false;
        esp32p4_i3c_mst_ibi_report(s, &s->ibi_pend);
        s->free_at = t;
        esp32p4_i3c_mst_idle(s);
        break;
    default:
        g_assert_not_reached();
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Software has given the buffers what a held transfer waits for.
 */
static void esp32p4_i3c_mst_resume(ESP32P4I3CMstState *s)
{
    if ((s->phase == PH_WAIT_TX && !fifo32_is_empty(&s->tx)) ||
        (s->phase == PH_WAIT_RX && !fifo32_is_full(&s->rx))) {
        s->phase = PH_NONE;
        s->phase_due = MAX(s->phase_due, esp32p4_i3c_now());
        esp32p4_i3c_mst_data(s);
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i3c_mst_trans_start(ESP32P4I3CMstState *s)
{
    if (s->busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TRANS_START while a transfer is "
                      "running\n", TYPE_ESP32P4_I3C_MST);
        return;
    }
    if (fifo32_num_used(&s->cmd) < 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TRANS_START with no command\n",
                      TYPE_ESP32P4_I3C_MST);
        return;
    }
    s->busy = true;
    if (!s->gdma &&
        (s->regs[R_DEVICE_CTRL] & (CTRL_DMA_TX_EN | CTRL_DMA_RX_EN))) {
        qemu_log_mask(LOG_UNIMP, "%s: no DMA controller is connected; the "
                      "buffers stay with the CPU\n", TYPE_ESP32P4_I3C_MST);
    }
    esp32p4_i3c_mst_dma_tx(s);
    if (s->regs[R_DEVICE_CTRL] &
        (CTRL_RX_BIT_ORDER | CTRL_RX_BYTE_ORDER | CTRL_MULTI_SLV_CCC)) {
        qemu_log_mask(LOG_UNIMP, "%s: RX_BIT_ORDER, RX_BYTE_ORDER and "
                      "MULTI_SLV_SINGLE_CCC_EN are not modelled\n",
                      TYPE_ESP32P4_I3C_MST);
    }
    esp32p4_i3c_mst_next(s);
}

/* Master: abandoning everything */
static void esp32p4_i3c_mst_abort(ESP32P4I3CMstState *s)
{
    timer_del(&s->timer);
    esp32p4_i3c_mst_end_dev(s);
    s->phase = PH_NONE;
    s->busy = false;
    s->in_txn = false;
    s->ibi_pend.valid = false;
    s->ibi_in.valid = false;
    esp32p4_i3c_mst_drive(s, true, true, esp32p4_i3c_now());
}

static void esp32p4_i3c_mst_reset_bufs(ESP32P4I3CMstState *s, uint32_t which)
{
    if (which & RST_CMD_BUF) {
        fifo32_reset(&s->cmd);
    }
    if (which & RST_RESP_BUF) {
        fifo32_reset(&s->resp);
    }
    if (which & RST_TX_DATA_BUF) {
        fifo32_reset(&s->tx);
        s->tx_bytes = 0;
        s->dma_tx_word = 0;
        s->dma_tx_bytes = 0;
    }
    if (which & RST_RX_DATA_BUF) {
        fifo32_reset(&s->rx);
        s->rx_bytes = 0;
        s->rx_word = 0;
        s->dma_rx_hold = false;
        s->dma_rx_eof = false;
    }
    if (which & RST_IBI_DATA_BUF) {
        fifo32_reset(&s->ibi_data);
    }
    if (which & RST_IBI_STATUS_BUF) {
        fifo32_reset(&s->ibi_status);
    }
}

/* Master: registers */

static bool esp32p4_i3c_mst_reg_exists(unsigned idx)
{
    switch (idx) {
    case R_COMMAND_BUF_PORT:
    case R_RESPONSE_BUF_PORT:
    case R_RX_DATA_PORT:
    case R_TX_DATA_PORT:
    case R_IBI_STATUS_BUF:
    case R_INT_CLR:
    case R_INT_RAW:
    case R_INT_ST:
    case R_IBI_DATA_BUF:
    case R_BUFFER_STATUS_LEVEL:
    case R_DATA_BUFFER_STATUS_LEVEL:
    case R_PRESENT_STATE0:
    case R_PRESENT_STATE1:
        return true;
    default:
        return idx < ESP32P4_I3C_MST_NREGS && esp32p4_i3c_mst_regs[idx].rw;
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static uint64_t esp32p4_i3c_mst_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    ESP32P4I3CMstState *s = opaque;
    unsigned idx = addr / 4;
    uint32_t v;

    if (addr >= A_DCT && addr < A_DCT + ESP32P4_I3C_DEVS * 16) {
        return s->dct[(addr - A_DCT) / 16][(addr / 4) % 4];
    }
    if (addr >= A_DAT && addr < A_DAT + ESP32P4_I3C_DEVS * 4) {
        /* The DAT is write-only. */
        return 0;
    }
    if (!esp32p4_i3c_mst_reg_exists(idx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved address "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I3C_MST, addr);
        return 0;
    }
    switch (idx) {
    case R_RESPONSE_BUF_PORT:
        v = fifo32_is_empty(&s->resp) ? 0 : fifo32_pop(&s->resp);
        esp32p4_i3c_mst_update_irq(s);
        return v;
    case R_RX_DATA_PORT:
        if (fifo32_is_empty(&s->rx)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: RX_DATA_PORT read with the "
                          "RX buffer empty\n", TYPE_ESP32P4_I3C_MST);
            return 0;
        }
        v = fifo32_pop(&s->rx);
        esp32p4_i3c_mst_update_irq(s);
        esp32p4_i3c_mst_resume(s);
        return v;
    case R_IBI_STATUS_BUF:
        v = fifo32_is_empty(&s->ibi_status) ? 0 : fifo32_pop(&s->ibi_status);
        esp32p4_i3c_mst_update_irq(s);
        return v;
    case R_IBI_DATA_BUF:
        v = fifo32_is_empty(&s->ibi_data) ? 0 : fifo32_pop(&s->ibi_data);
        esp32p4_i3c_mst_update_irq(s);
        return v;
    case R_COMMAND_BUF_PORT:
    case R_TX_DATA_PORT:
    case R_INT_CLR:
        return 0;
    case R_INT_RAW:
        return esp32p4_i3c_mst_raw(s);
    case R_INT_ST:
        return esp32p4_i3c_mst_raw(s) & s->regs[R_INT_ST_ENA];
    case R_BUFFER_STATUS_LEVEL:
        return fifo32_num_free(&s->cmd) |
               (fifo32_num_used(&s->resp) << 8) |
               (fifo32_num_used(&s->ibi_data) << 16) |
               (fifo32_num_used(&s->ibi_status) << 24);
    case R_DATA_BUFFER_STATUS_LEVEL:
        return fifo32_num_free(&s->tx) | (fifo32_num_used(&s->rx) << 16);
    case R_PRESENT_STATE0:
        return s->sda_in | (s->scl_in << 1) | ((s->busy || s->in_txn) << 2) |
               ((!s->busy && !s->in_txn &&
                 esp32p4_i3c_now() >= s->free_at) << 3) |
               (CMD_TID(s->cmd_l) << 9) | ((uint32_t)s->busy << 29);
    case R_PRESENT_STATE1:
        return s->byte_cnt;
    case R_DEVICE_TABLE:
        return s->regs[idx] | (s->present_dct << 8) | (s->present_dat << 12);
    case R_RND_ECO_CS:
        return s->regs[idx] & 1;
    default:
        return s->regs[idx];
    }
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i3c_mst_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    ESP32P4I3CMstState *s = opaque;
    unsigned idx = addr / 4;
    uint32_t v = value, old;

    if (addr >= A_DCT && addr < A_DCT + ESP32P4_I3C_DEVS * 16) {
        s->dct[(addr - A_DCT) / 16][(addr / 4) % 4] = v;
        return;
    }
    if (addr >= A_DAT && addr < A_DAT + ESP32P4_I3C_DEVS * 4) {
        s->dat[(addr - A_DAT) / 4] = v;
        return;
    }
    if (!esp32p4_i3c_mst_reg_exists(idx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved address "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I3C_MST, addr);
        return;
    }
    switch (idx) {
    case R_COMMAND_BUF_PORT:
        if (fifo32_is_full(&s->cmd)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: command buffer full\n",
                          TYPE_ESP32P4_I3C_MST);
            return;
        }
        fifo32_push(&s->cmd, v);
        break;
    case R_TX_DATA_PORT:
        if (fifo32_is_full(&s->tx)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TX buffer full\n",
                          TYPE_ESP32P4_I3C_MST);
            return;
        }
        fifo32_push(&s->tx, v);
        esp32p4_i3c_mst_resume(s);
        break;
    case R_RESPONSE_BUF_PORT:
    case R_RX_DATA_PORT:
    case R_IBI_STATUS_BUF:
    case R_IBI_DATA_BUF:
    case R_INT_ST:
    case R_BUFFER_STATUS_LEVEL:
    case R_DATA_BUFFER_STATUS_LEVEL:
    case R_PRESENT_STATE0:
    case R_PRESENT_STATE1:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I3C_MST, addr);
        return;
    case R_INT_CLR:
    case R_INT_RAW:
        s->int_raw &= ~v;
        break;
    case R_DEVICE_CTRL:
        old = s->regs[idx];
        s->regs[idx] = v & esp32p4_i3c_mst_regs[idx].rw;
        if ((v & CTRL_TRANS_START) && !(old & CTRL_TRANS_START)) {
            esp32p4_i3c_mst_trans_start(s);
        }
        esp32p4_i3c_mst_drive(s, s->scl_drv, s->sda_drv, esp32p4_i3c_now());
        break;
    case R_RESET_CTRL:
        if (v & RST_CORE_SOFT) {
            esp32p4_i3c_mst_abort(s);
            esp32p4_i3c_mst_reset_bufs(s, 0x7e);
            s->int_raw = 0;
        }
        esp32p4_i3c_mst_reset_bufs(s, v & 0x7e);
        s->regs[idx] = v & esp32p4_i3c_mst_regs[idx].rw;
        break;
    case R_DEVICE_TABLE:
        s->regs[idx] = v & esp32p4_i3c_mst_regs[idx].rw;
        s->present_dct = extract32(v, 0, 4);
        s->present_dat = extract32(v, 4, 4);
        break;
    case R_TIME_OUT_VALUE:
        s->regs[idx] = v & esp32p4_i3c_mst_regs[idx].rw;
        if (v & (BIT(5) | BIT(11) | BIT(17) | BIT(23))) {
            qemu_log_mask(LOG_UNIMP, "%s: the buffer timeouts are not "
                          "modelled\n", TYPE_ESP32P4_I3C_MST);
        }
        break;
    default:
        s->regs[idx] = v & esp32p4_i3c_mst_regs[idx].rw;
        break;
    }
    esp32p4_i3c_mst_update_irq(s);
}

static const MemoryRegionOps esp32p4_i3c_mst_ops = {
    .read = esp32p4_i3c_mst_read,
    .write = esp32p4_i3c_mst_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i3c_mst_reset_hold(Object *obj, ResetType type)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(obj);
    unsigned i;

    timer_del(&s->timer);
    if (s->bus) {
        esp32p4_i3c_mst_end_dev(s);
    }
    for (i = 0; i < ESP32P4_I3C_MST_NREGS; i++) {
        s->regs[i] = esp32p4_i3c_mst_regs[i].reset;
    }
    memset(s->dat, 0, sizeof(s->dat));
    memset(s->dct, 0, sizeof(s->dct));
    esp32p4_i3c_mst_reset_bufs(s, 0x7e);
    s->int_raw = 0;
    s->present_dct = 0;
    s->present_dat = 0;
    s->phase = PH_NONE;
    s->phase_period = 0;
    s->phase_left = 0;
    s->busy = false;
    s->in_txn = false;
    s->connected = false;
    s->dev_active = false;
    s->cmd_l = 0;
    s->cmd_h = 0;
    s->byte_cnt = 0;
    s->free_at = 0;
    s->ibi_in.valid = false;
    s->ibi_pend.valid = false;
    s->scl_drv = true;
    s->sda_drv = true;
}

static void esp32p4_i3c_mst_reset_exit(Object *obj, ResetType type)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(obj);

    esp32p4_i3c_mst_drive(s, true, true, esp32p4_i3c_now());
    esp32p4_i3c_mst_update_irq(s);
}

static void esp32p4_i3c_mst_init(Object *obj)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_i3c_mst_ops, s,
                          "esp32p4.i3c-mst", ESP32P4_I3C_BLOCK_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, &s->scl_out, ESP32P4_I3C_SCL_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->scl_oe, ESP32P4_I3C_SCL_OE, 1);
    qdev_init_gpio_out_named(dev, &s->sda_out, ESP32P4_I3C_SDA_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->sda_oe, ESP32P4_I3C_SDA_OE, 1);
    qdev_init_gpio_out_named(dev, &s->scl_pullup, ESP32P4_I3C_SCL_PULLUP, 1);
    qdev_init_gpio_out_named(dev, &s->sda_pullup, ESP32P4_I3C_SDA_PULLUP, 1);
    qdev_init_gpio_in_named(dev, esp32p4_i3c_mst_scl_in, ESP32P4_I3C_SCL_IN,
                            1);
    qdev_init_gpio_in_named(dev, esp32p4_i3c_mst_sda_in, ESP32P4_I3C_SDA_IN,
                            1);
    s->clk = qdev_init_clock_in(dev, "clk", esp32p4_i3c_mst_clock_update, s,
                                ClockUpdate);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, esp32p4_i3c_mst_timer, s);
    fifo32_create(&s->cmd, ESP32P4_I3C_CMD_WORDS);
    fifo32_create(&s->resp, ESP32P4_I3C_RESP_DEPTH);
    fifo32_create(&s->tx, ESP32P4_I3C_DATA_WORDS);
    fifo32_create(&s->rx, ESP32P4_I3C_DATA_WORDS);
    fifo32_create(&s->ibi_status, ESP32P4_I3C_IBI_STATUS_DEPTH);
    fifo32_create(&s->ibi_data, ESP32P4_I3C_IBI_DATA_WORDS);
    /* An input the matrix does not route reads high (table 10.12-1). */
    s->scl_in = true;
    s->sda_in = true;
}

static void esp32p4_i3c_mst_realize(DeviceState *dev, Error **errp)
{
    ESP32P4I3CMstState *s = ESP32P4_I3C_MST(dev);

    s->bus = i3c_init_bus_type(TYPE_ESP32P4_I3C_BUS, dev,
                               s->bus_name ? s->bus_name : "i3c");
    if (s->gdma) {
        esp32p4_gdma_attach(s->gdma, ESP32P4_GDMA_AHB_I3C,
                            &esp32p4_i3c_mst_dma_ops, s);
    }
}

static const VMStateDescription vmstate_esp32p4_i3c_ibi = {
    .name = "esp32p4-i3c-ibi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(valid, ESP32P4I3CIbi),
        VMSTATE_BOOL(acked, ESP32P4I3CIbi),
        VMSTATE_BOOL(notify, ESP32P4I3CIbi),
        VMSTATE_UINT8(id, ESP32P4I3CIbi),
        VMSTATE_UINT8(len, ESP32P4I3CIbi),
        VMSTATE_UINT8_ARRAY(data, ESP32P4I3CIbi,
                            ESP32P4_I3C_IBI_DATA_WORDS * 4),
        VMSTATE_UINT8(disec_addr, ESP32P4I3CIbi),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_i3c_mst = {
    .name = TYPE_ESP32P4_I3C_MST,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4I3CMstState),
        VMSTATE_TIMER(timer, ESP32P4I3CMstState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4I3CMstState,
                             ESP32P4_I3C_MST_NREGS),
        VMSTATE_UINT32_ARRAY(dat, ESP32P4I3CMstState, ESP32P4_I3C_DEVS),
        VMSTATE_UINT32_2DARRAY(dct, ESP32P4I3CMstState, ESP32P4_I3C_DEVS, 4),
        VMSTATE_UINT32(int_raw, ESP32P4I3CMstState),
        VMSTATE_BOOL(irq_level, ESP32P4I3CMstState),
        VMSTATE_FIFO32(cmd, ESP32P4I3CMstState),
        VMSTATE_FIFO32(resp, ESP32P4I3CMstState),
        VMSTATE_FIFO32(tx, ESP32P4I3CMstState),
        VMSTATE_FIFO32(rx, ESP32P4I3CMstState),
        VMSTATE_FIFO32(ibi_status, ESP32P4I3CMstState),
        VMSTATE_FIFO32(ibi_data, ESP32P4I3CMstState),
        VMSTATE_UINT8(present_dct, ESP32P4I3CMstState),
        VMSTATE_UINT8(present_dat, ESP32P4I3CMstState),
        VMSTATE_UINT32(rx_word, ESP32P4I3CMstState),
        VMSTATE_UINT8(rx_bytes, ESP32P4I3CMstState),
        VMSTATE_UINT32(tx_word, ESP32P4I3CMstState),
        VMSTATE_UINT8(tx_bytes, ESP32P4I3CMstState),
        VMSTATE_UINT32(dma_tx_word, ESP32P4I3CMstState),
        VMSTATE_UINT8(dma_tx_bytes, ESP32P4I3CMstState),
        VMSTATE_BOOL(dma_rx_hold, ESP32P4I3CMstState),
        VMSTATE_BOOL(dma_rx_eof, ESP32P4I3CMstState),
        VMSTATE_BOOL(scl_in, ESP32P4I3CMstState),
        VMSTATE_BOOL(sda_in, ESP32P4I3CMstState),
        VMSTATE_BOOL(scl_drv, ESP32P4I3CMstState),
        VMSTATE_BOOL(sda_drv, ESP32P4I3CMstState),
        VMSTATE_BOOL(connected, ESP32P4I3CMstState),
        VMSTATE_BOOL(busy, ESP32P4I3CMstState),
        VMSTATE_BOOL(in_txn, ESP32P4I3CMstState),
        VMSTATE_UINT32(cmd_l, ESP32P4I3CMstState),
        VMSTATE_UINT32(cmd_h, ESP32P4I3CMstState),
        VMSTATE_UINT32(phase, ESP32P4I3CMstState),
        VMSTATE_INT64(phase_due, ESP32P4I3CMstState),
        VMSTATE_UINT64(phase_period, ESP32P4I3CMstState),
        VMSTATE_UINT64(phase_left, ESP32P4I3CMstState),
        VMSTATE_UINT32(left, ESP32P4I3CMstState),
        VMSTATE_UINT32(done, ESP32P4I3CMstState),
        VMSTATE_UINT8(err, ESP32P4I3CMstState),
        VMSTATE_UINT8(dev, ESP32P4I3CMstState),
        VMSTATE_UINT8(retries, ESP32P4I3CMstState),
        VMSTATE_UINT8(rs_next, ESP32P4I3CMstState),
        VMSTATE_UINT16(byte_cnt, ESP32P4I3CMstState),
        VMSTATE_INT64(free_at, ESP32P4I3CMstState),
        VMSTATE_BOOL(dev_active, ESP32P4I3CMstState),
        VMSTATE_BOOL(dev_i2c, ESP32P4I3CMstState),
        VMSTATE_STRUCT(ibi_in, ESP32P4I3CMstState, 1, vmstate_esp32p4_i3c_ibi,
                       ESP32P4I3CIbi),
        VMSTATE_STRUCT(ibi_pend, ESP32P4I3CMstState, 1,
                       vmstate_esp32p4_i3c_ibi, ESP32P4I3CIbi),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_i3c_mst_properties[] = {
    DEFINE_PROP_STRING("bus-name", ESP32P4I3CMstState, bus_name),
    DEFINE_PROP_LINK("gdma", ESP32P4I3CMstState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
};

static void esp32p4_i3c_mst_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_i3c_mst_reset_hold;
    rc->phases.exit = esp32p4_i3c_mst_reset_exit;
    dc->realize = esp32p4_i3c_mst_realize;
    dc->vmsd = &vmstate_esp32p4_i3c_mst;
    dc->desc = "ESP32-P4 I3C master controller";
    device_class_set_props(dc, esp32p4_i3c_mst_properties);
}

static void esp32p4_i3c_bus_class_init(ObjectClass *klass, const void *data)
{
    I3CBusClass *bc = I3C_BUS_CLASS(klass);

    bc->ibi_handle = esp32p4_i3c_bus_ibi_handle;
    bc->ibi_recv = esp32p4_i3c_bus_ibi_recv;
    bc->ibi_finish = esp32p4_i3c_bus_ibi_finish;
}

/* Slave */

/* CONFIG */
#define CFG_SLVENA              BIT(0)
#define CFG_NACK                BIT(1)
#define CFG_IDRAND              BIT(8)
#define CFG_OFFLINE             BIT(9)
#define CFG_SADDR(v)            extract32((v), 25, 7)
#define CFG_RW                  0xfeff031f

/* STATUS */
#define ST_STNOTSTOP            BIT(0)
#define ST_STMSG                BIT(1)
#define ST_STCCCH               BIT(2)
#define ST_STREQRD              BIT(3)
#define ST_STREQWR              BIT(4)
#define ST_STDAA                BIT(5)
#define ST_START                BIT(8)
#define ST_MATCHED              BIT(9)
#define ST_STOP                 BIT(10)
#define ST_RXPEND               BIT(11)
#define ST_TXNOTFULL            BIT(12)
#define ST_DACHG                BIT(13)
#define ST_CCC                  BIT(14)
#define ST_ERRWARN              BIT(15)
#define ST_W1C                  (ST_START | ST_MATCHED | ST_STOP | ST_DACHG | \
                                 ST_CCC | BIT(16))

/* CTRL */
#define CTRL_EVENT(v)           extract32((v), 0, 2)
#define CTRL_EXTDATA            BIT(3)
#define CTRL_IBIDATA(v)         extract32((v), 8, 8)
#define CTRL_PENDINT(v)         extract32((v), 16, 4)
#define CTRL_ACTSTATE(v)        extract32((v), 20, 2)
#define CTRL_VENDINFO(v)        extract32((v), 24, 8)
#define CTRL_RW                 0xff3ffffb

#define SLV_INTS                (ST_STOP | ST_RXPEND | ST_TXNOTFULL)

/* DATACTRL */
#define DC_FLUSHTB              BIT(0)
#define DC_FLUSHFB              BIT(1)
#define DC_TXTRIG(v)            extract32((v), 4, 2)
#define DC_RXTRIG(v)            extract32((v), 6, 2)
#define DC_RW                   0xf0

/* Slave registers */
enum {
    A_SLV_CONFIG = 0x04,
    A_SLV_STATUS = 0x08,
    A_SLV_CTRL = 0x0c,
    A_SLV_INTSET = 0x10,
    A_SLV_INTCLR = 0x14,
    A_SLV_INTMASKED = 0x18,
    A_SLV_DATACTRL = 0x2c,
    A_SLV_WDATAB = 0x30,
    A_SLV_WDATABE = 0x34,
    A_SLV_RDARAB = 0x40,
    A_SLV_RDATAH = 0x48,
    A_SLV_CAPABILITIES2 = 0x5c,
    A_SLV_CAPABILITIES = 0x60,
    A_SLV_IDPARTNO = 0x6c,
    A_SLV_IDEXT = 0x70,
    A_SLV_VENDORID = 0x74,
};

#define SLV_CAPABILITIES2       0x00000100
#define SLV_CAPABILITIES        0x7c13fc1c
#define SLV_VENDORID_RESET      0x5550

static bool esp32p4_i3c_slv_running(ESP32P4I3CSlvState *s)
{
    return clock_get(s->clk) != 0;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * RXPEND and TXNOTFULL as the FIFO trigger levels give them: RX at not
 * empty, 1/4, 1/2 or 3/4 full; TX at empty, 1/4, 1/2 or not full.
 */
static uint32_t esp32p4_i3c_slv_status(ESP32P4I3CSlvState *s)
{
    unsigned depth = ESP32P4_I3C_SLV_FIFO;
    unsigned rx = fifo8_num_used(&s->rx), tx = fifo8_num_used(&s->tx);
    static const uint8_t rx_at[] = { 1, 4, 8, 12 };
    static const uint8_t tx_at[] = { 0, 4, 8, 15 };
    uint32_t st = s->status & ~(ST_RXPEND | ST_TXNOTFULL | ST_ERRWARN);

    if (rx >= rx_at[DC_RXTRIG(s->datactrl)]) {
        st |= ST_RXPEND;
    }
    if (tx <= tx_at[DC_TXTRIG(s->datactrl)] && tx < depth) {
        st |= ST_TXNOTFULL;
    }
    if (s->errwarn) {
        st |= ST_ERRWARN;
    }
    return st;
}

static void esp32p4_i3c_slv_update_irq(ESP32P4I3CSlvState *s)
{
    bool level = esp32p4_i3c_slv_status(s) & s->intset & SLV_INTS;

    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->irq, level);
    }
}

/* The provisioned ID, BCR and DCR the target logic answers with. */
static void esp32p4_i3c_slv_update_id(ESP32P4I3CSlvState *s)
{
    I3CTarget *t = I3C_TARGET(&s->target);
    uint64_t pid = ((uint64_t)extract32(s->vendorid, 0, 15) << 33) |
                   s->partno;

    if (s->config & CFG_IDRAND) {
        pid |= 1ull << 32;
    }
    /* QEMU's I3C core sends the PID least significant byte first. */
    t->pid = bswap64(pid) >> 16;
    t->bcr = extract32(s->idext, 16, 8);
    t->dcr = extract32(s->idext, 8, 8);
    t->static_address = CFG_SADDR(s->config);
}

/* The dynamic address has changed: DACHG. */
static void esp32p4_i3c_slv_check_da(ESP32P4I3CSlvState *s)
{
    I3CTarget *t = I3C_TARGET(&s->target);

    if (t->address != s->last_da) {
        s->last_da = t->address;
        s->status |= ST_DACHG;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * Ask for an IBI (CTRL.EVENT = 1) once the bus is free: the target pulls
 * SDA low, sends its address and, with BCR[2] set, IBIDATA, then its
 * extended data from the TX FIFO with EXTDATA. EVENT returns to 0 once
 * the master has taken or refused it. Controller role and hot-join
 * requests are not modelled.
 */
static void esp32p4_i3c_slv_try_ibi(ESP32P4I3CSlvState *s)
{
    I3CTarget *t = I3C_TARGET(&s->target);
    int ret;

    if (!s->ibi_waiting || !esp32p4_i3c_slv_running(s) ||
        (s->status & ST_STNOTSTOP) || !s->scl_in || !s->sda_in) {
        return;
    }
    s->ibi_waiting = false;
    if (!s->i3c_bus || !t->address || !s->ibi_enabled) {
        s->ctrl &= ~3u;
        return;
    }
    s->sda_drv = false;
    esp32_line_set_at(s->sda_out, 0, esp32p4_i3c_now());
    esp32_line_set_at(s->sda_oe, 1, esp32p4_i3c_now());
    ret = i3c_target_send_ibi(t, t->address, true);
    if (!ret && (t->bcr & BIT(2))) {
        i3c_target_send_ibi_bytes(t, CTRL_IBIDATA(s->ctrl));
        while ((s->ctrl & CTRL_EXTDATA) && !fifo8_is_empty(&s->tx)) {
            bool end = fifo8_pop(&s->tx_end);

            if (i3c_target_send_ibi_bytes(t, fifo8_pop(&s->tx)) || end) {
                break;
            }
        }
    }
    i3c_target_ibi_finish(t, 0);
    s->sda_drv = true;
    esp32_line_set_at(s->sda_oe, 0, esp32p4_i3c_now());
    esp32_line_set_at(s->sda_out, 1, esp32p4_i3c_now());
    s->ctrl &= ~3u;
    esp32p4_i3c_slv_update_irq(s);
}

/* The lines: START and STOP as the slave's own inputs see them. */
static void esp32p4_i3c_slv_lines(ESP32P4I3CSlvState *s, bool scl, bool sda)
{
    bool was_sda = s->sda_in;

    s->scl_in = scl;
    s->sda_in = sda;
    if (!esp32p4_i3c_slv_running(s) || !scl || was_sda == sda) {
        return;
    }
    if (!sda && s->sda_drv) {
        s->status |= ST_START | ST_STNOTSTOP;
    } else if (sda && (s->status & ST_STNOTSTOP)) {
        s->status &= ~(ST_STNOTSTOP | ST_STMSG | ST_STCCCH | ST_STREQRD |
                       ST_STREQWR | ST_STDAA);
        s->status |= ST_STOP;
        esp32p4_i3c_slv_try_ibi(s);
    }
    esp32p4_i3c_slv_update_irq(s);
}

static void esp32p4_i3c_slv_scl_in(void *opaque, int n, int level)
{
    ESP32P4I3CSlvState *s = opaque;

    esp32p4_i3c_slv_lines(s, level, s->sda_in);
    esp32p4_i3c_slv_try_ibi(s);
}

static void esp32p4_i3c_slv_sda_in(void *opaque, int n, int level)
{
    ESP32P4I3CSlvState *s = opaque;

    esp32p4_i3c_slv_lines(s, s->scl_in, level);
}

/* Slave: the target logic on the master's bus */

static ESP32P4I3CSlvState *esp32p4_i3c_slv_of(I3CTarget *t)
{
    return ESP32P4_I3C_SLV_TARGET(t)->slv;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * The target answers only while enabled and online, and only in a
 * transfer whose START its own inputs saw; with CONFIG.NACK it answers
 * nothing but broadcasts.
 */
static bool esp32p4_i3c_slv_match(I3CTarget *t, uint8_t address,
                                  bool is_read, bool broadcast,
                                  bool in_entdaa)
{
    ESP32P4I3CSlvState *s = esp32p4_i3c_slv_of(t);
    I3CTargetClass *parent = I3C_TARGET_CLASS(
        object_class_get_parent(object_get_class(OBJECT(t))));

    if (!esp32p4_i3c_slv_running(s) || !(s->config & CFG_SLVENA) ||
        (s->config & CFG_OFFLINE) || !(s->status & ST_STNOTSTOP)) {
        return false;
    }
    if ((s->config & CFG_NACK) && !broadcast && !in_entdaa) {
        return false;
    }
    return parent->target_match(t, address, is_read, broadcast, in_entdaa);
}

static int esp32p4_i3c_slv_event(I3CTarget *t, enum I3CEvent event)
{
    ESP32P4I3CSlvState *s = esp32p4_i3c_slv_of(t);
    I3CBus *bus = I3C_BUS(qdev_get_parent_bus(DEVICE(t)));

    switch (event) {
    case I3C_START_RECV:
    case I3C_START_SEND:
        s->status &= ~(ST_STREQRD | ST_STREQWR);
        if (bus->in_entdaa) {
            s->status |= ST_STDAA;
        } else if (!bus->broadcast) {
            s->status |= ST_MATCHED | ST_STMSG;
            if (event == I3C_START_RECV) {
                /* With nothing to send, the target refuses the read. */
                if (!bus->in_ccc && fifo8_is_empty(&s->tx)) {
                    s->status &= ~ST_STMSG;
                    esp32p4_i3c_slv_update_irq(s);
                    return -1;
                }
                s->status |= ST_STREQRD;
            } else {
                s->status |= ST_STREQWR;
            }
        }
        break;
    case I3C_STOP:
        esp32p4_i3c_slv_check_da(s);
        break;
    default:
        break;
    }
    esp32p4_i3c_slv_update_irq(s);
    return 0;
}

/*
 * A private write: the bytes go to the RX FIFO, those it lacks room for
 * are lost (ERRWARN).
 */
static int esp32p4_i3c_slv_send(I3CTarget *t, const uint8_t *data,
                                uint32_t num_to_send, uint32_t *num_sent)
{
    ESP32P4I3CSlvState *s = esp32p4_i3c_slv_of(t);
    uint32_t i;

    for (i = 0; i < num_to_send; i++) {
        if (fifo8_is_full(&s->rx)) {
            s->errwarn = true;
            continue;
        }
        fifo8_push(&s->rx, data[i]);
    }
    *num_sent = num_to_send;
    esp32p4_i3c_slv_update_irq(s);
    return 0;
}

/* A private read: the TX FIFO up to a byte marked as the end. */
static uint32_t esp32p4_i3c_slv_recv(I3CTarget *t, uint8_t *data,
                                     uint32_t num_to_read)
{
    ESP32P4I3CSlvState *s = esp32p4_i3c_slv_of(t);
    uint32_t i;

    for (i = 0; i < num_to_read && !fifo8_is_empty(&s->tx); ) {
        bool end = fifo8_pop(&s->tx_end);

        data[i++] = fifo8_pop(&s->tx);
        if (end) {
            break;
        }
    }
    esp32p4_i3c_slv_update_irq(s);
    return i;
}

/*
 * [spec:nuos:req:emu.esp32p4.i2c]
 * CCCs the target handles itself: the address assignments (with QEMU's
 * I3C core), ENEC and DISEC for its IBIs, GETPID, GETBCR, GETDCR and
 * GETSTATUS; others it reports with STATUS.CCC.
 */
static int esp32p4_i3c_slv_ccc_write(I3CTarget *t, const uint8_t *data,
                                     uint32_t num_to_send, uint32_t *num_sent)
{
    ESP32P4I3CSlvState *s = esp32p4_i3c_slv_of(t);
    uint32_t i = *num_sent;

    s->status |= ST_STCCCH;
    switch (t->curr_ccc) {
    case I3C_CCC_ENEC:
    case I3C_CCCD_ENEC:
        if (i < num_to_send && (data[i] & DISEC_INT)) {
            s->ibi_enabled = true;
        }
        break;
    case I3C_CCC_DISEC:
    case I3C_CCCD_DISEC:
        if (i < num_to_send && (data[i] & DISEC_INT)) {
            s->ibi_enabled = false;
        }
        break;
    case I3C_CCC_ENTDAA:
        s->status |= ST_STDAA;
        break;
    case I3C_CCC_RSTDAA:
    case I3C_CCCD_SETDASA:
    case I3C_CCCD_SETNEWDA:
    case I3C_CCC_SETAASA:
    case I3C_CCCD_GETPID:
    case I3C_CCCD_GETBCR:
    case I3C_CCCD_GETDCR:
    case I3C_CCCD_GETSTATUS:
        break;
    default:
        s->status |= ST_CCC;
        break;
    }
    *num_sent = num_to_send;
    esp32p4_i3c_slv_check_da(s);
    esp32p4_i3c_slv_update_irq(s);
    return 0;
}

static int esp32p4_i3c_slv_ccc_read(I3CTarget *t, uint8_t *data,
                                    uint32_t num_to_read, uint32_t *num_read)
{
    ESP32P4I3CSlvState *s = esp32p4_i3c_slv_of(t);
    uint8_t status[2] = {
        CTRL_VENDINFO(s->ctrl),
        (CTRL_ACTSTATE(s->ctrl) << 6) | CTRL_PENDINT(s->ctrl),
    };
    uint32_t n = 0;

    switch (t->curr_ccc) {
    case I3C_CCC_ENTDAA:
    case I3C_CCCD_GETPID:
    case I3C_CCCD_GETBCR:
    case I3C_CCCD_GETDCR:
        return 0;
    case I3C_CCCD_GETSTATUS:
        while (n < num_to_read && t->ccc_byte_offset < 2) {
            data[n++] = status[t->ccc_byte_offset++];
        }
        *num_read = n;
        return 0;
    default:
        return -1;
    }
}

/* Slave: registers */

/* [spec:nuos:req:emu.esp32p4.i2c] */
static uint64_t esp32p4_i3c_slv_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    ESP32P4I3CSlvState *s = opaque;
    uint32_t v;

    switch (addr) {
    case A_SLV_CONFIG:
        return s->config;
    case A_SLV_STATUS:
        return esp32p4_i3c_slv_status(s);
    case A_SLV_CTRL:
        return s->ctrl;
    case A_SLV_INTSET:
        return s->intset;
    case A_SLV_INTMASKED:
        return esp32p4_i3c_slv_status(s) & s->intset & SLV_INTS;
    case A_SLV_DATACTRL:
        return (s->datactrl & DC_RW) |
               (fifo8_num_used(&s->tx) << 16) |
               (fifo8_num_used(&s->rx) << 24) |
               ((uint32_t)fifo8_is_full(&s->tx) << 30) |
               ((uint32_t)fifo8_is_empty(&s->rx) << 31);
    case A_SLV_RDARAB:
        if (fifo8_is_empty(&s->rx)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: RDARAB read with the RX FIFO "
                          "empty\n", TYPE_ESP32P4_I3C_SLV);
            return 0;
        }
        v = fifo8_pop(&s->rx);
        esp32p4_i3c_slv_update_irq(s);
        return v;
    case A_SLV_RDATAH:
        if (fifo8_num_used(&s->rx) < 2) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: RDATAH read with fewer than "
                          "two bytes in the RX FIFO\n", TYPE_ESP32P4_I3C_SLV);
        }
        v = fifo8_is_empty(&s->rx) ? 0 : fifo8_pop(&s->rx);
        v |= (fifo8_is_empty(&s->rx) ? 0 : fifo8_pop(&s->rx)) << 8;
        esp32p4_i3c_slv_update_irq(s);
        return v;
    case A_SLV_CAPABILITIES2:
        return SLV_CAPABILITIES2;
    case A_SLV_CAPABILITIES:
        return SLV_CAPABILITIES;
    case A_SLV_IDPARTNO:
        return s->partno;
    case A_SLV_IDEXT:
        return s->idext;
    case A_SLV_VENDORID:
        return s->vendorid;
    case A_SLV_INTCLR:
    case A_SLV_WDATAB:
    case A_SLV_WDATABE:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved address "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I3C_SLV, addr);
        return 0;
    }
}

static void esp32p4_i3c_slv_push(ESP32P4I3CSlvState *s, uint8_t b, bool end)
{
    if (fifo8_is_full(&s->tx)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TX FIFO full\n",
                      TYPE_ESP32P4_I3C_SLV);
        s->errwarn = true;
        return;
    }
    fifo8_push(&s->tx, b);
    fifo8_push(&s->tx_end, end);
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i3c_slv_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    ESP32P4I3CSlvState *s = opaque;
    uint32_t v = value;

    switch (addr) {
    case A_SLV_CONFIG:
        s->config = v & CFG_RW;
        esp32p4_i3c_slv_update_id(s);
        break;
    case A_SLV_STATUS:
        s->status &= ~(v & ST_W1C);
        break;
    case A_SLV_CTRL:
        s->ctrl = v & CTRL_RW;
        switch (CTRL_EVENT(v)) {
        case 1:
            s->ibi_waiting = true;
            esp32p4_i3c_slv_try_ibi(s);
            break;
        case 2:
        case 3:
            qemu_log_mask(LOG_UNIMP, "%s: controller role and hot-join "
                          "requests are not modelled\n",
                          TYPE_ESP32P4_I3C_SLV);
            s->ctrl &= ~3u;
            break;
        default:
            s->ibi_waiting = false;
            break;
        }
        break;
    case A_SLV_INTSET:
        s->intset = v & SLV_INTS;
        break;
    case A_SLV_INTCLR:
        s->intset &= ~(v & SLV_INTS);
        break;
    case A_SLV_DATACTRL:
        if (v & DC_FLUSHTB) {
            fifo8_reset(&s->tx);
            fifo8_reset(&s->tx_end);
            s->errwarn = false;
        }
        if (v & DC_FLUSHFB) {
            fifo8_reset(&s->rx);
            s->errwarn = false;
        }
        s->datactrl = v & DC_RW;
        break;
    case A_SLV_WDATAB:
        esp32p4_i3c_slv_push(s, v, v & BIT(8));
        break;
    case A_SLV_WDATABE:
        esp32p4_i3c_slv_push(s, v, true);
        break;
    case A_SLV_IDPARTNO:
        s->partno = v;
        esp32p4_i3c_slv_update_id(s);
        break;
    case A_SLV_IDEXT:
        s->idext = v;
        esp32p4_i3c_slv_update_id(s);
        break;
    case A_SLV_VENDORID:
        s->vendorid = v & 0x7fff;
        esp32p4_i3c_slv_update_id(s);
        break;
    case A_SLV_INTMASKED:
    case A_SLV_RDARAB:
    case A_SLV_RDATAH:
    case A_SLV_CAPABILITIES2:
    case A_SLV_CAPABILITIES:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I3C_SLV, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved address "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_I3C_SLV, addr);
        return;
    }
    esp32p4_i3c_slv_update_irq(s);
}

static const MemoryRegionOps esp32p4_i3c_slv_ops = {
    .read = esp32p4_i3c_slv_read,
    .write = esp32p4_i3c_slv_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.i2c] */
static void esp32p4_i3c_slv_reset_hold(Object *obj, ResetType type)
{
    ESP32P4I3CSlvState *s = ESP32P4_I3C_SLV(obj);
    I3CTarget *t = I3C_TARGET(&s->target);

    s->config = CFG_SLVENA | (0x2f << 16);
    s->status = 0;
    s->ctrl = 0;
    s->intset = 0;
    s->datactrl = 0xb0;
    s->partno = 0;
    s->idext = 0;
    s->vendorid = SLV_VENDORID_RESET;
    fifo8_reset(&s->tx);
    fifo8_reset(&s->tx_end);
    fifo8_reset(&s->rx);
    s->errwarn = false;
    s->ibi_enabled = true;
    s->ibi_waiting = false;
    s->sda_drv = true;
    t->address = 0;
    s->last_da = 0;
    esp32p4_i3c_slv_update_id(s);
}

static void esp32p4_i3c_slv_reset_exit(Object *obj, ResetType type)
{
    ESP32P4I3CSlvState *s = ESP32P4_I3C_SLV(obj);
    int64_t now = esp32p4_i3c_now();

    /* A target drives SCL never, SDA only for an IBI. */
    esp32_line_set_at(s->scl_out, 1, now);
    esp32_line_set_at(s->scl_oe, 0, now);
    esp32_line_set_at(s->sda_out, 1, now);
    esp32_line_set_at(s->sda_oe, 0, now);
    esp32p4_i3c_slv_update_irq(s);
}

static void esp32p4_i3c_slv_init(Object *obj)
{
    ESP32P4I3CSlvState *s = ESP32P4_I3C_SLV(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_i3c_slv_ops, s,
                          "esp32p4.i3c-slv", ESP32P4_I3C_BLOCK_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, &s->scl_out, ESP32P4_I3C_SCL_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->scl_oe, ESP32P4_I3C_SCL_OE, 1);
    qdev_init_gpio_out_named(dev, &s->sda_out, ESP32P4_I3C_SDA_OUT, 1);
    qdev_init_gpio_out_named(dev, &s->sda_oe, ESP32P4_I3C_SDA_OE, 1);
    qdev_init_gpio_in_named(dev, esp32p4_i3c_slv_scl_in, ESP32P4_I3C_SCL_IN,
                            1);
    qdev_init_gpio_in_named(dev, esp32p4_i3c_slv_sda_in, ESP32P4_I3C_SDA_IN,
                            1);
    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
    object_initialize_child(obj, "target", &s->target,
                            TYPE_ESP32P4_I3C_SLV_TARGET);
    s->target.slv = s;
    fifo8_create(&s->tx, ESP32P4_I3C_SLV_FIFO);
    fifo8_create(&s->tx_end, ESP32P4_I3C_SLV_FIFO);
    fifo8_create(&s->rx, ESP32P4_I3C_SLV_FIFO);
    s->scl_in = true;
    s->sda_in = true;
}

/* The target logic goes on the master's bus, when the machine has one. */
static void esp32p4_i3c_slv_realize(DeviceState *dev, Error **errp)
{
    ESP32P4I3CSlvState *s = ESP32P4_I3C_SLV(dev);

    if (s->i3c_bus) {
        qdev_realize(DEVICE(&s->target), BUS(s->i3c_bus), errp);
    }
}

static const VMStateDescription vmstate_esp32p4_i3c_slv = {
    .name = TYPE_ESP32P4_I3C_SLV,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, ESP32P4I3CSlvState),
        VMSTATE_UINT32(config, ESP32P4I3CSlvState),
        VMSTATE_UINT32(status, ESP32P4I3CSlvState),
        VMSTATE_UINT32(ctrl, ESP32P4I3CSlvState),
        VMSTATE_UINT32(intset, ESP32P4I3CSlvState),
        VMSTATE_UINT32(datactrl, ESP32P4I3CSlvState),
        VMSTATE_UINT32(partno, ESP32P4I3CSlvState),
        VMSTATE_UINT32(idext, ESP32P4I3CSlvState),
        VMSTATE_UINT32(vendorid, ESP32P4I3CSlvState),
        VMSTATE_BOOL(irq_level, ESP32P4I3CSlvState),
        VMSTATE_FIFO8(tx, ESP32P4I3CSlvState),
        VMSTATE_FIFO8(tx_end, ESP32P4I3CSlvState),
        VMSTATE_FIFO8(rx, ESP32P4I3CSlvState),
        VMSTATE_BOOL(errwarn, ESP32P4I3CSlvState),
        VMSTATE_BOOL(scl_in, ESP32P4I3CSlvState),
        VMSTATE_BOOL(sda_in, ESP32P4I3CSlvState),
        VMSTATE_BOOL(sda_drv, ESP32P4I3CSlvState),
        VMSTATE_BOOL(ibi_enabled, ESP32P4I3CSlvState),
        VMSTATE_BOOL(ibi_waiting, ESP32P4I3CSlvState),
        VMSTATE_UINT8(last_da, ESP32P4I3CSlvState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_i3c_slv_properties[] = {
    DEFINE_PROP_LINK("i3c-bus", ESP32P4I3CSlvState, i3c_bus, TYPE_I3C_BUS,
                     I3CBus *),
};

static void esp32p4_i3c_slv_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_i3c_slv_reset_hold;
    rc->phases.exit = esp32p4_i3c_slv_reset_exit;
    dc->realize = esp32p4_i3c_slv_realize;
    dc->vmsd = &vmstate_esp32p4_i3c_slv;
    dc->desc = "ESP32-P4 I3C slave controller";
    device_class_set_props(dc, esp32p4_i3c_slv_properties);
}

static void esp32p4_i3c_slv_target_class_init(ObjectClass *klass,
                                              const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I3CTargetClass *tc = I3C_TARGET_CLASS(klass);

    dc->desc = "ESP32-P4 I3C slave controller's target logic";
    dc->user_creatable = false;
    tc->target_match = esp32p4_i3c_slv_match;
    tc->event = esp32p4_i3c_slv_event;
    tc->send = esp32p4_i3c_slv_send;
    tc->recv = esp32p4_i3c_slv_recv;
    tc->handle_ccc_write = esp32p4_i3c_slv_ccc_write;
    tc->handle_ccc_read = esp32p4_i3c_slv_ccc_read;
}

/* [spec:nuos:req:emu.esp32p4.i2c] */
static const TypeInfo esp32p4_i3c_types[] = {
    {
        .name          = TYPE_ESP32P4_I3C_MST,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4I3CMstState),
        .instance_init = esp32p4_i3c_mst_init,
        .class_init    = esp32p4_i3c_mst_class_init,
    },
    {
        .name          = TYPE_ESP32P4_I3C_BUS,
        .parent        = TYPE_I3C_BUS,
        .class_size    = sizeof(I3CBusClass),
        .class_init    = esp32p4_i3c_bus_class_init,
    },
    {
        .name          = TYPE_ESP32P4_I3C_SLV,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4I3CSlvState),
        .instance_init = esp32p4_i3c_slv_init,
        .class_init    = esp32p4_i3c_slv_class_init,
    },
    {
        .name          = TYPE_ESP32P4_I3C_SLV_TARGET,
        .parent        = TYPE_I3C_TARGET,
        .instance_size = sizeof(ESP32P4I3CSlvTarget),
        .class_init    = esp32p4_i3c_slv_target_class_init,
    },
};

DEFINE_TYPES(esp32p4_i3c_types)
