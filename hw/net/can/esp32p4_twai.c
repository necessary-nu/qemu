/*
 * ESP32-P4 Two-Wire Automotive Interface controllers TWAI0 to TWAI2
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 56; ESP-IDF's twai_reg.h and twai_struct.h for v3 and hal/twai_ll.h.
 * The controller descends from the NXP SJA1000 in PeliCAN mode, whose
 * datasheet settles some of what the TRM leaves open (noted where used).
 *
 * The registers
 * -------------
 *
 * All of TRM 56.7: MODE, CMD, STATUS, INTERRUPT (read to clear, but for
 * RI, which is high while messages are pending), INTERRUPT_ENABLE (IDLE's
 * enable is read-only zero, so the bus idle interrupt never fires),
 * BUS_TIMING_0/1, ARB_LOST_CAP and ERR_CODE_CAP (each holds its capture
 * until read), ERR_WARNING_LIMIT, RX_/TX_ERR_CNT, the 13 data registers
 * (the acceptance code and mask in reset mode, the receive buffer to read
 * and the transmit buffer to write in operation mode), RX_MESSAGE_COUNTER,
 * CLOCK_DIVIDER, the standby registers, ECO_CFG and the timestamp
 * registers. The configuration registers take writes only in reset mode,
 * the command register only in operation mode; writes they refuse are
 * logged. The status register resets to 0x0C, transmit buffer free and
 * last transmission complete, as the ESP32 TRM's description of this
 * controller gives (ESP-IDF's header lists 0, which would leave the
 * transmit buffer locked).
 *
 * Counter writes in reset mode act as reset mode ends, as the SJA1000's
 * do: a TEC of 255 forces bus-off, and a TEC write during bus-off ends
 * it. Entering reset mode, by software or at bus-off, abandons any
 * transmission and frees the transmit buffer, empties the receive FIFO,
 * clears TI and DOI, and takes the controller off the bus; leaving it, the
 * controller waits for 11 recessive bits before it takes part.
 *
 * The bus
 * -------
 *
 * Frames are bit streams: SOF to the CRC stuffed, the CRC-15, ACK and EOF
 * fields, timed by the controller's bit time, 2 * (BRP + 1) cycles of its
 * function clock TWAIn_CLK per time quantum and 3 + TSEG1 + TSEG2 quanta
 * per bit. A transmission starts on the bus's bit grid once the bus is
 * idle; controllers on one bus that start together arbitrate bit by bit,
 * the losers recording the bit in ARB_LOST_CAP and receiving the
 * winner's frame. A frame is acknowledged by a receiver that takes part,
 * drives the bus and is not in listen-only mode, or by another client of
 * the QEMU CAN bus that takes it; self-test mode needs none. A receiver
 * whose bit time differs from the transmitter's samples the transmitted
 * waveform at its own sample points, resynchronising on recessive to
 * dominant edges within its SJW, and detects the stuff, form or CRC error
 * its de-stuffed stream gives. Errors are signalled with active or
 * passive error flags, which the other nodes detect in turn; the error
 * counters follow TRM 56.2.3 (fault confinement), with the error warning
 * limit, error passive and bus-off, and bus-off recovery counting 128 runs
 * of 11 recessive bits on the bus.
 *
 * What each controller hears and drives follows the GPIO matrix's routing
 * of twai_rx and twai_tx. twai_rx from a pad not driven by the chip is the
 * bus behind a transceiver: the controller's QEMU CAN bus, if it has one.
 * twai_rx from the controller's own twai_tx through a pad is a bus of its
 * own, as ESP-IDF's loopback self-tests use. Otherwise the controller hears
 * twai_rx's level: recessive, as when unrouted, its own transmissions fail
 * with bit errors from SOF until it goes bus-off; dominant, it never sees
 * the bus idle. A controller whose twai_tx reaches no pad receives but
 * cannot acknowledge, signal errors or transmit.
 *
 * Frames from other clients of the QEMU CAN bus (can-host-socketcan)
 * arrive whole, as they end: they do not arbitrate against this device's
 * transmissions and cannot be destroyed by its error flags. A CAN FD frame
 * is a form error to this classical controller. Overload frames do not
 * occur. The pad lines carry the recessive level and the bus-off and
 * standby levels rather than each frame's bits; twai_clkout is TWAIn_CLK
 * divided per CLOCK_DIVIDER, evaluated whenever software samples the pads.
 *
 * The TRM leaves some things open, decided here as follows: the timestamp
 * counter counts TWAIn_CLK / (TS_DIV_NUM + 1) from 0 while TS_ENABLE is
 * set and is held at 0 while it is clear, and stamps each received frame
 * at its SOF; the standby signal is set by SW_STANDBY_EN rising, or by
 * the bus staying idle for HW_STANDBY_CNT cycles with HW_STANDBY_EN set,
 * and cleared while SW_STANDBY_CLR is set and by bus activity.
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
#include "hw/net/esp32p4_twai.h"
#include "migration/vmstate.h"

/* Registers */
#define A_MODE              0x00
#define A_CMD               0x04
#define A_STATUS            0x08
#define A_INT               0x0c
#define A_INT_ENA           0x10
#define A_BTR0              0x18
#define A_BTR1              0x1c
#define A_ALC               0x2c
#define A_ECC               0x30
#define A_EWL               0x34
#define A_RXERR             0x38
#define A_TXERR             0x3c
#define A_DATA0             0x40
#define A_DATA8             0x60
#define A_DATA12            0x70
#define A_RMC               0x74
#define A_CDR               0x7c
#define A_SW_STANDBY        0x80
#define A_HW_CFG            0x84
#define A_HW_STANDBY_CNT    0x88
#define A_IDLE_INTR_CNT     0x8c
#define A_ECO_CFG           0x90
#define A_TS_DATA           0x94
#define A_TS_PRESCALER      0x98
#define A_TS_CFG            0x9c

#define MODE_RM             BIT(0)
#define MODE_LOM            BIT(1)
#define MODE_STM            BIT(2)
#define MODE_AFM            BIT(3)

#define CMD_TR              BIT(0)
#define CMD_AT              BIT(1)
#define CMD_RRB             BIT(2)
#define CMD_CDO             BIT(3)
#define CMD_SRR             BIT(4)

#define ST_RBS              BIT(0)
#define ST_DOS              BIT(1)
#define ST_TBS              BIT(2)
#define ST_TCS              BIT(3)
#define ST_RS               BIT(4)
#define ST_TS               BIT(5)
#define ST_ES               BIT(6)
#define ST_BS               BIT(7)
#define ST_MS               BIT(8)

#define INT_RI              BIT(0)
#define INT_TI              BIT(1)
#define INT_EI              BIT(2)
#define INT_DOI             BIT(3)
#define INT_TSI             BIT(4)
#define INT_EPI             BIT(5)
#define INT_ALI             BIT(6)
#define INT_BEI             BIT(7)
/* INT_ENA bit 8, the bus idle interrupt's enable, is read-only zero. */
#define INT_ENA_MASK        0xffu

#define CDR_CD              0xffu
#define CDR_CLOCK_OFF       BIT(8)
#define SW_STANDBY_EN       BIT(0)
#define SW_STANDBY_CLR      BIT(1)
#define ECO_RDN_ENA         BIT(0)
#define ECO_RDN_RESULT      BIT(1)
#define TS_ENABLE           BIT(0)

/* ERR_CODE_CAP: type, direction and segment (TRM table 56.4-12). */
#define ECC_BIT             0
#define ECC_FORM            1
#define ECC_STUFF           2
#define ECC_OTHER           3
#define ECC_RX              BIT(5)
#define ECC(type, rx, seg)  (((type) << 6) | ((rx) ? ECC_RX : 0) | (seg))

enum {
    SEG_ID28_21 = 2,
    SEG_SOF = 3,
    SEG_SRTR = 4,
    SEG_IDE = 5,
    SEG_ID20_18 = 6,
    SEG_ID17_13 = 7,
    SEG_CRC = 8,
    SEG_R0 = 9,
    SEG_DATA = 10,
    SEG_DLC = 11,
    SEG_RTR = 12,
    SEG_R1 = 13,
    SEG_ID4_0 = 14,
    SEG_ID12_5 = 15,
    SEG_ACTIVE_FLAG = 17,
    SEG_CRC_DELIM = 24,
    SEG_ACK_SLOT = 25,
    SEG_EOF = 26,
    SEG_ACK_DELIM = 27,
};

/* What a controller's twai_rx hears (see the top of the file). */
enum {
    VIEW_CONST,
    VIEW_BUS,
    VIEW_LOOPBACK,
};

enum {
    ERR_ACTIVE,
    ERR_PASSIVE,
    ERR_BUS_OFF,
};

/* The timed actions of a frame on the bus, in the order they happen in. */
enum {
    ACT_ALI,
    ACT_ERR,
    ACT_REC8,
    ACT_PROBE,
    ACT_RX,
    ACT_TX_OK,
    ACT_END,
};

/* A failing transmission that reaches no bus. */
enum {
    LOCAL_SOF,
    LOCAL_FLAG,
    LOCAL_TAIL,
};

/* The pad signals, by controller. */
static const unsigned twai_sig_tx[] = {
    ESP32P4_SIG_TWAI0_TX_PAD_OUT, ESP32P4_SIG_TWAI1_TX_PAD_OUT,
    ESP32P4_SIG_TWAI2_TX_PAD_OUT,
};
static const unsigned twai_sig_rx[] = {
    ESP32P4_SIG_TWAI0_RX_PAD_IN, ESP32P4_SIG_TWAI1_RX_PAD_IN,
    ESP32P4_SIG_TWAI2_RX_PAD_IN,
};

static int64_t twai_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint8_t bitmask(int i)
{
    return 1u << i;
}

/*
 * Frames as bit streams
 * =====================
 *
 * A frame is kept as the 13 bytes of the transmit buffer's layout (TRM
 * table 56.4-3): frame information, then 2 (SFF) or 4 (EFF) identifier
 * bytes, then the data.
 */

#define TWAI_MAX_BITS   160
#define TWAI_RAW_BITS   128

typedef struct TwaiBits {
    unsigned len;
    unsigned crc_delim;
    uint8_t v[TWAI_MAX_BITS];
    uint8_t seg[TWAI_MAX_BITS];
    /* The bit's ARB_LOST_CAP position, -1 outside the arbitration field */
    int8_t alc[TWAI_MAX_BITS];
} TwaiBits;

static bool frame_eff(const uint8_t *f)
{
    return f[0] & 0x80;
}

static bool frame_rtr(const uint8_t *f)
{
    return f[0] & 0x40;
}

static unsigned frame_dlc(const uint8_t *f)
{
    return f[0] & 0x0f;
}

/* The data bytes on the bus: none for a remote frame, at most eight. */
static unsigned frame_ndata(const uint8_t *f)
{
    return frame_rtr(f) ? 0 : MIN(frame_dlc(f), 8);
}

static uint32_t frame_id(const uint8_t *f)
{
    uint32_t id;

    if (frame_eff(f)) {
        id = (f[1] << 21) | (f[2] << 13) | (f[3] << 5) | (f[4] >> 3);
    } else {
        id = (f[1] << 3) | (f[2] >> 5);
    }
    return id;
}

static const uint8_t *frame_data(const uint8_t *f)
{
    return f + (frame_eff(f) ? 5 : 3);
}

/*
 * The segment and arbitration position of unstuffed bit u of a frame,
 * whose format is known from u = 14 on (TRM table 56.4-12; the SJA1000's
 * ALC numbering).
 */
static uint8_t frame_bit_seg(bool eff, unsigned u, unsigned ndata,
                             int *alc)
{
    unsigned dlc0 = eff ? 35 : 15;
    unsigned data0 = dlc0 + 4;
    unsigned crc0 = data0 + 8 * ndata;

    *alc = -1;
    if (u == 0) {
        return SEG_SOF;
    }
    if (u <= 11) {
        *alc = u - 1;
        return u <= 8 ? SEG_ID28_21 : SEG_ID20_18;
    }
    if (u == 12) {
        *alc = 11;
        return SEG_SRTR;
    }
    if (u == 13) {
        *alc = 12;
        return SEG_IDE;
    }
    if (eff && u <= 31) {
        *alc = u - 1;
        return u <= 18 ? SEG_ID17_13 : u <= 26 ? SEG_ID12_5 : SEG_ID4_0;
    }
    if (eff && u == 32) {
        *alc = 31;
        return SEG_RTR;
    }
    if (eff && u == 33) {
        return SEG_R1;
    }
    if (u == dlc0 - 1) {
        return SEG_R0;
    }
    if (u < data0) {
        return SEG_DLC;
    }
    if (u < crc0) {
        return SEG_DATA;
    }
    return SEG_CRC;
}

static uint16_t crc15_step(uint16_t crc, unsigned bit)
{
    bool next = bit ^ ((crc >> 14) & 1);

    crc = (crc << 1) & 0x7fff;
    return next ? crc ^ 0x4599 : crc;
}

/* The bits of frame @f on the bus, SOF to the end of EOF. */
static void twai_encode(const uint8_t *f, TwaiBits *b)
{
    uint8_t raw[TWAI_RAW_BITS];
    bool eff = frame_eff(f), rtr = frame_rtr(f);
    unsigned ndata = frame_ndata(f), dlc = frame_dlc(f);
    uint32_t id = frame_id(f);
    const uint8_t *data = frame_data(f);
    unsigned n = 0, run = 0, i;
    uint16_t crc = 0;
    int last = -1;

    raw[n++] = 0;
    for (i = 0; i < 11; i++) {
        raw[n++] = (id >> ((eff ? 28 : 10) - i)) & 1;
    }
    if (eff) {
        raw[n++] = 1;           /* SRR */
        raw[n++] = 1;           /* IDE */
        for (i = 0; i < 18; i++) {
            raw[n++] = (id >> (17 - i)) & 1;
        }
        raw[n++] = rtr;
        raw[n++] = 0;           /* r1 */
        raw[n++] = 0;           /* r0 */
    } else {
        raw[n++] = rtr;
        raw[n++] = 0;           /* IDE */
        raw[n++] = 0;           /* r0 */
    }
    for (i = 0; i < 4; i++) {
        raw[n++] = (dlc >> (3 - i)) & 1;
    }
    for (i = 0; i < 8 * ndata; i++) {
        raw[n++] = (data[i / 8] >> (7 - i % 8)) & 1;
    }
    for (i = 0; i < n; i++) {
        crc = crc15_step(crc, raw[i]);
    }
    for (i = 0; i < 15; i++) {
        raw[n++] = (crc >> (14 - i)) & 1;
    }

    b->len = 0;
    for (i = 0; i < n; i++) {
        int alc;
        uint8_t seg = frame_bit_seg(eff, i, ndata, &alc);

        b->v[b->len] = raw[i];
        b->seg[b->len] = seg;
        b->alc[b->len++] = alc;
        run = raw[i] == last ? run + 1 : 1;
        last = raw[i];
        if (run == 5) {
            /* A stuff bit, the run's complement, starts the next run. */
            last = !last;
            run = 1;
            b->v[b->len] = last;
            b->seg[b->len] = seg;
            b->alc[b->len++] = -1;
        }
    }
    b->crc_delim = b->len;
    for (i = 0; i < 10; i++) {
        static const uint8_t tail[] = {
            SEG_CRC_DELIM, SEG_ACK_SLOT, SEG_ACK_DELIM, SEG_EOF, SEG_EOF,
            SEG_EOF, SEG_EOF, SEG_EOF, SEG_EOF, SEG_EOF,
        };

        b->v[b->len] = 1;
        b->seg[b->len] = tail[i];
        b->alc[b->len++] = -1;
    }
}

/* The frame as the receive FIFO stores it; returns its length. */
static unsigned twai_rx_bytes(const uint8_t *f, uint8_t *out)
{
    unsigned ndata = frame_ndata(f), hdr = frame_eff(f) ? 5 : 3;

    out[0] = f[0] & 0xcf;
    memcpy(out + 1, f + 1, hdr - 1);
    /*
     * The receive buffer has the RTR bit after the identifier, where the
     * TRM recommends software put it for self-reception.
     */
    if (frame_eff(f)) {
        out[4] = (f[4] & 0xf8) | (frame_rtr(f) ? 0x04 : 0);
    } else {
        out[2] = (f[2] & 0xe0) | (frame_rtr(f) ? 0x10 : 0);
    }
    memcpy(out + hdr, f + hdr, ndata);
    return hdr + ndata;
}

static void twai_frame_to_qemu(const uint8_t *f, qemu_can_frame *q)
{
    memset(q, 0, sizeof(*q));
    q->can_id = frame_id(f);
    if (frame_eff(f)) {
        q->can_id |= QEMU_CAN_EFF_FLAG;
    }
    if (frame_rtr(f)) {
        q->can_id |= QEMU_CAN_RTR_FLAG;
    }
    q->can_dlc = MIN(frame_dlc(f), 8);
    memcpy(q->data, frame_data(f), frame_ndata(f));
}

static void twai_frame_from_qemu(const qemu_can_frame *q, uint8_t *f)
{
    bool eff = q->can_id & QEMU_CAN_EFF_FLAG;
    bool rtr = q->can_id & QEMU_CAN_RTR_FLAG;
    uint32_t id = q->can_id & (eff ? QEMU_CAN_EFF_MASK : QEMU_CAN_SFF_MASK);

    memset(f, 0, 13);
    f[0] = (eff ? 0x80 : 0) | (rtr ? 0x40 : 0) | (q->can_dlc & 0x0f);
    if (eff) {
        f[1] = id >> 21;
        f[2] = id >> 13;
        f[3] = id >> 5;
        f[4] = id << 3;
    } else {
        f[1] = id >> 3;
        f[2] = id << 5;
    }
    memcpy(f + (eff ? 5 : 3), q->data, frame_ndata(f));
}

/*
 * The first bit at or after @from at which a receiver that has followed
 * @b detects an error when the bus is held at @level from @from on: the
 * sixth equal bit in the stuffed part, or a dominant bit where the fixed
 * fields are recessive. -1 if the frame is valid first, at the last but
 * one bit of EOF.
 */
static int twai_detect_forced(const TwaiBits *b, int from, bool level,
                              uint8_t *ecc)
{
    int run = 0, last = -1, k;

    for (k = 0; k < from && k < (int)b->crc_delim; k++) {
        run = b->v[k] == last ? run + 1 : 1;
        last = b->v[k];
    }
    for (k = from; k < (int)b->len - 1; k++) {
        if (k < (int)b->crc_delim) {
            run = level == last ? run + 1 : 1;
            last = level;
            if (run == 6) {
                *ecc = ECC(ECC_STUFF, true, b->seg[k]);
                return k;
            }
        } else if (k != (int)b->crc_delim + 1 && !level) {
            *ecc = ECC(ECC_FORM, true, b->seg[k]);
            return k;
        }
    }
    return -1;
}

/*
 * The first bit at or after @from at which a transmitter of @b sees a
 * dominant bus while it sends recessive, outside the ACK slot; -1 if none.
 */
static int twai_tx_detect(const TwaiBits *b, int from)
{
    for (int k = from; k < (int)b->len; k++) {
        if (k != (int)b->crc_delim + 1 && b->v[k]) {
            return k;
        }
    }
    return -1;
}

/* A receiver's bit timing, in picoseconds and time quanta. */
typedef struct TwaiTiming {
    int64_t tq;
    int pbs1;
    int pbs2;
    int sjw;
} TwaiTiming;

typedef struct TwaiDecode {
    int bit;
    uint8_t ecc;
    bool crc;
} TwaiDecode;

/* What a receiver makes of a frame sent at another bit time. */
enum {
    DECODE_OK,
    DECODE_ERROR,
    DECODE_NO_FRAME,
};

/* The bus level @t picoseconds after SOF, @b driven up to bit @stop. */
static bool twai_level_at(const TwaiBits *b, int stop, int64_t bit_ps,
                          int64_t t)
{
    int64_t k = t / bit_ps;

    return t < 0 || k >= stop || k >= (int64_t)b->len || b->v[k];
}

/*
 * Receive @b, transmitted at @bit_ps and driven up to bit @stop, with the
 * bit timing @rt: hard synchronisation at SOF, then a sample per bit at
 * the end of PBS1, resynchronising on each recessive to dominant edge
 * after a recessive sample, by the phase error in quanta up to SJW. An
 * SOF sampled recessive was a glitch: the receiver is idle again until
 * the next recessive to dominant edge. The de-stuffed stream is checked
 * for stuff and form errors up to the CRC delimiter and its CRC compared.
 * On DECODE_ERROR @out has the bus bit at which the receiver detected the
 * error, which for a CRC error is the ACK delimiter's (its error flag
 * follows that).
 */
static int twai_resample(const TwaiBits *b, int stop, int64_t bit_ps,
                         const TwaiTiming *rt, TwaiDecode *out)
{
    int nbt = 1 + rt->pbs1 + rt->pbs2;
    int64_t s = 0;
    bool prev = false;
    int run = 0, last = -1;
    unsigned u = 0, ndata = 0, crc_end = 0, dlc = 0;
    bool eff = false, rtr = false, crc_done = false;
    uint16_t crc = 0, crc_rx = 0;
    int j;

    for (j = 0; j < 4 * TWAI_MAX_BITS; j++) {
        int64_t end = s + nbt * rt->tq;
        int lengthen = 0, shorten = 0, alc;
        int64_t sp;
        bool v;
        uint8_t seg;

        if (prev) {
            for (int64_t k = s / bit_ps + 1; k * bit_ps < end; k++) {
                int q;

                if (k * bit_ps <= s) {
                    continue;
                }
                if (twai_level_at(b, stop, bit_ps, k * bit_ps) ||
                    !twai_level_at(b, stop, bit_ps, k * bit_ps - 1)) {
                    continue;
                }
                q = (k * bit_ps - s) / rt->tq;
                if (q == 0) {
                    break;
                }
                if (q <= rt->pbs1) {
                    lengthen = MIN(q, rt->sjw);
                } else {
                    shorten = MIN(nbt - q, rt->sjw);
                }
                break;
            }
        }
        sp = s + (1 + rt->pbs1 + lengthen) * rt->tq;
        v = twai_level_at(b, stop, bit_ps, sp);
        prev = v;
        s += (nbt + lengthen - shorten) * rt->tq;
        seg = crc_done ? SEG_CRC_DELIM : frame_bit_seg(eff, u, ndata, &alc);

        if (crc_done && run != 5) {
            /* The CRC delimiter. */
            if (!v) {
                out->ecc = ECC(ECC_FORM, true, SEG_CRC_DELIM);
                out->crc = false;
                out->bit = (s - 1) / bit_ps;
                return DECODE_ERROR;
            }
            if (crc_rx != crc) {
                out->ecc = ECC(ECC_OTHER, true, SEG_ACK_DELIM);
                out->crc = true;
                out->bit = (s + 2 * nbt * rt->tq - 1) / bit_ps;
                return DECODE_ERROR;
            }
            return DECODE_OK;
        }
        if (run == 5) {
            if (v == last) {
                out->ecc = ECC(ECC_STUFF, true, seg);
                out->crc = false;
                out->bit = (s - 1) / bit_ps;
                return DECODE_ERROR;
            }
            last = v;
            run = 1;
            continue;
        }
        run = v == last ? run + 1 : 1;
        last = v;

        if (u == 0 && v) {
            int64_t k;

            for (k = sp / bit_ps + 1; k < MIN(stop, (int)b->len); k++) {
                if (!b->v[k] && b->v[k - 1]) {
                    break;
                }
            }
            if (k >= MIN(stop, (int)b->len)) {
                return DECODE_NO_FRAME;
            }
            s = k * bit_ps;
            prev = false;
            run = 0;
            last = -1;
            crc = 0;
            continue;
        }
        if (!crc_end || u < crc_end - 15) {
            crc = crc15_step(crc, v);
        }
        if (u == 12) {
            rtr = v;
        } else if (u == 13) {
            eff = v;
        } else if (eff && u == 32) {
            rtr = v;
        }
        if (u >= (eff ? 35u : 15u) && u < (eff ? 39u : 19u)) {
            dlc = (dlc << 1) | v;
            if (u == (eff ? 38u : 18u)) {
                ndata = rtr ? 0 : MIN(dlc, 8);
                crc_end = (eff ? 39 : 19) + 8 * ndata + 15;
            }
        }
        if (crc_end && u >= crc_end - 15) {
            crc_rx = (crc_rx << 1) | v;
            if (u == crc_end - 1) {
                crc_done = true;
            }
        }
        u++;
    }
    out->ecc = ECC(ECC_STUFF, true, SEG_DATA);
    out->crc = false;
    out->bit = (s - 1) / bit_ps;
    return DECODE_ERROR;
}

/*
 * Controllers
 * ===========
 */

/* The bit time in picoseconds, or 0 while TWAIn_CLK is stopped. */
static int64_t twai_bit_ps(ESP32P4TwaiCtrl *c)
{
    uint32_t hz = clock_get_hz(c->clk);
    uint32_t brp = extract32(c->btr0, 0, 14);
    uint32_t nbt = 3 + extract32(c->btr1, 0, 4) + extract32(c->btr1, 4, 3);

    if (!hz) {
        return 0;
    }
    return muldiv64(1000000000000ULL, 2 * (brp + 1) * nbt, hz);
}

static void twai_timing(ESP32P4TwaiCtrl *c, TwaiTiming *t)
{
    uint32_t hz = clock_get_hz(c->clk);

    t->tq = hz ? muldiv64(1000000000000ULL,
                          2 * (extract32(c->btr0, 0, 14) + 1), hz) : 0;
    t->pbs1 = extract32(c->btr1, 0, 4) + 1;
    t->pbs2 = extract32(c->btr1, 4, 3) + 1;
    t->sjw = extract32(c->btr0, 14, 2) + 1;
}

static int64_t bits_ns(int64_t bit_ps, int64_t n)
{
    return n * bit_ps / 1000;
}

static int64_t twai_bits_ns(ESP32P4TwaiCtrl *c, int64_t n)
{
    return bits_ns(twai_bit_ps(c), n);
}

static bool twai_operating(ESP32P4TwaiCtrl *c)
{
    return !(c->mode & MODE_RM) && !c->in_reset && twai_bit_ps(c) > 0;
}

/* The bus the controller is on, or -1. */
static int twai_seg(ESP32P4TwaiCtrl *c)
{
    switch (c->view) {
    case VIEW_BUS:
        return c->s->bus_id[c->idx];
    case VIEW_LOOPBACK:
        return ESP32P4_TWAI_NUM + c->idx;
    default:
        return -1;
    }
}

static uint32_t twai_status(ESP32P4TwaiCtrl *c)
{
    uint32_t st = c->status;

    if (c->rmc) {
        st |= ST_RBS;
        if (!c->msg_valid[c->msg_rd]) {
            st |= ST_MS;
        }
    }
    return st;
}

static void twai_update_irq(ESP32P4TwaiCtrl *c)
{
    bool ri = c->rmc && (c->ier & INT_RI);

    qemu_set_irq(c->irq, c->ir || ri);
}

/* Latch interrupt @bit, if enabled, as its event happens. */
static void twai_raise(ESP32P4TwaiCtrl *c, uint32_t bit)
{
    if (c->ier & bit) {
        c->ir |= bit;
        twai_update_irq(c);
    }
}

/* twai_clkout: TWAIn_CLK / (2 * (CD + 1)), or itself for CD = 255. */
static bool twai_clkout(ESP32P4TwaiCtrl *c, int64_t now)
{
    uint32_t cd = c->cdr & CDR_CD;
    uint64_t div = cd == CDR_CD ? 1 : 2 * (cd + 1);

    if ((c->cdr & CDR_CLOCK_OFF) || !clock_is_enabled(c->clk)) {
        return false;
    }
    return (clock_ns_to_ticks(c->clk, 2 * now) / div) & 1;
}

static void twai_update_lines(ESP32P4TwaiCtrl *c)
{
    bool clkout = twai_clkout(c, twai_now());

    qemu_set_irq(c->tx_out, 1);
    qemu_set_irq(c->bus_off_out, c->err_state == ERR_BUS_OFF);
    qemu_set_irq(c->standby_out, c->standby);
    if (clkout != c->clkout_level) {
        c->clkout_level = clkout;
        qemu_set_irq(c->clkout_out, clkout);
    }
}

/* The standby signal */

static void twai_standby_schedule(ESP32P4TwaiCtrl *c)
{
    if (!(c->hw_cfg & 1) || c->standby || c->idle_since == INT64_MAX ||
        (c->sw_standby & SW_STANDBY_CLR) || !clock_get(c->clk)) {
        timer_del(c->standby_timer);
        return;
    }
    timer_mod(c->standby_timer,
              c->idle_since + clock_ticks_to_ns(c->clk, c->hw_standby_cnt));
}

static void twai_set_standby(ESP32P4TwaiCtrl *c, bool on)
{
    if (on && (c->sw_standby & SW_STANDBY_CLR)) {
        on = false;
    }
    c->standby = on;
    qemu_set_irq(c->standby_out, on);
    twai_standby_schedule(c);
}

static void twai_standby_timer(void *opaque)
{
    twai_set_standby(opaque, true);
}

/* The bus became busy (@busy) or idle at @t, as the controller sees it. */
static void twai_bus_activity(ESP32P4TwaiCtrl *c, bool busy, int64_t t)
{
    if (busy) {
        c->idle_since = INT64_MAX;
        if (c->standby) {
            twai_set_standby(c, false);
        }
    } else {
        c->idle_since = t;
    }
    twai_standby_schedule(c);
}

/* The timestamp counter */

static uint32_t twai_ts_div(ESP32P4TwaiCtrl *c)
{
    return extract32(c->ts_prescaler, 0, 16) + 1;
}

static uint64_t twai_ts_count(ESP32P4TwaiCtrl *c, int64_t t)
{
    if (!(c->ts_cfg & TS_ENABLE)) {
        return 0;
    }
    if (t < c->ts_base_ns) {
        return c->ts_count_base;
    }
    return c->ts_count_base +
           clock_ns_to_ticks(c->clk, t - c->ts_base_ns) / twai_ts_div(c);
}

static void twai_ts_schedule(ESP32P4TwaiCtrl *c)
{
    uint64_t next;

    if (!(c->ts_cfg & TS_ENABLE) || !clock_get(c->clk)) {
        timer_del(c->ts_timer);
        return;
    }
    next = (twai_ts_count(c, twai_now()) | 0xffffffffULL) + 1;
    timer_mod(c->ts_timer, c->ts_base_ns +
              clock_ticks_to_ns(c->clk, (next - c->ts_count_base) *
                                        twai_ts_div(c)));
}

/* Carry the count over a change of its clock or divider. */
static void twai_ts_rebase(ESP32P4TwaiCtrl *c)
{
    int64_t now = twai_now();

    c->ts_count_base = twai_ts_count(c, now);
    c->ts_base_ns = now;
}

static void twai_ts_timer(void *opaque)
{
    ESP32P4TwaiCtrl *c = opaque;

    twai_raise(c, INT_TSI);
    twai_ts_schedule(c);
}

/* The receive FIFO */

static void twai_fifo_clear(ESP32P4TwaiCtrl *c)
{
    c->fifo_rd = 0;
    c->fifo_used = 0;
    c->rmc = 0;
    c->msg_rd = 0;
    c->doi_armed = true;
}

/*
 * Store a received message: the space left takes what of it fits, and a
 * message that does not fit whole is an overrun message, counted but
 * marked missing (TRM 56.4.4).
 */
static void twai_fifo_push(ESP32P4TwaiCtrl *c, const uint8_t *msg,
                           unsigned len, uint32_t ts)
{
    unsigned room = ESP32P4_TWAI_FIFO_SIZE - c->fifo_used;
    unsigned n = MIN(len, room);
    unsigned slot = (c->msg_rd + c->rmc) % ESP32P4_TWAI_MAX_MSGS;
    unsigned wr = (c->fifo_rd + c->fifo_used) % ESP32P4_TWAI_FIFO_SIZE;

    if (c->rmc >= ESP32P4_TWAI_MAX_MSGS) {
        return;
    }
    for (unsigned i = 0; i < n; i++) {
        c->fifo[(wr + i) % ESP32P4_TWAI_FIFO_SIZE] = msg[i];
    }
    c->fifo_used += n;
    c->msg_len[slot] = n;
    c->msg_valid[slot] = n == len;
    c->msg_ts[slot] = ts;
    c->rmc++;
    if (n != len) {
        c->status |= ST_DOS;
        if (c->doi_armed) {
            c->doi_armed = false;
            twai_raise(c, INT_DOI);
        }
    }
    twai_update_irq(c);
}

static void twai_fifo_release(ESP32P4TwaiCtrl *c)
{
    if (!c->rmc) {
        return;
    }
    c->fifo_rd = (c->fifo_rd + c->msg_len[c->msg_rd]) %
                 ESP32P4_TWAI_FIFO_SIZE;
    c->fifo_used -= c->msg_len[c->msg_rd];
    c->msg_rd = (c->msg_rd + 1) % ESP32P4_TWAI_MAX_MSGS;
    c->rmc--;
    if (!c->rmc) {
        c->doi_armed = true;
    }
    twai_update_irq(c);
}

/*
 * The acceptance filter (TRM 56.4.5, the SJA1000's single and dual
 * filters), on the message as the FIFO stores it, with the RTR bit after
 * the identifier. A mask bit set makes its bit don't-care; data bytes a
 * frame does not carry are not compared.
 */
static bool twai_match(uint8_t v, uint8_t code, uint8_t mask)
{
    return !((v ^ code) & ~mask);
}

static bool twai_accept(ESP32P4TwaiCtrl *c, const uint8_t *m)
{
    const uint8_t *acr = c->acr, *amr = c->amr;
    unsigned ndata = frame_ndata(m);
    bool f1, f2;

    if (c->mode & MODE_AFM) {
        if (frame_eff(m)) {
            return twai_match(m[1], acr[0], amr[0]) &&
                   twai_match(m[2], acr[1], amr[1]) &&
                   twai_match(m[3], acr[2], amr[2]) &&
                   twai_match(m[4], acr[3], amr[3] | 0x03);
        }
        return twai_match(m[1], acr[0], amr[0]) &&
               twai_match(m[2], acr[1], amr[1] | 0x0f) &&
               (ndata < 1 || twai_match(m[3], acr[2], amr[2])) &&
               (ndata < 2 || twai_match(m[4], acr[3], amr[3]));
    }
    if (frame_eff(m)) {
        f1 = twai_match(m[1], acr[0], amr[0]) &&
             twai_match(m[2], acr[1], amr[1]);
        f2 = twai_match(m[1], acr[2], amr[2]) &&
             twai_match(m[2], acr[3], amr[3]);
        return f1 || f2;
    }
    f1 = twai_match(m[1], acr[0], amr[0]) &&
         twai_match(m[2], acr[1], amr[1] | 0x0f) &&
         (ndata < 1 || twai_match(m[3], (acr[1] << 4) | (acr[3] & 0x0f),
                                  (amr[1] << 4) | (amr[3] & 0x0f)));
    f2 = twai_match(m[1], acr[2], amr[2]) &&
         twai_match(m[2], acr[3], amr[3] | 0x0f);
    return f1 || f2;
}

/* Receive frame @f, whose SOF was at @t_sof, into the FIFO if accepted. */
static void twai_store(ESP32P4TwaiCtrl *c, const uint8_t *f, int64_t t_sof)
{
    uint8_t msg[13];
    unsigned len = twai_rx_bytes(f, msg);

    if (twai_accept(c, msg)) {
        twai_fifo_push(c, msg, len, twai_ts_count(c, t_sof));
    }
}

/*
 * Fault confinement
 * =================
 */

static void twai_kick(ESP32P4TwaiState *s, int seg);
static void twai_local_try(ESP32P4TwaiCtrl *c);
static void twai_enter_reset_mode(ESP32P4TwaiCtrl *c);

/*
 * STATUS's error and bus status and the error state from the counters,
 * with EI on a change of either status bit and EPI on a change between
 * error active and error passive.
 */
static void twai_update_error_state(ESP32P4TwaiCtrl *c)
{
    uint32_t old = c->status;

    if (c->err_state != ERR_BUS_OFF) {
        uint8_t st = c->tec >= 128 || c->rec >= 128 ? ERR_PASSIVE
                                                     : ERR_ACTIVE;

        if (st != c->err_state) {
            c->err_state = st;
            twai_raise(c, INT_EPI);
        }
    }
    if (c->tec >= c->ewl || c->rec >= c->ewl) {
        c->status |= ST_ES;
    } else {
        c->status &= ~ST_ES;
    }
    if (c->err_state == ERR_BUS_OFF) {
        c->status |= ST_BS;
    } else {
        c->status &= ~ST_BS;
    }
    if ((old ^ c->status) & (ST_ES | ST_BS)) {
        twai_raise(c, INT_EI);
    }
    qemu_set_irq(c->bus_off_out, c->err_state == ERR_BUS_OFF);
}

/*
 * Bus-off (TRM 56.4.6.3): the TEC set to 127 and the REC to 0, reset
 * mode entered.
 */
static void twai_bus_off(ESP32P4TwaiCtrl *c)
{
    c->tec = 127;
    c->rec = 0;
    c->err_state = ERR_BUS_OFF;
    twai_enter_reset_mode(c);
    twai_update_error_state(c);
}

/*
 * Count an error: @tec and @rec added to the counters, which are frozen
 * in listen-only mode. The REC does not count up from 128 (TRM
 * 56.2.3.3).
 */
static void twai_count(ESP32P4TwaiCtrl *c, int tec, int rec)
{
    if (c->mode & MODE_LOM) {
        return;
    }
    c->tec += tec;
    if (rec && c->rec < 128) {
        c->rec = MIN(c->rec + rec, 255);
    }
    if (c->tec >= 256) {
        twai_bus_off(c);
        return;
    }
    twai_update_error_state(c);
}

/* A bus error the controller detected, with its error code. */
static void twai_bus_error(ESP32P4TwaiCtrl *c, uint8_t ecc, bool exempt)
{
    if (!c->ecc_held) {
        c->ecc = ecc;
        c->ecc_held = true;
    }
    twai_raise(c, INT_BEI);
    if (ecc & ECC_RX) {
        twai_count(c, 0, 1);
    } else if (!exempt) {
        twai_count(c, 8, 0);
    }
}

/* The transmit buffer free again, the transmission abandoned or done. */
static void twai_tx_release(ESP32P4TwaiCtrl *c, bool done)
{
    c->tx_pending = false;
    c->status |= ST_TBS;
    if (done) {
        c->status |= ST_TCS;
    }
    twai_raise(c, INT_TI);
}

/*
 * Bus-off recovery (TRM 56.4.6.3): the TEC counts down from 127 with each
 * run of 11 recessive bits the controller sees, from recov_mark, while
 * the bus is idle, and the 128th run ends bus-off. The SJA1000 freezes it
 * if reset mode is entered again.
 */
static bool twai_hears_idle(ESP32P4TwaiCtrl *c)
{
    int seg = twai_seg(c);

    if (c->view == VIEW_CONST) {
        return c->rx_level;
    }
    return seg < 0 || !c->s->ep[seg].active;
}

static void twai_recov_fold(ESP32P4TwaiCtrl *c, int64_t now)
{
    int64_t occ = 11 * twai_bit_ps(c), n;

    if (!c->recovering || c->recov_paused || !occ || now <= c->recov_mark) {
        return;
    }
    n = (now - c->recov_mark) * 1000 / occ;
    n = MIN(n, c->tec);
    c->tec -= n;
    c->recov_mark += n * occ / 1000;
}

/* The next run that matters: the one that clears ES, or the 128th. */
static void twai_recov_schedule(ESP32P4TwaiCtrl *c)
{
    int64_t occ = 11 * twai_bit_ps(c);
    int64_t runs;

    if (!c->recovering || c->recov_paused || !occ) {
        timer_del(c->recov_timer);
        return;
    }
    runs = c->ewl && c->tec >= c->ewl ? c->tec - c->ewl + 1 : c->tec + 1;
    timer_mod(c->recov_timer, c->recov_mark + (runs * occ + 999) / 1000);
}

static void twai_recov_check(ESP32P4TwaiCtrl *c)
{
    int64_t occ = 11 * twai_bit_ps(c), now = twai_now();

    twai_recov_fold(c, now);
    if (c->recovering && !c->recov_paused && !c->tec && occ &&
        (now - c->recov_mark) * 1000 >= occ) {
        /* It has just seen 11 recessive bits: it is on the bus at once. */
        c->recovering = false;
        c->err_state = ERR_ACTIVE;
        c->sync_at = twai_now();
        twai_update_error_state(c);
        timer_del(c->recov_timer);
        if (twai_seg(c) >= 0) {
            twai_kick(c->s, twai_seg(c));
        }
        twai_local_try(c);
        return;
    }
    twai_update_error_state(c);
    twai_recov_schedule(c);
}

static void twai_recov_timer(void *opaque)
{
    twai_recov_check(opaque);
}

static void twai_recov_pause(ESP32P4TwaiCtrl *c, int64_t t)
{
    if (c->recovering && !c->recov_paused) {
        twai_recov_fold(c, t);
        c->recov_paused = true;
        twai_update_error_state(c);
        timer_del(c->recov_timer);
    }
}

/* Count again from @mark, the start of the bus's recessive run. */
static void twai_recov_resume(ESP32P4TwaiCtrl *c, int64_t mark)
{
    if (c->recovering && c->recov_paused) {
        c->recov_paused = false;
        c->recov_mark = mark;
        twai_recov_check(c);
    }
}

/*
 * Integration: on joining a bus the controller takes part once it has
 * seen 11 recessive bits (TRM 56.4.1.2).
 */
static void twai_integrate(ESP32P4TwaiCtrl *c)
{
    if (!twai_operating(c) || c->err_state == ERR_BUS_OFF ||
        (c->view == VIEW_CONST && !c->rx_level)) {
        c->sync_at = INT64_MAX;
        return;
    }
    c->sync_at = twai_now() + twai_bits_ns(c, 11);
}

/*
 * Frames on a bus
 * ===============
 *
 * An episode is a frame on one bus from SOF until the bus is idle again.
 * It starts with the contenders that are ready at its SOF, arbitrates
 * among them at once, and lays out what follows as timed actions, which
 * its timer carries out in order. Receivers leaving before the frame is
 * destroyed, and the ACK slot, where other clients of the QEMU CAN bus are
 * asked, lay the rest out again from the same start.
 */

static int64_t ep_time(ESP32P4TwaiEpisode *ep, int64_t boundary)
{
    return ep->t_sof + bits_ns(ep->bit_ps, boundary);
}

static int64_t act_key(const ESP32P4TwaiAction *a)
{
    return ((int64_t)a->boundary << 8) | (a->kind << 4) | (a->ctrl & 0xf);
}

static int act_cmp(const void *pa, const void *pb)
{
    int64_t a = act_key(pa), b = act_key(pb);

    return a < b ? -1 : a > b;
}

static void ep_add(ESP32P4TwaiEpisode *ep, int boundary, int kind, int ctrl,
                   unsigned arg)
{
    ESP32P4TwaiAction *a;

    if (ep->nact >= ESP32P4_TWAI_ACTIONS) {
        return;
    }
    a = &ep->act[ep->nact++];
    a->boundary = boundary;
    a->kind = kind;
    a->ctrl = ctrl;
    a->arg = arg;
    if (act_key(a) <= ep->done_key) {
        ep->nact--;
    }
}

/* What becomes of one controller in a frame. */
typedef struct TwaiOutcome {
    int det;
    uint8_t ecc;
    bool exempt;
    bool crc;
    /* A receiver that never found the frame's SOF. */
    bool no_frame;
    int flag;
} TwaiOutcome;

static bool twai_flags_active(ESP32P4TwaiEpisode *ep, ESP32P4TwaiCtrl *c)
{
    return !(ep->passive & bitmask(c->idx)) && c->drives &&
           !(c->mode & MODE_LOM);
}

/*
 * Lay out the frame from what is known: the arbitration, the receivers
 * still taking part, whether the winner still drives, and, once its ACK
 * slot has passed, whether the frame was acknowledged.
 */
/* [spec:nuos:req:emu.esp32p4.twai] */
static void twai_ep_plan(ESP32P4TwaiState *s, int seg)
{
    ESP32P4TwaiEpisode *ep = &s->ep[seg];
    uint8_t alive = ep->members & ~ep->departed;
    uint8_t txs = ep->survivors & alive;
    uint8_t collided = 0, receivers;
    TwaiOutcome o[ESP32P4_TWAI_NUM];
    TwaiBits wb;
    int L, a, stop, d = INT_MAX, fend = -1, i, end, arb_end = 0;

    twai_encode(ep->frame, &wb);
    L = wb.len;
    a = wb.crc_delim + 1;
    stop = ep->tx_stop >= 0 ? ep->tx_stop : L;
    for (i = 0; i < L; i++) {
        if (wb.alc[i] >= 0) {
            arb_end = i + 1;
        }
    }
    ep->nact = 0;
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        o[i] = (TwaiOutcome) { .det = -1, .flag = -1 };
    }

    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        if (!(ep->contenders & alive & bitmask(i))) {
            continue;
        }
        if (ep->lost_bit[i] >= 0) {
            ep_add(ep, ep->lost_bit[i] + 1, ACT_ALI, i, ep->lost_alc[i]);
        } else if (ep->coll_bit[i] >= 0) {
            collided |= bitmask(i);
            o[i].det = ep->coll_bit[i];
            o[i].ecc = ep->coll_ecc[i];
            if (!(ep->passive & bitmask(i))) {
                o[i].flag = o[i].det + 1;
            }
        }
    }

    /* Each receiver decodes the bus at its own bit timing. */
    receivers = alive & ~txs & ~collided;
    ep->ackers = 0;
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];

        if (!(receivers & bitmask(i))) {
            continue;
        }
        if (twai_bit_ps(c) == ep->bit_ps) {
            if (stop < L) {
                o[i].det = twai_detect_forced(&wb, stop, true, &o[i].ecc);
            }
        } else {
            TwaiTiming rt;
            TwaiDecode dec;

            twai_timing(c, &rt);
            switch (twai_resample(&wb, stop, ep->bit_ps, &rt, &dec)) {
            case DECODE_ERROR:
                o[i].det = dec.bit;
                o[i].ecc = dec.ecc;
                o[i].crc = dec.crc;
                break;
            case DECODE_NO_FRAME:
                o[i].no_frame = true;
                break;
            }
        }
        if (o[i].det >= L - 1) {
            /*
             * It is still receiving as the frame ends; its error flag
             * follows the frame and does not destroy it.
             */
            o[i].det = L - 2;
        } else if (o[i].det >= 0 && twai_flags_active(ep, c)) {
            o[i].flag = o[i].det + 1;
        }
        if ((o[i].det < 0 || (o[i].det > a && !o[i].crc)) &&
            !o[i].no_frame && c->drives && !(c->mode & MODE_LOM)) {
            ep->ackers |= bitmask(i);
        }
    }
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        if (o[i].flag >= 0) {
            d = MIN(d, o[i].flag);
        }
    }

    /* The ACK slot, reached intact with the winner still driving. */
    if (d > a && stop > a && txs) {
        if (!ep->ack_state) {
            ep_add(ep, a, ACT_PROBE, ep->winner, 0);
            for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
                if (o[i].det >= 0 && o[i].det < a) {
                    ep_add(ep, o[i].det + 1, ACT_ERR, i, o[i].ecc);
                }
            }
            ep->destroy_at = INT32_MAX;
            qsort(ep->act, ep->nact, sizeof(ep->act[0]), act_cmp);
            return;
        }
        if (ep->ack_state != 1) {
            for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
                if (!(txs & bitmask(i)) || (s->ctrl[i].mode & MODE_STM)) {
                    continue;
                }
                /*
                 * An acknowledgement error. An error passive transmitter's
                 * passive flag finds no dominant bit, and it does not count
                 * it (TRM 56.2.3.4, TEC rule 1).
                 */
                o[i].det = a;
                o[i].ecc = ECC(ECC_OTHER, false, SEG_ACK_SLOT);
                o[i].exempt = ep->passive & bitmask(i);
                if (!o[i].exempt) {
                    o[i].flag = a + 1;
                    d = MIN(d, a + 1);
                }
            }
        }
    }
    ep->destroy_at = d == INT_MAX ? INT32_MAX : d;

    if (d != INT_MAX) {
        /* An active error flag from bit d: the others detect it in turn. */
        for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
            ESP32P4TwaiCtrl *c = &s->ctrl[i];
            bool rx = !((txs | collided) & bitmask(i));
            int k;

            if (!(alive & bitmask(i)) || (o[i].det >= 0 && o[i].det < d)) {
                continue;
            }
            o[i] = (TwaiOutcome) { .det = -1, .flag = -1 };
            if (!rx) {
                k = twai_tx_detect(&wb, d);
                if (k >= 0 && wb.alc[k] >= 0) {
                    /* Overwritten in the arbitration field: it lost. */
                    ep_add(ep, k + 1, ACT_ALI, i, wb.alc[k]);
                    rx = true;
                } else if (k >= 0) {
                    /*
                     * A recessive stuff bit overwritten in the arbitration
                     * field is a stuff error the TEC does not count (TRM
                     * 56.2.3.4, TEC rule 1).
                     */
                    o[i].det = k;
                    o[i].exempt = k < arb_end;
                    o[i].ecc = ECC(k < arb_end ? ECC_STUFF : ECC_BIT, false,
                                   wb.seg[k]);
                    if (!(ep->passive & bitmask(i))) {
                        o[i].flag = k + 1;
                    }
                }
            }
            if (rx) {
                k = twai_detect_forced(&wb, d, false, &o[i].ecc);
                if (k >= 0) {
                    o[i].det = k;
                    if (twai_flags_active(ep, c)) {
                        o[i].flag = k + 1;
                    }
                }
            }
        }
        for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
            if (o[i].flag >= 0) {
                fend = MAX(fend, o[i].flag + 5);
            }
        }
        for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
            /*
             * A receiver that sees a dominant bit just after its own flag
             * counts 8 (TRM 56.2.3.4, REC rule 1).
             */
            if (o[i].flag >= 0 && o[i].flag + 5 < fend &&
                (o[i].ecc & ECC_RX)) {
                ep_add(ep, o[i].flag + 7, ACT_REC8, i, 0);
            }
        }
        end = fend + 12;
    } else {
        end = L + 3;
        for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
            if (o[i].det == a && !(o[i].ecc & ECC_RX)) {
                /* A passive flag, error delimiter and intermission. */
                end = MAX(end, a + 18);
            }
        }
    }

    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];

        if (!(alive & bitmask(i))) {
            continue;
        }
        if (o[i].det >= 0) {
            ep_add(ep, o[i].det + 1, ACT_ERR, i,
                   o[i].ecc | (o[i].exempt ? 0x100 : 0));
        } else if (txs & bitmask(i)) {
            if (stop >= L) {
                if (c->tx_self_rx) {
                    ep_add(ep, L - 1, ACT_RX, i, 1);
                }
                ep_add(ep, L, ACT_TX_OK, i, 0);
            }
        } else if (!(collided & bitmask(i)) && !o[i].no_frame) {
            ep_add(ep, L - 1, ACT_RX, i, 0);
        }
    }
    ep_add(ep, end, ACT_END, 0, 0);
    ep->idle_at = ep_time(ep, end);
    qsort(ep->act, ep->nact, sizeof(ep->act[0]), act_cmp);
}

static void twai_ep_schedule(ESP32P4TwaiEpisode *ep)
{
    for (uint32_t i = 0; i < ep->nact; i++) {
        if (act_key(&ep->act[i]) > ep->done_key) {
            timer_mod(ep->timer, ep_time(ep, ep->act[i].boundary));
            return;
        }
    }
    timer_del(ep->timer);
}

/*
 * The frame's end: the bus idle, failed single-shot transmissions given
 * up, error passive transmitters suspended for 8 bits (TRM table
 * 56.2-4), and the controllers waiting to integrate synchronised.
 */
static void twai_ep_end(ESP32P4TwaiState *s, int seg)
{
    ESP32P4TwaiEpisode *ep = &s->ep[seg];
    int64_t idle = ep->idle_at;
    uint8_t alive = ep->members & ~ep->departed;

    ep->active = false;
    timer_del(ep->timer);
    s->seg_idle_at[seg] = idle;
    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];

        if (alive & bitmask(i)) {
            c->status &= ~(ST_RS | ST_TS);
            if ((ep->contenders & bitmask(i)) && c->tx_pending &&
                c->tx_single_shot) {
                twai_tx_release(c, false);
            }
            if ((ep->contenders & bitmask(i)) && ep->lost_bit[i] < 0 &&
                c->err_state == ERR_PASSIVE) {
                c->ready_at = idle + twai_bits_ns(c, 8);
            }
        }
        if (twai_seg(c) != seg) {
            continue;
        }
        if (c->sync_at != INT64_MAX && c->sync_at > ep->t_sof) {
            c->sync_at = MAX(c->sync_at, idle);
        }
        twai_bus_activity(c, false, idle);
        twai_recov_resume(c, idle - twai_bits_ns(c, 11));
    }
    twai_kick(s, seg);
}

static void twai_ep_act(ESP32P4TwaiState *s, int seg, ESP32P4TwaiAction *a)
{
    ESP32P4TwaiEpisode *ep = &s->ep[seg];
    ESP32P4TwaiCtrl *c = &s->ctrl[a->ctrl];
    ESP32P4TwaiCtrl *w = &s->ctrl[ep->winner];
    qemu_can_frame frame;
    bool acked;

    if (a->kind != ACT_END && a->kind != ACT_PROBE &&
        (ep->departed & bitmask(a->ctrl))) {
        return;
    }
    switch (a->kind) {
    case ACT_ALI:
        if (!c->alc_held) {
            c->alc = a->arg;
            c->alc_held = true;
        }
        twai_raise(c, INT_ALI);
        c->status = (c->status & ~ST_TS) | ST_RS;
        if (c->tx_single_shot && c->tx_pending) {
            twai_tx_release(c, false);
        }
        break;
    case ACT_ERR:
        twai_bus_error(c, a->arg & 0xff, a->arg & 0x100);
        break;
    case ACT_REC8:
        twai_count(c, 0, 8);
        break;
    case ACT_PROBE:
        acked = ep->ackers & ~ep->departed;
        if (seg < ESP32P4_TWAI_NUM && w->canbus &&
            !(ep->departed & bitmask(ep->winner))) {
            twai_frame_to_qemu(ep->frame, &frame);
            s->sending = true;
            if (can_bus_client_send(&w->client, &frame, 1) > 0) {
                acked = true;
            }
            s->sending = false;
        }
        ep->ack_state = acked ? 1 : 2;
        twai_ep_plan(s, seg);
        break;
    case ACT_RX:
        if (!a->arg && !(c->mode & MODE_LOM)) {
            c->rec = c->rec > 127 ? 127 : c->rec ? c->rec - 1 : 0;
            twai_update_error_state(c);
        }
        twai_store(c, ep->frame, ep->t_sof);
        if (!a->arg) {
            c->status &= ~ST_RS;
        }
        break;
    case ACT_TX_OK:
        if (c->tec) {
            c->tec--;
            twai_update_error_state(c);
        }
        c->status &= ~ST_TS;
        twai_tx_release(c, true);
        break;
    case ACT_END:
        twai_ep_end(s, seg);
        break;
    }
}

static void twai_ep_start(ESP32P4TwaiState *s, int seg);

static void twai_ep_timer(void *opaque)
{
    ESP32P4TwaiEpisode *ep = opaque;
    ESP32P4TwaiState *s = ep->s;
    int64_t now = twai_now();

    if (!ep->active) {
        if (ep->start_pending && now >= ep->start_at) {
            twai_ep_start(s, ep->seg);
        }
        return;
    }
    while (ep->active) {
        ESP32P4TwaiAction a;
        uint32_t i;

        for (i = 0; i < ep->nact; i++) {
            if (act_key(&ep->act[i]) > ep->done_key) {
                break;
            }
        }
        if (i == ep->nact) {
            break;
        }
        if (ep_time(ep, ep->act[i].boundary) > now) {
            twai_ep_schedule(ep);
            return;
        }
        a = ep->act[i];
        ep->done_key = act_key(&a);
        twai_ep_act(s, ep->seg, &a);
    }
}

/* Whether the controller has a transmission it may start. */
static bool twai_wants_tx(ESP32P4TwaiCtrl *c)
{
    return c->tx_pending && twai_operating(c) &&
           c->err_state != ERR_BUS_OFF && !c->local_active;
}

/* Whether the controller takes part in a frame starting now. */
static bool twai_listening(ESP32P4TwaiCtrl *c, int64_t now)
{
    return twai_operating(c) && c->err_state != ERR_BUS_OFF &&
           !c->local_active && c->sync_at <= now;
}

static void twai_local_start(ESP32P4TwaiCtrl *c, int64_t t0);

/*
 * Start the bus's next frame when the first controller with a
 * transmission may, on the bus's bit grid from when it last went idle: at
 * the bus idle for a controller already waiting, otherwise at the first
 * bit boundary after its request.
 */
static void twai_kick(ESP32P4TwaiState *s, int seg)
{
    ESP32P4TwaiEpisode *ep = &s->ep[seg];
    int64_t now = twai_now(), earliest = INT64_MAX, bit = 0, origin, n;

    if (ep->active) {
        return;
    }
    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];
        int64_t t;

        if (twai_seg(c) != seg || !twai_wants_tx(c) ||
            c->sync_at == INT64_MAX) {
            continue;
        }
        t = MAX(MAX(s->seg_idle_at[seg], c->tx_req_at + 1),
                MAX(c->sync_at, c->ready_at));
        if (!c->drives) {
            twai_local_start(c, MAX(t, now));
            continue;
        }
        t = MAX(t, now);
        if (t < earliest) {
            earliest = t;
            bit = twai_bit_ps(c);
        }
    }
    if (earliest == INT64_MAX) {
        ep->start_pending = false;
        timer_del(ep->timer);
        return;
    }
    origin = s->seg_idle_at[seg];
    if (earliest - origin > INT64_MAX / 1000000) {
        /* Idle for hours: the grid's phase no longer fits the arithmetic. */
        origin = earliest;
    }
    n = ((earliest - origin) * 1000 + bit - 1) / bit;
    ep->start_pending = true;
    ep->start_at = origin + bits_ns(bit, n);
    timer_mod(ep->timer, ep->start_at);
}

/*
 * SOF: the contenders ready now arbitrate bit by bit. A contender sending
 * recessive where the bus is dominant loses in the arbitration field and
 * receives; after it, its frame differs from another's identifier, and it
 * has a bit error.
 */
/* [spec:nuos:req:emu.esp32p4.twai] */
static void twai_ep_start(ESP32P4TwaiState *s, int seg)
{
    ESP32P4TwaiEpisode *ep = &s->ep[seg];
    int64_t now = twai_now();
    TwaiBits b[ESP32P4_TWAI_NUM];
    uint8_t live = 0, members = 0;
    unsigned maxlen = 0;
    int i;

    ep->start_pending = false;
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];

        if (twai_seg(c) != seg || !twai_listening(c, now)) {
            continue;
        }
        members |= bitmask(i);
        if (twai_wants_tx(c) && c->drives && c->ready_at <= now &&
            c->tx_req_at < now) {
            live |= bitmask(i);
            twai_encode(c->txbuf, &b[i]);
            maxlen = MAX(maxlen, b[i].len);
        }
    }
    if (!live) {
        twai_kick(s, seg);
        return;
    }

    ep->members = members;
    ep->contenders = live;
    ep->departed = 0;
    ep->passive = 0;
    ep->tx_stop = -1;
    ep->ack_state = 0;
    ep->done_key = -1;
    ep->t_sof = now;
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ep->lost_bit[i] = -1;
        ep->coll_bit[i] = -1;
        if ((members & bitmask(i)) && s->ctrl[i].err_state != ERR_ACTIVE) {
            ep->passive |= bitmask(i);
        }
    }
    for (unsigned k = 0; k < maxlen && ctpop8(live) > 1; k++) {
        bool bus = true, destroyed = false;

        for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
            if ((live & bitmask(i)) && k < b[i].len && !b[i].v[k]) {
                bus = false;
            }
        }
        for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
            if (!(live & bitmask(i)) || bus || (k < b[i].len && !b[i].v[k])) {
                continue;
            }
            live &= ~bitmask(i);
            if (k < b[i].len && b[i].alc[k] >= 0) {
                ep->lost_bit[i] = k;
                ep->lost_alc[i] = b[i].alc[k];
            } else {
                ep->coll_bit[i] = k;
                ep->coll_ecc[i] = ECC(ECC_BIT, false,
                                      b[i].seg[MIN(k, b[i].len - 1)]);
                destroyed |= !(ep->passive & bitmask(i));
            }
        }
        if (destroyed) {
            break;
        }
    }
    ep->survivors = live;
    ep->winner = ctz32(live);
    memcpy(ep->frame, s->ctrl[ep->winner].txbuf, sizeof(ep->frame));
    ep->bit_ps = twai_bit_ps(&s->ctrl[ep->winner]);
    ep->active = true;

    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];

        if (members & bitmask(i)) {
            c->status |= (ep->contenders & bitmask(i)) ? ST_TS : ST_RS;
        }
        if (twai_seg(c) == seg) {
            twai_bus_activity(c, true, now);
            twai_recov_pause(c, now);
        }
    }
    twai_ep_plan(s, seg);
    twai_ep_schedule(ep);
}

/*
 * The controller leaves the frame on its bus: entering reset mode, reset,
 * its clock stopped or its pads rerouted. A transmitter leaving before its
 * frame ends leaves the bus recessive from that bit; until the frame is
 * destroyed, the rest is laid out again without the controller.
 */
static void twai_depart(ESP32P4TwaiCtrl *c)
{
    ESP32P4TwaiState *s = c->s;

    if (c->local_active) {
        c->local_active = false;
        timer_del(c->local_timer);
        c->status &= ~ST_TS;
    }
    for (int seg = 0; seg < ESP32P4_TWAI_SEGS; seg++) {
        ESP32P4TwaiEpisode *ep = &s->ep[seg];
        int64_t cur;

        if (!ep->active || !(ep->members & ~ep->departed & bitmask(c->idx))) {
            continue;
        }
        cur = (twai_now() - ep->t_sof) * 1000 / ep->bit_ps;
        ep->departed |= bitmask(c->idx);
        c->status &= ~(ST_RS | ST_TS);
        if (cur < ep->destroy_at) {
            if (!(ep->survivors & ep->members & ~ep->departed) &&
                ep->tx_stop < 0) {
                ep->tx_stop = cur;
            }
            if (ep->survivors & ep->members & ~ep->departed) {
                ep->winner = ctz32(ep->survivors & ep->members &
                                   ~ep->departed);
            }
            twai_ep_plan(s, seg);
            twai_ep_schedule(ep);
        }
    }
}

/*
 * A transmission that cannot reach a bus: the controller does not hear
 * its own SOF, a bit error. While error active it signals with an active
 * flag, and each flag bit it does not hear is another bit error (TRM
 * 56.2.3.4, TEC rule 2), until it is error passive; then a passive flag,
 * error delimiter, intermission and Suspend Transmission, and it tries
 * again unless the transmission was single-shot.
 */
static void twai_local_start(ESP32P4TwaiCtrl *c, int64_t t0)
{
    if (c->local_active) {
        return;
    }
    c->local_active = true;
    c->local_phase = LOCAL_SOF;
    c->local_bit = 0;
    c->local_t0 = t0 + twai_bits_ns(c, 1);
    c->status |= ST_TS;
    twai_bus_activity(c, true, t0);
    timer_mod(c->local_timer, c->local_t0);
}

static void twai_local_try(ESP32P4TwaiCtrl *c)
{
    if (c->view != VIEW_CONST || !twai_wants_tx(c) ||
        c->sync_at == INT64_MAX) {
        return;
    }
    twai_local_start(c, MAX(MAX(twai_now(), c->sync_at), c->ready_at));
}

/* [spec:nuos:req:emu.esp32p4.twai] */
static void twai_local_timer(void *opaque)
{
    ESP32P4TwaiCtrl *c = opaque;
    int seg;

    switch (c->local_phase) {
    case LOCAL_SOF:
    case LOCAL_FLAG:
        twai_bus_error(c, c->local_phase == LOCAL_SOF ?
                       ECC(ECC_BIT, false, SEG_SOF) :
                       ECC(ECC_BIT, false, SEG_ACTIVE_FLAG), false);
        if (!c->local_active) {
            return;
        }
        if (c->err_state == ERR_ACTIVE) {
            c->local_phase = LOCAL_FLAG;
            c->local_t0 += twai_bits_ns(c, 1);
        } else {
            c->local_phase = LOCAL_TAIL;
            c->local_t0 += twai_bits_ns(c, 6 + 8 + 3);
        }
        timer_mod(c->local_timer, c->local_t0);
        return;
    case LOCAL_TAIL:
        c->local_active = false;
        c->status &= ~ST_TS;
        if (c->tx_pending && c->tx_single_shot) {
            twai_tx_release(c, false);
        }
        c->ready_at = c->local_t0 +
                      (c->err_state == ERR_PASSIVE ? twai_bits_ns(c, 8) : 0);
        twai_bus_activity(c, false, c->local_t0);
        seg = twai_seg(c);
        if (seg >= 0) {
            twai_kick(c->s, seg);
        }
        twai_local_try(c);
        return;
    }
}

/* Try to transmit: on the bus, or failing alone. */
static void twai_tx_kick(ESP32P4TwaiCtrl *c)
{
    int seg = twai_seg(c);


    if (seg >= 0) {
        twai_kick(c->s, seg);
    } else {
        twai_local_try(c);
    }
}

/*
 * Reset mode
 * ==========
 */

static void twai_enter_reset_mode(ESP32P4TwaiCtrl *c)
{
    if (c->mode & MODE_RM) {
        return;
    }
    c->mode |= MODE_RM;
    twai_depart(c);
    if (c->recovering) {
        twai_recov_fold(c, twai_now());
        c->recovering = false;
        timer_del(c->recov_timer);
    }
    c->tx_pending = false;
    c->status = (c->status | ST_TBS) & ~(ST_RS | ST_TS | ST_DOS);
    twai_fifo_clear(c);
    c->ir &= ~(INT_TI | INT_DOI);
    c->sync_at = INT64_MAX;
    twai_update_irq(c);
}

static void twai_leave_reset_mode(ESP32P4TwaiCtrl *c)
{
    c->mode &= ~MODE_RM;
    if (c->counters_written) {
        c->counters_written = false;
        if (c->tec == 255) {
            /* A bus-off the CPU forced, as a bus error would. */
            c->busoff_clear = false;
            c->tec = 127;
            c->rec = 0;
            c->err_state = ERR_BUS_OFF;
            c->mode |= MODE_RM;
            twai_update_error_state(c);
            return;
        }
        if (c->busoff_clear) {
            c->busoff_clear = false;
            c->err_state = c->tec >= 128 || c->rec >= 128 ? ERR_PASSIVE
                                                         : ERR_ACTIVE;
        }
    }
    if (c->err_state == ERR_BUS_OFF) {
        c->recovering = true;
        c->recov_paused = true;
        if (twai_operating(c) && twai_hears_idle(c)) {
            twai_recov_resume(c, twai_now());
        }
        return;
    }
    twai_update_error_state(c);
    twai_integrate(c);
    twai_tx_kick(c);
}

/*
 * Commands
 * ========
 */

static bool twai_tx_in_progress(ESP32P4TwaiCtrl *c)
{
    int seg = twai_seg(c);

    if (c->local_active) {
        return true;
    }
    return seg >= 0 && c->s->ep[seg].active &&
           (c->s->ep[seg].contenders & ~c->s->ep[seg].departed &
            bitmask(c->idx)) && c->s->ep[seg].lost_bit[c->idx] < 0;
}

/* [spec:nuos:req:emu.esp32p4.twai] */
static void twai_command(ESP32P4TwaiCtrl *c, uint32_t cmd)
{
    if (c->mode & MODE_RM) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: command 0x%x in reset "
                      "mode ignored\n", TYPE_ESP32P4_TWAI, c->idx, cmd);
        return;
    }
    if (cmd & CMD_RRB) {
        twai_fifo_release(c);
    }
    if (cmd & CMD_CDO) {
        c->status &= ~ST_DOS;
    }
    if (cmd & (CMD_TR | CMD_SRR)) {
        if (c->mode & MODE_LOM) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: transmission "
                          "request in listen-only mode ignored\n",
                          TYPE_ESP32P4_TWAI, c->idx);
        } else if (!(c->status & ST_TBS)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: transmission "
                          "request while the transmit buffer is locked "
                          "ignored\n", TYPE_ESP32P4_TWAI, c->idx);
        } else {
            c->tx_pending = true;
            c->tx_req_at = twai_now();
            /* TR with AT is a single-shot transmission (TRM 56.4.3.1). */
            c->tx_single_shot = cmd & CMD_AT;
            /* With TR, SRR does nothing. */
            c->tx_self_rx = !(cmd & CMD_TR);
            c->status &= ~(ST_TBS | ST_TCS);
            twai_tx_kick(c);
        }
    } else if ((cmd & CMD_AT) && c->tx_pending) {
        /* A transmission under way is not retried; one waiting is gone. */
        if (twai_tx_in_progress(c)) {
            c->tx_single_shot = true;
        } else {
            twai_tx_release(c, false);
        }
    }
}

/*
 * Registers
 * =========
 */

static bool twai_config_write(ESP32P4TwaiCtrl *c, hwaddr addr)
{
    if (c->mode & MODE_RM) {
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: write to 0x%" HWADDR_PRIx
                  " outside reset mode ignored\n", TYPE_ESP32P4_TWAI,
                  c->idx, addr);
    return false;
}

static uint32_t twai_read(ESP32P4TwaiCtrl *c, hwaddr addr)
{
    uint32_t v;

    switch (addr) {
    case A_MODE:
        return c->mode;
    case A_CMD:
        return 0;
    case A_STATUS:
        twai_recov_fold(c, twai_now());
        return twai_status(c);
    case A_INT:
        v = c->ir | ((c->rmc && (c->ier & INT_RI)) ? INT_RI : 0);
        c->ir = 0;
        twai_update_irq(c);
        return v;
    case A_INT_ENA:
        return c->ier;
    case A_BTR0:
        return c->btr0;
    case A_BTR1:
        return c->btr1;
    case A_ALC:
        c->alc_held = false;
        return c->alc;
    case A_ECC:
        c->ecc_held = false;
        return c->ecc;
    case A_EWL:
        return c->ewl;
    case A_RXERR:
        return c->rec;
    case A_TXERR:
        twai_recov_fold(c, twai_now());
        return c->tec & 0xff;
    case A_DATA0 ... A_DATA12:
        if (!(c->mode & MODE_RM)) {
            return c->fifo[(c->fifo_rd + (addr - A_DATA0) / 4) %
                           ESP32P4_TWAI_FIFO_SIZE];
        }
        if (addr >= A_DATA8) {
            return 0;
        }
        return addr < A_DATA0 + 0x10 ? c->acr[(addr - A_DATA0) / 4]
                                     : c->amr[(addr - A_DATA0 - 0x10) / 4];
    case A_RMC:
        return c->rmc;
    case A_CDR:
        return c->cdr;
    case A_SW_STANDBY:
        return c->sw_standby;
    case A_HW_CFG:
        return c->hw_cfg;
    case A_HW_STANDBY_CNT:
        return c->hw_standby_cnt;
    case A_IDLE_INTR_CNT:
        return c->idle_intr_cnt;
    case A_ECO_CFG:
        return c->eco_cfg | ECO_RDN_RESULT;
    case A_TS_DATA:
        return c->msg_ts[c->msg_rd];
    case A_TS_PRESCALER:
        return c->ts_prescaler;
    case A_TS_CFG:
        return c->ts_cfg;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: read at 0x%" HWADDR_PRIx
                  ", which has no register\n", TYPE_ESP32P4_TWAI, c->idx,
                  addr);
    return 0;
}

/* [spec:nuos:req:emu.esp32p4.twai] */
static void twai_write(ESP32P4TwaiCtrl *c, hwaddr addr, uint32_t v)
{
    uint32_t old;

    switch (addr) {
    case A_MODE:
        if (c->mode & MODE_RM) {
            c->mode = MODE_RM | (v & (MODE_LOM | MODE_STM | MODE_AFM));
            if (!(v & MODE_RM)) {
                twai_leave_reset_mode(c);
            }
            return;
        }
        if ((v ^ c->mode) & (MODE_LOM | MODE_STM | MODE_AFM)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: mode bits change "
                          "only in reset mode\n", TYPE_ESP32P4_TWAI, c->idx);
        }
        if (v & MODE_RM) {
            twai_enter_reset_mode(c);
        }
        return;
    case A_CMD:
        twai_command(c, v & 0x1f);
        return;
    case A_INT_ENA:
        c->ier = v & INT_ENA_MASK;
        twai_update_irq(c);
        return;
    case A_BTR0:
        if (twai_config_write(c, addr)) {
            c->btr0 = v & 0xffff;
        }
        return;
    case A_BTR1:
        if (twai_config_write(c, addr)) {
            c->btr1 = v & 0xff;
        }
        return;
    case A_EWL:
        if (twai_config_write(c, addr)) {
            c->ewl = v & 0xff;
        }
        return;
    case A_RXERR:
        /* The SJA1000 ignores REC writes while bus-off. */
        if (twai_config_write(c, addr) && c->err_state != ERR_BUS_OFF) {
            c->rec = v & 0xff;
            c->counters_written = true;
        }
        return;
    case A_TXERR:
        if (twai_config_write(c, addr)) {
            c->tec = v & 0xff;
            c->counters_written = true;
            if (c->err_state == ERR_BUS_OFF && c->tec != 255) {
                c->busoff_clear = true;
            }
        }
        return;
    case A_DATA0 ... A_DATA12:
        if (!(c->mode & MODE_RM)) {
            if (!(c->status & ST_TBS)) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: write to the "
                              "locked transmit buffer ignored\n",
                              TYPE_ESP32P4_TWAI, c->idx);
                return;
            }
            c->txbuf[(addr - A_DATA0) / 4] = v;
        } else if (addr < A_DATA0 + 0x10) {
            c->acr[(addr - A_DATA0) / 4] = v;
        } else if (addr < A_DATA8) {
            c->amr[(addr - A_DATA0 - 0x10) / 4] = v;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: 0x%" HWADDR_PRIx
                          " is read-only in reset mode\n",
                          TYPE_ESP32P4_TWAI, c->idx, addr);
        }
        return;
    case A_CDR:
        old = c->cdr;
        c->cdr = (c->cdr & CDR_CLOCK_OFF) | (v & CDR_CD);
        if (((v ^ old) & CDR_CLOCK_OFF) && twai_config_write(c, addr)) {
            c->cdr = (c->cdr & ~CDR_CLOCK_OFF) | (v & CDR_CLOCK_OFF);
        }
        twai_update_lines(c);
        return;
    case A_SW_STANDBY:
        old = c->sw_standby;
        c->sw_standby = v & (SW_STANDBY_EN | SW_STANDBY_CLR);
        if (c->sw_standby & SW_STANDBY_CLR) {
            twai_set_standby(c, false);
        } else if ((c->sw_standby & ~old) & SW_STANDBY_EN) {
            twai_set_standby(c, true);
        } else {
            twai_standby_schedule(c);
        }
        return;
    case A_HW_CFG:
        c->hw_cfg = v & 1;
        twai_standby_schedule(c);
        return;
    case A_HW_STANDBY_CNT:
        c->hw_standby_cnt = v;
        twai_standby_schedule(c);
        return;
    case A_IDLE_INTR_CNT:
        c->idle_intr_cnt = v;
        return;
    case A_ECO_CFG:
        c->eco_cfg = v & ECO_RDN_ENA;
        return;
    case A_TS_PRESCALER:
        twai_ts_rebase(c);
        c->ts_prescaler = v & 0xffff;
        twai_ts_schedule(c);
        return;
    case A_TS_CFG:
        if ((v ^ c->ts_cfg) & TS_ENABLE) {
            c->ts_count_base = 0;
            c->ts_base_ns = twai_now();
        }
        c->ts_cfg = v & TS_ENABLE;
        twai_ts_schedule(c);
        return;
    case A_STATUS:
    case A_INT:
    case A_ALC:
    case A_ECC:
    case A_RMC:
    case A_TS_DATA:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: write to read-only "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_TWAI, c->idx,
                      addr);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: TWAI%d: write at 0x%" HWADDR_PRIx
                  ", which has no register\n", TYPE_ESP32P4_TWAI, c->idx,
                  addr);
}

static uint64_t esp32p4_twai_mmio_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    return extract32(twai_read(opaque, addr & ~3), (addr & 3) * 8,
                     size * 8);
}

static void esp32p4_twai_mmio_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    ESP32P4TwaiCtrl *c = opaque;
    unsigned shift = (addr & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t cur = 0;

    /* A narrow write merges with what the register holds. */
    switch (addr & ~3) {
    case A_MODE:
        cur = c->mode;
        break;
    case A_INT_ENA:
        cur = c->ier;
        break;
    case A_BTR0:
        cur = c->btr0;
        break;
    case A_CDR:
        cur = c->cdr;
        break;
    case A_SW_STANDBY:
        cur = c->sw_standby;
        break;
    case A_HW_STANDBY_CNT:
        cur = c->hw_standby_cnt;
        break;
    case A_IDLE_INTR_CNT:
        cur = c->idle_intr_cnt;
        break;
    case A_TS_PRESCALER:
        cur = c->ts_prescaler;
        break;
    }
    twai_write(c, addr & ~3, (cur & ~mask) | ((value << shift) & mask));
}

static const MemoryRegionOps esp32p4_twai_ops = {
    .read = esp32p4_twai_mmio_read,
    .write = esp32p4_twai_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * The QEMU CAN bus
 * ================
 */

static bool esp32p4_twai_can_receive(CanBusClientState *client)
{
    ESP32P4TwaiCtrl *c = container_of(client, ESP32P4TwaiCtrl, client);

    return !c->s->sending && c->view == VIEW_BUS && twai_operating(c);
}

/*
 * A frame from another client of the bus, which has just ended: it is
 * received as one of the device's own would be, and acknowledged by a
 * controller that drives the bus and is not in listen-only mode.
 */
/* [spec:nuos:req:emu.esp32p4.twai] */
static ssize_t esp32p4_twai_receive(CanBusClientState *client,
                                    const qemu_can_frame *frames,
                                    size_t frames_cnt)
{
    ESP32P4TwaiCtrl *c = container_of(client, ESP32P4TwaiCtrl, client);
    ESP32P4TwaiState *s = c->s;
    const qemu_can_frame *q = frames;
    int64_t now = twai_now(), t_sof;
    int seg = twai_seg(c);
    uint8_t f[13];
    TwaiBits b;

    if (s->sending || !frames_cnt || seg < 0 ||
        (q->can_id & QEMU_CAN_ERR_FLAG)) {
        return 0;
    }
    twai_bus_activity(c, false, now + twai_bits_ns(c, 3));
    if (c->standby) {
        twai_set_standby(c, false);
    }
    if (c->recovering) {
        twai_recov_pause(c, now);
        twai_recov_resume(c, now - twai_bits_ns(c, 8));
        return 0;
    }
    s->seg_idle_at[seg] = MAX(s->seg_idle_at[seg], now + twai_bits_ns(c, 3));
    if (s->ep[seg].start_pending &&
        s->ep[seg].start_at < s->seg_idle_at[seg]) {
        s->ep[seg].start_pending = false;
        twai_kick(s, seg);
    }
    if (!twai_listening(c, now)) {
        return 0;
    }
    if ((q->flags & QEMU_CAN_FRMF_TYPE_FD) || q->can_dlc > 8) {
        /*
         * The FDF bit is recessive where a classical frame's r0 (SFF) or
         * r1 (EFF) is dominant.
         */
        twai_bus_error(c, ECC(ECC_FORM, true, (q->can_id & QEMU_CAN_EFF_FLAG)
                              ? SEG_R1 : SEG_R0), false);
        return 0;
    }
    twai_frame_from_qemu(q, f);
    twai_encode(f, &b);
    t_sof = now - twai_bits_ns(c, b.len);
    if (!(c->mode & MODE_LOM)) {
        c->rec = c->rec > 127 ? 127 : c->rec ? c->rec - 1 : 0;
        twai_update_error_state(c);
    }
    twai_store(c, f, t_sof);
    return c->drives && !(c->mode & MODE_LOM) ? 1 : 0;
}

static CanBusClientInfo esp32p4_twai_client_info = {
    .can_receive = esp32p4_twai_can_receive,
    .receive = esp32p4_twai_receive,
};

/*
 * Pads and clocks
 * ===============
 */

static void twai_compute_view(ESP32P4TwaiCtrl *c, uint8_t *view,
                              bool *drives)
{
    ESP32P4GpioState *gpio = c->s->gpio;
    uint64_t tx_pads = esp32p4_gpio_sig_out_pads(gpio, twai_sig_tx[c->idx]);
    unsigned src;
    bool inv;
    int pad;

    *drives = tx_pads != 0;
    if (esp32p4_gpio_sig_in_source(gpio, twai_sig_rx[c->idx], &src, &inv)) {
        *view = src == twai_sig_tx[c->idx] && !inv ? VIEW_LOOPBACK
                                                   : VIEW_CONST;
        return;
    }
    pad = esp32p4_gpio_sig_in_pad(gpio, twai_sig_rx[c->idx]);
    if (pad < 0) {
        *view = VIEW_CONST;
    } else if (c->canbus) {
        *view = VIEW_BUS;
    } else {
        *view = (tx_pads >> pad) & 1 ? VIEW_LOOPBACK : VIEW_CONST;
    }
}

/* The controller's view of the bus changed: it leaves and joins again. */
static void twai_rejoin(ESP32P4TwaiCtrl *c)
{
    int old_seg = twai_seg(c);

    twai_depart(c);
    if (old_seg >= 0) {
        twai_kick(c->s, old_seg);
    }
    twai_integrate(c);
    if (c->recovering) {
        twai_recov_pause(c, twai_now());
        if (twai_operating(c) && twai_hears_idle(c)) {
            twai_recov_resume(c, twai_now());
        }
    }
    twai_tx_kick(c);
}

/* [spec:nuos:req:emu.esp32p4.twai] */
static void esp32p4_twai_route_changed(Notifier *n, void *data)
{
    ESP32P4TwaiState *s = container_of(n, ESP32P4TwaiState, route_notifier);

    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];
        uint8_t view;
        bool drives;

        twai_compute_view(c, &view, &drives);
        if (view != c->view || drives != c->drives) {
            twai_depart(c);
            c->view = view;
            c->drives = drives;
            twai_rejoin(c);
        }
    }
}

static void esp32p4_twai_rx_in(void *opaque, int n, int level)
{
    ESP32P4TwaiCtrl *c = &((ESP32P4TwaiState *)opaque)->ctrl[n];

    if (c->rx_level == !!level) {
        return;
    }
    c->rx_level = level;
    if (c->view == VIEW_CONST) {
        twai_rejoin(c);
    }
}

static void esp32p4_twai_line_sync(Notifier *n, void *data)
{
    ESP32P4TwaiState *s = container_of(n, ESP32P4TwaiState, line_source);

    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        twai_update_lines(&s->ctrl[i]);
    }
}

/* [spec:nuos:req:emu.esp32p4.twai] */
static void esp32p4_twai_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4TwaiCtrl *c = opaque;

    if (event == ClockPreUpdate) {
        twai_ts_rebase(c);
        twai_recov_fold(c, twai_now());
        return;
    }
    c->ts_base_ns = twai_now();
    twai_ts_schedule(c);
    twai_rejoin(c);
    twai_standby_schedule(c);
    twai_update_lines(c);
}

/*
 * Reset
 * =====
 */

static void twai_ctrl_reset(ESP32P4TwaiCtrl *c)
{
    int64_t now = twai_now();

    twai_depart(c);
    timer_del(c->recov_timer);
    timer_del(c->local_timer);
    timer_del(c->ts_timer);
    timer_del(c->standby_timer);
    c->mode = MODE_RM;
    c->status = ST_TBS | ST_TCS;
    c->ir = 0;
    c->ier = 0;
    c->btr0 = 0;
    c->btr1 = 0;
    c->alc = 0;
    c->ecc = 0;
    c->ewl = 96;
    c->rec = 0;
    c->tec = 0;
    memset(c->acr, 0, sizeof(c->acr));
    memset(c->amr, 0, sizeof(c->amr));
    memset(c->txbuf, 0, sizeof(c->txbuf));
    c->cdr = 0;
    c->sw_standby = SW_STANDBY_CLR;
    c->hw_cfg = 0;
    c->hw_standby_cnt = 1;
    c->idle_intr_cnt = 1;
    c->eco_cfg = 0;
    c->ts_prescaler = 31;
    c->ts_cfg = 0;
    memset(c->fifo, 0, sizeof(c->fifo));
    memset(c->msg_len, 0, sizeof(c->msg_len));
    memset(c->msg_valid, 0, sizeof(c->msg_valid));
    memset(c->msg_ts, 0, sizeof(c->msg_ts));
    twai_fifo_clear(c);
    c->alc_held = false;
    c->ecc_held = false;
    c->tx_pending = false;
    c->tx_single_shot = false;
    c->tx_self_rx = false;
    c->tx_req_at = 0;
    c->err_state = ERR_ACTIVE;
    c->counters_written = false;
    c->busoff_clear = false;
    c->sync_at = INT64_MAX;
    c->ready_at = 0;
    c->recovering = false;
    c->recov_paused = false;
    c->recov_mark = 0;
    c->local_active = false;
    c->ts_count_base = 0;
    c->ts_base_ns = now;
    c->standby = false;
    c->idle_since = now;
}

static void esp32p4_twai_reset_in(void *opaque, int n, int level)
{
    ESP32P4TwaiCtrl *c = &((ESP32P4TwaiState *)opaque)->ctrl[n];

    c->in_reset = level;
    twai_ctrl_reset(c);
    twai_update_irq(c);
    twai_update_lines(c);
}

static void esp32p4_twai_reset_hold(Object *obj, ResetType type)
{
    ESP32P4TwaiState *s = ESP32P4_TWAI(obj);

    for (int seg = 0; seg < ESP32P4_TWAI_SEGS; seg++) {
        s->ep[seg].active = false;
        s->ep[seg].start_pending = false;
        s->ep[seg].nact = 0;
        timer_del(s->ep[seg].timer);
        s->seg_idle_at[seg] = 0;
    }
    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        s->ctrl[i].in_reset = false;
        twai_ctrl_reset(&s->ctrl[i]);
    }
    s->sending = false;
}

static void esp32p4_twai_reset_exit(Object *obj, ResetType type)
{
    ESP32P4TwaiState *s = ESP32P4_TWAI(obj);

    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        twai_update_irq(&s->ctrl[i]);
        twai_update_lines(&s->ctrl[i]);
    }
}

/*
 * The device
 * ==========
 */

static void esp32p4_twai_init(Object *obj)
{
    ESP32P4TwaiState *s = ESP32P4_TWAI(obj);
    DeviceState *dev = DEVICE(obj);
    int i;

    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];
        g_autofree char *name = g_strdup_printf("esp32p4.twai%d", i);
        g_autofree char *clk = g_strdup_printf("twai%d", i);
        g_autofree char *link = g_strdup_printf("canbus%d", i);

        c->s = s;
        c->idx = i;
        memory_region_init_io(&c->iomem, obj, &esp32p4_twai_ops, c, name,
                              ESP32P4_TWAI_SIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &c->iomem);
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &c->irq);
        c->clk = qdev_init_clock_in(dev, clk, esp32p4_twai_clk_update, c,
                                    ClockPreUpdate | ClockUpdate);
        object_property_add_link(obj, link, TYPE_CAN_BUS,
                                 (Object **)&c->canbus,
                                 qdev_prop_allow_set_link_before_realize, 0);
        c->recov_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, twai_recov_timer, c);
        c->local_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, twai_local_timer, c);
        c->ts_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, twai_ts_timer, c);
        c->standby_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        twai_standby_timer, c);
        c->rx_level = true;
    }
    for (i = 0; i < ESP32P4_TWAI_SEGS; i++) {
        s->ep[i].s = s;
        s->ep[i].seg = i;
        s->ep[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, twai_ep_timer,
                                      &s->ep[i]);
    }
    object_property_add_link(obj, "gpio", TYPE_ESP32P4_GPIO,
                             (Object **)&s->gpio,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    qdev_init_gpio_in_named(dev, esp32p4_twai_reset_in, ESP32P4_TWAI_RESET,
                            ESP32P4_TWAI_NUM);
    qdev_init_gpio_in_named(dev, esp32p4_twai_rx_in, ESP32P4_TWAI_RX,
                            ESP32P4_TWAI_NUM);
    /* Each call adds the next line of the group: line i is controller i. */
    for (i = 0; i < ESP32P4_TWAI_NUM; i++) {
        qdev_init_gpio_out_named(dev, &s->ctrl[i].tx_out, ESP32P4_TWAI_TX, 1);
        qdev_init_gpio_out_named(dev, &s->ctrl[i].bus_off_out,
                                 ESP32P4_TWAI_BUS_OFF, 1);
        qdev_init_gpio_out_named(dev, &s->ctrl[i].clkout_out,
                                 ESP32P4_TWAI_CLKOUT, 1);
        qdev_init_gpio_out_named(dev, &s->ctrl[i].standby_out,
                                 ESP32P4_TWAI_STANDBY, 1);
    }
    s->route_notifier.notify = esp32p4_twai_route_changed;
    s->line_source.notify = esp32p4_twai_line_sync;
}

static void esp32p4_twai_realize(DeviceState *dev, Error **errp)
{
    ESP32P4TwaiState *s = ESP32P4_TWAI(dev);

    if (!s->gpio) {
        error_setg(errp, "%s: 'gpio' link not set", TYPE_ESP32P4_TWAI);
        return;
    }
    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        ESP32P4TwaiCtrl *c = &s->ctrl[i];

        s->bus_id[i] = -1;
        for (int j = 0; j <= i && c->canbus; j++) {
            if (s->ctrl[j].canbus == c->canbus) {
                s->bus_id[i] = j;
                break;
            }
        }
        if (c->canbus) {
            c->client.info = &esp32p4_twai_client_info;
            if (can_bus_insert_client(c->canbus, &c->client) < 0) {
                error_setg(errp, "%s: cannot join TWAI%d to its CAN bus",
                           TYPE_ESP32P4_TWAI, i);
                return;
            }
        }
        twai_compute_view(c, &c->view, &c->drives);
    }
    esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
    esp32_line_add_source(&s->line_source);
}

static int esp32p4_twai_post_load(void *opaque, int version_id)
{
    ESP32P4TwaiState *s = opaque;

    for (int i = 0; i < ESP32P4_TWAI_NUM; i++) {
        twai_update_irq(&s->ctrl[i]);
        twai_update_lines(&s->ctrl[i]);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_twai_action = {
    .name = "esp32p4-twai-action",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT32(boundary, ESP32P4TwaiAction),
        VMSTATE_UINT8(kind, ESP32P4TwaiAction),
        VMSTATE_INT8(ctrl, ESP32P4TwaiAction),
        VMSTATE_UINT16(arg, ESP32P4TwaiAction),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_twai_episode = {
    .name = "esp32p4-twai-episode",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, ESP32P4TwaiEpisode),
        VMSTATE_BOOL(start_pending, ESP32P4TwaiEpisode),
        VMSTATE_INT64(start_at, ESP32P4TwaiEpisode),
        VMSTATE_INT64(t_sof, ESP32P4TwaiEpisode),
        VMSTATE_INT64(bit_ps, ESP32P4TwaiEpisode),
        VMSTATE_UINT8(members, ESP32P4TwaiEpisode),
        VMSTATE_UINT8(departed, ESP32P4TwaiEpisode),
        VMSTATE_UINT8(passive, ESP32P4TwaiEpisode),
        VMSTATE_UINT8(ackers, ESP32P4TwaiEpisode),
        VMSTATE_INT32(destroy_at, ESP32P4TwaiEpisode),
        VMSTATE_UINT8(contenders, ESP32P4TwaiEpisode),
        VMSTATE_INT16_ARRAY(lost_bit, ESP32P4TwaiEpisode, ESP32P4_TWAI_NUM),
        VMSTATE_UINT8_ARRAY(lost_alc, ESP32P4TwaiEpisode, ESP32P4_TWAI_NUM),
        VMSTATE_INT16_ARRAY(coll_bit, ESP32P4TwaiEpisode, ESP32P4_TWAI_NUM),
        VMSTATE_UINT8_ARRAY(coll_ecc, ESP32P4TwaiEpisode, ESP32P4_TWAI_NUM),
        VMSTATE_UINT8(survivors, ESP32P4TwaiEpisode),
        VMSTATE_INT8(winner, ESP32P4TwaiEpisode),
        VMSTATE_UINT8_ARRAY(frame, ESP32P4TwaiEpisode, 13),
        VMSTATE_INT16(tx_stop, ESP32P4TwaiEpisode),
        VMSTATE_UINT8(ack_state, ESP32P4TwaiEpisode),
        VMSTATE_INT64(done_key, ESP32P4TwaiEpisode),
        VMSTATE_UINT32(nact, ESP32P4TwaiEpisode),
        VMSTATE_STRUCT_ARRAY(act, ESP32P4TwaiEpisode, ESP32P4_TWAI_ACTIONS,
                             1, vmstate_esp32p4_twai_action,
                             ESP32P4TwaiAction),
        VMSTATE_INT64(idle_at, ESP32P4TwaiEpisode),
        VMSTATE_TIMER_PTR(timer, ESP32P4TwaiEpisode),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_twai_ctrl = {
    .name = "esp32p4-twai-ctrl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mode, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(status, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(ir, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(ier, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(btr0, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(btr1, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(alc, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(ecc, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(ewl, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(rec, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(tec, ESP32P4TwaiCtrl),
        VMSTATE_UINT8_ARRAY(acr, ESP32P4TwaiCtrl, 4),
        VMSTATE_UINT8_ARRAY(amr, ESP32P4TwaiCtrl, 4),
        VMSTATE_UINT8_ARRAY(txbuf, ESP32P4TwaiCtrl, 13),
        VMSTATE_UINT32(cdr, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(sw_standby, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(hw_cfg, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(hw_standby_cnt, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(idle_intr_cnt, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(eco_cfg, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(ts_prescaler, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(ts_cfg, ESP32P4TwaiCtrl),
        VMSTATE_UINT8_ARRAY(fifo, ESP32P4TwaiCtrl, ESP32P4_TWAI_FIFO_SIZE),
        VMSTATE_UINT32(fifo_rd, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(fifo_used, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(rmc, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(msg_rd, ESP32P4TwaiCtrl),
        VMSTATE_UINT8_ARRAY(msg_len, ESP32P4TwaiCtrl, ESP32P4_TWAI_MAX_MSGS),
        VMSTATE_BOOL_ARRAY(msg_valid, ESP32P4TwaiCtrl, ESP32P4_TWAI_MAX_MSGS),
        VMSTATE_UINT32_ARRAY(msg_ts, ESP32P4TwaiCtrl, ESP32P4_TWAI_MAX_MSGS),
        VMSTATE_BOOL(doi_armed, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(alc_held, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(ecc_held, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(tx_pending, ESP32P4TwaiCtrl),
        VMSTATE_INT64(tx_req_at, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(tx_single_shot, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(tx_self_rx, ESP32P4TwaiCtrl),
        VMSTATE_UINT8(err_state, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(counters_written, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(busoff_clear, ESP32P4TwaiCtrl),
        VMSTATE_INT64(sync_at, ESP32P4TwaiCtrl),
        VMSTATE_INT64(ready_at, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(recovering, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(recov_paused, ESP32P4TwaiCtrl),
        VMSTATE_INT64(recov_mark, ESP32P4TwaiCtrl),
        VMSTATE_TIMER_PTR(recov_timer, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(local_active, ESP32P4TwaiCtrl),
        VMSTATE_UINT8(local_phase, ESP32P4TwaiCtrl),
        VMSTATE_UINT32(local_bit, ESP32P4TwaiCtrl),
        VMSTATE_INT64(local_t0, ESP32P4TwaiCtrl),
        VMSTATE_TIMER_PTR(local_timer, ESP32P4TwaiCtrl),
        VMSTATE_UINT8(view, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(drives, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(rx_level, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(in_reset, ESP32P4TwaiCtrl),
        VMSTATE_UINT64(ts_count_base, ESP32P4TwaiCtrl),
        VMSTATE_INT64(ts_base_ns, ESP32P4TwaiCtrl),
        VMSTATE_TIMER_PTR(ts_timer, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(standby, ESP32P4TwaiCtrl),
        VMSTATE_INT64(idle_since, ESP32P4TwaiCtrl),
        VMSTATE_TIMER_PTR(standby_timer, ESP32P4TwaiCtrl),
        VMSTATE_BOOL(clkout_level, ESP32P4TwaiCtrl),
        VMSTATE_CLOCK(clk, ESP32P4TwaiCtrl),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_twai = {
    .name = TYPE_ESP32P4_TWAI,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32p4_twai_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ctrl, ESP32P4TwaiState, ESP32P4_TWAI_NUM, 1,
                             vmstate_esp32p4_twai_ctrl, ESP32P4TwaiCtrl),
        VMSTATE_STRUCT_ARRAY(ep, ESP32P4TwaiState, ESP32P4_TWAI_SEGS, 1,
                             vmstate_esp32p4_twai_episode,
                             ESP32P4TwaiEpisode),
        VMSTATE_INT64_ARRAY(seg_idle_at, ESP32P4TwaiState,
                            ESP32P4_TWAI_SEGS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_twai_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_twai_realize;
    dc->vmsd = &vmstate_esp32p4_twai;
    rc->phases.hold = esp32p4_twai_reset_hold;
    rc->phases.exit = esp32p4_twai_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.twai] */
static const TypeInfo esp32p4_twai_info = {
    .name          = TYPE_ESP32P4_TWAI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4TwaiState),
    .instance_init = esp32p4_twai_init,
    .class_init    = esp32p4_twai_class_init,
};

static void esp32p4_twai_register_types(void)
{
    type_register_static(&esp32p4_twai_info);
}
type_init(esp32p4_twai_register_types)
