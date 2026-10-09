/*
 * ESP32 I2S controller, with its DMA engine and LCD and camera modes
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM chapter 22 "I2S Controller" and section 2.6 "I2S
 * DMA Interface", with the register layout and reset values of ESP-IDF's
 * soc/esp32 i2s_reg.h and i2s_struct.h, and the way ESP-IDF's
 * hal/esp32 i2s_ll.h drives the block.
 *
 * Each controller has a transmitter and a receiver, each with a FIFO of 64
 * 32-bit words, and a DMA engine with an outlink filling the transmit FIFO
 * and an inlink emptying the receive FIFO through lldesc_t descriptor
 * lists.
 *
 * I2S_CLK is PLL_F160M_CLK or APLL_CLK (CLKM_CONF.CLKA_ENA) divided by
 * N + b/a (CLKM_DIV_NUM, CLKM_DIV_B, CLKM_DIV_A); a master's BCK is I2S_CLK
 * divided by M (TX_BCK_DIV_NUM or RX_BCK_DIV_NUM), and a frame of the I2S
 * modes is two channels of BITS_MOD BCK cycles each. Data moves at those
 * rates on QEMU_CLOCK_VIRTUAL.
 *
 * The serial line is modelled bit by bit, BCK cycle by BCK cycle, with the
 * word select of the Philips (MSB_SHIFT), MSB-aligned and PCM short-sync
 * (SHORT_SYNC) formats, so a receiver decodes what a transmitter sends
 * whatever their settings. A master runs a whole frame of BCK cycles at a
 * time, at the end of the frame, without driving each cycle onto the
 * pads: it passes the bits to the units it clocks, which are its own
 * receiver with SIG_LOOPBACK, and any slave unit of either controller whose
 * BCK and WS inputs the GPIO matrix connects to its BCK and WS outputs;
 * a receiver whose data input the matrix connects to the data output of a
 * transmitter on the same clock receives that transmitter's bits, and
 * otherwise the level of its data input. A slave not clocked by either
 * controller follows the edges of its BCK input from the pads, driving
 * and sampling its data line on each one.
 *
 * An audio backend (the machine's audiodev) stands for a codec on the I2S
 * bus: it plays the frames a master transmitter sends, in the standard and
 * PDM modes, and gives a master receiver the frames it records when no
 * transmitter of the chip drives the receiver's data input.
 *
 * In LCD mode the transmitter drives its frame data on the parallel bus
 * I2SnO_DATA_out[23:24-BITS] with WS as the write strobe, one BCK cycle
 * per edge; the receiver samples I2SnI_Data_in[15:0] once per WS period as
 * master (the ADC mode's interface) or, in camera mode, on each rising edge
 * of I2SnI_WS_in (the camera's PCLK) while H_SYNC, V_SYNC and H_ENABLE are
 * high. I2S0's LCD mode also carries the built-in converters: with
 * SYSCON_SARADC_DATA_TO_I2S the receiver takes the SAR ADC DIG
 * controllers' results in place of the bus, and with
 * SENS_SAR_DAC_DIG_FORCE the transmitter's data feeds the DACs.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/audio/esp32_i2s.h"
#include "hw/dma/esp32_lldesc.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/misc/esp32_apb_ctrl.h"
#include "hw/misc/esp32_sens.h"
#include "migration/vmstate.h"

#define A_FIFO_WR               0x00
#define A_FIFO_RD               0x04
#define A_CONF                  0x08
#define A_INT_RAW               0x0c
#define A_INT_ST                0x10
#define A_INT_ENA               0x14
#define A_INT_CLR               0x18
#define A_TIMING                0x1c
#define A_FIFO_CONF             0x20
#define A_RXEOF_NUM             0x24
#define A_CONF_SINGLE_DATA      0x28
#define A_CONF_CHAN             0x2c
#define A_OUT_LINK              0x30
#define A_IN_LINK               0x34
#define A_OUT_EOF_DES_ADDR      0x38
#define A_IN_EOF_DES_ADDR       0x3c
#define A_OUT_EOF_BFR_DES_ADDR  0x40
#define A_AHB_TEST              0x44
#define A_INLINK_DSCR           0x48
#define A_INLINK_DSCR_BF0       0x4c
#define A_INLINK_DSCR_BF1       0x50
#define A_OUTLINK_DSCR          0x54
#define A_OUTLINK_DSCR_BF0      0x58
#define A_OUTLINK_DSCR_BF1      0x5c
#define A_LC_CONF               0x60
#define A_OUTFIFO_PUSH          0x64
#define A_INFIFO_POP            0x68
#define A_LC_STATE0             0x6c
#define A_LC_STATE1             0x70
#define A_LC_HUNG_CONF          0x74
#define A_CVSD_CONF0            0x80
#define A_CVSD_CONF2            0x88
#define A_PLC_CONF0             0x8c
#define A_PLC_CONF2             0x94
#define A_ESCO_CONF0            0x98
#define A_SCO_CONF0             0x9c
#define A_CONF1                 0xa0
#define A_PD_CONF               0xa4
#define A_CONF2                 0xa8
#define A_CLKM_CONF             0xac
#define A_SAMPLE_RATE_CONF      0xb0
#define A_PDM_CONF              0xb4
#define A_PDM_FREQ_CONF         0xb8
#define A_STATE                 0xbc
#define A_DATE                  0xfc

/* CONF */
#define CONF_TX_RESET           (1u << 0)
#define CONF_RX_RESET           (1u << 1)
#define CONF_TX_FIFO_RESET      (1u << 2)
#define CONF_RX_FIFO_RESET      (1u << 3)
#define CONF_TX_START           (1u << 4)
#define CONF_RX_START           (1u << 5)
#define CONF_TX_SLAVE_MOD       (1u << 6)
#define CONF_RX_SLAVE_MOD       (1u << 7)
#define CONF_TX_RIGHT_FIRST     (1u << 8)
#define CONF_RX_RIGHT_FIRST     (1u << 9)
#define CONF_TX_MSB_SHIFT       (1u << 10)
#define CONF_RX_MSB_SHIFT       (1u << 11)
#define CONF_TX_SHORT_SYNC      (1u << 12)
#define CONF_RX_SHORT_SYNC      (1u << 13)
#define CONF_TX_MONO            (1u << 14)
#define CONF_RX_MONO            (1u << 15)
#define CONF_TX_MSB_RIGHT       (1u << 16)
#define CONF_RX_MSB_RIGHT       (1u << 17)
#define CONF_SIG_LOOPBACK       (1u << 18)
#define CONF_MASK               0x0007ffffu
#define CONF_RESET              0x00030300u

/* INT_RAW, INT_ST, INT_ENA and INT_CLR */
#define INT_RX_TAKE_DATA        (1u << 0)
#define INT_TX_PUT_DATA         (1u << 1)
#define INT_RX_WFULL            (1u << 2)
#define INT_RX_REMPTY           (1u << 3)
#define INT_TX_WFULL            (1u << 4)
#define INT_TX_REMPTY           (1u << 5)
#define INT_RX_HUNG             (1u << 6)
#define INT_TX_HUNG             (1u << 7)
#define INT_IN_DONE             (1u << 8)
#define INT_IN_SUC_EOF          (1u << 9)
/* IN_ERR_EOF: the error EOF of the UHCI's decoder; I2S never raises it */
#define INT_IN_ERR_EOF          (1u << 10)
#define INT_OUT_DONE            (1u << 11)
#define INT_OUT_EOF             (1u << 12)
#define INT_IN_DSCR_ERR         (1u << 13)
#define INT_OUT_DSCR_ERR        (1u << 14)
#define INT_IN_DSCR_EMPTY       (1u << 15)
#define INT_OUT_TOTAL_EOF       (1u << 16)
#define INT_MASK                0x0001ffffu

#define TIMING_TX_BCK_IN_INV    (1u << 24)
#define TIMING_MASK             0x01ffffffu

/* FIFO_CONF */
#define FIFO_CONF_RX_DATA_NUM(v)    ((v) & 0x3f)
#define FIFO_CONF_TX_DATA_NUM(v)    (((v) >> 6) & 0x3f)
#define FIFO_CONF_DSCR_EN           (1u << 12)
#define FIFO_CONF_TX_FIFO_MOD(v)    (((v) >> 13) & 7)
#define FIFO_CONF_RX_FIFO_MOD(v)    (((v) >> 16) & 7)
#define FIFO_CONF_TX_FORCE_EN       (1u << 19)
#define FIFO_CONF_RX_FORCE_EN       (1u << 20)
#define FIFO_CONF_MASK              0x001fffffu
#define FIFO_CONF_RESET             0x00001820u

#define RXEOF_NUM_RESET         64

#define CONF_CHAN_TX(v)         ((v) & 7)
#define CONF_CHAN_RX(v)         (((v) >> 3) & 3)
#define CONF_CHAN_MASK          0x1fu

/* OUT_LINK and IN_LINK */
#define LINK_ADDR_MASK          0x000fffffu
#define LINK_STOP               (1u << 28)
#define LINK_START              (1u << 29)
#define LINK_RESTART            (1u << 30)
#define LINK_PARK               (1u << 31)

#define AHB_TEST_MASK           0x37u

/* LC_CONF */
#define LC_CONF_IN_RST          (1u << 0)
#define LC_CONF_OUT_RST         (1u << 1)
#define LC_CONF_AHBM_FIFO_RST   (1u << 2)
#define LC_CONF_AHBM_RST        (1u << 3)
#define LC_CONF_OUT_LOOP_TEST   (1u << 4)
#define LC_CONF_IN_LOOP_TEST    (1u << 5)
#define LC_CONF_OUT_AUTO_WRBACK (1u << 6)
#define LC_CONF_OUT_NO_RESTART_CLR (1u << 7)
#define LC_CONF_OUT_EOF_MODE    (1u << 8)
#define LC_CONF_CHECK_OWNER     (1u << 12)
#define LC_CONF_MEM_TRANS_EN    (1u << 13)
#define LC_CONF_MASK            0x00003fffu
#define LC_CONF_RESET           LC_CONF_OUT_EOF_MODE

#define OUTFIFO_PUSH_MASK       0x000101ffu
#define INFIFO_POP_POP          (1u << 16)

/* LC_HUNG_CONF */
#define HUNG_TIMEOUT(v)         ((v) & 0xff)
#define HUNG_SHIFT(v)           (((v) >> 8) & 7)
#define HUNG_ENA                (1u << 11)
#define HUNG_MASK               0x00000fffu
#define HUNG_RESET              0x00000810u
/* The timeout's tick counter wraps at 88000 >> SHIFT APB_CLK cycles. */
#define HUNG_TICK_CYCLES        88000

/* CVSD, PLC, ESCO and SCO: the Bluetooth voice codec's registers */
#define CVSD_CONF0_RESET        0x80007fffu
#define CVSD_CONF1_RESET        0x000a0500u
#define CVSD_CONF2_RESET        0x000502a4u
#define CVSD_CONF2_MASK         0x0007ffffu
#define PLC_CONF0_RESET         0x08a80339u
#define PLC_CONF0_MASK          0x0fffffffu
#define PLC_CONF1_RESET         0xa0178a05u
#define PLC_CONF2_RESET         0x00000028u
#define PLC_CONF2_MASK          0x0000007fu
#define ESCO_CONF0_MASK         0x00001fffu
#define ESCO_CONF0_EN           (1u << 0)
#define SCO_CONF0_MASK          0x0000000fu
#define SCO_CONF0_EN            0x00000003u

/* CONF1 */
#define CONF1_TX_PCM_CONF(v)    ((v) & 7)
#define CONF1_TX_PCM_BYPASS     (1u << 3)
#define CONF1_RX_PCM_CONF(v)    (((v) >> 4) & 7)
#define CONF1_RX_PCM_BYPASS     (1u << 7)
#define CONF1_TX_STOP_EN        (1u << 8)
#define CONF1_TX_ZEROS_RM_EN    (1u << 9)
#define CONF1_MASK              0x000003ffu
#define CONF1_RESET             0x00000089u

#define PD_CONF_FIFO_FORCE_PD   (1u << 0)
#define PD_CONF_MASK            0x0000000fu
#define PD_CONF_RESET           0x0000000au

/* CONF2 */
#define CONF2_CAMERA_EN         (1u << 0)
#define CONF2_LCD_TX_WRX2_EN    (1u << 1)
#define CONF2_LCD_TX_SDX2_EN    (1u << 2)
#define CONF2_DATA_ENABLE_TEST_EN (1u << 3)
#define CONF2_DATA_ENABLE       (1u << 4)
#define CONF2_LCD_EN            (1u << 5)
#define CONF2_EXT_ADC_START_EN  (1u << 6)
#define CONF2_INTER_VALID_EN    (1u << 7)
#define CONF2_MASK              0x000000ffu

/* CLKM_CONF */
#define CLKM_DIV_NUM(v)         ((v) & 0xff)
#define CLKM_DIV_B(v)           (((v) >> 8) & 0x3f)
#define CLKM_DIV_A(v)           (((v) >> 14) & 0x3f)
#define CLKM_CLKA_ENA           (1u << 21)
#define CLKM_MASK               0x003fffffu
#define CLKM_RESET              4

/* SAMPLE_RATE_CONF */
#define SR_TX_BCK_DIV(v)        ((v) & 0x3f)
#define SR_RX_BCK_DIV(v)        (((v) >> 6) & 0x3f)
#define SR_TX_BITS(v)           (((v) >> 12) & 0x3f)
#define SR_RX_BITS(v)           (((v) >> 18) & 0x3f)
#define SR_MASK                 0x00ffffffu
#define SR_RESET                0x00410186u

/* PDM_CONF and PDM_FREQ_CONF */
#define PDM_TX_EN               (1u << 0)
#define PDM_RX_EN               (1u << 1)
#define PDM_PCM2PDM_CONV_EN     (1u << 2)
#define PDM_PDM2PCM_CONV_EN     (1u << 3)
#define PDM_RX_SINC_DSR_16_EN   (1u << 24)
#define PDM_CONF_MASK           0x03ffffffu
#define PDM_CONF_RESET          0x01550020u
#define PDM_FREQ_FS(v)          ((v) & 0x3ff)
#define PDM_FREQ_FP(v)          (((v) >> 10) & 0x3ff)
#define PDM_FREQ_MASK           0x000fffffu
#define PDM_FREQ_RESET          ((960u << 10) | 480u)

/* STATE */
#define STATE_TX_IDLE           (1u << 0)

#define DATE_RESET              0x01604201u

/* APB_CTRL (SYSCON) SARADC_CTRL: the SAR ADC feeds I2S0 */
#define SARADC_DATA_TO_I2S      (1u << 26)

/* Model tags of transmit FIFO entries */
#define TAG_EOF                 (1u << 0)
#define TAG_TOTAL_EOF           (1u << 1)

/* Bound on descriptors walked without moving data: a loop of empty ones */
#define MAX_EMPTY_DSCRS         256
/* Bound on the events one timer callback runs to catch up */
#define MAX_CATCH_UP            65536

/* What a transmitter or receiver does, from CONF, CONF2 and PDM_CONF */
enum {
    MODE_OFF,
    MODE_MASTER,        /* I2S/PCM master: generates BCK and WS */
    MODE_SLAVE,         /* I2S/PCM slave: follows BCK and WS */
    MODE_PDM,           /* PDM, master, I2S0 only */
    MODE_LCD,           /* LCD master: parallel bus, WS as strobe */
    MODE_CAMERA,        /* receiver only: camera slave, parallel input */
};

/* GPIO matrix signal numbers (TRM table 6.9-1, ESP-IDF gpio_sig_map.h) */
static const struct {
    uint8_t o_bck, o_ws, i_bck, i_ws, data, h_sync, v_sync, h_enable;
} i2s_sig[ESP32_I2S_COUNT] = {
    { 23, 25, 27, 28, 140, 190, 191, 192 },
    { 24, 26, 164, 165, 166, 193, 194, 195 },
};

unsigned esp32_i2s_out_signal(unsigned id, Esp32I2sOutLine line)
{
    switch (line) {
    case ESP32_I2S_OUT_O_BCK:
        return i2s_sig[id].o_bck;
    case ESP32_I2S_OUT_O_WS:
        return i2s_sig[id].o_ws;
    case ESP32_I2S_OUT_I_BCK:
        return i2s_sig[id].i_bck;
    case ESP32_I2S_OUT_I_WS:
        return i2s_sig[id].i_ws;
    default:
        return i2s_sig[id].data + (line - ESP32_I2S_OUT_DATA0);
    }
}

unsigned esp32_i2s_in_signal(unsigned id, Esp32I2sInLine line)
{
    switch (line) {
    case ESP32_I2S_IN_O_BCK:
        return i2s_sig[id].o_bck;
    case ESP32_I2S_IN_O_WS:
        return i2s_sig[id].o_ws;
    case ESP32_I2S_IN_I_BCK:
        return i2s_sig[id].i_bck;
    case ESP32_I2S_IN_I_WS:
        return i2s_sig[id].i_ws;
    case ESP32_I2S_IN_H_SYNC:
        return i2s_sig[id].h_sync;
    case ESP32_I2S_IN_V_SYNC:
        return i2s_sig[id].v_sync;
    case ESP32_I2S_IN_H_ENABLE:
        return i2s_sig[id].h_enable;
    default:
        return i2s_sig[id].data + (line - ESP32_I2S_IN_DATA0);
    }
}

/* The serial data lines: DATA_out23 and DATA_in15 */
#define OUT_SD                  (ESP32_I2S_OUT_DATA0 + 23)
#define IN_SD                   (ESP32_I2S_IN_DATA0 + 15)

static void i2s_kick(Esp32I2sState *s);
static void i2s_update_units(Esp32I2sState *s, bool reanchor);

/* ---- Interrupts ---- */

static void i2s_update_irq(Esp32I2sState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static void i2s_raise(Esp32I2sState *s, uint32_t mask)
{
    s->int_raw |= mask;
    i2s_update_irq(s);
}

/* ---- Clocks ---- */

static bool i2s_running(Esp32I2sState *s)
{
    return clock_is_enabled(s->apb_clk);
}

/*
 * The BCK period of a master, in ns: I2S_CLK = source / (N + b/a), BCK =
 * I2S_CLK / M. The TRM requires N >= 2 and M >= 2; smaller values are
 * taken as 2. A zero denominator a leaves the fraction out.
 */
static double i2s_bck_ns(Esp32I2sState *s, bool tx)
{
    uint32_t n = CLKM_DIV_NUM(s->clkm_conf);
    uint32_t a = CLKM_DIV_A(s->clkm_conf);
    uint32_t b = CLKM_DIV_B(s->clkm_conf);
    uint32_t m = tx ? SR_TX_BCK_DIV(s->sample_rate_conf)
                    : SR_RX_BCK_DIV(s->sample_rate_conf);
    uint64_t src_hz;
    double div;

    if (s->clkm_conf & CLKM_CLKA_ENA) {
        src_hz = clock_get_hz(s->apll_clk);
    } else {
        src_hz = clock_get_hz(s->f160m_clk);
    }
    if (src_hz == 0) {
        return 0;
    }
    if (n < 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u CLKM_DIV_NUM %u is "
                      "below 2\n", s->id, n);
        n = 2;
    }
    if (m < 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u %s_BCK_DIV_NUM %u "
                      "is below 2\n", s->id, tx ? "TX" : "RX", m);
        m = 2;
    }
    div = n + (a ? (double)b / a : 0.0);
    return 1e9 * div * m / src_hz;
}

/* A channel's bit count, BITS_MOD; the hardware supports 8 to 32. */
static unsigned i2s_bits(Esp32I2sState *s, bool tx)
{
    unsigned bits = tx ? SR_TX_BITS(s->sample_rate_conf)
                       : SR_RX_BITS(s->sample_rate_conf);

    if (bits == 0 || bits > 32) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u %s_BITS_MOD %u is "
                      "out of range\n", s->id, tx ? "TX" : "RX", bits);
        return bits ? 32 : 16;
    }
    return bits;
}

/* ---- A-law (G.711) compression ---- */

static uint8_t alaw_encode(int16_t pcm)
{
    int v = pcm >> 3;
    uint8_t sign = 0x80;
    int seg = 0;

    if (v < 0) {
        sign = 0;
        v = -v - 1;
    }
    if (v > 0xfff) {
        v = 0xfff;
    }
    while (seg < 7 && v >= (0x20 << seg)) {
        seg++;
    }
    if (seg == 0) {
        v >>= 1;
    } else {
        v >>= seg;
    }
    return (sign | (seg << 4) | (v & 0xf)) ^ 0x55;
}

static int16_t alaw_decode(uint8_t a)
{
    int seg, v;

    a ^= 0x55;
    seg = (a >> 4) & 7;
    v = ((a & 0xf) << 4) + 8;
    if (seg) {
        v = (v + 0x100) << (seg - 1);
    }
    return (a & 0x80) ? v : -v;
}

/*
 * The A-law module between the FIFO and the shifter. CONF 0 decompresses
 * an 8-bit code in the sample's top byte into a 16-bit sample, CONF 1
 * compresses the sample's top 16 bits into an 8-bit code. Samples are
 * MSB-aligned in 32 bits.
 */
static uint32_t i2s_pcm(Esp32I2sState *s, uint32_t v, bool tx)
{
    uint32_t bypass = tx ? CONF1_TX_PCM_BYPASS : CONF1_RX_PCM_BYPASS;
    uint32_t conf = tx ? CONF1_TX_PCM_CONF(s->conf1)
                       : CONF1_RX_PCM_CONF(s->conf1);

    if (s->conf1 & bypass) {
        return v;
    }
    switch (conf) {
    case 0:
        return (uint32_t)(uint16_t)alaw_decode(v >> 24) << 16;
    case 1:
        return (uint32_t)alaw_encode((int16_t)(v >> 16)) << 24;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32_i2s: I2S%u %s_PCM_CONF %u is not "
                      "documented; data bypasses the A-law module\n",
                      s->id, tx ? "TX" : "RX", conf);
        return v;
    }
}

/* ---- FIFOs ---- */

static void fifo_reset(Esp32I2sFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static bool fifo_full(Esp32I2sFifo *f)
{
    return f->num == ESP32_I2S_FIFO_DEPTH;
}

static void fifo_push(Esp32I2sFifo *f, uint32_t v, uint8_t tag,
                      uint32_t dscr, uint32_t buf)
{
    uint32_t i = (f->head + f->num) % ESP32_I2S_FIFO_DEPTH;

    f->data[i] = v;
    f->tag[i] = tag;
    f->tag_dscr[i] = dscr;
    f->tag_buf[i] = buf;
    f->num++;
}

static uint32_t fifo_pop(Esp32I2sFifo *f, uint8_t *tag, uint32_t *dscr,
                         uint32_t *buf)
{
    uint32_t i = f->head;

    f->head = (f->head + 1) % ESP32_I2S_FIFO_DEPTH;
    f->num--;
    if (tag) {
        *tag = f->tag[i];
        *dscr = f->tag_dscr[i];
        *buf = f->tag_buf[i];
    }
    return f->data[i];
}

/* The tail entry, the one pushed last */
static uint32_t fifo_tail(Esp32I2sFifo *f)
{
    return (f->head + f->num - 1) % ESP32_I2S_FIFO_DEPTH;
}

static void i2s_tx_resume(Esp32I2sState *s);

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * A word enters the transmit FIFO from the CPU or the outlink. While
 * TX_FIFO_RESET is set the FIFO is held empty.
 */
static bool i2s_tx_fifo_push(Esp32I2sState *s, uint32_t v)
{
    if (s->conf & CONF_TX_FIFO_RESET) {
        return false;
    }
    if (fifo_full(&s->tx_fifo)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u transmit FIFO "
                      "overflow\n", s->id);
        i2s_raise(s, INT_TX_WFULL);
        return false;
    }
    fifo_push(&s->tx_fifo, v, 0, 0, 0);
    if (fifo_full(&s->tx_fifo)) {
        i2s_raise(s, INT_TX_WFULL);
    }
    i2s_tx_resume(s);
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * A word leaves the transmit FIFO for the shifter. The outlink refills
 * the FIFO; PUT_DATA flags a level below TX_DATA_NUM, REMPTY a FIFO left
 * empty. With OUT_EOF_MODE set the last word of an eof descriptor raises
 * OUT_EOF as it leaves.
 */
static uint32_t i2s_tx_fifo_pop(Esp32I2sState *s)
{
    uint32_t dscr, buf, v;
    uint8_t tag;

    v = fifo_pop(&s->tx_fifo, &tag, &dscr, &buf);
    if ((tag & TAG_EOF) && (s->lc_conf & LC_CONF_OUT_EOF_MODE)) {
        s->out_eof_des_addr = dscr;
        s->out_eof_bfr_des_addr = buf;
        i2s_raise(s, INT_OUT_EOF |
                  ((tag & TAG_TOTAL_EOF) ? INT_OUT_TOTAL_EOF : 0));
    }
    i2s_kick(s);
    if (s->tx_fifo.num < FIFO_CONF_TX_DATA_NUM(s->fifo_conf)) {
        i2s_raise(s, INT_TX_PUT_DATA);
    }
    if (s->tx_fifo.num == 0) {
        i2s_raise(s, INT_TX_REMPTY);
    }
    return v;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * A word enters the receive FIFO from the shifter. A full FIFO drops it.
 * TAKE_DATA flags a level above RX_DATA_NUM, WFULL a full FIFO.
 */
static void i2s_rx_fifo_push(Esp32I2sState *s, uint32_t v)
{
    if (s->conf & CONF_RX_FIFO_RESET) {
        return;
    }
    if (fifo_full(&s->rx_fifo)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u receive FIFO "
                      "overflow\n", s->id);
        i2s_raise(s, INT_RX_WFULL);
        return;
    }
    fifo_push(&s->rx_fifo, v, 0, 0, 0);
    if (fifo_full(&s->rx_fifo)) {
        i2s_raise(s, INT_RX_WFULL);
    }
    if (s->rx_fifo.num > FIFO_CONF_RX_DATA_NUM(s->fifo_conf)) {
        i2s_raise(s, INT_RX_TAKE_DATA);
    }
    i2s_kick(s);
}

/* ---- DMA ---- */

static void link_reset(Esp32I2sLink *l)
{
    memset(l, 0, sizeof(*l));
}

static bool i2s_dma_enabled(Esp32I2sState *s)
{
    return (s->fifo_conf & FIFO_CONF_DSCR_EN) && i2s_running(s);
}

static void i2s_link_show(Esp32I2sState *s, bool out)
{
    Esp32I2sLink *l = out ? &s->out : &s->in;

    if (out) {
        s->outlink_dscr = l->dscr;
        s->outlink_dscr_bf0 = l->next;
        s->outlink_dscr_bf1 = l->buf;
    } else {
        s->inlink_dscr = l->dscr;
        s->inlink_dscr_bf0 = l->next;
        s->inlink_dscr_bf1 = l->buf;
    }
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * Fetch the descriptor l->fetch. One outside DMA-capable SRAM or not
 * word-aligned, one the CPU owns while LC_CONF.CHECK_OWNER is set, or one
 * whose buffer runs outside DMA-capable SRAM is a descriptor error: the
 * link stops and raises OUT_DSCR_ERR or IN_DSCR_ERR.
 */
static bool i2s_link_load(Esp32I2sState *s, bool out)
{
    Esp32I2sLink *l = out ? &s->out : &s->in;
    Esp32Lldesc d;
    Esp32LldescResult r;
    static const char *const why[] = {
        [ESP32_LLDESC_BAD_ADDR] = "is not word-aligned DMA-capable SRAM",
        [ESP32_LLDESC_NOT_OWNED] = "is owned by the CPU",
        [ESP32_LLDESC_BAD_BUF] = "has a buffer outside DMA-capable SRAM",
    };

    r = esp32_lldesc_fetch(&s->dma_as, l->fetch, out,
                           s->lc_conf & LC_CONF_CHECK_OWNER, &d);
    l->dscr = d.addr;
    l->dw0 = d.dw0;
    l->buf = d.buf;
    l->next = d.next;
    l->pos = 0;
    i2s_link_show(s, out);
    if (r != ESP32_LLDESC_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u %slink descriptor "
                      "at 0x%08x %s\n", s->id, out ? "out" : "in", d.addr,
                      why[r]);
        l->active = false;
        l->have_desc = false;
        i2s_raise(s, out ? INT_OUT_DSCR_ERR : INT_IN_DSCR_ERR);
        return false;
    }
    l->have_desc = true;
    return true;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The outlink is done with a descriptor: all its length bytes are in the
 * transmit FIFO. OUT_DONE follows; with OUT_AUTO_WRBACK the descriptor
 * goes back to the CPU. An eof descriptor raises OUT_EOF, and OUT_TOTAL_EOF
 * if it ends the list, now or, with OUT_EOF_MODE, when its last word
 * leaves the FIFO. A list's last descriptor stops the link; RESTART goes
 * on from its next field.
 */
static void i2s_out_finish(Esp32I2sState *s, bool pushed)
{
    Esp32I2sLink *l = &s->out;
    bool eof = l->dw0 & ESP32_LLDESC_EOF;
    bool total = eof && l->next == 0;
    uint32_t ints = INT_OUT_DONE;

    if (s->lc_conf & LC_CONF_OUT_AUTO_WRBACK) {
        address_space_stl_le(&s->dma_as, l->dscr,
                             l->dw0 & ~ESP32_LLDESC_OWNER,
                             MEMTXATTRS_UNSPECIFIED, NULL);
    }
    if (eof) {
        if ((s->lc_conf & LC_CONF_OUT_EOF_MODE) && pushed) {
            uint32_t t = fifo_tail(&s->tx_fifo);

            s->tx_fifo.tag[t] = TAG_EOF | (total ? TAG_TOTAL_EOF : 0);
            s->tx_fifo.tag_dscr[t] = l->dscr;
            s->tx_fifo.tag_buf[t] = l->buf;
        } else {
            s->out_eof_des_addr = l->dscr;
            s->out_eof_bfr_des_addr = l->buf;
            ints |= INT_OUT_EOF | (total ? INT_OUT_TOTAL_EOF : 0);
        }
    }
    l->have_desc = false;
    l->have_last = true;
    l->last = l->dscr;
    if (l->next == 0) {
        l->active = false;
    } else {
        l->fetch = l->next;
    }
    i2s_raise(s, ints);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The outlink moves its descriptors' data into the transmit FIFO a word
 * at a time, as far as the FIFO has room. The I2S DMA moves whole words
 * (TRM 2.6); a length that is not a multiple of four ends in a word
 * padded with zeros.
 */
static bool i2s_out_run(Esp32I2sState *s)
{
    Esp32I2sLink *l = &s->out;
    bool progress = false;
    int empty = 0;

    while (l->active && !(s->conf & CONF_TX_FIFO_RESET)) {
        uint32_t len, n, w = 0;
        bool pushed = false;

        if (!l->have_desc && !i2s_link_load(s, true)) {
            break;
        }
        len = ESP32_LLDESC_LENGTH(l->dw0);
        while (l->pos < len && !fifo_full(&s->tx_fifo)) {
            n = MIN(4, len - l->pos);
            if (n < 4) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u outlink "
                              "descriptor 0x%08x length %u is not a whole "
                              "number of words\n", s->id, l->dscr, len);
            }
            w = 0;
            address_space_read(&s->dma_as, l->buf + l->pos,
                               MEMTXATTRS_UNSPECIFIED, &w, n);
            fifo_push(&s->tx_fifo, le32_to_cpu(w), 0, 0, 0);
            l->pos += n;
            pushed = true;
            progress = true;
        }
        if (pushed && fifo_full(&s->tx_fifo)) {
            i2s_raise(s, INT_TX_WFULL);
        }
        if (l->pos < len) {
            break;
        }
        if (len == 0 && ++empty > MAX_EMPTY_DSCRS) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u outlink loops "
                          "through empty descriptors\n", s->id);
            l->active = false;
            break;
        }
        i2s_out_finish(s, len != 0);
        progress = true;
    }
    if (progress) {
        i2s_tx_resume(s);
    }
    return progress;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The inlink hands a descriptor back to the CPU with the number of bytes
 * it received in its length, and eof set on the one that ends a packet of
 * RXEOF_NUM words.
 */
static void i2s_in_finish(Esp32I2sState *s, bool eof)
{
    Esp32I2sLink *l = &s->in;
    uint32_t ints = INT_IN_DONE;

    l->dw0 = (l->dw0 & (ESP32_LLDESC_RESERVED_MASK | 0xfffu)) |
             (MIN(l->pos, 0xfffu) << ESP32_LLDESC_LENGTH_SHIFT) |
             (eof ? ESP32_LLDESC_EOF : 0);
    address_space_stl_le(&s->dma_as, l->dscr, l->dw0,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    if (eof) {
        s->in_eof_des_addr = l->dscr;
        ints |= INT_IN_SUC_EOF;
    }
    l->have_desc = false;
    l->have_last = true;
    l->last = l->dscr;
    if (l->next == 0) {
        l->exhausted = true;
    } else {
        l->fetch = l->next;
    }
    i2s_raise(s, ints);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The inlink moves received words from the receive FIFO into its
 * descriptors' buffers. After RXEOF_NUM words it closes the descriptor in
 * hand with eof and raises IN_SUC_EOF. Data with no descriptor left to
 * take it raises IN_DSCR_EMPTY and waits in the FIFO.
 */
static bool i2s_in_run(Esp32I2sState *s)
{
    Esp32I2sLink *l = &s->in;
    bool progress = false;
    int empty = 0;

    while (l->active && s->rx_fifo.num) {
        uint32_t size, w, n, eof_num;

        if (l->exhausted) {
            if (!l->empty_flagged) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u inlink out "
                              "of descriptors\n", s->id);
                l->empty_flagged = true;
                i2s_raise(s, INT_IN_DSCR_EMPTY);
            }
            break;
        }
        if (!l->have_desc && !i2s_link_load(s, false)) {
            break;
        }
        size = ESP32_LLDESC_SIZE(l->dw0);
        if (l->pos >= size) {
            if (++empty > MAX_EMPTY_DSCRS) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u inlink "
                              "loops through empty descriptors\n", s->id);
                l->active = false;
                break;
            }
            i2s_in_finish(s, false);
            continue;
        }
        w = cpu_to_le32(fifo_pop(&s->rx_fifo, NULL, NULL, NULL));
        n = MIN(4, size - l->pos);
        if (n < 4) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u inlink "
                          "descriptor 0x%08x size %u is not a whole number "
                          "of words\n", s->id, l->dscr, size);
        }
        address_space_write(&s->dma_as, l->buf + l->pos,
                            MEMTXATTRS_UNSPECIFIED, &w, n);
        l->pos += n;
        progress = true;
        eof_num = s->rx_eof_num;
        if (eof_num == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u RXEOF_NUM is "
                          "0\n", s->id);
            eof_num = 1;
        }
        if (++s->rx_eof_count >= eof_num) {
            s->rx_eof_count = 0;
            i2s_in_finish(s, true);
        } else if (l->pos >= size) {
            i2s_in_finish(s, false);
        }
    }
    if (progress && s->rx_fifo.num == 0) {
        i2s_raise(s, INT_RX_REMPTY);
    }
    return progress;
}

/* ---- FIFO timeouts ---- */

static int64_t i2s_hung_ns(Esp32I2sState *s)
{
    uint64_t cycles = (uint64_t)HUNG_TIMEOUT(s->lc_hung_conf) *
                      (HUNG_TICK_CYCLES >> HUNG_SHIFT(s->lc_hung_conf));

    return clock_ticks_to_ns(s->apb_clk, cycles);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * LC_HUNG_CONF times the DMA engine out while it holds data it cannot
 * move: the outlink with data for a full transmit FIFO (TX_HUNG), the
 * receive FIFO with data and no inlink to take it (RX_HUNG). It fires once
 * per stall.
 */
static void i2s_hung_update(Esp32I2sState *s, QEMUTimer *t, bool *fired,
                            bool stalled, bool progress)
{
    if (!stalled || !(s->lc_hung_conf & HUNG_ENA) || !i2s_running(s)) {
        timer_del(t);
        *fired = false;
        return;
    }
    if (progress) {
        timer_del(t);
        *fired = false;
    }
    if (!*fired && !timer_pending(t)) {
        timer_mod_ns(t, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     i2s_hung_ns(s));
    }
}

static void i2s_tx_hung_cb(void *opaque)
{
    Esp32I2sState *s = opaque;

    s->tx_hung_fired = true;
    i2s_raise(s, INT_TX_HUNG);
}

static void i2s_rx_hung_cb(void *opaque)
{
    Esp32I2sState *s = opaque;

    s->rx_hung_fired = true;
    i2s_raise(s, INT_RX_HUNG);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * Move data between memory and the FIFOs as far as the links, the FIFOs
 * and I2S_DSCR_EN allow. A call from within a call repeats the run.
 */
static void i2s_kick(Esp32I2sState *s)
{
    bool out_progress = false, in_progress = false;

    if (s->in_kick) {
        s->kick_again = true;
        return;
    }
    s->in_kick = true;
    if (i2s_dma_enabled(s)) {
        do {
            s->kick_again = false;
            out_progress |= i2s_out_run(s);
            in_progress |= i2s_in_run(s);
        } while (s->kick_again);
    }
    s->in_kick = false;

    i2s_hung_update(s, &s->tx_hung_timer, &s->tx_hung_fired,
                    i2s_dma_enabled(s) && s->out.active &&
                    fifo_full(&s->tx_fifo), out_progress);
    i2s_hung_update(s, &s->rx_hung_timer, &s->rx_hung_fired,
                    i2s_dma_enabled(s) && s->rx_fifo.num &&
                    !s->in.active, in_progress);
    i2s_update_irq(s);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * OUTLINK_START / INLINK_START begin a list at the descriptor the link
 * register addresses, STOP stops the link where it is (and parks it).
 * RESTART resumes a stopped link at the descriptor in hand, or at the
 * next field of the last one it finished, so that software can append to
 * a list the link has run off.
 */
static void i2s_link_ctrl(Esp32I2sState *s, bool out, uint32_t val)
{
    Esp32I2sLink *l = out ? &s->out : &s->in;
    uint32_t rst = LC_CONF_AHBM_RST |
                   (out ? LC_CONF_OUT_RST : LC_CONF_IN_RST);
    uint32_t next;
    MemTxResult r;

    if (val & LINK_STOP) {
        if (l->active) {
            l->parked = true;
        }
        l->active = false;
    }
    if (!(val & (LINK_START | LINK_RESTART))) {
        return;
    }
    if (s->lc_conf & rst) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u %slink started "
                      "while held in reset\n", s->id, out ? "out" : "in");
        return;
    }
    if (val & LINK_START) {
        l->have_desc = false;
        l->exhausted = false;
        l->empty_flagged = false;
        l->fetch = ESP32_DMA_LINK_BASE | (val & LINK_ADDR_MASK);
        l->active = true;
        l->parked = false;
        if (!out) {
            s->rx_eof_count = 0;
        }
    } else if (!l->active || l->exhausted) {
        if (l->have_desc) {
            l->active = true;
        } else if (l->have_last) {
            next = address_space_ldl_le(&s->dma_as, l->last + 8,
                                        MEMTXATTRS_UNSPECIFIED, &r);
            if (r == MEMTX_OK && next) {
                l->fetch = next;
                l->exhausted = false;
                l->empty_flagged = false;
                l->active = true;
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u %slink "
                          "restarted with no descriptor to continue from\n",
                          s->id, out ? "out" : "in");
        }
        if (l->active) {
            l->parked = false;
        }
    }
    i2s_kick(s);
}

/* ---- Audio backend ---- */

static void i2s_audio_out_cb(void *opaque, int free)
{
    Esp32I2sState *s = opaque;
    int bytes = s->audio_out_bits / 8;
    int frame = 2 * bytes;
    uint8_t buf[4096];

    while (s->audio_out_num && free >= frame) {
        int n = MIN(MIN(s->audio_out_num, free / frame),
                    (int)sizeof(buf) / frame);
        size_t done;

        for (int i = 0; i < n; i++) {
            uint32_t k = (s->audio_out_head + i) % ESP32_I2S_AUDIO_FRAMES;

            for (int c = 0; c < 2; c++) {
                if (bytes == 2) {
                    stw_le_p(buf + i * frame + c * 2, s->audio_out[k][c]);
                } else {
                    stl_le_p(buf + i * frame + c * 4, s->audio_out[k][c]);
                }
            }
        }
        done = audio_be_write(s->audio_be, s->voice_out, buf, n * frame);
        n = done / frame;
        if (n == 0) {
            break;
        }
        s->audio_out_head = (s->audio_out_head + n) % ESP32_I2S_AUDIO_FRAMES;
        s->audio_out_num -= n;
        free -= n * frame;
    }
}

static void i2s_audio_in_cb(void *opaque, int avail)
{
    Esp32I2sState *s = opaque;
    int bytes = s->audio_in_bits / 8;
    int frame = 2 * bytes;
    uint8_t buf[4096];

    while (avail >= frame) {
        int n = MIN(avail, (int)sizeof(buf)) / frame;
        size_t got = audio_be_read(s->audio_be, s->voice_in, buf, n * frame);

        n = got / frame;
        if (n == 0) {
            break;
        }
        for (int i = 0; i < n; i++) {
            uint32_t k;

            if (s->audio_in_num == ESP32_I2S_AUDIO_FRAMES) {
                s->audio_in_head = (s->audio_in_head + 1) %
                                   ESP32_I2S_AUDIO_FRAMES;
                s->audio_in_num--;
            }
            k = (s->audio_in_head + s->audio_in_num) % ESP32_I2S_AUDIO_FRAMES;
            for (int c = 0; c < 2; c++) {
                s->audio_in[k][c] = bytes == 2 ?
                    (int16_t)lduw_le_p(buf + i * frame + c * 2) :
                    (int32_t)ldl_le_p(buf + i * frame + c * 4);
            }
            s->audio_in_num++;
        }
        avail -= n * frame;
    }
}

/*
 * Open the playback or recording voice for frames of the given rate and
 * width, or close it when freq is 0.
 */
static void i2s_audio_set(Esp32I2sState *s, bool out, int freq, int bits)
{
    struct audsettings as = {
        .freq = freq,
        .nchannels = 2,
        .fmt = bits == 16 ? AUDIO_FORMAT_S16 : AUDIO_FORMAT_S32,
        .big_endian = false,
    };
    g_autofree char *name = g_strdup_printf("esp32.i2s%u.%s", s->id,
                                            out ? "out" : "in");

    if (!s->audio_be) {
        return;
    }
    if (out) {
        if (freq == 0) {
            if (s->voice_out) {
                audio_be_set_active_out(s->audio_be, s->voice_out, false);
            }
            s->audio_out_num = 0;
            return;
        }
        if (!s->voice_out || freq != s->audio_out_freq ||
            bits != s->audio_out_bits) {
            s->voice_out = audio_be_open_out(s->audio_be, s->voice_out, name,
                                             s, i2s_audio_out_cb, &as);
            s->audio_out_freq = freq;
            s->audio_out_bits = bits;
            s->audio_out_num = 0;
        }
        if (s->voice_out) {
            audio_be_set_active_out(s->audio_be, s->voice_out, true);
        }
    } else {
        if (freq == 0) {
            if (s->voice_in) {
                audio_be_set_active_in(s->audio_be, s->voice_in, false);
            }
            s->audio_in_num = 0;
            return;
        }
        if (!s->voice_in || freq != s->audio_in_freq ||
            bits != s->audio_in_bits) {
            s->voice_in = audio_be_open_in(s->audio_be, s->voice_in, name,
                                           s, i2s_audio_in_cb, &as);
            s->audio_in_freq = freq;
            s->audio_in_bits = bits;
            s->audio_in_num = 0;
        }
        if (s->voice_in) {
            audio_be_set_active_in(s->audio_be, s->voice_in, true);
        }
    }
}

/* Queue a frame for the codec; a backend that falls behind loses frames. */
static void i2s_audio_play(Esp32I2sState *s, uint32_t l, uint32_t r)
{
    uint32_t k;

    if (!s->voice_out) {
        return;
    }
    if (s->audio_out_num == ESP32_I2S_AUDIO_FRAMES) {
        s->audio_out_head = (s->audio_out_head + 1) % ESP32_I2S_AUDIO_FRAMES;
        s->audio_out_num--;
    }
    k = (s->audio_out_head + s->audio_out_num) % ESP32_I2S_AUDIO_FRAMES;
    s->audio_out[k][0] = s->audio_out_bits == 16 ? (int16_t)(l >> 16)
                                                 : (int32_t)l;
    s->audio_out[k][1] = s->audio_out_bits == 16 ? (int16_t)(r >> 16)
                                                 : (int32_t)r;
    s->audio_out_num++;
}

/* The next recorded frame, MSB-aligned; silence when none is waiting. */
static void i2s_audio_record(Esp32I2sState *s, uint32_t *l, uint32_t *r)
{
    uint32_t k = s->audio_in_head;

    if (s->audio_in_num == 0) {
        *l = *r = 0;
        return;
    }
    if (s->audio_in_bits == 16) {
        *l = (uint32_t)(uint16_t)s->audio_in[k][0] << 16;
        *r = (uint32_t)(uint16_t)s->audio_in[k][1] << 16;
    } else {
        *l = s->audio_in[k][0];
        *r = s->audio_in[k][1];
    }
    s->audio_in_head = (s->audio_in_head + 1) % ESP32_I2S_AUDIO_FRAMES;
    s->audio_in_num--;
}

/* ---- Modes ---- */

static bool i2s_loopback(Esp32I2sState *s)
{
    return s->conf & CONF_SIG_LOOPBACK;
}

/* Only I2S0 has the PDM modules (TRM 22.1) */
static bool i2s_pdm(Esp32I2sState *s, bool tx)
{
    uint32_t en = tx ? PDM_TX_EN : PDM_RX_EN;

    if (!(s->pdm_conf & en)) {
        return false;
    }
    if (s->id != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S1 has no PDM "
                      "%s module\n", tx ? "transmit" : "receive");
        return false;
    }
    return true;
}

/* [spec:nuos:req:emu.esp32.i2s] */
static unsigned i2s_tx_mode(Esp32I2sState *s)
{
    bool slave = s->conf & CONF_TX_SLAVE_MOD;

    if (!(s->conf & CONF_TX_START) || (s->conf & CONF_TX_RESET) ||
        !i2s_running(s)) {
        return MODE_OFF;
    }
    if (s->conf2 & CONF2_LCD_EN) {
        if (slave) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u LCD mode "
                          "transmits as master only\n", s->id);
            return MODE_OFF;
        }
        return MODE_LCD;
    }
    if (i2s_pdm(s, true)) {
        if (slave) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: PDM slave transmission is "
                          "not modelled\n");
            return MODE_OFF;
        }
        return MODE_PDM;
    }
    return slave ? MODE_SLAVE : MODE_MASTER;
}

/* [spec:nuos:req:emu.esp32.i2s] */
static unsigned i2s_rx_mode(Esp32I2sState *s)
{
    bool slave = s->conf & CONF_RX_SLAVE_MOD;

    if (!(s->conf & CONF_RX_START) || (s->conf & CONF_RX_RESET) ||
        !i2s_running(s)) {
        return MODE_OFF;
    }
    if (s->conf2 & CONF2_LCD_EN) {
        if (s->conf2 & CONF2_CAMERA_EN) {
            if (!slave) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u camera "
                              "mode receives as slave only\n", s->id);
                return MODE_OFF;
            }
            return MODE_CAMERA;
        }
        if (slave) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u LCD mode "
                          "receives as master only, or as a camera slave\n",
                          s->id);
            return MODE_OFF;
        }
        return MODE_LCD;
    }
    if (i2s_pdm(s, false)) {
        if (slave) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: PDM slave reception is not "
                          "modelled\n");
            return MODE_OFF;
        }
        return MODE_PDM;
    }
    return slave ? MODE_SLAVE : MODE_MASTER;
}

/* ---- Transmitter ---- */

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * Take the next frame from the transmit FIFO (TRM 22.4.4). TX_FIFO_MOD
 * gives the FIFO's layout: 0, 16-bit dual channel, a word per frame;
 * 1, 16-bit single channel, a 16-bit half per sample, high half first;
 * 2, 32-bit dual channel, two words per frame; 3, 32-bit single channel,
 * a word per sample. With TX_MSB_RIGHT the right channel comes first,
 * in the high half or the first word. TX_CHAN_MOD then makes the channels
 * (table 22.4-2): 0 both from the FIFO, 1 and 2 one channel copied to the
 * other, 3 and 4 one channel from the FIFO and the other the constant
 * CONF_SINGLE_DATA. tx_frame[] holds the frame in the order it goes out,
 * the right channel first with TX_RIGHT_FIRST. Returns false, leaving the
 * last frame in place, if the FIFO cannot supply a frame.
 */
static bool i2s_tx_fetch(Esp32I2sState *s)
{
    unsigned fm = FIFO_CONF_TX_FIFO_MOD(s->fifo_conf);
    unsigned cm = CONF_CHAN_TX(s->conf_chan);
    bool mr = s->conf & CONF_TX_MSB_RIGHT;
    uint32_t k = s->single_data;
    uint32_t l = 0, r = 0, x0 = 0, x1 = 0, w;
    unsigned need, have;
    bool dual;

    if (fm > 3) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u reserved "
                      "TX_FIFO_MOD %u\n", s->id, fm);
        fm = 0;
    }
    if (cm > 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u reserved "
                      "TX_CHAN_MOD %u\n", s->id, cm);
        cm = 0;
    }
    dual = fm == 0 || fm == 2;
    /* Samples a frame takes: halves in mode 1, words in modes 2 and 3 */
    switch (fm) {
    case 0:
        need = 1;
        have = s->tx_fifo.num;
        break;
    case 1:
        need = cm == 0 ? 2 : 1;
        have = s->tx_fifo.num * 2 + s->tx_half_valid;
        break;
    case 2:
        need = 2;
        have = s->tx_fifo.num;
        break;
    default:
        need = cm == 0 ? 2 : 1;
        have = s->tx_fifo.num;
        break;
    }
    if (have < need) {
        return false;
    }

    switch (fm) {
    case 0:
        w = i2s_tx_fifo_pop(s);
        l = mr ? w << 16 : w & 0xffff0000u;
        r = mr ? w & 0xffff0000u : w << 16;
        break;
    case 2:
        x0 = i2s_tx_fifo_pop(s);
        x1 = i2s_tx_fifo_pop(s);
        l = mr ? x1 : x0;
        r = mr ? x0 : x1;
        break;
    case 1:
        for (unsigned i = 0; i < need; i++) {
            uint32_t x;

            if (s->tx_half_valid) {
                x = s->tx_half;
                s->tx_half_valid = false;
            } else {
                w = i2s_tx_fifo_pop(s);
                x = w & 0xffff0000u;
                s->tx_half = w << 16;
                s->tx_half_valid = true;
            }
            if (i == 0) {
                x0 = x;
            } else {
                x1 = x;
            }
        }
        break;
    default:
        x0 = i2s_tx_fifo_pop(s);
        if (need == 2) {
            x1 = i2s_tx_fifo_pop(s);
        }
        break;
    }

    if (dual) {
        switch (cm) {
        case 1:
            l = r = mr ? r : l;
            break;
        case 2:
            l = r = mr ? l : r;
            break;
        case 3:
            if (mr) {
                r = k;
            } else {
                l = k;
            }
            break;
        case 4:
            if (mr) {
                l = k;
            } else {
                r = k;
            }
            break;
        default:
            break;
        }
    } else {
        switch (cm) {
        case 0:
            /* Two samples, in the order they go out */
            l = (s->conf & CONF_TX_RIGHT_FIRST) ? x1 : x0;
            r = (s->conf & CONF_TX_RIGHT_FIRST) ? x0 : x1;
            break;
        case 3:
            l = mr ? x0 : k;
            r = mr ? k : x0;
            break;
        case 4:
            l = mr ? k : x0;
            r = mr ? x0 : k;
            break;
        default:
            l = r = x0;
            break;
        }
    }
    l = i2s_pcm(s, l, true);
    r = i2s_pcm(s, r, true);
    if (s->conf & CONF_TX_RIGHT_FIRST) {
        s->tx_frame[0] = r;
        s->tx_frame[1] = l;
    } else {
        s->tx_frame[0] = l;
        s->tx_frame[1] = r;
    }
    return true;
}

/* The frame last fetched, as left and right channels */
static void i2s_tx_lr(Esp32I2sState *s, uint32_t *l, uint32_t *r)
{
    bool rf = s->conf & CONF_TX_RIGHT_FIRST;

    *l = s->tx_frame[rf ? 1 : 0];
    *r = s->tx_frame[rf ? 0 : 1];
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The transmitter needs a frame. A FIFO that cannot supply one raises
 * TX_REMPTY; the master then stops its clocks with TX_STOP_EN, and
 * otherwise sends the last frame again (TRM 22.4.6). Returns false if the
 * transmitter stopped.
 */
static bool i2s_tx_next_frame(Esp32I2sState *s, bool master)
{
    if (i2s_tx_fetch(s)) {
        return true;
    }
    i2s_raise(s, INT_TX_REMPTY);
    if (master && (s->conf1 & CONF1_TX_STOP_EN)) {
        s->tx_stopped = true;
        return false;
    }
    return true;
}

static void i2s_drive(Esp32I2sState *s, unsigned line, bool level)
{
    qemu_set_irq(s->sig_out[line], level);
}

/* Drive v's top bits onto the LCD bus, I2SnO_DATA_out[23:24-bits] */
static void i2s_drive_bus(Esp32I2sState *s, uint32_t v, unsigned bits)
{
    uint32_t level = 0;

    bits = MIN(bits, ESP32_I2S_DATA_OUT_COUNT);
    for (unsigned b = 0; b < bits; b++) {
        if ((v >> (32 - bits + b)) & 1) {
            level |= 1u << (ESP32_I2S_DATA_OUT_COUNT - bits + b);
        }
    }
    for (unsigned i = 0; i < ESP32_I2S_DATA_OUT_COUNT; i++) {
        if (((level ^ s->data_out_level) >> i) & 1) {
            i2s_drive(s, ESP32_I2S_OUT_DATA0 + i, (level >> i) & 1);
        }
    }
    s->data_out_level = level;
}

static void i2s_drive_sd(Esp32I2sState *s, bool level)
{
    uint32_t bit = 1u << 23;

    if (!!(s->data_out_level & bit) != level) {
        s->data_out_level ^= bit;
        i2s_drive(s, OUT_SD, level);
    }
}

/* ---- The serial line ---- */

static void ser_reset(Esp32I2sSerial *ser, bool right_first)
{
    memset(ser, 0, sizeof(*ser));
    ser->next_ch = right_first;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * Word select and channel slots (TRM 22.4.1). Without SHORT_SYNC a change
 * of WS starts a slot whose channel is WS (low left, high right); with
 * MSB_SHIFT (Philips) its MSB follows one BCK cycle later, otherwise at
 * once (MSB-aligned). With SHORT_SYNC (PCM) WS pulses high for the BCK
 * cycle before a slot's MSB, and slots alternate between the channels.
 * A slot's bits follow its MSB, MSB first; the slot it replaces keeps the
 * line for the MSB_SHIFT cycle that its last bit may need.
 * Returns whether a slot starts on this cycle.
 */
static bool ser_slot_start(Esp32I2sSerial *ser, bool ws, bool short_sync,
                           uint8_t *ch)
{
    bool start = false;

    if (short_sync) {
        if (ser->pulse) {
            start = true;
            *ch = ser->next_ch;
            ser->next_ch ^= 1;
        }
        ser->pulse = ws && !ser->prev_ws;
    } else if (!ser->started || ws != ser->prev_ws) {
        start = true;
        *ch = ws;
    }
    ser->started = true;
    ser->prev_ws = ws;
    return start;
}

static void ser_new_slot(Esp32I2sSerial *ser, uint8_t ch, uint32_t val,
                         unsigned delay)
{
    ser->prev = ser->cur;
    ser->prev.limit = ser->prev.pos + delay;
    ser->cur.active = true;
    ser->cur.ch = ch;
    ser->cur.pos = 0;
    ser->cur.val = val;
    ser->cur.limit = UINT32_MAX;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * One BCK cycle of a transmitter: the data bit it drives for this cycle,
 * whose WS is ws. A slot of the first channel, the right with
 * TX_RIGHT_FIRST, starts a new frame from the FIFO.
 */
static bool i2s_tx_bit(Esp32I2sState *s, bool ws, bool master)
{
    Esp32I2sSerial *ser = &s->tx_ser;
    bool short_sync = s->conf & CONF_TX_SHORT_SYNC;
    unsigned delay = short_sync ? 0 : !!(s->conf & CONF_TX_MSB_SHIFT);
    unsigned bits = s->tx_bits;
    bool first = !!(s->conf & CONF_TX_RIGHT_FIRST);
    bool bit = false;
    uint8_t ch;

    if (ser_slot_start(ser, ws, short_sync, &ch)) {
        if (ch == first) {
            i2s_tx_next_frame(s, master);
        }
        ser_new_slot(ser, ch, s->tx_frame[ch == first ? 0 : 1], delay);
    }
    if (ser->prev.active) {
        int64_t k = (int64_t)ser->prev.pos - delay;

        if (ser->prev.pos >= ser->prev.limit) {
            ser->prev.active = false;
        } else if (k >= 0 && k < bits) {
            bit = (ser->prev.val >> (31 - k)) & 1;
        }
        ser->prev.pos++;
    }
    if (ser->cur.active) {
        int64_t k = (int64_t)ser->cur.pos - delay;

        if (k >= 0 && k < bits) {
            bit = (ser->cur.val >> (31 - k)) & 1;
        }
        ser->cur.pos++;
    }
    return bit;
}

/* ---- Receiver ---- */

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The receive FIFO's layout (TRM 22.4.5): the frame becomes a 64-bit pair,
 * the right channel high with RX_MSB_RIGHT. RX_FIFO_MOD 0 stores the two
 * channels' top 16 bits in a word; 2 stores both as words, high first;
 * 1 and 3 store one channel, the high one with RX_CHAN_MOD 1 and the low
 * one with 2, as 16-bit halves (two samples a word, the first high) or as
 * words.
 */
static void i2s_rx_frame(Esp32I2sState *s, uint32_t l, uint32_t r)
{
    unsigned fm = FIFO_CONF_RX_FIFO_MOD(s->fifo_conf);
    unsigned cm = CONF_CHAN_RX(s->conf_chan);
    bool mr = s->conf & CONF_RX_MSB_RIGHT;
    uint32_t hi = mr ? r : l;
    uint32_t lo = mr ? l : r;
    uint32_t x;

    if (fm > 3) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u reserved "
                      "RX_FIFO_MOD %u\n", s->id, fm);
        fm = 0;
    }
    switch (fm) {
    case 0:
        i2s_rx_fifo_push(s, (hi & 0xffff0000u) | (lo >> 16));
        return;
    case 2:
        i2s_rx_fifo_push(s, hi);
        i2s_rx_fifo_push(s, lo);
        return;
    default:
        break;
    }
    if (cm != 1 && cm != 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u RX_CHAN_MOD %u "
                      "selects no channel for a single-channel FIFO "
                      "mode\n", s->id, cm);
    }
    x = cm == 2 ? lo : hi;
    if (fm == 3) {
        i2s_rx_fifo_push(s, x);
    } else if (s->rx_half_valid) {
        i2s_rx_fifo_push(s, s->rx_half | (x >> 16));
        s->rx_half_valid = false;
    } else {
        s->rx_half = x & 0xffff0000u;
        s->rx_half_valid = true;
    }
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * A received channel sample. A frame starts with the right channel with
 * RX_RIGHT_FIRST, the left otherwise; a sample of the other channel with
 * no first one before it is dropped (TRM figure 22.4-5).
 */
static void i2s_rx_sample(Esp32I2sState *s, uint8_t ch, uint32_t v)
{
    uint8_t first = !!(s->conf & CONF_RX_RIGHT_FIRST);

    v = i2s_pcm(s, v, false);
    if (ch == first) {
        s->rx_first = v;
        s->rx_have_first = true;
        return;
    }
    if (!s->rx_have_first) {
        return;
    }
    s->rx_have_first = false;
    if (first) {
        i2s_rx_frame(s, v, s->rx_first);
    } else {
        i2s_rx_frame(s, s->rx_first, v);
    }
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * One BCK cycle of a receiver, sampling data bit sd with word select ws.
 * A slot is complete after RX_BITS_MOD bits, or when the line moves on to
 * the next slot; its sample is MSB-aligned in 32 bits.
 */
static void i2s_rx_bit(Esp32I2sState *s, bool ws, bool sd)
{
    Esp32I2sSerial *ser = &s->rx_ser;
    bool short_sync = s->conf & CONF_RX_SHORT_SYNC;
    unsigned delay = short_sync ? 0 : !!(s->conf & CONF_RX_MSB_SHIFT);
    unsigned bits = s->rx_bits;
    uint8_t ch;
    Esp32I2sSlot *slot[2] = { &ser->prev, &ser->cur };

    if (ser_slot_start(ser, ws, short_sync, &ch)) {
        if (ser->prev.active) {
            ser->prev.active = false;
            i2s_rx_sample(s, ser->prev.ch, ser->prev.val);
        }
        ser_new_slot(ser, ch, 0, delay);
    }
    for (int i = 0; i < 2; i++) {
        Esp32I2sSlot *t = slot[i];
        int64_t k;

        if (!t->active) {
            continue;
        }
        k = (int64_t)t->pos - delay;
        if (t->pos >= t->limit) {
            t->active = false;
            i2s_rx_sample(s, t->ch, t->val);
            continue;
        }
        if (k >= 0 && k < bits) {
            t->val |= (uint32_t)sd << (31 - k);
            if (k == bits - 1) {
                t->active = false;
                i2s_rx_sample(s, t->ch, t->val);
                continue;
            }
        }
        t->pos++;
    }
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * The parallel receive path of the LCD and camera modes: each sample is
 * I2SnI_Data_in[15:0], in the high half of a 32-bit word. RX_FIFO_MOD 0
 * stores each sample with the one before it in the high half; 1 stores
 * two samples a word, the first high; 2 stores them as words in pairs;
 * 3 stores each as a word.
 */
static void i2s_rx_parallel(Esp32I2sState *s, uint32_t x)
{
    unsigned fm = FIFO_CONF_RX_FIFO_MOD(s->fifo_conf);
    uint32_t prev = s->rx_prev_sample;

    s->rx_prev_sample = x;
    switch (fm) {
    case 0:
        i2s_rx_fifo_push(s, (prev & 0xffff0000u) | (x >> 16));
        break;
    case 1:
        if (s->rx_half_valid) {
            i2s_rx_fifo_push(s, s->rx_half | (x >> 16));
            s->rx_half_valid = false;
        } else {
            s->rx_half = x;
            s->rx_half_valid = true;
        }
        break;
    case 2:
        if (s->rx_half_valid) {
            i2s_rx_fifo_push(s, s->rx_half);
            i2s_rx_fifo_push(s, x);
            s->rx_half_valid = false;
        } else {
            s->rx_half = x;
            s->rx_half_valid = true;
        }
        break;
    default:
        if (fm > 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u reserved "
                          "RX_FIFO_MOD %u\n", s->id, fm);
        }
        i2s_rx_fifo_push(s, x);
        break;
    }
}

static uint32_t i2s_data_in(Esp32I2sState *s)
{
    return (s->in_level >> ESP32_I2S_IN_DATA0) & 0xffff;
}

static bool i2s_in(Esp32I2sState *s, unsigned line)
{
    return (s->in_level >> line) & 1;
}

/* ---- Master clocks and the bus between units ---- */

/* A unit, transmitter or receiver, of either controller */
typedef struct I2sUnit {
    Esp32I2sState *c;
    bool inv;
    bool audio;
    int src;
    bool src_inv;
    bool sd;
} I2sUnit;

/* The output signal driving input line `line` of c, or -1 */
static int i2s_route(Esp32I2sState *c, unsigned line, bool *inv)
{
    unsigned src;

    if (!c->gpio ||
        !esp32_gpio_sig_in_source(c->gpio, esp32_i2s_in_signal(c->id, line),
                                  &src, inv)) {
        return -1;
    }
    return src;
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * Whether slave unit (c, tx) runs on the clock of master unit (m, m_tx):
 * the other unit of the same controller with SIG_LOOPBACK, which shares
 * one BCK and WS, or a slave whose BCK and WS inputs the GPIO matrix
 * connects to the master's BCK and WS outputs. *inv says whether WS is
 * inverted on the way.
 */
static bool i2s_clocked_by(Esp32I2sState *c, bool tx, Esp32I2sState *m,
                           bool m_tx, bool *inv)
{
    unsigned bck_in, ws_in;
    bool bck_inv;
    int bck, ws;

    *inv = false;
    if (c == m && tx != m_tx && i2s_loopback(c)) {
        return true;
    }
    if (tx || i2s_loopback(c)) {
        bck_in = ESP32_I2S_IN_O_BCK;
        ws_in = ESP32_I2S_IN_O_WS;
    } else {
        bck_in = ESP32_I2S_IN_I_BCK;
        ws_in = ESP32_I2S_IN_I_WS;
    }
    bck = i2s_route(c, bck_in, &bck_inv);
    ws = i2s_route(c, ws_in, inv);
    return bck >= 0 && ws >= 0 &&
           bck == esp32_i2s_out_signal(m->id, m_tx ? ESP32_I2S_OUT_O_BCK
                                                   : ESP32_I2S_OUT_I_BCK) &&
           ws == esp32_i2s_out_signal(m->id, m_tx ? ESP32_I2S_OUT_O_WS
                                                  : ESP32_I2S_OUT_I_WS);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * A master unit's frame: 2 * BITS_MOD BCK cycles of its WS (high for the
 * first channel's slot with RIGHT_FIRST, or pulsed by SHORT_SYNC), run
 * through every unit on its clock. Transmitters drive their bits;
 * receivers take the bits of the transmitter on the same clock that the
 * GPIO matrix connects to their data input, or that input's level, or,
 * with nothing of the chip driving it, the audio backend's next frame.
 * The master transmitter's frame also goes to the audio backend.
 */
static void i2s_bus_frame(Esp32I2sState *m, bool m_tx)
{
    Esp32I2sState *ctl[2] = { m, m->peer };
    I2sUnit tx[3], rx[3];
    unsigned ntx = 0, nrx = 0;
    unsigned bits = m_tx ? m->tx_bits : m->rx_bits;
    bool short_sync = m->conf & (m_tx ? CONF_TX_SHORT_SYNC
                                      : CONF_RX_SHORT_SYNC);
    unsigned delay = !!(m->conf & (m_tx ? CONF_TX_MSB_SHIFT
                                        : CONF_RX_MSB_SHIFT));
    bool first = !!(m->conf & (m_tx ? CONF_TX_RIGHT_FIRST
                                    : CONF_RX_RIGHT_FIRST));
    uint32_t l, r;

    if (m_tx) {
        tx[ntx++] = (I2sUnit) { .c = m };
    } else {
        rx[nrx++] = (I2sUnit) { .c = m };
    }
    if (m_tx && m->rx_mode_cur == MODE_MASTER && i2s_loopback(m)) {
        rx[nrx++] = (I2sUnit) { .c = m };
    }
    for (int i = 0; i < 2; i++) {
        Esp32I2sState *c = ctl[i];
        bool inv;

        if (!c) {
            continue;
        }
        if (c->tx_mode_cur == MODE_SLAVE &&
            i2s_clocked_by(c, true, m, m_tx, &inv)) {
            tx[ntx++] = (I2sUnit) { .c = c, .inv = inv };
        }
        if (c->rx_mode_cur == MODE_SLAVE &&
            i2s_clocked_by(c, false, m, m_tx, &inv)) {
            rx[nrx++] = (I2sUnit) { .c = c, .inv = inv };
        }
    }
    for (unsigned j = 0; j < nrx; j++) {
        I2sUnit *u = &rx[j];
        int src = i2s_route(u->c, IN_SD, &u->src_inv);

        u->src = -1;
        for (unsigned i = 0; i < ntx && src >= 0; i++) {
            if (src == esp32_i2s_out_signal(tx[i].c->id, OUT_SD)) {
                u->src = i;
            }
        }
        u->audio = u->src < 0 && u->c->voice_in;
        u->sd = i2s_in(u->c, IN_SD);
    }

    for (unsigned i = 0; i < 2 * bits; i++) {
        bool ws;

        if (short_sync) {
            /* A pulse in the cycle before each slot's MSB */
            ws = (i + 1 + 2 * bits - delay) % bits == 0;
        } else {
            ws = (i < bits) ? first : !first;
        }
        for (unsigned t = 0; t < ntx; t++) {
            tx[t].sd = i2s_tx_bit(tx[t].c, ws ^ tx[t].inv, tx[t].c == m);
            if (tx[t].c->tx_stopped) {
                return;
            }
        }
        for (unsigned j = 0; j < nrx; j++) {
            bool sd = rx[j].sd;

            if (rx[j].audio) {
                continue;
            }
            if (rx[j].src >= 0) {
                sd = tx[rx[j].src].sd ^ rx[j].src_inv;
            }
            i2s_rx_bit(rx[j].c, ws ^ rx[j].inv, sd);
        }
    }
    for (unsigned t = 0; t < ntx; t++) {
        /* The data line rests at the frame's last bit */
        i2s_drive_sd(tx[t].c, tx[t].sd);
    }
    for (unsigned j = 0; j < nrx; j++) {
        if (rx[j].audio) {
            i2s_audio_record(rx[j].c, &l, &r);
            i2s_rx_frame(rx[j].c, l, r);
        }
    }
    if (m_tx) {
        i2s_tx_lr(m, &l, &r);
        i2s_audio_play(m, l, r);
    }
}

/*
 * The period, in ns, of a unit's timer events, and the offset of the
 * first event: frames end one period after the unit starts; the LCD
 * transmitter's half WS cycles begin at once.
 */
static double i2s_period(Esp32I2sState *s, bool tx, unsigned mode,
                         uint32_t *offset)
{
    double bck = i2s_bck_ns(s, tx);
    uint32_t fp, fs;

    *offset = 1;
    switch (mode) {
    case MODE_MASTER:
        return bck * 2 * (tx ? s->tx_bits : s->rx_bits);
    case MODE_LCD:
        if (tx) {
            *offset = 0;
            return bck;
        }
        /* WS runs at half BCK's rate; a sample per WS cycle */
        return bck * 2;
    case MODE_PDM:
        if (!tx) {
            /* PDM clock = BCK; a PCM sample per 64 or 128 PDM bits */
            return bck * 64 * ((s->pdm_conf & PDM_RX_SINC_DSR_16_EN) ? 2 : 1);
        }
        /* fPDM = 64 * fPCM * FP / FS (TRM 22.4.7) */
        fp = PDM_FREQ_FP(s->pdm_freq_conf);
        fs = PDM_FREQ_FS(s->pdm_freq_conf);
        if (fs == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S0 TX_PDM_FS is "
                          "0\n");
            fs = 1;
        }
        return bck * 64 * fp / fs;
    default:
        return 0;
    }
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * One BCK cycle of the LCD transmitter (TRM 22.5.1). Each datum of a frame
 * goes on the bus for one WS cycle, two with LCD_TX_WRX2_EN; WS, the
 * write strobe, falls with the datum and rises half a WS cycle later.
 */
static void i2s_lcd_tx_event(Esp32I2sState *s, uint64_t e)
{
    unsigned rep = (s->conf2 & CONF2_LCD_TX_WRX2_EN) ? 2 : 1;
    unsigned p = e % (2 * rep);
    uint64_t datum = e / (2 * rep);

    if (p == 0) {
        if (!(datum & 1) && !i2s_tx_next_frame(s, true)) {
            return;
        }
        i2s_drive_bus(s, s->tx_frame[datum & 1], s->tx_bits);
        if (s->id == 0 && s->sens && esp32_sens_dac_dma_enabled(s->sens)) {
            /*
             * [spec:nuos:req:emu.esp32.analog]
             * I2S0's DAC mode (TRM 22.5.3): the right channel's datum
             * goes to DAC1, the left's to DAC2, each its top 8 bits.
             */
            bool right = ((datum & 1) == 0) ==
                         !!(s->conf & CONF_TX_RIGHT_FIRST);

            esp32_sens_dac_dma(s->sens, right ? 0 : 1,
                               s->tx_frame[datum & 1] >> 24);
        }
    }
    i2s_drive(s, ESP32_I2S_OUT_O_WS, p & 1);
}

static void i2s_tx_event(Esp32I2sState *s, uint64_t e)
{
    uint32_t l, r;

    switch (s->tx_mode_cur) {
    case MODE_MASTER:
        if ((s->conf1 & CONF1_TX_STOP_EN) && s->tx_fifo.num == 0 &&
            !s->tx_half_valid) {
            /* Nothing to send: the master stops BCK and WS */
            i2s_raise(s, INT_TX_REMPTY);
            s->tx_stopped = true;
            return;
        }
        i2s_bus_frame(s, true);
        break;
    case MODE_PDM:
        if (i2s_tx_next_frame(s, true)) {
            i2s_tx_lr(s, &l, &r);
            i2s_audio_play(s, l, r);
        }
        break;
    case MODE_LCD:
        i2s_lcd_tx_event(s, e);
        break;
    default:
        break;
    }
}

static void i2s_rx_event(Esp32I2sState *s)
{
    uint32_t l, r;

    switch (s->rx_mode_cur) {
    case MODE_MASTER:
        i2s_bus_frame(s, false);
        break;
    case MODE_PDM:
        if (!s->logged_pdm_in) {
            if (!(s->pdm_conf & PDM_PDM2PCM_CONV_EN)) {
                qemu_log_mask(LOG_UNIMP, "esp32_i2s: PDM reception without "
                              "the PDM-to-PCM converter is not "
                              "modelled\n");
            }
            if (!s->voice_in) {
                qemu_log_mask(LOG_UNIMP, "esp32_i2s: PDM input from the "
                              "pads is not modelled; with no audio backend "
                              "the receiver records silence\n");
            }
            s->logged_pdm_in = true;
        }
        i2s_audio_record(s, &l, &r);
        i2s_rx_frame(s, l & 0xffff0000u, r & 0xffff0000u);
        break;
    case MODE_LCD:
        if (s->id == 0 && s->apb_ctrl && s->sens &&
            (s->apb_ctrl->regs[R_APB_CTRL_SARADC_CTRL] &
             SARADC_DATA_TO_I2S)) {
            /*
             * [spec:nuos:req:emu.esp32.analog]
             * I2S0's ADC mode: WS starts each scan step of the SAR ADC DIG
             * controllers, whose DMA words take the place of the bus.
             */
            uint16_t words[2];
            unsigned n = esp32_sens_dig_sample(s->sens, words);

            for (unsigned i = 0; i < n; i++) {
                i2s_rx_parallel(s, (uint32_t)words[i] << 16);
            }
            break;
        }
        i2s_rx_parallel(s, i2s_data_in(s) << 16);
        break;
    default:
        break;
    }
}

/* The time of a unit's event n */
static int64_t i2s_event_time(int64_t anchor, double period, uint64_t n)
{
    return anchor + (int64_t)(n * period);
}

static void i2s_tx_timer_cb(void *opaque)
{
    Esp32I2sState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double period = s->tx_period;
    unsigned runs = 0;

    if (period <= 0) {
        return;
    }
    while (!s->tx_stopped && s->tx_mode_cur != MODE_OFF &&
           i2s_event_time(s->tx_anchor, period,
                          s->tx_events + s->tx_offset) <= now) {
        uint64_t e = s->tx_events++;

        i2s_tx_event(s, e);
        if (++runs == MAX_CATCH_UP) {
            /* Too far behind to catch up: drop the backlog */
            s->tx_anchor = now - (int64_t)(s->tx_offset * period);
            s->tx_events = 0;
            break;
        }
    }
    if (!s->tx_stopped && s->tx_mode_cur != MODE_OFF) {
        timer_mod_ns(&s->tx_timer,
                     i2s_event_time(s->tx_anchor, period,
                                    s->tx_events + s->tx_offset));
    }
    i2s_update_irq(s);
}

static void i2s_rx_timer_cb(void *opaque)
{
    Esp32I2sState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double period = s->rx_period;
    unsigned runs = 0;

    if (period <= 0) {
        return;
    }
    while (s->rx_mode_cur != MODE_OFF &&
           i2s_event_time(s->rx_anchor, period,
                          s->rx_events + s->rx_offset) <= now) {
        s->rx_events++;
        i2s_rx_event(s);
        if (++runs == MAX_CATCH_UP) {
            s->rx_anchor = now - (int64_t)(s->rx_offset * period);
            s->rx_events = 0;
            break;
        }
    }
    if (s->rx_mode_cur != MODE_OFF) {
        timer_mod_ns(&s->rx_timer,
                     i2s_event_time(s->rx_anchor, period,
                                    s->rx_events + s->rx_offset));
    }
    i2s_update_irq(s);
}

static bool i2s_tx_timed(unsigned mode)
{
    return mode == MODE_MASTER || mode == MODE_PDM || mode == MODE_LCD;
}

static void i2s_tx_arm(Esp32I2sState *s)
{
    timer_del(&s->tx_timer);
    if (s->tx_period > 0 && !s->tx_stopped && i2s_tx_timed(s->tx_mode_cur)) {
        timer_mod_ns(&s->tx_timer,
                     i2s_event_time(s->tx_anchor, s->tx_period,
                                    s->tx_events + s->tx_offset));
    }
}

/* A transmitter that stopped its clocks for want of data goes on. */
static void i2s_tx_resume(Esp32I2sState *s)
{
    if (s->tx_stopped && s->tx_mode_cur != MODE_OFF) {
        s->tx_stopped = false;
        s->tx_anchor = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->tx_events = 0;
        i2s_tx_arm(s);
    }
}

/* Whether the receiver runs on its own clock, rather than its transmitter's */
static bool i2s_rx_own_clock(Esp32I2sState *s)
{
    if (s->rx_mode_cur == MODE_MASTER) {
        return !(i2s_loopback(s) && s->tx_mode_cur == MODE_MASTER);
    }
    return s->rx_mode_cur == MODE_LCD || s->rx_mode_cur == MODE_PDM;
}

static int i2s_audio_freq(double period)
{
    return period > 0 ? (int)(1e9 / period + 0.5) : 0;
}

/* The channel widths and event periods the configuration gives */
static void i2s_retime(Esp32I2sState *s)
{
    s->tx_bits = i2s_bits(s, true);
    s->rx_bits = i2s_bits(s, false);
    s->tx_period = i2s_period(s, true, s->tx_mode_cur, &s->tx_offset);
    s->rx_period = i2s_period(s, false, s->rx_mode_cur, &s->rx_offset);
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * Start, stop or retime the transmitter and the receiver after a change
 * of their configuration, their clock or DPORT's gate. A unit starting
 * begins a fresh frame; with reanchor, a running one restarts its clock's
 * timing from now.
 */
static void i2s_update_units(Esp32I2sState *s, bool reanchor)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned tx = i2s_tx_mode(s);
    unsigned rx = i2s_rx_mode(s);
    bool tx_restart = tx != s->tx_mode_cur;
    bool rx_restart = rx != s->rx_mode_cur;
    bool rx_own;

    s->tx_mode_cur = tx;
    s->rx_mode_cur = rx;
    i2s_retime(s);
    if ((tx_restart || rx_restart) && (tx != MODE_OFF || rx != MODE_OFF) &&
        (s->clkm_conf & CLKM_CLKA_ENA) && clock_get_hz(s->apll_clk) == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u runs from "
                      "APLL_CLK while the APLL is off\n", s->id);
    }

    if (tx_restart || reanchor) {
        timer_del(&s->tx_timer);
        if (tx_restart) {
            ser_reset(&s->tx_ser, s->conf & CONF_TX_RIGHT_FIRST);
            s->tx_stopped = false;
            if (tx == MODE_PDM) {
                qemu_log_mask(LOG_UNIMP, "esp32_i2s: the PDM bit stream on "
                              "the pads is not modelled; the PCM frames go "
                              "to the audio backend%s\n",
                              (s->pdm_conf & PDM_PCM2PDM_CONV_EN) ? "" :
                              ", and transmission without the PCM-to-PDM "
                              "converter is not modelled");
            }
            if (tx != MODE_OFF && !(s->fifo_conf & FIFO_CONF_TX_FORCE_EN)) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u transmits "
                              "with TX_FIFO_MOD_FORCE_EN clear\n", s->id);
            }
        }
        s->tx_anchor = now;
        s->tx_events = 0;
        i2s_tx_arm(s);
        i2s_audio_set(s, true,
                      (tx == MODE_MASTER || tx == MODE_PDM) ?
                      i2s_audio_freq(s->tx_period) : 0,
                      (tx == MODE_PDM || s->tx_bits <= 16) ? 16 : 32);
    }

    rx_own = i2s_rx_own_clock(s);
    if (rx_restart || reanchor || rx_own != timer_pending(&s->rx_timer)) {
        timer_del(&s->rx_timer);
        if (rx_restart) {
            ser_reset(&s->rx_ser, s->conf & CONF_RX_RIGHT_FIRST);
            s->rx_have_first = false;
            s->rx_half_valid = false;
            s->rx_prev_sample = 0;
            s->logged_pdm_in = false;
            if (rx != MODE_OFF && !(s->fifo_conf & FIFO_CONF_RX_FORCE_EN)) {
                qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u receives "
                              "with RX_FIFO_MOD_FORCE_EN clear\n", s->id);
            }
            if (rx == MODE_CAMERA && (s->conf2 & CONF2_INTER_VALID_EN)) {
                qemu_log_mask(LOG_UNIMP, "esp32_i2s: the camera's internal "
                              "validation (INTER_VALID_EN) is not "
                              "modelled\n");
            }
        }
        s->rx_anchor = now;
        s->rx_events = 0;
        if (rx_own && s->rx_period > 0) {
            timer_mod_ns(&s->rx_timer,
                         i2s_event_time(now, s->rx_period, s->rx_offset));
        }
        i2s_audio_set(s, false,
                      (rx == MODE_MASTER || rx == MODE_PDM) ?
                      i2s_audio_freq(s->rx_period) : 0,
                      (rx == MODE_PDM || s->rx_bits <= 16) ? 16 : 32);
    }
}

/* ---- Pads ---- */

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * An input signal changes. A slave transmitter not clocked by a master of
 * the chip drives its next bit on each falling edge of its BCK input
 * (rising with TX_BCK_IN_INV); a slave receiver samples its data input on
 * each rising edge of its BCK input. With SIG_LOOPBACK both use the
 * transmitter's BCK and WS inputs. A camera samples I2SnI_Data_in[15:0]
 * on each rising edge of its PCLK, I2SnI_WS_in, while H_SYNC, V_SYNC and
 * H_ENABLE are all high (TRM 22.5.2).
 */
static void i2s_sig_in(void *opaque, int n, int level)
{
    Esp32I2sState *s = opaque;
    bool old = i2s_in(s, n);
    bool rising = level && !old;
    bool falling = !level && old;
    bool tx_slave = s->tx_mode_cur == MODE_SLAVE;
    bool rx_slave = s->rx_mode_cur == MODE_SLAVE;

    s->in_level = deposit32(s->in_level, n, 1, !!level);
    if (!rising && !falling) {
        return;
    }
    switch (n) {
    case ESP32_I2S_IN_O_BCK:
        if (tx_slave) {
            bool inv = s->timing & TIMING_TX_BCK_IN_INV;

            if (inv ? rising : falling) {
                i2s_drive_sd(s, i2s_tx_bit(s, i2s_in(s, ESP32_I2S_IN_O_WS),
                                           false));
            }
        }
        if (rx_slave && i2s_loopback(s) && rising) {
            i2s_rx_bit(s, i2s_in(s, ESP32_I2S_IN_O_WS), i2s_in(s, IN_SD));
        }
        break;
    case ESP32_I2S_IN_I_BCK:
        if (rx_slave && !i2s_loopback(s) && rising) {
            i2s_rx_bit(s, i2s_in(s, ESP32_I2S_IN_I_WS), i2s_in(s, IN_SD));
        }
        break;
    case ESP32_I2S_IN_I_WS:
        if (s->rx_mode_cur == MODE_CAMERA && rising &&
            i2s_in(s, ESP32_I2S_IN_H_SYNC) &&
            i2s_in(s, ESP32_I2S_IN_V_SYNC) &&
            i2s_in(s, ESP32_I2S_IN_H_ENABLE)) {
            i2s_rx_parallel(s, i2s_data_in(s) << 16);
        }
        break;
    default:
        break;
    }
    i2s_update_irq(s);
}

/* ---- Registers ---- */

static uint32_t i2s_link_reg(Esp32I2sState *s, bool out)
{
    Esp32I2sLink *l = out ? &s->out : &s->in;

    return (out ? s->out_link : s->in_link) | (l->parked ? LINK_PARK : 0);
}

static uint32_t i2s_state(Esp32I2sState *s)
{
    bool idle = s->tx_mode_cur == MODE_OFF || s->tx_stopped;

    return idle ? STATE_TX_IDLE : 0;
}

/* [spec:nuos:req:emu.esp32.i2s] */
static uint64_t esp32_i2s_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32I2sState *s = opaque;
    uint32_t v;

    switch (addr) {
    case A_FIFO_WR:
        return 0;
    case A_FIFO_RD:
        if (s->rx_fifo.num == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u receive FIFO "
                          "read while empty\n", s->id);
            i2s_raise(s, INT_RX_REMPTY);
            return 0;
        }
        v = fifo_pop(&s->rx_fifo, NULL, NULL, NULL);
        if (s->rx_fifo.num == 0) {
            i2s_raise(s, INT_RX_REMPTY);
        }
        return v;
    case A_CONF:
        return s->conf;
    case A_INT_RAW:
        return s->int_raw;
    case A_INT_ST:
        return s->int_raw & s->int_ena;
    case A_INT_ENA:
        return s->int_ena;
    case A_INT_CLR:
        return 0;
    case A_TIMING:
        return s->timing;
    case A_FIFO_CONF:
        return s->fifo_conf;
    case A_RXEOF_NUM:
        return s->rx_eof_num;
    case A_CONF_SINGLE_DATA:
        return s->single_data;
    case A_CONF_CHAN:
        return s->conf_chan;
    case A_OUT_LINK:
        return i2s_link_reg(s, true);
    case A_IN_LINK:
        return i2s_link_reg(s, false);
    case A_OUT_EOF_DES_ADDR:
        return s->out_eof_des_addr;
    case A_IN_EOF_DES_ADDR:
        return s->in_eof_des_addr;
    case A_OUT_EOF_BFR_DES_ADDR:
        return s->out_eof_bfr_des_addr;
    case A_AHB_TEST:
        return s->ahb_test;
    case A_INLINK_DSCR:
        return s->inlink_dscr;
    case A_INLINK_DSCR_BF0:
        return s->inlink_dscr_bf0;
    case A_INLINK_DSCR_BF1:
        return s->inlink_dscr_bf1;
    case A_OUTLINK_DSCR:
        return s->outlink_dscr;
    case A_OUTLINK_DSCR_BF0:
        return s->outlink_dscr_bf0;
    case A_OUTLINK_DSCR_BF1:
        return s->outlink_dscr_bf1;
    case A_LC_CONF:
        return s->lc_conf;
    case A_OUTFIFO_PUSH:
        return s->outfifo_push;
    case A_INFIFO_POP:
        return s->infifo_pop;
    case A_LC_STATE0:
    case A_LC_STATE1:
        qemu_log_mask(LOG_UNIMP, "esp32_i2s: the undocumented DMA state "
                      "register 0x%02" HWADDR_PRIx " reads as 0\n", addr);
        return 0;
    case A_LC_HUNG_CONF:
        return s->lc_hung_conf;
    case A_CVSD_CONF0 ... A_CVSD_CONF2:
        return s->cvsd_conf[(addr - A_CVSD_CONF0) / 4];
    case A_PLC_CONF0 ... A_PLC_CONF2:
        return s->plc_conf[(addr - A_PLC_CONF0) / 4];
    case A_ESCO_CONF0:
        return s->esco_conf0;
    case A_SCO_CONF0:
        return s->sco_conf0;
    case A_CONF1:
        return s->conf1;
    case A_PD_CONF:
        return s->pd_conf;
    case A_CONF2:
        return s->conf2;
    case A_CLKM_CONF:
        return s->clkm_conf;
    case A_SAMPLE_RATE_CONF:
        return s->sample_rate_conf;
    case A_PDM_CONF:
        return s->pdm_conf;
    case A_PDM_FREQ_CONF:
        return s->pdm_freq_conf;
    case A_STATE:
        return i2s_state(s);
    case A_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u read of reserved "
                      "offset 0x%" HWADDR_PRIx "\n", s->id, addr);
        return 0;
    }
}

/*
 * [spec:nuos:req:emu.esp32.i2s]
 * CONF's reset bits (TRM 22.4.2) are levels: the unit or FIFO is held in
 * reset while its bit is set. LC_CONF's do the same for the DMA links.
 */
static void i2s_write_conf(Esp32I2sState *s, uint32_t val)
{
    uint32_t old = s->conf;

    s->conf = val & CONF_MASK;
    if (s->conf & CONF_TX_FIFO_RESET) {
        fifo_reset(&s->tx_fifo);
        s->tx_half_valid = false;
    }
    if (s->conf & CONF_RX_FIFO_RESET) {
        fifo_reset(&s->rx_fifo);
        s->rx_half_valid = false;
    }
    if (s->conf & CONF_TX_RESET) {
        ser_reset(&s->tx_ser, s->conf & CONF_TX_RIGHT_FIRST);
        s->tx_frame[0] = s->tx_frame[1] = 0;
        s->tx_half_valid = false;
    }
    if (s->conf & CONF_RX_RESET) {
        ser_reset(&s->rx_ser, s->conf & CONF_RX_RIGHT_FIRST);
        s->rx_have_first = false;
        s->rx_half_valid = false;
    }
    if ((s->conf & (CONF_TX_MONO | CONF_RX_MONO)) &&
        !(old & (CONF_TX_MONO | CONF_RX_MONO))) {
        qemu_log_mask(LOG_UNIMP, "esp32_i2s: the PCM mono modes "
                      "(TX_MONO, RX_MONO) are not modelled\n");
    }
    i2s_update_units(s, false);
    i2s_kick(s);
}

static void i2s_write_lc_conf(Esp32I2sState *s, uint32_t val)
{
    s->lc_conf = val & LC_CONF_MASK;
    if (s->lc_conf & (LC_CONF_OUT_RST | LC_CONF_AHBM_RST)) {
        link_reset(&s->out);
    }
    if (s->lc_conf & (LC_CONF_IN_RST | LC_CONF_AHBM_RST)) {
        link_reset(&s->in);
        s->rx_eof_count = 0;
    }
    if (s->lc_conf & (LC_CONF_OUT_LOOP_TEST | LC_CONF_IN_LOOP_TEST |
                      LC_CONF_MEM_TRANS_EN)) {
        qemu_log_mask(LOG_UNIMP, "esp32_i2s: the DMA loop test and memory "
                      "transfer modes are not modelled\n");
    }
    i2s_kick(s);
}

/* [spec:nuos:req:emu.esp32.i2s] */
static void esp32_i2s_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    Esp32I2sState *s = opaque;
    uint32_t val = value;

    switch (addr) {
    case A_FIFO_WR:
        i2s_tx_fifo_push(s, val);
        i2s_kick(s);
        break;
    case A_CONF:
        i2s_write_conf(s, val);
        break;
    case A_INT_ENA:
        s->int_ena = val & INT_MASK;
        break;
    case A_INT_CLR:
        s->int_raw &= ~(val & INT_MASK);
        break;
    case A_TIMING:
        s->timing = val & TIMING_MASK;
        break;
    case A_FIFO_CONF:
        s->fifo_conf = val & FIFO_CONF_MASK;
        i2s_update_units(s, false);
        i2s_kick(s);
        break;
    case A_RXEOF_NUM:
        s->rx_eof_num = val;
        break;
    case A_CONF_SINGLE_DATA:
        s->single_data = val;
        break;
    case A_CONF_CHAN:
        s->conf_chan = val & CONF_CHAN_MASK;
        break;
    case A_OUT_LINK:
        /* START, STOP and RESTART clear themselves; PARK is read-only. */
        s->out_link = val & LINK_ADDR_MASK;
        i2s_link_ctrl(s, true, val);
        break;
    case A_IN_LINK:
        s->in_link = val & LINK_ADDR_MASK;
        i2s_link_ctrl(s, false, val);
        break;
    case A_AHB_TEST:
        s->ahb_test = val & AHB_TEST_MASK;
        break;
    case A_LC_CONF:
        i2s_write_lc_conf(s, val);
        break;
    case A_OUTFIFO_PUSH:
        s->outfifo_push = val & OUTFIFO_PUSH_MASK;
        if (val & (1u << 16)) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: pushing the DMA FIFO "
                          "through OUTFIFO_PUSH is not modelled\n");
        }
        break;
    case A_INFIFO_POP:
        s->infifo_pop = val & INFIFO_POP_POP;
        if (val & INFIFO_POP_POP) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: popping the DMA FIFO "
                          "through INFIFO_POP is not modelled\n");
        }
        break;
    case A_LC_HUNG_CONF:
        s->lc_hung_conf = val & HUNG_MASK;
        timer_del(&s->tx_hung_timer);
        timer_del(&s->rx_hung_timer);
        i2s_kick(s);
        break;
    case A_CVSD_CONF0:
    case A_CVSD_CONF0 + 4:
        s->cvsd_conf[(addr - A_CVSD_CONF0) / 4] = val;
        break;
    case A_CVSD_CONF2:
        s->cvsd_conf[2] = val & CVSD_CONF2_MASK;
        break;
    case A_PLC_CONF0:
        s->plc_conf[0] = val & PLC_CONF0_MASK;
        break;
    case A_PLC_CONF0 + 4:
        s->plc_conf[1] = val;
        break;
    case A_PLC_CONF2:
        s->plc_conf[2] = val & PLC_CONF2_MASK;
        break;
    case A_ESCO_CONF0:
        s->esco_conf0 = val & ESCO_CONF0_MASK;
        if (val & ESCO_CONF0_EN) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: the eSCO/CVSD voice codec "
                          "is not modelled\n");
        }
        break;
    case A_SCO_CONF0:
        s->sco_conf0 = val & SCO_CONF0_MASK;
        if (val & SCO_CONF0_EN) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: the SCO/CVSD voice codec "
                          "is not modelled\n");
        }
        break;
    case A_CONF1:
        s->conf1 = val & CONF1_MASK;
        if (val & CONF1_TX_ZEROS_RM_EN) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: TX_ZEROS_RM_EN is not "
                          "modelled\n");
        }
        break;
    case A_PD_CONF:
        s->pd_conf = val & PD_CONF_MASK;
        if (val & PD_CONF_FIFO_FORCE_PD) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: powering the FIFOs down "
                          "is not modelled\n");
        }
        break;
    case A_CONF2:
        s->conf2 = val & CONF2_MASK;
        if (val & (CONF2_LCD_TX_SDX2_EN | CONF2_DATA_ENABLE_TEST_EN |
                   CONF2_EXT_ADC_START_EN)) {
            qemu_log_mask(LOG_UNIMP, "esp32_i2s: CONF2 0x%02x: LCD_TX_SDX2, "
                          "the data-enable test and the external ADC start "
                          "are not modelled\n", val & CONF2_MASK);
        }
        i2s_update_units(s, false);
        break;
    case A_CLKM_CONF:
        s->clkm_conf = val & CLKM_MASK;
        i2s_update_units(s, true);
        break;
    case A_SAMPLE_RATE_CONF:
        s->sample_rate_conf = val & SR_MASK;
        i2s_update_units(s, true);
        break;
    case A_PDM_CONF:
        s->pdm_conf = val & PDM_CONF_MASK;
        i2s_update_units(s, true);
        break;
    case A_PDM_FREQ_CONF:
        s->pdm_freq_conf = val & PDM_FREQ_MASK;
        i2s_update_units(s, true);
        break;
    case A_DATE:
        s->date = val;
        break;
    case A_FIFO_RD:
    case A_INT_RAW:
    case A_INT_ST:
    case A_OUT_EOF_DES_ADDR ... A_OUT_EOF_BFR_DES_ADDR:
    case A_INLINK_DSCR ... A_OUTLINK_DSCR_BF1:
    case A_LC_STATE0:
    case A_LC_STATE1:
    case A_STATE:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u write to read-only "
                      "offset 0x%" HWADDR_PRIx "\n", s->id, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32_i2s: I2S%u write to reserved "
                      "offset 0x%" HWADDR_PRIx "\n", s->id, addr);
        break;
    }
    i2s_update_irq(s);
}

static const MemoryRegionOps esp32_i2s_ops = {
    .read = esp32_i2s_read,
    .write = esp32_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---- Device ---- */

static void esp32_i2s_clock_update(void *opaque, ClockEvent event)
{
    Esp32I2sState *s = opaque;

    i2s_update_units(s, true);
    i2s_kick(s);
}

static void esp32_i2s_reset_hold(Object *obj, ResetType type)
{
    Esp32I2sState *s = ESP32_I2S(obj);

    s->conf = CONF_RESET;
    s->int_raw = 0;
    s->int_ena = 0;
    s->timing = 0;
    s->fifo_conf = FIFO_CONF_RESET;
    s->rx_eof_num = RXEOF_NUM_RESET;
    s->single_data = 0;
    s->conf_chan = 0;
    s->out_link = 0;
    s->in_link = 0;
    s->out_eof_des_addr = 0;
    s->in_eof_des_addr = 0;
    s->out_eof_bfr_des_addr = 0;
    s->ahb_test = 0;
    s->inlink_dscr = 0;
    s->inlink_dscr_bf0 = 0;
    s->inlink_dscr_bf1 = 0;
    s->outlink_dscr = 0;
    s->outlink_dscr_bf0 = 0;
    s->outlink_dscr_bf1 = 0;
    s->lc_conf = LC_CONF_RESET;
    s->outfifo_push = 0;
    s->infifo_pop = 0;
    s->lc_hung_conf = HUNG_RESET;
    s->cvsd_conf[0] = CVSD_CONF0_RESET;
    s->cvsd_conf[1] = CVSD_CONF1_RESET;
    s->cvsd_conf[2] = CVSD_CONF2_RESET;
    s->plc_conf[0] = PLC_CONF0_RESET;
    s->plc_conf[1] = PLC_CONF1_RESET;
    s->plc_conf[2] = PLC_CONF2_RESET;
    s->esco_conf0 = 0;
    s->sco_conf0 = 0;
    s->conf1 = CONF1_RESET;
    s->pd_conf = PD_CONF_RESET;
    s->conf2 = 0;
    s->clkm_conf = CLKM_RESET;
    s->sample_rate_conf = SR_RESET;
    s->pdm_conf = PDM_CONF_RESET;
    s->pdm_freq_conf = PDM_FREQ_RESET;
    s->date = DATE_RESET;

    fifo_reset(&s->tx_fifo);
    fifo_reset(&s->rx_fifo);
    link_reset(&s->out);
    link_reset(&s->in);
    s->rx_eof_count = 0;

    s->tx_stopped = false;
    s->tx_frame[0] = s->tx_frame[1] = 0;
    s->tx_half_valid = false;
    s->tx_anchor = 0;
    s->tx_events = 0;
    timer_del(&s->tx_timer);
    ser_reset(&s->tx_ser, true);
    s->rx_have_first = false;
    s->rx_half_valid = false;
    s->rx_prev_sample = 0;
    s->rx_anchor = 0;
    s->rx_events = 0;
    timer_del(&s->rx_timer);
    ser_reset(&s->rx_ser, true);
    s->tx_mode_cur = MODE_OFF;
    s->rx_mode_cur = MODE_OFF;

    timer_del(&s->tx_hung_timer);
    timer_del(&s->rx_hung_timer);
    s->tx_hung_fired = false;
    s->rx_hung_fired = false;
    s->logged_pdm_in = false;
    s->audio_out_num = 0;
    s->audio_in_num = 0;
}

static void esp32_i2s_reset_exit(Object *obj, ResetType type)
{
    Esp32I2sState *s = ESP32_I2S(obj);

    for (unsigned i = 0; i < ESP32_I2S_OUT_COUNT; i++) {
        qemu_set_irq(s->sig_out[i], 0);
    }
    s->data_out_level = 0;
    i2s_audio_set(s, true, 0, 16);
    i2s_audio_set(s, false, 0, 16);
    i2s_update_irq(s);
}

static void esp32_i2s_realize(DeviceState *dev, Error **errp)
{
    Esp32I2sState *s = ESP32_I2S(dev);

    if (s->id >= ESP32_I2S_COUNT) {
        error_setg(errp, "esp32_i2s: id must be 0 or 1");
        return;
    }
    if (!s->dma_mr) {
        error_setg(errp, "esp32_i2s: the dma-mr link is not set");
        return;
    }
    address_space_init(&s->dma_as, s->dma_mr, "esp32-i2s-dma");
}

static void esp32_i2s_init(Object *obj)
{
    Esp32I2sState *s = ESP32_I2S(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_i2s_ops, s, TYPE_ESP32_I2S,
                          ESP32_I2S_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->sig_out, ESP32_I2S_SIG_OUT,
                             ESP32_I2S_OUT_COUNT);
    qdev_init_gpio_in_named(DEVICE(obj), i2s_sig_in, ESP32_I2S_SIG_IN,
                            ESP32_I2S_IN_COUNT);
    s->apb_clk = qdev_init_clock_in(DEVICE(obj), "apb",
                                    esp32_i2s_clock_update, s, ClockUpdate);
    s->f160m_clk = qdev_init_clock_in(DEVICE(obj), "pll-f160m",
                                      esp32_i2s_clock_update, s, ClockUpdate);
    s->apll_clk = qdev_init_clock_in(DEVICE(obj), "apll",
                                     esp32_i2s_clock_update, s, ClockUpdate);
    timer_init_ns(&s->tx_timer, QEMU_CLOCK_VIRTUAL, i2s_tx_timer_cb, s);
    timer_init_ns(&s->rx_timer, QEMU_CLOCK_VIRTUAL, i2s_rx_timer_cb, s);
    timer_init_ns(&s->tx_hung_timer, QEMU_CLOCK_VIRTUAL, i2s_tx_hung_cb, s);
    timer_init_ns(&s->rx_hung_timer, QEMU_CLOCK_VIRTUAL, i2s_rx_hung_cb, s);
}

static const VMStateDescription vmstate_esp32_i2s_fifo = {
    .name = "esp32.i2s.fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(data, Esp32I2sFifo, ESP32_I2S_FIFO_DEPTH),
        VMSTATE_UINT8_ARRAY(tag, Esp32I2sFifo, ESP32_I2S_FIFO_DEPTH),
        VMSTATE_UINT32_ARRAY(tag_dscr, Esp32I2sFifo, ESP32_I2S_FIFO_DEPTH),
        VMSTATE_UINT32_ARRAY(tag_buf, Esp32I2sFifo, ESP32_I2S_FIFO_DEPTH),
        VMSTATE_UINT32(head, Esp32I2sFifo),
        VMSTATE_UINT32(num, Esp32I2sFifo),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_i2s_link = {
    .name = "esp32.i2s.link",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, Esp32I2sLink),
        VMSTATE_BOOL(parked, Esp32I2sLink),
        VMSTATE_BOOL(have_desc, Esp32I2sLink),
        VMSTATE_UINT32(dscr, Esp32I2sLink),
        VMSTATE_UINT32(dw0, Esp32I2sLink),
        VMSTATE_UINT32(buf, Esp32I2sLink),
        VMSTATE_UINT32(next, Esp32I2sLink),
        VMSTATE_UINT32(pos, Esp32I2sLink),
        VMSTATE_UINT32(fetch, Esp32I2sLink),
        VMSTATE_BOOL(have_last, Esp32I2sLink),
        VMSTATE_UINT32(last, Esp32I2sLink),
        VMSTATE_BOOL(exhausted, Esp32I2sLink),
        VMSTATE_BOOL(empty_flagged, Esp32I2sLink),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_i2s_slot = {
    .name = "esp32.i2s.slot",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(active, Esp32I2sSlot),
        VMSTATE_UINT8(ch, Esp32I2sSlot),
        VMSTATE_UINT32(pos, Esp32I2sSlot),
        VMSTATE_UINT32(val, Esp32I2sSlot),
        VMSTATE_UINT32(limit, Esp32I2sSlot),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_esp32_i2s_serial = {
    .name = "esp32.i2s.serial",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(started, Esp32I2sSerial),
        VMSTATE_UINT8(prev_ws, Esp32I2sSerial),
        VMSTATE_BOOL(pulse, Esp32I2sSerial),
        VMSTATE_UINT8(next_ch, Esp32I2sSerial),
        VMSTATE_STRUCT(cur, Esp32I2sSerial, 1, vmstate_esp32_i2s_slot,
                       Esp32I2sSlot),
        VMSTATE_STRUCT(prev, Esp32I2sSerial, 1, vmstate_esp32_i2s_slot,
                       Esp32I2sSlot),
        VMSTATE_END_OF_LIST()
    }
};

static int esp32_i2s_post_load(void *opaque, int version_id)
{
    i2s_retime(opaque);
    return 0;
}

static const VMStateDescription vmstate_esp32_i2s = {
    .name = TYPE_ESP32_I2S,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = esp32_i2s_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(apb_clk, Esp32I2sState),
        VMSTATE_CLOCK(f160m_clk, Esp32I2sState),
        VMSTATE_CLOCK(apll_clk, Esp32I2sState),
        VMSTATE_UINT32(conf, Esp32I2sState),
        VMSTATE_UINT32(int_raw, Esp32I2sState),
        VMSTATE_UINT32(int_ena, Esp32I2sState),
        VMSTATE_UINT32(timing, Esp32I2sState),
        VMSTATE_UINT32(fifo_conf, Esp32I2sState),
        VMSTATE_UINT32(rx_eof_num, Esp32I2sState),
        VMSTATE_UINT32(single_data, Esp32I2sState),
        VMSTATE_UINT32(conf_chan, Esp32I2sState),
        VMSTATE_UINT32(out_link, Esp32I2sState),
        VMSTATE_UINT32(in_link, Esp32I2sState),
        VMSTATE_UINT32(out_eof_des_addr, Esp32I2sState),
        VMSTATE_UINT32(in_eof_des_addr, Esp32I2sState),
        VMSTATE_UINT32(out_eof_bfr_des_addr, Esp32I2sState),
        VMSTATE_UINT32(ahb_test, Esp32I2sState),
        VMSTATE_UINT32(inlink_dscr, Esp32I2sState),
        VMSTATE_UINT32(inlink_dscr_bf0, Esp32I2sState),
        VMSTATE_UINT32(inlink_dscr_bf1, Esp32I2sState),
        VMSTATE_UINT32(outlink_dscr, Esp32I2sState),
        VMSTATE_UINT32(outlink_dscr_bf0, Esp32I2sState),
        VMSTATE_UINT32(outlink_dscr_bf1, Esp32I2sState),
        VMSTATE_UINT32(lc_conf, Esp32I2sState),
        VMSTATE_UINT32(outfifo_push, Esp32I2sState),
        VMSTATE_UINT32(infifo_pop, Esp32I2sState),
        VMSTATE_UINT32(lc_hung_conf, Esp32I2sState),
        VMSTATE_UINT32_ARRAY(cvsd_conf, Esp32I2sState, 3),
        VMSTATE_UINT32_ARRAY(plc_conf, Esp32I2sState, 3),
        VMSTATE_UINT32(esco_conf0, Esp32I2sState),
        VMSTATE_UINT32(sco_conf0, Esp32I2sState),
        VMSTATE_UINT32(conf1, Esp32I2sState),
        VMSTATE_UINT32(pd_conf, Esp32I2sState),
        VMSTATE_UINT32(conf2, Esp32I2sState),
        VMSTATE_UINT32(clkm_conf, Esp32I2sState),
        VMSTATE_UINT32(sample_rate_conf, Esp32I2sState),
        VMSTATE_UINT32(pdm_conf, Esp32I2sState),
        VMSTATE_UINT32(pdm_freq_conf, Esp32I2sState),
        VMSTATE_UINT32(date, Esp32I2sState),
        VMSTATE_STRUCT(tx_fifo, Esp32I2sState, 1, vmstate_esp32_i2s_fifo,
                       Esp32I2sFifo),
        VMSTATE_STRUCT(rx_fifo, Esp32I2sState, 1, vmstate_esp32_i2s_fifo,
                       Esp32I2sFifo),
        VMSTATE_STRUCT(out, Esp32I2sState, 1, vmstate_esp32_i2s_link,
                       Esp32I2sLink),
        VMSTATE_STRUCT(in, Esp32I2sState, 1, vmstate_esp32_i2s_link,
                       Esp32I2sLink),
        VMSTATE_UINT32(rx_eof_count, Esp32I2sState),
        VMSTATE_BOOL(tx_stopped, Esp32I2sState),
        VMSTATE_UINT32_ARRAY(tx_frame, Esp32I2sState, 2),
        VMSTATE_BOOL(tx_half_valid, Esp32I2sState),
        VMSTATE_UINT32(tx_half, Esp32I2sState),
        VMSTATE_INT64(tx_anchor, Esp32I2sState),
        VMSTATE_UINT64(tx_events, Esp32I2sState),
        VMSTATE_TIMER(tx_timer, Esp32I2sState),
        VMSTATE_STRUCT(tx_ser, Esp32I2sState, 1, vmstate_esp32_i2s_serial,
                       Esp32I2sSerial),
        VMSTATE_UINT32(data_out_level, Esp32I2sState),
        VMSTATE_UINT8(tx_mode_cur, Esp32I2sState),
        VMSTATE_BOOL(rx_have_first, Esp32I2sState),
        VMSTATE_UINT32(rx_first, Esp32I2sState),
        VMSTATE_BOOL(rx_half_valid, Esp32I2sState),
        VMSTATE_UINT32(rx_half, Esp32I2sState),
        VMSTATE_UINT32(rx_prev_sample, Esp32I2sState),
        VMSTATE_INT64(rx_anchor, Esp32I2sState),
        VMSTATE_UINT64(rx_events, Esp32I2sState),
        VMSTATE_TIMER(rx_timer, Esp32I2sState),
        VMSTATE_STRUCT(rx_ser, Esp32I2sState, 1, vmstate_esp32_i2s_serial,
                       Esp32I2sSerial),
        VMSTATE_UINT8(rx_mode_cur, Esp32I2sState),
        VMSTATE_UINT32(in_level, Esp32I2sState),
        VMSTATE_TIMER(tx_hung_timer, Esp32I2sState),
        VMSTATE_TIMER(rx_hung_timer, Esp32I2sState),
        VMSTATE_BOOL(tx_hung_fired, Esp32I2sState),
        VMSTATE_BOOL(rx_hung_fired, Esp32I2sState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32_i2s_properties[] = {
    DEFINE_PROP_UINT8("id", Esp32I2sState, id, 0),
    DEFINE_PROP_LINK("dma-mr", Esp32I2sState, dma_mr, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_LINK("gpio", Esp32I2sState, gpio, TYPE_ESP32_GPIO,
                     Esp32GpioState *),
    DEFINE_PROP_LINK("sens", Esp32I2sState, sens, TYPE_ESP32_SENS,
                     Esp32SensState *),
    DEFINE_PROP_LINK("apb-ctrl", Esp32I2sState, apb_ctrl, TYPE_ESP32_APB_CTRL,
                     Esp32ApbCtrlState *),
    DEFINE_PROP_LINK("peer", Esp32I2sState, peer, TYPE_ESP32_I2S,
                     Esp32I2sState *),
    DEFINE_AUDIO_PROPERTIES(Esp32I2sState, audio_be),
};

static void esp32_i2s_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_i2s_reset_hold;
    rc->phases.exit = esp32_i2s_reset_exit;
    dc->realize = esp32_i2s_realize;
    dc->vmsd = &vmstate_esp32_i2s;
    device_class_set_props(dc, esp32_i2s_properties);
}

/* [spec:nuos:req:emu.esp32.i2s] */
static const TypeInfo esp32_i2s_info = {
    .name = TYPE_ESP32_I2S,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2sState),
    .instance_init = esp32_i2s_init,
    .class_init = esp32_i2s_class_init,
};

static void esp32_i2s_register_types(void)
{
    type_register_static(&esp32_i2s_info);
}

type_init(esp32_i2s_register_types)
