/*
 * ESP32-P4 general-purpose DMA controllers, GDMA-AHB and GDMA-AXI
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 6
 * (GDMA Controller), with the layout, access types and reset values of
 * ESP-IDF's ahb_dma_reg.h and axi_dma_reg.h for v3, and its gdma_ll,
 * ahb_dma_ll and axi_dma_ll drivers. The register interface follows the
 * GDMA of the earlier RISC-V chips, which Espressif's QEMU models in
 * hw/dma/esp_gdma.c; this model is written afresh for the P4's.
 *
 * Each controller has three transmit (TX, "out") and three receive (RX,
 * "in") channels. A channel walks a linked list of descriptors in memory,
 * three words each: DW0 holds owner (31), suc_eof (30), err_eof (28),
 * length (23:12) and size (11:0); DW1 the buffer's address and DW2 the
 * next descriptor's, 0 at the end of the list. A TX channel reads each
 * buffer's length bytes into its FIFO, for the peripheral it selects (or,
 * with MEM_TRANS_EN on the RX channel of the same number, for that RX
 * channel); an RX channel writes what reaches its FIFO into its buffers
 * and writes each descriptor back as it closes it, with the length
 * received, suc_eof at the end of a frame, and owner cleared.
 *
 * Descriptors are checked as they are read (TRM 6.4.5): a descriptor
 * address outside the accessible memory (AHB_DMA_INTR_MEM_START_ADDR to
 * _END_ADDR; for GDMA-AXI also the EXTR_MEM range) or not word-aligned
 * (GDMA-AHB) or double-word-aligned (GDMA-AXI), a buffer outside it, the
 * descriptor's owner bit clear while CHECK_OWNER is set, or a bus error on
 * the read, raises DSCR_ERR and halts the channel. The TRM says only that
 * descriptors "should be" aligned; the model takes a misaligned one as a
 * descriptor error rather than guess which bits the bus drops. A bus error
 * on a data access (an access the DMA APM refuses) does not stop the
 * transfer (TRM 6.4.10): a read yields zeros and a write is lost, and
 * GDMA-AHB raises *_AHBINF_RESP_ERR and records the access in
 * AHBINF_RESP_ERR_STATUS0/1. A transaction whose accesses a memory
 * controller's AXI port holds (the flash's or the PSRAM's while it cannot
 * take an access: its clocks stopped, held in reset, AXI_REQ_EN clear;
 * esp32p4_mspi_dma_wait()) waits, and the arbiter that granted it with
 * it, without an error, until the port takes accesses again; it is then
 * made, before the arbiter grants another (chan_stalls()).
 *
 * Every memory access is a DMA master's on the bus through the access
 * permission managers (include/hw/misc/esp32p4_pms.h): the master of the
 * peripheral the channel serves (GDMA-AHB's AHB_PDMA_* or GDMA-AXI's
 * AXI_PDMA_*), or the controller's DUMMY master for memory-to-memory
 * transfers and dummy selections, with the master ID the ICM records:
 * 16 plus the peripheral number for GDMA-AHB (11 and 12, channels 1 and 2,
 * in memory-to-memory mode, TRM 6.4.3), and port 8 with the peripheral
 * number as unit for GDMA-AXI. The DMA reaches memory directly, not
 * through the CPUs' caches.
 *
 * Timing. The channels share the controller's bus, at its bus clock
 * (AHB_PDMA_SYS_CLK, AXI_PDMA_SYS_CLK): an arbiter grants one transaction
 * at a time, a descriptor read (three words), a descriptor write-back (one
 * word) or a data burst, which takes a bus cycle per word. A data burst
 * moves at most the channel's burst length: GDMA-AHB 4 bytes, or with
 * DATA_BURST_EN 16, 32 or 64 (DATA_BURST_MODE_SEL); GDMA-AXI 8 to 128
 * (BURST_SIZE_SEL). GDMA-AHB's six channels share one arbiter; GDMA-AXI's
 * TX channels (reads) and RX channels (writes) have one each and run side
 * by side. The arbiter grants the requesting channel of highest priority
 * (*_PRI_CHn, 0 to 5), round robin among equals and among all with
 * ARB_PRI_DIS; with weighted arbitration (WEIGHT_EN) each channel spends
 * a token of its weight per transaction and a channel without tokens
 * waits for the time slot (ARB_TIMEOUT bus cycles) to end, or for every
 * channel's tokens to be spent; a channel without requests holds its
 * tokens, and the slot, unless its ARB_WEIGH_OPT_DIR is clear (TRM 6.4.9).
 * A weight of 0 is never served while weighting is on. The rounds of the
 * bus fall on its clock's edges, counted exactly from its last change: a
 * round starts at the edge after the change that calls for it, and the
 * next comes as many cycles later as the round's transactions take. Each
 * happens at the time it falls due, however late the QEMU timer that
 * runs it: register accesses and ETM tasks first run the rounds due
 * before them, in order, and register accesses and bus clock changes
 * first let the attached peripherals that ask for it take their own
 * events due before them (include/hw/dma/esp32p4_gdma.h).
 *
 * The FIFOs are 64 bytes. Moving bytes from a TX channel's FIFO to the
 * peripheral or the paired RX channel is outside the bus. OUT_DONE,
 * OUT_TOTAL_EOF and (with OUT_EOF_MODE set) OUT_EOF are raised as the
 * last byte of a descriptor leaves the TX FIFO; with OUT_EOF_MODE clear,
 * OUT_EOF as it enters. GDMA-AXI's FIFO status shows the FIFO as its L3
 * FIFO; the L1 and L2 FIFOs read as empty.
 *
 * CRC (TRM 6.4.12): each channel computes a CRC of the bytes it moves, a
 * byte at a time, through the matrix software latches row by row:
 * crc[x] = parity(crc & lfsr_mask[x]) ^ parity(byte & data_mask[x]), for
 * the 8 (16, 24, 32) bits *_CRC_WIDTH selects (GDMA-AXI at most 16). The
 * CRC starts from *_CRC_INIT_DATA at each START and at *_CRC_CLEAR; a TX
 * channel computes it over the bytes it reads, an RX channel over those
 * it writes.
 *
 * ETM: the events are pulsed on the "etm-event" lines, each carrying the
 * time of the round, or of the peripheral's access, that raises it (see
 * esp32_line.h), and the tasks on "etm-task" start a channel whose
 * *_ETM_EN is set, at their events' time; *_LINK_START is ignored for
 * such a channel (TRM 6.5).
 *
 * Restart (TRM 6.4.4): at the end of its list a channel re-reads the last
 * descriptor's DW2 if *LINK_RESTART was set while it ran; set after the
 * list ended, it re-reads DW2 at once and continues from what it finds.
 * GDMA-AXI raises OUT_LINK_SWITCH, if LINK_SWITCH_STATE_CHn is set, when a
 * TX channel reads the first descriptor of a list it starts or restarts
 * on, and clears the bit.
 *
 * GDMA-AHB has memory-to-memory transfers on channels 1 and 2 only (TRM
 * 6.4.3); MEM_TRANS_EN on its channel 0 connects nothing.
 *
 * The BitScrambler interposes on the paths it is attached to, between
 * the channels and the peripheral, as include/hw/dma/esp32p4_gdma.h
 * describes: the peripheral's pulls and pushes on such a path reach it,
 * and it moves the bytes to and from the channel.
 *
 * Not modelled: the FIFO push and pop registers' extra bits beyond the
 * data byte (bit 8 of OUTFIFO_WDATA is taken as a frame end, IN_POP's
 * RDATA reads 0x800 when the FIFO is empty); the module clock forcing,
 * the AXI FIFO RAMs' power controls and the AHB/AXI test and redundancy
 * registers, kept as written; the AHB rev3 header's separate TX and RX
 * ARB_TIMEOUT and WEIGHT_EN registers (0x3CC to 0x3D8), kept as written,
 * as the TRM has the shared ARB_TIMEOUT and WEIGHT_EN drive the arbiter;
 * the 16-byte alignment that accesses to encrypted external memory need.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_pms.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "migration/vmstate.h"

#define IN  ESP32P4_GDMA_IN
#define OUT ESP32P4_GDMA_OUT
#define NCH ESP32P4_GDMA_CHANNELS

/* The interrupt bits, as *_INT_RAW, _ST, _ENA and _CLR hold them. */
#define IN_DONE             BIT(0)
#define IN_SUC_EOF          BIT(1)
#define IN_ERR_EOF          BIT(2)
#define IN_DSCR_ERR         BIT(3)
#define IN_DSCR_EMPTY       BIT(4)
#define IN_FIFO_OVF         BIT(5)
#define IN_FIFO_UDF         BIT(6)
#define IN_AHB_RESP_ERR     BIT(7)
#define OUT_DONE            BIT(0)
#define OUT_EOF             BIT(1)
#define OUT_DSCR_ERR        BIT(2)
#define OUT_TOTAL_EOF       BIT(3)
#define OUT_FIFO_OVF        BIT(4)
#define OUT_FIFO_UDF        BIT(5)
#define OUT_AHB_RESP_ERR    BIT(6)
#define OUT_AXI_LINK_SWITCH BIT(10)

/* *_LINK (GDMA-AXI's *_LINK1). */
#define IN_LINK_AUTO_RET    BIT(0)
#define IN_LINK_STOP        BIT(1)
#define IN_LINK_START       BIT(2)
#define IN_LINK_RESTART     BIT(3)
#define IN_LINK_PARK        BIT(4)
#define OUT_LINK_STOP       BIT(0)
#define OUT_LINK_START      BIT(1)
#define OUT_LINK_RESTART    BIT(2)
#define OUT_LINK_PARK       BIT(3)

#define CONF0_RST           BIT(0)
#define CONF0_LOOP_TEST     BIT(1)
#define CONF1_CHECK_OWNER   BIT(12)

/* Descriptor DW0. */
#define DW0_SIZE(d)         ((d) & 0xfff)
#define DW0_LENGTH(d)       (((d) >> 12) & 0xfff)
#define DW0_ERR_EOF         BIT(28)
#define DW0_SUC_EOF         BIT(30)
#define DW0_OWNER           BIT(31)

/* The FIFO entries' marks, for what their leaving completes. */
#define F_EOF               BIT(0)  /* the last byte of a frame */
#define F_ERR               BIT(1)  /* ... ended in error */
#define F_DESC_END          BIT(2)  /* a TX descriptor's last byte */
#define F_LIST_END          BIT(3)  /* ... of the last in its list */
#define F_NODATA            BIT(4)  /* a mark carrying no byte */

#define FIFO_SIZE           64

#define AHB_DATE_RESET      0x250220
#define AXI_DATE_RESET      0x2303140
#define AXI_MEM_RESET_START 0x30100000
#define AXI_MEM_RESET_END   0x8fffffff
#define MODULE_CLK_EN_RESET 0x7fff
#define MODULE_CLK_EN_MASK  0x18007fff
#define RESP_ERR_ID_RESET   0x1e
#define POP_EMPTY           0x800

/* GDMA-AHB's memory-to-memory master IDs (TRM 6.4.3). */
#define AHB_M2M_ID(ch)      ((ch) == 1 ? 11 : 12)
/* GDMA-AXI's port on the AXI interconnect. */
#define AXI_GDMA_ICM_CID    8

#define DESC_CYCLES         3

/* The registers, as both layouts decode to them. */
typedef enum {
    R_NONE,
    R_INT_RAW, R_INT_ST, R_INT_ENA, R_INT_CLR,
    R_CONF0, R_CONF1, R_FIFO_STATUS, R_PUSHPOP, R_LINK, R_LINK_ADDR,
    R_STATE, R_EOF_DES, R_EOF_BFR_DES, R_DSCR, R_DSCR_BF0, R_DSCR_BF1,
    R_PRI, R_PERI_SEL,
    R_CRC_INIT, R_CRC_WIDTH, R_CRC_CLEAR, R_CRC_FINAL, R_CRC_EN_WR,
    R_CRC_EN_ADDR, R_CRC_DATA_EN_WR, R_CRC_DATA_EN_ADDR,
    R_WEIGHT, R_WEIGHT_OPT, R_DONE_DES, R_FIFO_STATUS1, R_RESET_AVAIL,
    R_AHB_TEST, R_MISC_CONF, R_DATE, R_INTR_START, R_INTR_END,
    R_EXTR_START, R_EXTR_END, R_ARB_TIMEOUT, R_ARB_TIMEOUT_TX,
    R_ARB_TIMEOUT_RX, R_WEIGHT_EN, R_WEIGHT_EN_TX, R_WEIGHT_EN_RX,
    R_MODULE_CLK_EN, R_RESP_ERR0, R_RESP_ERR1, R_IN_MEM_CONF,
    R_RDN_RESULT, R_RDN_ECO_HIGH, R_RDN_ECO_LOW, R_WRESP_CNT, R_RRESP_CNT,
    R_LINK_SWITCH,
} GdmaReg;

typedef struct GdmaDecode {
    GdmaReg reg;
    int dir;
    int ch;
} GdmaDecode;

static const GdmaReg ahb_chan_regs[13] = {
    R_CONF0, R_CONF1, R_FIFO_STATUS, R_PUSHPOP, R_LINK, R_STATE,
    R_EOF_DES, R_EOF_BFR_DES, R_DSCR, R_DSCR_BF0, R_DSCR_BF1, R_PRI,
    R_PERI_SEL,
};

static const GdmaReg ahb_crc_regs[10] = {
    R_CRC_INIT, R_CRC_WIDTH, R_CRC_CLEAR, R_CRC_FINAL, R_CRC_EN_WR,
    R_CRC_EN_ADDR, R_CRC_DATA_EN_WR, R_CRC_DATA_EN_ADDR, R_WEIGHT,
    R_WEIGHT_OPT,
};

static const GdmaReg axi_chan_regs[26] = {
    R_INT_RAW, R_INT_ST, R_INT_ENA, R_INT_CLR, R_CONF0, R_CONF1,
    R_FIFO_STATUS, R_PUSHPOP, R_LINK, R_LINK_ADDR, R_STATE, R_EOF_DES,
    R_EOF_BFR_DES, R_DSCR, R_DSCR_BF0, R_DSCR_BF1, R_PRI, R_PERI_SEL,
    R_CRC_INIT, R_CRC_WIDTH, R_CRC_CLEAR, R_CRC_FINAL, R_CRC_EN_WR,
    R_CRC_EN_ADDR, R_CRC_DATA_EN_WR, R_CRC_DATA_EN_ADDR,
};

static GdmaDecode ahb_decode(hwaddr a)
{
    static const struct {
        hwaddr addr;
        GdmaReg reg;
    } globals[] = {
        { 0x060, R_AHB_TEST }, { 0x064, R_MISC_CONF }, { 0x068, R_DATE },
        { 0x3c4, R_INTR_START }, { 0x3c8, R_INTR_END },
        { 0x3cc, R_ARB_TIMEOUT_TX }, { 0x3d0, R_ARB_TIMEOUT_RX },
        { 0x3d4, R_WEIGHT_EN_TX }, { 0x3d8, R_WEIGHT_EN_RX },
        { 0x3dc, R_ARB_TIMEOUT }, { 0x400, R_WEIGHT_EN },
        { 0x404, R_MODULE_CLK_EN }, { 0x408, R_RESP_ERR0 },
        { 0x40c, R_RESP_ERR1 },
    };
    GdmaDecode d = { R_NONE, 0, 0 };
    hwaddr o;

    if (a < 0x60) {
        d.dir = a < 0x30 ? IN : OUT;
        d.ch = (a % 0x30) / 0x10;
        d.reg = R_INT_RAW + (a & 0xc) / 4;
    } else if (a >= 0x70 && a < 0x70 + NCH * 0xc0) {
        d.ch = (a - 0x70) / 0xc0;
        o = (a - 0x70) % 0xc0;
        if (o < 0x34) {
            d.dir = IN;
            d.reg = ahb_chan_regs[o / 4];
        } else if (o >= 0x60 && o < 0x94) {
            d.dir = OUT;
            d.reg = ahb_chan_regs[(o - 0x60) / 4];
        }
    } else if (a >= 0x2bc && a < 0x3ac) {
        d.dir = a < 0x334 ? OUT : IN;
        o = a - (a < 0x334 ? 0x2bc : 0x334);
        d.ch = o / 0x28;
        d.reg = ahb_crc_regs[(o % 0x28) / 4];
    } else if (a >= 0x3ac && a < 0x3c4) {
        d.dir = a < 0x3b8 ? IN : OUT;
        d.ch = ((a - 0x3ac) / 4) % NCH;
        d.reg = R_LINK_ADDR;
    } else if (a >= 0x410 && a < 0x428) {
        d.ch = (a - 0x410) / 8;
        d.dir = (a & 4) ? OUT : IN;
        d.reg = R_DONE_DES;
    } else {
        for (int i = 0; i < ARRAY_SIZE(globals); i++) {
            if (globals[i].addr == a) {
                d.reg = globals[i].reg;
            }
        }
    }
    return d;
}

static GdmaDecode axi_decode(hwaddr a)
{
    static const struct {
        hwaddr addr;
        GdmaReg reg;
    } globals[] = {
        { 0x270, R_ARB_TIMEOUT }, { 0x274, R_WEIGHT_EN },
        { 0x278, R_IN_MEM_CONF }, { 0x27c, R_INTR_START },
        { 0x280, R_INTR_END }, { 0x284, R_EXTR_START },
        { 0x288, R_EXTR_END }, { 0x2a8, R_MISC_CONF },
        { 0x2ac, R_RDN_RESULT }, { 0x2b0, R_RDN_ECO_HIGH },
        { 0x2b4, R_RDN_ECO_LOW }, { 0x2b8, R_WRESP_CNT },
        { 0x2bc, R_RRESP_CNT }, { 0x2d8, R_DATE }, { 0x2dc, R_LINK_SWITCH },
    };
    GdmaDecode d = { R_NONE, 0, 0 };

    if (a < 0x270) {
        d.dir = a < 0x138 ? IN : OUT;
        a -= d.dir == IN ? 0 : 0x138;
        d.ch = a / 0x68;
        d.reg = axi_chan_regs[(a % 0x68) / 4];
    } else if (a >= 0x28c && a < 0x2a4) {
        d.dir = a < 0x298 ? IN : OUT;
        d.ch = ((a - 0x28c) / 4) % NCH;
        d.reg = R_RESET_AVAIL;
    } else if (a >= 0x2c0 && a < 0x2d8) {
        d.dir = a < 0x2cc ? IN : OUT;
        d.ch = ((a - 0x2c0) / 4) % NCH;
        d.reg = R_FIFO_STATUS1;
    } else {
        for (int i = 0; i < ARRAY_SIZE(globals); i++) {
            if (globals[i].addr == a) {
                d.reg = globals[i].reg;
            }
        }
    }
    return d;
}

static bool gdma_axi(ESP32P4GdmaState *s)
{
    return ESP32P4_GDMA_GET_CLASS(s)->axi;
}

static const char *gdma_name(ESP32P4GdmaState *s)
{
    return gdma_axi(s) ? TYPE_ESP32P4_AXI_GDMA : TYPE_ESP32P4_AHB_GDMA;
}

/* Channel configuration */

static uint32_t int_mask(ESP32P4GdmaState *s, int dir)
{
    if (gdma_axi(s)) {
        return 0x7ff;
    }
    return dir == IN ? 0xff : 0x7f;
}

static uint32_t conf0_mask(ESP32P4GdmaState *s, int dir)
{
    if (gdma_axi(s)) {
        return dir == IN ? 0x3ff : 0x7ff;
    }
    return dir == IN ? 0xff : 0x37f;
}

static uint32_t conf0_reset(ESP32P4GdmaState *s, int dir)
{
    if (gdma_axi(s)) {
        return dir == IN ? 0 : 0x8;
    }
    return dir == IN ? 0x40 : 0x108;
}

static bool chan_etm_en(ESP32P4GdmaState *s, int dir, ESP32P4GdmaChan *c)
{
    if (gdma_axi(s)) {
        return c->conf0 & (dir == IN ? BIT(3) : BIT(4));
    }
    return c->conf0 & (dir == IN ? BIT(5) : BIT(6));
}

static bool chan_cmd_disabled(ESP32P4GdmaState *s, int dir,
                              ESP32P4GdmaChan *c)
{
    return gdma_axi(s) && (c->conf0 & (dir == IN ? BIT(7) : BIT(8)));
}

static bool chan_auto_wrback(ESP32P4GdmaChan *c)
{
    return c->conf0 & BIT(2);
}

static bool chan_eof_mode(ESP32P4GdmaChan *c)
{
    return c->conf0 & BIT(3);
}

/* The most bytes a data burst of channel @c moves. */
static uint32_t chan_burst(ESP32P4GdmaState *s, int dir, ESP32P4GdmaChan *c)
{
    uint32_t sel;

    if (gdma_axi(s)) {
        sel = extract32(c->conf0, dir == IN ? 4 : 5, 3);
        return 8u << MIN(sel, 4);
    }
    if (!(c->conf0 & (dir == IN ? BIT(3) : BIT(5)))) {
        return 4;
    }
    sel = extract32(c->conf0, dir == IN ? 6 : 8, 2);
    return sel ? 8u << sel : 4;
}

static uint32_t chan_priority(ESP32P4GdmaChan *c)
{
    return c->pri & 0xf;
}

static uint32_t chan_weight(ESP32P4GdmaState *s, ESP32P4GdmaChan *c)
{
    return gdma_axi(s) ? extract32(c->pri, 4, 4) : c->weight & 0xf;
}

static bool chan_weight_opt(ESP32P4GdmaState *s, ESP32P4GdmaChan *c)
{
    return !(gdma_axi(s) ? c->pri & BIT(8) : c->weight_opt_dis & 1);
}

/*
 * Whether channel @ch's TX output goes to its RX channel: MEM_TRANS_EN,
 * on the channels that have memory-to-memory transfers.
 */
static bool chan_m2m(ESP32P4GdmaState *s, int ch)
{
    uint32_t bit = gdma_axi(s) ? BIT(2) : BIT(4);

    return (s->ch[IN][ch].conf0 & bit) && (gdma_axi(s) || ch != 0);
}

/* Whether selection @sel is a peripheral rather than a dummy. */
static bool periph_real(ESP32P4GdmaState *s, uint32_t sel)
{
    if (gdma_axi(s)) {
        return sel <= ESP32P4_GDMA_AXI_SHA;
    }
    switch (sel) {
    case ESP32P4_GDMA_AHB_I3C:
    case ESP32P4_GDMA_AHB_UHCI0:
    case ESP32P4_GDMA_AHB_I2S0:
    case ESP32P4_GDMA_AHB_I2S1:
    case ESP32P4_GDMA_AHB_I2S2:
    case ESP32P4_GDMA_AHB_ADC:
    case ESP32P4_GDMA_AHB_RMT:
        return true;
    default:
        return false;
    }
}

/*
 * The attributes of the accesses of channel @ch of direction @dir: the DMA
 * master and ID of the peripheral it serves.
 */
static MemTxAttrs chan_attrs(ESP32P4GdmaState *s, int dir, int ch)
{
    static const uint8_t ahb_master[ESP32P4_GDMA_PERIPHS] = {
        [ESP32P4_GDMA_AHB_I3C] = ESP32P4_DMA_MASTER_AHB_PDMA_I3C_MST,
        [ESP32P4_GDMA_AHB_UHCI0] = ESP32P4_DMA_MASTER_AHB_PDMA_UHCI0,
        [ESP32P4_GDMA_AHB_I2S0] = ESP32P4_DMA_MASTER_AHB_PDMA_I2S0,
        [ESP32P4_GDMA_AHB_I2S1] = ESP32P4_DMA_MASTER_AHB_PDMA_I2S1,
        [ESP32P4_GDMA_AHB_I2S2] = ESP32P4_DMA_MASTER_AHB_PDMA_I2S2,
        [ESP32P4_GDMA_AHB_ADC] = ESP32P4_DMA_MASTER_AHB_PDMA_ADC,
        [ESP32P4_GDMA_AHB_RMT] = ESP32P4_DMA_MASTER_AHB_PDMA_RMT,
    };
    static const uint8_t axi_master[ESP32P4_GDMA_AXI_SHA + 1] = {
        [ESP32P4_GDMA_AXI_LCD_CAM] = ESP32P4_DMA_MASTER_AXI_PDMA_LCDCAM,
        [ESP32P4_GDMA_AXI_SPI2] = ESP32P4_DMA_MASTER_AXI_PDMA_GPSPI2,
        [ESP32P4_GDMA_AXI_SPI3] = ESP32P4_DMA_MASTER_AXI_PDMA_GPSPI3,
        [ESP32P4_GDMA_AXI_PARLIO] = ESP32P4_DMA_MASTER_AXI_PDMA_PARLIO,
        [ESP32P4_GDMA_AXI_AES] = ESP32P4_DMA_MASTER_AXI_PDMA_AES,
        [ESP32P4_GDMA_AXI_SHA] = ESP32P4_DMA_MASTER_AXI_PDMA_SHA,
    };
    uint32_t sel = s->ch[dir][ch].peri_sel;
    bool m2m = chan_m2m(s, ch);
    unsigned id = sel < ESP32P4_GDMA_PERIPHS ? sel : 15;

    if (gdma_axi(s)) {
        return esp32p4_dma_attrs(!m2m && periph_real(s, sel)
                                 ? axi_master[sel]
                                 : ESP32P4_DMA_MASTER_AXI_PDMA_DUMMY,
                                 ESP32P4_ICM_ID_AXI(AXI_GDMA_ICM_CID, id));
    }
    if (m2m) {
        id = AHB_M2M_ID(ch);
    }
    return esp32p4_dma_attrs(!m2m && periph_real(s, sel)
                             ? ahb_master[sel]
                             : ESP32P4_DMA_MASTER_AHB_PDMA_DUMMY,
                             ESP32P4_ICM_ID_AHB_PDMA(id));
}

/* Interrupts and events */

static void gdma_update_irq(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];

    qemu_set_irq(s->irq[dir * NCH + ch], !!(c->int_raw & c->int_ena));
}

static void gdma_raise(ESP32P4GdmaState *s, int dir, int ch, uint32_t bits)
{
    s->ch[dir][ch].int_raw |= bits;
    gdma_update_irq(s, dir, ch);
}

/* Pulse an ETM event, at the time of the round or access raising it. */
static void gdma_event(ESP32P4GdmaState *s, int kind, int ch)
{
    qemu_irq line = s->etm_event[ESP32P4_GDMA_ETM_EVT(kind, ch)];
    int64_t t = esp32_line_time();

    esp32_line_set_at(line, 1, t);
    esp32_line_set_at(line, 0, t);
}

/* FIFOs */

static void fifo_reset(ESP32P4GdmaFifo *f)
{
    f->head = 0;
    f->num = 0;
}

static uint32_t fifo_room(ESP32P4GdmaFifo *f)
{
    return FIFO_SIZE - f->num;
}

static void fifo_put(ESP32P4GdmaState *s, int dir, int ch, uint8_t byte,
                     uint8_t flags, uint32_t desc)
{
    ESP32P4GdmaFifo *f = &s->ch[dir][ch].fifo;
    uint32_t i = (f->head + f->num) % FIFO_SIZE;

    assert(f->num < FIFO_SIZE);
    f->data[i] = byte;
    f->flags[i] = flags;
    f->desc[i] = desc;
    f->num++;
    if (f->num == FIFO_SIZE) {
        gdma_event(s, dir == IN ? ESP32P4_GDMA_EVT_IN_FIFO_FULL
                                : ESP32P4_GDMA_EVT_OUT_FIFO_FULL, ch);
    }
}

static void fifo_take(ESP32P4GdmaState *s, int dir, int ch, uint8_t *byte,
                      uint8_t *flags, uint32_t *desc)
{
    ESP32P4GdmaFifo *f = &s->ch[dir][ch].fifo;

    assert(f->num > 0);
    *byte = f->data[f->head];
    *flags = f->flags[f->head];
    *desc = f->desc[f->head];
    f->head = (f->head + 1) % FIFO_SIZE;
    f->num--;
    if (f->num == 0) {
        gdma_event(s, dir == IN ? ESP32P4_GDMA_EVT_IN_FIFO_EMPTY
                                : ESP32P4_GDMA_EVT_OUT_FIFO_EMPTY, ch);
    }
}

/* CRC */

static uint32_t crc_bits(ESP32P4GdmaState *s, ESP32P4GdmaChan *c)
{
    uint32_t bits = 8 * ((c->crc_width & 3) + 1);

    return gdma_axi(s) ? MIN(bits, 16) : bits;
}

static uint32_t crc_mask(uint32_t bits)
{
    return bits == 32 ? UINT32_MAX : BIT(bits) - 1;
}

static void crc_restart(ESP32P4GdmaState *s, ESP32P4GdmaChan *c)
{
    c->crc = c->crc_init & crc_mask(crc_bits(s, c));
}

static void crc_update(ESP32P4GdmaState *s, ESP32P4GdmaChan *c,
                       const uint8_t *buf, uint32_t len)
{
    uint32_t bits = crc_bits(s, c);

    for (uint32_t i = 0; i < len; i++) {
        uint32_t next = 0;

        for (uint32_t x = 0; x < bits; x++) {
            uint32_t p = ctpop32(c->crc & c->crc_lfsr_mask[x]) ^
                         ctpop8(buf[i] & c->crc_data_mask[x]);

            next |= (p & 1) << x;
        }
        c->crc = next;
    }
}

/* Memory */

/* Whether [addr, addr + len) lies in memory the controller may reach. */
static bool gdma_reachable(ESP32P4GdmaState *s, uint32_t addr, uint32_t len)
{
    uint64_t end = (uint64_t)addr + len;

    if (addr >= s->intr_mem_start && end <= s->intr_mem_end) {
        return true;
    }
    return gdma_axi(s) && addr >= s->extr_mem_start &&
           end <= s->extr_mem_end;
}

/*
 * A bus access by channel @ch of direction @dir. GDMA-AHB records a bus
 * error response; GDMA-AXI has no such record (the APM's refusal is in the
 * ICM's).
 */
static bool gdma_bus(ESP32P4GdmaState *s, int dir, int ch, uint32_t addr,
                     void *buf, uint32_t len, bool write)
{
    MemTxAttrs attrs = chan_attrs(s, dir, ch);
    MemTxResult r = address_space_rw(s->as, addr, attrs, buf, len, write);

    if (r == MEMTX_OK) {
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: bus error on a %s "
                  "at 0x%08" PRIx32 "\n", gdma_name(s),
                  dir == IN ? "RX" : "TX", ch, write ? "write" : "read",
                  addr);
    if (!write) {
        memset(buf, 0, len);
    }
    if (!gdma_axi(s)) {
        s->resp_err_addr = addr;
        s->resp_err_info = (write ? 1 : 0) |
                           (ESP32P4_REQUESTER_ID(attrs.requester_id) & 0xf)
                           << 1 |
                           (ch | (dir == OUT ? 4 : 0)) << 5;
        gdma_raise(s, dir, ch, dir == IN ? IN_AHB_RESP_ERR
                                         : OUT_AHB_RESP_ERR);
    }
    return false;
}

static uint32_t bus_cycles(uint32_t bytes)
{
    return MAX(1, DIV_ROUND_UP(bytes, 4));
}

/* The link FSM */

static void chan_halt(ESP32P4GdmaChan *c)
{
    c->active = false;
    c->has_desc = false;
    c->end = false;
    c->restart = false;
}

static void chan_reset_fsm(ESP32P4GdmaChan *c)
{
    chan_halt(c);
    c->pos = 0;
    fifo_reset(&c->fifo);
}

static void gdma_kick(ESP32P4GdmaState *s);

/* [spec:nuos:req:emu.esp32p4.dma] */
static void chan_start(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];

    if (c->conf0 & CONF0_RST) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d started while "
                      "held in reset\n", gdma_name(s),
                      dir == IN ? "RX" : "TX", ch);
        return;
    }
    if (dir == IN && !gdma_axi(s) && ch == 0 &&
        (c->conf0 & BIT(4))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: memory-to-memory transfers are "
                      "on channels 1 and 2 only\n", gdma_name(s));
    }
    c->active = true;
    c->has_desc = false;
    c->end = false;
    c->restart = false;
    c->switch_pending = true;
    c->empty_raised = false;
    c->desc_next = c->link_addr;
    c->pos = 0;
    crc_restart(s, c);
    /* An RX channel takes bytes from now on: its peripheral may push. */
    if (dir == IN && !chan_m2m(s, ch) && c->peri_sel < ESP32P4_GDMA_PERIPHS) {
        s->notify |= BIT(16 + c->peri_sel);
    }
    gdma_kick(s);
}

/* Re-read the DW2 of the list's last descriptor; continue if it is set. */
static void chan_reread_next(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];
    uint32_t next = 0;

    if (!c->prev_desc) {
        c->end = true;
        return;
    }
    gdma_bus(s, dir, ch, c->prev_desc + 8, &next, 4, false);
    next = le32_to_cpu(next);
    if (next) {
        c->desc_next = next;
        c->end = false;
        c->switch_pending = true;
        c->empty_raised = false;
    } else {
        c->end = true;
    }
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static void chan_restart(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];

    if (c->conf0 & CONF0_RST) {
        return;
    }
    if (!c->active) {
        c->active = true;
        c->has_desc = false;
        c->pos = 0;
        if (c->prev_desc) {
            chan_reread_next(s, dir, ch);
        } else {
            c->desc_next = c->link_addr;
            c->end = false;
            c->switch_pending = true;
        }
    } else if (c->end) {
        chan_reread_next(s, dir, ch);
    } else {
        c->restart = true;
    }
    gdma_kick(s);
}

/*
 * Read the descriptor at the channel's link pointer and check it: 0 on a
 * descriptor error, which halts the channel, otherwise the bus cycles.
 */
/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t chan_fetch(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];
    uint32_t addr = c->desc_next;
    uint32_t align = gdma_axi(s) ? 8 : 4;
    uint32_t w[3];
    uint32_t n;
    const char *why = NULL;

    if (addr & (align - 1)) {
        why = "is misaligned";
    } else if (!gdma_reachable(s, addr, 12)) {
        why = "is outside the accessible memory";
    } else if (!gdma_bus(s, dir, ch, addr, w, sizeof(w), false)) {
        why = "could not be read";
    } else {
        c->desc_dw0 = le32_to_cpu(w[0]);
        c->desc_buf = le32_to_cpu(w[1]);
        c->desc_next = le32_to_cpu(w[2]);
        n = dir == IN ? DW0_SIZE(c->desc_dw0) : DW0_LENGTH(c->desc_dw0);
        if ((c->conf1 & CONF1_CHECK_OWNER) && !(c->desc_dw0 & DW0_OWNER)) {
            why = "is owned by the CPU";
        } else if (n && !gdma_reachable(s, c->desc_buf, n)) {
            why = "has a buffer outside the accessible memory";
        }
    }
    c->dscr_bf1 = c->dscr_bf0;
    c->dscr_bf0 = addr;
    c->dscr = why ? 0 : c->desc_next;
    if (why) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: the descriptor at "
                      "0x%08" PRIx32 " %s\n", gdma_name(s),
                      dir == IN ? "RX" : "TX", ch, addr, why);
        chan_halt(c);
        gdma_raise(s, dir, ch, dir == IN ? IN_DSCR_ERR : OUT_DSCR_ERR);
        return 0;
    }
    c->desc_addr = addr;
    c->has_desc = true;
    c->pos = 0;
    if (dir == OUT && c->switch_pending && gdma_axi(s) &&
        (s->link_switch_state & BIT(ch))) {
        s->link_switch_state &= ~BIT(ch);
        gdma_raise(s, OUT, ch, OUT_AXI_LINK_SWITCH);
    }
    c->switch_pending = false;
    return DESC_CYCLES;
}

/*
 * The list goes on from the descriptor just closed, or ends there, or
 * (after a RESTART during the run) goes on from its re-read DW2.
 */
static uint32_t chan_advance(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];

    c->prev_desc = c->desc_addr;
    c->has_desc = false;
    if (c->desc_next) {
        return 0;
    }
    if (c->restart) {
        c->restart = false;
        chan_reread_next(s, dir, ch);
        return 1;
    }
    c->end = true;
    return 0;
}

/* TX: memory to FIFO */

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t tx_service(ESP32P4GdmaState *s, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[OUT][ch];
    uint32_t len, n, cycles, i;
    uint8_t buf[128];
    uint8_t flags;
    bool eof;
    bool was_empty = c->fifo.num == 0;

    if (!c->has_desc) {
        return chan_fetch(s, OUT, ch);
    }
    len = DW0_LENGTH(c->desc_dw0);
    n = MIN(MIN(len - c->pos, chan_burst(s, OUT, c)), fifo_room(&c->fifo));
    if (!fifo_room(&c->fifo)) {
        return 0;
    }
    cycles = 0;
    if (n) {
        gdma_bus(s, OUT, ch, c->desc_buf + c->pos, buf, n, false);
        crc_update(s, c, buf, n);
        cycles = bus_cycles(n);
    }
    c->pos += n;
    eof = c->desc_dw0 & DW0_SUC_EOF;
    for (i = 0; i < n || (n == 0 && i == 0); i++) {
        flags = n ? 0 : F_NODATA;
        if (c->pos == len && (n == 0 || i == n - 1)) {
            flags |= F_DESC_END | (eof ? F_EOF : 0);
        }
        fifo_put(s, OUT, ch, n ? buf[i] : 0, flags, c->desc_addr);
    }
    if (c->pos == len) {
        if (eof && !chan_eof_mode(c)) {
            c->eof_bfr_des_addr = c->prev_desc;
            c->eof_des_addr = c->desc_addr;
            gdma_raise(s, OUT, ch, OUT_EOF);
            gdma_event(s, ESP32P4_GDMA_EVT_OUT_EOF, ch);
        }
        if (chan_auto_wrback(c) && !(c->conf0 & CONF0_LOOP_TEST)) {
            uint32_t dw0 = cpu_to_le32(c->desc_dw0 & ~DW0_OWNER);

            gdma_bus(s, OUT, ch, c->desc_addr, &dw0, 4, true);
            cycles++;
        }
        cycles += chan_advance(s, OUT, ch);
        /* The list's last byte, unless a RESTART has carried it on. */
        if (c->end) {
            c->fifo.flags[(c->fifo.head + c->fifo.num - 1) % FIFO_SIZE] |=
                F_LIST_END;
        }
    }
    if (was_empty && c->fifo.num && !chan_m2m(s, ch) &&
        c->peri_sel < ESP32P4_GDMA_PERIPHS) {
        s->notify |= BIT(c->peri_sel);
    }
    return MAX(cycles, 1);
}

/* The events of a byte, or a mark, leaving a TX FIFO. */
/* [spec:nuos:req:emu.esp32p4.dma] */
static void tx_left(ESP32P4GdmaState *s, int ch, uint8_t flags,
                    uint32_t desc)
{
    ESP32P4GdmaChan *c = &s->ch[OUT][ch];
    uint32_t bits = 0;

    if (!(flags & F_DESC_END)) {
        return;
    }
    if (flags & F_EOF) {
        if (chan_eof_mode(c)) {
            c->eof_bfr_des_addr = c->done_des_addr;
            c->eof_des_addr = desc;
            bits |= OUT_EOF;
            gdma_event(s, ESP32P4_GDMA_EVT_OUT_EOF, ch);
        }
        if (flags & F_LIST_END) {
            bits |= OUT_TOTAL_EOF;
            gdma_event(s, ESP32P4_GDMA_EVT_OUT_TOTAL_EOF, ch);
        }
    }
    c->done_des_addr = desc;
    bits |= OUT_DONE;
    gdma_event(s, ESP32P4_GDMA_EVT_OUT_DONE, ch);
    gdma_raise(s, OUT, ch, bits);
}

/* RX: FIFO to memory */

/* Whether RX channel @ch takes bytes into its FIFO. */
static bool rx_accepts(ESP32P4GdmaState *s, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[IN][ch];

    return c->active && !(c->conf0 & CONF0_RST) &&
           !chan_cmd_disabled(s, IN, c);
}

/* Bytes reached an RX channel whose list has ended. */
static void rx_check_empty(ESP32P4GdmaState *s, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[IN][ch];

    if (c->active && c->end && c->fifo.num && !c->empty_raised) {
        c->empty_raised = true;
        gdma_raise(s, IN, ch, IN_DSCR_EMPTY);
    }
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t rx_close(ESP32P4GdmaState *s, int ch, bool eof, bool err)
{
    ESP32P4GdmaChan *c = &s->ch[IN][ch];
    uint32_t dw0, bits = IN_DONE;

    if (err && (c->link & IN_LINK_AUTO_RET)) {
        /* The frame in error is dropped and the descriptor used again. */
        c->eof_bfr_des_addr = c->desc_addr;
        c->pos = 0;
        gdma_raise(s, IN, ch, IN_ERR_EOF);
        return 0;
    }
    dw0 = DW0_SIZE(c->desc_dw0) | (c->pos << 12) |
          (eof ? DW0_SUC_EOF : 0) | (err ? DW0_ERR_EOF : 0) |
          (c->conf0 & CONF0_LOOP_TEST ? c->desc_dw0 & DW0_OWNER : 0);
    dw0 = cpu_to_le32(dw0);
    gdma_bus(s, IN, ch, c->desc_addr, &dw0, 4, true);
    c->done_des_addr = c->desc_addr;
    gdma_event(s, ESP32P4_GDMA_EVT_IN_DONE, ch);
    if (eof) {
        c->eof_des_addr = c->desc_addr;
        bits |= IN_SUC_EOF;
        gdma_event(s, ESP32P4_GDMA_EVT_IN_SUC_EOF, ch);
    }
    if (err) {
        c->eof_bfr_des_addr = c->desc_addr;
        bits |= IN_ERR_EOF;
    }
    gdma_raise(s, IN, ch, bits);
    return 1 + chan_advance(s, IN, ch);
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t rx_service(ESP32P4GdmaState *s, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[IN][ch];
    uint32_t size, room, burst, n = 0, cycles = 0, desc;
    uint8_t buf[128], byte, flags = 0;
    bool full = c->fifo.num == FIFO_SIZE;

    if (!c->has_desc) {
        return chan_fetch(s, IN, ch);
    }
    size = DW0_SIZE(c->desc_dw0);
    room = size - c->pos;
    burst = chan_burst(s, IN, c);
    while (n < MIN(room, burst) && c->fifo.num) {
        fifo_take(s, IN, ch, &byte, &flags, &desc);
        if (!(flags & F_NODATA)) {
            buf[n++] = byte;
        }
        if (flags & (F_EOF | F_ERR)) {
            break;
        }
        flags = 0;
    }
    if (n) {
        gdma_bus(s, IN, ch, c->desc_buf + c->pos, buf, n, true);
        crc_update(s, c, buf, n);
        c->pos += n;
        cycles = bus_cycles(n);
    }
    if (flags & (F_EOF | F_ERR) || c->pos == size) {
        cycles += rx_close(s, ch, flags & F_EOF, flags & F_ERR);
    }
    rx_check_empty(s, ch);
    if (full && c->fifo.num < FIFO_SIZE && !chan_m2m(s, ch) &&
        s->ch[IN][ch].peri_sel < ESP32P4_GDMA_PERIPHS) {
        s->notify |= BIT(16 + s->ch[IN][ch].peri_sel);
    }
    return MAX(cycles, 1);
}

/* Memory-to-memory: TX FIFO to RX FIFO, outside the bus */

/* [spec:nuos:req:emu.esp32p4.dma] */
static void m2m_move(ESP32P4GdmaState *s, int ch)
{
    ESP32P4GdmaChan *tx = &s->ch[OUT][ch];
    ESP32P4GdmaChan *rx = &s->ch[IN][ch];
    uint8_t byte, flags;
    uint32_t desc;

    if (!chan_m2m(s, ch)) {
        return;
    }
    while (tx->fifo.num && rx_accepts(s, ch) && fifo_room(&rx->fifo)) {
        fifo_take(s, OUT, ch, &byte, &flags, &desc);
        tx_left(s, ch, flags, desc);
        if (!(flags & F_NODATA) || (flags & F_EOF)) {
            fifo_put(s, IN, ch, byte,
                     (flags & F_NODATA) | (flags & F_EOF), 0);
        }
    }
    rx_check_empty(s, ch);
}

/* Arbitration */

static bool chan_requests(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];

    if (!c->active || c->end || (c->conf0 & CONF0_RST) ||
        chan_cmd_disabled(s, dir, c)) {
        return false;
    }
    if (!c->has_desc) {
        return true;
    }
    if (dir == OUT) {
        return fifo_room(&c->fifo) > 0;
    }
    return c->fifo.num > 0 || DW0_SIZE(c->desc_dw0) == c->pos;
}

static bool arb_weighted(ESP32P4GdmaState *s, int arb)
{
    if (gdma_axi(s)) {
        return s->weight_en & BIT(arb);
    }
    return s->weight_en & 1;
}

/* The weighted arbiter's time slot, in bus cycles. */
static uint32_t arb_slot_cycles(ESP32P4GdmaState *s, int arb)
{
    return gdma_axi(s) ? extract32(s->arb_timeout, 16 * arb, 16)
                       : s->arb_timeout & 0xffff;
}

/* Requester @i of arbiter @arb: GDMA-AHB's are all six channels. */
static void arb_member(ESP32P4GdmaState *s, int arb, int i, int *dir,
                       int *ch)
{
    if (gdma_axi(s)) {
        *dir = arb == 0 ? OUT : IN;
        *ch = i;
    } else {
        *dir = i / NCH;
        *ch = i % NCH;
    }
}

static int arb_members(ESP32P4GdmaState *s)
{
    return gdma_axi(s) ? NCH : 2 * NCH;
}

static void arb_refill(ESP32P4GdmaState *s, int arb, int64_t cycle)
{
    int dir, ch;

    for (int i = 0; i < arb_members(s); i++) {
        arb_member(s, arb, i, &dir, &ch);
        s->ch[dir][ch].tokens = chan_weight(s, &s->ch[dir][ch]);
    }
    s->slot_start[arb] = cycle;
}

/*
 * The requester arbiter @arb grants at bus cycle @now, or -1; *hold is
 * set when a weighted slot holds the bus for a channel without requests.
 */
/* [spec:nuos:req:emu.esp32p4.dma] */
static int arb_grant(ESP32P4GdmaState *s, int arb, int64_t cycle,
                     bool *hold)
{
    int n = arb_members(s);
    bool weighted = arb_weighted(s, arb);
    bool pri = !(s->misc_conf & (gdma_axi(s) ? BIT(3) : BIT(2)));
    int best = -1, dir, ch;
    uint32_t best_pri = 0;

    *hold = false;
    if (weighted) {
        bool any = false, held = false;

        if (cycle - s->slot_start[arb] >= arb_slot_cycles(s, arb)) {
            arb_refill(s, arb, cycle);
        }
        for (int i = 0; i < n; i++) {
            ESP32P4GdmaChan *c;

            arb_member(s, arb, i, &dir, &ch);
            c = &s->ch[dir][ch];
            if (!c->tokens) {
                continue;
            }
            if (chan_requests(s, dir, ch)) {
                any = true;
            } else if (!chan_weight_opt(s, c)) {
                held = true;
            }
        }
        if (!any) {
            if (held) {
                *hold = true;
                return -1;
            }
            arb_refill(s, arb, cycle);
        }
    }
    for (int k = 1; k <= n; k++) {
        int i = (s->rr_last[arb] + k) % n;
        ESP32P4GdmaChan *c;

        arb_member(s, arb, i, &dir, &ch);
        c = &s->ch[dir][ch];
        if (!chan_requests(s, dir, ch) || (weighted && !c->tokens)) {
            continue;
        }
        if (best < 0 || (pri && chan_priority(c) > best_pri)) {
            best = i;
            best_pri = chan_priority(c);
        }
    }
    if (best >= 0) {
        s->rr_last[arb] = best;
        if (weighted) {
            arb_member(s, arb, best, &dir, &ch);
            s->ch[dir][ch].tokens--;
        }
    }
    return best;
}

/*
 * Whether an access by channel @ch of direction @dir waits on a memory
 * controller's AXI port (esp32p4_mspi_dma_wait()).
 */
static bool gdma_waits(ESP32P4GdmaState *s, int dir, int ch, uint32_t addr,
                       uint32_t len, bool write)
{
    return len && esp32p4_mspi_dma_wait(s->as, addr, len,
                                        chan_attrs(s, dir, ch), write,
                                        &s->bus_wait);
}

/*
 * The bytes RX channel @ch's next data burst takes from its FIFO, and in
 * *close whether it then closes its descriptor, *err in error.
 */
static uint32_t rx_plan(ESP32P4GdmaState *s, int ch, bool *close, bool *err)
{
    ESP32P4GdmaChan *c = &s->ch[IN][ch];
    uint32_t size = DW0_SIZE(c->desc_dw0);
    uint32_t most = MIN(size - c->pos, chan_burst(s, IN, c));
    uint32_t n = 0, i;
    uint8_t flags = 0;

    for (i = 0; n < most && i < c->fifo.num; i++) {
        flags = c->fifo.flags[(c->fifo.head + i) % FIFO_SIZE];
        if (!(flags & F_NODATA)) {
            n++;
        }
        if (flags & (F_EOF | F_ERR)) {
            break;
        }
        flags = 0;
    }
    *err = flags & F_ERR;
    *close = (flags & (F_EOF | F_ERR)) || c->pos + n == size;
    return n;
}

/*
 * [spec:nuos:req:emu.esp32p4.dma]
 * Whether the transaction channel @ch of direction @dir would make next
 * waits on a memory controller's AXI port: its descriptor read, its data
 * burst, and the descriptor write-back and the re-read of the next
 * pointer that close it. On the chip the transaction is under way and
 * the bus waits with it; the model makes none of it until the port takes
 * them all.
 */
static bool chan_stalls(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];
    uint32_t align = gdma_axi(s) ? 8 : 4;
    bool close, err = false;
    uint32_t n;

    if (!c->has_desc) {
        return !(c->desc_next & (align - 1)) &&
               gdma_reachable(s, c->desc_next, 12) &&
               gdma_waits(s, dir, ch, c->desc_next, 12, false);
    }
    if (dir == OUT) {
        uint32_t len = DW0_LENGTH(c->desc_dw0);

        if (!fifo_room(&c->fifo)) {
            return false;
        }
        n = MIN(MIN(len - c->pos, chan_burst(s, OUT, c)),
                fifo_room(&c->fifo));
        close = c->pos + n == len;
        if (gdma_waits(s, OUT, ch, c->desc_buf + c->pos, n, false) ||
            (close && chan_auto_wrback(c) &&
             !(c->conf0 & CONF0_LOOP_TEST) &&
             gdma_waits(s, OUT, ch, c->desc_addr, 4, true))) {
            return true;
        }
    } else {
        n = rx_plan(s, ch, &close, &err);
        if (gdma_waits(s, IN, ch, c->desc_buf + c->pos, n, true)) {
            return true;
        }
        if (close && err && (c->link & IN_LINK_AUTO_RET)) {
            return false;
        }
        if (close && gdma_waits(s, IN, ch, c->desc_addr, 4, true)) {
            return true;
        }
    }
    return close && !c->desc_next && c->restart &&
           gdma_waits(s, dir, ch, c->desc_addr + 8, 4, false);
}

static uint32_t arb_serve(ESP32P4GdmaState *s, int arb, int i)
{
    int dir, ch;

    arb_member(s, arb, i, &dir, &ch);
    return dir == OUT ? tx_service(s, ch) : rx_service(s, ch);
}

/*
 * Tell whoever takes the path of direction @dir of peripheral @periph of
 * the channel's readiness: an interposer, or the peripheral.
 */
static void gdma_ready(ESP32P4GdmaState *s, unsigned periph, int dir)
{
    const ESP32P4GdmaInterposerOps *ip = s->ip_ops[dir][periph];

    if (ip) {
        if (ip->chan_ready) {
            ip->chan_ready(s->ip_opaque[dir][periph], dir);
        }
        return;
    }
    esp32p4_gdma_periph_ready(s, periph, dir);
}

static void gdma_notify(ESP32P4GdmaState *s)
{
    uint32_t pending = s->notify;

    s->notify = 0;
    for (int p = 0; p < ESP32P4_GDMA_PERIPHS; p++) {
        if (pending & BIT(p)) {
            gdma_ready(s, p, OUT);
        }
        if (pending & BIT(16 + p)) {
            gdma_ready(s, p, IN);
        }
    }
}

/* Arm the engine's timer for a round at bus cycle @cycle. */
static void gdma_arm(ESP32P4GdmaState *s, int64_t cycle)
{
    s->due_cycle = cycle;
    s->due_ns = esp32p4_clk_count_time(&s->bus_edges, cycle);
    timer_mod(s->timer, s->due_ns);
}

/*
 * One round of the bus, at the bus cycle it falls due, at
 * esp32_line_time(): each arbiter grants a transaction, whose effects are
 * seen at once, and the engine comes back when it is over.
 */
/* [spec:nuos:req:emu.esp32p4.dma] */
static void gdma_run(void *opaque)
{
    ESP32P4GdmaState *s = opaque;
    int64_t cycle = s->due_cycle;
    int narb = gdma_axi(s) ? 2 : 1;
    uint32_t cost = 0;
    int64_t wait = 0;

    if (!clock_is_enabled(s->clk)) {
        return;
    }
    for (int ch = 0; ch < NCH; ch++) {
        m2m_move(s, ch);
    }
    for (int arb = 0; arb < narb; arb++) {
        bool hold = false;
        int g = s->stall_grant[arb], dir, ch;

        /* A transaction the AXI port held goes first, once it takes it. */
        s->stall_grant[arb] = -1;
        if (g >= 0) {
            arb_member(s, arb, g, &dir, &ch);
            if (!chan_requests(s, dir, ch)) {
                g = -1;
            }
        }
        if (g < 0) {
            g = arb_grant(s, arb, cycle, &hold);
        }
        if (g >= 0) {
            arb_member(s, arb, g, &dir, &ch);
        }
        if (g >= 0 && chan_stalls(s, dir, ch)) {
            s->stall_grant[arb] = g;
        } else if (g >= 0) {
            cost = MAX(cost, MAX(1, arb_serve(s, arb, g)));
        } else if (hold) {
            /* Wait for the weighted time slot to end. */
            int64_t end = s->slot_start[arb] + arb_slot_cycles(s, arb);

            if (end > cycle && (!wait || end - cycle < wait)) {
                wait = end - cycle;
            }
        }
    }
    for (int ch = 0; ch < NCH; ch++) {
        m2m_move(s, ch);
    }
    if (cost) {
        gdma_arm(s, cycle + cost);
    } else if (wait) {
        gdma_arm(s, cycle + wait);
    }
    gdma_notify(s);
}

/* The round falls due: run it at its time, which a late timer has passed. */
static void gdma_timer_cb(void *opaque)
{
    ESP32P4GdmaState *s = opaque;

    esp32_line_run_at(s->due_ns, gdma_run, s);
}

/*
 * [spec:nuos:req:emu.esp32p4.dma]
 * Run the rounds that fell due by @now, in order, each at its own time.
 * Each comes back at least a bus cycle later, or not at all.
 */
static void gdma_catch_up(ESP32P4GdmaState *s, int64_t now)
{
    while (timer_pending(s->timer) && s->due_ns <= now) {
        timer_del(s->timer);
        esp32_line_run_at(s->due_ns, gdma_run, s);
    }
}

/* [spec:nuos:req:emu.esp32p4.dma] */
void esp32p4_gdma_chan_sync(ESP32P4GdmaState *s, int64_t when)
{
    gdma_catch_up(s, when);
}

/* [spec:nuos:req:emu.esp32p4.dma] */
void esp32p4_gdma_sync(ESP32P4GdmaState *s, int64_t when)
{
    for (int dir = 0; dir < 2; dir++) {
        for (int p = 0; p < ESP32P4_GDMA_PERIPHS; p++) {
            const ESP32P4GdmaInterposerOps *ip = s->ip_ops[dir][p];

            if (ip && ip->sync) {
                ip->sync(s->ip_opaque[dir][p], when);
            }
        }
    }
    gdma_catch_up(s, when);
}

/*
 * Bring everything that moves data through the controller up to @now:
 * the attached peripherals that take their own events late, each running
 * the controller's rounds up to every event it takes, then the
 * interposers' and the rounds left.
 */
static void gdma_catch_up_all(ESP32P4GdmaState *s, int64_t now)
{
    for (int p = 0; p < ESP32P4_GDMA_PERIPHS; p++) {
        const ESP32P4GdmaPeriphOps *ops = s->periph_ops[p];

        if (ops && ops->sync) {
            ops->sync(s->periph_opaque[p], now);
        }
    }
    esp32p4_gdma_sync(s, now);
}

/*
 * Let the engine look for work at the bus cycle after the change that
 * calls for it, at esp32_line_time().
 */
static void gdma_kick(ESP32P4GdmaState *s)
{
    if (!timer_pending(s->timer) && clock_is_enabled(s->clk)) {
        gdma_arm(s, esp32p4_clk_count_at(&s->bus_edges,
                                         esp32_line_time()) + 1);
    }
}

/*
 * A memory controller's AXI port takes accesses again: the transactions
 * it held go on, from the bus cycle after.
 */
static void gdma_bus_ready(Notifier *n, void *data)
{
    ESP32P4GdmaState *s = container_of(n, ESP32P4GdmaState, bus_wait);

    notifier_remove(n);
    gdma_kick(s);
}

/*
 * The bus clock changes: the rounds due before run at the old rate, and
 * the engine goes on at the new one, or stops, its cycles counted afresh
 * from the change; a round under way takes the cycles it has left.
 */
static void gdma_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4GdmaState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t cycles;

    if (event == ClockPreUpdate) {
        gdma_catch_up_all(s, now);
        return;
    }
    cycles = esp32p4_clk_count_at(&s->bus_edges, now);
    s->slot_start[0] -= cycles;
    s->slot_start[1] -= cycles;
    s->due_cycle -= cycles;
    esp32p4_clk_count_start(&s->bus_edges, s->clk, now);
    if (!clock_is_enabled(s->clk)) {
        timer_del(s->timer);
    } else if (timer_pending(s->timer)) {
        gdma_arm(s, MAX(s->due_cycle, 1));
    } else {
        gdma_kick(s);
    }
}

/* The peripheral side */

/* The channel of direction @dir serving peripheral @periph, or -1. */
static int periph_chan(ESP32P4GdmaState *s, unsigned periph, int dir)
{
    for (int ch = 0; ch < NCH; ch++) {
        ESP32P4GdmaChan *c = &s->ch[dir][ch];

        if (c->peri_sel == periph && !chan_m2m(s, ch) &&
            !(c->conf0 & CONF0_RST)) {
            return ch;
        }
    }
    return -1;
}

void esp32p4_gdma_attach(ESP32P4GdmaState *s, unsigned periph,
                         const ESP32P4GdmaPeriphOps *ops, void *opaque)
{
    assert(periph < ESP32P4_GDMA_PERIPHS && periph_real(s, periph));
    s->periph_ops[periph] = ops;
    s->periph_opaque[periph] = opaque;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
void esp32p4_gdma_interpose(ESP32P4GdmaState *s, int dir, unsigned periph,
                            const ESP32P4GdmaInterposerOps *ops,
                            void *opaque)
{
    assert(periph < ESP32P4_GDMA_PERIPHS && (dir == IN || dir == OUT));
    s->ip_ops[dir][periph] = ops;
    s->ip_opaque[dir][periph] = ops ? opaque : NULL;
}

/* [spec:nuos:req:emu.esp32p4.parlio] */
void esp32p4_gdma_periph_ready(ESP32P4GdmaState *s, unsigned periph,
                               int dir)
{
    const ESP32P4GdmaPeriphOps *ops;

    assert(periph < ESP32P4_GDMA_PERIPHS);
    ops = s->periph_ops[periph];
    if (!ops) {
        return;
    }
    if (dir == OUT && ops->tx_ready) {
        ops->tx_ready(s->periph_opaque[periph]);
    } else if (dir == IN && ops->rx_ready) {
        ops->rx_ready(s->periph_opaque[periph]);
    }
}

size_t esp32p4_gdma_chan_tx_pending(ESP32P4GdmaState *s, unsigned periph)
{
    int ch = periph_chan(s, periph, OUT);

    return ch < 0 ? 0 : s->ch[OUT][ch].fifo.num;
}

size_t esp32p4_gdma_chan_rx_room(ESP32P4GdmaState *s, unsigned periph)
{
    int ch = periph_chan(s, periph, IN);

    return ch < 0 || !rx_accepts(s, ch) ? 0
                                         : fifo_room(&s->ch[IN][ch].fifo);
}

bool esp32p4_gdma_tx_flowing(ESP32P4GdmaState *s, unsigned periph)
{
    int ch = periph_chan(s, periph, OUT);
    ESP32P4GdmaChan *c;

    if (ch < 0) {
        return false;
    }
    c = &s->ch[OUT][ch];
    if (c->fifo.num) {
        return true;
    }
    return clock_is_enabled(s->clk) && c->active && !c->end &&
           !chan_cmd_disabled(s, OUT, c);
}

bool esp32p4_gdma_rx_flowing(ESP32P4GdmaState *s, unsigned periph)
{
    int ch = periph_chan(s, periph, IN);
    ESP32P4GdmaChan *c;

    if (ch < 0 || !rx_accepts(s, ch)) {
        return false;
    }
    c = &s->ch[IN][ch];
    if (fifo_room(&c->fifo)) {
        return true;
    }
    return clock_is_enabled(s->clk) && (c->has_desc || !c->end);
}

/* [spec:nuos:req:emu.esp32p4.dma] */
size_t esp32p4_gdma_chan_pull(ESP32P4GdmaState *s, unsigned periph,
                              uint8_t *buf, size_t len, bool *eof)
{
    int ch = periph_chan(s, periph, OUT);
    ESP32P4GdmaChan *c;
    size_t n = 0;
    uint8_t byte, flags;
    uint32_t desc;

    *eof = false;
    if (ch < 0) {
        return 0;
    }
    c = &s->ch[OUT][ch];
    while (c->fifo.num && n < len) {
        fifo_take(s, OUT, ch, &byte, &flags, &desc);
        if (!(flags & F_NODATA)) {
            buf[n++] = byte;
        }
        tx_left(s, ch, flags, desc);
        if (flags & F_EOF) {
            *eof = true;
            break;
        }
    }
    gdma_kick(s);
    return n;
}

/* [spec:nuos:req:emu.esp32p4.dma] */
size_t esp32p4_gdma_chan_push(ESP32P4GdmaState *s, unsigned periph,
                              const uint8_t *buf, size_t len, unsigned flags)
{
    int ch = periph_chan(s, periph, IN);
    ESP32P4GdmaChan *c;
    uint8_t mark = (flags & ESP32P4_GDMA_EOF ? F_EOF : 0) |
                   (flags & ESP32P4_GDMA_ERR_EOF ? F_ERR : 0);
    size_t n;

    if (ch < 0 || !rx_accepts(s, ch)) {
        return 0;
    }
    c = &s->ch[IN][ch];
    if (!len) {
        if (mark && fifo_room(&c->fifo)) {
            fifo_put(s, IN, ch, 0, F_NODATA | mark, 0);
        }
    } else {
        n = MIN(len, fifo_room(&c->fifo));
        for (size_t i = 0; i < n; i++) {
            fifo_put(s, IN, ch, buf[i], i == len - 1 ? mark : 0, 0);
        }
        len = n;
    }
    rx_check_empty(s, ch);
    gdma_kick(s);
    return len;
}

size_t esp32p4_gdma_tx_pending(ESP32P4GdmaState *s, unsigned periph)
{
    const ESP32P4GdmaInterposerOps *ip = s->ip_ops[OUT][periph];

    if (ip) {
        return ip->tx_pending(s->ip_opaque[OUT][periph]);
    }
    return esp32p4_gdma_chan_tx_pending(s, periph);
}

size_t esp32p4_gdma_rx_room(ESP32P4GdmaState *s, unsigned periph)
{
    const ESP32P4GdmaInterposerOps *ip = s->ip_ops[IN][periph];

    if (ip) {
        return ip->rx_room(s->ip_opaque[IN][periph]);
    }
    return esp32p4_gdma_chan_rx_room(s, periph);
}

size_t esp32p4_gdma_pull(ESP32P4GdmaState *s, unsigned periph,
                         uint8_t *buf, size_t len, bool *eof)
{
    const ESP32P4GdmaInterposerOps *ip = s->ip_ops[OUT][periph];

    if (ip) {
        *eof = false;
        return ip->pull(s->ip_opaque[OUT][periph], buf, len, eof);
    }
    return esp32p4_gdma_chan_pull(s, periph, buf, len, eof);
}

size_t esp32p4_gdma_push(ESP32P4GdmaState *s, unsigned periph,
                         const uint8_t *buf, size_t len, unsigned flags)
{
    const ESP32P4GdmaInterposerOps *ip = s->ip_ops[IN][periph];

    if (ip) {
        return ip->push(s->ip_opaque[IN][periph], buf, len, flags);
    }
    return esp32p4_gdma_chan_push(s, periph, buf, len, flags);
}

/* Registers */

static uint32_t fifo_status(ESP32P4GdmaState *s, int dir, int ch)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];
    uint32_t num = c->fifo.num;
    bool full = num == FIFO_SIZE || (dir == IN && !rx_accepts(s, ch));
    bool empty = num == 0;

    if (gdma_axi(s)) {
        /* The L3 FIFO's bits, then the L1 and L2 FIFOs, empty. */
        return (full ? BIT(0) : 0) | (empty ? BIT(1) : 0) |
               MIN(num, 63) << 2 | BIT(11) | BIT(15) |
               (dir == OUT ? 0x7f800000 : 0);
    }
    return (full ? BIT(0) : 0) | (empty ? BIT(1) : 0) | num << 8 |
           0x07800000;
}

static uint32_t link_read(ESP32P4GdmaState *s, int dir, ESP32P4GdmaChan *c)
{
    bool park = !c->active || (c->end && !c->has_desc);

    if (dir == IN) {
        return (c->link & IN_LINK_AUTO_RET) | (park ? IN_LINK_PARK : 0);
    }
    return park ? OUT_LINK_PARK : 0;
}

static bool gdma_reg_read(ESP32P4GdmaState *s, GdmaDecode d, uint32_t *v)
{
    ESP32P4GdmaChan *c = &s->ch[d.dir][d.ch];

    switch (d.reg) {
    case R_INT_RAW:
        *v = c->int_raw;
        break;
    case R_INT_ST:
        *v = c->int_raw & c->int_ena;
        break;
    case R_INT_ENA:
        *v = c->int_ena;
        break;
    case R_INT_CLR:
        *v = 0;
        break;
    case R_CONF0:
        *v = c->conf0;
        break;
    case R_CONF1:
        *v = c->conf1;
        break;
    case R_FIFO_STATUS:
        *v = fifo_status(s, d.dir, d.ch);
        break;
    case R_PUSHPOP:
        *v = c->pop;
        break;
    case R_LINK:
        *v = link_read(s, d.dir, c);
        break;
    case R_LINK_ADDR:
        *v = c->link_addr;
        break;
    case R_STATE:
        *v = (c->has_desc ? c->desc_addr : c->desc_next) & 0x3ffff;
        break;
    case R_EOF_DES:
        *v = c->eof_des_addr;
        break;
    case R_EOF_BFR_DES:
        *v = c->eof_bfr_des_addr;
        break;
    case R_DSCR:
        *v = c->dscr;
        break;
    case R_DSCR_BF0:
        *v = c->dscr_bf0;
        break;
    case R_DSCR_BF1:
        *v = c->dscr_bf1;
        break;
    case R_PRI:
        *v = c->pri;
        break;
    case R_PERI_SEL:
        *v = c->peri_sel;
        break;
    case R_CRC_INIT:
        *v = c->crc_init;
        break;
    case R_CRC_WIDTH:
        *v = c->crc_width;
        break;
    case R_CRC_CLEAR:
        *v = c->crc_clear;
        break;
    case R_CRC_FINAL:
        *v = c->crc;
        break;
    case R_CRC_EN_WR:
        *v = c->crc_en_wr_data;
        break;
    case R_CRC_EN_ADDR:
        *v = c->crc_en_addr;
        break;
    case R_CRC_DATA_EN_WR:
        *v = c->crc_data_en_wr_data;
        break;
    case R_CRC_DATA_EN_ADDR:
        *v = c->crc_data_en_addr;
        break;
    case R_WEIGHT:
        *v = c->weight;
        break;
    case R_WEIGHT_OPT:
        *v = c->weight_opt_dis;
        break;
    case R_DONE_DES:
        *v = c->done_des_addr;
        break;
    case R_FIFO_STATUS1:
        *v = 0;
        break;
    case R_RESET_AVAIL:
        /* Every transaction completes at once: a reset may always go. */
        *v = 1;
        break;
    case R_AHB_TEST:
        *v = s->ahb_test;
        break;
    case R_MISC_CONF:
        *v = s->misc_conf;
        break;
    case R_DATE:
        *v = s->date;
        break;
    case R_INTR_START:
        *v = s->intr_mem_start;
        break;
    case R_INTR_END:
        *v = s->intr_mem_end;
        break;
    case R_EXTR_START:
        *v = s->extr_mem_start;
        break;
    case R_EXTR_END:
        *v = s->extr_mem_end;
        break;
    case R_ARB_TIMEOUT:
        *v = s->arb_timeout;
        break;
    case R_ARB_TIMEOUT_TX:
        *v = s->arb_timeout_tx;
        break;
    case R_ARB_TIMEOUT_RX:
        *v = s->arb_timeout_rx;
        break;
    case R_WEIGHT_EN:
        *v = s->weight_en;
        break;
    case R_WEIGHT_EN_TX:
        *v = s->weight_en_tx;
        break;
    case R_WEIGHT_EN_RX:
        *v = s->weight_en_rx;
        break;
    case R_MODULE_CLK_EN:
        *v = s->module_clk_en;
        break;
    case R_RESP_ERR0:
        *v = s->resp_err_addr;
        break;
    case R_RESP_ERR1:
        *v = s->resp_err_info;
        break;
    case R_IN_MEM_CONF:
        *v = s->in_mem_conf;
        break;
    case R_RDN_RESULT:
        *v = s->rdn_result;
        break;
    case R_RDN_ECO_HIGH:
        *v = s->rdn_eco_high;
        break;
    case R_RDN_ECO_LOW:
        *v = s->rdn_eco_low;
        break;
    case R_WRESP_CNT:
    case R_RRESP_CNT:
        *v = 0;
        break;
    case R_LINK_SWITCH:
        *v = s->link_switch_state;
        break;
    default:
        *v = 0;
        return false;
    }
    return true;
}

static uint64_t esp32p4_gdma_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4GdmaState *s = opaque;
    hwaddr reg = addr & ~3;
    GdmaDecode d = gdma_axi(s) ? axi_decode(reg) : ahb_decode(reg);
    uint32_t v;

    gdma_catch_up_all(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (!gdma_reg_read(s, d, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", gdma_name(s), addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

static void crc_width_write(ESP32P4GdmaState *s, ESP32P4GdmaChan *c,
                            uint32_t v)
{
    /* Latching on the flag's rising edge loads one row of the matrix. */
    if ((v & BIT(2)) && !(c->crc_width & BIT(2))) {
        if (c->crc_en_addr < 32) {
            c->crc_lfsr_mask[c->crc_en_addr] = c->crc_en_wr_data;
        }
        if (c->crc_data_en_addr < 32) {
            c->crc_data_mask[c->crc_data_en_addr] =
                c->crc_data_en_wr_data & 0xff;
        }
    }
    c->crc_width = v & 7;
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static void link_write(ESP32P4GdmaState *s, int dir, int ch, uint32_t v)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];
    uint32_t stop = dir == IN ? IN_LINK_STOP : OUT_LINK_STOP;
    uint32_t start = dir == IN ? IN_LINK_START : OUT_LINK_START;
    uint32_t restart = dir == IN ? IN_LINK_RESTART : OUT_LINK_RESTART;

    if (dir == IN) {
        c->link = v & IN_LINK_AUTO_RET;
    }
    if (v & stop) {
        chan_halt(c);
    }
    if (v & start) {
        if (chan_etm_en(s, dir, c)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: LINK_START "
                          "is ignored while ETM_EN is set\n", gdma_name(s),
                          dir == IN ? "RX" : "TX", ch);
        } else {
            chan_start(s, dir, ch);
        }
    }
    if (v & restart) {
        chan_restart(s, dir, ch);
    }
}

static void pushpop_write(ESP32P4GdmaState *s, int dir, int ch, uint32_t v)
{
    ESP32P4GdmaChan *c = &s->ch[dir][ch];
    uint8_t byte, flags;
    uint32_t desc;

    if (dir == OUT) {
        c->pop = v & 0x1ff;
        if (!(v & BIT(9))) {
            return;
        }
        if (!fifo_room(&c->fifo)) {
            gdma_raise(s, OUT, ch, OUT_FIFO_OVF);
            return;
        }
        fifo_put(s, OUT, ch, v & 0xff, v & BIT(8) ? F_EOF : 0, 0);
        gdma_kick(s);
        return;
    }
    if (!(v & BIT(12))) {
        return;
    }
    if (!c->fifo.num) {
        c->pop = POP_EMPTY;
        gdma_raise(s, IN, ch, IN_FIFO_UDF);
        return;
    }
    fifo_take(s, IN, ch, &byte, &flags, &desc);
    c->pop = byte | (flags & (F_EOF | F_ERR) ? BIT(8) : 0);
    gdma_kick(s);
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static bool gdma_reg_write(ESP32P4GdmaState *s, GdmaDecode d, uint32_t v)
{
    ESP32P4GdmaChan *c = &s->ch[d.dir][d.ch];
    bool axi = gdma_axi(s);

    switch (d.reg) {
    case R_INT_RAW:
    case R_INT_CLR:
        /* INT_RAW is write-1-to-clear, as INT_CLR is. */
        c->int_raw &= ~v;
        gdma_update_irq(s, d.dir, d.ch);
        break;
    case R_INT_ENA:
        c->int_ena = v & int_mask(s, d.dir);
        gdma_update_irq(s, d.dir, d.ch);
        break;
    case R_CONF0:
        c->conf0 = v & conf0_mask(s, d.dir);
        if (c->conf0 & CONF0_RST) {
            chan_reset_fsm(c);
        }
        gdma_kick(s);
        break;
    case R_CONF1:
        c->conf1 = v & CONF1_CHECK_OWNER;
        break;
    case R_PUSHPOP:
        pushpop_write(s, d.dir, d.ch, v);
        break;
    case R_LINK:
        link_write(s, d.dir, d.ch, v);
        break;
    case R_LINK_ADDR:
        c->link_addr = v;
        break;
    case R_PRI:
        c->pri = v & (axi ? 0x1ff : 0xf);
        break;
    case R_PERI_SEL:
        c->peri_sel = v & 0x3f;
        if (c->peri_sel >= ESP32P4_GDMA_PERIPHS &&
            c->peri_sel != ESP32P4_GDMA_PERI_NONE) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: %s channel %d: invalid "
                          "peripheral %u\n", gdma_name(s),
                          d.dir == IN ? "RX" : "TX", d.ch, c->peri_sel);
        }
        gdma_kick(s);
        break;
    case R_CRC_INIT:
        c->crc_init = v;
        break;
    case R_CRC_WIDTH:
        crc_width_write(s, c, v);
        break;
    case R_CRC_CLEAR:
        if ((v & 1) && !(c->crc_clear & 1)) {
            crc_restart(s, c);
        }
        c->crc_clear = v & 1;
        break;
    case R_CRC_EN_WR:
        c->crc_en_wr_data = v;
        break;
    case R_CRC_EN_ADDR:
        c->crc_en_addr = v;
        break;
    case R_CRC_DATA_EN_WR:
        c->crc_data_en_wr_data = v & (axi ? 0xffff : 0xff);
        break;
    case R_CRC_DATA_EN_ADDR:
        c->crc_data_en_addr = v;
        break;
    case R_WEIGHT:
        c->weight = v & 0xf;
        break;
    case R_WEIGHT_OPT:
        c->weight_opt_dis = v & 1;
        break;
    case R_AHB_TEST:
        s->ahb_test = v & 0x37;
        break;
    case R_MISC_CONF:
        s->misc_conf = v & (axi ? 0x1b : 0xd);
        break;
    case R_DATE:
        s->date = v;
        break;
    case R_INTR_START:
        s->intr_mem_start = v;
        break;
    case R_INTR_END:
        s->intr_mem_end = v;
        break;
    case R_EXTR_START:
        s->extr_mem_start = v;
        break;
    case R_EXTR_END:
        s->extr_mem_end = v;
        break;
    case R_ARB_TIMEOUT:
        s->arb_timeout = axi ? v : v & 0xffff;
        break;
    case R_ARB_TIMEOUT_TX:
        s->arb_timeout_tx = v & 0xffff;
        break;
    case R_ARB_TIMEOUT_RX:
        s->arb_timeout_rx = v & 0xffff;
        break;
    case R_WEIGHT_EN:
        s->weight_en = v & (axi ? 3 : 1);
        gdma_kick(s);
        break;
    case R_WEIGHT_EN_TX:
        s->weight_en_tx = v & 1;
        break;
    case R_WEIGHT_EN_RX:
        s->weight_en_rx = v & 1;
        break;
    case R_MODULE_CLK_EN:
        s->module_clk_en = v & MODULE_CLK_EN_MASK;
        break;
    case R_IN_MEM_CONF:
        s->in_mem_conf = v & 0x3f;
        break;
    case R_RDN_RESULT:
        s->rdn_result = v & 1;
        break;
    case R_RDN_ECO_HIGH:
        s->rdn_eco_high = v;
        break;
    case R_RDN_ECO_LOW:
        s->rdn_eco_low = v;
        break;
    case R_LINK_SWITCH:
        s->link_switch_state = v & 7;
        break;
    case R_INT_ST:
    case R_FIFO_STATUS:
    case R_STATE:
    case R_EOF_DES:
    case R_EOF_BFR_DES:
    case R_DSCR:
    case R_DSCR_BF0:
    case R_DSCR_BF1:
    case R_CRC_FINAL:
    case R_DONE_DES:
    case R_FIFO_STATUS1:
    case R_RESET_AVAIL:
    case R_RESP_ERR0:
    case R_RESP_ERR1:
    case R_WRESP_CNT:
    case R_RRESP_CNT:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to a read-only register\n",
                      gdma_name(s));
        break;
    default:
        return false;
    }
    return true;
}

static void esp32p4_gdma_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    ESP32P4GdmaState *s = opaque;
    hwaddr reg = addr & ~3;
    GdmaDecode d = gdma_axi(s) ? axi_decode(reg) : ahb_decode(reg);
    uint32_t old = 0, v;

    gdma_catch_up_all(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    /*
     * A narrow write keeps the other bytes; the write-1 bits of the
     * interrupt clears and the link and push/pop commands act only in the
     * bytes written.
     */
    switch (d.reg) {
    case R_INT_RAW:
    case R_INT_CLR:
        break;
    case R_LINK:
        old = d.dir == IN ? s->ch[IN][d.ch].link & IN_LINK_AUTO_RET : 0;
        break;
    case R_PUSHPOP:
        old = d.dir == OUT ? s->ch[OUT][d.ch].pop : 0;
        break;
    default:
        gdma_reg_read(s, d, &old);
        break;
    }
    v = deposit32(old, (addr & 3) * 8, size * 8, value);
    if (!gdma_reg_write(s, d, v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", gdma_name(s), addr);
    }
}

static const MemoryRegionOps esp32p4_gdma_ops = {
    .read = esp32p4_gdma_read,
    .write = esp32p4_gdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * ETM tasks: a rising edge starts the channel if its ETM_EN is set, at
 * the event's time, which the task's pulse carries.
 */
/* [spec:nuos:req:emu.esp32p4.dma] */
static void esp32p4_gdma_etm_task(void *opaque, int n, int level)
{
    ESP32P4GdmaState *s = opaque;
    int dir = n < NCH ? IN : OUT;
    int ch = n % NCH;
    bool rise = level && !s->etm_task_level[n];

    s->etm_task_level[n] = level;
    if (rise && chan_etm_en(s, dir, &s->ch[dir][ch])) {
        gdma_catch_up(s, esp32_line_time());
        chan_start(s, dir, ch);
    }
}

/*
 * The qtest hook through which a test plays a peripheral, for those with
 * no model yet: setting "periph" to "push <periph> <hex bytes> <flags>"
 * pushes the bytes as esp32p4_gdma_push() does, and reading it back gives
 * how many were taken; "pull <periph> <count>" pulls as
 * esp32p4_gdma_pull() does, and reading back gives "<count> <eof> <hex
 * bytes>".
 */
static char *esp32p4_gdma_get_periph(Object *obj, Error **errp)
{
    ESP32P4GdmaState *s = ESP32P4_GDMA(obj);

    return g_strdup(s->periph_probe ? s->periph_probe : "");
}

static void esp32p4_gdma_set_periph(Object *obj, const char *value,
                                    Error **errp)
{
    ESP32P4GdmaState *s = ESP32P4_GDMA(obj);
    g_auto(GStrv) f = g_strsplit(value, " ", 0);
    unsigned int periph, n, flags;
    g_autofree uint8_t *buf = NULL;
    GString *out;
    bool eof;
    size_t len;

    if (g_strv_length(f) < 3 || qemu_strtoui(f[1], NULL, 0, &periph) ||
        periph >= ESP32P4_GDMA_PERIPHS) {
        error_setg(errp, "periph: want \"push <periph> <hex> <flags>\" or "
                   "\"pull <periph> <count>\"");
        return;
    }
    g_free(s->periph_probe);
    s->periph_probe = NULL;
    if (!strcmp(f[0], "push") && g_strv_length(f) >= 4 &&
        !qemu_strtoui(f[3], NULL, 0, &flags)) {
        len = strlen(f[2]) / 2;
        buf = g_malloc(len + 1);
        for (size_t i = 0; i < len; i++) {
            char byte[3] = { f[2][2 * i], f[2][2 * i + 1], 0 };

            buf[i] = strtoul(byte, NULL, 16);
        }
        s->periph_probe = g_strdup_printf("%zu", esp32p4_gdma_push(s, periph,
                                          buf, len, flags));
    } else if (!strcmp(f[0], "pull") && !qemu_strtoui(f[2], NULL, 0, &n)) {
        buf = g_malloc(n + 1);
        len = esp32p4_gdma_pull(s, periph, buf, n, &eof);
        out = g_string_new(NULL);
        g_string_printf(out, "%zu %d ", len, eof);
        for (size_t i = 0; i < len; i++) {
            g_string_append_printf(out, "%02x", buf[i]);
        }
        s->periph_probe = g_string_free(out, false);
    } else {
        error_setg(errp, "periph: unknown command '%s'", value);
    }
}

static void esp32p4_gdma_reset_hold(Object *obj, ResetType type)
{
    ESP32P4GdmaState *s = ESP32P4_GDMA(obj);
    bool axi = gdma_axi(s);

    timer_del(s->timer);
    for (int dir = 0; dir < 2; dir++) {
        for (int ch = 0; ch < NCH; ch++) {
            ESP32P4GdmaChan *c = &s->ch[dir][ch];

            memset(c, 0, sizeof(*c));
            c->conf0 = conf0_reset(s, dir);
            c->link = dir == IN ? IN_LINK_AUTO_RET : 0;
            c->peri_sel = ESP32P4_GDMA_PERI_NONE;
            c->crc_init = UINT32_MAX;
            c->pop = dir == IN ? POP_EMPTY : 0;
        }
    }
    s->ahb_test = 0;
    s->misc_conf = 0;
    s->date = axi ? AXI_DATE_RESET : AHB_DATE_RESET;
    s->intr_mem_start = axi ? AXI_MEM_RESET_START : 0;
    s->intr_mem_end = axi ? AXI_MEM_RESET_END : UINT32_MAX;
    s->extr_mem_start = axi ? AXI_MEM_RESET_START : 0;
    s->extr_mem_end = axi ? AXI_MEM_RESET_END : 0;
    s->arb_timeout = 0;
    s->arb_timeout_tx = 0;
    s->arb_timeout_rx = 0;
    s->weight_en = 0;
    s->weight_en_tx = 0;
    s->weight_en_rx = 0;
    s->module_clk_en = axi ? 0 : MODULE_CLK_EN_RESET;
    s->resp_err_addr = 0;
    s->resp_err_info = axi ? 0 : RESP_ERR_ID_RESET;
    s->in_mem_conf = 0;
    s->rdn_result = 0;
    s->rdn_eco_high = axi ? UINT32_MAX : 0;
    s->rdn_eco_low = 0;
    s->link_switch_state = 0;
    s->rr_last[0] = s->rr_last[1] = 0;
    s->slot_start[0] = s->slot_start[1] = 0;
    s->stall_grant[0] = s->stall_grant[1] = -1;
    if (QLIST_IS_INSERTED(&s->bus_wait, node)) {
        notifier_remove(&s->bus_wait);
    }
    if (!esp32p4_clk_count_running(&s->bus_edges)) {
        esp32p4_clk_count_start(&s->bus_edges, s->clk,
                                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    s->notify = 0;
}

static void esp32p4_gdma_reset_exit(Object *obj, ResetType type)
{
    ESP32P4GdmaState *s = ESP32P4_GDMA(obj);

    for (int i = 0; i < 2 * NCH; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void esp32p4_gdma_init(Object *obj)
{
    ESP32P4GdmaState *s = ESP32P4_GDMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    bool axi = ESP32P4_GDMA_GET_CLASS(obj)->axi;

    memory_region_init_io(&s->iomem, obj, &esp32p4_gdma_ops, s,
                          axi ? "esp32p4.axi-dma" : "esp32p4.ahb-dma",
                          axi ? ESP32P4_AXI_GDMA_SIZE
                              : ESP32P4_AHB_GDMA_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < 2 * NCH; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
    qdev_init_gpio_out_named(DEVICE(obj), s->etm_event,
                             ESP32P4_GDMA_ETM_EVENT, ESP32P4_GDMA_ETM_EVENTS);
    qdev_init_gpio_in_named(DEVICE(obj), esp32p4_gdma_etm_task,
                            ESP32P4_GDMA_ETM_TASK, ESP32P4_GDMA_ETM_TASKS);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", gdma_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, gdma_timer_cb, s);
    s->bus_wait.notify = gdma_bus_ready;
    s->stall_grant[0] = s->stall_grant[1] = -1;
    object_property_add_str(obj, "periph", esp32p4_gdma_get_periph,
                            esp32p4_gdma_set_periph);
}

static void esp32p4_gdma_realize(DeviceState *dev, Error **errp)
{
    ESP32P4GdmaState *s = ESP32P4_GDMA(dev);

    if (!s->pms) {
        error_setg(errp, "%s: the 'pms' link is not set",
                   object_get_typename(OBJECT(dev)));
        return;
    }
    s->as = esp32p4_pms_bus_as(ESP32P4_PMS(s->pms));
}

static void esp32p4_gdma_finalize(Object *obj)
{
    timer_free(ESP32P4_GDMA(obj)->timer);
    g_free(ESP32P4_GDMA(obj)->periph_probe);
}

static const Property esp32p4_gdma_properties[] = {
    DEFINE_PROP_LINK("pms", ESP32P4GdmaState, pms, TYPE_ESP32P4_PMS,
                     Object *),
};

static const VMStateDescription vmstate_esp32p4_gdma_fifo = {
    .name = "esp32p4-gdma-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, ESP32P4GdmaFifo, ESP32P4_GDMA_FIFO_MAX),
        VMSTATE_UINT8_ARRAY(flags, ESP32P4GdmaFifo, ESP32P4_GDMA_FIFO_MAX),
        VMSTATE_UINT32_ARRAY(desc, ESP32P4GdmaFifo, ESP32P4_GDMA_FIFO_MAX),
        VMSTATE_UINT32(head, ESP32P4GdmaFifo),
        VMSTATE_UINT32(num, ESP32P4GdmaFifo),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_gdma_chan = {
    .name = "esp32p4-gdma-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(int_raw, ESP32P4GdmaChan),
        VMSTATE_UINT32(int_ena, ESP32P4GdmaChan),
        VMSTATE_UINT32(conf0, ESP32P4GdmaChan),
        VMSTATE_UINT32(conf1, ESP32P4GdmaChan),
        VMSTATE_UINT32(link, ESP32P4GdmaChan),
        VMSTATE_UINT32(link_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32(pri, ESP32P4GdmaChan),
        VMSTATE_UINT32(peri_sel, ESP32P4GdmaChan),
        VMSTATE_UINT32(weight, ESP32P4GdmaChan),
        VMSTATE_UINT32(weight_opt_dis, ESP32P4GdmaChan),
        VMSTATE_UINT32(eof_des_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32(eof_bfr_des_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32(dscr, ESP32P4GdmaChan),
        VMSTATE_UINT32(dscr_bf0, ESP32P4GdmaChan),
        VMSTATE_UINT32(dscr_bf1, ESP32P4GdmaChan),
        VMSTATE_UINT32(done_des_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32(pop, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_init, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_width, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_clear, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_en_wr_data, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_en_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_data_en_wr_data, ESP32P4GdmaChan),
        VMSTATE_UINT32(crc_data_en_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32_ARRAY(crc_lfsr_mask, ESP32P4GdmaChan, 32),
        VMSTATE_UINT32_ARRAY(crc_data_mask, ESP32P4GdmaChan, 32),
        VMSTATE_UINT32(crc, ESP32P4GdmaChan),
        VMSTATE_BOOL(active, ESP32P4GdmaChan),
        VMSTATE_BOOL(has_desc, ESP32P4GdmaChan),
        VMSTATE_BOOL(end, ESP32P4GdmaChan),
        VMSTATE_BOOL(restart, ESP32P4GdmaChan),
        VMSTATE_BOOL(switch_pending, ESP32P4GdmaChan),
        VMSTATE_BOOL(empty_raised, ESP32P4GdmaChan),
        VMSTATE_UINT32(desc_addr, ESP32P4GdmaChan),
        VMSTATE_UINT32(desc_dw0, ESP32P4GdmaChan),
        VMSTATE_UINT32(desc_buf, ESP32P4GdmaChan),
        VMSTATE_UINT32(desc_next, ESP32P4GdmaChan),
        VMSTATE_UINT32(pos, ESP32P4GdmaChan),
        VMSTATE_UINT32(prev_desc, ESP32P4GdmaChan),
        VMSTATE_UINT32(tokens, ESP32P4GdmaChan),
        VMSTATE_STRUCT(fifo, ESP32P4GdmaChan, 1, vmstate_esp32p4_gdma_fifo,
                       ESP32P4GdmaFifo),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * A transaction an AXI port held: ask again, which waits on the port anew
 * if it still holds it.
 */
static int esp32p4_gdma_post_load(void *opaque, int version_id)
{
    ESP32P4GdmaState *s = opaque;

    if (s->stall_grant[0] >= 0 || s->stall_grant[1] >= 0) {
        gdma_kick(s);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32p4_gdma = {
    .name = TYPE_ESP32P4_GDMA,
    .version_id = 4,
    .minimum_version_id = 4,
    .post_load = esp32p4_gdma_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_2DARRAY(ch, ESP32P4GdmaState, 2, NCH, 1,
                               vmstate_esp32p4_gdma_chan, ESP32P4GdmaChan),
        VMSTATE_UINT32(ahb_test, ESP32P4GdmaState),
        VMSTATE_UINT32(misc_conf, ESP32P4GdmaState),
        VMSTATE_UINT32(date, ESP32P4GdmaState),
        VMSTATE_UINT32(intr_mem_start, ESP32P4GdmaState),
        VMSTATE_UINT32(intr_mem_end, ESP32P4GdmaState),
        VMSTATE_UINT32(extr_mem_start, ESP32P4GdmaState),
        VMSTATE_UINT32(extr_mem_end, ESP32P4GdmaState),
        VMSTATE_UINT32(arb_timeout, ESP32P4GdmaState),
        VMSTATE_UINT32(arb_timeout_tx, ESP32P4GdmaState),
        VMSTATE_UINT32(arb_timeout_rx, ESP32P4GdmaState),
        VMSTATE_UINT32(weight_en, ESP32P4GdmaState),
        VMSTATE_UINT32(weight_en_tx, ESP32P4GdmaState),
        VMSTATE_UINT32(weight_en_rx, ESP32P4GdmaState),
        VMSTATE_UINT32(module_clk_en, ESP32P4GdmaState),
        VMSTATE_UINT32(resp_err_addr, ESP32P4GdmaState),
        VMSTATE_UINT32(resp_err_info, ESP32P4GdmaState),
        VMSTATE_UINT32(in_mem_conf, ESP32P4GdmaState),
        VMSTATE_UINT32(rdn_result, ESP32P4GdmaState),
        VMSTATE_UINT32(rdn_eco_high, ESP32P4GdmaState),
        VMSTATE_UINT32(rdn_eco_low, ESP32P4GdmaState),
        VMSTATE_UINT32(link_switch_state, ESP32P4GdmaState),
        VMSTATE_BOOL_ARRAY(etm_task_level, ESP32P4GdmaState,
                           ESP32P4_GDMA_ETM_TASKS),
        VMSTATE_UINT32_ARRAY(rr_last, ESP32P4GdmaState, 2),
        VMSTATE_INT64_ARRAY(slot_start, ESP32P4GdmaState, 2),
        VMSTATE_INT32_ARRAY(stall_grant, ESP32P4GdmaState, 2),
        VMSTATE_CLOCK(clk, ESP32P4GdmaState),
        VMSTATE_ESP32P4_CLK_COUNT(bus_edges, ESP32P4GdmaState),
        VMSTATE_INT64(due_cycle, ESP32P4GdmaState),
        VMSTATE_INT64(due_ns, ESP32P4GdmaState),
        VMSTATE_TIMER_PTR(timer, ESP32P4GdmaState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_gdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_gdma_realize;
    dc->vmsd = &vmstate_esp32p4_gdma;
    device_class_set_props(dc, esp32p4_gdma_properties);
    rc->phases.hold = esp32p4_gdma_reset_hold;
    rc->phases.exit = esp32p4_gdma_reset_exit;
}

static void esp32p4_axi_gdma_class_init(ObjectClass *klass, const void *data)
{
    ESP32P4_GDMA_CLASS(klass)->axi = true;
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static const TypeInfo esp32p4_gdma_types[] = {
    {
        .name           = TYPE_ESP32P4_GDMA,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(ESP32P4GdmaState),
        .instance_init  = esp32p4_gdma_init,
        .instance_finalize = esp32p4_gdma_finalize,
        .class_size     = sizeof(ESP32P4GdmaClass),
        .class_init     = esp32p4_gdma_class_init,
        .abstract       = true,
    }, {
        .name           = TYPE_ESP32P4_AHB_GDMA,
        .parent         = TYPE_ESP32P4_GDMA,
    }, {
        .name           = TYPE_ESP32P4_AXI_GDMA,
        .parent         = TYPE_ESP32P4_GDMA,
        .class_init     = esp32p4_axi_gdma_class_init,
    },
};

DEFINE_TYPES(esp32p4_gdma_types)
