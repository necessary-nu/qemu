/*
 * ESP32-P4 VDMA, a Synopsys DesignWare AXI DMA controller (DW_axi_dmac)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter 7
 * (VDMA Controller), with the layout, access types and reset values of
 * ESP-IDF's dw_gdma_reg.h for v3 and the way its dw_gdma_ll driver uses
 * them. The P4's DW_axi_dmac has four channels, two AXI master
 * interfaces, a 64-bit data bus, 64 x 64-bit channel FIFOs and three
 * hardware handshaking interfaces (MIPI DSI, ISP to memory, memory to
 * ISP); its registers are 32 bits wide, the IP's 64-bit ones split in
 * halves, and only the low half of an address is decoded.
 *
 * A channel moves blocks of (BLOCK_TS + 1) items of SRC_TR_WIDTH from SAR
 * to DAR, through its FIFO: source transactions read SRC_MSIZE items (or
 * one, for a peripheral's single request) into the FIFO, destination
 * transactions write DST_MSIZE items of DST_TR_WIDTH from it, each
 * address stepping on (SINC/DINC 0) or staying (1). A peripheral source or
 * destination paces its side through its handshake (include/hw/dma/
 * esp32p4_dw_gdma.h); a peripheral that is the flow controller (TT_FC 4
 * to 7) ends the block with its last request, and BLOCK_TS does not
 * count. Memory needs no handshake. SRC_TRANSCOMP and DST_TRANSCOMP are
 * raised as a handshaking side's transactions complete. A burst
 * transaction of more items than the channel's FIFO holds (MSIZE 128 and
 * up at 64 bits, as ESP-IDF's MIPI DSI and CSI drivers use) moves in
 * pieces as the FIFO fills or empties, and its request is answered as its
 * last piece goes; a destination burst the block's end cuts short is
 * answered then.
 *
 * Multi-block transfers (TRM 7.5.4.2, table 7.5-1), by CFG0's
 * SRC_MULTBLK_TYPE and DST_MULTBLK_TYPE: a contiguous side's address
 * goes on from where the block ended; an auto-reloaded side's reloads
 * the value the channel was enabled with; a shadow-register side's loads
 * the shadow registers, which software's writes to SAR, DAR, BLOCK_TS,
 * CTL0 and CTL1 reach while the channel runs, and which must hold
 * SHADOWREG_OR_LLI_VALID; a linked-list side's loads the next LLI from
 * LLP0 (64-byte aligned, on the AXI master LMS selects), whose
 * SHADOWREG_OR_LLI_VALID must be set. A shadow or LLI not valid raises
 * SHADOWREG_OR_LLI_INVALID_ERR and the channel waits for
 * BLK_TFR_RESUMEREQ to fetch it again; LLP0 still points at the LLI in
 * error. CTL and BLOCK_TS follow a linked-list side, else a shadow side,
 * else an auto-reloaded side. Shadow and linked-list transfers end after
 * the block whose SHADOWREG_OR_LLI_LAST is set; contiguous and reloaded
 * ones after a block at whose end both MULTBLK_TYPEs are contiguous, and
 * between blocks they wait, if they raised BLOCK_TFR_DONE, until software
 * clears it. At each block's end an LLI is written back (TRM figure
 * 7.5-6): CTL0, CTL1 with VALID clear, SSTAT and DSTAT as SRC_STAT_EN and
 * DST_STAT_EN fetch them, and the status, BLOCK_TFR_DONE and DMA_TFR_DONE
 * last. MULTBLK_TYPE pairs 2-3 and 3-2 raise SLVIF_MULTIBLKTYPE_ERR and
 * the channel does not start.
 *
 * Interrupts: an event sets its CHn_INTSTATUS bit if INTSTATUS_ENABLE
 * lets it, BLOCK_TFR_DONE only for a block with IOC_BLKTFR; the interrupt
 * output is high while CFG0.INT_EN is set and a status bit is set whose
 * INTSIGNAL_ENABLE bit is. DMAC_INTSTATUS shows which channels (and the
 * common registers) signal one. The register file reports its misuse as
 * the IP does: a write to a read-only register, a read of a write-only
 * one, an undefined offset, a write to a running channel's registers
 * (SLVIF_WRONCHEN_ERR, but CFG0, which software clears to end a reloaded
 * transfer, and in shadow mode the shadow registers), and a write to the
 * shadow registers while they hold a valid block. INTSTATUS_ENABLE and
 * INTSIGNAL_ENABLE have the bits of SHADOWREG_OR_LLI_INVALID_ERR and
 * SLVIF_SHADOWREG_WRON_VALID_ERR, as the IP has; ESP-IDF relies on the
 * former, which v3's header omits.
 *
 * Channel control: CHEN's EN, SUSP and ABORT bits change only with their
 * write-enable bits. Suspending stops the source, drains the FIFO, then
 * raises CH_SRC_SUSPENDED and CH_SUSPENDED; disabling a running channel
 * does the same and then disables it, raising CH_DISABLED; aborting
 * drops the FIFO and disables it at once, raising CH_ABORTED. A bus error
 * (an access the DMA APM refuses, SLV_ERR, or an address its AXI master
 * has no path to, DEC_ERR) raises the SRC_, DST_ or LLI_ error and ends
 * the transfer: the channel is disabled. Both masters reach the memories
 * at their 0x4... addresses, master 1 also the CSI and DSI bridges'
 * buffers (TRM 7.4, 9.3.4): the bus decides (hw/misc/esp32p4_pms.c), and
 * an address above 32 bits reaches nothing.
 *
 * Every access is DMA master GDMA_CHn (n the channel less one) on the bus
 * through the APMs (include/hw/misc/esp32p4_pms.h), recorded by the ICM
 * with AXI port 5 (master 1) or 6 (master 2) and the channel as unit.
 *
 * Timing: at AXI_DMAC_CORE_CLK, the controller's GDMA_SYS_CLK, each
 * transaction takes a bus cycle per 64-bit beat, an LLI fetch five; the
 * rounds fall on the clock's edges, counted exactly from its last
 * change. A
 * read arbiter (LLI fetches first, then source reads) and a write arbiter
 * (destination writes) each grant one transaction at a time, to the
 * highest CH_PRIOR, round robin among equals where the TRM says the
 * choice is random. Each round of the bus happens at the time it falls
 * due, however late the QEMU timer that runs it (esp32_line_run_at()),
 * and the next is timed from it; register accesses, the handshake inputs
 * and clock changes first run the rounds due before them, in order.
 *
 * Not modelled: the low-power clock gating, the AXI IDs, QoS, cache and
 * protection attributes, burst lengths (ARLEN, AWLEN), outstanding limits
 * and non-posted writes, all kept as written; ECC protection, whose
 * status stays clear; channel locking, which the P4 lacks.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/dma/esp32p4_dw_gdma.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_clkrst.h"
#include "hw/misc/esp32p4_pms.h"
#include "migration/vmstate.h"

#define NCH                 ESP32P4_DW_GDMA_CHANNELS
#define FIFO_SIZE           ESP32P4_DW_GDMA_FIFO

/* Common registers. */
#define A_ID0               0x00
#define A_COMPVER0          0x08
#define A_CFG0              0x10
#define A_CHEN0             0x18
#define A_CHEN1             0x1c
#define A_INTSTATUS0        0x30
#define A_COMMON_INTCLEAR0  0x38
#define A_COMMON_INTST_ENA0 0x40
#define A_COMMON_INTSIG_ENA0 0x48
#define A_COMMON_INTSTATUS0 0x50
#define A_RESET0            0x58
#define A_LOWPOWER_CFG0     0x60
#define A_LOWPOWER_CFG1     0x64
#define A_CHANNELS_END      0x500

/* Channel registers, at 0x100 * (n + 1). */
#define C_SAR0              0x00
#define C_SAR1              0x04
#define C_DAR0              0x08
#define C_DAR1              0x0c
#define C_BLOCK_TS0         0x10
#define C_CTL0              0x18
#define C_CTL1              0x1c
#define C_CFG0              0x20
#define C_CFG1              0x24
#define C_LLP0              0x28
#define C_LLP1              0x2c
#define C_STATUS0           0x30
#define C_STATUS1           0x34
#define C_SWHSSRC0          0x38
#define C_SWHSDST0          0x40
#define C_BLK_TFR_RESUMEREQ0 0x48
#define C_AXI_ID0           0x50
#define C_AXI_QOS0          0x58
#define C_SSTAT0            0x60
#define C_DSTAT0            0x68
#define C_SSTATAR0          0x70
#define C_SSTATAR1          0x74
#define C_DSTATAR0          0x78
#define C_DSTATAR1          0x7c
#define C_INTST_ENA0        0x80
#define C_INTST_ENA1        0x84
#define C_INTSTATUS0        0x88
#define C_INTSTATUS1        0x8c
#define C_INTSIG_ENA0       0x90
#define C_INTSIG_ENA1       0x94
#define C_INTCLEAR0         0x98
#define C_INTCLEAR1         0x9c

#define COMPVER             0x3230302a
#define CFG0_DMAC_EN        BIT(0)
#define CFG0_INT_EN         BIT(1)

/* The common interrupts. */
#define CI_DEC_ERR          BIT(0)
#define CI_WR2RO_ERR        BIT(1)
#define CI_RD2WO_ERR        BIT(2)
#define CI_UNDEFINEDREG     BIT(8)
#define CI_RW_MASK          0x10f
/* The ECC and parity enables, which always read as set. */
#define CI_RO_ONES          0x1ffe80
#define CI_STATUS_GLOBAL    BIT(16)

/* The channel interrupts. */
#define BLOCK_TFR_DONE      BIT(0)
#define DMA_TFR_DONE        BIT(1)
#define SRC_TRANSCOMP       BIT(3)
#define DST_TRANSCOMP       BIT(4)
#define SRC_DEC_ERR         BIT(5)
#define DST_DEC_ERR         BIT(6)
#define SRC_SLV_ERR         BIT(7)
#define DST_SLV_ERR         BIT(8)
#define LLI_RD_DEC_ERR      BIT(9)
#define LLI_WR_DEC_ERR      BIT(10)
#define LLI_RD_SLV_ERR      BIT(11)
#define LLI_WR_SLV_ERR      BIT(12)
#define LLI_INVALID_ERR     BIT(13)
#define MULTIBLKTYPE_ERR    BIT(14)
#define SLVIF_DEC_ERR       BIT(16)
#define SLVIF_WR2RO_ERR     BIT(17)
#define SLVIF_RD2RWO_ERR    BIT(18)
#define SLVIF_WRONCHEN_ERR  BIT(19)
#define SLVIF_WRON_VALID_ERR BIT(20)
#define CH_SRC_SUSPENDED    BIT(28)
#define CH_SUSPENDED        BIT(29)
#define CH_DISABLED         BIT(30)
#define CH_ABORTED          BIT(31)
#define CH_INT_RW_MASK      0xf83f7ffb
#define CH_INT_RO_ONES      BIT(25)
#define CH_INT_ENA1_ONES    0xf

#define CTL0_MASK           0x7fffff55
#define CTL0_RESET          0x1200
#define CTL0_SMS            BIT(0)
#define CTL0_DMS            BIT(2)
#define CTL0_SINC           BIT(4)
#define CTL0_DINC           BIT(6)
#define CTL1_MASK           0xc7ffffff
#define CTL1_SRC_STAT_EN    BIT(24)
#define CTL1_DST_STAT_EN    BIT(25)
#define CTL1_IOC_BLKTFR     BIT(26)
#define CTL1_LAST           BIT(30)
#define CTL1_VALID          BIT(31)
#define BLOCK_TS_MASK       0x3fffff
#define CFG0_MASK           0xf
#define CFG1_MASK           0x7f8e319f
#define CFG1_RESET          0x6001b
#define CFG1_HS_SEL_SRC     BIT(3)
#define CFG1_HS_SEL_DST     BIT(4)
#define LLP0_MASK           0xffffffc1
#define LLP0_LMS            BIT(0)
#define LLP0_LOC            0xffffffc0
#define SWHS_REQ            BIT(0)
#define SWHS_SGLREQ         BIT(2)
#define SWHS_LST            BIT(4)
#define AXI_ID_MASK         0x10001
#define AXI_QOS_MASK        0xff
#define LOWPOWER0_RESET     0xf
#define LOWPOWER1_RESET     0x404040
#define LOWPOWER1_MASK      0xffffff

/* The multi-block types. */
#define MB_CONTIGUOUS       0
#define MB_RELOAD           1
#define MB_SHADOW           2
#define MB_LLI              3

/* LLI words written back at the end of a block. */
#define LLI_CTL0            0x20
#define LLI_SSTAT           0x28
#define LLI_DSTAT           0x2c
#define LLI_STATUS_LO       0x30
#define LLI_STATUS_HI       0x34
#define LLI_FETCH_BYTES     0x28
#define LLI_BLOCK_TFR_DONE  BIT(30)
#define LLI_DMA_TFR_DONE    BIT(31)

#define LLI_CYCLES          5
#define WB_CYCLES           3

/*
 * A channel's phase: disabled; between blocks, setting up the next;
 * waiting for BLK_TFR_RESUMEREQ after an invalid shadow or LLI; moving a
 * block; or stalled after a block until its BLOCK_TFR_DONE is cleared.
 */
enum {
    P_IDLE,
    P_SETUP,
    P_WAIT_RESUME,
    P_XFER,
    P_STALL,
};

static void dw_update_irq(ESP32P4DwGdmaState *s)
{
    bool level = false;

    for (int n = 0; n < NCH; n++) {
        level |= !!(s->ch[n].int_status & s->ch[n].int_sig_ena);
    }
    level |= !!(s->common_int_status & s->common_int_sig_ena);
    qemu_set_irq(s->irq, level && (s->cfg0 & CFG0_INT_EN));
}

static void ch_event(ESP32P4DwGdmaState *s, int n, uint32_t bits)
{
    s->ch[n].int_status |= bits & s->ch[n].int_st_ena;
    dw_update_irq(s);
}

static void common_event(ESP32P4DwGdmaState *s, uint32_t bits)
{
    s->common_int_status |= bits & s->common_int_st_ena;
    dw_update_irq(s);
}

/* Channel configuration */

static int ch_src_type(ESP32P4DwGdmaChan *c)
{
    return c->cfg0 & 3;
}

static int ch_dst_type(ESP32P4DwGdmaChan *c)
{
    return (c->cfg0 >> 2) & 3;
}

static bool ch_lli(ESP32P4DwGdmaChan *c)
{
    return ch_src_type(c) == MB_LLI || ch_dst_type(c) == MB_LLI;
}

static bool ch_shadow(ESP32P4DwGdmaChan *c)
{
    return ch_src_type(c) == MB_SHADOW || ch_dst_type(c) == MB_SHADOW;
}

static uint32_t ch_tt_fc(ESP32P4DwGdmaChan *c)
{
    return c->cfg1 & 7;
}

static bool ch_src_periph(ESP32P4DwGdmaChan *c)
{
    return (0xbc >> ch_tt_fc(c)) & 1;   /* TT_FC 2, 3, 4, 5, 7 */
}

static bool ch_dst_periph(ESP32P4DwGdmaChan *c)
{
    return (0xea >> ch_tt_fc(c)) & 1;   /* TT_FC 1, 3, 5, 6, 7 */
}

static bool ch_fc_src(ESP32P4DwGdmaChan *c)
{
    return ch_tt_fc(c) == 4 || ch_tt_fc(c) == 5;
}

static bool ch_fc_dst(ESP32P4DwGdmaChan *c)
{
    return ch_tt_fc(c) == 6 || ch_tt_fc(c) == 7;
}

/* Bytes per item: TR_WIDTH 0 to 3, 8 to 64 bits, the data bus's width. */
static uint32_t ch_width(ESP32P4DwGdmaChan *c, bool src)
{
    return 1u << MIN(extract32(c->ctl0, src ? 8 : 11, 3), 3);
}

static uint32_t ch_msize(ESP32P4DwGdmaChan *c, bool src)
{
    uint32_t m = extract32(c->ctl0, src ? 14 : 18, 4);

    return m ? 2u << MIN(m, 9) : 1;
}

static uint32_t ch_block_bytes(ESP32P4DwGdmaChan *c)
{
    return ((c->block_ts & BLOCK_TS_MASK) + 1) * ch_width(c, true);
}

/* Handshakes */

typedef struct DwRequest {
    bool burst;
    bool single;
    bool last;
} DwRequest;

static DwRequest ch_request(ESP32P4DwGdmaState *s, ESP32P4DwGdmaChan *c,
                            bool src)
{
    DwRequest r = { false, false, false };
    uint32_t per = extract32(c->cfg1, src ? 7 : 12, 2);
    uint32_t sw = src ? c->swhs_src : c->swhs_dst;

    if (c->cfg1 & (src ? CFG1_HS_SEL_SRC : CFG1_HS_SEL_DST)) {
        r.burst = sw & SWHS_REQ;
        r.single = sw & SWHS_SGLREQ;
        r.last = sw & SWHS_LST;
    } else if (per < ESP32P4_DW_GDMA_HS) {
        r.burst = s->hs_req[per];
        r.single = s->hs_single[per];
        r.last = s->hs_last[per];
    }
    return r;
}

/* A handshaking side's transaction completed. */
static void ch_ack(ESP32P4DwGdmaState *s, int n, bool src)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    uint32_t per = extract32(c->cfg1, src ? 7 : 12, 2);

    if (c->cfg1 & (src ? CFG1_HS_SEL_SRC : CFG1_HS_SEL_DST)) {
        if (src) {
            c->swhs_src = 0;
        } else {
            c->swhs_dst = 0;
        }
    } else if (per < ESP32P4_DW_GDMA_HS) {
        qemu_irq_pulse(s->hs_ack[per]);
    }
    ch_event(s, n, src ? SRC_TRANSCOMP : DST_TRANSCOMP);
}

/* Bus */

static void ch_stop(ESP32P4DwGdmaChan *c)
{
    c->src_txn = 0;
    c->dst_txn = 0;
    c->enabled = false;
    c->disabling = false;
    c->susp = false;
    c->src_suspended = false;
    c->phase = P_IDLE;
}

/*
 * An access by channel @n on AXI master @master: MEMTX_OK, or the error
 * that ends the transfer, raising @dec or @slv.
 */
static bool ch_bus(ESP32P4DwGdmaState *s, int n, int master, uint32_t hi,
                   uint32_t addr, void *buf, uint32_t len, bool write,
                   uint32_t dec, uint32_t slv)
{
    MemTxAttrs attrs = esp32p4_dma_attrs(ESP32P4_DMA_MASTER_GDMA_CH0 + n,
        ESP32P4_ICM_ID_AXI(master ? ESP32P4_ICM_CID_VDMA_M2
                                  : ESP32P4_ICM_CID_VDMA_M1, n));
    MemTxResult r;
    const char *why;

    if (hi) {
        r = MEMTX_DECODE_ERROR;
    } else {
        r = address_space_rw(s->as, addr, attrs, buf, len, write);
    }
    if (r == MEMTX_OK) {
        return true;
    }
    why = r == MEMTX_DECODE_ERROR ? "a decode" : "a slave";
    qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: %s error on a %s at "
                  "0x%08" PRIx32 " through master %d\n", TYPE_ESP32P4_DW_GDMA,
                  n + 1, why, write ? "write" : "read", addr, master + 1);
    ch_event(s, n, r == MEMTX_DECODE_ERROR ? dec : slv);
    ch_stop(&s->ch[n]);
    s->ch[n].fifo_num = 0;
    return false;
}

static uint32_t beats(uint32_t bytes)
{
    return MAX(1, DIV_ROUND_UP(bytes, 8));
}

/* Blocks */

/*
 * Load the block a shadow-register or contiguous/reload transfer runs
 * next; for a linked-list transfer the LLI fetch does it.
 */
static void ch_setup(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    int st = ch_src_type(c), dt = ch_dst_type(c);

    if (ch_shadow(c)) {
        if (!(c->sh_ctl1 & CTL1_VALID)) {
            c->phase = P_WAIT_RESUME;
            ch_event(s, n, LLI_INVALID_ERR);
            return;
        }
        if (st == MB_SHADOW) {
            c->sar = c->sh_sar;
        }
        if (dt == MB_SHADOW) {
            c->dar = c->sh_dar;
        }
        c->block_ts = c->sh_block_ts;
        c->ctl0 = c->sh_ctl0;
        c->ctl1 = c->sh_ctl1;
        c->sh_ctl1 &= ~CTL1_VALID;
    }
    c->phase = P_XFER;
}

/* The end of a block that is not the last: the auto-reloads. */
static void ch_reload(ESP32P4DwGdmaChan *c)
{
    int st = ch_src_type(c), dt = ch_dst_type(c);

    if (st == MB_RELOAD) {
        c->sar = c->init_sar;
    }
    if (dt == MB_RELOAD) {
        c->dar = c->init_dar;
    }
    if (!ch_lli(c) && !ch_shadow(c) && (st == MB_RELOAD || dt == MB_RELOAD)) {
        c->block_ts = c->init_block_ts;
        c->ctl0 = c->init_ctl0;
        c->ctl1 = c->init_ctl1;
    }
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t ch_fetch_lli(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    uint32_t addr = c->llp0 & LLP0_LOC;
    uint32_t w[LLI_FETCH_BYTES / 4];
    int st = ch_src_type(c), dt = ch_dst_type(c);

    if (!ch_bus(s, n, c->llp0 & LLP0_LMS, c->llp1, addr, w, sizeof(w),
                false, LLI_RD_DEC_ERR, LLI_RD_SLV_ERR)) {
        return LLI_CYCLES;
    }
    for (int i = 0; i < ARRAY_SIZE(w); i++) {
        w[i] = le32_to_cpu(w[i]);
    }
    if (!(w[9] & CTL1_VALID)) {
        c->phase = P_WAIT_RESUME;
        ch_event(s, n, LLI_INVALID_ERR);
        return LLI_CYCLES;
    }
    if (st == MB_LLI) {
        c->sar = w[0];
        c->sar_hi = w[1];
    }
    if (dt == MB_LLI) {
        c->dar = w[2];
        c->dar_hi = w[3];
    }
    c->block_ts = w[4] & BLOCK_TS_MASK;
    c->llp0 = w[6] & LLP0_MASK;
    c->llp1 = w[7];
    c->ctl0 = w[8] & CTL0_MASK;
    c->ctl1 = w[9] & CTL1_MASK;
    c->cur_lli = addr;
    c->phase = P_XFER;
    return LLI_CYCLES;
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t ch_block_end(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    uint32_t sw = ch_width(c, true);
    bool lli = ch_lli(c), shadow = ch_shadow(c);
    bool last;
    uint32_t cycles = 0, v;

    c->status0 = (c->dst_bytes / sw) & BLOCK_TS_MASK;
    c->status1 = (c->fifo_num / sw) & 0x7fff;
    if (c->ctl1 & CTL1_SRC_STAT_EN) {
        if (!ch_bus(s, n, c->ctl0 & CTL0_SMS ? 1 : 0, c->sstatar_hi,
                    c->sstatar, &v, 4, false, SRC_DEC_ERR, SRC_SLV_ERR)) {
            return 1;
        }
        c->sstat = le32_to_cpu(v);
        cycles++;
    }
    if (c->ctl1 & CTL1_DST_STAT_EN) {
        if (!ch_bus(s, n, c->ctl0 & CTL0_DMS ? 1 : 0, c->dstatar_hi,
                    c->dstatar, &v, 4, false, DST_DEC_ERR, DST_SLV_ERR)) {
            return cycles + 1;
        }
        c->dstat = le32_to_cpu(v);
        cycles++;
    }
    if (lli || shadow) {
        last = c->ctl1 & CTL1_LAST;
    } else {
        last = ch_src_type(c) == MB_CONTIGUOUS &&
               ch_dst_type(c) == MB_CONTIGUOUS;
    }
    if (lli) {
        uint32_t wb[6] = {
            cpu_to_le32(c->ctl0), cpu_to_le32(c->ctl1 & ~CTL1_VALID),
            cpu_to_le32(c->sstat), cpu_to_le32(c->dstat),
            cpu_to_le32(c->status0),
            cpu_to_le32(c->status1 | LLI_BLOCK_TFR_DONE |
                        (last ? LLI_DMA_TFR_DONE : 0)),
        };
        int master = c->llp0 & LLP0_LMS;

        /* The status words go last, the done bits with them. */
        if (!ch_bus(s, n, master, c->llp1, c->cur_lli + LLI_CTL0, wb, 8,
                    true, LLI_WR_DEC_ERR, LLI_WR_SLV_ERR) ||
            ((c->ctl1 & CTL1_SRC_STAT_EN) &&
             !ch_bus(s, n, master, c->llp1, c->cur_lli + LLI_SSTAT, &wb[2],
                     4, true, LLI_WR_DEC_ERR, LLI_WR_SLV_ERR)) ||
            ((c->ctl1 & CTL1_DST_STAT_EN) &&
             !ch_bus(s, n, master, c->llp1, c->cur_lli + LLI_DSTAT, &wb[3],
                     4, true, LLI_WR_DEC_ERR, LLI_WR_SLV_ERR)) ||
            !ch_bus(s, n, master, c->llp1, c->cur_lli + LLI_STATUS_LO,
                    &wb[4], 8, true, LLI_WR_DEC_ERR, LLI_WR_SLV_ERR)) {
            return cycles + WB_CYCLES;
        }
        cycles += WB_CYCLES;
    }
    c->src_bytes = 0;
    c->dst_bytes = 0;
    c->src_done = false;
    c->dst_done = false;
    if (c->dst_txn) {
        /* The block ended within a destination burst: it is answered. */
        c->dst_txn = 0;
        ch_ack(s, n, false);
    }
    c->src_txn = 0;
    if (c->ctl1 & CTL1_IOC_BLKTFR) {
        ch_event(s, n, BLOCK_TFR_DONE);
    }
    if (last) {
        ch_stop(c);
        ch_event(s, n, DMA_TFR_DONE);
        return cycles;
    }
    ch_reload(c);
    if (!lli && !shadow && (c->int_status & BLOCK_TFR_DONE)) {
        c->phase = P_STALL;
    } else {
        c->phase = P_SETUP;
    }
    return cycles;
}

/* Transactions */

static uint32_t fifo_room(ESP32P4DwGdmaChan *c)
{
    return FIFO_SIZE - c->fifo_num;
}

static void fifo_in(ESP32P4DwGdmaChan *c, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        c->fifo[(c->fifo_head + c->fifo_num + i) % FIFO_SIZE] = buf[i];
    }
    c->fifo_num += len;
}

static void fifo_out(ESP32P4DwGdmaChan *c, uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = c->fifo[(c->fifo_head + i) % FIFO_SIZE];
    }
    c->fifo_head = (c->fifo_head + len) % FIFO_SIZE;
    c->fifo_num -= len;
}

/*
 * The bytes the next source transaction moves, or 0 if it cannot go yet;
 * *start the bytes of the handshaking transaction it starts (0 for none, or
 * for one under way), *last for a flow-controlling source's last request.
 * A transaction the FIFO can hold goes in one round once it has room; a
 * burst of more (an MSIZE of 128 items or more) moves as the FIFO makes
 * room, round by round, its request answered as its last item is in.
 */
static uint32_t src_plan(ESP32P4DwGdmaState *s, ESP32P4DwGdmaChan *c,
                         uint32_t *start, bool *last)
{
    uint32_t w = ch_width(c, true);
    uint32_t items = ch_msize(c, true), want;

    *start = 0;
    *last = false;
    if (c->phase != P_XFER || c->src_done || c->susp || c->disabling) {
        return 0;
    }
    if (ch_src_periph(c) && c->src_txn) {
        *last = c->src_txn_last;
        return MIN(c->src_txn, fifo_room(c) / w * w);
    }
    if (ch_src_periph(c)) {
        DwRequest r = ch_request(s, c, true);

        if (!r.burst && !r.single) {
            return 0;
        }
        items = r.burst ? items : 1;
        *last = r.last && ch_fc_src(c);
    }
    if (!ch_fc_src(c) && !ch_fc_dst(c)) {
        items = MIN(items, (ch_block_bytes(c) - c->src_bytes) / w);
    }
    want = items * w;
    if (!items || (want <= FIFO_SIZE && want > fifo_room(c))) {
        return 0;
    }
    if (ch_src_periph(c)) {
        *start = want;
    }
    return MIN(want, fifo_room(c) / w * w);
}

static uint32_t dst_plan(ESP32P4DwGdmaState *s, ESP32P4DwGdmaChan *c,
                         uint32_t *start, bool *last)
{
    uint32_t w = ch_width(c, false);
    uint32_t items = ch_msize(c, false), want;
    bool drain = c->src_done || c->susp || c->disabling;

    *start = 0;
    *last = false;
    if (c->phase != P_XFER || c->dst_done || !c->fifo_num) {
        return 0;
    }
    if (ch_dst_periph(c) && c->dst_txn) {
        *last = c->dst_txn_last;
        return MIN(c->dst_txn, c->fifo_num);
    }
    if (ch_dst_periph(c)) {
        DwRequest r = ch_request(s, c, false);

        if (!r.burst && !r.single) {
            return 0;
        }
        items = r.burst ? items : 1;
        *last = r.last && ch_fc_dst(c);
    }
    want = items * w;
    if (want <= c->fifo_num) {
        *start = ch_dst_periph(c) ? want : 0;
        return want;
    }
    if (ch_dst_periph(c) && want > FIFO_SIZE) {
        *start = want;
        return c->fifo_num;
    }
    /* At the end, what is left, though less than a burst or an item. */
    if (drain || ch_fc_src(c)) {
        if (ch_dst_periph(c)) {
            *start = MIN(c->fifo_num, want);
            return *start;
        }
        return c->fifo_num;
    }
    return ch_dst_periph(c) ? 0 : (c->fifo_num / w) * w;
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t ch_src_xfer(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    bool last;
    uint32_t start;
    uint32_t bytes = src_plan(s, c, &start, &last);
    uint32_t w = ch_width(c, true);
    int master = c->ctl0 & CTL0_SMS ? 1 : 0;
    g_autofree uint8_t *buf = g_malloc(bytes);

    if (c->ctl0 & CTL0_SINC) {
        for (uint32_t i = 0; i < bytes; i += w) {
            if (!ch_bus(s, n, master, c->sar_hi, c->sar, buf + i, w, false,
                        SRC_DEC_ERR, SRC_SLV_ERR)) {
                return beats(bytes);
            }
        }
    } else {
        if (!ch_bus(s, n, master, c->sar_hi, c->sar, buf, bytes, false,
                    SRC_DEC_ERR, SRC_SLV_ERR)) {
            return beats(bytes);
        }
        c->sar += bytes;
    }
    fifo_in(c, buf, bytes);
    c->src_bytes += bytes;
    if (ch_src_periph(c)) {
        if (start) {
            c->src_txn = start;
            c->src_txn_last = last;
        }
        c->src_txn -= MIN(bytes, c->src_txn);
        if (c->src_txn) {
            return beats(bytes);
        }
        last = c->src_txn_last;
        ch_ack(s, n, true);
    }
    if (ch_fc_src(c) ? last
                     : !ch_fc_dst(c) && c->src_bytes >= ch_block_bytes(c)) {
        c->src_done = true;
    }
    return beats(bytes);
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static uint32_t ch_dst_xfer(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    bool last;
    uint32_t start;
    uint32_t bytes = dst_plan(s, c, &start, &last);
    uint32_t w = ch_width(c, false);
    int master = c->ctl0 & CTL0_DMS ? 1 : 0;
    g_autofree uint8_t *buf = g_malloc(bytes);

    fifo_out(c, buf, bytes);
    if (c->ctl0 & CTL0_DINC) {
        for (uint32_t i = 0; i < bytes; i += w) {
            if (!ch_bus(s, n, master, c->dar_hi, c->dar, buf + i,
                        MIN(w, bytes - i), true, DST_DEC_ERR, DST_SLV_ERR)) {
                return beats(bytes);
            }
        }
    } else {
        if (!ch_bus(s, n, master, c->dar_hi, c->dar, buf, bytes, true,
                    DST_DEC_ERR, DST_SLV_ERR)) {
            return beats(bytes);
        }
        c->dar += bytes;
    }
    c->dst_bytes += bytes;
    if (ch_dst_periph(c)) {
        if (start) {
            c->dst_txn = start;
            c->dst_txn_last = last;
        }
        c->dst_txn -= MIN(bytes, c->dst_txn);
        if (c->dst_txn && !(c->src_done && !c->fifo_num)) {
            return beats(bytes);
        }
        last = c->dst_txn_last;
        c->dst_txn = 0;
        ch_ack(s, n, false);
    }
    if (last) {
        c->dst_done = true;
    }
    return beats(bytes);
}

/*
 * What a channel does without the bus: set up its next block, finish
 * one, and complete a suspension or a disable once its FIFO is empty.
 */
static uint32_t ch_progress(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    uint32_t cycles = 0;

    if (!c->enabled) {
        return 0;
    }
    if (c->disabling && (c->phase != P_XFER || !c->fifo_num)) {
        c->status0 = (c->dst_bytes / ch_width(c, true)) & BLOCK_TS_MASK;
        ch_stop(c);
        c->fifo_num = 0;
        ch_event(s, n, CH_DISABLED);
        return 0;
    }
    if (c->phase == P_SETUP && !ch_lli(c)) {
        ch_setup(s, n);
    }
    if (c->phase == P_XFER) {
        bool done = ch_fc_dst(c) ? c->dst_done
                                 : c->src_done && !c->fifo_num;

        if (done) {
            cycles = ch_block_end(s, n);
        } else if (c->susp && !c->fifo_num) {
            if (!c->src_suspended) {
                c->src_suspended = true;
                ch_event(s, n, CH_SRC_SUSPENDED | CH_SUSPENDED);
            }
        }
    }
    return cycles;
}

/* Arbitration */

static bool ch_reads(ESP32P4DwGdmaState *s, int n, bool *lli)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    uint32_t start;
    bool last;

    *lli = c->enabled && c->phase == P_SETUP && ch_lli(c) &&
           !c->disabling;
    return *lli || (c->enabled && src_plan(s, c, &start, &last));
}

static bool ch_writes(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    uint32_t start;
    bool last;

    return c->enabled && dst_plan(s, c, &start, &last);
}

static uint32_t ch_prior(ESP32P4DwGdmaChan *c)
{
    return extract32(c->cfg1, 17, 3);
}

/* The channel arbiter @write grants, or -1: LLI fetches first. */
/* [spec:nuos:req:emu.esp32p4.dma] */
static int dw_grant(ESP32P4DwGdmaState *s, bool write, bool *lli)
{
    int best = -1;
    uint32_t best_key = 0;

    *lli = false;
    for (int k = 1; k <= NCH; k++) {
        int n = (s->rr_last[write] + k) % NCH;
        bool is_lli = false;
        uint32_t key;

        if (write ? !ch_writes(s, n) : !ch_reads(s, n, &is_lli)) {
            continue;
        }
        key = (is_lli ? 8 : 0) + ch_prior(&s->ch[n]);
        if (best < 0 || key > best_key) {
            best = n;
            best_key = key;
            *lli = is_lli;
        }
    }
    if (best >= 0) {
        s->rr_last[write] = best;
    }
    return best;
}

/* When the round @timer runs falls due. */
static int64_t dw_due_ns(ESP32P4DwGdmaState *s)
{
    return esp32p4_clk_count_time(&s->bus_edges, s->due_cycle);
}

/* Arm the engine's timer for a round at bus cycle @cycle. */
static void dw_arm(ESP32P4DwGdmaState *s, uint64_t cycle)
{
    s->due_cycle = cycle;
    timer_mod(s->timer, dw_due_ns(s));
}

/*
 * One round of the bus, on an edge of its clock, at the time it falls
 * due, esp32_line_time(): the read and the write arbiters each grant a
 * transaction, and the engine comes back when they are over, as many
 * cycles later.
 */
/* [spec:nuos:req:emu.esp32p4.dma] */
static void dw_run(void *opaque)
{
    ESP32P4DwGdmaState *s = opaque;
    uint32_t cost = 0;
    bool lli;
    int r, w;

    if (!clock_is_enabled(s->clk) || !(s->cfg0 & CFG0_DMAC_EN)) {
        return;
    }
    for (int n = 0; n < NCH; n++) {
        cost += ch_progress(s, n);
    }
    r = dw_grant(s, false, &lli);
    w = dw_grant(s, true, &lli);
    if (r >= 0) {
        if (s->ch[r].phase == P_SETUP) {
            cost = MAX(cost, ch_fetch_lli(s, r));
        } else {
            cost = MAX(cost, ch_src_xfer(s, r));
        }
    }
    if (w >= 0 && s->ch[w].enabled) {
        cost = MAX(cost, ch_dst_xfer(s, w));
    }
    for (int n = 0; n < NCH; n++) {
        cost += ch_progress(s, n);
    }
    if (cost || r >= 0 || w >= 0) {
        dw_arm(s, s->due_cycle + MAX(cost, 1));
    }
}

/* The round falls due: run it at its time, which a late timer has passed. */
static void dw_timer_cb(void *opaque)
{
    ESP32P4DwGdmaState *s = opaque;

    esp32_line_run_at(dw_due_ns(s), dw_run, s);
}

/*
 * [spec:nuos:req:emu.esp32p4.dma]
 * Run the rounds that fell due by @now, in order, each at its own time.
 */
static void dw_catch_up(ESP32P4DwGdmaState *s, int64_t now)
{
    while (timer_pending(s->timer) && dw_due_ns(s) <= now) {
        timer_del(s->timer);
        esp32_line_run_at(dw_due_ns(s), dw_run, s);
    }
}

/*
 * Let the engine look for work at the bus clock's next edge after the
 * change that calls for it, at esp32_line_time().
 */
static void dw_kick(ESP32P4DwGdmaState *s)
{
    if (!timer_pending(s->timer) && clock_is_enabled(s->clk)) {
        dw_arm(s, esp32p4_clk_count_at(&s->bus_edges,
                                       esp32_line_time()) + 1);
    }
}

/*
 * The bus clock changes: the rounds due before it run at the old rate;
 * its cycles are counted afresh from the change, and a round under way
 * takes the cycles it has left, at the new rate.
 */
static void dw_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4DwGdmaState *s = opaque;
    int64_t now = esp32_line_time();
    uint64_t done, left = 0;

    if (event == ClockPreUpdate) {
        dw_catch_up(s, now);
        return;
    }
    done = esp32p4_clk_count_at(&s->bus_edges, now);
    if (timer_pending(s->timer)) {
        left = s->due_cycle > done ? s->due_cycle - done : 1;
    }
    esp32p4_clk_count_start(&s->bus_edges, s->clk, now);
    if (!clock_is_enabled(s->clk)) {
        timer_del(s->timer);
    } else if (left) {
        dw_arm(s, left);
    } else {
        dw_kick(s);
    }
}

/* Channel control */

/* [spec:nuos:req:emu.esp32p4.dma] */
static void ch_enable(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    int st = ch_src_type(c), dt = ch_dst_type(c);

    if ((st == MB_SHADOW && dt == MB_LLI) ||
        (st == MB_LLI && dt == MB_SHADOW)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: multi-block types "
                      "%d and %d do not go together\n", TYPE_ESP32P4_DW_GDMA,
                      n + 1, st, dt);
        ch_event(s, n, MULTIBLKTYPE_ERR);
        return;
    }
    c->enabled = true;
    c->disabling = false;
    c->src_suspended = false;
    c->init_sar = c->sar;
    c->init_dar = c->dar;
    c->init_block_ts = c->block_ts;
    c->init_ctl0 = c->ctl0;
    c->init_ctl1 = c->ctl1;
    c->src_bytes = 0;
    c->dst_bytes = 0;
    c->src_done = false;
    c->dst_done = false;
    c->src_txn = 0;
    c->dst_txn = 0;
    c->fifo_head = 0;
    c->fifo_num = 0;
    c->cur_lli = 0;
    if (ch_lli(c)) {
        c->phase = P_SETUP;
    } else {
        ch_setup(s, n);
    }
    dw_kick(s);
}

static void ch_abort(ESP32P4DwGdmaState *s, int n)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];

    if (!c->enabled) {
        return;
    }
    c->status0 = (c->dst_bytes / ch_width(c, true)) & BLOCK_TS_MASK;
    ch_stop(c);
    c->fifo_num = 0;
    ch_event(s, n, CH_ABORTED);
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static void chen0_write(ESP32P4DwGdmaState *s, uint32_t v)
{
    for (int n = 0; n < NCH; n++) {
        ESP32P4DwGdmaChan *c = &s->ch[n];

        if (v & BIT(24 + n)) {
            c->susp = v & BIT(16 + n);
            if (!c->susp) {
                c->src_suspended = false;
            }
        }
        if (v & BIT(8 + n)) {
            if (v & BIT(n)) {
                if (!c->enabled) {
                    ch_enable(s, n);
                }
            } else if (c->enabled && !c->disabling) {
                c->disabling = true;
            }
        }
    }
    dw_kick(s);
}

/* Registers */

/* Registers a write may change while the channel runs. */
static bool ch_reg_live(hwaddr o)
{
    switch (o) {
    case C_CFG0:
    case C_SWHSSRC0:
    case C_SWHSDST0:
    case C_BLK_TFR_RESUMEREQ0:
    case C_AXI_QOS0:
    case C_INTST_ENA0:
    case C_INTSIG_ENA0:
    case C_INTCLEAR0:
    case C_INTCLEAR1:
        return true;
    default:
        return false;
    }
}

static bool ch_reg_shadowed(hwaddr o)
{
    return o == C_SAR0 || o == C_DAR0 || o == C_BLOCK_TS0 || o == C_CTL0 ||
           o == C_CTL1;
}

static uint32_t ch_read(ESP32P4DwGdmaState *s, int n, hwaddr o)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];

    switch (o) {
    case C_SAR0:
        return c->sar;
    case C_SAR1:
        return c->sar_hi;
    case C_DAR0:
        return c->dar;
    case C_DAR1:
        return c->dar_hi;
    case C_BLOCK_TS0:
        return c->block_ts;
    case C_CTL0:
        return c->ctl0;
    case C_CTL1:
        /* In shadow mode VALID is the shadow's, for software to poll. */
        if (ch_shadow(c)) {
            return (c->ctl1 & ~CTL1_VALID) | (c->sh_ctl1 & CTL1_VALID);
        }
        return c->ctl1;
    case C_CFG0:
        return c->cfg0;
    case C_CFG1:
        return c->cfg1;
    case C_LLP0:
        return c->llp0;
    case C_LLP1:
        return c->llp1;
    case C_STATUS0:
        return c->status0;
    case C_STATUS1:
        return c->status1;
    case C_SWHSSRC0:
        return c->swhs_src;
    case C_SWHSDST0:
        return c->swhs_dst;
    case C_AXI_ID0:
        return c->axi_id;
    case C_AXI_QOS0:
        return c->axi_qos;
    case C_SSTAT0:
        return c->sstat;
    case C_DSTAT0:
        return c->dstat;
    case C_SSTATAR0:
        return c->sstatar;
    case C_SSTATAR1:
        return c->sstatar_hi;
    case C_DSTATAR0:
        return c->dstatar;
    case C_DSTATAR1:
        return c->dstatar_hi;
    case C_INTST_ENA0:
        return c->int_st_ena;
    case C_INTST_ENA1:
    case C_INTSIG_ENA1:
        return CH_INT_ENA1_ONES;
    case C_INTSTATUS0:
        return c->int_status;
    case C_INTSTATUS1:
        return 0;
    case C_INTSIG_ENA0:
        return c->int_sig_ena;
    case C_BLK_TFR_RESUMEREQ0:
    case C_INTCLEAR0:
    case C_INTCLEAR1:
        ch_event(s, n, SLVIF_RD2RWO_ERR);
        qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: read of write-only "
                      "offset 0x%02" HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA,
                      n + 1, o);
        return 0;
    default:
        ch_event(s, n, SLVIF_DEC_ERR);
        qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: read of undefined "
                      "offset 0x%02" HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA,
                      n + 1, o);
        return 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static void ch_write(ESP32P4DwGdmaState *s, int n, hwaddr o, uint32_t v)
{
    ESP32P4DwGdmaChan *c = &s->ch[n];
    bool live = c->enabled;

    if (live && ch_reg_shadowed(o) && ch_shadow(c)) {
        if (c->sh_ctl1 & CTL1_VALID) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: shadow register "
                          "write while the shadow block is valid\n",
                          TYPE_ESP32P4_DW_GDMA, n + 1);
            ch_event(s, n, SLVIF_WRON_VALID_ERR);
            return;
        }
        switch (o) {
        case C_SAR0:
            c->sh_sar = v;
            break;
        case C_DAR0:
            c->sh_dar = v;
            break;
        case C_BLOCK_TS0:
            c->sh_block_ts = v & BLOCK_TS_MASK;
            break;
        case C_CTL0:
            c->sh_ctl0 = v & CTL0_MASK;
            break;
        default:
            c->sh_ctl1 = v & CTL1_MASK;
            break;
        }
        dw_kick(s);
        return;
    }
    if (live && !ch_reg_live(o)) {
        switch (o) {
        case C_STATUS0:
        case C_STATUS1:
        case C_SSTAT0:
        case C_DSTAT0:
        case C_INTSTATUS0:
        case C_INTSTATUS1:
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: write to offset "
                          "0x%02" HWADDR_PRIx " while it is enabled\n",
                          TYPE_ESP32P4_DW_GDMA, n + 1, o);
            ch_event(s, n, SLVIF_WRONCHEN_ERR);
            return;
        }
    }
    switch (o) {
    case C_SAR0:
        c->sar = c->sh_sar = v;
        break;
    case C_SAR1:
        c->sar_hi = v;
        break;
    case C_DAR0:
        c->dar = c->sh_dar = v;
        break;
    case C_DAR1:
        c->dar_hi = v;
        break;
    case C_BLOCK_TS0:
        c->block_ts = c->sh_block_ts = v & BLOCK_TS_MASK;
        break;
    case C_CTL0:
        c->ctl0 = c->sh_ctl0 = v & CTL0_MASK;
        break;
    case C_CTL1:
        c->ctl1 = c->sh_ctl1 = v & CTL1_MASK;
        break;
    case C_CFG0:
        c->cfg0 = v & CFG0_MASK;
        break;
    case C_CFG1:
        c->cfg1 = v & CFG1_MASK;
        break;
    case C_LLP0:
        c->llp0 = v & LLP0_MASK;
        break;
    case C_LLP1:
        c->llp1 = v;
        break;
    case C_SWHSSRC0:
    case C_SWHSDST0: {
        uint32_t *r = o == C_SWHSSRC0 ? &c->swhs_src : &c->swhs_dst;

        /* Each bit changes only with its write-enable, the bit above. */
        for (int b = 0; b < 6; b += 2) {
            if (v & BIT(b + 1)) {
                *r = (*r & ~BIT(b)) | (v & BIT(b));
            }
        }
        dw_kick(s);
        break;
    }
    case C_BLK_TFR_RESUMEREQ0:
        if ((v & 1) && c->phase == P_WAIT_RESUME) {
            c->phase = P_SETUP;
            dw_kick(s);
        }
        break;
    case C_AXI_ID0:
        c->axi_id = v & AXI_ID_MASK;
        break;
    case C_AXI_QOS0:
        c->axi_qos = v & AXI_QOS_MASK;
        break;
    case C_SSTATAR0:
        c->sstatar = v;
        break;
    case C_SSTATAR1:
        c->sstatar_hi = v;
        break;
    case C_DSTATAR0:
        c->dstatar = v;
        break;
    case C_DSTATAR1:
        c->dstatar_hi = v;
        break;
    case C_INTST_ENA0:
        c->int_st_ena = (v & CH_INT_RW_MASK) | CH_INT_RO_ONES;
        break;
    case C_INTSIG_ENA0:
        c->int_sig_ena = (v & CH_INT_RW_MASK) | CH_INT_RO_ONES;
        dw_update_irq(s);
        break;
    case C_INTCLEAR0:
        c->int_status &= ~v;
        dw_update_irq(s);
        if (c->phase == P_STALL && !(c->int_status & BLOCK_TFR_DONE)) {
            c->phase = P_SETUP;
            dw_kick(s);
        }
        break;
    case C_INTCLEAR1:
        break;
    case C_STATUS0:
    case C_STATUS1:
    case C_SSTAT0:
    case C_DSTAT0:
    case C_INTSTATUS0:
    case C_INTSTATUS1:
    case C_INTST_ENA1:
    case C_INTSIG_ENA1:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: write to read-only "
                      "offset 0x%02" HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA,
                      n + 1, o);
        ch_event(s, n, SLVIF_WR2RO_ERR);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: channel %d: write to undefined "
                      "offset 0x%02" HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA,
                      n + 1, o);
        ch_event(s, n, SLVIF_DEC_ERR);
        break;
    }
}

static uint32_t dw_intstatus(ESP32P4DwGdmaState *s)
{
    uint32_t v = 0;

    for (int n = 0; n < NCH; n++) {
        if (s->ch[n].int_status & s->ch[n].int_sig_ena) {
            v |= BIT(n);
        }
    }
    if (s->common_int_status & s->common_int_sig_ena) {
        v |= CI_STATUS_GLOBAL;
    }
    return v;
}

static bool dw_active(ESP32P4DwGdmaState *s)
{
    for (int n = 0; n < NCH; n++) {
        if (s->ch[n].enabled) {
            return true;
        }
    }
    return false;
}

static uint64_t esp32p4_dw_gdma_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    ESP32P4DwGdmaState *s = opaque;
    uint32_t v = 0;

    dw_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (addr >= 0x100 && addr < A_CHANNELS_END) {
        return ch_read(s, (addr >> 8) - 1, addr & 0xff);
    }
    switch (addr) {
    case A_ID0:
        return 0;
    case A_COMPVER0:
        return COMPVER;
    case A_CFG0:
        /* DMAC_EN reads as set until every channel has stopped. */
        return s->cfg0 | (dw_active(s) ? CFG0_DMAC_EN : 0);
    case A_CHEN0:
        for (int n = 0; n < NCH; n++) {
            v |= (s->ch[n].enabled ? BIT(n) : 0) |
                 (s->ch[n].susp ? BIT(16 + n) : 0);
        }
        return v;
    case A_CHEN1:
        return 0;
    case A_INTSTATUS0:
        return dw_intstatus(s);
    case A_COMMON_INTST_ENA0:
        return s->common_int_st_ena;
    case A_COMMON_INTSIG_ENA0:
        return s->common_int_sig_ena;
    case A_COMMON_INTSTATUS0:
        return s->common_int_status;
    case A_RESET0:
        return 0;
    case A_LOWPOWER_CFG0:
        return s->lowpower_cfg0;
    case A_LOWPOWER_CFG1:
        return s->lowpower_cfg1;
    case A_COMMON_INTCLEAR0:
        common_event(s, CI_RD2WO_ERR);
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of write-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA, addr);
        return 0;
    default:
        common_event(s, addr < 0x100 ? CI_DEC_ERR : CI_UNDEFINEDREG);
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of undefined offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA, addr);
        return 0;
    }
}

static void esp32p4_dw_gdma_reset_hold(Object *obj, ResetType type);

/* [spec:nuos:req:emu.esp32p4.dma] */
static void esp32p4_dw_gdma_write(void *opaque, hwaddr addr, uint64_t value,
                                  unsigned size)
{
    ESP32P4DwGdmaState *s = opaque;
    uint32_t v = value;

    dw_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (addr >= 0x100 && addr < A_CHANNELS_END) {
        ch_write(s, (addr >> 8) - 1, addr & 0xff, v);
        return;
    }
    switch (addr) {
    case A_CFG0:
        s->cfg0 = v & (CFG0_DMAC_EN | CFG0_INT_EN);
        dw_update_irq(s);
        dw_kick(s);
        break;
    case A_CHEN0:
        chen0_write(s, v);
        break;
    case A_CHEN1:
        for (int n = 0; n < NCH; n++) {
            if ((v & BIT(8 + n)) && (v & BIT(n))) {
                ch_abort(s, n);
            }
        }
        break;
    case A_COMMON_INTCLEAR0:
        s->common_int_status &= ~v;
        dw_update_irq(s);
        break;
    case A_COMMON_INTST_ENA0:
        s->common_int_st_ena = (v & CI_RW_MASK) | CI_RO_ONES;
        break;
    case A_COMMON_INTSIG_ENA0:
        s->common_int_sig_ena = (v & CI_RW_MASK) | CI_RO_ONES;
        dw_update_irq(s);
        break;
    case A_RESET0:
        /*
         * Everything but the register interface resets, and the bit
         * clears itself as soon as the reset is done.
         */
        if (v & 1) {
            esp32p4_dw_gdma_reset_hold(OBJECT(s), RESET_TYPE_COLD);
            dw_update_irq(s);
        }
        break;
    case A_LOWPOWER_CFG0:
        s->lowpower_cfg0 = v & 0xf;
        break;
    case A_LOWPOWER_CFG1:
        s->lowpower_cfg1 = v & LOWPOWER1_MASK;
        break;
    case A_ID0:
    case A_COMPVER0:
    case A_INTSTATUS0:
    case A_COMMON_INTSTATUS0:
        common_event(s, CI_WR2RO_ERR);
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA, addr);
        break;
    default:
        common_event(s, addr < 0x100 ? CI_DEC_ERR : CI_UNDEFINEDREG);
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to undefined offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DW_GDMA, addr);
        break;
    }
}

static const MemoryRegionOps esp32p4_dw_gdma_ops = {
    .read = esp32p4_dw_gdma_read,
    .write = esp32p4_dw_gdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* Handshake inputs */

/*
 * Whether the memory core would refuse an access to @mr as re-entrant: an
 * access to a block in the middle of one of its own, directly or through
 * the clock gate it is mapped behind (esp32p4_clkrst_map_gated()).
 */
static bool dw_region_in_access(MemoryRegion *mr)
{
    DeviceState *gated = esp32p4_clkrst_gated_device(mr);

    if (gated) {
        return gated->mem_reentrancy_guard.engaged_in_io;
    }
    return mr->dev && !mr->disable_reentrancy_guard && !mr->ram &&
           !mr->ram_device && !mr->rom_device && !mr->readonly &&
           mr->dev->mem_reentrancy_guard.engaged_in_io;
}

/*
 * Whether a running channel's source or destination is a device in the
 * middle of an access of its own, which the channel's next access would
 * re-enter.
 */
static bool dw_target_in_access(ESP32P4DwGdmaState *s)
{
    RCU_READ_LOCK_GUARD();

    for (int n = 0; n < NCH; n++) {
        ESP32P4DwGdmaChan *c = &s->ch[n];
        int master[2] = { c->ctl0 & CTL0_SMS ? 1 : 0,
                          c->ctl0 & CTL0_DMS ? 1 : 0 };
        uint32_t addr[2] = { c->sar, c->dar };
        uint32_t hi[2] = { c->sar_hi, c->dar_hi };

        if (!c->enabled) {
            continue;
        }
        for (int i = 0; i < 2; i++) {
            MemTxAttrs attrs = esp32p4_dma_attrs(
                ESP32P4_DMA_MASTER_GDMA_CH0 + n,
                ESP32P4_ICM_ID_AXI(master[i] ? ESP32P4_ICM_CID_VDMA_M2
                                             : ESP32P4_ICM_CID_VDMA_M1, n));
            hwaddr xlat, len = 1;
            MemoryRegion *mr;

            if (hi[i]) {
                continue;
            }
            mr = address_space_translate(s->as, addr[i], &xlat, &len, i == 1,
                                         attrs);
            if (dw_region_in_access(mr)) {
                return true;
            }
        }
    }
    return false;
}

/*
 * A handshake input changes at esp32_line_time(), after the rounds due
 * before it. Those rounds wait for their timer, though, when a channel's
 * peripheral is in the middle of an access, as when software's write of
 * a bridge's register changes its request: their accesses to it would be
 * refused as re-entrant (a bus error the hardware never gives). Rounds
 * fall due before a register access only on the host's clock, where
 * QEMU's timers can run late; then those see the change before their
 * time, and their timer runs them as soon as the access is over.
 */
static void dw_hs_catch_up(ESP32P4DwGdmaState *s)
{
    int64_t now = esp32_line_time();

    if (timer_pending(s->timer) && dw_due_ns(s) <= now &&
        dw_target_in_access(s)) {
        return;
    }
    dw_catch_up(s, now);
}

static void dw_hs_req(void *opaque, int n, int level)
{
    ESP32P4DwGdmaState *s = opaque;

    dw_hs_catch_up(s);
    s->hs_req[n] = level;
    dw_kick(s);
}

static void dw_hs_single(void *opaque, int n, int level)
{
    ESP32P4DwGdmaState *s = opaque;

    dw_hs_catch_up(s);
    s->hs_single[n] = level;
    dw_kick(s);
}

static void dw_hs_last(void *opaque, int n, int level)
{
    ESP32P4DwGdmaState *s = opaque;

    dw_hs_catch_up(s);
    s->hs_last[n] = level;
}

static void esp32p4_dw_gdma_reset_hold(Object *obj, ResetType type)
{
    ESP32P4DwGdmaState *s = ESP32P4_DW_GDMA(obj);

    timer_del(s->timer);
    if (!esp32p4_clk_count_running(&s->bus_edges)) {
        esp32p4_clk_count_start(&s->bus_edges, s->clk,
                                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    for (int n = 0; n < NCH; n++) {
        ESP32P4DwGdmaChan *c = &s->ch[n];

        memset(c, 0, sizeof(*c));
        c->ctl0 = c->sh_ctl0 = CTL0_RESET;
        c->cfg1 = CFG1_RESET;
        c->int_st_ena = CH_INT_RW_MASK | CH_INT_RO_ONES;
        c->int_sig_ena = CH_INT_RW_MASK | CH_INT_RO_ONES;
        c->phase = P_IDLE;
    }
    s->cfg0 = 0;
    s->common_int_st_ena = CI_RW_MASK | CI_RO_ONES;
    s->common_int_sig_ena = CI_RW_MASK | CI_RO_ONES;
    s->common_int_status = 0;
    s->lowpower_cfg0 = LOWPOWER0_RESET;
    s->lowpower_cfg1 = LOWPOWER1_RESET;
    s->rr_last[0] = s->rr_last[1] = 0;
}

static void esp32p4_dw_gdma_reset_exit(Object *obj, ResetType type)
{
    dw_update_irq(ESP32P4_DW_GDMA(obj));
}

static void esp32p4_dw_gdma_init(Object *obj)
{
    ESP32P4DwGdmaState *s = ESP32P4_DW_GDMA(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_dw_gdma_ops, s,
                          "esp32p4.vdma", ESP32P4_DW_GDMA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(dev, dw_hs_req, ESP32P4_DW_GDMA_HS_REQ,
                            ESP32P4_DW_GDMA_HS);
    qdev_init_gpio_in_named(dev, dw_hs_single, ESP32P4_DW_GDMA_HS_SINGLE,
                            ESP32P4_DW_GDMA_HS);
    qdev_init_gpio_in_named(dev, dw_hs_last, ESP32P4_DW_GDMA_HS_LAST,
                            ESP32P4_DW_GDMA_HS);
    qdev_init_gpio_out_named(dev, s->hs_ack, ESP32P4_DW_GDMA_HS_ACK,
                             ESP32P4_DW_GDMA_HS);
    s->clk = qdev_init_clock_in(dev, "clk", dw_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dw_timer_cb, s);
}

static void esp32p4_dw_gdma_realize(DeviceState *dev, Error **errp)
{
    ESP32P4DwGdmaState *s = ESP32P4_DW_GDMA(dev);

    if (!s->pms) {
        error_setg(errp, "%s: the 'pms' link is not set",
                   TYPE_ESP32P4_DW_GDMA);
        return;
    }
    s->as = esp32p4_pms_bus_as(ESP32P4_PMS(s->pms));
}

static void esp32p4_dw_gdma_finalize(Object *obj)
{
    timer_free(ESP32P4_DW_GDMA(obj)->timer);
}

static const Property esp32p4_dw_gdma_properties[] = {
    DEFINE_PROP_LINK("pms", ESP32P4DwGdmaState, pms, TYPE_ESP32P4_PMS,
                     Object *),
};

static const VMStateDescription vmstate_esp32p4_dw_gdma_chan = {
    .name = "esp32p4-dw-gdma-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sar_hi, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dar_hi, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(block_ts, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(ctl0, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(ctl1, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(cfg0, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(cfg1, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(llp0, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(llp1, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sh_sar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sh_dar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sh_block_ts, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sh_ctl0, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sh_ctl1, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(init_sar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(init_dar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(init_block_ts, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(init_ctl0, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(init_ctl1, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(status0, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(status1, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(swhs_src, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(swhs_dst, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(axi_id, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(axi_qos, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sstat, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dstat, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sstatar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(sstatar_hi, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dstatar, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dstatar_hi, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(int_st_ena, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(int_status, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(int_sig_ena, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(phase, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(enabled, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(disabling, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(susp, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(src_suspended, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(cur_lli, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(src_bytes, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dst_bytes, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(src_done, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(dst_done, ESP32P4DwGdmaChan),
        VMSTATE_UINT8_ARRAY(fifo, ESP32P4DwGdmaChan, ESP32P4_DW_GDMA_FIFO),
        VMSTATE_UINT32(fifo_head, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(fifo_num, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(src_txn, ESP32P4DwGdmaChan),
        VMSTATE_UINT32(dst_txn, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(src_txn_last, ESP32P4DwGdmaChan),
        VMSTATE_BOOL(dst_txn_last, ESP32P4DwGdmaChan),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_dw_gdma = {
    .name = TYPE_ESP32P4_DW_GDMA,
    .version_id = 4,
    .minimum_version_id = 4,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, ESP32P4DwGdmaState, NCH, 1,
                             vmstate_esp32p4_dw_gdma_chan,
                             ESP32P4DwGdmaChan),
        VMSTATE_UINT32(cfg0, ESP32P4DwGdmaState),
        VMSTATE_UINT32(common_int_st_ena, ESP32P4DwGdmaState),
        VMSTATE_UINT32(common_int_sig_ena, ESP32P4DwGdmaState),
        VMSTATE_UINT32(common_int_status, ESP32P4DwGdmaState),
        VMSTATE_UINT32(lowpower_cfg0, ESP32P4DwGdmaState),
        VMSTATE_UINT32(lowpower_cfg1, ESP32P4DwGdmaState),
        VMSTATE_BOOL_ARRAY(hs_req, ESP32P4DwGdmaState, ESP32P4_DW_GDMA_HS),
        VMSTATE_BOOL_ARRAY(hs_single, ESP32P4DwGdmaState,
                           ESP32P4_DW_GDMA_HS),
        VMSTATE_BOOL_ARRAY(hs_last, ESP32P4DwGdmaState, ESP32P4_DW_GDMA_HS),
        VMSTATE_UINT32_ARRAY(rr_last, ESP32P4DwGdmaState, 2),
        VMSTATE_CLOCK(clk, ESP32P4DwGdmaState),
        VMSTATE_ESP32P4_CLK_COUNT(bus_edges, ESP32P4DwGdmaState),
        VMSTATE_UINT64(due_cycle, ESP32P4DwGdmaState),
        VMSTATE_TIMER_PTR(timer, ESP32P4DwGdmaState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_dw_gdma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_dw_gdma_realize;
    dc->vmsd = &vmstate_esp32p4_dw_gdma;
    device_class_set_props(dc, esp32p4_dw_gdma_properties);
    rc->phases.hold = esp32p4_dw_gdma_reset_hold;
    rc->phases.exit = esp32p4_dw_gdma_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.dma] */
static const TypeInfo esp32p4_dw_gdma_info = {
    .name           = TYPE_ESP32P4_DW_GDMA,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(ESP32P4DwGdmaState),
    .instance_init  = esp32p4_dw_gdma_init,
    .instance_finalize = esp32p4_dw_gdma_finalize,
    .class_init     = esp32p4_dw_gdma_class_init,
};

static void esp32p4_dw_gdma_register_types(void)
{
    type_register_static(&esp32p4_dw_gdma_info);
}
type_init(esp32p4_dw_gdma_register_types)
