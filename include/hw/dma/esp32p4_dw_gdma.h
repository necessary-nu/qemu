/*
 * ESP32-P4 VDMA, a Synopsys DesignWare AXI DMA controller (DW_axi_dmac)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32P4_DW_GDMA_H
#define HW_DMA_ESP32P4_DW_GDMA_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/core/clock.h"
#include "qemu/notify.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/memory.h"

#define TYPE_ESP32P4_DW_GDMA "esp32p4-dw-gdma"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4DwGdmaState, ESP32P4_DW_GDMA)

#define ESP32P4_DW_GDMA_BASE        0x50081000
#define ESP32P4_DW_GDMA_SIZE        0x1000

#define ESP32P4_DW_GDMA_CHANNELS    4
/* Each channel's FIFO: 64 entries of 64 bits. */
#define ESP32P4_DW_GDMA_FIFO        512

/*
 * The hardware handshaking interfaces (TRM table 7.4-1): a channel's
 * SRC_PER or DST_PER names one of them.
 */
#define ESP32P4_DW_GDMA_HS_DSI          0
#define ESP32P4_DW_GDMA_HS_ISP_TO_MEM   1
#define ESP32P4_DW_GDMA_HS_MEM_TO_ISP   2
#define ESP32P4_DW_GDMA_HS              3

/*
 * How a peripheral handshakes with VDMA
 * =====================================
 *
 * The data moves over the bus: VDMA reads a source peripheral's FIFO, or
 * writes a destination's, at the address SAR or DAR holds, through AXI
 * master 1, the only one that reaches the peripherals. A peripheral paces
 * the transfer through its handshaking interface, the named GPIOs below,
 * one line per interface:
 *
 *   ESP32P4_DW_GDMA_HS_REQ, "hs-req" (input): high while the peripheral
 *     asks for a burst transaction of SRC_MSIZE (DST_MSIZE) items.
 *   ESP32P4_DW_GDMA_HS_SINGLE, "hs-single" (input): high while it asks for
 *     a single item.
 *   ESP32P4_DW_GDMA_HS_LAST, "hs-last" (input): high with a request that is
 *     the block's last, where the peripheral is the flow controller.
 *   ESP32P4_DW_GDMA_HS_ACK, "hs-ack" (output): pulsed as the transaction a
 *     request asked for completes; a request still high asks for another.
 *
 * A channel takes these when its CFG1.HS_SEL_SRC (_DST) selects hardware
 * handshaking; with software handshaking SWHSSRC (SWHSDST) gives them.
 *
 * The interrupt is sysbus IRQ 0, AXI_DMAC_INTR (interrupt source 24).
 */
#define ESP32P4_DW_GDMA_HS_REQ      "hs-req"
#define ESP32P4_DW_GDMA_HS_SINGLE   "hs-single"
#define ESP32P4_DW_GDMA_HS_LAST     "hs-last"
#define ESP32P4_DW_GDMA_HS_ACK      "hs-ack"

typedef struct ESP32P4DwGdmaChan {
    /* The registers the current block runs with. */
    uint32_t sar;
    uint32_t sar_hi;
    uint32_t dar;
    uint32_t dar_hi;
    uint32_t block_ts;
    uint32_t ctl0;
    uint32_t ctl1;
    uint32_t cfg0;
    uint32_t cfg1;
    uint32_t llp0;
    uint32_t llp1;
    /* Shadow-register mode: software's next block. */
    uint32_t sh_sar;
    uint32_t sh_dar;
    uint32_t sh_block_ts;
    uint32_t sh_ctl0;
    uint32_t sh_ctl1;
    /* Auto-reload: the values the channel was enabled with. */
    uint32_t init_sar;
    uint32_t init_dar;
    uint32_t init_block_ts;
    uint32_t init_ctl0;
    uint32_t init_ctl1;
    uint32_t status0;
    uint32_t status1;
    uint32_t swhs_src;
    uint32_t swhs_dst;
    uint32_t axi_id;
    uint32_t axi_qos;
    uint32_t sstat;
    uint32_t dstat;
    uint32_t sstatar;
    uint32_t sstatar_hi;
    uint32_t dstatar;
    uint32_t dstatar_hi;
    uint32_t int_st_ena;
    uint32_t int_status;
    uint32_t int_sig_ena;

    /* The channel's state: see the .c. */
    uint32_t phase;
    bool enabled;
    bool disabling;
    bool susp;
    bool src_suspended;
    uint32_t cur_lli;
    uint32_t src_bytes;
    uint32_t dst_bytes;
    bool src_done;
    bool dst_done;
    uint8_t fifo[ESP32P4_DW_GDMA_FIFO];
    uint32_t fifo_head;
    uint32_t fifo_num;
    /* The bytes left of a handshaking burst under way, larger than it. */
    uint32_t src_txn;
    uint32_t dst_txn;
    bool src_txn_last;
    bool dst_txn_last;
} ESP32P4DwGdmaChan;

struct ESP32P4DwGdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq hs_ack[ESP32P4_DW_GDMA_HS];
    Clock *clk;
    QEMUTimer *timer;
    /*
     * The bus clock's cycles, counted from its last change, and the cycle
     * of the round @timer runs.
     */
    ESP32P4ClkCount bus_edges;
    uint64_t due_cycle;
    AddressSpace *as;
    Object *pms;

    ESP32P4DwGdmaChan ch[ESP32P4_DW_GDMA_CHANNELS];
    uint32_t cfg0;
    uint32_t common_int_st_ena;
    uint32_t common_int_sig_ena;
    uint32_t common_int_status;
    uint32_t lowpower_cfg0;
    uint32_t lowpower_cfg1;
    bool hs_req[ESP32P4_DW_GDMA_HS];
    bool hs_single[ESP32P4_DW_GDMA_HS];
    bool hs_last[ESP32P4_DW_GDMA_HS];
    /* The arbiters' last grants, reads and writes. */
    uint32_t rr_last[2];

    /* The streams feeding VDMA's handshakes (esp32p4_dw_gdma_add_source()) */
    NotifierList sources;
    bool syncing_sources;
    /* A round is under way (dw_run()) */
    bool in_round;
};

/*
 * Run the rounds of the bus that fell due by @now, in order, each at its
 * own time, which VDMA's QEMU timer, late without icount, has yet to run.
 * For a peripheral VDMA feeds (the DSI bridge's FIFO) that takes data out
 * at @now, also while it catches up in one of its own register accesses:
 * the rounds' writes reach it re-entrantly then, so the region they write
 * must allow that (disable_reentrancy_guard).
 */
void esp32p4_dw_gdma_sync(ESP32P4DwGdmaState *s, int64_t now);

/*
 * A peripheral that VDMA reads through a handshake interface, its data
 * coming from a stream of QEMU timers of its own that run late without
 * icount (the CSI bridge, a camera's frames through the CSI host and the
 * ISP), adds a notifier: VDMA's register accesses first call it, at their
 * time (esp32_line_time()), and it brings the stream up to then, each of
 * the stream's events at its own time, with VDMA's rounds due before each
 * brought in first (esp32p4_dw_gdma_sync()). The requests the stream
 * raises then come at their times, and VDMA's rounds take what it had
 * brought by theirs, however late its timers.
 */
void esp32p4_dw_gdma_add_source(ESP32P4DwGdmaState *s, Notifier *n);

#endif
