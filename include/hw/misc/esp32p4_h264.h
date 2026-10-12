/*
 * ESP32-P4 H264 encoder and its dedicated DMA (H264_DMA)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_H264_H
#define HW_MISC_ESP32P4_H264_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clkwait.h"
#include "hw/misc/esp32p4_h264_enc.h"
#include "qom/object.h"

#define TYPE_ESP32P4_H264 "esp32p4-h264"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4H264State, ESP32P4_H264)

#define ESP32P4_H264_BASE           0x50084000
#define ESP32P4_H264_DMA_BASE       0x500a7000

/* The encoder's registers and H264_DMA's, the device's MMIO regions. */
enum {
    ESP32P4_H264_MMIO_ENC,
    ESP32P4_H264_MMIO_DMA,
};
#define ESP32P4_H264_ENC_SIZE       0x1000
#define ESP32P4_H264_DMA_SIZE       0x1000

#define ESP32P4_H264_TX             5
#define ESP32P4_H264_RX             6

/*
 * The sysbus interrupts, in the order of their interrupt sources (115 to
 * 126): H264_DMA_OUT_CH0..4, H264_DMA_IN_CH0..5, then H264_REG_INT.
 */
#define ESP32P4_H264_IRQ_TX(n)      (n)
#define ESP32P4_H264_IRQ_RX(n)      (ESP32P4_H264_TX + (n))
#define ESP32P4_H264_IRQ_REG        (ESP32P4_H264_TX + ESP32P4_H264_RX)
#define ESP32P4_H264_IRQS           (ESP32P4_H264_IRQ_REG + 1)

/* A video sequence's 19 registers (H264_x_SYS_MB_RES to _ROI_CONFIG). */
#define ESP32P4_H264_SEQ_REGS       19

typedef struct ESP32P4H264Chan {
    /* Registers. */
    uint32_t conf0;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t link_conf;
    uint32_t link_addr;
    uint32_t arb;
    uint32_t etm_conf;
    uint32_t pd_conf;
    uint32_t eof_addr;
    uint32_t err_eof_addr;
    uint32_t dscr;
    uint32_t dscr_bf0;
    uint32_t dscr_bf1;

    /* The link: not started, running, or ended (its list, or a check). */
    int32_t running;
    bool have_desc;
    uint32_t desc_addr;
    uint32_t dw[4];
    /* Bytes (1D) or lines (2D) of the current descriptor done. */
    uint32_t pos;
} ESP32P4H264Chan;

struct ESP32P4H264State {
    SysBusDevice parent_obj;

    MemoryRegion iomem[2];
    qemu_irq irq[ESP32P4_H264_IRQS];
    Clock *clk;
    Object *pms;
    AddressSpace *as;

    /* The encoder's registers. */
    bool frame_mode;
    uint32_t gop_conf;
    uint32_t seq[2][ESP32P4_H264_SEQ_REGS];
    uint32_t rc_status[3];
    uint32_t slice_remain;
    uint32_t slice_len;
    uint32_t bs_threshold;
    uint32_t slice_byte[2];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf;
    uint32_t mv_merge;
    uint32_t debug_dma_sel;
    uint32_t sys_status;
    uint32_t frame_code_length;
    uint32_t date;
    uint32_t ori_conf[2];
    uint32_t debug_conf[7];

    /* H264_DMA's registers. */
    ESP32P4H264Chan tx[ESP32P4_H264_TX];
    ESP32P4H264Chan rx[ESP32P4_H264_RX];
    uint32_t mode_enable;
    uint32_t mode_yuv;
    uint32_t block_start;
    uint32_t block_row_len;
    uint32_t block_len;
    uint32_t rst_conf;
    uint32_t mem_addr[8];
    uint32_t arb_config[2];
    uint32_t dma_date;
    uint32_t counter_rst;
    uint32_t pbyte;
    uint32_t ch_dbg_en;
    uint32_t dbg_data[22];
    /* RX channels 0, 1, 2 and 5's counters, in 8-byte units. */
    uint32_t cnt[4];
    uint32_t axi_err[2];

    /* The picture being coded. */
    H264Enc enc;
    int32_t state;
    int32_t stream;
    int32_t row;
    uint32_t frames;
    bool intra;
    uint64_t code_bytes;
    bool bs_overflow;
    int32_t tmp_written;
    bool tmp_ready;
    bool tmp_pending;
    ESP32P4ClkWait row_wait;
    ESP32P4ClkWait tmp_wait;

    /*
     * Moving the reference from the deblocking filter's output in memory
     * into the three-row window RX channel 5 keeps, a row at a time, in
     * passes over the picture: the pass (generation) and row moved next,
     * the rows DMA_MOVE_START still moves before DMA_MOVE_2MB_LINE_DONE,
     * the pass the next P picture and the current one take, and what
     * each of the window's rows holds.
     */
    bool mv_started;
    uint32_t mv_gen;
    int32_t mv_row;
    int32_t mv_eager;
    uint32_t next_ref_gen;
    uint32_t ref_gen;
    int32_t ring_row[3];
    uint32_t ring_gen[3];
    int32_t ring_next;
    ESP32P4ClkWait move_wait;

    /* Lines of the source read for one macroblock row. */
    uint8_t *lines;
    uint32_t lines_size;
};

#endif
