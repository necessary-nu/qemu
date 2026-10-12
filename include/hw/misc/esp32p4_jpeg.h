/*
 * ESP32-P4 JPEG codec
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_JPEG_H
#define HW_MISC_ESP32P4_JPEG_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/misc/esp32p4_clkwait.h"
#include "hw/misc/esp32p4_jpeg_codec.h"
#include "qom/object.h"

#define TYPE_ESP32P4_JPEG "esp32p4-jpeg"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4JpegState, ESP32P4_JPEG)

#define ESP32P4_JPEG_BASE   0x50086000
#define ESP32P4_JPEG_SIZE   0x1000

/*
 * The codec's 2D-DMA paths
 * ========================
 *
 * The codec moves its data only through the 2D-DMA, as its peripheral 0
 * on both paths (DMA2D_OUT_PERI_SEL_CHn and DMA2D_IN_PERI_SEL_CHn 0, TRM
 * tables 8.4-2 and 8.4-3). What crosses each path:
 *
 *   - TX, 2D-DMA to codec. Encoding: the source image as the TX
 *     channel's macroblock reordering (DMA2D_OUT_MACRO_BLOCK_SIZE_CHn) and
 *     padding (TRM 8.4.3) deliver it, one MCU's macroblock after another,
 *     left to right and top to bottom, each macroblock's pixels row by
 *     row at the source format's bytes per pixel (1.5 for YUV420, whose
 *     rows alternate U Y Y and V Y Y per pixel pair). Decoding: the
 *     bitstream, from its SOS marker. The end of the frame, the suc_eof
 *     of the TX descriptor, comes with the byte it ends.
 *   - RX, codec to 2D-DMA. Encoding: the scan's entropy-coded data,
 *     followed by EOI where JPEG_TAILER_EN asks. Decoding: each MCU's data
 *     units, 64 samples of 8 bits each row by row, in the order of TRM
 *     figure 8.4-4: GRAY Y; YUV444 Y U V; YUV422 Y0 Y1 U V; YUV420 Y0 Y1 Y2
 *     Y3 U V, the Y units left to right then top to bottom. The frame's
 *     last byte carries ESP32P4_JPEG_DMA_EOF, or ESP32P4_JPEG_DMA_ERR_EOF
 *     where the decoder found the data in error (the descriptor's
 *     err_eof).
 *
 * The 2D-DMA, or whatever serves the codec's paths, connects with
 * esp32p4_jpeg_connect_dma(), giving its side of them with the meaning
 * GDMA gives a peripheral (include/hw/dma/esp32p4_gdma.h): ops->pull
 * takes up to @len TX bytes, returning how many it got, and sets *eof at
 * the frame's end; ops->push gives RX bytes and returns how many the
 * channel took, a flag marking the last byte given (none unless it took
 * them all), the flags' values GDMA's. It tells the codec of bytes
 * arriving in an empty TX FIFO and of room appearing in a full RX FIFO
 * (and of an RX channel starting) with esp32p4_jpeg_dma_ready(), and
 * esp32p4_jpeg_dma_sync() runs what the codec has due by a time, as
 * GDMA's ops->sync does: the two serve as the tx_ready, rx_ready and sync
 * callbacks a GDMA peripheral gives its controller.
 *
 * Unconnected, the paths are a qtest's: the codec's "dma" property
 * (hw/misc/esp32p4_jpeg.c) queues TX data from memory and RX buffers in
 * memory, the 2D-DMA's descriptors as a test plays them.
 */
#define ESP32P4_JPEG_DMA_EOF        (1u << 0)
#define ESP32P4_JPEG_DMA_ERR_EOF    (1u << 1)

typedef struct ESP32P4JpegDmaOps {
    size_t (*pull)(void *opaque, uint8_t *buf, size_t len, bool *eof);
    size_t (*push)(void *opaque, const uint8_t *buf, size_t len,
                   unsigned flags);
} ESP32P4JpegDmaOps;

void esp32p4_jpeg_connect_dma(ESP32P4JpegState *s,
                              const ESP32P4JpegDmaOps *ops, void *opaque);
void esp32p4_jpeg_dma_ready(void *opaque);
void esp32p4_jpeg_dma_sync(void *opaque, int64_t when);

#define ESP32P4_JPEG_HT(cls, t)    ((cls) * 2 + (t))

/* The qtest stand-in's queue of descriptors, each way. */
#define ESP32P4_JPEG_QT_SEGS    16

typedef struct ESP32P4JpegQtSeg {
    uint32_t addr;
    uint32_t len;
    uint32_t done;
    bool eof;
} ESP32P4JpegQtSeg;

/* The codec's data FIFOs and the MCU in its pipeline, in bytes. */
#define ESP32P4_JPEG_IN_SIZE    8192
#define ESP32P4_JPEG_OUT_SIZE   4096
#define ESP32P4_JPEG_STAGE_SIZE 3072

struct ESP32P4JpegState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    /* The MCU in the pipeline; the decoder's two timeouts. */
    ESP32P4ClkWait work;
    ESP32P4ClkWait tmo_stall;
    ESP32P4ClkWait tmo_frame;

    /* Registers. */
    uint32_t config;
    uint32_t dqt_info;
    uint32_t pic_size;
    uint32_t extd_config;
    uint32_t decode_conf;
    uint32_t comp[4];
    uint32_t dht_info;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t decoder_status5;
    uint32_t eco_low;
    uint32_t eco_high;
    uint32_t sys;
    uint32_t version;

    /* The quantisation and Huffman table RAMs and their FIFO ports. */
    uint16_t qnr[4][64];
    /* By ESP32P4_JPEG_HT(class, table), class 0 DC and 1 AC; length - 1. */
    uint16_t dht_totlen[4][16];
    uint16_t dht_codemin[4][16];
    uint8_t dht_val_dc[2][16];
    uint8_t dht_val_ac[2][256];
    uint8_t qnr_ptr[4];
    uint8_t totlen_ptr[4];
    uint8_t codemin_ptr[4];
    uint16_t val_ptr[4];

    /* The frame in progress. */
    uint32_t phase;
    uint32_t format;            /* coded format: GRAY, YUV444, ... */
    uint32_t src;               /* encoder: source format */
    bool pixel_rev;
    bool ff_check;
    bool tailer;
    bool direct;
    uint32_t ncomp;
    uint32_t mcus;
    uint32_t mcu;
    uint32_t restart;
    bool in_eof;
    bool frame_err;
    int32_t pred[4];
    /* Decoder: the scan's tables by component, as SOS selects them. */
    uint8_t dc_sel[4];
    uint8_t ac_sel[4];
    uint8_t q_sel[4];
    uint32_t qmask[4];
    /* Encoder: the luminance and chrominance tables' contents. */
    uint16_t enc_q[2][64];
    /* Coded data: the encoder's bits not yet a byte, the decoder's. */
    uint64_t bit_acc;
    int32_t bit_count;

    uint8_t in_buf[ESP32P4_JPEG_IN_SIZE];
    uint32_t in_len;
    uint32_t in_pos;
    uint8_t stage[ESP32P4_JPEG_STAGE_SIZE];
    uint32_t stage_len;
    uint8_t out[ESP32P4_JPEG_OUT_SIZE];
    uint32_t out_head;
    uint32_t out_num;
    /* The frame's output is all in out[], its end marked with these. */
    bool out_final;
    uint32_t out_flags;

    /* Built from the RAM at the scan's start; not migrated. */
    ESP32P4JpegHuffDec hdec[2][2];

    const ESP32P4JpegDmaOps *dma_ops;
    void *dma_opaque;
    /* jpeg_run() is running; something asked for it again meanwhile. */
    bool running;
    bool rerun;

    /* The qtest stand-in for the 2D-DMA. */
    ESP32P4JpegQtSeg qt_tx[ESP32P4_JPEG_QT_SEGS];
    ESP32P4JpegQtSeg qt_rx[ESP32P4_JPEG_QT_SEGS];
    uint32_t qt_tx_num;
    uint32_t qt_rx_num;
    uint32_t qt_rx_total;
    uint32_t qt_rx_flags;
};

#endif
