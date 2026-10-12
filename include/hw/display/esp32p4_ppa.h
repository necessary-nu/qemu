/*
 * ESP32-P4 pixel-processing accelerator (PPA)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_ESP32P4_PPA_H
#define HW_DISPLAY_ESP32P4_PPA_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/dma/esp32p4_dma2d.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qom/object.h"

#define TYPE_ESP32P4_PPA "esp32p4-ppa"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4PpaState, ESP32P4_PPA)

#define ESP32P4_PPA_BASE        0x50087000
#define ESP32P4_PPA_SIZE        0x1000

/* The registers, 0x000 to 0x100, as words. */
#define ESP32P4_PPA_REGS        (0x104 / 4)
#define ESP32P4_PPA_CLUT_SIZE   256

/*
 * The SRM's pixel blocks: an input block of at most 36 x 36 pixels of four
 * bytes, and an output block of at most 32 x 32, two of each.
 */
#define ESP32P4_PPA_SRM_IN_MAX  (36 * 36 * 4)
#define ESP32P4_PPA_SRM_OUT_MAX (32 * 32 * 4)
/* BLEND: the background's row pair and the foreground's row. */
#define ESP32P4_PPA_ROW_MAX     (8191 * 4)
#define ESP32P4_PPA_OUT_MAX     8192
#define ESP32P4_PPA_BATCH_MAX   1024

typedef struct ESP32P4PpaOutBlock {
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
    uint32_t len;
    uint32_t pushed;
    bool declared;
    bool last;
    uint8_t data[ESP32P4_PPA_SRM_OUT_MAX];
} ESP32P4PpaOutBlock;

struct ESP32P4PpaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4ClkEdges edges;
    Object *dma2d_obj;
    ESP32P4Dma2dState *dma2d;
    int srm_timer;
    int blend_timer;

    uint32_t regs[ESP32P4_PPA_REGS];
    uint32_t clut[2][ESP32P4_PPA_CLUT_SIZE];
    uint32_t clut_waddr[2];
    uint32_t clut_raddr[2];
    uint32_t clut_cnt[2];

    /*
     * SRM: 0 idle, 1 waiting for its descriptors, 2 running. The
     * configuration is latched at the start.
     */
    uint32_t srm_state;
    /* SR_COLOR_MODE, _BYTE_ORDER, _FIX_ALPHA and _SCAL_ROTATE. */
    uint32_t srm_cfg[4];
    uint32_t srm_dbg[2];
    uint32_t srm_gray;
    uint32_t srm_hb;
    uint32_t srm_vb;
    uint32_t srm_mb;            /* the macroblock size, 16 or 32 */
    uint32_t srm_nmb;
    uint32_t srm_req;           /* the next macroblock to ask for */
    uint32_t srm_comp;          /* the macroblock being computed */
    uint32_t srm_sub;           /* ... its output block */
    uint32_t srm_slot_mb[2];
    uint32_t srm_slot_fill[2];  /* bytes in, or UINT32_MAX when empty */
    uint8_t srm_in[2][ESP32P4_PPA_SRM_IN_MAX];
    bool srm_computing;
    int64_t srm_busy_until;
    ESP32P4PpaOutBlock srm_out[2];
    uint32_t srm_out_head;
    uint32_t srm_out_num;
    bool srm_stalled;

    /*
     * BLEND: 0 idle, 1 waiting for its descriptors, 2 running; mode 0
     * blend, 1 bypass, 2 fill.
     */
    uint32_t blend_state;
    uint32_t blend_mode;
    uint32_t blend_cfg[12];     /* BLEND_COLOR_MODE .. CK_DEFAULT, latched */
    uint32_t blend_dbg[2];
    uint32_t blend_gray;
    uint32_t blend_w;
    uint32_t blend_h;
    uint32_t blend_row;
    uint32_t blend_col;
    uint8_t *bg_row;            /* two rows */
    uint32_t bg_fill;
    uint8_t *fg_row;
    uint32_t fg_fill;
    uint8_t *out;               /* a ring of the bytes ready to push */
    uint32_t out_head;
    uint32_t out_num;
    uint32_t out_total;         /* the bytes of the frame still to push */
    uint8_t batch[ESP32P4_PPA_BATCH_MAX * 4 + 8];
    uint32_t batch_len;
    bool batch_pending;
    int64_t blend_busy_until;
    uint32_t enc_pend;          /* YUV output: a pixel waiting for its pair */

    uint32_t row_buf_size;
    uint32_t out_buf_size;
};

#endif
