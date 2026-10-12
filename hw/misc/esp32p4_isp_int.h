/*
 * ESP32-P4 ISP: register layout, and the pipeline's arithmetic interface
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ISP_INT_H
#define HW_MISC_ESP32P4_ISP_INT_H

#include "hw/misc/esp32p4_isp.h"

/* Registers, byte offsets (isp_reg.h for v3) */
#define A_VER_DATE                 0x000
#define A_CLK_EN                   0x004
#define A_CNTL                     0x008
#define A_HSYNC_CNT                0x00c
#define A_FRAME_CFG                0x010
#define A_CCM_COEF0                0x014
#define A_CCM_COEF1                0x018
#define A_CCM_COEF3                0x01c
#define A_CCM_COEF4                0x020
#define A_CCM_COEF5                0x024
#define A_BF_MATRIX_CTRL           0x028
#define A_BF_SIGMA                 0x02c
#define A_BF_GAU0                  0x030
#define A_BF_GAU1                  0x034
#define A_DPC_CTRL                 0x038
#define A_DPC_CONF                 0x03c
#define A_DPC_MATRIX_CTRL          0x040
#define A_DPC_DEADPIX_CNT          0x044
#define A_LUT_CMD                  0x048
#define A_LUT_WDATA                0x04c
#define A_LUT_RDATA                0x050
#define A_LSC_TABLESIZE            0x054
#define A_DEMOSAIC_MATRIX_CTRL     0x058
#define A_DEMOSAIC_GRAD_RATIO      0x05c
#define A_MEDIAN_MATRIX_CTRL       0x060
#define A_INT_RAW                  0x064
#define A_INT_ST                   0x068
#define A_INT_ENA                  0x06c
#define A_INT_CLR                  0x070
#define A_GAMMA_CTRL               0x074
#define A_GAMMA_RY1                0x078
#define A_GAMMA_RY2                0x07c
#define A_GAMMA_RY3                0x080
#define A_GAMMA_RY4                0x084
#define A_GAMMA_GY1                0x088
#define A_GAMMA_GY2                0x08c
#define A_GAMMA_GY3                0x090
#define A_GAMMA_GY4                0x094
#define A_GAMMA_BY1                0x098
#define A_GAMMA_BY2                0x09c
#define A_GAMMA_BY3                0x0a0
#define A_GAMMA_BY4                0x0a4
#define A_GAMMA_RX1                0x0a8
#define A_GAMMA_RX2                0x0ac
#define A_GAMMA_GX1                0x0b0
#define A_GAMMA_GX2                0x0b4
#define A_GAMMA_BX1                0x0b8
#define A_GAMMA_BX2                0x0bc
#define A_AE_CTRL                  0x0c0
#define A_AE_MONITOR               0x0c4
#define A_AE_BX                    0x0c8
#define A_AE_BY                    0x0cc
#define A_AE_WINPIXNUM             0x0d0
#define A_AE_WIN_RECIPROCAL        0x0d4
#define A_AE_BLOCK_MEAN_0          0x0d8
#define A_AE_BLOCK_MEAN_1          0x0dc
#define A_AE_BLOCK_MEAN_2          0x0e0
#define A_AE_BLOCK_MEAN_3          0x0e4
#define A_AE_BLOCK_MEAN_4          0x0e8
#define A_AE_BLOCK_MEAN_5          0x0ec
#define A_AE_BLOCK_MEAN_6          0x0f0
#define A_SHARP_CTRL0              0x0f4
#define A_SHARP_FILTER0            0x0f8
#define A_SHARP_FILTER1            0x0fc
#define A_SHARP_FILTER2            0x100
#define A_SHARP_MATRIX_CTRL        0x104
#define A_SHARP_CTRL1              0x108
#define A_DMA_CNTL                 0x10c
#define A_DMA_RAW_DATA             0x110
#define A_CAM_CNTL                 0x114
#define A_CAM_CONF                 0x118
#define A_AF_CTRL0                 0x11c
#define A_AF_CTRL1                 0x120
#define A_AF_GEN_TH_CTRL           0x124
#define A_AF_ENV_USER_TH_SUM       0x128
#define A_AF_ENV_USER_TH_LUM       0x12c
#define A_AF_THRESHOLD             0x130
#define A_AF_HSCALE_A              0x134
#define A_AF_VSCALE_A              0x138
#define A_AF_HSCALE_B              0x13c
#define A_AF_VSCALE_B              0x140
#define A_AF_HSCALE_C              0x144
#define A_AF_VSCALE_C              0x148
#define A_AF_SUM_A                 0x14c
#define A_AF_SUM_B                 0x150
#define A_AF_SUM_C                 0x154
#define A_AF_LUM_A                 0x158
#define A_AF_LUM_B                 0x15c
#define A_AF_LUM_C                 0x160
#define A_AWB_MODE                 0x164
#define A_AWB_HSCALE               0x168
#define A_AWB_VSCALE               0x16c
#define A_AWB_TH_LUM               0x170
#define A_AWB_TH_RG                0x174
#define A_AWB_TH_BG                0x178
#define A_AWB0_WHITE_CNT           0x17c
#define A_AWB0_ACC_R               0x180
#define A_AWB0_ACC_G               0x184
#define A_AWB0_ACC_B               0x188
#define A_COLOR_CTRL               0x18c
#define A_BLC_VALUE                0x190
#define A_BLC_CTRL0                0x194
#define A_BLC_CTRL1                0x198
#define A_BLC_CTRL2                0x19c
#define A_BLC_MEAN                 0x1a0
#define A_HIST_MODE                0x1a4
#define A_HIST_COEFF               0x1a8
#define A_HIST_OFFS                0x1ac
#define A_HIST_SIZE                0x1b0
#define A_HIST_SEG0                0x1b4
#define A_HIST_SEG1                0x1b8
#define A_HIST_SEG2                0x1bc
#define A_HIST_SEG3                0x1c0
#define A_HIST_WEIGHT0             0x1c4
#define A_HIST_WEIGHT1             0x1c8
#define A_HIST_WEIGHT2             0x1cc
#define A_HIST_WEIGHT3             0x1d0
#define A_HIST_WEIGHT4             0x1d4
#define A_HIST_WEIGHT5             0x1d8
#define A_HIST_WEIGHT6             0x1dc
#define A_HIST_BIN0                0x1e0
#define A_HIST_BIN1                0x1e4
#define A_HIST_BIN2                0x1e8
#define A_HIST_BIN3                0x1ec
#define A_HIST_BIN4                0x1f0
#define A_HIST_BIN5                0x1f4
#define A_HIST_BIN6                0x1f8
#define A_HIST_BIN7                0x1fc
#define A_HIST_BIN8                0x200
#define A_HIST_BIN9                0x204
#define A_HIST_BIN10               0x208
#define A_HIST_BIN11               0x20c
#define A_HIST_BIN12               0x210
#define A_HIST_BIN13               0x214
#define A_HIST_BIN14               0x218
#define A_HIST_BIN15               0x21c
#define A_MEM_AUX_CTRL_0           0x220
#define A_MEM_AUX_CTRL_1           0x224
#define A_MEM_AUX_CTRL_2           0x228
#define A_MEM_AUX_CTRL_3           0x22c
#define A_MEM_AUX_CTRL_4           0x230
#define A_YUV_FORMAT               0x234
#define A_RDN_ECO_CS               0x238
#define A_RDN_ECO_LOW              0x23c
#define A_RDN_ECO_HIGH             0x240
#define A_CROP_CTRL                0x244
#define A_CROP_Y_CAPTURE           0x248
#define A_CROP_X_CAPTURE           0x24c
#define A_CROP_ERR_ST              0x250
#define A_WBG_COEF_R               0x254
#define A_WBG_COEF_G               0x258
#define A_WBG_COEF_B               0x25c
#define A_COLOR_HUE_CTRL           0x260
#define A_AWB_BX                   0x264
#define A_AWB_BY                   0x268
#define A_STATE                    0x26c
#define A_SHADOW_REG_CTRL          0x270

#define R(a)                       ((a) / 4)

/* CNTL */
#define CNTL_MIPI_DATA_EN          BIT(0)
#define CNTL_ISP_EN                BIT(1)
#define CNTL_BLC_EN                BIT(2)
#define CNTL_DPC_EN                BIT(3)
#define CNTL_BF_EN                 BIT(4)
#define CNTL_LSC_EN                BIT(5)
#define CNTL_DEMOSAIC_EN           BIT(6)
#define CNTL_MEDIAN_EN             BIT(7)
#define CNTL_CCM_EN                BIT(8)
#define CNTL_GAMMA_EN              BIT(9)
#define CNTL_RGB2YUV_EN            BIT(10)
#define CNTL_SHARP_EN              BIT(11)
#define CNTL_COLOR_EN              BIT(12)
#define CNTL_YUV2RGB_EN            BIT(13)
#define CNTL_AE_EN                 BIT(14)
#define CNTL_AF_EN                 BIT(15)
#define CNTL_AWB_EN                BIT(16)
#define CNTL_HIST_EN               BIT(17)
#define CNTL_CROP_EN               BIT(18)
#define CNTL_WBG_EN                BIT(19)
#define CNTL_BYTE_ENDIAN_ORDER     BIT(24)

/* Output types (CNTL's ISP_OUT_TYPE) */
#define OUT_RAW8                   0
#define OUT_YUV422                 1
#define OUT_RGB888                 2
#define OUT_YUV420                 3
#define OUT_RGB565                 4

/* INT_RAW and friends */
#define INT_DATA_TYPE_ERR          BIT(0)
#define INT_ASYNC_FIFO_OVF         BIT(1)
#define INT_BUF_FULL               BIT(2)
#define INT_HVNUM_SETTING_ERR      BIT(3)
#define INT_DATA_TYPE_SETTING_ERR  BIT(4)
#define INT_MIPI_HNUM_UNMATCH      BIT(5)
#define INT_DPC_CHECK_DONE         BIT(6)
#define INT_GAMMA_XCOORD_ERR       BIT(7)
#define INT_AE_MONITOR             BIT(8)
#define INT_AE_FRAME_DONE          BIT(9)
#define INT_AF_FDONE               BIT(10)
#define INT_AF_ENV                 BIT(11)
#define INT_AWB_FDONE              BIT(12)
#define INT_HIST_FDONE             BIT(13)
#define INT_FRAME                  BIT(14)
#define INT_BLC_FRAME              BIT(15)
#define INT_LSC_FRAME              BIT(16)
#define INT_DPC_FRAME              BIT(17)
#define INT_BF_FRAME               BIT(18)
#define INT_DEMOSAIC_FRAME         BIT(19)
#define INT_MEDIAN_FRAME           BIT(20)
#define INT_CCM_FRAME              BIT(21)
#define INT_GAMMA_FRAME            BIT(22)
#define INT_RGB2YUV_FRAME          BIT(23)
#define INT_SHARP_FRAME            BIT(24)
#define INT_COLOR_FRAME            BIT(25)
#define INT_YUV2RGB_FRAME          BIT(26)
#define INT_TAIL_IDI_FRAME         BIT(27)
#define INT_HEADER_IDI_FRAME       BIT(28)
#define INT_CROP_FRAME             BIT(29)
#define INT_WBG_FRAME              BIT(30)
#define INT_CROP_ERR               BIT(31)

/* LUTs (LUT_CMD's LUT_NUM) */
#define LUT_LSC                    0
#define LUT_DPC                    1
#define LUT_AWB                    2
#define ISP_LSC_LUT_DEPTH          560
#define ISP_DPC_LUT_DEPTH          512
#define ISP_AWB_LUT_DEPTH          100

/*
 * The pipeline
 * ============
 *
 * esp32p4_isp_pipe.c computes a frame a row at a time, as the rows come
 * in: isp_pipe_row() takes each input row, its samples already reduced
 * to 8 bits by the header, and every row the pipeline can complete with
 * it goes out through the emit callback, packed in ISP_OUT_TYPE's layout
 * (or with no data, for a row CROP drops), each stage holding the few
 * rows its 3 x 3 or 5 x 5 window needs; isp_pipe_finish() completes the
 * frame's remaining rows and its statistics. The frame's configuration
 * is fixed at its start (IspFrameCfg), as the device latches it.
 */
typedef struct IspFrameCfg {
    /* The frame's register values, shadowed modules' as in effect */
    uint32_t r[ESP32P4_ISP_NREGS];
    /* Input size */
    uint32_t w;
    uint32_t h;
    /* The LSC LUTs (Gb/B and R/Gr words) and the DPC LUT */
    const uint32_t *lsc_gb_b;
    const uint32_t *lsc_r_gr;
    uint32_t *dpc_lut;
    /* The gamma curves in effect: X powers and Y points, R, G, B */
    const uint8_t (*gamma_x)[16];
    const uint8_t (*gamma_y)[16];
    /* The AF edge threshold for this frame */
    uint32_t af_threshold;
} IspFrameCfg;

typedef struct IspStats {
    /* BLC_MEAN's channels */
    uint8_t blc_mean[4];
    /* AE: each sub-window's mean */
    uint8_t ae_mean[25];
    /* AF: windows A, B, C */
    uint32_t af_sum[3];
    uint32_t af_lum[3];
    /* AF: the edge threshold the frame's gradients give */
    uint32_t af_gen_threshold;
    /* AWB algorithm 0: the window's white patches, and the sub-windows' */
    uint32_t awb_cnt;
    uint32_t awb_acc[3];
    uint32_t awb_sub[25][4];
    /* HIST */
    uint32_t hist[16];
    /* SHARP_GRADIENT_MAX */
    uint8_t sharp_max;
    /* DPC calibration: coordinates found (DPC_DEADPIX_CNT) */
    uint32_t dpc_dead;
    /* CROP_ERR_ST's bits for the frame's window, 0 if valid */
    uint32_t crop_err;
    /* The frame's output rows, after CROP */
    uint32_t out_rows;
    /* The output row's width in pixels, after CROP */
    uint32_t out_w;
} IspStats;

typedef struct IspPipe IspPipe;

/*
 * A completed row: @data, @len bytes in the output layout, or NULL for a
 * row CROP leaves out (or with DPC calibration's output off).
 */
typedef void IspEmitFn(void *opaque, const uint8_t *data, uint32_t len);

IspPipe *isp_pipe_new(const IspFrameCfg *cfg, IspEmitFn *emit, void *opaque);
void isp_pipe_row(IspPipe *p, const uint8_t *row);
void isp_pipe_finish(IspPipe *p);
const IspStats *isp_pipe_stats(const IspPipe *p);
void isp_pipe_free(IspPipe *p);

/* Rows of input the pipeline holds before its first output row */
uint32_t isp_pipe_latency_rows(const IspFrameCfg *cfg);

#endif
