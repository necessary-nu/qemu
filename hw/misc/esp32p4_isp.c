/*
 * ESP32-P4 image signal processor (ISP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 38 (ISP), chapter 7 (VDMA's handshaking); ESP-IDF's isp_reg.h and
 * isp_struct.h for v3, the hal/isp_ll.h and isp_hal.c that drive them,
 * esp_driver_isp (isp_core.c, isp_dma.c and each module's driver) and
 * its isp.rst, and the silicon captures in esp_driver_isp's test app.
 *
 * The ISP at 0x500A1000, behind CSI_BRG_SYS_CLK (TRM 38.7.1), reset by
 * RST_EN_ISP and RST_EN_CSI_BRG, its pipeline running at ISP_CLK.
 *
 *   - Inputs (ISP_IN_SRC). 0, MIPI CSI: CSI_CM's output, a frame at a
 *     time between its frame start and end packets, each long packet a
 *     row. 1, DVP: the board's camera on the CAM_* inputs (the machine's
 *     dvp-camera), with CAM_EN and out of CAM_RESET, a frame from VSYNC's
 *     active edge (by CAM_VSYNC_INV) to its VNUM-th row, a row each DE;
 *     RAW samples on D[n-1:0] by CAM_DATA_TYPE, or with CAM_2BYTE_MODE
 *     two RAW8 pixels a sample, the earlier in D[7:0] unless
 *     CAM_DATA_ORDER. CAM_CONF takes effect at CAM_UPDATE_REG, which
 *     clears two ISP_CLK cycles later; a sensor signal whose pad the GPIO
 *     matrix does not route to its CAM_* input does not reach the ISP
 *     (the board's pads, include/hw/misc/esp32p4_image_source.h): without
 *     PCLK, VSYNC or DE nothing is taken, a missing data line reads 0.
 *     2, VDMA: DMA_EN starts a frame of DMA_RAW_NUM_TOTAL 64-bit words
 *     (taken at DMA_RAW_NUM_TOTAL_SET); the ISP is the flow controller on
 *     VDMA's handshaking interface 2, asking for DMA_BURST_LEN words while
 *     the frame has them, then single words, the last request marked,
 *     and after each acknowledge waits while the ISP takes the words in
 *     (a pixel an ISP_CLK cycle) and DMA_INTERVAL cycles (its set bits).
 *     DMA_DATA_TYPE, DMA_BURST_LEN and DMA_INTERVAL take effect at
 *     DMA_UPDATE_REG, which clears two cycles later. VDMA writes the words
 *     at the CSI bridge's memory port, as ESP-IDF's isp_dma.c does.
 *     Samples are RAW8, RAW10 or RAW12 (ISP_DATA_TYPE; CSI-2's packing for
 *     MIPI and VDMA), reduced to their top 8 bits: the pipeline is 8-bit,
 *     as its thresholds, BLC's stretch to 0-255 and AWB's 765 are.
 *   - ISP_Tail and the bypass. While the ISP is enabled the tail takes
 *     the pipeline's output; MIPI CSI's data reach the bridge unprocessed
 *     (each 32-bit word's bytes reversed with BYTE_ENDIAN_ORDER) while it
 *     is disabled, and also while its input is MIPI CSI with
 *     MIPI_DATA_EN clear: the reset state (ISP_EN set), in which ESP-IDF's
 *     CSI driver captures without an ISP processor. With another input
 *     the MIPI data are dropped. HEADER_IDI_FRAME_INT rises at each MIPI
 *     frame's end, TAIL_IDI_FRAME_INT at each frame the tail gives the
 *     bridge.
 *   - Checks. ISP_HVNUM_SETTING_ERR_INT for an odd width or height, or a
 *     RAW10 width not a multiple of 4; ISP_DATA_TYPE_SETTING_ERR_INT for
 *     ISP_DATA_TYPE 3 (taken as RAW8) or a RAW row of another RAW type,
 *     unpacked as the setting says; ISP_DATA_TYPE_ERR_INT for a non-RAW
 *     row, dropped; ISP_MIPI_HNUM_UNMATCH_INT for a row of other than
 *     HNUM pixels, padded with 0 or cut. A frame that ends early is
 *     completed with rows of 0 (logged), rows past VNUM dropped. The line
 *     start and end packets (HSYNC_START_EXIST, HSYNC_END_EXIST) do not
 *     reach the ISP (the image interface's), which takes its rows from
 *     the long packets, so those settings are kept as written.
 *   - The pipeline, in esp32p4_isp_pipe.c: BLC, DPC, BF, LSC, demosaic,
 *     MEDIAN, WBG, CCM, gamma, RGB2YUV, SHARP, COLOR, YUV_Limit or
 *     YUV2RGB, CROP and the tail's packing (TRM table 38.5-2), with AE,
 *     AF, AWB and HIST at their sampling points; each stage's arithmetic
 *     and interpretation is stated there.
 *   - Configuration. The frame's registers are taken at its start: the
 *     shadowed modules' (BLC, DPC, BF, WBG, CCM, SHARP, COLOR; TRM
 *     38.5.5) by SHADOW_UPDATE_SEL: 0 as they are written, 1 as they are
 *     at each frame start, 2 only a module whose *_UPDATE bit is set at
 *     a frame start, the bit clearing then; the others' (the enables, the
 *     frame's size, demosaic, LSC, the statistics' windows, CROP) as they
 *     are. GAMMA_UPDATE takes the curves' registers into effect 16 ISP_CLK
 *     cycles after it is set and clears; the X intervals of a channel not
 *     adding up to 256 raise ISP_GAMMA_XCOORD_ERR_INT then (the TRM's
 *     condition; ESP-IDF notes a silicon defect, DIG-820, that raises it
 *     at every update, and ignores it). The LUTs are memories: LUT_CMD
 *     reads one into LUT_RDATA or writes LUT_WDATA to it at once, the AWB
 *     LUT read only; the device's reset leaves them.
 *   - Timing. The pipeline takes a pixel an ISP_CLK cycle: a row leaves
 *     HNUM + HSYNC_CNT cycles after the later of its last input row's
 *     arrival and the previous row's leaving, its last input row being the
 *     one its modules' windows reach down to (up to 10 rows below with
 *     every windowed module on; the frame's last, which the bottom rows'
 *     windows mirror). The bridge holding its output back (its
 *     buffer past CSI_BUF_AFULL_THRD) delays the rows; when they lag their
 *     input by more than 4 rows, ISP_Header's buffer is full
 *     (ISP_BUF_FULL_INT) and the row coming in is lost (as 0). A row
 *     coming in while ISP_CLK is stopped overflows the input's
 *     asynchronous FIFO (ISP_ASYNC_FIFO_OVF_INT) and is lost likewise.
 *     The frame's end goes to the bridge after its last row; the
 *     modules' frame interrupts (each enabled module's), ISP_FRAME_INT,
 *     and the statistics come then.
 *   - Statistics. AE (with AE_EN) measures the frame after an AE_UPDATE,
 *     and with a monitor threshold set every AE_MONITOR_PERIOD-th frame,
 *     whose 25 means' average outside AE_MONITOR_TL..TH raises
 *     ISP_AE_MONITOR_INT; each measured frame's means go to
 *     AE_BLOCK_MEAN with ISP_AE_FRAME_DONE_INT. AF (with AF_EN) measures
 *     every frame with AF_AUTO_UPDATE, else the frame after an
 *     AF_MANUAL_UPDATE, results and ISP_AF_FDONE_INT then, each result the
 *     new baseline; without auto update and with AF_ENV_PERIOD, every
 *     period-th frame's sums against the baseline: a change of the
 *     windows' total SUM or LUM above its threshold (AF_ENV_USER_*, or
 *     with that 0 the baseline times AF_ENV_THRESHOLD / 16) twice running
 *     raises ISP_AF_ENV_INT. With AF_THRESHOLD 0 the edge threshold is the
 *     one the last frame's gradients gave (AF_GEN_THRESHOLD): the least
 *     that at most AF_THPIXNUM of its window pixels' gradients reach,
 *     within AF_GEN_THRESHOLD_MIN..MAX. AWB measures one frame for each
 *     AWB_EN rising edge (ESP-IDF's trigger, writing 0 then 1): the
 *     window's results to AWB0_*, the sub-windows' to the AWB LUT (four
 *     words each: count, R, G, B), ISP_AWB_FDONE_INT; AWB_MODE's algorithm
 *     1 is not modelled (logged). HIST (with HIST_EN) measures every
 *     frame, ISP_HIST_FDONE_INT. BLC_MEAN and SHARP_GRADIENT_MAX are each
 *     enabled module's for the frame; DPC's calibration writes the LUT as
 *     it finds the samples, DPC_DEADPIX_CNT and ISP_DPC_CHECK_DONE_INT at
 *     the frame's end. CROP's window is checked at the frame's start
 *     (CROP_ERR_ST, sticky until CROP_SFT_RST; ISP_CROP_ERR_INT), and a
 *     wrong one leaves the frame uncropped.
 *   - Not modelled: the clock force-on bits, the memories' power
 *     controls, the tail rows' rate control (*_TAIL_PIXEN_PULSE_*:
 *     rows go at the pipeline's rate), and a frame in progress across
 *     migration (dropped).
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/gpio/esp32p4_gpio_sig.h"
#include "migration/vmstate.h"
#include "esp32p4_isp_int.h"

typedef struct IspReg {
    hwaddr off;
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
} IspReg;

static const IspReg isp_regs[] = {
    { A_VER_DATE,              0x20210608, 0xffffffff, 0x00000000 },
    { A_CLK_EN,                0x00000000, 0x001fffff, 0x00000000 },
    { A_CNTL,                  0x40002442, 0xff0fffff, 0x00000000 },
    { A_HSYNC_CNT,             0x00000007, 0x000000ff, 0x00000000 },
    { A_FRAME_CFG,             0x601e01e0, 0x78ffffff, 0x00000000 },
    { A_CCM_COEF0,             0x00000100, 0x03ffffff, 0x00000000 },
    { A_CCM_COEF1,             0x00000000, 0x03ffffff, 0x00000000 },
    { A_CCM_COEF3,             0x00000100, 0x03ffffff, 0x00000000 },
    { A_CCM_COEF4,             0x00000000, 0x03ffffff, 0x00000000 },
    { A_CCM_COEF5,             0x00000100, 0x00001fff, 0x00000000 },
    { A_BF_MATRIX_CTRL,        0x00000000, 0x01ffffff, 0x00000000 },
    { A_BF_SIGMA,              0x00000002, 0x0000003f, 0x00000000 },
    { A_BF_GAU0,               0xffffffff, 0xffffffff, 0x00000000 },
    { A_BF_GAU1,               0x0000000f, 0x0000000f, 0x00000000 },
    { A_DPC_CTRL,              0x00000004, 0x0000003f, 0x00000000 },
    { A_DPC_CONF,              0x04103030, 0x0fffffff, 0x00000000 },
    { A_DPC_MATRIX_CTRL,       0x00000000, 0x01ffffff, 0x00000000 },
    { A_DPC_DEADPIX_CNT,       0x00000000, 0x00000000, 0x00000000 },
    { A_LUT_CMD,               0x00000000, 0x00000000, 0x0001ffff },
    { A_LUT_WDATA,             0x00000000, 0xffffffff, 0x00000000 },
    { A_LUT_RDATA,             0x00000000, 0x00000000, 0x00000000 },
    { A_LSC_TABLESIZE,         0x0000001f, 0x0000001f, 0x00000000 },
    { A_DEMOSAIC_MATRIX_CTRL,  0x00000000, 0x01ffffff, 0x00000000 },
    { A_DEMOSAIC_GRAD_RATIO,   0x00000010, 0x0000003f, 0x00000000 },
    { A_MEDIAN_MATRIX_CTRL,    0x00000000, 0x000001ff, 0x00000000 },
    { A_INT_RAW,               0x00000000, 0x00000000, 0x00000000 },
    { A_INT_ST,                0x00000000, 0x00000000, 0x00000000 },
    { A_INT_ENA,               0x000000c3, 0xffffffff, 0x00000000 },
    { A_INT_CLR,               0x00000000, 0x00000000, 0xffffffff },
    { A_GAMMA_CTRL,            0x0000000e, 0x0000000f, 0x00000000 },
    { A_GAMMA_RY1,             0x10203040, 0xffffffff, 0x00000000 },
    { A_GAMMA_RY2,             0x50607080, 0xffffffff, 0x00000000 },
    { A_GAMMA_RY3,             0x90a0b0c0, 0xffffffff, 0x00000000 },
    { A_GAMMA_RY4,             0xd0e0f0ff, 0xffffffff, 0x00000000 },
    { A_GAMMA_GY1,             0x10203040, 0xffffffff, 0x00000000 },
    { A_GAMMA_GY2,             0x50607080, 0xffffffff, 0x00000000 },
    { A_GAMMA_GY3,             0x90a0b0c0, 0xffffffff, 0x00000000 },
    { A_GAMMA_GY4,             0xd0e0f0ff, 0xffffffff, 0x00000000 },
    { A_GAMMA_BY1,             0x10203040, 0xffffffff, 0x00000000 },
    { A_GAMMA_BY2,             0x50607080, 0xffffffff, 0x00000000 },
    { A_GAMMA_BY3,             0x90a0b0c0, 0xffffffff, 0x00000000 },
    { A_GAMMA_BY4,             0xd0e0f0ff, 0xffffffff, 0x00000000 },
    { A_GAMMA_RX1,             0x00924924, 0x00ffffff, 0x00000000 },
    { A_GAMMA_RX2,             0x00924924, 0x00ffffff, 0x00000000 },
    { A_GAMMA_GX1,             0x00924924, 0x00ffffff, 0x00000000 },
    { A_GAMMA_GX2,             0x00924924, 0x00ffffff, 0x00000000 },
    { A_GAMMA_BX1,             0x00924924, 0x00ffffff, 0x00000000 },
    { A_GAMMA_BX2,             0x00924924, 0x00ffffff, 0x00000000 },
    { A_AE_CTRL,               0x00000000, 0x00000002, 0x00000001 },
    { A_AE_MONITOR,            0x00000000, 0x003fffff, 0x00000000 },
    { A_AE_BX,                 0x00000180, 0x003fffff, 0x00000000 },
    { A_AE_BY,                 0x000000d8, 0x003fffff, 0x00000000 },
    { A_AE_WINPIXNUM,          0x00014400, 0x0001ffff, 0x00000000 },
    { A_AE_WIN_RECIPROCAL,     0x00000000, 0x000fffff, 0x00000000 },
    { A_AE_BLOCK_MEAN_0,       0x00000000, 0x00000000, 0x00000000 },
    { A_AE_BLOCK_MEAN_1,       0x00000000, 0x00000000, 0x00000000 },
    { A_AE_BLOCK_MEAN_2,       0x00000000, 0x00000000, 0x00000000 },
    { A_AE_BLOCK_MEAN_3,       0x00000000, 0x00000000, 0x00000000 },
    { A_AE_BLOCK_MEAN_4,       0x00000000, 0x00000000, 0x00000000 },
    { A_AE_BLOCK_MEAN_5,       0x00000000, 0x00000000, 0x00000000 },
    { A_AE_BLOCK_MEAN_6,       0x00000000, 0x00000000, 0x00000000 },
    { A_SHARP_CTRL0,           0x00000000, 0xffffffff, 0x00000000 },
    { A_SHARP_FILTER0,         0x00000441, 0x00007fff, 0x00000000 },
    { A_SHARP_FILTER1,         0x00000882, 0x00007fff, 0x00000000 },
    { A_SHARP_FILTER2,         0x00000441, 0x00007fff, 0x00000000 },
    { A_SHARP_MATRIX_CTRL,     0x00000000, 0x01ffffff, 0x00000000 },
    { A_SHARP_CTRL1,           0x00000000, 0x00000000, 0x00000000 },
    { A_DMA_CNTL,              0x001080a8, 0xfffffffe, 0x00000001 },
    { A_DMA_RAW_DATA,          0x00000000, 0x003fffff, 0x80000000 },
    { A_CAM_CNTL,              0x00000004, 0x0000000f, 0x00000000 },
    { A_CAM_CONF,              0x000000a8, 0x0000ffff, 0x00000000 },
    { A_AF_CTRL0,              0x00000000, 0x00ff0f01, 0x00000010 },
    { A_AF_CTRL1,              0x00000000, 0x003fffff, 0x00000000 },
    { A_AF_GEN_TH_CTRL,        0x04400080, 0xffffffff, 0x00000000 },
    { A_AF_ENV_USER_TH_SUM,    0x00000000, 0xffffffff, 0x00000000 },
    { A_AF_ENV_USER_TH_LUM,    0x00000000, 0x3fffffff, 0x00000000 },
    { A_AF_THRESHOLD,          0x00000100, 0x0000ffff, 0x00000000 },
    { A_AF_HSCALE_A,           0x00010080, 0x0fff0fff, 0x00000000 },
    { A_AF_VSCALE_A,           0x00010080, 0x0fff0fff, 0x00000000 },
    { A_AF_HSCALE_B,           0x00010080, 0x0fff0fff, 0x00000000 },
    { A_AF_VSCALE_B,           0x00010080, 0x0fff0fff, 0x00000000 },
    { A_AF_HSCALE_C,           0x00010080, 0x0fff0fff, 0x00000000 },
    { A_AF_VSCALE_C,           0x00010080, 0x0fff0fff, 0x00000000 },
    { A_AF_SUM_A,              0x00000000, 0x00000000, 0x00000000 },
    { A_AF_SUM_B,              0x00000000, 0x00000000, 0x00000000 },
    { A_AF_SUM_C,              0x00000000, 0x00000000, 0x00000000 },
    { A_AF_LUM_A,              0x00000000, 0x00000000, 0x00000000 },
    { A_AF_LUM_B,              0x00000000, 0x00000000, 0x00000000 },
    { A_AF_LUM_C,              0x00000000, 0x00000000, 0x00000000 },
    { A_AWB_MODE,              0x00000003, 0x00000013, 0x00000000 },
    { A_AWB_HSCALE,            0x0000077f, 0x0fff0fff, 0x00000000 },
    { A_AWB_VSCALE,            0x00000437, 0x0fff0fff, 0x00000000 },
    { A_AWB_TH_LUM,            0x02fd0000, 0x03ff03ff, 0x00000000 },
    { A_AWB_TH_RG,             0x03ff0000, 0x03ff03ff, 0x00000000 },
    { A_AWB_TH_BG,             0x03ff0000, 0x03ff03ff, 0x00000000 },
    { A_AWB0_WHITE_CNT,        0x00000000, 0x00000000, 0x00000000 },
    { A_AWB0_ACC_R,            0x00000000, 0x00000000, 0x00000000 },
    { A_AWB0_ACC_G,            0x00000000, 0x00000000, 0x00000000 },
    { A_AWB0_ACC_B,            0x00000000, 0x00000000, 0x00000000 },
    { A_COLOR_CTRL,            0x00800080, 0xffffffff, 0x00000000 },
    { A_BLC_VALUE,             0x00000000, 0xffffffff, 0x00000000 },
    { A_BLC_CTRL0,             0x00000000, 0x0000000f, 0x00000000 },
    { A_BLC_CTRL1,             0x00000000, 0x7fffffff, 0x00000000 },
    { A_BLC_CTRL2,             0x00000000, 0xffffffff, 0x00000000 },
    { A_BLC_MEAN,              0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_MODE,             0x00000004, 0x00000007, 0x00000000 },
    { A_HIST_COEFF,            0x00555555, 0x00ffffff, 0x00000000 },
    { A_HIST_OFFS,             0x00000000, 0x0fff0fff, 0x00000000 },
    { A_HIST_SIZE,             0x00120020, 0x01ff01ff, 0x00000000 },
    { A_HIST_SEG0,             0x10203040, 0xffffffff, 0x00000000 },
    { A_HIST_SEG1,             0x50607080, 0xffffffff, 0x00000000 },
    { A_HIST_SEG2,             0x90a0b0c0, 0xffffffff, 0x00000000 },
    { A_HIST_SEG3,             0x00d0e0f0, 0x00ffffff, 0x00000000 },
    { A_HIST_WEIGHT0,          0x01010101, 0xffffffff, 0x00000000 },
    { A_HIST_WEIGHT1,          0x01010101, 0xffffffff, 0x00000000 },
    { A_HIST_WEIGHT2,          0x01010101, 0xffffffff, 0x00000000 },
    { A_HIST_WEIGHT3,          0xe8010101, 0xffffffff, 0x00000000 },
    { A_HIST_WEIGHT4,          0x01010101, 0xffffffff, 0x00000000 },
    { A_HIST_WEIGHT5,          0x01010101, 0xffffffff, 0x00000000 },
    { A_HIST_WEIGHT6,          0x00000001, 0x000000ff, 0x00000000 },
    { A_HIST_BIN0,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN1,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN2,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN3,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN4,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN5,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN6,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN7,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN8,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN9,             0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN10,            0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN11,            0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN12,            0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN13,            0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN14,            0x00000000, 0x00000000, 0x00000000 },
    { A_HIST_BIN15,            0x00000000, 0x00000000, 0x00000000 },
    { A_MEM_AUX_CTRL_0,        0x13201320, 0x3fff3fff, 0x00000000 },
    { A_MEM_AUX_CTRL_1,        0x13201320, 0x3fff3fff, 0x00000000 },
    { A_MEM_AUX_CTRL_2,        0x13201320, 0x3fff3fff, 0x00000000 },
    { A_MEM_AUX_CTRL_3,        0x13201320, 0x3fff3fff, 0x00000000 },
    { A_MEM_AUX_CTRL_4,        0x00001320, 0x00003fff, 0x00000000 },
    { A_YUV_FORMAT,            0x00000000, 0x00000003, 0x00000000 },
    { A_RDN_ECO_CS,            0x00000000, 0x00000001, 0x00000000 },
    { A_RDN_ECO_LOW,           0x00000000, 0xffffffff, 0x00000000 },
    { A_RDN_ECO_HIGH,          0xffffffff, 0xffffffff, 0x00000000 },
    { A_CROP_CTRL,             0x00000000, 0x00000000, 0x00000001 },
    { A_CROP_Y_CAPTURE,        0x00000000, 0x00ffffff, 0x00000000 },
    { A_CROP_X_CAPTURE,        0x00000000, 0x00ffffff, 0x00000000 },
    { A_CROP_ERR_ST,           0x00000000, 0x00000000, 0x00000000 },
    { A_WBG_COEF_R,            0x00000100, 0x00000fff, 0x00000000 },
    { A_WBG_COEF_G,            0x00000100, 0x00000fff, 0x00000000 },
    { A_WBG_COEF_B,            0x00000100, 0x00000fff, 0x00000000 },
    { A_COLOR_HUE_CTRL,        0x00000000, 0x00000001, 0x00000000 },
    { A_AWB_BX,                0x00000000, 0x00ffffff, 0x00000000 },
    { A_AWB_BY,                0x00000000, 0x00ffffff, 0x00000000 },
    { A_STATE,                 0x00000000, 0x00000000, 0x00000000 },
    { A_SHADOW_REG_CTRL,       0x40000000, 0xc00000df, 0x00000000 },
};

/* The shadowed modules: SHADOW_REG_CTRL's *_UPDATE bit, and registers */
static const struct {
    uint32_t bit;
    hwaddr regs[5];
    int n;
} shadow_groups[] = {
    { BIT(0), { A_BLC_VALUE, A_BLC_CTRL0, A_BLC_CTRL1, A_BLC_CTRL2 }, 4 },
    { BIT(1), { A_DPC_CTRL, A_DPC_CONF, A_DPC_MATRIX_CTRL }, 3 },
    { BIT(2), { A_BF_MATRIX_CTRL, A_BF_SIGMA, A_BF_GAU0, A_BF_GAU1 }, 4 },
    { BIT(3), { A_WBG_COEF_R, A_WBG_COEF_G, A_WBG_COEF_B }, 3 },
    { BIT(4), { A_CCM_COEF0, A_CCM_COEF1, A_CCM_COEF3, A_CCM_COEF4,
                A_CCM_COEF5 }, 5 },
    { BIT(6), { A_SHARP_CTRL0, A_SHARP_FILTER0, A_SHARP_FILTER1,
                A_SHARP_FILTER2, A_SHARP_MATRIX_CTRL }, 5 },
    { BIT(7), { A_COLOR_CTRL, A_COLOR_HUE_CTRL }, 2 },
};

/* Rows the pipeline may lag its input by before the header's buffer fills */
#define HEADER_ROWS         4
/* ISP_CLK cycles GAMMA_UPDATE, DMA_UPDATE_REG and CAM_UPDATE_REG take */
#define GAMMA_UPDATE_CYCLES 16
#define REG_UPDATE_CYCLES   2

enum { EV_FS, EV_LINE, EV_FE };

typedef struct IspEvent {
    int kind;
    int64_t due;
    int64_t start;
    uint8_t vc;
    uint16_t number;
    uint8_t dt;
    uint8_t *data;
    uint32_t len;
    bool crc_error;
    /* A frame the ISP passes on, which the bridge cannot hold back */
    bool pass;
    /* A processed frame's end: its statistics and what they take */
    IspStats *stats;
    uint32_t cntl;
    uint32_t dpc_ctrl;
    bool ae;
    bool ae_monitor;
    bool af;
    bool af_env;
    bool awb;
} IspEvent;

static uint32_t isp_reg(ESP32P4IspState *s, hwaddr a)
{
    return s->regs[R(a)];
}

static void isp_update_irq(ESP32P4IspState *s)
{
    qemu_set_irq(s->irq, !!(isp_reg(s, A_INT_RAW) & isp_reg(s, A_INT_ENA)));
}

static void isp_raise(ESP32P4IspState *s, uint32_t bits)
{
    s->regs[R(A_INT_RAW)] |= bits;
    isp_update_irq(s);
}

/* The time @n ISP_CLK cycles after @t, or INT64_MAX while it is stopped. */
static int64_t isp_after(ESP32P4IspState *s, int64_t t, uint64_t n)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->clk, t);

    if (!esp32p4_clk_edges_running(&e)) {
        return INT64_MAX;
    }
    return esp32p4_clk_edges_after(&e, t, n);
}

static bool isp_clk_on(ESP32P4IspState *s)
{
    return clock_is_enabled(s->clk);
}

/* ---- The output to the bridge, in time ---- */

static void ev_free(IspEvent *ev)
{
    g_free(ev->data);
    g_free(ev->stats);
    g_free(ev);
}

static void ev_arm(ESP32P4IspState *s)
{
    IspEvent *ev = g_queue_peek_head(s->events);

    if (ev) {
        timer_mod(s->ev_timer, ev->due);
    } else {
        timer_del(s->ev_timer);
    }
}

static void ev_push(ESP32P4IspState *s, IspEvent *ev)
{
    g_queue_push_tail(s->events, ev);
    if (g_queue_get_length(s->events) == 1) {
        ev_arm(s);
    }
}

static void isp_publish(ESP32P4IspState *s, IspEvent *ev);

static void ev_deliver(void *opaque)
{
    ESP32P4IspState *s = opaque;
    IspEvent *ev = g_queue_peek_head(s->events);
    const ESP32P4CsiIdiOps *tail = &esp32p4_csi_bridge_tail_ops;

    if (!ev) {
        return;
    }
    if (ev->kind == EV_LINE && !ev->pass && s->bridge &&
        esp32p4_csi_bridge_almost_full(s->bridge)) {
        int64_t d = MAX(ev->due - ev->start, 1);

        /* The bridge holds the rows back: every one waits a row longer. */
        for (GList *l = s->events->head; l; l = l->next) {
            IspEvent *e = l->data;

            e->due += d;
            e->start += d;
        }
        s->last_due += d;
        ev_arm(s);
        return;
    }
    g_queue_pop_head(s->events);
    if (s->bridge) {
        switch (ev->kind) {
        case EV_FS:
            tail->frame_start(s->bridge, ev->vc, ev->number, ev->due);
            break;
        case EV_LINE: {
            ESP32P4CsiIdiLine line = {
                .vc = ev->vc,
                .dt = ev->dt,
                .data = ev->data,
                .len = ev->len,
                .start_ns = ev->start,
                .end_ns = ev->due,
            };

            tail->line(s->bridge, &line);
            break;
        }
        case EV_FE:
            tail->frame_end(s->bridge, ev->vc, ev->number, ev->crc_error,
                            ev->due);
            break;
        }
    }
    if (ev->kind == EV_FE) {
        isp_raise(s, INT_TAIL_IDI_FRAME);
        if (ev->stats) {
            isp_publish(s, ev);
        }
    }
    ev_free(ev);
    ev_arm(s);
}

static void ev_timer_cb(void *opaque)
{
    ESP32P4IspState *s = opaque;
    IspEvent *ev = g_queue_peek_head(s->events);

    if (ev) {
        esp32_line_run_at(ev->due, ev_deliver, s);
    }
}

static void upd_run(void *opaque);

/* What fell due before an access at esp32_line_time(), in order. */
static void isp_sync(ESP32P4IspState *s)
{
    int64_t now = esp32_line_time();
    IspEvent *ev;

    upd_run(s);
    while ((ev = g_queue_peek_head(s->events)) && ev->due <= now) {
        timer_del(s->ev_timer);
        esp32_line_run_at(ev->due, ev_deliver, s);
    }
    ev_arm(s);
}

/* An event of a frame the ISP passes on: at once, or after those queued. */
static void pass_event(ESP32P4IspState *s, IspEvent *ev)
{
    int64_t now = esp32_line_time();

    ev->pass = true;
    if (g_queue_is_empty(s->events)) {
        ev->due = now;
        g_queue_push_tail(s->events, ev);
        ev_deliver(s);
        return;
    }
    ev->due = MAX(now, s->last_due);
    s->last_due = ev->due;
    ev_push(s, ev);
}

/* ---- Statistics' results ---- */

static void isp_publish(ESP32P4IspState *s, IspEvent *ev)
{
    const IspStats *st = ev->stats;
    uint32_t cntl = ev->cntl, ints = INT_FRAME;
    static const struct {
        uint32_t en;
        uint32_t intr;
    } frame_ints[] = {
        { CNTL_BLC_EN, INT_BLC_FRAME }, { CNTL_DPC_EN, INT_DPC_FRAME },
        { CNTL_BF_EN, INT_BF_FRAME }, { CNTL_LSC_EN, INT_LSC_FRAME },
        { CNTL_DEMOSAIC_EN, INT_DEMOSAIC_FRAME },
        { CNTL_MEDIAN_EN, INT_MEDIAN_FRAME }, { CNTL_CCM_EN, INT_CCM_FRAME },
        { CNTL_GAMMA_EN, INT_GAMMA_FRAME },
        { CNTL_RGB2YUV_EN, INT_RGB2YUV_FRAME },
        { CNTL_SHARP_EN, INT_SHARP_FRAME }, { CNTL_COLOR_EN, INT_COLOR_FRAME },
        { CNTL_YUV2RGB_EN, INT_YUV2RGB_FRAME },
        { CNTL_CROP_EN, INT_CROP_FRAME }, { CNTL_WBG_EN, INT_WBG_FRAME },
    };

    for (int i = 0; i < ARRAY_SIZE(frame_ints); i++) {
        if (cntl & frame_ints[i].en) {
            ints |= frame_ints[i].intr;
        }
    }
    if (cntl & CNTL_BLC_EN) {
        s->regs[R(A_BLC_MEAN)] = st->blc_mean[0] << 24 |
                                 st->blc_mean[1] << 16 |
                                 st->blc_mean[2] << 8 | st->blc_mean[3];
    }
    if (cntl & CNTL_SHARP_EN) {
        s->regs[R(A_SHARP_CTRL1)] = st->sharp_max;
    }
    if ((cntl & CNTL_DPC_EN) && (ev->dpc_ctrl & BIT(0))) {
        s->regs[R(A_DPC_DEADPIX_CNT)] = st->dpc_dead;
        ints |= INT_DPC_CHECK_DONE;
    }
    if (ev->ae) {
        uint32_t sum = 0;

        for (int i = 0; i < 25; i++) {
            uint32_t *r = &s->regs[R(A_AE_BLOCK_MEAN_0) + i / 4];

            *r = deposit32(*r, 24 - 8 * (i % 4), 8, st->ae_mean[i]);
            sum += st->ae_mean[i];
        }
        ints |= INT_AE_FRAME_DONE;
        if (ev->ae_monitor) {
            uint32_t mon = isp_reg(s, A_AE_MONITOR), avg = sum / 25;

            if (avg < extract32(mon, 0, 8) || avg > extract32(mon, 8, 8)) {
                ints |= INT_AE_MONITOR;
            }
        }
    }
    if (cntl & CNTL_AF_EN) {
        uint64_t tsum = 0, tlum = 0;

        s->af_gen = st->af_gen_threshold;
        s->regs[R(A_AF_THRESHOLD)] = deposit32(isp_reg(s, A_AF_THRESHOLD),
                                               16, 16, s->af_gen);
        for (int k = 0; k < 3; k++) {
            tsum += st->af_sum[k];
            tlum += st->af_lum[k];
        }
        if (ev->af) {
            for (int k = 0; k < 3; k++) {
                s->regs[R(A_AF_SUM_A) + k] = st->af_sum[k];
                s->regs[R(A_AF_LUM_A) + k] = st->af_lum[k];
            }
            s->af_base_sum = tsum;
            s->af_base_lum = tlum;
            s->af_over = 0;
            ints |= INT_AF_FDONE;
        } else if (ev->af_env) {
            uint32_t th = extract32(isp_reg(s, A_AF_CTRL0), 8, 4);
            uint64_t sum_th = isp_reg(s, A_AF_ENV_USER_TH_SUM);
            uint64_t lum_th = isp_reg(s, A_AF_ENV_USER_TH_LUM);
            uint64_t dsum = tsum > s->af_base_sum ? tsum - s->af_base_sum
                                                  : s->af_base_sum - tsum;
            uint64_t dlum = tlum > s->af_base_lum ? tlum - s->af_base_lum
                                                  : s->af_base_lum - tlum;

            if (!sum_th) {
                sum_th = (s->af_base_sum * th) >> 4;
            }
            if (!lum_th) {
                lum_th = (s->af_base_lum * th) >> 4;
            }
            if (dsum > sum_th || dlum > lum_th) {
                if (++s->af_over >= 2) {
                    ints |= INT_AF_ENV;
                    s->af_over = 0;
                }
            } else {
                s->af_over = 0;
            }
        }
    }
    if (ev->awb) {
        s->regs[R(A_AWB0_WHITE_CNT)] = st->awb_cnt;
        s->regs[R(A_AWB0_ACC_R)] = st->awb_acc[0];
        s->regs[R(A_AWB0_ACC_G)] = st->awb_acc[1];
        s->regs[R(A_AWB0_ACC_B)] = st->awb_acc[2];
        for (int i = 0; i < 25; i++) {
            for (int k = 0; k < 4; k++) {
                s->awb_lut[4 * i + k] = st->awb_sub[i][k];
            }
        }
        ints |= INT_AWB_FDONE;
    }
    if (cntl & CNTL_HIST_EN) {
        for (int k = 0; k < 16; k++) {
            s->regs[R(A_HIST_BIN0) + k] = st->hist[k];
        }
        ints |= INT_HIST_FDONE;
    }
    isp_raise(s, ints);
}

/* ---- Frames through the pipeline ---- */

static uint8_t out_dt(uint32_t type)
{
    static const uint8_t dt[8] = { 0x2a, 0x1e, 0x24, 0x18, 0x22, 0x24, 0x24,
                                   0x24 };

    return dt[type & 7];
}

static uint32_t row_cycles(ESP32P4IspState *s)
{
    return s->frame_w + extract32(isp_reg(s, A_HSYNC_CNT), 0, 8);
}

/* A row the pipeline completes, at the time the ISP has it out. */
static void isp_emit(void *opaque, const uint8_t *data, uint32_t len)
{
    ESP32P4IspState *s = opaque;
    int64_t start = MAX(esp32_line_time(), s->last_due);
    int64_t due = isp_after(s, start, row_cycles(s));
    IspEvent *ev;

    if (due == INT64_MAX) {
        due = start;
    }
    s->last_due = due;
    if (!data) {
        return;
    }
    ev = g_new0(IspEvent, 1);
    ev->kind = EV_LINE;
    ev->due = due;
    ev->start = start;
    ev->vc = s->frame_vc;
    ev->dt = out_dt(extract32(s->frame_cntl, 29, 3));
    ev->data = g_memdup2(data, len);
    ev->len = len;
    ev_push(s, ev);
}

static bool shadowed(hwaddr a)
{
    for (int g = 0; g < ARRAY_SIZE(shadow_groups); g++) {
        for (int i = 0; i < shadow_groups[g].n; i++) {
            if (shadow_groups[g].regs[i] == a) {
                return true;
            }
        }
    }
    return false;
}

/* At a frame's start: the shadowed modules' registers, as the TRM says. */
static void shadow_vsync(ESP32P4IspState *s)
{
    uint32_t ctrl = isp_reg(s, A_SHADOW_REG_CTRL);
    uint32_t mode = extract32(ctrl, 30, 2);

    for (int g = 0; g < ARRAY_SIZE(shadow_groups); g++) {
        if (mode == 0 || (mode == 2 && !(ctrl & shadow_groups[g].bit))) {
            continue;
        }
        for (int i = 0; i < shadow_groups[g].n; i++) {
            hwaddr a = shadow_groups[g].regs[i];

            s->act[R(a)] = s->regs[R(a)];
        }
    }
    if (mode == 2) {
        s->regs[R(A_SHADOW_REG_CTRL)] &= ~0xdfu;
    }
}

static void frame_finish(ESP32P4IspState *s, int64_t when);

static void frame_begin(ESP32P4IspState *s, uint8_t src, uint8_t vc,
                        uint16_t number, uint32_t bits)
{
    IspFrameCfg c = { 0 };
    uint32_t cntl = isp_reg(s, A_CNTL), fc = isp_reg(s, A_FRAME_CFG);
    uint32_t mon = isp_reg(s, A_AE_MONITOR), af0 = isp_reg(s, A_AF_CTRL0);
    int64_t now = esp32_line_time();
    const IspStats *st;
    IspEvent *ev;

    if (s->frame_on) {
        frame_finish(s, now);
    }
    shadow_vsync(s);
    memcpy(c.r, s->regs, sizeof(c.r));
    for (int g = 0; g < ARRAY_SIZE(shadow_groups); g++) {
        for (int i = 0; i < shadow_groups[g].n; i++) {
            hwaddr a = shadow_groups[g].regs[i];

            c.r[R(a)] = s->act[R(a)];
        }
    }
    c.w = extract32(fc, 12, 12) + 1;
    c.h = extract32(fc, 0, 12) + 1;
    c.lsc_gb_b = s->lsc_gb_b;
    c.lsc_r_gr = s->lsc_r_gr;
    c.dpc_lut = s->dpc_lut;
    c.gamma_x = s->gamma_x;
    c.gamma_y = s->gamma_y;
    c.af_threshold = extract32(isp_reg(s, A_AF_THRESHOLD), 0, 16);
    if (!c.af_threshold) {
        c.af_threshold = s->af_gen;
    }
    if ((c.w & 1) || (c.h & 1) || (bits == 10 && (c.w & 3))) {
        isp_raise(s, INT_HVNUM_SETTING_ERR);
    }

    /* The statistics this frame takes */
    s->ae_take = s->ae_monitor = false;
    if (cntl & CNTL_AE_EN) {
        s->ae_take = s->ae_armed;
        s->ae_armed = false;
        if ((extract32(mon, 0, 8) || extract32(mon, 8, 8)) &&
            extract32(mon, 16, 6) &&
            ++s->ae_frames >= extract32(mon, 16, 6)) {
            s->ae_frames = 0;
            s->ae_take = s->ae_monitor = true;
        }
    }
    s->af_take = s->af_env = false;
    if (cntl & CNTL_AF_EN) {
        s->af_take = (af0 & BIT(0)) || s->af_manual;
        s->af_manual = false;
        if (!(af0 & BIT(0)) && extract32(af0, 16, 8) &&
            ++s->af_frames >= extract32(af0, 16, 8)) {
            s->af_frames = 0;
            s->af_env = true;
        }
    }
    s->awb_take = false;
    if ((cntl & CNTL_AWB_EN) && s->awb_armed) {
        s->awb_take = true;
        s->awb_armed = false;
        if (!(isp_reg(s, A_AWB_MODE) & 1)) {
            qemu_log_mask(LOG_UNIMP, "%s: AWB_MODE %u: only algorithm 0 is "
                          "modelled\n", TYPE_ESP32P4_ISP,
                          extract32(isp_reg(s, A_AWB_MODE), 0, 2));
        }
    }

    s->frame_on = true;
    s->frame_cntl = cntl;
    s->frame_src = src;
    s->frame_vc = vc;
    s->frame_num = number;
    s->frame_w = c.w;
    s->frame_h = c.h;
    s->frame_bits = bits;
    s->rows_in = 0;
    s->pipe = isp_pipe_new(&c, isp_emit, s);
    st = isp_pipe_stats(s->pipe);
    if (st->crop_err) {
        s->regs[R(A_CROP_ERR_ST)] |= st->crop_err;
        isp_raise(s, INT_CROP_ERR);
    }
    ev = g_new0(IspEvent, 1);
    ev->kind = EV_FS;
    ev->vc = vc;
    ev->number = number;
    ev->due = MAX(now, s->last_due);
    s->last_due = ev->due;
    ev_push(s, ev);
}

/* An input row of 8-bit samples, @n of them, at esp32_line_time(). */
static void frame_row(ESP32P4IspState *s, const uint8_t *px, uint32_t n)
{
    int64_t now = esp32_line_time();
    g_autofree uint8_t *row = g_malloc0(s->frame_w);
    int64_t row_ns;

    if (!s->frame_on || s->rows_in >= s->frame_h) {
        return;
    }
    memcpy(row, px, MIN(n, s->frame_w));
    if (!isp_clk_on(s)) {
        isp_raise(s, INT_ASYNC_FIFO_OVF);
        memset(row, 0, s->frame_w);
    } else {
        row_ns = isp_after(s, now, row_cycles(s)) - now;
        if (s->last_due > now &&
            s->last_due - now > HEADER_ROWS * row_ns) {
            isp_raise(s, INT_BUF_FULL);
            memset(row, 0, s->frame_w);
        }
    }
    s->rows_in++;
    isp_pipe_row(s->pipe, row);
}

static void frame_finish(ESP32P4IspState *s, int64_t when)
{
    IspEvent *ev;

    if (!s->frame_on) {
        return;
    }
    if (s->rows_in < s->frame_h) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: frame of %u rows where VNUM "
                      "asks for %u: completed with 0\n", TYPE_ESP32P4_ISP,
                      s->rows_in, s->frame_h);
    }
    isp_pipe_finish(s->pipe);
    ev = g_new0(IspEvent, 1);
    ev->kind = EV_FE;
    ev->vc = s->frame_vc;
    ev->number = s->frame_num;
    ev->due = MAX(when, s->last_due);
    s->last_due = ev->due;
    ev->stats = g_memdup2(isp_pipe_stats(s->pipe), sizeof(IspStats));
    ev->cntl = s->frame_cntl;
    ev->dpc_ctrl = s->act[R(A_DPC_CTRL)];
    ev->ae = s->ae_take;
    ev->ae_monitor = s->ae_monitor;
    ev->af = s->af_take;
    ev->af_env = s->af_env;
    ev->awb = s->awb_take;
    ev_push(s, ev);
    isp_pipe_free(s->pipe);
    s->pipe = NULL;
    s->frame_on = false;
}

static void frame_drop(ESP32P4IspState *s)
{
    isp_pipe_free(s->pipe);
    s->pipe = NULL;
    s->frame_on = false;
}

/* Packed samples of @bits each, reduced to their top 8 bits. */
static uint32_t unpack8(const uint8_t *in, uint32_t len, uint32_t bits,
                        uint8_t *out, uint32_t max)
{
    uint32_t n = 0;

    if (bits == 10) {
        for (uint32_t i = 0; i + 5 <= len && n + 4 <= max; i += 5) {
            memcpy(out + n, in + i, 4);
            n += 4;
        }
    } else if (bits == 12) {
        for (uint32_t i = 0; i + 3 <= len && n + 2 <= max; i += 3) {
            out[n++] = in[i];
            out[n++] = in[i + 1];
        }
    } else {
        n = MIN(len, max);
        memcpy(out, in, n);
    }
    return n;
}

static uint32_t type_bits(uint32_t type)
{
    return type == 1 ? 10 : type == 2 ? 12 : 8;
}

/* ---- MIPI CSI: CSI_CM's output ---- */

static void isp_csi_frame_start(void *opaque, uint8_t vc, uint16_t number,
                                int64_t when_ns)
{
    ESP32P4IspState *s = opaque;
    uint32_t cntl = isp_reg(s, A_CNTL), src = extract32(cntl, 27, 2);
    uint32_t type = extract32(cntl, 25, 2);

    if (!(cntl & CNTL_ISP_EN) || (src == 0 && !(cntl & CNTL_MIPI_DATA_EN))) {
        IspEvent *ev = g_new0(IspEvent, 1);

        s->mipi_mode = 0;
        ev->kind = EV_FS;
        ev->vc = vc;
        ev->number = number;
        pass_event(s, ev);
        return;
    }
    if (src != 0) {
        s->mipi_mode = 2;
        return;
    }
    s->mipi_mode = 1;
    if (type == 3) {
        isp_raise(s, INT_DATA_TYPE_SETTING_ERR);
    }
    frame_begin(s, 0, vc, number, type_bits(type));
}

static void isp_csi_line(void *opaque, const ESP32P4CsiIdiLine *line)
{
    ESP32P4IspState *s = opaque;
    uint32_t type = extract32(isp_reg(s, A_CNTL), 25, 2);
    uint32_t bits = type_bits(type), n;
    g_autofree uint8_t *px = NULL;

    if (s->mipi_mode == 0) {
        IspEvent *ev = g_new0(IspEvent, 1);
        uint32_t len = ROUND_UP(line->len, 4);

        ev->kind = EV_LINE;
        ev->vc = line->vc;
        ev->dt = line->dt;
        ev->data = g_malloc0(len);
        memcpy(ev->data, line->data, line->len);
        if (isp_reg(s, A_CNTL) & CNTL_BYTE_ENDIAN_ORDER) {
            for (uint32_t i = 0; i < len; i += 4) {
                bswap32s((uint32_t *)(ev->data + i));
            }
            ev->len = len;
        } else {
            ev->len = line->len;
        }
        ev->start = line->start_ns;
        pass_event(s, ev);
        return;
    }
    if (s->mipi_mode != 1 || !s->frame_on || s->frame_src != 0) {
        return;
    }
    if (line->dt < 0x2a || line->dt > 0x2c) {
        isp_raise(s, INT_DATA_TYPE_ERR);
        return;
    }
    if (line->dt != 0x2a + (type == 3 ? 0 : type)) {
        isp_raise(s, INT_DATA_TYPE_SETTING_ERR);
    }
    px = g_malloc0(s->frame_w + 4);
    n = unpack8(line->data, line->len, bits, px, s->frame_w + 4);
    if (n != s->frame_w) {
        isp_raise(s, INT_MIPI_HNUM_UNMATCH);
    }
    frame_row(s, px, MIN(n, s->frame_w));
}

static void isp_csi_frame_end(void *opaque, uint8_t vc, uint16_t number,
                              bool crc_error, int64_t when_ns)
{
    ESP32P4IspState *s = opaque;

    isp_raise(s, INT_HEADER_IDI_FRAME);
    if (s->mipi_mode == 0) {
        IspEvent *ev = g_new0(IspEvent, 1);

        ev->kind = EV_FE;
        ev->vc = vc;
        ev->number = number;
        ev->crc_error = crc_error;
        pass_event(s, ev);
    } else if (s->mipi_mode == 1 && s->frame_on && s->frame_src == 0) {
        frame_finish(s, when_ns);
    }
}

static const ESP32P4CsiIdiOps esp32p4_isp_idi_ops = {
    .frame_start = isp_csi_frame_start,
    .line = isp_csi_line,
    .frame_end = isp_csi_frame_end,
};

/* ---- DVP ---- */

static bool dvp_capturing(ESP32P4IspState *s)
{
    uint32_t cntl = isp_reg(s, A_CNTL), cam = isp_reg(s, A_CAM_CNTL);

    return (cntl & CNTL_ISP_EN) && extract32(cntl, 27, 2) == 1 &&
           (cam & BIT(0)) && !(cam & BIT(2));
}

/* Whether the sensor's @sig reaches CAM input @in through the matrix. */
static bool dvp_reaches(ESP32P4IspState *s, ESP32P4DvpSignal sig,
                        unsigned in)
{
    int pad = esp32p4_image_source_dvp_pad(s->camera, sig);

    return pad < 0 || !s->gpio || esp32p4_gpio_sig_in_pad(s->gpio, in) == pad;
}

static void dvp_vsync(void *opaque, uint32_t frame, bool active,
                      int64_t when_ns)
{
    ESP32P4IspState *s = opaque;
    const ESP32P4ImageTiming *t = esp32p4_image_source_timing(s->camera);
    bool inv = extract32(s->cam_conf, 10, 1);
    uint32_t type = extract32(s->cam_conf, 2, 6);

    /* The ISP sees VSYNC rise with its polarity set to the sensor's. */
    if (active == (inv != t->vsync_active_low)) {
        return;
    }
    if (s->frame_on && s->frame_src == 1) {
        frame_finish(s, when_ns);
    }
    if (!dvp_capturing(s)) {
        return;
    }
    if (!dvp_reaches(s, ESP32P4_DVP_PCLK, ESP32P4_SIG_CAM_PCLK_PAD_IN) ||
        !dvp_reaches(s, ESP32P4_DVP_VSYNC, ESP32P4_SIG_CAM_V_SYNC_PAD_IN) ||
        !dvp_reaches(s, ESP32P4_DVP_DE, ESP32P4_SIG_CAM_H_ENABLE_PAD_IN)) {
        if (!s->dvp_warned) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: the camera's PCLK, VSYNC or "
                          "DE does not reach the ISP's CAM inputs\n",
                          TYPE_ESP32P4_ISP);
            s->dvp_warned = true;
        }
        return;
    }
    if (type < 0x2a || type > 0x2c) {
        isp_raise(s, INT_DATA_TYPE_ERR);
        return;
    }
    if (++s->out_frames == 0) {
        s->out_frames = 1;
    }
    frame_begin(s, 1, 0, s->out_frames, type_bits(type - 0x2a));
}

static void dvp_line(void *opaque, const ESP32P4DvpLine *line)
{
    ESP32P4IspState *s = opaque;
    uint32_t conf = s->cam_conf, bits = s->frame_bits, mask = 0, n = 0;
    bool two = conf & BIT(1), order = conf & BIT(0);
    g_autofree uint8_t *px = NULL;

    if (!s->frame_on || s->frame_src != 1) {
        return;
    }
    for (unsigned b = 0; b < (two ? 16 : bits); b++) {
        if (dvp_reaches(s, ESP32P4_DVP_D0 + b,
                        ESP32P4_SIG_CAM_DATA_IN_PAD_IN0 + b)) {
            mask |= BIT(b);
        }
    }
    px = g_malloc0(s->frame_w + 2);
    for (uint32_t k = 0; k < line->count && n < s->frame_w; k++) {
        uint32_t v = line->samples[k] & mask;

        if (two) {
            if (order) {
                v = bswap16(v);
            }
            px[n++] = v & 0xff;
            if (n < s->frame_w) {
                px[n++] = v >> 8;
            }
        } else {
            px[n++] = (v >> (bits - 8)) & 0xff;
        }
    }
    if (n != s->frame_w) {
        isp_raise(s, INT_MIPI_HNUM_UNMATCH);
    }
    frame_row(s, px, n);
    if (s->rows_in == s->frame_h) {
        frame_finish(s, esp32_line_time());
    }
}

static void dvp_stream(void *opaque, bool on, int64_t when_ns)
{
    ESP32P4IspState *s = opaque;

    if (!on && s->frame_on && s->frame_src == 1) {
        frame_finish(s, when_ns);
    }
}

static const ESP32P4ImageSinkOps dvp_ops = {
    .stream = dvp_stream,
    .vsync = dvp_vsync,
    .dvp_line = dvp_line,
};

/* ---- VDMA: memory to the ISP ---- */

static uint32_t dma_bits(ESP32P4IspState *s)
{
    uint32_t dt = extract32(s->dma_cntl, 2, 6);

    return dt == 0x2b ? 10 : dt == 0x2c ? 12 : 8;
}

static void dma_set_req(ESP32P4IspState *s, uint8_t req, bool last)
{
    qemu_set_irq(s->hs_last, last);
    qemu_set_irq(s->hs_req, req == 1);
    qemu_set_irq(s->hs_single, req == 2);
    s->dma_req = req;
}

static void dma_update_req(ESP32P4IspState *s)
{
    int64_t now = esp32_line_time();
    uint32_t burst = MAX(extract32(s->dma_cntl, 8, 12), 1);

    if (!s->dma_on || s->dma_req || !s->dma_left) {
        return;
    }
    if (now < s->dma_hold) {
        timer_mod(s->dma_timer, s->dma_hold);
        return;
    }
    if (s->dma_left >= burst) {
        dma_set_req(s, 1, s->dma_left == burst);
    } else {
        dma_set_req(s, 2, s->dma_left == 1);
    }
}

static void dma_timer_cb(void *opaque)
{
    ESP32P4IspState *s = opaque;

    esp32_line_run_at(s->dma_hold, (void (*)(void *))dma_update_req, s);
}

static void dma_hs_ack(void *opaque, int n, int level)
{
    ESP32P4IspState *s = opaque;
    int64_t now = esp32_line_time();
    uint32_t burst = MAX(extract32(s->dma_cntl, 8, 12), 1), words;
    uint32_t interval = ctpop32(extract32(s->dma_cntl, 20, 12));

    if (!level || !s->dma_req) {
        return;
    }
    words = s->dma_req == 1 ? burst : 1;
    dma_set_req(s, 0, false);
    s->dma_left -= MIN(words, s->dma_left);
    s->dma_hold = isp_after(s, now, words * 64 / dma_bits(s) + interval);
    if (!s->dma_left) {
        s->dma_on = false;
        if (s->frame_on && s->frame_src == 2) {
            frame_finish(s, now);
        }
        return;
    }
    dma_update_req(s);
}

static void isp_dma_write(void *opaque, const uint8_t *data, unsigned size)
{
    ESP32P4IspState *s = opaque;
    uint32_t bits = dma_bits(s), line = s->frame_w * bits / 8;

    if (!s->frame_on || s->frame_src != 2 || !s->dma_line) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: VDMA write with no frame coming "
                      "in from memory: dropped\n", TYPE_ESP32P4_ISP);
        return;
    }
    for (unsigned i = 0; i < size; i++) {
        s->dma_line[s->dma_fill++] = data[i];
        if (s->dma_fill == line) {
            g_autofree uint8_t *px = g_malloc0(s->frame_w + 4);
            uint32_t n = unpack8(s->dma_line, line, bits, px, s->frame_w + 4);

            frame_row(s, px, MIN(n, s->frame_w));
            s->dma_fill = 0;
        }
    }
    s->dma_words++;
}

static void dma_start(ESP32P4IspState *s)
{
    uint32_t cntl = isp_reg(s, A_CNTL), type = extract32(cntl, 25, 2);
    uint32_t bits = dma_bits(s);

    if (!(cntl & CNTL_ISP_EN) || extract32(cntl, 27, 2) != 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: DMA_EN with the ISP's input not "
                      "VDMA: ignored\n", TYPE_ESP32P4_ISP);
        return;
    }
    if (type == 3 || bits != type_bits(type)) {
        isp_raise(s, INT_DATA_TYPE_SETTING_ERR);
    }
    if (++s->out_frames == 0) {
        s->out_frames = 1;
    }
    frame_begin(s, 2, 0, s->out_frames, bits);
    g_free(s->dma_line);
    s->dma_line = g_malloc0(s->frame_w * 12 / 8 + 8);
    s->dma_fill = 0;
    s->dma_words = 0;
    s->dma_left = s->dma_total;
    s->dma_on = true;
    s->dma_hold = esp32_line_time();
    dma_set_req(s, 0, false);
    dma_update_req(s);
}

/* ---- Registers ---- */

static const IspReg *isp_reg_of(hwaddr a)
{
    if (a & 3 || a >= ESP32P4_ISP_NREGS * 4) {
        return NULL;
    }
    return &isp_regs[a / 4];
}

static void gamma_take(ESP32P4IspState *s)
{
    bool bad = false;

    for (int ch = 0; ch < 3; ch++) {
        uint32_t sum = 0;

        for (int n = 0; n < 16; n++) {
            uint32_t y = s->regs[R(A_GAMMA_RY1) + 4 * ch + n / 4];
            uint32_t x = s->regs[R(A_GAMMA_RX1) + 2 * ch + n / 8];

            s->gamma_y[ch][n] = extract32(y, 24 - 8 * (n % 4), 8);
            s->gamma_x[ch][n] = extract32(x, 21 - 3 * (n % 8), 3);
            sum += 1u << s->gamma_x[ch][n];
        }
        bad |= sum != 256;
    }
    if (bad) {
        isp_raise(s, INT_GAMMA_XCOORD_ERR);
    }
}

static void upd_arm(ESP32P4IspState *s)
{
    int64_t next = MIN(s->gamma_due, MIN(s->dma_upd_due, s->cam_upd_due));

    if (next == INT64_MAX) {
        timer_del(s->upd_timer);
    } else {
        timer_mod(s->upd_timer, next);
    }
}

/* Self-clearing updates that fell due by esp32_line_time(). */
static void upd_run(void *opaque)
{
    ESP32P4IspState *s = opaque;
    int64_t now = esp32_line_time();

    if (s->gamma_due <= now) {
        s->gamma_due = INT64_MAX;
        gamma_take(s);
        s->regs[R(A_GAMMA_CTRL)] &= ~BIT(0);
    }
    if (s->dma_upd_due <= now) {
        s->dma_upd_due = INT64_MAX;
        s->dma_cntl = isp_reg(s, A_DMA_CNTL);
        s->regs[R(A_DMA_CNTL)] &= ~BIT(1);
    }
    if (s->cam_upd_due <= now) {
        s->cam_upd_due = INT64_MAX;
        s->cam_conf = isp_reg(s, A_CAM_CONF);
        s->regs[R(A_CAM_CNTL)] &= ~BIT(1);
    }
    upd_arm(s);
}

static void upd_timer_cb(void *opaque)
{
    ESP32P4IspState *s = opaque;
    int64_t next = MIN(s->gamma_due, MIN(s->dma_upd_due, s->cam_upd_due));

    if (next != INT64_MAX) {
        esp32_line_run_at(next, upd_run, s);
    }
}

static void lut_cmd(ESP32P4IspState *s, uint32_t v)
{
    uint32_t num = extract32(v, 12, 4), addr = extract32(v, 0, 12);
    bool wr = v & BIT(16);
    uint32_t wdata = isp_reg(s, A_LUT_WDATA), *rdata = &s->regs[R(A_LUT_RDATA)];

    switch (num) {
    case LUT_LSC: {
        uint32_t idx = addr & 0x3ff, half = extract32(addr, 10, 2);
        uint32_t *lut = half == 0 ? s->lsc_gb_b : half == 1 ? s->lsc_r_gr
                                                          : NULL;

        if (!lut || idx >= ISP_LSC_LUT_DEPTH) {
            break;
        }
        if (wr) {
            lut[idx] = wdata & 0xfffff;
        } else {
            *rdata = lut[idx];
        }
        return;
    }
    case LUT_DPC:
        if (addr >= ISP_DPC_LUT_DEPTH) {
            break;
        }
        if (wr) {
            s->dpc_lut[addr] = wdata;
        } else {
            *rdata = s->dpc_lut[addr];
        }
        return;
    case LUT_AWB:
        if (addr >= ISP_AWB_LUT_DEPTH || wr) {
            break;
        }
        *rdata = s->awb_lut[addr];
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: LUT_CMD 0x%05x: no such LUT entry, "
                  "or a write to the AWB LUT\n", TYPE_ESP32P4_ISP, v);
}

static uint64_t isp_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4IspState *s = opaque;
    const IspReg *r = isp_reg_of(addr);

    isp_sync(s);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ISP, addr);
        return 0;
    }
    switch (addr) {
    case A_INT_ST:
        return isp_reg(s, A_INT_RAW) & isp_reg(s, A_INT_ENA);
    case A_STATE:
        return (g_queue_is_empty(s->events) ? 0 : BIT(0)) |
               (s->frame_on ? BIT(1) : 0);
    default:
        return s->regs[addr / 4] & ~r->wt;
    }
}

static void isp_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4IspState *s = opaque;
    const IspReg *r = isp_reg_of(addr);
    int64_t now = esp32_line_time();
    uint32_t v = value, old;

    isp_sync(s);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ISP, addr);
        return;
    }
    if (s->held) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write while held in reset: "
                      "dropped\n", TYPE_ESP32P4_ISP);
        return;
    }
    old = s->regs[addr / 4];
    s->regs[addr / 4] = (old & ~r->rw) | (v & r->rw);
    if (!extract32(isp_reg(s, A_SHADOW_REG_CTRL), 30, 2) && shadowed(addr)) {
        s->act[addr / 4] = s->regs[addr / 4];
    }
    switch (addr) {
    case A_INT_CLR:
        s->regs[R(A_INT_RAW)] &= ~v;
        break;
    case A_LUT_CMD:
        lut_cmd(s, v);
        break;
    case A_CNTL:
        if ((v & CNTL_AWB_EN) && !(old & CNTL_AWB_EN)) {
            s->awb_armed = true;
        }
        break;
    case A_GAMMA_CTRL:
        if ((v & BIT(0)) && s->gamma_due == INT64_MAX) {
            s->gamma_due = isp_after(s, now, GAMMA_UPDATE_CYCLES);
        }
        break;
    case A_AE_CTRL:
        if (v & BIT(0)) {
            s->ae_armed = true;
            s->ae_frames = 0;
        }
        break;
    case A_AF_CTRL0:
        if (v & BIT(4)) {
            s->af_manual = true;
        }
        break;
    case A_DMA_CNTL:
        if ((v & BIT(1)) && s->dma_upd_due == INT64_MAX) {
            s->dma_upd_due = isp_after(s, now, REG_UPDATE_CYCLES);
        }
        if (v & BIT(0)) {
            dma_start(s);
        }
        break;
    case A_DMA_RAW_DATA:
        if (v & BIT(31)) {
            s->dma_total = v & 0x3fffff;
        }
        break;
    case A_CAM_CNTL:
        if ((v & BIT(1)) && s->cam_upd_due == INT64_MAX) {
            s->cam_upd_due = isp_after(s, now, REG_UPDATE_CYCLES);
        }
        break;
    case A_CROP_CTRL:
        if (v & BIT(0)) {
            s->regs[R(A_CROP_ERR_ST)] = 0;
        }
        break;
    }
    upd_arm(s);
    isp_update_irq(s);
}

static const MemoryRegionOps isp_ops = {
    .read = isp_read,
    .write = isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---- Reset ---- */

static void isp_flush(ESP32P4IspState *s)
{
    IspEvent *ev;

    while ((ev = g_queue_pop_head(s->events))) {
        ev_free(ev);
    }
    timer_del(s->ev_timer);
    frame_drop(s);
    s->last_due = 0;
}

static void isp_reset_hold(Object *obj, ResetType type)
{
    ESP32P4IspState *s = ESP32P4_ISP(obj);

    for (int i = 0; i < ARRAY_SIZE(isp_regs); i++) {
        s->regs[i] = isp_regs[i].reset;
        s->act[i] = isp_regs[i].reset;
    }
    isp_flush(s);
    timer_del(s->upd_timer);
    timer_del(s->dma_timer);
    s->gamma_due = s->dma_upd_due = s->cam_upd_due = INT64_MAX;
    gamma_take(s);
    s->regs[R(A_INT_RAW)] = 0;
    s->cam_conf = isp_reg(s, A_CAM_CONF);
    s->dma_cntl = isp_reg(s, A_DMA_CNTL);
    s->dma_total = 0;
    s->ae_armed = s->ae_take = s->ae_monitor = false;
    s->ae_frames = 0;
    s->af_manual = s->af_take = s->af_env = false;
    s->af_frames = 0;
    s->af_gen = extract32(isp_reg(s, A_AF_GEN_TH_CTRL), 0, 16);
    s->af_base_sum = s->af_base_lum = 0;
    s->af_over = 0;
    s->awb_armed = s->awb_take = false;
    s->dma_on = false;
    s->dma_left = s->dma_words = s->dma_fill = 0;
    s->dma_req = 0;
    s->dma_hold = 0;
    s->mipi_mode = 0;
    s->out_frames = 0;
    s->dvp_warned = false;
}

static void isp_reset_exit(Object *obj, ResetType type)
{
    ESP32P4IspState *s = ESP32P4_ISP(obj);

    dma_set_req(s, 0, false);
    isp_update_irq(s);
}

static void isp_reset_in(void *opaque, int n, int level)
{
    ESP32P4IspState *s = opaque;
    bool held;

    s->reset_in[n] = level;
    held = s->reset_in[0] || s->reset_in[1];
    if (held != s->held) {
        device_cold_reset(DEVICE(s));
        s->held = held;
    }
}

/* ---- Device ---- */

static void isp_realize(DeviceState *dev, Error **errp)
{
    ESP32P4IspState *s = ESP32P4_ISP(dev);

    if (s->camera) {
        esp32p4_image_source_attach(s->camera, &dvp_ops, s);
    }
    if (s->bridge) {
        esp32p4_csi_bridge_attach_isp(s->bridge, &esp32p4_isp_idi_ops, s);
        esp32p4_csi_bridge_attach_isp_dma(s->bridge, isp_dma_write, s);
    }
}

static void isp_init(Object *obj)
{
    ESP32P4IspState *s = ESP32P4_ISP(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &isp_ops, s, "esp32p4.isp",
                          ESP32P4_ISP_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(dev, "clk", NULL, NULL, 0);
    qdev_init_gpio_out_named(dev, &s->hs_req, ESP32P4_ISP_HS_REQ, 1);
    qdev_init_gpio_out_named(dev, &s->hs_single, ESP32P4_ISP_HS_SINGLE, 1);
    qdev_init_gpio_out_named(dev, &s->hs_last, ESP32P4_ISP_HS_LAST, 1);
    qdev_init_gpio_in_named(dev, dma_hs_ack, ESP32P4_ISP_HS_ACK, 1);
    qdev_init_gpio_in_named(dev, isp_reset_in, ESP32P4_ISP_RESET, 2);
    s->events = g_queue_new();
    s->ev_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ev_timer_cb, s);
    s->upd_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, upd_timer_cb, s);
    s->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dma_timer_cb, s);
    s->gamma_due = s->dma_upd_due = s->cam_upd_due = INT64_MAX;
}

static void isp_finalize(Object *obj)
{
    ESP32P4IspState *s = ESP32P4_ISP(obj);

    isp_flush(s);
    g_queue_free(s->events);
    timer_free(s->ev_timer);
    timer_free(s->upd_timer);
    timer_free(s->dma_timer);
    g_free(s->dma_line);
}

static int isp_post_load(void *opaque, int version_id)
{
    ESP32P4IspState *s = opaque;

    /* A frame in progress is not carried across: the next one starts. */
    isp_flush(s);
    s->dma_on = false;
    dma_set_req(s, 0, false);
    upd_arm(s);
    return 0;
}

static const VMStateDescription vmstate_isp = {
    .name = TYPE_ESP32P4_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = isp_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4IspState, ESP32P4_ISP_NREGS),
        VMSTATE_UINT32_ARRAY(act, ESP32P4IspState, ESP32P4_ISP_NREGS),
        VMSTATE_UINT8_2DARRAY(gamma_x, ESP32P4IspState, 3, 16),
        VMSTATE_UINT8_2DARRAY(gamma_y, ESP32P4IspState, 3, 16),
        VMSTATE_UINT32_ARRAY(lsc_gb_b, ESP32P4IspState, 560),
        VMSTATE_UINT32_ARRAY(lsc_r_gr, ESP32P4IspState, 560),
        VMSTATE_UINT32_ARRAY(dpc_lut, ESP32P4IspState, 512),
        VMSTATE_UINT32_ARRAY(awb_lut, ESP32P4IspState, 100),
        VMSTATE_UINT32(cam_conf, ESP32P4IspState),
        VMSTATE_UINT32(dma_cntl, ESP32P4IspState),
        VMSTATE_UINT32(dma_total, ESP32P4IspState),
        VMSTATE_BOOL_ARRAY(reset_in, ESP32P4IspState, 2),
        VMSTATE_BOOL(held, ESP32P4IspState),
        VMSTATE_TIMER_PTR(upd_timer, ESP32P4IspState),
        VMSTATE_INT64(gamma_due, ESP32P4IspState),
        VMSTATE_INT64(dma_upd_due, ESP32P4IspState),
        VMSTATE_INT64(cam_upd_due, ESP32P4IspState),
        VMSTATE_UINT16(out_frames, ESP32P4IspState),
        VMSTATE_BOOL(ae_armed, ESP32P4IspState),
        VMSTATE_UINT32(ae_frames, ESP32P4IspState),
        VMSTATE_BOOL(af_manual, ESP32P4IspState),
        VMSTATE_UINT32(af_frames, ESP32P4IspState),
        VMSTATE_UINT32(af_gen, ESP32P4IspState),
        VMSTATE_UINT64(af_base_sum, ESP32P4IspState),
        VMSTATE_UINT64(af_base_lum, ESP32P4IspState),
        VMSTATE_UINT32(af_over, ESP32P4IspState),
        VMSTATE_BOOL(awb_armed, ESP32P4IspState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property isp_props[] = {
    DEFINE_PROP_LINK("bridge", ESP32P4IspState, bridge,
                     TYPE_ESP32P4_CSI_BRIDGE, ESP32P4CsiBridgeState *),
    DEFINE_PROP_LINK("camera", ESP32P4IspState, camera,
                     TYPE_ESP32P4_IMAGE_SOURCE, ESP32P4ImageSource *),
    DEFINE_PROP_LINK("gpio", ESP32P4IspState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
};

static void isp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = isp_realize;
    dc->vmsd = &vmstate_isp;
    device_class_set_props(dc, isp_props);
    rc->phases.hold = isp_reset_hold;
    rc->phases.exit = isp_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.isp] */
static const TypeInfo esp32p4_isp_types[] = {
    {
        .name = TYPE_ESP32P4_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4IspState),
        .instance_init = isp_init,
        .instance_finalize = isp_finalize,
        .class_init = isp_class_init,
    },
};

DEFINE_TYPES(esp32p4_isp_types)
