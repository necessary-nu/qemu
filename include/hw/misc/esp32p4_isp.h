/*
 * ESP32-P4 image signal processor (ISP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ISP_H
#define HW_MISC_ESP32P4_ISP_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/gpio/esp32p4_gpio.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/misc/esp32p4_image_source.h"
#include "hw/misc/esp32p4_mipi_csi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ISP "esp32p4-isp"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4IspState, ESP32P4_ISP)

#define ESP32P4_ISP_BASE        0x500a1000
#define ESP32P4_ISP_REGS_SIZE   0x1000
#define ESP32P4_ISP_NREGS       (0x274 / 4)

/*
 * How the ISP connects
 * ====================
 *
 * The ISP (TRM chapter 38) sits between MIPI CSI's CSI_CM and the CSI
 * bridge (include/hw/misc/esp32p4_mipi_csi.h, "The ISP's hook"): the
 * machine attaches it to the bridge, which then gives it the CM's output,
 * and it gives its own to the bridge's tail. It takes two more inputs:
 * the board's DVP camera ("camera", the machine's dvp-camera, whose
 * signals reach it through the GPIO matrix's CAM_* inputs, "gpio"), and
 * memory through VDMA's handshaking interface 2, whose writes reach it
 * at the CSI bridge's memory port (esp32p4_csi_bridge_attach_isp_dma()),
 * as ESP-IDF's isp_dma.c addresses them.
 *
 * Its named GPIO outputs "hs-req", "hs-single" and "hs-last" are its
 * requests to VDMA's interface 2, the input "hs-ack" VDMA's acknowledge;
 * "reset" (two lines) its resets, HP_SYS_CLKRST's RST_EN_ISP and
 * RST_EN_CSI_BRG, either of which resets it (TRM 38.7.1). The clock input
 * "clk" is ISP_CLK, at which the pipeline runs; its registers are behind
 * CSI_BRG_SYS_CLK, which the machine gates.
 */
#define ESP32P4_ISP_HS_REQ      "hs-req"
#define ESP32P4_ISP_HS_SINGLE   "hs-single"
#define ESP32P4_ISP_HS_LAST     "hs-last"
#define ESP32P4_ISP_HS_ACK      "hs-ack"
#define ESP32P4_ISP_RESET       "reset"

struct ESP32P4IspState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq hs_req;
    qemu_irq hs_single;
    qemu_irq hs_last;
    Clock *clk;

    ESP32P4CsiBridgeState *bridge;
    ESP32P4ImageSource *camera;
    ESP32P4GpioState *gpio;

    uint32_t regs[ESP32P4_ISP_NREGS];
    /* The shadowed modules' registers in effect */
    uint32_t act[ESP32P4_ISP_NREGS];
    /* The gamma curves in effect, R, G and B: X powers, Y points */
    uint8_t gamma_x[3][16];
    uint8_t gamma_y[3][16];
    uint32_t lsc_gb_b[560];
    uint32_t lsc_r_gr[560];
    uint32_t dpc_lut[512];
    uint32_t awb_lut[100];
    /* CAM_CONF, and DMA_CNTL's fields, as last updated */
    uint32_t cam_conf;
    uint32_t dma_cntl;
    uint32_t dma_total;
    bool reset_in[2];
    bool held;

    QEMUTimer *upd_timer;
    int64_t gamma_due;
    int64_t dma_upd_due;
    int64_t cam_upd_due;

    /* A frame coming in, and its CNTL */
    bool frame_on;
    /* Its input: 0 MIPI CSI, 1 DVP, 2 VDMA */
    uint8_t frame_src;
    uint8_t frame_vc;
    uint16_t frame_num;
    uint32_t frame_w;
    uint32_t frame_h;
    uint32_t frame_bits;
    uint32_t rows_in;
    uint32_t frame_cntl;
    /* MIPI CSI's frame: 0 passed on, 1 processed, 2 dropped */
    uint8_t mipi_mode;
    uint16_t out_frames;
    void *pipe;
    /* The output's events, in order, and the last one's time */
    GQueue *events;
    QEMUTimer *ev_timer;
    int64_t last_due;

    /* Statistics' triggers and monitors */
    bool ae_armed;
    bool ae_take;
    bool ae_monitor;
    uint32_t ae_frames;
    bool af_manual;
    bool af_take;
    bool af_env;
    uint32_t af_frames;
    uint32_t af_gen;
    uint64_t af_base_sum;
    uint64_t af_base_lum;
    uint32_t af_over;
    bool awb_armed;
    bool awb_take;

    /* VDMA input */
    bool dma_on;
    uint32_t dma_left;
    uint32_t dma_words;
    uint8_t dma_req;
    int64_t dma_hold;
    QEMUTimer *dma_timer;
    uint8_t *dma_line;
    uint32_t dma_fill;

    /* DVP input */
    bool dvp_warned;
};

#endif
