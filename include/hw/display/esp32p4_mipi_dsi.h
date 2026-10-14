/*
 * ESP32-P4 MIPI DSI: the DSI host, the DSI bridge, and a DSI panel
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_ESP32P4_MIPI_DSI_H
#define HW_DISPLAY_ESP32P4_MIPI_DSI_H

#include "hw/core/clock.h"
#include "hw/core/qdev.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/misc/esp32p4_mipi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_ESP32P4_DSI_HOST   "esp32p4-dsi-host"
#define TYPE_ESP32P4_DSI_BRIDGE "esp32p4-dsi-bridge"
#define TYPE_ESP32P4_DSI_BUS    "esp32p4-dsi-bus"
#define TYPE_ESP32P4_DSI_PANEL  "esp32p4-dsi-panel"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4DsiHostState, ESP32P4_DSI_HOST)
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4DsiBridgeState, ESP32P4_DSI_BRIDGE)
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4DsiPanelState, ESP32P4_DSI_PANEL)

#define ESP32P4_DSI_HOST_BASE   0x500a0000
#define ESP32P4_DSI_BRIDGE_BASE 0x500a0800
#define ESP32P4_DSI_REGS_SIZE   0x800
/* The bridge's FIFO, which VDMA writes (MIPI_DSI_BRG_MEM_BASE). */
#define ESP32P4_DSI_MEM_BASE    0x50105000
#define ESP32P4_DSI_MEM_SIZE    0x1000

/*
 * How the blocks connect
 * ======================
 *
 * VDMA (hw/dma/esp32p4_dw_gdma.c), through its handshaking interface 0,
 * writes the picture into the DSI bridge's FIFO at ESP32P4_DSI_MEM_BASE.
 * The bridge times DPI frames at DSI_DPI_CLK by its DPI_* registers,
 * converts each line's pixels to DPI words and gives them to the DSI
 * host with esp32p4_dsi_host_dpi_*(); the host packs them into DSI video
 * packets, or sends the commands software queues through its generic
 * interface, over its D-PHY to the panel on its bus ("dsi"), the
 * board's display: -device esp32p4-dsi-panel,width=W,height=H.
 *
 * The bridge's named GPIO outputs "hs-req", "hs-single" and "hs-last" are
 * its requests to VDMA's interface 0, and the input "hs-ack" VDMA's
 * acknowledge; its output "cfg-ref-clk-en" (DSI_BRG_HOST_CTRL's
 * DSI_CFG_REF_CLK_EN) gates the host's PHY configuration and PLL
 * reference clocks, the host's input of that name. The host's clock
 * output "lanebyteclk" is the D-PHY's TX_BYTE_CLK, which the reset and
 * clock control takes as "dsi-dphy-in" (timer group 0's calibration
 * clock source 12).
 */
#define ESP32P4_DSI_HS_REQ          "hs-req"
#define ESP32P4_DSI_HS_SINGLE       "hs-single"
#define ESP32P4_DSI_HS_LAST         "hs-last"
#define ESP32P4_DSI_HS_ACK          "hs-ack"
#define ESP32P4_DSI_CFG_REF_CLK_EN  "cfg-ref-clk-en"

/* A packet on the link, host to panel. */
typedef struct ESP32P4DsiPacket {
    uint8_t vc;
    uint8_t dt;
    /* A long packet's word count, or a short packet's two data bytes. */
    uint16_t wc;
    bool long_packet;
    const uint8_t *payload;
    /* Sent in high speed on @lanes data lanes, or in LPDT on lane 0. */
    bool hs;
    uint8_t lanes;
    /* The transmission it ends asks for a bus turnaround. */
    bool bta;
    /* Its last byte has gone. */
    int64_t end_ns;
} ESP32P4DsiPacket;

/* What a panel sends back after a bus turnaround. */
typedef struct ESP32P4DsiResponse {
    /* The Acknowledge trigger, and no packet. */
    bool ack;
    /* Nothing: the panel does not drive the lane. */
    bool none;
    /* The packets as they go on the lane: headers with ECC, payloads. */
    uint8_t *bytes;
    uint32_t len;
} ESP32P4DsiResponse;

/* The DSI bus: one panel. */
typedef struct ESP32P4DsiBus {
    BusState parent_obj;
} ESP32P4DsiBus;

#define ESP32P4_DSI_HOST_NREGS      (0x194 / 4)
#define ESP32P4_DSI_GEN_CMD_DEPTH   16
#define ESP32P4_DSI_GEN_PLD_DEPTH   128
#define ESP32P4_DSI_MAX_LINE        4096

struct ESP32P4DsiHostState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk_cfg;
    Clock *clk_ref;
    Clock *lanebyteclk;
    QEMUTimer *phy_timer;
    QEMUTimer *link_timer;
    QEMUTimer *vpg_timer;
    ESP32P4DsiBus bus;

    uint32_t regs[ESP32P4_DSI_HOST_NREGS];
    uint32_t int_st[2];

    /* The D-PHY: its test interface and registers, its state. */
    ESP32P4DphyTest test;
    uint8_t t10, t11, t12, t17, t18[2], t19, t22[2], t44;
    bool cfg_ref_en;
    uint32_t phy_state;
    int64_t phy_due_ns;
    ESP32P4ClkEdges ref_edges;
    ESP32P4ClkEdges byte_edges;
    ESP32P4ClkEdges byte_out;
    uint64_t lane_mbps_x1000;
    bool data_ulps;
    bool clk_ulps;
    int64_t ulps_due_ns;
    int64_t trigger_end_ns;
    bool warned_range;

    /* The generic interface's FIFOs. */
    uint32_t cmd_fifo[ESP32P4_DSI_GEN_CMD_DEPTH];
    uint32_t cmd_head;
    uint32_t cmd_num;
    uint32_t wpld[ESP32P4_DSI_GEN_PLD_DEPTH];
    uint32_t wpld_head;
    uint32_t wpld_num;
    uint32_t rpld[ESP32P4_DSI_GEN_PLD_DEPTH];
    uint32_t rpld_head;
    uint32_t rpld_num;

    /*
     * The link: the transmission under way, when it ends, the packet it
     * carries (its header and payload), and when the link is free again.
     */
    bool txn;
    uint32_t txn_hdr;
    uint32_t txn_kind;
    int64_t txn_end_ns;
    int64_t link_free_ns;
    uint8_t txn_pld[ESP32P4_DSI_GEN_PLD_DEPTH * 4];
    uint32_t txn_pld_len;
    bool rd_busy;

    /* Video mode: the host's line in the frame, and its shadows. */
    bool video;
    bool vpg;
    uint32_t vline;
    uint32_t vframe;
    int64_t vline_ns;
    int64_t vline_next_ns;
    uint64_t vpg_cycle;
    uint32_t act[ESP32P4_DSI_HOST_NREGS];
    bool shadow_pending;
    bool frame_warned;
    bool polarity_warned;
    uint32_t dpi_lcd_ctl;

    /* The panel's answer to a turnaround, until it is all in. */
    uint8_t ans[1024];
    uint32_t ans_len;
    uint32_t ans_lost;
    bool ans_none;
};

struct ESP32P4DsiBridgeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion mem;
    qemu_irq irq;
    qemu_irq hs_req;
    qemu_irq hs_single;
    qemu_irq hs_last;
    qemu_irq cfg_ref_clk_en;
    Clock *clk_bus;
    Clock *clk_dpi;
    QEMUTimer *timer;
    QEMUTimer *req_timer;
    ESP32P4DsiHostState *host;

    uint32_t regs[0x104 / 4];
    /* The DPI registers in effect, from DPI_CONFIG_UPDATE. */
    uint32_t dpi_act[8];

    /* The FIFO, 1024 x 64 bits. */
    uint8_t fifo[8192];
    uint32_t fifo_head;
    uint32_t fifo_num;
    bool overflow_warned;

    /*
     * The DPI timing generator: the DPI clock's cycles counted from the
     * frame's start, the line, and where in it the next event is.
     */
    bool running;
    ESP32P4ClkCount dpi_count;
    uint64_t cyc_base;
    uint64_t frame_cyc;
    uint32_t frame;
    uint32_t line;
    uint32_t step;
    uint64_t next_cyc;
    bool lag_logged;
    uint32_t vtotal, vsync, vbank, vdisp;
    uint32_t htotal, hsync, hbank, hdisp;

    /* The line being taken from the FIFO, and its DPI words. */
    uint8_t raw[ESP32P4_DSI_MAX_LINE * 3 + 16];
    uint32_t raw_len;
    uint32_t raw_need;
    uint32_t dpi[ESP32P4_DSI_MAX_LINE];
    bool underrun;
    /* YUV420: the last rows' U and V samples. */
    uint8_t chroma[2][ESP32P4_DSI_MAX_LINE / 2];

    /* VDMA requests: the one asserted, and the bridge's own counts. */
    bool req;
    bool req_single;
    bool req_last;
    int64_t req_hold_ns;
    uint32_t frame_words;
    uint32_t blk_words;
};

/* The board's panel: a command-level MIPI DSI display module. */
#define ESP32P4_DSI_PANEL_VREGS     256
#define ESP32P4_DSI_PANEL_VREG_LEN  16

struct ESP32P4DsiPanelState {
    DeviceState parent_obj;

    QemuConsole *con;
    bool powered;
    uint32_t width;
    uint32_t height;
    uint32_t lanes;
    uint32_t vc;
    bool display_on_reset;
    uint8_t id[3];

    /* The frame memory, 0x00RRGGBB per pixel, row by row. */
    uint32_t *fb;
    uint32_t fb_words;
    bool dirty;

    bool sleep_in;
    int64_t sleep_out_ns;
    bool display_on;
    bool inversion;
    bool idle;
    bool partial;
    bool shut_down;
    uint8_t madctl;
    uint8_t colmod;
    uint8_t brightness;
    uint8_t ctrl_display;
    uint8_t cabc;
    uint8_t tear;
    uint16_t sc, ec, sp, ep;
    uint32_t wx, wy;
    uint32_t rx, ry;
    uint16_t max_return;
    uint16_t errors;
    uint32_t error_count;
    uint32_t video_row;
    uint32_t video_col;
    bool in_frame;
    uint32_t frames;

    /* Manufacturer commands' last parameters, by command. */
    uint8_t vreg[ESP32P4_DSI_PANEL_VREGS][ESP32P4_DSI_PANEL_VREG_LEN];
    uint8_t vreg_len[ESP32P4_DSI_PANEL_VREGS];

    /* The command a read answers, as the last read request gave it. */
    bool read_pending;
    uint8_t read_dt;
    uint8_t read_cmd;
    uint8_t read_vc;
};

/*
 * The panel's side of the link, for the host: a packet, as its last byte
 * arrives; a trigger (bit n of @trigger, the PPI's TxTriggerEsc[n]); and a
 * bus turnaround, to which the panel answers in @r (bytes g_malloc'd), at
 * @when_ns.
 */
void esp32p4_dsi_panel_receive(ESP32P4DsiPanelState *p,
                               const ESP32P4DsiPacket *pkt);
void esp32p4_dsi_panel_trigger(ESP32P4DsiPanelState *p, unsigned trigger,
                               int64_t when_ns);
void esp32p4_dsi_panel_turnaround(ESP32P4DsiPanelState *p,
                                  ESP32P4DsiResponse *r, int64_t when_ns);

/*
 * The DPI, bridge to host: the start (HSYNC) of line @line of the frame,
 * 0 the first of VSYNC, ending at @next_ns; the line's pixels, DPI words
 * in bits 23:0, as the last of them has gone; and the DPI stopping.
 */
void esp32p4_dsi_host_dpi_hsync(ESP32P4DsiHostState *s, uint32_t line,
                                bool vsync, int64_t start_ns,
                                int64_t next_ns);
void esp32p4_dsi_host_dpi_pixels(ESP32P4DsiHostState *s, uint32_t line,
                                 const uint32_t *pixels, uint32_t count,
                                 int64_t active_ns, int64_t end_ns);
void esp32p4_dsi_host_dpi_stop(ESP32P4DsiHostState *s, int64_t when_ns);
/* The DPI's dpishutdn and dpicolorm signals, DSI_BRG_DPI_LCD_CTL. */
void esp32p4_dsi_host_dpi_ctl(ESP32P4DsiHostState *s, uint32_t lcd_ctl);

#endif
