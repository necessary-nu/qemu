/*
 * ESP32-P4 MIPI CSI: the CSI-2 host with its D-PHY, and the CSI bridge
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_MIPI_CSI_H
#define HW_MISC_ESP32P4_MIPI_CSI_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "hw/misc/esp32p4_image_source.h"
#include "hw/misc/esp32p4_mipi.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_CSI_HOST   "esp32p4-csi-host"
#define TYPE_ESP32P4_CSI_BRIDGE "esp32p4-csi-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4CsiHostState, ESP32P4_CSI_HOST)
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4CsiBridgeState, ESP32P4_CSI_BRIDGE)

#define ESP32P4_CSI_HOST_BASE   0x5009f000
#define ESP32P4_CSI_BRIDGE_BASE 0x5009f800
#define ESP32P4_CSI_REGS_SIZE   0x800
/* The bridge's buffer, which VDMA reads (MIPI_CSI_BRG_MEM_BASE). */
#define ESP32P4_CSI_MEM_BASE    0x50104000
#define ESP32P4_CSI_MEM_SIZE    0x1000

/*
 * The image interface
 * ===================
 *
 * The CSI-2 host gives what it receives to the chip as an image interface
 * (IDI, TRM 38.2: the 32-bit "Image Interface 32"), the CSI bridge takes
 * it into memory as "Image Interface 64", and the ISP sits between them
 * (TRM figure 38.1-1): CSI host -> CSI_CM (the colour conversion in the
 * bridge's register block, CSI_BRIG_HOST_CM_CTRL) -> ISP -> CSI_Bridge.
 * Each stage passes the stream on through these callbacks, each called at
 * its event's virtual time, which is also esp32_line_time():
 *
 *   frame_start  a frame start packet (FS) on virtual channel @vc, frame
 *                number @number;
 *   line         a long packet: its data type, its payload as received
 *                (an IDI word holds 4 bytes, the first in bits 7:0), and
 *                whether its checksum was wrong;
 *   frame_end    the frame end packet (FE), with whether any of the
 *                frame's packets had a checksum error.
 *
 * Short packets other than FS and FE (line start/end, generic) do not
 * reach the interface.
 *
 * The ISP's hook
 * --------------
 *
 * With no ISP the CSI_CM's output goes straight to the bridge, through
 * ISP_Tail's bypass (TRM 38.5.3: Image Interface 32 to 64, its byte order
 * ISP_BYTE_ENDIAN_ORDER's reset value, unchanged), as on a chip whose ISP
 * is disabled. An ISP model takes the CM's output instead with
 * esp32p4_csi_bridge_attach_isp(), and gives its own output (ISP_Tail's,
 * the processed frame or, with the ISP disabled, the CM's data with
 * ISP_BYTE_ENDIAN_ORDER applied) to the bridge through
 * esp32p4_csi_bridge_tail_ops with the bridge as opaque.
 * esp32p4_csi_bridge_almost_full() says whether the bridge's buffer has
 * passed CSI_BRIG_CSI_BUF_AFULL_THRD, for an ISP that holds its output
 * back. The CSI bridge's reset (HP_SYS_CLKRST_RST_EN_CSI_BRG) also resets
 * the ISP (TRM 38.7.1), which the machine wires.
 *
 * VDMA's writes to the bridge's memory port are the ISP's input from
 * memory (handshaking interface 2, as ESP-IDF's isp_dma.c addresses
 * them): an ISP takes them with esp32p4_csi_bridge_attach_isp_dma(); with
 * none they are logged and dropped.
 */
typedef struct ESP32P4CsiIdiLine {
    uint8_t vc;
    uint8_t dt;
    const uint8_t *data;
    uint32_t len;
    bool crc_error;
    /* The packet's first payload byte's arrival, and its end. */
    int64_t start_ns;
    int64_t end_ns;
} ESP32P4CsiIdiLine;

typedef struct ESP32P4CsiIdiOps {
    void (*frame_start)(void *opaque, uint8_t vc, uint16_t number,
                        int64_t when_ns);
    void (*line)(void *opaque, const ESP32P4CsiIdiLine *line);
    void (*frame_end)(void *opaque, uint8_t vc, uint16_t number,
                      bool crc_error, int64_t when_ns);
} ESP32P4CsiIdiOps;

/* The CSI_CM's input, which the machine gives the host as its sink. */
extern const ESP32P4CsiIdiOps esp32p4_csi_bridge_cm_ops;
/* The bridge's own input, ISP_Tail's output. */
extern const ESP32P4CsiIdiOps esp32p4_csi_bridge_tail_ops;

void esp32p4_csi_host_set_sink(ESP32P4CsiHostState *s,
                               const ESP32P4CsiIdiOps *ops, void *opaque);
void esp32p4_csi_bridge_attach_isp(ESP32P4CsiBridgeState *s,
                                   const ESP32P4CsiIdiOps *ops,
                                   void *opaque);
bool esp32p4_csi_bridge_almost_full(ESP32P4CsiBridgeState *s);

typedef void ESP32P4CsiDmaWriteFn(void *opaque, const uint8_t *data,
                                  unsigned size);
void esp32p4_csi_bridge_attach_isp_dma(ESP32P4CsiBridgeState *s,
                                       ESP32P4CsiDmaWriteFn *write,
                                       void *opaque);

/*
 * The bridge's named GPIO outputs "hs-req", "hs-single" and "hs-last" are
 * its requests to VDMA's handshaking interface 1, "hs-ack" VDMA's
 * acknowledge; "cfg-clk-en" and "enableclk" (CSI_BRIG_HOST_CTRL's
 * CSI_CFG_CLK_EN and CSI_ENABLECLK) gate the host's PHY configuration
 * clock and its clock lane, the host's inputs of those names.
 */
#define ESP32P4_CSI_HS_REQ          "hs-req"
#define ESP32P4_CSI_HS_SINGLE       "hs-single"
#define ESP32P4_CSI_HS_LAST         "hs-last"
#define ESP32P4_CSI_HS_ACK          "hs-ack"
#define ESP32P4_CSI_CFG_CLK_EN      "cfg-clk-en"
#define ESP32P4_CSI_ENABLECLK       "enableclk"

#define ESP32P4_CSI_HOST_NREGS      (0x30c / 4)
#define ESP32P4_CSI_INT_GROUPS      9
#define ESP32P4_CSI_VCS             16

struct ESP32P4CsiHostState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk_cfg;
    QEMUTimer *phy_timer;
    ESP32P4ImageSource *camera;
    ESP32P4ImageSource *attached;
    const ESP32P4CsiIdiOps *sink_ops;
    void *sink;

    uint32_t regs[ESP32P4_CSI_HOST_NREGS];
    /* Each error group's status, mask and force. */
    uint32_t st[ESP32P4_CSI_INT_GROUPS];
    uint32_t main_latch;

    ESP32P4DphyTest test;
    uint8_t t44;
    bool cfg_clk_en;
    bool enableclk;
    uint32_t phy_state;
    int64_t phy_due_ns;

    /* Each virtual channel's frame: started, its number, a CRC error. */
    bool in_frame[ESP32P4_CSI_VCS];
    uint16_t frame_no[ESP32P4_CSI_VCS];
    uint16_t last_no[ESP32P4_CSI_VCS];
    bool crc_in_frame[ESP32P4_CSI_VCS];
};

#define ESP32P4_CSI_BUF_WORDS       2048

struct ESP32P4CsiBridgeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion mem;
    qemu_irq irq;
    qemu_irq hs_req;
    qemu_irq hs_single;
    qemu_irq hs_last;
    qemu_irq cfg_clk_en;
    qemu_irq enableclk;
    Clock *clk_bus;
    QEMUTimer *req_timer;
    const ESP32P4CsiIdiOps *isp_ops;
    void *isp;
    ESP32P4CsiDmaWriteFn *isp_dma_write;
    void *isp_dma;

    uint32_t regs[0x50 / 4];

    /* The buffer of 64-bit words, as bytes. */
    uint8_t buf[ESP32P4_CSI_BUF_WORDS * 8];
    uint32_t buf_head;
    uint32_t buf_num;

    /* The frame coming in: whether it is taken, its lines and bytes. */
    bool capturing;
    bool frame_bad;
    uint32_t lines;
    uint32_t frame_bytes;
    uint32_t line_bytes;
    /* CM: the frame's line count, for YUV420's alternate rows. */
    uint32_t cm_row;
    /* The bytes up to the end of a frame that has ended, still buffered. */
    uint32_t end_bytes;
    bool end_pending;

    /* VDMA requests and the burst sizes in effect. */
    bool req;
    bool req_single;
    bool req_last;
    int64_t req_hold_ns;
    uint32_t burst_len;
    uint32_t blk_size;
    uint32_t bursts;
};

#endif
