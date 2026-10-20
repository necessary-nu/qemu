/*
 * ESP32-P4 MIPI DSI: the DSI host with its D-PHY, and the DSI bridge
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 43 (MIPI DSI), section 42.5.3 (the D-PHY test code interface), chapter 7
 * (VDMA's handshaking); ESP-IDF's mipi_dsi_host_reg.h, mipi_dsi_bridge_reg.h
 * and their structs for v3, the mipi_dsi_host_ll.h, mipi_dsi_brg_ll.h,
 * mipi_dsi_phy_ll.h and mipi_dsi_hal.c that drive them, and esp_lcd's
 * DSI driver (esp_lcd_mipi_dsi_bus.c, esp_lcd_panel_io_dbi.c,
 * esp_lcd_panel_dpi.c); MIPI DSI 1.1 for the link's packets.
 *
 * The DSI bridge (0x500A0800) takes the picture from memory through VDMA,
 * its handshaking interface 0, into a FIFO of 1024 64-bit words (the
 * credit threshold's reset value), which VDMA writes at MIPI_DSI_BRG_MEM
 * (0x50105000; any address there is the FIFO's port, written in order).
 *
 *   - Requests. With VDMA the flow controller (DSI_DMA_FLOW_CONTROLLER 0,
 *     as ESP-IDF has it) the bridge asks for a burst while its FIFO holds
 *     no more than RAW_BUF_ALMOST_EMPTY_THRD words, VDMA moving DST_MSIZE
 *     items a burst. As the flow controller (1, the reset value) it asks
 *     for DMA_BURST_LEN words while the FIFO holds no more than
 *     CREDIT_BURST_THRD, then single words, the block's last request
 *     marked: a block is BLK_RAW_NUM_TOTAL words with DMA_MULTIBLK_EN,
 *     else the frame's RAW_NUM_TOTAL (reloaded at the frame's end with
 *     RAW_NUM_TOTAL_AUTO_RELOAD, or by RAW_NUM_TOTAL_SET), with
 *     DMA_BLOCK_SLOT x DMA_BLOCK_INTERVAL and DMA_FRAME_SLOT x
 *     DMA_FRAME_INTERVAL bus cycles between blocks and frames where their
 *     intervals are enabled. After each acknowledge the bridge waits
 *     DMA_REQ_INTERVAL bus cycles, the field's set bits counting cycles
 *     as CSI_BRIG_DMA_REQ_INTERVAL's description has it. Bytes VDMA writes
 *     beyond the FIFO's room are lost (logged).
 *   - DPI timing. The DPI_* registers take effect as DPI_CONFIG_UPDATE is
 *     written; with DPI_EN, DSI_EN and DSI_DPI_CLK running, frames of
 *     VTOTAL lines of HTOTAL DPI clocks go out: VSYNC lines of sync, VBANK
 *     of back porch, VDISP of pixels, the rest front porch; each line
 *     HSYNC clocks of sync, HBANK of back porch, HDISP pixels, the rest
 *     front porch. The timing in effect is taken at each frame's start; a
 *     total no greater than its parts is logged and stretched by a clock
 *     (a line) past them. DPI_VSYNC_INT rises at each frame's start.
 *   - Pixels. A line's pixels leave the FIFO as the DPI sends them, taken
 *     in pieces of up to 2 KiB, each at its last pixel's DPI clock; a
 *     piece the FIFO cannot supply underruns: the line's remaining pixels
 *     are DPI_RSV_DPI_DATA's, the FIFO's late data goes to the lines that
 *     follow, and DPI_UNDERRUN_INT rises if the line is below
 *     FIFO_UNDERRUN_DISCARD_VCNT (counted from the frame's first line), as
 *     ESP-IDF sets it to report a whole frame's underruns (the TRM's text
 *     says the reverse). The input formats of TRM table 43.4-1 (RAW_TYPE):
 *     RGB888 as B, G, R in memory; RGB565 a little-endian word, R in bits
 *     15:11, widened as the 2D-DMA does on silicon (c << 3 | c & 7, the
 *     6-bit c << 2 | c & 3); GRAY one byte; YUV422 in YUV422_FORMAT's
 *     order from address 0 (0: U Y0 V Y1, 1: V Y0 U Y1, 2: Y0 U Y1 V,
 *     3: Y0 V Y1 U, as ESP-IDF's driver has them with YUV_PIX_ENDIAN 1;
 *     YUV_PIX_ENDIAN 0 takes each 64-bit word's bytes in reverse); YUV420
 *     as the table has it, rows of U Y0 Y1 then of V Y0 Y1, a row of U
 *     shown with the V of the row pair before (128 for the frame's first)
 *     and a row of V with the U of the row before; YUV444 as V, U, Y (the
 *     table names a pixel's components from its most significant byte);
 *     RGB666 as three bytes, B, G, R, each component in bits 5:0. YUV
 *     becomes RGB by YUV_CFG's BT.601 or BT.709 in its range, with the
 *     coefficients and rounding of the PPA (TRM 39.5.1.1). Each pixel goes
 *     to the host as a DPI word of DPI_TYPE in DPI_CONFIG's arrangement
 *     (TRM table 43.4-2), RGB666 and RGB565 its components' top bits.
 *   - DSI_CFG_REF_CLK_EN gates the host's PHY clocks; the loopback tests,
 *     BIST, the credit counters' reset and the memory controls are kept as
 *     written (logged where they would act).
 *   - Every event happens at its time, however late the QEMU timer that
 *     runs it (esp32_line_run_at()); register accesses, VDMA's writes and
 *     acknowledges, and DPI clock changes first run the DPI's events and
 *     the request interval's end that fell due before them, in order.
 *     Each piece of a line first brings in VDMA's rounds due by its time
 *     (the "vdma" link, esp32p4_dw_gdma_sync()), whose QEMU timer can run
 *     late as well: else an access catching the DPI up would take the
 *     FIFO short of what VDMA had written by then, an underrun the chip
 *     never has. VDMA's writes then reach the FIFO's port during the
 *     bridge's own register access, which its port allows.
 *
 * The DSI host (0x500A0000), DesignWare-derived, with its D-PHY:
 *
 *   - The PHY. Its test codes (esp32p4_mipi.h) 0x10, 0x11, 0x12, 0x17,
 *     0x18 (two sets), 0x19, 0x22 (two sets) and 0x44 hold what TRM
 *     43.4.3.3 gives them, TESTDOUT showing each as the TRM's loopback
 *     tables do; other codes are reserved (logged). The PHY works only on
 *     VDD_MIPI_DPHY ("vdd-mipi-dphy") within its recommended 2.25 to
 *     2.75 V (datasheet table 5-2): off, lower or higher (beyond the
 *     2.75 V absolute maximum, logged) it is as if shut down, its PLL
 *     unlocked and no lane driven. With PHY_SHUTDOWNZ and
 *     PHY_RSTZ set and DSI_DPHY_CFG_CLK and DSI_DPHY_PLL_REFCLK running
 *     (and the bridge's DSI_CFG_REF_CLK_EN), the PLL locks 2000 cycles of
 *     the configuration clock later, and the lanes reach LP-11 1000 after
 *     that (the TRM gives no times; ESP-IDF polls PHY_LOCK and the stop
 *     states); a change to the PLL's codes (0x10, 0x17, 0x18, 0x19, 0x44)
 *     meanwhile or after starts the lock again, as ESP-IDF programs them
 *     with the PHY already out of reset. The PLL runs at REFCLK x M / N,
 *     M and N from codes 0x18 and 0x17 where 0x19's bits 5 and 4 take
 *     them; f_REF / N outside 5 to 40 MHz, an odd M, f_VCO outside 80 to
 *     1500 MHz, or one outside the VCO range code 0x10 forces, does not
 *     lock (logged). Without 0x19's bits the TRM's undocumented defaults
 *     "for the hsfreqrange" stand: the model runs the PLL at the middle of
 *     code 0x44's range (logged).
 *     A rate outside hsfreqrange's range, or above 1 Gbit/s without code
 *     0x22's 1.5 Gbit/s bit, is logged and runs. TX_BYTE_CLK, the lane
 *     rate over 8, is the "lanebyteclk" output while locked.
 *   - The generic interface. GEN_HDR queues a packet in the command FIFO
 *     (16 entries), GEN_PLD_DATA writes the payload FIFO (128 words) and
 *     reads the read FIFO (128 words); a write to a full FIFO is dropped
 *     (GEN_CMD_WR_ERR, GEN_PLD_WR_ERR), a read of an empty one reads 0
 *     (GEN_PLD_RD_ERR). The FIFO depths are not documented; these are the
 *     model's. The packet engine takes the queued packets in order while
 *     the host is powered (PWR_UP) and its lanes are up: in command mode
 *     at once, in video mode in the vertical blanking lines' time. A
 *     long packet takes its word count's bytes from the payload FIFO
 *     (GEN_PLD_SEND_ERR, and zeros, for any it lacks). Each goes in LP
 *     (escape mode on lane 0: entry, the LPDT command, 16 escape clocks a
 *     byte, the exit; TX_ESC_CLK, TX_BYTE_CLK / TX_ESC_CLK_DIVISION,
 *     stopped below 2) or HS (PHY_LP2HS_TIME, the bytes over the lanes,
 *     PHY_HS2LP_TIME, and the clock lane's PHY_CLKLP2HS_TIME and
 *     PHY_CLKHS2LP_TIME around it under AUTO_CLKLANE_CTRL; with neither
 *     that nor PHY_TXREQUESTCLKHS there is no HS clock and the panel gets
 *     nothing), as CMD_MODE_CFG's bit for its type (or VID_MODE_CFG's
 *     LP_CMD_EN in video mode) says, with an EoTp where EOTP_TX_EN
 *     (EOTP_TX_LP_EN) asks. A read, or any packet with ACK_RQST_EN, is
 *     followed by a bus turnaround (BTA_EN) of 10 TLPX each way, an
 *     escape clock each, the panel's answer in LPDT between: read data
 *     to the read FIFO (GEN_PLD_RECEV_ERR when it overflows), an
 *     Acknowledge and Error Report into INT_ST0's ACK_WITH_ERR bits, the
 *     answer's ECC and checksum checked (ECC_RX_EN: ECC_SINGLE_ERR,
 *     ECC_MILTI_ERR; CRC_RX_EN: CRC_ERR; PKT_SIZE_ERR for a short one;
 *     EOPT_ERR with EOTP_RX_EN and no EoTp). No answer: TO_LP_RX after
 *     LPRX_TO_CNT x TO_CLK_DIVISION byte clocks. GEN_RD_CMD_BUSY holds
 *     from the read's queuing until its answer is in (or the timeout).
 *     The link then stays still for HS_RD/LP_RD/HS_WR/LP_WR_TO_CNT and
 *     BTA_TO_CNT byte clocks. GEN_VCID_RX names the virtual channel
 *     answers are taken from.
 *   - Video mode (MODE_CFG 0, PWR_UP, the lanes up). The host follows the
 *     bridge's DPI: the frame starts at its VSYNC, the host counting lines
 *     from there through VSA, VBP, VACTIVE and VFP; each line starts with
 *     a sync packet (VSS, then HSS; with sync pulses VSE and HSE as well)
 *     and a line in VACTIVE carries its pixels in max(1, VID_NUM_CHUNKS)
 *     packets of VID_PKT_SIZE pixels (with null packets of VID_NULL_SIZE
 *     between), of DPI_COLOR_CODING's data type (0x0E, 0x1E or with
 *     LOOSELY18_EN 0x2E, 0x3E) on virtual channel DPI_VCID, each pixel
 *     taken from its DPI word as table 43.4-2 has it. A line whose
 *     pixels are fewer raises DPI_BUFF_PLD_UNDER (the rest 0), more or
 *     outside VACTIVE DPI_PLD_WR_ERR (the rest dropped), and one whose
 *     packets do not fit in the line, from HSA + HBP on at the lanes'
 *     rate, DPI_PLD_WR_ERR (the line lost); each once a frame. The bridge
 *     drives HSYNC, VSYNC and DE active high: DPI_CFG_POL must say so, or
 *     the host finds no timing (logged). With FRAME_BTA_ACK_EN the first
 *     VFP line asks the panel for an acknowledge. DPI's shutdown and
 *     colour mode lines (the bridge's DPI_LCD_CTL) send Shut Down / Turn
 *     On Peripheral and Color Mode On / Off. With VPG_EN the host times
 *     frames itself (HLINE_TIME byte clocks a line, VSA + VBP + VACTIVE +
 *     VFP lines) and sends TRM 43.4.2.3's colour bars or BER pattern,
 *     VID_PKT_SIZE x max(1, VID_NUM_CHUNKS) pixels wide: eight bars of
 *     the width (height) over eight, black taking the remainder; the BER
 *     pattern's eight byte values in runs of an eighth of the line's
 *     bytes, 0x80 taking the remainder. VID_SHADOW_EN: the *_ACT
 *     registers, which video uses, take the written values on
 *     VID_SHADOW_REQ at the next frame's start (at once outside video
 *     mode), the request clearing then; without it they follow the
 *     writes.
 *   - PHY_ULPS_CTRL puts the data or clock lanes in ULPS (an escape entry
 *     later) and takes them out; PHY_TX_TRIGGERS sends lane 0's escape
 *     triggers, bit 0 the Reset-Trigger, which resets the panel.
 *   - Interrupts. INT_ST0/1 latch their errors and clear as they are
 *     read; INT_FORCE0/1's bits read as set in them while set. DSI_INTR
 *     is high while a status bit is set whose INT_MSK0/1 bit is set: the
 *     DesignWare core's meaning, as the CSI host's masks have it (TRM
 *     42.6); TRM 43.4.2.4 says the reverse.
 *   - Register accesses first run what fell due before them, in order: the
 *     PHY's start-up, the link's phases, the pattern generator's lines and
 *     the bridge's DPI events (the bridge the host follows, which its
 *     "dsi-host" link names), each at its time.
 *   - Not modelled (logged when used): the DBI and eDPI interfaces, 3D,
 *     display stream compression, the TE hardware, HS skew calibration,
 *     VID_SHADOW_PIN_REQ, HS transmit timeouts (TO_HS_TX: the model's link
 *     never stalls) and the D-PHY's escape-mode errors from the panel
 *     (DPHY_ERRORS).
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/esp32p4_mipi_dsi.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_image_source.h"
#include "hw/misc/esp32p4_mipi.h"
#include "migration/vmstate.h"

/* ---- The DSI host's registers ---- */

#define A_VERSION               0x000
#define A_PWR_UP                0x004
#define A_CLKMGR_CFG            0x008
#define A_DPI_VCID              0x00c
#define A_DPI_COLOR_CODING      0x010
#define A_DPI_CFG_POL           0x014
#define A_DPI_LP_CMD_TIM        0x018
#define A_DBI_VCID              0x01c
#define A_DBI_CFG               0x020
#define A_DBI_PARTITIONING_EN   0x024
#define A_DBI_CMDSIZE           0x028
#define A_PCKHDL_CFG            0x02c
#define A_GEN_VCID              0x030
#define A_MODE_CFG              0x034
#define A_VID_MODE_CFG          0x038
#define A_VID_PKT_SIZE          0x03c
#define A_VID_NUM_CHUNKS        0x040
#define A_VID_NULL_SIZE         0x044
#define A_VID_HSA_TIME          0x048
#define A_VID_HBP_TIME          0x04c
#define A_VID_HLINE_TIME        0x050
#define A_VID_VSA_LINES         0x054
#define A_VID_VBP_LINES         0x058
#define A_VID_VFP_LINES         0x05c
#define A_VID_VACTIVE_LINES     0x060
#define A_EDPI_CMD_SIZE         0x064
#define A_CMD_MODE_CFG          0x068
#define A_GEN_HDR               0x06c
#define A_GEN_PLD_DATA          0x070
#define A_CMD_PKT_STATUS        0x074
#define A_TO_CNT_CFG            0x078
#define A_HS_RD_TO_CNT          0x07c
#define A_LP_RD_TO_CNT          0x080
#define A_HS_WR_TO_CNT          0x084
#define A_LP_WR_TO_CNT          0x088
#define A_BTA_TO_CNT            0x08c
#define A_SDF_3D                0x090
#define A_LPCLK_CTRL            0x094
#define A_PHY_TMR_LPCLK_CFG     0x098
#define A_PHY_TMR_CFG           0x09c
#define A_PHY_RSTZ              0x0a0
#define A_PHY_IF_CFG            0x0a4
#define A_PHY_ULPS_CTRL         0x0a8
#define A_PHY_TX_TRIGGERS       0x0ac
#define A_PHY_STATUS            0x0b0
#define A_PHY_TST_CTRL0         0x0b4
#define A_PHY_TST_CTRL1         0x0b8
#define A_INT_ST0               0x0bc
#define A_INT_ST1               0x0c0
#define A_INT_MSK0              0x0c4
#define A_INT_MSK1              0x0c8
#define A_PHY_CAL               0x0cc
#define A_INT_FORCE0            0x0d8
#define A_INT_FORCE1            0x0dc
#define A_DSC_PARAMETER         0x0f0
#define A_PHY_TMR_RD_CFG        0x0f4
#define A_VID_SHADOW_CTRL       0x100
#define A_DPI_VCID_ACT          0x10c
#define A_DPI_COLOR_CODING_ACT  0x110
#define A_DPI_LP_CMD_TIM_ACT    0x118
#define A_EDPI_TE_HW_CFG        0x11c
#define A_VID_MODE_CFG_ACT      0x138
#define A_VID_PKT_SIZE_ACT      0x13c
#define A_VID_NUM_CHUNKS_ACT    0x140
#define A_VID_NULL_SIZE_ACT     0x144
#define A_VID_HSA_TIME_ACT      0x148
#define A_VID_HBP_TIME_ACT      0x14c
#define A_VID_HLINE_TIME_ACT    0x150
#define A_VID_VSA_LINES_ACT     0x154
#define A_VID_VBP_LINES_ACT     0x158
#define A_VID_VFP_LINES_ACT     0x15c
#define A_VID_VACTIVE_LINES_ACT 0x160
#define A_VID_PKT_STATUS        0x168
#define A_SDF_3D_ACT            0x190

#define R(a)                    ((a) / 4)

/* PCKHDL_CFG */
#define EOTP_TX_EN              BIT(0)
#define EOTP_RX_EN              BIT(1)
#define BTA_EN                  BIT(2)
#define ECC_RX_EN               BIT(3)
#define CRC_RX_EN               BIT(4)
#define EOTP_TX_LP_EN           BIT(5)
/* VID_MODE_CFG */
#define LP_CMD_EN               BIT(15)
#define FRAME_BTA_ACK_EN        BIT(14)
#define VPG_EN                  BIT(16)
#define VPG_MODE                BIT(20)
#define VPG_ORIENTATION         BIT(24)
/* CMD_MODE_CFG */
#define TEAR_FX_EN              BIT(0)
#define ACK_RQST_EN             BIT(1)
/* LPCLK_CTRL */
#define TXREQUESTCLKHS          BIT(0)
#define AUTO_CLKLANE_CTRL       BIT(1)
/* PHY_RSTZ */
#define PHY_SHUTDOWNZ           BIT(0)
#define PHY_RSTZ                BIT(1)
#define PHY_ENABLECLK           BIT(2)
#define PHY_FORCEPLL            BIT(3)
/* PHY_ULPS_CTRL */
#define TXREQULPSCLK            BIT(0)
#define TXEXITULPSCLK           BIT(1)
#define TXREQULPSLAN            BIT(2)
#define TXEXITULPSLAN           BIT(3)
/* VID_SHADOW_CTRL */
#define VID_SHADOW_EN           BIT(0)
#define VID_SHADOW_REQ          BIT(8)
#define VID_SHADOW_PIN_REQ      BIT(16)

/* INT_ST1 */
#define TO_HS_TX                BIT(0)
#define TO_LP_RX                BIT(1)
#define ECC_SINGLE_ERR          BIT(2)
#define ECC_MILTI_ERR           BIT(3)
#define CRC_ERR                 BIT(4)
#define PKT_SIZE_ERR            BIT(5)
#define EOPT_ERR                BIT(6)
#define DPI_PLD_WR_ERR          BIT(7)
#define GEN_CMD_WR_ERR          BIT(8)
#define GEN_PLD_WR_ERR          BIT(9)
#define GEN_PLD_SEND_ERR        BIT(10)
#define GEN_PLD_RD_ERR          BIT(11)
#define GEN_PLD_RECEV_ERR       BIT(12)
#define DPI_BUFF_PLD_UNDER      BIT(19)
#define INT0_MASK               0x1fffff
#define INT1_MASK               0x81fff

enum {
    K_RW,       /* stored as written */
    K_RO,       /* computed or fixed; writes are logged */
    K_SPECIAL,  /* stored, with side effects */
    K_UNIMP,    /* stored; non-zero values are logged as unimplemented */
};

typedef struct DsiReg {
    uint16_t off;
    uint8_t kind;
    uint32_t mask;
    uint32_t reset;
} DsiReg;

static const DsiReg host_regs[] = {
    { A_VERSION, K_RO, 0, 0x3134312a },
    { A_PWR_UP, K_SPECIAL, 0x1, 0 },
    { A_CLKMGR_CFG, K_RW, 0xffff, 0 },
    { A_DPI_VCID, K_SPECIAL, 0x3, 0 },
    { A_DPI_COLOR_CODING, K_SPECIAL, 0x10f, 0 },
    { A_DPI_CFG_POL, K_RW, 0x1f, 0 },
    { A_DPI_LP_CMD_TIM, K_SPECIAL, 0xff00ff, 0 },
    { A_DBI_VCID, K_UNIMP, 0x3, 0 },
    { A_DBI_CFG, K_UNIMP, 0x30f0f, 0 },
    { A_DBI_PARTITIONING_EN, K_UNIMP, 0x1, 0 },
    { A_DBI_CMDSIZE, K_UNIMP, 0xffffffff, 0 },
    { A_PCKHDL_CFG, K_RW, 0x3f, 0 },
    { A_GEN_VCID, K_RW, 0x30303, 0 },
    { A_MODE_CFG, K_SPECIAL, 0x1, 0x1 },
    { A_VID_MODE_CFG, K_SPECIAL, 0x0111ff03, 0 },
    { A_VID_PKT_SIZE, K_SPECIAL, 0x3fff, 0 },
    { A_VID_NUM_CHUNKS, K_SPECIAL, 0x1fff, 0 },
    { A_VID_NULL_SIZE, K_SPECIAL, 0x1fff, 0 },
    { A_VID_HSA_TIME, K_SPECIAL, 0xfff, 0 },
    { A_VID_HBP_TIME, K_SPECIAL, 0xfff, 0 },
    { A_VID_HLINE_TIME, K_SPECIAL, 0x7fff, 0 },
    { A_VID_VSA_LINES, K_SPECIAL, 0x3ff, 0 },
    { A_VID_VBP_LINES, K_SPECIAL, 0x3ff, 0 },
    { A_VID_VFP_LINES, K_SPECIAL, 0x3ff, 0 },
    { A_VID_VACTIVE_LINES, K_SPECIAL, 0x3fff, 0 },
    { A_EDPI_CMD_SIZE, K_UNIMP, 0xffff, 0 },
    { A_CMD_MODE_CFG, K_SPECIAL, 0x010f7f03, 0 },
    { A_GEN_HDR, K_SPECIAL, 0xffffff, 0 },
    { A_GEN_PLD_DATA, K_SPECIAL, 0xffffffff, 0 },
    { A_CMD_PKT_STATUS, K_RO, 0, 0x50015 },
    { A_TO_CNT_CFG, K_RW, 0xffffffff, 0 },
    { A_HS_RD_TO_CNT, K_RW, 0xffff, 0 },
    { A_LP_RD_TO_CNT, K_RW, 0xffff, 0 },
    { A_HS_WR_TO_CNT, K_RW, 0xffff, 0 },
    { A_LP_WR_TO_CNT, K_RW, 0xffff, 0 },
    { A_BTA_TO_CNT, K_RW, 0xffff, 0 },
    { A_SDF_3D, K_SPECIAL, 0x1003f, 0 },
    { A_LPCLK_CTRL, K_RW, 0x3, 0 },
    { A_PHY_TMR_LPCLK_CFG, K_RW, 0x03ff03ff, 0 },
    { A_PHY_TMR_CFG, K_RW, 0x03ff03ff, 0 },
    { A_PHY_RSTZ, K_SPECIAL, 0xf, 0 },
    { A_PHY_IF_CFG, K_RW, 0xff03, 0x1 },
    { A_PHY_ULPS_CTRL, K_SPECIAL, 0xf, 0 },
    { A_PHY_TX_TRIGGERS, K_SPECIAL, 0xf, 0 },
    { A_PHY_STATUS, K_RO, 0, 0x140 },
    { A_PHY_TST_CTRL0, K_SPECIAL, 0x3, 0x1 },
    { A_PHY_TST_CTRL1, K_SPECIAL, 0x100ff, 0 },
    { A_INT_ST0, K_RO, 0, 0 },
    { A_INT_ST1, K_RO, 0, 0 },
    { A_INT_MSK0, K_SPECIAL, INT0_MASK, 0 },
    { A_INT_MSK1, K_SPECIAL, INT1_MASK, 0 },
    { A_PHY_CAL, K_UNIMP, 0x1, 0 },
    { A_INT_FORCE0, K_SPECIAL, INT0_MASK, 0 },
    { A_INT_FORCE1, K_SPECIAL, INT1_MASK, 0 },
    { A_DSC_PARAMETER, K_UNIMP, 0x30301, 0 },
    { A_PHY_TMR_RD_CFG, K_RW, 0x7fff, 0 },
    { A_VID_SHADOW_CTRL, K_SPECIAL, 0x10101, 0 },
    { A_DPI_VCID_ACT, K_RO, 0, 0 },
    { A_DPI_COLOR_CODING_ACT, K_RO, 0, 0 },
    { A_DPI_LP_CMD_TIM_ACT, K_RO, 0, 0 },
    { A_EDPI_TE_HW_CFG, K_UNIMP, 0xffff0013, 0 },
    { A_VID_MODE_CFG_ACT, K_RO, 0, 0 },
    { A_VID_PKT_SIZE_ACT, K_RO, 0, 0 },
    { A_VID_NUM_CHUNKS_ACT, K_RO, 0, 0 },
    { A_VID_NULL_SIZE_ACT, K_RO, 0, 0 },
    { A_VID_HSA_TIME_ACT, K_RO, 0, 0 },
    { A_VID_HBP_TIME_ACT, K_RO, 0, 0 },
    { A_VID_HLINE_TIME_ACT, K_RO, 0, 0 },
    { A_VID_VSA_LINES_ACT, K_RO, 0, 0 },
    { A_VID_VBP_LINES_ACT, K_RO, 0, 0 },
    { A_VID_VFP_LINES_ACT, K_RO, 0, 0 },
    { A_VID_VACTIVE_LINES_ACT, K_RO, 0, 0 },
    { A_VID_PKT_STATUS, K_RO, 0, 0x50005 },
    { A_SDF_3D_ACT, K_RO, 0, 0 },
};

/* The registers the *_ACT registers shadow, each in its own layout. */
static const uint16_t shadowed[] = {
    A_DPI_VCID, A_DPI_COLOR_CODING, A_DPI_LP_CMD_TIM, A_VID_MODE_CFG,
    A_VID_PKT_SIZE, A_VID_NUM_CHUNKS, A_VID_NULL_SIZE, A_VID_HSA_TIME,
    A_VID_HBP_TIME, A_VID_HLINE_TIME, A_VID_VSA_LINES, A_VID_VBP_LINES,
    A_VID_VFP_LINES, A_VID_VACTIVE_LINES, A_SDF_3D,
};

static const DsiReg *host_reg(hwaddr off)
{
    for (int i = 0; i < ARRAY_SIZE(host_regs); i++) {
        if (host_regs[i].off == off) {
            return &host_regs[i];
        }
    }
    return NULL;
}

/* The D-PHY's states. */
enum {
    PHY_OFF,
    PHY_LOCKING,
    PHY_INIT,
    PHY_READY,
    PHY_NOLOCK,
};

#define PHY_LOCK_CYCLES     2000
#define PHY_INIT_CYCLES     1000

/* The link's transmissions. */
enum {
    TXN_SEND,
    TXN_RESP,
    TXN_FRAME_ACK,
};

/* LP signalling, in escape clocks (TLPX): see the file's comment. */
#define LP_ESC_OVERHEAD     22
#define LP_ESC_PER_BYTE     16
#define LP_TA_ESC           10

/* ---- Small helpers ---- */

static bool dsi_dt_long(uint8_t dt)
{
    switch (dt) {
    case 0x09: case 0x19: case 0x29: case 0x39:
    case 0x0c: case 0x1c: case 0x2c:
    case 0x0d: case 0x1d: case 0x3d:
    case 0x0e: case 0x1e: case 0x2e: case 0x3e:
        return true;
    default:
        return false;
    }
}

static bool dsi_dt_read(uint8_t dt)
{
    return dt == 0x04 || dt == 0x14 || dt == 0x24 || dt == 0x06;
}

static uint32_t hreg(ESP32P4DsiHostState *s, hwaddr a)
{
    return s->regs[R(a)];
}

static uint32_t hact(ESP32P4DsiHostState *s, hwaddr a)
{
    return s->act[R(a)];
}

static void host_update_irq(ESP32P4DsiHostState *s)
{
    uint32_t st0 = s->int_st[0] | hreg(s, A_INT_FORCE0);
    uint32_t st1 = s->int_st[1] | hreg(s, A_INT_FORCE1);

    qemu_set_irq(s->irq, (st0 & hreg(s, A_INT_MSK0)) ||
                         (st1 & hreg(s, A_INT_MSK1)));
}

static void host_error(ESP32P4DsiHostState *s, int n, uint32_t bits)
{
    s->int_st[n] |= bits;
    host_update_irq(s);
}

static ESP32P4DsiPanelState *host_panel(ESP32P4DsiHostState *s)
{
    BusChild *kid = QTAILQ_FIRST(&BUS(&s->bus)->children);

    return kid ? ESP32P4_DSI_PANEL(kid->child) : NULL;
}

static uint32_t host_lanes(ESP32P4DsiHostState *s)
{
    return (hreg(s, A_PHY_IF_CFG) & 3) + 1;
}

/* The time @n lane byte clocks after @t. */
static int64_t byte_after(ESP32P4DsiHostState *s, int64_t t, uint64_t n)
{
    return esp32p4_clk_edges_after(&s->byte_edges, t, n);
}

static uint32_t esc_div(ESP32P4DsiHostState *s)
{
    return hreg(s, A_CLKMGR_CFG) & 0xff;
}

/* The time @n escape clocks after @t; INT64_MAX while it is stopped. */
static int64_t esc_after(ESP32P4DsiHostState *s, int64_t t, uint64_t n)
{
    if (esc_div(s) < 2) {
        return INT64_MAX;
    }
    return byte_after(s, t, n * esc_div(s));
}

/* ---- The D-PHY ---- */

static uint8_t phy_dout(ESP32P4DsiHostState *s)
{
    uint8_t din = esp32p4_dphy_test_din(&s->test);

    switch (s->test.code) {
    case 0x10:
        return s->t10 & 0xbf;
    case 0x11:
        return s->t11;
    case 0x12:
        return s->t12;
    case 0x17:
        return s->t17 & 0x7f;
    case 0x18:
        return din & 0x80 ? s->t18[1] & 0x0f : s->t18[0] & 0x1f;
    case 0x19:
        return s->t19 & 0x30;
    case 0x22:
        return din & 0x80 ? s->t22[1] & 0x0f : s->t22[0] & 0x7f;
    case 0x44:
        return s->t44 & 0x7f;
    default:
        return 0;
    }
}

static void phy_test_defaults(ESP32P4DsiHostState *s)
{
    s->t10 = 0x01;
    s->t11 = 0x02;
    s->t12 = 0x02;
    s->t17 = 0x01;
    s->t18[0] = 0x05;
    s->t18[1] = 0x00;
    s->t19 = 0x00;
    s->t22[0] = 0x00;
    s->t22[1] = 0x00;
    s->t44 = 0x00;
}

static void phy_test_data(ESP32P4DsiHostState *s, uint8_t data)
{
    switch (s->test.code) {
    case 0x00:
        break;
    case 0x10:
        s->t10 = data;
        break;
    case 0x11:
        s->t11 = data;
        break;
    case 0x12:
        s->t12 = data;
        break;
    case 0x17:
        s->t17 = data & 0x7f;
        break;
    case 0x18:
        s->t18[data >> 7] = data & (data & 0x80 ? 0x0f : 0x1f);
        break;
    case 0x19:
        s->t19 = data & 0x30;
        break;
    case 0x22:
        s->t22[data >> 7] = data & (data & 0x80 ? 0x0f : 0x7f);
        break;
    case 0x44:
        if (data & 0x80) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: test code 0x44's program "
                          "selector 1 is invalid\n", TYPE_ESP32P4_DSI_HOST);
            break;
        }
        s->t44 = data;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: test data 0x%02x for reserved "
                      "test code 0x%02x\n", TYPE_ESP32P4_DSI_HOST, data,
                      s->test.code);
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.dphy-supply] */
static bool phy_supplied(ESP32P4DsiHostState *s)
{
    return s->vdd_mv >= ESP32P4_DPHY_VDD_MIN_MV &&
           s->vdd_mv <= ESP32P4_DPHY_VDD_MAX_MV;
}

static bool phy_wanted(ESP32P4DsiHostState *s)
{
    uint32_t r = hreg(s, A_PHY_RSTZ);

    return (r & PHY_SHUTDOWNZ) && (r & PHY_RSTZ) && s->cfg_ref_en &&
           phy_supplied(s) && clock_is_enabled(s->clk_cfg) &&
           clock_is_enabled(s->clk_ref);
}

static void phy_set_byte_out(ESP32P4DsiHostState *s, ESP32P4ClkEdges e)
{
    ESP32P4ClkEdges old = s->byte_out;

    s->byte_out = e;
    esp32p4_clk_update(s->lanebyteclk, &old, &e);
}

/*
 * The PLL's lock at @now: its rate, and the byte clock's edges, from the
 * reference clock's; false if it cannot lock.
 */
static bool phy_pll_lock(ESP32P4DsiHostState *s, int64_t now)
{
    uint32_t lo, hi, n, m;
    uint64_t ref_hz = clock_get_hz(s->clk_ref), vco_hz;
    ESP32P4ClkPeriod byte;

    if (!esp32p4_dphy_hsfreqrange((s->t44 >> 1) & 0x3f, &lo, &hi)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: hsfreqrange 0x%02x is not one "
                      "the PHY has; its PLL does not lock\n",
                      TYPE_ESP32P4_DSI_HOST, (s->t44 >> 1) & 0x3f);
        return false;
    }
    s->ref_edges = esp32p4_clk_edges_or_at(s->clk_ref, now);
    if (!(s->t19 & 0x30)) {
        uint64_t mbps = (lo + hi + 1) / 2;

        qemu_log_mask(LOG_UNIMP, "%s: the PLL's default dividers for an "
                      "hsfreqrange are not documented; it runs at %" PRIu64
                      " Mbit/s, the range's middle\n", TYPE_ESP32P4_DSI_HOST,
                      mbps);
        s->lane_mbps_x1000 = mbps * 1000;
        byte = esp32p4_clk_period_of_hz(mbps * 1000000, 8);
        s->byte_edges = esp32p4_clk_edges_from(byte, &s->ref_edges, now);
        return true;
    }
    if ((s->t19 & 0x30) != 0x30) {
        qemu_log_mask(LOG_UNIMP, "%s: one PLL divider from the test codes "
                      "and one default: the defaults are not documented; "
                      "the model takes both from codes 0x17 and 0x18\n",
                      TYPE_ESP32P4_DSI_HOST);
    }
    n = (s->t17 & 0x7f) + 1;
    m = ((s->t18[1] & 0x0f) << 5 | (s->t18[0] & 0x1f)) + 1;
    vco_hz = ref_hz * m / n;
    if (ref_hz / n < 5000000 || ref_hz / n > 40000000 || (m & 1) ||
        vco_hz < 80000000 || vco_hz > 1500000000) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: PLL M %u, N %u on a %" PRIu64
                      " Hz reference (f_REF / N 5 to 40 MHz, M even, "
                      "f_VCO 80 to 1500 MHz): it does not lock\n",
                      TYPE_ESP32P4_DSI_HOST, m, n, ref_hz);
        return false;
    }
    if (s->t10 & 0x80) {
        static const uint16_t vco_lo[8] = {
            80, 200, 300, 500, 700, 900, 1100, 1300,
        };
        uint32_t r = (s->t10 >> 3) & 7;
        uint64_t mhz = vco_hz / 1000000;

        if (mhz < vco_lo[r] || mhz > (r == 7 ? 1500 : vco_lo[r + 1])) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: VCO range %u forced for a "
                          "%" PRIu64 " MHz VCO: the PLL does not lock\n",
                          TYPE_ESP32P4_DSI_HOST, r, mhz);
            return false;
        }
    }
    s->lane_mbps_x1000 = vco_hz / 1000;
    byte = esp32p4_clk_period_scale(s->ref_edges.period, 8ull * n, m);
    s->byte_edges = esp32p4_clk_edges_from(byte, &s->ref_edges, now);
    return true;
}

static void phy_check_rate(ESP32P4DsiHostState *s)
{
    uint32_t lo, hi;
    uint64_t mbps = s->lane_mbps_x1000 / 1000;

    if (s->warned_range ||
        !esp32p4_dphy_hsfreqrange((s->t44 >> 1) & 0x3f, &lo, &hi)) {
        return;
    }
    if (mbps < lo || mbps > hi) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the lanes run at %" PRIu64
                      " Mbit/s, outside hsfreqrange's %u to %u\n",
                      TYPE_ESP32P4_DSI_HOST, mbps, lo, hi);
        s->warned_range = true;
    }
    if (mbps > 1000 && !(s->t22[1] & 0x08)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %" PRIu64 " Mbit/s without test "
                      "code 0x22's 1.5 Gbit/s support\n",
                      TYPE_ESP32P4_DSI_HOST, mbps);
        s->warned_range = true;
    }
}

static void link_kick(ESP32P4DsiHostState *s);
static void video_stop(ESP32P4DsiHostState *s);
static void vpg_update(ESP32P4DsiHostState *s);
static void phy_update(ESP32P4DsiHostState *s);

static int64_t phy_cfg_after(ESP32P4DsiHostState *s, int64_t t, uint64_t n)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->clk_cfg, t);

    return esp32p4_clk_edges_after(&e, t, n);
}

/* Bring the PHY's state up to date at esp32_line_time(). */
static void phy_update(ESP32P4DsiHostState *s)
{
    int64_t now = esp32_line_time();

    if (!phy_wanted(s)) {
        if (s->phy_state != PHY_OFF) {
            s->phy_state = PHY_OFF;
            timer_del(s->phy_timer);
            phy_set_byte_out(s, esp32p4_clk_edges_stopped());
            s->byte_edges = esp32p4_clk_edges_stopped();
            s->data_ulps = false;
            s->clk_ulps = false;
            s->txn = false;
            s->rd_busy = false;
            video_stop(s);
        }
        return;
    }
    for (;;) {
        switch (s->phy_state) {
        case PHY_OFF:
            s->phy_state = PHY_LOCKING;
            s->phy_due_ns = phy_cfg_after(s, now, PHY_LOCK_CYCLES);
            break;
        case PHY_LOCKING:
            if (s->phy_due_ns > now) {
                timer_mod(s->phy_timer, s->phy_due_ns);
                return;
            }
            if (!phy_pll_lock(s, s->phy_due_ns)) {
                s->phy_state = PHY_NOLOCK;
                return;
            }
            phy_check_rate(s);
            phy_set_byte_out(s, s->byte_edges);
            s->phy_state = PHY_INIT;
            s->phy_due_ns = phy_cfg_after(s, s->phy_due_ns, PHY_INIT_CYCLES);
            break;
        case PHY_INIT:
            if (s->phy_due_ns > now) {
                timer_mod(s->phy_timer, s->phy_due_ns);
                return;
            }
            s->phy_state = PHY_READY;
            s->link_free_ns = s->phy_due_ns;
            link_kick(s);
            vpg_update(s);
            return;
        default:
            return;
        }
    }
}

static void phy_timer_cb(void *opaque)
{
    ESP32P4DsiHostState *s = opaque;

    esp32_line_run_at(s->phy_due_ns, (void (*)(void *))phy_update, s);
}

static bool phy_ready(ESP32P4DsiHostState *s)
{
    return s->phy_state == PHY_READY;
}

static uint32_t phy_status(ESP32P4DsiHostState *s)
{
    int64_t now = esp32_line_time();
    uint32_t st = 0;
    bool lane1 = host_lanes(s) > 1;
    bool busy = s->txn || now < s->trigger_end_ns;

    if (s->phy_state == PHY_OFF || s->phy_state == PHY_LOCKING ||
        s->phy_state == PHY_NOLOCK) {
        return 0x140;
    }
    st |= BIT(0);                                   /* PHY_LOCK */
    if (s->phy_state != PHY_READY) {
        return st | BIT(3) | BIT(5) | BIT(8);
    }
    if (s->txn && s->txn_kind == TXN_RESP) {
        st |= BIT(1);                               /* PHY_DIRECTION */
    }
    if (!s->clk_ulps && !(hreg(s, A_LPCLK_CTRL) & TXREQUESTCLKHS &&
                          !(hreg(s, A_LPCLK_CTRL) & AUTO_CLKLANE_CTRL))) {
        st |= BIT(2);                               /* STOPSTATECLKLANE */
    }
    if (!s->clk_ulps || now < s->ulps_due_ns) {
        st |= BIT(3);                               /* ULPSACTIVENOTCLK */
    }
    if (!s->data_ulps && !busy) {
        st |= BIT(4);                               /* STOPSTATE0LANE */
        if (lane1) {
            st |= BIT(7);                           /* STOPSTATE1LANE */
        }
    }
    if (!s->data_ulps || now < s->ulps_due_ns) {
        st |= BIT(5);                               /* ULPSACTIVENOT0LANE */
        st |= BIT(8);                               /* ULPSACTIVENOT1LANE */
    }
    return st;
}

/* ---- The link: packets to the panel and its answers ---- */

/* Whether HS transmissions have a clock on the clock lane. */
static bool hs_clock(ESP32P4DsiHostState *s)
{
    uint32_t c = hreg(s, A_LPCLK_CTRL);

    return (c & (TXREQUESTCLKHS | AUTO_CLKLANE_CTRL)) && !s->clk_ulps &&
           (hreg(s, A_PHY_RSTZ) & PHY_ENABLECLK);
}

/* The byte clocks of an HS transmission of @bytes. */
static uint64_t hs_cycles(ESP32P4DsiHostState *s, uint32_t bytes)
{
    uint32_t tmr = hreg(s, A_PHY_TMR_CFG), clk = hreg(s, A_PHY_TMR_LPCLK_CFG);
    uint64_t n;

    if (hreg(s, A_PCKHDL_CFG) & EOTP_TX_EN) {
        bytes += 4;
    }
    n = (tmr & 0x3ff) + DIV_ROUND_UP(bytes, host_lanes(s)) +
        extract32(tmr, 16, 10);
    if (hreg(s, A_LPCLK_CTRL) & AUTO_CLKLANE_CTRL) {
        n += (clk & 0x3ff) + extract32(clk, 16, 10);
    }
    return n;
}

/* The escape clocks of an LPDT transmission of @bytes. */
static uint64_t lp_cycles(ESP32P4DsiHostState *s, uint32_t bytes)
{
    if (hreg(s, A_PCKHDL_CFG) & EOTP_TX_LP_EN) {
        bytes += 4;
    }
    return LP_ESC_OVERHEAD + (uint64_t)LP_ESC_PER_BYTE * bytes;
}

static void link_deliver(ESP32P4DsiHostState *s, const ESP32P4DsiPacket *p)
{
    ESP32P4DsiPanelState *panel = host_panel(s);

    if (!panel) {
        return;
    }
    if (p->hs && !hs_clock(s)) {
        static bool warned;

        if (!warned) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: HS transmission with the "
                          "clock lane in LP (neither AUTO_CLKLANE_CTRL nor "
                          "PHY_TXREQUESTCLKHS): the panel receives nothing\n",
                          TYPE_ESP32P4_DSI_HOST);
            warned = true;
        }
        return;
    }
    esp32p4_dsi_panel_receive(panel, p);
}

/* Whether the generic interface sends a packet of type @dt in LP. */
static bool cmd_lp(ESP32P4DsiHostState *s, uint8_t dt)
{
    /* CMD_MODE_CFG's bits 8 to 24 for these data types, in order. */
    static const uint8_t dts[] = {
        0x03, 0x13, 0x23, 0x04, 0x14, 0x24, 0x29, 0x00, 0x05, 0x15, 0x06,
        0x39, 0x00, 0x00, 0x00, 0x00, 0x37,
    };
    uint32_t c = hreg(s, A_CMD_MODE_CFG);

    if (!(hreg(s, A_MODE_CFG) & 1)) {
        return hact(s, A_VID_MODE_CFG) & LP_CMD_EN;
    }
    for (int i = 0; i < ARRAY_SIZE(dts); i++) {
        if (dts[i] && dts[i] == dt) {
            return c & BIT(8 + i);
        }
    }
    return false;
}

static uint32_t rfifo_room(ESP32P4DsiHostState *s)
{
    return ESP32P4_DSI_GEN_PLD_DEPTH - s->rpld_num;
}

static void rfifo_push(ESP32P4DsiHostState *s, const uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i += 4) {
        uint32_t w = 0;

        for (uint32_t k = 0; k < 4 && i + k < n; k++) {
            w |= (uint32_t)b[i + k] << (8 * k);
        }
        if (!rfifo_room(s)) {
            host_error(s, 1, GEN_PLD_RECEV_ERR);
            return;
        }
        s->rpld[(s->rpld_head + s->rpld_num++) % ESP32P4_DSI_GEN_PLD_DEPTH] =
            w;
    }
}

/*
 * The panel's answer, @b of @len bytes, as it has all arrived: each packet
 * checked and taken.
 */
static void link_take_answer(ESP32P4DsiHostState *s, const uint8_t *b,
                             uint32_t len, bool read)
{
    uint32_t pck = hreg(s, A_PCKHDL_CFG), pos = 0;
    bool eotp = false;

    while (pos + 4 <= len) {
        uint32_t hdr = b[pos] | b[pos + 1] << 8 | b[pos + 2] << 16, fixed;
        uint8_t dt, vc;
        uint16_t wc;
        int ecc = esp32p4_mipi_ecc_check(hdr, b[pos + 3], &fixed);

        if (pck & ECC_RX_EN) {
            if (ecc == ESP32P4_MIPI_ECC_MULTI) {
                host_error(s, 1, ECC_MILTI_ERR);
                return;
            }
            if (ecc == ESP32P4_MIPI_ECC_CORRECTED) {
                host_error(s, 1, ECC_SINGLE_ERR);
                hdr = fixed;
            }
        }
        dt = hdr & 0x3f;
        vc = (hdr >> 6) & 3;
        wc = hdr >> 8;
        pos += 4;
        if (dt == 0x08) {
            eotp = true;
            continue;
        }
        if (dt == 0x02) {
            s->int_st[0] |= wc;
            host_update_irq(s);
            continue;
        }
        if (vc != (hreg(s, A_GEN_VCID) & 3)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: an answer on virtual channel "
                          "%u, not GEN_VCID_RX's %u: dropped\n",
                          TYPE_ESP32P4_DSI_HOST, vc,
                          hreg(s, A_GEN_VCID) & 3);
            if (dt == 0x1a || dt == 0x1c) {
                pos += wc + 2;
            }
            continue;
        }
        switch (dt) {
        case 0x11: case 0x21:
        case 0x12: case 0x22: {
            uint8_t d[2] = { wc & 0xff, wc >> 8 };

            rfifo_push(s, d, (dt & 0xf) == 1 ? 1 : 2);
            break;
        }
        case 0x1a: case 0x1c:
            if (pos + wc + 2 > len) {
                host_error(s, 1, PKT_SIZE_ERR);
                rfifo_push(s, b + pos, len - pos);
                pos = len;
                break;
            }
            if ((pck & CRC_RX_EN) &&
                (b[pos + wc] | b[pos + wc + 1] << 8) != 0 &&
                esp32p4_csi2_crc16(b + pos, wc) !=
                (b[pos + wc] | b[pos + wc + 1] << 8)) {
                host_error(s, 1, CRC_ERR);
            }
            rfifo_push(s, b + pos, wc);
            pos += wc + 2;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: the panel answered with "
                          "data type 0x%02x\n", TYPE_ESP32P4_DSI_HOST, dt);
            break;
        }
    }
    if (pos < len) {
        host_error(s, 1, PKT_SIZE_ERR);
    }
    if ((pck & EOTP_RX_EN) && len && !eotp) {
        host_error(s, 1, EOPT_ERR);
    }
}

/*
 * Turn the bus around at @t for the panel's answer: the time the answer
 * is in and the link back with the host, kept in @s->txn_end_ns.
 */
static void link_turnaround(ESP32P4DsiHostState *s, int64_t t)
{
    ESP32P4DsiPanelState *panel = host_panel(s);
    ESP32P4DsiResponse r = { .none = true };
    uint64_t esc;

    if (panel) {
        esp32p4_dsi_panel_turnaround(panel, &r, esc_after(s, t, LP_TA_ESC));
    }
    s->ans_none = r.none;
    s->ans_len = MIN(r.len, sizeof(s->ans));
    s->ans_lost = r.len - s->ans_len;
    if (r.len) {
        memcpy(s->ans, r.bytes, s->ans_len);
    }
    g_free(r.bytes);
    s->txn_kind = TXN_RESP;
    if (r.none) {
        uint32_t to = hreg(s, A_TO_CNT_CFG) & 0xffff;
        uint32_t div = extract32(hreg(s, A_CLKMGR_CFG), 8, 8);

        s->txn_end_ns = to && div ? byte_after(s, esc_after(s, t, LP_TA_ESC),
                                               (uint64_t)to * div)
                                  : INT64_MAX;
        return;
    }
    esc = 2 * LP_TA_ESC + (r.ack ? LP_ESC_OVERHEAD
                                 : LP_ESC_OVERHEAD +
                                   (uint64_t)LP_ESC_PER_BYTE * r.len);
    s->txn_end_ns = esc_after(s, t, esc);
}

static uint32_t wr_hold(ESP32P4DsiHostState *s, bool hs, bool read)
{
    if (read) {
        return hreg(s, hs ? A_HS_RD_TO_CNT : A_LP_RD_TO_CNT) & 0xffff;
    }
    return hreg(s, hs ? A_HS_WR_TO_CNT : A_LP_WR_TO_CNT) & 0xffff;
}

/* The transmission's phase ends at s->txn_end_ns. */
static void link_step(void *opaque)
{
    ESP32P4DsiHostState *s = opaque;
    int64_t t = s->txn_end_ns;
    uint8_t dt = s->txn_hdr & 0x3f;
    bool read = dsi_dt_read(dt), hs = !cmd_lp(s, dt);
    bool bta;

    switch (s->txn_kind) {
    case TXN_SEND: {
        ESP32P4DsiPacket p = {
            .vc = (s->txn_hdr >> 6) & 3,
            .dt = dt,
            .wc = s->txn_hdr >> 8,
            .long_packet = dsi_dt_long(dt),
            .hs = hs,
            .lanes = host_lanes(s),
            .end_ns = t,
        };
        g_autofree uint8_t *pld = NULL;

        bta = (hreg(s, A_PCKHDL_CFG) & BTA_EN) &&
              (read || (hreg(s, A_CMD_MODE_CFG) & ACK_RQST_EN));
        p.bta = bta;
        if (p.long_packet) {
            pld = g_malloc0(MAX(p.wc, 1));
            memcpy(pld, s->txn_pld, MIN(p.wc, s->txn_pld_len));
            p.payload = pld;
        }
        link_deliver(s, &p);
        if (bta) {
            link_turnaround(s, t);
            return;
        }
        s->txn = false;
        if (read) {
            s->rd_busy = false;
        }
        s->link_free_ns = byte_after(s, t, wr_hold(s, hs, read));
        break;
    }
    case TXN_RESP:
        if (s->ans_none) {
            host_error(s, 1, TO_LP_RX);
        } else {
            link_take_answer(s, s->ans, s->ans_len, read);
            if (s->ans_lost) {
                host_error(s, 1, GEN_PLD_RECEV_ERR);
            }
        }
        s->ans_len = 0;
        s->ans_lost = 0;
        s->txn = false;
        if (read) {
            s->rd_busy = false;
        }
        s->link_free_ns = byte_after(s, t,
                                     (hreg(s, A_BTA_TO_CNT) & 0xffff) +
                                     wr_hold(s, hs, read));
        break;
    case TXN_FRAME_ACK:
        link_turnaround(s, t);
        s->txn_hdr = 0;
        return;
    }
}

/* Whether the engine may start a queued packet now. */
static bool link_can_start(ESP32P4DsiHostState *s, int64_t now)
{
    return s->cmd_num && !s->txn && (hreg(s, A_PWR_UP) & 1) &&
           phy_ready(s) && !s->data_ulps && now >= s->trigger_end_ns &&
           (hreg(s, A_MODE_CFG) & 1);
}

/*
 * Start the packet at the command FIFO's head at @t: false if it cannot
 * finish by @deadline (video mode's blanking) or the escape clock is
 * stopped.
 */
static bool link_start(ESP32P4DsiHostState *s, int64_t t, int64_t deadline)
{
    uint32_t hdr = s->cmd_fifo[s->cmd_head];
    uint8_t dt = hdr & 0x3f;
    uint16_t wc = hdr >> 8;
    bool lng = dsi_dt_long(dt), lp = cmd_lp(s, dt);
    uint32_t bytes = lng ? 6u + wc : 4u;
    int64_t end;

    if (lp) {
        end = esc_after(s, t, lp_cycles(s, bytes));
        if (end == INT64_MAX) {
            static bool warned;

            if (!warned) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: an LP command with "
                              "TX_ESC_CLK stopped (TX_ESC_CLK_DIVISION below "
                              "2) waits\n", TYPE_ESP32P4_DSI_HOST);
                warned = true;
            }
            return false;
        }
    } else {
        end = byte_after(s, t, hs_cycles(s, bytes));
    }
    if (end > deadline) {
        return false;
    }
    s->cmd_head = (s->cmd_head + 1) % ESP32P4_DSI_GEN_CMD_DEPTH;
    s->cmd_num--;
    s->txn_pld_len = 0;
    if (lng) {
        uint32_t words = DIV_ROUND_UP(wc, 4);

        if (words > s->wpld_num) {
            host_error(s, 1, GEN_PLD_SEND_ERR);
        }
        for (uint32_t i = 0; i < words && s->wpld_num; i++) {
            uint32_t w = s->wpld[s->wpld_head];

            s->wpld_head = (s->wpld_head + 1) % ESP32P4_DSI_GEN_PLD_DEPTH;
            s->wpld_num--;
            if (s->txn_pld_len + 4 <= sizeof(s->txn_pld)) {
                stl_le_p(s->txn_pld + s->txn_pld_len, w);
                s->txn_pld_len += 4;
            }
        }
    }
    s->txn = true;
    s->txn_kind = TXN_SEND;
    s->txn_hdr = hdr;
    s->txn_end_ns = end;
    return true;
}

/*
 * Run the link up to the present: the transmissions' phases that have
 * ended, each at its own time, then what the FIFO holds while it may.
 */
static void link_run(ESP32P4DsiHostState *s)
{
    for (;;) {
        int64_t now = esp32_line_time();

        if (s->txn) {
            if (s->txn_end_ns > now) {
                if (s->txn_end_ns != INT64_MAX) {
                    timer_mod(s->link_timer, s->txn_end_ns);
                }
                return;
            }
            esp32_line_run_at(s->txn_end_ns, link_step, s);
            continue;
        }
        if (!link_can_start(s, now)) {
            return;
        }
        if (s->link_free_ns > now) {
            timer_mod(s->link_timer, s->link_free_ns);
            return;
        }
        if (!link_start(s, now, INT64_MAX)) {
            return;
        }
    }
}

static void link_timer_cb(void *opaque)
{
    link_run(opaque);
}

static void link_kick(ESP32P4DsiHostState *s)
{
    link_run(s);
}

/*
 * Video mode's blanking line from @t to @end: the queued packets that fit,
 * in turn; with @frame_ack first a turnaround for the frame's acknowledge.
 */
static void link_window(ESP32P4DsiHostState *s, int64_t t, int64_t end,
                        bool frame_ack)
{
    link_run(s);
    if (s->txn || !(hreg(s, A_PWR_UP) & 1) || !phy_ready(s) ||
        s->data_ulps) {
        return;
    }
    t = MAX(t, s->link_free_ns);
    if (frame_ack && (hreg(s, A_PCKHDL_CFG) & BTA_EN)) {
        s->txn = true;
        s->txn_kind = TXN_FRAME_ACK;
        s->txn_hdr = 0;
        s->txn_end_ns = t;
        link_run(s);
        return;
    }
    while (s->cmd_num && !s->txn && t < end) {
        if (!link_start(s, t, end)) {
            return;
        }
        link_run(s);
        t = MAX(t, s->link_free_ns);
    }
}

/* ---- Video mode ---- */

static bool video_on(ESP32P4DsiHostState *s)
{
    return (hreg(s, A_PWR_UP) & 1) && !(hreg(s, A_MODE_CFG) & 1) &&
           phy_ready(s) && !s->data_ulps;
}

static void video_stop(ESP32P4DsiHostState *s)
{
    s->video = false;
    s->vline = UINT32_MAX;
    timer_del(s->vpg_timer);
    s->vpg = false;
}

static void shadow_apply(ESP32P4DsiHostState *s)
{
    for (int i = 0; i < ARRAY_SIZE(shadowed); i++) {
        s->act[R(shadowed[i])] = s->regs[R(shadowed[i])];
    }
    s->shadow_pending = false;
    s->regs[R(A_VID_SHADOW_CTRL)] &= ~VID_SHADOW_REQ;
}

/* Bytes of DPI_COLOR_CODING's packets for @px pixels, or 0 if reserved. */
static uint32_t coding_bytes(uint32_t coding, bool loosely, uint32_t px)
{
    switch (coding) {
    case 0: case 1: case 2:
        return px * 2;
    case 3: case 4:
        return loosely ? px * 3 : DIV_ROUND_UP(px * 18, 8);
    case 5:
        return px * 3;
    default:
        return 0;
    }
}

static uint8_t coding_dt(uint32_t coding, bool loosely)
{
    switch (coding) {
    case 0: case 1: case 2:
        return 0x0e;
    case 3: case 4:
        return loosely ? 0x2e : 0x1e;
    default:
        return 0x3e;
    }
}

/* A DPI word's components as DPI_COLOR_CODING takes them (table 43.4-2). */
static void dpi_components(uint32_t coding, uint32_t w, uint32_t *r,
                           uint32_t *g, uint32_t *b)
{
    switch (coding) {
    case 0:
        *r = extract32(w, 11, 5);
        *g = extract32(w, 5, 6);
        *b = extract32(w, 0, 5);
        break;
    case 1:
        *r = extract32(w, 16, 5);
        *g = extract32(w, 8, 6);
        *b = extract32(w, 0, 5);
        break;
    case 2:
        *r = extract32(w, 17, 5);
        *g = extract32(w, 8, 6);
        *b = extract32(w, 1, 5);
        break;
    case 3:
        *r = extract32(w, 12, 6);
        *g = extract32(w, 6, 6);
        *b = extract32(w, 0, 6);
        break;
    case 4:
        *r = extract32(w, 16, 6);
        *g = extract32(w, 8, 6);
        *b = extract32(w, 0, 6);
        break;
    default:
        *r = extract32(w, 16, 8);
        *g = extract32(w, 8, 8);
        *b = extract32(w, 0, 8);
        break;
    }
}

/* Components into a DPI word of DPI_COLOR_CODING's arrangement. */
static uint32_t dpi_word(uint32_t coding, uint32_t r, uint32_t g, uint32_t b)
{
    switch (coding) {
    case 0:
        return r << 11 | g << 5 | b;
    case 1:
        return r << 16 | g << 8 | b;
    case 2:
        return r << 17 | g << 8 | b << 1;
    case 3:
        return r << 12 | g << 6 | b;
    case 4:
        return r << 16 | g << 8 | b;
    default:
        return r << 16 | g << 8 | b;
    }
}

/*
 * Pack @n DPI words into the payload of DPI_COLOR_CODING's packet type
 * (MIPI DSI 1.1 8.8.20 to 8.8.23: components LSB first, red first).
 */
static uint32_t pack_pixels(uint32_t coding, bool loosely, const uint32_t *px,
                            uint32_t n, uint8_t *out)
{
    uint32_t len = coding_bytes(coding, loosely, n), r, g, b;

    memset(out, 0, len);
    for (uint32_t i = 0; i < n; i++) {
        dpi_components(coding, px[i], &r, &g, &b);
        switch (coding) {
        case 0: case 1: case 2: {
            uint16_t v = r | g << 5 | b << 11;

            out[2 * i] = v;
            out[2 * i + 1] = v >> 8;
            break;
        }
        case 3: case 4:
            if (loosely) {
                out[3 * i] = r << 2;
                out[3 * i + 1] = g << 2;
                out[3 * i + 2] = b << 2;
            } else {
                uint32_t v = r | g << 6 | b << 12, bit = 18 * i;

                for (int k = 0; k < 18; k++, bit++) {
                    if (v & BIT(k)) {
                        out[bit / 8] |= BIT(bit % 8);
                    }
                }
            }
            break;
        default:
            out[3 * i] = r;
            out[3 * i + 1] = g;
            out[3 * i + 2] = b;
            break;
        }
    }
    return len;
}

static void video_short(ESP32P4DsiHostState *s, uint8_t dt, int64_t end)
{
    ESP32P4DsiPacket p = {
        .vc = hact(s, A_DPI_VCID) & 3,
        .dt = dt,
        .hs = true,
        .lanes = host_lanes(s),
        .end_ns = end,
    };

    link_deliver(s, &p);
}

/* The host's line @s->vline of the frame starts at @t, ending at @next. */
static void video_line(ESP32P4DsiHostState *s, int64_t t, int64_t next)
{
    uint32_t vsa = hact(s, A_VID_VSA_LINES), vbp = hact(s, A_VID_VBP_LINES);
    uint32_t vact = hact(s, A_VID_VACTIVE_LINES);
    uint32_t type = hact(s, A_VID_MODE_CFG) & 3, l = s->vline;
    uint64_t sync = (hreg(s, A_PHY_TMR_CFG) & 0x3ff) +
                    DIV_ROUND_UP(4, host_lanes(s));
    int64_t sync_end = byte_after(s, t, sync);

    s->vline_ns = t;
    s->vline_next_ns = next;
    if (l == 0) {
        video_short(s, 0x01, sync_end);
    } else if (type == 0 && l == vsa) {
        video_short(s, 0x11, sync_end);
    } else {
        video_short(s, 0x21, sync_end);
    }
    if (type == 0) {
        video_short(s, 0x31, byte_after(s, t, hact(s, A_VID_HSA_TIME) +
                                               sync));
    }
    if (l < vsa + vbp || l >= vsa + vbp + vact) {
        link_window(s, sync_end, next,
                    l == vsa + vbp + vact &&
                    (hact(s, A_VID_MODE_CFG) & FRAME_BTA_ACK_EN));
    }
}

static void video_frame_start(ESP32P4DsiHostState *s)
{
    if (s->shadow_pending) {
        shadow_apply(s);
    }
    s->vline = 0;
    s->vframe++;
    s->frame_warned = false;
}

static void video_frame_error(ESP32P4DsiHostState *s, uint32_t bit)
{
    if (!(s->frame_warned)) {
        host_error(s, 1, bit);
    }
    s->frame_warned = true;
}

/*
 * A line's pixels, @n DPI words, sent in its packets as the last of them
 * is in at @end; false if they do not fit in the line.
 */
static void video_pixels(ESP32P4DsiHostState *s, const uint32_t *px,
                         uint32_t n, int64_t end, const uint8_t *raw,
                         uint32_t raw_len)
{
    uint32_t coding = hact(s, A_DPI_COLOR_CODING) & 0xf;
    bool loosely = hact(s, A_DPI_COLOR_CODING) & BIT(8);
    uint32_t size = hact(s, A_VID_PKT_SIZE);
    uint32_t chunks = MAX(hact(s, A_VID_NUM_CHUNKS), 1);
    uint32_t null = hact(s, A_VID_NULL_SIZE);
    uint32_t total = size * chunks, bytes, cyc;
    uint8_t dt = coding_dt(coding, loosely);
    g_autofree uint8_t *buf = NULL;
    g_autofree uint32_t *line = NULL;
    int64_t t;

    if (coding > 5) {
        static bool warned;

        if (!warned) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DPI_COLOR_CODING %u is "
                          "reserved: no video\n", TYPE_ESP32P4_DSI_HOST,
                          coding);
            warned = true;
        }
        return;
    }
    if (!raw && n < total) {
        video_frame_error(s, DPI_BUFF_PLD_UNDER);
    } else if (!raw && n > total) {
        video_frame_error(s, DPI_PLD_WR_ERR);
    }
    if ((coding == 3 || coding == 4) && !loosely && (size & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: VID_PKT_SIZE %u is not a multiple "
                      "of 4 for packed 18-bit pixels\n",
                      TYPE_ESP32P4_DSI_HOST, size);
    }
    bytes = raw ? raw_len : coding_bytes(coding, loosely, size);
    cyc = (hact(s, A_VID_HSA_TIME) + hact(s, A_VID_HBP_TIME)) +
          (hreg(s, A_PHY_TMR_CFG) & 0x3ff) +
          DIV_ROUND_UP(chunks * (bytes + 6) + (chunks - 1) * (null ? null + 6
                                                                   : 0) +
                       ((hreg(s, A_PCKHDL_CFG) & EOTP_TX_EN) ? 4 : 0),
                       host_lanes(s));
    if (byte_after(s, s->vline_ns, cyc) > s->vline_next_ns) {
        video_frame_error(s, DPI_PLD_WR_ERR);
        return;
    }
    line = g_new0(uint32_t, MAX(total, 1));
    memcpy(line, px, MIN(n, total) * sizeof(uint32_t));
    buf = g_malloc(MAX(bytes, 1) + 16);
    t = MAX(end, byte_after(s, s->vline_ns, cyc));
    for (uint32_t c = 0; c < chunks; c++) {
        ESP32P4DsiPacket p = {
            .vc = hact(s, A_DPI_VCID) & 3,
            .dt = dt,
            .long_packet = true,
            .hs = true,
            .lanes = host_lanes(s),
            .end_ns = t,
        };

        if (raw) {
            p.wc = raw_len;
            p.payload = raw;
        } else {
            p.wc = pack_pixels(coding, loosely, line + c * size, size, buf);
            p.payload = buf;
        }
        link_deliver(s, &p);
        if (raw) {
            break;
        }
    }
}

void esp32p4_dsi_host_dpi_hsync(ESP32P4DsiHostState *s, uint32_t line,
                                bool vsync, int64_t start_ns,
                                int64_t next_ns)
{
    link_run(s);
    if (!video_on(s) || s->vpg) {
        s->video = false;
        return;
    }
    if (hreg(s, A_DPI_CFG_POL) & 0x7) {
        if (!s->polarity_warned) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: DPI_CFG_POL takes the "
                          "bridge's HSYNC, VSYNC or DE as active low: the "
                          "host finds no video timing\n",
                          TYPE_ESP32P4_DSI_HOST);
            s->polarity_warned = true;
        }
        return;
    }
    if (line == 0 && vsync) {
        s->video = true;
        video_frame_start(s);
    } else if (!s->video || s->vline == UINT32_MAX) {
        return;
    } else {
        s->vline++;
    }
    video_line(s, start_ns, next_ns);
}

void esp32p4_dsi_host_dpi_pixels(ESP32P4DsiHostState *s, uint32_t line,
                                 const uint32_t *pixels, uint32_t count,
                                 int64_t active_ns, int64_t end_ns)
{
    uint32_t vsa = hact(s, A_VID_VSA_LINES), vbp = hact(s, A_VID_VBP_LINES);
    uint32_t vact = hact(s, A_VID_VACTIVE_LINES);

    if (!s->video || !video_on(s) || s->vpg || s->vline == UINT32_MAX) {
        return;
    }
    if (s->vline < vsa + vbp || s->vline >= vsa + vbp + vact) {
        video_frame_error(s, DPI_PLD_WR_ERR);
        return;
    }
    video_pixels(s, pixels, count, end_ns, NULL, 0);
}

void esp32p4_dsi_host_dpi_stop(ESP32P4DsiHostState *s, int64_t when_ns)
{
    if (!s->vpg) {
        s->video = false;
        s->vline = UINT32_MAX;
    }
}

void esp32p4_dsi_host_dpi_ctl(ESP32P4DsiHostState *s, uint32_t lcd_ctl)
{
    uint32_t pol = hreg(s, A_DPI_CFG_POL);
    bool shut = !!(lcd_ctl & BIT(0)) != !!(pol & BIT(3));
    bool colorm = !!(lcd_ctl & BIT(1)) != !!(pol & BIT(4));
    bool was_shut = !!(s->dpi_lcd_ctl & BIT(0)) != !!(pol & BIT(3));
    bool was_colorm = !!(s->dpi_lcd_ctl & BIT(1)) != !!(pol & BIT(4));
    int64_t now = esp32_line_time();

    s->dpi_lcd_ctl = lcd_ctl;
    if (!video_on(s)) {
        return;
    }
    if (shut != was_shut) {
        video_short(s, shut ? 0x22 : 0x32,
                    byte_after(s, now, hs_cycles(s, 4)));
    }
    if (colorm != was_colorm) {
        video_short(s, colorm ? 0x12 : 0x02,
                    byte_after(s, now, hs_cycles(s, 4)));
    }
    if (lcd_ctl & BIT(2)) {
        qemu_log_mask(LOG_UNIMP, "%s: dpiupdatecfg is not modelled\n",
                      TYPE_ESP32P4_DSI_HOST);
    }
}

/* ---- The video pattern generator ---- */

static uint32_t vpg_lines(ESP32P4DsiHostState *s)
{
    return hact(s, A_VID_VSA_LINES) + hact(s, A_VID_VBP_LINES) +
           hact(s, A_VID_VACTIVE_LINES) + hact(s, A_VID_VFP_LINES);
}

static void vpg_arm(ESP32P4DsiHostState *s)
{
    timer_mod(s->vpg_timer, s->vline_next_ns);
}

/* The colour bar or BER line of the pattern, sent at the line's end. */
static void vpg_pixels(ESP32P4DsiHostState *s, uint32_t y)
{
    uint32_t coding = hact(s, A_DPI_COLOR_CODING) & 0xf;
    bool loosely = hact(s, A_DPI_COLOR_CODING) & BIT(8);
    uint32_t w = hact(s, A_VID_PKT_SIZE) *
                 MAX(hact(s, A_VID_NUM_CHUNKS), 1);
    uint32_t h = hact(s, A_VID_VACTIVE_LINES), cmode = hact(s, A_VID_MODE_CFG);
    uint32_t rmax, gmax, bmax;
    int64_t end = s->vline_next_ns;
    g_autofree uint32_t *px = g_new0(uint32_t, MAX(w, 1));

    dpi_components(coding, 0xffffffff, &rmax, &gmax, &bmax);
    if (cmode & VPG_MODE) {
        static const uint8_t ber[8] = {
            0xaa, 0x33, 0xf0, 0x7f, 0x55, 0xcc, 0x0f, 0x80,
        };
        uint32_t len = coding_bytes(coding, loosely, w), x = len / 8;
        g_autofree uint8_t *b = g_malloc(MAX(len, 1));

        for (uint32_t i = 0; i < len; i++) {
            b[i] = ber[x ? MIN(i / x, 7) : 7];
        }
        video_pixels(s, NULL, 0, end, b, len);
        return;
    }
    for (uint32_t x = 0; x < w; x++) {
        /* White, yellow, cyan, green, magenta, red, blue, black. */
        static const uint8_t rgb[8] = { 7, 6, 3, 2, 5, 4, 1, 0 };
        uint32_t pos = cmode & VPG_ORIENTATION ? y : x;
        uint32_t size = cmode & VPG_ORIENTATION ? h : w;
        uint32_t bar = size >= 8 ? MIN(pos / (size / 8), 7) : MIN(pos, 7);

        px[x] = dpi_word(coding, rgb[bar] & 4 ? rmax : 0,
                         rgb[bar] & 2 ? gmax : 0, rgb[bar] & 1 ? bmax : 0);
    }
    video_pixels(s, px, w, end, NULL, 0);
}

static void vpg_line(void *opaque)
{
    ESP32P4DsiHostState *s = opaque;
    uint32_t lines = vpg_lines(s), act0;
    int64_t t = s->vline_next_ns;

    if (!s->vpg) {
        return;
    }
    s->vline++;
    if (s->vline >= lines) {
        video_frame_start(s);
    }
    lines = vpg_lines(s);
    act0 = hact(s, A_VID_VSA_LINES) + hact(s, A_VID_VBP_LINES);
    video_line(s, t, byte_after(s, t, hact(s, A_VID_HLINE_TIME)));
    if (s->vline >= act0 && s->vline < act0 + hact(s, A_VID_VACTIVE_LINES)) {
        vpg_pixels(s, s->vline - act0);
    }
    vpg_arm(s);
}

static void vpg_timer_cb(void *opaque)
{
    ESP32P4DsiHostState *s = opaque;

    esp32_line_run_at(s->vline_next_ns, vpg_line, s);
}

/* Start or stop the pattern generator as video mode and VPG_EN say. */
static void vpg_update(ESP32P4DsiHostState *s)
{
    bool want = video_on(s) && (hact(s, A_VID_MODE_CFG) & VPG_EN);
    int64_t now = esp32_line_time();

    if (want == s->vpg) {
        return;
    }
    s->vpg = want;
    if (!want) {
        timer_del(s->vpg_timer);
        s->video = false;
        s->vline = UINT32_MAX;
        return;
    }
    if (!hact(s, A_VID_HLINE_TIME) || !vpg_lines(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the pattern generator needs "
                      "HLINE_TIME and the vertical timing\n",
                      TYPE_ESP32P4_DSI_HOST);
        s->vpg = false;
        return;
    }
    /* The frame's first line starts now. */
    s->video = true;
    s->vline = vpg_lines(s) - 1;
    s->vline_ns = now;
    s->vline_next_ns = now;
    vpg_line(s);
}

/* ---- ULPS and triggers ---- */

static void host_ulps(ESP32P4DsiHostState *s, uint32_t v)
{
    int64_t now = esp32_line_time();
    int64_t entry = esc_after(s, now, LP_ESC_OVERHEAD);

    if (!phy_ready(s)) {
        return;
    }
    if ((v & TXREQULPSLAN) && !(v & TXEXITULPSLAN) && !s->data_ulps) {
        s->data_ulps = true;
        s->ulps_due_ns = entry;
    } else if (!(v & TXREQULPSLAN) && s->data_ulps) {
        s->data_ulps = false;
    }
    if ((v & TXREQULPSCLK) && !(v & TXEXITULPSCLK) && !s->clk_ulps) {
        s->clk_ulps = true;
        s->ulps_due_ns = entry;
    } else if (!(v & TXREQULPSCLK) && s->clk_ulps) {
        s->clk_ulps = false;
    }
    link_kick(s);
}

static void host_triggers(ESP32P4DsiHostState *s, uint32_t v)
{
    int64_t now = esp32_line_time();
    ESP32P4DsiPanelState *panel = host_panel(s);

    if (!v || !phy_ready(s) || s->data_ulps) {
        return;
    }
    if (ctpop32(v) > 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: PHY_TX_TRIGGERS 0x%x asks for "
                      "more than one trigger\n", TYPE_ESP32P4_DSI_HOST, v);
        return;
    }
    s->trigger_end_ns = esc_after(s, MAX(now, s->link_free_ns),
                                  LP_ESC_OVERHEAD);
    if (s->trigger_end_ns == INT64_MAX) {
        s->trigger_end_ns = now;
        return;
    }
    if (panel) {
        esp32p4_dsi_panel_trigger(panel, v, s->trigger_end_ns);
    }
}

/* ---- The host's registers ---- */

static int64_t brg_next_due(ESP32P4DsiBridgeState *s);
static void brg_catch_up(ESP32P4DsiBridgeState *s, int64_t now);

static void link_run_cb(void *opaque)
{
    link_run(opaque);
}

/*
 * A register access at esp32_line_time(): first what fell due before it,
 * the PHY's start-up, the link's phases, the pattern generator's lines
 * and the bridge's DPI events (its lines' syncs and pixels, the "bridge"
 * the bridge's "dsi-host" link sets), in order, each at its own time,
 * however late their QEMU timers run. Else a change of mode (MODE_CFG,
 * PWR_UP) would take the host out of video before the lines the DPI had
 * sent by then, which would then find it out, and the panel never get
 * them.
 */
static void host_sync(ESP32P4DsiHostState *s)
{
    int64_t now = esp32_line_time();
    QEMUTimer *timers[3] = { s->phy_timer, s->link_timer, s->vpg_timer };
    void (*const fns[3])(void *) = {
        (void (*)(void *))phy_update, link_run_cb, vpg_line,
    };

    for (;;) {
        int64_t due = INT64_MAX, dpi;
        int first = -1;

        for (int i = 0; i < 3; i++) {
            int64_t t = timer_expire_time_ns(timers[i]);

            if (t >= 0 && t <= now && t < due) {
                due = t;
                first = i;
            }
        }
        dpi = s->bridge ? brg_next_due(s->bridge) : INT64_MAX;
        if (dpi <= now && dpi < due) {
            brg_catch_up(s->bridge, dpi);
            continue;
        }
        if (first < 0) {
            break;
        }
        timer_del(timers[first]);
        esp32_line_run_at(due, fns[first], s);
    }
    link_run(s);
}

static uint32_t cmd_pkt_status(ESP32P4DsiHostState *s)
{
    uint32_t st = 0;
    bool building = s->txn && s->txn_kind == TXN_SEND;

    st |= !s->cmd_num ? BIT(0) : 0;
    st |= s->cmd_num == ESP32P4_DSI_GEN_CMD_DEPTH ? BIT(1) : 0;
    st |= !s->wpld_num ? BIT(2) : 0;
    st |= s->wpld_num == ESP32P4_DSI_GEN_PLD_DEPTH ? BIT(3) : 0;
    st |= !s->rpld_num ? BIT(4) : 0;
    st |= s->rpld_num == ESP32P4_DSI_GEN_PLD_DEPTH ? BIT(5) : 0;
    st |= s->rd_busy ? BIT(6) : 0;
    st |= !building ? BIT(16) : 0;
    st |= !building || !dsi_dt_long(s->txn_hdr & 0x3f) ? BIT(18) : 0;
    return st;
}

static uint32_t vid_mode_cfg_act(uint32_t v)
{
    return (v & 3) | extract32(v, 8, 8) << 2;
}

static uint64_t host_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4DsiHostState *s = opaque;
    const DsiReg *r = host_reg(addr);
    uint32_t v;

    host_sync(s);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DSI_HOST, addr);
        return 0;
    }
    switch (addr) {
    case A_GEN_PLD_DATA:
        if (!s->rpld_num) {
            host_error(s, 1, GEN_PLD_RD_ERR);
            return 0;
        }
        v = s->rpld[s->rpld_head];
        s->rpld_head = (s->rpld_head + 1) % ESP32P4_DSI_GEN_PLD_DEPTH;
        s->rpld_num--;
        return v;
    case A_CMD_PKT_STATUS:
        return cmd_pkt_status(s);
    case A_PHY_STATUS:
        return phy_status(s);
    case A_PHY_TST_CTRL0:
        return s->test.ctrl0;
    case A_PHY_TST_CTRL1:
        return esp32p4_dphy_test_ctrl1(&s->test, phy_dout(s));
    case A_INT_ST0:
    case A_INT_ST1: {
        int n = addr == A_INT_ST1;

        v = s->int_st[n] | hreg(s, n ? A_INT_FORCE1 : A_INT_FORCE0);
        s->int_st[n] = 0;
        host_update_irq(s);
        return v;
    }
    case A_DPI_VCID_ACT:
        return hact(s, A_DPI_VCID);
    case A_DPI_COLOR_CODING_ACT:
        return hact(s, A_DPI_COLOR_CODING);
    case A_DPI_LP_CMD_TIM_ACT:
        return hact(s, A_DPI_LP_CMD_TIM);
    case A_VID_MODE_CFG_ACT:
        return vid_mode_cfg_act(hact(s, A_VID_MODE_CFG));
    case A_VID_PKT_SIZE_ACT ... A_VID_VACTIVE_LINES_ACT:
        return s->act[R(addr - 0x100)];
    case A_VID_PKT_STATUS:
        return 0x50005;
    case A_SDF_3D_ACT:
        return hact(s, A_SDF_3D);
    default:
        return s->regs[R(addr)];
    }
}

static void host_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4DsiHostState *s = opaque;
    const DsiReg *r = host_reg(addr);
    uint32_t v = value, old;
    uint8_t data;

    host_sync(s);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DSI_HOST, addr);
        return;
    }
    if (r->kind == K_RO) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DSI_HOST, addr);
        return;
    }
    if (r->kind == K_UNIMP && (v & r->mask)) {
        qemu_log_mask(LOG_UNIMP, "%s: register 0x%" HWADDR_PRIx " (the DBI, "
                      "eDPI, DSC or skew calibration) is not modelled\n",
                      TYPE_ESP32P4_DSI_HOST, addr);
    }
    old = s->regs[R(addr)];
    s->regs[R(addr)] = v & r->mask;
    if (addr == A_SDF_3D && (v & r->mask)) {
        qemu_log_mask(LOG_UNIMP, "%s: 3D video is not modelled\n",
                      TYPE_ESP32P4_DSI_HOST);
    }
    for (int i = 0; i < ARRAY_SIZE(shadowed); i++) {
        if (shadowed[i] == addr &&
            !(hreg(s, A_VID_SHADOW_CTRL) & VID_SHADOW_EN)) {
            s->act[R(addr)] = s->regs[R(addr)];
        }
    }
    switch (addr) {
    case A_PWR_UP:
        if (!(v & 1) && (old & 1)) {
            /* The core's soft reset: the FIFOs and the link start over. */
            s->cmd_num = s->wpld_num = s->rpld_num = 0;
            s->txn = false;
            s->rd_busy = false;
            video_stop(s);
        }
        link_kick(s);
        vpg_update(s);
        break;
    case A_MODE_CFG:
        if ((v ^ old) & 1) {
            video_stop(s);
        }
        link_kick(s);
        vpg_update(s);
        break;
    case A_VID_MODE_CFG:
        vpg_update(s);
        break;
    case A_CMD_MODE_CFG:
        if (v & TEAR_FX_EN) {
            qemu_log_mask(LOG_UNIMP, "%s: tearing effect acknowledges are "
                          "not modelled\n", TYPE_ESP32P4_DSI_HOST);
        }
        break;
    case A_GEN_HDR:
        if (s->cmd_num == ESP32P4_DSI_GEN_CMD_DEPTH) {
            host_error(s, 1, GEN_CMD_WR_ERR);
            break;
        }
        s->cmd_fifo[(s->cmd_head + s->cmd_num++) %
                    ESP32P4_DSI_GEN_CMD_DEPTH] = v & 0xffffff;
        if (dsi_dt_read(v & 0x3f)) {
            s->rd_busy = true;
        }
        link_kick(s);
        break;
    case A_GEN_PLD_DATA:
        if (s->wpld_num == ESP32P4_DSI_GEN_PLD_DEPTH) {
            host_error(s, 1, GEN_PLD_WR_ERR);
            break;
        }
        s->wpld[(s->wpld_head + s->wpld_num++) % ESP32P4_DSI_GEN_PLD_DEPTH] =
            v;
        break;
    case A_PHY_RSTZ:
        phy_update(s);
        link_kick(s);
        break;
    case A_PHY_ULPS_CTRL:
        host_ulps(s, v);
        break;
    case A_PHY_TX_TRIGGERS:
        host_triggers(s, v & 0xf);
        break;
    case A_PHY_TST_CTRL0:
    case A_PHY_TST_CTRL1:
        switch (esp32p4_dphy_test_write(&s->test, addr == A_PHY_TST_CTRL1, v,
                                        &data)) {
        case ESP32P4_DPHY_TEST_CLEAR:
            phy_test_defaults(s);
            break;
        case ESP32P4_DPHY_TEST_DATA: {
            uint8_t before[6] = { s->t10, s->t17, s->t18[0], s->t18[1],
                                  s->t19, s->t44 };
            uint8_t after[6];

            phy_test_data(s, data);
            after[0] = s->t10;
            after[1] = s->t17;
            after[2] = s->t18[0];
            after[3] = s->t18[1];
            after[4] = s->t19;
            after[5] = s->t44;
            if (memcmp(before, after, sizeof(after)) &&
                s->phy_state != PHY_OFF) {
                /* The PLL's dividers or range changed: it locks again. */
                s->phy_state = PHY_OFF;
                timer_del(s->phy_timer);
                phy_set_byte_out(s, esp32p4_clk_edges_stopped());
                s->byte_edges = esp32p4_clk_edges_stopped();
                video_stop(s);
                phy_update(s);
            }
            break;
        }
        default:
            break;
        }
        s->regs[R(A_PHY_TST_CTRL0)] = s->test.ctrl0;
        s->regs[R(A_PHY_TST_CTRL1)] = s->test.ctrl1;
        break;
    case A_INT_MSK0:
    case A_INT_MSK1:
    case A_INT_FORCE0:
    case A_INT_FORCE1:
        host_update_irq(s);
        break;
    case A_VID_SHADOW_CTRL:
        if (v & VID_SHADOW_PIN_REQ) {
            qemu_log_mask(LOG_UNIMP, "%s: VID_SHADOW_PIN_REQ: the request "
                          "pin is not modelled\n", TYPE_ESP32P4_DSI_HOST);
        }
        if (!(v & VID_SHADOW_EN)) {
            shadow_apply(s);
        } else if (v & VID_SHADOW_REQ) {
            if (s->video) {
                s->shadow_pending = true;
            } else {
                shadow_apply(s);
            }
        }
        vpg_update(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps host_ops = {
    .read = host_read,
    .write = host_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---- The host's device ---- */

static void host_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4DsiHostState *s = opaque;

    if (event == ClockPreUpdate) {
        link_run(s);
        return;
    }
    phy_update(s);
}

static void host_cfg_ref_en(void *opaque, int n, int level)
{
    ESP32P4DsiHostState *s = opaque;

    s->cfg_ref_en = level;
    phy_update(s);
}

/* [spec:nuos:req:emu.esp32p4.dphy-supply] VDD_MIPI_DPHY, @level mV. */
static void host_vdd(void *opaque, int n, int level)
{
    ESP32P4DsiHostState *s = opaque;

    s->vdd_mv = MAX(level, 0);
    if (s->vdd_mv > ESP32P4_DPHY_VDD_MAX_MV && !s->vdd_warned) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: VDD_MIPI_DPHY at %u mV, beyond "
                      "the 2.75 V the chip allows: the PHY does not work\n",
                      TYPE_ESP32P4_DSI_HOST, s->vdd_mv);
        s->vdd_warned = true;
    }
    phy_update(s);
}

void esp32p4_dsi_host_set_connector_i2c(ESP32P4DsiHostState *s, I2CBus *bus)
{
    s->connector_i2c = bus;
}

static bool host_exact_edges(Object *obj, Clock *out, ESP32P4ClkEdges *e)
{
    ESP32P4DsiHostState *s = ESP32P4_DSI_HOST(obj);

    if (out != s->lanebyteclk) {
        return false;
    }
    *e = s->byte_out;
    return true;
}

static void host_reset_hold(Object *obj, ResetType type)
{
    ESP32P4DsiHostState *s = ESP32P4_DSI_HOST(obj);

    for (int i = 0; i < ARRAY_SIZE(host_regs); i++) {
        s->regs[R(host_regs[i].off)] = host_regs[i].reset;
    }
    memcpy(s->act, s->regs, sizeof(s->act));
    s->int_st[0] = s->int_st[1] = 0;
    esp32p4_dphy_test_reset(&s->test);
    phy_test_defaults(s);
    s->phy_state = PHY_OFF;
    timer_del(s->phy_timer);
    timer_del(s->link_timer);
    timer_del(s->vpg_timer);
    s->byte_edges = esp32p4_clk_edges_stopped();
    s->data_ulps = s->clk_ulps = false;
    s->ulps_due_ns = 0;
    s->trigger_end_ns = 0;
    s->warned_range = false;
    s->cmd_head = s->cmd_num = 0;
    s->wpld_head = s->wpld_num = 0;
    s->rpld_head = s->rpld_num = 0;
    s->txn = false;
    s->rd_busy = false;
    s->link_free_ns = 0;
    s->video = false;
    s->vpg = false;
    s->vline = UINT32_MAX;
    s->vframe = 0;
    s->shadow_pending = false;
    s->frame_warned = false;
    s->polarity_warned = false;
    s->dpi_lcd_ctl = 0;
}

static void host_reset_exit(Object *obj, ResetType type)
{
    ESP32P4DsiHostState *s = ESP32P4_DSI_HOST(obj);

    phy_set_byte_out(s, esp32p4_clk_edges_stopped());
    host_update_irq(s);
}

static void host_init(Object *obj)
{
    ESP32P4DsiHostState *s = ESP32P4_DSI_HOST(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &host_ops, s, "esp32p4.dsi-host",
                          ESP32P4_DSI_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk_cfg = qdev_init_clock_in(dev, "phy-cfg", host_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->clk_ref = qdev_init_clock_in(dev, "phy-ref", host_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->lanebyteclk = qdev_init_clock_out(dev, "lanebyteclk");
    qdev_init_gpio_in_named(dev, host_cfg_ref_en, ESP32P4_DSI_CFG_REF_CLK_EN,
                            1);
    qdev_init_gpio_in_named(dev, host_vdd, ESP32P4_VDD_MIPI_DPHY, 1);
    s->phy_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, phy_timer_cb, s);
    s->link_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, link_timer_cb, s);
    s->vpg_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vpg_timer_cb, s);
    qbus_init(&s->bus, sizeof(s->bus), TYPE_ESP32P4_DSI_BUS, dev, "dsi");
    s->cfg_ref_en = true;
    s->byte_out = esp32p4_clk_edges_stopped();
}

static void host_finalize(Object *obj)
{
    ESP32P4DsiHostState *s = ESP32P4_DSI_HOST(obj);

    timer_free(s->phy_timer);
    timer_free(s->link_timer);
    timer_free(s->vpg_timer);
}

static const VMStateDescription vmstate_host = {
    .name = TYPE_ESP32P4_DSI_HOST,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(phy_timer, ESP32P4DsiHostState),
        VMSTATE_TIMER_PTR(link_timer, ESP32P4DsiHostState),
        VMSTATE_TIMER_PTR(vpg_timer, ESP32P4DsiHostState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4DsiHostState,
                             ESP32P4_DSI_HOST_NREGS),
        VMSTATE_UINT32_ARRAY(int_st, ESP32P4DsiHostState, 2),
        VMSTATE_ESP32P4_DPHY_TEST(test, ESP32P4DsiHostState),
        VMSTATE_UINT8(t10, ESP32P4DsiHostState),
        VMSTATE_UINT8(t11, ESP32P4DsiHostState),
        VMSTATE_UINT8(t12, ESP32P4DsiHostState),
        VMSTATE_UINT8(t17, ESP32P4DsiHostState),
        VMSTATE_UINT8_ARRAY(t18, ESP32P4DsiHostState, 2),
        VMSTATE_UINT8(t19, ESP32P4DsiHostState),
        VMSTATE_UINT8_ARRAY(t22, ESP32P4DsiHostState, 2),
        VMSTATE_UINT8(t44, ESP32P4DsiHostState),
        VMSTATE_BOOL(cfg_ref_en, ESP32P4DsiHostState),
        VMSTATE_UINT32(vdd_mv, ESP32P4DsiHostState),
        VMSTATE_BOOL(vdd_warned, ESP32P4DsiHostState),
        VMSTATE_UINT32(phy_state, ESP32P4DsiHostState),
        VMSTATE_INT64(phy_due_ns, ESP32P4DsiHostState),
        VMSTATE_ESP32P4_CLK_EDGES(ref_edges, ESP32P4DsiHostState),
        VMSTATE_ESP32P4_CLK_EDGES(byte_edges, ESP32P4DsiHostState),
        VMSTATE_ESP32P4_CLK_EDGES(byte_out, ESP32P4DsiHostState),
        VMSTATE_UINT64(lane_mbps_x1000, ESP32P4DsiHostState),
        VMSTATE_BOOL(data_ulps, ESP32P4DsiHostState),
        VMSTATE_BOOL(clk_ulps, ESP32P4DsiHostState),
        VMSTATE_INT64(ulps_due_ns, ESP32P4DsiHostState),
        VMSTATE_INT64(trigger_end_ns, ESP32P4DsiHostState),
        VMSTATE_BOOL(warned_range, ESP32P4DsiHostState),
        VMSTATE_UINT32_ARRAY(cmd_fifo, ESP32P4DsiHostState,
                             ESP32P4_DSI_GEN_CMD_DEPTH),
        VMSTATE_UINT32(cmd_head, ESP32P4DsiHostState),
        VMSTATE_UINT32(cmd_num, ESP32P4DsiHostState),
        VMSTATE_UINT32_ARRAY(wpld, ESP32P4DsiHostState,
                             ESP32P4_DSI_GEN_PLD_DEPTH),
        VMSTATE_UINT32(wpld_head, ESP32P4DsiHostState),
        VMSTATE_UINT32(wpld_num, ESP32P4DsiHostState),
        VMSTATE_UINT32_ARRAY(rpld, ESP32P4DsiHostState,
                             ESP32P4_DSI_GEN_PLD_DEPTH),
        VMSTATE_UINT32(rpld_head, ESP32P4DsiHostState),
        VMSTATE_UINT32(rpld_num, ESP32P4DsiHostState),
        VMSTATE_BOOL(txn, ESP32P4DsiHostState),
        VMSTATE_UINT32(txn_hdr, ESP32P4DsiHostState),
        VMSTATE_UINT32(txn_kind, ESP32P4DsiHostState),
        VMSTATE_INT64(txn_end_ns, ESP32P4DsiHostState),
        VMSTATE_INT64(link_free_ns, ESP32P4DsiHostState),
        VMSTATE_UINT8_ARRAY(txn_pld, ESP32P4DsiHostState,
                            ESP32P4_DSI_GEN_PLD_DEPTH * 4),
        VMSTATE_UINT32(txn_pld_len, ESP32P4DsiHostState),
        VMSTATE_BOOL(rd_busy, ESP32P4DsiHostState),
        VMSTATE_BOOL(video, ESP32P4DsiHostState),
        VMSTATE_BOOL(vpg, ESP32P4DsiHostState),
        VMSTATE_UINT32(vline, ESP32P4DsiHostState),
        VMSTATE_UINT32(vframe, ESP32P4DsiHostState),
        VMSTATE_INT64(vline_ns, ESP32P4DsiHostState),
        VMSTATE_INT64(vline_next_ns, ESP32P4DsiHostState),
        VMSTATE_UINT32_ARRAY(act, ESP32P4DsiHostState,
                             ESP32P4_DSI_HOST_NREGS),
        VMSTATE_BOOL(shadow_pending, ESP32P4DsiHostState),
        VMSTATE_BOOL(frame_warned, ESP32P4DsiHostState),
        VMSTATE_UINT32(dpi_lcd_ctl, ESP32P4DsiHostState),
        VMSTATE_UINT8_ARRAY(ans, ESP32P4DsiHostState, 1024),
        VMSTATE_UINT32(ans_len, ESP32P4DsiHostState),
        VMSTATE_UINT32(ans_lost, ESP32P4DsiHostState),
        VMSTATE_BOOL(ans_none, ESP32P4DsiHostState),
        VMSTATE_END_OF_LIST()
    },
};

static void host_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    ESP32P4ClkSourceClass *csc = ESP32P4_CLK_SOURCE_CLASS(klass);

    dc->vmsd = &vmstate_host;
    rc->phases.hold = host_reset_hold;
    rc->phases.exit = host_reset_exit;
    csc->exact_edges = host_exact_edges;
}

/* ---- The DSI bridge ---- */

#define B_CLK_EN                0x00
#define B_EN                    0x04
#define B_DMA_REQ_CFG           0x08
#define B_RAW_NUM_CFG           0x0c
#define B_RAW_BUF_CREDIT_CTL    0x10
#define B_FIFO_FLOW_STATUS      0x14
#define B_PIXEL_TYPE            0x18
#define B_DMA_BLOCK_INTERVAL    0x1c
#define B_DMA_REQ_INTERVAL      0x20
#define B_DPI_LCD_CTL           0x24
#define B_DPI_RSV_DPI_DATA      0x28
#define B_DPI_V_CFG0            0x30
#define B_DPI_V_CFG1            0x34
#define B_DPI_H_CFG0            0x38
#define B_DPI_H_CFG1            0x3c
#define B_DPI_MISC_CONFIG       0x40
#define B_DPI_CONFIG_UPDATE     0x44
#define B_INT_ENA               0x50
#define B_INT_CLR               0x54
#define B_INT_RAW               0x58
#define B_INT_ST                0x5c
#define B_HOST_BIST_CTL         0x60
#define B_HOST_TRIGGER_REV      0x64
#define B_BLK_RAW_NUM_CFG       0x68
#define B_DMA_FRAME_INTERVAL    0x6c
#define B_MEM_AUX_CTRL          0x70
#define B_RDN_ECO_CS            0x74
#define B_RDN_ECO_LOW           0x78
#define B_RDN_ECO_HIGH          0x7c
#define B_HOST_CTRL             0x80
#define B_MEM_CLK_CTRL          0x84
#define B_DMA_FLOW_CTRL         0x88
#define B_RAW_BUF_ALMOST_EMPTY  0x8c
#define B_YUV_CFG               0x90
#define B_PHY_LP_LOOPBACK_CTRL  0x94
#define B_PHY_HS_LOOPBACK_CTRL  0x98
#define B_PHY_LOOPBACK_CNT      0x9c
#define B_VER_DATE              0x100

#define DSI_EN                  BIT(0)
#define DSI_BRIG_RST            BIT(1)
#define INT_UNDERRUN            BIT(0)
#define INT_VSYNC               BIT(1)
#define FIFO_BYTES              8192u
#define CHUNK_BYTES             2048u

static const DsiReg brg_regs[] = {
    { B_CLK_EN, K_RW, 0x1, 0 },
    { B_EN, K_SPECIAL, 0x3, 0 },
    { B_DMA_REQ_CFG, K_SPECIAL, 0xfff, 0x80 },
    { B_RAW_NUM_CFG, K_SPECIAL, 0x7fffff, 0x38400 },
    { B_RAW_BUF_CREDIT_CTL, K_SPECIAL, 0xffff7fff, 0x3200400 },
    { B_FIFO_FLOW_STATUS, K_RO, 0, 0 },
    { B_PIXEL_TYPE, K_RW, 0x7ff, 0 },
    { B_DMA_BLOCK_INTERVAL, K_SPECIAL, 0x3fffffff, 0x30002409 },
    { B_DMA_REQ_INTERVAL, K_SPECIAL, 0xffff, 0x1 },
    { B_DPI_LCD_CTL, K_RW, 0x7, 0 },
    { B_DPI_RSV_DPI_DATA, K_SPECIAL, 0x7fffffff, 0x3fff },
    { B_DPI_V_CFG0, K_RW, 0x0fff0fff, 0x01e0020d },
    { B_DPI_V_CFG1, K_RW, 0x0fff0fff, 0x00020021 },
    { B_DPI_H_CFG0, K_RW, 0x0fff0fff, 0x02800320 },
    { B_DPI_H_CFG1, K_RW, 0x0fff0fff, 0x00600030 },
    { B_DPI_MISC_CONFIG, K_RW, 0xfff1, 0x19d0 },
    { B_DPI_CONFIG_UPDATE, K_SPECIAL, 0, 0 },
    { B_INT_ENA, K_SPECIAL, 0x3, 0 },
    { B_INT_CLR, K_SPECIAL, 0, 0 },
    { B_INT_RAW, K_SPECIAL, 0x3, 0 },
    { B_INT_ST, K_RO, 0, 0 },
    { B_HOST_BIST_CTL, K_SPECIAL, 0x2, 0 },
    { B_HOST_TRIGGER_REV, K_UNIMP, 0x3, 0 },
    { B_BLK_RAW_NUM_CFG, K_SPECIAL, 0x3fffff, 0x38400 },
    { B_DMA_FRAME_INTERVAL, K_RW, 0x3fffffff, 0x20002409 },
    { B_MEM_AUX_CTRL, K_RW, 0x3fff, 0x1320 },
    { B_RDN_ECO_CS, K_RW, 0x1, 0 },
    { B_RDN_ECO_LOW, K_RW, 0xffffffff, 0 },
    { B_RDN_ECO_HIGH, K_RW, 0xffffffff, 0xffffffff },
    { B_HOST_CTRL, K_SPECIAL, 0x1, 0x1 },
    { B_MEM_CLK_CTRL, K_RW, 0x3, 0 },
    { B_DMA_FLOW_CTRL, K_SPECIAL, 0xf1, 0x11 },
    { B_RAW_BUF_ALMOST_EMPTY, K_SPECIAL, 0x7ff, 0x200 },
    { B_YUV_CFG, K_RW, 0x1f, 0 },
    { B_PHY_LP_LOOPBACK_CTRL, K_SPECIAL, 0x4fff0fff, 0 },
    { B_PHY_HS_LOOPBACK_CTRL, K_SPECIAL, 0x4bff03ff, 0x200 },
    { B_PHY_LOOPBACK_CNT, K_RW, 0x00ff00ff, 0x00400040 },
    { B_VER_DATE, K_RW, 0xffffffff, 0x20250109 },
};

static const DsiReg *brg_reg(hwaddr off)
{
    for (int i = 0; i < ARRAY_SIZE(brg_regs); i++) {
        if (brg_regs[i].off == off) {
            return &brg_regs[i];
        }
    }
    return NULL;
}

/* The DPI registers DPI_CONFIG_UPDATE takes, by index in dpi_act. */
static const uint16_t dpi_shadowed[7] = {
    B_DPI_LCD_CTL, B_DPI_RSV_DPI_DATA, B_DPI_V_CFG0, B_DPI_V_CFG1,
    B_DPI_H_CFG0, B_DPI_H_CFG1, B_DPI_MISC_CONFIG,
};
enum {
    D_LCD_CTL, D_RSV, D_V0, D_V1, D_H0, D_H1, D_MISC,
};

static uint32_t breg(ESP32P4DsiBridgeState *s, hwaddr a)
{
    return s->regs[a / 4];
}

static void brg_update_irq(ESP32P4DsiBridgeState *s)
{
    qemu_set_irq(s->irq, !!(breg(s, B_INT_RAW) & breg(s, B_INT_ENA)));
}

static void brg_int(ESP32P4DsiBridgeState *s, uint32_t bits)
{
    s->regs[B_INT_RAW / 4] |= bits;
    brg_update_irq(s);
}

static bool brg_active(ESP32P4DsiBridgeState *s)
{
    return (breg(s, B_EN) & DSI_EN) && !(breg(s, B_EN) & DSI_BRIG_RST) &&
           clock_is_enabled(s->clk_bus);
}

/* ---- The bridge's VDMA requests ---- */

static uint32_t req_interval_cycles(uint32_t v)
{
    return ctpop32(v & 0xffff);
}

static int64_t bus_after(ESP32P4DsiBridgeState *s, int64_t t, uint64_t n)
{
    ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->clk_bus, t);

    return esp32p4_clk_edges_after(&e, t, n);
}

static void brg_set_req(ESP32P4DsiBridgeState *s, bool req, bool single,
                        bool last)
{
    if (req != s->req) {
        s->req = req;
        qemu_set_irq(s->hs_req, req);
    }
    if (single != s->req_single) {
        s->req_single = single;
        qemu_set_irq(s->hs_single, single);
    }
    if (last != s->req_last) {
        s->req_last = last;
        qemu_set_irq(s->hs_last, last);
    }
}

static uint32_t brg_burst(ESP32P4DsiBridgeState *s)
{
    return MAX(breg(s, B_DMA_REQ_CFG) & 0xfff, 1);
}

static bool brg_multiblk(ESP32P4DsiBridgeState *s)
{
    return breg(s, B_DMA_FRAME_INTERVAL) & BIT(28);
}

/* What the bridge asks VDMA for now. */
static void brg_update_req(ESP32P4DsiBridgeState *s)
{
    int64_t now = esp32_line_time();
    uint32_t words = s->fifo_num / 8, credit = breg(s, B_RAW_BUF_CREDIT_CTL);

    if (!brg_active(s) || now < s->req_hold_ns) {
        brg_set_req(s, false, false, false);
        if (brg_active(s)) {
            timer_mod(s->req_timer, s->req_hold_ns);
        }
        return;
    }
    if (!(breg(s, B_DMA_FLOW_CTRL) & 1)) {
        brg_set_req(s, words <= (breg(s, B_RAW_BUF_ALMOST_EMPTY) & 0x7ff),
                    false, false);
        return;
    }
    /* The bridge as the flow controller: bursts, then single words. */
    {
        uint32_t burst = brg_burst(s);
        uint32_t left = brg_multiblk(s) ? MIN(s->blk_words, s->frame_words)
                                        : s->frame_words;

        if (left >= burst && words <= extract32(credit, 16, 15)) {
            brg_set_req(s, true, false, left == burst);
        } else if (left && left < burst && words < (credit & 0x7fff)) {
            brg_set_req(s, false, true, left == 1);
        } else {
            brg_set_req(s, false, false, false);
        }
    }
}

static void brg_catch_up(ESP32P4DsiBridgeState *s, int64_t now);

static void brg_req_timer_cb(void *opaque)
{
    ESP32P4DsiBridgeState *s = opaque;

    esp32_line_run_at(s->req_hold_ns, (void (*)(void *))brg_update_req, s);
}

/* VDMA's acknowledge: the transaction asked for is done. */
static void brg_hs_ack(void *opaque, int n, int level)
{
    ESP32P4DsiBridgeState *s = opaque;
    int64_t now = esp32_line_time();
    uint32_t gap = req_interval_cycles(breg(s, B_DMA_REQ_INTERVAL));

    if (!level) {
        return;
    }
    brg_catch_up(s, now);
    if (breg(s, B_DMA_FLOW_CTRL) & 1) {
        uint32_t got = s->req ? brg_burst(s) : 1;
        uint32_t blk = breg(s, B_DMA_BLOCK_INTERVAL);
        uint32_t frm = breg(s, B_DMA_FRAME_INTERVAL);

        s->frame_words -= MIN(got, s->frame_words);
        s->blk_words -= MIN(got, s->blk_words);
        if (!s->frame_words) {
            if (blk & BIT(28)) {
                s->frame_words = breg(s, B_RAW_NUM_CFG) & 0x3fffff;
            }
            s->blk_words = breg(s, B_BLK_RAW_NUM_CFG) & 0x3fffff;
            if (frm & BIT(29)) {
                gap += (frm & 0x3ff) * extract32(frm, 10, 18);
            }
        } else if (brg_multiblk(s) && !s->blk_words) {
            s->blk_words = breg(s, B_BLK_RAW_NUM_CFG) & 0x3fffff;
            if (blk & BIT(29)) {
                gap += (blk & 0x3ff) * extract32(blk, 10, 18);
            }
        }
    }
    s->req_hold_ns = bus_after(s, now, gap);
    brg_set_req(s, false, false, false);
    brg_update_req(s);
}

/* ---- The bridge's FIFO ---- */

static void brg_fifo_in(ESP32P4DsiBridgeState *s, const uint8_t *b,
                        uint32_t n)
{
    uint32_t room = FIFO_BYTES - s->fifo_num;

    if (n > room) {
        if (!s->overflow_warned) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: VDMA writes past the FIFO's "
                          "room: %u bytes lost\n", TYPE_ESP32P4_DSI_BRIDGE,
                          n - room);
            s->overflow_warned = true;
        }
        n = room;
    }
    for (uint32_t i = 0; i < n; i++) {
        s->fifo[(s->fifo_head + s->fifo_num + i) % FIFO_BYTES] = b[i];
    }
    s->fifo_num += n;
}

static uint32_t brg_fifo_out(ESP32P4DsiBridgeState *s, uint8_t *b,
                             uint32_t n)
{
    n = MIN(n, s->fifo_num);
    for (uint32_t i = 0; i < n; i++) {
        b[i] = s->fifo[(s->fifo_head + i) % FIFO_BYTES];
    }
    s->fifo_head = (s->fifo_head + n) % FIFO_BYTES;
    s->fifo_num -= n;
    return n;
}

static uint64_t brg_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read of the FIFO's port at 0x%"
                  HWADDR_PRIx ", which only VDMA writes\n",
                  TYPE_ESP32P4_DSI_BRIDGE, addr);
    return 0;
}

static void brg_mem_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    ESP32P4DsiBridgeState *s = opaque;
    uint8_t b[8];

    brg_catch_up(s, esp32_line_time());
    stq_le_p(b, value);
    if (!brg_active(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: FIFO write while the bridge is "
                      "disabled or in reset: dropped\n",
                      TYPE_ESP32P4_DSI_BRIDGE);
        return;
    }
    brg_fifo_in(s, b, size);
    brg_update_req(s);
}

static const MemoryRegionOps brg_mem_ops = {
    .read = brg_mem_read,
    .write = brg_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

/* ---- Pixels ---- */

/* Bits a pixel of RAW_TYPE @t takes in memory; 0 for a reserved type. */
static uint32_t raw_bits(uint32_t t)
{
    switch (t) {
    case 0: case 1: case 8:
        return 24;
    case 2: case 9:
        return 16;
    case 10:
        return 12;
    case 12:
        return 8;
    default:
        return 0;
    }
}

static int32_t clamp8(int32_t v)
{
    return MIN(MAX(v, 0), 255);
}

static int32_t shr8(int32_t v)
{
    return v >= 0 ? v >> 8 : -((-v + 255) >> 8);
}

/* YUV_CFG's conversion to RGB: the PPA's (TRM 39.5.1.1). */
static void yuv_rgb(uint32_t yuv_cfg, int32_t y, int32_t u, int32_t v,
                    uint32_t rgb[3])
{
    int32_t r, g, b;

    if (yuv_cfg & BIT(4)) {
        y = shr8(220 * y + 4096);
        u = shr8(225 * u + 4096);
        v = shr8(225 * v + 4096);
    }
    if (yuv_cfg & BIT(0)) {
        r = shr8(298 * y + 459 * v - 63367);
        g = shr8(298 * y - 55 * u - 136 * v + 19681);
        b = shr8(298 * y + 541 * u - 73918);
    } else {
        r = shr8(298 * y + 409 * v - 56906);
        g = shr8(298 * y - 100 * u - 208 * v + 34707);
        b = shr8(298 * y + 516 * u - 70836);
    }
    rgb[0] = clamp8(r);
    rgb[1] = clamp8(g);
    rgb[2] = clamp8(b);
}

/* An RGB888 pixel as a DPI word of DPI_TYPE and DPI_CONFIG. */
static uint32_t brg_dpi_word(uint32_t pixel_type, const uint32_t rgb[3])
{
    uint32_t cfg = extract32(pixel_type, 4, 2), type = extract32(pixel_type,
                                                                  7, 4);
    uint32_t r = rgb[0], g = rgb[1], b = rgb[2];

    switch (type) {
    case 1:
        return dpi_word(cfg == 1 ? 4 : 3, r >> 2, g >> 2, b >> 2);
    case 2:
        return dpi_word(MIN(cfg, 2), r >> 3, g >> 2, b >> 3);
    default:
        return r << 16 | g << 8 | b;
    }
}

/*
 * Decode @n pixels of the line in s->raw, the frame's active line @row,
 * into s->dpi; the line's pixels beyond @avail are DPI_RSV_DPI_DATA.
 */
static void brg_convert(ESP32P4DsiBridgeState *s, uint32_t n, uint32_t avail,
                        uint32_t row)
{
    uint32_t pt = breg(s, B_PIXEL_TYPE), type = pt & 0xf;
    uint32_t yuv = breg(s, B_YUV_CFG), fmt = extract32(yuv, 2, 2);
    uint32_t rsv = s->dpi_act[D_RSV] & 0xffffff;
    const uint8_t *raw = s->raw;
    g_autofree uint8_t *rev = NULL;
    static bool warned;

    if (type >= 8 && type <= 10 && !(pt & BIT(6)) && !warned) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: a YUV RAW_TYPE with DATA_IN_TYPE "
                      "RGB: converted as YUV\n", TYPE_ESP32P4_DSI_BRIDGE);
        warned = true;
    }
    if (type == 9 && !(yuv & BIT(1))) {
        rev = g_malloc(s->raw_len + 8);
        for (uint32_t i = 0; i < s->raw_len; i++) {
            rev[i] = s->raw[(i & ~7u) + 7 - (i & 7)];
        }
        raw = rev;
    }
    for (uint32_t x = 0; x < n; x++) {
        uint32_t rgb[3];
        const uint8_t *p;

        if (x >= avail) {
            s->dpi[x] = rsv;
            continue;
        }
        switch (type) {
        case 0:
            p = raw + 3 * x;
            rgb[0] = p[2];
            rgb[1] = p[1];
            rgb[2] = p[0];
            break;
        case 1:
            p = raw + 3 * x;
            rgb[0] = (p[2] & 0x3f) << 2 | (p[2] & 0x3f) >> 4;
            rgb[1] = (p[1] & 0x3f) << 2 | (p[1] & 0x3f) >> 4;
            rgb[2] = (p[0] & 0x3f) << 2 | (p[0] & 0x3f) >> 4;
            break;
        case 2: {
            uint16_t v = lduw_le_p(raw + 2 * x);
            uint32_t r = v >> 11, g = (v >> 5) & 0x3f, b = v & 0x1f;

            rgb[0] = r << 3 | (r & 7);
            rgb[1] = g << 2 | (g & 3);
            rgb[2] = b << 3 | (b & 7);
            break;
        }
        case 8:
            p = raw + 3 * x;
            yuv_rgb(yuv, p[2], p[1], p[0], rgb);
            break;
        case 9: {
            /* The pair's bytes: Y0, Y1, U, V positions by the format. */
            static const uint8_t pos[4][4] = {
                { 1, 3, 0, 2 }, { 1, 3, 2, 0 }, { 0, 2, 1, 3 },
                { 0, 2, 3, 1 },
            };
            const uint8_t *q = raw + 4 * (x / 2);

            yuv_rgb(yuv, q[pos[fmt][x & 1]], q[pos[fmt][2]], q[pos[fmt][3]],
                    rgb);
            break;
        }
        case 10: {
            const uint8_t *q = raw + 3 * (x / 2);
            int c = row & 1;

            s->chroma[c][x / 2] = q[0];
            yuv_rgb(yuv, q[1 + (x & 1)], s->chroma[0][x / 2],
                    row == 0 && !c ? 128 : s->chroma[1][x / 2], rgb);
            break;
        }
        case 12:
            p = raw + x;
            rgb[0] = rgb[1] = rgb[2] = p[0];
            break;
        default:
            s->dpi[x] = rsv;
            continue;
        }
        s->dpi[x] = brg_dpi_word(pt, rgb);
    }
}

/* ---- The DPI timing generator ---- */

static bool brg_dpi_on(ESP32P4DsiBridgeState *s)
{
    return brg_active(s) && (s->dpi_act[D_MISC] & 1) &&
           clock_is_enabled(s->clk_dpi);
}

/* The time of DPI clock @cyc counted from the generator's start. */
static int64_t brg_cyc_ns(ESP32P4DsiBridgeState *s, uint64_t cyc)
{
    return esp32p4_clk_count_time(&s->dpi_count,
                                  cyc > s->cyc_base ? cyc - s->cyc_base : 0);
}

static void brg_latch_timing(ESP32P4DsiBridgeState *s)
{
    uint32_t v0 = s->dpi_act[D_V0], v1 = s->dpi_act[D_V1];
    uint32_t h0 = s->dpi_act[D_H0], h1 = s->dpi_act[D_H1];

    s->vtotal = v0 & 0xfff;
    s->vdisp = extract32(v0, 16, 12);
    s->vbank = v1 & 0xfff;
    s->vsync = extract32(v1, 16, 12);
    s->htotal = h0 & 0xfff;
    s->hdisp = MIN(extract32(h0, 16, 12), ESP32P4_DSI_MAX_LINE);
    s->hbank = h1 & 0xfff;
    s->hsync = extract32(h1, 16, 12);
    if (s->vtotal <= s->vdisp + s->vsync + s->vbank ||
        s->htotal <= s->hdisp + s->hsync + s->hbank) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: VTOTAL %u / HTOTAL %u not above "
                      "their parts (%u, %u): stretched\n",
                      TYPE_ESP32P4_DSI_BRIDGE, s->vtotal, s->htotal,
                      s->vdisp + s->vsync + s->vbank,
                      s->hdisp + s->hsync + s->hbank);
        s->vtotal = MAX(s->vtotal, s->vdisp + s->vsync + s->vbank + 1);
        s->htotal = MAX(s->htotal, s->hdisp + s->hsync + s->hbank + 1);
    }
}

/* Pixels in the line's piece @k, and the line's pieces. */
static uint32_t brg_chunk_px(ESP32P4DsiBridgeState *s)
{
    uint32_t bits = raw_bits(breg(s, B_PIXEL_TYPE) & 0xf);

    return MAX(bits ? (CHUNK_BYTES * 8 / bits) & ~1u : s->hdisp, 2);
}

static uint32_t brg_chunks(ESP32P4DsiBridgeState *s)
{
    return MAX(DIV_ROUND_UP(s->hdisp, brg_chunk_px(s)), 1);
}

static uint64_t brg_line_cyc(ESP32P4DsiBridgeState *s, uint32_t line)
{
    return s->frame_cyc + (uint64_t)line * s->htotal;
}

static void brg_schedule(ESP32P4DsiBridgeState *s)
{
    uint64_t c = brg_line_cyc(s, s->line);

    if (s->step) {
        c += s->hsync + s->hbank +
             MIN(s->step * brg_chunk_px(s), s->hdisp);
    }
    s->next_cyc = c;
    timer_mod(s->timer, brg_cyc_ns(s, c));
}

static bool brg_line_active(ESP32P4DsiBridgeState *s, uint32_t line)
{
    uint32_t a = s->vsync + s->vbank;

    return line >= a && line < a + s->vdisp && s->hdisp;
}

/* The event the generator is at, at its time. */
static void brg_event(void *opaque)
{
    ESP32P4DsiBridgeState *s = opaque;
    int64_t t = brg_cyc_ns(s, s->next_cyc);

    if (!s->running) {
        return;
    }
    if (s->step == 0) {
        if (s->line == 0) {
            brg_latch_timing(s);
            brg_int(s, INT_VSYNC);
        }
        if (s->host) {
            esp32p4_dsi_host_dpi_hsync(s->host, s->line,
                                       s->line < s->vsync, t,
                                       brg_cyc_ns(s, brg_line_cyc(s, s->line
                                                                  + 1)));
        }
        if (brg_line_active(s, s->line)) {
            uint32_t bits = raw_bits(breg(s, B_PIXEL_TYPE) & 0xf);

            s->raw_len = 0;
            s->raw_need = DIV_ROUND_UP(s->hdisp * MAX(bits, 8), 8);
            s->underrun = false;
            s->step = 1;
            brg_schedule(s);
            return;
        }
    } else {
        uint32_t k = s->step, px = brg_chunk_px(s), nk = brg_chunks(s);
        uint32_t bits = raw_bits(breg(s, B_PIXEL_TYPE) & 0xf);
        uint32_t want = DIV_ROUND_UP(MIN(k * px, s->hdisp) * MAX(bits, 8), 8);
        uint32_t row = s->line - s->vsync - s->vbank;

        if (s->vdma) {
            esp32p4_dw_gdma_sync(s->vdma, t);
        }
        if (!s->underrun && want > s->raw_len) {
            uint32_t got = brg_fifo_out(s, s->raw + s->raw_len,
                                        want - s->raw_len);

            s->raw_len += got;
            if (s->raw_len < want) {
                s->underrun = true;
                if (s->line < extract32(s->dpi_act[D_MISC], 4, 12)) {
                    brg_int(s, INT_UNDERRUN);
                }
            }
            brg_update_req(s);
        }
        if (k < nk) {
            s->step++;
            brg_schedule(s);
            return;
        }
        brg_convert(s, s->hdisp, bits ? s->raw_len * 8 / bits : 0, row);
        if (s->host) {
            esp32p4_dsi_host_dpi_pixels(s->host, s->line, s->dpi, s->hdisp,
                                        brg_cyc_ns(s, brg_line_cyc(s,
                                                   s->line) + s->hsync +
                                                   s->hbank), t);
        }
    }
    s->step = 0;
    s->line++;
    if (s->line >= s->vtotal) {
        s->line = 0;
        s->frame++;
        s->frame_cyc += (uint64_t)s->vtotal * s->htotal;
    }
    brg_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.mipi]
 * The host has fallen LINE_MAX_LAG_NS behind the generator
 * (hw/core/line-time.h), at esp32_line_time(), the present: its timing
 * restarts there with a new frame, the frames between dropped.
 */
static void brg_slip(void *opaque)
{
    ESP32P4DsiBridgeState *s = opaque;
    int64_t now = esp32_line_time();

    if (!s->lag_logged) {
        s->lag_logged = true;
        qemu_log_mask(LOG_UNIMP, "%s: the host is too slow for the DPI "
                      "frames: they restart at the present, frames dropped\n",
                      TYPE_ESP32P4_DSI_BRIDGE);
    }
    if (s->host) {
        esp32p4_dsi_host_dpi_stop(s->host, now);
    }
    esp32p4_clk_count_start(&s->dpi_count, s->clk_dpi, now);
    s->cyc_base = 0;
    s->frame_cyc = 0;
    s->line = 0;
    s->step = 0;
    brg_latch_timing(s);
    brg_schedule(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.mipi]
 * The generator's event at @t, its time: on the host's clock, an event
 * LINE_MAX_LAG_NS behind the present, which the host cannot follow,
 * makes the generator slip to the present instead.
 */
static void brg_take_event(ESP32P4DsiBridgeState *s, int64_t t)
{
    if (line_lags(t)) {
        esp32_line_run_at(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), brg_slip, s);
    } else {
        esp32_line_run_at(t, brg_event, s);
    }
}

/*
 * The timer takes the event it was set for and no later one, however late
 * it runs: re-armed in the past, it runs again among every other device's
 * timers in order of time, so a generator the host held up catches up
 * event by event.
 */
static void brg_timer_cb(void *opaque)
{
    ESP32P4DsiBridgeState *s = opaque;

    brg_take_event(s, brg_cyc_ns(s, s->next_cyc));
}

/* When the bridge's next event, or request interval's end, falls due. */
static int64_t brg_next_due(ESP32P4DsiBridgeState *s)
{
    int64_t gen = s->running && timer_pending(s->timer) ?
                  brg_cyc_ns(s, s->next_cyc) : INT64_MAX;
    int64_t req = timer_pending(s->req_timer) ? s->req_hold_ns : INT64_MAX;

    return MIN(gen, req);
}

/*
 * Run the generator's events and the request interval's ends due by @now,
 * in order, each at its own time.
 */
static void brg_catch_up(ESP32P4DsiBridgeState *s, int64_t now)
{
    for (;;) {
        int64_t gen = s->running && timer_pending(s->timer) ?
                      brg_cyc_ns(s, s->next_cyc) : INT64_MAX;
        int64_t req = timer_pending(s->req_timer) ? s->req_hold_ns
                                                  : INT64_MAX;

        if (gen <= now && gen <= req) {
            timer_del(s->timer);
            brg_take_event(s, gen);
        } else if (req <= now) {
            timer_del(s->req_timer);
            esp32_line_run_at(req, (void (*)(void *))brg_update_req, s);
        } else {
            return;
        }
    }
}

/* Start or stop the DPI as the registers and the DPI clock say. */
static void brg_update_dpi(ESP32P4DsiBridgeState *s)
{
    bool on = brg_dpi_on(s);
    int64_t now = esp32_line_time();

    if (on == s->running) {
        return;
    }
    s->running = on;
    if (!on) {
        timer_del(s->timer);
        if (s->host) {
            esp32p4_dsi_host_dpi_stop(s->host, now);
        }
        return;
    }
    esp32p4_clk_count_start(&s->dpi_count, s->clk_dpi, now);
    s->cyc_base = 0;
    s->frame_cyc = 0;
    s->frame = 0;
    s->line = 0;
    s->step = 0;
    brg_latch_timing(s);
    brg_schedule(s);
}

static void brg_dpi_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4DsiBridgeState *s = opaque;
    int64_t now = esp32_line_time();

    if (event == ClockPreUpdate) {
        brg_catch_up(s, now);
        if (s->running) {
            s->cyc_base += esp32p4_clk_count_at(&s->dpi_count, now);
        }
        return;
    }
    if (s->running && clock_is_enabled(s->clk_dpi)) {
        esp32p4_clk_count_start(&s->dpi_count, s->clk_dpi, now);
        brg_schedule(s);
        return;
    }
    brg_update_dpi(s);
}

static void brg_bus_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4DsiBridgeState *s = opaque;

    brg_update_req(s);
    brg_update_dpi(s);
}

/* ---- The bridge's registers ---- */

static void brg_reset_regs(ESP32P4DsiBridgeState *s)
{
    for (int i = 0; i < ARRAY_SIZE(brg_regs); i++) {
        s->regs[brg_regs[i].off / 4] = brg_regs[i].reset;
    }
    for (int i = 0; i < ARRAY_SIZE(dpi_shadowed); i++) {
        s->dpi_act[i] = breg(s, dpi_shadowed[i]);
    }
}

/* DSI_BRIG_RST and the block's reset: the FIFO and the generator. */
static void brg_soft_reset(ESP32P4DsiBridgeState *s)
{
    s->fifo_head = 0;
    s->fifo_num = 0;
    s->overflow_warned = false;
    s->frame_words = breg(s, B_RAW_NUM_CFG) & 0x3fffff;
    s->blk_words = breg(s, B_BLK_RAW_NUM_CFG) & 0x3fffff;
    s->req_hold_ns = 0;
    timer_del(s->req_timer);
}

static uint64_t brg_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4DsiBridgeState *s = opaque;
    const DsiReg *r = brg_reg(addr);

    brg_catch_up(s, esp32_line_time());
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DSI_BRIDGE, addr);
        return 0;
    }
    switch (addr) {
    case B_FIFO_FLOW_STATUS:
        return MIN(s->fifo_num / 8, 0x3fff);
    case B_INT_ST:
        return breg(s, B_INT_RAW) & breg(s, B_INT_ENA);
    case B_INT_CLR:
    case B_DPI_CONFIG_UPDATE:
        return 0;
    case B_RAW_NUM_CFG:
    case B_BLK_RAW_NUM_CFG:
        return breg(s, addr) & ~BIT(31);
    default:
        return breg(s, addr);
    }
}

static void brg_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4DsiBridgeState *s = opaque;
    const DsiReg *r = brg_reg(addr);
    uint32_t v = value, old;

    brg_catch_up(s, esp32_line_time());
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DSI_BRIDGE, addr);
        return;
    }
    if (r->kind == K_RO) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_DSI_BRIDGE, addr);
        return;
    }
    if (r->kind == K_UNIMP && v) {
        qemu_log_mask(LOG_UNIMP, "%s: register 0x%" HWADDR_PRIx " (the PHY "
                      "trigger reversal) is not modelled\n",
                      TYPE_ESP32P4_DSI_BRIDGE, addr);
    }
    old = breg(s, addr);
    switch (addr) {
    case B_INT_CLR:
        s->regs[B_INT_RAW / 4] &= ~(v & 3);
        brg_update_irq(s);
        return;
    case B_INT_RAW:
        s->regs[B_INT_RAW / 4] &= ~(v & 3);
        brg_update_irq(s);
        return;
    case B_DPI_CONFIG_UPDATE:
        if (v & 1) {
            for (int i = 0; i < ARRAY_SIZE(dpi_shadowed); i++) {
                s->dpi_act[i] = breg(s, dpi_shadowed[i]);
            }
            if (s->dpi_act[D_RSV] & BIT(30)) {
                qemu_log_mask(LOG_UNIMP, "%s: DPI_DBG_EN is not modelled\n",
                              TYPE_ESP32P4_DSI_BRIDGE);
            }
            if (s->host) {
                esp32p4_dsi_host_dpi_ctl(s->host, s->dpi_act[D_LCD_CTL]);
            }
            brg_update_dpi(s);
        }
        return;
    case B_RAW_NUM_CFG:
        s->regs[addr / 4] = v & 0x7fffff;
        if (v & BIT(31)) {
            s->frame_words = v & 0x3fffff;
            brg_update_req(s);
        }
        return;
    case B_BLK_RAW_NUM_CFG:
        s->regs[addr / 4] = v & 0x3fffff;
        if (v & BIT(31)) {
            s->blk_words = v & 0x3fffff;
            brg_update_req(s);
        }
        return;
    default:
        break;
    }
    s->regs[addr / 4] = v & r->mask;
    switch (addr) {
    case B_EN:
        if ((v & DSI_BRIG_RST) && !(old & DSI_BRIG_RST)) {
            brg_soft_reset(s);
        }
        brg_update_req(s);
        brg_update_dpi(s);
        break;
    case B_RAW_BUF_CREDIT_CTL:
        if (v & BIT(31)) {
            qemu_log_mask(LOG_UNIMP, "%s: the credit counter's reset: the "
                          "model counts the FIFO's room directly\n",
                          TYPE_ESP32P4_DSI_BRIDGE);
        }
        brg_update_req(s);
        break;
    case B_DMA_REQ_CFG:
    case B_DMA_BLOCK_INTERVAL:
    case B_DMA_REQ_INTERVAL:
    case B_DMA_FLOW_CTRL:
    case B_RAW_BUF_ALMOST_EMPTY:
        brg_update_req(s);
        break;
    case B_INT_ENA:
        brg_update_irq(s);
        break;
    case B_HOST_CTRL:
        qemu_set_irq(s->cfg_ref_clk_en, v & 1);
        break;
    case B_HOST_BIST_CTL:
        if (v & 2) {
            qemu_log_mask(LOG_UNIMP, "%s: the host's memory BIST is not "
                          "modelled\n", TYPE_ESP32P4_DSI_BRIDGE);
        }
        break;
    case B_PHY_LP_LOOPBACK_CTRL:
    case B_PHY_HS_LOOPBACK_CTRL:
        if (v & BIT(30)) {
            qemu_log_mask(LOG_UNIMP, "%s: the D-PHY loopback tests are not "
                          "modelled\n", TYPE_ESP32P4_DSI_BRIDGE);
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps brg_ops = {
    .read = brg_read,
    .write = brg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void brg_reset_hold(Object *obj, ResetType type)
{
    ESP32P4DsiBridgeState *s = ESP32P4_DSI_BRIDGE(obj);

    brg_reset_regs(s);
    brg_soft_reset(s);
    timer_del(s->timer);
    s->running = false;
    s->frame = 0;
    s->line = 0;
    s->step = 0;
    s->raw_len = 0;
    s->req = s->req_single = s->req_last = false;
}

static void brg_reset_exit(Object *obj, ResetType type)
{
    ESP32P4DsiBridgeState *s = ESP32P4_DSI_BRIDGE(obj);

    qemu_set_irq(s->hs_req, 0);
    qemu_set_irq(s->hs_single, 0);
    qemu_set_irq(s->hs_last, 0);
    qemu_set_irq(s->cfg_ref_clk_en, 1);
    brg_update_irq(s);
}

static void brg_init(Object *obj)
{
    ESP32P4DsiBridgeState *s = ESP32P4_DSI_BRIDGE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &brg_ops, s, "esp32p4.dsi-bridge",
                          ESP32P4_DSI_REGS_SIZE);
    memory_region_init_io(&s->mem, obj, &brg_mem_ops, s,
                          "esp32p4.mipi-dsi-mem", ESP32P4_DSI_MEM_SIZE);
    /* VDMA's writes, which a pixel event brings in, during any access. */
    s->mem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk_bus = qdev_init_clock_in(dev, "bus", brg_bus_clk_update, s,
                                    ClockUpdate);
    s->clk_dpi = qdev_init_clock_in(dev, "dpi", brg_dpi_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_out_named(dev, &s->hs_req, ESP32P4_DSI_HS_REQ, 1);
    qdev_init_gpio_out_named(dev, &s->hs_single, ESP32P4_DSI_HS_SINGLE, 1);
    qdev_init_gpio_out_named(dev, &s->hs_last, ESP32P4_DSI_HS_LAST, 1);
    qdev_init_gpio_out_named(dev, &s->cfg_ref_clk_en,
                             ESP32P4_DSI_CFG_REF_CLK_EN, 1);
    qdev_init_gpio_in_named(dev, brg_hs_ack, ESP32P4_DSI_HS_ACK, 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, brg_timer_cb, s);
    s->req_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, brg_req_timer_cb, s);
}

static void brg_realize(DeviceState *dev, Error **errp)
{
    ESP32P4DsiBridgeState *s = ESP32P4_DSI_BRIDGE(dev);

    if (s->host) {
        s->host->bridge = s;
    }
}

static void brg_finalize(Object *obj)
{
    ESP32P4DsiBridgeState *s = ESP32P4_DSI_BRIDGE(obj);

    timer_free(s->timer);
    timer_free(s->req_timer);
}

static const Property brg_properties[] = {
    DEFINE_PROP_LINK("dsi-host", ESP32P4DsiBridgeState, host,
                     TYPE_ESP32P4_DSI_HOST, ESP32P4DsiHostState *),
    DEFINE_PROP_LINK("vdma", ESP32P4DsiBridgeState, vdma,
                     TYPE_ESP32P4_DW_GDMA, ESP32P4DwGdmaState *),
};

static const VMStateDescription vmstate_brg = {
    .name = TYPE_ESP32P4_DSI_BRIDGE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, ESP32P4DsiBridgeState),
        VMSTATE_TIMER_PTR(req_timer, ESP32P4DsiBridgeState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4DsiBridgeState, 0x104 / 4),
        VMSTATE_UINT32_ARRAY(dpi_act, ESP32P4DsiBridgeState, 8),
        VMSTATE_UINT8_ARRAY(fifo, ESP32P4DsiBridgeState, 8192),
        VMSTATE_UINT32(fifo_head, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(fifo_num, ESP32P4DsiBridgeState),
        VMSTATE_BOOL(running, ESP32P4DsiBridgeState),
        VMSTATE_ESP32P4_CLK_COUNT(dpi_count, ESP32P4DsiBridgeState),
        VMSTATE_UINT64(cyc_base, ESP32P4DsiBridgeState),
        VMSTATE_UINT64(frame_cyc, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(frame, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(line, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(step, ESP32P4DsiBridgeState),
        VMSTATE_UINT64(next_cyc, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(vtotal, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(vsync, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(vbank, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(vdisp, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(htotal, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(hsync, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(hbank, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(hdisp, ESP32P4DsiBridgeState),
        VMSTATE_UINT8_ARRAY(raw, ESP32P4DsiBridgeState,
                            ESP32P4_DSI_MAX_LINE * 3 + 16),
        VMSTATE_UINT32(raw_len, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(raw_need, ESP32P4DsiBridgeState),
        VMSTATE_BOOL(underrun, ESP32P4DsiBridgeState),
        VMSTATE_BOOL(req, ESP32P4DsiBridgeState),
        VMSTATE_BOOL(req_single, ESP32P4DsiBridgeState),
        VMSTATE_BOOL(req_last, ESP32P4DsiBridgeState),
        VMSTATE_INT64(req_hold_ns, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(frame_words, ESP32P4DsiBridgeState),
        VMSTATE_UINT32(blk_words, ESP32P4DsiBridgeState),
        VMSTATE_UINT8_2DARRAY(chroma, ESP32P4DsiBridgeState, 2,
                              ESP32P4_DSI_MAX_LINE / 2),
        VMSTATE_END_OF_LIST()
    },
};

static void brg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_brg;
    dc->realize = brg_realize;
    rc->phases.hold = brg_reset_hold;
    rc->phases.exit = brg_reset_exit;
    device_class_set_props(dc, brg_properties);
}

static void dsi_bus_class_init(ObjectClass *klass, const void *data)
{
    BusClass *k = BUS_CLASS(klass);

    k->max_dev = 1;
}

/* [spec:nuos:req:emu.esp32p4.mipi] */
static const TypeInfo esp32p4_mipi_dsi_types[] = {
    {
        .name = TYPE_ESP32P4_DSI_HOST,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4DsiHostState),
        .instance_init = host_init,
        .instance_finalize = host_finalize,
        .class_init = host_class_init,
        .interfaces = (const InterfaceInfo[]) {
            { TYPE_ESP32P4_CLK_SOURCE },
            { }
        },
    },
    {
        .name = TYPE_ESP32P4_DSI_BRIDGE,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4DsiBridgeState),
        .instance_init = brg_init,
        .instance_finalize = brg_finalize,
        .class_init = brg_class_init,
    },
    {
        .name = TYPE_ESP32P4_DSI_BUS,
        .parent = TYPE_BUS,
        .instance_size = sizeof(ESP32P4DsiBus),
        .class_init = dsi_bus_class_init,
    },
};

DEFINE_TYPES(esp32p4_mipi_dsi_types)
