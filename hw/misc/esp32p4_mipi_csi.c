/*
 * ESP32-P4 MIPI CSI: the CSI-2 host with its D-PHY, and the CSI bridge
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 42 (MIPI CSI), sections 38.5.3 to 38.5.8 and 38.7.2 to 38.7.3 (ISP_Tail,
 * CSI_CM and CSI_Bridge) and chapter 7 (VDMA's handshaking); ESP-IDF's
 * mipi_csi_host_reg.h and mipi_csi_bridge_reg.h with their structs for v3,
 * the mipi_csi_host_ll.h, mipi_csi_phy_ll.h, mipi_csi_brg_ll.h and
 * mipi_csi_hal.c that drive them, and esp_driver_cam's CSI controller
 * (esp_cam_ctlr_csi.c); MIPI CSI-2 for the packets.
 *
 * The CSI-2 host (0x5009F000), DesignWare-derived, with its RX D-PHY,
 * takes the packets of the board's camera sensor (the machine's
 * csi-camera, include/hw/misc/esp32p4_image_source.h):
 *
 *   - The PHY. Test code 0x44 holds hsfreqrange (TRM 42.5.3.2: TESTDOUT
 *     its loopback with TESTDIN[7] 0); other codes are reserved (logged).
 *     With PHY_SHUTDOWNZ and DPHY_RSTZ set and CSI_DPHY_CFG_CLK running
 *     (and the bridge's CSI_CFG_CLK_EN), the PHY is up 1000 configuration
 *     clocks later (the TRM gives no time). PHY_STOPSTATE shows the lanes
 *     in LP-11: the data lanes between the sensor's bursts while it
 *     streams, and while it does not (a sensor in standby holds LP-11);
 *     the clock lane likewise, but while a continuous clock runs. PHY_RX's
 *     RXCLKACTIVEHS shows the clock lane in HS, and the ULPS bits that no
 *     lane is in ULPS (the sensor never enters it).
 *   - Reception, while the PHY is up, CSI2_RESETN is set, the bridge's
 *     CSI_ENABLECLK enables the clock lane and N_LANES is valid (0 or 1;
 *     others logged). A packet comes as its burst ends, at the sensor's
 *     lane timing. The PHY's HS-SETTLE, set by hsfreqrange, is taken as
 *     115 ns + 8 UI of the range's middle rate: if it ends before the
 *     burst's HS-PREPARE (50 ns + 5 UI after LP-00, the sensor's D-PHY
 *     1.1 timing) the lanes report ErrSotHS (PHY group) and the packet is
 *     taken; if after its HS-ZERO (a further 100 ns + 6 UI) the sync is
 *     missed, ErrSotSyncHS (PHY_FATAL), and the packet is lost. The bytes
 *     go out on the sensor's lanes in turn and are merged from the host's
 *     N_LANES + 1 lanes in turn, so a count that differs from the
 *     sensor's garbles the packet as it would on the board (a lane without
 *     a burst ends the merge); SCRAMBLE_ENABLE descrambles each lane with
 *     x^16 + x^5 + x^4 + x^3 + 1 from its SCRAMBLE_SEED_LANEn, reseeded
 *     at each burst (the sensor does not scramble, so descrambling garbles
 *     too). The header's ECC is checked (a single bit corrected,
 *     ERR_ECC_CORRECTED_VCn; more, ERR_ECC_DOUBLE and the packet dropped);
 *     with VC_EXTENSION's VCX 0 the ECC byte's top bits extend the virtual
 *     channel. A data type CSI-2 does not define (0x13 to 0x17, 0x25 to
 *     0x27, 0x38 to 0x3F) raises ERR_ID_VCn and is dropped; a long packet
 *     whose burst ends before its word count and checksum, SHORTER_PAYLOAD,
 *     dropped; a checksum that does not match its payload, ERR_CRC_VCn,
 *     the payload going on marked, and ERR_FRAME_DATA_VCn at the frame's
 *     end. Frame starts and ends are matched per virtual channel
 *     (ERR_F_BNDRY_MATCH_VCn: an end without a start, or with another
 *     number, or a start before the last frame's end) and their numbers
 *     checked to step by one, from 65535 to 1, where both are non-zero
 *     (ERR_F_SEQ_VCn). The P4 takes every virtual channel's packets alike
 *     (TRM 42.3); the error bits are each channel's. Frames and long
 *     packets go on to the image interface (include/hw/misc/
 *     esp32p4_mipi_csi.h) as they arrive.
 *   - Interrupts (TRM 42.6). Each group's INT_ST latches its errors and
 *     clears as it is read, its INT_FORCE bits reading as set while set;
 *     CSI_INTR is high while a group has a status bit whose INT_MSK bit is
 *     set. INT_ST_MAIN shows the groups that have, latched until read.
 *     Register accesses first bring the PHY's start-up up to their time,
 *     however late the QEMU timer that times it.
 *   - "packet", a qtest's hook: a hex string of a whole packet as it comes
 *     off the lanes (header, ECC, payload, checksum) is received now, past
 *     the PHY, as the sensor cannot make errors.
 *
 * The CSI bridge (0x5009F800), with the CSI_CM its HOST_CM_CTRL drives:
 *
 *   - CSI_CM (TRM 38.5.6) on the host's output: nothing passes without
 *     CSI_HOST_CM_EN; the 8-bit swap (each 16-bit half's bytes) and the
 *     16-bit swap (each 32-bit word's halves) apply, as ESP-IDF sets them
 *     with the bypass too; without CSI_HOST_CM_BYPASS the conversions the
 *     TRM lists: RGB888 in CSI_HOST_CM_RX_RGB_FORMAT's order (the
 *     pixel's components named from its most significant byte, as the
 *     TRM's memory tables name them: RGB is B, G, R on the wire, CSI-2's
 *     order) to RGB888 or RGB565, RGB565 in RGB or BGR order to RGB565,
 *     YUV422 in CSI_HOST_CM_RX_YUV422_FORMAT's order (byte order: 0 YVYU,
 *     1 YUYV, 2 VYUY, 3 UYVY, as ESP-IDF maps them) to YUV422 or YUV420,
 *     and YUV420 to itself; others pass unconverted (logged). The output
 *     is TRM table 38.5-2's: RGB888 B, G, R; RGB565 little-endian, R in
 *     bits 15:11; YUV422 U, Y0, V, Y1; YUV420 rows of U Y0 Y1 then V Y0 Y1
 *     from the frame's first. RGB888 to RGB565 keeps each component's top
 *     bits; YUV422 to YUV420 the pair's U on the one row and V on the
 *     next. CSI_HOST_CM_LANE_NUM and HOST_SIZE_CTRL are kept as written:
 *     the model takes the lines from the packets.
 *   - The ISP's place and ISP_Tail's bypass: see the header.
 *   - CSI_Bridge (TRM 38.5.7): with CSI_BRIG_EN, from the next frame start,
 *     long packets of a data type in DATA_TYPE_MIN..MAX go, each 64-bit
 *     word byte-reversed with BYTE_ENDIAN_ORDER, into a buffer of 2048
 *     words (the model's size; the reset value of CSI_BUF_AFULL_THRD,
 *     0x7F8, is near it), the frame's last word padded with zeros. A
 *     buffer that cannot take a packet's bytes overruns
 *     (CSI_BUF_OVERRUN_INT, the rest lost); bytes coming with
 *     CSI_BRG_SYS_CLK stopped, or faster than its eight bytes a cycle,
 *     overflow the asynchronous FIFO (CSI_ASYNC_FIFO_OVF_INT, lost); a
 *     frame that lost bytes, or ended by another frame's start or by the
 *     bridge's disabling, is discarded (CSI_BRIG_DISCARD_INT). With
 *     VADR_NUM_CHECK the frame's lines (packets, or with HAS_HSYNC_E clear
 *     its bytes over HADR_NUM words) are checked against VADR_NUM
 *     (VADR_NUM_GT/LT_INT). ESP-IDF writes HADR_NUM in pixels where the
 *     TRM has 64-bit words; with HAS_HSYNC_E, as ESP-IDF leaves it, the
 *     value does not matter.
 *   - Requests to VDMA's interface 1, VDMA reading the buffer at
 *     MIPI_CSI_BRG_MEM (any address there is the buffer's port, read in
 *     order). As the flow controller (CSI_DMA_FLOW_CONTROLLER 1, the reset
 *     value and ESP-IDF's) the bridge asks for DMA_BURST_LEN words while it
 *     has them, then for single words at the frame's end, marking the
 *     request that ends the frame, or the block of DMABLK_SIZE bursts, as
 *     the last; with VDMA the flow controller the same, unmarked. After
 *     each acknowledge it waits DMA_REQ_INTERVAL bus cycles (the set bits:
 *     1 is one cycle, 0xFFFF sixteen). DMA_BURST_LEN and DMABLK_SIZE take
 *     effect at a frame's start, or with DMA_CFG_UPD_BY_BLK at a block's
 *     end as ESP-IDF's header has it (the TRM says the reverse),
 *     DMA_CFG_HAS_UPDATED_INT rising as they change.
 *     DMA_FORCE_RD_STATUS is kept: VDMA's status fetch reads the buffer
 *     like any read.
 *   - HOST_CTRL's CSI_ENABLECLK and CSI_CFG_CLK_EN gate the host's clock
 *     lane and PHY clock; LOOPBK_TEST_EN (the DSI PHY looped to this one)
 *     is not modelled (logged).
 *   - Register accesses, VDMA's reads and its acknowledges first end a
 *     request interval that fell due before them, at its time.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_mipi_csi.h"
#include "migration/vmstate.h"

/* ---- The CSI-2 host ---- */

#define A_VERSION           0x000
#define A_N_LANES           0x004
#define A_CSI2_RESETN       0x008
#define A_INT_ST_MAIN       0x00c
#define A_PHY_SHUTDOWNZ     0x040
#define A_DPHY_RSTZ         0x044
#define A_PHY_RX            0x048
#define A_PHY_STOPSTATE     0x04c
#define A_PHY_TEST_CTRL0    0x050
#define A_PHY_TEST_CTRL1    0x054
#define A_VC_EXTENSION      0x0c8
#define A_PHY_CAL           0x0cc
#define A_SCRAMBLING        0x300
#define A_SCRAMBLING_SEED1  0x304
#define A_SCRAMBLING_SEED2  0x308

enum {
    G_PHY_FATAL,
    G_PKT_FATAL,
    G_BNDRY,
    G_SEQ,
    G_CRC_FRAME,
    G_PLD_CRC,
    G_DATA_ID,
    G_ECC_CORRECTED,
    G_PHY,
};

static const struct {
    uint16_t st;
    uint32_t mask;
    uint32_t main;
} groups[ESP32P4_CSI_INT_GROUPS] = {
    [G_PHY_FATAL] = { 0x0e0, 0x3, BIT(0) },
    [G_PKT_FATAL] = { 0x0f0, 0x3, BIT(1) },
    [G_BNDRY] = { 0x280, 0xffff, BIT(2) },
    [G_SEQ] = { 0x290, 0xffff, BIT(3) },
    [G_CRC_FRAME] = { 0x2a0, 0xffff, BIT(4) },
    [G_PLD_CRC] = { 0x2b0, 0xffff, BIT(5) },
    [G_DATA_ID] = { 0x2c0, 0xffff, BIT(6) },
    [G_ECC_CORRECTED] = { 0x2d0, 0xffff, BIT(7) },
    [G_PHY] = { 0x110, 0x30003, BIT(16) },
};

#define ERR_ECC_DOUBLE      BIT(0)
#define SHORTER_PAYLOAD     BIT(1)

enum {
    PHY_OFF,
    PHY_INIT,
    PHY_READY,
};

#define PHY_INIT_CYCLES     1000

/* The sensor's D-PHY 1.1 burst timing, in ps and UI (image source). */
#define PREPARE_PS          50000
#define PREPARE_UI          5
#define ZERO_PS             100000
#define ZERO_UI             6
#define SETTLE_PS           115000
#define SETTLE_UI           8

static uint32_t hreg(ESP32P4CsiHostState *s, hwaddr a)
{
    return s->regs[a / 4];
}

static int host_group(hwaddr a, int *k)
{
    for (int g = 0; g < ESP32P4_CSI_INT_GROUPS; g++) {
        if (a >= groups[g].st && a < groups[g].st + 0xc) {
            *k = (a - groups[g].st) / 4;
            return g;
        }
    }
    return -1;
}

static uint32_t group_live(ESP32P4CsiHostState *s, int g)
{
    return (s->st[g] | hreg(s, groups[g].st + 8)) &
           hreg(s, groups[g].st + 4);
}

static void host_update_irq(ESP32P4CsiHostState *s)
{
    bool level = false;

    for (int g = 0; g < ESP32P4_CSI_INT_GROUPS; g++) {
        if (group_live(s, g)) {
            s->main_latch |= groups[g].main;
            level = true;
        }
    }
    qemu_set_irq(s->irq, level);
}

static void host_err(ESP32P4CsiHostState *s, int g, uint32_t bits)
{
    s->st[g] |= bits & groups[g].mask;
    host_update_irq(s);
}

/* ---- The PHY ---- */

static bool phy_wanted(ESP32P4CsiHostState *s)
{
    return (hreg(s, A_PHY_SHUTDOWNZ) & 1) && (hreg(s, A_DPHY_RSTZ) & 1) &&
           s->cfg_clk_en && clock_is_enabled(s->clk_cfg);
}

static void phy_update(void *opaque)
{
    ESP32P4CsiHostState *s = opaque;
    int64_t now = esp32_line_time();

    if (!phy_wanted(s)) {
        s->phy_state = PHY_OFF;
        timer_del(s->phy_timer);
        return;
    }
    if (s->phy_state == PHY_OFF) {
        ESP32P4ClkEdges e = esp32p4_clk_edges_or_at(s->clk_cfg, now);

        s->phy_state = PHY_INIT;
        s->phy_due_ns = esp32p4_clk_edges_after(&e, now, PHY_INIT_CYCLES);
    }
    if (s->phy_state == PHY_INIT) {
        if (s->phy_due_ns > now) {
            timer_mod(s->phy_timer, s->phy_due_ns);
            return;
        }
        s->phy_state = PHY_READY;
    }
}

static void phy_timer_cb(void *opaque)
{
    ESP32P4CsiHostState *s = opaque;

    esp32_line_run_at(s->phy_due_ns, phy_update, s);
}

/*
 * A register access at esp32_line_time(): first the PHY's start-up, if it
 * fell due before, at its own time, then the camera's packets due by then
 * (esp32p4_image_source_sync()), which its timer, late without icount, has
 * yet to bring: INT_ST_MAIN and the groups' errors as they are then.
 */
static void phy_catch_up(ESP32P4CsiHostState *s)
{
    if (timer_pending(s->phy_timer) && s->phy_due_ns <= esp32_line_time()) {
        timer_del(s->phy_timer);
        esp32_line_run_at(s->phy_due_ns, phy_update, s);
    }
    if (s->attached) {
        esp32p4_image_source_sync(s->attached);
    }
}

static uint32_t host_lanes(ESP32P4CsiHostState *s)
{
    uint32_t n = hreg(s, A_N_LANES) & 7;

    return n <= 1 ? n + 1 : 0;
}

static bool host_receiving(ESP32P4CsiHostState *s)
{
    return s->phy_state == PHY_READY && (hreg(s, A_CSI2_RESETN) & 1) &&
           s->enableclk && host_lanes(s);
}

static uint32_t phy_stopstate(ESP32P4CsiHostState *s)
{
    const ESP32P4ImageTiming *t;
    bool streaming, burst;
    uint32_t v = 0;

    if (s->phy_state != PHY_READY || !s->attached) {
        return 0;
    }
    t = esp32p4_image_source_timing(s->attached);
    streaming = esp32p4_image_source_streaming(s->attached);
    burst = esp32p4_image_source_csi_in_burst(s->attached);
    if (!burst) {
        v |= BIT(0);
        if (host_lanes(s) > 1 || t->lanes > 1) {
            v |= BIT(1);
        }
    }
    if (!(streaming && (t->clock_continuous || burst))) {
        v |= BIT(16);
    }
    return v;
}

static uint32_t phy_rx(ESP32P4CsiHostState *s)
{
    uint32_t v = BIT(16);

    if (s->phy_state != PHY_READY) {
        return v;
    }
    v |= BIT(0) | BIT(1);
    if (s->attached && s->enableclk &&
        esp32p4_image_source_streaming(s->attached) &&
        (esp32p4_image_source_timing(s->attached)->clock_continuous ||
         esp32p4_image_source_csi_in_burst(s->attached))) {
        v |= BIT(17);
    }
    return v;
}

/* ---- Packets ---- */

static bool dt_recognised(uint8_t dt)
{
    return dt <= 0x12 || (dt >= 0x18 && dt <= 0x24) ||
           (dt >= 0x28 && dt <= 0x37);
}

/* Descramble @n bytes of one lane with the LFSR seeded @seed. */
static void descramble(uint8_t *b, uint32_t n, uint16_t seed)
{
    uint16_t lfsr = seed;

    for (uint32_t i = 0; i < n; i++) {
        for (int k = 0; k < 8; k++) {
            uint16_t out = lfsr >> 15;
            uint16_t fb = (lfsr >> 15 ^ lfsr >> 4 ^ lfsr >> 3 ^ lfsr >> 2) & 1;

            b[i] ^= out << k;
            lfsr = lfsr << 1 | fb;
        }
    }
}

/*
 * The bytes @wire of a burst sent on @tx lanes, as the host receives them
 * on its own lanes, descrambled where it descrambles; their count.
 */
static uint32_t lanes_merge(ESP32P4CsiHostState *s, const uint8_t *wire,
                            uint32_t n, uint32_t tx, uint8_t *out)
{
    uint32_t rx = host_lanes(s), len = 0;
    g_autofree uint8_t *lane = g_malloc(n + 1);

    for (uint32_t j = 0;; j++) {
        uint32_t l = j % rx, idx = j / rx, k = idx * tx + l;

        if (l >= tx || k >= n) {
            break;
        }
        out[len++] = wire[k];
    }
    if (hreg(s, A_SCRAMBLING) & 1) {
        /* Each lane's bytes descrambled apart, then merged again. */
        for (uint32_t l = 0; l < rx; l++) {
            uint32_t m = 0;

            for (uint32_t j = l; j < len; j += rx) {
                lane[m++] = out[j];
            }
            descramble(lane, m, hreg(s, l ? A_SCRAMBLING_SEED2
                                          : A_SCRAMBLING_SEED1) & 0xffff);
            m = 0;
            for (uint32_t j = l; j < len; j += rx) {
                out[j] = lane[m++];
            }
        }
    }
    return len;
}

static void sink_frame_start(ESP32P4CsiHostState *s, uint8_t vc, uint16_t no,
                             int64_t t)
{
    if (s->sink_ops && s->sink_ops->frame_start) {
        s->sink_ops->frame_start(s->sink, vc, no, t);
    }
}

static void sink_frame_end(ESP32P4CsiHostState *s, uint8_t vc, uint16_t no,
                           bool crc, int64_t t)
{
    if (s->sink_ops && s->sink_ops->frame_end) {
        s->sink_ops->frame_end(s->sink, vc, no, crc, t);
    }
}

/* A packet's bytes as received: checked and passed on, at @t. */
static void host_take(ESP32P4CsiHostState *s, const uint8_t *b, uint32_t len,
                      int64_t start, int64_t t)
{
    uint32_t hdr, fixed;
    uint8_t ecc, dt, vc;
    uint16_t wc;
    int r;

    if (len < 4) {
        host_err(s, G_PKT_FATAL, ERR_ECC_DOUBLE);
        return;
    }
    hdr = b[0] | b[1] << 8 | b[2] << 16;
    ecc = b[3];
    r = esp32p4_mipi_ecc_check(hdr, ecc & 0x3f, &fixed);
    if (r == ESP32P4_MIPI_ECC_MULTI) {
        host_err(s, G_PKT_FATAL, ERR_ECC_DOUBLE);
        return;
    }
    hdr = fixed;
    dt = hdr & 0x3f;
    wc = hdr >> 8;
    vc = (hdr >> 6) & 3;
    if (!(hreg(s, A_VC_EXTENSION) & 1)) {
        vc |= (ecc >> 6) << 2;
    }
    if (r == ESP32P4_MIPI_ECC_CORRECTED) {
        host_err(s, G_ECC_CORRECTED, BIT(vc));
    }
    if (!dt_recognised(dt)) {
        host_err(s, G_DATA_ID, BIT(vc));
        return;
    }
    if (dt < 0x10) {
        switch (dt) {
        case 0x00:
            if (s->in_frame[vc]) {
                host_err(s, G_BNDRY, BIT(vc));
            }
            if (wc && s->last_no[vc] &&
                wc != (s->last_no[vc] == 0xffff ? 1 : s->last_no[vc] + 1)) {
                host_err(s, G_SEQ, BIT(vc));
            }
            s->in_frame[vc] = true;
            s->frame_no[vc] = wc;
            s->last_no[vc] = wc;
            s->crc_in_frame[vc] = false;
            sink_frame_start(s, vc, wc, t);
            break;
        case 0x01:
            if (!s->in_frame[vc] || wc != s->frame_no[vc]) {
                host_err(s, G_BNDRY, BIT(vc));
            }
            if (s->crc_in_frame[vc]) {
                host_err(s, G_CRC_FRAME, BIT(vc));
            }
            sink_frame_end(s, vc, wc, s->crc_in_frame[vc], t);
            s->in_frame[vc] = false;
            s->crc_in_frame[vc] = false;
            break;
        default:
            break;
        }
        return;
    }
    if (len < 4u + wc + 2) {
        host_err(s, G_PKT_FATAL, SHORTER_PAYLOAD);
        return;
    }
    {
        ESP32P4CsiIdiLine line = {
            .vc = vc,
            .dt = dt,
            .data = b + 4,
            .len = wc,
            .start_ns = start,
            .end_ns = t,
        };

        if (esp32p4_csi2_crc16(b + 4, wc) != (b[4 + wc] | b[5 + wc] << 8)) {
            host_err(s, G_PLD_CRC, BIT(vc));
            s->crc_in_frame[vc] = true;
            line.crc_error = true;
        }
        if (s->sink_ops && s->sink_ops->line) {
            s->sink_ops->line(s->sink, &line);
        }
    }
}

/* The PHY's HS-SETTLE against a burst at @bps: 0, ErrSotHS or ErrSotSync. */
enum {
    SOT_OK,
    SOT_ERR,
    SOT_SYNC_ERR,
};

static int sot_check(ESP32P4CsiHostState *s, uint64_t bps)
{
    uint32_t lo, hi;
    uint64_t ui_ps = 1000000000000ull / bps, settle, prepare, zero;

    if (!esp32p4_dphy_hsfreqrange((s->t44 >> 1) & 0x3f, &lo, &hi)) {
        return SOT_SYNC_ERR;
    }
    settle = SETTLE_PS + SETTLE_UI * 2000000ull / (lo + hi);
    prepare = PREPARE_PS + PREPARE_UI * ui_ps;
    zero = prepare + ZERO_PS + ZERO_UI * ui_ps;
    if (settle > zero) {
        return SOT_SYNC_ERR;
    }
    return settle < prepare ? SOT_ERR : SOT_OK;
}

static void host_csi_packet(void *opaque, const ESP32P4CsiPacket *p)
{
    ESP32P4CsiHostState *s = opaque;
    const ESP32P4ImageTiming *t = esp32p4_image_source_timing(s->attached);
    uint32_t n = 4 + (p->dt >= 0x10 ? p->wc + 2 : 0), rx, len, lanes;
    g_autofree uint8_t *wire = g_malloc(n);
    g_autofree uint8_t *got = g_malloc(n);
    uint32_t hdr = p->vc << 6 | p->dt | (uint32_t)p->wc << 8;

    if (!host_receiving(s)) {
        if (s->phy_state == PHY_READY && !host_lanes(s)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: N_LANES %u is invalid: "
                          "nothing received\n", TYPE_ESP32P4_CSI_HOST,
                          hreg(s, A_N_LANES));
        }
        return;
    }
    rx = host_lanes(s);
    lanes = (1u << MIN(rx, t->lanes)) - 1;
    switch (sot_check(s, t->lane_bps)) {
    case SOT_SYNC_ERR:
        host_err(s, G_PHY_FATAL, lanes);
        return;
    case SOT_ERR:
        host_err(s, G_PHY, lanes);
        break;
    default:
        break;
    }
    wire[0] = hdr;
    wire[1] = hdr >> 8;
    wire[2] = hdr >> 16;
    wire[3] = p->ecc;
    if (p->dt >= 0x10) {
        memcpy(wire + 4, p->payload, p->wc);
        wire[4 + p->wc] = p->crc;
        wire[5 + p->wc] = p->crc >> 8;
    }
    len = lanes_merge(s, wire, n, t->lanes, got);
    host_take(s, got, len, p->data_ns, p->end_ns);
}

static void host_stream(void *opaque, bool on, int64_t when_ns)
{
}

static const ESP32P4ImageSinkOps host_sink_ops = {
    .stream = host_stream,
    .csi_packet = host_csi_packet,
};

void esp32p4_csi_host_set_sink(ESP32P4CsiHostState *s,
                               const ESP32P4CsiIdiOps *ops, void *opaque)
{
    s->sink_ops = ops;
    s->sink = opaque;
}

static void host_set_packet(Object *obj, const char *value, Error **errp)
{
    ESP32P4CsiHostState *s = ESP32P4_CSI_HOST(obj);
    size_t n = strlen(value) / 2;
    g_autofree uint8_t *b = g_malloc(n + 1);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (size_t i = 0; i < n; i++) {
        char h[3] = { value[2 * i], value[2 * i + 1], 0 };
        unsigned int v;

        if (qemu_strtoui(h, NULL, 16, &v)) {
            error_setg(errp, "packet: want the packet's bytes in hex");
            return;
        }
        b[i] = v;
    }
    if (host_receiving(s)) {
        host_take(s, b, n, now, now);
    }
}

/* ---- The host's registers ---- */

static uint64_t host_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4CsiHostState *s = opaque;
    int g, k;
    uint32_t v;

    phy_catch_up(s);
    switch (addr) {
    case A_VERSION:
        return 0x3135302a;
    case A_N_LANES:
    case A_CSI2_RESETN:
    case A_PHY_SHUTDOWNZ:
    case A_DPHY_RSTZ:
    case A_VC_EXTENSION:
    case A_SCRAMBLING:
    case A_SCRAMBLING_SEED1:
    case A_SCRAMBLING_SEED2:
        return hreg(s, addr);
    case A_INT_ST_MAIN:
        v = s->main_latch;
        for (g = 0; g < ESP32P4_CSI_INT_GROUPS; g++) {
            if (group_live(s, g)) {
                v |= groups[g].main;
            }
        }
        s->main_latch = 0;
        return v;
    case A_PHY_RX:
        return phy_rx(s);
    case A_PHY_STOPSTATE:
        return phy_stopstate(s);
    case A_PHY_TEST_CTRL0:
        return s->test.ctrl0;
    case A_PHY_TEST_CTRL1:
        return esp32p4_dphy_test_ctrl1(&s->test,
                                       s->test.code == 0x44 &&
                                       !(esp32p4_dphy_test_din(&s->test) &
                                         0x80) ? s->t44 & 0x7e : 0);
    case A_PHY_CAL:
        return 0;
    default:
        g = host_group(addr, &k);
        if (g < 0) {
            break;
        }
        if (k == 0) {
            v = s->st[g] | hreg(s, groups[g].st + 8);
            s->st[g] = 0;
            host_update_irq(s);
            return v;
        }
        return hreg(s, addr);
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown register 0x%"
                  HWADDR_PRIx "\n", TYPE_ESP32P4_CSI_HOST, addr);
    return 0;
}

static void host_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    ESP32P4CsiHostState *s = opaque;
    uint32_t v = value;
    uint8_t data;
    int g, k;

    phy_catch_up(s);
    switch (addr) {
    case A_N_LANES:
        if ((v & 7) > 1) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: N_LANES %u is invalid (0: "
                          "lane 0, 1: lanes 0 and 1)\n",
                          TYPE_ESP32P4_CSI_HOST, v & 7);
        }
        if (s->attached && esp32p4_image_source_csi_in_burst(s->attached)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: N_LANES changed with the "
                          "lanes out of their stop state\n",
                          TYPE_ESP32P4_CSI_HOST);
        }
        s->regs[addr / 4] = v & 7;
        return;
    case A_CSI2_RESETN:
        s->regs[addr / 4] = v & 1;
        if (!(v & 1)) {
            memset(s->in_frame, 0, sizeof(s->in_frame));
            memset(s->crc_in_frame, 0, sizeof(s->crc_in_frame));
            memset(s->last_no, 0, sizeof(s->last_no));
        }
        return;
    case A_PHY_SHUTDOWNZ:
    case A_DPHY_RSTZ:
        s->regs[addr / 4] = v & 1;
        phy_update(s);
        return;
    case A_PHY_TEST_CTRL0:
    case A_PHY_TEST_CTRL1:
        switch (esp32p4_dphy_test_write(&s->test, addr == A_PHY_TEST_CTRL1, v,
                                        &data)) {
        case ESP32P4_DPHY_TEST_CLEAR:
            s->t44 = 0;
            break;
        case ESP32P4_DPHY_TEST_DATA:
            if (s->test.code == 0x44) {
                if (!(data & 0x80)) {
                    s->t44 = data & 0x7e;
                }
            } else if (s->test.code) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: test data 0x%02x for "
                              "reserved test code 0x%02x\n",
                              TYPE_ESP32P4_CSI_HOST, data, s->test.code);
            }
            break;
        default:
            break;
        }
        return;
    case A_VC_EXTENSION:
        s->regs[addr / 4] = v & 1;
        return;
    case A_SCRAMBLING:
        s->regs[addr / 4] = v & 1;
        return;
    case A_SCRAMBLING_SEED1:
    case A_SCRAMBLING_SEED2:
        s->regs[addr / 4] = v & 0xffff;
        return;
    case A_PHY_CAL:
        if (v & 1) {
            qemu_log_mask(LOG_UNIMP, "%s: HS skew calibration is not "
                          "modelled\n", TYPE_ESP32P4_CSI_HOST);
        }
        return;
    default:
        g = host_group(addr, &k);
        if (g >= 0 && k > 0) {
            s->regs[addr / 4] = v & groups[g].mask;
            host_update_irq(s);
            return;
        }
        if (g >= 0 || addr == A_VERSION || addr == A_INT_ST_MAIN ||
            addr == A_PHY_RX || addr == A_PHY_STOPSTATE) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register "
                          "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_CSI_HOST,
                          addr);
            return;
        }
        break;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: write of unknown register 0x%"
                  HWADDR_PRIx "\n", TYPE_ESP32P4_CSI_HOST, addr);
}

static const MemoryRegionOps host_ops = {
    .read = host_read,
    .write = host_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void host_clk_update(void *opaque, ClockEvent event)
{
    phy_update(opaque);
}

static void host_cfg_clk_en(void *opaque, int n, int level)
{
    ESP32P4CsiHostState *s = opaque;

    s->cfg_clk_en = level;
    phy_update(s);
}

static void host_enableclk(void *opaque, int n, int level)
{
    ESP32P4CsiHostState *s = opaque;

    s->enableclk = level;
}

static void host_reset_hold(Object *obj, ResetType type)
{
    ESP32P4CsiHostState *s = ESP32P4_CSI_HOST(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[A_N_LANES / 4] = 1;
    s->regs[A_SCRAMBLING_SEED1 / 4] = 0x1008;
    s->regs[A_SCRAMBLING_SEED2 / 4] = 0x1188;
    memset(s->st, 0, sizeof(s->st));
    s->main_latch = 0;
    esp32p4_dphy_test_reset(&s->test);
    s->t44 = 0;
    s->phy_state = PHY_OFF;
    timer_del(s->phy_timer);
    memset(s->in_frame, 0, sizeof(s->in_frame));
    memset(s->frame_no, 0, sizeof(s->frame_no));
    memset(s->last_no, 0, sizeof(s->last_no));
    memset(s->crc_in_frame, 0, sizeof(s->crc_in_frame));
}

static void host_reset_exit(Object *obj, ResetType type)
{
    host_update_irq(ESP32P4_CSI_HOST(obj));
}

static void host_init(Object *obj)
{
    ESP32P4CsiHostState *s = ESP32P4_CSI_HOST(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &host_ops, s, "esp32p4.csi-host",
                          ESP32P4_CSI_REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk_cfg = qdev_init_clock_in(dev, "phy-cfg", host_clk_update, s,
                                    ClockUpdate);
    qdev_init_gpio_in_named(dev, host_cfg_clk_en, ESP32P4_CSI_CFG_CLK_EN, 1);
    qdev_init_gpio_in_named(dev, host_enableclk, ESP32P4_CSI_ENABLECLK, 1);
    s->phy_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, phy_timer_cb, s);
    s->cfg_clk_en = true;
    s->enableclk = true;
    object_property_add_str(obj, "packet", NULL, host_set_packet);
}

static void host_realize(DeviceState *dev, Error **errp)
{
    ESP32P4CsiHostState *s = ESP32P4_CSI_HOST(dev);

    if (s->camera) {
        esp32p4_image_source_attach(s->camera, &host_sink_ops, s);
        s->attached = s->camera;
    }
}

static void host_finalize(Object *obj)
{
    ESP32P4CsiHostState *s = ESP32P4_CSI_HOST(obj);

    if (s->attached) {
        esp32p4_image_source_detach(s->attached, s);
    }
    timer_free(s->phy_timer);
}

static const Property host_properties[] = {
    DEFINE_PROP_LINK("camera", ESP32P4CsiHostState, camera,
                     TYPE_ESP32P4_IMAGE_SOURCE, ESP32P4ImageSource *),
};

static const VMStateDescription vmstate_host = {
    .name = TYPE_ESP32P4_CSI_HOST,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(phy_timer, ESP32P4CsiHostState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4CsiHostState,
                             ESP32P4_CSI_HOST_NREGS),
        VMSTATE_UINT32_ARRAY(st, ESP32P4CsiHostState,
                             ESP32P4_CSI_INT_GROUPS),
        VMSTATE_UINT32(main_latch, ESP32P4CsiHostState),
        VMSTATE_ESP32P4_DPHY_TEST(test, ESP32P4CsiHostState),
        VMSTATE_UINT8(t44, ESP32P4CsiHostState),
        VMSTATE_BOOL(cfg_clk_en, ESP32P4CsiHostState),
        VMSTATE_BOOL(enableclk, ESP32P4CsiHostState),
        VMSTATE_UINT32(phy_state, ESP32P4CsiHostState),
        VMSTATE_INT64(phy_due_ns, ESP32P4CsiHostState),
        VMSTATE_BOOL_ARRAY(in_frame, ESP32P4CsiHostState, ESP32P4_CSI_VCS),
        VMSTATE_UINT16_ARRAY(frame_no, ESP32P4CsiHostState,
                             ESP32P4_CSI_VCS),
        VMSTATE_UINT16_ARRAY(last_no, ESP32P4CsiHostState, ESP32P4_CSI_VCS),
        VMSTATE_BOOL_ARRAY(crc_in_frame, ESP32P4CsiHostState,
                           ESP32P4_CSI_VCS),
        VMSTATE_END_OF_LIST()
    },
};

static void host_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = host_realize;
    dc->vmsd = &vmstate_host;
    rc->phases.hold = host_reset_hold;
    rc->phases.exit = host_reset_exit;
    device_class_set_props(dc, host_properties);
}

/* ---- The CSI bridge ---- */

#define C_CLK_EN            0x00
#define C_CSI_EN            0x04
#define C_DMA_REQ_CFG       0x08
#define C_BUF_FLOW_CTL      0x0c
#define C_DATA_TYPE_CFG     0x10
#define C_FRAME_CFG         0x14
#define C_ENDIAN_MODE       0x18
#define C_INT_RAW           0x1c
#define C_INT_CLR           0x20
#define C_INT_ST            0x24
#define C_INT_ENA           0x28
#define C_DMA_REQ_INTERVAL  0x2c
#define C_DMABLK_SIZE       0x30
#define C_HOST_CTRL         0x40
#define C_HOST_CM_CTRL      0x48
#define C_HOST_SIZE_CTRL    0x4c

#define BRIG_EN             BIT(0)
#define BRIG_RST            BIT(1)
#define CFG_UPD_BY_BLK      BIT(12)
#define FLOW_BY_BRIDGE      BIT(17)
#define INT_VADR_GT         BIT(0)
#define INT_VADR_LT         BIT(1)
#define INT_DISCARD         BIT(2)
#define INT_OVERRUN         BIT(3)
#define INT_ASYNC_OVF       BIT(4)
#define INT_CFG_UPDATED     BIT(5)
#define BUF_BYTES           (ESP32P4_CSI_BUF_WORDS * 8)

/* CM_CTRL */
#define CM_EN               BIT(0)
#define CM_BYPASS           BIT(1)
#define CM_16BIT_SWAP       BIT(12)
#define CM_8BIT_SWAP        BIT(13)

typedef struct CsiReg {
    uint16_t off;
    uint32_t mask;
    uint32_t reset;
} CsiReg;

static const CsiReg brg_regs[] = {
    { C_CLK_EN, 0x1, 0 },
    { C_CSI_EN, 0x3, 0 },
    { C_DMA_REQ_CFG, 0x31fff, 0x20080 },
    { C_BUF_FLOW_CTL, 0x3fff, 0x7f8 },
    { C_DATA_TYPE_CFG, 0x3f3f, 0x2f18 },
    { C_FRAME_CFG, 0x3ffffff, 0x011e01e0 },
    { C_ENDIAN_MODE, 0x3, 0 },
    { C_INT_RAW, 0x3f, 0 },
    { C_INT_CLR, 0, 0 },
    { C_INT_ST, 0, 0 },
    { C_INT_ENA, 0x3f, 0 },
    { C_DMA_REQ_INTERVAL, 0xffff, 0x1 },
    { C_DMABLK_SIZE, 0x1fff, 0x1fff },
    { C_HOST_CTRL, 0x7, 0x3 },
    { C_HOST_CM_CTRL, 0x3fff, 0x803 },
    { C_HOST_SIZE_CTRL, 0xffffff, 0 },
};

static uint32_t breg(ESP32P4CsiBridgeState *s, hwaddr a)
{
    return s->regs[a / 4];
}

static void brg_update_irq(ESP32P4CsiBridgeState *s)
{
    qemu_set_irq(s->irq, !!(breg(s, C_INT_RAW) & breg(s, C_INT_ENA)));
}

static void brg_int(ESP32P4CsiBridgeState *s, uint32_t bits)
{
    s->regs[C_INT_RAW / 4] |= bits;
    brg_update_irq(s);
}

static bool brg_enabled(ESP32P4CsiBridgeState *s)
{
    return (breg(s, C_CSI_EN) & BRIG_EN) && !(breg(s, C_CSI_EN) & BRIG_RST);
}

bool esp32p4_csi_bridge_almost_full(ESP32P4CsiBridgeState *s)
{
    return s->buf_num / 8 > (breg(s, C_BUF_FLOW_CTL) & 0x3fff);
}

/* Take DMA_BURST_LEN and DMABLK_SIZE into effect. */
static void brg_take_cfg(ESP32P4CsiBridgeState *s, bool irq)
{
    uint32_t burst = MAX(breg(s, C_DMA_REQ_CFG) & 0xfff, 1);
    uint32_t blk = MAX(breg(s, C_DMABLK_SIZE) & 0x1fff, 1);

    if (irq && (burst != s->burst_len || blk != s->blk_size)) {
        brg_int(s, INT_CFG_UPDATED);
    }
    s->burst_len = burst;
    s->blk_size = blk;
}

/* ---- The bridge's VDMA requests ---- */

static void brg_set_req(ESP32P4CsiBridgeState *s, bool req, bool single,
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

/* Words VDMA may read now: those of an ended frame first. */
static uint32_t brg_words(ESP32P4CsiBridgeState *s)
{
    return (s->end_pending ? s->end_bytes : s->buf_num) / 8;
}

static void brg_update_req(ESP32P4CsiBridgeState *s)
{
    int64_t now = esp32_line_time();
    uint32_t words = brg_words(s);
    bool fc = breg(s, C_DMA_REQ_CFG) & FLOW_BY_BRIDGE;

    if (!brg_enabled(s) || !clock_is_enabled(s->clk_bus) ||
        now < s->req_hold_ns) {
        brg_set_req(s, false, false, false);
        if (brg_enabled(s) && now < s->req_hold_ns) {
            timer_mod(s->req_timer, s->req_hold_ns);
        }
        return;
    }
    if (words >= s->burst_len) {
        bool last = s->bursts + 1 >= s->blk_size ||
                    (s->end_pending && words == s->burst_len);

        brg_set_req(s, true, false, fc && last);
    } else if (s->end_pending && words) {
        brg_set_req(s, false, true, fc && words == 1);
    } else {
        brg_set_req(s, false, false, false);
    }
}

static void brg_req_timer_cb(void *opaque)
{
    ESP32P4CsiBridgeState *s = opaque;

    esp32_line_run_at(s->req_hold_ns, (void (*)(void *))brg_update_req, s);
}

/*
 * An access or acknowledge at esp32_line_time(): first the request
 * interval's end, if it fell due before, at its own time.
 */
static void brg_catch_up(ESP32P4CsiBridgeState *s)
{
    if (timer_pending(s->req_timer) && s->req_hold_ns <= esp32_line_time()) {
        timer_del(s->req_timer);
        esp32_line_run_at(s->req_hold_ns,
                          (void (*)(void *))brg_update_req, s);
    }
}

static void brg_hs_ack(void *opaque, int n, int level)
{
    ESP32P4CsiBridgeState *s = opaque;
    int64_t now = esp32_line_time();
    ESP32P4ClkEdges e;

    if (!level) {
        return;
    }
    brg_catch_up(s);
    if (s->req_last || (s->req && ++s->bursts >= s->blk_size)) {
        s->bursts = 0;
        if (breg(s, C_DMA_REQ_CFG) & CFG_UPD_BY_BLK) {
            brg_take_cfg(s, true);
        }
    }
    e = esp32p4_clk_edges_or_at(s->clk_bus, now);
    s->req_hold_ns = esp32p4_clk_edges_after(&e, now,
        ctpop32(breg(s, C_DMA_REQ_INTERVAL) & 0xffff));
    brg_set_req(s, false, false, false);
    brg_update_req(s);
}

/* ---- The buffer ---- */

static uint32_t brg_buf_in(ESP32P4CsiBridgeState *s, const uint8_t *b,
                           uint32_t n)
{
    uint32_t room = BUF_BYTES - s->buf_num;

    n = MIN(n, room);
    for (uint32_t i = 0; i < n; i++) {
        s->buf[(s->buf_head + s->buf_num + i) % BUF_BYTES] = b[i];
    }
    s->buf_num += n;
    return n;
}

static uint64_t brg_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4CsiBridgeState *s = opaque;
    uint8_t b[8] = { 0 };
    uint32_t n;

    brg_catch_up(s);
    n = MIN(size, s->buf_num);
    if (n < size) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of the buffer's port with "
                      "%u of %u bytes there\n", TYPE_ESP32P4_CSI_BRIDGE, n,
                      size);
    }
    for (uint32_t i = 0; i < n; i++) {
        b[i] = s->buf[(s->buf_head + i) % BUF_BYTES];
    }
    s->buf_head = (s->buf_head + n) % BUF_BYTES;
    s->buf_num -= n;
    if (s->end_pending) {
        s->end_bytes -= MIN(n, s->end_bytes);
        if (!s->end_bytes) {
            s->end_pending = false;
        }
    }
    return ldq_le_p(b);
}

static void brg_mem_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    ESP32P4CsiBridgeState *s = opaque;
    uint8_t b[8];

    if (s->isp_dma_write) {
        stq_le_p(b, value);
        s->isp_dma_write(s->isp_dma, b, size);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: write to the buffer's port at 0x%"
                  HWADDR_PRIx ", with no ISP to take it\n",
                  TYPE_ESP32P4_CSI_BRIDGE, addr);
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

/* ---- CSI_Bridge's input (ISP_Tail's output) ---- */

/* The frame that was coming in ends at the buffer's present end. */
static void brg_frame_close(ESP32P4CsiBridgeState *s, bool discard)
{
    uint8_t pad[8] = { 0 };
    uint32_t cfg = breg(s, C_FRAME_CFG), lines = s->lines;

    if (s->frame_bytes % 8 && !brg_buf_in(s, pad, 8 - s->frame_bytes % 8)) {
        discard = true;
    }
    if (!(cfg & BIT(24))) {
        uint32_t hadr = extract32(cfg, 12, 12);

        lines = hadr ? s->frame_bytes / (hadr * 8) : 0;
    }
    if (cfg & BIT(25)) {
        uint32_t vadr = cfg & 0xfff;

        if (vadr > lines) {
            brg_int(s, INT_VADR_GT);
        } else if (vadr < lines) {
            brg_int(s, INT_VADR_LT);
        }
    }
    if (discard || s->frame_bad) {
        brg_int(s, INT_DISCARD);
    }
    s->capturing = false;
    s->end_pending = true;
    s->end_bytes = s->buf_num;
    brg_update_req(s);
}

static void tail_frame_start(void *opaque, uint8_t vc, uint16_t number,
                             int64_t when_ns)
{
    ESP32P4CsiBridgeState *s = opaque;

    if (s->capturing) {
        brg_frame_close(s, true);
    }
    if (!brg_enabled(s)) {
        return;
    }
    if (!(breg(s, C_DMA_REQ_CFG) & CFG_UPD_BY_BLK)) {
        brg_take_cfg(s, true);
    }
    s->capturing = true;
    s->frame_bad = false;
    s->lines = 0;
    s->frame_bytes = 0;
}

static void tail_line(void *opaque, const ESP32P4CsiIdiLine *l)
{
    ESP32P4CsiBridgeState *s = opaque;
    uint32_t dtc = breg(s, C_DATA_TYPE_CFG), n = l->len;
    g_autofree uint8_t *b = NULL;
    uint64_t f;

    if (!s->capturing || l->dt < (dtc & 0x3f) || l->dt > extract32(dtc, 8, 6)) {
        return;
    }
    f = clock_get_hz(s->clk_bus);
    if (!f || (l->end_ns > l->start_ns &&
               (uint64_t)n * NANOSECONDS_PER_SECOND >
               8 * f * (uint64_t)(l->end_ns - l->start_ns))) {
        brg_int(s, INT_ASYNC_OVF);
        s->frame_bad = true;
        return;
    }
    b = g_malloc(n + 8);
    if (breg(s, C_ENDIAN_MODE) & 1) {
        /* Each 64-bit word of the frame's stream byte-reversed. */
        for (uint32_t i = 0; i < n; i++) {
            uint32_t pos = s->frame_bytes + i;
            uint32_t word = pos & ~7u, k = 7 - (pos & 7);

            b[(word + k) - s->frame_bytes] = l->data[i];
        }
        if (breg(s, C_ENDIAN_MODE) & 2) {
            qemu_log_mask(LOG_UNIMP, "%s: BIT_ENDIAN_ORDER is not modelled\n",
                          TYPE_ESP32P4_CSI_BRIDGE);
        }
    } else {
        memcpy(b, l->data, n);
    }
    if (brg_buf_in(s, b, n) < n) {
        brg_int(s, INT_OVERRUN);
        s->frame_bad = true;
    }
    s->frame_bytes += n;
    s->lines++;
    brg_update_req(s);
}

static void tail_frame_end(void *opaque, uint8_t vc, uint16_t number,
                           bool crc_error, int64_t when_ns)
{
    ESP32P4CsiBridgeState *s = opaque;

    if (s->capturing) {
        brg_frame_close(s, false);
    }
}

const ESP32P4CsiIdiOps esp32p4_csi_bridge_tail_ops = {
    .frame_start = tail_frame_start,
    .line = tail_line,
    .frame_end = tail_frame_end,
};

/* ---- CSI_CM ---- */

static void cm_frame_start(void *opaque, uint8_t vc, uint16_t number,
                           int64_t when_ns)
{
    ESP32P4CsiBridgeState *s = opaque;

    if (!(breg(s, C_HOST_CM_CTRL) & CM_EN)) {
        return;
    }
    s->cm_row = 0;
    if (s->isp_ops) {
        s->isp_ops->frame_start(s->isp, vc, number, when_ns);
    } else {
        tail_frame_start(s, vc, number, when_ns);
    }
}

static void cm_frame_end(void *opaque, uint8_t vc, uint16_t number,
                         bool crc_error, int64_t when_ns)
{
    ESP32P4CsiBridgeState *s = opaque;

    if (!(breg(s, C_HOST_CM_CTRL) & CM_EN)) {
        return;
    }
    if (s->isp_ops) {
        s->isp_ops->frame_end(s->isp, vc, number, crc_error, when_ns);
    } else {
        tail_frame_end(s, vc, number, crc_error, when_ns);
    }
}

/* RGB888 pixel @p's red, green and blue byte positions by its order. */
static void rgb888_pos(uint32_t fmt, int pos[3])
{
    /* The components from the most significant byte: R, G, B indices. */
    static const char *const names[8] = {
        "RGB", "BGR", "RGB", "BRG", "GRB", "GBR", "RGB", "RGB",
    };
    const char *n = names[fmt & 7];

    for (int i = 0; i < 3; i++) {
        int c = n[2 - i] == 'R' ? 0 : n[2 - i] == 'G' ? 1 : 2;

        pos[c] = i;
    }
}

/* Convert @n bytes at @in by CM_CTRL into @out; its length. */
static uint32_t cm_convert(ESP32P4CsiBridgeState *s, uint32_t cm,
                           const uint8_t *in, uint32_t n, uint8_t *out)
{
    uint32_t rx = extract32(cm, 2, 2), tx = extract32(cm, 9, 2);
    uint32_t rgbf = extract32(cm, 4, 3), yuvf = extract32(cm, 7, 2);
    uint32_t len = 0;
    static bool warned;

    if (rx == 0 && (tx == 0 || tx == 1)) {
        int pos[3];

        rgb888_pos(rgbf, pos);
        if ((n / 3) % 4 && !warned) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: CSI_CM's RGB888 needs a "
                          "multiple of 4 pixels a line\n",
                          TYPE_ESP32P4_CSI_BRIDGE);
            warned = true;
        }
        for (uint32_t i = 0; i + 3 <= n; i += 3) {
            uint32_t r = in[i + pos[0]], g = in[i + pos[1]];
            uint32_t b = in[i + pos[2]];

            if (tx == 0) {
                out[len++] = b;
                out[len++] = g;
                out[len++] = r;
            } else {
                uint16_t v = (r >> 3) << 11 | (g >> 2) << 5 | b >> 3;

                out[len++] = v;
                out[len++] = v >> 8;
            }
        }
        return len;
    }
    if (rx == 1 && tx == 1) {
        for (uint32_t i = 0; i + 2 <= n; i += 2) {
            uint16_t v = in[i] | in[i + 1] << 8;

            if (rgbf == 1) {
                v = (v & 0x1f) << 11 | (v & 0x7e0) | v >> 11;
            } else if (rgbf != 0 && rgbf != 2 && !warned) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: RGB565 takes the RGB or "
                              "BGR order only\n", TYPE_ESP32P4_CSI_BRIDGE);
                warned = true;
            }
            out[len++] = v;
            out[len++] = v >> 8;
        }
        return len;
    }
    if (rx == 2 && (tx == 2 || tx == 3)) {
        /* Y0, U, Y1, V positions in the pair by YUV422_FORMAT. */
        static const uint8_t pos[4][4] = {
            { 0, 3, 2, 1 }, { 0, 1, 2, 3 }, { 1, 2, 3, 0 }, { 1, 0, 3, 2 },
        };
        const uint8_t *p = pos[yuvf];

        for (uint32_t i = 0; i + 4 <= n; i += 4) {
            uint8_t y0 = in[i + p[0]], u = in[i + p[1]];
            uint8_t y1 = in[i + p[2]], v = in[i + p[3]];

            if (tx == 2) {
                out[len++] = u;
                out[len++] = y0;
                out[len++] = v;
                out[len++] = y1;
            } else {
                out[len++] = s->cm_row & 1 ? v : u;
                out[len++] = y0;
                out[len++] = y1;
            }
        }
        return len;
    }
    if (rx == 3 && tx == 3) {
        memcpy(out, in, n);
        return n;
    }
    if (!warned) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CSI_CM has no conversion from "
                      "format %u to %u: passed unconverted\n",
                      TYPE_ESP32P4_CSI_BRIDGE, rx, tx);
        warned = true;
    }
    memcpy(out, in, n);
    return n;
}

static void cm_line(void *opaque, const ESP32P4CsiIdiLine *l)
{
    ESP32P4CsiBridgeState *s = opaque;
    uint32_t cm = breg(s, C_HOST_CM_CTRL), n = ROUND_UP(l->len, 4);
    g_autofree uint8_t *a = g_malloc0(n + 4);
    g_autofree uint8_t *b = NULL;
    ESP32P4CsiIdiLine out = *l;

    if (!(cm & CM_EN)) {
        return;
    }
    memcpy(a, l->data, l->len);
    for (uint32_t i = 0; i < n; i += 4) {
        if (cm & CM_8BIT_SWAP) {
            uint8_t t0 = a[i], t2 = a[i + 2];

            a[i] = a[i + 1];
            a[i + 1] = t0;
            a[i + 2] = a[i + 3];
            a[i + 3] = t2;
        }
        if (cm & CM_16BIT_SWAP) {
            uint8_t t0 = a[i], t1 = a[i + 1];

            a[i] = a[i + 2];
            a[i + 1] = a[i + 3];
            a[i + 2] = t0;
            a[i + 3] = t1;
        }
    }
    out.data = a;
    if (!(cm & CM_BYPASS)) {
        b = g_malloc0(n + 8);
        out.len = cm_convert(s, cm, a, l->len, b);
        out.data = b;
    }
    if (s->isp_ops) {
        s->isp_ops->line(s->isp, &out);
    } else {
        tail_line(s, &out);
    }
    s->cm_row++;
}

const ESP32P4CsiIdiOps esp32p4_csi_bridge_cm_ops = {
    .frame_start = cm_frame_start,
    .line = cm_line,
    .frame_end = cm_frame_end,
};

void esp32p4_csi_bridge_attach_isp(ESP32P4CsiBridgeState *s,
                                   const ESP32P4CsiIdiOps *ops,
                                   void *opaque)
{
    s->isp_ops = ops;
    s->isp = opaque;
}

void esp32p4_csi_bridge_attach_isp_dma(ESP32P4CsiBridgeState *s,
                                       ESP32P4CsiDmaWriteFn *write,
                                       void *opaque)
{
    s->isp_dma_write = write;
    s->isp_dma = opaque;
}

/* ---- The bridge's registers ---- */

static const CsiReg *brg_reg(hwaddr a)
{
    for (int i = 0; i < ARRAY_SIZE(brg_regs); i++) {
        if (brg_regs[i].off == a) {
            return &brg_regs[i];
        }
    }
    return NULL;
}

static void brg_flush(ESP32P4CsiBridgeState *s)
{
    s->buf_head = 0;
    s->buf_num = 0;
    s->capturing = false;
    s->end_pending = false;
    s->end_bytes = 0;
    s->bursts = 0;
    s->req_hold_ns = 0;
    timer_del(s->req_timer);
}

static uint64_t brg_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4CsiBridgeState *s = opaque;
    const CsiReg *r = brg_reg(addr);

    brg_catch_up(s);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_CSI_BRIDGE, addr);
        return 0;
    }
    switch (addr) {
    case C_BUF_FLOW_CTL:
        return breg(s, addr) | MIN(s->buf_num / 8, 0x3fff) << 16;
    case C_INT_ST:
        return breg(s, C_INT_RAW) & breg(s, C_INT_ENA);
    case C_INT_CLR:
        return 0;
    default:
        return breg(s, addr);
    }
}

static void brg_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4CsiBridgeState *s = opaque;
    const CsiReg *r = brg_reg(addr);
    uint32_t v = value, old;

    brg_catch_up(s);
    if (!r) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of unknown register 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_CSI_BRIDGE, addr);
        return;
    }
    switch (addr) {
    case C_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only INT_ST\n",
                      TYPE_ESP32P4_CSI_BRIDGE);
        return;
    case C_INT_CLR:
    case C_INT_RAW:
        s->regs[C_INT_RAW / 4] &= ~(v & 0x3f);
        brg_update_irq(s);
        return;
    default:
        break;
    }
    old = breg(s, addr);
    s->regs[addr / 4] = v & r->mask;
    switch (addr) {
    case C_CSI_EN:
        if ((v & BRIG_RST) && !(old & BRIG_RST)) {
            brg_flush(s);
        }
        if (!brg_enabled(s) && s->capturing) {
            brg_frame_close(s, true);
        }
        if (brg_enabled(s) && !((old & BRIG_EN) && !(old & BRIG_RST))) {
            brg_take_cfg(s, false);
        }
        brg_update_req(s);
        break;
    case C_DMA_REQ_CFG:
    case C_DMA_REQ_INTERVAL:
    case C_DMABLK_SIZE:
        if (!brg_enabled(s)) {
            brg_take_cfg(s, false);
        }
        brg_update_req(s);
        break;
    case C_INT_ENA:
        brg_update_irq(s);
        break;
    case C_HOST_CTRL:
        qemu_set_irq(s->enableclk, v & 1);
        qemu_set_irq(s->cfg_clk_en, (v >> 1) & 1);
        if (v & 4) {
            qemu_log_mask(LOG_UNIMP, "%s: LOOPBK_TEST_EN, the DSI PHY looped "
                          "to the CSI PHY, is not modelled\n",
                          TYPE_ESP32P4_CSI_BRIDGE);
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

static void brg_clk_update(void *opaque, ClockEvent event)
{
    brg_update_req(opaque);
}

static void brg_reset_hold(Object *obj, ResetType type)
{
    ESP32P4CsiBridgeState *s = ESP32P4_CSI_BRIDGE(obj);

    for (int i = 0; i < ARRAY_SIZE(brg_regs); i++) {
        s->regs[brg_regs[i].off / 4] = brg_regs[i].reset;
    }
    brg_flush(s);
    s->frame_bad = false;
    s->lines = 0;
    s->frame_bytes = 0;
    s->cm_row = 0;
    s->req = s->req_single = s->req_last = false;
    brg_take_cfg(s, false);
}

static void brg_reset_exit(Object *obj, ResetType type)
{
    ESP32P4CsiBridgeState *s = ESP32P4_CSI_BRIDGE(obj);

    qemu_set_irq(s->hs_req, 0);
    qemu_set_irq(s->hs_single, 0);
    qemu_set_irq(s->hs_last, 0);
    qemu_set_irq(s->enableclk, 1);
    qemu_set_irq(s->cfg_clk_en, 1);
    brg_update_irq(s);
}

static void brg_init(Object *obj)
{
    ESP32P4CsiBridgeState *s = ESP32P4_CSI_BRIDGE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &brg_ops, s, "esp32p4.csi-bridge",
                          ESP32P4_CSI_REGS_SIZE);
    memory_region_init_io(&s->mem, obj, &brg_mem_ops, s,
                          "esp32p4.mipi-csi-mem", ESP32P4_CSI_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk_bus = qdev_init_clock_in(dev, "bus", brg_clk_update, s,
                                    ClockUpdate);
    qdev_init_gpio_out_named(dev, &s->hs_req, ESP32P4_CSI_HS_REQ, 1);
    qdev_init_gpio_out_named(dev, &s->hs_single, ESP32P4_CSI_HS_SINGLE, 1);
    qdev_init_gpio_out_named(dev, &s->hs_last, ESP32P4_CSI_HS_LAST, 1);
    qdev_init_gpio_out_named(dev, &s->cfg_clk_en, ESP32P4_CSI_CFG_CLK_EN, 1);
    qdev_init_gpio_out_named(dev, &s->enableclk, ESP32P4_CSI_ENABLECLK, 1);
    qdev_init_gpio_in_named(dev, brg_hs_ack, ESP32P4_CSI_HS_ACK, 1);
    s->req_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, brg_req_timer_cb, s);
}

static void brg_finalize(Object *obj)
{
    timer_free(ESP32P4_CSI_BRIDGE(obj)->req_timer);
}

static const VMStateDescription vmstate_brg = {
    .name = TYPE_ESP32P4_CSI_BRIDGE,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(req_timer, ESP32P4CsiBridgeState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4CsiBridgeState, 0x50 / 4),
        VMSTATE_UINT8_ARRAY(buf, ESP32P4CsiBridgeState,
                            ESP32P4_CSI_BUF_WORDS * 8),
        VMSTATE_UINT32(buf_head, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(buf_num, ESP32P4CsiBridgeState),
        VMSTATE_BOOL(capturing, ESP32P4CsiBridgeState),
        VMSTATE_BOOL(frame_bad, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(lines, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(frame_bytes, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(cm_row, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(end_bytes, ESP32P4CsiBridgeState),
        VMSTATE_BOOL(end_pending, ESP32P4CsiBridgeState),
        VMSTATE_BOOL(req, ESP32P4CsiBridgeState),
        VMSTATE_BOOL(req_single, ESP32P4CsiBridgeState),
        VMSTATE_BOOL(req_last, ESP32P4CsiBridgeState),
        VMSTATE_INT64(req_hold_ns, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(burst_len, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(blk_size, ESP32P4CsiBridgeState),
        VMSTATE_UINT32(bursts, ESP32P4CsiBridgeState),
        VMSTATE_END_OF_LIST()
    },
};

static void brg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_brg;
    rc->phases.hold = brg_reset_hold;
    rc->phases.exit = brg_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.mipi] */
static const TypeInfo esp32p4_mipi_csi_types[] = {
    {
        .name = TYPE_ESP32P4_CSI_HOST,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4CsiHostState),
        .instance_init = host_init,
        .instance_finalize = host_finalize,
        .class_init = host_class_init,
    },
    {
        .name = TYPE_ESP32P4_CSI_BRIDGE,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4CsiBridgeState),
        .instance_init = brg_init,
        .instance_finalize = brg_finalize,
        .class_init = brg_class_init,
    },
};

DEFINE_TYPES(esp32p4_mipi_csi_types)
