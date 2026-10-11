/*
 * ESP32-P4 AXI performance monitor (AXI_PERF_MON)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 23 (AXI Performance Monitor), and ESP-IDF's axi_perf_mon_reg.h for v3,
 * the authority for the registers and their reset values (the TRM gives
 * SW_RECORD_STOP_DATA_LIMIT a write limit too, which the header does not
 * have); ESP-IDF's peripherals linker script places the block at
 * 0x500A8000, which the TRM's address map leaves reserved.
 *
 * Two monitoring units, each watching the port AGENT_SELECT gives it (bits
 * 4:0 for unit 0, 9:5 for unit 1: the cache, the VDMA's two masters, the
 * 2D-DMA, the H264 DMA's two masters, PSRAM, the camera and the LCD), with
 * six counters, each measuring the metric its byte of METRIC_SELECT1 (0
 * to 3) or _SELECT2 (4, 5) selects in bits 4:0 and keeping range
 * statistics of instantaneous bandwidth by bits 7:5. A counter's result
 * holds the smallest value measured in bits 15:0 and the largest in bits
 * 31:16, or, for average bandwidth, the data or time count in bits 31:16
 * (0xFFFF once it overflows); deselecting its metric resets it.
 *
 * The monitor sees the AXI transactions the machine's masters and slaves
 * report (esp32p4_axi_perf_mon_transfer()), each made and completed at
 * once, as QEMU's bus makes every access. Of a unit's transactions, those
 * whose ID differs from ID_FILTER on a bit ID_MASK sets are not counted;
 * with the address filter on (ADDR_FILTER_EN), bandwidth counts only those
 * whose address matches, on the bits its mask clears, one of the three
 * filters ADDR_REGION_SEL selects. What the counters measure of them:
 *
 *   - Outstanding transactions (metrics 1, 2): one at a time, as each
 *     completes before the next is made.
 *   - Latencies (metrics 3 to 9: address and data channel handshakes,
 *     WVALID idle cycles, transaction latency): the transactions take no
 *     cycles, so they measure 0. The bus's cycle-by-cycle timing is not
 *     modelled, and selecting such a metric is logged as unimplemented.
 *   - Average bandwidth (10 to 13), while BANDW_TEST_EN and not
 *     BANDW_TEST_STOP: the beats transferred and the AXI_CLK cycles since
 *     the test began, as at the end of the last transaction, each divided
 *     by its VALID_STROBE_NUM plus one.
 *   - Instantaneous bandwidth (14 to 17), while INS_BANDW_TEST_EN: windows
 *     of INS_BANDW_TIME_THR AXI_CLK cycles, timed on the clock whether or
 *     not anything is transferred. At each window's end its beats go to
 *     the data counters, its length to the time counters and the range
 *     statistics (RANGEn counts the windows whose beats fall in
 *     METRIC_RANGEn's [LOW, HIGH]), and to INS_BANDW_DATA0 (the newest) to
 *     3, read windows in bits 15:0 and write windows in bits 31:16. A read
 *     window with fewer beats than SW_RECORD_STOP_RD_DATA_LIMIT raises the
 *     unit's RECORD_CNT_UNDER_LIMIT interrupt (AXI_PERF_MON_INTR, source
 *     130) and stops both units' windows and records, as does
 *     SW_RECORD_STOP_EN for one unit, until SW_RECORD_STOP_CLR.
 *   - The event records, RD_ and WR_AXI_INFO_RECORD0 (the newest) to 3: the
 *     transactions' AxLEN in bits 7:0 and their ID in bits 15:8.
 *
 * No modelled block reports its transactions: the caches fill and write
 * back whole pages of the external memories at once rather than in AXI
 * bursts, and the AXI DMA engines, the camera and the LCD are not
 * modelled. So the counters count nothing and the windows measure an idle
 * port, which their under-limit interrupt reports. The SoC trigger inputs
 * of BANDW_TRIGGER_IN_SEL are not documented: the selections are kept,
 * and software's BANDW_TEST_EN and _STOP start and stop the average
 * bandwidth tests.
 *
 * The registers are behind HP_SYS_CLKRST's AXI_PERF_MON_SYS_CLK and its
 * reset, AXI_PERF_MON_SYS_RST_EN; the monitor counts AXI_CLK, the SoC's
 * SYS_CLK. CLK_EN forces the register clock on and has no other effect.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/esp32p4_axi_perf_mon.h"
#include "migration/vmstate.h"

#define A_CLK_EN                0x000
#define A_AGENT_SELECT          0x004
#define A_COUNTER(c, n)         (0x008 + 8 * (c) + 4 * (n))
#define A_RANGE(m, n)           (0x038 + 8 * (m) + 4 * (n))
#define A_RD_INFO(x, n)         (0x068 + 8 * (x) + 4 * (n))
#define A_WR_INFO(x, n)         (0x088 + 8 * (x) + 4 * (n))
#define A_INS_DATA(x, n)        (0x0a8 + 8 * (x) + 4 * (n))
#define A_METRIC_RANGE(m, n)    (0x0c8 + 8 * (m) + 4 * (n))
#define A_RD_ADDR_MASK(m, n)    (0x0f8 + 8 * (m) + 4 * (n))
#define A_WR_ADDR_MASK(m, n)    (0x110 + 8 * (m) + 4 * (n))
#define A_RD_ADDR_FILTER(m, n)  (0x128 + 8 * (m) + 4 * (n))
#define A_WR_ADDR_FILTER(m, n)  (0x140 + 8 * (m) + 4 * (n))
#define A_METRIC_SELECT(k, n)   (0x158 + 8 * (k) + 4 * (n))
#define A_RD_ADDR_REGION_SEL    0x168
#define A_WR_ADDR_REGION_SEL    0x16c
#define A_ADDR_FILTER_EN        0x170
#define A_SW_RECORD_STOP_EN     0x174
#define A_SW_RECORD_STOP_CLR    0x178
#define A_INS_BANDW_TEST_EN     0x17c
#define A_STOP_DATA_LIMIT(n)    (0x180 + 4 * (n))
#define A_ID_MASK(n)            (0x188 + 4 * (n))
#define A_ID_FILTER(n)          (0x190 + 4 * (n))
#define A_BANDW_TEST_EN         0x198
#define A_BANDW_TEST_STOP       0x19c
#define A_BANDW_TRIGGER_IN_SEL(n) (0x1a0 + 4 * (n))
#define A_WR_STROBE(n)          (0x1a8 + 4 * (n))
#define A_RD_STROBE(n)          (0x1b0 + 4 * (n))
#define A_INS_TIME_THR(n)       (0x1b8 + 4 * (n))
#define A_INT_RAW               0x1c0
#define A_INT_ST                0x1c4
#define A_INT_ENA               0x1c8
#define A_INT_CLR               0x1cc

#define R(a)                    ((a) / 4)
#define UNITS                   ESP32P4_AXI_PERF_MON_UNITS
#define COUNTERS                ESP32P4_AXI_PERF_MON_COUNTERS
#define RD                      0
#define WR                      1

/* The measurement metrics (TRM table 23.5-1) and range metrics (23.5-2). */
enum {
    METRIC_WR_OUTSTANDING = 1,
    METRIC_RD_OUTSTANDING = 2,
    METRIC_WR_ADDR_LATENCY = 3,
    METRIC_RD_ADDR_LATENCY = 4,
    METRIC_WR_DATA_LATENCY = 5,
    METRIC_RD_DATA_LATENCY = 6,
    METRIC_WR_IDLE = 7,
    METRIC_RD_TRANS_LATENCY = 8,
    METRIC_WR_TRANS_LATENCY = 9,
    METRIC_RD_AVG_DATA = 10,
    METRIC_RD_AVG_TIME = 11,
    METRIC_WR_AVG_DATA = 12,
    METRIC_WR_AVG_TIME = 13,
    METRIC_RD_INS_DATA = 14,
    METRIC_RD_INS_TIME = 15,
    METRIC_WR_INS_DATA = 16,
    METRIC_WR_INS_TIME = 17,
};

#define RANGE_RD_INS            1
#define RANGE_WR_INS            2

#define RESULT_MAX              0xffff

typedef struct {
    uint32_t reset;
    uint32_t wmask;
    bool valid;
} ESP32P4AxiPerfMonReg;

static ESP32P4AxiPerfMonReg esp32p4_axi_perf_mon_regs[
    ESP32P4_AXI_PERF_MON_NREGS];

/* The register table: built once, the registers being regular. */
static void esp32p4_axi_perf_mon_build_regs(void)
{
    ESP32P4AxiPerfMonReg *t = esp32p4_axi_perf_mon_regs;
    int n, i;

#define SET(a, r, w) \
    (t[R(a)] = (ESP32P4AxiPerfMonReg) { .reset = (r), .wmask = (w), \
                                        .valid = true })
    SET(A_CLK_EN, 1, 1);
    SET(A_AGENT_SELECT, 0, 0xffffffff);
    for (n = 0; n < UNITS; n++) {
        for (i = 0; i < COUNTERS; i++) {
            SET(A_COUNTER(i, n), 0, 0);
            SET(A_RANGE(i, n), 0, 0);
            SET(A_METRIC_RANGE(i, n), 0, 0xffffffff);
        }
        for (i = 0; i < 4; i++) {
            SET(A_RD_INFO(i, n), 0, 0);
            SET(A_WR_INFO(i, n), 0, 0);
            SET(A_INS_DATA(i, n), 0, 0);
        }
        for (i = 0; i < 3; i++) {
            SET(A_RD_ADDR_MASK(i, n), 0, 0xffffffff);
            SET(A_WR_ADDR_MASK(i, n), 0, 0xffffffff);
            SET(A_RD_ADDR_FILTER(i, n), 0, 0xffffffff);
            SET(A_WR_ADDR_FILTER(i, n), 0, 0xffffffff);
        }
        SET(A_METRIC_SELECT(0, n), 0, 0xffffffff);
        SET(A_METRIC_SELECT(1, n), 0, 0xffffffff);
        SET(A_STOP_DATA_LIMIT(n), 0, 0xffff);
        SET(A_ID_MASK(n), 0, 0xffff);
        SET(A_ID_FILTER(n), 0, 0xffff);
        SET(A_BANDW_TRIGGER_IN_SEL(n), 0, 0xffff);
        SET(A_WR_STROBE(n), 0, 0xffff);
        SET(A_RD_STROBE(n), 0, 0xffff);
        SET(A_INS_TIME_THR(n), 0, 0xffffffff);
    }
    SET(A_RD_ADDR_REGION_SEL, 0, 0x3f);
    SET(A_WR_ADDR_REGION_SEL, 0, 0x3f);
    SET(A_ADDR_FILTER_EN, 0, 0xf);
    SET(A_SW_RECORD_STOP_EN, 0, 0);
    SET(A_SW_RECORD_STOP_CLR, 0, 0);
    SET(A_INS_BANDW_TEST_EN, 0, 0xf);
    SET(A_BANDW_TEST_EN, 0, 0xf);
    SET(A_BANDW_TEST_STOP, 0, 0xf);
    SET(A_INT_RAW, 0, 0);
    SET(A_INT_ST, 0, 0);
    SET(A_INT_ENA, 0, 0x3);
    SET(A_INT_CLR, 0, 0);
#undef SET
}

static uint32_t *esp32p4_apm_r(ESP32P4AxiPerfMonState *s, hwaddr a)
{
    return &s->regs[R(a)];
}

static void esp32p4_axi_perf_mon_update_irq(ESP32P4AxiPerfMonState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R(A_INT_RAW)] & s->regs[R(A_INT_ENA)]));
}

/* The metric (bits 4:0) or range metric (7:5) of counter @c of unit @n. */
static unsigned esp32p4_apm_select(ESP32P4AxiPerfMonState *s, int n, int c)
{
    return extract32(s->regs[R(A_METRIC_SELECT(c / 4, n))], 8 * (c % 4), 8);
}

static unsigned esp32p4_apm_metric(ESP32P4AxiPerfMonState *s, int n, int c)
{
    return esp32p4_apm_select(s, n, c) & 0x1f;
}

static unsigned esp32p4_apm_range_metric(ESP32P4AxiPerfMonState *s, int n,
                                         int c)
{
    return esp32p4_apm_select(s, n, c) >> 5;
}

/* A measurement for a min/max counter. */
static void esp32p4_apm_minmax(ESP32P4AxiPerfMonState *s, int n, int c,
                               uint64_t v)
{
    uint32_t *r = esp32p4_apm_r(s, A_COUNTER(c, n));
    uint32_t value = MIN(v, RESULT_MAX);

    if (!(s->counter_valid[n] & BIT(c))) {
        *r = value | (value << 16);
        s->counter_valid[n] |= BIT(c);
    } else {
        *r = MIN(*r & 0xffff, value) | (MAX(*r >> 16, value) << 16);
    }
}

/* Every counter of unit @n measuring @metric takes the min/max of @v. */
static void esp32p4_apm_measure(ESP32P4AxiPerfMonState *s, int n,
                                unsigned metric, uint64_t v)
{
    for (int c = 0; c < COUNTERS; c++) {
        if (esp32p4_apm_metric(s, n, c) == metric) {
            esp32p4_apm_minmax(s, n, c, v);
        }
    }
}

/* Every counter of unit @n measuring @metric holds @v in bits 31:16. */
static void esp32p4_apm_total(ESP32P4AxiPerfMonState *s, int n,
                              unsigned metric, uint64_t v)
{
    for (int c = 0; c < COUNTERS; c++) {
        if (esp32p4_apm_metric(s, n, c) == metric) {
            *esp32p4_apm_r(s, A_COUNTER(c, n)) = MIN(v, RESULT_MAX) << 16;
            s->counter_valid[n] |= BIT(c);
        }
    }
}

static uint64_t esp32p4_apm_cycles(ESP32P4AxiPerfMonState *s, int64_t ns)
{
    return clock_ns_to_ticks(s->axi_clk, MAX(ns, 0));
}

/* VALID_STROBE_NUM's time (bits 7:0) or data (15:8) divisor, less one. */
static unsigned esp32p4_apm_strobe(ESP32P4AxiPerfMonState *s, int n, int dir,
                                   bool data)
{
    uint32_t v = s->regs[R(dir == WR ? A_WR_STROBE(n) : A_RD_STROBE(n))];

    return extract32(v, data ? 8 : 0, 8);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_apm_avg_update(ESP32P4AxiPerfMonState *s, int n, int dir)
{
    ESP32P4AxiPerfMonMeasure *m = &s->measure[n][dir];
    uint64_t data = m->avg_beats / (esp32p4_apm_strobe(s, n, dir, true) + 1);
    uint64_t time = esp32p4_apm_cycles(s, m->avg_last_ns - m->avg_start_ns) /
                    (esp32p4_apm_strobe(s, n, dir, false) + 1);

    esp32p4_apm_total(s, n, dir == RD ? METRIC_RD_AVG_DATA
                                      : METRIC_WR_AVG_DATA, data);
    esp32p4_apm_total(s, n, dir == RD ? METRIC_RD_AVG_TIME
                                      : METRIC_WR_AVG_TIME, time);
}

static int64_t esp32p4_apm_window_ns(ESP32P4AxiPerfMonState *s, int n,
                                     int dir)
{
    uint32_t thr = extract32(s->regs[R(A_INS_TIME_THR(n))], dir * 16, 16);

    if (!thr || !clock_get(s->axi_clk)) {
        return 0;
    }
    return clock_ticks_to_ns(s->axi_clk, thr);
}

static bool esp32p4_apm_ins_enabled(ESP32P4AxiPerfMonState *s, int n,
                                    int dir)
{
    return extract32(s->regs[R(A_INS_BANDW_TEST_EN)], 2 * n + dir, 1) &&
           !s->record_stopped[n];
}

/* Start the next window, if the test runs and the clock does. */
static void esp32p4_apm_ins_start(ESP32P4AxiPerfMonState *s, int n, int dir,
                                  int64_t now)
{
    ESP32P4AxiPerfMonMeasure *m = &s->measure[n][dir];
    int64_t len = esp32p4_apm_window_ns(s, n, dir);

    m->ins_beats = 0;
    m->ins_start_ns = now;
    m->ins_running = esp32p4_apm_ins_enabled(s, n, dir) && len;
    if (m->ins_running) {
        timer_mod(m->ins_timer, now + len);
    } else {
        timer_del(m->ins_timer);
    }
}

/*
 * The end of an instantaneous bandwidth window: its beats and length to
 * the counters, the range statistics and the records, and, for a read
 * window under the limit, the interrupt that stops both units.
 */
/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_apm_ins_end(ESP32P4AxiPerfMonState *s, int n, int dir)
{
    ESP32P4AxiPerfMonMeasure *m = &s->measure[n][dir];
    uint32_t data = MIN(m->ins_beats, RESULT_MAX);
    uint32_t thr = extract32(s->regs[R(A_INS_TIME_THR(n))], dir * 16, 16);
    unsigned range = dir == RD ? RANGE_RD_INS : RANGE_WR_INS;
    int i;

    esp32p4_apm_measure(s, n, dir == RD ? METRIC_RD_INS_DATA
                                        : METRIC_WR_INS_DATA, data);
    esp32p4_apm_measure(s, n, dir == RD ? METRIC_RD_INS_TIME
                                        : METRIC_WR_INS_TIME, thr);
    for (i = 0; i < COUNTERS; i++) {
        uint32_t lim = s->regs[R(A_METRIC_RANGE(i, n))];
        uint32_t *count = esp32p4_apm_r(s, A_RANGE(i, n));

        if (esp32p4_apm_range_metric(s, n, i) == range &&
            data >= (lim >> 16) && data <= (lim & 0xffff) &&
            *count < RESULT_MAX) {
            (*count)++;
        }
    }
    if (!s->record_stopped[n]) {
        for (i = 3; i > 0; i--) {
            *esp32p4_apm_r(s, A_INS_DATA(i, n)) =
                deposit32(s->regs[R(A_INS_DATA(i, n))], dir * 16, 16,
                          extract32(s->regs[R(A_INS_DATA(i - 1, n))],
                                    dir * 16, 16));
        }
        *esp32p4_apm_r(s, A_INS_DATA(0, n)) =
            deposit32(s->regs[R(A_INS_DATA(0, n))], dir * 16, 16, data);
    }
    if (dir == RD && data < (s->regs[R(A_STOP_DATA_LIMIT(n))] & 0xffff)) {
        s->regs[R(A_INT_RAW)] |= BIT(n);
        for (i = 0; i < UNITS; i++) {
            s->record_stopped[i] = true;
        }
        esp32p4_axi_perf_mon_update_irq(s);
    }
}

static void esp32p4_apm_ins_timer(ESP32P4AxiPerfMonState *s, int n, int dir)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    esp32p4_apm_ins_end(s, n, dir);
    if (s->record_stopped[n]) {
        for (i = 0; i < UNITS * 2; i++) {
            ESP32P4AxiPerfMonMeasure *m = &s->measure[i / 2][i % 2];

            m->ins_running = false;
            timer_del(m->ins_timer);
        }
        return;
    }
    esp32p4_apm_ins_start(s, n, dir, now);
}

static void esp32p4_apm_timer_u0_rd(void *opaque)
{
    esp32p4_apm_ins_timer(opaque, 0, RD);
}

static void esp32p4_apm_timer_u0_wr(void *opaque)
{
    esp32p4_apm_ins_timer(opaque, 0, WR);
}

static void esp32p4_apm_timer_u1_rd(void *opaque)
{
    esp32p4_apm_ins_timer(opaque, 1, RD);
}

static void esp32p4_apm_timer_u1_wr(void *opaque)
{
    esp32p4_apm_ins_timer(opaque, 1, WR);
}

/* Whether @addr passes unit @n's address filter for direction @dir. */
static bool esp32p4_apm_addr_passes(ESP32P4AxiPerfMonState *s, int n, int dir,
                                    uint32_t addr)
{
    uint32_t sel = s->regs[R(dir == RD ? A_RD_ADDR_REGION_SEL
                                       : A_WR_ADDR_REGION_SEL)];

    if (!extract32(s->regs[R(A_ADDR_FILTER_EN)], n + 2 * dir, 1)) {
        return true;
    }
    for (int m = 0; m < 3; m++) {
        uint32_t mask = s->regs[R(dir == RD ? A_RD_ADDR_MASK(m, n)
                                            : A_WR_ADDR_MASK(m, n))];
        uint32_t filter = s->regs[R(dir == RD ? A_RD_ADDR_FILTER(m, n)
                                              : A_WR_ADDR_FILTER(m, n))];

        if (extract32(sel, 3 * n + m, 1) && !((addr ^ filter) & ~mask)) {
            return true;
        }
    }
    return false;
}

/* [spec:nuos:req:emu.esp32p4.debug] */
void esp32p4_axi_perf_mon_transfer(ESP32P4AxiPerfMonState *s, int agent,
                                   bool write, uint32_t addr, uint8_t id,
                                   unsigned beats)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int dir = write ? WR : RD;
    int n, i;

    for (n = 0; n < UNITS; n++) {
        uint32_t mask = extract32(s->regs[R(A_ID_MASK(n))], 8 * dir, 8);
        uint32_t filter = extract32(s->regs[R(A_ID_FILTER(n))], 8 * dir, 8);
        ESP32P4AxiPerfMonMeasure *m = &s->measure[n][dir];

        if (extract32(s->regs[R(A_AGENT_SELECT)], 5 * n, 5) != agent ||
            ((id ^ filter) & mask)) {
            continue;
        }
        if (!s->record_stopped[n]) {
            hwaddr rec0 = write ? A_WR_INFO(0, n) : A_RD_INFO(0, n);

            for (i = 3; i > 0; i--) {
                s->regs[R(rec0) + 2 * i] = s->regs[R(rec0) + 2 * (i - 1)];
            }
            s->regs[R(rec0)] = ((beats - 1) & 0xff) | ((uint32_t)id << 8);
        }
        esp32p4_apm_measure(s, n, write ? METRIC_WR_OUTSTANDING
                                        : METRIC_RD_OUTSTANDING, 1);
        if (write) {
            esp32p4_apm_measure(s, n, METRIC_WR_ADDR_LATENCY, 0);
            esp32p4_apm_measure(s, n, METRIC_WR_DATA_LATENCY, 0);
            esp32p4_apm_measure(s, n, METRIC_WR_IDLE, 0);
            esp32p4_apm_measure(s, n, METRIC_WR_TRANS_LATENCY, 0);
        } else {
            esp32p4_apm_measure(s, n, METRIC_RD_ADDR_LATENCY, 0);
            esp32p4_apm_measure(s, n, METRIC_RD_DATA_LATENCY, 0);
            esp32p4_apm_measure(s, n, METRIC_RD_TRANS_LATENCY, 0);
        }
        if (!esp32p4_apm_addr_passes(s, n, dir, addr)) {
            continue;
        }
        if (m->avg_running) {
            m->avg_beats += beats;
            m->avg_last_ns = now;
            esp32p4_apm_avg_update(s, n, dir);
        }
        if (m->ins_running) {
            m->ins_beats += beats;
        }
    }
}

/* BANDW_TEST_EN and _STOP: start and stop the average bandwidth tests. */
static void esp32p4_apm_update_avg(ESP32P4AxiPerfMonState *s, uint32_t old_en)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t en = s->regs[R(A_BANDW_TEST_EN)];
    uint32_t stop = s->regs[R(A_BANDW_TEST_STOP)];

    for (int b = 0; b < 2 * UNITS; b++) {
        ESP32P4AxiPerfMonMeasure *m = &s->measure[b / 2][b % 2];

        if ((en & BIT(b)) && !(old_en & BIT(b))) {
            m->avg_start_ns = now;
            m->avg_last_ns = now;
            m->avg_beats = 0;
        }
        m->avg_running = (en & BIT(b)) && !(stop & BIT(b));
    }
}

/* INS_BANDW_TEST_EN and the window lengths: start or stop the windows. */
static void esp32p4_apm_update_ins(ESP32P4AxiPerfMonState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (int b = 0; b < 2 * UNITS; b++) {
        int n = b / 2, dir = b % 2;
        ESP32P4AxiPerfMonMeasure *m = &s->measure[n][dir];
        bool want = esp32p4_apm_ins_enabled(s, n, dir) &&
                    esp32p4_apm_window_ns(s, n, dir);

        if (want && !m->ins_running) {
            esp32p4_apm_ins_start(s, n, dir, now);
        } else if (!want && m->ins_running) {
            m->ins_running = false;
            timer_del(m->ins_timer);
        }
    }
}

/*
 * Counters whose metric changes start over; a latency metric, which the
 * model measures as zero, is logged.
 */
static void esp32p4_apm_select_changed(ESP32P4AxiPerfMonState *s, int n,
                                       int k, uint32_t old)
{
    uint32_t now = s->regs[R(A_METRIC_SELECT(k, n))];

    for (int i = 0; i < 4 && 4 * k + i < COUNTERS; i++) {
        int c = 4 * k + i;
        uint32_t was = extract32(old, 8 * i, 8);
        uint32_t is = extract32(now, 8 * i, 8);
        unsigned metric = is & 0x1f;

        if ((was & 0x1f) != metric) {
            *esp32p4_apm_r(s, A_COUNTER(c, n)) = 0;
            s->counter_valid[n] &= ~BIT(c);
            if (metric >= METRIC_WR_ADDR_LATENCY &&
                metric <= METRIC_WR_TRANS_LATENCY) {
                qemu_log_mask(LOG_UNIMP, "%s: unit %d counter %d: the bus's "
                              "cycle timing is not modelled; latency metric "
                              "%u measures 0\n", TYPE_ESP32P4_AXI_PERF_MON,
                              n, c, metric);
            }
        }
        if ((was >> 5) != (is >> 5)) {
            *esp32p4_apm_r(s, A_RANGE(c, n)) = 0;
        }
    }
}

static uint64_t esp32p4_axi_perf_mon_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    ESP32P4AxiPerfMonState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;

    if (reg >= ESP32P4_AXI_PERF_MON_NREGS * 4 ||
        !esp32p4_axi_perf_mon_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_AXI_PERF_MON, addr);
        return 0;
    }
    switch (reg) {
    case A_INT_ST:
        v = s->regs[R(A_INT_RAW)] & s->regs[R(A_INT_ENA)];
        break;
    case A_INT_CLR:
    case A_SW_RECORD_STOP_EN:
    case A_SW_RECORD_STOP_CLR:
        v = 0;
        break;
    default:
        v = s->regs[R(reg)];
        break;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static void esp32p4_axi_perf_mon_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned size)
{
    ESP32P4AxiPerfMonState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t bytes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & bytes;
    const ESP32P4AxiPerfMonReg *info;
    uint32_t *r, old;
    int n;

    if (reg >= ESP32P4_AXI_PERF_MON_NREGS * 4 ||
        !esp32p4_axi_perf_mon_regs[R(reg)].valid) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_AXI_PERF_MON, addr);
        return;
    }
    info = &esp32p4_axi_perf_mon_regs[R(reg)];
    r = &s->regs[R(reg)];

    switch (reg) {
    case A_INT_RAW:
    case A_INT_CLR:
        s->regs[R(A_INT_RAW)] &= ~(v & 0x3);
        esp32p4_axi_perf_mon_update_irq(s);
        return;
    case A_SW_RECORD_STOP_EN:
        for (n = 0; n < UNITS; n++) {
            if (v & BIT(n)) {
                s->record_stopped[n] = true;
            }
        }
        esp32p4_apm_update_ins(s);
        return;
    case A_SW_RECORD_STOP_CLR:
        for (n = 0; n < UNITS; n++) {
            if (v & BIT(n)) {
                s->record_stopped[n] = false;
            }
        }
        esp32p4_apm_update_ins(s);
        return;
    default:
        break;
    }
    if (!info->wmask) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_AXI_PERF_MON, addr);
        return;
    }
    old = *r;
    *r = (*r & ~(info->wmask & bytes)) | (v & info->wmask);

    switch (reg) {
    case A_METRIC_SELECT(0, 0):
    case A_METRIC_SELECT(0, 1):
    case A_METRIC_SELECT(1, 0):
    case A_METRIC_SELECT(1, 1):
        n = R(reg - A_METRIC_SELECT(0, 0)) % 2;
        esp32p4_apm_select_changed(s, n, R(reg - A_METRIC_SELECT(0, 0)) / 2,
                                   old);
        break;
    case A_BANDW_TEST_EN:
        esp32p4_apm_update_avg(s, old);
        break;
    case A_BANDW_TEST_STOP:
        esp32p4_apm_update_avg(s, s->regs[R(A_BANDW_TEST_EN)]);
        break;
    case A_INS_BANDW_TEST_EN:
    case A_INS_TIME_THR(0):
    case A_INS_TIME_THR(1):
        esp32p4_apm_update_ins(s);
        break;
    case A_INT_ENA:
        esp32p4_axi_perf_mon_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp32p4_axi_perf_mon_ops = {
    .read = esp32p4_axi_perf_mon_read,
    .write = esp32p4_axi_perf_mon_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* A stopped AXI_CLK stops the windows, which start again with it. */
static void esp32p4_axi_perf_mon_clock_update(void *opaque, ClockEvent event)
{
    ESP32P4AxiPerfMonState *s = opaque;

    for (int b = 0; b < 2 * UNITS; b++) {
        ESP32P4AxiPerfMonMeasure *m = &s->measure[b / 2][b % 2];

        m->ins_running = false;
        timer_del(m->ins_timer);
    }
    esp32p4_apm_update_ins(s);
}

static void esp32p4_axi_perf_mon_reset_hold(Object *obj, ResetType type)
{
    ESP32P4AxiPerfMonState *s = ESP32P4_AXI_PERF_MON(obj);

    for (int i = 0; i < ESP32P4_AXI_PERF_MON_NREGS; i++) {
        s->regs[i] = esp32p4_axi_perf_mon_regs[i].reset;
    }
    for (int n = 0; n < UNITS; n++) {
        s->record_stopped[n] = false;
        s->counter_valid[n] = 0;
        for (int dir = 0; dir < 2; dir++) {
            ESP32P4AxiPerfMonMeasure *m = &s->measure[n][dir];

            m->avg_running = false;
            m->avg_beats = 0;
            m->avg_start_ns = 0;
            m->avg_last_ns = 0;
            m->ins_running = false;
            m->ins_beats = 0;
            m->ins_start_ns = 0;
            timer_del(m->ins_timer);
        }
    }
}

static void esp32p4_axi_perf_mon_reset_exit(Object *obj, ResetType type)
{
    esp32p4_axi_perf_mon_update_irq(ESP32P4_AXI_PERF_MON(obj));
}

static void esp32p4_axi_perf_mon_init(Object *obj)
{
    ESP32P4AxiPerfMonState *s = ESP32P4_AXI_PERF_MON(obj);
    static QEMUTimerCB *const cb[UNITS][2] = {
        { esp32p4_apm_timer_u0_rd, esp32p4_apm_timer_u0_wr },
        { esp32p4_apm_timer_u1_rd, esp32p4_apm_timer_u1_wr },
    };

    memory_region_init_io(&s->iomem, obj, &esp32p4_axi_perf_mon_ops, s,
                          "esp32p4.axi-perf-mon", ESP32P4_AXI_PERF_MON_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->axi_clk = qdev_init_clock_in(DEVICE(obj), "axi",
                                    esp32p4_axi_perf_mon_clock_update, s,
                                    ClockUpdate);
    for (int n = 0; n < UNITS; n++) {
        for (int dir = 0; dir < 2; dir++) {
            s->measure[n][dir].ins_timer =
                timer_new_ns(QEMU_CLOCK_VIRTUAL, cb[n][dir], s);
        }
    }
}

static const VMStateDescription vmstate_esp32p4_apm_measure = {
    .name = "esp32p4-axi-perf-mon/measure",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(avg_running, ESP32P4AxiPerfMonMeasure),
        VMSTATE_INT64(avg_start_ns, ESP32P4AxiPerfMonMeasure),
        VMSTATE_UINT64(avg_beats, ESP32P4AxiPerfMonMeasure),
        VMSTATE_INT64(avg_last_ns, ESP32P4AxiPerfMonMeasure),
        VMSTATE_BOOL(ins_running, ESP32P4AxiPerfMonMeasure),
        VMSTATE_INT64(ins_start_ns, ESP32P4AxiPerfMonMeasure),
        VMSTATE_UINT64(ins_beats, ESP32P4AxiPerfMonMeasure),
        VMSTATE_TIMER_PTR(ins_timer, ESP32P4AxiPerfMonMeasure),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_axi_perf_mon = {
    .name = TYPE_ESP32P4_AXI_PERF_MON,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(axi_clk, ESP32P4AxiPerfMonState),
        VMSTATE_UINT32_ARRAY(regs, ESP32P4AxiPerfMonState,
                             ESP32P4_AXI_PERF_MON_NREGS),
        VMSTATE_BOOL_ARRAY(record_stopped, ESP32P4AxiPerfMonState, UNITS),
        VMSTATE_STRUCT_2DARRAY(measure, ESP32P4AxiPerfMonState, UNITS, 2, 1,
                               vmstate_esp32p4_apm_measure,
                               ESP32P4AxiPerfMonMeasure),
        VMSTATE_UINT32_ARRAY(counter_valid, ESP32P4AxiPerfMonState, UNITS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_axi_perf_mon_class_init(ObjectClass *klass,
                                            const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    esp32p4_axi_perf_mon_build_regs();
    rc->phases.hold = esp32p4_axi_perf_mon_reset_hold;
    rc->phases.exit = esp32p4_axi_perf_mon_reset_exit;
    dc->vmsd = &vmstate_esp32p4_axi_perf_mon;
}

/* [spec:nuos:req:emu.esp32p4.debug] */
static const TypeInfo esp32p4_axi_perf_mon_info = {
    .name          = TYPE_ESP32P4_AXI_PERF_MON,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4AxiPerfMonState),
    .instance_init = esp32p4_axi_perf_mon_init,
    .class_init    = esp32p4_axi_perf_mon_class_init,
};

static void esp32p4_axi_perf_mon_register_types(void)
{
    type_register_static(&esp32p4_axi_perf_mon_info);
}
type_init(esp32p4_axi_perf_mon_register_types)
