/*
 * ESP32-P4 AXI performance monitor (AXI_PERF_MON)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_AXI_PERF_MON_H
#define HW_MISC_ESP32P4_AXI_PERF_MON_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_AXI_PERF_MON "esp32p4-axi-perf-mon"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4AxiPerfMonState, ESP32P4_AXI_PERF_MON)

#define ESP32P4_AXI_PERF_MON_BASE   0x500a8000
#define ESP32P4_AXI_PERF_MON_SIZE   0x1000
/* The registers, 0x000 to 0x1CC, one word each. */
#define ESP32P4_AXI_PERF_MON_NREGS  (0x1d0 / 4)

/* The monitoring units and each one's measurement counters. */
#define ESP32P4_AXI_PERF_MON_UNITS      2
#define ESP32P4_AXI_PERF_MON_COUNTERS   6

/*
 * The monitored ports, AGENT_SELECT's values (TRM table 23.5-3), the
 * @agent of esp32p4_axi_perf_mon_transfer().
 */
enum {
    ESP32P4_AXI_AGENT_CACHE = 1,
    ESP32P4_AXI_AGENT_VDMA_MST1 = 2,
    ESP32P4_AXI_AGENT_VDMA_MST2 = 3,
    ESP32P4_AXI_AGENT_DMA2D = 4,
    ESP32P4_AXI_AGENT_H264_DMA_MST1 = 5,
    ESP32P4_AXI_AGENT_H264_DMA_MST2 = 6,
    ESP32P4_AXI_AGENT_PSRAM = 7,
    ESP32P4_AXI_AGENT_CAMERA = 8,
    ESP32P4_AXI_AGENT_LCD = 9,
};

/* A measurement in progress, of one unit in one direction. */
typedef struct ESP32P4AxiPerfMonMeasure {
    /* Average bandwidth: running since, its beats, its last burst's end. */
    bool avg_running;
    int64_t avg_start_ns;
    uint64_t avg_beats;
    int64_t avg_last_ns;
    /*
     * Instantaneous bandwidth: the AXI clock's cycles counted from the
     * first of a train of windows, the cycle the current one ends at, and
     * its beats.
     */
    bool ins_running;
    ESP32P4ClkCount ins_edges;
    uint64_t ins_end;
    uint64_t ins_beats;
    QEMUTimer *ins_timer;
} ESP32P4AxiPerfMonMeasure;

struct ESP32P4AxiPerfMonState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    /* AXI_CLK, the clock of the monitored bus. */
    Clock *axi_clk;

    /* The registers, results included, indexed by offset / 4. */
    uint32_t regs[ESP32P4_AXI_PERF_MON_NREGS];
    /* Per unit: event recording stopped (by software or an interrupt). */
    bool record_stopped[ESP32P4_AXI_PERF_MON_UNITS];
    /* Per unit, read (0) and write (1). */
    ESP32P4AxiPerfMonMeasure measure[ESP32P4_AXI_PERF_MON_UNITS][2];
    /* Per unit: a counter has measured something (its result is valid). */
    uint32_t counter_valid[ESP32P4_AXI_PERF_MON_UNITS];
};

/*
 * An AXI transaction on the bus of monitored port @agent: a read or a
 * write of @beats data beats (AxLEN + 1) at @addr with transaction ID @id,
 * made and completed now. The masters and slaves whose transactions the
 * machine can see report them here.
 */
void esp32p4_axi_perf_mon_transfer(ESP32P4AxiPerfMonState *s, int agent,
                                   bool write, uint32_t addr, uint8_t id,
                                   unsigned beats);

#endif
