/*
 * RP2350 trace capture FIFO (CORESIGHT_TRACE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_CORESIGHT_TRACE_H
#define HW_MISC_RP2350_CORESIGHT_TRACE_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_CORESIGHT_TRACE "rp2350-coresight-trace"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350CoreSightTraceState, RP2350_CORESIGHT_TRACE)

#define RP2350_CORESIGHT_TRACE_SIZE 0x100000
#define RP2350_TRACE_FIFO_DEPTH 8

struct RP2350CoreSightTraceState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* High while the FIFO holds data: DREQ_CORESIGHT (53). */
    qemu_irq dreq;

    uint32_t ctrl_status;
    uint32_t fifo[RP2350_TRACE_FIFO_DEPTH];
    uint8_t fifo_head;
    uint8_t fifo_count;
};

/*
 * One sample of the TPIU's 32-bit TRACEDATA output, pushed on a clk_sys
 * edge when the TPIU qualifies it as trace data. Dropped, and the
 * overflow flag set, when the FIFO is full; ignored while it is held
 * flushed.
 */
void rp2350_coresight_trace_capture(RP2350CoreSightTraceState *s,
                                    uint32_t word);

#endif
