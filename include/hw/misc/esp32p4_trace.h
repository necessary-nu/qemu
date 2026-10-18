/*
 * ESP32-P4 RISC-V trace encoder (TRACE)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_TRACE_H
#define HW_MISC_ESP32P4_TRACE_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_TRACE "esp32p4-trace"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4TraceState, ESP32P4_TRACE)

#define ESP32P4_TRACE0_BASE     0x3ff04000
#define ESP32P4_TRACE1_BASE     0x3ff05000
#define ESP32P4_TRACE_SIZE      0x1000

/* The encoder's FIFO, 128 deep and 8 bits wide (TRM 3.4). */
#define ESP32P4_TRACE_FIFO      128

/*
 * With TRACE_STALL_ENA the stall request is asserted while the FIFO is
 * "almost full" (TRM 3.5.3, 3.8.1), which the TRM does not quantify: while
 * it has less room than the most the encoder puts into it for one
 * instruction, two packets of the longest, 13 bytes (TRM 3.5.5), and an
 * anchor, 14 zero bytes, so that the instructions the core retires never
 * overflow it.
 */
#define ESP32P4_TRACE_STALL_ROOM (2 * 13 + 14)

/*
 * Named GPIO input ESP32P4_TRACE_CORE_RESET: high while the traced core
 * is held in reset, the encoder's reset sideband signal (TRACE_RESET_ENA).
 *
 * Sysbus IRQ 0 is COREn_TRACE_INTR (source 97 for core 0, 98 for core 1).
 * Clock input "sys-clk": TRACE_SYS_CLK, the clock of its AHB master,
 * which writes the trace memory. Links "cpu" (the traced HP core) and
 * "pms" (the bus, through which it writes); property "core", the core's
 * number. The master's writes into a memory controller's window wait on
 * its held AXI port as the other DMA masters' do (esp32p4_mspi_dma_wait()).
 */
#define ESP32P4_TRACE_CORE_RESET "core-reset"

/* An instruction, or a trap, as the encoder's pipeline holds it. */
typedef struct ESP32P4TraceInsn {
    uint32_t iaddr;
    uint32_t tval;
    uint8_t itype;
    uint8_t priv;
    uint8_t cause;
    bool retired;
    bool qualified;
    bool notify;
    bool valid;
} ESP32P4TraceInsn;

struct ESP32P4CPU;
struct ESP32P4PmsState;

struct ESP32P4TraceState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    /*
     * Drives the interrupt once the core's block of translated code ends,
     * for a report made in its middle (see esp32p4_cpu_mid_block()).
     */
    QEMUTimer *irq_timer;
    Clock *sys_clk;
    struct ESP32P4CPU *cpu;
    struct ESP32P4PmsState *pms;
    uint8_t core;
    /*
     * Taken by the core's reports, which come without the BQL, and by
     * everything else, which holds it first.
     */
    QemuMutex lock;

    /* Registers. */
    uint32_t mem_start;
    uint32_t mem_end;
    uint32_t mem_current;
    uint32_t intr_ena;
    uint32_t intr_raw;
    uint32_t trigger;
    uint32_t config;
    uint32_t filter_control;
    uint32_t filter_match_control;
    uint32_t filter_comparator_control;
    uint32_t p_match;
    uint32_t s_match;
    uint32_t resync_prolonged;
    uint32_t ahb_config;
    uint32_t clock_gate;
    uint32_t date;

    /* TRACE_WORK_STATUS: idle, working, waiting, lost. */
    uint32_t work_status;
    /* The core is held in reset. */
    bool core_reset;
    /* The level last driven on the interrupt. */
    bool irq_level;
    /*
     * The master's write of the FIFO's oldest bytes waits on a memory
     * controller's held AXI port (esp32p4_mspi_dma_wait()), until dma_wait
     * says the port takes accesses again; and the stall request to the
     * core is asserted (TRACE_STALL_ENA).
     */
    ESP32P4MspiDmaWait dma_wait;
    bool held;
    bool stall;

    /* The FIFO's bytes, oldest at fifo_head. */
    uint8_t fifo[ESP32P4_TRACE_FIFO];
    uint32_t fifo_head;
    uint32_t fifo_count;

    /*
     * The encoder: the instruction started and not yet retired, and its
     * kind; whether the next to start is to be notified; the pipeline
     * (previous, current, next); the branches not yet reported, bit n
     * for the n-th, set for not taken; the resynchronisation count, and
     * the cycles counted and their count at the last synchronisation;
     * whether the last trap was
     * reported with its address; the address last reported; the packet
     * index; packets since the last anchor; whether a trace-lost support
     * packet is owed.
     */
    bool pend_valid;
    uint32_t pend_pc;
    uint32_t pend_kind;
    uint8_t pend_priv;
    bool pend_notify;
    bool notify_next;
    ESP32P4TraceInsn pipe[3];
    uint32_t branches;
    uint32_t branch_map;
    uint32_t resync_count;
    uint64_t cycles;
    uint64_t sync_cycles;
    bool resync_due;
    bool trap_reported;
    uint32_t last_addr;
    uint16_t index;
    uint32_t anchor_count;
    bool lost_owed;
    /* The encoder has been started since its reset. */
    bool ran;

    /*
     * The filter: qualification by an exception or interrupt that matched
     * (and the trap depth whose return ends it), and the comparators'
     * latch (match mode 3).
     */
    bool trap_match;
    uint32_t trap_depth;
    bool comp_latch;
};

#endif
