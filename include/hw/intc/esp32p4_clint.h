/*
 * ESP32-P4 HP cores' core-local interrupts (CLINT): the machine timer and
 * software interrupts
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_ESP32P4_CLINT_H
#define HW_INTC_ESP32P4_CLINT_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "target/riscv/esp32p4_cpu.h"

#define TYPE_ESP32P4_CLINT "esp32p4-clint"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ClintState, ESP32P4_CLINT)

#define ESP32P4_CLINT_CORES     2

/*
 * The device's MMIO region, at 0x20000000: the accessing core's own
 * registers in the first 64 KiB and the other core's in the second.
 */
#define ESP32P4_CLINT_BASE      0x20000000
#define ESP32P4_CLINT_WINDOW    0x20000

/*
 * Named GPIO outputs, a line per core: its machine software interrupt
 * (MSIP), to its CLIC's interrupt 3, and its machine timer interrupt, to
 * interrupt 7.
 *
 * Clock inputs: "mtime", which the shared counter counts; "core0" and
 * "core1", each core's CLIC bus clock, without which its registers drop
 * writes.
 *
 * Link properties "cpu0" and "cpu1": the HP cores, whose time CSR reads
 * the counter.
 */
#define ESP32P4_CLINT_MSIP      "msip"
#define ESP32P4_CLINT_MTIP      "mtip"

typedef struct ESP32P4ClintCore {
    struct ESP32P4ClintState *s;
    int index;
    ESP32P4CPU *cpu;
    Clock *clk;
    qemu_irq msip_irq;
    qemu_irq mtip_irq;
    QEMUTimer *timer;

    uint32_t msip;
    uint64_t mtimecmp;
    uint32_t mtimeload_lo;
    uint32_t mtimeload_hi;
    /* mtimectl: MTIME_OVF and MTIME_SAM, and on core 0 MTIME_EN. */
    uint32_t mtimectl;
    /*
     * The half of the count a read of the other half sampled, for the
     * next read of this half (ESP32P4_CLINT_SAMPLED_*), or none.
     */
    uint32_t sample;
    uint8_t sampled;
    /* The level driven on the timer interrupt. */
    bool mtip;
} ESP32P4ClintCore;

struct ESP32P4ClintState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *mtime_clk;
    ESP32P4ClintCore core[ESP32P4_CLINT_CORES];

    /* The count at sync_ns, and whether it is counting. */
    uint64_t count;
    int64_t sync_ns;
    bool running;
};

#endif
