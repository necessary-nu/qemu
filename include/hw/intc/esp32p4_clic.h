/*
 * ESP32-P4 HP cores' core-local interrupt controllers (CLIC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_ESP32P4_CLIC_H
#define HW_INTC_ESP32P4_CLIC_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "target/riscv/esp32p4_cpu.h"

#define TYPE_ESP32P4_CLIC "esp32p4-clic"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ClicState, ESP32P4_CLIC)

#define ESP32P4_CLIC_CORES      2

/* Interrupt IDs: 48 of them, of which 3, 7 and 16 to 47 exist. */
#define ESP32P4_CLIC_NUM_IRQS   48
#define ESP32P4_CLIC_IRQ_MSIP   3
#define ESP32P4_CLIC_IRQ_MTIP   7
#define ESP32P4_CLIC_IRQ_EXT0   16
#define ESP32P4_CLIC_EXT_IRQS   32

/*
 * The device's MMIO regions: the machine-mode registers at 0x20800000 and
 * the user-mode view at 0x20B00000, each 128 KiB: the accessing core's own
 * CLIC in the first 64 KiB and the other core's in the second.
 */
#define ESP32P4_CLIC_MMIO_M     0
#define ESP32P4_CLIC_MMIO_U     1
#define ESP32P4_CLIC_BASE       0x20800000
#define ESP32P4_CLIC_U_BASE     0x20b00000
#define ESP32P4_CLIC_WINDOW     0x20000

/*
 * Named GPIO inputs.
 *
 * ESP32P4_CLIC_IRQ: the interrupt lines of both cores' CLICs, line
 * ESP32P4_CLIC_IRQ_LINE(core, id) for interrupt @id of HP core @core: the
 * CLINT's software (3) and timer (7) interrupts, and the external
 * interrupts 16 to 47, which the interrupt matrix drives. Lines of IDs the
 * CLIC does not have are ignored.
 * ESP32P4_CLIC_CORE_RESET: per core, its reset, which resets the core's
 * CLIC as it rises and again as it falls; held high, the CLIC presents
 * nothing and ignores writes.
 *
 * Clock inputs "core0" and "core1": each CLIC's bus clock,
 * HP_SYS_CLKRST's CORE<n>_CLIC_CLK. A CLIC whose clock is stopped
 * presents no interrupt, latches no edge and drops writes.
 *
 * Link properties "cpu0" and "cpu1": the HP cores.
 */
#define ESP32P4_CLIC_IRQ        "irq"
#define ESP32P4_CLIC_IRQ_LINE(core, id) ((core) * ESP32P4_CLIC_NUM_IRQS + (id))
#define ESP32P4_CLIC_CORE_RESET "core-reset"

typedef struct ESP32P4ClicCore {
    ESP32P4ClicState *s;
    int index;
    ESP32P4CPU *cpu;
    Clock *clk;

    /*
     * Per interrupt: pending (bit 0), enable (bit 0), attributes (SHV,
     * TRIG, and MODE's upper bit as stored, the only one implemented) and
     * control (the upper three bits, the rest reading one).
     */
    uint8_t ip[ESP32P4_CLIC_NUM_IRQS];
    uint8_t ie[ESP32P4_CLIC_NUM_IRQS];
    uint8_t attr[ESP32P4_CLIC_NUM_IRQS];
    uint8_t ctl[ESP32P4_CLIC_NUM_IRQS];
    /* The level of each interrupt line. */
    bool line[ESP32P4_CLIC_NUM_IRQS];
    /* mcliccfg's MNLBITS, NMBITS and UNLBITS. */
    uint8_t mnlbits;
    uint8_t nmbits;
    uint8_t unlbits;
    bool in_reset;
} ESP32P4ClicCore;

struct ESP32P4ClicState {
    SysBusDevice parent_obj;

    MemoryRegion m_iomem;
    MemoryRegion u_iomem;
    ESP32P4ClicCore core[ESP32P4_CLIC_CORES];
};

#endif
