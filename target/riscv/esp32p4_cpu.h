/*
 * Espressif ESP32-P4 HP and LP RISC-V cores
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TARGET_RISCV_ESP32P4_CPU_H
#define TARGET_RISCV_ESP32P4_CPU_H

#include "target/riscv/cpu.h"
#include "qom/object.h"

#define TYPE_ESP32P4_CPU "esp32p4-cpu"
#define TYPE_ESP32P4_HP_CPU RISCV_CPU_TYPE_NAME("esp32p4-hp")
#define TYPE_ESP32P4_LP_CPU RISCV_CPU_TYPE_NAME("esp32p4-lp")

OBJECT_DECLARE_TYPE(ESP32P4CPU, ESP32P4CPUClass, ESP32P4_CPU)

/*
 * Named GPIO inputs: the core's reset, held while high, and its clock
 * enable, the core's clock running while high. A core starts with its
 * reset released and its clock running.
 */
#define ESP32P4_CPU_RESET "reset"
#define ESP32P4_CPU_CLOCK "clock"

struct ESP32P4CPU {
    RISCVCPU parent_obj;

    /* The reset input is high: the core is held in its reset state. */
    bool reset;
    /* The clock enable is low: the core does not execute. */
    bool clock_stopped;
    /*
     * The core was executing when its clock stopped, so it carries on
     * when the clock restarts; a core waiting in WFI goes on waiting.
     */
    bool clock_halted;
};

struct ESP32P4CPUClass {
    RISCVCPUClass parent_class;

    ResettablePhases parent_phases;
};

#endif
