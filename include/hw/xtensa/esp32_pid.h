/*
 * ESP32 process ID controller (PIDCTRL)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_XTENSA_ESP32_PID_H
#define HW_XTENSA_ESP32_PID_H

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "target/xtensa/cpu.h"

#define TYPE_ESP32_PID "misc.esp32.pidctrl"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32PidState, ESP32_PID)

/* Each CPU reaches its own controller here (TRM 3.3.5.1). */
#define ESP32_PID_BASE          0x3ff1f000
#define ESP32_PID_SIZE          0x1000

/* Interrupt levels the controller recognises: 1 to 6, and 7 for the NMI */
#define ESP32_PID_LEVELS        7

/* The countdowns the controller runs on its CPU */
#define ESP32_PID_COUNTDOWN_PID 0
#define ESP32_PID_COUNTDOWN_NMI 1

REG32(PIDCTRL_INTERRUPT_ENABLE, 0x00)
REG32(PIDCTRL_INTERRUPT_ADDR_1, 0x04)
REG32(PIDCTRL_INTERRUPT_ADDR_7, 0x1c)
REG32(PIDCTRL_PID_DELAY, 0x20)
REG32(PIDCTRL_NMI_DELAY, 0x24)
REG32(PIDCTRL_LEVEL, 0x28)
REG32(PIDCTRL_FROM_1, 0x2c)
REG32(PIDCTRL_FROM_7, 0x44)
REG32(PIDCTRL_PID_NEW, 0x48)
REG32(PIDCTRL_PID_CONFIRM, 0x4c)
REG32(PIDCTRL_NMI_MASK_ENABLE, 0x54)
REG32(PIDCTRL_NMI_MASK_DISABLE, 0x58)

struct Esp32PidState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    XtensaCPU *cpu;

    uint32_t int_enable;
    uint32_t int_addr[ESP32_PID_LEVELS];
    uint32_t pid_delay;
    uint32_t nmi_delay;
    uint32_t level;
    uint32_t from[ESP32_PID_LEVELS];
    uint32_t pid_new;

    /* The process ID the MMUs and MPUs see for this CPU */
    uint32_t pid;
    bool nmi_masked;
};

unsigned esp32_pid_current(Esp32PidState *s);
void esp32_pid_vector(Esp32PidState *s, uint32_t pc);
void esp32_pid_countdown_expired(Esp32PidState *s, unsigned n);

#endif
