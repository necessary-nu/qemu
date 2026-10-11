/*
 * ESP32-P4 LP Mailbox
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_LP_MAILBOX_H
#define HW_MISC_ESP32P4_LP_MAILBOX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_LP_MAILBOX "esp32p4-lp-mailbox"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4LPMailboxState, ESP32P4_LP_MAILBOX)

#define ESP32P4_LP_MAILBOX_BASE     0x50118000
#define ESP32P4_LP_MAILBOX_SIZE     0x1000
#define ESP32P4_LP_MAILBOX_MESSAGES 16

/*
 * The interrupts, sysbus IRQs in this order: MB_HP_INTR (interrupt matrix
 * source 4), for the HP cores, and MB_LP_INTR (source 5), for the LP core,
 * which takes it, as every source, from the matrix's fan-out of the
 * sources.
 */
enum {
    ESP32P4_LP_MAILBOX_IRQ_HP,
    ESP32P4_LP_MAILBOX_IRQ_LP,
    ESP32P4_LP_MAILBOX_IRQS,
};

struct ESP32P4LPMailboxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[ESP32P4_LP_MAILBOX_IRQS];

    uint32_t message[ESP32P4_LP_MAILBOX_MESSAGES];
    uint32_t int_raw[ESP32P4_LP_MAILBOX_IRQS];
    uint32_t int_ena[ESP32P4_LP_MAILBOX_IRQS];
    uint32_t clk_en;
};

#endif
