/*
 * RP2350 Global Exclusive Monitor
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_EXCLMON_H
#define HW_MISC_RP2350_EXCLMON_H

#include "hw/core/sysbus.h"
#include "qemu/thread.h"
#include "qom/object.h"

#define TYPE_RP2350_EXCLMON "rp2350-exclmon"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350ExclMonState, RP2350_EXCLMON)

#define RP2350_EXCLMON_CORES   2
#define RP2350_EXCLMON_GRANULE 16

typedef struct RP2350ExclReservation {
    /* Exclusive (true) or Open (false) */
    bool valid;
    /* log2 of the size of the exclusive read that took it */
    uint8_t size;
    /* Security and privilege state of that read */
    uint8_t tag;
    /* Base address of the reserved granule */
    uint32_t granule;
    /* The granule's contents when it was reserved */
    uint8_t data[RP2350_EXCLMON_GRANULE];
} RP2350ExclReservation;

struct RP2350ExclMonState {
    SysBusDevice parent_obj;

    /* The 520 KiB SRAM, and its contents in host memory */
    MemoryRegion *sram;
    uint8_t *ram;

    /* Serialises reservation changes between the cores and the DMA. */
    QemuMutex lock;
    CPUState *core[RP2350_EXCLMON_CORES];
    /*
     * One reservation per core. The DMA is the third processing element
     * the monitor watches, but it makes no exclusive accesses and so
     * never holds a reservation.
     */
    RP2350ExclReservation res[RP2350_EXCLMON_CORES];
};

/* Connect `cpu` to the monitor as RP2350 core `core`. */
void rp2350_exclmon_attach(RP2350ExclMonState *s, int core, CPUState *cpu);

/* A DMA write of `len` bytes at `addr`, which has completed. */
void rp2350_exclmon_dma_write(RP2350ExclMonState *s, uint32_t addr,
                              unsigned len);

#endif
