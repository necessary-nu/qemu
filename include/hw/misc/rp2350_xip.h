/*
 * RP2350 XIP subsystem: XIP_CTRL, the QSPI memory interface and XIP_AUX
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_XIP_H
#define HW_MISC_RP2350_XIP_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/fifo32.h"
#include "qemu/timer.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_RP2350_XIP "rp2350-xip"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350XIPState, RP2350_XIP)

/* QMI chip selects, each decoding a 16 MiB window. */
#define RP2350_QMI_CS 2
#define RP2350_QMI_WINDOW_SIZE (16 * MiB)
/* Address translation panes: four 4 MiB panes per chip select. */
#define RP2350_QMI_PANES 4
#define RP2350_QMI_ATRANS (RP2350_QMI_CS * RP2350_QMI_PANES)

/* The 16 KiB cache: two ways of 1024 sets of 8-byte lines. */
#define RP2350_XIP_CACHE_SIZE (16 * KiB)
#define RP2350_XIP_CACHE_LINES (RP2350_XIP_CACHE_SIZE / 8)

/* The four 64 MiB mirrors of the 26-bit XIP space, from 0x10000000. */
#define RP2350_XIP_SPACE_SIZE (256 * MiB)

/* Copies of the smallest (1 MiB) device that fill a 32 MiB mirror. */
#define RP2350_XIP_MIRROR_ALIASES 32
#define RP2350_XIP_VIEWS (2 * RP2350_QMI_PANES + 1)
/*
 * Pinned-line runs mapped directly. Beyond this many, the cached window
 * is left entirely to the access model.
 */
#define RP2350_XIP_PIN_VIEWS 64

/* DREQ outputs, in the order of their DREQ numbers (49, 50, 51). */
#define RP2350_XIP_DREQ_STREAM 0
#define RP2350_XIP_DREQ_QMI_TX 1
#define RP2350_XIP_DREQ_QMI_RX 2
#define RP2350_XIP_NUM_DREQ 3

/*
 * MMIO regions: 0 XIP_CTRL, 1 QMI (each with the atomic aliases),
 * 2 XIP_AUX, 3 the XIP address space from 0x10000000.
 */
#define RP2350_XIP_MMIO_CTRL 0
#define RP2350_XIP_MMIO_QMI 1
#define RP2350_XIP_MMIO_AUX 2
#define RP2350_XIP_MMIO_SPACE 3

typedef struct RP2350XIPArray {
    RP2350XIPState *xip;
    int cs;
    /* The device's storage, read directly by memory-mapped accesses. */
    MemoryRegion mr;
} RP2350XIPArray;

struct RP2350XIPState {
    SysBusDevice parent_obj;

    MemoryRegion ctrl_iomem;
    MemoryRegion qmi_iomem;
    MemoryRegion aux_iomem;
    /* The XIP address space: the access model, under its fast views. */
    MemoryRegion space;
    MemoryRegion space_io;

    RP2350XIPArray array[RP2350_QMI_CS];
    /* Each array repeated over 32 MiB, as the device ignores high bits. */
    MemoryRegion mirror[RP2350_QMI_CS];
    MemoryRegion mirror_alias[RP2350_QMI_CS][RP2350_XIP_MIRROR_ALIASES];
    /*
     * Views of each chip select's mirror: the cached and the uncached
     * panes, then the untranslated window.
     */
    MemoryRegion view[RP2350_QMI_CS][RP2350_XIP_VIEWS];
    /* Runs of pinned cache lines: as RAM, or as the access model. */
    MemoryRegion pin_ram[RP2350_XIP_PIN_VIEWS];
    MemoryRegion pin_io[RP2350_XIP_PIN_VIEWS];
    MemoryRegion cache_data;
    /* Writes to the cache data RAM, keeping translated code coherent. */
    AddressSpace cache_as;

    SSIBus *qspi;
    qemu_irq cs[RP2350_QMI_CS];
    qemu_irq dreq[RP2350_XIP_NUM_DREQ];
    QEMUTimer *busy_timer;
    QEMUTimer *stream_timer;

    /* Properties: the size of the device on each chip select (0: none). */
    uint32_t cs_size[RP2350_QMI_CS];
    uint32_t sysclk_hz;

    /* XIP_CTRL */
    uint32_t ctrl;
    uint32_t ctr_hit;
    uint32_t ctr_acc;
    uint32_t stream_addr;
    uint32_t stream_ctr;
    Fifo32 stream_fifo;

    /* QMI */
    uint32_t direct_csr;
    Fifo32 direct_tx;
    Fifo32 direct_rx;
    /* QEMU_CLOCK_VIRTUAL time at which the last direct frame ends. */
    int64_t direct_busy_until;
    uint32_t timing[RP2350_QMI_CS];
    uint32_t rfmt[RP2350_QMI_CS];
    uint32_t rcmd[RP2350_QMI_CS];
    uint32_t wfmt[RP2350_QMI_CS];
    uint32_t wcmd[RP2350_QMI_CS];
    uint32_t atrans[RP2350_QMI_ATRANS];
    /* Chip select asserted for a memory-mapped transfer, or -1. */
    int32_t mm_cs;
    /* Chip select levels last driven (1: deasserted), -1 if not driven. */
    int32_t cs_level[RP2350_QMI_CS];

    /*
     * Cache tag memory, per line (way * 1024 + set): the line's address
     * bits 25:3 and its state. Only pinned lines are tracked; every other
     * line behaves as invalid.
     */
    uint32_t tag[RP2350_XIP_CACHE_LINES];
};

/* The ROM device region backing the device on chip select `cs`. */
MemoryRegion *rp2350_xip_array(RP2350XIPState *s, int cs);

#endif
