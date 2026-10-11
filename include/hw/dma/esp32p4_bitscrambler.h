/*
 * ESP32-P4 BitScrambler
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32P4_BITSCRAMBLER_H
#define HW_DMA_ESP32P4_BITSCRAMBLER_H

#include "hw/core/sysbus.h"
#include "hw/core/clock.h"
#include "hw/dma/esp32p4_gdma.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_BITSCRAMBLER "esp32p4-bitscrambler"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4BitScramblerState, ESP32P4_BITSCRAMBLER)

#define ESP32P4_BITSCRAMBLER_BASE   0x500a3000
#define ESP32P4_BITSCRAMBLER_SIZE   0x1000

/* The TX (memory to peripheral) and RX (peripheral to memory) cores. */
#define ESP32P4_BS_TX               0
#define ESP32P4_BS_RX               1

#define ESP32P4_BS_INSTS            8
#define ESP32P4_BS_INST_WORDS       9
#define ESP32P4_BS_LUT_BYTES        2048
/* The cores' FIFOs, in bytes, each way. */
#define ESP32P4_BS_FIFO             16

/*
 * HP_SYSTEM_BITSCRAMBLER_PERI_SEL's values (TRM register 21.50): the
 * peripheral whose DMA path a core is attached to. 13 to 15 attach
 * nothing.
 */
#define ESP32P4_BS_PERIPHS          13

/*
 * Named GPIO inputs:
 * ESP32P4_BS_PERI_SEL, "peri-sel" (8 lines): HP_SYSTEM_BITSCRAMBLER_
 *   PERI_SEL's bits, RX_SEL in lines 0 to 3, TX_SEL in 4 to 7, from
 *   HP_SYSTEM. They read 0xff (nothing attached) until driven.
 * ESP32P4_BS_CORE_RESET, "core-reset" (2 lines): RST_EN_BITSCRAMBLER_TX
 *   and _RX, each high while that core is held in reset.
 *
 * Clock inputs "tx-clk" and "rx-clk", BITSCRAMBLER_TX_SYS_CLK and
 * BITSCRAMBLER_RX_SYS_CLK, at which the cores execute an instruction a
 * cycle. Links "ahb-gdma" and "axi-gdma" to the controllers whose
 * peripherals' paths the cores attach to.
 */
#define ESP32P4_BS_PERI_SEL         "peri-sel"
#define ESP32P4_BS_CORE_RESET       "core-reset"

/* A core's FIFO: bytes, or marks, each with what it carries. */
typedef struct ESP32P4BsFifo {
    uint8_t data[ESP32P4_BS_FIFO];
    uint8_t flags[ESP32P4_BS_FIFO];
    uint32_t head;
    uint32_t num;
} ESP32P4BsFifo;

typedef struct ESP32P4BsCore {
    struct ESP32P4BitScramblerState *s;
    int dir;

    /* Registers. */
    uint32_t inst_cfg0;
    uint32_t lut_cfg0;
    uint32_t tailing_bits;
    uint32_t ctrl;
    bool eof_overload;
    uint32_t eof_get_cnt;
    uint32_t inst[ESP32P4_BS_INSTS][ESP32P4_BS_INST_WORDS];
    uint8_t lut[ESP32P4_BS_LUT_BYTES];

    /* The execution state. */
    uint8_t state;
    bool reset_held;
    uint8_t prefetch;       /* bytes of the start's prefetch still to read */
    uint8_t ip;
    bool fetched;           /* FETCH_MODE: the read for ip has been done */
    uint64_t in;            /* the input register */
    uint32_t last;          /* the output register's previous value */
    uint16_t ctr[2];        /* counters A and B */
    bool eof_seen;          /* the input frame's end has been read */
    bool eof_err;           /* ... and it ended in error */
    uint32_t eof_count;     /* bytes counted since, toward TAILING_BITS */
    ESP32P4BsFifo fifo_in;
    ESP32P4BsFifo fifo_out;

    /* The path the core is attached to: a controller and its peripheral. */
    ESP32P4GdmaState *gdma;
    unsigned periph;
    bool loop_rx;           /* also interposed on the RX path, loopback */

    Clock *clk;
    QEMUTimer *timer;
    /* When the cycle @timer runs falls due. */
    int64_t due_ns;
} ESP32P4BsCore;

struct ESP32P4BitScramblerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    ESP32P4BsCore core[2];
    uint32_t sys;
    uint32_t version;
    uint32_t peri_sel;

    Object *gdma_obj[2];    /* GDMA-AHB, GDMA-AXI */
};

#endif
