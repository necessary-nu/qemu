/*
 * RP2350 DMA controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Sixteen channels move data over the DMA's own bus master port. The
 * board gives the device that port as the "bus" link (the system bus as
 * ACCESSCTRL filters it for the DMA master); every channel access carries
 * the channel's security level and is checked by the DMA MPU before it
 * reaches the bus.
 *
 * Inputs and outputs:
 *
 *  - Sysbus IRQs 0-3: DMA_IRQ_0 to DMA_IRQ_3.
 *  - Named GPIO input array "dreq", indexed by DREQ number (0-54, the
 *    pico-sdk dreq.h numbering): each peripheral's data request. A rising
 *    edge is one transfer credit for the channels pacing on that DREQ (a
 *    one-cycle pulse on hardware); while the line is held high the
 *    peripheral has room or data for one more transfer after every
 *    transfer. Pacing timers (TREQ 59-62) and the permanent request (63)
 *    are internal.
 */

#ifndef HW_DMA_RP2350_DMA_H
#define HW_DMA_RP2350_DMA_H

#include "hw/core/sysbus.h"
#include "hw/misc/rp2350_accessctrl.h"
#include "hw/misc/rp2350_busctrl.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_DMA "rp2350-dma"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350DMAState, RP2350_DMA)

#define RP2350_DMA_BASE 0x50000000

#define RP2350_DMA_CHANNELS 16
#define RP2350_DMA_IRQS 4
#define RP2350_DMA_TIMERS 4
#define RP2350_DMA_MPU_REGIONS 8

#define RP2350_DMA_DREQ "dreq"
#define RP2350_DMA_NUM_DREQ 55

/* DREQ numbers of the sources the SoC wires (pico-sdk dreq.h). */
#define RP2350_DREQ_SPI0_TX 24
#define RP2350_DREQ_SPI0_RX 25
#define RP2350_DREQ_SPI1_TX 26
#define RP2350_DREQ_SPI1_RX 27
#define RP2350_DREQ_UART0_TX 28
#define RP2350_DREQ_UART0_RX 29
#define RP2350_DREQ_UART1_TX 30
#define RP2350_DREQ_UART1_RX 31
#define RP2350_DREQ_PWM_WRAP0 32
#define RP2350_DREQ_ADC 48
#define RP2350_DREQ_XIP_STREAM 49
#define RP2350_DREQ_HSTX 52
#define RP2350_DREQ_CORESIGHT 53
#define RP2350_DREQ_SHA256 54

typedef struct RP2350DMAChannel {
    uint32_t read_addr;
    uint32_t write_addr;
    /* The live transfer counter, MODE in bits 31:28. */
    uint32_t trans_count;
    /* The value the counter reloads from on a trigger (DBG_TCR). */
    uint32_t reload;
    /* CTRL, including BUSY and the error flags. */
    uint32_t ctrl;
    uint32_t seccfg;
    /* The DREQ credit counter (DBG_CTDREQ), six bits, saturating. */
    uint32_t dreq_credit;
} RP2350DMAChannel;

struct RP2350DMAState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /*
     * The DMA's bus master port: the "bus" link with the core-local
     * blocks (SIO, PPB), which the DMA cannot reach, failing decode.
     */
    MemoryRegion port;
    MemoryRegion port_bus;
    MemoryRegion port_hole;
    AddressSpace as;

    MemoryRegion *bus;
    RP2350AccessCtrlState *accessctrl;
    RP2350BusCtrlState *busctrl;
    uint32_t sysclk_hz;

    QEMUTimer *timer;
    qemu_irq irq[RP2350_DMA_IRQS];

    RP2350DMAChannel ch[RP2350_DMA_CHANNELS];
    uint32_t intr;
    uint32_t inte[RP2350_DMA_IRQS];
    uint32_t intf[RP2350_DMA_IRQS];
    uint32_t pacing[RP2350_DMA_TIMERS];
    /* Each pacing timer's fractional accumulator. */
    uint32_t pacing_acc[RP2350_DMA_TIMERS];
    uint32_t sniff_ctrl;
    uint32_t sniff_data;
    uint32_t seccfg_irq[RP2350_DMA_IRQS];
    uint32_t seccfg_misc;
    uint32_t mpu_ctrl;
    uint32_t mpu_bar[RP2350_DMA_MPU_REGIONS];
    uint32_t mpu_lar[RP2350_DMA_MPU_REGIONS];

    /* Levels of the DREQ inputs, bit n for DREQ n. */
    uint64_t dreq_level;

    /* Channels triggered with a zero transfer count, completing next. */
    uint32_t zero_pending;
    /* The channel that last completed a write, or -1. */
    int32_t last_write;
    /* Round-robin state: high-priority channels served this round. */
    uint32_t hp_served;
    uint32_t rr_high;
    uint32_t rr_low;

    /* clk_sys cycles the transfer engine has run up to. */
    uint64_t cycle;
    bool running;
};

#endif
