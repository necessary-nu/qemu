/*
 * RP2350 one-time-programmable storage (OTP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The fuse array is 4096 rows of 24 bits. With a "drive" property the
 * rows persist in that block device, as 4096 little-endian 32-bit words
 * (the 24-bit row in the low bits, as the raw read alias returns it):
 *
 *   -drive if=none,id=otp,format=raw,file=otp.img
 *   -global rp2350-otp.drive=otp
 *
 * An image that reads as all zeroes is a chip fresh from manufacturing
 * test: the model writes the factory-programmed rows into it. Without a
 * drive the rows live in RAM and start in that factory state.
 *
 * Outputs: sysbus IRQ 0 is OTP_IRQ; the named GPIO outputs
 * "debug-disable" and "secure-debug-disable" are the debug disable
 * signals the CoreSight blocks take from the OTP critical flags, debug
 * keys and DEBUGEN, and "glitch-detector-enable" is the critical flag
 * that arms the glitch detectors.
 */

#ifndef HW_MISC_RP2350_OTP_H
#define HW_MISC_RP2350_OTP_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_OTP "rp2350-otp"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350OTPState, RP2350_OTP)

#define RP2350_OTP_BASE      0x40120000
#define RP2350_OTP_DATA_BASE 0x40130000
#define RP2350_OTP_DATA_SIZE 0x10000

#define RP2350_OTP_ROWS      4096
#define RP2350_OTP_PAGES     64
#define RP2350_OTP_KEYS      6
#define RP2350_OTP_SBPI_REGS 64

struct RP2350OTPState {
    SysBusDevice parent_obj;

    MemoryRegion ctrl_iomem;
    MemoryRegion data_iomem;
    qemu_irq irq;
    qemu_irq debug_disable;
    qemu_irq secure_debug_disable;
    qemu_irq glitch_detector_enable;

    BlockBackend *blk;
    bool blk_writable;

    /* The fuse array: 24 bits per row. */
    uint32_t fuse[RP2350_OTP_ROWS];

    /* Latched by the power-up state machine at OTP reset. */
    uint32_t critical;
    uint32_t key_valid;
    uint32_t key[RP2350_OTP_KEYS][4];
    /* Each lock word's key configuration (its voted LOCK0 byte). */
    uint32_t key_cfg[RP2350_OTP_PAGES];
    bool rma;

    uint32_t sw_lock[RP2350_OTP_PAGES];
    uint32_t sbpi_instr;
    uint32_t sbpi_wdata[4];
    uint32_t sbpi_rdata[4];
    uint32_t sbpi_status;
    uint32_t usr;
    uint32_t dbg;
    uint32_t bist;
    uint32_t crt_key[4];
    uint32_t debugen;
    uint32_t debugen_lock;
    uint32_t archsel;
    uint32_t archsel_status;
    uint32_t bootdis;
    uint32_t intr;
    uint32_t inte;
    uint32_t intf;

    /* Registers of the SBPI targets: the DAP and the PMC. */
    uint8_t dap[RP2350_OTP_SBPI_REGS];
    uint8_t pmc[RP2350_OTP_SBPI_REGS];
};

/* Reset the OTP block as its PSM stage does. */
void rp2350_otp_reset_stage(RP2350OTPState *s);

#endif
