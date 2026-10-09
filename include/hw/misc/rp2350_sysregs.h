/*
 * RP2350 system register blocks (SYSINFO, SYSCFG, TBMAN, glitch detector,
 * DFT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_SYSREGS_H
#define HW_MISC_RP2350_SYSREGS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_RP2350_SYSINFO "rp2350-sysinfo"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350SysInfoState, RP2350_SYSINFO)

#define TYPE_RP2350_SYSCFG "rp2350-syscfg"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350SysCfgState, RP2350_SYSCFG)

#define TYPE_RP2350_TBMAN "rp2350-tbman"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350TBManState, RP2350_TBMAN)

#define TYPE_RP2350_GLITCH_DETECTOR "rp2350-glitch-detector"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350GlitchDetectorState,
                           RP2350_GLITCH_DETECTOR)

#define TYPE_RP2350_DFT "rp2350-dft"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350DFTState, RP2350_DFT)

/* Every block is a 4 KiB register window plus its three atomic aliases. */
struct RP2350SysInfoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* PACKAGE_SEL: 1 for the QFN-60 RP2350A, 0 for the QFN-80 RP2350B. */
    bool qfn60;
    uint32_t gitref;
};

struct RP2350SysCfgState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t proc_in_sync_bypass;
    uint32_t proc_in_sync_bypass_hi;
    uint32_t dbgforce;
    uint32_t mempowerdown;
    uint32_t auxctrl;
};

struct RP2350TBManState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
};

struct RP2350GlitchDetectorState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /*
     * Pulsed when an armed detector trigger resets the chip, for the
     * power manager's CHIP_RESET.HAD_GLITCH_DETECT latch.
     */
    qemu_irq chip_reset;

    /* OTP CRIT1.GLITCH_DETECTOR_ENABLE, as sampled at OTP reset. */
    bool otp_enable;

    uint32_t arm;
    uint32_t disarm;
    uint32_t sensitivity;
    uint32_t lock;
    uint32_t trig_status;
    /* The pending system reset was requested by a detector trigger. */
    bool reset_pending;
};

struct RP2350DFTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
};

#endif
