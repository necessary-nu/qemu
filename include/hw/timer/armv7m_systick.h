/*
 * ARMv7M SysTick timer
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 * Copyright (c) 2017 Linaro Ltd
 * Written by Peter Maydell
 *
 * This code is licensed under the GPL (version 2 or later).
 */

#ifndef HW_TIMER_ARMV7M_SYSTICK_H
#define HW_TIMER_ARMV7M_SYSTICK_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/core/ptimer.h"
#include "hw/core/clock.h"

#define TYPE_SYSTICK "armv7m_systick"

OBJECT_DECLARE_SIMPLE_TYPE(SysTickState, SYSTICK)

/*
 * QEMU interface:
 *  + sysbus MMIO region 0 is the register interface (covering
 *    the registers which are mapped at address 0xE000E010)
 *  + sysbus IRQ 0 is the interrupt line to the NVIC
 *  + Clock input "refclk" is the external reference clock
 *    (used when SYST_CSR.CLKSOURCE == 0)
 *  + Clock input "cpuclk" is the main CPU clock
 *    (used when SYST_CSR.CLKSOURCE == 1)
 *  + Property "calib" is the value SYST_CALIB reads, which hardware takes
 *    from the core's STCALIB tie-off. Its default,
 *    SYSTICK_CALIB_FROM_REFCLK, derives SYST_CALIB from the refclk
 *    frequency instead. A board that sets it also sets NOREF through it.
 */

/* Never a valid SYST_CALIB value: bits [29:24] are reserved. */
#define SYSTICK_CALIB_FROM_REFCLK UINT32_MAX

struct SysTickState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    uint32_t control;
    uint32_t reload;
    int64_t tick;
    ptimer_state *ptimer;
    MemoryRegion iomem;
    qemu_irq irq;
    Clock *refclk;
    Clock *cpuclk;
    uint32_t calib;
};

#endif
