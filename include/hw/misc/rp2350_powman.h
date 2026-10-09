/*
 * RP2350 power manager and always-on timer (POWMAN)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_RP2350_POWMAN_H
#define HW_MISC_RP2350_POWMAN_H

#include "hw/core/sysbus.h"
#include "hw/gpio/rp2350_gpio.h"
#include "hw/misc/rp2350_psm.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_RP2350_POWMAN "rp2350-powman"
OBJECT_DECLARE_SIMPLE_TYPE(RP2350PowmanState, RP2350_POWMAN)

#define RP2350_POWMAN_SCRATCH 8
#define RP2350_POWMAN_BOOT 4
#define RP2350_POWMAN_PWRUPS 4
#define RP2350_POWMAN_TIME_REGS 4

/*
 * The switched power domains, numbered as their bits in STATE.CURRENT and
 * STATE.REQ, where a set bit is a powered-down domain.
 */
enum {
    RP2350_POWMAN_SRAM1,
    RP2350_POWMAN_SRAM0,
    RP2350_POWMAN_XIP,
    RP2350_POWMAN_SWCORE,
    RP2350_POWMAN_DOMAINS,
};

/* Sysbus IRQs: POWMAN_IRQ_POW and POWMAN_IRQ_TIMER. */
enum {
    RP2350_POWMAN_IRQ_POW,
    RP2350_POWMAN_IRQ_TIMER,
};

/*
 * Named GPIO inputs:
 *   "watchdog"      the watchdog's reset request, which the power manager
 *                   sees before the PSM so that WDSEL can widen it
 *   "glitch-reset"  a glitch detector trigger resetting the chip
 *   "gpio"          one line per pin (RP2350_GPIO_PINS), the pin's input
 *                   level, for the power-up and time reference sources
 *   "xosc-dormant"  high while the XOSC is stopped by DORMANT, which stops
 *                   an AON timer running from it
 * Named GPIO outputs:
 *   "psm-watchdog"  the watchdog reset passed on to the PSM
 *   "psm-reset"     pulsed to run the PSM's full sequence (a chip-level
 *                   reset that leaves the power manager and switched core
 *                   alone)
 *   "watchdog-reset" pulsed to reset the watchdog, scratch registers
 *                   included, in a chip-level reset carried out without a
 *                   system reset (a glitch detector trigger)
 *   "alarm-wake"    pulsed by each AON timer alarm event (the rising edge
 *                   of the alarm comparison), a DORMANT wake event
 */
#define RP2350_POWMAN_WATCHDOG "watchdog"
#define RP2350_POWMAN_GLITCH_RESET "glitch-reset"
#define RP2350_POWMAN_GPIO "gpio"
#define RP2350_POWMAN_PSM_WATCHDOG "psm-watchdog"
#define RP2350_POWMAN_PSM_RESET "psm-reset"
#define RP2350_POWMAN_WATCHDOG_RESET "watchdog-reset"
#define RP2350_POWMAN_XOSC_DORMANT "xosc-dormant"
#define RP2350_POWMAN_ALARM_WAKE "alarm-wake"

/*
 * What the power manager asks of the rest of the chip. The switched core
 * domain, which holds the processors, and the memory domains are the
 * SoC's; the power manager only sequences them.
 */
typedef struct RP2350PowmanOps {
    /* Whether both processors are asleep (in WFI or WFE) or powered off. */
    bool (*procs_asleep)(void *opaque);
    /* The switched core starts to power down: stop both processors. */
    void (*swcore_stop)(void *opaque);
    /*
     * A memory domain (SRAM0, SRAM1 or XIP) finished powering down, losing
     * its contents, or powering up.
     */
    void (*domain_power)(void *opaque, int domain, bool on);
} RP2350PowmanOps;

struct RP2350PowmanState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[2];
    qemu_irq psm_watchdog;
    qemu_irq psm_reset;
    qemu_irq watchdog_reset;
    qemu_irq alarm_wake;
    /* Sequencer steps, and polling the processors while WAITING. */
    QEMUTimer *seq_timer;
    QEMUTimer *alarm_timer;
    /* A tick source switch, which waits for the current source's tick. */
    QEMUTimer *switch_timer;
    QEMUTimer *vreg_timer;

    /* Properties */
    RP2350PSMState *psm;
    uint32_t lposc_hz;
    uint32_t ref_hz;
    uint32_t bonded_gpios;

    const RP2350PowmanOps *ops;
    void *ops_opaque;

    /* The chip-level reset the next system reset carries out. */
    uint32_t next_reset;

    uint32_t badpasswd;
    uint32_t vreg_ctrl;
    uint32_t vreg;
    uint32_t vreg_lp_entry;
    uint32_t vreg_lp_exit;
    /* VREG_STS.STARTUP, and VOUT_OK held low while the output ramps. */
    bool vreg_startup;
    bool vout_low;
    uint32_t bod_ctrl;
    uint32_t bod;
    uint32_t bod_lp_entry;
    uint32_t bod_lp_exit;
    uint32_t lposc;
    uint32_t chip_reset;
    uint32_t wdsel;
    uint32_t seq_cfg;

    /* STATE: CURRENT, REQ and the flags, in the register layout. */
    uint32_t state;
    /* The sequencer's activity, and the state a change is heading for. */
    uint32_t seq;
    uint32_t seq_target;
    /* The power-up requests that started a switched-core power up. */
    uint32_t wake_reqs;

    uint32_t pow_fastdiv;
    uint32_t pow_delay;
    uint32_t ext_ctrl[2];
    uint32_t ext_time_ref;
    uint32_t lposc_freq_int;
    uint32_t lposc_freq_frac;
    uint32_t xosc_freq_int;
    uint32_t xosc_freq_frac;
    /* SET_TIME and ALARM_TIME, most significant halfword first. */
    uint16_t set_time[RP2350_POWMAN_TIME_REGS];
    uint16_t alarm_time[RP2350_POWMAN_TIME_REGS];

    /*
     * The AON timer: TIMER's stored bits (including the pending USE_*
     * switches and the USING_* source), and the count as `aon_base` at
     * virtual time `aon_start_ns`, counting since if a clock drives it.
     */
    uint32_t timer;
    uint64_t aon_base;
    int64_t aon_start_ns;
    /* With the 1 Hz sync, the count waits at this second boundary. */
    uint64_t aon_sec_limit;
    /* An external clock in place of the LPOSC: cycles, scaled by 2^16. */
    uint32_t aon_lpck_acc;
    /* The alarm comparison, whose rising edge sets TIMER.ALARM. */
    bool alarm_cmp;
    /* The XOSC is stopped by DORMANT. */
    bool xosc_dormant;

    uint32_t pwrup[RP2350_POWMAN_PWRUPS];
    /* Latched edges of the edge-sensitive power-up sources. */
    uint32_t pwrup_latch;
    uint32_t last_swcore_pwrup;
    uint32_t dbg_pwrcfg;
    uint32_t bootdis;
    uint32_t dbgconfig;
    uint32_t scratch[RP2350_POWMAN_SCRATCH];
    uint32_t boot[RP2350_POWMAN_BOOT];
    uint32_t intr;
    uint32_t inte;
    uint32_t intf;

    /* Pin input levels. */
    uint8_t pin[RP2350_GPIO_PINS];
};

void rp2350_powman_set_ops(RP2350PowmanState *s, const RP2350PowmanOps *ops,
                           void *opaque);

/* Whether `domain` is powered (STATE.CURRENT). */
bool rp2350_powman_domain_on(RP2350PowmanState *s, int domain);

/*
 * The QEMU reset type of the next system reset. The power manager's own
 * chip-level resets (the switched core's, a watchdog's through WDSEL)
 * leave the always-on domain, such as the pad isolation latches, and the
 * memory power domains, such as Boot RAM, powered: they are
 * RESET_TYPE_WAKEUP, which devices holding such state survive. Power-on,
 * brownout and RUN pin resets are RESET_TYPE_COLD. A glitch detector
 * trigger is no system reset at all: the power manager resets the PSM and
 * the watchdog.
 */
ResetType rp2350_powman_next_reset_type(RP2350PowmanState *s);

/* POWMAN_BOOTDIS_NOW, which the boot ROM checks and clears. */
#define RP2350_POWMAN_BOOTDIS_NOW 1u

#endif
