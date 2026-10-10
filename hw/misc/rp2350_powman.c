/*
 * RP2350 power manager and always-on timer (POWMAN)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Power management", "Core voltage
 * regulator", "Power management (POWMAN) registers" and "Always-on timer".
 *
 * POWMAN lives in the always-on (AON) power domain. It sequences the
 * switched core (SWCORE: processors, bus fabric, peripherals), XIP (XIP
 * cache and Boot RAM), SRAM0 and SRAM1 power domains, owns the chip-level
 * resets and their reasons, and contains the always-on timer.
 *
 * Registers up to and including DBGCONFIG (0xac) are password protected:
 * a write must carry 0x5afe in its top 16 bits or it is ignored and sets
 * BADPASSWD. The password is checked on the bus write data itself, and an
 * atomic alias (XOR/SET/CLR) then applies its operation to the low 16 bits
 * of the register, so pico-sdk's hw_set_bits(reg, 0x5afe0000 | bits) and
 * hw_write_masked() work as on hardware. Write-1-to-clear fields act on
 * the written data whichever alias is used. Non-secure code may read and,
 * with TIMER.NONSEC_WRITE, write the AON timer registers (SET_TIME,
 * READ_TIME, ALARM_TIME, TIMER, except TIMER's Secure-only fields); every
 * other register reads as zero and ignores Non-secure writes.
 *
 * Power states. A software request in STATE.REQ is checked against the
 * datasheet's valid transitions; an invalid one sets BAD_SW_REQ and one
 * made while a power-up request is pending sets REQ_IGNORED. Powering down
 * the switched core waits (STATE.WAITING) until both processors sleep in
 * WFI or WFE, then (STATE.CHANGING) stops them and runs the power-down
 * sequence. Each domain's sequence is 8 steps long (the RP-AP's
 * DBG_POW_STATE bits 1-8): POW_DELAY.SWCORE_STEP and XIP_STEP in LPOSC
 * periods, SRAM_STEP in POWMAN ticks, which are POW_FASTDIV clk_ref
 * cycles while the switched core is powered and SEQ_CFG.USE_FAST_POWCK
 * is set, and LPOSC periods otherwise. A memory domain loses its contents
 * when it powers down. In a low-power state (P1.x) the switched core is in
 * reset; a power-up request (PWRUP0-3 or the AON timer alarm with
 * PWRUP_ON_ALARM) powers it up again to the state SEQ_CFG.HW_PWRUP_SRAM0/1
 * select, and the chip boots as after a reset, with CHIP_RESET reporting
 * HAD_SWCORE_PD and LAST_SWCORE_PWRUP the request. The resets themselves
 * are QEMU system resets that this device survives (see next_reset); the
 * SoC keeps the processors off while the switched core is unpowered.
 *
 * Chip-level resets. The first reset is power-on (HAD_POR). Any later
 * system reset not started by this device, such as a monitor system_reset,
 * stands for the RUN pin (HAD_RUN_LOW), which keeps DOUBLE_TAP. Watchdog
 * resets pass through here: with WDSEL set and PSM_WDSEL reaching the
 * CLOCKS stage or earlier, they become RESET_PSM (the full PSM sequence),
 * RESET_SWCORE or RESET_POWMAN(_ASYNC) chip resets, each with its own
 * CHIP_RESET reason and the datasheet's set of what survives. RESET_PSM
 * and a glitch detector trigger are chip-level resets of the PSM and the
 * watchdog only: each latches its reason and runs the full PSM sequence,
 * leaving the power manager, the power state, the AON timer and every
 * block outside the PSM's and RESETS' reach (SRAM contents, the glitch
 * detector's own registers) as they were.
 *
 * The AON timer counts ticks of its source: the LPOSC (an ideal oscillator
 * at the lposc-hz property, which matches the OTP LPOSC_CALIB row), the
 * XOSC through clk_ref (the "clk-ref" clock, at its current frequency,
 * whatever it runs from), falling edges of the selected EXT_TIME_REF GPIO as
 * a 1 kHz tick, or GPIO edges replacing the LPOSC (DRIVE_LPCK). The
 * fractional divider divides the source by LPOSC_FREQ_KHZ or
 * XOSC_FREQ_KHZ (16.16, at least 2.0) to make the tick. Switching to the
 * LPOSC or XOSC takes effect on the current source's next tick, as the
 * datasheet's synchronisation does; the sampling error of up to two
 * periods of the new clock is not modelled. With USE_GPIO_1HZ the count
 * waits at each second boundary for the 1 Hz GPIO's falling edge, and an
 * edge before the boundary advances it to the boundary. The alarm fires
 * when the count reaches ALARM_TIME while ALARM_ENAB is set (the rising
 * edge of that comparison), setting TIMER.ALARM; that event also wakes
 * the chip from DORMANT, which stops an AON timer running from the XOSC
 * by stopping clk_ref.
 *
 * The regulator and brown-out detector have no analogue behaviour to
 * model: VREG accepts writes once VREG_CTRL.UNLOCK is set, shows
 * UPDATE_IN_PROGRESS for VREG_UPDATE_NS, drops VOUT_OK (raising
 * INTR.VREG_OUTPUT_LOW) while it ramps to a selection more than 13% above
 * the old one, and entering high-impedance mode with the regulator
 * supplying the core is a brownout reset. The BOD never trips. LPOSC.TRIM
 * is stored but does not move the LPOSC frequency. The power-mode-aware
 * GPIO outputs (EXT_CTRL0/1) and the debugger's power-up request are not
 * modelled.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_powman.h"
#include "migration/vmstate.h"
#include "system/runstate.h"

#define POWMAN_PASSWORD     0x5afe

#define A_BADPASSWD         0x00
#define A_VREG_CTRL         0x04
#define A_VREG_STS          0x08
#define A_VREG              0x0c
#define A_VREG_LP_ENTRY     0x10
#define A_VREG_LP_EXIT      0x14
#define A_BOD_CTRL          0x18
#define A_BOD               0x1c
#define A_BOD_LP_ENTRY      0x20
#define A_BOD_LP_EXIT       0x24
#define A_LPOSC             0x28
#define A_CHIP_RESET        0x2c
#define A_WDSEL             0x30
#define A_SEQ_CFG           0x34
#define A_STATE             0x38
#define A_POW_FASTDIV       0x3c
#define A_POW_DELAY         0x40
#define A_EXT_CTRL0         0x44
#define A_EXT_CTRL1         0x48
#define A_EXT_TIME_REF      0x4c
#define A_LPOSC_FREQ_KHZ_INT  0x50
#define A_LPOSC_FREQ_KHZ_FRAC 0x54
#define A_XOSC_FREQ_KHZ_INT   0x58
#define A_XOSC_FREQ_KHZ_FRAC  0x5c
#define A_SET_TIME_63TO48   0x60
#define A_SET_TIME_15TO0    0x6c
#define A_READ_TIME_UPPER   0x70
#define A_READ_TIME_LOWER   0x74
#define A_ALARM_TIME_63TO48 0x78
#define A_ALARM_TIME_15TO0  0x84
#define A_TIMER             0x88
#define A_PWRUP0            0x8c
#define A_PWRUP3            0x98
#define A_CURRENT_PWRUP_REQ 0x9c
#define A_LAST_SWCORE_PWRUP 0xa0
#define A_DBG_PWRCFG        0xa4
#define A_BOOTDIS           0xa8
#define A_DBGCONFIG         0xac
#define A_SCRATCH0          0xb0
#define A_SCRATCH7          0xcc
#define A_BOOT0             0xd0
#define A_BOOT3             0xdc
#define A_INTR              0xe0
#define A_INTE              0xe4
#define A_INTF              0xe8
#define A_INTS              0xec

#define A_LAST_PROTECTED    A_DBGCONFIG

#define VREG_CTRL_RST_N     BIT(15)
#define VREG_CTRL_UNLOCK    BIT(13)
#define VREG_CTRL_DISABLE_VOLTAGE_LIMIT BIT(8)
#define VREG_CTRL_MASK      0xb170u
#define VREG_CTRL_RESET     0x8050u

#define VREG_STS_VOUT_OK    BIT(4)
#define VREG_STS_STARTUP    BIT(0)

#define VREG_UPDATE_IN_PROGRESS BIT(15)
#define VREG_VSEL_SHIFT     4
#define VREG_VSEL_MASK      (0x1fu << VREG_VSEL_SHIFT)
#define VREG_HIZ            BIT(1)
#define VREG_MASK           (VREG_VSEL_MASK | VREG_HIZ)
#define VREG_RESET          0xb0u
#define VREG_LP_MODE        BIT(2)
#define VREG_LP_MASK        0x1f6u
#define VREG_LP_ENTRY_RESET 0xb4u
#define VREG_LP_EXIT_RESET  0xb0u
/* 1.30 V, the highest selection allowed without DISABLE_VOLTAGE_LIMIT. */
#define VREG_VSEL_LIMIT     0x0f

#define BOD_CTRL_MASK       0x1000u
#define BOD_EN              BIT(0)
#define BOD_MASK            0x1f1u
#define BOD_RESET           0xb1u
#define BOD_LP_ENTRY_RESET  0xb0u
#define BOD_LP_EXIT_RESET   0xb1u

#define LPOSC_MASK          0x3f3u
#define LPOSC_RESET         0x203u

#define CHIP_RESET_HAD_POR                  BIT(16)
#define CHIP_RESET_HAD_BOR                  BIT(17)
#define CHIP_RESET_HAD_RUN_LOW              BIT(18)
#define CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_ASYNC BIT(22)
#define CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN BIT(23)
#define CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE BIT(24)
#define CHIP_RESET_HAD_SWCORE_PD            BIT(25)
#define CHIP_RESET_HAD_GLITCH_DETECT        BIT(26)
#define CHIP_RESET_HAD_WATCHDOG_RESET_PSM   BIT(28)
#define CHIP_RESET_HAD_MASK                 0x1fef0000u
#define CHIP_RESET_RESCUE_FLAG              BIT(4)
#define CHIP_RESET_DOUBLE_TAP               BIT(0)

#define WDSEL_RESET_PSM             BIT(12)
#define WDSEL_RESET_SWCORE          BIT(8)
#define WDSEL_RESET_POWMAN          BIT(4)
#define WDSEL_RESET_POWMAN_ASYNC    BIT(0)
#define WDSEL_MASK                  0x1111u

/* PSM stages up to and including CLOCKS, which a chip reset needs. */
#define PSM_THROUGH_CLOCKS  (BIT(RP2350_PSM_CLOCKS + 1) - 1)

#define SEQ_CFG_USING_FAST_POWCK BIT(20)
#define SEQ_CFG_USING_BOD_LP     BIT(17)
#define SEQ_CFG_USING_VREG_LP    BIT(16)
#define SEQ_CFG_USE_FAST_POWCK   BIT(12)
#define SEQ_CFG_USE_BOD_HP       BIT(7)
#define SEQ_CFG_USE_BOD_LP       BIT(6)
#define SEQ_CFG_USE_VREG_HP      BIT(5)
#define SEQ_CFG_USE_VREG_LP      BIT(4)
#define SEQ_CFG_HW_PWRUP_SRAM0   BIT(1)
#define SEQ_CFG_HW_PWRUP_SRAM1   BIT(0)
#define SEQ_CFG_RW               0x11f3u
#define SEQ_CFG_RESET            0x001011f0u

#define STATE_CHANGING              BIT(13)
#define STATE_WAITING               BIT(12)
#define STATE_BAD_HW_REQ            BIT(11)
#define STATE_BAD_SW_REQ            BIT(10)
#define STATE_PWRUP_WHILE_WAITING   BIT(9)
#define STATE_REQ_IGNORED           BIT(8)
#define STATE_REQ_SHIFT             4
#define STATE_CURRENT_MASK          0xfu
#define STATE_FLAGS (STATE_BAD_HW_REQ | STATE_BAD_SW_REQ | \
                     STATE_PWRUP_WHILE_WAITING | STATE_REQ_IGNORED)

#define DOM(d)              BIT(RP2350_POWMAN_##d)

#define POW_FASTDIV_MASK    0x7ffu
#define POW_FASTDIV_RESET   0x40u
#define POW_DELAY_MASK      0xffffu
#define POW_DELAY_RESET     0x2011u

#define EXT_CTRL_MASK       0x713fu
#define EXT_CTRL_RESET      0x3fu
#define EXT_CTRL_GPIO_SELECT 0x3fu
/* GPIO_SELECT values from 31 up disable the output. */
#define EXT_CTRL_GPIO_NONE  31

#define EXT_TIME_REF_DRIVE_LPCK BIT(4)
#define EXT_TIME_REF_SOURCE_SEL 0x3u
#define EXT_TIME_REF_MASK   0x13u

#define LPOSC_FREQ_KHZ_INT_RESET  0x20u
#define LPOSC_FREQ_KHZ_FRAC_RESET 0xc49cu
#define XOSC_FREQ_KHZ_INT_RESET   0x2ee0u

#define TIMER_NONSEC_WRITE      BIT(0)
#define TIMER_RUN               BIT(1)
#define TIMER_CLEAR             BIT(2)
#define TIMER_ALARM_ENAB        BIT(4)
#define TIMER_PWRUP_ON_ALARM    BIT(5)
#define TIMER_ALARM             BIT(6)
#define TIMER_USE_LPOSC         BIT(8)
#define TIMER_USE_XOSC          BIT(9)
#define TIMER_USE_GPIO_1KHZ     BIT(10)
#define TIMER_USE_GPIO_1HZ      BIT(13)
#define TIMER_USING_XOSC        BIT(16)
#define TIMER_USING_LPOSC       BIT(17)
#define TIMER_USING_GPIO_1KHZ   BIT(18)
#define TIMER_USING_GPIO_1HZ    BIT(19)
#define TIMER_USE_SRC (TIMER_USE_LPOSC | TIMER_USE_XOSC | TIMER_USE_GPIO_1KHZ)
#define TIMER_USING_SRC (TIMER_USING_XOSC | TIMER_USING_LPOSC | \
                         TIMER_USING_GPIO_1KHZ)
/* TIMER fields only Secure code may write. */
#define TIMER_SECURE_ONLY (TIMER_NONSEC_WRITE | TIMER_PWRUP_ON_ALARM | \
                           TIMER_USE_GPIO_1KHZ | TIMER_USE_GPIO_1HZ)

#define PWRUP_SOURCE        0x3fu
#define PWRUP_ENABLE        BIT(6)
#define PWRUP_DIRECTION     BIT(7)
#define PWRUP_MODE_EDGE     BIT(8)
#define PWRUP_STATUS        BIT(9)
#define PWRUP_RAW_STATUS    BIT(10)
#define PWRUP_RW            0x1ffu
#define PWRUP_RESET         0x3fu

/* CURRENT_PWRUP_REQ and LAST_SWCORE_PWRUP: bit n is source n. */
#define PWRUP_REQ_GPIO(n)   BIT(1 + (n))
#define PWRUP_REQ_ALARM     BIT(6)

#define BOOTDIS_NEXT        BIT(1)
#define BOOTDIS_NOW         BIT(0)

#define INTR_PWRUP_WHILE_WAITING BIT(3)
#define INTR_STATE_REQ_IGNORED   BIT(2)
#define INTR_TIMER               BIT(1)
#define INTR_VREG_OUTPUT_LOW     BIT(0)
#define INT_MASK                 0xfu
#define INT_POW (INTR_PWRUP_WHILE_WAITING | INTR_STATE_REQ_IGNORED | \
                 INTR_VREG_OUTPUT_LOW)

/* Steps in each domain's power sequence (DBG_POW_STATE bits 1-8). */
#define SEQ_STEPS           8

/* How long a VREG update takes to settle. */
#define VREG_UPDATE_NS      (50 * SCALE_US)

/* The AON timer's divider is at least 2.0 source cycles per tick. */
#define AON_DIV_MIN         (2u << 16)
/* Alarms further ahead than this many ticks are not scheduled. */
#define AON_MAX_TICKS       (1ull << 40)

enum {
    SEQ_IDLE,
    SEQ_WAITING,
    SEQ_CHANGING,
};

/* The chip-level resets, by what the next system reset stands for. */
typedef enum PowmanReset {
    RESET_POR,
    RESET_BOR,
    RESET_RUN_LOW,
    RESET_WATCHDOG_POWMAN_ASYNC,
    RESET_WATCHDOG_POWMAN,
    RESET_WATCHDOG_SWCORE,
    RESET_SWCORE_PD,
    /* Not a reset reason: the switched core lost power entering P1.x. */
    RESET_SWCORE_OFF,
} PowmanReset;

static const uint32_t powman_reset_reason[] = {
    [RESET_POR] = CHIP_RESET_HAD_POR,
    [RESET_BOR] = CHIP_RESET_HAD_BOR,
    [RESET_RUN_LOW] = CHIP_RESET_HAD_RUN_LOW,
    [RESET_WATCHDOG_POWMAN_ASYNC] = CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_ASYNC,
    [RESET_WATCHDOG_POWMAN] = CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN,
    [RESET_WATCHDOG_SWCORE] = CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE,
    [RESET_SWCORE_PD] = CHIP_RESET_HAD_SWCORE_PD,
    [RESET_SWCORE_OFF] = 0,
};

/* EXT_TIME_REF.SOURCE_SEL: the GPIOs that can be a time reference. */
static const int powman_time_ref_pin[] = { 12, 20, 14, 22 };

static void powman_update_irq(RP2350PowmanState *s);
static void powman_pwrup_changed(RP2350PowmanState *s);

static uint32_t powman_current(RP2350PowmanState *s)
{
    return s->state & STATE_CURRENT_MASK;
}

static void powman_set_current(RP2350PowmanState *s, uint32_t current)
{
    s->state = (s->state & ~(STATE_CURRENT_MASK << STATE_REQ_SHIFT |
                             STATE_CURRENT_MASK)) |
               current << STATE_REQ_SHIFT | current;
}

static uint32_t powman_req(RP2350PowmanState *s)
{
    return (s->state >> STATE_REQ_SHIFT) & STATE_CURRENT_MASK;
}

static void powman_set_req(RP2350PowmanState *s, uint32_t req)
{
    s->state = (s->state & ~(STATE_CURRENT_MASK << STATE_REQ_SHIFT)) |
               req << STATE_REQ_SHIFT;
}

/* AON timer */

static uint64_t aon_alarm(RP2350PowmanState *s)
{
    uint64_t v = 0;
    int i;

    for (i = 0; i < RP2350_POWMAN_TIME_REGS; i++) {
        v = v << 16 | s->alarm_time[i];
    }
    return v;
}

static uint64_t aon_set_time(RP2350PowmanState *s)
{
    uint64_t v = 0;
    int i;

    for (i = 0; i < RP2350_POWMAN_TIME_REGS; i++) {
        v = v << 16 | s->set_time[i];
    }
    return v;
}

static uint32_t aon_div(uint32_t khz_int, uint32_t khz_frac)
{
    return MAX(khz_int << 16 | khz_frac, AON_DIV_MIN);
}

/*
 * The clock that advances the count with time: its frequency, and the
 * divider (source cycles per tick, 16.16). False when the count is
 * stopped or advances only on GPIO edges.
 */
static bool aon_clock(RP2350PowmanState *s, uint64_t *hz, uint64_t *div)
{
    if (!(s->timer & TIMER_RUN)) {
        return false;
    }
    /* [spec:nuos:req:emu.clock-tree] */
    if (s->timer & TIMER_USING_XOSC) {
        if (!clock_is_enabled(s->clk_ref)) {
            return false;
        }
        *hz = clock_get_hz(s->clk_ref);
        *div = aon_div(s->xosc_freq_int, s->xosc_freq_frac);
        return true;
    }
    if ((s->timer & TIMER_USING_LPOSC) &&
        !(s->ext_time_ref & EXT_TIME_REF_DRIVE_LPCK)) {
        *hz = s->lposc_hz;
        *div = aon_div(s->lposc_freq_int, s->lposc_freq_frac);
        return true;
    }
    return false;
}

/* Whole ticks in `ns` of the clock: ns * hz * 2^16 / (div * 10^9). */
static uint64_t aon_ticks_in(uint64_t ns, uint64_t hz, uint64_t div)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, ns, hz << 16);
    divu128(&lo, &hi, div * NANOSECONDS_PER_SECOND);
    return hi ? UINT64_MAX : lo;
}

/* Virtual time taken by n ticks (n <= AON_MAX_TICKS), rounded up. */
static int64_t aon_ticks_ns(uint64_t n, uint64_t hz, uint64_t div)
{
    uint64_t lo, hi, plo, phi, rem;

    /* n * div is below 2^72 and n * div * 10^9 below 2^102. */
    mulu64(&lo, &hi, n, div);
    mulu64(&plo, &phi, lo, NANOSECONDS_PER_SECOND);
    phi += hi * NANOSECONDS_PER_SECOND;
    rem = divu128(&plo, &phi, hz << 16);
    if (phi || plo >= INT64_MAX) {
        return INT64_MAX;
    }
    return plo + (rem != 0);
}

static uint64_t aon_elapsed(RP2350PowmanState *s, int64_t now)
{
    uint64_t hz, div;

    if (!aon_clock(s, &hz, &div) || now <= s->aon_start_ns) {
        return 0;
    }
    return aon_ticks_in(now - s->aon_start_ns, hz, div);
}

static uint64_t aon_time(RP2350PowmanState *s)
{
    uint64_t t = s->aon_base +
                 aon_elapsed(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));

    if ((s->timer & TIMER_USING_GPIO_1HZ) && t > s->aon_sec_limit) {
        t = s->aon_sec_limit;
    }
    return t;
}

/*
 * Fold elapsed ticks into aon_base, keeping aon_start_ns on a tick
 * boundary so that partial ticks are not lost. Called before anything
 * that changes the count, its clock or whether it runs.
 */
static void aon_sync(RP2350PowmanState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t hz, div, n, t;

    if (!aon_clock(s, &hz, &div)) {
        s->aon_start_ns = now;
        return;
    }
    if (now <= s->aon_start_ns) {
        return;
    }
    n = aon_ticks_in(now - s->aon_start_ns, hz, div);
    t = s->aon_base + n;
    if ((s->timer & TIMER_USING_GPIO_1HZ) && t >= s->aon_sec_limit) {
        /* Waiting at the second boundary for the 1 Hz edge. */
        s->aon_base = MAX(s->aon_base, s->aon_sec_limit);
        s->aon_start_ns = now;
        return;
    }
    s->aon_base = t;
    s->aon_start_ns += n <= AON_MAX_TICKS ? aon_ticks_ns(n, hz, div)
                                          : now - s->aon_start_ns;
}

/* The next second boundary, where the 1 Hz sync holds the count. */
static uint64_t aon_next_second(uint64_t t)
{
    return (t / 1000 + 1) * 1000;
}

/*
 * Re-evaluate the alarm comparison, whose rising edge sets TIMER.ALARM,
 * and schedule the tick on which the count next reaches the alarm.
 */
static void aon_update(RP2350PowmanState *s)
{
    uint64_t t = aon_time(s);
    uint64_t alarm = aon_alarm(s);
    uint64_t hz, div;
    bool cmp = (s->timer & TIMER_ALARM_ENAB) && t >= alarm;

    if (cmp && !s->alarm_cmp) {
        s->timer |= TIMER_ALARM;
        qemu_irq_pulse(s->alarm_wake);
    }
    s->alarm_cmp = cmp;

    timer_del(s->alarm_timer);
    if ((s->timer & TIMER_ALARM_ENAB) && !cmp && aon_clock(s, &hz, &div) &&
        !((s->timer & TIMER_USING_GPIO_1HZ) && alarm > s->aon_sec_limit) &&
        alarm - s->aon_base <= AON_MAX_TICKS) {
        int64_t ns = aon_ticks_ns(alarm - s->aon_base, hz, div);

        if (ns < INT64_MAX - s->aon_start_ns) {
            timer_mod(s->alarm_timer, s->aon_start_ns + ns);
        }
    }
    powman_update_irq(s);
    powman_pwrup_changed(s);
}

static void aon_alarm_cb(void *opaque)
{
    aon_update(opaque);
}

/* Switch the tick source to the one TIMER.USE_* selects. */
static void aon_switch(RP2350PowmanState *s)
{
    uint32_t use = s->timer & TIMER_USE_SRC;
    uint32_t using;

    timer_del(s->switch_timer);
    if (!use) {
        return;
    }
    aon_sync(s);
    if (use & TIMER_USE_LPOSC) {
        using = TIMER_USING_LPOSC;
    } else if (use & TIMER_USE_XOSC) {
        using = TIMER_USING_XOSC;
    } else {
        using = TIMER_USING_GPIO_1KHZ;
    }
    s->timer = (s->timer & ~(TIMER_USE_SRC | TIMER_USING_SRC)) | using;
    s->aon_lpck_acc = 0;
    aon_update(s);
}

static void aon_switch_cb(void *opaque)
{
    aon_switch(opaque);
}

/*
 * A USE_LPOSC or USE_XOSC request takes effect on the current source's
 * next tick; the GPIO tick is not synchronised and switches at once. A
 * stopped timer switches at once too.
 */
static void aon_use(RP2350PowmanState *s, uint32_t use)
{
    uint64_t hz, div;

    s->timer = (s->timer & ~TIMER_USE_SRC) | use;
    if (use == TIMER_USE_GPIO_1KHZ || !aon_clock(s, &hz, &div)) {
        aon_switch(s);
        return;
    }
    aon_sync(s);
    timer_mod(s->switch_timer,
              s->aon_start_ns + aon_ticks_ns(1, hz, div));
}

/* A falling edge on the EXT_TIME_REF GPIO. */
static void aon_ref_edge(RP2350PowmanState *s)
{
    uint32_t div;

    if (!(s->timer & TIMER_RUN)) {
        return;
    }
    if (s->timer & TIMER_USING_GPIO_1HZ) {
        /*
         * The second boundary: a slow count catches up to it (counting
         * every millisecond on the way, as far as the alarm is
         * concerned); a count waiting there carries on.
         */
        aon_sync(s);
        s->aon_base = MAX(s->aon_base, s->aon_sec_limit);
        s->aon_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->aon_sec_limit = s->aon_base + 1000;
    } else if (s->timer & TIMER_USING_GPIO_1KHZ) {
        s->aon_base++;
    } else if ((s->timer & TIMER_USING_LPOSC) &&
               (s->ext_time_ref & EXT_TIME_REF_DRIVE_LPCK)) {
        div = aon_div(s->lposc_freq_int, s->lposc_freq_frac);
        s->aon_lpck_acc += 1u << 16;
        while (s->aon_lpck_acc >= div) {
            s->aon_lpck_acc -= div;
            s->aon_base++;
        }
    } else {
        return;
    }
    aon_update(s);
}

static void aon_reset(RP2350PowmanState *s)
{
    timer_del(s->alarm_timer);
    timer_del(s->switch_timer);
    s->timer = 0;
    s->aon_base = 0;
    s->aon_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->aon_sec_limit = 0;
    s->aon_lpck_acc = 0;
    s->alarm_cmp = false;
    s->lposc_freq_int = LPOSC_FREQ_KHZ_INT_RESET;
    s->lposc_freq_frac = LPOSC_FREQ_KHZ_FRAC_RESET;
    s->xosc_freq_int = XOSC_FREQ_KHZ_INT_RESET;
    s->xosc_freq_frac = 0;
    memset(s->set_time, 0, sizeof(s->set_time));
    memset(s->alarm_time, 0, sizeof(s->alarm_time));
}

/* [spec:nuos:req:emu.powman] */
static void aon_timer_write(RP2350PowmanState *s, uint32_t eff,
                            uint32_t wdata, bool secure)
{
    uint32_t old = s->timer;
    uint32_t use;

    if (!secure) {
        eff = (eff & ~TIMER_SECURE_ONLY) | (old & TIMER_SECURE_ONLY);
        wdata &= ~TIMER_SECURE_ONLY;
    }
    aon_sync(s);
    if (wdata & TIMER_ALARM) {
        s->timer &= ~TIMER_ALARM;
    }
    s->timer = (s->timer & ~(TIMER_NONSEC_WRITE | TIMER_ALARM_ENAB |
                             TIMER_PWRUP_ON_ALARM)) |
               (eff & (TIMER_NONSEC_WRITE | TIMER_ALARM_ENAB |
                       TIMER_PWRUP_ON_ALARM));

    if ((eff & TIMER_USE_GPIO_1HZ) && !(old & TIMER_USE_GPIO_1HZ)) {
        s->timer |= TIMER_USE_GPIO_1HZ | TIMER_USING_GPIO_1HZ;
        s->aon_sec_limit = aon_next_second(s->aon_base);
    } else if (!(eff & TIMER_USE_GPIO_1HZ) && (old & TIMER_USE_GPIO_1HZ)) {
        s->timer &= ~(TIMER_USE_GPIO_1HZ | TIMER_USING_GPIO_1HZ);
    }

    if (eff & TIMER_CLEAR) {
        s->aon_base = 0;
        s->aon_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->aon_lpck_acc = 0;
        s->aon_sec_limit = aon_next_second(0);
    }

    if ((eff & TIMER_RUN) && !(old & TIMER_RUN)) {
        /* Starting loads SET_TIME; with no source yet it is the LPOSC. */
        s->timer |= TIMER_RUN;
        s->aon_base = aon_set_time(s);
        s->aon_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->aon_lpck_acc = 0;
        s->aon_sec_limit = aon_next_second(s->aon_base);
        if (!(s->timer & TIMER_USING_SRC)) {
            s->timer |= TIMER_USING_LPOSC;
        }
    } else if (!(eff & TIMER_RUN) && (old & TIMER_RUN)) {
        s->timer &= ~TIMER_RUN;
        s->aon_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }

    use = eff & TIMER_USE_SRC;
    if (use && use != (old & TIMER_USE_SRC)) {
        aon_use(s, use & -use);
    }
    aon_update(s);
}

/* Power-up sources */

/* The pin a PWRUP source selects, or -1 for a selection that is ignored. */
static int pwrup_pin(RP2350PowmanState *s, int n)
{
    static const int qspi[] = {
        RP2350_GPIO_QSPI_SS, RP2350_GPIO_QSPI_SD0, RP2350_GPIO_QSPI_SD1,
        RP2350_GPIO_QSPI_SD2, RP2350_GPIO_QSPI_SD3, RP2350_GPIO_QSPI_SCLK,
    };
    uint32_t src = s->pwrup[n] & PWRUP_SOURCE;

    if (src < s->bonded_gpios) {
        return src;
    }
    if (src >= RP2350_GPIO_BANK0_PINS &&
        src < RP2350_GPIO_BANK0_PINS + ARRAY_SIZE(qspi)) {
        return qspi[src - RP2350_GPIO_BANK0_PINS];
    }
    return -1;
}

/* Whether a PWRUP source's pin is at its active level. */
static bool pwrup_level(RP2350PowmanState *s, int n)
{
    int pin = pwrup_pin(s, n);

    return pin >= 0 &&
           s->pin[pin] == !!(s->pwrup[n] & PWRUP_DIRECTION);
}

static bool pwrup_active(RP2350PowmanState *s, int n)
{
    if (!(s->pwrup[n] & PWRUP_ENABLE)) {
        return false;
    }
    if (s->pwrup[n] & PWRUP_MODE_EDGE) {
        return s->pwrup_latch & BIT(n);
    }
    return pwrup_level(s, n);
}

/* CURRENT_PWRUP_REQ */
static uint32_t powman_pwrup_reqs(RP2350PowmanState *s)
{
    uint32_t reqs = 0;
    int n;

    for (n = 0; n < RP2350_POWMAN_PWRUPS; n++) {
        if (pwrup_active(s, n)) {
            reqs |= PWRUP_REQ_GPIO(n);
        }
    }
    if ((s->timer & (TIMER_PWRUP_ON_ALARM | TIMER_ALARM_ENAB | TIMER_ALARM)) ==
        (TIMER_PWRUP_ON_ALARM | TIMER_ALARM_ENAB | TIMER_ALARM)) {
        reqs |= PWRUP_REQ_ALARM;
    }
    return reqs;
}

static uint32_t pwrup_read(RP2350PowmanState *s, int n)
{
    uint32_t v = s->pwrup[n];
    int pin = pwrup_pin(s, n);

    if (v & PWRUP_MODE_EDGE ? s->pwrup_latch & BIT(n) : pwrup_active(s, n)) {
        v |= PWRUP_STATUS;
    }
    if ((v & PWRUP_ENABLE) && pin >= 0 && s->pin[pin]) {
        v |= PWRUP_RAW_STATUS;
    }
    return v;
}

/* Power sequencer */

static uint32_t powman_delay(RP2350PowmanState *s, int shift, int bits)
{
    return MAX(extract32(s->pow_delay, shift, bits), 1);
}

/*
 * The time clk_ref, the fast POWMAN clock, takes for `cycles`. A stopped
 * clk_ref makes no progress.
 */
/* [spec:nuos:req:emu.clock-tree] */
static int64_t powman_ref_ns(RP2350PowmanState *s, uint64_t cycles)
{
    if (!cycles) {
        return 0;
    }
    if (!clock_is_enabled(s->clk_ref)) {
        return INT64_MAX / 4;
    }
    return clock_ticks_to_ns(s->clk_ref, cycles);
}

/*
 * How long the sequencer takes from power state `from` to `to`: SWCORE
 * and XIP steps in LPOSC periods, SRAM steps in POWMAN ticks.
 */
static int64_t powman_seq_ns(RP2350PowmanState *s, uint32_t from, uint32_t to)
{
    uint32_t changed = from ^ to;
    uint64_t lposc_cycles = 0, ref_cycles = 0, sram_ticks = 0;
    bool fast = !(from & DOM(SWCORE)) && !(to & DOM(SWCORE)) &&
                (s->seq_cfg & SEQ_CFG_USING_FAST_POWCK);

    if (changed & DOM(SWCORE)) {
        lposc_cycles += SEQ_STEPS * powman_delay(s, 0, 4);
    }
    if (changed & DOM(XIP)) {
        lposc_cycles += SEQ_STEPS * powman_delay(s, 4, 4);
    }
    sram_ticks = SEQ_STEPS * powman_delay(s, 8, 8) *
                 ctpop32(changed & (DOM(SRAM0) | DOM(SRAM1)));
    if (fast) {
        ref_cycles = sram_ticks * MAX(s->pow_fastdiv, 1);
    } else {
        lposc_cycles += sram_ticks;
    }
    return muldiv64(lposc_cycles, NANOSECONDS_PER_SECOND, s->lposc_hz) +
           powman_ref_ns(s, ref_cycles);
}

/* The polling interval while WAITING: one POWMAN tick. */
static int64_t powman_tick_ns(RP2350PowmanState *s)
{
    if (s->seq_cfg & SEQ_CFG_USING_FAST_POWCK) {
        return powman_ref_ns(s, MAX(s->pow_fastdiv, 1));
    }
    return muldiv64(1, NANOSECONDS_PER_SECOND, s->lposc_hz);
}

static void powman_seq_start(RP2350PowmanState *s, uint32_t target)
{
    s->seq = SEQ_CHANGING;
    s->seq_target = target;
    timer_mod(s->seq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                            powman_seq_ns(s, powman_current(s), target));
}

/* Ask for a system reset that this device carries out as `kind`. */
static void powman_chip_reset(RP2350PowmanState *s, PowmanReset kind,
                              ShutdownCause cause)
{
    s->next_reset = kind;
    qemu_system_reset_request(cause);
}

/*
 * The switched core powers down: the processors stop, the AON timer falls
 * back from the XOSC, which stops with it, and the regulator and brown-out
 * detector take their low-power settings.
 */
/* [spec:nuos:req:emu.powman] */
static void powman_swcore_down(RP2350PowmanState *s)
{
    int i;

    timer_del(s->seq_timer);
    if (s->ops) {
        s->ops->swcore_stop(s->ops_opaque);
    }
    if (s->timer & (TIMER_USING_XOSC | TIMER_USE_XOSC)) {
        aon_sync(s);
        timer_del(s->switch_timer);
        s->timer = (s->timer & ~(TIMER_USE_SRC | TIMER_USING_SRC)) |
                   TIMER_USING_LPOSC;
        aon_update(s);
    }
    if (s->seq_cfg & SEQ_CFG_USE_VREG_LP) {
        s->vreg = (s->vreg & ~VREG_MASK) | (s->vreg_lp_entry & VREG_MASK);
        s->seq_cfg = deposit32(s->seq_cfg, 16, 1,
                               !!(s->vreg_lp_entry & VREG_LP_MODE));
    }
    if (s->seq_cfg & SEQ_CFG_USE_BOD_LP) {
        s->bod = s->bod_lp_entry;
        s->seq_cfg |= SEQ_CFG_USING_BOD_LP;
    }
    for (i = 0; i < ARRAY_SIZE(s->ext_ctrl); i++) {
        if ((s->ext_ctrl[i] & EXT_CTRL_GPIO_SELECT) < EXT_CTRL_GPIO_NONE) {
            qemu_log_mask(LOG_UNIMP, "rp2350-powman: EXT_CTRL%d: the "
                          "power-mode-aware GPIO output is not driven\n", i);
        }
    }
    powman_seq_start(s, powman_req(s));
}

/*
 * The state a power-up from a low-power state reaches: everything but
 * the SRAM domains SEQ_CFG.HW_PWRUP_SRAM0/1 leave as they are.
 */
static uint32_t powman_wake_target(RP2350PowmanState *s)
{
    uint32_t target = 0;

    if (s->seq_cfg & SEQ_CFG_HW_PWRUP_SRAM0) {
        target |= powman_current(s) & DOM(SRAM0);
    }
    if (s->seq_cfg & SEQ_CFG_HW_PWRUP_SRAM1) {
        target |= powman_current(s) & DOM(SRAM1);
    }
    return target;
}

/* [spec:nuos:req:emu.powman] */
static void powman_seq_complete(RP2350PowmanState *s)
{
    uint32_t from = powman_current(s);
    uint32_t to = s->seq_target;
    int d;

    s->seq = SEQ_IDLE;
    powman_set_current(s, to);
    for (d = RP2350_POWMAN_SRAM1; d <= RP2350_POWMAN_XIP; d++) {
        if (s->ops && ((from ^ to) & BIT(d))) {
            s->ops->domain_power(s->ops_opaque, d, !(to & BIT(d)));
        }
    }

    if (!(from & DOM(SWCORE)) && (to & DOM(SWCORE))) {
        /* In a low-power state: POWMAN runs from the LPOSC. */
        s->seq_cfg &= ~SEQ_CFG_USING_FAST_POWCK;
        powman_chip_reset(s, RESET_SWCORE_OFF, SHUTDOWN_CAUSE_SUBSYSTEM_RESET);
        /* A request that came during the power-down powers up at once. */
        s->wake_reqs = powman_pwrup_reqs(s);
        if (s->wake_reqs) {
            powman_seq_start(s, powman_wake_target(s));
        }
    } else if ((from & DOM(SWCORE)) && !(to & DOM(SWCORE))) {
        if (s->seq_cfg & SEQ_CFG_USE_FAST_POWCK) {
            s->seq_cfg |= SEQ_CFG_USING_FAST_POWCK;
        }
        if (s->seq_cfg & SEQ_CFG_USE_VREG_HP) {
            s->vreg = (s->vreg & ~VREG_MASK) | (s->vreg_lp_exit & VREG_MASK);
            s->seq_cfg &= ~SEQ_CFG_USING_VREG_LP;
            if (s->vreg_lp_exit & (VREG_LP_MODE | VREG_HIZ)) {
                qemu_log_mask(LOG_UNIMP, "rp2350-powman: VREG_LP_EXIT "
                              "selects low-power or high-impedance mode "
                              "for the running chip\n");
            }
        }
        if (s->seq_cfg & SEQ_CFG_USE_BOD_HP) {
            s->bod = s->bod_lp_exit;
            s->seq_cfg &= ~SEQ_CFG_USING_BOD_LP;
        }
        s->last_swcore_pwrup = s->wake_reqs;
        powman_chip_reset(s, RESET_SWCORE_PD, SHUTDOWN_CAUSE_SUBSYSTEM_RESET);
    }
    powman_update_irq(s);
}

static void powman_seq_cb(void *opaque)
{
    RP2350PowmanState *s = opaque;

    switch (s->seq) {
    case SEQ_WAITING:
        if (!s->ops || s->ops->procs_asleep(s->ops_opaque)) {
            powman_swcore_down(s);
        } else {
            timer_mod(s->seq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                    powman_tick_ns(s));
        }
        break;
    case SEQ_CHANGING:
        powman_seq_complete(s);
        break;
    }
}

/*
 * A power-up request: it cancels a switched-core power-down still waiting
 * for the processors, and powers up a chip in a low-power state.
 */
/* [spec:nuos:req:emu.powman] */
static void powman_pwrup_changed(RP2350PowmanState *s)
{
    uint32_t reqs = powman_pwrup_reqs(s);

    if (!reqs) {
        return;
    }
    switch (s->seq) {
    case SEQ_WAITING:
        s->seq = SEQ_IDLE;
        timer_del(s->seq_timer);
        s->state |= STATE_PWRUP_WHILE_WAITING | STATE_REQ_IGNORED;
        powman_set_req(s, powman_current(s));
        powman_update_irq(s);
        break;
    case SEQ_IDLE:
        if (powman_current(s) & DOM(SWCORE)) {
            s->wake_reqs = reqs;
            powman_seq_start(s, powman_wake_target(s));
        }
        break;
    }
}

/*
 * The datasheet's valid software transitions: from a normal state (P0.x),
 * either powering domains up or down but not both, keeping XIP powered
 * while the switched core is.
 */
static bool powman_valid_req(uint32_t cur, uint32_t req)
{
    uint32_t up = cur & ~req;
    uint32_t down = req & ~cur;

    if (cur & DOM(SWCORE)) {
        return false;
    }
    if (!(req & DOM(SWCORE)) && (req & DOM(XIP))) {
        return false;
    }
    return !(up && down);
}

/* [spec:nuos:req:emu.powman] */
static void powman_state_write(RP2350PowmanState *s, uint32_t eff,
                               uint32_t wdata)
{
    uint32_t req = (eff >> STATE_REQ_SHIFT) & STATE_CURRENT_MASK;
    uint32_t cur = powman_current(s);

    if (s->seq == SEQ_CHANGING) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: STATE written while "
                      "a power state change is in progress\n");
        return;
    }
    s->state &= ~(wdata & (STATE_PWRUP_WHILE_WAITING | STATE_REQ_IGNORED));

    if (s->seq == SEQ_WAITING) {
        if (req == powman_req(s)) {
            goto out;
        }
        /* A new request replaces, or cancels, the waiting one. */
        s->seq = SEQ_IDLE;
        timer_del(s->seq_timer);
        powman_set_req(s, cur);
    }
    if (req == cur) {
        /* Asking for the current state is valid and does nothing. */
        powman_set_req(s, cur);
        s->state &= ~STATE_BAD_SW_REQ;
        goto out;
    }
    if (req == powman_req(s)) {
        goto out;
    }
    if (powman_pwrup_reqs(s)) {
        s->state |= STATE_REQ_IGNORED;
        goto out;
    }
    powman_set_req(s, req);
    if (!powman_valid_req(cur, req)) {
        s->state |= STATE_BAD_SW_REQ;
        goto out;
    }
    s->state &= ~STATE_BAD_SW_REQ;
    if (req & DOM(SWCORE)) {
        /* The processors' sleep is sampled on each POWMAN tick. */
        s->seq = SEQ_WAITING;
        timer_mod(s->seq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                powman_tick_ns(s));
    } else {
        powman_seq_start(s, req);
    }
out:
    powman_update_irq(s);
}

static uint32_t powman_state_read(RP2350PowmanState *s)
{
    return s->state |
           (s->seq == SEQ_WAITING ? STATE_WAITING : 0) |
           (s->seq == SEQ_CHANGING ? STATE_CHANGING : 0);
}

/* Regulator */

/* The output voltage of a VSEL selection, in mV. */
static unsigned vreg_mv(uint32_t vsel)
{
    static const uint16_t high[] = {
        1350, 1400, 1500, 1600, 1650, 1700, 1800, 1900,
        2000, 2350, 2500, 2650, 2800, 3000, 3150, 3300,
    };

    return vsel < 16 ? 550 + 50 * vsel : high[vsel - 16];
}

/* The selection in effect, after the 1.3 V limit. */
static uint32_t vreg_vsel(RP2350PowmanState *s, uint32_t vreg)
{
    uint32_t vsel = extract32(vreg, VREG_VSEL_SHIFT, 5);

    if (!(s->vreg_ctrl & VREG_CTRL_DISABLE_VOLTAGE_LIMIT)) {
        vsel = MIN(vsel, VREG_VSEL_LIMIT);
    }
    return vsel;
}

static void vreg_reset(RP2350PowmanState *s)
{
    timer_del(s->vreg_timer);
    s->vreg = VREG_RESET;
    s->vreg_startup = true;
    s->vout_low = false;
}

/* [spec:nuos:req:emu.powman] */
static void vreg_write(RP2350PowmanState *s, uint32_t eff)
{
    uint32_t v = eff & VREG_MASK;
    unsigned old_mv, new_mv;

    if (!(s->vreg_ctrl & VREG_CTRL_UNLOCK)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: VREG written while "
                      "VREG_CTRL.UNLOCK is clear\n");
        return;
    }
    if (!(s->vreg_ctrl & VREG_CTRL_RST_N) || timer_pending(s->vreg_timer) ||
        v == s->vreg) {
        return;
    }
    if (v & VREG_HIZ) {
        /*
         * The on-chip regulator supplies the core, so high-impedance mode
         * takes its supply away: the brown-out detector resets the chip,
         * or without it the core supply's power-on reset does.
         */
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: VREG.HIZ set while "
                      "the regulator supplies the core; resetting\n");
        powman_chip_reset(s, s->bod & BOD_EN ? RESET_BOR : RESET_POR,
                          SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    old_mv = vreg_mv(vreg_vsel(s, s->vreg));
    new_mv = vreg_mv(vreg_vsel(s, v));
    s->vreg = v;
    s->vreg_startup = false;
    /* VOUT_OK deasserts below 87% of the new selection. */
    s->vout_low = old_mv * 100 < new_mv * 87;
    if (s->vout_low) {
        s->intr |= INTR_VREG_OUTPUT_LOW;
    }
    timer_mod(s->vreg_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + VREG_UPDATE_NS);
    powman_update_irq(s);
}

static void vreg_cb(void *opaque)
{
    RP2350PowmanState *s = opaque;

    s->vout_low = false;
}

/* Interrupts */

static uint32_t powman_intr(RP2350PowmanState *s)
{
    return s->intr |
           (s->timer & TIMER_ALARM ? INTR_TIMER : 0) |
           (s->state & STATE_REQ_IGNORED ? INTR_STATE_REQ_IGNORED : 0) |
           (s->state & STATE_PWRUP_WHILE_WAITING ?
            INTR_PWRUP_WHILE_WAITING : 0);
}

static uint32_t powman_ints(RP2350PowmanState *s)
{
    return (powman_intr(s) | s->intf) & s->inte;
}

/* [spec:nuos:req:emu.powman] */
static void powman_update_irq(RP2350PowmanState *s)
{
    uint32_t ints = powman_ints(s);

    qemu_set_irq(s->irq[RP2350_POWMAN_IRQ_POW], !!(ints & INT_POW));
    qemu_set_irq(s->irq[RP2350_POWMAN_IRQ_TIMER], !!(ints & INTR_TIMER));
}

/* Registers */

static uint32_t powman_read_reg(RP2350PowmanState *s, hwaddr reg)
{
    switch (reg) {
    case A_BADPASSWD:
        return s->badpasswd;
    case A_VREG_CTRL:
        return s->vreg_ctrl;
    case A_VREG_STS:
        return (s->vout_low || (s->vreg & VREG_HIZ) ? 0 : VREG_STS_VOUT_OK) |
               (s->vreg_startup ? VREG_STS_STARTUP : 0);
    case A_VREG:
        return s->vreg |
               (timer_pending(s->vreg_timer) ? VREG_UPDATE_IN_PROGRESS : 0);
    case A_VREG_LP_ENTRY:
        return s->vreg_lp_entry;
    case A_VREG_LP_EXIT:
        return s->vreg_lp_exit;
    case A_BOD_CTRL:
        return s->bod_ctrl;
    case A_BOD:
        return s->bod;
    case A_BOD_LP_ENTRY:
        return s->bod_lp_entry;
    case A_BOD_LP_EXIT:
        return s->bod_lp_exit;
    case A_LPOSC:
        return s->lposc;
    case A_CHIP_RESET:
        return s->chip_reset;
    case A_WDSEL:
        return s->wdsel;
    case A_SEQ_CFG:
        return s->seq_cfg;
    case A_STATE:
        return powman_state_read(s);
    case A_POW_FASTDIV:
        return s->pow_fastdiv;
    case A_POW_DELAY:
        return s->pow_delay;
    case A_EXT_CTRL0:
    case A_EXT_CTRL1:
        return s->ext_ctrl[(reg - A_EXT_CTRL0) / 4];
    case A_EXT_TIME_REF:
        return s->ext_time_ref;
    case A_LPOSC_FREQ_KHZ_INT:
        return s->lposc_freq_int;
    case A_LPOSC_FREQ_KHZ_FRAC:
        return s->lposc_freq_frac;
    case A_XOSC_FREQ_KHZ_INT:
        return s->xosc_freq_int;
    case A_XOSC_FREQ_KHZ_FRAC:
        return s->xosc_freq_frac;
    case A_SET_TIME_63TO48 ... A_SET_TIME_15TO0:
        return s->set_time[(reg - A_SET_TIME_63TO48) / 4];
    case A_READ_TIME_UPPER:
        return aon_time(s) >> 32;
    case A_READ_TIME_LOWER:
        return aon_time(s);
    case A_ALARM_TIME_63TO48 ... A_ALARM_TIME_15TO0:
        return s->alarm_time[(reg - A_ALARM_TIME_63TO48) / 4];
    case A_TIMER:
        return s->timer;
    case A_PWRUP0 ... A_PWRUP3:
        return pwrup_read(s, (reg - A_PWRUP0) / 4);
    case A_CURRENT_PWRUP_REQ:
        return powman_pwrup_reqs(s);
    case A_LAST_SWCORE_PWRUP:
        return s->last_swcore_pwrup;
    case A_DBG_PWRCFG:
        return s->dbg_pwrcfg;
    case A_BOOTDIS:
        return s->bootdis;
    case A_DBGCONFIG:
        return s->dbgconfig;
    case A_SCRATCH0 ... A_SCRATCH7:
        return s->scratch[(reg - A_SCRATCH0) / 4];
    case A_BOOT0 ... A_BOOT3:
        return s->boot[(reg - A_BOOT0) / 4];
    case A_INTR:
        return powman_intr(s);
    case A_INTE:
        return s->inte;
    case A_INTF:
        return s->intf;
    case A_INTS:
        return powman_ints(s);
    }
    qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: read of bad offset 0x%"
                  HWADDR_PRIx "\n", reg);
    return 0;
}

/* Registers Non-secure code may access: the AON timer's. */
static bool powman_nonsec_reg(hwaddr reg)
{
    return reg >= A_SET_TIME_63TO48 && reg <= A_TIMER;
}

/* [spec:nuos:req:emu.powman] */
static MemTxResult rp2350_powman_read(void *opaque, hwaddr addr,
                                      uint64_t *data, unsigned size,
                                      MemTxAttrs attrs)
{
    RP2350PowmanState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr) & ~3;
    uint32_t v;

    if (!attrs.secure && !attrs.unspecified && !powman_nonsec_reg(reg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: Non-secure read of "
                      "offset 0x%" HWADDR_PRIx "\n", reg);
        *data = 0;
        return MEMTX_OK;
    }
    v = powman_read_reg(s, reg);
    /* A narrow read returns its byte lanes of the register. */
    *data = extract32(v, (addr & 3) * 8, size * 8);
    return MEMTX_OK;
}

static void powman_write_reg(RP2350PowmanState *s, hwaddr reg, uint32_t eff,
                             uint32_t wdata, bool secure)
{
    uint32_t old;

    switch (reg) {
    case A_BADPASSWD:
        s->badpasswd &= ~wdata;
        break;
    case A_VREG_CTRL:
        old = s->vreg_ctrl;
        /* UNLOCK cannot be cleared once set. */
        s->vreg_ctrl = (eff & VREG_CTRL_MASK) | (old & VREG_CTRL_UNLOCK);
        if ((old & VREG_CTRL_RST_N) && !(s->vreg_ctrl & VREG_CTRL_RST_N)) {
            /* The regulator returns to its startup settings. */
            vreg_reset(s);
        }
        break;
    case A_VREG:
        vreg_write(s, eff);
        break;
    case A_VREG_LP_ENTRY:
        s->vreg_lp_entry = eff & VREG_LP_MASK;
        break;
    case A_VREG_LP_EXIT:
        s->vreg_lp_exit = eff & VREG_LP_MASK;
        break;
    case A_BOD_CTRL:
        s->bod_ctrl = eff & BOD_CTRL_MASK;
        break;
    case A_BOD:
        s->bod = eff & BOD_MASK;
        break;
    case A_BOD_LP_ENTRY:
        s->bod_lp_entry = eff & BOD_MASK;
        break;
    case A_BOD_LP_EXIT:
        s->bod_lp_exit = eff & BOD_MASK;
        break;
    case A_LPOSC:
        s->lposc = eff & LPOSC_MASK;
        break;
    case A_CHIP_RESET:
        s->chip_reset = (s->chip_reset & ~CHIP_RESET_DOUBLE_TAP) |
                        (eff & CHIP_RESET_DOUBLE_TAP);
        s->chip_reset &= ~(wdata & CHIP_RESET_RESCUE_FLAG);
        break;
    case A_WDSEL:
        s->wdsel = eff & WDSEL_MASK;
        break;
    case A_SEQ_CFG:
        if (s->seq == SEQ_CHANGING) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: SEQ_CFG written "
                          "while a power state change is in progress\n");
            break;
        }
        s->seq_cfg = (s->seq_cfg & ~SEQ_CFG_RW) | (eff & SEQ_CFG_RW);
        break;
    case A_STATE:
        powman_state_write(s, eff, wdata);
        break;
    case A_POW_FASTDIV:
        s->pow_fastdiv = eff & POW_FASTDIV_MASK;
        break;
    case A_POW_DELAY:
        s->pow_delay = eff & POW_DELAY_MASK;
        break;
    case A_EXT_CTRL0:
    case A_EXT_CTRL1:
        s->ext_ctrl[(reg - A_EXT_CTRL0) / 4] = eff & EXT_CTRL_MASK;
        break;
    case A_EXT_TIME_REF:
        if ((s->timer & TIMER_RUN) &&
            ((eff ^ s->ext_time_ref) & EXT_TIME_REF_DRIVE_LPCK)) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: "
                          "EXT_TIME_REF.DRIVE_LPCK written while the timer "
                          "runs\n");
        }
        aon_sync(s);
        s->ext_time_ref = eff & EXT_TIME_REF_MASK;
        s->aon_lpck_acc = 0;
        aon_update(s);
        break;
    case A_LPOSC_FREQ_KHZ_INT:
    case A_LPOSC_FREQ_KHZ_FRAC:
    case A_XOSC_FREQ_KHZ_INT:
    case A_XOSC_FREQ_KHZ_FRAC:
        if ((s->timer & TIMER_RUN) &&
            (s->timer & (reg < A_XOSC_FREQ_KHZ_INT ? TIMER_USING_LPOSC
                                                   : TIMER_USING_XOSC))) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: the frequency of "
                          "the running timer's source written\n");
        }
        aon_sync(s);
        switch (reg) {
        case A_LPOSC_FREQ_KHZ_INT:
            s->lposc_freq_int = eff & 0x3f;
            break;
        case A_LPOSC_FREQ_KHZ_FRAC:
            s->lposc_freq_frac = eff & 0xffff;
            break;
        case A_XOSC_FREQ_KHZ_INT:
            s->xosc_freq_int = eff & 0xffff;
            break;
        default:
            s->xosc_freq_frac = eff & 0xffff;
            break;
        }
        aon_update(s);
        break;
    case A_SET_TIME_63TO48 ... A_SET_TIME_15TO0:
        if (s->timer & TIMER_RUN) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: SET_TIME written "
                          "while the timer runs\n");
        }
        s->set_time[(reg - A_SET_TIME_63TO48) / 4] = eff;
        break;
    case A_ALARM_TIME_63TO48 ... A_ALARM_TIME_15TO0:
        if (s->timer & TIMER_ALARM_ENAB) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: ALARM_TIME "
                          "written while the alarm is enabled\n");
        }
        s->alarm_time[(reg - A_ALARM_TIME_63TO48) / 4] = eff;
        aon_update(s);
        break;
    case A_TIMER:
        aon_timer_write(s, eff, wdata, secure);
        break;
    case A_PWRUP0 ... A_PWRUP3: {
        int n = (reg - A_PWRUP0) / 4;

        s->pwrup[n] = eff & PWRUP_RW;
        if (wdata & PWRUP_STATUS) {
            s->pwrup_latch &= ~BIT(n);
        }
        powman_pwrup_changed(s);
        break;
    }
    case A_DBG_PWRCFG:
        s->dbg_pwrcfg = eff & 1;
        break;
    case A_BOOTDIS:
        /* NEXT can be set but not cleared; NOW is write-1-to-clear. */
        s->bootdis |= eff & BOOTDIS_NEXT;
        s->bootdis &= ~(wdata & BOOTDIS_NOW);
        break;
    case A_DBGCONFIG:
        s->dbgconfig = eff & 0xf;
        break;
    case A_SCRATCH0 ... A_SCRATCH7:
        s->scratch[(reg - A_SCRATCH0) / 4] = eff;
        break;
    case A_BOOT0 ... A_BOOT3:
        s->boot[(reg - A_BOOT0) / 4] = eff;
        break;
    case A_INTR:
        s->intr &= ~(wdata & INTR_VREG_OUTPUT_LOW);
        powman_update_irq(s);
        break;
    case A_INTE:
        s->inte = eff & INT_MASK;
        powman_update_irq(s);
        break;
    case A_INTF:
        s->intf = eff & INT_MASK;
        powman_update_irq(s);
        break;
    case A_VREG_STS:
    case A_READ_TIME_UPPER:
    case A_READ_TIME_LOWER:
    case A_CURRENT_PWRUP_REQ:
    case A_LAST_SWCORE_PWRUP:
    case A_INTS:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", reg);
        break;
    }
}

/* [spec:nuos:req:emu.powman] */
static MemTxResult rp2350_powman_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned size,
                                       MemTxAttrs attrs)
{
    RP2350PowmanState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr) & ~3;
    bool secure = attrs.secure || attrs.unspecified;
    uint32_t v = value, old, eff;

    /* A narrow write is replicated across the bus and writes the whole. */
    if (size == 1) {
        v = (v & 0xff) * 0x01010101u;
    } else if (size == 2) {
        v = (v & 0xffff) * 0x00010001u;
    }
    if (!secure && !(powman_nonsec_reg(reg) &&
                     (s->timer & TIMER_NONSEC_WRITE))) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: Non-secure write to "
                      "offset 0x%" HWADDR_PRIx " ignored\n", reg);
        return MEMTX_OK;
    }
    old = powman_read_reg(s, reg);
    if (reg <= A_LAST_PROTECTED) {
        if (v >> 16 != POWMAN_PASSWORD) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-powman: write to offset "
                          "0x%" HWADDR_PRIx " without the password\n", reg);
            s->badpasswd = 1;
            return MEMTX_OK;
        }
        v &= 0xffff;
        eff = rp2350_atomic_apply(addr, old & 0xffff, v) & 0xffff;
    } else {
        eff = rp2350_atomic_apply(addr, old, v);
    }
    powman_write_reg(s, reg, eff, v, secure);
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_powman_ops = {
    .read_with_attrs = rp2350_powman_read,
    .write_with_attrs = rp2350_powman_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* GPIO inputs */

/* [spec:nuos:req:emu.powman] */
static void powman_gpio_in(void *opaque, int n, int level)
{
    RP2350PowmanState *s = opaque;
    bool old = s->pin[n];
    int i;

    level = !!level;
    if (old == level) {
        return;
    }
    s->pin[n] = level;
    for (i = 0; i < RP2350_POWMAN_PWRUPS; i++) {
        if ((s->pwrup[i] & (PWRUP_ENABLE | PWRUP_MODE_EDGE)) ==
            (PWRUP_ENABLE | PWRUP_MODE_EDGE) &&
            pwrup_pin(s, i) == n && pwrup_level(s, i)) {
            s->pwrup_latch |= BIT(i);
        }
    }
    if (n == powman_time_ref_pin[s->ext_time_ref & EXT_TIME_REF_SOURCE_SEL] &&
        old && !level) {
        aon_ref_edge(s);
    }
    powman_pwrup_changed(s);
}

/*
 * A chip-level reset that resets the PSM and the watchdog and leaves the
 * power manager and the switched core's power alone: the datasheet's
 * chip-level reset table lists the glitch detector and the watchdog's
 * RESET_PSM with no effect on POWMAN or the power state, and every
 * chip-level reset resets the watchdog, SCRATCH0-7 and REASON included
 * (a watchdog-triggered one too). It records `reason` and moves
 * BOOTDIS.NEXT to NOW, as every reset of the PSM through the power
 * manager does, then runs the PSM's full sequence.
 */
static void powman_psm_reset(RP2350PowmanState *s, uint32_t reason)
{
    s->chip_reset = (s->chip_reset & ~CHIP_RESET_HAD_MASK) | reason;
    if (s->bootdis & BOOTDIS_NEXT) {
        s->bootdis = BOOTDIS_NOW;
    }
    qemu_irq_pulse(s->watchdog_reset);
    qemu_irq_pulse(s->psm_reset);
}

/*
 * The watchdog's reset request. POWMAN acts on it only when WDSEL selects
 * a chip-level reset and the PSM's WDSEL reaches at least the CLOCKS
 * stage; otherwise it is the PSM's to carry out.
 */
/* [spec:nuos:req:emu.powman] */
static void powman_watchdog(void *opaque, int n, int level)
{
    RP2350PowmanState *s = opaque;

    if (!level) {
        return;
    }
    if (!s->wdsel || !(s->psm->wdsel & PSM_THROUGH_CLOCKS)) {
        qemu_irq_pulse(s->psm_watchdog);
        return;
    }
    if (s->wdsel & WDSEL_RESET_POWMAN_ASYNC) {
        powman_chip_reset(s, RESET_WATCHDOG_POWMAN_ASYNC,
                          SHUTDOWN_CAUSE_GUEST_RESET);
    } else if (s->wdsel & WDSEL_RESET_POWMAN) {
        powman_chip_reset(s, RESET_WATCHDOG_POWMAN,
                          SHUTDOWN_CAUSE_GUEST_RESET);
    } else if (s->wdsel & WDSEL_RESET_SWCORE) {
        powman_chip_reset(s, RESET_WATCHDOG_SWCORE,
                          SHUTDOWN_CAUSE_GUEST_RESET);
    } else {
        powman_psm_reset(s, CHIP_RESET_HAD_WATCHDOG_RESET_PSM);
    }
}

/*
 * An armed glitch detector trigger. Like the watchdog's RESET_PSM it runs
 * the full PSM sequence, which resets the processors and, through the
 * RESETS block, every subsystem; it also resets the watchdog, scratch
 * registers included. The switched core stays powered and the power
 * manager, the AON timer and the power state are untouched.
 */
/* [spec:nuos:req:emu.powman] */
static void powman_glitch_reset(void *opaque, int n, int level)
{
    RP2350PowmanState *s = opaque;

    if (level) {
        powman_psm_reset(s, CHIP_RESET_HAD_GLITCH_DETECT);
    }
}

void rp2350_powman_set_ops(RP2350PowmanState *s, const RP2350PowmanOps *ops,
                           void *opaque)
{
    s->ops = ops;
    s->ops_opaque = opaque;
}

bool rp2350_powman_domain_on(RP2350PowmanState *s, int domain)
{
    return !(powman_current(s) & BIT(domain));
}

ResetType rp2350_powman_next_reset_type(RP2350PowmanState *s)
{
    switch (s->next_reset) {
    case RESET_POR:
    case RESET_BOR:
    case RESET_RUN_LOW:
        return RESET_TYPE_COLD;
    default:
        return RESET_TYPE_WAKEUP;
    }
}

/* Everything a reset of the power manager itself restores. */
static void powman_reset_regs(RP2350PowmanState *s)
{
    int i;

    timer_del(s->seq_timer);
    s->badpasswd = 0;
    s->vreg_ctrl = VREG_CTRL_RESET;
    vreg_reset(s);
    s->vreg_lp_entry = VREG_LP_ENTRY_RESET;
    s->vreg_lp_exit = VREG_LP_EXIT_RESET;
    s->bod_ctrl = 0;
    s->bod = BOD_RESET;
    s->bod_lp_entry = BOD_LP_ENTRY_RESET;
    s->bod_lp_exit = BOD_LP_EXIT_RESET;
    s->lposc = LPOSC_RESET;
    s->wdsel = 0;
    s->seq_cfg = SEQ_CFG_RESET;
    /* The power-on sequence brings every domain up: P0.0. */
    s->state = 0;
    s->seq = SEQ_IDLE;
    s->seq_target = 0;
    s->wake_reqs = 0;
    s->pow_fastdiv = POW_FASTDIV_RESET;
    s->pow_delay = POW_DELAY_RESET;
    s->ext_ctrl[0] = EXT_CTRL_RESET;
    s->ext_ctrl[1] = EXT_CTRL_RESET;
    s->ext_time_ref = 0;
    aon_reset(s);
    for (i = 0; i < RP2350_POWMAN_PWRUPS; i++) {
        s->pwrup[i] = PWRUP_RESET;
    }
    s->pwrup_latch = 0;
    s->last_swcore_pwrup = 0;
    s->dbg_pwrcfg = 0;
    s->bootdis = 0;
    s->dbgconfig = 0;
    memset(s->scratch, 0, sizeof(s->scratch));
    memset(s->boot, 0, sizeof(s->boot));
    s->intr = 0;
    s->inte = 0;
    s->intf = 0;
}

/*
 * A system reset is the chip-level reset next_reset names, which decides
 * what of the power manager it resets. Only power-on and brownout resets
 * clear DOUBLE_TAP; the RUN pin also clears RESCUE_FLAG. The switched-core
 * resets leave the power manager and the AON timer alone. Every chip
 * reset that resets the PSM through the power manager moves
 * BOOTDIS.NEXT to NOW.
 */
/* [spec:nuos:req:emu.powman] */
static void rp2350_powman_hold_reset(Object *obj, ResetType type)
{
    RP2350PowmanState *s = RP2350_POWMAN(obj);
    PowmanReset kind = s->next_reset;
    uint32_t keep;

    s->next_reset = RESET_RUN_LOW;
    switch (kind) {
    case RESET_POR:
    case RESET_BOR:
        keep = 0;
        break;
    case RESET_RUN_LOW:
        keep = CHIP_RESET_DOUBLE_TAP;
        break;
    case RESET_WATCHDOG_POWMAN:
    case RESET_WATCHDOG_POWMAN_ASYNC:
        keep = CHIP_RESET_DOUBLE_TAP | CHIP_RESET_RESCUE_FLAG;
        break;
    case RESET_SWCORE_OFF:
        return;
    default:
        if (s->bootdis & BOOTDIS_NEXT) {
            s->bootdis = BOOTDIS_NOW;
        }
        s->chip_reset = (s->chip_reset & ~CHIP_RESET_HAD_MASK) |
                        powman_reset_reason[kind];
        return;
    }
    powman_reset_regs(s);
    s->chip_reset = (s->chip_reset & keep) | powman_reset_reason[kind];
}

static void rp2350_powman_exit_reset(Object *obj, ResetType type)
{
    powman_update_irq(RP2350_POWMAN(obj));
}

static void rp2350_powman_realize(DeviceState *dev, Error **errp)
{
    RP2350PowmanState *s = RP2350_POWMAN(dev);

    if (!s->psm) {
        error_setg(errp, "psm property was not set");
        return;
    }
    if (!s->lposc_hz) {
        error_setg(errp, "lposc-hz must be set");
        return;
    }
    if (s->bonded_gpios > RP2350_GPIO_BANK0_PINS) {
        error_setg(errp, "bonded-gpios must be at most %d",
                   RP2350_GPIO_BANK0_PINS);
        return;
    }
    s->seq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, powman_seq_cb, s);
    s->alarm_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aon_alarm_cb, s);
    s->switch_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aon_switch_cb, s);
    s->vreg_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vreg_cb, s);
}

/*
 * clk_ref changes rate, starts or stops: count up to the change at the
 * old rate.
 */
/* [spec:nuos:req:emu.clock-tree] */
static void powman_clk_ref_changed(void *opaque, ClockEvent event)
{
    RP2350PowmanState *s = opaque;

    if (event == ClockPreUpdate) {
        aon_sync(s);
    } else {
        aon_update(s);
    }
}

static void rp2350_powman_init(Object *obj)
{
    RP2350PowmanState *s = RP2350_POWMAN(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_powman_ops, s,
                          TYPE_RP2350_POWMAN, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[RP2350_POWMAN_IRQ_POW]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[RP2350_POWMAN_IRQ_TIMER]);
    qdev_init_gpio_in_named(dev, powman_watchdog, RP2350_POWMAN_WATCHDOG, 1);
    qdev_init_gpio_in_named(dev, powman_glitch_reset,
                            RP2350_POWMAN_GLITCH_RESET, 1);
    qdev_init_gpio_in_named(dev, powman_gpio_in, RP2350_POWMAN_GPIO,
                            RP2350_GPIO_PINS);
    qdev_init_gpio_out_named(dev, &s->psm_watchdog,
                             RP2350_POWMAN_PSM_WATCHDOG, 1);
    qdev_init_gpio_out_named(dev, &s->psm_reset, RP2350_POWMAN_PSM_RESET, 1);
    qdev_init_gpio_out_named(dev, &s->watchdog_reset,
                             RP2350_POWMAN_WATCHDOG_RESET, 1);
    s->clk_ref = qdev_init_clock_in(dev, "clk-ref", powman_clk_ref_changed, s,
                                    ClockPreUpdate | ClockUpdate);
    qdev_init_gpio_out_named(dev, &s->alarm_wake, RP2350_POWMAN_ALARM_WAKE,
                             1);
    /* The first reset is the power-on reset. */
    s->next_reset = RESET_POR;
}

static const VMStateDescription vmstate_rp2350_powman = {
    .name = TYPE_RP2350_POWMAN,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(seq_timer, RP2350PowmanState),
        VMSTATE_TIMER_PTR(alarm_timer, RP2350PowmanState),
        VMSTATE_TIMER_PTR(switch_timer, RP2350PowmanState),
        VMSTATE_TIMER_PTR(vreg_timer, RP2350PowmanState),
        VMSTATE_UINT32(next_reset, RP2350PowmanState),
        VMSTATE_UINT32(badpasswd, RP2350PowmanState),
        VMSTATE_UINT32(vreg_ctrl, RP2350PowmanState),
        VMSTATE_UINT32(vreg, RP2350PowmanState),
        VMSTATE_UINT32(vreg_lp_entry, RP2350PowmanState),
        VMSTATE_UINT32(vreg_lp_exit, RP2350PowmanState),
        VMSTATE_BOOL(vreg_startup, RP2350PowmanState),
        VMSTATE_BOOL(vout_low, RP2350PowmanState),
        VMSTATE_UINT32(bod_ctrl, RP2350PowmanState),
        VMSTATE_UINT32(bod, RP2350PowmanState),
        VMSTATE_UINT32(bod_lp_entry, RP2350PowmanState),
        VMSTATE_UINT32(bod_lp_exit, RP2350PowmanState),
        VMSTATE_UINT32(lposc, RP2350PowmanState),
        VMSTATE_UINT32(chip_reset, RP2350PowmanState),
        VMSTATE_UINT32(wdsel, RP2350PowmanState),
        VMSTATE_UINT32(seq_cfg, RP2350PowmanState),
        VMSTATE_UINT32(state, RP2350PowmanState),
        VMSTATE_UINT32(seq, RP2350PowmanState),
        VMSTATE_UINT32(seq_target, RP2350PowmanState),
        VMSTATE_UINT32(wake_reqs, RP2350PowmanState),
        VMSTATE_UINT32(pow_fastdiv, RP2350PowmanState),
        VMSTATE_UINT32(pow_delay, RP2350PowmanState),
        VMSTATE_UINT32_ARRAY(ext_ctrl, RP2350PowmanState, 2),
        VMSTATE_UINT32(ext_time_ref, RP2350PowmanState),
        VMSTATE_UINT32(lposc_freq_int, RP2350PowmanState),
        VMSTATE_UINT32(lposc_freq_frac, RP2350PowmanState),
        VMSTATE_UINT32(xosc_freq_int, RP2350PowmanState),
        VMSTATE_UINT32(xosc_freq_frac, RP2350PowmanState),
        VMSTATE_UINT16_ARRAY(set_time, RP2350PowmanState,
                             RP2350_POWMAN_TIME_REGS),
        VMSTATE_UINT16_ARRAY(alarm_time, RP2350PowmanState,
                             RP2350_POWMAN_TIME_REGS),
        VMSTATE_UINT32(timer, RP2350PowmanState),
        VMSTATE_UINT64(aon_base, RP2350PowmanState),
        VMSTATE_INT64(aon_start_ns, RP2350PowmanState),
        VMSTATE_UINT64(aon_sec_limit, RP2350PowmanState),
        VMSTATE_UINT32(aon_lpck_acc, RP2350PowmanState),
        VMSTATE_BOOL(alarm_cmp, RP2350PowmanState),
        VMSTATE_CLOCK(clk_ref, RP2350PowmanState),
        VMSTATE_UINT32_ARRAY(pwrup, RP2350PowmanState, RP2350_POWMAN_PWRUPS),
        VMSTATE_UINT32(pwrup_latch, RP2350PowmanState),
        VMSTATE_UINT32(last_swcore_pwrup, RP2350PowmanState),
        VMSTATE_UINT32(dbg_pwrcfg, RP2350PowmanState),
        VMSTATE_UINT32(bootdis, RP2350PowmanState),
        VMSTATE_UINT32(dbgconfig, RP2350PowmanState),
        VMSTATE_UINT32_ARRAY(scratch, RP2350PowmanState,
                             RP2350_POWMAN_SCRATCH),
        VMSTATE_UINT32_ARRAY(boot, RP2350PowmanState, RP2350_POWMAN_BOOT),
        VMSTATE_UINT32(intr, RP2350PowmanState),
        VMSTATE_UINT32(inte, RP2350PowmanState),
        VMSTATE_UINT32(intf, RP2350PowmanState),
        VMSTATE_UINT8_ARRAY(pin, RP2350PowmanState, RP2350_GPIO_PINS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_powman_properties[] = {
    DEFINE_PROP_LINK("psm", RP2350PowmanState, psm, TYPE_RP2350_PSM,
                     RP2350PSMState *),
    DEFINE_PROP_UINT32("lposc-hz", RP2350PowmanState, lposc_hz, 32768),
    DEFINE_PROP_UINT32("bonded-gpios", RP2350PowmanState, bonded_gpios,
                       RP2350_GPIO_BANK0_PINS),
};

/* [spec:nuos:req:emu.powman] */
static void rp2350_powman_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_powman_realize;
    dc->vmsd = &vmstate_rp2350_powman;
    rc->phases.hold = rp2350_powman_hold_reset;
    rc->phases.exit = rp2350_powman_exit_reset;
    device_class_set_props(dc, rp2350_powman_properties);
}

static const TypeInfo rp2350_powman_info = {
    .name          = TYPE_RP2350_POWMAN,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350PowmanState),
    .instance_init = rp2350_powman_init,
    .class_init    = rp2350_powman_class_init,
};

static void rp2350_powman_register_types(void)
{
    type_register_static(&rp2350_powman_info);
}
type_init(rp2350_powman_register_types)
