/*
 * RP2350 ring oscillator (ROSC)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Ring oscillator (ROSC)".
 *
 * The ring's frequency follows its configuration approximately, since on
 * hardware it varies with process, voltage and temperature:
 *
 *  - Each stage in the loop contributes a delay. With no extra drive a
 *    stage's delay is 1; extra drives shorten it with diminishing effect,
 *    to 0.53, 0.36 and 0.28. Those figures make randomising both of the
 *    first two stages (each driven by an LFSR across all four strengths)
 *    raise the LOW range by up to 22%, as the datasheet gives; the model
 *    uses the LFSR's average, a 13% rise.
 *  - FREQ_RANGE selects stages 0-7, 0-5, 0-3 or 0-1, so MEDIUM, HIGH and
 *    TOOHIGH run 1.33, 2 and 4 times as fast as LOW.
 *  - The full LOW ring at default drive runs at 88 MHz, so the divided
 *    output at the reset divisor of 8 is the datasheet's nominal 11 MHz
 *    (12.4 MHz with the reset value's randomisation applied).
 *
 * COUNT counts down at the divided output frequency, the frequency the
 * datasheet calls the ROSC's. RANDOMBIT samples the host's random source
 * while the oscillator runs.
 *
 * Writing the DORMANT keyword to DORMANT stops the oscillator until a
 * DORMANT wake event (the GPIO banks' dormant_wake interrupt or the AON
 * timer alarm, on the "dormant-wake" input). It then restarts in the same
 * configuration, and its output is ungated once it is stable, about 1us
 * later. The "dormant" output is high from entry until then, while the
 * SoC stops every clock running from the ROSC.
 */

#include "qemu/osdep.h"
#include "qemu/guest-random.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_rosc.h"
#include "migration/vmstate.h"

#define A_CTRL      0x00
#define A_FREQA     0x04
#define A_FREQB     0x08
#define A_RANDOM    0x0c
#define A_DORMANT   0x10
#define A_DIV       0x14
#define A_PHASE     0x18
#define A_STATUS    0x1c
#define A_RANDOMBIT 0x20
#define A_COUNT     0x24

#define CTRL_MASK           0x00ffffffu
#define CTRL_ENABLE_SHIFT   12
#define CTRL_ENABLE_MASK    0xfffu
#define CTRL_RANGE_MASK     0xfffu
#define ENABLE_DISABLE      0xd1eu
#define ENABLE_ENABLE       0xfabu
#define RANGE_LOW           0xfa4u
#define RANGE_MEDIUM        0xfa5u
#define RANGE_HIGH          0xfa7u
#define RANGE_TOOHIGH       0xfa6u
#define RANGE_RESET         0xaa0u

#define FREQA_MASK          0xffff77ffu
#define FREQB_MASK          0xffff7777u
#define FREQ_PASSWD_SHIFT   16
#define FREQ_PASSWD         0x9696u
#define FREQA_DS0_RANDOM    (1u << 3)
#define FREQA_DS1_RANDOM    (1u << 7)
#define FREQA_RESET         0x00000088u

#define RANDOM_RESET        0x3f04b16du

#define DORMANT_DORMANT     0x636f6d61u
#define DORMANT_WAKE        0x77616b65u

#define DIV_MASK            0xffffu
#define DIV_PASS            0xaa00u
#define DIV_RESET           (DIV_PASS + 8)

#define PHASE_MASK          0xfffu
#define PHASE_PASSWD_SHIFT  4
#define PHASE_PASSWD_MASK   0xffu
#define PHASE_PASSWD        0xaau
#define PHASE_RESET         0x8u

#define STATUS_ENABLED      (1u << 12)
#define STATUS_DIV_RUNNING  (1u << 16)
#define STATUS_BADWRITE     (1u << 24)
#define STATUS_STABLE       (1u << 31)

#define COUNT_MASK          0xffffu

/* The datasheet gives about 1us for the ROSC to start. */
#define ROSC_STARTUP_NS     1000

#define ROSC_STAGES         8
/* The full LOW-range ring at default drive strength. */
#define ROSC_RING_HZ        88000000ull

/* Stage delay in thousandths, by number of extra drives (bits set). */
static const unsigned rosc_stage_delay[4] = { 1000, 530, 360, 280 };
/* The average over the four strengths an LFSR-randomised stage takes. */
#define ROSC_RANDOM_DELAY   543

static bool rosc_ctrl_enabled(uint32_t ctrl)
{
    /* Any ENABLE code other than DISABLE enables the oscillator. */
    return ((ctrl >> CTRL_ENABLE_SHIFT) & CTRL_ENABLE_MASK) != ENABLE_DISABLE;
}

static bool rosc_running(RP2350ROSCState *s)
{
    return rosc_ctrl_enabled(s->ctrl) && !s->dormant_stopped;
}

static unsigned rosc_stages_in_loop(uint32_t ctrl)
{
    switch (ctrl & CTRL_RANGE_MASK) {
    case RANGE_MEDIUM:
        return 6;
    case RANGE_HIGH:
        return 4;
    case RANGE_TOOHIGH:
        return 2;
    default:
        /* LOW, the reset code and invalid codes all use the full ring. */
        return 8;
    }
}

static uint64_t rosc_ring_hz(RP2350ROSCState *s)
{
    unsigned stages = rosc_stages_in_loop(s->ctrl);
    unsigned delay = 0;
    unsigned i;

    for (i = 0; i < stages; i++) {
        uint32_t reg = i < 4 ? s->freqa_applied : s->freqb_applied;
        unsigned ds = (reg >> ((i % 4) * 4)) & 0x7;

        if ((i == 0 && (s->freqa_applied & FREQA_DS0_RANDOM)) ||
            (i == 1 && (s->freqa_applied & FREQA_DS1_RANDOM))) {
            delay += ROSC_RANDOM_DELAY;
        } else {
            delay += rosc_stage_delay[ctpop32(ds)];
        }
    }
    return ROSC_RING_HZ * ROSC_STAGES * 1000 / delay;
}

static unsigned rosc_divisor(uint32_t div)
{
    if ((div & 0xff00) == DIV_PASS && (div & 0xff)) {
        return div & 0xff;
    }
    /* div = 0 and every invalid value divide by 256. */
    return 256;
}

static uint32_t rosc_output_hz(RP2350ROSCState *s)
{
    return rosc_ring_hz(s) / rosc_divisor(s->div);
}

static uint32_t rosc_count_now(RP2350ROSCState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ticks;

    if (!rosc_running(s) || !s->count) {
        return s->count;
    }
    ticks = muldiv64(now - s->count_ns, rosc_output_hz(s),
                     NANOSECONDS_PER_SECOND);
    return ticks >= s->count ? 0 : s->count - ticks;
}

/*
 * Bring COUNT up to date before anything that changes its rate or stops
 * the oscillator.
 */
static void rosc_count_sync(RP2350ROSCState *s)
{
    s->count = rosc_count_now(s);
    s->count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint32_t rosc_randombit(RP2350ROSCState *s)
{
    /*
     * A stopped oscillator's output holds its last level, so reads return
     * the same bit.
     */
    if (!rosc_running(s)) {
        return s->randombit;
    }
    if (!s->entropy_bits) {
        qemu_guest_getrandom_nofail(&s->entropy, sizeof(s->entropy));
        s->entropy_bits = 32;
    }
    s->randombit = s->entropy & 1;
    s->entropy >>= 1;
    s->entropy_bits--;
    return s->randombit;
}

static uint32_t rosc_status(RP2350ROSCState *s)
{
    uint32_t st = 0;

    if (rosc_ctrl_enabled(s->ctrl)) {
        st |= STATUS_ENABLED;
    }
    if (rosc_running(s)) {
        st |= STATUS_DIV_RUNNING;
        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->stable_ns) {
            st |= STATUS_STABLE;
        }
    }
    if (s->badwrite) {
        st |= STATUS_BADWRITE;
    }
    return st;
}

bool rp2350_rosc_dormant(RP2350ROSCState *s)
{
    return s->gated;
}

static void rosc_set_gated(RP2350ROSCState *s, bool gated)
{
    s->gated = gated;
    qemu_set_irq(s->dormant_irq, gated);
}

/*
 * A wake event restarts the oscillator in the configuration it stopped
 * in; its output is ungated once it is stable. A disabled oscillator has
 * nothing to restart.
 */
/* [spec:nuos:req:emu.rosc-trng] */
static void rosc_wake(RP2350ROSCState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->dormant_stopped = false;
    s->dormant = DORMANT_WAKE;
    s->count_ns = now;
    if (!rosc_ctrl_enabled(s->ctrl)) {
        rosc_set_gated(s, false);
        return;
    }
    s->stable_ns = now + ROSC_STARTUP_NS;
    timer_mod(s->startup_timer, s->stable_ns);
}

static void rosc_startup_cb(void *opaque)
{
    RP2350ROSCState *s = opaque;

    if (!s->dormant_stopped) {
        rosc_set_gated(s, false);
    }
}

/* [spec:nuos:req:emu.rosc-trng] */
static void rosc_enter_dormant(RP2350ROSCState *s)
{
    if (s->dormant_stopped) {
        return;
    }
    rosc_count_sync(s);
    timer_del(s->startup_timer);
    s->dormant_stopped = true;
    s->dormant = DORMANT_DORMANT;
    rosc_set_gated(s, true);
    /* A wake event already asserted restarts the oscillator at once. */
    if (s->wake) {
        rosc_wake(s);
    }
}

static void rosc_set_wake(void *opaque, int n, int level)
{
    RP2350ROSCState *s = opaque;

    s->wake = level;
    if (level && s->dormant_stopped) {
        rosc_wake(s);
    }
}

/* [spec:nuos:req:emu.rosc-trng] */
static uint64_t rp2350_rosc_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350ROSCState *s = opaque;

    switch (rp2350_atomic_reg(addr)) {
    case A_CTRL:
        return s->ctrl;
    case A_FREQA:
        return s->freqa;
    case A_FREQB:
        return s->freqb;
    case A_RANDOM:
        return s->random;
    case A_DORMANT:
        return s->dormant;
    case A_DIV:
        return s->div;
    case A_PHASE:
        return s->phase;
    case A_STATUS:
        return rosc_status(s);
    case A_RANDOMBIT:
        return rosc_randombit(s);
    case A_COUNT:
        return rosc_count_now(s);
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of bad offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return 0;
    }
}

static void rosc_write_ctrl(RP2350ROSCState *s, uint32_t value)
{
    uint32_t enable = (value >> CTRL_ENABLE_SHIFT) & CTRL_ENABLE_MASK;
    uint32_t range = value & CTRL_RANGE_MASK;
    bool was_running = rosc_running(s);

    if (enable != ENABLE_ENABLE && enable != ENABLE_DISABLE) {
        s->badwrite = true;
    }
    if (range != RANGE_LOW && range != RANGE_MEDIUM && range != RANGE_HIGH &&
        range != RANGE_TOOHIGH) {
        s->badwrite = true;
    }
    rosc_count_sync(s);
    s->ctrl = value & CTRL_MASK;
    if (!was_running && rosc_running(s)) {
        s->stable_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                       ROSC_STARTUP_NS;
        s->count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    }
}

static uint32_t rosc_write_freq(RP2350ROSCState *s, uint32_t value,
                                uint32_t mask)
{
    rosc_count_sync(s);
    if ((value >> FREQ_PASSWD_SHIFT) == FREQ_PASSWD) {
        return value & mask & 0xffff;
    }
    /* Any other password sets every drive strength to 0. */
    s->badwrite = true;
    return 0;
}

/* [spec:nuos:req:emu.rosc-trng] */
static void rp2350_rosc_write(void *opaque, hwaddr addr, uint64_t value64,
                              unsigned size)
{
    RP2350ROSCState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t value = value64;

    switch (reg) {
    case A_CTRL:
        rosc_write_ctrl(s, rp2350_atomic_apply(addr, s->ctrl, value));
        break;
    case A_FREQA:
        s->freqa = rp2350_atomic_apply(addr, s->freqa, value) & FREQA_MASK;
        s->freqa_applied = rosc_write_freq(s, s->freqa, FREQA_MASK);
        break;
    case A_FREQB:
        s->freqb = rp2350_atomic_apply(addr, s->freqb, value) & FREQB_MASK;
        s->freqb_applied = rosc_write_freq(s, s->freqb, FREQB_MASK);
        break;
    case A_RANDOM:
        /* Seeds and restarts the LFSR that randomises stages 0 and 1. */
        s->random = rp2350_atomic_apply(addr, s->random, value);
        break;
    case A_DORMANT:
        value = rp2350_atomic_apply(addr, s->dormant, value);
        if (value == DORMANT_DORMANT) {
            rosc_enter_dormant(s);
        } else if (value != DORMANT_WAKE) {
            /* An invalid write selects WAKE, which the running ROSC is. */
            s->badwrite = true;
        }
        break;
    case A_DIV:
        value = rp2350_atomic_apply(addr, s->div, value) & DIV_MASK;
        if ((value & 0xff00) != DIV_PASS) {
            s->badwrite = true;
        }
        rosc_count_sync(s);
        s->div = value;
        break;
    case A_PHASE:
        value = rp2350_atomic_apply(addr, s->phase, value) & PHASE_MASK;
        /*
         * Any other password enables the phase-shifted output with no
         * shift. The phase-shifted output only feeds clock muxes, whose
         * clocks are not modelled.
         */
        if (((value >> PHASE_PASSWD_SHIFT) & PHASE_PASSWD_MASK) !=
            PHASE_PASSWD) {
            s->badwrite = true;
        }
        s->phase = value;
        break;
    case A_STATUS:
        /* BADWRITE is write-1-to-clear through any alias. */
        if (value & STATUS_BADWRITE) {
            s->badwrite = false;
        }
        break;
    case A_RANDOMBIT:
        break;
    case A_COUNT:
        value = rp2350_atomic_apply(addr, rosc_count_now(s), value);
        s->count = value & COUNT_MASK;
        s->count_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to bad offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
}

static const MemoryRegionOps rp2350_rosc_ops = {
    .read = rp2350_rosc_read,
    .write = rp2350_rosc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_rosc_hold_reset(Object *obj, ResetType type)
{
    RP2350ROSCState *s = RP2350_ROSC(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /*
     * The oscillator starts during chip power-up, before software runs,
     * so it is enabled and stable from reset.
     */
    s->ctrl = (ENABLE_ENABLE << CTRL_ENABLE_SHIFT) | RANGE_RESET;
    s->freqa = FREQA_RESET;
    s->freqa_applied = FREQA_RESET;
    s->freqb = 0;
    s->freqb_applied = 0;
    s->random = RANDOM_RESET;
    s->dormant = DORMANT_WAKE;
    s->div = DIV_RESET;
    s->phase = PHASE_RESET;
    s->badwrite = false;
    s->stable_ns = now;
    s->count = 0;
    s->count_ns = now;
    s->entropy = 0;
    s->entropy_bits = 0;
    s->randombit = 1;
    timer_del(s->startup_timer);
    s->dormant_stopped = false;
    s->gated = false;
}

static void rp2350_rosc_exit_reset(Object *obj, ResetType type)
{
    RP2350ROSCState *s = RP2350_ROSC(obj);

    qemu_set_irq(s->dormant_irq, s->gated);
}

static void rp2350_rosc_init(Object *obj)
{
    RP2350ROSCState *s = RP2350_ROSC(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_rosc_ops, s,
                          TYPE_RP2350_ROSC, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    s->startup_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rosc_startup_cb, s);
    qdev_init_gpio_in_named(DEVICE(obj), rosc_set_wake,
                            RP2350_OSC_DORMANT_WAKE, 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dormant_irq,
                             RP2350_OSC_DORMANT, 1);
}

static const VMStateDescription vmstate_rp2350_rosc = {
    .name = TYPE_RP2350_ROSC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, RP2350ROSCState),
        VMSTATE_UINT32(freqa, RP2350ROSCState),
        VMSTATE_UINT32(freqb, RP2350ROSCState),
        VMSTATE_UINT32(random, RP2350ROSCState),
        VMSTATE_UINT32(dormant, RP2350ROSCState),
        VMSTATE_UINT32(div, RP2350ROSCState),
        VMSTATE_UINT32(phase, RP2350ROSCState),
        VMSTATE_BOOL(badwrite, RP2350ROSCState),
        VMSTATE_UINT32(freqa_applied, RP2350ROSCState),
        VMSTATE_UINT32(freqb_applied, RP2350ROSCState),
        VMSTATE_INT64(stable_ns, RP2350ROSCState),
        VMSTATE_UINT32(count, RP2350ROSCState),
        VMSTATE_INT64(count_ns, RP2350ROSCState),
        VMSTATE_UINT32(entropy, RP2350ROSCState),
        VMSTATE_UINT32(entropy_bits, RP2350ROSCState),
        VMSTATE_UINT32(randombit, RP2350ROSCState),
        VMSTATE_TIMER_PTR(startup_timer, RP2350ROSCState),
        VMSTATE_BOOL(dormant_stopped, RP2350ROSCState),
        VMSTATE_BOOL(gated, RP2350ROSCState),
        VMSTATE_BOOL(wake, RP2350ROSCState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_rosc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_rosc_hold_reset;
    rc->phases.exit = rp2350_rosc_exit_reset;
    dc->vmsd = &vmstate_rp2350_rosc;
}

/* [spec:nuos:req:emu.rosc-trng] */
static const TypeInfo rp2350_rosc_info = {
    .name          = TYPE_RP2350_ROSC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350ROSCState),
    .instance_init = rp2350_rosc_init,
    .class_init    = rp2350_rosc_class_init,
};

static void rp2350_rosc_register_types(void)
{
    type_register_static(&rp2350_rosc_info);
}

type_init(rp2350_rosc_register_types)
