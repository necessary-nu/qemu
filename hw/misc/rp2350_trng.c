/*
 * RP2350 true random number generator (TRNG)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "TRNG", and the pico-sdk trng.h register
 * descriptions. The block is Arm's TrustZone TRNG: it samples its own
 * free-running ring oscillator every SAMPLE_CNT1 rng_clk (clk_sys, the
 * "clk" clock, at its current frequency) cycles,
 * passes the samples through the von Neumann decorrelator and the CRNGT
 * check, and collects 192 bits into the Entropy Holding Register (EHR).
 *
 * The ring oscillator's samples come from the host's random source, so
 * they honour -seed. The decorrelator and CRNGT run on them as on
 * hardware, so collection takes as many samples as the data needs and
 * the error checks fire as rarely as they do on hardware. Each
 * collection is drawn whole when it starts and completes in virtual time
 * after its sample count's worth of SAMPLE_CNT1 periods.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/guest-random.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_trng.h"
#include "migration/vmstate.h"

#define A_RNG_IMR               0x100
#define A_RNG_ISR               0x104
#define A_RNG_ICR               0x108
#define A_TRNG_CONFIG           0x10c
#define A_TRNG_VALID            0x110
#define A_EHR_DATA0             0x114
#define A_EHR_DATA5             0x128
#define A_RND_SOURCE_ENABLE     0x12c
#define A_SAMPLE_CNT1           0x130
#define A_AUTOCORR_STATISTIC    0x134
#define A_TRNG_DEBUG_CONTROL    0x138
#define A_TRNG_SW_RESET         0x140
#define A_RNG_DEBUG_EN_INPUT    0x1b4
#define A_TRNG_BUSY             0x1b8
#define A_RST_BITS_COUNTER      0x1bc
#define A_RNG_VERSION           0x1c0
#define A_RNG_BIST_CNTR_0       0x1e0
#define A_RNG_BIST_CNTR_2       0x1e8

/* RNG_IMR, RNG_ISR and RNG_ICR bits. */
#define INT_EHR_VALID           (1u << 0)
#define INT_AUTOCORR_ERR        (1u << 1)
#define INT_CRNGT_ERR           (1u << 2)
#define INT_VN_ERR              (1u << 3)
#define INT_ALL                 0xfu
/* Only a reset of the RNG clears AUTOCORR_ERR. */
#define ICR_CLEARABLE           (INT_EHR_VALID | INT_CRNGT_ERR | INT_VN_ERR)

#define CONFIG_MASK             0x3u
#define DEBUG_CONTROL_MASK      0xeu
#define DEBUG_VNC_BYPASS        (1u << 1)
#define DEBUG_CRNGT_BYPASS      (1u << 2)
#define DEBUG_AUTOCORR_BYPASS   (1u << 3)
#define SAMPLE_CNT1_RESET       0xffffu

#define EHR_BITS                (RP2350_TRNG_EHR_WORDS * 32)
/* The von Neumann check fails on this many identical raw samples. */
#define VN_RUN_LIMIT            32
/* CRNGT compares consecutive blocks of this many collected bits. */
#define CRNGT_BLOCK_BITS        16

/* How a collection that is under way ends. */
enum {
    TRNG_EVENT_EHR_VALID,
    TRNG_EVENT_VN_ERR,
    TRNG_EVENT_CRNGT_ERR,
};

static void trng_update_irq(RP2350TRNGState *s)
{
    qemu_set_irq(s->irq, !!(s->isr & ~s->imr & INT_ALL));
}

static unsigned trng_sample_bit(RP2350TRNGState *s)
{
    unsigned bit;

    if (!s->entropy_bits) {
        qemu_guest_getrandom_nofail(&s->entropy, sizeof(s->entropy));
        s->entropy_bits = 32;
    }
    bit = s->entropy & 1;
    s->entropy >>= 1;
    s->entropy_bits--;
    return bit;
}

/*
 * Draw a whole collection from fresh: sample the ring oscillator until
 * the EHR fills or an entropy check fails, recording which happened and
 * after how many samples.
 */
static void trng_draw_collection(RP2350TRNGState *s)
{
    bool vnc = !(s->debug_control & DEBUG_VNC_BYPASS);
    bool crngt = !(s->debug_control & DEBUG_CRNGT_BYPASS);
    unsigned collected = 0, run = 0;
    int last = -1, first = -1;
    uint32_t block = 0, prev_block = 0;
    bool have_prev_block = false;
    uint64_t samples = 0;

    memset(s->pending_ehr, 0, sizeof(s->pending_ehr));
    for (;;) {
        unsigned raw = trng_sample_bit(s);
        int out = -1;

        samples++;
        if (vnc) {
            run = (int)raw == last ? run + 1 : 1;
            last = raw;
            if (run >= VN_RUN_LIMIT) {
                s->pending_event = TRNG_EVENT_VN_ERR;
                break;
            }
            /* Unequal pairs give their first bit; equal pairs nothing. */
            if (first < 0) {
                first = raw;
            } else {
                if (first != (int)raw) {
                    out = first;
                }
                first = -1;
            }
        } else {
            out = raw;
        }
        if (out < 0) {
            continue;
        }

        s->pending_ehr[collected / 32] |= (uint32_t)out << (collected % 32);
        block = (block << 1) | out;
        collected++;
        if (collected % CRNGT_BLOCK_BITS == 0) {
            block &= (1u << CRNGT_BLOCK_BITS) - 1;
            if (crngt && have_prev_block && block == prev_block) {
                s->pending_event = TRNG_EVENT_CRNGT_ERR;
                break;
            }
            prev_block = block;
            have_prev_block = true;
            block = 0;
        }
        if (collected == EHR_BITS) {
            s->pending_event = TRNG_EVENT_EHR_VALID;
            break;
        }
    }
    s->samples_left = samples;
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* rng_clk cycles between samples; a count of 0 samples every cycle. */
static uint64_t trng_sample_period(RP2350TRNGState *s)
{
    return MAX(s->sample_cnt1, 1);
}

/* [spec:nuos:req:emu.clock-tree] */
static bool trng_sampling(RP2350TRNGState *s)
{
    return s->collecting && (s->rnd_source_enable & 1) &&
           clock_is_enabled(s->clk);
}

/* Virtual time taken by `cycles` rng_clk cycles, rounded up. */
static int64_t trng_cycles_ns(RP2350TRNGState *s, uint64_t cycles)
{
    uint64_t ns = clock_ticks_to_ns(s->clk, cycles);

    while (clock_ns_to_ticks(s->clk, ns) < cycles) {
        ns++;
    }
    return MIN(ns, (uint64_t)INT64_MAX / 2);
}

/*
 * Account for the samples taken since sync_ns. sync_ns advances only by
 * whole samples, so a partly elapsed sample period is not lost.
 */
static void trng_sync(RP2350TRNGState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t period = trng_sample_period(s);
    uint64_t taken;

    if (!trng_sampling(s)) {
        s->sync_ns = now;
        return;
    }
    taken = clock_ns_to_ticks(s->clk, MAX(now - s->sync_ns, 0)) / period;
    taken = MIN(taken, s->samples_left);
    s->samples_left -= taken;
    s->sync_ns += trng_cycles_ns(s, taken * period);
}

static void trng_schedule(RP2350TRNGState *s)
{
    if (trng_sampling(s)) {
        uint64_t cycles = s->samples_left * trng_sample_period(s);

        timer_mod(s->timer, s->sync_ns + trng_cycles_ns(s, cycles));
    } else {
        timer_del(s->timer);
    }
}

/*
 * Collection runs while the source is enabled and the EHR has no unread
 * result; AUTOCORR_ERR would stop it until reset, but the autocorrelation
 * test never fails here.
 */
static void trng_maybe_start(RP2350TRNGState *s)
{
    if ((s->rnd_source_enable & 1) && !s->ehr_valid && !s->collecting &&
        !(s->isr & INT_AUTOCORR_ERR)) {
        s->collecting = true;
        trng_draw_collection(s);
        trng_schedule(s);
    }
}

/* [spec:nuos:req:emu.rosc-trng] */
static void trng_collection_done(void *opaque)
{
    RP2350TRNGState *s = opaque;

    trng_sync(s);
    if (!trng_sampling(s) || s->samples_left) {
        trng_schedule(s);
        return;
    }
    switch (s->pending_event) {
    case TRNG_EVENT_EHR_VALID:
        memcpy(s->ehr, s->pending_ehr, sizeof(s->ehr));
        s->ehr_valid = true;
        s->isr |= INT_EHR_VALID;
        s->collecting = false;
        break;
    case TRNG_EVENT_VN_ERR:
    case TRNG_EVENT_CRNGT_ERR:
        /* The failed bits are discarded and collection starts again. */
        s->isr |= s->pending_event == TRNG_EVENT_VN_ERR ? INT_VN_ERR
                                                        : INT_CRNGT_ERR;
        trng_draw_collection(s);
        trng_schedule(s);
        break;
    }
    trng_update_irq(s);
}

static void trng_reset_state(RP2350TRNGState *s)
{
    timer_del(s->timer);
    s->imr = INT_ALL;
    s->isr = 0;
    s->config = 0;
    s->rnd_source_enable = 0;
    s->sample_cnt1 = SAMPLE_CNT1_RESET;
    s->debug_control = 0;
    s->debug_en = 0;
    s->ehr_valid = false;
    memset(s->ehr, 0, sizeof(s->ehr));
    s->collecting = false;
    s->pending_event = TRNG_EVENT_EHR_VALID;
    memset(s->pending_ehr, 0, sizeof(s->pending_ehr));
    s->samples_left = 0;
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->entropy = 0;
    s->entropy_bits = 0;
}

/* [spec:nuos:req:emu.rosc-trng] */
static uint64_t rp2350_trng_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350TRNGState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t v;

    switch (reg) {
    case A_RNG_IMR:
        return s->imr;
    case A_RNG_ISR:
        return s->isr;
    case A_RNG_ICR:
        return 0;
    case A_TRNG_CONFIG:
        return s->config;
    case A_TRNG_VALID:
        return s->ehr_valid;
    case A_EHR_DATA0 ... A_EHR_DATA5:
        /* The EHR reads 0 until a collection completes. */
        if (!s->ehr_valid) {
            return 0;
        }
        v = s->ehr[(reg - A_EHR_DATA0) / 4];
        if (reg == A_EHR_DATA5) {
            /* Reading the last word empties the EHR and collects again. */
            s->ehr_valid = false;
            memset(s->ehr, 0, sizeof(s->ehr));
            trng_maybe_start(s);
        }
        return v;
    case A_RND_SOURCE_ENABLE:
        return s->rnd_source_enable;
    case A_SAMPLE_CNT1:
        return s->sample_cnt1;
    case A_AUTOCORR_STATISTIC:
        /*
         * The autocorrelation test is not modelled (it never runs, so it
         * never fails), and its statistics stay 0.
         */
        return 0;
    case A_TRNG_DEBUG_CONTROL:
        return s->debug_control;
    case A_TRNG_SW_RESET:
        return 0;
    case A_RNG_DEBUG_EN_INPUT:
        return s->debug_en;
    case A_TRNG_BUSY:
        return trng_sampling(s);
    case A_RST_BITS_COUNTER:
        return 0;
    case A_RNG_VERSION:
        /* The RP2350's instance reads 0 here, per its register list. */
        return 0;
    case A_RNG_BIST_CNTR_0:
    case A_RNG_BIST_CNTR_0 + 4:
    case A_RNG_BIST_CNTR_2:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of bad offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return 0;
    }
}

/* [spec:nuos:req:emu.rosc-trng] */
static void rp2350_trng_write(void *opaque, hwaddr addr, uint64_t value64,
                              unsigned size)
{
    RP2350TRNGState *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr);
    uint32_t value = value64;

    switch (reg) {
    case A_RNG_IMR:
        s->imr = rp2350_atomic_apply(addr, s->imr, value) & INT_ALL;
        break;
    case A_RNG_ICR:
        /* Write 1 to clear, through any alias. */
        s->isr &= ~(value & ICR_CLEARABLE);
        break;
    case A_TRNG_CONFIG:
        /*
         * The chain length changes the ring oscillator's frequency, not
         * the sampling rate, so collection timing does not depend on it.
         */
        s->config = rp2350_atomic_apply(addr, s->config, value) & CONFIG_MASK;
        break;
    case A_RND_SOURCE_ENABLE:
        trng_sync(s);
        s->rnd_source_enable =
            rp2350_atomic_apply(addr, s->rnd_source_enable, value) & 1;
        trng_maybe_start(s);
        trng_schedule(s);
        break;
    case A_SAMPLE_CNT1:
        trng_sync(s);
        s->sample_cnt1 = rp2350_atomic_apply(addr, s->sample_cnt1, value);
        trng_schedule(s);
        break;
    case A_AUTOCORR_STATISTIC:
        /* Any write resets the counters. */
        break;
    case A_TRNG_DEBUG_CONTROL:
        /* Applies from the next collection drawn. */
        s->debug_control = rp2350_atomic_apply(addr, s->debug_control,
                                               value) & DEBUG_CONTROL_MASK;
        break;
    case A_TRNG_SW_RESET:
        if (rp2350_atomic_apply(addr, 0, value) & 1) {
            trng_reset_state(s);
        }
        break;
    case A_RNG_DEBUG_EN_INPUT:
        s->debug_en = rp2350_atomic_apply(addr, s->debug_en, value) & 1;
        if (s->debug_en) {
            qemu_log_mask(LOG_UNIMP, "%s: RNG debug mode not modelled\n",
                          __func__);
        }
        break;
    case A_RST_BITS_COUNTER:
        /* Takes effect only while the source is disabled. */
        if (!(s->rnd_source_enable & 1)) {
            s->collecting = false;
            s->ehr_valid = false;
            memset(s->ehr, 0, sizeof(s->ehr));
            trng_schedule(s);
        }
        break;
    case A_RNG_ISR:
    case A_TRNG_VALID:
    case A_EHR_DATA0 ... A_EHR_DATA5:
    case A_TRNG_BUSY:
    case A_RNG_VERSION:
    case A_RNG_BIST_CNTR_0:
    case A_RNG_BIST_CNTR_0 + 4:
    case A_RNG_BIST_CNTR_2:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to bad offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    trng_update_irq(s);
}

static const MemoryRegionOps rp2350_trng_ops = {
    .read = rp2350_trng_read,
    .write = rp2350_trng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_trng_hold_reset(Object *obj, ResetType type)
{
    RP2350TRNGState *s = RP2350_TRNG(obj);

    trng_reset_state(s);
    trng_update_irq(s);
}

/*
 * rng_clk changes rate, starts or stops: count the samples taken at the
 * old rate, and the next sample afresh at the new one.
 */
static void trng_clk_changed(void *opaque, ClockEvent event)
{
    RP2350TRNGState *s = opaque;

    if (event == ClockPreUpdate) {
        trng_sync(s);
        return;
    }
    s->sync_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    trng_schedule(s);
}

static void rp2350_trng_init(Object *obj)
{
    RP2350TRNGState *s = RP2350_TRNG(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_trng_ops, s,
                          TYPE_RP2350_TRNG, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, trng_collection_done, s);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", trng_clk_changed, s,
                                ClockPreUpdate | ClockUpdate);
}

static void rp2350_trng_realize(DeviceState *dev, Error **errp)
{
    RP2350TRNGState *s = RP2350_TRNG(dev);

    if (!clock_has_source(s->clk)) {
        error_setg(errp, "the clk clock must be connected");
    }
}

static const VMStateDescription vmstate_rp2350_trng = {
    .name = TYPE_RP2350_TRNG,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, RP2350TRNGState),
        VMSTATE_TIMER_PTR(timer, RP2350TRNGState),
        VMSTATE_UINT32(imr, RP2350TRNGState),
        VMSTATE_UINT32(isr, RP2350TRNGState),
        VMSTATE_UINT32(config, RP2350TRNGState),
        VMSTATE_UINT32(rnd_source_enable, RP2350TRNGState),
        VMSTATE_UINT32(sample_cnt1, RP2350TRNGState),
        VMSTATE_UINT32(debug_control, RP2350TRNGState),
        VMSTATE_UINT32(debug_en, RP2350TRNGState),
        VMSTATE_BOOL(ehr_valid, RP2350TRNGState),
        VMSTATE_UINT32_ARRAY(ehr, RP2350TRNGState, RP2350_TRNG_EHR_WORDS),
        VMSTATE_BOOL(collecting, RP2350TRNGState),
        VMSTATE_UINT32(pending_event, RP2350TRNGState),
        VMSTATE_UINT32_ARRAY(pending_ehr, RP2350TRNGState,
                             RP2350_TRNG_EHR_WORDS),
        VMSTATE_UINT64(samples_left, RP2350TRNGState),
        VMSTATE_INT64(sync_ns, RP2350TRNGState),
        VMSTATE_UINT32(entropy, RP2350TRNGState),
        VMSTATE_UINT32(entropy_bits, RP2350TRNGState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_trng_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = rp2350_trng_realize;
    rc->phases.hold = rp2350_trng_hold_reset;
    dc->vmsd = &vmstate_rp2350_trng;
}

/* [spec:nuos:req:emu.rosc-trng] */
static const TypeInfo rp2350_trng_info = {
    .name          = TYPE_RP2350_TRNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350TRNGState),
    .instance_init = rp2350_trng_init,
    .class_init    = rp2350_trng_class_init,
};

static void rp2350_trng_register_types(void)
{
    type_register_static(&rp2350_trng_info);
}

type_init(rp2350_trng_register_types)
