/*
 * RP2350 redundancy coprocessor (RCP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Redundancy coprocessor (RCP)". Each core
 * has an RCP as coprocessor 7. The pseudorandom instruction delays are not
 * modelled as time, but delayed (non-"2") Secure instructions still
 * advance the PRNG as on hardware.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_rcp.h"
#include "migration/vmstate.h"
#include "target/arm/cpu.h"

#define RCP_TRUE      0xa500a500
#define RCP_FALSE     0x00c300c3
#define RCP_IVALID    0x96009600

/* Instruction forms, from the T32 coprocessor encodings. */
static bool is_mcr(uint32_t insn)
{
    return (insn & 0x0f100010) == 0x0e000010;
}

static bool is_mrc(uint32_t insn)
{
    return (insn & 0x0f100010) == 0x0e100010;
}

static bool is_cdp(uint32_t insn)
{
    return (insn & 0x0f000010) == 0x0e000000;
}

static bool is_mcrr(uint32_t insn)
{
    return (insn & 0x0ff00000) == 0x0c400000;
}

/* The "2" forms (bit 28 set) skip the pseudorandom delay. */
static bool is_delayed(uint32_t insn)
{
    return !extract32(insn, 28, 1);
}

static uint32_t canary(uint64_t salt, uint8_t tag)
{
    uint32_t b1 = (salt & 0xff) ^ ((salt >> 24) & tag);
    uint32_t b2 = ((salt >> 8) & 0xff) ^ ((salt >> 32) & (uint8_t)~tag);
    uint32_t b3 = ((salt >> 16) & 0xff) ^ tag;

    return (b3 << 24) | (b2 << 16) | ((b1 & 0xff) << 8);
}

/* One LFSR field of the 24-bit PRNG state: width, position, taps. */
static uint32_t lfsr_step(uint32_t state, int lsb, int width, uint32_t taps)
{
    uint32_t mask = MAKE_64BIT_MASK(0, width);
    uint32_t f = (state >> lsb) & mask;

    if (f == 0) {
        f = 1;
    } else {
        f = ((f << 1) | (ctpop32(f & taps) & 1)) & mask;
    }
    return (state & ~(mask << lsb)) | (f << lsb);
}

static uint8_t prng_output(uint32_t st)
{
    /* Datasheet "RCP PRNG output": six XOR taps and a 3-way majority. */
    static const uint8_t taps[8][9] = {
        { 11, 3, 4, 19, 10, 14, 1, 2, 9 },
        { 4, 16, 11, 18, 9, 6, 14, 21, 16 },
        { 15, 13, 20, 21, 8, 12, 7, 22, 9 },
        { 23, 12, 7, 16, 14, 5, 17, 3, 15 },
        { 4, 19, 17, 0, 18, 7, 18, 11, 3 },
        { 7, 5, 2, 18, 11, 1, 18, 14, 7 },
        { 14, 21, 19, 6, 16, 13, 4, 14, 6 },
        { 7, 17, 6, 16, 13, 8, 9, 12, 21 },
    };
    uint8_t out = 0;
    int bit, i;

    for (bit = 0; bit < 8; bit++) {
        const uint8_t *t = taps[bit];
        int x = 0, maj;

        for (i = 0; i < 6; i++) {
            x ^= (st >> t[i]) & 1;
        }
        maj = ((st >> t[6]) & 1) + ((st >> t[7]) & 1) + ((st >> t[8]) & 1);
        out |= (x ^ (maj >= 2)) << bit;
    }
    return out;
}

static uint8_t prng_next(RP2350RCPCore *c)
{
    uint8_t out = prng_output(c->prng);

    c->prng = lfsr_step(c->prng, 20, 4, 0xc);
    c->prng = lfsr_step(c->prng, 15, 5, 0x14);
    c->prng = lfsr_step(c->prng, 8, 7, 0x60);
    c->prng = lfsr_step(c->prng, 0, 8, 0xb4);
    return out;
}

static void set_salt(RP2350RCPCore *c, uint64_t salt)
{
    c->salt = salt;
    c->salt_valid = true;
    c->prng = salt >> 40;
}

/* An RCP fault on one core is a fault on both. */
static void rcp_fault(RP2350RCPState *s)
{
    int i;

    for (i = 0; i < RP2350_RCP_CORES; i++) {
        s->core[i].faulted = true;
        qemu_irq_raise(s->nmi[i]);
    }
}

static bool is_bool(uint32_t v)
{
    return v == RCP_TRUE || v == RCP_FALSE;
}

typedef enum {
    RCP_INVALID,
    RCP_FAILED,
    RCP_PASSED,
    RCP_PANIC,
} RCPOutcome;

/* Decoded fields of an RCP instruction. */
typedef struct RCPInsn {
    uint32_t opc1;
    uint8_t tag;
} RCPInsn;

/*
 * Validate an RCP instruction's encoding, following the datasheet's rules
 * with one correction: every documented encoding has an odd number of 1
 * bits across Opc1 and the parity bit, so that is what is required.
 */
static bool rcp_decode(uint32_t insn, RCPInsn *d)
{
    uint32_t crn = extract32(insn, 16, 4);
    uint32_t crm = extract32(insn, 0, 4);
    uint32_t opc2 = extract32(insn, 5, 3);
    bool tagged;

    d->tag = (crn << 4) | crm;
    if (is_mcr(insn)) {
        d->opc1 = extract32(insn, 21, 3);
        tagged = d->opc1 == 0 || d->opc1 == 4 || d->opc1 == 5;
        return (ctpop32(d->opc1) + (opc2 & 1)) & 1 &&
               d->opc1 <= 5 && (tagged || !d->tag);
    }
    if (is_mrc(insn)) {
        d->opc1 = extract32(insn, 21, 3);
        return (ctpop32(d->opc1) + (opc2 & 1)) & 1 &&
               d->opc1 <= 2 && (d->opc1 == 0 || !d->tag);
    }
    if (is_mcrr(insn)) {
        d->opc1 = extract32(insn, 4, 4);
        return (ctpop32(d->opc1) + extract32(crm, 3, 1)) & 1 &&
               d->opc1 <= 8 && (crm & 7) <= (d->opc1 == 8 ? 1 : 0);
    }
    if (is_cdp(insn)) {
        /* rcp_panic is the only cdp. */
        d->opc1 = extract32(insn, 20, 4);
        return d->opc1 == 0 && !d->tag && !extract32(insn, 12, 4) &&
               opc2 == 1;
    }
    /* LDC, STC and MRRC */
    return false;
}

/* Execute one valid Secure RCP instruction. */
static RCPOutcome rcp_exec(RP2350RCPCore *c, uint32_t insn, RCPInsn *d,
                           uint32_t a, uint32_t b, uint32_t *result)
{
    RP2350RCPState *s = c->rcp;
    int target;

    /* Only the salt writes and rcp_canary_status work before a salt. */
    if (!c->salt_valid &&
        !(is_mcrr(insn) && d->opc1 == 8) &&
        !(is_mrc(insn) && d->opc1 == 1)) {
        return RCP_INVALID;
    }

    if (is_mcr(insn)) {
        switch (d->opc1) {
        case 0: /* rcp_canary_check */
            return a == canary(c->salt, d->tag) ? RCP_PASSED : RCP_FAILED;
        case 1: /* rcp_bvalid */
            return is_bool(a) ? RCP_PASSED : RCP_FAILED;
        case 2: /* rcp_btrue */
            return a == RCP_TRUE ? RCP_PASSED : RCP_FAILED;
        case 3: /* rcp_bfalse */
            return a == RCP_FALSE ? RCP_PASSED : RCP_FAILED;
        case 4: /* rcp_count_set */
            c->count = d->tag;
            return RCP_PASSED;
        default: /* rcp_count_check */
            if (d->tag != c->count) {
                return RCP_FAILED;
            }
            c->count++;
            return RCP_PASSED;
        }
    }

    if (is_mrc(insn)) {
        switch (d->opc1) {
        case 0: /* rcp_canary_get */
            *result = canary(c->salt, d->tag);
            break;
        case 1: /* rcp_canary_status */
            *result = c->salt_valid ? RCP_TRUE : RCP_FALSE;
            break;
        default: /* rcp_random_byte */
            *result = prng_next(c);
            break;
        }
        return RCP_PASSED;
    }

    if (is_mcrr(insn)) {
        switch (d->opc1) {
        case 0: /* rcp_b2valid */
            return is_bool(a) && is_bool(b) ? RCP_PASSED : RCP_FAILED;
        case 1: /* rcp_b2and */
            return a == RCP_TRUE && b == RCP_TRUE ? RCP_PASSED : RCP_FAILED;
        case 2: /* rcp_b2or */
            return is_bool(a) && is_bool(b) &&
                   (a == RCP_TRUE || b == RCP_TRUE) ? RCP_PASSED : RCP_FAILED;
        case 3: /* rcp_bxorvalid */
            return is_bool(a ^ b) ? RCP_PASSED : RCP_FAILED;
        case 4: /* rcp_bxortrue */
            return (a ^ b) == RCP_TRUE ? RCP_PASSED : RCP_FAILED;
        case 5: /* rcp_bxorfalse */
            return (a ^ b) == RCP_FALSE ? RCP_PASSED : RCP_FAILED;
        case 6: /* rcp_ivalid */
            return (a ^ b) == RCP_IVALID ? RCP_PASSED : RCP_FAILED;
        case 7: /* rcp_iequal */
            return a == b ? RCP_PASSED : RCP_FAILED;
        default: /* rcp_salt_core0, rcp_salt_core1 */
            target = extract32(insn, 0, 1);
            if (s->core[target].salt_valid) {
                return RCP_FAILED;
            }
            set_salt(&s->core[target], ((uint64_t)b << 32) | a);
            return RCP_PASSED;
        }
    }

    return RCP_PANIC;
}

/* [spec:nuos:req:emu.rcp] */
static ARMMCoprocResult rp2350_rcp_op(void *opaque, ARMCPU *cpu,
                                      uint32_t insn, uint32_t rt,
                                      uint32_t rt2, bool secure,
                                      uint64_t *result)
{
    RP2350RCPCore *c = opaque;
    uint32_t value = 0;
    RCPOutcome outcome;
    RCPInsn d;

    if (c->faulted) {
        return ARM_M_COPROC_STALL;
    }

    if (!secure) {
        /*
         * Non-secure code reads zeroes, has its writes ignored, and cannot
         * fault the RCP: invalid instructions are UsageFaults instead.
         */
        if (!rcp_decode(insn, &d)) {
            return ARM_M_COPROC_UNDEF;
        }
        *result = 0;
        return ARM_M_COPROC_OK;
    }

    if (is_delayed(insn) && c->salt_valid) {
        prng_next(c);
    }
    outcome = rcp_decode(insn, &d) ? rcp_exec(c, insn, &d, rt, rt2, &value)
                                   : RCP_INVALID;
    switch (outcome) {
    case RCP_PASSED:
        *result = value;
        return ARM_M_COPROC_OK;
    case RCP_PANIC:
        rcp_fault(c->rcp);
        return ARM_M_COPROC_STALL;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-rcp: core %d fault on "
                      "instruction 0x%08x (%s)\n", c->core, insn,
                      outcome == RCP_INVALID ? "invalid" : "assertion failed");
        rcp_fault(c->rcp);
        *result = 0;
        return ARM_M_COPROC_OK;
    }
}

void rp2350_rcp_attach(RP2350RCPState *s, int core, ARMCPU *cpu)
{
    arm_m_set_coprocessor(cpu, 7, rp2350_rcp_op, &s->core[core]);
}

static void rp2350_rcp_hold_reset(Object *obj, ResetType type)
{
    RP2350RCPState *s = RP2350_RCP(obj);
    int i;

    for (i = 0; i < RP2350_RCP_CORES; i++) {
        RP2350RCPCore *c = &s->core[i];

        c->salt = 0;
        c->salt_valid = false;
        c->count = 0;
        c->faulted = false;
        c->prng = 0;
        /* [spec:nuos:req:emu.rcp-handoff] */
        if (s->boot_rom_handoff) {
            uint64_t salt;

            qemu_guest_getrandom_nofail(&salt, sizeof(salt));
            set_salt(c, salt);
        }
    }
}

static void rp2350_rcp_exit_reset(Object *obj, ResetType type)
{
    RP2350RCPState *s = RP2350_RCP(obj);
    int i;

    for (i = 0; i < RP2350_RCP_CORES; i++) {
        qemu_irq_lower(s->nmi[i]);
    }
}

static void rp2350_rcp_init(Object *obj)
{
    RP2350RCPState *s = RP2350_RCP(obj);
    int i;

    for (i = 0; i < RP2350_RCP_CORES; i++) {
        s->core[i].rcp = s;
        s->core[i].core = i;
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->nmi[i]);
    }
}

static const VMStateDescription vmstate_rp2350_rcp_core = {
    .name = "rp2350-rcp-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(salt, RP2350RCPCore),
        VMSTATE_BOOL(salt_valid, RP2350RCPCore),
        VMSTATE_UINT8(count, RP2350RCPCore),
        VMSTATE_BOOL(faulted, RP2350RCPCore),
        VMSTATE_UINT32(prng, RP2350RCPCore),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_rcp = {
    .name = TYPE_RP2350_RCP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, RP2350RCPState, RP2350_RCP_CORES, 1,
                             vmstate_rp2350_rcp_core, RP2350RCPCore),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_rcp_properties[] = {
    DEFINE_PROP_BOOL("boot-rom-handoff", RP2350RCPState, boot_rom_handoff,
                     false),
};

static void rp2350_rcp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_rcp_hold_reset;
    rc->phases.exit = rp2350_rcp_exit_reset;
    dc->vmsd = &vmstate_rp2350_rcp;
    device_class_set_props(dc, rp2350_rcp_properties);
}

static const TypeInfo rp2350_rcp_info = {
    .name          = TYPE_RP2350_RCP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350RCPState),
    .instance_init = rp2350_rcp_init,
    .class_init    = rp2350_rcp_class_init,
};

static void rp2350_rcp_register_types(void)
{
    type_register_static(&rp2350_rcp_info);
}
type_init(rp2350_rcp_register_types)
