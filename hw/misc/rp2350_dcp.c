/*
 * RP2350 double-precision coprocessor (DCP)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Double-precision coprocessor (DCP)", and
 * the instruction encodings in pico-sdk's hardware/dcp_instr.inc.S.
 *
 * Modelled: the register state of each instance, INIT, the direct writes
 * (WXMD, WYMD, WEFD) and reads (RXMD, RYMD, REFD, and their non-clearing
 * P forms) used to save and restore it, and RCMP/PCMP as used to clear the
 * engaged flag. The arithmetic, conversion and packed-result instructions
 * are not modelled and raise an UNDEFINSTR UsageFault, so a missing model
 * can never produce a wrong floating-point result.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/misc/rp2350_dcp.h"
#include "migration/vmstate.h"
#include "target/arm/cpu.h"

#define STATUS_ENGAGED (1u << 8)
#define STATUS_MASK    0x1ff
#define EXP_MASK       0x3fff
#define FLAG_MASK      0xf

static bool is_cdp(uint32_t insn)
{
    return (insn & 0x0f000010) == 0x0e000000;
}

static bool is_mrc(uint32_t insn)
{
    return (insn & 0x0f100010) == 0x0e100010;
}

static bool is_mcrr(uint32_t insn)
{
    return (insn & 0x0ff00000) == 0x0c400000;
}

static bool is_mrrc(uint32_t insn)
{
    return (insn & 0x0ff00000) == 0x0c500000;
}

/* The "2" forms of the reads (Pxxx) leave the engaged flag alone. */
static bool is_peek(uint32_t insn)
{
    return extract32(insn, 28, 1);
}

/*
 * WEFD and REFD move the exponents, flags and status as a pair of words.
 * The datasheet does not give the layout; this one round-trips, which is
 * all a save and restore needs: low word xe | xf << 14 | status << 18,
 * high word ye | yf << 14.
 */
static uint64_t pack_efd(RP2350DCPInstance *d)
{
    uint32_t lo = d->xe | d->xf << 14 | d->status << 18;
    uint32_t hi = d->ye | d->yf << 14;

    return ((uint64_t)hi << 32) | lo;
}

static void unpack_efd(RP2350DCPInstance *d, uint32_t lo, uint32_t hi)
{
    d->xe = lo & EXP_MASK;
    d->xf = (lo >> 14) & FLAG_MASK;
    d->status = (lo >> 18) & STATUS_MASK;
    d->ye = hi & EXP_MASK;
    d->yf = (hi >> 14) & FLAG_MASK;
}

/* [spec:nuos:req:emu.dcp-state] */
static ARMMCoprocResult rp2350_dcp_op(void *opaque, ARMCPU *cpu,
                                      uint32_t insn, uint32_t rt,
                                      uint32_t rt2, bool secure,
                                      uint64_t *result)
{
    RP2350DCPCore *c = opaque;
    int cp = extract32(insn, 8, 4);
    uint32_t opc1, crn = extract32(insn, 16, 4), crm = extract32(insn, 0, 4);
    uint32_t opc2 = extract32(insn, 5, 3);
    RP2350DCPInstance *d;

    /* cp4 is this state's instance; cp5 is the Non-secure one, for Secure. */
    if (cp == 5 && !secure) {
        return ARM_M_COPROC_UNDEF;
    }
    d = &c->inst[cp == 5 ? 1 : !secure];

    if (is_cdp(insn)) {
        opc1 = extract32(insn, 20, 4);
        if (opc1 == 0 && crn == 0 && crm == 0 && opc2 == 0 &&
            !extract32(insn, 12, 4)) {
            /* INIT: zero all registers, starting a canned sequence. */
            memset(d, 0, sizeof(*d));
            d->status = STATUS_ENGAGED;
            return ARM_M_COPROC_OK;
        }
    } else if (is_mcrr(insn)) {
        opc1 = extract32(insn, 4, 4);
        if (opc1 == 0) {
            switch (crm) {
            case 0: /* WXMD */
                d->xm = ((uint64_t)rt2 << 32) | rt;
                d->status |= STATUS_ENGAGED;
                return ARM_M_COPROC_OK;
            case 1: /* WYMD */
                d->ym = ((uint64_t)rt2 << 32) | rt;
                d->status |= STATUS_ENGAGED;
                return ARM_M_COPROC_OK;
            case 2: /* WEFD, which restores the engaged flag too */
                unpack_efd(d, rt, rt2);
                return ARM_M_COPROC_OK;
            }
        }
    } else if (is_mrrc(insn)) {
        opc1 = extract32(insn, 4, 4);
        if (opc1 == 0) {
            switch (crm) {
            case 8: /* RXMD */
                *result = d->xm;
                return ARM_M_COPROC_OK;
            case 9: /* RYMD, the last read of a save: engaged = 0 */
                *result = d->ym;
                if (!is_peek(insn)) {
                    d->status &= ~STATUS_ENGAGED;
                }
                return ARM_M_COPROC_OK;
            case 10: /* REFD */
                *result = pack_efd(d);
                return ARM_M_COPROC_OK;
            }
        }
    } else if (is_mrc(insn)) {
        opc1 = extract32(insn, 21, 3);
        if (opc1 == 0 && crn == 0 && crm == 0 && opc2 == 1) {
            /*
             * RCMP ends a comparison sequence. Comparisons are not
             * modelled (ADD0 faults), so it only ever reads an idle
             * status, and returns 0 after clearing the engaged flag.
             */
            *result = 0;
            if (!is_peek(insn)) {
                d->status &= ~STATUS_ENGAGED;
            }
            return ARM_M_COPROC_OK;
        }
    }

    qemu_log_mask(LOG_UNIMP, "rp2350-dcp: core %d: instruction 0x%08x is not "
                  "modelled\n", c->core, insn);
    return ARM_M_COPROC_UNDEF;
}

void rp2350_dcp_attach(RP2350DCPState *s, int core, ARMCPU *cpu)
{
    arm_m_set_coprocessor(cpu, 4, rp2350_dcp_op, &s->core[core]);
    arm_m_set_coprocessor(cpu, 5, rp2350_dcp_op, &s->core[core]);
}

static void rp2350_dcp_hold_reset(Object *obj, ResetType type)
{
    RP2350DCPState *s = RP2350_DCP(obj);
    int i;

    for (i = 0; i < RP2350_DCP_CORES; i++) {
        memset(s->core[i].inst, 0, sizeof(s->core[i].inst));
    }
}

static void rp2350_dcp_init(Object *obj)
{
    RP2350DCPState *s = RP2350_DCP(obj);
    int i;

    for (i = 0; i < RP2350_DCP_CORES; i++) {
        s->core[i].dcp = s;
        s->core[i].core = i;
    }
}

static const VMStateDescription vmstate_rp2350_dcp_instance = {
    .name = "rp2350-dcp-instance",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(xm, RP2350DCPInstance),
        VMSTATE_UINT64(ym, RP2350DCPInstance),
        VMSTATE_UINT32(xe, RP2350DCPInstance),
        VMSTATE_UINT32(ye, RP2350DCPInstance),
        VMSTATE_UINT32(xf, RP2350DCPInstance),
        VMSTATE_UINT32(yf, RP2350DCPInstance),
        VMSTATE_UINT32(status, RP2350DCPInstance),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_dcp_core = {
    .name = "rp2350-dcp-core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(inst, RP2350DCPCore, 2, 1,
                             vmstate_rp2350_dcp_instance, RP2350DCPInstance),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_dcp = {
    .name = TYPE_RP2350_DCP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, RP2350DCPState, RP2350_DCP_CORES, 1,
                             vmstate_rp2350_dcp_core, RP2350DCPCore),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_dcp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_dcp_hold_reset;
    dc->vmsd = &vmstate_rp2350_dcp;
}

static const TypeInfo rp2350_dcp_info = {
    .name          = TYPE_RP2350_DCP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350DCPState),
    .instance_init = rp2350_dcp_init,
    .class_init    = rp2350_dcp_class_init,
};

static void rp2350_dcp_register_types(void)
{
    type_register_static(&rp2350_dcp_info);
}
type_init(rp2350_dcp_register_types)
