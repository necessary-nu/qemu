/*
 * RP2350 single-cycle IO block (SIO)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "SIO". Modelled: CPUID, the inter-core
 * FIFOs, the hardware spinlocks, the doorbells, and GPIO output/enable
 * storage. Secure and Non-secure banks are separate, as on hardware.
 * Interpolators, TMDS encoders and the RISC-V platform timer are not
 * modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/rp2350_sio.h"
#include "migration/vmstate.h"
#include "target/arm/cpu.h"

#define A_CPUID            0x000
#define A_GPIO_IN          0x004
#define A_GPIO_HI_IN       0x008
#define A_GPIO_OUT         0x010
#define A_GPIO_OE          0x030
#define A_GPIO_OE_XOR_HI   0x04c
#define A_FIFO_ST          0x050
#define A_FIFO_WR          0x054
#define A_FIFO_RD          0x058
#define A_SPINLOCK_ST      0x05c
#define A_SPINLOCK0        0x100
#define A_SPINLOCK31       0x17c
#define A_DOORBELL_OUT_SET 0x180
#define A_DOORBELL_OUT_CLR 0x184
#define A_DOORBELL_IN_SET  0x188
#define A_DOORBELL_IN_CLR  0x18c

#define FIFO_ST_VLD (1u << 0)
#define FIFO_ST_RDY (1u << 1)
#define FIFO_ST_WOF (1u << 2)
#define FIFO_ST_ROE (1u << 3)

#define DOORBELL_MASK 0xff

static uint32_t fifo_status(RP2350SIOBank *b, int core)
{
    int other = !core;
    uint32_t st = 0;

    if (b->fifo_count[other]) {
        st |= FIFO_ST_VLD;
    }
    if (b->fifo_count[core] < RP2350_SIO_FIFO_DEPTH) {
        st |= FIFO_ST_RDY;
    }
    if (b->wof[core]) {
        st |= FIFO_ST_WOF;
    }
    if (b->roe[core]) {
        st |= FIFO_ST_ROE;
    }
    return st;
}

/* [spec:nuos:req:emu.irq-routing+1] */
static void rp2350_sio_update_irqs(RP2350SIOState *s)
{
    int bank, core;

    for (bank = 0; bank < RP2350_SIO_BANKS; bank++) {
        RP2350SIOBank *b = &s->bank[bank];

        for (core = 0; core < RP2350_SIO_CORES; core++) {
            uint32_t st = fifo_status(b, core);

            qemu_set_irq(s->irq_fifo[bank][core],
                         !!(st & (FIFO_ST_VLD | FIFO_ST_WOF | FIFO_ST_ROE)));
            qemu_set_irq(s->irq_bell[bank][core], b->doorbell[core] != 0);
        }
    }
}

static void fifo_write(RP2350SIOBank *b, int core, uint32_t value)
{
    if (b->fifo_count[core] == RP2350_SIO_FIFO_DEPTH) {
        b->wof[core] = true;
        return;
    }
    b->fifo[core][(b->fifo_head[core] + b->fifo_count[core]) %
                  RP2350_SIO_FIFO_DEPTH] = value;
    b->fifo_count[core]++;
}

static uint32_t fifo_read(RP2350SIOBank *b, int core)
{
    int other = !core;
    uint32_t value;

    if (b->fifo_count[other] == 0) {
        b->roe[core] = true;
        return 0;
    }
    value = b->fifo[other][b->fifo_head[other]];
    b->fifo_head[other] = (b->fifo_head[other] + 1) % RP2350_SIO_FIFO_DEPTH;
    b->fifo_count[other]--;
    return value;
}

/*
 * GPIO registers are shared between banks. Non-secure access is filtered
 * per pin by ACCESSCTRL GPIO_NSMASK, which is not modelled and resets to
 * all pins Secure-only, so Non-secure GPIO accesses read zero and are
 * ignored.
 */
static uint32_t gpio_read(RP2350SIOState *s, hwaddr reg)
{
    int hi;

    switch (reg) {
    case A_GPIO_IN:
    case A_GPIO_HI_IN:
        hi = reg == A_GPIO_HI_IN;
        return s->gpio_out[hi] & s->gpio_oe[hi];
    }
    hi = (reg / 4) & 1;
    if (reg < A_GPIO_OE) {
        return s->gpio_out[hi];
    }
    return s->gpio_oe[hi];
}

static void gpio_write(RP2350SIOState *s, hwaddr reg, uint32_t value)
{
    uint32_t *r;
    int hi = (reg / 4) & 1;

    if (reg < A_GPIO_OUT) {
        return;
    }
    r = reg < A_GPIO_OE ? &s->gpio_out[hi] : &s->gpio_oe[hi];
    /* Each group is the register then its SET, CLR and XOR aliases. */
    switch (((reg - A_GPIO_OUT) / 8) % 4) {
    case 0:
        *r = value;
        break;
    case 1:
        *r |= value;
        break;
    case 2:
        *r &= ~value;
        break;
    case 3:
        *r ^= value;
        break;
    }
}

/*
 * Core 1's side of the boot ROM launch handshake (the wait_for_vector loop
 * in the boot ROM's Arm startup code). Core 1 drains its FIFO and sends 0,
 * then expects 1, the vector table, the stack pointer and the entry point
 * in turn, echoing each. Receiving 0 at any step, or anything but 1 first,
 * restarts the handshake.
 */
enum {
    C1_DRAIN,       /* drain incoming, then send 0 */
    C1_SEND,        /* send c1_send, then go to c1_next */
    C1_RECV_CMD,
    C1_RECV_VTOR,
    C1_RECV_SP,
    C1_RECV_ENTRY,
    C1_LAUNCH,
    C1_LAUNCHED,
};

/* [spec:nuos:req:emu.core1-launch] */
static void core1_handshake(RP2350SIOState *s)
{
    RP2350SIOBank *b = &s->bank[RP2350_SIO_SECURE];
    uint32_t v;

    if (!s->core1_launch) {
        return;
    }
    for (;;) {
        switch (s->c1_state) {
        case C1_DRAIN:
            while (b->fifo_count[0]) {
                fifo_read(b, 1);
            }
            s->c1_send = 0;
            s->c1_next = C1_RECV_CMD;
            s->c1_state = C1_SEND;
            break;
        case C1_SEND:
            if (b->fifo_count[1] == RP2350_SIO_FIFO_DEPTH) {
                return;
            }
            fifo_write(b, 1, s->c1_send);
            s->c1_state = s->c1_next;
            break;
        case C1_RECV_CMD:
        case C1_RECV_VTOR:
        case C1_RECV_SP:
        case C1_RECV_ENTRY:
            if (!b->fifo_count[0]) {
                return;
            }
            v = fifo_read(b, 1);
            if (v == 0 || (s->c1_state == C1_RECV_CMD && v != 1)) {
                s->c1_state = C1_DRAIN;
                break;
            }
            if (s->c1_state == C1_RECV_VTOR) {
                s->c1_vtor = v;
            } else if (s->c1_state == C1_RECV_SP) {
                s->c1_sp = v;
            }
            s->c1_send = v;
            s->c1_next = s->c1_state == C1_RECV_ENTRY ? C1_LAUNCH
                                                      : s->c1_state + 1;
            s->c1_state = C1_SEND;
            break;
        case C1_LAUNCH:
            s->c1_state = C1_LAUNCHED;
            if (s->c1_launch_fn) {
                s->c1_launch_fn(s->c1_launch_opaque, s->c1_vtor, s->c1_sp,
                                s->c1_send);
            }
            return;
        default:
            return;
        }
    }
}

void rp2350_sio_set_core1_launch(RP2350SIOState *s, RP2350SIOCore1Launch *fn,
                                 void *opaque)
{
    s->c1_launch_fn = fn;
    s->c1_launch_opaque = opaque;
}

/* Apply a GPIOC write operation: 0 write, 1 XOR, 2 set, 3 clear. */
static void gpio_op(uint32_t *r, int op, uint32_t v)
{
    switch (op & 3) {
    case 0:
        *r = v;
        break;
    case 1:
        *r ^= v;
        break;
    case 2:
        *r |= v;
        break;
    default:
        *r &= ~v;
        break;
    }
}

/* GPIO output (group 0) or output-enable (group 1) registers. */
static uint32_t *gpio_group(RP2350SIOState *s, int group)
{
    return group ? s->gpio_oe : s->gpio_out;
}

/*
 * The GPIO coprocessor: single instructions for the SIO GPIO registers
 * (datasheet "GPIO coprocessor (GPIOC)"). CRm bits 3:2 select outputs,
 * output enables or inputs, and bit 0 the low or high register; opc1 bits
 * 1:0 the write operation and bits 3:2 the addressing mode. Like SIO
 * itself, Non-secure accesses see no GPIOs, since ACCESSCTRL GPIO_NSMASK
 * is not modelled and resets to none.
 */
/* [spec:nuos:req:emu.gpioc] */
static ARMMCoprocResult rp2350_gpioc_op(void *opaque, ARMCPU *cpu,
                                        uint32_t insn, uint32_t rt,
                                        uint32_t rt2, bool secure,
                                        uint64_t *result)
{
    RP2350SIOState *s = ((RP2350SIOView *)opaque)->sio;
    uint32_t crm = extract32(insn, 0, 4);
    int group = crm >> 2, hi = crm & 1;
    uint32_t opc1, *r;
    uint64_t bit;

    /* CRm bit 1 is reserved; in mcr and mrc, CRn must be zero. */
    if (extract32(crm, 1, 1) ||
        ((insn & 0x0f000010) == 0x0e000010 && extract32(insn, 16, 4))) {
        return ARM_M_COPROC_UNDEF;
    }

    if ((insn & 0x0f100010) == 0x0e000010) {
        /* mcr: whole-register writes, and single-bit writes by index */
        opc1 = extract32(insn, 21, 3);
        if (group > 1 || (opc1 >= 4 && (hi || opc1 == 4))) {
            return ARM_M_COPROC_UNDEF;
        }
        if (secure) {
            r = gpio_group(s, group);
            if (opc1 < 4) {
                gpio_op(&r[hi], opc1, rt);
            } else {
                bit = 1ull << (rt & 63);
                gpio_op(&r[bit >> 32 ? 1 : 0], opc1 - 4, bit | bit >> 32);
            }
        }
        return ARM_M_COPROC_OK;
    }

    if ((insn & 0x0ff00000) == 0x0c400000) {
        /* mcrr */
        opc1 = extract32(insn, 4, 4);
        if (group > 1 || hi || opc1 > 11) {
            return ARM_M_COPROC_UNDEF;
        }
        if (!secure) {
            return ARM_M_COPROC_OK;
        }
        r = gpio_group(s, group);
        if (opc1 < 4) {
            /* Both registers at once: Rt low, Rt2 high. */
            gpio_op(&r[0], opc1, rt);
            gpio_op(&r[1], opc1, rt2);
        } else if (opc1 < 8) {
            /* One bit, numbered by Rt; 4 writes Rt2 & 1, 5-7 are gated. */
            bit = 1ull << (rt & 63);
            int word = bit >> 32 ? 1 : 0;
            uint32_t mask = word ? bit >> 32 : bit;

            if (opc1 == 4) {
                gpio_op(&r[word], rt2 & 1 ? 2 : 3, mask);
            } else if (rt2 & 1) {
                gpio_op(&r[word], opc1 - 4, mask);
            }
        } else if (rt2 < 2) {
            /* Indexed: Rt2 selects the register. */
            gpio_op(&r[rt2], opc1 - 8, rt);
        }
        return ARM_M_COPROC_OK;
    }

    if ((insn & 0x0f100010) == 0x0e100010) {
        /* mrc */
        if (extract32(insn, 21, 3) || group > 2) {
            return ARM_M_COPROC_UNDEF;
        }
        *result = secure ? (group == 2 ? s->gpio_out[hi] & s->gpio_oe[hi]
                                       : gpio_group(s, group)[hi]) : 0;
        return ARM_M_COPROC_OK;
    }

    if ((insn & 0x0ff00000) == 0x0c500000) {
        /* mrrc: both registers of a group */
        if (extract32(insn, 4, 4) || group > 2 || hi) {
            return ARM_M_COPROC_UNDEF;
        }
        if (secure) {
            uint32_t lo_v, hi_v;

            if (group == 2) {
                lo_v = s->gpio_out[0] & s->gpio_oe[0];
                hi_v = s->gpio_out[1] & s->gpio_oe[1];
            } else {
                lo_v = gpio_group(s, group)[0];
                hi_v = gpio_group(s, group)[1];
            }
            *result = ((uint64_t)hi_v << 32) | lo_v;
        }
        return ARM_M_COPROC_OK;
    }
    return ARM_M_COPROC_UNDEF;
}

void rp2350_sio_attach_gpioc(RP2350SIOState *s, int core, ARMCPU *cpu)
{
    s->gpioc_opaque[core].sio = s;
    s->gpioc_opaque[core].core = core;
    arm_m_set_coprocessor(cpu, 0, rp2350_gpioc_op, &s->gpioc_opaque[core]);
}

static RP2350SIOBank *view_bank(RP2350SIOView *v, MemTxAttrs attrs)
{
    int bank = v->bank;

    if (bank < 0) {
        /* Unspecified attributes (qtest, debug) see the Secure bank. */
        bank = (attrs.secure || attrs.unspecified) ? RP2350_SIO_SECURE
                                                   : RP2350_SIO_NONSECURE;
    }
    return &v->sio->bank[bank];
}

static bool view_is_secure(RP2350SIOView *v, MemTxAttrs attrs)
{
    return view_bank(v, attrs) == &v->sio->bank[RP2350_SIO_SECURE];
}

/* Only Secure code may use the Non-secure mirror. */
static bool view_accessible(RP2350SIOView *v, MemTxAttrs attrs)
{
    return v->bank != RP2350_SIO_NONSECURE || attrs.secure ||
           attrs.unspecified;
}

/* [spec:nuos:req:emu.sio] */
static MemTxResult rp2350_sio_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    RP2350SIOView *v = opaque;
    RP2350SIOState *s = v->sio;
    RP2350SIOBank *b;
    uint32_t bit;

    if (!view_accessible(v, attrs)) {
        return MEMTX_ERROR;
    }
    b = view_bank(v, attrs);

    switch (addr) {
    case A_CPUID:
        *data = v->core;
        break;
    case A_GPIO_IN ... A_GPIO_OE_XOR_HI:
        *data = view_is_secure(v, attrs) ? gpio_read(s, addr) : 0;
        break;
    case A_FIFO_ST:
        *data = fifo_status(b, v->core);
        break;
    case A_FIFO_RD:
        *data = fifo_read(b, v->core);
        core1_handshake(s);
        rp2350_sio_update_irqs(s);
        break;
    case A_FIFO_WR:
        *data = 0;
        break;
    case A_SPINLOCK_ST:
        *data = b->spinlocks;
        break;
    case A_SPINLOCK0 ... A_SPINLOCK31:
        bit = 1u << ((addr - A_SPINLOCK0) / 4);
        if (b->spinlocks & bit) {
            *data = 0;
        } else {
            b->spinlocks |= bit;
            *data = bit;
        }
        break;
    case A_DOORBELL_OUT_SET:
    case A_DOORBELL_OUT_CLR:
        *data = b->doorbell[!v->core];
        break;
    case A_DOORBELL_IN_SET:
    case A_DOORBELL_IN_CLR:
        *data = b->doorbell[v->core];
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "rp2350-sio: unimplemented read at 0x%"
                      HWADDR_PRIx "\n", addr);
        *data = 0;
        break;
    }
    return MEMTX_OK;
}

static MemTxResult rp2350_sio_write(void *opaque, hwaddr addr, uint64_t value,
                                    unsigned size, MemTxAttrs attrs)
{
    RP2350SIOView *v = opaque;
    RP2350SIOState *s = v->sio;
    RP2350SIOBank *b;

    if (!view_accessible(v, attrs)) {
        return MEMTX_ERROR;
    }
    b = view_bank(v, attrs);

    switch (addr) {
    case A_CPUID:
    case A_FIFO_RD:
    case A_SPINLOCK_ST:
        break;
    case A_GPIO_IN ... A_GPIO_OE_XOR_HI:
        if (view_is_secure(v, attrs)) {
            gpio_write(s, addr, value);
        }
        break;
    case A_FIFO_ST:
        b->roe[v->core] = false;
        b->wof[v->core] = false;
        break;
    case A_FIFO_WR:
        fifo_write(b, v->core, value);
        core1_handshake(s);
        break;
    case A_SPINLOCK0 ... A_SPINLOCK31:
        b->spinlocks &= ~(1u << ((addr - A_SPINLOCK0) / 4));
        break;
    case A_DOORBELL_OUT_SET:
        b->doorbell[!v->core] |= value & DOORBELL_MASK;
        break;
    case A_DOORBELL_OUT_CLR:
        b->doorbell[!v->core] &= ~value;
        break;
    case A_DOORBELL_IN_SET:
        b->doorbell[v->core] |= value & DOORBELL_MASK;
        break;
    case A_DOORBELL_IN_CLR:
        b->doorbell[v->core] &= ~value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "rp2350-sio: unimplemented write at 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
    rp2350_sio_update_irqs(s);
    return MEMTX_OK;
}

/*
 * Narrower accesses act on the containing word; pico-sdk reads CPUID with
 * a byte load.
 */
static const MemoryRegionOps rp2350_sio_ops = {
    .read_with_attrs = rp2350_sio_read,
    .write_with_attrs = rp2350_sio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

MemoryRegion *rp2350_sio_view(RP2350SIOState *s, int core, bool mirror)
{
    return &s->view[core][mirror];
}

static void rp2350_sio_hold_reset(Object *obj, ResetType type)
{
    RP2350SIOState *s = RP2350_SIO(obj);

    memset(s->bank, 0, sizeof(s->bank));
    memset(s->gpio_out, 0, sizeof(s->gpio_out));
    memset(s->gpio_oe, 0, sizeof(s->gpio_oe));
}

static void rp2350_sio_exit_reset(Object *obj, ResetType type)
{
    RP2350SIOState *s = RP2350_SIO(obj);

    s->c1_state = C1_DRAIN;
    core1_handshake(s);
    rp2350_sio_update_irqs(s);
}

static void rp2350_sio_init(Object *obj)
{
    RP2350SIOState *s = RP2350_SIO(obj);
    int core, mirror, bank;

    for (core = 0; core < RP2350_SIO_CORES; core++) {
        for (mirror = 0; mirror < 2; mirror++) {
            RP2350SIOView *v = &s->view_opaque[core][mirror];

            v->sio = s;
            v->core = core;
            v->bank = mirror ? RP2350_SIO_NONSECURE : -1;
            memory_region_init_io(&s->view[core][mirror], obj,
                                  &rp2350_sio_ops, v,
                                  mirror ? "rp2350-sio-nonsec" : "rp2350-sio",
                                  RP2350_SIO_VIEW_SIZE);
        }
    }

    /* GPIO outputs: FIFO then doorbell, per bank, per core. */
    for (bank = 0; bank < RP2350_SIO_BANKS; bank++) {
        for (core = 0; core < RP2350_SIO_CORES; core++) {
            sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq_fifo[bank][core]);
            sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq_bell[bank][core]);
        }
    }
}

static const VMStateDescription vmstate_rp2350_sio_bank = {
    .name = "rp2350-sio-bank",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(fifo, RP2350SIOBank, RP2350_SIO_CORES,
                               RP2350_SIO_FIFO_DEPTH),
        VMSTATE_UINT8_ARRAY(fifo_head, RP2350SIOBank, RP2350_SIO_CORES),
        VMSTATE_UINT8_ARRAY(fifo_count, RP2350SIOBank, RP2350_SIO_CORES),
        VMSTATE_BOOL_ARRAY(roe, RP2350SIOBank, RP2350_SIO_CORES),
        VMSTATE_BOOL_ARRAY(wof, RP2350SIOBank, RP2350_SIO_CORES),
        VMSTATE_UINT8_ARRAY(doorbell, RP2350SIOBank, RP2350_SIO_CORES),
        VMSTATE_UINT32(spinlocks, RP2350SIOBank),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_rp2350_sio = {
    .name = TYPE_RP2350_SIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(bank, RP2350SIOState, RP2350_SIO_BANKS, 1,
                             vmstate_rp2350_sio_bank, RP2350SIOBank),
        VMSTATE_UINT32_ARRAY(gpio_out, RP2350SIOState, 2),
        VMSTATE_UINT32_ARRAY(gpio_oe, RP2350SIOState, 2),
        VMSTATE_UINT32(c1_state, RP2350SIOState),
        VMSTATE_UINT32(c1_send, RP2350SIOState),
        VMSTATE_UINT32(c1_next, RP2350SIOState),
        VMSTATE_UINT32(c1_vtor, RP2350SIOState),
        VMSTATE_UINT32(c1_sp, RP2350SIOState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property rp2350_sio_properties[] = {
    DEFINE_PROP_BOOL("core1-launch", RP2350SIOState, core1_launch, false),
};

static void rp2350_sio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, rp2350_sio_properties);
    rc->phases.hold = rp2350_sio_hold_reset;
    rc->phases.exit = rp2350_sio_exit_reset;
    dc->vmsd = &vmstate_rp2350_sio;
}

static const TypeInfo rp2350_sio_info = {
    .name          = TYPE_RP2350_SIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350SIOState),
    .instance_init = rp2350_sio_init,
    .class_init    = rp2350_sio_class_init,
};

static void rp2350_sio_register_types(void)
{
    type_register_static(&rp2350_sio_info);
}
type_init(rp2350_sio_register_types)
