/*
 * RP2350 Cortex-M33 extended private peripheral bus registers (EPPB)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "Cortex-M33 EPPB registers" and
 * "Non-maskable interrupt (NMI)". Register layout and reset values match
 * the pico-sdk m33_eppb.h.
 *
 * The EPPB is core-local: each core has its own copy, reset by a warm reset
 * of that core. Unlike the Arm-defined PPB registers it has the RP2350
 * atomic XOR/SET/CLR aliases.
 *
 * Security: the EPPB is not banked over Secure and Non-secure. The RP2350
 * IDAU leaves 0xe0080000 Non-secure, so its security attribution is the
 * SAU's: a Non-secure access to it is a SecureFault unless Secure software
 * has made the range Non-secure through the SAU, which the CPU enforces
 * before the access reaches this block. Like the rest of the PPB it is
 * privileged-only; an unprivileged access is a BusFault.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_eppb.h"
#include "migration/vmstate.h"

#define A_NMI_MASK0 0x0
#define A_NMI_MASK1 0x4
#define A_SLEEPCTRL 0x8

#define NMI_MASK_BITS MAKE_64BIT_MASK(0, RP2350_EPPB_NUM_IRQS)

#define SLEEPCTRL_LIGHT_SLEEP (1u << 0)
#define SLEEPCTRL_WICENREQ    (1u << 1)
#define SLEEPCTRL_WICENACK    (1u << 2)
#define SLEEPCTRL_RW          (SLEEPCTRL_LIGHT_SLEEP | SLEEPCTRL_WICENREQ)
#define SLEEPCTRL_RESET       SLEEPCTRL_WICENREQ

/*
 * WICENACK follows WICENREQ once the processor's interrupt controller has
 * taken the request. The datasheet gives no latency; ten system clock
 * cycles at 150 MHz is a handful of cycles, long enough for software that
 * polls WICENACK as the datasheet asks to see it lag.
 */
#define WICEN_ACK_DELAY_NS 67

/*
 * Each system IRQ whose bit is set in NMI_MASK drives NMI, alongside its
 * normal NVIC input; an RCP fault drives NMI regardless of the mask.
 */
/* [spec:nuos:req:emu.eppb] */
static void rp2350_eppb_update_nmi(RP2350EPPBState *s)
{
    qemu_set_irq(s->nmi, s->rcp_nmi || (s->irq_level & s->nmi_mask));
}

static void rp2350_eppb_set_irq(void *opaque, int n, int level)
{
    RP2350EPPBState *s = opaque;

    s->irq_level = deposit64(s->irq_level, n, 1, level != 0);
    qemu_set_irq(s->irq_out[n], level);
    rp2350_eppb_update_nmi(s);
}

static void rp2350_eppb_set_rcp_nmi(void *opaque, int n, int level)
{
    RP2350EPPBState *s = opaque;

    s->rcp_nmi = level != 0;
    rp2350_eppb_update_nmi(s);
}

static void rp2350_eppb_wicen_ack(void *opaque)
{
    RP2350EPPBState *s = opaque;

    if (s->sleepctrl & SLEEPCTRL_WICENREQ) {
        s->sleepctrl |= SLEEPCTRL_WICENACK;
    } else {
        s->sleepctrl &= ~SLEEPCTRL_WICENACK;
    }
}

static void rp2350_eppb_schedule_wicen_ack(RP2350EPPBState *s)
{
    timer_mod(s->wicen_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + WICEN_ACK_DELAY_NS);
}

static void rp2350_eppb_write_sleepctrl(RP2350EPPBState *s, uint32_t value)
{
    uint32_t old = s->sleepctrl;

    s->sleepctrl = (old & ~SLEEPCTRL_RW) | (value & SLEEPCTRL_RW);
    if ((old ^ s->sleepctrl) & SLEEPCTRL_LIGHT_SLEEP) {
        /*
         * LIGHT_SLEEP only decides whether a sleeping core keeps the
         * system clock request asserted, which costs 5 cycles of wakeup
         * latency when it does not. Clock requests and wakeup latency are
         * not modelled.
         */
        qemu_log_mask(LOG_UNIMP,
                      "rp2350-eppb: SLEEPCTRL.LIGHT_SLEEP=%d has no effect: "
                      "the sleep clock request is not modelled\n",
                      !!(s->sleepctrl & SLEEPCTRL_LIGHT_SLEEP));
    }
    if ((old ^ s->sleepctrl) & SLEEPCTRL_WICENREQ) {
        /*
         * A WIC sleep hands wakeup detection to the wake-up interrupt
         * controller during deep sleep. QEMU wakes a sleeping core on any
         * pending interrupt, so both kinds of deep sleep behave alike.
         */
        qemu_log_mask(LOG_UNIMP,
                      "rp2350-eppb: SLEEPCTRL.WICENREQ=%d has no effect: "
                      "WIC deep sleep is not modelled\n",
                      !!(s->sleepctrl & SLEEPCTRL_WICENREQ));
        rp2350_eppb_schedule_wicen_ack(s);
    }
}

static MemTxResult rp2350_eppb_read(void *opaque, hwaddr addr,
                                    uint64_t *data, unsigned size,
                                    MemTxAttrs attrs)
{
    RP2350EPPBState *s = opaque;

    if (attrs.user) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-eppb: unprivileged read of offset 0x%"
                      HWADDR_PRIx "\n", addr);
        return MEMTX_ERROR;
    }

    switch (rp2350_atomic_reg(addr)) {
    case A_NMI_MASK0:
        *data = extract64(s->nmi_mask, 0, 32);
        break;
    case A_NMI_MASK1:
        *data = extract64(s->nmi_mask, 32, 32);
        break;
    case A_SLEEPCTRL:
        *data = s->sleepctrl;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-eppb: read of bad offset 0x%" HWADDR_PRIx "\n",
                      addr);
        *data = 0;
        break;
    }
    return MEMTX_OK;
}

static MemTxResult rp2350_eppb_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size,
                                     MemTxAttrs attrs)
{
    RP2350EPPBState *s = opaque;
    uint32_t old;

    if (attrs.user) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-eppb: unprivileged write to offset 0x%"
                      HWADDR_PRIx "\n", addr);
        return MEMTX_ERROR;
    }

    switch (rp2350_atomic_reg(addr)) {
    case A_NMI_MASK0:
        old = extract64(s->nmi_mask, 0, 32);
        s->nmi_mask = deposit64(s->nmi_mask, 0, 32,
                                rp2350_atomic_apply(addr, old, value));
        rp2350_eppb_update_nmi(s);
        break;
    case A_NMI_MASK1:
        old = extract64(s->nmi_mask, 32, 32);
        s->nmi_mask = deposit64(s->nmi_mask, 32, 32,
                                rp2350_atomic_apply(addr, old, value));
        s->nmi_mask &= NMI_MASK_BITS;
        rp2350_eppb_update_nmi(s);
        break;
    case A_SLEEPCTRL:
        rp2350_eppb_write_sleepctrl(
            s, rp2350_atomic_apply(addr, s->sleepctrl, value));
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "rp2350-eppb: write to bad offset 0x%" HWADDR_PRIx "\n",
                      addr);
        break;
    }
    return MEMTX_OK;
}

static const MemoryRegionOps rp2350_eppb_ops = {
    .read_with_attrs = rp2350_eppb_read,
    .write_with_attrs = rp2350_eppb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void rp2350_eppb_hold_reset(Object *obj, ResetType type)
{
    RP2350EPPBState *s = RP2350_EPPB(obj);

    s->nmi_mask = 0;
    s->sleepctrl = SLEEPCTRL_RESET;
    timer_del(s->wicen_timer);
}

static void rp2350_eppb_exit_reset(Object *obj, ResetType type)
{
    RP2350EPPBState *s = RP2350_EPPB(obj);

    /* WICENACK resets to 0, then acknowledges WICENREQ's reset value. */
    rp2350_eppb_schedule_wicen_ack(s);
    rp2350_eppb_update_nmi(s);
}

static void rp2350_eppb_init(Object *obj)
{
    RP2350EPPBState *s = RP2350_EPPB(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_eppb_ops, s,
                          TYPE_RP2350_EPPB, RP2350_ATOMIC_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_in(dev, rp2350_eppb_set_irq, RP2350_EPPB_NUM_IRQS);
    qdev_init_gpio_in_named(dev, rp2350_eppb_set_rcp_nmi, "rcp-nmi", 1);
    qdev_init_gpio_out_named(dev, s->irq_out, "irq", RP2350_EPPB_NUM_IRQS);
    qdev_init_gpio_out_named(dev, &s->nmi, "nmi", 1);

    s->wicen_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rp2350_eppb_wicen_ack,
                                  s);
}

static void rp2350_eppb_finalize(Object *obj)
{
    RP2350EPPBState *s = RP2350_EPPB(obj);

    timer_free(s->wicen_timer);
}

static const VMStateDescription vmstate_rp2350_eppb = {
    .name = TYPE_RP2350_EPPB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(nmi_mask, RP2350EPPBState),
        VMSTATE_UINT32(sleepctrl, RP2350EPPBState),
        VMSTATE_UINT64(irq_level, RP2350EPPBState),
        VMSTATE_BOOL(rcp_nmi, RP2350EPPBState),
        VMSTATE_TIMER_PTR(wicen_timer, RP2350EPPBState),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_eppb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_eppb_hold_reset;
    rc->phases.exit = rp2350_eppb_exit_reset;
    dc->vmsd = &vmstate_rp2350_eppb;
}

/* [spec:nuos:req:emu.eppb] */
static const TypeInfo rp2350_eppb_info = {
    .name              = TYPE_RP2350_EPPB,
    .parent            = TYPE_SYS_BUS_DEVICE,
    .instance_size     = sizeof(RP2350EPPBState),
    .instance_init     = rp2350_eppb_init,
    .instance_finalize = rp2350_eppb_finalize,
    .class_init        = rp2350_eppb_class_init,
};

static void rp2350_eppb_register_types(void)
{
    type_register_static(&rp2350_eppb_info);
}
type_init(rp2350_eppb_register_types)
