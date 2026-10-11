/*
 * ESP32-P4 LP Mailbox
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 24 (LP Mailbox), and ESP-IDF's lp_mailbox_reg.h for v3, which gives the
 * registers' layout, access and reset values; its hal lp_mailbox_ll.h and
 * ulp lp_core_mailbox.c show how software drives it.
 *
 * Sixteen 32-bit message registers, MB_MASSEGE_0 to _15 (the TRM's and
 * the header's spelling), through which the LP core and the HP cores
 * exchange words, and two interrupts. A write to message n, from any
 * master and whatever it writes, sets bit n of both MB_LP_INT_RAW and
 * MB_HP_INT_RAW (TRM table 24.3-1): the writer's own interrupt is raised
 * too, which ESP-IDF's mailbox protocol clears after writing its
 * acknowledgement. The enabled raw bits drive MB_LP_INTR and MB_HP_INTR;
 * the raw bits clear only through MB_LP_INT_CLR and MB_HP_INT_CLR. Out of
 * reset messages 0 to 7 have their interrupts enabled.
 *
 * MB_REG_CLK_EN keeps the register clock running between accesses; the
 * registers behave the same either way. The block is in the LP always-on
 * domain, its registers behind LP_DYN_FAST_CLK and its reset
 * LP_CLKRST_RST_EN_LP_MAILBOX.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/esp32p4_lp_mailbox.h"
#include "migration/vmstate.h"

/*
 * The messages, then two blocks of interrupt registers, MB_LP_INT_* at
 * 0x40 and MB_HP_INT_* at 0x50, each RAW, ST, ENA and CLR.
 */
#define A_INT_BLOCKS        0x40
#define A_REG_CLK_EN        0x60
#define INT_BLOCK_SIZE      0x10
#define INT_RAW             0x0
#define INT_ST              0x4
#define INT_ENA             0x8
#define INT_CLR             0xc

#define INT_MASK            0xffff
#define INT_ENA_RESET       0x00ff
#define REG_CLK_EN          BIT(0)

/* The interrupt whose register block holds offset @reg, 0x40 to 0x5f. */
static int esp32p4_lp_mailbox_irq_of(hwaddr reg)
{
    return (reg - A_INT_BLOCKS) / INT_BLOCK_SIZE ? ESP32P4_LP_MAILBOX_IRQ_HP
                                                 : ESP32P4_LP_MAILBOX_IRQ_LP;
}

static void esp32p4_lp_mailbox_update(ESP32P4LPMailboxState *s)
{
    int i;

    for (i = 0; i < ESP32P4_LP_MAILBOX_IRQS; i++) {
        qemu_set_irq(s->irq[i], (s->int_raw[i] & s->int_ena[i]) != 0);
    }
}

static uint64_t esp32p4_lp_mailbox_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    ESP32P4LPMailboxState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v;
    int i;

    if (reg < A_INT_BLOCKS) {
        v = s->message[reg / 4];
    } else if (reg < A_REG_CLK_EN) {
        i = esp32p4_lp_mailbox_irq_of(reg);
        switch ((reg - A_INT_BLOCKS) % INT_BLOCK_SIZE) {
        case INT_RAW:
            v = s->int_raw[i];
            break;
        case INT_ST:
            v = s->int_raw[i] & s->int_ena[i];
            break;
        case INT_ENA:
            v = s->int_ena[i];
            break;
        default:
            /* The clear registers are write-only. */
            v = 0;
            break;
        }
    } else if (reg == A_REG_CLK_EN) {
        v = s->clk_en;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of %u bytes at reserved "
                      "offset 0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_MAILBOX,
                      size, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static void esp32p4_lp_mailbox_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size)
{
    ESP32P4LPMailboxState *s = opaque;
    hwaddr reg = addr & ~3;
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t v = ((uint32_t)value << shift) & lanes;
    int i, n;

    if (reg < A_INT_BLOCKS) {
        n = reg / 4;
        s->message[n] = (s->message[n] & ~lanes) | v;
        for (i = 0; i < ESP32P4_LP_MAILBOX_IRQS; i++) {
            s->int_raw[i] |= BIT(n);
        }
        esp32p4_lp_mailbox_update(s);
        return;
    }
    if (reg < A_REG_CLK_EN) {
        i = esp32p4_lp_mailbox_irq_of(reg);
        switch ((reg - A_INT_BLOCKS) % INT_BLOCK_SIZE) {
        case INT_ENA:
            s->int_ena[i] = ((s->int_ena[i] & ~lanes) | v) & INT_MASK;
            break;
        case INT_CLR:
            s->int_raw[i] &= ~(v & INT_MASK);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset "
                          "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_MAILBOX,
                          addr);
            return;
        }
        esp32p4_lp_mailbox_update(s);
        return;
    }
    if (reg == A_REG_CLK_EN) {
        s->clk_en = ((s->clk_en & ~lanes) | v) & REG_CLK_EN;
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes at reserved offset "
                  "0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                  TYPE_ESP32P4_LP_MAILBOX, size, addr, value);
}

static const MemoryRegionOps esp32p4_lp_mailbox_ops = {
    .read = esp32p4_lp_mailbox_read,
    .write = esp32p4_lp_mailbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_lp_mailbox_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPMailboxState *s = ESP32P4_LP_MAILBOX(obj);
    int i;

    memset(s->message, 0, sizeof(s->message));
    for (i = 0; i < ESP32P4_LP_MAILBOX_IRQS; i++) {
        s->int_raw[i] = 0;
        s->int_ena[i] = INT_ENA_RESET;
    }
    s->clk_en = 0;
}

static void esp32p4_lp_mailbox_reset_exit(Object *obj, ResetType type)
{
    esp32p4_lp_mailbox_update(ESP32P4_LP_MAILBOX(obj));
}

static void esp32p4_lp_mailbox_init(Object *obj)
{
    ESP32P4LPMailboxState *s = ESP32P4_LP_MAILBOX(obj);
    int i;

    memory_region_init_io(&s->iomem, obj, &esp32p4_lp_mailbox_ops, s,
                          "esp32p4.lp-mailbox", ESP32P4_LP_MAILBOX_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (i = 0; i < ESP32P4_LP_MAILBOX_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[i]);
    }
}

static const VMStateDescription vmstate_esp32p4_lp_mailbox = {
    .name = TYPE_ESP32P4_LP_MAILBOX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(message, ESP32P4LPMailboxState,
                             ESP32P4_LP_MAILBOX_MESSAGES),
        VMSTATE_UINT32_ARRAY(int_raw, ESP32P4LPMailboxState,
                             ESP32P4_LP_MAILBOX_IRQS),
        VMSTATE_UINT32_ARRAY(int_ena, ESP32P4LPMailboxState,
                             ESP32P4_LP_MAILBOX_IRQS),
        VMSTATE_UINT32(clk_en, ESP32P4LPMailboxState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_lp_mailbox_class_init(ObjectClass *klass,
                                          const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_lp_mailbox_reset_hold;
    rc->phases.exit = esp32p4_lp_mailbox_reset_exit;
    dc->vmsd = &vmstate_esp32p4_lp_mailbox;
}

/* [spec:nuos:req:emu.esp32p4.low-power] */
static const TypeInfo esp32p4_lp_mailbox_types[] = {
    {
        .name          = TYPE_ESP32P4_LP_MAILBOX,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4LPMailboxState),
        .instance_init = esp32p4_lp_mailbox_init,
        .class_init    = esp32p4_lp_mailbox_class_init,
    },
};

DEFINE_TYPES(esp32p4_lp_mailbox_types)
