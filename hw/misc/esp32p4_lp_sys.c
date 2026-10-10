/*
 * ESP32-P4 LP system registers (LP_SYS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 21 (System Registers), and ESP-IDF's lp_system_reg.h for v3.
 *
 * Of the block:
 *
 * - BOOT_ADDR_HP_CORE1, the word through which software hands HP core 1
 *   its entry point once released: the boot ROM's
 *   ets_set_appcpu_boot_addr() writes it, and the ROM on core 1 jumps to
 *   it, at once if bit 31 is set (to the address with bit 31 clear), or
 *   once it is non-zero after the ROM's own start-up.
 *
 * - The LP AHB matrix's record of the access it refused (21.2.2.11), one
 *   the LP APM did not permit (SECURE 0) or one to an address nothing
 *   decodes (SECURE 1): LP_ADDRHOLE_ADDR and LP_ADDRHOLE_INFO, the master's
 *   ID, whether it was a write, and SECURE. The record is taken as
 *   LP_ADDRHOLE_INT's raw bit rises and holds until software clears the
 *   bit, so it is the first refusal since. The LP APM records its refusals
 *   here; undecoded addresses answer with a bus error but are not
 *   recorded, so the record of the LP core's IBUS and DBUS, which only
 *   such accesses raise, IDBUS_ADDRHOLE_ADDR and _INFO, stays clear.
 *
 * - The interrupt registers INT_RAW, INT_ST, INT_ENA and INT_CLR, whose
 *   enabled raw bits drive LP_SYSREG_INTR, source 19 of the interrupt
 *   matrix. Of the raw bits only LP_ADDRHOLE_INT is ever set: the LP
 *   core's bus timeouts, the ETM's ULP task and the slow clock tick are
 *   not modelled.
 *
 * The rest of the block's registers are logged as unimplemented and read
 * as zero.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "migration/vmstate.h"

#define A_BOOT_ADDR_HP_CORE1    0x164
#define A_LP_ADDRHOLE_ADDR      0x168
#define A_LP_ADDRHOLE_INFO      0x16c
#define A_INT_RAW               0x170
#define A_INT_ST                0x174
#define A_INT_ENA               0x178
#define A_INT_CLR               0x17c
#define A_IDBUS_ADDRHOLE_ADDR   0x198
#define A_IDBUS_ADDRHOLE_INFO   0x19c

/* LP_ADDRHOLE_INT, IDBUS_ADDRHOLE_INT, the three bus timeouts, ETM, tick. */
#define INT_LP_ADDRHOLE         BIT(0)
#define INT_ALL                 0x7f

/* LP_ADDRHOLE_INFO: the master's ID in bits 4:0, then WR and SECURE. */
#define ADDRHOLE_ID_MASK        0x1f
#define ADDRHOLE_WR             BIT(5)
#define ADDRHOLE_SECURE         BIT(6)

static void esp32p4_lp_sys_update(ESP32P4LPSysState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

/* [spec:nuos:req:emu.esp32p4.pms] */
void esp32p4_lp_sys_addrhole(ESP32P4LPSysState *s, uint32_t addr,
                             uint8_t id, bool write, bool illegal)
{
    if (!(s->int_raw & INT_LP_ADDRHOLE)) {
        s->lp_addrhole_addr = addr;
        s->lp_addrhole_info = (id & ADDRHOLE_ID_MASK) |
                              (write ? ADDRHOLE_WR : 0) |
                              (illegal ? ADDRHOLE_SECURE : 0);
        s->int_raw |= INT_LP_ADDRHOLE;
    }
    esp32p4_lp_sys_update(s);
}

static bool esp32p4_lp_sys_reg(ESP32P4LPSysState *s, hwaddr addr,
                               uint32_t *v)
{
    switch (addr) {
    case A_BOOT_ADDR_HP_CORE1:
        *v = s->boot_addr_hp_core1;
        return true;
    case A_LP_ADDRHOLE_ADDR:
        *v = s->lp_addrhole_addr;
        return true;
    case A_LP_ADDRHOLE_INFO:
        *v = s->lp_addrhole_info;
        return true;
    case A_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_INT_CLR:
    case A_IDBUS_ADDRHOLE_ADDR:
    case A_IDBUS_ADDRHOLE_INFO:
        *v = 0;
        return true;
    default:
        return false;
    }
}

static uint64_t esp32p4_lp_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LPSysState *s = opaque;
    uint32_t v;

    if (!esp32p4_lp_sys_reg(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read of %u bytes at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_SYS, size, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
/* [spec:nuos:req:emu.esp32p4.pms] */
static void esp32p4_lp_sys_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ESP32P4LPSysState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v;

    switch (reg) {
    case A_BOOT_ADDR_HP_CORE1:
        s->boot_addr_hp_core1 = deposit32(s->boot_addr_hp_core1, shift,
                                          size * 8, value);
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        /* INT_RAW is write-1-to-clear, as INT_CLR is. */
        v = deposit32(0, shift, size * 8, value);
        s->int_raw &= ~(v & INT_ALL);
        esp32p4_lp_sys_update(s);
        break;
    case A_INT_ENA:
        s->int_ena = deposit32(s->int_ena, shift, size * 8, value) & INT_ALL;
        esp32p4_lp_sys_update(s);
        break;
    case A_LP_ADDRHOLE_ADDR:
    case A_LP_ADDRHOLE_INFO:
    case A_INT_ST:
    case A_IDBUS_ADDRHOLE_ADDR:
    case A_IDBUS_ADDRHOLE_INFO:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_SYS, addr);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write of %u bytes at "
                      "0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_LP_SYS, size, addr, value);
        break;
    }
}

static const MemoryRegionOps esp32p4_lp_sys_ops = {
    .read = esp32p4_lp_sys_read,
    .write = esp32p4_lp_sys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_lp_sys_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPSysState *s = ESP32P4_LP_SYS(obj);

    s->boot_addr_hp_core1 = 0;
    s->lp_addrhole_addr = 0;
    s->lp_addrhole_info = 0;
    s->int_raw = 0;
    s->int_ena = 0;
}

static void esp32p4_lp_sys_reset_exit(Object *obj, ResetType type)
{
    esp32p4_lp_sys_update(ESP32P4_LP_SYS(obj));
}

static void esp32p4_lp_sys_init(Object *obj)
{
    ESP32P4LPSysState *s = ESP32P4_LP_SYS(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_lp_sys_ops, s,
                          TYPE_ESP32P4_LP_SYS, ESP32P4_LP_SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_esp32p4_lp_sys = {
    .name = TYPE_ESP32P4_LP_SYS,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(boot_addr_hp_core1, ESP32P4LPSysState),
        VMSTATE_UINT32(lp_addrhole_addr, ESP32P4LPSysState),
        VMSTATE_UINT32(lp_addrhole_info, ESP32P4LPSysState),
        VMSTATE_UINT32(int_raw, ESP32P4LPSysState),
        VMSTATE_UINT32(int_ena, ESP32P4LPSysState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_lp_sys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_lp_sys_reset_hold;
    rc->phases.exit = esp32p4_lp_sys_reset_exit;
    dc->vmsd = &vmstate_esp32p4_lp_sys;
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static const TypeInfo esp32p4_lp_sys_info = {
    .name          = TYPE_ESP32P4_LP_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4LPSysState),
    .instance_init = esp32p4_lp_sys_init,
    .class_init    = esp32p4_lp_sys_class_init,
};

static void esp32p4_lp_sys_register_types(void)
{
    type_register_static(&esp32p4_lp_sys_info);
}
type_init(esp32p4_lp_sys_register_types)
