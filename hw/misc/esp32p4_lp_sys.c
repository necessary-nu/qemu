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
 * Of the block, BOOT_ADDR_HP_CORE1, the word through which software hands
 * HP core 1 its entry point once released: the boot ROM's
 * ets_set_appcpu_boot_addr() writes it, and the ROM on core 1 jumps to
 * it, at once if bit 31 is set (to the address with bit 31 clear), or
 * once it is non-zero after the ROM's own start-up. The rest of the
 * block's registers are logged as unimplemented and read as zero.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "migration/vmstate.h"

#define A_BOOT_ADDR_HP_CORE1 0x164

static uint64_t esp32p4_lp_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LPSysState *s = opaque;

    if ((addr & ~3) != A_BOOT_ADDR_HP_CORE1) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read of %u bytes at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_LP_SYS, size, addr);
        return 0;
    }
    return extract32(s->boot_addr_hp_core1, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_lp_sys_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ESP32P4LPSysState *s = opaque;

    if ((addr & ~3) != A_BOOT_ADDR_HP_CORE1) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write of %u bytes at "
                      "0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_LP_SYS, size, addr, value);
        return;
    }
    s->boot_addr_hp_core1 = deposit32(s->boot_addr_hp_core1,
                                      (addr & 3) * 8, size * 8, value);
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
    ESP32P4_LP_SYS(obj)->boot_addr_hp_core1 = 0;
}

static void esp32p4_lp_sys_init(Object *obj)
{
    ESP32P4LPSysState *s = ESP32P4_LP_SYS(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_lp_sys_ops, s,
                          TYPE_ESP32P4_LP_SYS, ESP32P4_LP_SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_esp32p4_lp_sys = {
    .name = TYPE_ESP32P4_LP_SYS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(boot_addr_hp_core1, ESP32P4LPSysState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_lp_sys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_lp_sys_reset_hold;
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
