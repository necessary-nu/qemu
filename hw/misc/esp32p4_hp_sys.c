/*
 * ESP32-P4 HP system registers (HP_SYSTEM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 21 (System Registers), and ESP-IDF's hp_system_reg.h for v3.
 *
 * Of the block, HP_SYSTEM_CPU_INT_FROM_CPU_0_REG to _3_REG (0x10 to
 * 0x1C): the software interrupts through which the HP cores signal each
 * other, CPU_INTR_FROM_CPU_0 to _3, sources 79 to 82 of the interrupt
 * matrix. Each register's bit 0 is read-write and its interrupt is high
 * while it is set; software clears it by writing 0, as ESP-IDF's
 * crosscore_int_ll does. The rest of the block's registers are logged as
 * unimplemented and read as zero.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/esp32p4_hp_sys.h"
#include "migration/vmstate.h"

#define A_CPU_INT_FROM_CPU_0    0x10
#define A_CPU_INT_FROM_CPU_3    0x1c
#define CPU_INT_FROM_CPU        BIT(0)

static bool esp32p4_hp_sys_is_cpu_intr(hwaddr addr)
{
    return addr >= A_CPU_INT_FROM_CPU_0 && addr < A_CPU_INT_FROM_CPU_3 + 4;
}

static uint64_t esp32p4_hp_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4HPSysState *s = opaque;

    if (!esp32p4_hp_sys_is_cpu_intr(addr)) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read of %u bytes at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_HP_SYS, size, addr);
        return 0;
    }
    return extract32(s->cpu_int_from_cpu[(addr - A_CPU_INT_FROM_CPU_0) / 4],
                     (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static void esp32p4_hp_sys_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    ESP32P4HPSysState *s = opaque;
    int n = (addr - A_CPU_INT_FROM_CPU_0) / 4;
    uint32_t *reg;

    if (!esp32p4_hp_sys_is_cpu_intr(addr)) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write of %u bytes at "
                      "0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_HP_SYS, size, addr, value);
        return;
    }
    reg = &s->cpu_int_from_cpu[n];
    *reg = deposit32(*reg, (addr & 3) * 8, size * 8, value) &
           CPU_INT_FROM_CPU;
    qemu_set_irq(s->cpu_intr[n], *reg);
}

static const MemoryRegionOps esp32p4_hp_sys_ops = {
    .read = esp32p4_hp_sys_read,
    .write = esp32p4_hp_sys_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void esp32p4_hp_sys_reset_hold(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    memset(s->cpu_int_from_cpu, 0, sizeof(s->cpu_int_from_cpu));
}

static void esp32p4_hp_sys_reset_exit(Object *obj, ResetType type)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    for (int i = 0; i < ESP32P4_HP_SYS_CPU_INTRS; i++) {
        qemu_set_irq(s->cpu_intr[i], s->cpu_int_from_cpu[i]);
    }
}

static void esp32p4_hp_sys_init(Object *obj)
{
    ESP32P4HPSysState *s = ESP32P4_HP_SYS(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_hp_sys_ops, s,
                          "esp32p4.hp-sys", ESP32P4_HP_SYS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), s->cpu_intr,
                             ESP32P4_HP_SYS_CPU_INTR,
                             ESP32P4_HP_SYS_CPU_INTRS);
}

static const VMStateDescription vmstate_esp32p4_hp_sys = {
    .name = TYPE_ESP32P4_HP_SYS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cpu_int_from_cpu, ESP32P4HPSysState,
                             ESP32P4_HP_SYS_CPU_INTRS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_hp_sys_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_hp_sys_reset_hold;
    rc->phases.exit = esp32p4_hp_sys_reset_exit;
    dc->vmsd = &vmstate_esp32p4_hp_sys;
}

/* [spec:nuos:req:emu.esp32p4.interrupt-matrix] */
static const TypeInfo esp32p4_hp_sys_info = {
    .name          = TYPE_ESP32P4_HP_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4HPSysState),
    .instance_init = esp32p4_hp_sys_init,
    .class_init    = esp32p4_hp_sys_class_init,
};

static void esp32p4_hp_sys_register_types(void)
{
    type_register_static(&esp32p4_hp_sys_info);
}
type_init(esp32p4_hp_sys_register_types)
