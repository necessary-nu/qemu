/*
 * ESP32-P4 HP system clock and reset control (HP_SYS_CLKRST)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 11 (Reset and Clock), and ESP-IDF's hp_sys_clkrst_reg.h for v3.
 *
 * Of the block, the two registers that hold and release the HP cores:
 * SOC_CLK_CTRL0, whose CORE0_CPU_CLK_EN and CORE1_CPU_CLK_EN gate each
 * core's clock, and HP_RST_EN0, whose RST_EN_CORE0_GLOBAL and
 * RST_EN_CORE1_GLOBAL hold each core in reset. Core 1 comes out of chip
 * reset with its clock gated and its reset held, and software starts it
 * by enabling the clock and releasing the reset; it then runs from its
 * reset vector. Every bit of both registers reads back what was written,
 * but the other bits' effects, and the rest of the block's registers, are
 * logged as unimplemented, the latter reading as zero.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev.h"
#include "hw/misc/esp32p4_hp_sys_clkrst.h"
#include "migration/vmstate.h"

#define A_SOC_CLK_CTRL0 0x14
#define A_HP_RST_EN0    0xc0

#define SOC_CLK_CTRL0_RESET 0xe6df97afu
#define HP_RST_EN0_RESET    0x00000100u

#define CORE_CPU_CLK_EN(n)    BIT(3 + (n))
#define RST_EN_CORE_GLOBAL(n) BIT(7 + (n))

#define SOC_CLK_CTRL0_MODELLED (CORE_CPU_CLK_EN(0) | CORE_CPU_CLK_EN(1))
#define HP_RST_EN0_MODELLED    (RST_EN_CORE_GLOBAL(0) | \
                                RST_EN_CORE_GLOBAL(1))

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_hp_sys_clkrst_update(ESP32P4HPSysClkRstState *s)
{
    int n;

    for (n = 0; n < ESP32P4_HP_SYS_CLKRST_CORES; n++) {
        qemu_set_irq(s->core_reset[n],
                     !!(s->hp_rst_en0 & RST_EN_CORE_GLOBAL(n)));
        qemu_set_irq(s->core_clock[n],
                     !!(s->soc_clk_ctrl0 & CORE_CPU_CLK_EN(n)));
    }
}

static uint32_t *esp32p4_hp_sys_clkrst_reg(ESP32P4HPSysClkRstState *s,
                                           hwaddr reg, uint32_t *modelled)
{
    switch (reg) {
    case A_SOC_CLK_CTRL0:
        *modelled = SOC_CLK_CTRL0_MODELLED;
        return &s->soc_clk_ctrl0;
    case A_HP_RST_EN0:
        *modelled = HP_RST_EN0_MODELLED;
        return &s->hp_rst_en0;
    default:
        return NULL;
    }
}

static uint64_t esp32p4_hp_sys_clkrst_read(void *opaque, hwaddr addr,
                                           unsigned size)
{
    ESP32P4HPSysClkRstState *s = opaque;
    uint32_t modelled;
    uint32_t *reg = esp32p4_hp_sys_clkrst_reg(s, addr & ~3, &modelled);

    if (!reg) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented read of %u bytes at "
                      "0x%" HWADDR_PRIx "\n", TYPE_ESP32P4_HP_SYS_CLKRST,
                      size, addr);
        return 0;
    }
    return extract32(*reg, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_hp_sys_clkrst_write(void *opaque, hwaddr addr,
                                        uint64_t value, unsigned size)
{
    ESP32P4HPSysClkRstState *s = opaque;
    uint32_t modelled;
    uint32_t *reg = esp32p4_hp_sys_clkrst_reg(s, addr & ~3, &modelled);
    uint32_t v;

    if (!reg) {
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented write of %u bytes at "
                      "0x%" HWADDR_PRIx " (value 0x%" PRIx64 ")\n",
                      TYPE_ESP32P4_HP_SYS_CLKRST, size, addr, value);
        return;
    }
    v = deposit32(*reg, (addr & 3) * 8, size * 8, value);
    if ((v ^ *reg) & ~modelled) {
        qemu_log_mask(LOG_UNIMP, "%s: write to 0x%" HWADDR_PRIx " changes "
                      "bits 0x%08" PRIx32 ", whose effect is not modelled\n",
                      TYPE_ESP32P4_HP_SYS_CLKRST, addr & ~3,
                      (v ^ *reg) & ~modelled);
    }
    *reg = v;
    esp32p4_hp_sys_clkrst_update(s);
}

static const MemoryRegionOps esp32p4_hp_sys_clkrst_ops = {
    .read = esp32p4_hp_sys_clkrst_read,
    .write = esp32p4_hp_sys_clkrst_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.machine] */
static void esp32p4_hp_sys_clkrst_reset_hold(Object *obj, ResetType type)
{
    ESP32P4HPSysClkRstState *s = ESP32P4_HP_SYS_CLKRST(obj);

    s->soc_clk_ctrl0 = SOC_CLK_CTRL0_RESET;
    s->hp_rst_en0 = HP_RST_EN0_RESET;
    esp32p4_hp_sys_clkrst_update(s);
}

static void esp32p4_hp_sys_clkrst_init(Object *obj)
{
    ESP32P4HPSysClkRstState *s = ESP32P4_HP_SYS_CLKRST(obj);

    memory_region_init_io(&s->iomem, obj, &esp32p4_hp_sys_clkrst_ops, s,
                          TYPE_ESP32P4_HP_SYS_CLKRST,
                          ESP32P4_HP_SYS_CLKRST_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), s->core_reset,
                             ESP32P4_HP_SYS_CLKRST_CORE_RESET,
                             ESP32P4_HP_SYS_CLKRST_CORES);
    qdev_init_gpio_out_named(DEVICE(obj), s->core_clock,
                             ESP32P4_HP_SYS_CLKRST_CORE_CLOCK,
                             ESP32P4_HP_SYS_CLKRST_CORES);
}

static const VMStateDescription vmstate_esp32p4_hp_sys_clkrst = {
    .name = TYPE_ESP32P4_HP_SYS_CLKRST,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(soc_clk_ctrl0, ESP32P4HPSysClkRstState),
        VMSTATE_UINT32(hp_rst_en0, ESP32P4HPSysClkRstState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32p4_hp_sys_clkrst_class_init(ObjectClass *klass,
                                             const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_hp_sys_clkrst_reset_hold;
    dc->vmsd = &vmstate_esp32p4_hp_sys_clkrst;
}

/* [spec:nuos:req:emu.esp32p4.machine] */
static const TypeInfo esp32p4_hp_sys_clkrst_info = {
    .name          = TYPE_ESP32P4_HP_SYS_CLKRST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4HPSysClkRstState),
    .instance_init = esp32p4_hp_sys_clkrst_init,
    .class_init    = esp32p4_hp_sys_clkrst_class_init,
};

static void esp32p4_hp_sys_clkrst_register_types(void)
{
    type_register_static(&esp32p4_hp_sys_clkrst_info);
}
type_init(esp32p4_hp_sys_clkrst_register_types)
