/*
 * ESP32 APB control registers (SYSCON)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM, "Reset and Clock", and ESP-IDF's
 * soc/esp32/register/soc/apb_ctrl_reg.h for the reset values. SYSCON holds
 * the CPU_CLK pre-divider used with XTAL_CLK and RC_FAST_CLK and the four
 * REF_TICK dividers, one per CPU_CLK source; the SoC derives the clock tree
 * from them. The SAR ADC control words also live here: they are plain
 * storage, as the SAR ADCs are not modelled.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_apb_ctrl.h"
#include "hw/misc/esp32_rtc_cntl.h"

#define ESP32_APB_CTRL_SIZE 0x1000

/*
 * Not a register of the chip, which reads 0 here: Espressif's QEMU puts
 * "QEMU" at this reserved word so that ESP-IDF can tell it is emulated.
 */
#define ESP32_APB_CTRL_QEMU_MARKER_VAL 0x51454d55

/* SYSCON_DATE with bit 31 set, which ESP-IDF reads as ECO3 silicon. */
#define ESP32_APB_CTRL_DATE_RESET 0x96042000

static const uint32_t esp32_apb_ctrl_reset[ESP32_APB_CTRL_NREGS] = {
    [R_APB_CTRL_SYSCLK_CONF] = R_APB_CTRL_SYSCLK_CONF_QUICK_CLK_CHNG_MASK,
    [R_APB_CTRL_XTAL_TICK_CONF] = 39,
    [R_APB_CTRL_PLL_TICK_CONF] = 79,
    [R_APB_CTRL_CK8M_TICK_CONF] = 11,
    [R_APB_CTRL_SARADC_CTRL] = (15 << 19) | (15 << 15) | (4 << 7) | (1 << 6),
    [R_APB_CTRL_SARADC_CTRL2] = 255 << 1,
    [R_APB_CTRL_SARADC_FSM] = (2 << 24) | (8 << 16) | (255 << 8) | 8,
    [R_APB_CTRL_SARADC_SAR1_PATT_TAB1 ... R_APB_CTRL_SARADC_SAR2_PATT_TAB4] =
        0x0f0f0f0f,
    [R_APB_CTRL_APLL_TICK_CONF] = 99,
    [R_APB_CTRL_QEMU_MARKER] = ESP32_APB_CTRL_QEMU_MARKER_VAL,
    [R_APB_CTRL_DATE] = ESP32_APB_CTRL_DATE_RESET,
};

/* Implemented bits of each register; reserved bits read 0. */
static const uint32_t esp32_apb_ctrl_mask[ESP32_APB_CTRL_NREGS] = {
    [R_APB_CTRL_SYSCLK_CONF] = 0x3fff,
    [R_APB_CTRL_XTAL_TICK_CONF] = ESP32_APB_CTRL_TICK_NUM_MASK,
    [R_APB_CTRL_PLL_TICK_CONF] = ESP32_APB_CTRL_TICK_NUM_MASK,
    [R_APB_CTRL_CK8M_TICK_CONF] = ESP32_APB_CTRL_TICK_NUM_MASK,
    [R_APB_CTRL_SARADC_CTRL] = 0x07ffffff,
    [R_APB_CTRL_SARADC_CTRL2] = 0x7ff,
    [R_APB_CTRL_SARADC_FSM] = 0xffffffff,
    [R_APB_CTRL_SARADC_SAR1_PATT_TAB1 ... R_APB_CTRL_SARADC_SAR2_PATT_TAB4] =
        0xffffffff,
    [R_APB_CTRL_APLL_TICK_CONF] = ESP32_APB_CTRL_TICK_NUM_MASK,
    [R_APB_CTRL_DATE] = 0xffffffff,
};

/* [spec:nuos:req:emu.esp32.clock-gating] */
uint32_t esp32_apb_ctrl_pre_div(Esp32ApbCtrlState *s)
{
    return FIELD_EX32(s->regs[R_APB_CTRL_SYSCLK_CONF], APB_CTRL_SYSCLK_CONF,
                      PRE_DIV_CNT) + 1;
}

/* [spec:nuos:req:emu.esp32.clock-gating] */
uint32_t esp32_apb_ctrl_tick_div(Esp32ApbCtrlState *s, unsigned soc_clk_sel)
{
    static const unsigned reg[] = {
        [ESP32_SOC_CLK_XTAL] = R_APB_CTRL_XTAL_TICK_CONF,
        [ESP32_SOC_CLK_PLL] = R_APB_CTRL_PLL_TICK_CONF,
        [ESP32_SOC_CLK_8M] = R_APB_CTRL_CK8M_TICK_CONF,
        [ESP32_SOC_CLK_APLL] = R_APB_CTRL_APLL_TICK_CONF,
    };

    assert(soc_clk_sel < ARRAY_SIZE(reg));
    return (s->regs[reg[soc_clk_sel]] & ESP32_APB_CTRL_TICK_NUM_MASK) + 1;
}

static uint64_t esp32_apb_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32ApbCtrlState *s = ESP32_APB_CTRL(opaque);

    if (addr >= sizeof(s->regs)) {
        return 0;
    }
    return s->regs[addr / 4];
}

static void esp32_apb_ctrl_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    Esp32ApbCtrlState *s = ESP32_APB_CTRL(opaque);
    unsigned idx = addr / 4;

    if (addr >= sizeof(s->regs) || !esp32_apb_ctrl_mask[idx]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to reserved offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return;
    }
    s->regs[idx] = value & esp32_apb_ctrl_mask[idx];

    switch (idx) {
    case R_APB_CTRL_SYSCLK_CONF:
    case R_APB_CTRL_XTAL_TICK_CONF:
    case R_APB_CTRL_PLL_TICK_CONF:
    case R_APB_CTRL_CK8M_TICK_CONF:
    case R_APB_CTRL_APLL_TICK_CONF:
        qemu_irq_pulse(s->clk_update);
        break;
    case R_APB_CTRL_SARADC_CTRL:
        if (value & (R_APB_CTRL_SARADC_CTRL_START_MASK |
                     R_APB_CTRL_SARADC_CTRL_START_FORCE_MASK)) {
            qemu_log_mask(LOG_UNIMP, "%s: SAR ADC not modelled\n", __func__);
        }
        break;
    }
}

static const MemoryRegionOps esp32_apb_ctrl_ops = {
    .read = esp32_apb_ctrl_read,
    .write = esp32_apb_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32_apb_ctrl_reset_hold(Object *obj, ResetType type)
{
    Esp32ApbCtrlState *s = ESP32_APB_CTRL(obj);

    memcpy(s->regs, esp32_apb_ctrl_reset, sizeof(s->regs));
}

static void esp32_apb_ctrl_init(Object *obj)
{
    Esp32ApbCtrlState *s = ESP32_APB_CTRL(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_apb_ctrl_ops, s,
                          TYPE_ESP32_APB_CTRL, ESP32_APB_CTRL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->clk_update,
                             ESP32_APB_CTRL_CLK_UPDATE_GPIO, 1);
}

static const VMStateDescription vmstate_esp32_apb_ctrl = {
    .name = TYPE_ESP32_APB_CTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Esp32ApbCtrlState, ESP32_APB_CTRL_NREGS),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32_apb_ctrl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_apb_ctrl_reset_hold;
    dc->vmsd = &vmstate_esp32_apb_ctrl;
}

/* [spec:nuos:req:emu.esp32.clock-gating] */
static const TypeInfo esp32_apb_ctrl_info = {
    .name = TYPE_ESP32_APB_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32ApbCtrlState),
    .instance_init = esp32_apb_ctrl_init,
    .class_init = esp32_apb_ctrl_class_init,
};

static void esp32_apb_ctrl_register_types(void)
{
    type_register_static(&esp32_apb_ctrl_info);
}

type_init(esp32_apb_ctrl_register_types)
