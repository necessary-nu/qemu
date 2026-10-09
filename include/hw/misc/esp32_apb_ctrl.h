/*
 * ESP32 APB control registers (SYSCON)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32_APB_CTRL_H
#define HW_MISC_ESP32_APB_CTRL_H

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"

#define TYPE_ESP32_APB_CTRL "misc.esp32.apb_ctrl"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32ApbCtrlState, ESP32_APB_CTRL)

/* Pulsed whenever a register that shapes CPU_CLK, APB_CLK or REF_TICK is
 * written. */
#define ESP32_APB_CTRL_CLK_UPDATE_GPIO "clk-update"
/* Pulsed whenever SARADC_CTRL is written, for the SAR ADC DIG controllers */
#define ESP32_APB_CTRL_SARADC_CTRL_GPIO "saradc-ctrl"

REG32(APB_CTRL_SYSCLK_CONF, 0x00)
    FIELD(APB_CTRL_SYSCLK_CONF, PRE_DIV_CNT, 0, 10)
    FIELD(APB_CTRL_SYSCLK_CONF, CLK_320M_EN, 10, 1)
    FIELD(APB_CTRL_SYSCLK_CONF, CLK_EN, 11, 1)
    FIELD(APB_CTRL_SYSCLK_CONF, RST_TICK_CNT, 12, 1)
    FIELD(APB_CTRL_SYSCLK_CONF, QUICK_CLK_CHNG, 13, 1)
REG32(APB_CTRL_XTAL_TICK_CONF, 0x04)
REG32(APB_CTRL_PLL_TICK_CONF, 0x08)
REG32(APB_CTRL_CK8M_TICK_CONF, 0x0c)
REG32(APB_CTRL_SARADC_CTRL, 0x10)
    FIELD(APB_CTRL_SARADC_CTRL, START_FORCE, 0, 1)
    FIELD(APB_CTRL_SARADC_CTRL, START, 1, 1)
REG32(APB_CTRL_SARADC_CTRL2, 0x14)
REG32(APB_CTRL_SARADC_FSM, 0x18)
REG32(APB_CTRL_SARADC_SAR1_PATT_TAB1, 0x1c)
REG32(APB_CTRL_SARADC_SAR2_PATT_TAB1, 0x2c)
REG32(APB_CTRL_SARADC_SAR2_PATT_TAB4, 0x38)
REG32(APB_CTRL_APLL_TICK_CONF, 0x3c)
REG32(APB_CTRL_QEMU_MARKER, 0x78)
REG32(APB_CTRL_DATE, 0x7c)

#define ESP32_APB_CTRL_TICK_NUM_MASK 0xff
#define ESP32_APB_CTRL_NREGS (A_APB_CTRL_DATE / 4 + 1)

struct Esp32ApbCtrlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq clk_update;
    qemu_irq saradc_ctrl;

    uint32_t regs[ESP32_APB_CTRL_NREGS];
};

/* CPU_CLK divider applied to XTAL_CLK and RC_FAST_CLK: PRE_DIV_CNT + 1. */
uint32_t esp32_apb_ctrl_pre_div(Esp32ApbCtrlState *s);

/*
 * REF_TICK divider for the given RTC_CNTL_SOC_CLK_SEL source: the
 * matching *_TICK_NUM + 1.
 */
uint32_t esp32_apb_ctrl_tick_div(Esp32ApbCtrlState *s, unsigned soc_clk_sel);

#endif
