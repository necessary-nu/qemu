/*
 * ESP32-P4 analog voltage comparators
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 66 (Analog Voltage Comparator) and 21.2.2.10; ESP-IDF's
 * lp_system_reg.h for v3 (LP_SYSTEM_REG_PAD_COMPn) and its hal
 * ana_cmpr_ll.h and ana_cmpr_periph.c.
 *
 * Comparator n compares its source pad, PAD1 (GPIO52 + 2n), with its
 * reference: with MODE_COMPn the voltage on PAD0 (GPIO51 + 2n), else the
 * internal reference, DREF_COMPn tenths of VDD_IO_6 (0 to 0.7 VDD_IO_6).
 * COMP_OUT is high while the source is above the reference (TRM 66.3),
 * low while the comparator is powered down (XPD_COMPn clear). GPIO_EXT
 * takes its edges as the interrupts GPIO_COMPn_POS_INT, _NEG_INT and
 * _ALL_INT and the ETM events GPIO_EVT_ZERO_DET_POSn and _NEGn (TRM
 * 66.5, 66.6), through its ZERO_DET filter (hw/gpio/esp32p4_gpio.c).
 *
 * The pads' voltages are the ADC's properties for ADC2's channels 2 to 5
 * (adc2-ch2-mv to adc2-ch5-mv, GPIO51 to GPIO54), or the supply or ground
 * where a digital driver holds a pad. The comparators compare again
 * whenever their configuration, a pad's voltage, or a pad's digital drive
 * changes, at that moment.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_adc.h"
#include "hw/misc/esp32p4_ana_cmpr.h"
#include "hw/misc/esp32p4_lp_sys.h"
#include "migration/vmstate.h"

/* LP_SYSTEM_REG_PAD_COMPn */
#define A_PAD_COMP(n)           (0x148 + 4 * (n))
#define PAD_COMP_DREF(v)        extract32((v), 0, 3)
#define PAD_COMP_MODE           BIT(3)
#define PAD_COMP_XPD            BIT(4)

/* [spec:nuos:req:emu.esp32p4.analog] */
static void ana_cmpr_compare(ESP32P4AnaCmprState *s, unsigned n)
{
    uint32_t cfg = esp32p4_lp_sys_reg(s->lp_sys, A_PAD_COMP(n));
    unsigned pad0 = ESP32P4_ANA_CMPR_PAD0 + 2 * n;
    uint64_t ref_uv, src_uv;
    bool level = false;

    if (cfg & PAD_COMP_XPD) {
        if (cfg & PAD_COMP_MODE) {
            ref_uv = (uint64_t)esp32p4_adc_pad_mv(s->adc, pad0) * 1000;
        } else {
            ref_uv = (uint64_t)PAD_COMP_DREF(cfg) * s->vdd_io_mv * 100;
        }
        src_uv = (uint64_t)esp32p4_adc_pad_mv(s->adc, pad0 + 1) * 1000;
        level = src_uv > ref_uv;
    }
    if (level != s->level[n]) {
        s->level[n] = level;
        qemu_set_irq(s->out[n], level);
    }
}

static void ana_cmpr_compare_all(ESP32P4AnaCmprState *s)
{
    for (unsigned n = 0; n < ESP32P4_ANA_CMPRS; n++) {
        ana_cmpr_compare(s, n);
    }
}

static void ana_cmpr_config(void *opaque, int n, int level)
{
    if (level) {
        ana_cmpr_compare(opaque, n);
    }
}

static void ana_cmpr_pad_change(void *opaque, int n, int level)
{
    if (level) {
        ana_cmpr_compare_all(opaque);
    }
}

/* A digital driver has taken or left pad 51 + n / 2. */
static void ana_cmpr_pad(void *opaque, int n, int level)
{
    ana_cmpr_compare(opaque, n / 4);
}

static void esp32p4_ana_cmpr_reset_hold(Object *obj, ResetType type)
{
    ESP32P4AnaCmprState *s = ESP32P4_ANA_CMPR(obj);

    for (unsigned n = 0; n < ESP32P4_ANA_CMPRS; n++) {
        s->level[n] = false;
    }
}

static void esp32p4_ana_cmpr_reset_exit(Object *obj, ResetType type)
{
    ESP32P4AnaCmprState *s = ESP32P4_ANA_CMPR(obj);

    for (unsigned n = 0; n < ESP32P4_ANA_CMPRS; n++) {
        qemu_set_irq(s->out[n], 0);
    }
    ana_cmpr_compare_all(s);
}

static void esp32p4_ana_cmpr_init(Object *obj)
{
    ESP32P4AnaCmprState *s = ESP32P4_ANA_CMPR(obj);
    DeviceState *dev = DEVICE(obj);

    qdev_init_gpio_in_named(dev, ana_cmpr_config, ESP32P4_ANA_CMPR_CONFIG,
                            ESP32P4_ANA_CMPRS);
    qdev_init_gpio_in_named(dev, ana_cmpr_pad_change,
                            ESP32P4_ANA_CMPR_PAD_CHANGE, 1);
    qdev_init_gpio_in_named(dev, ana_cmpr_pad, ESP32P4_ANA_CMPR_PAD,
                            ESP32P4_ANA_CMPR_PAD_LINES);
    qdev_init_gpio_out_named(dev, s->out, ESP32P4_ANA_CMPR_OUT,
                             ESP32P4_ANA_CMPRS);
}

static void esp32p4_ana_cmpr_realize(DeviceState *dev, Error **errp)
{
    ESP32P4AnaCmprState *s = ESP32P4_ANA_CMPR(dev);

    if (!s->adc || !s->lp_sys) {
        error_setg(errp, "%s: the adc and lp-sys links must be set",
                   TYPE_ESP32P4_ANA_CMPR);
    }
}

static const VMStateDescription vmstate_esp32p4_ana_cmpr = {
    .name = TYPE_ESP32P4_ANA_CMPR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL_ARRAY(level, ESP32P4AnaCmprState, ESP32P4_ANA_CMPRS),
        VMSTATE_END_OF_LIST()
    }
};

static const Property esp32p4_ana_cmpr_properties[] = {
    DEFINE_PROP_LINK("adc", ESP32P4AnaCmprState, adc, TYPE_ESP32P4_ADC,
                     ESP32P4AdcState *),
    DEFINE_PROP_LINK("lp-sys", ESP32P4AnaCmprState, lp_sys,
                     TYPE_ESP32P4_LP_SYS, ESP32P4LPSysState *),
    DEFINE_PROP_UINT32("vdd-io-mv", ESP32P4AnaCmprState, vdd_io_mv, 3300),
};

static void esp32p4_ana_cmpr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32p4_ana_cmpr_reset_hold;
    rc->phases.exit = esp32p4_ana_cmpr_reset_exit;
    dc->realize = esp32p4_ana_cmpr_realize;
    dc->vmsd = &vmstate_esp32p4_ana_cmpr;
    device_class_set_props(dc, esp32p4_ana_cmpr_properties);
}

/* [spec:nuos:req:emu.esp32p4.analog] */
static const TypeInfo esp32p4_ana_cmpr_info = {
    .name = TYPE_ESP32P4_ANA_CMPR,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4AnaCmprState),
    .instance_init = esp32p4_ana_cmpr_init,
    .class_init = esp32p4_ana_cmpr_class_init,
};

static void esp32p4_ana_cmpr_register_types(void)
{
    type_register_static(&esp32p4_ana_cmpr_info);
}

type_init(esp32p4_ana_cmpr_register_types)
