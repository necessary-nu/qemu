/*
 * RP2350 I2C controllers (I2C0, I2C1)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reset values and parameters are those of the pico-sdk hardware_regs
 * i2c.h and the RP2350 Datasheet, section 12.2.
 */

#include "qemu/osdep.h"
#include "hw/i2c/rp2350_i2c.h"

/* [spec:nuos:req:emu.i2c] */
static void rp2350_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    DesignWareI2CClass *dwc = DESIGNWARE_I2C_CLASS(klass);

    dc->desc = "RP2350 I2C controller (DW_apb_i2c)";
    /* Master, fast mode, RESTART enabled, slave disabled. */
    dwc->con_reset = 0x65;
    dwc->tar_reset = 0x55;
    dwc->tar_mask = 0xfff;
    dwc->sar_reset = 0x55;
    dwc->ss_scl_hcnt_reset = 0x28;
    dwc->ss_scl_lcnt_reset = 0x2f;
    dwc->fs_scl_hcnt_reset = 0x6;
    dwc->fs_scl_lcnt_reset = 0xd;
    dwc->fs_spklen_reset = 0x7;
    dwc->max_speed = 2;
    dwc->comp_param_1 = 0;
    dwc->comp_version = 0x3230312a;
    dwc->has_smbus_intr_mask = false;
}

static const TypeInfo rp2350_i2c_types[] = {
    {
        .name = TYPE_RP2350_I2C,
        .parent = TYPE_DESIGNWARE_I2C,
        .class_init = rp2350_i2c_class_init,
    },
};

DEFINE_TYPES(rp2350_i2c_types)
