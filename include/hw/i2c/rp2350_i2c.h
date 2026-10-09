/*
 * RP2350 I2C controllers (I2C0, I2C1)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Each is a DW_apb_i2c v2.03a (TYPE_DESIGNWARE_I2C) in the RP2350's
 * configuration: fast mode plus at most, 10-bit addressing, 16-entry
 * FIFOs, DMA handshaking and IC_COMP_PARAM_1 not implemented.
 */

#ifndef HW_I2C_RP2350_I2C_H
#define HW_I2C_RP2350_I2C_H

#include "hw/i2c/designware_i2c.h"

#define TYPE_RP2350_I2C "rp2350-i2c"

#endif
