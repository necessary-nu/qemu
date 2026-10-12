/*
 * ESP32-P4 MIPI DSI and CSI: D-PHY test codes, HS ranges, packet ECC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), sections
 * 42.5.3 (the test code interface) and tables 42.5-2 and 43.4-4 (the HS
 * frequency ranges); the ECC of MIPI DSI 1.1 section 9.4 and CSI-2, the
 * same code.
 */

#include "qemu/osdep.h"
#include "hw/misc/esp32p4_image_source.h"
#include "hw/misc/esp32p4_mipi.h"

const VMStateDescription vmstate_esp32p4_dphy_test = {
    .name = "esp32p4-dphy-test",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl0, ESP32P4DphyTest),
        VMSTATE_UINT32(ctrl1, ESP32P4DphyTest),
        VMSTATE_UINT8(code, ESP32P4DphyTest),
        VMSTATE_END_OF_LIST()
    },
};

static const struct {
    uint16_t lo;
    uint16_t hi;
    uint8_t code;
} hsfreqrange[] = {
    { 80, 89, 0x00 }, { 90, 99, 0x10 }, { 100, 109, 0x20 },
    { 110, 129, 0x01 }, { 130, 139, 0x11 }, { 140, 149, 0x21 },
    { 150, 169, 0x02 }, { 170, 179, 0x12 }, { 180, 199, 0x22 },
    { 200, 219, 0x03 }, { 220, 239, 0x13 }, { 240, 249, 0x23 },
    { 250, 269, 0x04 }, { 270, 299, 0x14 }, { 300, 329, 0x05 },
    { 330, 359, 0x15 }, { 360, 399, 0x25 }, { 400, 449, 0x06 },
    { 450, 499, 0x16 }, { 500, 549, 0x07 }, { 550, 599, 0x17 },
    { 600, 649, 0x08 }, { 650, 699, 0x18 }, { 700, 749, 0x09 },
    { 750, 799, 0x19 }, { 800, 849, 0x29 }, { 850, 899, 0x39 },
    { 900, 949, 0x0a }, { 950, 999, 0x1a }, { 1000, 1049, 0x2a },
    { 1050, 1099, 0x3a }, { 1100, 1149, 0x0b }, { 1150, 1199, 0x1b },
    { 1200, 1249, 0x2b }, { 1250, 1299, 0x3b }, { 1300, 1349, 0x0c },
    { 1350, 1399, 0x1c }, { 1400, 1449, 0x2c }, { 1450, 1500, 0x3c },
};

bool esp32p4_dphy_hsfreqrange(uint8_t code, uint32_t *lo_mbps,
                              uint32_t *hi_mbps)
{
    for (int i = 0; i < ARRAY_SIZE(hsfreqrange); i++) {
        if (hsfreqrange[i].code == code) {
            *lo_mbps = hsfreqrange[i].lo;
            *hi_mbps = hsfreqrange[i].hi;
            return true;
        }
    }
    return false;
}

int esp32p4_mipi_ecc_check(uint32_t header, uint8_t ecc, uint32_t *fixed)
{
    uint8_t syndrome = (esp32p4_csi2_ecc(header & 0xffffff) ^ ecc) & 0x3f;

    *fixed = header & 0xffffff;
    if (!syndrome) {
        return ESP32P4_MIPI_ECC_OK;
    }
    /* One parity bit wrong: the header is right. */
    if (is_power_of_2(syndrome)) {
        return ESP32P4_MIPI_ECC_CORRECTED;
    }
    /* One header bit wrong: the syndrome is that bit's parity column. */
    for (int i = 0; i < 24; i++) {
        if (esp32p4_csi2_ecc(1u << i) == syndrome) {
            *fixed ^= 1u << i;
            return ESP32P4_MIPI_ECC_CORRECTED;
        }
    }
    return ESP32P4_MIPI_ECC_MULTI;
}
