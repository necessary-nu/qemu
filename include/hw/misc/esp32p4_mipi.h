/*
 * ESP32-P4 MIPI DSI and CSI: D-PHY test codes, HS ranges, packet ECC
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_MIPI_H
#define HW_MISC_ESP32P4_MIPI_H

#include "qemu/osdep.h"
#include "migration/vmstate.h"

/*
 * The test code interface
 * =======================
 *
 * Both of the P4's D-PHYs, MIPI DSI's transmitter and MIPI CSI's receiver,
 * are configured through the test code interface their host controller's
 * PHY_TEST_CTRL0/1 (DSI: PHY_TST_CTRL0/1) registers drive (TRM 42.5.3.1,
 * which chapter 43 refers to): TESTCLR (CTRL0 bit 0) and TESTCLK (bit 1);
 * TESTDIN (CTRL1 bits 7:0), TESTEN (bit 16) and the read-only TESTDOUT
 * (bits 15:8). With TESTEN set, TESTCLK's falling edge latches TESTDIN as
 * the test code; with it clear, TESTCLK's rising edge writes TESTDIN to
 * the latched code as its test data. TESTCLR, while set, holds the
 * interface in reset: the test code is 0 (normal operation) and the PHY's
 * test registers take their defaults; edges are ignored. TESTDOUT shows,
 * at once, what the latched code gives for the TESTDIN on the bus.
 */
typedef struct ESP32P4DphyTest {
    uint32_t ctrl0;
    uint32_t ctrl1;
    uint8_t code;
} ESP32P4DphyTest;

#define ESP32P4_DPHY_TESTCLR        (1u << 0)
#define ESP32P4_DPHY_TESTCLK        (1u << 1)
#define ESP32P4_DPHY_TESTEN         (1u << 16)
#define ESP32P4_DPHY_CTRL0_RESET    ESP32P4_DPHY_TESTCLR

typedef enum ESP32P4DphyTestEvent {
    ESP32P4_DPHY_TEST_NONE,
    /* TESTCLR rose: the PHY's test registers take their defaults. */
    ESP32P4_DPHY_TEST_CLEAR,
    ESP32P4_DPHY_TEST_CODE,
    /* Test data, *@data, was written to the latched code. */
    ESP32P4_DPHY_TEST_DATA,
} ESP32P4DphyTestEvent;

static inline void esp32p4_dphy_test_reset(ESP32P4DphyTest *t)
{
    t->ctrl0 = ESP32P4_DPHY_CTRL0_RESET;
    t->ctrl1 = 0;
    t->code = 0;
}

/* A write of @v to CTRL0 (@ctrl1 false) or CTRL1, and what it did. */
static inline ESP32P4DphyTestEvent esp32p4_dphy_test_write(
    ESP32P4DphyTest *t, bool ctrl1, uint32_t v, uint8_t *data)
{
    uint32_t old = t->ctrl0;

    if (ctrl1) {
        t->ctrl1 = v & (ESP32P4_DPHY_TESTEN | 0xff);
        return ESP32P4_DPHY_TEST_NONE;
    }
    t->ctrl0 = v & (ESP32P4_DPHY_TESTCLR | ESP32P4_DPHY_TESTCLK);
    if (t->ctrl0 & ESP32P4_DPHY_TESTCLR) {
        t->code = 0;
        return (old & ESP32P4_DPHY_TESTCLR) ? ESP32P4_DPHY_TEST_NONE
                                            : ESP32P4_DPHY_TEST_CLEAR;
    }
    if ((old & ESP32P4_DPHY_TESTCLK) && !(t->ctrl0 & ESP32P4_DPHY_TESTCLK) &&
        (t->ctrl1 & ESP32P4_DPHY_TESTEN)) {
        t->code = t->ctrl1 & 0xff;
        return ESP32P4_DPHY_TEST_CODE;
    }
    if (!(old & ESP32P4_DPHY_TESTCLK) && (t->ctrl0 & ESP32P4_DPHY_TESTCLK) &&
        !(t->ctrl1 & ESP32P4_DPHY_TESTEN)) {
        *data = t->ctrl1 & 0xff;
        return ESP32P4_DPHY_TEST_DATA;
    }
    return ESP32P4_DPHY_TEST_NONE;
}

/* CTRL1 as read, with @dout in TESTDOUT. */
static inline uint32_t esp32p4_dphy_test_ctrl1(const ESP32P4DphyTest *t,
                                               uint8_t dout)
{
    return t->ctrl1 | (uint32_t)dout << 8;
}

static inline uint8_t esp32p4_dphy_test_din(const ESP32P4DphyTest *t)
{
    return t->ctrl1 & 0xff;
}

extern const VMStateDescription vmstate_esp32p4_dphy_test;

#define VMSTATE_ESP32P4_DPHY_TEST(field, state) \
    VMSTATE_STRUCT(field, state, 0, vmstate_esp32p4_dphy_test, \
                   ESP32P4DphyTest)

/*
 * The HS frequency ranges both PHYs take in test code 0x44's
 * hsfreqrange[5:0] (TRM tables 42.5-2 and 43.4-4, which agree): the
 * range's lowest and highest lane rates in Mbit/s, or false for a code
 * neither table has.
 */
bool esp32p4_dphy_hsfreqrange(uint8_t code, uint32_t *lo_mbps,
                              uint32_t *hi_mbps);

/*
 * A received packet header's ECC (DSI and CSI-2 alike): the 24 header
 * bits @header (the data identifier in bits 7:0, then the word count)
 * against the ECC byte @ecc's six parity bits. ESP32P4_MIPI_ECC_OK, or
 * _CORRECTED with the one wrong bit (of the header or the ECC) put right
 * in *@fixed, or _MULTI for two or more, which the code cannot correct.
 */
enum {
    ESP32P4_MIPI_ECC_OK,
    ESP32P4_MIPI_ECC_CORRECTED,
    ESP32P4_MIPI_ECC_MULTI,
};
int esp32p4_mipi_ecc_check(uint32_t header, uint8_t ecc, uint32_t *fixed);

#endif
