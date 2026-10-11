/*
 * ESP32-P4 HUK Generator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_HUK_H
#define HW_MISC_ESP32P4_HUK_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_crypto.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qom/object.h"

#define TYPE_ESP32P4_HUK "esp32p4-huk"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4HukState, ESP32P4_HUK)

#define ESP32P4_HUK_BASE        0x50114000
#define ESP32P4_HUK_SIZE        0x1000
/* huk_info, 165 words (TRM 36.7.1, and the boot ROM's HUK_INFO_LEN). */
#define ESP32P4_HUK_INFO_BYTES  660

/*
 * The clock input "clk" is LP_DYN_FAST_CLK, which its registers in the
 * always-on domain are behind; LP_CLKRST_RST_EN_LP_HUK resets it. The
 * "efuse" link gives EFUSE_KM_HUK_GEN_STATE and the chip's identity, from
 * which, with the "puf-seed" property, the SRAM PUF's response is
 * derived. Its interrupt, HUK_INTR (LP_HUK_INTR), is sysbus IRQ 0.
 */
struct ESP32P4HukState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4EfuseState *efuse;
    ESP32P4CryptoTimer op;
    uint64_t puf_seed;

    uint32_t clk_reg;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf;
    uint32_t state;
    uint32_t status;
    uint32_t date;
    uint8_t info[ESP32P4_HUK_INFO_BYTES];

    /* The HUK, while HUK_STATUS is 1. */
    uint8_t huk[32];
    /* The phase in progress, and whether it generates or recovers. */
    uint32_t phase;
    bool generate;
};

/* The HUK, for the Key Manager; false while there is no valid HUK. */
bool esp32p4_huk_key(ESP32P4HukState *s, uint8_t key[32]);

#endif
