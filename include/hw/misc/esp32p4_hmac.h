/*
 * ESP32-P4 HMAC accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_HMAC_H
#define HW_MISC_ESP32P4_HMAC_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_crypto.h"
#include "hw/misc/esp32p4_keymng.h"
#include "hw/misc/esp32p4_sha.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qom/object.h"

#define TYPE_ESP32P4_HMAC "esp32p4-hmac"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4HmacState, ESP32P4_HMAC)

#define ESP32P4_HMAC_BASE   0x50095000
#define ESP32P4_HMAC_SIZE   0x1000

/*
 * The clock input "clk" is CRYPTO_HMAC_CLK; the registers are behind
 * CRYPTO_SYS_CLK. The named GPIO input "reset" is RST_EN_HMAC. The links
 * "efuse", "keymng" and "sha" are the key sources and the SHA accelerator
 * it occupies while it computes.
 *
 * Named GPIO outputs: "ds-key", high while the RSA_DS key the downstream
 * mode derived is valid, to the RSA_DS peripheral; "jtag-enable", high
 * while a JTAG re-enable token has matched (TRM 29.2.2).
 */
#define ESP32P4_HMAC_RESET          "reset"
#define ESP32P4_HMAC_DS_KEY         "ds-key"
#define ESP32P4_HMAC_JTAG_ENABLE    "jtag-enable"

struct ESP32P4HmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;
    ESP32P4EfuseState *efuse;
    ESP32P4KeyMngState *keymng;
    ESP32P4ShaDevState *sha;
    ESP32P4CryptoTimer op;
    qemu_irq ds_key_out;
    qemu_irq jtag_out;
    bool in_reset;

    uint32_t purpose;
    uint32_t key_sel;
    uint8_t msg[64];
    uint8_t result[32];
    bool busy;
    bool query_error;
    uint32_t date;

    /* The session (see esp32p4_hmac.c): its phase and SHA-256 states. */
    uint32_t phase;
    uint32_t pending;
    uint8_t k0[64];
    ESP32P4ShaState inner;
    uint32_t blocks;
    bool last_next;
    bool sha_held;

    /* The downstream results. */
    uint8_t ds_key[32];
    bool ds_key_valid;
    bool ds_tried;
    uint8_t jtag_hmac[32];
    bool jtag_valid;
    bool jtag_mode;
    uint32_t jtag_count;
    uint32_t jtag_words[8];
    bool jtag_enabled;
};

/*
 * The RSA_DS key the downstream mode derived: false if there is none.
 * esp32p4_hmac_ds_tried() says whether software has started a
 * calculation for it since the key was last cleared.
 */
bool esp32p4_hmac_ds_key(ESP32P4HmacState *s, uint8_t key[32]);
bool esp32p4_hmac_ds_tried(ESP32P4HmacState *s);

#endif
