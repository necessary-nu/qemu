/*
 * ESP32-P4 RSA digital signature peripheral (RSA_DS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_DS_H
#define HW_MISC_ESP32P4_DS_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_aes.h"
#include "hw/misc/esp32p4_crypto.h"
#include "hw/misc/esp32p4_hmac.h"
#include "hw/misc/esp32p4_keymng.h"
#include "hw/misc/esp32p4_rsa.h"
#include "hw/misc/esp32p4_sha.h"
#include "qom/object.h"

#define TYPE_ESP32P4_DS "esp32p4-ds"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4DsState, ESP32P4_DS)

#define ESP32P4_DS_BASE     0x50094000
#define ESP32P4_DS_SIZE     0x1000
/* The ciphertext C: Y, M and r of 4096 bits and the 384-bit box. */
#define ESP32P4_DS_C_BYTES  (3 * 512 + 48)

/*
 * The clock input "clk" is CRYPTO_DS_CLK; the registers are behind
 * CRYPTO_SYS_CLK. Named GPIO inputs: "reset", RST_EN_DS; "hmac-key", the
 * HMAC accelerator's "ds-key". The links "hmac" and "keymng" are the
 * sources of RSA_DS_KEY, and "aes", "sha" and "rsa" the accelerators it
 * occupies.
 */
#define ESP32P4_DS_RESET    "reset"
#define ESP32P4_DS_HMAC_KEY "hmac-key"

struct ESP32P4DsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    Clock *clk;
    ESP32P4HmacState *hmac;
    ESP32P4KeyMngState *keymng;
    ESP32P4AesState *aes;
    ESP32P4ShaDevState *sha;
    ESP32P4RsaState *rsa;
    ESP32P4CryptoTimer op;
    bool in_reset;

    uint8_t c[ESP32P4_DS_C_BYTES];
    uint8_t iv[16];
    uint32_t x[ESP32P4_RSA_WORDS];
    uint32_t z[ESP32P4_RSA_WORDS];
    uint32_t key_source;
    uint32_t check;
    uint32_t date;
    bool busy;

    /* The operation's phase (see esp32p4_ds.c) and its key. */
    uint32_t phase;
    uint8_t key[32];
};

#endif
