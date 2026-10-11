/*
 * ESP32-P4 Key Manager
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_KEYMNG_H
#define HW_MISC_ESP32P4_KEYMNG_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_crypto.h"
#include "hw/misc/esp32p4_huk.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qom/object.h"

#define TYPE_ESP32P4_KEYMNG "esp32p4-keymng"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4KeyMngState, ESP32P4_KEYMNG)

#define ESP32P4_KEYMNG_BASE 0x50089000
#define ESP32P4_KEYMNG_SIZE 0x1000

/*
 * The clock input "clk" is CRYPTO_KM_CLK; the registers are behind
 * KEY_MANAGER_SYS_CLK. The named GPIO input "reset" is RST_EN_KM, which
 * clears the deployed keys. The "efuse"
 * and "huk" links give the eFuse parameters and keys and the HUK. Its
 * interrupt, KEYMNG_INTR, is sysbus IRQ 0.
 */
#define ESP32P4_KEYMNG_RESET "reset"

/* KEYMNG_KEY_PURPOSE: the keys the Key Manager holds, 32 bytes each. */
enum {
    ESP32P4_KM_PURPOSE_ECDSA_192 = 1,
    ESP32P4_KM_PURPOSE_ECDSA_256 = 2,
    ESP32P4_KM_PURPOSE_FLASH_256_1 = 3,
    ESP32P4_KM_PURPOSE_FLASH_256_2 = 4,
    ESP32P4_KM_PURPOSE_FLASH_128 = 5,
    ESP32P4_KM_PURPOSE_HMAC = 6,
    ESP32P4_KM_PURPOSE_DS = 7,
    ESP32P4_KM_PURPOSE_PSRAM_256_1 = 8,
    ESP32P4_KM_PURPOSE_PSRAM_256_2 = 9,
    ESP32P4_KM_PURPOSE_PSRAM_128 = 10,
    ESP32P4_KM_PURPOSE_ECDSA_384_L = 11,
    ESP32P4_KM_PURPOSE_ECDSA_384_H = 12,
    ESP32P4_KM_PURPOSES,
};

/*
 * The modules whose key may come from the Key Manager, in the order of
 * KEYMNG_USE_EFUSE_KEY's and EFUSE_FORCE_USE_KEY_MANAGER_KEY's bits.
 */
enum {
    ESP32P4_KM_USER_ECDSA,
    ESP32P4_KM_USER_FLASH,
    ESP32P4_KM_USER_HMAC,
    ESP32P4_KM_USER_DS,
    ESP32P4_KM_USER_PSRAM,
    ESP32P4_KM_USERS,
};

struct ESP32P4KeyMngState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4EfuseState *efuse;
    ESP32P4HukState *huk;
    ESP32P4CryptoTimer op;
    bool in_reset;

    uint32_t clk_reg;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t static_reg;
    uint32_t lock;
    uint32_t conf;
    uint32_t state;
    uint32_t result;
    uint32_t date;
    uint8_t assist_info[64];
    uint8_t public_info[64];
    uint8_t sw_init_key[32];

    /* The deployed keys, by purpose (index 0 unused). */
    uint8_t key[ESP32P4_KM_PURPOSES][32];
    bool key_valid[ESP32P4_KM_PURPOSES];
    /* The key types deployed since power-on, for KM_DEPLOY_ONLY_ONCE. */
    uint32_t deployed_once;

    /* The phase in progress (see esp32p4_keymng.c) and its mode. */
    uint32_t phase;
    uint32_t op_mode;
    uint32_t op_purpose;
    uint8_t op_rand[32];
};

/*
 * Whether module @user (ESP32P4_KM_USER_*) takes the Key Manager's key:
 * unless KEYMNG_USE_EFUSE_KEY's bit for it selects the eFuses' key, which
 * EFUSE_FORCE_USE_KEY_MANAGER_KEY's bit overrides.
 */
bool esp32p4_keymng_selects_own(ESP32P4KeyMngState *s, int user);
/* Whether EFUSE_FORCE_USE_KEY_MANAGER_KEY forbids @user the eFuses' key. */
bool esp32p4_keymng_forced(ESP32P4KeyMngState *s, int user);
/* The key deployed for @purpose; false if none is. */
bool esp32p4_keymng_key(ESP32P4KeyMngState *s, unsigned purpose,
                        uint8_t key[32]);
/*
 * The external memory encryption key for the flash (@psram false) or the
 * PSRAM: false when the eFuses' key is the one selected; else the Key
 * Manager's in the order the XTS-AES cipher takes it (data key, then
 * tweak key), *bits 256 for XTS-AES-128 or 512 for XTS-AES-256 as
 * KEYMNG_FLASH_KEY_LEN or _PSRAM_KEY_LEN gives, zeros where no key is
 * deployed.
 */
bool esp32p4_keymng_xts_key(ESP32P4KeyMngState *s, bool psram,
                            uint8_t key[64], unsigned *bits);

#endif
