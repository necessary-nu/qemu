/*
 * ESP32-P4 Key Manager
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 36 (Key Manager), ESP-IDF's keymng_reg.h for v3 (whose descriptions of
 * KEYMNG_START and KEYMNG_CONTINUE are swapped; key_mgr_ll.h has them as
 * the TRM does), key_mgr_ll.h, esp_key_mgr.c, the boot ROM's key recovery
 * and the Key Manager tests' generator gen_key_manager_test_cases.py,
 * which fixes the byte orders.
 *
 * A deployment runs IDLE, PREP (BUSY), LOAD, PROC (BUSY), GAIN, POST (BUSY)
 * back to IDLE: KEYMNG_START from IDLE, KEYMNG_CONTINUE from LOAD and from
 * GAIN; KEYMNG_PREP_DONE_INT, _PROC_DONE_INT and _POST_DONE_INT raise at
 * the ends of the busy phases. KEYMNG_CONF_REG (the mode and the purpose)
 * is written in IDLE; the information memories are written in LOAD and read
 * in LOAD and GAIN, and clear when the deployment ends. In PROC, with the
 * HUK from the HUK Generator and random numbers from the TRNG:
 *
 *   - Random (0): the key is random (an ECDSA key a valid scalar).
 *   - AES (1): k2 = AES_dec(rand, a) from k2_info = AES(init_key, a ||
 *     rand), a = AES(rand, k2), in KEYMNG_ASSIST_INFO_MEM; the key is
 *     AES_dec(k2, k1_encrypted) from KEYMNG_PUBLIC_INFO_MEM.
 *   - ECDH0 (2): from k1*G in KEYMNG_PUBLIC_INFO_MEM and a random k2, the
 *     key is the x coordinate of k2 * k1*G on P-256, and k2*G goes to
 *     KEYMNG_ASSIST_INFO_MEM.
 *   - ECDH1 (3): k2 from k2_info as in AES mode, read as a little-endian
 *     scalar; the key is the x coordinate of k2 * k1*G.
 *   - Recovery (4): the key from key_info in KEYMNG_ASSIST_INFO_MEM.
 *   - Export (5): a new key_info for the purpose's deployed key.
 *
 * key_info = AES(HUK, AES(rand, key) || rand), with a fresh 256-bit rand,
 * goes to KEYMNG_PUBLIC_INFO_MEM in GAIN (TRM 36.3), and recovery inverts
 * it. Every AES here is AES-256-ECB. init_key is the eFuse block of
 * purpose KM_INIT_KEY (12) or, with KEYMNG_USE_SW_INIT_KEY unless
 * EFUSE_FORCE_DISABLE_SW_INIT_KEY, KEYMNG_SW_INIT_KEY_MEM. Points are x
 * then y, each 32 bytes little-endian, and an ECDH key is its x
 * coordinate's little-endian bytes. The key holds the bytes the
 * deployment gives; its users take them as ESP-IDF's tests have it: the
 * external memory encryption byte-reversed per 256-bit half, as from the
 * eFuses; RSA_DS byte-reversed; HMAC as they are; ECDSA as a little-endian
 * scalar, P-192 from the first 24 bytes and P-384 from the 256-bit L key
 * and the first 128 bits of the H key.
 *
 * The deployment fails, KEYMNG_PROC_RESULT 0 and no key deployed, if the
 * HUK is not valid, the mode or purpose is reserved, init_key is needed
 * but there is none, k1*G is not on the curve, export finds no key of the
 * purpose deployed, or EFUSE_KM_DEPLOY_ONLY_ONCE allows the key's type
 * one deployment since power-on and it has had it (a type's deployment
 * counts once its key is valid). KEYMNG_KEY_VLD_REG shows the deployed
 * keys: the flash and PSRAM keys per KEYMNG_FLASH_KEY_LEN and
 * _PSRAM_KEY_LEN (one 128-bit key, or both halves of a 256-bit one), the
 * P-384 key with both halves.
 *
 * KEYMNG_USE_EFUSE_KEY's bits, which KEYMNG_LOCK_REG's lock (as it locks
 * the other static fields), choose the eFuses' key over the Key Manager's
 * for ECDSA, the flash, HMAC, RSA_DS and the PSRAM, unless
 * EFUSE_FORCE_USE_KEY_MANAGER_KEY's bit forces the Key Manager's. With
 * the reset value 0 the users take the Key Manager's keys: the boot ROM
 * selects the eFuses' flash key when it boots an encrypted flash
 * (esp_rom_check_recover_key), and ESP-IDF selects keys before using
 * them. A selected key that is not deployed is zero.
 *
 * Timing, in cycles of CRYPTO_KM_CLK (the model's figures): PREP 64, POST
 * 32, PROC 64 plus a P-256 point multiplication of the ECC accelerator
 * for each the mode needs. KEYMNG_RND_SWITCH_CYCLE and EFUSE_KM_RND_
 * SWITCH_CYCLE only pace the TRNG's sampling, and are held; random numbers
 * come from the host. RST_EN_KM clears the deployed keys and the
 * registers, but not the record of deployments since power-on.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "crypto/aes.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_keymng.h"
#include "migration/vmstate.h"

#define A_CLK               0x04
#define A_INT_RAW           0x08
#define A_INT_ST            0x0c
#define A_INT_ENA           0x10
#define A_INT_CLR           0x14
#define A_STATIC            0x18
#define A_LOCK              0x1c
#define A_CONF              0x20
#define A_START             0x24
#define A_STATE             0x28
#define A_RESULT            0x2c
#define A_KEY_VLD           0x30
#define A_HUK_VLD           0x34
#define A_DATE              0xfc
#define A_ASSIST_INFO       0x100
#define A_PUBLIC_INFO       0x140
#define A_SW_INIT_KEY       0x180

#define CLK_RESET           0x1
#define DATE_RESET          37781824

#define STATIC_USE_EFUSE_KEY(v) extract32(v, 0, 5)
#define STATIC_RND_SWITCH_S 5
#define STATIC_USE_SW_INIT  BIT(10)
#define STATIC_FLASH_LEN    BIT(11)
#define STATIC_PSRAM_LEN    BIT(12)
#define STATIC_MASK         0x1fff
#define STATIC_RESET        (15u << STATIC_RND_SWITCH_S)

#define LOCK_RND_SWITCH     BIT(5)
#define LOCK_USE_SW_INIT    BIT(6)
#define LOCK_FLASH_LEN      BIT(7)
#define LOCK_PSRAM_LEN      BIT(8)
#define LOCK_MASK           0x1ff

#define CONF_MODE(c)        extract32(c, 0, 3)
#define CONF_PURPOSE(c)     extract32(c, 3, 4)
#define CONF_MASK           0x7f

#define START_START         BIT(0)
#define START_CONTINUE      BIT(1)

#define INT_PREP_DONE       BIT(0)
#define INT_PROC_DONE       BIT(1)
#define INT_POST_DONE       BIT(2)
#define INT_ALL             0x7

/* KEYMNG_KEY_VLD_REG. */
#define VLD_ECDSA_192       BIT(0)
#define VLD_ECDSA_256       BIT(1)
#define VLD_FLASH           BIT(2)
#define VLD_HMAC            BIT(3)
#define VLD_DS              BIT(4)
#define VLD_PSRAM           BIT(5)
#define VLD_ECDSA_384       BIT(6)

#define PREP_CYCLES         64
#define POST_CYCLES         32
#define PROC_CYCLES         64

#define PURPOSE_KM_INIT_KEY 12

enum {
    KM_STATE_IDLE,
    KM_STATE_LOAD,
    KM_STATE_GAIN,
    KM_STATE_BUSY,
};

enum {
    PHASE_IDLE,
    PHASE_PREP,
    PHASE_LOAD,
    PHASE_PROC,
    PHASE_GAIN,
    PHASE_POST,
};

enum {
    MODE_RANDOM,
    MODE_AES,
    MODE_ECDH0,
    MODE_ECDH1,
    MODE_RECOVER,
    MODE_EXPORT,
};

static const uint8_t purpose_user[ESP32P4_KM_PURPOSES] = {
    [ESP32P4_KM_PURPOSE_ECDSA_192] = ESP32P4_KM_USER_ECDSA,
    [ESP32P4_KM_PURPOSE_ECDSA_256] = ESP32P4_KM_USER_ECDSA,
    [ESP32P4_KM_PURPOSE_FLASH_256_1] = ESP32P4_KM_USER_FLASH,
    [ESP32P4_KM_PURPOSE_FLASH_256_2] = ESP32P4_KM_USER_FLASH,
    [ESP32P4_KM_PURPOSE_FLASH_128] = ESP32P4_KM_USER_FLASH,
    [ESP32P4_KM_PURPOSE_HMAC] = ESP32P4_KM_USER_HMAC,
    [ESP32P4_KM_PURPOSE_DS] = ESP32P4_KM_USER_DS,
    [ESP32P4_KM_PURPOSE_PSRAM_256_1] = ESP32P4_KM_USER_PSRAM,
    [ESP32P4_KM_PURPOSE_PSRAM_256_2] = ESP32P4_KM_USER_PSRAM,
    [ESP32P4_KM_PURPOSE_PSRAM_128] = ESP32P4_KM_USER_PSRAM,
    [ESP32P4_KM_PURPOSE_ECDSA_384_L] = ESP32P4_KM_USER_ECDSA,
    [ESP32P4_KM_PURPOSE_ECDSA_384_H] = ESP32P4_KM_USER_ECDSA,
};

static void km_update_irq(ESP32P4KeyMngState *s)
{
    qemu_set_irq(s->irq, s->int_raw & s->int_ena);
}

static void km_raise(ESP32P4KeyMngState *s, uint32_t bits)
{
    s->int_raw |= bits;
    km_update_irq(s);
}

/* The eFuse fields of EFUSE_RD_REPEAT_DATA1 and _DATA2 the KM takes. */
static uint32_t km_efuse_bits(ESP32P4KeyMngState *s, unsigned lo_word,
                              unsigned lo_bit, unsigned hi_word,
                              unsigned hi_bit)
{
    if (!s->efuse) {
        return 0;
    }
    return extract32(esp32p4_efuse_word(s->efuse, 0, lo_word), lo_bit, 4) |
           extract32(esp32p4_efuse_word(s->efuse, 0, hi_word), hi_bit, 1) << 4;
}

/* EFUSE_FORCE_USE_KEY_MANAGER_KEY: DATA1[12:9] and DATA2[23]. */
static uint32_t km_force(ESP32P4KeyMngState *s)
{
    return km_efuse_bits(s, 2, 9, 3, 23);
}

/* EFUSE_KM_DEPLOY_ONLY_ONCE: DATA1[8:5] and DATA2[22]. */
static uint32_t km_deploy_once(ESP32P4KeyMngState *s)
{
    return km_efuse_bits(s, 2, 5, 3, 22);
}

/* EFUSE_FORCE_DISABLE_SW_INIT_KEY: DATA1[13]. */
static bool km_sw_init_disabled(ESP32P4KeyMngState *s)
{
    return s->efuse && (esp32p4_efuse_word(s->efuse, 0, 2) & BIT(13));
}

bool esp32p4_keymng_forced(ESP32P4KeyMngState *s, int user)
{
    return km_force(s) & BIT(user);
}

bool esp32p4_keymng_selects_own(ESP32P4KeyMngState *s, int user)
{
    return esp32p4_keymng_forced(s, user) ||
           !(STATIC_USE_EFUSE_KEY(s->static_reg) & BIT(user));
}

bool esp32p4_keymng_key(ESP32P4KeyMngState *s, unsigned purpose,
                        uint8_t key[32])
{
    if (purpose == 0 || purpose >= ESP32P4_KM_PURPOSES ||
        !s->key_valid[purpose]) {
        return false;
    }
    memcpy(key, s->key[purpose], 32);
    return true;
}

/* The external memory key halves, byte-reversed as the cipher takes them. */
static void km_xts_half(ESP32P4KeyMngState *s, unsigned purpose,
                        uint8_t out[32])
{
    int i;

    for (i = 0; i < 32; i++) {
        out[i] = s->key_valid[purpose] ? s->key[purpose][31 - i] : 0;
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
bool esp32p4_keymng_xts_key(ESP32P4KeyMngState *s, bool psram,
                            uint8_t key[64], unsigned *bits)
{
    int user = psram ? ESP32P4_KM_USER_PSRAM : ESP32P4_KM_USER_FLASH;
    bool len256 = s->static_reg & (psram ? STATIC_PSRAM_LEN
                                         : STATIC_FLASH_LEN);

    if (!esp32p4_keymng_selects_own(s, user)) {
        return false;
    }
    memset(key, 0, 64);
    if (len256) {
        km_xts_half(s, psram ? ESP32P4_KM_PURPOSE_PSRAM_256_1
                             : ESP32P4_KM_PURPOSE_FLASH_256_1, key);
        km_xts_half(s, psram ? ESP32P4_KM_PURPOSE_PSRAM_256_2
                             : ESP32P4_KM_PURPOSE_FLASH_256_2, key + 32);
        *bits = 512;
    } else {
        km_xts_half(s, psram ? ESP32P4_KM_PURPOSE_PSRAM_128
                             : ESP32P4_KM_PURPOSE_FLASH_128, key);
        *bits = 256;
    }
    return true;
}

static uint32_t km_key_vld(ESP32P4KeyMngState *s)
{
    bool *v = s->key_valid;
    uint32_t r = 0;

    if (v[ESP32P4_KM_PURPOSE_ECDSA_192]) {
        r |= VLD_ECDSA_192;
    }
    if (v[ESP32P4_KM_PURPOSE_ECDSA_256]) {
        r |= VLD_ECDSA_256;
    }
    if ((s->static_reg & STATIC_FLASH_LEN)
        ? v[ESP32P4_KM_PURPOSE_FLASH_256_1] && v[ESP32P4_KM_PURPOSE_FLASH_256_2]
        : v[ESP32P4_KM_PURPOSE_FLASH_128]) {
        r |= VLD_FLASH;
    }
    if (v[ESP32P4_KM_PURPOSE_HMAC]) {
        r |= VLD_HMAC;
    }
    if (v[ESP32P4_KM_PURPOSE_DS]) {
        r |= VLD_DS;
    }
    if ((s->static_reg & STATIC_PSRAM_LEN)
        ? v[ESP32P4_KM_PURPOSE_PSRAM_256_1] && v[ESP32P4_KM_PURPOSE_PSRAM_256_2]
        : v[ESP32P4_KM_PURPOSE_PSRAM_128]) {
        r |= VLD_PSRAM;
    }
    if (v[ESP32P4_KM_PURPOSE_ECDSA_384_L] &&
        v[ESP32P4_KM_PURPOSE_ECDSA_384_H]) {
        r |= VLD_ECDSA_384;
    }
    return r;
}

static bool km_huk(ESP32P4KeyMngState *s, uint8_t huk[32])
{
    return s->huk && esp32p4_huk_key(s->huk, huk);
}

/* AES-256-ECB over @len bytes. */
static void km_aes(const uint8_t key[32], const uint8_t *in, uint8_t *out,
                   size_t len, bool enc)
{
    AES_KEY k;
    size_t i;

    if (enc) {
        AES_set_encrypt_key(key, 256, &k);
    } else {
        AES_set_decrypt_key(key, 256, &k);
    }
    for (i = 0; i < len; i += 16) {
        if (enc) {
            AES_encrypt(in + i, out + i, &k);
        } else {
            AES_decrypt(in + i, out + i, &k);
        }
    }
}

/* key_info = AES(HUK, AES(rand, key) || rand). */
static void km_key_info(const uint8_t huk[32], const uint8_t key[32],
                        const uint8_t rand[32], uint8_t info[64])
{
    uint8_t t[64];

    km_aes(rand, key, t, 32, true);
    memcpy(t + 32, rand, 32);
    km_aes(huk, t, info, 64, true);
}

static void km_key_from_info(const uint8_t huk[32], const uint8_t info[64],
                             uint8_t key[32])
{
    uint8_t t[64];

    km_aes(huk, info, t, 64, false);
    km_aes(t + 32, t, key, 32, false);
}

/* k2 from k2_info in KEYMNG_ASSIST_INFO_MEM, with init_key. */
static bool km_k2(ESP32P4KeyMngState *s, uint8_t k2[32])
{
    uint8_t init_key[32];
    int block;

    if ((s->static_reg & STATIC_USE_SW_INIT) && !km_sw_init_disabled(s)) {
        memcpy(init_key, s->sw_init_key, 32);
    } else {
        block = s->efuse ? esp32p4_efuse_find_purpose(s->efuse,
                                                      PURPOSE_KM_INIT_KEY)
                         : -1;
        if (block < 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: no init_key: no eFuse block "
                          "has the purpose KM_INIT_KEY and the software "
                          "init_key is not in use\n", TYPE_ESP32P4_KEYMNG);
            return false;
        }
        esp32p4_efuse_key(s->efuse, block, init_key);
    }
    km_key_from_info(init_key, s->assist_info, k2);
    return true;
}

/* x(k * (px, py)) on P-256, little-endian; false if (px, py) is not on it. */
static bool km_ecdh(const uint8_t *point, const uint32_t *k, uint8_t x[32])
{
    const ESP32P4Curve *c = esp32p4_curve(ESP32P4_P256);
    uint32_t px[ESP32P4_EC_MAX_WORDS], py[ESP32P4_EC_MAX_WORDS];
    uint32_t rx[ESP32P4_EC_MAX_WORDS], ry[ESP32P4_EC_MAX_WORDS];
    ESP32P4EcPoint r;

    esp32p4_bn_from_le(px, 8, point, 32);
    esp32p4_bn_from_le(py, 8, point + 32, 32);
    if (!esp32p4_ec_on_curve(c, px, py)) {
        return false;
    }
    esp32p4_ec_mul(c, &r, k, px, py);
    if (!esp32p4_ec_affine(c, &r, rx, ry)) {
        return false;
    }
    esp32p4_bn_to_le(x, 32, rx, 8);
    return true;
}

/* A random key for @purpose: an ECDSA key a valid scalar of its curve. */
static void km_random_key(unsigned purpose, uint8_t key[32])
{
    const ESP32P4Curve *c = NULL;
    uint32_t d[ESP32P4_EC_MAX_WORDS] = { 0 };

    if (purpose == ESP32P4_KM_PURPOSE_ECDSA_192) {
        c = esp32p4_curve(ESP32P4_P192);
    } else if (purpose == ESP32P4_KM_PURPOSE_ECDSA_256) {
        c = esp32p4_curve(ESP32P4_P256);
    }
    memset(key, 0, 32);
    if (!c) {
        qemu_guest_getrandom_nofail(key, purpose ==
                                    ESP32P4_KM_PURPOSE_ECDSA_384_H ? 16 : 32);
        return;
    }
    do {
        qemu_guest_getrandom_nofail(d, esp32p4_curve_bytes(c));
    } while (!esp32p4_ec_scalar_valid(c, d));
    esp32p4_bn_to_le(key, esp32p4_curve_bytes(c), d, esp32p4_curve_words(c));
}

/*
 * The PROC phase: deploy the key, or fail. Returns the phase's time in
 * cycles.
 */
/* [spec:nuos:req:emu.esp32p4.crypto] */
static uint64_t km_proc(ESP32P4KeyMngState *s)
{
    const ESP32P4Curve *p256 = esp32p4_curve(ESP32P4_P256);
    uint64_t mul = esp32p4_ec_mul_cycles(p256, 256, 256, true);
    unsigned purpose = s->op_purpose;
    uint8_t huk[32], key[32], k2[32];
    uint32_t k[ESP32P4_EC_MAX_WORDS], gx[ESP32P4_EC_MAX_WORDS];
    uint32_t gy[ESP32P4_EC_MAX_WORDS];
    ESP32P4EcPoint q;
    uint64_t cycles = PROC_CYCLES;
    unsigned user;

    s->result = 0;
    if (purpose == 0 || purpose >= ESP32P4_KM_PURPOSES ||
        s->op_mode > MODE_EXPORT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: KEYMNG_KGEN_MODE %u with "
                      "KEYMNG_KEY_PURPOSE %u is not valid; the deployment "
                      "fails\n", TYPE_ESP32P4_KEYMNG, s->op_mode, purpose);
        return cycles;
    }
    if (!km_huk(s, huk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the HUK is not valid; the "
                      "deployment fails\n", TYPE_ESP32P4_KEYMNG);
        return cycles;
    }
    user = purpose_user[purpose];
    if (s->op_mode != MODE_EXPORT &&
        (km_deploy_once(s) & BIT(user)) && (s->deployed_once & BIT(user))) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: EFUSE_KM_DEPLOY_ONLY_ONCE allows "
                      "this key type one deployment, which it has had\n",
                      TYPE_ESP32P4_KEYMNG);
        return cycles;
    }

    switch (s->op_mode) {
    case MODE_RANDOM:
        km_random_key(purpose, key);
        break;
    case MODE_AES:
        if (!km_k2(s, k2)) {
            return cycles;
        }
        km_aes(k2, s->public_info, key, 32, false);
        break;
    case MODE_ECDH0:
        cycles += 2 * mul;
        do {
            qemu_guest_getrandom_nofail(k, 32);
        } while (!esp32p4_ec_scalar_valid(p256, k));
        if (!km_ecdh(s->public_info, k, key)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: k1*G is not on P-256; the "
                          "deployment fails\n", TYPE_ESP32P4_KEYMNG);
            return cycles;
        }
        esp32p4_curve_g(p256, gx, gy);
        esp32p4_ec_mul(p256, &q, k, gx, gy);
        esp32p4_ec_affine(p256, &q, gx, gy);
        esp32p4_bn_to_le(s->assist_info, 32, gx, 8);
        esp32p4_bn_to_le(s->assist_info + 32, 32, gy, 8);
        break;
    case MODE_ECDH1:
        cycles += mul;
        if (!km_k2(s, k2)) {
            return cycles;
        }
        esp32p4_bn_from_le(k, 8, k2, 32);
        if (!km_ecdh(s->public_info, k, key)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: k1*G is not on P-256; the "
                          "deployment fails\n", TYPE_ESP32P4_KEYMNG);
            return cycles;
        }
        break;
    case MODE_RECOVER:
        km_key_from_info(huk, s->assist_info, key);
        break;
    case MODE_EXPORT:
        if (!s->key_valid[purpose]) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: no key of purpose %u is "
                          "deployed to export\n", TYPE_ESP32P4_KEYMNG,
                          purpose);
            return cycles;
        }
        memcpy(key, s->key[purpose], 32);
        break;
    default:
        return cycles;
    }

    if (s->op_mode != MODE_RECOVER) {
        qemu_guest_getrandom_nofail(s->op_rand, sizeof(s->op_rand));
        km_key_info(huk, key, s->op_rand, s->public_info);
    }
    memcpy(s->key[purpose], key, 32);
    s->key_valid[purpose] = true;
    if (s->op_mode != MODE_EXPORT) {
        s->deployed_once |= BIT(user);
    }
    s->result = 1;
    return cycles;
}

static void km_end(ESP32P4KeyMngState *s)
{
    memset(s->assist_info, 0, sizeof(s->assist_info));
    memset(s->public_info, 0, sizeof(s->public_info));
    memset(s->sw_init_key, 0, sizeof(s->sw_init_key));
    memset(s->op_rand, 0, sizeof(s->op_rand));
    s->phase = PHASE_IDLE;
    s->state = KM_STATE_IDLE;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void km_op_done(void *opaque)
{
    ESP32P4KeyMngState *s = opaque;

    switch (s->phase) {
    case PHASE_PREP:
        s->phase = PHASE_LOAD;
        s->state = KM_STATE_LOAD;
        km_raise(s, INT_PREP_DONE);
        break;
    case PHASE_PROC:
        s->phase = PHASE_GAIN;
        s->state = KM_STATE_GAIN;
        km_raise(s, INT_PROC_DONE);
        break;
    case PHASE_POST:
        km_end(s);
        km_raise(s, INT_POST_DONE);
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void km_start_reg(ESP32P4KeyMngState *s, uint32_t v)
{
    if ((v & START_START) && s->phase == PHASE_IDLE) {
        s->op_mode = CONF_MODE(s->conf);
        s->op_purpose = CONF_PURPOSE(s->conf);
        s->phase = PHASE_PREP;
        s->state = KM_STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, PREP_CYCLES);
    } else if ((v & START_CONTINUE) && s->phase == PHASE_LOAD) {
        s->phase = PHASE_PROC;
        s->state = KM_STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, km_proc(s));
    } else if ((v & START_CONTINUE) && s->phase == PHASE_GAIN) {
        s->phase = PHASE_POST;
        s->state = KM_STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, POST_CYCLES);
    } else if (v & (START_START | START_CONTINUE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: KEYMNG_START_REG 0x%x in state %u "
                      "is ignored\n", TYPE_ESP32P4_KEYMNG, v, s->state);
    }
}

/* Where a memory offset is, and whether this phase lets it be accessed. */
static uint8_t *km_mem(ESP32P4KeyMngState *s, hwaddr reg, bool write)
{
    uint8_t *p;

    if (reg >= A_SW_INIT_KEY) {
        p = s->sw_init_key + reg - A_SW_INIT_KEY;
    } else if (reg >= A_PUBLIC_INFO) {
        p = s->public_info + reg - A_PUBLIC_INFO;
    } else {
        p = s->assist_info + reg - A_ASSIST_INFO;
    }
    if (s->phase == PHASE_LOAD || (!write && s->phase == PHASE_GAIN)) {
        return p;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of the memory at 0x%" HWADDR_PRIx
                  " in state %u is refused\n", TYPE_ESP32P4_KEYMNG,
                  write ? "write" : "read", reg, s->state);
    return NULL;
}

static uint64_t km_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4KeyMngState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;
    uint8_t huk[32];
    uint8_t *p;

    switch (reg) {
    case A_CLK:
        v = s->clk_reg;
        break;
    case A_INT_RAW:
        v = s->int_raw;
        break;
    case A_INT_ST:
        v = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_STATIC:
        v = s->static_reg;
        break;
    case A_LOCK:
        v = s->lock;
        break;
    case A_CONF:
        v = s->conf;
        break;
    case A_STATE:
        v = s->state;
        break;
    case A_RESULT:
        v = s->result;
        break;
    case A_KEY_VLD:
        v = km_key_vld(s);
        break;
    case A_HUK_VLD:
        v = km_huk(s, huk);
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_ASSIST_INFO ... A_SW_INIT_KEY + 0x1f:
        p = km_mem(s, reg, false);
        if (p) {
            v = ldl_le_p(p);
        }
        break;
    case A_INT_CLR:
    case A_START:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_KEYMNG, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* KEYMNG_STATIC_REG's fields that KEYMNG_LOCK_REG has not locked. */
static uint32_t km_static_writable(ESP32P4KeyMngState *s)
{
    uint32_t m = STATIC_MASK & ~(s->lock & 0x1f);

    if (s->lock & LOCK_RND_SWITCH) {
        m &= ~(0x1fu << STATIC_RND_SWITCH_S);
    }
    if (s->lock & LOCK_USE_SW_INIT) {
        m &= ~STATIC_USE_SW_INIT;
    }
    if (s->lock & LOCK_FLASH_LEN) {
        m &= ~STATIC_FLASH_LEN;
    }
    if (s->lock & LOCK_PSRAM_LEN) {
        m &= ~STATIC_PSRAM_LEN;
    }
    return m;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void km_write(void *opaque, hwaddr addr, uint64_t value,
                     unsigned size)
{
    ESP32P4KeyMngState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);
    uint32_t m;
    uint8_t *p;

    if (s->in_reset || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the Key Manager is %s\n",
                      TYPE_ESP32P4_KEYMNG, addr, s->in_reset ? "held in reset"
                      : "not clocked (CRYPTO_KM_CLK)");
        return;
    }

    switch (reg) {
    case A_CLK:
        s->clk_reg = deposit32(s->clk_reg, shift, size * 8, value) & 3;
        break;
    case A_INT_ENA:
        s->int_ena = deposit32(s->int_ena, shift, size * 8, value) & INT_ALL;
        km_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_ALL);
        km_update_irq(s);
        break;
    case A_STATIC:
        m = km_static_writable(s);
        s->static_reg = (s->static_reg & ~m) |
                        (deposit32(s->static_reg, shift, size * 8, value) & m);
        break;
    case A_LOCK:
        s->lock |= v & LOCK_MASK;
        break;
    case A_CONF:
        if (s->phase != PHASE_IDLE) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: KEYMNG_CONF_REG written "
                          "outside IDLE; dropped\n", TYPE_ESP32P4_KEYMNG);
            break;
        }
        s->conf = deposit32(s->conf, shift, size * 8, value) & CONF_MASK;
        break;
    case A_START:
        km_start_reg(s, v);
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value) & 0x0fffffff;
        break;
    case A_ASSIST_INFO ... A_SW_INIT_KEY + 0x1f:
        p = km_mem(s, reg, true);
        if (p) {
            stl_le_p(p, deposit32(ldl_le_p(p), shift, size * 8, value));
        }
        break;
    case A_INT_RAW:
    case A_INT_ST:
    case A_STATE:
    case A_RESULT:
    case A_KEY_VLD:
    case A_HUK_VLD:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_KEYMNG, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_KEYMNG, addr);
        break;
    }
}

static const MemoryRegionOps km_ops = {
    .read = km_read,
    .write = km_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void km_reset_registers(ESP32P4KeyMngState *s)
{
    esp32p4_crypto_timer_cancel(&s->op);
    km_end(s);
    s->clk_reg = CLK_RESET;
    s->int_raw = 0;
    s->int_ena = 0;
    s->static_reg = STATIC_RESET;
    s->lock = 0;
    s->conf = 0;
    s->result = 0;
    s->date = DATE_RESET;
    s->op_mode = 0;
    s->op_purpose = 0;
    memset(s->key, 0, sizeof(s->key));
    memset(s->key_valid, 0, sizeof(s->key_valid));
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void km_reset_line(void *opaque, int n, int level)
{
    ESP32P4KeyMngState *s = opaque;

    if (s->in_reset != level) {
        s->in_reset = level;
        km_reset_registers(s);
        km_update_irq(s);
    }
}

static void km_clk_update(void *opaque, ClockEvent event)
{
    esp32p4_crypto_timer_clock_update(&ESP32P4_KEYMNG(opaque)->op);
}

static void km_reset_hold(Object *obj, ResetType type)
{
    ESP32P4KeyMngState *s = ESP32P4_KEYMNG(obj);

    km_reset_registers(s);
    s->deployed_once = 0;
}

static void km_reset_exit(Object *obj, ResetType type)
{
    km_update_irq(ESP32P4_KEYMNG(obj));
}

static void km_init(Object *obj)
{
    ESP32P4KeyMngState *s = ESP32P4_KEYMNG(obj);

    memory_region_init_io(&s->iomem, obj, &km_ops, s, "esp32p4.keymng",
                          ESP32P4_KEYMNG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", km_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), km_reset_line, ESP32P4_KEYMNG_RESET,
                            1);
}

static void km_realize(DeviceState *dev, Error **errp)
{
    ESP32P4KeyMngState *s = ESP32P4_KEYMNG(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, km_op_done, s);
}

static const Property km_properties[] = {
    DEFINE_PROP_LINK("efuse", ESP32P4KeyMngState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
    DEFINE_PROP_LINK("huk", ESP32P4KeyMngState, huk, TYPE_ESP32P4_HUK,
                     ESP32P4HukState *),
};

static const VMStateDescription vmstate_esp32p4_keymng = {
    .name = TYPE_ESP32P4_KEYMNG,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4KeyMngState),
        VMSTATE_BOOL(in_reset, ESP32P4KeyMngState),
        VMSTATE_UINT32(clk_reg, ESP32P4KeyMngState),
        VMSTATE_UINT32(int_raw, ESP32P4KeyMngState),
        VMSTATE_UINT32(int_ena, ESP32P4KeyMngState),
        VMSTATE_UINT32(static_reg, ESP32P4KeyMngState),
        VMSTATE_UINT32(lock, ESP32P4KeyMngState),
        VMSTATE_UINT32(conf, ESP32P4KeyMngState),
        VMSTATE_UINT32(state, ESP32P4KeyMngState),
        VMSTATE_UINT32(result, ESP32P4KeyMngState),
        VMSTATE_UINT32(date, ESP32P4KeyMngState),
        VMSTATE_UINT8_ARRAY(assist_info, ESP32P4KeyMngState, 64),
        VMSTATE_UINT8_ARRAY(public_info, ESP32P4KeyMngState, 64),
        VMSTATE_UINT8_ARRAY(sw_init_key, ESP32P4KeyMngState, 32),
        VMSTATE_UINT8_2DARRAY(key, ESP32P4KeyMngState, ESP32P4_KM_PURPOSES,
                              32),
        VMSTATE_BOOL_ARRAY(key_valid, ESP32P4KeyMngState,
                           ESP32P4_KM_PURPOSES),
        VMSTATE_UINT32(deployed_once, ESP32P4KeyMngState),
        VMSTATE_UINT32(phase, ESP32P4KeyMngState),
        VMSTATE_UINT32(op_mode, ESP32P4KeyMngState),
        VMSTATE_UINT32(op_purpose, ESP32P4KeyMngState),
        VMSTATE_UINT8_ARRAY(op_rand, ESP32P4KeyMngState, 32),
        VMSTATE_END_OF_LIST()
    },
};

static void km_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = km_realize;
    dc->vmsd = &vmstate_esp32p4_keymng;
    device_class_set_props(dc, km_properties);
    rc->phases.hold = km_reset_hold;
    rc->phases.exit = km_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo km_info = {
    .name          = TYPE_ESP32P4_KEYMNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4KeyMngState),
    .instance_init = km_init,
    .class_init    = km_class_init,
};

static void km_register_types(void)
{
    type_register_static(&km_info);
}
type_init(km_register_types)
