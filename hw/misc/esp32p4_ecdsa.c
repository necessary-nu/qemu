/*
 * ESP32-P4 ECDSA digital signature peripheral (ECDSA_DS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 33 (ECDSA Digital Signature Peripheral), which describes verification
 * only, and ESP-IDF's ecdsa_reg.h for v3 (the signature generation and
 * public key export modes, software and deterministic k, and the memories
 * of 48 bytes at 0x3E0 onwards for P-384), ecdsa_ll.h, ecdsa_hal.c and
 * its tests, whose RFC 6979 known-answer test fixes the deterministic k.
 *
 * ECDSA (FIPS 186-5) on P-192, P-256 or P-384 (ECDSA_ECC_CURVE). Each
 * operation goes IDLE, PREP (BUSY), LOAD, PROC (BUSY), then for signing
 * and key export GET, and POST (BUSY) back to IDLE, ECDSA_PREP_DONE_INT,
 * _PROC_DONE_INT and _POST_DONE_INT raising at the ends of the busy
 * stages. ECDSA_START starts one from IDLE, ECDSA_LOAD_DONE ends LOAD and
 * ECDSA_GET_DONE ends GET. In LOAD software writes z to ECDSA_Z_MEM (with
 * ECDSA_SOFTWARE_SET_Z) or hashes the message through the peripheral's
 * SHA interface (ECDSA_SHA_MODE 1, SHA-224, or 2, SHA-256; blocks it pads
 * itself in ECDSA_MESSAGE_MEM, with ECDSA_SHA_START and _CONTINUE), z
 * being the digest's leftmost bits, as many as n has; for verification
 * also r, s and the public key (QAx, QAy). Then:
 *
 *   - Verification (ECDSA_WORK_MODE 0): ECDSA_OPERATION_RESULT is 1 if
 *     the signature is valid.
 *   - Signature generation (1): r and s in GET, with k from the TRNG, from
 *     RFC 6979 with HMAC-SHA-256 (ECDSA_DETERMINISTIC_K), or from
 *     software in ECDSA_QAX_MEM (ECDSA_SOFTWARE_SET_K).
 *   - Public key export (2): QAx, QAy = d G in GET.
 *
 * The private key d is the eFuses' or the Key Manager's, as
 * KEYMNG_USE_EFUSE_KEY's ECDSA bit (cleared from reset: the Key Manager's)
 * and EFUSE_FORCE_USE_KEY_MANAGER_KEY select. From the eFuses it is the
 * block EFUSE_ECDSA_REG names for the curve, which must have the purpose
 * ECDSA_KEY_P192 (16), ECDSA_KEY (1, P-256) or, for P-384,
 * ECDSA_KEY_P384_L (17) for the low 256 bits and _H (18) for the high
 * 128; from the Key Manager its ECDSA key, the 384-bit one from its L and
 * H halves the same way. Keys are little-endian. Without a valid key, or
 * with a k out of range, signing and export fail: ECDSA_OPERATION_RESULT
 * is 0 and the outputs zero.
 *
 * State-dependent access (TRM 33.4.3.2): ECDSA_CONF_REG and _CLK_REG take
 * writes and read back only in IDLE, the SHA interface only in LOAD, the
 * memories only in LOAD (and read in GET); other accesses are dropped and
 * logged, reading as zero. The memories clear when the operation ends.
 * From ECDSA_START the peripheral occupies the ECC accelerator until it is
 * back in IDLE and the SHA accelerator until LOAD ends, which raises
 * ECDSA_SHA_RELEASE_INT; each resets when released.
 *
 * Timing, in cycles of CRYPTO_ECDSA_CLK (the model's figures): PREP 64,
 * POST 32, a SHA block 72, and PROC the ECC accelerator's constant-time
 * point multiplications (one to sign or export, two to verify) and field
 * inversions.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/guest-random.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_ecdsa.h"
#include "migration/vmstate.h"

#define A_CONF          0x04
#define A_CLK           0x08
#define A_INT_RAW       0x0c
#define A_INT_ST        0x10
#define A_INT_ENA       0x14
#define A_INT_CLR       0x18
#define A_START         0x1c
#define A_STATE         0x20
#define A_RESULT        0x24
#define A_DATE          0xfc
#define A_SHA_MODE      0x200
#define A_SHA_START     0x210
#define A_SHA_CONTINUE  0x214
#define A_SHA_BUSY      0x218
#define A_MESSAGE       0x280
#define A_MEM           0x3e0
#define A_MEM_END       (A_MEM + ESP32P4_ECDSA_MEMS * ESP32P4_ECDSA_MEM_BYTES)

#define CONF_WORK_MODE(c)   extract32(c, 0, 2)
#define CONF_CURVE(c)       extract32(c, 2, 2)
#define CONF_SOFTWARE_K     BIT(4)
#define CONF_SOFTWARE_Z     BIT(5)
#define CONF_DETERMINISTIC  BIT(6)
#define CONF_MASK           0x7f

#define START_START         BIT(0)
#define START_LOAD_DONE     BIT(1)
#define START_GET_DONE      BIT(2)

#define INT_PREP_DONE       BIT(0)
#define INT_PROC_DONE       BIT(1)
#define INT_POST_DONE       BIT(2)
#define INT_SHA_RELEASE     BIT(3)
#define INT_ALL             0xf

#define DATE_RESET          37785984
#define PREP_CYCLES         64
#define POST_CYCLES         32
#define SHA_CYCLES          72

/* EFUSE_KEY_PURPOSE values (ESP-IDF's esp_efuse_purpose_t for v3). */
#define PURPOSE_ECDSA_KEY       1
#define PURPOSE_ECDSA_P192      16
#define PURPOSE_ECDSA_P384_L    17
#define PURPOSE_ECDSA_P384_H    18

/* ECDSA_STATE_REG. */
enum {
    STATE_IDLE,
    STATE_LOAD,
    STATE_GET,
    STATE_BUSY,
};

enum {
    STAGE_IDLE,
    STAGE_PREP,
    STAGE_LOAD,
    STAGE_PROC,
    STAGE_GET,
    STAGE_POST,
};

enum {
    MODE_VERIFY,
    MODE_SIGN,
    MODE_EXPORT,
};

static void ecdsa_update_irq(ESP32P4EcdsaState *s)
{
    qemu_set_irq(s->irq, s->int_raw & s->int_ena);
}

static void ecdsa_raise(ESP32P4EcdsaState *s, uint32_t bits)
{
    s->int_raw |= bits;
    ecdsa_update_irq(s);
}

static void ecdsa_hold(ESP32P4EcdsaState *s, bool sha, bool ecc)
{
    if (s->sha && sha != s->sha_held) {
        esp32p4_sha_occupy(s->sha, sha);
    }
    if (s->ecc && ecc != s->ecc_held) {
        esp32p4_ecc_occupy(s->ecc, ecc);
    }
    s->sha_held = sha;
    s->ecc_held = ecc;
}

static void ecdsa_get(ESP32P4EcdsaState *s, int mem, uint32_t *v,
                      unsigned w)
{
    esp32p4_bn_from_le(v, w, s->mem[mem], 4 * w);
}

static void ecdsa_put(ESP32P4EcdsaState *s, int mem, const uint32_t *v,
                      unsigned w)
{
    memset(s->mem[mem], 0, ESP32P4_ECDSA_MEM_BYTES);
    esp32p4_bn_to_le(s->mem[mem], 4 * w, v, w);
}

/* One eFuse key block of the purpose ECDSA wants, or false. */
static bool ecdsa_efuse_block(ESP32P4EcdsaState *s, unsigned block,
                              uint32_t purpose, uint8_t key[32])
{
    if (block < ESP32P4_EFUSE_KEY_BLOCK0 ||
        block >= ESP32P4_EFUSE_KEY_BLOCK0 + ESP32P4_EFUSE_KEY_BLOCKS ||
        esp32p4_efuse_key_purpose(s->efuse, block) != purpose) {
        return false;
    }
    esp32p4_efuse_key(s->efuse, block, key);
    return true;
}

/* The private key d for @curve, if there is a valid one. */
static bool ecdsa_key(ESP32P4EcdsaState *s, unsigned curve, uint32_t *d)
{
    const ESP32P4Curve *c = esp32p4_curve(curve);
    uint8_t buf[48] = { 0 };
    uint8_t part[32];
    unsigned len = esp32p4_curve_bytes(c);

    if (s->keymng &&
        esp32p4_keymng_selects_own(s->keymng, ESP32P4_KM_USER_ECDSA)) {
        switch (curve) {
        case ESP32P4_P192:
            if (!esp32p4_keymng_key(s->keymng, ESP32P4_KM_PURPOSE_ECDSA_192,
                                    part)) {
                return false;
            }
            memcpy(buf, part, 24);
            break;
        case ESP32P4_P256:
            if (!esp32p4_keymng_key(s->keymng, ESP32P4_KM_PURPOSE_ECDSA_256,
                                    part)) {
                return false;
            }
            memcpy(buf, part, 32);
            break;
        default:
            if (!esp32p4_keymng_key(s->keymng,
                                    ESP32P4_KM_PURPOSE_ECDSA_384_L, part)) {
                return false;
            }
            memcpy(buf, part, 32);
            if (!esp32p4_keymng_key(s->keymng,
                                    ESP32P4_KM_PURPOSE_ECDSA_384_H, part)) {
                return false;
            }
            memcpy(buf + 32, part, 16);
            break;
        }
    } else {
        uint32_t cfg;

        if (!s->efuse || (s->keymng &&
            esp32p4_keymng_forced(s->keymng, ESP32P4_KM_USER_ECDSA))) {
            return false;
        }
        cfg = s->efuse->ecdsa;
        switch (curve) {
        case ESP32P4_P192:
            if (!ecdsa_efuse_block(s, extract32(cfg, 0, 4),
                                   PURPOSE_ECDSA_P192, part)) {
                return false;
            }
            memcpy(buf, part, 24);
            break;
        case ESP32P4_P256:
            if (!ecdsa_efuse_block(s, extract32(cfg, 4, 4),
                                   PURPOSE_ECDSA_KEY, part)) {
                return false;
            }
            memcpy(buf, part, 32);
            break;
        default:
            if (!ecdsa_efuse_block(s, extract32(cfg, 8, 4),
                                   PURPOSE_ECDSA_P384_L, part)) {
                return false;
            }
            memcpy(buf, part, 32);
            if (!ecdsa_efuse_block(s, extract32(cfg, 12, 4),
                                   PURPOSE_ECDSA_P384_H, part)) {
                return false;
            }
            memcpy(buf + 32, part, 16);
            break;
        }
    }
    esp32p4_bn_from_le(d, esp32p4_curve_words(c), buf, len);
    return esp32p4_ec_scalar_valid(c, d);
}

/* z: ECDSA_Z_MEM, or the SHA interface's digest cut to n's length. */
static void ecdsa_z(ESP32P4EcdsaState *s, const ESP32P4Curve *c,
                    uint32_t *z)
{
    unsigned w = esp32p4_curve_words(c);
    unsigned bytes = esp32p4_curve_bytes(c);
    uint8_t digest[64];
    unsigned dlen;

    if (s->conf & CONF_SOFTWARE_Z) {
        ecdsa_get(s, ESP32P4_ECDSA_MEM_Z, z, w);
        return;
    }
    if (!s->sha_started) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: z is to come from the SHA "
                      "interface, which has hashed nothing; it is zero\n",
                      TYPE_ESP32P4_ECDSA);
        memset(z, 0, w * sizeof(uint32_t));
        return;
    }
    esp32p4_sha_state_to_bytes(s->op_sha_mode, &s->sha_state, digest);
    dlen = esp32p4_sha_digest_size(s->op_sha_mode);
    /* The curves' orders are whole bytes long: take the leftmost bytes. */
    esp32p4_bn_from_be(z, w, digest, MIN(dlen, bytes));
}

/*
 * The PROC stage's calculation, its results held until software may see
 * them; returns its time in cycles.
 */
/* [spec:nuos:req:emu.esp32p4.crypto] */
static uint64_t ecdsa_proc(ESP32P4EcdsaState *s)
{
    unsigned curve = CONF_CURVE(s->conf);
    const ESP32P4Curve *c = esp32p4_curve(curve);
    unsigned w = esp32p4_curve_words(c);
    uint64_t mul = esp32p4_ec_mul_cycles(c, 32 * w, 32 * w, true);
    uint64_t inv = 32ULL * w * esp32p4_ec_op_cycles(c);
    uint32_t z[ESP32P4_EC_MAX_WORDS], d[ESP32P4_EC_MAX_WORDS];
    uint32_t k[ESP32P4_EC_MAX_WORDS], r[ESP32P4_EC_MAX_WORDS];
    uint32_t sv[ESP32P4_EC_MAX_WORDS], qx[ESP32P4_EC_MAX_WORDS];
    uint32_t qy[ESP32P4_EC_MAX_WORDS], gx[ESP32P4_EC_MAX_WORDS];
    uint32_t gy[ESP32P4_EC_MAX_WORDS];
    ESP32P4EcPoint q;
    unsigned i;

    ecdsa_z(s, c, z);
    switch (CONF_WORK_MODE(s->conf)) {
    case MODE_VERIFY:
        ecdsa_get(s, ESP32P4_ECDSA_MEM_R, r, w);
        ecdsa_get(s, ESP32P4_ECDSA_MEM_S, sv, w);
        ecdsa_get(s, ESP32P4_ECDSA_MEM_QAX, qx, w);
        ecdsa_get(s, ESP32P4_ECDSA_MEM_QAY, qy, w);
        s->result = esp32p4_ecdsa_verify(c, qx, qy, z, r, sv);
        return 2 * mul + 2 * inv;
    case MODE_SIGN:
        s->result = 0;
        memset(r, 0, sizeof(r));
        memset(sv, 0, sizeof(sv));
        if (ecdsa_key(s, curve, d)) {
            if (s->conf & CONF_SOFTWARE_K) {
                ecdsa_get(s, ESP32P4_ECDSA_MEM_QAX, k, w);
                s->result = esp32p4_ecdsa_sign(c, d, z, k, r, sv);
            } else {
                for (i = 0; !s->result && i < 64; i++) {
                    if (s->conf & CONF_DETERMINISTIC) {
                        esp32p4_ecdsa_rfc6979_k(c, d, z, k, i);
                    } else {
                        do {
                            qemu_guest_getrandom_nofail(k, 4 * w);
                        } while (!esp32p4_ec_scalar_valid(c, k));
                    }
                    s->result = esp32p4_ecdsa_sign(c, d, z, k, r, sv);
                }
            }
        }
        if (!s->result) {
            memset(r, 0, sizeof(r));
            memset(sv, 0, sizeof(sv));
        }
        ecdsa_put(s, ESP32P4_ECDSA_MEM_R, r, w);
        ecdsa_put(s, ESP32P4_ECDSA_MEM_S, sv, w);
        return mul + 2 * inv;
    case MODE_EXPORT:
        memset(qx, 0, sizeof(qx));
        memset(qy, 0, sizeof(qy));
        s->result = ecdsa_key(s, curve, d);
        if (s->result) {
            esp32p4_curve_g(c, gx, gy);
            esp32p4_ec_mul(c, &q, d, gx, gy);
            esp32p4_ec_affine(c, &q, qx, qy);
        }
        ecdsa_put(s, ESP32P4_ECDSA_MEM_QAX, qx, w);
        ecdsa_put(s, ESP32P4_ECDSA_MEM_QAY, qy, w);
        return mul + inv;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_WORK_MODE 3 is invalid; "
                      "the operation fails\n", TYPE_ESP32P4_ECDSA);
        s->result = 0;
        return 1;
    }
}

static void ecdsa_end(ESP32P4EcdsaState *s)
{
    memset(s->mem, 0, sizeof(s->mem));
    memset(s->message, 0, sizeof(s->message));
    memset(&s->sha_state, 0, sizeof(s->sha_state));
    s->sha_started = false;
    s->stage = STAGE_IDLE;
    s->state = STATE_IDLE;
    ecdsa_hold(s, false, false);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecdsa_op_done(void *opaque)
{
    ESP32P4EcdsaState *s = opaque;

    switch (s->stage) {
    case STAGE_PREP:
        s->stage = STAGE_LOAD;
        s->state = STATE_LOAD;
        ecdsa_raise(s, INT_PREP_DONE);
        break;
    case STAGE_PROC:
        ecdsa_raise(s, INT_PROC_DONE);
        if (CONF_WORK_MODE(s->conf) == MODE_VERIFY) {
            s->stage = STAGE_POST;
            esp32p4_crypto_timer_start(&s->op, POST_CYCLES);
        } else {
            s->stage = STAGE_GET;
            s->state = STATE_GET;
        }
        break;
    case STAGE_POST:
        ecdsa_end(s);
        ecdsa_raise(s, INT_POST_DONE);
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecdsa_sha_done(void *opaque)
{
    ESP32P4EcdsaState *s = opaque;

    esp32p4_sha_compress(s->op_sha_mode, &s->sha_state, s->message);
    s->sha_started = true;
    s->sha_busy = false;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecdsa_start_reg(ESP32P4EcdsaState *s, uint32_t v)
{
    if ((v & START_START) && s->stage == STAGE_IDLE) {
        s->stage = STAGE_PREP;
        s->state = STATE_BUSY;
        s->result = 0;
        ecdsa_hold(s, true, true);
        esp32p4_crypto_timer_start(&s->op, PREP_CYCLES);
    } else if ((v & START_LOAD_DONE) && s->stage == STAGE_LOAD) {
        if (s->sha_busy) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_LOAD_DONE while the SHA "
                          "interface is busy; its block is abandoned\n",
                          TYPE_ESP32P4_ECDSA);
            esp32p4_crypto_timer_cancel(&s->sha_op);
            s->sha_busy = false;
        }
        s->stage = STAGE_PROC;
        s->state = STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, ecdsa_proc(s));
        ecdsa_hold(s, false, true);
        ecdsa_raise(s, INT_SHA_RELEASE);
    } else if ((v & START_GET_DONE) && s->stage == STAGE_GET) {
        s->stage = STAGE_POST;
        s->state = STATE_BUSY;
        esp32p4_crypto_timer_start(&s->op, POST_CYCLES);
    } else if (v & (START_START | START_LOAD_DONE | START_GET_DONE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_START_REG 0x%x in state %u "
                      "is ignored\n", TYPE_ESP32P4_ECDSA, v, s->state);
    }
}

static bool ecdsa_mem_ok(ESP32P4EcdsaState *s, hwaddr addr, bool write)
{
    if (s->stage == STAGE_LOAD || (!write && s->stage == STAGE_GET)) {
        return true;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of memory at 0x%" HWADDR_PRIx
                  " in state %u is refused\n", TYPE_ESP32P4_ECDSA,
                  write ? "write" : "read", addr, s->state);
    return false;
}

static uint64_t ecdsa_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4EcdsaState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    if (reg >= A_MEM && reg < A_MEM_END) {
        unsigned off = reg - A_MEM;

        if (ecdsa_mem_ok(s, addr, false)) {
            v = ldl_le_p(&s->mem[off / ESP32P4_ECDSA_MEM_BYTES]
                                [off % ESP32P4_ECDSA_MEM_BYTES]);
        }
        return extract32(v, (addr & 3) * 8, size * 8);
    }
    switch (reg) {
    case A_CONF:
        v = s->stage == STAGE_IDLE ? s->conf : 0;
        break;
    case A_CLK:
        v = s->stage == STAGE_IDLE ? s->clk_reg : 0;
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
    case A_STATE:
        v = s->state;
        break;
    case A_RESULT:
        v = s->state == STATE_BUSY ? 0 : s->result;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_SHA_MODE:
        v = s->stage == STAGE_LOAD ? s->sha_mode : 0;
        break;
    case A_SHA_BUSY:
        v = s->stage == STAGE_LOAD && s->sha_busy;
        break;
    case A_MESSAGE ... A_MESSAGE + 0x3f:
        if (s->stage == STAGE_LOAD) {
            v = ldl_le_p(s->message + reg - A_MESSAGE);
        }
        break;
    case A_INT_CLR:
    case A_START:
    case A_SHA_START:
    case A_SHA_CONTINUE:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ECDSA, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

static void ecdsa_sha_reg(ESP32P4EcdsaState *s, bool start)
{
    if (s->stage != STAGE_LOAD || s->sha_busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_SHA_%s outside LOAD or "
                      "while the SHA interface is busy is ignored\n",
                      TYPE_ESP32P4_ECDSA, start ? "START" : "CONTINUE");
        return;
    }
    if (s->sha_mode != ESP32P4_SHA224 && s->sha_mode != ESP32P4_SHA256) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_SHA_MODE %u is not one the "
                      "interface takes (SHA-224, SHA-256); ignored\n",
                      TYPE_ESP32P4_ECDSA, s->sha_mode);
        return;
    }
    if (start) {
        s->op_sha_mode = s->sha_mode;
        esp32p4_sha_init(s->sha_mode, &s->sha_state);
    } else if (!s->sha_started) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_SHA_CONTINUE before "
                      "ECDSA_SHA_START is ignored\n", TYPE_ESP32P4_ECDSA);
        return;
    }
    s->sha_busy = true;
    esp32p4_crypto_timer_start(&s->sha_op, SHA_CYCLES);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecdsa_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    ESP32P4EcdsaState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v = deposit32(0, shift, size * 8, value);

    if (s->in_reset || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the peripheral is %s\n", TYPE_ESP32P4_ECDSA,
                      addr, s->in_reset ? "held in reset"
                                        : "not clocked (CRYPTO_ECDSA_CLK)");
        return;
    }
    if (reg >= A_MEM && reg < A_MEM_END) {
        unsigned off = reg - A_MEM;
        uint8_t *p = &s->mem[off / ESP32P4_ECDSA_MEM_BYTES]
                            [off % ESP32P4_ECDSA_MEM_BYTES];

        if (ecdsa_mem_ok(s, addr, true)) {
            stl_le_p(p, deposit32(ldl_le_p(p), shift, size * 8, value));
        }
        return;
    }
    switch (reg) {
    case A_CONF:
    case A_CLK:
        if (s->stage != STAGE_IDLE) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                          " outside IDLE is dropped\n", TYPE_ESP32P4_ECDSA,
                          addr);
        } else if (reg == A_CONF) {
            s->conf = deposit32(s->conf, shift, size * 8, value) & CONF_MASK;
        } else {
            s->clk_reg = deposit32(s->clk_reg, shift, size * 8, value) & 1;
        }
        break;
    case A_INT_ENA:
        s->int_ena = deposit32(s->int_ena, shift, size * 8, value) & INT_ALL;
        ecdsa_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~(v & INT_ALL);
        ecdsa_update_irq(s);
        break;
    case A_START:
        ecdsa_start_reg(s, v);
        break;
    case A_DATE:
        s->date = deposit32(s->date, shift, size * 8, value) & 0x0fffffff;
        break;
    case A_SHA_MODE:
        if (s->stage != STAGE_LOAD) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_SHA_MODE_REG outside "
                          "LOAD is dropped\n", TYPE_ESP32P4_ECDSA);
        } else {
            s->sha_mode = deposit32(s->sha_mode, shift, size * 8, value) & 7;
        }
        break;
    case A_SHA_START:
    case A_SHA_CONTINUE:
        if (v & 1) {
            ecdsa_sha_reg(s, reg == A_SHA_START);
        }
        break;
    case A_MESSAGE ... A_MESSAGE + 0x3f:
        if (s->stage != STAGE_LOAD || s->sha_busy) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: ECDSA_MESSAGE_MEM written "
                          "outside LOAD or while the SHA interface is busy; "
                          "dropped\n", TYPE_ESP32P4_ECDSA);
        } else {
            stl_le_p(s->message + reg - A_MESSAGE,
                     deposit32(ldl_le_p(s->message + reg - A_MESSAGE),
                               shift, size * 8, value));
        }
        break;
    case A_INT_RAW:
    case A_INT_ST:
    case A_STATE:
    case A_RESULT:
    case A_SHA_BUSY:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ECDSA, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_ECDSA, addr);
        break;
    }
}

static const MemoryRegionOps ecdsa_ops = {
    .read = ecdsa_read,
    .write = ecdsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void ecdsa_reset_line(void *opaque, int n, int level)
{
    ESP32P4EcdsaState *s = opaque;

    if (s->in_reset != level) {
        s->in_reset = level;
        device_cold_reset(DEVICE(s));
    }
}

static void ecdsa_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4EcdsaState *s = opaque;

    esp32p4_crypto_timer_clock_update(&s->op);
    esp32p4_crypto_timer_clock_update(&s->sha_op);
}

static void ecdsa_reset_hold(Object *obj, ResetType type)
{
    ESP32P4EcdsaState *s = ESP32P4_ECDSA(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    esp32p4_crypto_timer_cancel(&s->sha_op);
    ecdsa_end(s);
    s->conf = 0;
    s->clk_reg = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->result = 0;
    s->date = DATE_RESET;
    s->sha_mode = 0;
    s->sha_busy = false;
    s->op_sha_mode = 0;
}

static void ecdsa_reset_exit(Object *obj, ResetType type)
{
    ecdsa_update_irq(ESP32P4_ECDSA(obj));
}

static void ecdsa_init(Object *obj)
{
    ESP32P4EcdsaState *s = ESP32P4_ECDSA(obj);

    memory_region_init_io(&s->iomem, obj, &ecdsa_ops, s, "esp32p4.ecdsa",
                          ESP32P4_ECDSA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", ecdsa_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), ecdsa_reset_line,
                            ESP32P4_ECDSA_RESET, 1);
}

static void ecdsa_realize(DeviceState *dev, Error **errp)
{
    ESP32P4EcdsaState *s = ESP32P4_ECDSA(dev);

    esp32p4_crypto_timer_init(&s->op, s->clk, ecdsa_op_done, s);
    esp32p4_crypto_timer_init(&s->sha_op, s->clk, ecdsa_sha_done, s);
}

static const Property ecdsa_properties[] = {
    DEFINE_PROP_LINK("efuse", ESP32P4EcdsaState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
    DEFINE_PROP_LINK("keymng", ESP32P4EcdsaState, keymng,
                     TYPE_ESP32P4_KEYMNG, ESP32P4KeyMngState *),
    DEFINE_PROP_LINK("ecc", ESP32P4EcdsaState, ecc, TYPE_ESP32P4_ECC,
                     ESP32P4EccState *),
    DEFINE_PROP_LINK("sha", ESP32P4EcdsaState, sha, TYPE_ESP32P4_SHA,
                     ESP32P4ShaDevState *),
};

static const VMStateDescription vmstate_esp32p4_ecdsa_sha = {
    .name = "esp32p4-ecdsa-sha",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64_ARRAY(h, ESP32P4ShaState, 8),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_ecdsa = {
    .name = TYPE_ESP32P4_ECDSA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4EcdsaState),
        VMSTATE_ESP32P4_CRYPTO_TIMER(sha_op, ESP32P4EcdsaState),
        VMSTATE_BOOL(in_reset, ESP32P4EcdsaState),
        VMSTATE_UINT32(conf, ESP32P4EcdsaState),
        VMSTATE_UINT32(clk_reg, ESP32P4EcdsaState),
        VMSTATE_UINT32(int_raw, ESP32P4EcdsaState),
        VMSTATE_UINT32(int_ena, ESP32P4EcdsaState),
        VMSTATE_UINT32(state, ESP32P4EcdsaState),
        VMSTATE_UINT32(result, ESP32P4EcdsaState),
        VMSTATE_UINT32(date, ESP32P4EcdsaState),
        VMSTATE_UINT32(sha_mode, ESP32P4EcdsaState),
        VMSTATE_BOOL(sha_busy, ESP32P4EcdsaState),
        VMSTATE_UINT8_ARRAY(message, ESP32P4EcdsaState, 64),
        VMSTATE_UINT8_2DARRAY(mem, ESP32P4EcdsaState, ESP32P4_ECDSA_MEMS,
                              ESP32P4_ECDSA_MEM_BYTES),
        VMSTATE_UINT32(stage, ESP32P4EcdsaState),
        VMSTATE_UINT32(op_sha_mode, ESP32P4EcdsaState),
        VMSTATE_STRUCT(sha_state, ESP32P4EcdsaState, 1,
                       vmstate_esp32p4_ecdsa_sha, ESP32P4ShaState),
        VMSTATE_BOOL(sha_started, ESP32P4EcdsaState),
        VMSTATE_BOOL(sha_held, ESP32P4EcdsaState),
        VMSTATE_BOOL(ecc_held, ESP32P4EcdsaState),
        VMSTATE_END_OF_LIST()
    },
};

static void ecdsa_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ecdsa_realize;
    dc->vmsd = &vmstate_esp32p4_ecdsa;
    device_class_set_props(dc, ecdsa_properties);
    rc->phases.hold = ecdsa_reset_hold;
    rc->phases.exit = ecdsa_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo ecdsa_info = {
    .name          = TYPE_ESP32P4_ECDSA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4EcdsaState),
    .instance_init = ecdsa_init,
    .class_init    = ecdsa_class_init,
};

static void ecdsa_register_types(void)
{
    type_register_static(&ecdsa_info);
}
type_init(ecdsa_register_types)
