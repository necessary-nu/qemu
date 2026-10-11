/*
 * ESP32-P4 cryptography blocks: what they share
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_CRYPTO_H
#define HW_MISC_ESP32P4_CRYPTO_H

#include "hw/core/clock.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"

/*
 * An operation that takes a number of cycles of a block's clock and waits
 * while the clock is stopped. When the cycles have run the callback is
 * called, at that time however late its QEMU timer runs: as of the time
 * the cycles ended (esp32_line_run_at(), include/hw/gpio/esp32_line.h).
 * An operation starts at esp32_line_time(), or with _start_at() at a
 * given time, such as that of the register write that began a
 * calculation the model made first. esp32p4_crypto_timer_sync() ends an
 * operation whose cycles have run by then, for a block to call before a
 * register access shows what the operation leaves. The block calls
 * esp32p4_crypto_timer_clock_update() from its clock's callback.
 */
typedef void ESP32P4CryptoDoneFn(void *opaque);

typedef struct ESP32P4CryptoTimer {
    QEMUTimer timer;
    Clock *clk;
    ESP32P4CryptoDoneFn *done;
    void *opaque;
    bool active;
    /* The cycles left when the current stretch of constant clock began. */
    uint64_t left;
    /*
     * When that stretch began, and the clock's period through it (0 while
     * the clock is stopped).
     */
    int64_t stretch_ns;
    uint64_t stretch_period;
    /* When the cycles end, while the timer runs. */
    int64_t due_ns;
} ESP32P4CryptoTimer;

void esp32p4_crypto_timer_init(ESP32P4CryptoTimer *t, Clock *clk,
                               ESP32P4CryptoDoneFn *done, void *opaque);
void esp32p4_crypto_timer_start(ESP32P4CryptoTimer *t, uint64_t cycles);
void esp32p4_crypto_timer_start_at(ESP32P4CryptoTimer *t, uint64_t cycles,
                                   int64_t when);
void esp32p4_crypto_timer_sync(ESP32P4CryptoTimer *t, int64_t now);
void esp32p4_crypto_timer_cancel(ESP32P4CryptoTimer *t);
void esp32p4_crypto_timer_clock_update(ESP32P4CryptoTimer *t);

extern const VMStateDescription vmstate_esp32p4_crypto_timer;

#define VMSTATE_ESP32P4_CRYPTO_TIMER(_f, _s) \
    VMSTATE_STRUCT(_f, _s, 1, vmstate_esp32p4_crypto_timer, \
                   ESP32P4CryptoTimer)

/*
 * The SHA accelerator's algorithms, as SHA_MODE_REG numbers them, with
 * the state of a hash in progress kept as the eight (five for SHA-1)
 * words of FIPS 180-4's H, 32-bit words in the low halves.
 */
enum {
    ESP32P4_SHA1,
    ESP32P4_SHA224,
    ESP32P4_SHA256,
    ESP32P4_SHA384,
    ESP32P4_SHA512,
    ESP32P4_SHA512_224,
    ESP32P4_SHA512_256,
    ESP32P4_SHA512_T,
};

typedef struct ESP32P4ShaState {
    uint64_t h[8];
} ESP32P4ShaState;

/* The block size, 64 or 128 bytes, and the size of H, 20, 32 or 64. */
unsigned esp32p4_sha_block_size(int alg);
unsigned esp32p4_sha_state_size(int alg);
/* The digest's length in bytes. */
unsigned esp32p4_sha_digest_size(int alg);
void esp32p4_sha_init(int alg, ESP32P4ShaState *st);
/*
 * SHA-512/t's initial hash value, the hardware's way (TRM 31.4.1.3): one
 * SHA-512 block of "SHA-512/", t_string and the 128-bit t_length, from
 * SHA-512's initial value with each byte XORed with 0xA5.
 */
void esp32p4_sha512_t_init(ESP32P4ShaState *st, uint32_t t_string,
                           uint32_t t_length);
void esp32p4_sha_compress(int alg, ESP32P4ShaState *st,
                          const uint8_t *block);
/* H as the byte string a digest is: its words big-endian, in order. */
void esp32p4_sha_state_to_bytes(int alg, const ESP32P4ShaState *st,
                                uint8_t *out);
void esp32p4_sha_state_from_bytes(int alg, ESP32P4ShaState *st,
                                  const uint8_t *in);
/* A whole message's SHA-256, and HMAC-SHA-256 (RFC 2104). */
void esp32p4_sha256(const uint8_t *msg, size_t len, uint8_t out[32]);
void esp32p4_hmac_sha256(const uint8_t *key, size_t key_len,
                         const uint8_t *msg, size_t len, uint8_t out[32]);

/*
 * Multiplication in GF(2^128) as GCM defines it (NIST SP 800-38D 6.3),
 * on blocks in GCM's byte order.
 */
void esp32p4_gf128_mul(uint8_t out[16], const uint8_t x[16],
                       const uint8_t y[16]);

/*
 * Large numbers: little-endian arrays of 32-bit words of @n words,
 * as the RSA accelerator's memories hold them.
 */
#define ESP32P4_BN_MAX_WORDS 128

int esp32p4_bn_cmp(const uint32_t *a, const uint32_t *b, unsigned n);
bool esp32p4_bn_is_zero(const uint32_t *a, unsigned n);
uint32_t esp32p4_bn_add(uint32_t *r, const uint32_t *a, const uint32_t *b,
                        unsigned n);
uint32_t esp32p4_bn_sub(uint32_t *r, const uint32_t *a, const uint32_t *b,
                        unsigned n);
/* The bit length of @a: 0 for zero. */
unsigned esp32p4_bn_bits(const uint32_t *a, unsigned n);
static inline bool esp32p4_bn_bit(const uint32_t *a, unsigned i)
{
    return (a[i / 32] >> (i % 32)) & 1;
}
/*
 * Montgomery product r = a * b * R^-1 mod m, R = 2^(32 * n), with M' =
 * @mprime (-m^-1 mod 2^32) as the caller gives it: like the accelerator,
 * a wrong M' or operands out of range give a wrong result. r may alias a
 * or b.
 */
void esp32p4_bn_mont_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                         const uint32_t *m, uint32_t mprime, unsigned n);
/* -m0^-1 mod 2^32, for odd m0. */
uint32_t esp32p4_bn_mprime(uint32_t m0);
/* R^2 mod m. */
void esp32p4_bn_r2(uint32_t *r2, const uint32_t *m, unsigned n);
/* The n-word product of a and b, 2n words. */
void esp32p4_bn_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                    unsigned n);
/* Load @len bytes, little-endian or big-endian, into @n words. */
void esp32p4_bn_from_le(uint32_t *a, unsigned n, const uint8_t *b,
                        size_t len);
void esp32p4_bn_from_be(uint32_t *a, unsigned n, const uint8_t *b,
                        size_t len);
void esp32p4_bn_to_le(uint8_t *b, size_t len, const uint32_t *a,
                      unsigned n);
void esp32p4_bn_to_be(uint8_t *b, size_t len, const uint32_t *a,
                      unsigned n);

/*
 * The NIST curves the ECC accelerator, the ECDSA peripheral and the Key
 * Manager compute on, numbered as ECC_MULT_KEY_LENGTH and ECDSA_ECC_CURVE
 * number them. Scalars and coordinates are @words-word numbers.
 */
enum {
    ESP32P4_P192,
    ESP32P4_P256,
    ESP32P4_P384,
    ESP32P4_CURVES,
};

#define ESP32P4_EC_MAX_WORDS 12

typedef struct ESP32P4Curve ESP32P4Curve;

/* A point in Jacobian coordinates, Z = 0 at infinity. */
typedef struct ESP32P4EcPoint {
    uint32_t x[ESP32P4_EC_MAX_WORDS];
    uint32_t y[ESP32P4_EC_MAX_WORDS];
    uint32_t z[ESP32P4_EC_MAX_WORDS];
} ESP32P4EcPoint;

const ESP32P4Curve *esp32p4_curve(int curve);
unsigned esp32p4_curve_words(const ESP32P4Curve *c);
unsigned esp32p4_curve_bytes(const ESP32P4Curve *c);
const uint32_t *esp32p4_curve_p(const ESP32P4Curve *c);
const uint32_t *esp32p4_curve_n(const ESP32P4Curve *c);
void esp32p4_curve_g(const ESP32P4Curve *c, uint32_t *x, uint32_t *y);

/* Whether affine (x, y) is on the curve, with x and y below p. */
bool esp32p4_ec_on_curve(const ESP32P4Curve *c, const uint32_t *x,
                         const uint32_t *y);
/* Whether Jacobian (X, Y, Z) satisfies Y^2 = X^3 + aXZ^4 + bZ^6. */
bool esp32p4_ec_on_curve_jacobian(const ESP32P4Curve *c, const uint32_t *x,
                                  const uint32_t *y, const uint32_t *z);
/* R = k * (x, y, 1), in Jacobian coordinates. */
void esp32p4_ec_mul(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                    const uint32_t *k, const uint32_t *x, const uint32_t *y);
/* R = P + Q, Jacobian. */
void esp32p4_ec_add(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                    const ESP32P4EcPoint *p, const ESP32P4EcPoint *q);
/* The affine coordinates of P; false (and zeros) at infinity. */
bool esp32p4_ec_affine(const ESP32P4Curve *c, const ESP32P4EcPoint *p,
                       uint32_t *x, uint32_t *y);
void esp32p4_ec_point_from_affine(const ESP32P4Curve *c, ESP32P4EcPoint *p,
                                  const uint32_t *x, const uint32_t *y);

/* Arithmetic modulo p (@order false) or the order n (@order true). */
void esp32p4_ec_mod(const ESP32P4Curve *c, bool order, uint32_t *r,
                    const uint32_t *a);
void esp32p4_ec_mod_add(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a, const uint32_t *b);
void esp32p4_ec_mod_sub(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a, const uint32_t *b);
void esp32p4_ec_mod_mul(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a, const uint32_t *b);
/* a^-1, by Fermat (p and n are prime); 0 for 0. */
void esp32p4_ec_mod_inv(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a);

/*
 * The time the ECC engine takes, in its clock's cycles: a point
 * multiplication by a scalar of @bits bits of which @ones are set
 * (every bit of the curve when @constant), and a field operation of each
 * kind. These are the model's own: the TRM gives no figures.
 */
uint64_t esp32p4_ec_mul_cycles(const ESP32P4Curve *c, unsigned bits,
                               unsigned ones, bool constant);
uint64_t esp32p4_ec_op_cycles(const ESP32P4Curve *c);

/*
 * ECDSA (FIPS 186-5) on @c with the private key @d: the signature (r, s)
 * of @z with nonce @k, false if k is out of range or r or s is zero; the
 * check of a signature against the public key (qx, qy); and RFC 6979's
 * deterministic nonce for @d and @z, with HMAC-SHA-256.
 */
bool esp32p4_ecdsa_sign(const ESP32P4Curve *c, const uint32_t *d,
                        const uint32_t *z, const uint32_t *k, uint32_t *r,
                        uint32_t *s);
bool esp32p4_ecdsa_verify(const ESP32P4Curve *c, const uint32_t *qx,
                          const uint32_t *qy, const uint32_t *z,
                          const uint32_t *r, const uint32_t *s);
void esp32p4_ecdsa_rfc6979_k(const ESP32P4Curve *c, const uint32_t *d,
                             const uint32_t *z, uint32_t *k, unsigned retry);
/* Whether 1 <= a < n. */
bool esp32p4_ec_scalar_valid(const ESP32P4Curve *c, const uint32_t *a);

#endif
