/*
 * ESP32-P4 cryptography blocks: timing, hashes, large numbers and the
 * NIST curves they share
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Everything here is computed in plain C: the SHA compression functions
 * of crypto/sha*-internal.c, a word-serial Montgomery multiplier like the
 * RSA accelerator's, and Jacobian-coordinate arithmetic on the FIPS 186-5
 * curves P-192, P-256 and P-384.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "crypto/sha1_i.h"
#include "crypto/sha256_i.h"
#include "crypto/sha384_i.h"
#include "crypto/sha512_i.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_crypto.h"

/* Run the cycles left from @now at the clock's present rate. */
static void crypto_timer_arm(ESP32P4CryptoTimer *t, int64_t now)
{
    esp32p4_clk_count_start(&t->stretch, t->clk, now);
    if (!esp32p4_clk_count_running(&t->stretch)) {
        timer_del(&t->timer);
        return;
    }
    t->due_ns = esp32p4_clk_count_time(&t->stretch, t->left);
    timer_mod(&t->timer, t->due_ns);
}

/* The cycles have run, at esp32_line_time(). */
static void crypto_timer_done(void *opaque)
{
    ESP32P4CryptoTimer *t = opaque;

    t->active = false;
    t->left = 0;
    t->done(t->opaque);
}

static void crypto_timer_cb(void *opaque)
{
    ESP32P4CryptoTimer *t = opaque;

    esp32_line_run_at(t->due_ns, crypto_timer_done, t);
}

void esp32p4_crypto_timer_init(ESP32P4CryptoTimer *t, Clock *clk,
                               ESP32P4CryptoDoneFn *done, void *opaque)
{
    t->clk = clk;
    t->done = done;
    t->opaque = opaque;
    timer_init_ns(&t->timer, QEMU_CLOCK_VIRTUAL, crypto_timer_cb, t);
}

void esp32p4_crypto_timer_start_at(ESP32P4CryptoTimer *t, uint64_t cycles,
                                   int64_t when)
{
    t->active = true;
    t->left = MAX(cycles, 1);
    crypto_timer_arm(t, when);
}

void esp32p4_crypto_timer_start(ESP32P4CryptoTimer *t, uint64_t cycles)
{
    esp32p4_crypto_timer_start_at(t, cycles, esp32_line_time());
}

void esp32p4_crypto_timer_sync(ESP32P4CryptoTimer *t, int64_t now)
{
    if (t->active && timer_pending(&t->timer) && t->due_ns <= now) {
        timer_del(&t->timer);
        esp32_line_run_at(t->due_ns, crypto_timer_done, t);
    }
}

void esp32p4_crypto_timer_cancel(ESP32P4CryptoTimer *t)
{
    t->active = false;
    t->left = 0;
    timer_del(&t->timer);
}

/*
 * The clock changed: the cycles run at the old rate are done, the rest
 * run at the new one (none while it is stopped).
 */
void esp32p4_crypto_timer_clock_update(ESP32P4CryptoTimer *t)
{
    int64_t now = esp32_line_time();
    uint64_t done;

    if (!t->active) {
        return;
    }
    if (esp32p4_clk_count_running(&t->stretch)) {
        done = esp32p4_clk_count_at(&t->stretch, now);
        t->left = done >= t->left ? 1 : t->left - done;
    }
    crypto_timer_arm(t, now);
}

const VMStateDescription vmstate_esp32p4_crypto_timer = {
    .name = "esp32p4-crypto-timer",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4CryptoTimer),
        VMSTATE_BOOL(active, ESP32P4CryptoTimer),
        VMSTATE_UINT64(left, ESP32P4CryptoTimer),
        VMSTATE_ESP32P4_CLK_COUNT(stretch, ESP32P4CryptoTimer),
        VMSTATE_INT64(due_ns, ESP32P4CryptoTimer),
        VMSTATE_END_OF_LIST()
    },
};

/* SHA (FIPS 180-4) */

static const uint32_t sha1_iv[5] = {
    0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0,
};

static const uint32_t sha224_iv[8] = {
    0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939,
    0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4,
};

static const uint32_t sha256_iv[8] = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};

static const uint64_t sha384_iv[8] = {
    0xcbbb9d5dc1059ed8ULL, 0x629a292a367cd507ULL, 0x9159015a3070dd17ULL,
    0x152fecd8f70e5939ULL, 0x67332667ffc00b31ULL, 0x8eb44a8768581511ULL,
    0xdb0c2e0d64f98fa7ULL, 0x47b5481dbefa4fa4ULL,
};

static const uint64_t sha512_iv[8] = {
    0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
    0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
    0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL,
};

static const uint64_t sha512_224_iv[8] = {
    0x8c3d37c819544da2ULL, 0x73e1996689dcd4d6ULL, 0x1dfab7ae32ff9c82ULL,
    0x679dd514582f9fcfULL, 0x0f6d2b697bd44da8ULL, 0x77e36f7304c48942ULL,
    0x3f9d85a86a1d36c8ULL, 0x1112e6ad91d692a1ULL,
};

static const uint64_t sha512_256_iv[8] = {
    0x22312194fc2bf72cULL, 0x9f555fa3c84c64c2ULL, 0x2393b86b6f53b151ULL,
    0x963877195940eabdULL, 0x96283ee2a88effe3ULL, 0xbe5e1e2553863992ULL,
    0x2b0199fc2c85b8aaULL, 0x0eb72ddc81c52ca2ULL,
};

static bool sha_is_512(int alg)
{
    return alg >= ESP32P4_SHA384;
}

unsigned esp32p4_sha_block_size(int alg)
{
    return sha_is_512(alg) ? 128 : 64;
}

unsigned esp32p4_sha_state_size(int alg)
{
    return alg == ESP32P4_SHA1 ? 20 : sha_is_512(alg) ? 64 : 32;
}

unsigned esp32p4_sha_digest_size(int alg)
{
    static const uint8_t size[] = {
        [ESP32P4_SHA1] = 20, [ESP32P4_SHA224] = 28, [ESP32P4_SHA256] = 32,
        [ESP32P4_SHA384] = 48, [ESP32P4_SHA512] = 64,
        [ESP32P4_SHA512_224] = 28, [ESP32P4_SHA512_256] = 32,
        [ESP32P4_SHA512_T] = 64,
    };

    return size[alg];
}

void esp32p4_sha_init(int alg, ESP32P4ShaState *st)
{
    int i;

    memset(st, 0, sizeof(*st));
    switch (alg) {
    case ESP32P4_SHA1:
        for (i = 0; i < 5; i++) {
            st->h[i] = sha1_iv[i];
        }
        break;
    case ESP32P4_SHA224:
    case ESP32P4_SHA256:
        for (i = 0; i < 8; i++) {
            st->h[i] = alg == ESP32P4_SHA224 ? sha224_iv[i] : sha256_iv[i];
        }
        break;
    case ESP32P4_SHA384:
        memcpy(st->h, sha384_iv, sizeof(st->h));
        break;
    case ESP32P4_SHA512_224:
        memcpy(st->h, sha512_224_iv, sizeof(st->h));
        break;
    case ESP32P4_SHA512_256:
        memcpy(st->h, sha512_256_iv, sizeof(st->h));
        break;
    default:
        memcpy(st->h, sha512_iv, sizeof(st->h));
        break;
    }
}

void esp32p4_sha512_t_init(ESP32P4ShaState *st, uint32_t t_string,
                           uint32_t t_length)
{
    uint8_t block[128] = "SHA-512/";
    int i;

    for (i = 0; i < 8; i++) {
        st->h[i] = sha512_iv[i] ^ 0xa5a5a5a5a5a5a5a5ULL;
    }
    stl_be_p(block + 8, t_string);
    block[127] = t_length;
    esp32p4_sha_compress(ESP32P4_SHA512, st, block);
}

void esp32p4_sha_compress(int alg, ESP32P4ShaState *st,
                          const uint8_t *block)
{
    uint8_t buf[128];
    int i;

    memcpy(buf, block, esp32p4_sha_block_size(alg));
    if (alg == ESP32P4_SHA1) {
        uint32_t h[5];

        for (i = 0; i < 5; i++) {
            h[i] = st->h[i];
        }
        sha1_compress(h, buf);
        for (i = 0; i < 5; i++) {
            st->h[i] = h[i];
        }
    } else if (!sha_is_512(alg)) {
        struct sha256_state s256;

        for (i = 0; i < 8; i++) {
            s256.state[i] = st->h[i];
        }
        sha256_compress(&s256, buf);
        for (i = 0; i < 8; i++) {
            st->h[i] = s256.state[i];
        }
    } else {
        struct sha512_state s512;

        memcpy(s512.state, st->h, sizeof(s512.state));
        sha512_compress(&s512, buf);
        memcpy(st->h, s512.state, sizeof(st->h));
    }
}

void esp32p4_sha_state_to_bytes(int alg, const ESP32P4ShaState *st,
                                uint8_t *out)
{
    unsigned words = esp32p4_sha_state_size(alg) / (sha_is_512(alg) ? 8 : 4);
    unsigned i;

    for (i = 0; i < words; i++) {
        if (sha_is_512(alg)) {
            stq_be_p(out + 8 * i, st->h[i]);
        } else {
            stl_be_p(out + 4 * i, st->h[i]);
        }
    }
}

void esp32p4_sha_state_from_bytes(int alg, ESP32P4ShaState *st,
                                  const uint8_t *in)
{
    unsigned words = esp32p4_sha_state_size(alg) / (sha_is_512(alg) ? 8 : 4);
    unsigned i;

    memset(st, 0, sizeof(*st));
    for (i = 0; i < words; i++) {
        st->h[i] = sha_is_512(alg) ? ldq_be_p(in + 8 * i)
                                   : ldl_be_p(in + 4 * i);
    }
}

/*
 * The rest of a SHA-256 whose first @prefix_bytes (whole blocks) are
 * already in @st: @msg, then the padding for the whole length.
 */
static void sha256_finish(ESP32P4ShaState *st, uint64_t prefix_bytes,
                          const uint8_t *msg, size_t len, uint8_t out[32])
{
    uint8_t block[64];
    uint64_t bits = (prefix_bytes + len) * 8;
    size_t rest;

    while (len >= 64) {
        esp32p4_sha_compress(ESP32P4_SHA256, st, msg);
        msg += 64;
        len -= 64;
    }
    rest = len;
    memset(block, 0, sizeof(block));
    memcpy(block, msg, rest);
    block[rest] = 0x80;
    if (rest >= 56) {
        esp32p4_sha_compress(ESP32P4_SHA256, st, block);
        memset(block, 0, sizeof(block));
    }
    stq_be_p(block + 56, bits);
    esp32p4_sha_compress(ESP32P4_SHA256, st, block);
    esp32p4_sha_state_to_bytes(ESP32P4_SHA256, st, out);
}

void esp32p4_sha256(const uint8_t *msg, size_t len, uint8_t out[32])
{
    ESP32P4ShaState st;

    esp32p4_sha_init(ESP32P4_SHA256, &st);
    sha256_finish(&st, 0, msg, len, out);
}

void esp32p4_hmac_sha256(const uint8_t *key, size_t key_len,
                         const uint8_t *msg, size_t len, uint8_t out[32])
{
    uint8_t k0[64] = { 0 };
    uint8_t pad[64];
    uint8_t inner[32];
    ESP32P4ShaState st;
    int i;

    if (key_len > 64) {
        esp32p4_sha256(key, key_len, k0);
    } else {
        memcpy(k0, key, key_len);
    }
    for (i = 0; i < 64; i++) {
        pad[i] = k0[i] ^ 0x36;
    }
    esp32p4_sha_init(ESP32P4_SHA256, &st);
    esp32p4_sha_compress(ESP32P4_SHA256, &st, pad);
    sha256_finish(&st, 64, msg, len, inner);
    for (i = 0; i < 64; i++) {
        pad[i] = k0[i] ^ 0x5c;
    }
    esp32p4_sha_init(ESP32P4_SHA256, &st);
    esp32p4_sha_compress(ESP32P4_SHA256, &st, pad);
    sha256_finish(&st, 64, inner, sizeof(inner), out);
}

/* GCM's GF(2^128) */

void esp32p4_gf128_mul(uint8_t out[16], const uint8_t x[16],
                       const uint8_t y[16])
{
    uint8_t z[16] = { 0 };
    uint8_t v[16];
    int i, j;

    memcpy(v, y, 16);
    for (i = 0; i < 128; i++) {
        bool lsb;

        if ((x[i / 8] >> (7 - i % 8)) & 1) {
            for (j = 0; j < 16; j++) {
                z[j] ^= v[j];
            }
        }
        lsb = v[15] & 1;
        for (j = 15; j > 0; j--) {
            v[j] = (v[j] >> 1) | (v[j - 1] << 7);
        }
        v[0] >>= 1;
        if (lsb) {
            v[0] ^= 0xe1;
        }
    }
    memcpy(out, z, 16);
}

/* Large numbers */

int esp32p4_bn_cmp(const uint32_t *a, const uint32_t *b, unsigned n)
{
    while (n--) {
        if (a[n] != b[n]) {
            return a[n] > b[n] ? 1 : -1;
        }
    }
    return 0;
}

bool esp32p4_bn_is_zero(const uint32_t *a, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        if (a[i]) {
            return false;
        }
    }
    return true;
}

uint32_t esp32p4_bn_add(uint32_t *r, const uint32_t *a, const uint32_t *b,
                        unsigned n)
{
    uint64_t c = 0;
    unsigned i;

    for (i = 0; i < n; i++) {
        c += (uint64_t)a[i] + b[i];
        r[i] = c;
        c >>= 32;
    }
    return c;
}

uint32_t esp32p4_bn_sub(uint32_t *r, const uint32_t *a, const uint32_t *b,
                        unsigned n)
{
    int64_t borrow = 0;
    unsigned i;

    for (i = 0; i < n; i++) {
        borrow += (int64_t)a[i] - b[i];
        r[i] = borrow;
        borrow >>= 32;
    }
    return borrow ? 1 : 0;
}

unsigned esp32p4_bn_bits(const uint32_t *a, unsigned n)
{
    while (n--) {
        if (a[n]) {
            return 32 * n + 32 - clz32(a[n]);
        }
    }
    return 0;
}

void esp32p4_bn_mont_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                         const uint32_t *m, uint32_t mprime, unsigned n)
{
    uint32_t t[ESP32P4_BN_MAX_WORDS + 2] = { 0 };
    uint32_t d[ESP32P4_BN_MAX_WORDS];
    uint64_t c;
    unsigned i, j;
    bool ge;

    for (i = 0; i < n; i++) {
        uint32_t q;

        c = 0;
        for (j = 0; j < n; j++) {
            c += (uint64_t)t[j] + (uint64_t)a[j] * b[i];
            t[j] = c;
            c >>= 32;
        }
        c += t[n];
        t[n] = c;
        t[n + 1] = c >> 32;

        q = t[0] * mprime;
        c = ((uint64_t)t[0] + (uint64_t)q * m[0]) >> 32;
        for (j = 1; j < n; j++) {
            c += (uint64_t)t[j] + (uint64_t)q * m[j];
            t[j - 1] = c;
            c >>= 32;
        }
        c += t[n];
        t[n - 1] = c;
        t[n] = t[n + 1] + (c >> 32);
    }

    ge = t[n] != 0 || esp32p4_bn_cmp(t, m, n) >= 0;
    if (ge) {
        esp32p4_bn_sub(d, t, m, n);
        memcpy(r, d, n * sizeof(uint32_t));
    } else {
        memcpy(r, t, n * sizeof(uint32_t));
    }
}

uint32_t esp32p4_bn_mprime(uint32_t m0)
{
    uint32_t inv = 1;
    int i;

    /* Newton's iteration doubles the correct low bits each time. */
    for (i = 0; i < 5; i++) {
        inv *= 2 - m0 * inv;
    }
    return -inv;
}

void esp32p4_bn_r2(uint32_t *r2, const uint32_t *m, unsigned n)
{
    unsigned i;

    memset(r2, 0, n * sizeof(uint32_t));
    r2[0] = 1;
    if (n == 1 && m[0] == 1) {
        r2[0] = 0;
        return;
    }
    for (i = 0; i < 64 * n; i++) {
        uint32_t carry = esp32p4_bn_add(r2, r2, r2, n);

        if (carry || esp32p4_bn_cmp(r2, m, n) >= 0) {
            esp32p4_bn_sub(r2, r2, m, n);
        }
    }
}

void esp32p4_bn_mul(uint32_t *r, const uint32_t *a, const uint32_t *b,
                    unsigned n)
{
    uint32_t t[2 * ESP32P4_BN_MAX_WORDS] = { 0 };
    unsigned i, j;

    for (i = 0; i < n; i++) {
        uint64_t c = 0;

        for (j = 0; j < n; j++) {
            c += (uint64_t)t[i + j] + (uint64_t)a[j] * b[i];
            t[i + j] = c;
            c >>= 32;
        }
        t[i + n] = c;
    }
    memcpy(r, t, 2 * n * sizeof(uint32_t));
}

void esp32p4_bn_from_le(uint32_t *a, unsigned n, const uint8_t *b,
                        size_t len)
{
    size_t i;

    memset(a, 0, n * sizeof(uint32_t));
    for (i = 0; i < len && i < 4 * n; i++) {
        a[i / 4] |= (uint32_t)b[i] << (8 * (i % 4));
    }
}

void esp32p4_bn_from_be(uint32_t *a, unsigned n, const uint8_t *b,
                        size_t len)
{
    size_t i;

    memset(a, 0, n * sizeof(uint32_t));
    for (i = 0; i < len && i < 4 * n; i++) {
        a[i / 4] |= (uint32_t)b[len - 1 - i] << (8 * (i % 4));
    }
}

void esp32p4_bn_to_le(uint8_t *b, size_t len, const uint32_t *a,
                      unsigned n)
{
    size_t i;

    for (i = 0; i < len; i++) {
        b[i] = i < 4 * n ? a[i / 4] >> (8 * (i % 4)) : 0;
    }
}

void esp32p4_bn_to_be(uint8_t *b, size_t len, const uint32_t *a,
                      unsigned n)
{
    size_t i;

    for (i = 0; i < len; i++) {
        b[len - 1 - i] = i < 4 * n ? a[i / 4] >> (8 * (i % 4)) : 0;
    }
}

/* The NIST curves */

struct ESP32P4Curve {
    const char *p_hex;
    const char *b_hex;
    const char *gx_hex;
    const char *gy_hex;
    const char *n_hex;
    unsigned words;
    bool ready;
    uint32_t p[ESP32P4_EC_MAX_WORDS];
    uint32_t n[ESP32P4_EC_MAX_WORDS];
    uint32_t gx[ESP32P4_EC_MAX_WORDS];
    uint32_t gy[ESP32P4_EC_MAX_WORDS];
    /* Montgomery constants for p and n. */
    uint32_t mprime[2];
    uint32_t r2[2][ESP32P4_EC_MAX_WORDS];
    uint32_t one_m[2][ESP32P4_EC_MAX_WORDS];
    /* a = p - 3 and b, in Montgomery form mod p. */
    uint32_t a_m[ESP32P4_EC_MAX_WORDS];
    uint32_t b_m[ESP32P4_EC_MAX_WORDS];
};

static ESP32P4Curve esp32p4_curves[ESP32P4_CURVES] = {
    [ESP32P4_P192] = {
        .p_hex = "fffffffffffffffffffffffffffffffeffffffffffffffff",
        .b_hex = "64210519e59c80e70fa7e9ab72243049feb8deecc146b9b1",
        .gx_hex = "188da80eb03090f67cbf20eb43a18800f4ff0afd82ff1012",
        .gy_hex = "07192b95ffc8da78631011ed6b24cdd573f977a11e794811",
        .n_hex = "ffffffffffffffffffffffff99def836146bc9b1b4d22831",
        .words = 6,
    },
    [ESP32P4_P256] = {
        .p_hex = "ffffffff000000010000000000000000"
                 "00000000ffffffffffffffffffffffff",
        .b_hex = "5ac635d8aa3a93e7b3ebbd55769886bc"
                 "651d06b0cc53b0f63bce3c3e27d2604b",
        .gx_hex = "6b17d1f2e12c4247f8bce6e563a440f2"
                  "77037d812deb33a0f4a13945d898c296",
        .gy_hex = "4fe342e2fe1a7f9b8ee7eb4a7c0f9e16"
                  "2bce33576b315ececbb6406837bf51f5",
        .n_hex = "ffffffff00000000ffffffffffffffff"
                 "bce6faada7179e84f3b9cac2fc632551",
        .words = 8,
    },
    [ESP32P4_P384] = {
        .p_hex = "ffffffffffffffffffffffffffffffffffffffffffffffff"
                 "fffffffffffffffeffffffff0000000000000000ffffffff",
        .b_hex = "b3312fa7e23ee7e4988e056be3f82d19181d9c6efe814112"
                 "0314088f5013875ac656398d8a2ed19d2a85c8edd3ec2aef",
        .gx_hex = "aa87ca22be8b05378eb1c71ef320ad746e1d3b628ba79b98"
                  "59f741e082542a385502f25dbf55296c3a545e3872760ab7",
        .gy_hex = "3617de4a96262c6f5d9e98bf9292dc29f8f41dbd289a147c"
                  "e9da3113b5f0b8c00a60b1ce1d7e819d7a431d7c90ea0e5f",
        .n_hex = "ffffffffffffffffffffffffffffffffffffffffffffffff"
                 "c7634d81f4372ddf581a0db248b0a77aecec196accc52973",
        .words = 12,
    },
};

static void bn_from_hex(uint32_t *a, unsigned n, const char *hex)
{
    size_t len = strlen(hex);
    size_t i;

    memset(a, 0, n * sizeof(uint32_t));
    for (i = 0; i < len; i++) {
        char ch = hex[len - 1 - i];
        uint32_t v = g_ascii_xdigit_value(ch);

        a[i / 8] |= v << (4 * (i % 8));
    }
}

static void curve_setup(ESP32P4Curve *c)
{
    uint32_t one[ESP32P4_EC_MAX_WORDS] = { 1 };
    uint32_t three[ESP32P4_EC_MAX_WORDS] = { 3 };
    uint32_t t[ESP32P4_EC_MAX_WORDS];
    unsigned n = c->words;
    int i;

    bn_from_hex(c->p, n, c->p_hex);
    bn_from_hex(c->n, n, c->n_hex);
    bn_from_hex(c->gx, n, c->gx_hex);
    bn_from_hex(c->gy, n, c->gy_hex);
    for (i = 0; i < 2; i++) {
        const uint32_t *m = i ? c->n : c->p;

        c->mprime[i] = esp32p4_bn_mprime(m[0]);
        esp32p4_bn_r2(c->r2[i], m, n);
        esp32p4_bn_mont_mul(c->one_m[i], one, c->r2[i], m, c->mprime[i], n);
    }
    esp32p4_bn_sub(t, c->p, three, n);
    esp32p4_bn_mont_mul(c->a_m, t, c->r2[0], c->p, c->mprime[0], n);
    bn_from_hex(t, n, c->b_hex);
    esp32p4_bn_mont_mul(c->b_m, t, c->r2[0], c->p, c->mprime[0], n);
    c->ready = true;
}

const ESP32P4Curve *esp32p4_curve(int curve)
{
    ESP32P4Curve *c;

    assert(curve >= 0 && curve < ESP32P4_CURVES);
    c = &esp32p4_curves[curve];
    if (!c->ready) {
        curve_setup(c);
    }
    return c;
}

unsigned esp32p4_curve_words(const ESP32P4Curve *c)
{
    return c->words;
}

unsigned esp32p4_curve_bytes(const ESP32P4Curve *c)
{
    return 4 * c->words;
}

const uint32_t *esp32p4_curve_p(const ESP32P4Curve *c)
{
    return c->p;
}

const uint32_t *esp32p4_curve_n(const ESP32P4Curve *c)
{
    return c->n;
}

void esp32p4_curve_g(const ESP32P4Curve *c, uint32_t *x, uint32_t *y)
{
    memcpy(x, c->gx, c->words * sizeof(uint32_t));
    memcpy(y, c->gy, c->words * sizeof(uint32_t));
}

/* Modular arithmetic in Montgomery form, modulo p (o = 0) or n (o = 1). */

static const uint32_t *mod_m(const ESP32P4Curve *c, int o)
{
    return o ? c->n : c->p;
}

static void m_mul(const ESP32P4Curve *c, int o, uint32_t *r,
                  const uint32_t *a, const uint32_t *b)
{
    esp32p4_bn_mont_mul(r, a, b, mod_m(c, o), c->mprime[o], c->words);
}

static void m_add(const ESP32P4Curve *c, int o, uint32_t *r,
                  const uint32_t *a, const uint32_t *b)
{
    uint32_t carry = esp32p4_bn_add(r, a, b, c->words);

    if (carry || esp32p4_bn_cmp(r, mod_m(c, o), c->words) >= 0) {
        esp32p4_bn_sub(r, r, mod_m(c, o), c->words);
    }
}

static void m_sub(const ESP32P4Curve *c, int o, uint32_t *r,
                  const uint32_t *a, const uint32_t *b)
{
    if (esp32p4_bn_sub(r, a, b, c->words)) {
        esp32p4_bn_add(r, r, mod_m(c, o), c->words);
    }
}

/* a mod m, for any a of the curve's width, in normal form. */
static void m_reduce(const ESP32P4Curve *c, int o, uint32_t *r,
                     const uint32_t *a)
{
    uint32_t one[ESP32P4_EC_MAX_WORDS] = { 1 };
    uint32_t t[ESP32P4_EC_MAX_WORDS];

    /* a * R^2 * R^-1 = a * R mod m, then back: a mod m. */
    m_mul(c, o, t, a, c->r2[o]);
    m_mul(c, o, r, t, one);
}

static void m_to(const ESP32P4Curve *c, int o, uint32_t *r,
                 const uint32_t *a)
{
    m_mul(c, o, r, a, c->r2[o]);
}

static void m_from(const ESP32P4Curve *c, int o, uint32_t *r,
                   const uint32_t *a)
{
    uint32_t one[ESP32P4_EC_MAX_WORDS] = { 1 };

    m_mul(c, o, r, a, one);
}

/* a^(m - 2), in Montgomery form. */
static void m_inv(const ESP32P4Curve *c, int o, uint32_t *r,
                  const uint32_t *a)
{
    uint32_t two[ESP32P4_EC_MAX_WORDS] = { 2 };
    uint32_t e[ESP32P4_EC_MAX_WORDS];
    uint32_t acc[ESP32P4_EC_MAX_WORDS];
    int i;

    esp32p4_bn_sub(e, mod_m(c, o), two, c->words);
    memcpy(acc, c->one_m[o], sizeof(acc));
    for (i = esp32p4_bn_bits(e, c->words) - 1; i >= 0; i--) {
        m_mul(c, o, acc, acc, acc);
        if (esp32p4_bn_bit(e, i)) {
            m_mul(c, o, acc, acc, a);
        }
    }
    memcpy(r, acc, c->words * sizeof(uint32_t));
}

void esp32p4_ec_mod(const ESP32P4Curve *c, bool order, uint32_t *r,
                    const uint32_t *a)
{
    m_reduce(c, order, r, a);
}

void esp32p4_ec_mod_add(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a, const uint32_t *b)
{
    uint32_t x[ESP32P4_EC_MAX_WORDS], y[ESP32P4_EC_MAX_WORDS];

    m_reduce(c, order, x, a);
    m_reduce(c, order, y, b);
    m_add(c, order, r, x, y);
}

void esp32p4_ec_mod_sub(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a, const uint32_t *b)
{
    uint32_t x[ESP32P4_EC_MAX_WORDS], y[ESP32P4_EC_MAX_WORDS];

    m_reduce(c, order, x, a);
    m_reduce(c, order, y, b);
    m_sub(c, order, r, x, y);
}

void esp32p4_ec_mod_mul(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a, const uint32_t *b)
{
    uint32_t x[ESP32P4_EC_MAX_WORDS], y[ESP32P4_EC_MAX_WORDS];

    m_reduce(c, order, x, a);
    m_to(c, order, y, b);
    /* x * (b * R) * R^-1 = x * b. */
    m_mul(c, order, r, x, y);
}

void esp32p4_ec_mod_inv(const ESP32P4Curve *c, bool order, uint32_t *r,
                        const uint32_t *a)
{
    uint32_t x[ESP32P4_EC_MAX_WORDS];

    m_to(c, order, x, a);
    m_inv(c, order, x, x);
    m_from(c, order, r, x);
}

/* Points, in Jacobian coordinates whose values are in Montgomery form. */

static bool pt_is_inf(const ESP32P4Curve *c, const ESP32P4EcPoint *p)
{
    return esp32p4_bn_is_zero(p->z, c->words);
}

static void pt_set_inf(ESP32P4EcPoint *p)
{
    memset(p, 0, sizeof(*p));
}

static void pt_double(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                      const ESP32P4EcPoint *p)
{
    uint32_t delta[ESP32P4_EC_MAX_WORDS], gamma[ESP32P4_EC_MAX_WORDS];
    uint32_t beta[ESP32P4_EC_MAX_WORDS], alpha[ESP32P4_EC_MAX_WORDS];
    uint32_t t1[ESP32P4_EC_MAX_WORDS], t2[ESP32P4_EC_MAX_WORDS];
    uint32_t x3[ESP32P4_EC_MAX_WORDS], y3[ESP32P4_EC_MAX_WORDS];
    uint32_t z3[ESP32P4_EC_MAX_WORDS];

    if (pt_is_inf(c, p) || esp32p4_bn_is_zero(p->y, c->words)) {
        pt_set_inf(r);
        return;
    }
    /* dbl-2001-b, for a = -3. */
    m_mul(c, 0, delta, p->z, p->z);
    m_mul(c, 0, gamma, p->y, p->y);
    m_mul(c, 0, beta, p->x, gamma);
    m_sub(c, 0, t1, p->x, delta);
    m_add(c, 0, t2, p->x, delta);
    m_mul(c, 0, alpha, t1, t2);
    m_add(c, 0, t1, alpha, alpha);
    m_add(c, 0, alpha, t1, alpha);
    /* X3 = alpha^2 - 8 beta */
    m_mul(c, 0, x3, alpha, alpha);
    m_add(c, 0, t1, beta, beta);
    m_add(c, 0, t1, t1, t1);
    m_add(c, 0, t2, t1, t1);
    m_sub(c, 0, x3, x3, t2);
    /* Z3 = (Y + Z)^2 - gamma - delta */
    m_add(c, 0, t2, p->y, p->z);
    m_mul(c, 0, z3, t2, t2);
    m_sub(c, 0, z3, z3, gamma);
    m_sub(c, 0, z3, z3, delta);
    /* Y3 = alpha (4 beta - X3) - 8 gamma^2 */
    m_sub(c, 0, t1, t1, x3);
    m_mul(c, 0, y3, alpha, t1);
    m_mul(c, 0, t2, gamma, gamma);
    m_add(c, 0, t2, t2, t2);
    m_add(c, 0, t2, t2, t2);
    m_add(c, 0, t2, t2, t2);
    m_sub(c, 0, y3, y3, t2);
    memcpy(r->x, x3, sizeof(x3));
    memcpy(r->y, y3, sizeof(y3));
    memcpy(r->z, z3, sizeof(z3));
}

static void pt_add(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                   const ESP32P4EcPoint *p, const ESP32P4EcPoint *q)
{
    uint32_t z1z1[ESP32P4_EC_MAX_WORDS], z2z2[ESP32P4_EC_MAX_WORDS];
    uint32_t u1[ESP32P4_EC_MAX_WORDS], u2[ESP32P4_EC_MAX_WORDS];
    uint32_t s1[ESP32P4_EC_MAX_WORDS], s2[ESP32P4_EC_MAX_WORDS];
    uint32_t h[ESP32P4_EC_MAX_WORDS], rr[ESP32P4_EC_MAX_WORDS];
    uint32_t hh[ESP32P4_EC_MAX_WORDS], hhh[ESP32P4_EC_MAX_WORDS];
    uint32_t v[ESP32P4_EC_MAX_WORDS], t[ESP32P4_EC_MAX_WORDS];
    ESP32P4EcPoint out;

    if (pt_is_inf(c, p)) {
        *r = *q;
        return;
    }
    if (pt_is_inf(c, q)) {
        *r = *p;
        return;
    }
    m_mul(c, 0, z1z1, p->z, p->z);
    m_mul(c, 0, z2z2, q->z, q->z);
    m_mul(c, 0, u1, p->x, z2z2);
    m_mul(c, 0, u2, q->x, z1z1);
    m_mul(c, 0, t, q->z, z2z2);
    m_mul(c, 0, s1, p->y, t);
    m_mul(c, 0, t, p->z, z1z1);
    m_mul(c, 0, s2, q->y, t);
    m_sub(c, 0, h, u2, u1);
    m_sub(c, 0, rr, s2, s1);
    if (esp32p4_bn_is_zero(h, c->words)) {
        if (esp32p4_bn_is_zero(rr, c->words)) {
            pt_double(c, r, p);
        } else {
            pt_set_inf(r);
        }
        return;
    }
    m_mul(c, 0, hh, h, h);
    m_mul(c, 0, hhh, h, hh);
    m_mul(c, 0, v, u1, hh);
    /* X3 = r^2 - H^3 - 2 V */
    m_mul(c, 0, out.x, rr, rr);
    m_sub(c, 0, out.x, out.x, hhh);
    m_sub(c, 0, out.x, out.x, v);
    m_sub(c, 0, out.x, out.x, v);
    /* Y3 = r (V - X3) - S1 H^3 */
    m_sub(c, 0, t, v, out.x);
    m_mul(c, 0, out.y, rr, t);
    m_mul(c, 0, t, s1, hhh);
    m_sub(c, 0, out.y, out.y, t);
    /* Z3 = Z1 Z2 H */
    m_mul(c, 0, t, p->z, q->z);
    m_mul(c, 0, out.z, t, h);
    *r = out;
}

static void pt_to_mont(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                       const ESP32P4EcPoint *p)
{
    m_to(c, 0, r->x, p->x);
    m_to(c, 0, r->y, p->y);
    m_to(c, 0, r->z, p->z);
}

static void pt_from_mont(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                         const ESP32P4EcPoint *p)
{
    m_from(c, 0, r->x, p->x);
    m_from(c, 0, r->y, p->y);
    m_from(c, 0, r->z, p->z);
}

void esp32p4_ec_point_from_affine(const ESP32P4Curve *c, ESP32P4EcPoint *p,
                                  const uint32_t *x, const uint32_t *y)
{
    memset(p, 0, sizeof(*p));
    memcpy(p->x, x, c->words * sizeof(uint32_t));
    memcpy(p->y, y, c->words * sizeof(uint32_t));
    p->z[0] = 1;
}

void esp32p4_ec_mul(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                    const uint32_t *k, const uint32_t *x, const uint32_t *y)
{
    ESP32P4EcPoint base, acc;
    int i;

    esp32p4_ec_point_from_affine(c, &base, x, y);
    pt_to_mont(c, &base, &base);
    pt_set_inf(&acc);
    for (i = esp32p4_bn_bits(k, c->words) - 1; i >= 0; i--) {
        pt_double(c, &acc, &acc);
        if (esp32p4_bn_bit(k, i)) {
            pt_add(c, &acc, &acc, &base);
        }
    }
    pt_from_mont(c, r, &acc);
}

void esp32p4_ec_add(const ESP32P4Curve *c, ESP32P4EcPoint *r,
                    const ESP32P4EcPoint *p, const ESP32P4EcPoint *q)
{
    ESP32P4EcPoint pm, qm, rm;

    pt_to_mont(c, &pm, p);
    pt_to_mont(c, &qm, q);
    pt_add(c, &rm, &pm, &qm);
    pt_from_mont(c, r, &rm);
}

bool esp32p4_ec_affine(const ESP32P4Curve *c, const ESP32P4EcPoint *p,
                       uint32_t *x, uint32_t *y)
{
    uint32_t z[ESP32P4_EC_MAX_WORDS], zi[ESP32P4_EC_MAX_WORDS];
    uint32_t zi2[ESP32P4_EC_MAX_WORDS], t[ESP32P4_EC_MAX_WORDS];

    if (esp32p4_bn_is_zero(p->z, c->words)) {
        memset(x, 0, c->words * sizeof(uint32_t));
        memset(y, 0, c->words * sizeof(uint32_t));
        return false;
    }
    m_to(c, 0, z, p->z);
    m_inv(c, 0, zi, z);
    m_mul(c, 0, zi2, zi, zi);
    m_to(c, 0, t, p->x);
    m_mul(c, 0, t, t, zi2);
    m_from(c, 0, x, t);
    m_mul(c, 0, zi2, zi2, zi);
    m_to(c, 0, t, p->y);
    m_mul(c, 0, t, t, zi2);
    m_from(c, 0, y, t);
    return true;
}

/* y^2 == x^3 + a x z^4 + b z^6, with every value in Montgomery form. */
static bool on_curve_m(const ESP32P4Curve *c, const uint32_t *x,
                       const uint32_t *y, const uint32_t *z)
{
    uint32_t lhs[ESP32P4_EC_MAX_WORDS], rhs[ESP32P4_EC_MAX_WORDS];
    uint32_t z2[ESP32P4_EC_MAX_WORDS], z4[ESP32P4_EC_MAX_WORDS];
    uint32_t t[ESP32P4_EC_MAX_WORDS];

    m_mul(c, 0, lhs, y, y);
    m_mul(c, 0, rhs, x, x);
    m_mul(c, 0, rhs, rhs, x);
    m_mul(c, 0, z2, z, z);
    m_mul(c, 0, z4, z2, z2);
    m_mul(c, 0, t, c->a_m, x);
    m_mul(c, 0, t, t, z4);
    m_add(c, 0, rhs, rhs, t);
    m_mul(c, 0, t, z4, z2);
    m_mul(c, 0, t, t, c->b_m);
    m_add(c, 0, rhs, rhs, t);
    return !esp32p4_bn_cmp(lhs, rhs, c->words);
}

bool esp32p4_ec_on_curve(const ESP32P4Curve *c, const uint32_t *x,
                         const uint32_t *y)
{
    uint32_t xm[ESP32P4_EC_MAX_WORDS], ym[ESP32P4_EC_MAX_WORDS];

    if (esp32p4_bn_cmp(x, c->p, c->words) >= 0 ||
        esp32p4_bn_cmp(y, c->p, c->words) >= 0) {
        return false;
    }
    m_to(c, 0, xm, x);
    m_to(c, 0, ym, y);
    return on_curve_m(c, xm, ym, c->one_m[0]);
}

bool esp32p4_ec_on_curve_jacobian(const ESP32P4Curve *c, const uint32_t *x,
                                  const uint32_t *y, const uint32_t *z)
{
    uint32_t xm[ESP32P4_EC_MAX_WORDS], ym[ESP32P4_EC_MAX_WORDS];
    uint32_t zm[ESP32P4_EC_MAX_WORDS];

    if (esp32p4_bn_cmp(x, c->p, c->words) >= 0 ||
        esp32p4_bn_cmp(y, c->p, c->words) >= 0 ||
        esp32p4_bn_cmp(z, c->p, c->words) >= 0) {
        return false;
    }
    m_to(c, 0, xm, x);
    m_to(c, 0, ym, y);
    m_to(c, 0, zm, z);
    return on_curve_m(c, xm, ym, zm);
}

bool esp32p4_ec_scalar_valid(const ESP32P4Curve *c, const uint32_t *a)
{
    return !esp32p4_bn_is_zero(a, c->words) &&
           esp32p4_bn_cmp(a, c->n, c->words) < 0;
}

/*
 * A field multiplication takes the engine about 2 * words^2 + 10
 * cycles; a doubling 8 of them and an addition 12.
 */
static uint64_t ec_fmul_cycles(const ESP32P4Curve *c)
{
    return 2 * c->words * c->words + 10;
}

uint64_t esp32p4_ec_mul_cycles(const ESP32P4Curve *c, unsigned bits,
                               unsigned ones, bool constant)
{
    uint64_t f = ec_fmul_cycles(c);

    if (constant) {
        return (uint64_t)32 * c->words * 20 * f;
    }
    return (uint64_t)bits * 8 * f + (uint64_t)ones * 12 * f + f;
}

uint64_t esp32p4_ec_op_cycles(const ESP32P4Curve *c)
{
    return ec_fmul_cycles(c);
}

/* ECDSA */

bool esp32p4_ecdsa_sign(const ESP32P4Curve *c, const uint32_t *d,
                        const uint32_t *z, const uint32_t *k, uint32_t *r,
                        uint32_t *s)
{
    uint32_t x[ESP32P4_EC_MAX_WORDS], y[ESP32P4_EC_MAX_WORDS];
    uint32_t e[ESP32P4_EC_MAX_WORDS], t[ESP32P4_EC_MAX_WORDS];
    uint32_t ki[ESP32P4_EC_MAX_WORDS];
    ESP32P4EcPoint kg;

    if (!esp32p4_ec_scalar_valid(c, k) || !esp32p4_ec_scalar_valid(c, d)) {
        return false;
    }
    esp32p4_ec_mul(c, &kg, k, c->gx, c->gy);
    if (!esp32p4_ec_affine(c, &kg, x, y)) {
        return false;
    }
    m_reduce(c, 1, r, x);
    if (esp32p4_bn_is_zero(r, c->words)) {
        return false;
    }
    /* s = k^-1 (z + r d) mod n */
    esp32p4_ec_mod_mul(c, true, t, r, d);
    esp32p4_ec_mod_add(c, true, e, z, t);
    esp32p4_ec_mod_inv(c, true, ki, k);
    esp32p4_ec_mod_mul(c, true, s, ki, e);
    return !esp32p4_bn_is_zero(s, c->words);
}

bool esp32p4_ecdsa_verify(const ESP32P4Curve *c, const uint32_t *qx,
                          const uint32_t *qy, const uint32_t *z,
                          const uint32_t *r, const uint32_t *s)
{
    uint32_t w[ESP32P4_EC_MAX_WORDS], u1[ESP32P4_EC_MAX_WORDS];
    uint32_t u2[ESP32P4_EC_MAX_WORDS], x[ESP32P4_EC_MAX_WORDS];
    uint32_t y[ESP32P4_EC_MAX_WORDS], v[ESP32P4_EC_MAX_WORDS];
    ESP32P4EcPoint p1, p2, sum;

    if (!esp32p4_ec_scalar_valid(c, r) || !esp32p4_ec_scalar_valid(c, s) ||
        !esp32p4_ec_on_curve(c, qx, qy)) {
        return false;
    }
    esp32p4_ec_mod_inv(c, true, w, s);
    esp32p4_ec_mod_mul(c, true, u1, z, w);
    esp32p4_ec_mod_mul(c, true, u2, r, w);
    esp32p4_ec_mul(c, &p1, u1, c->gx, c->gy);
    esp32p4_ec_mul(c, &p2, u2, qx, qy);
    esp32p4_ec_add(c, &sum, &p1, &p2);
    if (!esp32p4_ec_affine(c, &sum, x, y)) {
        return false;
    }
    m_reduce(c, 1, v, x);
    return !esp32p4_bn_cmp(v, r, c->words);
}

/*
 * RFC 6979 section 3.2 with HMAC-SHA-256, h1 being z as a qlen-bit
 * string; @retry candidates are rejected before the one taken.
 */
void esp32p4_ecdsa_rfc6979_k(const ESP32P4Curve *c, const uint32_t *d,
                             const uint32_t *z, uint32_t *k, unsigned retry)
{
    unsigned rlen = 4 * c->words;
    uint8_t v[32], key[32];
    uint8_t buf[32 + 1 + 2 * 48];
    uint8_t t[64];
    uint32_t h[ESP32P4_EC_MAX_WORDS];
    int round;

    memset(v, 0x01, sizeof(v));
    memset(key, 0, sizeof(key));
    m_reduce(c, 1, h, z);
    for (round = 0; round < 2; round++) {
        memcpy(buf, v, 32);
        buf[32] = round;
        esp32p4_bn_to_be(buf + 33, rlen, d, c->words);
        esp32p4_bn_to_be(buf + 33 + rlen, rlen, h, c->words);
        esp32p4_hmac_sha256(key, 32, buf, 33 + 2 * rlen, key);
        esp32p4_hmac_sha256(key, 32, v, 32, v);
    }
    for (;;) {
        unsigned tlen = 0;

        while (tlen < rlen) {
            esp32p4_hmac_sha256(key, 32, v, 32, v);
            memcpy(t + tlen, v, 32);
            tlen += 32;
        }
        /* The curves' orders are whole words long: bits2int takes rlen. */
        esp32p4_bn_from_be(k, c->words, t, rlen);
        if (esp32p4_ec_scalar_valid(c, k)) {
            if (!retry) {
                return;
            }
            retry--;
        }
        memcpy(buf, v, 32);
        buf[32] = 0;
        esp32p4_hmac_sha256(key, 32, buf, 33, key);
        esp32p4_hmac_sha256(key, 32, v, 32, v);
    }
}
