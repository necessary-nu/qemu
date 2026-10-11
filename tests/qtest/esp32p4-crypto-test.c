/*
 * QTest testcase for the ESP32-P4 cryptography blocks: AES, SHA, RSA, ECC,
 * HMAC, RSA_DS, ECDSA_DS, the Key Manager, the HUK Generator and the
 * random number generator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Known-answer tests from FIPS 197, NIST SP 800-38A and 800-38D, FIPS
 * 180-4's examples and RSA and curve vectors computed with Python; virtual
 * time steps over the blocks' work at the crypto clock's 40 MHz (XTAL_CLK)
 * from reset. Interrupts are seen in the interrupt matrix's status
 * registers, which follow every source's level.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define AES             0x50090000
#define SHA             0x50091000
#define RSA             0x50092000
#define ECC             0x50093000
#define DS              0x50094000
#define HMAC            0x50095000
#define ECDSA           0x50096000
#define KM              0x50089000
#define HUK             0x50114000
#define TRNG            0x50126000
#define LP_SYS_RNG_DATA 0x501101a4
#define HP_SYS          0x500e5000
#define MEM             0x4ff40000

#define HP_SYS_CLKRST_PERI_CLK_CTRL25   0x500e60a8
#define HP_SYS_CLKRST_HP_RST_EN2        0x500e60c8
#define CRYPTO_AES_CLK_EN               (1u << 14)
#define RST_EN_DS                       (1u << 17)
#define RST_EN_KM                       (1u << 23)
#define LP_CLKRST_LP_RST_EN             0x5011100c
#define RST_EN_LP_HUK                   (1u << 24)

/* Interrupt sources, in core 0's interrupt matrix status. */
#define INTMTX_STATUS(w)    (0x500d6000 + 0x200 + 4 * (w))
#define SRC_RSA         68
#define SRC_AES         69
#define SRC_SHA         70
#define SRC_ECC         71
#define SRC_ECDSA       72
#define SRC_KM          73
#define SRC_LP_HUK      20

/* GDMA-AXI channel 0, as the AES and SHA peripherals' channel. */
#define AXI_IN(ch)      (0x5008a000 + 0x68 * (ch))
#define AXI_OUT(ch)     (0x5008a000 + 0x138 + 0x68 * (ch))
#define X_CONF0         0x10
#define X_LINK1         0x20
#define X_LINK2         0x24
#define X_PERI_SEL      0x44
#define IN_LINK_START   (1u << 2)
#define OUT_LINK_START  (1u << 1)
#define DW0(size, len)  ((size) | (len) << 12)
#define SUC_EOF         (1u << 30)
#define OWNER           (1u << 31)
#define PERI_AES        4
#define PERI_SHA        5

/* AES. */
#define AES_KEY(n)          (AES + 4 * (n))
#define AES_TEXT_IN(n)      (AES + 0x20 + 4 * (n))
#define AES_TEXT_OUT(n)     (AES + 0x30 + 4 * (n))
#define AES_MODE            (AES + 0x40)
#define AES_TRIGGER         (AES + 0x48)
#define AES_STATE           (AES + 0x4c)
#define AES_IV              (AES + 0x50)
#define AES_H               (AES + 0x60)
#define AES_J0              (AES + 0x70)
#define AES_T0              (AES + 0x80)
#define AES_DMA_ENABLE      (AES + 0x90)
#define AES_BLOCK_MODE      (AES + 0x94)
#define AES_BLOCK_NUM       (AES + 0x98)
#define AES_INC_SEL         (AES + 0x9c)
#define AES_AAD_BLOCK_NUM   (AES + 0xa0)
#define AES_REMAINDER_BITS  (AES + 0xa4)
#define AES_CONTINUE        (AES + 0xa8)
#define AES_INT_CLEAR       (AES + 0xac)
#define AES_INT_ENA         (AES + 0xb0)
#define AES_DATE            (AES + 0xb4)
#define AES_DMA_EXIT        (AES + 0xb8)
#define AES_PSEUDO          (AES + 0xd0)

/* SHA. */
#define SHA_MODE            (SHA + 0x00)
#define SHA_T_STRING        (SHA + 0x04)
#define SHA_T_LENGTH        (SHA + 0x08)
#define SHA_DMA_BLOCK_NUM   (SHA + 0x0c)
#define SHA_START           (SHA + 0x10)
#define SHA_CONTINUE        (SHA + 0x14)
#define SHA_BUSY            (SHA + 0x18)
#define SHA_DMA_START       (SHA + 0x1c)
#define SHA_CLEAR_IRQ       (SHA + 0x24)
#define SHA_IRQ_ENA         (SHA + 0x28)
#define SHA_DATE            (SHA + 0x2c)
#define SHA_H(n)            (SHA + 0x40 + 4 * (n))
#define SHA_M(n)            (SHA + 0x80 + 4 * (n))

/* RSA. */
#define RSA_M_MEM           (RSA + 0x000)
#define RSA_Z_MEM           (RSA + 0x200)
#define RSA_Y_MEM           (RSA + 0x400)
#define RSA_X_MEM           (RSA + 0x600)
#define RSA_M_PRIME         (RSA + 0x800)
#define RSA_MODE            (RSA + 0x804)
#define RSA_QUERY_CLEAN     (RSA + 0x808)
#define RSA_START_MODEXP    (RSA + 0x80c)
#define RSA_START_MODMULT   (RSA + 0x810)
#define RSA_START_MULT      (RSA + 0x814)
#define RSA_QUERY_IDLE      (RSA + 0x818)
#define RSA_INT_CLR         (RSA + 0x81c)
#define RSA_CONSTANT_TIME   (RSA + 0x820)
#define RSA_SEARCH_ENABLE   (RSA + 0x824)
#define RSA_SEARCH_POS      (RSA + 0x828)
#define RSA_INT_ENA         (RSA + 0x82c)
#define RSA_DATE            (RSA + 0x830)

/* ECC. */
#define ECC_INT_RAW         (ECC + 0x0c)
#define ECC_INT_ENA         (ECC + 0x14)
#define ECC_INT_CLR         (ECC + 0x18)
#define ECC_CONF            (ECC + 0x1c)
#define ECC_K               (ECC + 0x100)
#define ECC_PX              (ECC + 0x130)
#define ECC_PY              (ECC + 0x160)
#define ECC_QX              (ECC + 0x190)
#define ECC_QY              (ECC + 0x1c0)
#define ECC_QZ              (ECC + 0x1f0)
#define ECC_START           (1u << 0)
#define ECC_CURVE(c)        ((c) << 2)
#define ECC_MOD_BASE_P      (1u << 4)
#define ECC_MODE(m)         ((m) << 5)
#define ECC_VERIFIED        (1u << 29)

/* HMAC, RSA_DS. */
#define HMAC_SET_START      (HMAC + 0x40)
#define HMAC_SET_PURPOSE    (HMAC + 0x44)
#define HMAC_SET_KEY        (HMAC + 0x48)
#define HMAC_SET_PARA_END   (HMAC + 0x4c)
#define HMAC_SET_MSG_ONE    (HMAC + 0x50)
#define HMAC_SET_MSG_ING    (HMAC + 0x54)
#define HMAC_SET_MSG_END    (HMAC + 0x58)
#define HMAC_SET_RESULT_END (HMAC + 0x5c)
#define HMAC_QUERY_ERROR    (HMAC + 0x68)
#define HMAC_QUERY_BUSY     (HMAC + 0x6c)
#define HMAC_WR_MESSAGE(n)  (HMAC + 0x80 + 4 * (n))
#define HMAC_RD_RESULT(n)   (HMAC + 0xc0 + 4 * (n))
#define DS_SET_START        (DS + 0xe00)
#define DS_SET_FINISH       (DS + 0xe08)
#define DS_QUERY_BUSY       (DS + 0xe0c)
#define DS_QUERY_KEY_WRONG  (DS + 0xe10)

/* The Key Manager and the HUK Generator. */
#define KM_INT_RAW          (KM + 0x08)
#define KM_STATIC           (KM + 0x18)
#define KM_LOCK             (KM + 0x1c)
#define KM_CONF             (KM + 0x20)
#define KM_START            (KM + 0x24)
#define KM_STATE            (KM + 0x28)
#define KM_RESULT           (KM + 0x2c)
#define KM_KEY_VLD          (KM + 0x30)
#define KM_HUK_VLD          (KM + 0x34)
#define KM_ASSIST_INFO      (KM + 0x100)
#define KM_PUBLIC_INFO      (KM + 0x140)
#define KM_CONF_SET(mode, purpose) ((mode) | (purpose) << 3)
#define HUK_INT_RAW         (HUK + 0x08)
#define HUK_CONF            (HUK + 0x20)
#define HUK_START           (HUK + 0x24)
#define HUK_STATE           (HUK + 0x28)
#define HUK_STATUS          (HUK + 0x34)
#define HUK_INFO            (HUK + 0x100)
#define HUK_INFO_WORDS      165

/* The random number generator. */
#define RNG_CFG             (TRNG + 0x00)
#define RNG_DATA            (TRNG + 0x04)
#define RNG_RSTN            (TRNG + 0x08)
#define RNG_DATE            (TRNG + 0x0c)
#define RNG_CLK_EN          (1u << 31)

static QTestState *start(void)
{
    return qtest_init("-M esp32p4");
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

static void hex_bytes(const char *hex, uint8_t *out, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        unsigned v;

        g_assert(sscanf(hex + 2 * i, "%2x", &v) == 1);
        out[i] = v;
    }
}

/* Write @len bytes, a multiple of 4, to registers from @addr, byte 0 low. */
static void wr_bytes(QTestState *qts, uint32_t addr, const uint8_t *b,
                     size_t len)
{
    for (size_t i = 0; i < len; i += 4) {
        qtest_writel(qts, addr + i, ldl_le_p(b + i));
    }
}

static void rd_bytes(QTestState *qts, uint32_t addr, uint8_t *b, size_t len)
{
    for (size_t i = 0; i < len; i += 4) {
        stl_le_p(b + i, qtest_readl(qts, addr + i));
    }
}

static void wr_hex(QTestState *qts, uint32_t addr, const char *hex)
{
    size_t len = strlen(hex) / 2;
    g_autofree uint8_t *b = g_malloc0(len + 4);

    hex_bytes(hex, b, len);
    wr_bytes(qts, addr, b, ROUND_UP(len, 4));
}

static void check_hex(QTestState *qts, uint32_t addr, const char *hex)
{
    size_t len = strlen(hex) / 2;
    g_autofree uint8_t *want = g_malloc0(len);
    g_autofree uint8_t *got = g_malloc0(ROUND_UP(len, 4));

    hex_bytes(hex, want, len);
    rd_bytes(qts, addr, got, ROUND_UP(len, 4));
    g_assert_cmpmem(got, len, want, len);
}

static void wr_words(QTestState *qts, uint32_t addr, const uint32_t *w,
                     int n)
{
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, addr + 4 * i, w[i]);
    }
}

static void check_words(QTestState *qts, uint32_t addr, const uint32_t *w,
                        int n)
{
    for (int i = 0; i < n; i++) {
        g_assert_cmphex(qtest_readl(qts, addr + 4 * i), ==, w[i]);
    }
}

/*
 * Step virtual time until @reg & @mask == @want, failing after @limit ns,
 * in steps of a thousandth of it (at least 1 us).
 */
static void wait_for(QTestState *qts, uint32_t reg, uint32_t mask,
                     uint32_t want, int64_t limit)
{
    int64_t step = MAX(limit / 1000, 1000);

    for (int64_t t = 0; t <= limit; t += step) {
        if ((qtest_readl(qts, reg) & mask) == want) {
            return;
        }
        qtest_clock_step(qts, step);
    }
    g_assert_cmphex(qtest_readl(qts, reg) & mask, ==, want);
}

/* Vectors computed with Python (pow(), and P-256 and P-384 arithmetic). */

static const uint32_t rsa_m[32] = {
    0x3c6da5d7, 0x4da4f9fc, 0x1a6916c7, 0xb8a1abcd,
    0x656412a9, 0x7a97c643, 0x27ac435a, 0x1710cf53,
    0x11072231, 0x0512bd13, 0x66ceab36, 0x8ca59966,
    0xeaff1a09, 0x4a14876a, 0xccea71ff, 0xfd724452,
    0xc3e1b258, 0x0f1099c6, 0x38d048ec, 0x8534f457,
    0x8963dc6e, 0x5c3902b3, 0x46d4ac7a, 0xc79d6793,
    0x2c33be0a, 0xd3addccb, 0x1b2ed40e, 0x43000de0,
    0x36e2f24b, 0xf165c8ce, 0xed6f0b09, 0x86905269,
};

static const uint32_t rsa_x[32] = {
    0xd4341aad, 0xa4042bb3, 0xce80c4b0, 0x42a00403,
    0xccea2645, 0x459142de, 0x3184ff27, 0x2a318785,
    0x4f5253a0, 0x4a25e466, 0xa0817910, 0xde08caa1,
    0xbb5d7385, 0xf5ff0c03, 0xdaca3c06, 0xd93936e1,
    0xe14b0190, 0x5f552773, 0x16332aca, 0xd8441b56,
    0x9b191bf4, 0x56600224, 0xabf4a07c, 0x634f806f,
    0x81862fc9, 0x3fb62d2c, 0x2d83a823, 0x3f508249,
    0x793d0e45, 0x47adec26, 0x16df6486, 0x0f1cfd99,
};

static const uint32_t rsa_y[32] = {
    0xef412ed6, 0xd160c5d0, 0xdd905ecf, 0xf1347e0c,
    0x8c320f89, 0xd7288ff6, 0x4cdce7a6, 0x01d89a02,
    0xe8abb93f, 0x4abcb06a, 0x9286a175, 0xb474c7e8,
    0xe1df8af9, 0x4fcfa583, 0xd9196ada, 0xc3e4a892,
    0x8224b122, 0x31f3b923, 0x69f85e31, 0x6c79a3de,
    0x995253fd, 0x49c7b59b, 0x6e58d5ca, 0x738d243a,
    0x294c4ea3, 0x3bb4a570, 0x4e1bcb38, 0x4278c261,
    0xd0060cc5, 0xcc21ce88, 0x0b11ad28, 0x94c15c91,
};

static const uint32_t rsa_r[32] = {
    0x770ebc71, 0x6719ae3b, 0x6008e31e, 0xb4e15735,
    0xb473951a, 0xafcd1ddf, 0xf5585e83, 0x95aa0e20,
    0xa6085505, 0x76514dff, 0x184610c4, 0x902c0f01,
    0x19b78417, 0x6d08813a, 0x86c953ea, 0xf4bf0fd3,
    0x8b1eb956, 0xe9675085, 0xc18e15b5, 0xfb198edf,
    0x2b762fbb, 0x90857886, 0x95660e8a, 0x77b73a7d,
    0x66ce4dce, 0x55b554b7, 0xf24f4024, 0x3e849d68,
    0xe05aae2a, 0xb4fedb63, 0xdcc3884c, 0x2c9373a6,
};

#define RSA_MPRIME 0x17d5e219

static const uint32_t rsa_z[32] = {
    0x55be4cb4, 0xb6c5e6cf, 0x19cef7e8, 0x6ef4761a,
    0xcb943fc5, 0x1f91f256, 0x9016a1a9, 0xc00d21fb,
    0xa729dab3, 0xa16a739f, 0x3fb74623, 0x863f55ae,
    0xbbbb67a9, 0x28efb7e7, 0x09064809, 0x8a499a69,
    0xf0c1359b, 0x8c3dae1d, 0xcb55ad92, 0x700d20d8,
    0x654a1eed, 0x0b968bac, 0x155684d3, 0xa99e2420,
    0x4db719b9, 0x0c74fe4e, 0x964fe28b, 0xc04ecd3d,
    0x3b39564d, 0x17b98033, 0xc3bae285, 0x42d86462,
};

static const uint32_t rsa_xy_mod[32] = {
    0xdfa70907, 0x1b14c08d, 0xc9a0535a, 0x3fbf31c9,
    0xc07f4315, 0x447e0710, 0x2e3393e4, 0xd34ef967,
    0xc133af56, 0xf997c2c0, 0x8ef6dce3, 0x81af8e2d,
    0x30864e2b, 0xae6aa858, 0xb8e93da4, 0x1605c7e5,
    0x58921406, 0x698bda84, 0xfde3b1c9, 0xe5566fed,
    0xc5b77e0a, 0x6881a95b, 0xdae17964, 0x5c7554ce,
    0x9e5c08f2, 0xf98ead03, 0xb3b9cc6a, 0xd188434e,
    0x999721b6, 0x868d4f27, 0x50e6dbc9, 0x5ce2ddc1,
};

static const uint32_t mul_a[16] = {
    0x0bdbc23a, 0x7671863c, 0xa05885ac, 0xff5a52f1,
    0x47ca7883, 0x84d4cd1f, 0x88dcf943, 0xa5e333cb,
    0x78a330a1, 0xb36cc9aa, 0x57c49391, 0x2522d538,
    0xff4dab10, 0xac7cc4a4, 0x32111ac1, 0x11021c9e,
};

static const uint32_t mul_b[16] = {
    0x69ace913, 0xe9dd38b8, 0x33e238b4, 0xa2909cb6,
    0xa1f65507, 0x70ef55b1, 0x46bbe9e8, 0x2f0733c8,
    0x5b17b966, 0x6f98bca3, 0xbf37a2be, 0x96b98b5f,
    0x52175b7a, 0xa26a25c8, 0x8efbc170, 0x32decd6b,
};

static const uint32_t mul_ab[32] = {
    0x2e0e344e, 0xfd510140, 0xf27c26f4, 0xdaa1aff7,
    0x4e1561e6, 0xbddfd5ab, 0xfb2d5e32, 0x03a52ef9,
    0x8b8c46b2, 0x8208f028, 0x4233e4e5, 0x2f1be5f9,
    0x434c0502, 0xcc42e484, 0x43e50db1, 0x43447c1c,
    0x8a3fc2a6, 0x806bc79f, 0x2cd268d9, 0x457b830e,
    0x26f1c9cf, 0xb0b25f9a, 0x17ddd08d, 0xf3d81f65,
    0xf3ef2bc4, 0xfd86660a, 0x2272df2c, 0x9df1f139,
    0xcb36ee9f, 0x8961f8bd, 0x8d434c11, 0x03613711,
};

static const uint32_t p256_k[8] = {
    0xe78131c1, 0x52d32377, 0x19d9c9cc, 0xd6e4a515,
    0x0fc5b043, 0xb54a2302, 0x3a973000, 0x4708d989,
};

static const uint32_t p256_qx[8] = {
    0x48c0a956, 0x5f250e33, 0xa8ca3e4c, 0xdd1eb4f3,
    0x01699bec, 0xb89a29bf, 0xc5bca92d, 0x7019f88b,
};

static const uint32_t p256_qy[8] = {
    0x44ae09a8, 0xfe6ea736, 0xfdefd2ac, 0x072c7a77,
    0x0c4bd027, 0x05872fe9, 0xa34b4938, 0xb22508ea,
};

static const uint32_t p256_gx[8] = {
    0xd898c296, 0xf4a13945, 0x2deb33a0, 0x77037d81,
    0x63a440f2, 0xf8bce6e5, 0xe12c4247, 0x6b17d1f2,
};

static const uint32_t p256_gy[8] = {
    0x37bf51f5, 0xcbb64068, 0x6b315ece, 0x2bce3357,
    0x7c0f9e16, 0x8ee7eb4a, 0xfe1a7f9b, 0x4fe342e2,
};

static const uint32_t p384_a[12] = {
    0xc3e1ac3b, 0x950b16ff, 0x9d8cf4d4, 0xdcb285f8,
    0x3cc75f3e, 0x1f44ebd1, 0x54c0ce68, 0xef40af2e,
    0x2d733dcd, 0x4a7a0305, 0x758240df, 0x0692b534,
};

static const uint32_t p384_b[12] = {
    0x0af5ac87, 0x5b69dc23, 0xb281b888, 0x1525f363,
    0xe55b85dd, 0xf6e7d078, 0xf469aef8, 0x4922b9cc,
    0xbc20f626, 0xacdac615, 0xf5b9e1f5, 0x53be4721,
};

static const uint32_t p384_mul_n[12] = {
    0x6d900237, 0x04129617, 0x4107d599, 0xc2dbf9bf,
    0x073577ac, 0x66d5161c, 0xef919b94, 0x7ca1204c,
    0x0f177cfd, 0x574763e3, 0x6c178f95, 0xb1e8fe4b,
};

static const uint32_t p384_div_n[12] = {
    0x436d8231, 0x943739dd, 0xdda93b20, 0xbdbb8c92,
    0x8ad4e50b, 0xfd6f1b21, 0x890b73ec, 0x596b6a1a,
    0xa4f6e766, 0xb740bff1, 0x8de6a078, 0x27b1b1ba,
};

static const uint32_t p384_add_p[12] = {
    0xced758c2, 0xf074f322, 0x500ead5c, 0xf1d8795c,
    0x2222e51b, 0x162cbc4a, 0x492a7d61, 0x386368fb,
    0xe99433f4, 0xf754c91a, 0x6b3c22d4, 0x5a50fc56,
};

static const uint32_t p384_sub_p[12] = {
    0xb8ebffb3, 0x39a13add, 0xeb0b3c4c, 0xc78c9293,
    0x576bd960, 0x285d1b58, 0x60571f6f, 0xa61df561,
    0x715247a7, 0x9d9f3cef, 0x7fc85ee9, 0xb2d46e12,
};


/* AES */

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_aes_typical(void)
{
    QTestState *qts = start();

    g_assert_cmpuint(qtest_readl(qts, AES_DATE), ==, 36774000);
    g_assert_cmphex(qtest_readl(qts, AES_PSEUDO), ==, 0x3c4);
    g_assert_cmphex(qtest_readl(qts, AES_STATE), ==, 0);

    /* FIPS 197 C.1, AES-128. */
    wr_hex(qts, AES_KEY(0), "000102030405060708090a0b0c0d0e0f");
    wr_hex(qts, AES_TEXT_IN(0), "00112233445566778899aabbccddeeff");
    qtest_writel(qts, AES_MODE, 0);
    qtest_writel(qts, AES_TRIGGER, 1);
    g_assert_cmphex(qtest_readl(qts, AES_STATE), ==, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, AES_STATE), ==, 0);
    check_hex(qts, AES_TEXT_OUT(0), "69c4e0d86a7b0430d8cdb78070b4c55a");

    /* Decryption. */
    wr_hex(qts, AES_TEXT_IN(0), "69c4e0d86a7b0430d8cdb78070b4c55a");
    qtest_writel(qts, AES_MODE, 4);
    qtest_writel(qts, AES_TRIGGER, 1);
    qtest_clock_step(qts, 1000);
    check_hex(qts, AES_TEXT_OUT(0), "00112233445566778899aabbccddeeff");

    /* FIPS 197 C.3, AES-256. */
    wr_hex(qts, AES_KEY(0), "000102030405060708090a0b0c0d0e0f"
                            "101112131415161718191a1b1c1d1e1f");
    wr_hex(qts, AES_TEXT_IN(0), "00112233445566778899aabbccddeeff");
    qtest_writel(qts, AES_MODE, 2);
    qtest_writel(qts, AES_TRIGGER, 1);
    qtest_clock_step(qts, 1000);
    check_hex(qts, AES_TEXT_OUT(0), "8ea2b7ca516745bfeafc49904b496089");

    /* AES-192 is reserved on this chip: the trigger is ignored. */
    qtest_writel(qts, AES_MODE, 1);
    qtest_writel(qts, AES_TRIGGER, 1);
    g_assert_cmphex(qtest_readl(qts, AES_STATE), ==, 0);
    check_hex(qts, AES_TEXT_OUT(0), "8ea2b7ca516745bfeafc49904b496089");

    /* Writes are dropped while CRYPTO_AES_CLK is gated. */
    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL25,
                 qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL25) &
                 ~CRYPTO_AES_CLK_EN);
    qtest_writel(qts, AES_KEY(0), 0x12345678);
    g_assert_cmphex(qtest_readl(qts, AES_KEY(0)), ==, 0x03020100);
    qtest_writel(qts, HP_SYS_CLKRST_PERI_CLK_CTRL25,
                 qtest_readl(qts, HP_SYS_CLKRST_PERI_CLK_CTRL25) |
                 CRYPTO_AES_CLK_EN);

    /* RST_EN_DS also resets the AES accelerator, which RSA_DS reuses. */
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN2,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN2) | RST_EN_DS);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN2,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN2) & ~RST_EN_DS);
    g_assert_cmphex(qtest_readl(qts, AES_KEY(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, AES_MODE), ==, 0);
    qtest_quit(qts);
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

/* GDMA-AXI channel 0 for @periph, from @tx's list and into @rx's. */
static void axi_periph(QTestState *qts, unsigned periph, uint32_t tx,
                       uint32_t rx)
{
    qtest_writel(qts, AXI_OUT(0) + X_PERI_SEL, periph);
    qtest_writel(qts, AXI_OUT(0) + X_CONF0, (1u << 3) | (4u << 5));
    qtest_writel(qts, AXI_OUT(0) + X_LINK2, tx);
    qtest_writel(qts, AXI_OUT(0) + X_LINK1, OUT_LINK_START);
    if (rx) {
        qtest_writel(qts, AXI_IN(0) + X_PERI_SEL, periph);
        qtest_writel(qts, AXI_IN(0) + X_CONF0, 4u << 4);
        qtest_writel(qts, AXI_IN(0) + X_LINK2, rx);
        qtest_writel(qts, AXI_IN(0) + X_LINK1, IN_LINK_START | 1);
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_aes_dma(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x100, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;
    uint8_t pt[64], ct[64], got[64];

    /* NIST SP 800-38A F.2.1, CBC-AES128.Encrypt. */
    hex_bytes("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51"
              "30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710",
              pt, 64);
    hex_bytes("7649abac8119b246cee98e9b12e9197d5086cb9b507219ee95db113a917678b2"
              "73bed6b8e3c1743b7116e69e222295163ff1caa1681fac09120eca307586e1a7",
              ct, 64);
    qtest_memwrite(qts, src, pt, 64);
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    axi_periph(qts, PERI_AES, tx, rx);

    wr_hex(qts, AES_KEY(0), "2b7e151628aed2a6abf7158809cf4f3c");
    wr_hex(qts, AES_IV, "000102030405060708090a0b0c0d0e0f");
    qtest_writel(qts, AES_MODE, 0);
    qtest_writel(qts, AES_DMA_ENABLE, 1);
    qtest_writel(qts, AES_BLOCK_MODE, 1);
    qtest_writel(qts, AES_BLOCK_NUM, 4);
    qtest_writel(qts, AES_INT_ENA, 1);
    qtest_writel(qts, AES_TRIGGER, 1);
    g_assert_cmphex(qtest_readl(qts, AES_STATE), ==, 1);
    wait_for(qts, AES_STATE, 3, 2, 1000 * 1000);
    qtest_clock_step(qts, 10000);
    qtest_memread(qts, dst, got, 64);
    g_assert_cmpmem(got, 64, ct, 64);
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(64, 64));
    /* IV_MEM holds the chaining value, the last ciphertext block. */
    check_hex(qts, AES_IV, "3ff1caa1681fac09120eca307586e1a7");
    g_assert_true(source_level(qts, SRC_AES));
    qtest_writel(qts, AES_INT_CLEAR, 1);
    g_assert_false(source_level(qts, SRC_AES));
    /* ESP-IDF's aes_ll_dma_exit() writes 0. */
    qtest_writel(qts, AES_DMA_EXIT, 0);
    g_assert_cmphex(qtest_readl(qts, AES_STATE), ==, 0);

    /*
     * NIST GCM test case 4: AES-128, a 96-bit IV, 20 bytes of AAD and 60
     * of plaintext, ESP-IDF's way: H, J0 from software, then the AAD,
     * text and length blocks.
     */
    {
        uint8_t aad[32] = { 0 }, text[64] = { 0 }, lens[16] = { 0 };
        uint8_t want[60];

        hex_bytes("feedfacedeadbeeffeedfacedeadbeefabaddad2", aad, 20);
        hex_bytes("d9313225f88406e5a55909c5aff5269a86a7a9531534f7da2e4c303d8a31"
                  "8a721c3c0c95956809532fcf0e2449a6b525b16aedf5aa0de657ba637b39",
                  text, 60);
        hex_bytes("42831ec2217774244b7221b784d0d49ce3aa212f2c02a4e035c17e2329ac"
                  "a12e21d514b25466931c7d8f6a5aac84aa051ba30b396a0aac973d58e091",
                  want, 60);
        stq_be_p(lens, 20 * 8);
        stq_be_p(lens + 8, 60 * 8);
        qtest_memwrite(qts, src, aad, 32);
        qtest_memwrite(qts, src + 32, text, 64);
        qtest_memwrite(qts, src + 96, lens, 16);
        qtest_memset(qts, dst, 0, 64);
        desc(qts, tx, OWNER | SUC_EOF | DW0(112, 112), src, 0);
        desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
        wr_hex(qts, AES_KEY(0), "feffe9928665731c6d6a8f9467308308");
        qtest_writel(qts, AES_MODE, 0);
        qtest_writel(qts, AES_DMA_ENABLE, 1);
        qtest_writel(qts, AES_BLOCK_MODE, 6);
        qtest_writel(qts, AES_AAD_BLOCK_NUM, 2);
        qtest_writel(qts, AES_REMAINDER_BITS, (60 % 16) * 8);
        qtest_writel(qts, AES_TRIGGER, 1);
        wait_for(qts, AES_STATE, 3, 0, 100000);
        check_hex(qts, AES_H, "b83b533708bf535d0aa6e52980d53b78");
        wr_hex(qts, AES_J0, "cafebabefacedbaddecaf88800000001");
        axi_periph(qts, PERI_AES, tx, rx);
        qtest_writel(qts, AES_BLOCK_NUM, 4);
        qtest_writel(qts, AES_CONTINUE, 1);
        wait_for(qts, AES_STATE, 3, 2, 1000 * 1000);
        qtest_clock_step(qts, 10000);
        qtest_memread(qts, dst, got, 60);
        g_assert_cmpmem(got, 60, want, 60);
        check_hex(qts, AES_T0, "5bc94fbc3221a5db94fae95ae7121a47");
        qtest_writel(qts, AES_DMA_EXIT, 0);
    }
    qtest_quit(qts);
}

/* SHA */

static const struct {
    int mode;
    const char *digest;
} sha_abc[] = {
    { 0, "a9993e364706816aba3e25717850c26c9cd0d89d" },
    { 1, "23097d223405d8228642a477bda255b32aadbce4bda0b3f7e36c9da7" },
    { 2, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad" },
    { 3, "cb00753f45a35e8bb5a03d699ac65007272c32ab0eded1631a8b605a43ff5bed"
         "8086072ba1e7cc2358baeca134c825a7" },
    { 4, "ddaf35a193617abacc417349ae20413112e6fa4e89a97ea20a9eeee64b55d39a"
         "2192992a274fc1a836ba3c23a3feebbd454d4423643ce80e2a9ac94fa54ca49f" },
    { 5, "4634270f707b6a54daae7530460842e20e37ed265ceee9a43e8924aa" },
    { 6, "53048e2681941ef99b2e29b76b4c7dabe4c2d0c634fc6d46e0e2f13107e7af23" },
};

/* "abc", padded to one block of @bs bytes. */
static void sha_abc_block(uint8_t *block, int bs)
{
    memset(block, 0, bs);
    memcpy(block, "abc", 3);
    block[3] = 0x80;
    block[bs - 1] = 24;
}

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_sha(void)
{
    QTestState *qts = start();
    uint8_t block[128];
    uint8_t saved[32];

    g_assert_cmpuint(qtest_readl(qts, SHA_DATE), ==, 539232291);
    for (int i = 0; i < ARRAY_SIZE(sha_abc); i++) {
        int bs = sha_abc[i].mode >= 3 ? 128 : 64;

        sha_abc_block(block, bs);
        wr_bytes(qts, SHA_M(0), block, bs);
        qtest_writel(qts, SHA_MODE, sha_abc[i].mode);
        qtest_writel(qts, SHA_START, 1);
        g_assert_cmphex(qtest_readl(qts, SHA_BUSY), ==, 1);
        qtest_clock_step(qts, 10000);
        g_assert_cmphex(qtest_readl(qts, SHA_BUSY), ==, 0);
        check_hex(qts, SHA_H(0), sha_abc[i].digest);
    }

    /*
     * Interleaving: the second block of a two-block SHA-256 message after
     * another hash, with SHA_H restored (FIPS 180-4's two-block example).
     */
    {
        const char *msg = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmno"
                          "mnopnopq";
        uint8_t m2[64] = { 0 };

        memset(block, 0, 64);
        memcpy(block, msg, 56);
        block[56] = 0x80;
        wr_bytes(qts, SHA_M(0), block, 64);
        qtest_writel(qts, SHA_MODE, 2);
        qtest_writel(qts, SHA_START, 1);
        qtest_clock_step(qts, 10000);
        rd_bytes(qts, SHA_H(0), saved, 32);

        sha_abc_block(block, 64);
        wr_bytes(qts, SHA_M(0), block, 64);
        qtest_writel(qts, SHA_MODE, 0);
        qtest_writel(qts, SHA_START, 1);
        qtest_clock_step(qts, 10000);
        check_hex(qts, SHA_H(0), sha_abc[0].digest);

        stq_be_p(m2 + 56, 448);
        wr_bytes(qts, SHA_M(0), m2, 64);
        wr_bytes(qts, SHA_H(0), saved, 32);
        qtest_writel(qts, SHA_MODE, 2);
        qtest_writel(qts, SHA_CONTINUE, 1);
        qtest_clock_step(qts, 10000);
        check_hex(qts, SHA_H(0), "248d6a61d20638b8e5c026930c3e6039"
                                 "a33ce45964ff2167f6ecedd419db06c1");
    }

    /* SHA-512/t: SHA_START computes t = 56's initial value. */
    sha_abc_block(block, 128);
    qtest_writel(qts, SHA_MODE, 7);
    qtest_writel(qts, SHA_T_STRING, 0x35368000);
    qtest_writel(qts, SHA_T_LENGTH, 0x50);
    qtest_writel(qts, SHA_START, 1);
    qtest_clock_step(qts, 10000);
    wr_bytes(qts, SHA_M(0), block, 128);
    qtest_writel(qts, SHA_CONTINUE, 1);
    qtest_clock_step(qts, 10000);
    check_hex(qts, SHA_H(0), "ad5b8ae73d205e");
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_sha_dma(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, src = MEM + 0x1000;
    uint8_t blocks[128];

    /* SHA-256 of FIPS 180-4's two-block message by DMA. */
    memset(blocks, 0, sizeof(blocks));
    memcpy(blocks, "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
           56);
    blocks[56] = 0x80;
    stq_be_p(blocks + 120, 448);
    qtest_memwrite(qts, src, blocks, 128);
    desc(qts, tx, OWNER | SUC_EOF | DW0(128, 128), src, 0);
    axi_periph(qts, PERI_SHA, tx, 0);
    qtest_writel(qts, SHA_MODE, 2);
    qtest_writel(qts, SHA_DMA_BLOCK_NUM, 2);
    qtest_writel(qts, SHA_IRQ_ENA, 1);
    qtest_writel(qts, SHA_DMA_START, 1);
    g_assert_cmphex(qtest_readl(qts, SHA_BUSY), ==, 1);
    wait_for(qts, SHA_BUSY, 1, 0, 1000 * 1000);
    check_hex(qts, SHA_H(0), "248d6a61d20638b8e5c026930c3e6039"
                             "a33ce45964ff2167f6ecedd419db06c1");
    g_assert_true(source_level(qts, SRC_SHA));
    qtest_writel(qts, SHA_CLEAR_IRQ, 1);
    g_assert_false(source_level(qts, SRC_SHA));
    qtest_quit(qts);
}

/* RSA */

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_rsa(void)
{
    QTestState *qts = start();
    uint32_t m = 0xfffffffb, mprime;
    uint64_t rmod;

    /* Out of reset the memories are cleared first. */
    g_assert_cmpuint(qtest_readl(qts, RSA_DATE), ==, 538969624);
    g_assert_cmphex(qtest_readl(qts, RSA_CONSTANT_TIME), ==, 1);
    wait_for(qts, RSA_QUERY_CLEAN, 1, 1, 100000);
    g_assert_cmphex(qtest_readl(qts, RSA_QUERY_IDLE), ==, 1);

    /* One word: 2^10 mod 0xfffffffb, with r = R^2 mod M = 25. */
    rmod = (1ULL << 32) % m;
    mprime = 1;
    for (int i = 0; i < 5; i++) {
        mprime *= 2 - m * mprime;
    }
    mprime = -mprime;
    qtest_writel(qts, RSA_MODE, 0);
    qtest_writel(qts, RSA_M_PRIME, mprime);
    qtest_writel(qts, RSA_M_MEM, m);
    qtest_writel(qts, RSA_X_MEM, 2);
    qtest_writel(qts, RSA_Y_MEM, 10);
    qtest_writel(qts, RSA_Z_MEM, rmod * rmod % m);
    qtest_writel(qts, RSA_INT_ENA, 1);
    qtest_writel(qts, RSA_START_MODEXP, 1);
    g_assert_cmphex(qtest_readl(qts, RSA_QUERY_IDLE), ==, 0);
    /* The memories cannot be reached while it computes. */
    g_assert_cmphex(qtest_readl(qts, RSA_M_MEM), ==, 0);
    wait_for(qts, RSA_QUERY_IDLE, 1, 1, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, RSA_Z_MEM), ==, 1024);
    g_assert_true(source_level(qts, SRC_RSA));
    qtest_writel(qts, RSA_INT_CLR, 1);
    g_assert_false(source_level(qts, SRC_RSA));

    /* 1024 bits, without and with the acceleration options. */
    qtest_writel(qts, RSA_MODE, 31);
    qtest_writel(qts, RSA_M_PRIME, RSA_MPRIME);
    wr_words(qts, RSA_M_MEM, rsa_m, 32);
    wr_words(qts, RSA_Y_MEM, rsa_y, 32);
    wr_words(qts, RSA_X_MEM, rsa_x, 32);
    wr_words(qts, RSA_Z_MEM, rsa_r, 32);
    qtest_writel(qts, RSA_START_MODEXP, 1);
    /* 2050 multiplications of 3328 cycles at 40 MHz: about 170.6 ms. */
    qtest_clock_step(qts, 170 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, RSA_QUERY_IDLE), ==, 0);
    wait_for(qts, RSA_QUERY_IDLE, 1, 1, 2 * 1000 * 1000);
    check_words(qts, RSA_Z_MEM, rsa_z, 32);

    qtest_writel(qts, RSA_CONSTANT_TIME, 0);
    qtest_writel(qts, RSA_SEARCH_ENABLE, 1);
    qtest_writel(qts, RSA_SEARCH_POS, 1023);
    wr_words(qts, RSA_X_MEM, rsa_x, 32);
    wr_words(qts, RSA_Z_MEM, rsa_r, 32);
    qtest_writel(qts, RSA_START_MODEXP, 1);
    wait_for(qts, RSA_QUERY_IDLE, 1, 1, 200 * 1000 * 1000);
    check_words(qts, RSA_Z_MEM, rsa_z, 32);

    /* A search position below Y's top bit gives a wrong result. */
    qtest_writel(qts, RSA_SEARCH_POS, 1000);
    wr_words(qts, RSA_X_MEM, rsa_x, 32);
    wr_words(qts, RSA_Z_MEM, rsa_r, 32);
    qtest_writel(qts, RSA_START_MODEXP, 1);
    wait_for(qts, RSA_QUERY_IDLE, 1, 1, 200 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, RSA_Z_MEM), !=, rsa_z[0]);
    qtest_writel(qts, RSA_SEARCH_ENABLE, 0);
    qtest_writel(qts, RSA_CONSTANT_TIME, 1);

    /* Modular multiplication. */
    wr_words(qts, RSA_X_MEM, rsa_x, 32);
    wr_words(qts, RSA_Z_MEM, rsa_r, 32);
    qtest_writel(qts, RSA_START_MODMULT, 1);
    wait_for(qts, RSA_QUERY_IDLE, 1, 1, 10 * 1000 * 1000);
    check_words(qts, RSA_Z_MEM, rsa_xy_mod, 32);
    check_words(qts, RSA_X_MEM, rsa_x, 32);

    /* Multiplication: 512 by 512 bits, Y in Z_MEM's upper half. */
    qtest_writel(qts, RSA_MODE, 31);
    wr_words(qts, RSA_X_MEM, mul_a, 16);
    wr_words(qts, RSA_Z_MEM + 64, mul_b, 16);
    qtest_writel(qts, RSA_START_MULT, 1);
    wait_for(qts, RSA_QUERY_IDLE, 1, 1, 10 * 1000 * 1000);
    check_words(qts, RSA_Z_MEM, mul_ab, 32);

    /* HP_SYSTEM_RSA_PD_CTRL_REG's power-down loses the memories. */
    qtest_writel(qts, HP_SYS + 0x1d0, 1);
    g_assert_cmphex(qtest_readl(qts, RSA_Z_MEM), ==, 0);
    qtest_writel(qts, HP_SYS + 0x1d0, 2);
    g_assert_cmphex(qtest_readl(qts, RSA_Z_MEM), ==, 0);
    qtest_writel(qts, RSA_Z_MEM, 5);
    g_assert_cmphex(qtest_readl(qts, RSA_Z_MEM), ==, 5);
    qtest_quit(qts);
}

/* ECC */

static void ecc_run(QTestState *qts, uint32_t conf)
{
    qtest_writel(qts, ECC_CONF, conf | ECC_START);
    g_assert_cmphex(qtest_readl(qts, ECC_CONF) & ECC_START, ==, ECC_START);
    wait_for(qts, ECC_CONF, ECC_START, 0, 100 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, ECC_INT_RAW), ==, 1);
    qtest_writel(qts, ECC_INT_CLR, 1);
}

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_ecc(void)
{
    QTestState *qts = start();
    uint32_t jx[8], jy[8], jz[8];

    g_assert_cmpuint(qtest_readl(qts, ECC + 0xfc), ==, 37781792);
    qtest_writel(qts, ECC_INT_ENA, 1);

    /* P-256 point multiplication k G. */
    wr_words(qts, ECC_K, p256_k, 8);
    wr_words(qts, ECC_PX, p256_gx, 8);
    wr_words(qts, ECC_PY, p256_gy, 8);
    qtest_writel(qts, ECC_CONF, ECC_CURVE(1) | ECC_MODE(0) | ECC_START);
    g_assert_true(qtest_readl(qts, ECC_CONF) & ECC_START);
    g_assert_cmphex(qtest_readl(qts, ECC_PX), ==, 0);
    wait_for(qts, ECC_CONF, ECC_START, 0, 100 * 1000 * 1000);
    g_assert_true(source_level(qts, SRC_ECC));
    qtest_writel(qts, ECC_INT_CLR, 1);
    g_assert_false(source_level(qts, SRC_ECC));
    check_words(qts, ECC_PX, p256_qx, 8);
    check_words(qts, ECC_PY, p256_qy, 8);

    /* Verification of the result, and of a point off the curve. */
    ecc_run(qts, ECC_CURVE(1) | ECC_MODE(2));
    g_assert_true(qtest_readl(qts, ECC_CONF) & ECC_VERIFIED);
    qtest_writel(qts, ECC_PY, p256_qy[0] ^ 1);
    ecc_run(qts, ECC_CURVE(1) | ECC_MODE(2));
    g_assert_false(qtest_readl(qts, ECC_CONF) & ECC_VERIFIED);

    /* Jacobian multiplication, and its result verified as Jacobian. */
    wr_words(qts, ECC_PX, p256_gx, 8);
    wr_words(qts, ECC_PY, p256_gy, 8);
    ecc_run(qts, ECC_CURVE(1) | ECC_MODE(4));
    for (int i = 0; i < 8; i++) {
        jx[i] = qtest_readl(qts, ECC_QX + 4 * i);
        jy[i] = qtest_readl(qts, ECC_QY + 4 * i);
        jz[i] = qtest_readl(qts, ECC_QZ + 4 * i);
    }
    ecc_run(qts, ECC_CURVE(1) | ECC_MODE(6));
    g_assert_true(qtest_readl(qts, ECC_CONF) & ECC_VERIFIED);

    /* Adding (Px, Py, 1) = G to kG: (k + 1) G, checked with mode 3. */
    wr_words(qts, ECC_QX, jx, 8);
    wr_words(qts, ECC_QY, jy, 8);
    wr_words(qts, ECC_QZ, jz, 8);
    ecc_run(qts, ECC_CURVE(1) | ECC_MODE(5));
    {
        uint32_t ax[8], ay[8], k1[8];
        uint64_t c = 1;

        for (int i = 0; i < 8; i++) {
            ax[i] = qtest_readl(qts, ECC_PX + 4 * i);
            ay[i] = qtest_readl(qts, ECC_PY + 4 * i);
            c += p256_k[i];
            k1[i] = c;
            c >>= 32;
        }
        wr_words(qts, ECC_K, k1, 8);
        wr_words(qts, ECC_PX, p256_gx, 8);
        wr_words(qts, ECC_PY, p256_gy, 8);
        ecc_run(qts, ECC_CURVE(1) | ECC_MODE(3));
        g_assert_true(qtest_readl(qts, ECC_CONF) & ECC_VERIFIED);
        check_words(qts, ECC_PX, ax, 8);
        check_words(qts, ECC_PY, ay, 8);
    }

    /* P-384 modular arithmetic, modulo n and modulo p. */
    wr_words(qts, ECC_PX, p384_a, 12);
    wr_words(qts, ECC_PY, p384_b, 12);
    ecc_run(qts, ECC_CURVE(2) | ECC_MODE(10));
    check_words(qts, ECC_PY, p384_mul_n, 12);
    wr_words(qts, ECC_PY, p384_b, 12);
    ecc_run(qts, ECC_CURVE(2) | ECC_MODE(11));
    check_words(qts, ECC_PY, p384_div_n, 12);
    wr_words(qts, ECC_PY, p384_b, 12);
    ecc_run(qts, ECC_CURVE(2) | ECC_MOD_BASE_P | ECC_MODE(8));
    check_words(qts, ECC_PX, p384_add_p, 12);
    wr_words(qts, ECC_PX, p384_a, 12);
    ecc_run(qts, ECC_CURVE(2) | ECC_MOD_BASE_P | ECC_MODE(9));
    check_words(qts, ECC_PX, p384_sub_p, 12);

    /* HP_SYSTEM_ECC_PD_CTRL_REG's power-down. */
    qtest_writel(qts, HP_SYS + 0x1d4, 1);
    g_assert_cmphex(qtest_readl(qts, ECC_PX), ==, 0);
    qtest_writel(qts, HP_SYS + 0x1d4, 2);
    qtest_quit(qts);
}

/* The random number generator */

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_rng(void)
{
    QTestState *qts = start();
    uint32_t a, b, c;

    g_assert_cmphex(qtest_readl(qts, RNG_CFG), ==, (255u << 1) | (1u << 9));
    g_assert_cmphex(qtest_readl(qts, RNG_DATE), ==, 2425091);
    /* Until RNG_CLK_EN the generator does not run. */
    g_assert_cmphex(qtest_readl(qts, RNG_DATA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RNG_DATA), ==, 0);

    qtest_writel(qts, RNG_DATE, qtest_readl(qts, RNG_DATE) | RNG_CLK_EN);
    a = qtest_readl(qts, RNG_DATA);
    b = qtest_readl(qts, RNG_DATA);
    c = qtest_readl(qts, LP_SYS_RNG_DATA);
    g_assert_true(a != b || b != c);
    g_assert_true(a || b || c);

    /* RNG_RSTN low clears it and stops it. */
    qtest_writel(qts, RNG_RSTN, 0);
    g_assert_cmphex(qtest_readl(qts, RNG_DATA), ==, 0);
    qtest_writel(qts, RNG_RSTN, 1);

    /* The BUF_CHAIN's samples, one each 256 cycles of its clock. */
    qtest_writel(qts, RNG_CFG, qtest_readl(qts, RNG_CFG) | 1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmpuint(qtest_readl(qts, RNG_CFG) >> 24, !=, 0);
    qtest_quit(qts);
}

/* The HUK Generator and the Key Manager */

static void huk_continue(QTestState *qts, uint32_t state)
{
    qtest_writel(qts, HUK_START, 2);
    wait_for(qts, HUK_STATE, 3, state, 10 * 1000 * 1000);
}

static void km_step(QTestState *qts, uint32_t bit, uint32_t state)
{
    qtest_writel(qts, KM_START, bit);
    wait_for(qts, KM_STATE, 3, state, 100 * 1000 * 1000);
}

/* HMAC-SHA-256 with the Key Manager's key of one padded block, "abc". */
static void hmac_km(QTestState *qts, uint8_t out[32])
{
    uint8_t block[64];

    memset(block, 0, sizeof(block));
    memcpy(block, "abc", 3);
    block[3] = 0x80;
    stq_be_p(block + 56, (64 + 3) * 8);
    qtest_writel(qts, HMAC_SET_START, 1);
    qtest_writel(qts, HMAC_SET_PURPOSE, 8);
    qtest_writel(qts, HMAC_SET_KEY, 7);
    qtest_writel(qts, HMAC_SET_PARA_END, 1);
    wait_for(qts, HMAC_QUERY_BUSY, 1, 0, 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, HMAC_QUERY_ERROR), ==, 0);
    wr_bytes(qts, HMAC_WR_MESSAGE(0), block, 64);
    qtest_writel(qts, HMAC_SET_MSG_ONE, 1);
    wait_for(qts, HMAC_QUERY_BUSY, 1, 0, 1000 * 1000);
    qtest_writel(qts, HMAC + 0xf4, 1);
    wait_for(qts, HMAC_QUERY_BUSY, 1, 0, 1000 * 1000);
    rd_bytes(qts, HMAC_RD_RESULT(0), out, 32);
    qtest_writel(qts, HMAC_SET_RESULT_END, 2);
}

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_huk_km(void)
{
    QTestState *qts = start();
    uint32_t info[HUK_INFO_WORDS], key_info[16];
    uint8_t mac1[32], mac2[32];
    int i;

    g_assert_cmphex(qtest_readl(qts, HUK_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, KM_HUK_VLD), ==, 0);

    /* A deployment without a HUK fails. */
    qtest_writel(qts, KM_CONF, KM_CONF_SET(0, 6));
    km_step(qts, 1, 1);
    km_step(qts, 2, 2);
    g_assert_cmphex(qtest_readl(qts, KM_RESULT), ==, 0);
    km_step(qts, 2, 0);
    g_assert_cmphex(qtest_readl(qts, KM_KEY_VLD), ==, 0);

    /* HUK generation: EFUSE_KM_HUK_GEN_STATE is 0, an even count. */
    qtest_writel(qts, HUK_CONF, 1);
    qtest_writel(qts, HUK_START, 1);
    g_assert_cmphex(qtest_readl(qts, HUK_STATE), ==, 3);
    wait_for(qts, HUK_STATE, 3, 1, 10 * 1000 * 1000);
    huk_continue(qts, 2);
    for (i = 0; i < HUK_INFO_WORDS; i++) {
        info[i] = qtest_readl(qts, HUK_INFO + 4 * i);
    }
    huk_continue(qts, 0);
    g_assert_cmphex(qtest_readl(qts, HUK_STATUS), ==, 1);
    g_assert_cmphex(qtest_readl(qts, HUK_INT_RAW), ==, 7);
    g_assert_true(source_level(qts, SRC_LP_HUK) == false);
    g_assert_cmphex(qtest_readl(qts, KM_HUK_VLD), ==, 1);
    /* huk_info is gone from the memory once the process ends. */
    g_assert_cmphex(qtest_readl(qts, HUK_INFO), ==, 0);

    /* A random HMAC key, used through HMAC key 7. */
    qtest_writel(qts, KM_CONF, KM_CONF_SET(0, 6));
    km_step(qts, 1, 1);
    km_step(qts, 2, 2);
    g_assert_cmphex(qtest_readl(qts, KM_RESULT), ==, 1);
    for (i = 0; i < 16; i++) {
        key_info[i] = qtest_readl(qts, KM_PUBLIC_INFO + 4 * i);
    }
    km_step(qts, 2, 0);
    g_assert_cmphex(qtest_readl(qts, KM_KEY_VLD), ==, 1u << 3);
    g_assert_cmphex(qtest_readl(qts, KM_INT_RAW), ==, 7);
    hmac_km(qts, mac1);

    /* RST_EN_KM clears the key; recovery from key_info restores it. */
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN2,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN2) | RST_EN_KM);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN2,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN2) & ~RST_EN_KM);
    g_assert_cmphex(qtest_readl(qts, KM_KEY_VLD), ==, 0);
    qtest_writel(qts, HMAC_SET_START, 1);
    qtest_writel(qts, HMAC_SET_PURPOSE, 8);
    qtest_writel(qts, HMAC_SET_KEY, 7);
    qtest_writel(qts, HMAC_SET_PARA_END, 1);
    g_assert_cmphex(qtest_readl(qts, HMAC_QUERY_ERROR), ==, 1);
    qtest_writel(qts, HMAC_SET_RESULT_END, 2);

    qtest_writel(qts, KM_CONF, KM_CONF_SET(4, 6));
    km_step(qts, 1, 1);
    wr_words(qts, KM_ASSIST_INFO, key_info, 16);
    km_step(qts, 2, 2);
    g_assert_cmphex(qtest_readl(qts, KM_RESULT), ==, 1);
    km_step(qts, 2, 0);
    g_assert_cmphex(qtest_readl(qts, KM_KEY_VLD), ==, 1u << 3);
    hmac_km(qts, mac2);
    g_assert_cmpmem(mac1, 32, mac2, 32);

    /* With KEYMNG_USE_EFUSE_KEY's HMAC bit, key 7 is refused. */
    qtest_writel(qts, KM_STATIC, qtest_readl(qts, KM_STATIC) | (1u << 2));
    qtest_writel(qts, HMAC_SET_START, 1);
    qtest_writel(qts, HMAC_SET_PURPOSE, 8);
    qtest_writel(qts, HMAC_SET_KEY, 7);
    qtest_writel(qts, HMAC_SET_PARA_END, 1);
    g_assert_cmphex(qtest_readl(qts, HMAC_QUERY_ERROR), ==, 1);
    qtest_writel(qts, HMAC_SET_RESULT_END, 2);
    /* ... and once locked it stays. */
    qtest_writel(qts, KM_LOCK, 1u << 2);
    qtest_writel(qts, KM_STATIC, qtest_readl(qts, KM_STATIC) & ~(1u << 2));
    g_assert_cmphex(qtest_readl(qts, KM_STATIC) & (1u << 2), ==, 1u << 2);

    /* The HUK is lost with its reset, and recovered from huk_info. */
    qtest_writel(qts, LP_CLKRST_LP_RST_EN,
                 qtest_readl(qts, LP_CLKRST_LP_RST_EN) | RST_EN_LP_HUK);
    qtest_writel(qts, LP_CLKRST_LP_RST_EN,
                 qtest_readl(qts, LP_CLKRST_LP_RST_EN) & ~RST_EN_LP_HUK);
    g_assert_cmphex(qtest_readl(qts, HUK_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, KM_HUK_VLD), ==, 0);
    qtest_writel(qts, HUK_CONF, 0);
    qtest_writel(qts, HUK_START, 1);
    wait_for(qts, HUK_STATE, 3, 1, 10 * 1000 * 1000);
    wr_words(qts, HUK_INFO, info, HUK_INFO_WORDS);
    huk_continue(qts, 2);
    huk_continue(qts, 0);
    g_assert_cmphex(qtest_readl(qts, HUK_STATUS), ==, 1);

    /* Corrupted huk_info: the HUK is invalid, at the highest risk. */
    info[20] ^= 1;
    qtest_writel(qts, HUK_START, 1);
    wait_for(qts, HUK_STATE, 3, 1, 10 * 1000 * 1000);
    wr_words(qts, HUK_INFO, info, HUK_INFO_WORDS);
    huk_continue(qts, 2);
    huk_continue(qts, 0);
    g_assert_cmphex(qtest_readl(qts, HUK_STATUS), ==, 2 | (7u << 2));
    g_assert_cmphex(qtest_readl(qts, KM_HUK_VLD), ==, 0);
    qtest_quit(qts);
}

/* RSA_DS without a key */

/* [spec:nuos:req:emu.esp32p4.crypto/test] */
static void test_ds_no_key(void)
{
    QTestState *qts = start();

    qtest_clock_step(qts, 100000);
    qtest_writel(qts, DS_SET_START, 1);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(qtest_readl(qts, DS_QUERY_BUSY), ==, 1);
    g_assert_cmphex(qtest_readl(qts, DS_QUERY_KEY_WRONG), ==, 0);
    /* The AES accelerator is occupied meanwhile. */
    g_assert_cmphex(qtest_readl(qts, AES_DATE), ==, 0);
    qtest_writel(qts, DS_SET_FINISH, 1);
    wait_for(qts, DS_QUERY_BUSY, 1, 0, 100000);
    g_assert_cmpuint(qtest_readl(qts, AES_DATE), ==, 36774000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4-crypto/aes-typical", test_aes_typical);
    qtest_add_func("/esp32p4-crypto/aes-dma", test_aes_dma);
    qtest_add_func("/esp32p4-crypto/sha", test_sha);
    qtest_add_func("/esp32p4-crypto/sha-dma", test_sha_dma);
    qtest_add_func("/esp32p4-crypto/rsa", test_rsa);
    qtest_add_func("/esp32p4-crypto/ecc", test_ecc);
    qtest_add_func("/esp32p4-crypto/rng", test_rng);
    qtest_add_func("/esp32p4-crypto/huk-km", test_huk_km);
    qtest_add_func("/esp32p4-crypto/ds-no-key", test_ds_no_key);
    return g_test_run();
}
