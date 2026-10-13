/*
 * ESP32-P4 AES accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 27 (AES Accelerator), ESP-IDF's aes_reg.h for v3 and its aes_ll.h and
 * mbedtls port, which show how software drives the accelerator.
 *
 * Typical AES: AES_MODE_REG selects AES-128 or AES-256 and encryption or
 * decryption (the AES-192 values are reserved on this chip, and a trigger
 * with one is ignored and logged); AES_TRIGGER_REG ciphers TEXT_IN into
 * TEXT_OUT, AES_STATE_REG reading 1 (WORK) until it is done.
 *
 * DMA-AES (AES_DMA_ENABLE_REG set): the accelerator is GDMA-AXI's AES
 * peripheral. A trigger takes AES_BLOCK_NUM_REG 16-byte blocks from the
 * TX channel that selects it, ciphers them in the block mode of
 * AES_BLOCK_MODE_REG (ECB, CBC, OFB, CTR with AES_INC_SEL_REG's INC32 or
 * INC128, CFB8, CFB128) with AES_IV_MEM's IV or initial counter block,
 * and gives the results to the RX channel, the last byte ending the
 * frame (suc_eof). AES_IV_MEM then holds the chaining value a next
 * operation continues from (the last ciphertext block for CBC and
 * CFB128, OFB's last output block, CTR's next counter block, CFB8's
 * shifted register), as ESP-IDF reads it back. AES_STATE_REG reads 2
 * (DONE) at the end, AES_DMA_DONE_INT raises AES_INTR while
 * AES_INT_ENA_REG is set, and any write to AES_DMA_EXIT_REG returns the
 * accelerator to IDLE: ESP-IDF's aes_ll_dma_exit() writes 0 to it.
 *
 * GCM (block mode 6): the trigger computes the hash subkey H =
 * CIPH_K(0^128) into AES_H_MEM and returns to IDLE, which ESP-IDF waits
 * for (the TRM says DONE); AES_CONTINUE_REG then takes from the TX
 * channel AES_AAD_BLOCK_NUM_REG blocks of padded AAD, the
 * AES_BLOCK_NUM_REG blocks of text, which it ciphers with GCTR from
 * inc32(J0) (AES_J0_MEM) to the RX channel, and the 16-byte block of the
 * AAD and text lengths, all into GHASH, and leaves T0 = CIPH_K(J0) xor
 * the hash in AES_T0_MEM (NIST SP 800-38D). In encryption the bits of the
 * last ciphertext block past AES_REMAINDER_BIT_NUM_REG (0 for a whole
 * block) go into GHASH as zeros; in decryption the padded ciphertext does,
 * as the TRM describes.
 *
 * The data, key and memories hold bytes in the order the TRM's tables
 * give (byte 0 of a block in bits 7:0 of the first word). AES_ENDIAN_REG,
 * which the P4's TRM does not describe, reorders them as the ESP32's did,
 * with this chip's reset value 0 being the order above: per 2-bit field
 * (key, input, output), bit 0 reverses the bytes of each word and bit 1
 * the order of the words.
 *
 * Timing: a block takes 12 cycles of CRYPTO_AES_CLK with a 128-bit key and
 * 16 with a 256-bit one (the model's figures; the TRM gives none), and the
 * accelerator waits while the clock is stopped. Registers written while
 * the accelerator works are dropped and logged, as are writes while its
 * clock is stopped or it is held in reset; while the RSA_DS peripheral
 * occupies it its registers read as zero and take no writes, and it
 * resets when released.
 *
 * The pseudo-round function (AES_PSEUDO_REG) adds rounds with a random
 * pseudo-key that do not change the result; it changes only the power
 * profile and the time, by a random amount, and is held.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_aes.h"
#include "migration/vmstate.h"

#define A_KEY0              0x00
#define A_TEXT_IN0          0x20
#define A_TEXT_OUT0         0x30
#define A_MODE              0x40
#define A_ENDIAN            0x44
#define A_TRIGGER           0x48
#define A_STATE             0x4c
#define A_IV_MEM            0x50
#define A_H_MEM             0x60
#define A_J0_MEM            0x70
#define A_T0_MEM            0x80
#define A_DMA_ENABLE        0x90
#define A_BLOCK_MODE        0x94
#define A_BLOCK_NUM         0x98
#define A_INC_SEL           0x9c
#define A_AAD_BLOCK_NUM     0xa0
#define A_REMAINDER_BIT_NUM 0xa4
#define A_CONTINUE          0xa8
#define A_INT_CLEAR         0xac
#define A_INT_ENA           0xb0
#define A_DATE              0xb4
#define A_DMA_EXIT          0xb8
#define A_RX_RESET          0xc0
#define A_TX_RESET          0xc4
#define A_PSEUDO            0xd0

#define DATE_RESET          36774000
#define PSEUDO_RESET        ((7u << 7) | (2u << 5) | (2u << 1))
#define PSEUDO_MASK         0x3ff

enum {
    STATE_IDLE,
    STATE_WORK,
    STATE_DONE,
};

enum {
    BLOCK_ECB,
    BLOCK_CBC,
    BLOCK_OFB,
    BLOCK_CTR,
    BLOCK_CFB8,
    BLOCK_CFB128,
    BLOCK_GCM,
};

/* What the accelerator is doing. */
enum {
    PHASE_IDLE,
    PHASE_TYPICAL,      /* a Typical AES block */
    PHASE_DMA,          /* DMA-AES blocks */
    PHASE_GCM_HASH,     /* computing H */
    PHASE_GCM_WAIT,     /* H done, waiting for CONTINUE */
    PHASE_GCM_AAD,
    PHASE_GCM_TEXT,
    PHASE_GCM_LEN,
};

#define MODE_DECRYPT        BIT(2)

static bool aes_in_reset(ESP32P4AesState *s)
{
    for (int i = 0; i < ESP32P4_AES_RESETS; i++) {
        if (s->reset_in[i]) {
            return true;
        }
    }
    return false;
}

static void aes_update_irq(ESP32P4AesState *s)
{
    qemu_set_irq(s->irq, s->done_int && (s->int_ena & 1));
}

/* AES_MODE_REG's key length in bytes, or 0 for a reserved value. */
static unsigned aes_key_bytes(uint32_t mode)
{
    switch (mode & 3) {
    case 0:
        return 16;
    case 2:
        return 32;
    default:
        return 0;
    }
}

/* AES_ENDIAN_REG's reordering of a block, @e its 2-bit field. */
static void aes_reorder(uint8_t *b, unsigned words, unsigned e)
{
    uint8_t t[32];
    unsigned i, j;

    if (!e) {
        return;
    }
    for (i = 0; i < words; i++) {
        unsigned w = (e & 2) ? words - 1 - i : i;

        for (j = 0; j < 4; j++) {
            t[4 * i + j] = b[4 * w + ((e & 1) ? 3 - j : j)];
        }
    }
    memcpy(b, t, 4 * words);
}

static void aes_keys(ESP32P4AesState *s, AES_KEY *enc, AES_KEY *dec)
{
    unsigned len = aes_key_bytes(s->op_mode);
    uint8_t key[32];
    int i;

    for (i = 0; i < 8; i++) {
        stl_le_p(key + 4 * i, s->key[i]);
    }
    aes_reorder(key, len / 4, s->endian & 3);
    AES_set_encrypt_key(key, len * 8, enc);
    if (dec) {
        AES_set_decrypt_key(key, len * 8, dec);
    }
}

static uint64_t aes_block_cycles(ESP32P4AesState *s)
{
    return aes_key_bytes(s->op_mode) == 32 ? 16 : 12;
}

static void aes_inc(uint8_t ctr[16], bool inc128)
{
    int i;

    for (i = 15; i >= (inc128 ? 0 : 12); i--) {
        if (++ctr[i]) {
            break;
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_finish(ESP32P4AesState *s)
{
    s->phase = PHASE_IDLE;
    s->state = STATE_DONE;
    s->done_int = true;
    aes_update_irq(s);
}

/* One block of a DMA-AES block mode: in_buf to out_buf, IV updated. */
static void aes_dma_block(ESP32P4AesState *s)
{
    bool dec = s->op_mode & MODE_DECRYPT;
    AES_KEY enc, dk;
    uint8_t o[16];
    int i;

    aes_keys(s, &enc, &dk);
    switch (s->op_block_mode) {
    case BLOCK_ECB:
        if (dec) {
            AES_decrypt(s->in_buf, s->out_buf, &dk);
        } else {
            AES_encrypt(s->in_buf, s->out_buf, &enc);
        }
        break;
    case BLOCK_CBC:
        if (dec) {
            AES_decrypt(s->in_buf, o, &dk);
            for (i = 0; i < 16; i++) {
                s->out_buf[i] = o[i] ^ s->iv[i];
            }
            memcpy(s->iv, s->in_buf, 16);
        } else {
            for (i = 0; i < 16; i++) {
                o[i] = s->in_buf[i] ^ s->iv[i];
            }
            AES_encrypt(o, s->out_buf, &enc);
            memcpy(s->iv, s->out_buf, 16);
        }
        break;
    case BLOCK_OFB:
        AES_encrypt(s->iv, o, &enc);
        for (i = 0; i < 16; i++) {
            s->out_buf[i] = s->in_buf[i] ^ o[i];
        }
        memcpy(s->iv, o, 16);
        break;
    case BLOCK_CTR:
        AES_encrypt(s->iv, o, &enc);
        for (i = 0; i < 16; i++) {
            s->out_buf[i] = s->in_buf[i] ^ o[i];
        }
        aes_inc(s->iv, s->inc_sel & 1);
        break;
    case BLOCK_CFB8:
        for (i = 0; i < 16; i++) {
            uint8_t c;

            AES_encrypt(s->iv, o, &enc);
            s->out_buf[i] = s->in_buf[i] ^ o[0];
            c = dec ? s->in_buf[i] : s->out_buf[i];
            memmove(s->iv, s->iv + 1, 15);
            s->iv[15] = c;
        }
        break;
    case BLOCK_CFB128:
        AES_encrypt(s->iv, o, &enc);
        for (i = 0; i < 16; i++) {
            s->out_buf[i] = s->in_buf[i] ^ o[i];
        }
        memcpy(s->iv, dec ? s->in_buf : s->out_buf, 16);
        break;
    }
}

static void aes_ghash(ESP32P4AesState *s, const uint8_t *block)
{
    uint8_t x[16];
    int i;

    for (i = 0; i < 16; i++) {
        x[i] = s->ghash[i] ^ block[i];
    }
    esp32p4_gf128_mul(s->ghash, x, s->h);
}

/* One block of GCM's text: GCTR, and the ciphertext into GHASH. */
static void aes_gcm_text_block(ESP32P4AesState *s, bool last)
{
    bool dec = s->op_mode & MODE_DECRYPT;
    AES_KEY enc;
    uint8_t o[16], c[16];
    int i;

    aes_keys(s, &enc, NULL);
    AES_encrypt(s->ctr, o, &enc);
    aes_inc(s->ctr, false);
    for (i = 0; i < 16; i++) {
        s->out_buf[i] = s->in_buf[i] ^ o[i];
    }
    memcpy(c, dec ? s->in_buf : s->out_buf, 16);
    if (!dec && last && s->remainder_bit_num) {
        unsigned bits = s->remainder_bit_num;

        for (i = 0; i < 16; i++) {
            if (bits >= 8) {
                bits -= 8;
            } else {
                c[i] &= 0xff00 >> bits;
                bits = 0;
            }
        }
    }
    aes_ghash(s, c);
}

static void aes_pump(ESP32P4AesState *s);

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_op_done(void *opaque)
{
    ESP32P4AesState *s = opaque;
    AES_KEY enc, dec;
    uint8_t in[16], out[16];
    bool last;

    s->computing = false;
    switch (s->phase) {
    case PHASE_TYPICAL:
        memcpy(in, s->text_in, 16);
        aes_reorder(in, 4, (s->endian >> 2) & 3);
        aes_keys(s, &enc, &dec);
        if (s->op_mode & MODE_DECRYPT) {
            AES_decrypt(in, out, &dec);
        } else {
            AES_encrypt(in, out, &enc);
        }
        aes_reorder(out, 4, (s->endian >> 4) & 3);
        memcpy(s->text_out, out, 16);
        s->phase = PHASE_IDLE;
        s->state = STATE_IDLE;
        return;
    case PHASE_GCM_HASH:
        memset(in, 0, 16);
        aes_keys(s, &enc, NULL);
        AES_encrypt(in, s->h, &enc);
        s->phase = PHASE_GCM_WAIT;
        s->state = STATE_IDLE;
        return;
    case PHASE_DMA:
        aes_dma_block(s);
        s->left--;
        s->out_len = 16;
        s->out_pos = 0;
        s->out_last = !s->left;
        break;
    case PHASE_GCM_AAD:
        aes_ghash(s, s->in_buf);
        s->left--;
        break;
    case PHASE_GCM_TEXT:
        last = s->left == 1;
        aes_gcm_text_block(s, last);
        s->left--;
        s->out_len = 16;
        s->out_pos = 0;
        s->out_last = last;
        break;
    case PHASE_GCM_LEN:
        aes_ghash(s, s->in_buf);
        aes_keys(s, &enc, NULL);
        AES_encrypt(s->j0, out, &enc);
        for (int i = 0; i < 16; i++) {
            s->t0[i] = out[i] ^ s->ghash[i];
        }
        s->left = 0;
        break;
    default:
        return;
    }
    s->in_len = 0;
    aes_pump(s);
}

/* The next section of a GCM operation once one runs out of blocks. */
static void aes_gcm_next(ESP32P4AesState *s)
{
    while (!s->left) {
        switch (s->phase) {
        case PHASE_GCM_AAD:
            s->phase = PHASE_GCM_TEXT;
            s->left = s->text_total;
            break;
        case PHASE_GCM_TEXT:
            s->phase = PHASE_GCM_LEN;
            s->left = 1;
            break;
        default:
            aes_finish(s);
            return;
        }
    }
}

/*
 * Move the DMA operation on: give out the last result as the RX channel
 * takes it, take the next block from the TX channel and cipher it.
 */
/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_pump(ESP32P4AesState *s)
{
    bool eof;

    while (s->phase >= PHASE_DMA && s->phase != PHASE_GCM_WAIT &&
           s->phase != PHASE_GCM_HASH && !s->computing) {
        if (aes_in_reset(s) || !clock_is_enabled(s->clk)) {
            return;
        }
        if (s->out_pos < s->out_len) {
            uint8_t blk[16];
            size_t want = s->out_len - s->out_pos;
            size_t n;

            memcpy(blk, s->out_buf, 16);
            aes_reorder(blk, 4, (s->endian >> 4) & 3);
            n = esp32p4_gdma_push(s->gdma, ESP32P4_GDMA_AXI_AES,
                                  blk + s->out_pos, want,
                                  s->out_last ? ESP32P4_GDMA_EOF : 0);
            s->out_pos += n;
            if (s->out_pos < s->out_len) {
                return;
            }
            s->out_len = s->out_pos = 0;
            s->out_last = false;
            continue;
        }
        if (s->phase == PHASE_DMA) {
            if (!s->left) {
                aes_finish(s);
                return;
            }
        } else {
            aes_gcm_next(s);
            if (s->phase == PHASE_IDLE) {
                return;
            }
        }
        if (s->in_len < 16) {
            s->in_len += esp32p4_gdma_pull(s->gdma, ESP32P4_GDMA_AXI_AES,
                                           s->in_buf + s->in_len,
                                           16 - s->in_len, &eof);
            if (s->in_len < 16) {
                return;
            }
            aes_reorder(s->in_buf, 4, (s->endian >> 2) & 3);
        }
        s->computing = true;
        esp32p4_crypto_timer_start(&s->op, aes_block_cycles(s));
    }
}

static void aes_gdma_ready(void *opaque)
{
    aes_pump(opaque);
}

/*
 * [spec:nuos:req:emu.esp32p4.crypto]
 * GDMA's register access: the blocks whose cycles have run by then are
 * done, their output in GDMA's channel and its EOF.
 */
static void aes_gdma_sync(void *opaque, int64_t when)
{
    ESP32P4AesState *s = opaque;

    esp32p4_crypto_timer_sync(&s->op, when);
}

static const ESP32P4GdmaPeriphOps aes_gdma_ops = {
    .tx_ready = aes_gdma_ready,
    .rx_ready = aes_gdma_ready,
    .sync = aes_gdma_sync,
};

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_trigger(ESP32P4AesState *s)
{
    if (!aes_key_bytes(s->mode)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: AES_MODE %u is reserved; the "
                      "trigger is ignored\n", TYPE_ESP32P4_AES, s->mode);
        return;
    }
    s->op_mode = s->mode;
    s->op_block_mode = s->block_mode;
    s->in_len = 0;
    s->out_len = s->out_pos = 0;
    if (!(s->dma_enable & 1)) {
        s->phase = PHASE_TYPICAL;
        s->state = STATE_WORK;
        s->computing = true;
        esp32p4_crypto_timer_start(&s->op, aes_block_cycles(s));
        return;
    }
    if (s->block_mode == BLOCK_GCM) {
        s->phase = PHASE_GCM_HASH;
        s->state = STATE_WORK;
        s->computing = true;
        esp32p4_crypto_timer_start(&s->op, aes_block_cycles(s));
        return;
    }
    if (s->block_mode > BLOCK_GCM) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: AES_BLOCK_MODE 7 is reserved; "
                      "the trigger is ignored\n", TYPE_ESP32P4_AES);
        return;
    }
    s->phase = PHASE_DMA;
    s->state = STATE_WORK;
    s->left = s->block_num;
    aes_pump(s);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_continue(ESP32P4AesState *s)
{
    if (s->phase != PHASE_GCM_WAIT) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: AES_CONTINUE_REG with no GCM "
                      "operation waiting for it is ignored\n",
                      TYPE_ESP32P4_AES);
        return;
    }
    memset(s->ghash, 0, 16);
    memcpy(s->ctr, s->j0, 16);
    aes_inc(s->ctr, false);
    s->text_total = s->block_num;
    s->phase = PHASE_GCM_AAD;
    s->left = s->aad_block_num;
    s->state = STATE_WORK;
    aes_pump(s);
}

static uint64_t aes_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4AesState *s = opaque;
    hwaddr reg = addr & ~3;
    uint32_t v = 0;

    esp32p4_crypto_timer_sync(&s->op, esp32_line_time());
    if (s->occupied) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read at 0x%" HWADDR_PRIx
                      " while the RSA_DS peripheral occupies the "
                      "accelerator\n", TYPE_ESP32P4_AES, addr);
        return 0;
    }
    switch (reg) {
    case A_KEY0 ... A_KEY0 + 0x1c:
        v = s->key[(reg - A_KEY0) / 4];
        break;
    case A_TEXT_IN0 ... A_TEXT_IN0 + 0xc:
        v = ldl_le_p(s->text_in + reg - A_TEXT_IN0);
        break;
    case A_TEXT_OUT0 ... A_TEXT_OUT0 + 0xc:
        v = ldl_le_p(s->text_out + reg - A_TEXT_OUT0);
        break;
    case A_MODE:
        v = s->mode;
        break;
    case A_ENDIAN:
        v = s->endian;
        break;
    case A_STATE:
        v = s->state;
        break;
    case A_IV_MEM ... A_IV_MEM + 0xc:
        v = ldl_le_p(s->iv + reg - A_IV_MEM);
        break;
    case A_H_MEM ... A_H_MEM + 0xc:
        v = ldl_le_p(s->h + reg - A_H_MEM);
        break;
    case A_J0_MEM ... A_J0_MEM + 0xc:
        v = ldl_le_p(s->j0 + reg - A_J0_MEM);
        break;
    case A_T0_MEM ... A_T0_MEM + 0xc:
        v = ldl_le_p(s->t0 + reg - A_T0_MEM);
        break;
    case A_DMA_ENABLE:
        v = s->dma_enable;
        break;
    case A_BLOCK_MODE:
        v = s->block_mode;
        break;
    case A_BLOCK_NUM:
        v = s->block_num;
        break;
    case A_INC_SEL:
        v = s->inc_sel;
        break;
    case A_AAD_BLOCK_NUM:
        v = s->aad_block_num;
        break;
    case A_REMAINDER_BIT_NUM:
        v = s->remainder_bit_num;
        break;
    case A_INT_ENA:
        v = s->int_ena;
        break;
    case A_DATE:
        v = s->date;
        break;
    case A_PSEUDO:
        v = s->pseudo;
        break;
    case A_TRIGGER:
    case A_CONTINUE:
    case A_INT_CLEAR:
    case A_DMA_EXIT:
    case A_RX_RESET:
    case A_TX_RESET:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_AES, addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* The registers an operation in progress uses, which it does not let go. */
static bool aes_reg_busy(hwaddr reg)
{
    return reg < A_STATE || (reg >= A_IV_MEM && reg < A_T0_MEM) ||
           (reg >= A_DMA_ENABLE && reg <= A_REMAINDER_BIT_NUM);
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    ESP32P4AesState *s = opaque;
    hwaddr reg = addr & ~3;
    int shift = (addr & 3) * 8;
    uint32_t v;

    esp32p4_crypto_timer_sync(&s->op, esp32_line_time());
    if (s->occupied || aes_in_reset(s) || !clock_is_enabled(s->clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " dropped: the accelerator is %s\n", TYPE_ESP32P4_AES,
                      addr, s->occupied ? "occupied by the RSA_DS peripheral"
                      : aes_in_reset(s) ? "held in reset"
                      : "not clocked (CRYPTO_AES_CLK)");
        return;
    }
    if (s->state == STATE_WORK && aes_reg_busy(reg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write at 0x%" HWADDR_PRIx
                      " while the accelerator works is dropped\n",
                      TYPE_ESP32P4_AES, addr);
        return;
    }
    v = deposit32(0, shift, size * 8, value);

#define MERGE(old) deposit32((old), shift, size * 8, value)
    switch (reg) {
    case A_KEY0 ... A_KEY0 + 0x1c:
        s->key[(reg - A_KEY0) / 4] = MERGE(s->key[(reg - A_KEY0) / 4]);
        break;
    case A_TEXT_IN0 ... A_TEXT_IN0 + 0xc:
        stl_le_p(s->text_in + reg - A_TEXT_IN0,
                 MERGE(ldl_le_p(s->text_in + reg - A_TEXT_IN0)));
        break;
    case A_MODE:
        s->mode = MERGE(s->mode) & 7;
        break;
    case A_ENDIAN:
        s->endian = MERGE(s->endian) & 0x3f;
        break;
    case A_TRIGGER:
        if (v & 1) {
            if (s->state != STATE_IDLE) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: AES_TRIGGER_REG while "
                              "the accelerator is not idle is ignored\n",
                              TYPE_ESP32P4_AES);
            } else {
                aes_trigger(s);
            }
        }
        break;
    case A_IV_MEM ... A_IV_MEM + 0xc:
        stl_le_p(s->iv + reg - A_IV_MEM,
                 MERGE(ldl_le_p(s->iv + reg - A_IV_MEM)));
        break;
    case A_J0_MEM ... A_J0_MEM + 0xc:
        stl_le_p(s->j0 + reg - A_J0_MEM,
                 MERGE(ldl_le_p(s->j0 + reg - A_J0_MEM)));
        break;
    case A_DMA_ENABLE:
        s->dma_enable = MERGE(s->dma_enable) & 1;
        break;
    case A_BLOCK_MODE:
        s->block_mode = MERGE(s->block_mode) & 7;
        break;
    case A_BLOCK_NUM:
        s->block_num = MERGE(s->block_num);
        break;
    case A_INC_SEL:
        s->inc_sel = MERGE(s->inc_sel) & 1;
        break;
    case A_AAD_BLOCK_NUM:
        s->aad_block_num = MERGE(s->aad_block_num);
        break;
    case A_REMAINDER_BIT_NUM:
        s->remainder_bit_num = MERGE(s->remainder_bit_num) & 0x7f;
        break;
    case A_CONTINUE:
        if (v & 1) {
            aes_continue(s);
        }
        break;
    case A_INT_CLEAR:
        if (v & 1) {
            s->done_int = false;
            aes_update_irq(s);
        }
        break;
    case A_INT_ENA:
        s->int_ena = MERGE(s->int_ena) & 1;
        aes_update_irq(s);
        break;
    case A_DATE:
        s->date = MERGE(s->date) & 0x0fffffff;
        break;
    case A_DMA_EXIT:
        if (s->state == STATE_WORK) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: AES_DMA_EXIT_REG while the "
                          "accelerator works is ignored\n", TYPE_ESP32P4_AES);
        } else {
            s->state = STATE_IDLE;
            s->phase = PHASE_IDLE;
        }
        break;
    case A_RX_RESET:
        if (v & 1) {
            s->out_len = s->out_pos = 0;
        }
        break;
    case A_TX_RESET:
        if (v & 1) {
            s->in_len = 0;
        }
        break;
    case A_PSEUDO:
        s->pseudo = MERGE(s->pseudo) & PSEUDO_MASK;
        break;
    case A_TEXT_OUT0 ... A_TEXT_OUT0 + 0xc:
    case A_STATE:
    case A_H_MEM ... A_H_MEM + 0xc:
    case A_T0_MEM ... A_T0_MEM + 0xc:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_AES, addr);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to unknown offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_AES, addr);
        break;
    }
#undef MERGE
}

static const MemoryRegionOps aes_ops = {
    .read = aes_read,
    .write = aes_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

void esp32p4_aes_occupy(ESP32P4AesState *s, bool occupied)
{
    if (s->occupied && !occupied) {
        s->occupied = false;
        device_cold_reset(DEVICE(s));
        return;
    }
    s->occupied = occupied;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static void aes_reset_line(void *opaque, int n, int level)
{
    ESP32P4AesState *s = opaque;
    bool was = aes_in_reset(s);

    s->reset_in[n] = level;
    if (was != aes_in_reset(s)) {
        device_cold_reset(DEVICE(s));
    }
}

static void aes_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4AesState *s = opaque;

    esp32p4_crypto_timer_clock_update(&s->op);
    aes_pump(s);
}

static void aes_reset_hold(Object *obj, ResetType type)
{
    ESP32P4AesState *s = ESP32P4_AES(obj);

    esp32p4_crypto_timer_cancel(&s->op);
    memset(s->key, 0, sizeof(s->key));
    memset(s->text_in, 0, sizeof(s->text_in));
    memset(s->text_out, 0, sizeof(s->text_out));
    memset(s->iv, 0, sizeof(s->iv));
    memset(s->h, 0, sizeof(s->h));
    memset(s->j0, 0, sizeof(s->j0));
    memset(s->t0, 0, sizeof(s->t0));
    s->mode = 0;
    s->endian = 0;
    s->state = STATE_IDLE;
    s->dma_enable = 0;
    s->block_mode = 0;
    s->block_num = 0;
    s->inc_sel = 0;
    s->aad_block_num = 0;
    s->remainder_bit_num = 0;
    s->int_ena = 0;
    s->date = DATE_RESET;
    s->pseudo = PSEUDO_RESET;
    s->done_int = false;
    s->phase = PHASE_IDLE;
    s->op_mode = 0;
    s->op_block_mode = 0;
    s->left = 0;
    s->text_total = 0;
    s->in_len = 0;
    s->out_len = s->out_pos = 0;
    s->out_last = false;
    s->computing = false;
    memset(s->ghash, 0, sizeof(s->ghash));
    memset(s->ctr, 0, sizeof(s->ctr));
}

static void aes_reset_exit(Object *obj, ResetType type)
{
    aes_update_irq(ESP32P4_AES(obj));
}

static void aes_init(Object *obj)
{
    ESP32P4AesState *s = ESP32P4_AES(obj);

    memory_region_init_io(&s->iomem, obj, &aes_ops, s, "esp32p4.aes",
                          ESP32P4_AES_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", aes_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), aes_reset_line, ESP32P4_AES_RESET,
                            ESP32P4_AES_RESETS);
}

static void aes_realize(DeviceState *dev, Error **errp)
{
    ESP32P4AesState *s = ESP32P4_AES(dev);

    if (!s->gdma) {
        error_setg(errp, "%s: the 'gdma' link is not set", TYPE_ESP32P4_AES);
        return;
    }
    esp32p4_crypto_timer_init(&s->op, s->clk, aes_op_done, s);
    esp32p4_gdma_attach(s->gdma, ESP32P4_GDMA_AXI_AES, &aes_gdma_ops, s);
}

static const Property aes_properties[] = {
    DEFINE_PROP_LINK("gdma", ESP32P4AesState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
};

static const VMStateDescription vmstate_esp32p4_aes = {
    .name = TYPE_ESP32P4_AES,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_ESP32P4_CRYPTO_TIMER(op, ESP32P4AesState),
        VMSTATE_BOOL_ARRAY(reset_in, ESP32P4AesState, ESP32P4_AES_RESETS),
        VMSTATE_BOOL(occupied, ESP32P4AesState),
        VMSTATE_UINT32_ARRAY(key, ESP32P4AesState, 8),
        VMSTATE_UINT8_ARRAY(text_in, ESP32P4AesState, 16),
        VMSTATE_UINT8_ARRAY(text_out, ESP32P4AesState, 16),
        VMSTATE_UINT32(mode, ESP32P4AesState),
        VMSTATE_UINT32(endian, ESP32P4AesState),
        VMSTATE_UINT32(state, ESP32P4AesState),
        VMSTATE_UINT8_ARRAY(iv, ESP32P4AesState, 16),
        VMSTATE_UINT8_ARRAY(h, ESP32P4AesState, 16),
        VMSTATE_UINT8_ARRAY(j0, ESP32P4AesState, 16),
        VMSTATE_UINT8_ARRAY(t0, ESP32P4AesState, 16),
        VMSTATE_UINT32(dma_enable, ESP32P4AesState),
        VMSTATE_UINT32(block_mode, ESP32P4AesState),
        VMSTATE_UINT32(block_num, ESP32P4AesState),
        VMSTATE_UINT32(inc_sel, ESP32P4AesState),
        VMSTATE_UINT32(aad_block_num, ESP32P4AesState),
        VMSTATE_UINT32(remainder_bit_num, ESP32P4AesState),
        VMSTATE_UINT32(int_ena, ESP32P4AesState),
        VMSTATE_UINT32(date, ESP32P4AesState),
        VMSTATE_UINT32(pseudo, ESP32P4AesState),
        VMSTATE_BOOL(done_int, ESP32P4AesState),
        VMSTATE_UINT32(phase, ESP32P4AesState),
        VMSTATE_UINT32(op_mode, ESP32P4AesState),
        VMSTATE_UINT32(op_block_mode, ESP32P4AesState),
        VMSTATE_UINT32(left, ESP32P4AesState),
        VMSTATE_UINT32(text_total, ESP32P4AesState),
        VMSTATE_UINT8_ARRAY(in_buf, ESP32P4AesState, 16),
        VMSTATE_UINT32(in_len, ESP32P4AesState),
        VMSTATE_UINT8_ARRAY(out_buf, ESP32P4AesState, 16),
        VMSTATE_UINT32(out_len, ESP32P4AesState),
        VMSTATE_UINT32(out_pos, ESP32P4AesState),
        VMSTATE_BOOL(out_last, ESP32P4AesState),
        VMSTATE_BOOL(computing, ESP32P4AesState),
        VMSTATE_UINT8_ARRAY(ghash, ESP32P4AesState, 16),
        VMSTATE_UINT8_ARRAY(ctr, ESP32P4AesState, 16),
        VMSTATE_END_OF_LIST()
    },
};

static void aes_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = aes_realize;
    dc->vmsd = &vmstate_esp32p4_aes;
    device_class_set_props(dc, aes_properties);
    rc->phases.hold = aes_reset_hold;
    rc->phases.exit = aes_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.crypto] */
static const TypeInfo aes_info = {
    .name          = TYPE_ESP32P4_AES,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4AesState),
    .instance_init = aes_init,
    .class_init    = aes_class_init,
};

static void aes_register_types(void)
{
    type_register_static(&aes_info);
}
type_init(aes_register_types)
