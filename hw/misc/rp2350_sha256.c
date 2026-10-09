/*
 * RP2350 SHA-256 accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: RP2350 Datasheet, "SHA-256 accelerator", and the pico-sdk
 * sha256.h register descriptions. The block is the FIPS 180-4 SHA-256
 * compression function behind a bus interface: software pads the
 * message itself and writes it to WDATA one 512-bit block at a time.
 * Byte and halfword writes are gathered little-endian into 32-bit words,
 * each optionally byte-swapped (CSR.BSWAP) as it is committed to the
 * core. After the 16th word of a block the core is busy for 57 clk_sys
 * cycles, with WDATA_RDY low, before SUM0-7 show the updated digest and
 * SUM_VLD rises.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/rp2350_atomic.h"
#include "hw/misc/rp2350_sha256.h"
#include "migration/vmstate.h"

#define A_CSR                   0x00
#define A_WDATA                 0x04
#define A_SUM0                  0x08
#define A_SUM7                  0x24

#define CSR_START               (1u << 0)
#define CSR_WDATA_RDY           (1u << 1)
#define CSR_SUM_VLD             (1u << 2)
#define CSR_ERR_WDATA_NOT_RDY   (1u << 4)
#define CSR_DMA_SIZE_SHIFT      8
#define CSR_DMA_SIZE_MASK       (3u << CSR_DMA_SIZE_SHIFT)
#define CSR_BSWAP               (1u << 12)

#define DMA_SIZE_8BIT           0
#define DMA_SIZE_16BIT          1
#define DMA_SIZE_32BIT          2

/* Address bit 14 selects zeroed, rather than replicated, narrow lanes. */
#define NO_REPLICATE            0x4000

/* clk_sys cycles the core takes to digest a block. */
#define DIGEST_CYCLES           57

static const uint32_t sha256_iv[RP2350_SHA256_SUM_WORDS] = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};

static const uint32_t sha256_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

/*
 * The SHA-256 compression function (FIPS 180-4 section 6.2.2): fold one
 * 512-bit block of big-endian message words into the hash state.
 */
/* [spec:nuos:req:emu.sha256] */
static void sha256_compress(uint32_t *h, const uint32_t *m)
{
    uint32_t w[64];
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
    uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];
    int t;

    for (t = 0; t < 16; t++) {
        w[t] = m[t];
    }
    for (t = 16; t < 64; t++) {
        uint32_t s0 = ror32(w[t - 15], 7) ^ ror32(w[t - 15], 18) ^
                      (w[t - 15] >> 3);
        uint32_t s1 = ror32(w[t - 2], 17) ^ ror32(w[t - 2], 19) ^
                      (w[t - 2] >> 10);

        w[t] = w[t - 16] + s0 + w[t - 7] + s1;
    }
    for (t = 0; t < 64; t++) {
        uint32_t s1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t t1 = hh + s1 + ch + sha256_k[t] + w[t];
        uint32_t s0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2 = s0 + maj;

        hh = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
}

/*
 * DMA transfers per 512-bit block for CSR.DMA_SIZE. The reserved
 * encoding 3 requests as for 32-bit transfers.
 */
static uint32_t sha256_transfers_per_block(RP2350SHA256State *s)
{
    switch (s->dma_size) {
    case DMA_SIZE_8BIT:
        return RP2350_SHA256_BLOCK_WORDS * 4;
    case DMA_SIZE_16BIT:
        return RP2350_SHA256_BLOCK_WORDS * 2;
    default:
        return RP2350_SHA256_BLOCK_WORDS;
    }
}

static void sha256_update_dreq(RP2350SHA256State *s)
{
    qemu_set_irq(s->dreq, !s->busy && s->dreq_credits);
}

/* No data has reached the current block yet. */
static bool sha256_block_empty(RP2350SHA256State *s)
{
    return !s->busy && !s->block_words && !s->shift_bytes;
}

static uint32_t sha256_csr(RP2350SHA256State *s)
{
    return (s->bswap ? CSR_BSWAP : 0) |
           (s->dma_size << CSR_DMA_SIZE_SHIFT) |
           (s->err_not_rdy ? CSR_ERR_WDATA_NOT_RDY : 0) |
           (s->sum_vld ? CSR_SUM_VLD : 0) |
           (s->busy ? 0 : CSR_WDATA_RDY);
}

/* [spec:nuos:req:emu.sha256] */
static void sha256_start(RP2350SHA256State *s)
{
    timer_del(s->timer);
    s->busy = false;
    s->sum_vld = true;
    memcpy(s->sum, sha256_iv, sizeof(s->sum));
    memset(s->block, 0, sizeof(s->block));
    s->block_words = 0;
    s->shift = 0;
    s->shift_bytes = 0;
    s->dreq_credits = sha256_transfers_per_block(s);
}

/* [spec:nuos:req:emu.sha256] */
static void sha256_digest_done(void *opaque)
{
    RP2350SHA256State *s = opaque;

    sha256_compress(s->sum, s->block);
    s->busy = false;
    s->sum_vld = true;
    s->dreq_credits = sha256_transfers_per_block(s);
    sha256_update_dreq(s);
}

/*
 * Pass an assembled word to the core. The 16th word of a block starts
 * the digest, which ends DIGEST_CYCLES of clk_sys later.
 */
static void sha256_commit_word(RP2350SHA256State *s, uint32_t word)
{
    s->block[s->block_words++] = s->bswap ? bswap32(word) : word;
    if (s->block_words == RP2350_SHA256_BLOCK_WORDS) {
        s->block_words = 0;
        s->busy = true;
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  clock_ticks_to_ns(s->clk, DIGEST_CYCLES));
    }
}

/*
 * A write of `size` bytes of `data` to WDATA. The bus interface shifts
 * narrow data into a 32-bit register from the top, so the first byte
 * written ends up least significant, and commits the register once it
 * holds 32 bits. A write that takes it past 32 bits shifts the oldest
 * data out, losing it, and a word write replaces whatever it held.
 */
/* [spec:nuos:req:emu.sha256] */
static void sha256_write_data(RP2350SHA256State *s, uint32_t data,
                              unsigned size)
{
    if (s->busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sha256: WDATA written while "
                      "WDATA_RDY is low\n");
        s->err_not_rdy = true;
        return;
    }
    s->sum_vld = false;
    if (s->dreq_credits) {
        s->dreq_credits--;
    }
    if (size == 4) {
        s->shift = data;
    } else {
        s->shift = (s->shift >> (8 * size)) | (data << (32 - 8 * size));
    }
    s->shift_bytes += size;
    if (s->shift_bytes >= 4) {
        s->shift_bytes = 0;
        sha256_commit_word(s, s->shift);
    }
}

/* [spec:nuos:req:emu.sha256] */
static void sha256_write_csr(RP2350SHA256State *s, hwaddr addr,
                             uint32_t value)
{
    uint32_t v = rp2350_atomic_apply(addr, sha256_csr(s), value);
    uint32_t dma_size = extract32(v, CSR_DMA_SIZE_SHIFT, 2);

    s->bswap = v & CSR_BSWAP;
    if (dma_size != s->dma_size) {
        if (dma_size == 3) {
            qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sha256: reserved "
                          "CSR.DMA_SIZE 3\n");
        }
        s->dma_size = dma_size;
        /* A block not yet begun requests the new transfer count. */
        if (sha256_block_empty(s)) {
            s->dreq_credits = sha256_transfers_per_block(s);
        }
    }
    /*
     * ERR_WDATA_NOT_RDY clears, and START strobes, on a 1 in the written
     * data through any alias: pico-sdk clears the error through the
     * clear alias.
     */
    if (value & CSR_ERR_WDATA_NOT_RDY) {
        s->err_not_rdy = false;
    }
    if (value & CSR_START) {
        sha256_start(s);
    }
}

/* [spec:nuos:req:emu.sha256] */
static uint64_t rp2350_sha256_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350SHA256State *s = opaque;
    hwaddr reg = rp2350_atomic_reg(addr) & ~3;
    uint32_t v;

    switch (reg) {
    case A_CSR:
        v = sha256_csr(s);
        break;
    case A_WDATA:
        v = 0;
        break;
    case A_SUM0 ... A_SUM7:
        /*
         * While a block is digested the sums are undefined on hardware;
         * here they hold the digest before the block.
         */
        v = s->sum[(reg - A_SUM0) / 4];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sha256: read of bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.sha256] */
static void rp2350_sha256_write(void *opaque, hwaddr addr, uint64_t value64,
                                unsigned size)
{
    RP2350SHA256State *s = opaque;
    hwaddr window = addr & ~(hwaddr)NO_REPLICATE;
    hwaddr reg = rp2350_atomic_reg(window) & ~3;
    unsigned lane = addr & 3;
    uint32_t value = value64;

    /*
     * The bus carries a narrow write replicated across all its lanes, or
     * in the address bit 14 window, in its own lanes with the rest zero.
     */
    if (size < 4) {
        if (addr & NO_REPLICATE) {
            value <<= lane * 8;
        } else if (size == 1) {
            value = (value & 0xff) * 0x01010101u;
        } else {
            value = (value & 0xffff) * 0x00010001u;
        }
    }

    switch (reg) {
    case A_CSR:
        sha256_write_csr(s, window, value);
        break;
    case A_WDATA:
        /*
         * WDATA takes the addressed lanes, whatever the transfer size.
         * Its aliases are no different: a FIFO write has no stored value
         * to modify.
         */
        sha256_write_data(s, extract32(value, lane * 8, size * 8), size);
        break;
    case A_SUM0 ... A_SUM7:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sha256: write to read-only "
                      "SUM%u\n", (unsigned)(reg - A_SUM0) / 4);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "rp2350-sha256: write to bad offset "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
    sha256_update_dreq(s);
}

static const MemoryRegionOps rp2350_sha256_ops = {
    .read = rp2350_sha256_read,
    .write = rp2350_sha256_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void rp2350_sha256_hold_reset(Object *obj, ResetType type)
{
    RP2350SHA256State *s = RP2350_SHA256(obj);

    timer_del(s->timer);
    s->bswap = true;
    s->dma_size = DMA_SIZE_32BIT;
    s->err_not_rdy = false;
    s->sum_vld = true;
    s->busy = false;
    memset(s->sum, 0, sizeof(s->sum));
    memset(s->block, 0, sizeof(s->block));
    s->block_words = 0;
    s->shift = 0;
    s->shift_bytes = 0;
    s->dreq_credits = sha256_transfers_per_block(s);
    sha256_update_dreq(s);
}

static void rp2350_sha256_init(Object *obj)
{
    RP2350SHA256State *s = RP2350_SHA256(obj);

    memory_region_init_io(&s->iomem, obj, &rp2350_sha256_ops, s,
                          TYPE_RP2350_SHA256, RP2350_SHA256_REGION_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dreq, RP2350_SHA256_DREQ, 1);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", NULL, NULL, 0);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sha256_digest_done, s);
}

static const VMStateDescription vmstate_rp2350_sha256 = {
    .name = TYPE_RP2350_SHA256,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, RP2350SHA256State),
        VMSTATE_CLOCK(clk, RP2350SHA256State),
        VMSTATE_BOOL(bswap, RP2350SHA256State),
        VMSTATE_UINT32(dma_size, RP2350SHA256State),
        VMSTATE_BOOL(err_not_rdy, RP2350SHA256State),
        VMSTATE_BOOL(sum_vld, RP2350SHA256State),
        VMSTATE_BOOL(busy, RP2350SHA256State),
        VMSTATE_UINT32_ARRAY(sum, RP2350SHA256State, RP2350_SHA256_SUM_WORDS),
        VMSTATE_UINT32_ARRAY(block, RP2350SHA256State,
                             RP2350_SHA256_BLOCK_WORDS),
        VMSTATE_UINT32(block_words, RP2350SHA256State),
        VMSTATE_UINT32(shift, RP2350SHA256State),
        VMSTATE_UINT32(shift_bytes, RP2350SHA256State),
        VMSTATE_UINT32(dreq_credits, RP2350SHA256State),
        VMSTATE_END_OF_LIST()
    },
};

static void rp2350_sha256_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = rp2350_sha256_hold_reset;
    dc->vmsd = &vmstate_rp2350_sha256;
}

/* [spec:nuos:req:emu.sha256] */
static const TypeInfo rp2350_sha256_info = {
    .name          = TYPE_RP2350_SHA256,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RP2350SHA256State),
    .instance_init = rp2350_sha256_init,
    .class_init    = rp2350_sha256_class_init,
};

static void rp2350_sha256_register_types(void)
{
    type_register_static(&rp2350_sha256_info);
}

type_init(rp2350_sha256_register_types)
