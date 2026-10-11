/*
 * ESP32-P4 SHA accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_SHA_H
#define HW_MISC_ESP32P4_SHA_H

#include "hw/core/sysbus.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/misc/esp32p4_crypto.h"
#include "qom/object.h"

#define TYPE_ESP32P4_SHA "esp32p4-sha"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ShaDevState, ESP32P4_SHA)

#define ESP32P4_SHA_BASE    0x50091000
#define ESP32P4_SHA_SIZE    0x1000

/*
 * Named GPIO input "reset", ESP32P4_SHA_RESETS lines: RST_EN_SHA,
 * RST_EN_DS, RST_EN_HMAC and RST_EN_ECDSA, any of which holds the
 * accelerator in reset (TRM 31.3: those blocks reuse it).
 *
 * The clock input "clk" is CRYPTO_SHA_CLK; the registers are behind
 * CRYPTO_SYS_CLK. In DMA-SHA mode it is GDMA-AXI's peripheral SHA (the
 * "gdma" link). Its interrupt, SHA_INTR, is sysbus IRQ 0.
 */
#define ESP32P4_SHA_RESET   "reset"
#define ESP32P4_SHA_RESETS  4

struct ESP32P4ShaDevState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4GdmaState *gdma;
    ESP32P4CryptoTimer op;

    bool reset_in[ESP32P4_SHA_RESETS];
    /* Held by HMAC, RSA_DS or ECDSA_DS, which use it. */
    bool occupied;

    uint32_t mode;
    uint32_t t_string;
    uint32_t t_length;
    uint32_t dma_block_num;
    uint32_t irq_ena;
    uint32_t date;
    /* SHA_H_n_REG and SHA_M_n_REG, as the bytes of the digest and block. */
    uint8_t h[64];
    uint8_t m[128];
    bool busy;
    bool done_int;

    /* The operation in progress. */
    uint32_t op_kind;
    uint32_t op_mode;
    uint32_t left;
    uint8_t in_buf[128];
    uint32_t in_len;
    ESP32P4ShaState state;
};

/*
 * Hardware occupation by a block that uses the accelerator: while
 * occupied software cannot reach its registers, and when released it
 * resets.
 */
void esp32p4_sha_occupy(ESP32P4ShaDevState *s, bool occupied);

#endif
