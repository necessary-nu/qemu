/*
 * ESP32-P4 AES accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_AES_H
#define HW_MISC_ESP32P4_AES_H

#include "crypto/aes.h"
#include "hw/core/sysbus.h"
#include "hw/dma/esp32p4_gdma.h"
#include "hw/misc/esp32p4_crypto.h"
#include "qom/object.h"

#define TYPE_ESP32P4_AES "esp32p4-aes"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4AesState, ESP32P4_AES)

#define ESP32P4_AES_BASE    0x50090000
#define ESP32P4_AES_SIZE    0x1000

/*
 * Named GPIO input "reset", ESP32P4_AES_RESETS lines: HP_SYS_CLKRST's
 * RST_EN_AES, RST_EN_DS and RST_EN_KM, any of which holds the accelerator
 * in reset (TRM 27.3: the RSA_DS peripheral and the Key Manager reuse it).
 *
 * The clock input "clk" is CRYPTO_AES_CLK; its registers are behind
 * CRYPTO_SYS_CLK. In DMA-AES mode it is the GDMA-AXI peripheral AES (the
 * "gdma" link). Its interrupt, AES_INTR, is sysbus IRQ 0.
 */
#define ESP32P4_AES_RESET   "reset"
#define ESP32P4_AES_RESETS  3

struct ESP32P4AesState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4GdmaState *gdma;
    ESP32P4CryptoTimer op;

    bool reset_in[ESP32P4_AES_RESETS];
    /* Held by the RSA_DS peripheral, which uses it. */
    bool occupied;

    uint32_t key[8];
    uint8_t text_in[16];
    uint8_t text_out[16];
    uint32_t mode;
    uint32_t endian;
    uint32_t state;
    uint8_t iv[16];
    uint8_t h[16];
    uint8_t j0[16];
    uint8_t t0[16];
    uint32_t dma_enable;
    uint32_t block_mode;
    uint32_t block_num;
    uint32_t inc_sel;
    uint32_t aad_block_num;
    uint32_t remainder_bit_num;
    uint32_t int_ena;
    uint32_t date;
    uint32_t pseudo;
    bool done_int;

    /* The operation in progress: see esp32p4_aes.c. */
    uint32_t phase;
    uint32_t op_mode;
    uint32_t op_block_mode;
    uint32_t left;
    uint32_t text_total;
    uint8_t in_buf[16];
    uint32_t in_len;
    uint8_t out_buf[16];
    uint32_t out_len;
    uint32_t out_pos;
    bool out_last;
    bool computing;
    uint8_t ghash[16];
    uint8_t ctr[16];
};

/*
 * Hardware occupation (TRM 33.4.3.3) by a block that uses the
 * accelerator: while occupied software cannot reach its registers, and
 * when released it resets.
 */
void esp32p4_aes_occupy(ESP32P4AesState *s, bool occupied);

#endif
