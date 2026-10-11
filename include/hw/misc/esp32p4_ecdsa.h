/*
 * ESP32-P4 ECDSA digital signature peripheral (ECDSA_DS)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ECDSA_H
#define HW_MISC_ESP32P4_ECDSA_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_crypto.h"
#include "hw/misc/esp32p4_ecc.h"
#include "hw/misc/esp32p4_keymng.h"
#include "hw/misc/esp32p4_sha.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ECDSA "esp32p4-ecdsa"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4EcdsaState, ESP32P4_ECDSA)

#define ESP32P4_ECDSA_BASE  0x50096000
#define ESP32P4_ECDSA_SIZE  0x1000

/*
 * The clock input "clk" is CRYPTO_ECDSA_CLK; the registers are behind
 * CRYPTO_SYS_CLK. The named GPIO input "reset" is RST_EN_ECDSA. The links
 * "efuse" and "keymng" are the key sources and "ecc" and "sha" the
 * accelerators it occupies. Its interrupt, ECDSA_INTR, is sysbus IRQ 0.
 */
#define ESP32P4_ECDSA_RESET "reset"

/* The memories: r, s, z, QAx (or software's k) and QAy. */
enum {
    ESP32P4_ECDSA_MEM_R,
    ESP32P4_ECDSA_MEM_S,
    ESP32P4_ECDSA_MEM_Z,
    ESP32P4_ECDSA_MEM_QAX,
    ESP32P4_ECDSA_MEM_QAY,
    ESP32P4_ECDSA_MEMS,
};
#define ESP32P4_ECDSA_MEM_BYTES 48

struct ESP32P4EcdsaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4EfuseState *efuse;
    ESP32P4KeyMngState *keymng;
    ESP32P4EccState *ecc;
    ESP32P4ShaDevState *sha;
    ESP32P4CryptoTimer op;
    ESP32P4CryptoTimer sha_op;
    bool in_reset;

    uint32_t conf;
    uint32_t clk_reg;
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t state;
    uint32_t result;
    uint32_t date;
    uint32_t sha_mode;
    bool sha_busy;
    uint8_t message[64];
    uint8_t mem[ESP32P4_ECDSA_MEMS][ESP32P4_ECDSA_MEM_BYTES];

    /* The stage in progress (see esp32p4_ecdsa.c) and the SHA state. */
    uint32_t stage;
    uint32_t op_sha_mode;
    ESP32P4ShaState sha_state;
    bool sha_started;
    bool sha_held;
    bool ecc_held;
};

#endif
