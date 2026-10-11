/*
 * ESP32-P4 RSA accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_RSA_H
#define HW_MISC_ESP32P4_RSA_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_crypto.h"
#include "qom/object.h"

#define TYPE_ESP32P4_RSA "esp32p4-rsa"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4RsaState, ESP32P4_RSA)

#define ESP32P4_RSA_BASE    0x50092000
#define ESP32P4_RSA_SIZE    0x1000
#define ESP32P4_RSA_WORDS   128

/*
 * Named GPIO inputs:
 *
 * "reset", ESP32P4_RSA_RESETS lines: RST_EN_RSA, RST_EN_DS and
 *   RST_EN_ECDSA, any of which holds the accelerator in reset (TRM 30.3).
 * "mem-pd": high while HP_SYSTEM_RSA_PD_CTRL_REG powers its memories
 *   down.
 *
 * The clock input "clk" is CRYPTO_RSA_CLK; the registers are behind
 * CRYPTO_SYS_CLK. Its interrupt, RSA_INTR, is sysbus IRQ 0.
 */
#define ESP32P4_RSA_RESET   "reset"
#define ESP32P4_RSA_RESETS  3
#define ESP32P4_RSA_MEM_PD  "mem-pd"

enum {
    ESP32P4_RSA_MEM_M,
    ESP32P4_RSA_MEM_Z,
    ESP32P4_RSA_MEM_Y,
    ESP32P4_RSA_MEM_X,
    ESP32P4_RSA_MEMS,
};

struct ESP32P4RsaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4CryptoTimer op;

    bool reset_in[ESP32P4_RSA_RESETS];
    bool mem_pd;
    /* Held by the RSA_DS peripheral, which uses it. */
    bool occupied;

    uint32_t mem[ESP32P4_RSA_MEMS][ESP32P4_RSA_WORDS];
    uint32_t m_prime;
    uint32_t mode;
    uint32_t constant_time;
    uint32_t search_enable;
    uint32_t search_pos;
    uint32_t int_ena;
    uint32_t date;
    /* QUERY_CLEAN and QUERY_IDLE; the interrupt's state. */
    bool clean;
    bool idle;
    bool done_int;
    /* The operation in progress (RSA_OP_*). */
    uint32_t op_kind;
};

/* As esp32p4_aes_occupy(): while the RSA_DS peripheral computes on it. */
void esp32p4_rsa_occupy(ESP32P4RsaState *s, bool occupied);

/*
 * Large-number modular exponentiation as the accelerator computes it
 * (TRM 30.3.2): Z = X^Y mod M for @n-word numbers, with r = R^2 mod M and
 * M' as given; bits of Y above @top are not used (the SEARCH option).
 * Returns the cycles it takes the accelerator, per @constant_time.
 */
uint64_t esp32p4_rsa_modexp(uint32_t *z, const uint32_t *x,
                            const uint32_t *y, const uint32_t *m,
                            const uint32_t *r, uint32_t mprime, unsigned n,
                            unsigned top, bool constant_time,
                            uint32_t *x_mont);

#endif
