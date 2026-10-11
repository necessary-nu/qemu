/*
 * ESP32-P4 ECC accelerator
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_ECC_H
#define HW_MISC_ESP32P4_ECC_H

#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_crypto.h"
#include "hw/nvram/esp32p4_efuse.h"
#include "qom/object.h"

#define TYPE_ESP32P4_ECC "esp32p4-ecc"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4EccState, ESP32P4_ECC)

#define ESP32P4_ECC_BASE    0x50093000
#define ESP32P4_ECC_SIZE    0x1000

/*
 * Named GPIO inputs:
 *
 * "reset", ESP32P4_ECC_RESETS lines: RST_EN_ECC and RST_EN_ECDSA, either of
 *   which holds the accelerator in reset (TRM 28.5).
 * "mem-pd": high while HP_SYSTEM_ECC_PD_CTRL_REG powers its memories down.
 *
 * The clock input "clk" is CRYPTO_ECC_CLK; the registers are behind
 * CRYPTO_SYS_CLK. The "efuse" link gives EFUSE_ECC_FORCE_CONST_TIME. Its
 * interrupt, ECC_INTR, is sysbus IRQ 0.
 */
#define ESP32P4_ECC_RESET   "reset"
#define ESP32P4_ECC_RESETS  2
#define ESP32P4_ECC_MEM_PD  "mem-pd"

/* The memories, 48 bytes each: k, Px, Py, Qx, Qy, Qz. */
enum {
    ESP32P4_ECC_MEM_K,
    ESP32P4_ECC_MEM_PX,
    ESP32P4_ECC_MEM_PY,
    ESP32P4_ECC_MEM_QX,
    ESP32P4_ECC_MEM_QY,
    ESP32P4_ECC_MEM_QZ,
    ESP32P4_ECC_MEMS,
};
#define ESP32P4_ECC_MEM_BYTES 48

struct ESP32P4EccState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    ESP32P4EfuseState *efuse;
    ESP32P4CryptoTimer op;

    bool reset_in[ESP32P4_ECC_RESETS];
    bool mem_pd;
    /* Held by the ECDSA_DS peripheral, which uses it. */
    bool occupied;

    uint8_t mem[ESP32P4_ECC_MEMS][ESP32P4_ECC_MEM_BYTES];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t conf;
    uint32_t date;
};

/* As esp32p4_aes_occupy(): while the ECDSA_DS peripheral computes. */
void esp32p4_ecc_occupy(ESP32P4EccState *s, bool occupied);

#endif
