/*
 * ESP32 DMA linked-list descriptors (lldesc_t), shared by the DMA engines
 * of SPI, UHCI and I2S
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_ESP32_LLDESC_H
#define HW_DMA_ESP32_LLDESC_H

#include "system/memory.h"

/*
 * The DMA engines reach internal SRAM 1 and 2 only, the 328 KB DMA address
 * space at 0x3ffae000..0x3fffffff (TRM 3.3.2.6). Their link registers hold
 * the low 20 bits of a descriptor address in 0x3ffxxxxx.
 */
#define ESP32_DMA_RAM_START     0x3ffae000u
#define ESP32_DMA_RAM_END       0x40000000u
#define ESP32_DMA_LINK_BASE     0x3ff00000u
#define ESP32_DMA_LINK_ADDR_MASK 0xfffffu

/*
 * Descriptors are three words (TRM 2.3.2):
 *   DW0: size[11:0], length[23:12], reserved[29:24], eof[30], owner[31]
 *   DW1: buffer address
 *   DW2: next descriptor address, 0 at the end of the list
 * owner is 1 while the DMA engine owns the descriptor.
 */
#define ESP32_LLDESC_SIZE(dw0)      ((dw0) & 0xfffu)
#define ESP32_LLDESC_LENGTH(dw0)    (((dw0) >> 12) & 0xfffu)
#define ESP32_LLDESC_LENGTH_SHIFT   12
#define ESP32_LLDESC_RESERVED_MASK  0x3f000000u
#define ESP32_LLDESC_EOF            (1u << 30)
#define ESP32_LLDESC_OWNER          (1u << 31)

typedef struct Esp32Lldesc {
    uint32_t addr;
    uint32_t dw0;
    uint32_t buf;
    uint32_t next;
} Esp32Lldesc;

typedef enum Esp32LldescResult {
    ESP32_LLDESC_OK,
    /* The descriptor is not word-aligned or not in DMA-capable SRAM */
    ESP32_LLDESC_BAD_ADDR,
    /* Owner checking is on and the CPU owns the descriptor */
    ESP32_LLDESC_NOT_OWNED,
    /* The bytes the descriptor covers run outside DMA-capable SRAM */
    ESP32_LLDESC_BAD_BUF,
} Esp32LldescResult;

static inline bool esp32_dma_ram_contains(uint32_t addr, uint32_t len)
{
    return addr >= ESP32_DMA_RAM_START && addr < ESP32_DMA_RAM_END &&
           len <= ESP32_DMA_RAM_END - addr;
}

/*
 * Read the descriptor at addr from as into *d. The buffer checked is the
 * part the engine will touch: length bytes for an outlink, which reads
 * them, and size bytes for an inlink, which fills them. On an error *d
 * holds what could be read, zeros otherwise.
 */
static inline Esp32LldescResult esp32_lldesc_fetch(AddressSpace *as,
                                                   uint32_t addr, bool out,
                                                   bool check_owner,
                                                   Esp32Lldesc *d)
{
    MemTxResult r0, r1, r2;
    uint32_t n;

    d->addr = addr;
    d->dw0 = d->buf = d->next = 0;
    if ((addr & 3) || !esp32_dma_ram_contains(addr, 12)) {
        return ESP32_LLDESC_BAD_ADDR;
    }
    d->dw0 = address_space_ldl_le(as, addr, MEMTXATTRS_UNSPECIFIED, &r0);
    d->buf = address_space_ldl_le(as, addr + 4, MEMTXATTRS_UNSPECIFIED, &r1);
    d->next = address_space_ldl_le(as, addr + 8, MEMTXATTRS_UNSPECIFIED,
                                   &r2);
    if (r0 != MEMTX_OK || r1 != MEMTX_OK || r2 != MEMTX_OK) {
        return ESP32_LLDESC_BAD_ADDR;
    }
    if (check_owner && !(d->dw0 & ESP32_LLDESC_OWNER)) {
        return ESP32_LLDESC_NOT_OWNED;
    }
    n = out ? ESP32_LLDESC_LENGTH(d->dw0) : ESP32_LLDESC_SIZE(d->dw0);
    if (n && !esp32_dma_ram_contains(d->buf, n)) {
        return ESP32_LLDESC_BAD_BUF;
    }
    return ESP32_LLDESC_OK;
}

#endif
