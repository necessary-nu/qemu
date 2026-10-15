/*
 * ESP32-P4 cache: the HP cores' L1 caches, the L2 cache and the L2MEM split
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_CACHE_H
#define HW_MISC_ESP32P4_CACHE_H

#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_hp_sys.h"
#include "hw/ssi/esp32p4_mspi.h"
#include "system/memory.h"
#include "qom/object.h"

#define TYPE_ESP32P4_CACHE "esp32p4-cache"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4CacheState, ESP32P4_CACHE)

#define TYPE_ESP32P4_CACHE_VIEW "esp32p4-cache-view"

#define ESP32P4_CACHE_REGS_SIZE     0x1000
#define ESP32P4_CACHE_NREGS         (ESP32P4_CACHE_REGS_SIZE / 4)
#define ESP32P4_CACHE_LINE          64

/*
 * The address ranges the HP cores reach through the caches, which the
 * cache keeps its contents for: the flash and PSRAM windows (through the
 * L1 and L2 caches) and HP L2MEM (through the L1 caches).
 */
enum {
    ESP32P4_CACHE_VIEW_FLASH,
    ESP32P4_CACHE_VIEW_PSRAM,
    ESP32P4_CACHE_VIEW_L2MEM,
    ESP32P4_CACHE_NVIEWS,
};

/*
 * The device's MMIO regions, in this order:
 *   - the registers, 4 KiB (0x3FF10000);
 *   - the part of L2MEM the L2 cache occupies, at L2MEM's two addresses in
 *     the bus's view (0x4FF00000 and 0x8FF00000, mapped over L2MEM with a
 *     higher priority), its size following the cache's;
 *   - the HP cores' cached views of the flash window, the PSRAM window and
 *     L2MEM, for the HP cores' address space at 0x40000000, 0x48000000
 *     and 0x4FF00000.
 */
enum {
    ESP32P4_CACHE_MMIO_REGS,
    ESP32P4_CACHE_MMIO_L2MEM_CACHED,
    ESP32P4_CACHE_MMIO_L2MEM_DIRECT,
    ESP32P4_CACHE_MMIO_VIEW0,
};

/*
 * Named GPIO inputs:
 *
 * "reset": HP_SYS_CLKRST_RST_EN_HP_CACHE, high while held: the controller
 *   and every cache are reset.
 * "cache-reset": 4 lines, RST_EN_L1_I0_CACHE, _L1_I1_CACHE, _L1_D_CACHE and
 *   _L2_CACHE, each emptying its cache while held.
 * "sys-cache-reset": 4 lines in the same order, HP_SYSTEM_CACHE_RESET_
 *   CONFIG's, which has no L2 reset: each empties its cache as the
 *   "cache-reset" line does.
 * "l2mem-err-resp": HP_SYSTEM_L2_MEM_ERR_RESP_EN.
 *
 * Its interrupt, CACHE_INTR (source 83), is sysbus IRQ 0.
 */
#define ESP32P4_CACHE_RESET         "reset"
#define ESP32P4_CACHE_CACHE_RESET   "cache-reset"
#define ESP32P4_CACHE_SYS_CACHE_RESET "sys-cache-reset"
#define ESP32P4_CACHE_L2MEM_ERR_RESP "l2mem-err-resp"

enum {
    ESP32P4_CACHE_RESET_L1_I0,
    ESP32P4_CACHE_RESET_L1_I1,
    ESP32P4_CACHE_RESET_L1_D,
    ESP32P4_CACHE_RESET_L2,
    ESP32P4_CACHE_NRESETS,
};

/*
 * The L1 caches, each with its own lines: HP core 0's and HP core 1's
 * instruction caches and the data cache both cores share (TRM figure
 * 9.3-3).
 */
enum {
    ESP32P4_CACHE_L1_I0,
    ESP32P4_CACHE_L1_I1,
    ESP32P4_CACHE_L1_D,
    ESP32P4_CACHE_NL1,
};

/* An L1 cache's state for the lines of a view, per 64-byte line. */
typedef struct ESP32P4CacheL1 {
    unsigned long *valid;
    unsigned long *lock;
    int32_t nlines;
} ESP32P4CacheL1;

typedef struct ESP32P4CacheView {
    IOMMUMemoryRegion iommu;
    struct ESP32P4CacheState *cache;
    int kind;
    uint64_t size;

    /*
     * What the translations reach, `size` apart: the data cache's
     * contents, RAM whose stores the model watches (a store to a page
     * with a clean line marks its line dirty); each instruction cache's
     * contents; and the region through which the accesses no cache's
     * contents serve go.
     */
    MemoryRegion target;
    AddressSpace target_as;
    MemoryRegion l1[ESP32P4_CACHE_NL1];
    MemoryRegion miss;
    MemoryRegion l2;
    /* The host memory of each cache's contents. */
    uint8_t *l1_host[ESP32P4_CACHE_NL1];
    uint8_t *l2_host;

    ESP32P4CacheL1 l1c[ESP32P4_CACHE_NL1];
    /* Per 64-byte line. */
    unsigned long *l1d_dirty;
    unsigned long *l2_valid;
    unsigned long *l2_dirty;
    unsigned long *l2_lock;
    int32_t nlines;
    /*
     * Per 4 KiB page: whether a core may have been given a translation
     * that stores to the data cache's contents directly, past the watch.
     */
    unsigned long *l1d_direct;
    int32_t npages;
} ESP32P4CacheView;

struct ESP32P4CacheState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion l2mem_cached;
    MemoryRegion l2mem_direct;
    ESP32P4CacheView view[ESP32P4_CACHE_NVIEWS];

    ESP32P4MSPIState *flash_mspi;
    ESP32P4MSPIState *psram_mspi;
    MemoryRegion *l2mem_mr;
    uint8_t *l2mem_host;
    ESP32P4HPSysState *hp_sys;

    qemu_irq irq;
    uint32_t regs[ESP32P4_CACHE_NREGS];
    uint32_t l2_cache_size;
    bool cache_reset[ESP32P4_CACHE_NRESETS];
    bool sys_cache_reset[ESP32P4_CACHE_NRESETS];
    bool l2mem_err_resp;
    bool counters_logged;
};

#endif
