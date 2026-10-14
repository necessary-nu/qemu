/*
 * ESP32-P4 cache: the HP cores' L1 caches, the L2 cache and the L2MEM split
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 9.3.2
 * (Internal Memory: the HP L2MEM configurations), 9.3.3.2 (Cache) and
 * 9.3.3.3 (Cache Operations); ESP-IDF's cache_reg.h for v3, which gives
 * the controller's registers, access and reset values (the TRM does not
 * document them); its cache_ll.h, the esp_mm component, and the v3 ROM's
 * Cache_* routines, which show how software drives them; and ESP-IDF's
 * linker scripts for v3, which place the L2 cache at L2MEM's start.
 *
 * The HP cores reach the external memory through two levels of cache: an
 * L1 instruction cache per core and a shared L1 data cache, then the L2
 * cache; and HP L2MEM through the L1 caches. The other masters (DMA, the
 * LP core) reach both directly. The caches are virtually tagged: the MMU
 * (in the MSPI controllers, esp32p4_mspi.c) translates only what the L2
 * cache fetches and writes back.
 *
 * How the model keeps the caches' contents: software observes a write-back
 * cache only where its contents and the memory's differ, so the model
 * keeps the contents of each line at each level, with its state, and
 * never evicts a line: a cache as large as the windows it caches. A line
 * leaves a cache only when an operation or a reset invalidates it, or is
 * written back only when an operation writes it back. Software that
 * relies on a line having been evicted, as real caches of 64 KiB (L1) and
 * up to 512 KiB (L2) eventually do, sees what it would with a cache that
 * kept it; software that relies on a line not being in the cache sees it
 * stale. Each is a defect on the hardware too. For each 64-byte line:
 *
 *   - the L1 contents, which the cores read and write, executing from
 *     them, at the speed of RAM; the L1 contents as it last filled,
 *     cleaned or wrote the line back, its being dirty being its differing
 *     from them (so a write of the value already there leaves it clean);
 *     and whether it is valid and locked;
 *   - for the external memory, the L2 contents, and whether the line is
 *     valid, dirty and locked there. With the L2's 128-byte lines, the
 *     two 64-byte lines of each share that state: the L2 holds, dirties,
 *     cleans, writes back and drops the 128 bytes as one.
 *
 * A core's access to a line the L1 does not hold fills the L1, from the L2
 * where it holds the line and otherwise from the memory through the L2,
 * 4 KiB at a time: the 64 lines that share the line's page, which is what
 * lets every access to a filled page run at full speed. The fill takes the
 * page's other lines as an L2 autoload of the neighbouring lines would.
 * Each core's accesses go through a translation per page (an IOMMU region
 * in its address space) to the L1 contents or, while the page is not all
 * held, to the controller. An access through those translations that no
 * core makes, qtest's, goes to the bus past the caches, as every other
 * master's does; a debugger's reads the L1 where it holds the line.
 *
 * What the model does not keep apart: the instruction caches and the data
 * cache share the L1 contents, so a core executes what was written
 * through the data cache without a write-back; invalidating only the
 * instruction caches drops the L1's clean lines (the data cache keeps the
 * dirty ones). The HP ROM is read through the L1 cache on the chip and
 * read directly here, as no line of it can differ.
 *
 * Operations (CACHE_SYNC_*, CACHE_LOCK_*, *_PRELOAD_*): each completes at
 * once, as the TRM gives no timing, setting its done bit and raising its
 * done interrupt. SYNC_MAP picks the caches: L1-ICache0 and 1 (bits 0 and
 * 1; 2 and 3 are reserved), the L1-DCache (bit 4), the L2 (bit 5). A
 * SYNC_SIZE of zero operates on the whole cache, as the ROM's
 * Cache_*_All() use it; otherwise on every line SYNC_ADDR to SYNC_ADDR +
 * SYNC_SIZE touches, in each cache's own line size: the L1's 64 bytes,
 * the L2's 64 or 128 (L2_CACHE_BLOCKSIZE_CONF). An operation acts on whole
 * lines, as the chip's do: a line the range touches is dropped, cleaned or
 * written back entirely, so that invalidating a range not aligned to
 * lines drops the dirty data of the touched lines' bytes outside it too,
 * which is why ESP-IDF's esp_cache_msync() refuses such an invalidation.
 * The TRM gives no rule for a range not aligned to lines. The ROM's
 * Cache_Op_Addr(), through which ESP-IDF's cache_hal operations go
 * (Cache_Invalidate_Addr() and the rest), rounds the range out to whole
 * lines of the largest line size the map names before it writes SYNC_ADDR
 * and SYNC_SIZE, so that the controller gets whole lines from ESP-IDF; a
 * range written to the registers unaligned is taken to name the lines it
 * touches likewise. Invalidate drops the lines (and their dirty data);
 * clean leaves the lines and makes them clean; write-back writes the
 * dirty lines to the next level, the L1's to the L2 (or to the memory for
 * L2MEM, or with the L2 bypassed) and the L2's to the memory, L1 before
 * L2; write-back-invalidate does both. An L1 write-back into a line the
 * L2 does not hold allocates the line there, reading the rest of a
 * 128-byte line from the memory as a miss would. Operating on
 * an address range leaves locked lines; on the whole cache, it does not.
 * A SYNC_MAP of zero, or more than one operation at once, completes with
 * SYNC_ERR and SYNC_ERR_CODE 0 or 1. Lock and unlock (LOCK_CTRL, with
 * LOCK_MAP, LOCK_ADDR and LOCK_SIZE) mark the lines held in the range;
 * prelock sections (*_PRELOCK_*) lock the lines filled inside them.
 * Preload fills the range's lines into the cache it names. Freeze sets
 * its done bit at once, there being no access in flight; autoload's
 * settings are held, its done bit always set, as fills already take whole
 * pages.
 *
 * Access errors: where the next level answers a fill with an error (the
 * MSPI's port closed, or an invalid MMU entry while it answers those with
 * errors), the L2 records the access in L2_CACHE_ACS_FAIL_ADDR and raises
 * its FAIL interrupt, and with L2_CACHE_ACS_FAIL_CHECK_MODE set passes the
 * error to the L1, which records it in the L1-DCache's registers and
 * raises its FAIL interrupt, and with its check mode set passes it to the
 * core, which takes an access fault. An error not passed on reads as
 * zero, fills nothing and drops the write. The FAIL_ID and FAIL_ATTR
 * fields stay zero: their encoding is not published.
 *
 * Bypass and shut-down: L1_BYPASS_CACHE_CONF's ICache<n> or DCache bit
 * sends core n's accesses past the L1, to the L2 or L2MEM; with the L2
 * bypassed (L2_BYPASS_CACHE_CONF), the L1 fills from and writes back to
 * the memory directly. L1_ICACHE_CTRL_SHUT_IBUS<n> and L1_DCACHE_CTRL_
 * SHUT_DBUS<n> close core n's buses: an access fails as above in the
 * L1-DCache (or ICache<n> for the instruction bus). The model cannot tell
 * a core's instruction fetches from its data accesses apart there, so
 * either bit applies to both.
 *
 * The L2MEM split (TRM 9.3.2): the L2 cache occupies the start of L2MEM,
 * L2_CACHE_CACHESIZE_CONF's 128 KiB (at reset), 256 KiB or 512 KiB, or none
 * with the L2 bypassed (the 768 KiB RAM configuration) or no size bit set;
 * the RAM is what follows it. An access to the part the cache occupies, by
 * a core or any other master, is not made: it reads as zero, raises
 * HP_SYSTEM's EXCEED_ADDR interrupt, and is answered with an error while
 * HP_SYSTEM_L2_MEM_ERR_RESP_EN ("l2mem-err-resp") is set. The cache's
 * contents are kept apart from L2MEM's, which keeps what it held.
 *
 * The registers are behind L1CACHE_CPU_CLK and RST_EN_HP_CACHE; the four
 * cache resets ("cache-reset"), and HP_SYSTEM_CACHE_RESET_CONFIG's for the
 * L1 caches ("sys-cache-reset"), empty their caches.
 *
 * Not modelled: the access counters (*_ACS_*_CNT), which count nothing
 * (logged as unimplemented when enabled); access to the tag and data
 * memories through the debug bus (*_OBJECT_CTRL); their power-down
 * (*_MEM_FORCE_PD); DMA through the caches: the DMA masters reach the
 * memory directly whatever L1_DCACHE_CTRL_SHUT_DMA (clear from reset) and
 * L2_CACHE_CTRL_SHUT_DMA say, as ESP-IDF's synchronisation of its DMA
 * buffers with the caches has them (the L2's is logged as unimplemented
 * when cleared); the PMA's write-through and no-allocate attributes,
 * which the cores do not pass on (its non-cacheable one decides whether a
 * core's access goes through the caches at all); and a frozen cache's
 * stalling of misses.
 */

#include "qemu/osdep.h"
#include "qemu/bitmap.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "exec/cputlb.h"
#include "exec/target_page.h"
#include "exec/translation-block.h"
#include "system/address-spaces.h"
#include "system/tcg.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/esp32p4_cache.h"
#include "migration/vmstate.h"

/*
 * Each register's name, reset value and fields by access, from the v3
 * header: stored as written (rw), acting when written with 1 and reading
 * 0 (wt), and raw interrupt bits a write of 1 clears (w1c). The rest are
 * read-only: the hardware's status, and reserved fields, which read as
 * their reset values.
 */
typedef struct {
    const char *name;
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
    uint32_t w1c;
} ESP32P4CacheReg;

#define REG(off, n, rst, w, t, c) \
    [(off) / 4] = { .name = (n), .reset = (rst), .rw = (w), .wt = (t), \
                    .w1c = (c) }

static const ESP32P4CacheReg esp32p4_cache_regs[ESP32P4_CACHE_NREGS] = {
    REG(0x000, "L1_ICACHE_CTRL",
        0x00000000, 0x00000003, 0x00000000, 0x00000000),
    REG(0x004, "L1_DCACHE_CTRL",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x008, "L1_BYPASS_CACHE_CONF",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x00c, "L1_CACHE_ATOMIC_CONF",
        0x00000001, 0x00000001, 0x00000000, 0x00000000),
    REG(0x010, "L1_ICACHE_CACHESIZE_CONF",
        0x00000040, 0x00000000, 0x00000000, 0x00000000),
    REG(0x014, "L1_ICACHE_BLOCKSIZE_CONF",
        0x00000008, 0x00000000, 0x00000000, 0x00000000),
    REG(0x018, "L1_DCACHE_CACHESIZE_CONF",
        0x00000100, 0x00000000, 0x00000000, 0x00000000),
    REG(0x01c, "L1_DCACHE_BLOCKSIZE_CONF",
        0x00000008, 0x00000000, 0x00000000, 0x00000000),
    REG(0x020, "L1_CACHE_WRAP_AROUND_CTRL",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x024, "L1_CACHE_TAG_MEM_POWER_CTRL",
        0x00055555, 0x00070077, 0x00000000, 0x00000000),
    REG(0x028, "L1_CACHE_DATA_MEM_POWER_CTRL",
        0x00055555, 0x00070077, 0x00000000, 0x00000000),
    REG(0x02c, "L1_CACHE_FREEZE_CTRL",
        0x00000000, 0x00030033, 0x00000000, 0x00000000),
    REG(0x030, "L1_CACHE_DATA_MEM_ACS_CONF",
        0x00033333, 0x00030033, 0x00000000, 0x00000000),
    REG(0x034, "L1_CACHE_TAG_MEM_ACS_CONF",
        0x00033333, 0x00030033, 0x00000000, 0x00000000),
    REG(0x038, "L1_ICACHE0_PRELOCK_CONF",
        0x00000000, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x03c, "L1_ICACHE0_PRELOCK_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x040, "L1_ICACHE0_PRELOCK_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x044, "L1_ICACHE0_PRELOCK_SCT_SIZE",
        0x3fff3fff, 0x3fff3fff, 0x00000000, 0x00000000),
    REG(0x048, "L1_ICACHE1_PRELOCK_CONF",
        0x00000000, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x04c, "L1_ICACHE1_PRELOCK_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x050, "L1_ICACHE1_PRELOCK_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x054, "L1_ICACHE1_PRELOCK_SCT_SIZE",
        0x3fff3fff, 0x3fff3fff, 0x00000000, 0x00000000),
    REG(0x058, "L1_ICACHE2_PRELOCK_CONF",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x05c, "L1_ICACHE2_PRELOCK_SCT0_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x060, "L1_ICACHE2_PRELOCK_SCT1_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x064, "L1_ICACHE2_PRELOCK_SCT_SIZE",
        0x3fff3fff, 0x00000000, 0x00000000, 0x00000000),
    REG(0x068, "L1_ICACHE3_PRELOCK_CONF",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x06c, "L1_ICACHE3_PRELOCK_SCT0_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x070, "L1_ICACHE3_PRELOCK_SCT1_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x074, "L1_ICACHE3_PRELOCK_SCT_SIZE",
        0x3fff3fff, 0x00000000, 0x00000000, 0x00000000),
    REG(0x078, "L1_DCACHE_PRELOCK_CONF",
        0x00000000, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x07c, "L1_DCACHE_PRELOCK_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x080, "L1_DCACHE_PRELOCK_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x084, "L1_DCACHE_PRELOCK_SCT_SIZE",
        0x3fff3fff, 0x3fff3fff, 0x00000000, 0x00000000),
    REG(0x088, "LOCK_CTRL", 0x00000004, 0x0000007b, 0x00000000, 0x00000000),
    REG(0x08c, "LOCK_MAP", 0x00000000, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x090, "LOCK_ADDR", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x094, "LOCK_SIZE", 0x00000000, 0x0000ffff, 0x00000000, 0x00000000),
    REG(0x098, "SYNC_CTRL", 0x00000001, 0x000001ef, 0x00000000, 0x00000000),
    REG(0x09c, "SYNC_MAP", 0x0000001f, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x0a0, "SYNC_ADDR", 0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0a4, "SYNC_SIZE", 0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x0a8, "L1_ICACHE0_PRELOAD_CTRL",
        0x00000002, 0x000000fd, 0x00000000, 0x00000000),
    REG(0x0ac, "L1_ICACHE0_PRELOAD_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0b0, "L1_ICACHE0_PRELOAD_SIZE",
        0x00000000, 0x00003fff, 0x00000000, 0x00000000),
    REG(0x0b4, "L1_ICACHE1_PRELOAD_CTRL",
        0x00000002, 0x000000fd, 0x00000000, 0x00000000),
    REG(0x0b8, "L1_ICACHE1_PRELOAD_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0bc, "L1_ICACHE1_PRELOAD_SIZE",
        0x00000000, 0x00003fff, 0x00000000, 0x00000000),
    REG(0x0c0, "L1_ICACHE2_PRELOAD_CTRL",
        0x00000002, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0c4, "L1_ICACHE2_PRELOAD_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0c8, "L1_ICACHE2_PRELOAD_SIZE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0cc, "L1_ICACHE3_PRELOAD_CTRL",
        0x00000002, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d0, "L1_ICACHE3_PRELOAD_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d4, "L1_ICACHE3_PRELOAD_SIZE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x0d8, "L1_DCACHE_PRELOAD_CTRL",
        0x00000002, 0x000000fd, 0x00000000, 0x00000000),
    REG(0x0dc, "L1_DCACHE_PRELOAD_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0e0, "L1_DCACHE_PRELOAD_SIZE",
        0x00000000, 0x00003fff, 0x00000000, 0x00000000),
    REG(0x0e4, "L1_ICACHE0_AUTOLOAD_CTRL",
        0x00000002, 0x00003f1d, 0x00000000, 0x00000000),
    REG(0x0e8, "L1_ICACHE0_AUTOLOAD_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0ec, "L1_ICACHE0_AUTOLOAD_SCT0_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x0f0, "L1_ICACHE0_AUTOLOAD_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x0f4, "L1_ICACHE0_AUTOLOAD_SCT1_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x0f8, "L1_ICACHE1_AUTOLOAD_CTRL",
        0x00000002, 0x00003f1d, 0x00000000, 0x00000000),
    REG(0x0fc, "L1_ICACHE1_AUTOLOAD_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x100, "L1_ICACHE1_AUTOLOAD_SCT0_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x104, "L1_ICACHE1_AUTOLOAD_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x108, "L1_ICACHE1_AUTOLOAD_SCT1_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x10c, "L1_ICACHE2_AUTOLOAD_CTRL",
        0x00000002, 0x00000000, 0x00000000, 0x00000000),
    REG(0x110, "L1_ICACHE2_AUTOLOAD_SCT0_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x114, "L1_ICACHE2_AUTOLOAD_SCT0_SIZE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x118, "L1_ICACHE2_AUTOLOAD_SCT1_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x11c, "L1_ICACHE2_AUTOLOAD_SCT1_SIZE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x120, "L1_ICACHE3_AUTOLOAD_CTRL",
        0x00000002, 0x00000000, 0x00000000, 0x00000000),
    REG(0x124, "L1_ICACHE3_AUTOLOAD_SCT0_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x128, "L1_ICACHE3_AUTOLOAD_SCT0_SIZE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x12c, "L1_ICACHE3_AUTOLOAD_SCT1_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x130, "L1_ICACHE3_AUTOLOAD_SCT1_SIZE",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x134, "L1_DCACHE_AUTOLOAD_CTRL",
        0x00000002, 0x0000ff1d, 0x00000000, 0x00000000),
    REG(0x138, "L1_DCACHE_AUTOLOAD_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x13c, "L1_DCACHE_AUTOLOAD_SCT0_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x140, "L1_DCACHE_AUTOLOAD_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x144, "L1_DCACHE_AUTOLOAD_SCT1_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x148, "L1_DCACHE_AUTOLOAD_SCT2_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x14c, "L1_DCACHE_AUTOLOAD_SCT2_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x150, "L1_DCACHE_AUTOLOAD_SCT3_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x154, "L1_DCACHE_AUTOLOAD_SCT3_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x158, "L1_CACHE_ACS_CNT_INT_ENA",
        0x00000000, 0x00000033, 0x00000000, 0x00000000),
    REG(0x15c, "L1_CACHE_ACS_CNT_INT_CLR",
        0x00000000, 0x00000000, 0x00000033, 0x00000000),
    REG(0x160, "L1_CACHE_ACS_CNT_INT_RAW",
        0x00000000, 0x00000000, 0x00000000, 0x000000ff),
    REG(0x164, "L1_CACHE_ACS_CNT_INT_ST",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x168, "L1_CACHE_ACS_FAIL_CTRL",
        0x00000000, 0x0000001f, 0x00000000, 0x00000000),
    REG(0x16c, "L1_CACHE_ACS_FAIL_INT_ENA",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x170, "L1_CACHE_ACS_FAIL_INT_CLR",
        0x00000000, 0x00000000, 0x00000013, 0x00000000),
    REG(0x174, "L1_CACHE_ACS_FAIL_INT_RAW",
        0x00000000, 0x00000000, 0x00000000, 0x0000001f),
    REG(0x178, "L1_CACHE_ACS_FAIL_INT_ST",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x17c, "L1_CACHE_ACS_CNT_CTRL",
        0x00000000, 0x00000033, 0x00330000, 0x00000000),
    REG(0x180, "L1_IBUS0_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x184, "L1_IBUS0_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x188, "L1_IBUS0_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x18c, "L1_IBUS0_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x190, "L1_IBUS1_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x194, "L1_IBUS1_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x198, "L1_IBUS1_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x19c, "L1_IBUS1_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1a0, "L1_IBUS2_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1a4, "L1_IBUS2_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1a8, "L1_IBUS2_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1ac, "L1_IBUS2_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1b0, "L1_IBUS3_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1b4, "L1_IBUS3_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1b8, "L1_IBUS3_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1bc, "L1_IBUS3_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1c0, "L1_DBUS0_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1c4, "L1_DBUS0_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1c8, "L1_DBUS0_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1cc, "L1_DBUS0_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1d0, "L1_DBUS0_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1d4, "L1_DBUS1_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1d8, "L1_DBUS1_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1dc, "L1_DBUS1_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1e0, "L1_DBUS1_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1e4, "L1_DBUS1_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1e8, "L1_DBUS2_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1ec, "L1_DBUS2_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1f0, "L1_DBUS2_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1f4, "L1_DBUS2_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1f8, "L1_DBUS2_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x1fc, "L1_DBUS3_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x200, "L1_DBUS3_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x204, "L1_DBUS3_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x208, "L1_DBUS3_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x20c, "L1_DBUS3_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x210, "L1_ICACHE0_ACS_FAIL_ID_ATTR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x214, "L1_ICACHE0_ACS_FAIL_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x218, "L1_ICACHE1_ACS_FAIL_ID_ATTR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x21c, "L1_ICACHE1_ACS_FAIL_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x220, "L1_ICACHE2_ACS_FAIL_ID_ATTR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x224, "L1_ICACHE2_ACS_FAIL_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x228, "L1_ICACHE3_ACS_FAIL_ID_ATTR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x22c, "L1_ICACHE3_ACS_FAIL_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x230, "L1_DCACHE_ACS_FAIL_ID_ATTR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x234, "L1_DCACHE_ACS_FAIL_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x238, "SYNC_L1_CACHE_PRELOAD_INT_ENA",
        0x00000000, 0x000029d3, 0x00000000, 0x00000000),
    REG(0x23c, "SYNC_L1_CACHE_PRELOAD_INT_CLR",
        0x00000000, 0x00000000, 0x000029d3, 0x00000000),
    REG(0x240, "SYNC_L1_CACHE_PRELOAD_INT_RAW",
        0x00000000, 0x00000000, 0x00000000, 0x00002fdf),
    REG(0x244, "SYNC_L1_CACHE_PRELOAD_INT_ST",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x248, "SYNC_L1_CACHE_PRELOAD_EXCEPTION",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x24c, "L1_CACHE_SYNC_RST_CTRL",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x250, "L1_CACHE_PRELOAD_RST_CTRL",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x254, "L1_CACHE_AUTOLOAD_BUF_CLR_CTRL",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x258, "L1_UNALLOCATE_BUFFER_CLEAR",
        0x00000000, 0x00000013, 0x00000000, 0x00000000),
    REG(0x25c, "L1_CACHE_OBJECT_CTRL",
        0x00000000, 0x000004d3, 0x00000000, 0x00000000),
    REG(0x260, "L1_CACHE_WAY_OBJECT",
        0x00000000, 0x00000007, 0x00000000, 0x00000000),
    REG(0x264, "L1_CACHE_VADDR",
        0x40000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x268, "L1_CACHE_DEBUG_BUS",
        0x00000268, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x270, "L2_CACHE_CTRL", 0x00000010, 0x00000010, 0x00000000, 0x00000000),
    REG(0x274, "L2_BYPASS_CACHE_CONF",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x278, "L2_CACHE_CACHESIZE_CONF",
        0x00000200, 0x00000e00, 0x00000000, 0x00000000),
    REG(0x27c, "L2_CACHE_BLOCKSIZE_CONF",
        0x00000008, 0x00000018, 0x00000000, 0x00000000),
    REG(0x280, "L2_CACHE_WRAP_AROUND_CTRL",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x284, "L2_CACHE_TAG_MEM_POWER_CTRL",
        0x00500000, 0x00700000, 0x00000000, 0x00000000),
    REG(0x288, "L2_CACHE_DATA_MEM_POWER_CTRL",
        0x00500000, 0x00700000, 0x00000000, 0x00000000),
    REG(0x28c, "L2_CACHE_FREEZE_CTRL",
        0x00000000, 0x00300000, 0x00000000, 0x00000000),
    REG(0x290, "L2_CACHE_DATA_MEM_ACS_CONF",
        0x00300000, 0x00300000, 0x00000000, 0x00000000),
    REG(0x294, "L2_CACHE_TAG_MEM_ACS_CONF",
        0x00300000, 0x00300000, 0x00000000, 0x00000000),
    REG(0x298, "L2_CACHE_PRELOCK_CONF",
        0x00000000, 0x0000003f, 0x00000000, 0x00000000),
    REG(0x29c, "L2_CACHE_PRELOCK_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2a0, "L2_CACHE_PRELOCK_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2a4, "L2_CACHE_PRELOCK_SCT_SIZE",
        0xffffffff, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2a8, "L2_CACHE_PRELOAD_CTRL",
        0x00000002, 0x000000fd, 0x00000000, 0x00000000),
    REG(0x2ac, "L2_CACHE_PRELOAD_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2b0, "L2_CACHE_PRELOAD_SIZE",
        0x00000000, 0x0000ffff, 0x00000000, 0x00000000),
    REG(0x2b4, "L2_CACHE_AUTOLOAD_CTRL",
        0x00000002, 0x0000ff1d, 0x00000000, 0x00000000),
    REG(0x2b8, "L2_CACHE_AUTOLOAD_SCT0_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2bc, "L2_CACHE_AUTOLOAD_SCT0_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x2c0, "L2_CACHE_AUTOLOAD_SCT1_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2c4, "L2_CACHE_AUTOLOAD_SCT1_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x2c8, "L2_CACHE_AUTOLOAD_SCT2_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2cc, "L2_CACHE_AUTOLOAD_SCT2_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x2d0, "L2_CACHE_AUTOLOAD_SCT3_ADDR",
        0x00000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x2d4, "L2_CACHE_AUTOLOAD_SCT3_SIZE",
        0x00000000, 0x0fffffff, 0x00000000, 0x00000000),
    REG(0x2d8, "L2_CACHE_ACS_CNT_INT_ENA",
        0x00000000, 0x00003300, 0x00000000, 0x00000000),
    REG(0x2dc, "L2_CACHE_ACS_CNT_INT_CLR",
        0x00000000, 0x00000000, 0x00003300, 0x00000000),
    REG(0x2e0, "L2_CACHE_ACS_CNT_INT_RAW",
        0x00000000, 0x00000000, 0x00000000, 0x0000ff00),
    REG(0x2e4, "L2_CACHE_ACS_CNT_INT_ST",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x2e8, "L2_CACHE_ACS_FAIL_CTRL",
        0x00000000, 0x00000001, 0x00000000, 0x00000000),
    REG(0x2ec, "L2_CACHE_ACS_FAIL_INT_ENA",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x2f0, "L2_CACHE_ACS_FAIL_INT_CLR",
        0x00000000, 0x00000000, 0x00000020, 0x00000000),
    REG(0x2f4, "L2_CACHE_ACS_FAIL_INT_RAW",
        0x00000000, 0x00000000, 0x00000000, 0x00000020),
    REG(0x2f8, "L2_CACHE_ACS_FAIL_INT_ST",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x2fc, "L2_CACHE_ACS_CNT_CTRL",
        0x00000000, 0x00003300, 0x33000000, 0x00000000),
    REG(0x300, "L2_IBUS0_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x304, "L2_IBUS0_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x308, "L2_IBUS0_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x30c, "L2_IBUS0_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x310, "L2_IBUS1_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x314, "L2_IBUS1_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x318, "L2_IBUS1_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x31c, "L2_IBUS1_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x320, "L2_IBUS2_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x324, "L2_IBUS2_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x328, "L2_IBUS2_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x32c, "L2_IBUS2_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x330, "L2_IBUS3_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x334, "L2_IBUS3_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x338, "L2_IBUS3_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x33c, "L2_IBUS3_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x340, "L2_DBUS0_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x344, "L2_DBUS0_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x348, "L2_DBUS0_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x34c, "L2_DBUS0_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x350, "L2_DBUS0_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x354, "L2_DBUS1_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x358, "L2_DBUS1_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x35c, "L2_DBUS1_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x360, "L2_DBUS1_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x364, "L2_DBUS1_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x368, "L2_DBUS2_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x36c, "L2_DBUS2_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x370, "L2_DBUS2_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x374, "L2_DBUS2_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x378, "L2_DBUS2_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x37c, "L2_DBUS3_ACS_HIT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x380, "L2_DBUS3_ACS_MISS_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x384, "L2_DBUS3_ACS_CONFLICT_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x388, "L2_DBUS3_ACS_NXTLVL_RD_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x38c, "L2_DBUS3_ACS_NXTLVL_WR_CNT",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x390, "L2_CACHE_ACS_FAIL_ID_ATTR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x394, "L2_CACHE_ACS_FAIL_ADDR",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x398, "L2_CACHE_SYNC_PRELOAD_INT_ENA",
        0x00000000, 0x00001020, 0x00000000, 0x00000000),
    REG(0x39c, "L2_CACHE_SYNC_PRELOAD_INT_CLR",
        0x00000000, 0x00000000, 0x00001020, 0x00000000),
    REG(0x3a0, "L2_CACHE_SYNC_PRELOAD_INT_RAW",
        0x00000000, 0x00000000, 0x00000000, 0x00001020),
    REG(0x3a4, "L2_CACHE_SYNC_PRELOAD_INT_ST",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x3a8, "L2_CACHE_SYNC_PRELOAD_EXCEPTION",
        0x00000000, 0x00000000, 0x00000000, 0x00000000),
    REG(0x3ac, "L2_CACHE_SYNC_RST_CTRL",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x3b0, "L2_CACHE_PRELOAD_RST_CTRL",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x3b4, "L2_CACHE_AUTOLOAD_BUF_CLR_CTRL",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x3b8, "L2_UNALLOCATE_BUFFER_CLEAR",
        0x00000000, 0x00000020, 0x00000000, 0x00000000),
    REG(0x3bc, "L2_CACHE_ACCESS_ATTR_CTRL",
        0x0000000f, 0x0000000f, 0x00000000, 0x00000000),
    REG(0x3c0, "L2_CACHE_OBJECT_CTRL",
        0x00000000, 0x00000820, 0x00000000, 0x00000000),
    REG(0x3c4, "L2_CACHE_WAY_OBJECT",
        0x00000000, 0x00000007, 0x00000000, 0x00000000),
    REG(0x3c8, "L2_CACHE_VADDR",
        0x40000000, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x3cc, "L2_CACHE_DEBUG_BUS",
        0x000003cc, 0xffffffff, 0x00000000, 0x00000000),
    REG(0x3d4, "CLOCK_GATE", 0x00000001, 0x00000001, 0x00000000, 0x00000000),
    REG(0x3fc, "DATE", 0x02503250, 0x0fffffff, 0x00000000, 0x00000000),
};

#undef REG

#define R(a) ((a) / 4)

#define A_L1_ICACHE_CTRL                0x000
#define A_L1_DCACHE_CTRL                0x004
#define A_L1_BYPASS_CACHE_CONF          0x008
#define A_L1_TAG_MEM_POWER_CTRL         0x024
#define A_L1_DATA_MEM_POWER_CTRL        0x028
#define A_L1_FREEZE_CTRL                0x02c
#define A_L1_ICACHE0_PRELOCK_CONF       0x038
#define A_L1_ICACHE1_PRELOCK_CONF       0x048
#define A_L1_DCACHE_PRELOCK_CONF        0x078
#define A_LOCK_CTRL                     0x088
#define A_LOCK_MAP                      0x08c
#define A_LOCK_ADDR                     0x090
#define A_LOCK_SIZE                     0x094
#define A_SYNC_CTRL                     0x098
#define A_SYNC_MAP                      0x09c
#define A_SYNC_ADDR                     0x0a0
#define A_SYNC_SIZE                     0x0a4
#define A_L1_ICACHE0_PRELOAD_CTRL       0x0a8
#define A_L1_ICACHE1_PRELOAD_CTRL       0x0b4
#define A_L1_DCACHE_PRELOAD_CTRL        0x0d8
#define A_L1_ACS_CNT_INT_ENA            0x158
#define A_L1_ACS_FAIL_CTRL              0x168
#define A_L1_ACS_FAIL_INT_ENA           0x16c
#define A_L1_ACS_CNT_CTRL               0x17c
#define A_L1_ICACHE0_ACS_FAIL_ADDR      0x214
#define A_L1_ICACHE1_ACS_FAIL_ADDR      0x21c
#define A_L1_DCACHE_ACS_FAIL_ADDR       0x234
#define A_SYNC_L1_INT_ENA               0x238
#define A_SYNC_L1_EXCEPTION             0x248
#define A_L1_CACHE_OBJECT_CTRL          0x25c
#define A_L2_CACHE_CTRL                 0x270
#define A_L2_BYPASS_CACHE_CONF          0x274
#define A_L2_CACHESIZE_CONF             0x278
#define A_L2_BLOCKSIZE_CONF             0x27c
#define A_L2_TAG_MEM_POWER_CTRL         0x284
#define A_L2_DATA_MEM_POWER_CTRL        0x288
#define A_L2_FREEZE_CTRL                0x28c
#define A_L2_PRELOCK_CONF               0x298
#define A_L2_PRELOAD_CTRL               0x2a8
#define A_L2_ACS_CNT_INT_ENA            0x2d8
#define A_L2_ACS_FAIL_CTRL              0x2e8
#define A_L2_ACS_FAIL_INT_ENA           0x2ec
#define A_L2_ACS_CNT_CTRL               0x2fc
#define A_L2_ACS_FAIL_ADDR              0x394
#define A_L2_SYNC_INT_ENA               0x398
#define A_L2_SYNC_EXCEPTION             0x3a8
#define A_L2_CACHE_OBJECT_CTRL          0x3c0

/* The interrupt groups: ENA, CLR, RAW, ST at 4-byte steps. */
static const hwaddr esp32p4_cache_int_groups[] = {
    A_L1_ACS_CNT_INT_ENA, A_L1_ACS_FAIL_INT_ENA, A_SYNC_L1_INT_ENA,
    A_L2_ACS_CNT_INT_ENA, A_L2_ACS_FAIL_INT_ENA, A_L2_SYNC_INT_ENA,
};
#define INT_ENA     0
#define INT_CLR     4
#define INT_RAW     8
#define INT_ST      12

/* SYNC_MAP and LOCK_MAP. */
#define MAP_L1_I0       BIT(0)
#define MAP_L1_I1       BIT(1)
#define MAP_L1_I        (MAP_L1_I0 | MAP_L1_I1 | BIT(2) | BIT(3))
#define MAP_L1_D        BIT(4)
#define MAP_L2          BIT(5)

#define SYNC_INVALIDATE_ENA     BIT(0)
#define SYNC_CLEAN_ENA          BIT(1)
#define SYNC_WRITEBACK_ENA      BIT(2)
#define SYNC_WB_INVALIDATE_ENA  BIT(3)
#define SYNC_OPS                0xf
#define SYNC_DONE               BIT(4)
#define LOCK_ENA                BIT(0)
#define UNLOCK_ENA              BIT(1)
#define PRELOAD_ENA             BIT(0)
#define PRELOAD_MODE            BIT(7)

/* SYNC_L1_CACHE_PRELOAD_INT and L2_CACHE_SYNC_PRELOAD_INT. */
#define INT_L1_I0_PLD_DONE      BIT(0)
#define INT_L1_I1_PLD_DONE      BIT(1)
#define INT_L1_D_PLD_DONE       BIT(4)
#define INT_SYNC_DONE           BIT(6)
#define INT_SYNC_ERR            BIT(13)
#define INT_L2_PLD_DONE         BIT(5)
/* L1_CACHE_ACS_FAIL_INT and L2_CACHE_ACS_FAIL_INT. */
#define INT_L1_I0_FAIL          BIT(0)
#define INT_L1_I1_FAIL          BIT(1)
#define INT_L1_D_FAIL           BIT(4)
#define INT_L2_FAIL             BIT(5)
/* *_ACS_FAIL_CTRL. */
#define FAIL_CHECK_L1_I0        BIT(0)
#define FAIL_CHECK_L1_I1        BIT(1)
#define FAIL_CHECK_L1_D         BIT(4)
#define FAIL_CHECK_L2           BIT(0)

#define SYNC_ERR_CODE_MAP       0
#define SYNC_ERR_CODE_CONFLICT  1

#define BYPASS_L1_ICACHE0       BIT(0)
#define BYPASS_L1_ICACHE1       BIT(1)
#define BYPASS_L1_DCACHE        BIT(4)
#define BYPASS_L2_CACHE         BIT(5)
#define SHUT_BUS0               BIT(0)
#define SHUT_BUS1               BIT(1)
#define SHUT_L2_DMA             BIT(4)
#define CACHESIZE_128K          BIT(9)
#define CACHESIZE_256K          BIT(10)
#define CACHESIZE_512K          BIT(11)
#define BLOCKSIZE_128           BIT(4)

/* The granule of the L1's fills, and of the cores' translations. */
#define FILL            4096
#define LINES_PER_FILL  (FILL / ESP32P4_CACHE_LINE)

static const hwaddr esp32p4_cache_view_base[ESP32P4_CACHE_NVIEWS] = {
    [ESP32P4_CACHE_VIEW_FLASH] = 0x40000000,
    [ESP32P4_CACHE_VIEW_PSRAM] = 0x48000000,
    [ESP32P4_CACHE_VIEW_L2MEM] = 0x4ff00000,
};

static const char *const esp32p4_cache_view_name[ESP32P4_CACHE_NVIEWS] = {
    [ESP32P4_CACHE_VIEW_FLASH] = "flash",
    [ESP32P4_CACHE_VIEW_PSRAM] = "psram",
    [ESP32P4_CACHE_VIEW_L2MEM] = "l2mem",
};

#define L2MEM_SIZE      (768 * KiB)

/* Registers and interrupts */

static uint32_t reg(ESP32P4CacheState *c, hwaddr a)
{
    return c->regs[R(a)];
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void cache_update_irq(ESP32P4CacheState *c)
{
    bool level = false;
    int i;

    for (i = 0; i < ARRAY_SIZE(esp32p4_cache_int_groups); i++) {
        hwaddr g = esp32p4_cache_int_groups[i];

        level |= (reg(c, g + INT_RAW) & reg(c, g + INT_ENA)) != 0;
    }
    qemu_set_irq(c->irq, level);
}

static void cache_raise(ESP32P4CacheState *c, hwaddr group, uint32_t bits)
{
    c->regs[R(group + INT_RAW)] |= bits;
}

/* The translations */

static void cache_flush_cpu(CPUState *cs, run_on_cpu_data data)
{
    tlb_flush(cs);
}

/*
 * Make every core translate its accesses through the caches anew, before
 * it runs its next instructions.
 */
static void cache_flush_tlbs(void)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        async_run_on_cpu(cs, cache_flush_cpu, RUN_ON_CPU_NULL);
    }
}

static void cache_flush_cpu_page(CPUState *cs, run_on_cpu_data data)
{
    tlb_flush_page(cs, data.target_ptr);
}

/*
 * Make every core translate its accesses to the page at `vaddr` anew, as
 * once the L1 holds all of it they go straight to its contents.
 */
static void cache_flush_tlb_page(uint32_t vaddr)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        async_run_on_cpu(cs, cache_flush_cpu_page,
                         RUN_ON_CPU_TARGET_PTR(vaddr));
    }
}

/* The HP core making the access in progress. */
static int cache_core(void)
{
    return current_cpu && current_cpu->cpu_index == 1 ? 1 : 0;
}

static bool cache_l1_bypassed(ESP32P4CacheState *c, int core)
{
    return reg(c, A_L1_BYPASS_CACHE_CONF) &
           (BYPASS_L1_DCACHE | (core ? BYPASS_L1_ICACHE1 : BYPASS_L1_ICACHE0));
}

static bool cache_bus_shut(ESP32P4CacheState *c, int core, bool *dbus)
{
    uint32_t bit = core ? SHUT_BUS1 : SHUT_BUS0;

    *dbus = reg(c, A_L1_DCACHE_CTRL) & bit;
    return *dbus || (reg(c, A_L1_ICACHE_CTRL) & bit);
}

static bool cache_l2_bypassed(ESP32P4CacheState *c)
{
    return reg(c, A_L2_BYPASS_CACHE_CONF) & BYPASS_L2_CACHE;
}

/* The L2's line size: 64 or 128 bytes (L2_CACHE_BLOCKSIZE_CONF). */
static unsigned cache_l2_line(ESP32P4CacheState *c)
{
    return reg(c, A_L2_BLOCKSIZE_CONF) & BLOCKSIZE_128 ? 128 : 64;
}

/*
 * The 64-byte units [*first, *end) of the L2 line that holds unit `unit`:
 * the model keeps the L2's state per 64 bytes, and a 128-byte line's two
 * units share theirs.
 */
static void cache_l2_units(ESP32P4CacheState *c, uint32_t unit,
                           uint32_t *first, uint32_t *end)
{
    uint32_t n = cache_l2_line(c) / ESP32P4_CACHE_LINE;

    *first = QEMU_ALIGN_DOWN(unit, n);
    *end = *first + n;
}

static bool view_ext(ESP32P4CacheView *v)
{
    return v->kind != ESP32P4_CACHE_VIEW_L2MEM;
}

static ESP32P4MSPIState *view_mspi(ESP32P4CacheView *v)
{
    return v->kind == ESP32P4_CACHE_VIEW_FLASH ? v->cache->flash_mspi
                                                : v->cache->psram_mspi;
}

static bool view_occupied(ESP32P4CacheView *v, uint32_t off)
{
    return v->kind == ESP32P4_CACHE_VIEW_L2MEM &&
           off < v->cache->l2_cache_size;
}

static uint8_t *view_l1(ESP32P4CacheView *v)
{
    return memory_region_get_ram_ptr(&v->l1);
}

static uint8_t *view_clean(ESP32P4CacheView *v)
{
    return memory_region_get_ram_ptr(&v->l1_clean);
}

static uint8_t *view_l2(ESP32P4CacheView *v)
{
    return memory_region_get_ram_ptr(&v->l2);
}

/* Whether the cores' accesses to the page at `off` may go straight to L1. */
static bool view_page_fast(ESP32P4CacheView *v, uint32_t off, int core)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t first = off / ESP32P4_CACHE_LINE;
    bool dbus;

    if (cache_l1_bypassed(c, core) ||
        cache_bus_shut(c, core, &dbus) || view_occupied(v, off)) {
        return false;
    }
    return find_next_zero_bit(v->l1_valid, first + LINES_PER_FILL, first) >=
           first + LINES_PER_FILL;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static IOMMUTLBEntry cache_view_translate(IOMMUMemoryRegion *iommu,
                                          hwaddr addr,
                                          IOMMUAccessFlags flags,
                                          int iommu_idx)
{
    ESP32P4CacheView *v = container_of(iommu, ESP32P4CacheView, iommu);
    hwaddr page = addr & ~(hwaddr)(FILL - 1);
    bool fast = current_cpu && view_page_fast(v, page, cache_core());
    IOMMUTLBEntry ret = {
        .target_as = &v->target_as,
        .iova = page,
        .translated_addr = fast ? page : v->size + page,
        .addr_mask = FILL - 1,
        .perm = IOMMU_RW,
    };

    return ret;
}

/* The L2MEM split */

/* [spec:nuos:req:emu.esp32p4.memory] */
static uint32_t cache_l2_size(ESP32P4CacheState *c)
{
    uint32_t conf = reg(c, A_L2_CACHESIZE_CONF);

    if (cache_l2_bypassed(c)) {
        return 0;
    }
    if (conf & CACHESIZE_512K) {
        return 512 * KiB;
    }
    if (conf & CACHESIZE_256K) {
        return 256 * KiB;
    }
    if (conf & CACHESIZE_128K) {
        return 128 * KiB;
    }
    return 0;
}

static void cache_update_split(ESP32P4CacheState *c)
{
    uint32_t size = cache_l2_size(c);
    MemoryRegion *mr[2] = { &c->l2mem_cached, &c->l2mem_direct };
    int i;

    if (size == c->l2_cache_size) {
        return;
    }
    c->l2_cache_size = size;
    memory_region_transaction_begin();
    for (i = 0; i < 2; i++) {
        memory_region_set_enabled(mr[i], size != 0);
        if (size) {
            memory_region_set_size(mr[i], size);
        }
    }
    memory_region_transaction_commit();
    cache_flush_tlbs();
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult cache_l2mem_occupied(ESP32P4CacheState *c, uint32_t off,
                                        uint64_t *data, bool write,
                                        MemTxAttrs attrs)
{
    if (!write) {
        *data = 0;
    }
    if (attrs.debug) {
        return MEMTX_OK;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s at 0x%08" PRIx32 " in the "
                  "%" PRIu32 " KiB of L2MEM the L2 cache occupies\n",
                  TYPE_ESP32P4_CACHE, write ? "write" : "read",
                  (uint32_t)(esp32p4_cache_view_base[ESP32P4_CACHE_VIEW_L2MEM]
                             + off), c->l2_cache_size / (uint32_t)KiB);
    if (c->hp_sys) {
        esp32p4_hp_sys_l2mem_exceed(c->hp_sys, off, write);
    }
    return c->l2mem_err_resp ? MEMTX_ERROR : MEMTX_OK;
}

static MemTxResult cache_l2mem_overlay_read(void *opaque, hwaddr addr,
                                            uint64_t *data, unsigned size,
                                            MemTxAttrs attrs)
{
    return cache_l2mem_occupied(opaque, addr, data, false, attrs);
}

static MemTxResult cache_l2mem_overlay_write(void *opaque, hwaddr addr,
                                             uint64_t data, unsigned size,
                                             MemTxAttrs attrs)
{
    return cache_l2mem_occupied(opaque, addr, &data, true, attrs);
}

static const MemoryRegionOps cache_l2mem_overlay_ops = {
    .read_with_attrs = cache_l2mem_overlay_read,
    .write_with_attrs = cache_l2mem_overlay_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/* Lines */

static bool line_dirty(ESP32P4CacheView *v, uint32_t line)
{
    uint32_t off = line * ESP32P4_CACHE_LINE;

    return memcmp(view_l1(v) + off, view_clean(v) + off,
                  ESP32P4_CACHE_LINE) != 0;
}

/*
 * Put `data` into L1 line `line`, as filled: valid and clean. A fill can
 * be made while a core translates the instruction it fetches, so it may
 * not invalidate translations itself: the line's were invalidated when
 * the line left the L1 (l1_untranslate()), and none are made from a page
 * the L1 does not hold all of.
 */
static void l1_fill_line(ESP32P4CacheView *v, uint32_t line,
                         const uint8_t *data)
{
    uint32_t off = line * ESP32P4_CACHE_LINE;

    memcpy(view_l1(v) + off, data, ESP32P4_CACHE_LINE);
    memory_region_set_dirty(&v->l1, off, ESP32P4_CACHE_LINE);
    memcpy(view_clean(v) + off, data, ESP32P4_CACHE_LINE);
    set_bit(line, v->l1_valid);
}

/* Drop what the cores translated from L1 lines [first, end). */
static void l1_untranslate(ESP32P4CacheView *v, uint32_t first, uint32_t end)
{
    ram_addr_t ra = memory_region_get_ram_addr(&v->l1);

    if (tcg_enabled() && first < end) {
        tb_invalidate_phys_range(NULL, ra + first * ESP32P4_CACHE_LINE,
                                 ra + end * ESP32P4_CACHE_LINE - 1);
    }
}

/* Whether `vaddr` is in an enabled prelock section of the cache at `conf`. */
static bool cache_prelocked(ESP32P4CacheState *c, hwaddr conf, bool l2,
                            uint32_t vaddr)
{
    uint32_t en = reg(c, conf);
    uint32_t sizes = reg(c, conf + 12);
    int i;

    for (i = 0; i < 2; i++) {
        uint32_t base = reg(c, conf + 4 + 4 * i);
        uint32_t size = l2 ? extract32(sizes, 16 * i, 16)
                           : extract32(sizes, 16 * i, 14);

        if ((en & BIT(i)) && vaddr >= base && vaddr - base < size) {
            return true;
        }
    }
    return false;
}

static bool l1_prelocked(ESP32P4CacheState *c, uint32_t vaddr)
{
    return cache_prelocked(c, A_L1_DCACHE_PRELOCK_CONF, false, vaddr) ||
           cache_prelocked(c, A_L1_ICACHE0_PRELOCK_CONF, false, vaddr) ||
           cache_prelocked(c, A_L1_ICACHE1_PRELOCK_CONF, false, vaddr);
}

static uint32_t view_vaddr(ESP32P4CacheView *v, uint32_t off)
{
    return esp32p4_cache_view_base[v->kind] + off;
}

/*
 * Record an error the next level answered an access at `vaddr` with, by
 * core `core` through its instruction or data bus: in the L2 for the
 * external memory, then passed to the L1 and then to the core as their
 * check modes allow. Returns what the core is answered with.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult cache_access_fail(ESP32P4CacheState *c, uint32_t vaddr,
                                     bool from_l2, int core, bool dbus)
{
    hwaddr fail_addr;
    uint32_t fail_int, check;

    if (from_l2) {
        c->regs[R(A_L2_ACS_FAIL_ADDR)] = vaddr;
        cache_raise(c, A_L2_ACS_FAIL_INT_ENA, INT_L2_FAIL);
        if (!(reg(c, A_L2_ACS_FAIL_CTRL) & FAIL_CHECK_L2)) {
            cache_update_irq(c);
            return MEMTX_OK;
        }
    }
    if (dbus) {
        fail_addr = A_L1_DCACHE_ACS_FAIL_ADDR;
        fail_int = INT_L1_D_FAIL;
        check = FAIL_CHECK_L1_D;
    } else {
        fail_addr = core ? A_L1_ICACHE1_ACS_FAIL_ADDR
                         : A_L1_ICACHE0_ACS_FAIL_ADDR;
        fail_int = core ? INT_L1_I1_FAIL : INT_L1_I0_FAIL;
        check = core ? FAIL_CHECK_L1_I1 : FAIL_CHECK_L1_I0;
    }
    c->regs[R(fail_addr)] = vaddr;
    cache_raise(c, A_L1_ACS_FAIL_INT_ENA, fail_int);
    cache_update_irq(c);
    return reg(c, A_L1_ACS_FAIL_CTRL) & check ? MEMTX_ERROR : MEMTX_OK;
}

/*
 * Whether an L2 prelock section takes the L2 line holding unit `line` as
 * it fills, by the line's start: a 128-byte line is locked whole or not.
 */
static bool l2_prelocked(ESP32P4CacheView *v, uint32_t line)
{
    uint32_t first, end;

    cache_l2_units(v->cache, line, &first, &end);
    return cache_prelocked(v->cache, A_L2_PRELOCK_CONF, true,
                           view_vaddr(v, first * ESP32P4_CACHE_LINE));
}

/*
 * Fill the L2 lines of the fill block at `off` that it does not hold and
 * that `need` marks, from the memory, into `buf` (the block's contents);
 * with the L2 bypassed, only read them. Returns the memory's answer.
 */
static MemTxResult l2_fill_block(ESP32P4CacheView *v, uint32_t off,
                                 uint8_t *buf, bool l2_alloc)
{
    uint32_t first = off / ESP32P4_CACHE_LINE;
    MemTxResult r;
    uint32_t i;

    r = esp32p4_mspi_axi_read(view_mspi(v), off, buf, FILL);
    if (r != MEMTX_OK) {
        return r;
    }
    for (i = 0; i < LINES_PER_FILL; i++) {
        uint32_t line = first + i;
        uint32_t loff = i * ESP32P4_CACHE_LINE;

        if (test_bit(line, v->l2_valid)) {
            /* The L2's copy, which may be newer than the memory's. */
            memcpy(buf + loff, view_l2(v) + line * ESP32P4_CACHE_LINE,
                   ESP32P4_CACHE_LINE);
        } else if (l2_alloc) {
            memcpy(view_l2(v) + line * ESP32P4_CACHE_LINE, buf + loff,
                   ESP32P4_CACHE_LINE);
            set_bit(line, v->l2_valid);
            clear_bit(line, v->l2_dirty);
            if (l2_prelocked(v, line)) {
                set_bit(line, v->l2_lock);
            }
        }
    }
    return MEMTX_OK;
}

/*
 * Fill the L1 lines of the fill block holding `off` that it does not
 * hold. Returns the next level's answer; on an error nothing is filled.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult l1_fill_block(ESP32P4CacheView *v, uint32_t off)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t start = QEMU_ALIGN_DOWN(off, FILL);
    uint32_t first = start / ESP32P4_CACHE_LINE;
    uint8_t buf[FILL];
    uint32_t i;

    if (find_next_zero_bit(v->l1_valid, first + LINES_PER_FILL, first) >=
        first + LINES_PER_FILL) {
        return MEMTX_OK;
    }
    if (view_ext(v)) {
        bool all_l2 = find_next_zero_bit(v->l2_valid, first + LINES_PER_FILL,
                                         first) >= first + LINES_PER_FILL;

        if (all_l2) {
            memcpy(buf, view_l2(v) + start, FILL);
        } else {
            MemTxResult r = l2_fill_block(v, start, buf,
                                          !cache_l2_bypassed(c));

            if (r != MEMTX_OK) {
                return r;
            }
        }
    } else {
        memcpy(buf, (uint8_t *)memory_region_get_ram_ptr(c->l2mem_mr) +
               start, FILL);
    }
    for (i = 0; i < LINES_PER_FILL; i++) {
        uint32_t line = first + i;

        if (test_bit(line, v->l1_valid)) {
            continue;
        }
        l1_fill_line(v, line, buf + i * ESP32P4_CACHE_LINE);
        if (l1_prelocked(c, view_vaddr(v, line * ESP32P4_CACHE_LINE))) {
            set_bit(line, v->l1_lock);
        }
    }
    cache_flush_tlb_page(view_vaddr(v, start));
    return MEMTX_OK;
}

/*
 * Store into L2MEM at `off`, as a write on the bus would: the cores'
 * translations of code there are dropped.
 */
static void l2mem_store(ESP32P4CacheState *c, uint32_t off,
                        const uint8_t *data, unsigned len)
{
    ram_addr_t ra = memory_region_get_ram_addr(c->l2mem_mr);

    memcpy((uint8_t *)memory_region_get_ram_ptr(c->l2mem_mr) + off, data,
           len);
    memory_region_set_dirty(c->l2mem_mr, off, len);
    if (tcg_enabled()) {
        tb_invalidate_phys_range(NULL, ra + off, ra + off + len - 1);
    }
}

/* Mark the L2 line holding unit `line` dirty: the whole line. */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void l2_mark_dirty(ESP32P4CacheView *v, uint32_t line)
{
    uint32_t first, end;

    cache_l2_units(v->cache, line, &first, &end);
    bitmap_set(v->l2_dirty, first, end - first);
}

/*
 * Allocate the L2 line holding unit `line` for an L1 write-back into it,
 * which brings the line's other units from the memory where the L2 does
 * not hold them (only a 128-byte line has another), filled as a miss
 * fills them. Returns the memory's answer; on an error nothing is
 * allocated.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult l2_allocate_line(ESP32P4CacheView *v, uint32_t line)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t first, end, u;

    cache_l2_units(c, line, &first, &end);
    for (u = first; u < end; u++) {
        if (u != line && !test_bit(u, v->l2_valid)) {
            MemTxResult r = esp32p4_mspi_axi_read(view_mspi(v),
                                                  u * ESP32P4_CACHE_LINE,
                                                  view_l2(v) +
                                                  u * ESP32P4_CACHE_LINE,
                                                  ESP32P4_CACHE_LINE);

            if (r != MEMTX_OK) {
                return r;
            }
        }
    }
    for (u = first; u < end; u++) {
        if (test_bit(u, v->l2_valid)) {
            continue;
        }
        set_bit(u, v->l2_valid);
        clear_bit(u, v->l2_dirty);
        if (l2_prelocked(v, u)) {
            set_bit(u, v->l2_lock);
        }
    }
    return MEMTX_OK;
}

/* Write L2 line `line` back to the memory if it is dirty. */
static MemTxResult l2_writeback_line(ESP32P4CacheView *v, uint32_t line)
{
    MemTxResult r;

    if (!test_bit(line, v->l2_dirty)) {
        return MEMTX_OK;
    }
    r = esp32p4_mspi_axi_write(view_mspi(v), line * ESP32P4_CACHE_LINE,
                               view_l2(v) + line * ESP32P4_CACHE_LINE,
                               ESP32P4_CACHE_LINE);
    if (r == MEMTX_OK) {
        clear_bit(line, v->l2_dirty);
    }
    return r;
}

/* Write L1 line `line` back to the next level if it is dirty. */
static void l1_writeback_line(ESP32P4CacheView *v, uint32_t line)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t off = line * ESP32P4_CACHE_LINE;
    uint8_t *data = view_l1(v) + off;

    if (!test_bit(line, v->l1_valid) || !line_dirty(v, line)) {
        return;
    }
    if (!view_ext(v)) {
        if (view_occupied(v, off)) {
            return;
        }
        l2mem_store(c, off, data, ESP32P4_CACHE_LINE);
    } else if (cache_l2_bypassed(c)) {
        if (esp32p4_mspi_axi_write(view_mspi(v), off, data,
                                   ESP32P4_CACHE_LINE) != MEMTX_OK) {
            return;
        }
    } else {
        if (l2_allocate_line(v, line) != MEMTX_OK) {
            return;
        }
        memcpy(view_l2(v) + off, data, ESP32P4_CACHE_LINE);
        l2_mark_dirty(v, line);
    }
    memcpy(view_clean(v) + off, data, ESP32P4_CACHE_LINE);
}

/* The cores' accesses that do not go straight to the L1 */

/*
 * An access past the L1 (L1 bypassed for the core, or a debugger's): to
 * the L2's copy, or the memory. A debugger's access allocates nothing,
 * and reads from the L1 where it holds the line.
 */
static MemTxResult cache_past_l1(ESP32P4CacheView *v, uint32_t off,
                                 uint8_t *buf, unsigned size, bool write,
                                 bool debug)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t line = off / ESP32P4_CACHE_LINE;
    uint8_t *l2mem = memory_region_get_ram_ptr(c->l2mem_mr);
    uint8_t block[FILL];
    uint32_t start = QEMU_ALIGN_DOWN(off, FILL);
    MemTxResult r;

    if (debug && test_bit(line, v->l1_valid)) {
        if (write) {
            address_space_write(&v->target_as, off, MEMTXATTRS_UNSPECIFIED,
                                buf, size);
        } else {
            memcpy(buf, view_l1(v) + off, size);
        }
        return MEMTX_OK;
    }
    if (!view_ext(v)) {
        if (write) {
            l2mem_store(c, off, buf, size);
        } else {
            memcpy(buf, l2mem + off, size);
        }
        return MEMTX_OK;
    }
    if (!test_bit(line, v->l2_valid)) {
        if (debug || cache_l2_bypassed(c)) {
            return write ? esp32p4_mspi_axi_write(view_mspi(v), off, buf, size)
                         : esp32p4_mspi_axi_read(view_mspi(v), off, buf,
                                                 size);
        }
        r = l2_fill_block(v, start, block, true);
        if (r != MEMTX_OK) {
            return r;
        }
    }
    if (write) {
        memcpy(view_l2(v) + off, buf, size);
        l2_mark_dirty(v, line);
    } else {
        memcpy(buf, view_l2(v) + off, size);
    }
    return MEMTX_OK;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult cache_view_access(ESP32P4CacheView *v, uint32_t off,
                                     uint64_t *data, unsigned size,
                                     bool write, MemTxAttrs attrs)
{
    ESP32P4CacheState *c = v->cache;
    int core = cache_core();
    uint8_t buf[8];
    bool dbus;
    MemTxResult r;

    if (!current_cpu && !attrs.debug) {
        if (write) {
            stn_le_p(buf, size, *data);
        }
        r = address_space_rw(&address_space_memory, view_vaddr(v, off), attrs,
                             buf, size, write);
        if (!write) {
            *data = ldn_le_p(buf, size);
        }
        return r;
    }
    if (view_occupied(v, off)) {
        return cache_l2mem_occupied(c, off, data, write, attrs);
    }
    if (write) {
        stn_le_p(buf, size, *data);
    }
    if (attrs.debug || cache_l1_bypassed(c, core)) {
        r = cache_past_l1(v, off, buf, size, write, attrs.debug);
        if (r != MEMTX_OK && !attrs.debug) {
            r = cache_access_fail(c, view_vaddr(v, off), true, core, true);
        }
        if (!write) {
            *data = r == MEMTX_OK ? ldn_le_p(buf, size) : 0;
        }
        return r;
    }
    if (cache_bus_shut(c, core, &dbus)) {
        if (!write) {
            *data = 0;
        }
        return cache_access_fail(c, view_vaddr(v, off), false, core, dbus);
    }
    r = l1_fill_block(v, off);
    if (r != MEMTX_OK) {
        if (!write) {
            *data = 0;
        }
        return cache_access_fail(c, view_vaddr(v, off), true, core, true);
    }
    if (write) {
        address_space_write(&v->target_as, off, MEMTXATTRS_UNSPECIFIED, buf,
                            size);
    } else {
        *data = ldn_le_p(view_l1(v) + off, size);
    }
    return MEMTX_OK;
}

static MemTxResult cache_miss_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    return cache_view_access(opaque, addr, data, size, false, attrs);
}

static MemTxResult cache_miss_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size, MemTxAttrs attrs)
{
    return cache_view_access(opaque, addr, &data, size, true, attrs);
}

static const MemoryRegionOps cache_miss_ops = {
    .read_with_attrs = cache_miss_read,
    .write_with_attrs = cache_miss_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

/* Operations */

enum {
    OP_INVALIDATE,
    OP_CLEAN,
    OP_WRITEBACK,
    OP_WB_INVALIDATE,
};

/*
 * The lines of view `v` that [vaddr, vaddr + size) touches, in units of
 * `align` bytes, or all its lines for a size of zero. Returns false if it
 * touches none.
 */
static bool view_lines(ESP32P4CacheView *v, uint32_t vaddr, uint32_t size,
                       unsigned align, uint32_t *first, uint32_t *end)
{
    uint64_t base = esp32p4_cache_view_base[v->kind];
    uint64_t lo, hi;

    if (!size) {
        *first = 0;
        *end = v->nlines;
        return true;
    }
    lo = MAX((uint64_t)vaddr, base);
    hi = MIN((uint64_t)vaddr + size, base + v->size);
    if (lo >= hi) {
        return false;
    }
    lo = QEMU_ALIGN_DOWN(lo - base, align);
    hi = MIN(QEMU_ALIGN_UP(hi - base, align), v->size);
    *first = lo / ESP32P4_CACHE_LINE;
    *end = hi / ESP32P4_CACHE_LINE;
    return true;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static bool cache_sync_view(ESP32P4CacheView *v, int op, uint32_t map,
                            uint32_t vaddr, uint32_t size)
{
    ESP32P4CacheState *c = v->cache;
    bool all = size == 0;
    bool inval = op == OP_INVALIDATE || op == OP_WB_INVALIDATE;
    bool wb = op == OP_WRITEBACK || op == OP_WB_INVALIDATE;
    uint32_t dropped_lo = UINT32_MAX, dropped_hi = 0;
    uint32_t first, end, line;

    if ((map & (MAP_L1_D | MAP_L1_I)) &&
        view_lines(v, vaddr, size, ESP32P4_CACHE_LINE, &first, &end)) {
        for (line = find_next_bit(v->l1_valid, end, first); line < end;
             line = find_next_bit(v->l1_valid, end, line + 1)) {
            bool locked = !all && test_bit(line, v->l1_lock);
            uint32_t off = line * ESP32P4_CACHE_LINE;

            if (map & MAP_L1_D) {
                if (wb) {
                    l1_writeback_line(v, line);
                } else if (op == OP_CLEAN) {
                    memcpy(view_clean(v) + off, view_l1(v) + off,
                           ESP32P4_CACHE_LINE);
                }
                if (inval && !locked) {
                    clear_bit(line, v->l1_valid);
                    clear_bit(line, v->l1_lock);
                    dropped_lo = MIN(dropped_lo, line);
                    dropped_hi = line + 1;
                }
            } else if (inval && !locked && !line_dirty(v, line)) {
                clear_bit(line, v->l1_valid);
                clear_bit(line, v->l1_lock);
                dropped_lo = MIN(dropped_lo, line);
                dropped_hi = line + 1;
            }
        }
        l1_untranslate(v, dropped_lo, dropped_hi);
    }
    if ((map & MAP_L2) && view_ext(v) &&
        view_lines(v, vaddr, size, cache_l2_line(c), &first, &end)) {
        for (line = find_next_bit(v->l2_valid, end, first); line < end;
             line = find_next_bit(v->l2_valid, end, line + 1)) {
            bool locked = !all && test_bit(line, v->l2_lock);

            if (wb) {
                l2_writeback_line(v, line);
            } else if (op == OP_CLEAN) {
                clear_bit(line, v->l2_dirty);
            }
            if (inval && !locked) {
                clear_bit(line, v->l2_valid);
                clear_bit(line, v->l2_dirty);
                clear_bit(line, v->l2_lock);
            }
        }
    }
    return dropped_lo < dropped_hi;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void cache_sync(ESP32P4CacheState *c, uint32_t ops)
{
    uint32_t map = reg(c, A_SYNC_MAP) & 0x3f;
    uint32_t vaddr = reg(c, A_SYNC_ADDR);
    uint32_t size = reg(c, A_SYNC_SIZE);
    bool dropped = false;
    int op, i;

    if (!map || ctpop32(ops) > 1) {
        c->regs[R(A_SYNC_L1_EXCEPTION)] =
            deposit32(reg(c, A_SYNC_L1_EXCEPTION), 12, 2,
                      map ? SYNC_ERR_CODE_CONFLICT : SYNC_ERR_CODE_MAP);
        cache_raise(c, A_SYNC_L1_INT_ENA, INT_SYNC_ERR);
    } else {
        op = ctz32(ops);
        for (i = 0; i < ESP32P4_CACHE_NVIEWS; i++) {
            dropped |= cache_sync_view(&c->view[i], op, map, vaddr, size);
        }
        if (dropped) {
            cache_flush_tlbs();
        }
    }
    c->regs[R(A_SYNC_CTRL)] = (reg(c, A_SYNC_CTRL) & ~SYNC_OPS) | SYNC_DONE;
    cache_raise(c, A_SYNC_L1_INT_ENA, INT_SYNC_DONE);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void cache_lock(ESP32P4CacheState *c, bool lock)
{
    uint32_t map = reg(c, A_LOCK_MAP) & 0x3f;
    uint32_t vaddr = reg(c, A_LOCK_ADDR);
    uint32_t size = extract32(reg(c, A_LOCK_SIZE), 0, 16);
    uint32_t first, end, line;
    int i;

    for (i = 0; size && i < ESP32P4_CACHE_NVIEWS; i++) {
        ESP32P4CacheView *v = &c->view[i];

        if ((map & (MAP_L1_I | MAP_L1_D)) &&
            view_lines(v, vaddr, size, ESP32P4_CACHE_LINE, &first, &end)) {
            for (line = first; line < end; line++) {
                if (!lock) {
                    clear_bit(line, v->l1_lock);
                } else if (test_bit(line, v->l1_valid)) {
                    set_bit(line, v->l1_lock);
                }
            }
        }
        if ((map & MAP_L2) && view_ext(v) &&
            view_lines(v, vaddr, size, cache_l2_line(c), &first, &end)) {
            for (line = first; line < end; line++) {
                if (!lock) {
                    clear_bit(line, v->l2_lock);
                } else if (test_bit(line, v->l2_valid)) {
                    set_bit(line, v->l2_lock);
                }
            }
        }
    }
    c->regs[R(A_LOCK_CTRL)] &= ~(LOCK_ENA | UNLOCK_ENA);
}

/*
 * A manual preload of the cache whose PRELOAD_CTRL is at `ctrl`: its range
 * filled into the L1, or into the L2.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void cache_preload(ESP32P4CacheState *c, hwaddr ctrl, bool l2,
                          hwaddr int_group, uint32_t done_int)
{
    uint32_t vaddr = reg(c, ctrl + 4);
    uint32_t size = extract32(reg(c, ctrl + 8), 0, l2 ? 16 : 14);
    uint8_t block[FILL];
    int i;

    if (reg(c, ctrl) & PRELOAD_MODE) {
        qemu_log_mask(LOG_UNIMP, "%s: preload without loading the data "
                      "(PRELOAD_MODE) is not modelled; the data is "
                      "loaded\n", TYPE_ESP32P4_CACHE);
    }
    for (i = 0; size && i < ESP32P4_CACHE_NVIEWS; i++) {
        ESP32P4CacheView *v = &c->view[i];
        uint32_t first, end, off;

        if (l2 && !view_ext(v)) {
            continue;
        }
        if (!view_lines(v, vaddr, size, FILL, &first, &end)) {
            continue;
        }
        for (off = first * ESP32P4_CACHE_LINE;
             off < end * ESP32P4_CACHE_LINE; off += FILL) {
            MemTxResult r;

            if (view_occupied(v, off)) {
                continue;
            }
            r = l2 ? l2_fill_block(v, off, block, true) : l1_fill_block(v, off);
            if (r != MEMTX_OK) {
                cache_access_fail(c, view_vaddr(v, off), true, 0, true);
            }
        }
    }
    c->regs[R(ctrl)] &= ~PRELOAD_ENA;
    cache_raise(c, int_group, done_int);
}

/* Empty the caches `map` names, as their resets do. */
static void cache_empty(ESP32P4CacheState *c, uint32_t map)
{
    int i;

    for (i = 0; i < ESP32P4_CACHE_NVIEWS; i++) {
        cache_sync_view(&c->view[i], OP_INVALIDATE, map, 0, 0);
    }
    cache_flush_tlbs();
}

/* Register access */

static void cache_check_unimp(ESP32P4CacheState *c, hwaddr a)
{
    static const uint32_t pd = BIT(1) | BIT(5) | BIT(17) | BIT(21);

    switch (a) {
    case A_L1_TAG_MEM_POWER_CTRL:
    case A_L1_DATA_MEM_POWER_CTRL:
    case A_L2_TAG_MEM_POWER_CTRL:
    case A_L2_DATA_MEM_POWER_CTRL:
        if (reg(c, a) & pd) {
            qemu_log_mask(LOG_UNIMP, "%s: powering the caches' memories "
                          "down (*_MEM_FORCE_PD) is not modelled; they keep "
                          "their contents\n", TYPE_ESP32P4_CACHE);
        }
        break;
    case A_L1_CACHE_OBJECT_CTRL:
    case A_L2_CACHE_OBJECT_CTRL:
        if (reg(c, a)) {
            qemu_log_mask(LOG_UNIMP, "%s: access to the caches' tag and data "
                          "memories (*_OBJECT_CTRL) is not modelled\n",
                          TYPE_ESP32P4_CACHE);
        }
        break;
    case A_L2_CACHE_CTRL:
        if (!(reg(c, a) & SHUT_L2_DMA)) {
            qemu_log_mask(LOG_UNIMP, "%s: DMA through the L2 cache "
                          "(SHUT_DMA cleared) is not modelled; DMA reaches "
                          "the memory directly\n", TYPE_ESP32P4_CACHE);
        }
        break;
    case A_L1_ACS_CNT_CTRL:
    case A_L2_ACS_CNT_CTRL:
        if ((reg(c, a) & 0xffff) && !c->counters_logged) {
            c->counters_logged = true;
            qemu_log_mask(LOG_UNIMP, "%s: the access counters are not "
                          "modelled; they count nothing\n",
                          TYPE_ESP32P4_CACHE);
        }
        break;
    }
}

static int cache_int_group(hwaddr a, unsigned *which)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(esp32p4_cache_int_groups); i++) {
        hwaddr g = esp32p4_cache_int_groups[i];

        if (a >= g && a < g + 16) {
            *which = a - g;
            return i;
        }
    }
    return -1;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static uint64_t cache_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4CacheState *c = opaque;
    hwaddr a = addr & ~3;
    unsigned which;
    int g;
    uint32_t v;

    if (!esp32p4_cache_regs[R(a)].name) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%03"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_CACHE, addr);
        return 0;
    }
    g = cache_int_group(a, &which);
    if (g >= 0 && which == INT_ST) {
        hwaddr base = esp32p4_cache_int_groups[g];

        v = reg(c, base + INT_RAW) & reg(c, base + INT_ENA);
    } else {
        v = c->regs[R(a)];
    }
    return extract32(v, 8 * (addr & 3), 8 * size);
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static void cache_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
    ESP32P4CacheState *c = opaque;
    hwaddr a = addr & ~3;
    const ESP32P4CacheReg *r = &esp32p4_cache_regs[R(a)];
    unsigned shift = 8 * (addr & 3);
    uint32_t v = val << shift;
    uint32_t mask = MAKE_64BIT_MASK(shift, 8 * size);
    uint32_t *p = &c->regs[R(a)];
    uint32_t old = *p;
    uint32_t trig;
    unsigned which;
    int g;

    if (!r->name) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%03"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_CACHE, addr);
        return;
    }
    *p = (*p & ~(r->rw & mask)) | (v & r->rw & mask);
    *p &= ~(v & r->w1c & mask);
    trig = v & r->wt & mask;

    g = cache_int_group(a, &which);
    if (g >= 0 && which == INT_CLR) {
        c->regs[R(esp32p4_cache_int_groups[g] + INT_RAW)] &= ~trig;
    }

    switch (a) {
    case A_SYNC_CTRL:
        if (*p & SYNC_OPS) {
            cache_sync(c, *p & SYNC_OPS);
        }
        break;
    case A_LOCK_CTRL:
        if (*p & (LOCK_ENA | UNLOCK_ENA)) {
            if ((*p & LOCK_ENA) && (*p & UNLOCK_ENA)) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: LOCK_ENA and UNLOCK_ENA "
                              "together; the lock runs\n",
                              TYPE_ESP32P4_CACHE);
            }
            cache_lock(c, *p & LOCK_ENA);
        }
        break;
    case A_L1_ICACHE0_PRELOAD_CTRL:
    case A_L1_ICACHE1_PRELOAD_CTRL:
    case A_L1_DCACHE_PRELOAD_CTRL:
        if (*p & PRELOAD_ENA) {
            cache_preload(c, a, false, A_SYNC_L1_INT_ENA,
                          a == A_L1_DCACHE_PRELOAD_CTRL ? INT_L1_D_PLD_DONE :
                          a == A_L1_ICACHE1_PRELOAD_CTRL ? INT_L1_I1_PLD_DONE
                                                         : INT_L1_I0_PLD_DONE);
        }
        break;
    case A_L2_PRELOAD_CTRL:
        if (*p & PRELOAD_ENA) {
            cache_preload(c, a, true, A_L2_SYNC_INT_ENA, INT_L2_PLD_DONE);
        }
        break;
    case A_L1_FREEZE_CTRL:
        /* Each cache's FREEZE_DONE (bits 2, 6, 18) follows its FREEZE_EN. */
        *p = (*p & ~(BIT(2) | BIT(6) | BIT(18))) |
             ((*p & (BIT(0) | BIT(4) | BIT(16))) << 2);
        break;
    case A_L2_FREEZE_CTRL:
        *p = deposit32(*p, 22, 1, extract32(*p, 20, 1));
        break;
    case A_L1_ICACHE_CTRL:
    case A_L1_DCACHE_CTRL:
    case A_L1_BYPASS_CACHE_CONF:
        if ((*p ^ old) & (SHUT_BUS0 | SHUT_BUS1 | BYPASS_L1_ICACHE0 |
                          BYPASS_L1_ICACHE1 | BYPASS_L1_DCACHE)) {
            cache_flush_tlbs();
        }
        break;
    case A_L2_BYPASS_CACHE_CONF:
    case A_L2_CACHESIZE_CONF:
        cache_update_split(c);
        break;
    }
    cache_check_unimp(c, a);
    cache_update_irq(c);
}

static const MemoryRegionOps cache_ops = {
    .read = cache_read,
    .write = cache_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* Reset, hooks, realize */

/*
 * Cache `n`'s reset, from HP_SYS_CLKRST (`line` cache_reset) or HP_SYSTEM
 * (sys_cache_reset): either empties the cache as it is asserted.
 */
static void cache_reset_line(ESP32P4CacheState *c, bool *line, int n,
                             int level)
{
    static const uint32_t map[ESP32P4_CACHE_NRESETS] = {
        [ESP32P4_CACHE_RESET_L1_I0] = MAP_L1_I0,
        [ESP32P4_CACHE_RESET_L1_I1] = MAP_L1_I1,
        [ESP32P4_CACHE_RESET_L1_D] = MAP_L1_D,
        [ESP32P4_CACHE_RESET_L2] = MAP_L2,
    };
    bool was = c->cache_reset[n] || c->sys_cache_reset[n];

    line[n] = level;
    if (!was && (c->cache_reset[n] || c->sys_cache_reset[n])) {
        cache_empty(c, map[n]);
    }
}

static void cache_set_cache_reset(void *opaque, int n, int level)
{
    ESP32P4CacheState *c = opaque;

    cache_reset_line(c, c->cache_reset, n, level);
}

static void cache_set_sys_cache_reset(void *opaque, int n, int level)
{
    ESP32P4CacheState *c = opaque;

    cache_reset_line(c, c->sys_cache_reset, n, level);
}

static void cache_set_l2mem_err_resp(void *opaque, int n, int level)
{
    ESP32P4CacheState *c = opaque;

    c->l2mem_err_resp = level;
}

/*
 * A reset: the registers take their reset values and every cache is
 * emptied. SYNC_CTRL's reset value starts an invalidation of the L1
 * caches, which ends at once: it reads done.
 */
static void cache_reset_hold(Object *obj, ResetType type)
{
    ESP32P4CacheState *c = ESP32P4_CACHE(obj);
    int i;

    for (i = 0; i < ESP32P4_CACHE_NREGS; i++) {
        c->regs[i] = esp32p4_cache_regs[i].reset;
    }
    for (i = 0; i < ESP32P4_CACHE_NVIEWS; i++) {
        ESP32P4CacheView *v = &c->view[i];

        l1_untranslate(v, 0, v->nlines);
        bitmap_zero(v->l1_valid, v->nlines);
        bitmap_zero(v->l1_lock, v->nlines);
        bitmap_zero(v->l2_valid, v->nlines);
        bitmap_zero(v->l2_dirty, v->nlines);
        bitmap_zero(v->l2_lock, v->nlines);
    }
    c->regs[R(A_SYNC_CTRL)] = (reg(c, A_SYNC_CTRL) & ~SYNC_OPS) | SYNC_DONE;
    c->counters_logged = false;
}

static void cache_reset_exit(Object *obj, ResetType type)
{
    ESP32P4CacheState *c = ESP32P4_CACHE(obj);

    c->l2_cache_size = UINT32_MAX;
    cache_update_split(c);
    cache_flush_tlbs();
    cache_update_irq(c);
}

static void cache_view_init(ESP32P4CacheState *c, int kind, Error **errp)
{
    ESP32P4CacheView *v = &c->view[kind];
    Object *obj = OBJECT(c);
    const char *n = esp32p4_cache_view_name[kind];
    g_autofree char *iommu = g_strdup_printf("esp32p4.cache-%s", n);
    g_autofree char *target = g_strdup_printf("esp32p4.cache-%s-target", n);
    g_autofree char *l1 = g_strdup_printf("esp32p4.cache-%s-l1", n);
    g_autofree char *clean = g_strdup_printf("esp32p4.cache-%s-l1-clean", n);
    g_autofree char *l2 = g_strdup_printf("esp32p4.cache-%s-l2", n);
    g_autofree char *miss = g_strdup_printf("esp32p4.cache-%s-miss", n);

    v->cache = c;
    v->kind = kind;
    v->size = kind == ESP32P4_CACHE_VIEW_L2MEM ? L2MEM_SIZE
                                               : ESP32P4_MSPI_WINDOW_SIZE;
    v->nlines = v->size / ESP32P4_CACHE_LINE;

    memory_region_init_iommu(&v->iommu, sizeof(v->iommu),
                             TYPE_ESP32P4_CACHE_VIEW, obj, iommu, v->size);
    memory_region_init(&v->target, obj, target, 2 * v->size);
    if (!memory_region_init_ram(&v->l1, obj, l1, v->size, errp) ||
        !memory_region_init_ram(&v->l1_clean, obj, clean, v->size, errp)) {
        return;
    }
    if (kind != ESP32P4_CACHE_VIEW_L2MEM &&
        !memory_region_init_ram(&v->l2, obj, l2, v->size, errp)) {
        return;
    }
    memory_region_init_io(&v->miss, obj, &cache_miss_ops, v, miss, v->size);
    memory_region_add_subregion(&v->target, 0, &v->l1);
    memory_region_add_subregion(&v->target, v->size, &v->miss);
    address_space_init(&v->target_as, &v->target, target);

    v->l1_valid = bitmap_new(v->nlines);
    v->l1_lock = bitmap_new(v->nlines);
    v->l2_valid = bitmap_new(v->nlines);
    v->l2_dirty = bitmap_new(v->nlines);
    v->l2_lock = bitmap_new(v->nlines);
}

static void cache_realize(DeviceState *dev, Error **errp)
{
    ESP32P4CacheState *c = ESP32P4_CACHE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    if (!c->flash_mspi || !c->psram_mspi || !c->l2mem_mr) {
        error_setg(errp, "%s: the 'flash-mspi', 'psram-mspi' and 'l2mem' "
                   "links must be set", TYPE_ESP32P4_CACHE);
        return;
    }
    if (memory_region_size(c->l2mem_mr) != L2MEM_SIZE ||
        !memory_region_is_ram(c->l2mem_mr)) {
        error_setg(errp, "%s: 'l2mem' must be the 768 KiB of L2MEM",
                   TYPE_ESP32P4_CACHE);
        return;
    }
    if (qemu_target_page_size() > FILL) {
        error_setg(errp, "%s: the target's pages are larger than the "
                   "caches' fills", TYPE_ESP32P4_CACHE);
        return;
    }

    memory_region_init_io(&c->iomem, OBJECT(c), &cache_ops, c,
                          "esp32p4.cache", ESP32P4_CACHE_REGS_SIZE);
    memory_region_init_io(&c->l2mem_cached, OBJECT(c),
                          &cache_l2mem_overlay_ops, c,
                          "esp32p4.l2mem-l2-cache", 128 * KiB);
    memory_region_init_io(&c->l2mem_direct, OBJECT(c),
                          &cache_l2mem_overlay_ops, c,
                          "esp32p4.l2mem-direct-l2-cache", 128 * KiB);
    sysbus_init_mmio(sbd, &c->iomem);
    sysbus_init_mmio(sbd, &c->l2mem_cached);
    sysbus_init_mmio(sbd, &c->l2mem_direct);
    for (i = 0; i < ESP32P4_CACHE_NVIEWS; i++) {
        cache_view_init(c, i, errp);
        if (*errp) {
            return;
        }
        sysbus_init_mmio(sbd, MEMORY_REGION(&c->view[i].iommu));
    }
    c->l2_cache_size = 128 * KiB;
}

static void cache_init(Object *obj)
{
    ESP32P4CacheState *c = ESP32P4_CACHE(obj);
    DeviceState *dev = DEVICE(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &c->irq);
    qdev_init_gpio_in_named(dev, cache_set_cache_reset,
                            ESP32P4_CACHE_CACHE_RESET, ESP32P4_CACHE_NRESETS);
    qdev_init_gpio_in_named(dev, cache_set_sys_cache_reset,
                            ESP32P4_CACHE_SYS_CACHE_RESET,
                            ESP32P4_CACHE_NRESETS);
    qdev_init_gpio_in_named(dev, cache_set_l2mem_err_resp,
                            ESP32P4_CACHE_L2MEM_ERR_RESP, 1);
}

static int cache_post_load(void *opaque, int version_id)
{
    ESP32P4CacheState *c = opaque;
    uint32_t size = c->l2_cache_size;

    c->l2_cache_size = UINT32_MAX;
    cache_update_split(c);
    return size == c->l2_cache_size ? 0 : -EINVAL;
}

static const VMStateDescription vmstate_esp32p4_cache_view = {
    .name = TYPE_ESP32P4_CACHE_VIEW,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BITMAP(l1_valid, ESP32P4CacheView, 1, nlines),
        VMSTATE_BITMAP(l1_lock, ESP32P4CacheView, 1, nlines),
        VMSTATE_BITMAP(l2_valid, ESP32P4CacheView, 1, nlines),
        VMSTATE_BITMAP(l2_dirty, ESP32P4CacheView, 1, nlines),
        VMSTATE_BITMAP(l2_lock, ESP32P4CacheView, 1, nlines),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_cache = {
    .name = TYPE_ESP32P4_CACHE,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = cache_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ESP32P4CacheState, ESP32P4_CACHE_NREGS),
        VMSTATE_UINT32(l2_cache_size, ESP32P4CacheState),
        VMSTATE_BOOL_ARRAY(cache_reset, ESP32P4CacheState,
                           ESP32P4_CACHE_NRESETS),
        VMSTATE_BOOL_ARRAY(sys_cache_reset, ESP32P4CacheState,
                           ESP32P4_CACHE_NRESETS),
        VMSTATE_BOOL(l2mem_err_resp, ESP32P4CacheState),
        VMSTATE_BOOL(counters_logged, ESP32P4CacheState),
        VMSTATE_STRUCT_ARRAY(view, ESP32P4CacheState, ESP32P4_CACHE_NVIEWS,
                             1, vmstate_esp32p4_cache_view,
                             ESP32P4CacheView),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_cache_properties[] = {
    DEFINE_PROP_LINK("flash-mspi", ESP32P4CacheState, flash_mspi,
                     TYPE_ESP32P4_MSPI, ESP32P4MSPIState *),
    DEFINE_PROP_LINK("psram-mspi", ESP32P4CacheState, psram_mspi,
                     TYPE_ESP32P4_MSPI, ESP32P4MSPIState *),
    DEFINE_PROP_LINK("l2mem", ESP32P4CacheState, l2mem_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("hp-sys", ESP32P4CacheState, hp_sys,
                     TYPE_ESP32P4_HP_SYS, ESP32P4HPSysState *),
};

static void esp32p4_cache_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = cache_reset_hold;
    rc->phases.exit = cache_reset_exit;
    dc->realize = cache_realize;
    dc->vmsd = &vmstate_esp32p4_cache;
    device_class_set_props(dc, esp32p4_cache_properties);
}

/* Every core's accesses take the same translation. */
static int cache_view_attrs_to_index(IOMMUMemoryRegion *iommu,
                                     MemTxAttrs attrs)
{
    return 0;
}

static int cache_view_num_indexes(IOMMUMemoryRegion *iommu)
{
    return 1;
}

static void esp32p4_cache_view_class_init(ObjectClass *klass,
                                          const void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->translate = cache_view_translate;
    imrc->attrs_to_index = cache_view_attrs_to_index;
    imrc->num_indexes = cache_view_num_indexes;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static const TypeInfo esp32p4_cache_types[] = {
    {
        .name          = TYPE_ESP32P4_CACHE,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32P4CacheState),
        .instance_init = cache_init,
        .class_init    = esp32p4_cache_class_init,
    },
    {
        .name          = TYPE_ESP32P4_CACHE_VIEW,
        .parent        = TYPE_IOMMU_MEMORY_REGION,
        .class_init    = esp32p4_cache_view_class_init,
    },
};

DEFINE_TYPES(esp32p4_cache_types)
