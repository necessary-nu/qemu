/*
 * ESP32-P4 cache: the HP cores' L1 caches, the L2 cache and the L2MEM split
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), 9.3.2
 * (Internal Memory: the HP L2MEM configurations), 9.3.3.2 (Cache, figure
 * 9.3-3) and 9.3.3.3 (Cache Operations); ESP-IDF's cache_reg.h for v3,
 * which gives the controller's registers, access and reset values (the
 * TRM does not document them); its cache_ll.h, esp_cache_msync.c, the
 * esp_mm component, and the v3 ROM's Cache_* routines, which show how
 * software drives them; and ESP-IDF's linker scripts for v3, which place
 * the L2 cache at L2MEM's start.
 *
 * The HP cores reach the external memory through two levels of cache, and
 * HP L2MEM through the first (TRM figure 9.3-3): each core's instruction
 * fetches go through its own L1 instruction cache, both cores' loads and
 * stores through the L1 data cache they share, and those caches' fills
 * and write-backs to the L2 cache for the external memory, or to L2MEM.
 * The other masters (DMA, the LP core) reach both memories directly. The
 * caches are virtually tagged: the MMU (in the MSPI controllers,
 * esp32p4_mspi.c) translates only what the L2 cache fetches and writes
 * back.
 *
 * How the model keeps the caches' contents: software observes a write-back
 * cache only where its contents and the memory's differ, so the model
 * keeps the contents of each line in each cache, with its state, and never
 * evicts a line: a cache as large as the windows it caches. A line leaves
 * a cache only when an operation or a reset invalidates it, or is written
 * back only when an operation writes it back. Software that relies on a
 * line having been evicted, as real caches of 16 KiB (L1 instruction), 64
 * KiB (L1 data) and up to 512 KiB (L2) eventually do, sees what it would
 * with a cache that kept it; software that relies on a line not being in
 * the cache sees it stale. Each is a defect on the hardware too. For each
 * 64-byte line:
 *
 *   - in each L1 cache, its contents and whether it is valid and locked,
 *     and in the data cache whether it is dirty. A store marks its line
 *     dirty whatever it writes, the value the line holds too, and a
 *     write-back writes the whole line, as on the chip: over whatever the
 *     next level was given behind the cache meanwhile, by DMA, by a core
 *     through an uncached address or by the LP core.
 *   - for the external memory, the L2 contents, and whether the line is
 *     valid, dirty and locked there. With the L2's 128-byte lines, the
 *     two 64-byte lines of each share that state: the L2 holds, dirties,
 *     cleans, writes back and drops the 128 bytes as one.
 *
 * The caches are coherent only as the chip's are. An L1 cache fills a line
 * from the L2 where it holds the line, otherwise from the memory through
 * the L2, which then holds it too, and L2MEM's lines from L2MEM; the data
 * cache writes its dirty lines back to the L2, or to L2MEM, or with the L2
 * bypassed to the memory; and nothing passes between the L1 caches. So an
 * instruction cache keeps what it filled, whatever is stored there since,
 * until it is invalidated; and it fills a line that the data cache holds
 * dirty with the line as the L2 or L2MEM holds it, without the stores.
 * Code stored through the data cache runs once the data cache has written
 * it back (to the L2 is enough for the external memory) and the
 * instruction caches holding the line before have been invalidated, as
 * ESP-IDF's esp_cache_msync() has it: a write-back of the data cache, then
 * an invalidation of the caches. FENCE.I acts on no cache: the TRM gives
 * it no part in them.
 *
 * A core's access to a line its L1 cache does not hold fills that cache,
 * 4 KiB at a time: the 64 lines that share the line's page, which is what
 * lets every access to a filled page run at full speed. The fill takes the
 * page's other lines as an L2 autoload of the neighbouring lines would.
 * Each core's accesses go through a translation per page (an IOMMU region
 * in its address space), its instruction fetches apart from its loads and
 * stores: the core's TLB fill keeps their translations apart
 * (ESP32P4_MMU_IDX_IFETCH) and marks a fetch's accesses
 * (ESP32P4_REQUESTER_IBUS). A fetch goes to the contents of the core's
 * instruction cache, from which QEMU translates the code, and a load or
 * store to the data cache's, while that cache holds all of the page;
 * otherwise each goes to the controller. The data cache's contents are RAM
 * to the cores, which watches their stores (memory_region_set_store_watch()):
 * a store to a page with a clean line passes the model on the TLB's slow
 * path, which marks the line dirty before the store is made, and once
 * every line of the page is dirty the page's stores go straight to the
 * RAM, until an operation cleans or drops one of its lines. An atomic
 * memory operation is made there as atomically as anywhere.
 *
 * An operation, or a change to the bypass, the buses or the L2MEM split,
 * takes effect for every core at its instant. Each core's translations
 * change on its own thread, before it runs its next translation block:
 * under parallel TCG, where the other cores run on threads of their own,
 * the register write that starts it is made again with the other cores
 * stopped between translation blocks (cpu_hold_access_exclusive()), and
 * none of them runs again before it has translated anew what the write
 * changed; the core that made the write ends its block with it, as with
 * any write to I/O, and does the same. Under round-robin TCG (icount) the
 * other cores do not run meanwhile. The cache resets, which other blocks'
 * registers drive, act with the cores stopped likewise.
 *
 * An access through those translations that no core makes, qtest's, goes
 * to the bus past the caches, as every other master's does. A debugger's
 * read takes the data cache's copy where it holds the line, and otherwise
 * the L2's or the memory's, filling nothing; its write goes to every copy:
 * each L1 cache's that holds the line, the data cache's then dirty, and
 * where the data cache does not hold the line the L2's or the memory's.
 * The HP ROM is read through the L1 caches on the chip and read directly
 * here, as no line of it can differ.
 *
 * Operations (CACHE_SYNC_*, CACHE_LOCK_*, *_PRELOAD_*): each completes at
 * once, as the TRM gives no timing, setting its done bit and raising its
 * done interrupt. SYNC_MAP picks the caches, as ESP-IDF's CACHE_MAP_*
 * values name them, and each acts on its own lines alone: L1-ICache0 and 1
 * (bits 0 and 1; 2 and 3 are reserved), the L1-DCache (bit 4), the L2
 * (bit 5). A SYNC_SIZE of zero operates on the whole cache, as the ROM's
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
 * dirty lines to the next level, the data cache's to the L2 (or to L2MEM,
 * or with the L2 bypassed to the memory) and the L2's to the memory, L1
 * before L2; write-back-invalidate does both. The instruction caches,
 * whose lines are never dirty, take write-back and clean as nothing to do
 * (the TRM gives those for the data cache and the L2 only) and
 * write-back-invalidate as an invalidation. A data cache write-back into
 * a line the L2 does not hold allocates the line there, reading the rest
 * of a 128-byte line from the memory as a miss would. Operating on an
 * address range leaves locked lines; on the whole cache, it does not. A
 * SYNC_MAP of zero, or more than one operation at once, completes with
 * SYNC_ERR and SYNC_ERR_CODE 0 or 1. Lock and unlock (LOCK_CTRL, with
 * LOCK_MAP, LOCK_ADDR and LOCK_SIZE) mark the lines each cache LOCK_MAP
 * names holds in the range; each cache's prelock sections (*_PRELOCK_*)
 * lock the lines it fills inside them. Each preload (*_PRELOAD_*) fills
 * the range's lines into its own cache. Freeze sets its done bit at once,
 * there being no access in flight; autoload's settings are held, its done
 * bit always set, as fills already take whole pages.
 *
 * Access errors: where the next level answers a fill with an error (the
 * MSPI's port closed, or an invalid MMU entry while it answers those with
 * errors), the L2 records the access in L2_CACHE_ACS_FAIL_ADDR and raises
 * its FAIL interrupt, and with L2_CACHE_ACS_FAIL_CHECK_MODE set passes the
 * error to the L1 cache the access is for, which records it in its
 * registers (the L1-ICache<n>'s for core n's instruction fetches, the
 * L1-DCache's for loads and stores) and raises its FAIL interrupt, and
 * with its check mode set passes it to the core, which takes an access
 * fault. An error not passed on reads as zero, fills nothing and drops the
 * write. The FAIL_ID and FAIL_ATTR fields stay zero: their encoding is not
 * published.
 *
 * Bypass and shut-down: L1_BYPASS_CACHE_CONF's ICache<n> bit sends core
 * n's instruction fetches past its instruction cache, and its DCache bit
 * both cores' loads and stores past the data cache, to the L2 or L2MEM;
 * with the L2 bypassed (L2_BYPASS_CACHE_CONF), the L1 caches fill from and
 * write back to the memory directly. L1_ICACHE_CTRL_SHUT_IBUS<n> closes
 * core n's instruction bus and L1_DCACHE_CTRL_SHUT_DBUS<n> its data bus:
 * an access on a closed bus fails as above, in the L1-ICache<n>'s
 * registers or the L1-DCache's.
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
 * L1 caches ("sys-cache-reset"), each empty their own cache.
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
#include "qemu/atomic.h"
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
#include "hw/misc/esp32p4_pms.h"
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

#define L1_I0           ESP32P4_CACHE_L1_I0
#define L1_I1           ESP32P4_CACHE_L1_I1
#define L1_D            ESP32P4_CACHE_L1_D
#define NL1             ESP32P4_CACHE_NL1

/* What a translation reaches, in multiples of the view's size. */
enum {
    TARGET_L1D,
    TARGET_L1I0,
    TARGET_L1I1,
    TARGET_MISS,
    NTARGETS,
};

/* Each L1 cache's bit in SYNC_MAP and LOCK_MAP, and in the bypass. */
static const uint32_t l1_map[NL1] = {
    [L1_I0] = MAP_L1_I0,
    [L1_I1] = MAP_L1_I1,
    [L1_D] = MAP_L1_D,
};

static const uint32_t l1_bypass[NL1] = {
    [L1_I0] = BYPASS_L1_ICACHE0,
    [L1_I1] = BYPASS_L1_ICACHE1,
    [L1_D] = BYPASS_L1_DCACHE,
};

static const hwaddr l1_prelock[NL1] = {
    [L1_I0] = A_L1_ICACHE0_PRELOCK_CONF,
    [L1_I1] = A_L1_ICACHE1_PRELOCK_CONF,
    [L1_D] = A_L1_DCACHE_PRELOCK_CONF,
};

static const char *const l1_name[NL1] = {
    [L1_I0] = "l1i0",
    [L1_I1] = "l1i1",
    [L1_D] = "l1d",
};

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
 * once a cache holds all of it they go straight to its contents.
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

/*
 * The core making the access in progress, on its thread, if it runs in
 * parallel with the others: under TCG with each core on a thread of its
 * own, outside an exclusive section. NULL otherwise: no other core runs
 * until this one leaves its translation block (round-robin TCG, icount),
 * or the access is not a core's.
 */
static CPUState *cache_parallel_core(void)
{
    CPUState *cs = current_cpu;

    if (!tcg_enabled() || !cs || !qemu_cpu_is_self(cs) || !cs->running ||
        !tcg_cflags_has(cs, CF_PARALLEL) || cpu_in_exclusive_context(cs)) {
        return NULL;
    }
    return cs;
}

/*
 * Whether `attrs` are those of an HP core's instruction fetch
 * (esp32p4_hp_attrs()), and if so whose.
 */
static bool cache_fetch(MemTxAttrs attrs, int *core)
{
    uint16_t rid = attrs.requester_id;
    uint16_t kind = ESP32P4_REQUESTER_VALID | ESP32P4_REQUESTER_IS_DMA |
                    ESP32P4_REQUESTER_IBUS;

    if (attrs.unspecified ||
        (rid & kind) != (ESP32P4_REQUESTER_VALID | ESP32P4_REQUESTER_IBUS)) {
        return false;
    }
    *core = ESP32P4_REQUESTER_ID(rid) == ESP32P4_MASTER_HP_CORE1;
    return true;
}

/*
 * The L1 cache a core's access goes through, the core's instruction cache
 * for a fetch and the data cache for a load or a store, and the core.
 */
static int cache_access_l1(MemTxAttrs attrs, bool write, int *core)
{
    if (!write && cache_fetch(attrs, core)) {
        return L1_I0 + *core;
    }
    *core = cache_core();
    return L1_D;
}

static bool cache_l1_bypassed(ESP32P4CacheState *c, int l1)
{
    return reg(c, A_L1_BYPASS_CACHE_CONF) & l1_bypass[l1];
}

/*
 * Whether core `core`'s bus to L1 cache `l1` is shut: its instruction bus
 * to its instruction cache, or its data bus to the data cache.
 */
static bool cache_bus_shut(ESP32P4CacheState *c, int l1, int core)
{
    return reg(c, l1 == L1_D ? A_L1_DCACHE_CTRL : A_L1_ICACHE_CTRL) &
           (core ? SHUT_BUS1 : SHUT_BUS0);
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

static uint8_t *view_l1(ESP32P4CacheView *v, int l1)
{
    return v->l1_host[l1];
}

static uint8_t *view_l2(ESP32P4CacheView *v)
{
    return v->l2_host;
}

static uint32_t view_vaddr(ESP32P4CacheView *v, uint32_t off)
{
    return esp32p4_cache_view_base[v->kind] + off;
}

/* Whether L1 cache `l1` holds every line of the page from line `first`. */
static bool l1_holds_page(ESP32P4CacheView *v, int l1, uint32_t first)
{
    return find_next_zero_bit(v->l1c[l1].valid, first + LINES_PER_FILL,
                              first) >= first + LINES_PER_FILL;
}

/* Whether every line of the page from line `first` is dirty there. */
static bool l1d_page_dirty(ESP32P4CacheView *v, uint32_t first)
{
    return find_next_zero_bit(v->l1d_dirty, first + LINES_PER_FILL,
                              first) >= first + LINES_PER_FILL;
}

/*
 * What core `core`'s instruction fetches from the page at `off` reach: the
 * contents of its instruction cache while the fetches go through it and it
 * holds the whole page, and otherwise the controller.
 */
static int view_fetch_target(ESP32P4CacheView *v, uint32_t off, int core)
{
    ESP32P4CacheState *c = v->cache;
    int l1 = L1_I0 + core;

    if (cache_l1_bypassed(c, l1) || cache_bus_shut(c, l1, core) ||
        view_occupied(v, off) ||
        !l1_holds_page(v, l1, off / ESP32P4_CACHE_LINE)) {
        return TARGET_MISS;
    }
    /* The lines' contents after their valid bits: l1_fill_line(). */
    smp_rmb();
    return TARGET_L1I0 + core;
}

/*
 * Record that a core may be given a translation that stores to the page
 * at `off` of the data cache's contents directly, as every line of it is
 * dirty, so that an operation that cleans or drops one of its lines has
 * the cores translate it anew (l1d_undirect()). The operations that do
 * run with no core translating meanwhile (cpu_hold_access_exclusive()),
 * and the cores' stores only ever dirty lines.
 */
static bool l1d_page_direct(ESP32P4CacheView *v, uint32_t off)
{
    if (!l1d_page_dirty(v, off / ESP32P4_CACHE_LINE)) {
        return false;
    }
    set_bit_atomic(off / FILL, v->l1d_direct);
    return true;
}

/*
 * What core `core`'s loads and stores to the page at `off` reach: while
 * they go through the data cache and it holds the whole page, its
 * contents, a store passing l1d_watch_store() first unless every line of
 * the page is dirty already (l1d_watch_page()); and otherwise the
 * controller.
 */
static int view_data_target(ESP32P4CacheView *v, uint32_t off, int core)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t first = off / ESP32P4_CACHE_LINE;

    if (cache_l1_bypassed(c, L1_D) || cache_bus_shut(c, L1_D, core) ||
        view_occupied(v, off) || !l1_holds_page(v, L1_D, first)) {
        return TARGET_MISS;
    }
    /* The lines' contents after their valid bits: l1_fill_line(). */
    smp_rmb();
    l1d_page_direct(v, off);
    return TARGET_L1D;
}

/*
 * The translation of a core's access: an instruction fetch by its own
 * IOMMU index (cache_view_attrs_to_index()), a load or store by the core's
 * thread.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static IOMMUTLBEntry cache_view_translate(IOMMUMemoryRegion *iommu,
                                          hwaddr addr,
                                          IOMMUAccessFlags flags,
                                          int iommu_idx)
{
    ESP32P4CacheView *v = container_of(iommu, ESP32P4CacheView, iommu);
    hwaddr page = addr & ~(hwaddr)(FILL - 1);
    int target = TARGET_MISS;
    IOMMUTLBEntry ret = {
        .target_as = &v->target_as,
        .iova = page,
        .addr_mask = FILL - 1,
        .perm = IOMMU_RW,
    };

    if (current_cpu) {
        target = iommu_idx ? view_fetch_target(v, page, iommu_idx - 1)
                           : view_data_target(v, page, cache_core());
    }
    ret.translated_addr = target * v->size + page;
    return ret;
}

/* The L2MEM split */

/*
 * The part of L2MEM the L2 cache occupies, with L2_CACHE_CACHESIZE_CONF
 * `conf` and L2_BYPASS_CACHE_CONF `bypass`.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static uint32_t cache_l2_size_of(uint32_t conf, uint32_t bypass)
{
    if (bypass & BYPASS_L2_CACHE) {
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

static uint32_t cache_l2_size(ESP32P4CacheState *c)
{
    return cache_l2_size_of(reg(c, A_L2_CACHESIZE_CONF),
                            reg(c, A_L2_BYPASS_CACHE_CONF));
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

/*
 * Note a write of the model's to L1 cache `l1`'s contents for those that
 * track writes to memory, a migration and a display. QEMU's own tracking
 * of the code it translated has no part here: no code is translated from
 * the data cache's contents, and an instruction cache's line has none
 * translated from it while it is invalid, which is when the model fills
 * it (l1_untranslate()).
 */
static void l1_written(ESP32P4CacheView *v, int l1, uint32_t off,
                       unsigned len)
{
    MemoryRegion *mr = &v->l1[l1];

    if (memory_region_get_dirty_log_mask(mr) & ~(1 << DIRTY_MEMORY_CODE)) {
        memory_region_set_dirty(mr, off, len);
    }
}

/*
 * Put `data` into line `line` of L1 cache `l1`, as filled: valid, and in
 * the data cache clean. A fill can be made while a core translates the
 * instruction it fetches, so it may not invalidate translations itself:
 * an instruction cache line's were invalidated when the line left the
 * cache (l1_untranslate()), and none are made from a page the cache does
 * not hold all of. The contents are in place before the line is valid,
 * for the cores on other threads, which translate a page to them once
 * every line of it is (view_fetch_target(), view_data_target()).
 */
static void l1_fill_line(ESP32P4CacheView *v, int l1, uint32_t line,
                         const uint8_t *data)
{
    uint32_t off = line * ESP32P4_CACHE_LINE;

    memcpy(view_l1(v, l1) + off, data, ESP32P4_CACHE_LINE);
    l1_written(v, l1, off, ESP32P4_CACHE_LINE);
    /* The contents before the valid bit: view_*_target() read them so. */
    smp_wmb();
    set_bit(line, v->l1c[l1].valid);
    if (l1 == L1_D) {
        clear_bit(line, v->l1d_dirty);
    }
}

/*
 * Drop what the cores translated from lines [first, end) of L1 cache `l1`:
 * QEMU translates code from the instruction caches' contents alone.
 */
static void l1_untranslate(ESP32P4CacheView *v, int l1, uint32_t first,
                           uint32_t end)
{
    ram_addr_t ra = memory_region_get_ram_addr(&v->l1[l1]);

    if (tcg_enabled() && l1 != L1_D && first < end) {
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

/*
 * Make the cores translate the page holding data cache line `line` anew if
 * one of them may store to it directly, as a line of it is no longer
 * dirty: see l1d_page_direct().
 */
static void l1d_undirect(ESP32P4CacheView *v, uint32_t line)
{
    uint32_t page = line / LINES_PER_FILL;

    if (test_bit(page, v->l1d_direct)) {
        clear_bit_atomic(page, v->l1d_direct);
        cache_flush_tlb_page(view_vaddr(v, page * FILL));
    }
}

static void l1d_clean_line(ESP32P4CacheView *v, uint32_t line)
{
    if (test_bit(line, v->l1d_dirty)) {
        clear_bit(line, v->l1d_dirty);
        l1d_undirect(v, line);
    }
}

/* Drop line `line` from L1 cache `l1`, and its dirty data with it. */
static void l1_drop_line(ESP32P4CacheView *v, int l1, uint32_t line)
{
    clear_bit(line, v->l1c[l1].valid);
    clear_bit(line, v->l1c[l1].lock);
    if (l1 == L1_D) {
        l1d_clean_line(v, line);
    }
}

/*
 * Mark data cache lines [line, end), of one page, dirty, as a store to
 * them does, whatever it writes. Once every line of the page is, the
 * cores translate the page anew, to store to it directly.
 */
static void l1d_dirty_lines(ESP32P4CacheView *v, uint32_t line, uint32_t end)
{
    uint32_t first = QEMU_ALIGN_DOWN(line, LINES_PER_FILL);
    bool marked = false;
    uint32_t l;

    for (l = line; l < end; l++) {
        if (!test_bit(l, v->l1d_dirty)) {
            set_bit_atomic(l, v->l1d_dirty);
            marked = true;
        }
    }
    if (marked && l1d_page_dirty(v, first)) {
        cache_flush_tlb_page(view_vaddr(v, first * ESP32P4_CACHE_LINE));
    }
}

/*
 * A core's store into the data cache through the controller, which holds
 * the line (the memory system makes it within a line): the line is dirty.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void l1d_store(ESP32P4CacheView *v, uint32_t off, const uint8_t *buf,
                      unsigned size)
{
    uint32_t line = off / ESP32P4_CACHE_LINE;

    memcpy(view_l1(v, L1_D) + off, buf, size);
    l1_written(v, L1_D, off, size);
    l1d_dirty_lines(v, line, line + 1);
}

/*
 * Whether a core's stores to the page at `off` of the data cache's
 * contents, which it is given as the cache holds all of the page, pass
 * l1d_watch_store(): while a line of the page is clean.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static bool l1d_watch_page(void *opaque, hwaddr off)
{
    return !l1d_page_direct(opaque, off);
}

/*
 * A core's store to the data cache's contents, of a page with a clean
 * line, about to be made: the lines it reaches are dirty, whatever it
 * writes. The store that dirties a page's last clean line is made as an
 * access to the controller would be, the last instruction of its block
 * (`last`), so that the cores translating the page anew follows it as it
 * would that access.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static bool l1d_watch_store(void *opaque, hwaddr off, unsigned size,
                            bool last)
{
    ESP32P4CacheView *v = opaque;
    uint32_t line = off / ESP32P4_CACHE_LINE;
    uint32_t end = DIV_ROUND_UP(off + size, ESP32P4_CACHE_LINE);
    uint32_t first = QEMU_ALIGN_DOWN(line, LINES_PER_FILL);
    uint32_t page_end = first + LINES_PER_FILL;

    if (end == line + 1 ? test_bit(line, v->l1d_dirty)
                        : find_next_zero_bit(v->l1d_dirty, end, line) >= end) {
        return true;
    }
    if (!last && find_next_zero_bit(v->l1d_dirty, line, first) >= line &&
        find_next_zero_bit(v->l1d_dirty, page_end, end) >= page_end) {
        return false;
    }
    l1d_dirty_lines(v, line, end);
    return true;
}

static const MemoryRegionStoreWatchOps l1d_watch_ops = {
    .page_watched = l1d_watch_page,
    .store = l1d_watch_store,
};

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
 * Read the fill block at `off` into `buf`, the L2's copy of each line it
 * holds and the memory's of the others, and fill those into the L2 when
 * `l2_alloc` says. Returns the memory's answer.
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
 * Fill the lines of the fill block at `start` that L1 cache `l1` does not
 * hold, from the L2 or the memory through it, or from L2MEM: never from
 * another L1 cache. Returns the next level's answer; on an error nothing
 * is filled.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult l1_fill_missing(ESP32P4CacheView *v, int l1,
                                   uint32_t start)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t first = start / ESP32P4_CACHE_LINE;
    uint8_t buf[FILL];
    uint32_t i;

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
        memcpy(buf, c->l2mem_host + start, FILL);
    }
    for (i = 0; i < LINES_PER_FILL; i++) {
        uint32_t line = first + i;

        if (test_bit(line, v->l1c[l1].valid)) {
            continue;
        }
        l1_fill_line(v, l1, line, buf + i * ESP32P4_CACHE_LINE);
        if (cache_prelocked(c, l1_prelock[l1], false,
                            view_vaddr(v, line * ESP32P4_CACHE_LINE))) {
            set_bit(line, v->l1c[l1].lock);
        }
    }
    cache_flush_tlb_page(view_vaddr(v, start));
    return MEMTX_OK;
}

/* Fill the fill block holding `off` into L1 cache `l1`, as it needs. */
static MemTxResult l1_fill_block(ESP32P4CacheView *v, int l1, uint32_t off)
{
    uint32_t start = QEMU_ALIGN_DOWN(off, FILL);

    if (l1_holds_page(v, l1, start / ESP32P4_CACHE_LINE)) {
        return MEMTX_OK;
    }
    return l1_fill_missing(v, l1, start);
}

/*
 * Store into L2MEM at `off`, as a write on the bus would: the cores'
 * translations of code there are dropped.
 */
static void l2mem_store(ESP32P4CacheState *c, uint32_t off,
                        const uint8_t *data, unsigned len)
{
    ram_addr_t ra = memory_region_get_ram_addr(c->l2mem_mr);

    memcpy(c->l2mem_host + off, data, len);
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
 * Allocate the L2 line holding unit `line` for a data cache write-back
 * into it, which brings the line's other units from the memory where the
 * L2 does not hold them (only a 128-byte line has another), filled as a
 * miss fills them. Returns the memory's answer; on an error nothing is
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

/*
 * Write data cache line `line` back to the next level if it is dirty: the
 * whole line, whatever its stores wrote.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void l1_writeback_line(ESP32P4CacheView *v, uint32_t line)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t off = line * ESP32P4_CACHE_LINE;
    uint8_t *data = view_l1(v, L1_D) + off;

    if (!test_bit(line, v->l1c[L1_D].valid) ||
        !test_bit(line, v->l1d_dirty)) {
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
    l1d_clean_line(v, line);
}

/* The cores' accesses that do not go straight to an L1 cache's contents */

/*
 * An access past the L1 caches (the L1 cache bypassed, or a debugger's):
 * to the L2's copy, or the memory. A debugger's access allocates nothing.
 */
static MemTxResult cache_past_l1(ESP32P4CacheView *v, uint32_t off,
                                 uint8_t *buf, unsigned size, bool write,
                                 bool debug)
{
    ESP32P4CacheState *c = v->cache;
    uint32_t line = off / ESP32P4_CACHE_LINE;
    uint8_t block[FILL];
    uint32_t start = QEMU_ALIGN_DOWN(off, FILL);
    MemTxResult r;

    if (!view_ext(v)) {
        if (write) {
            l2mem_store(c, off, buf, size);
        } else {
            memcpy(buf, c->l2mem_host + off, size);
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

/*
 * A debugger's access: a read takes the data cache's copy where it holds
 * the line, and otherwise the L2's or the memory's; a write goes to each
 * L1 cache's copy, the data cache's then dirty, and where the data cache
 * does not hold the line to the L2's or the memory's.
 */
static MemTxResult cache_debug_access(ESP32P4CacheView *v, uint32_t off,
                                      uint8_t *buf, unsigned size,
                                      bool write)
{
    MemTxResult r = MEMTX_OK;

    while (size) {
        uint32_t line = off / ESP32P4_CACHE_LINE;
        unsigned n = MIN(size, ESP32P4_CACHE_LINE - off % ESP32P4_CACHE_LINE);
        bool in_d = test_bit(line, v->l1c[L1_D].valid);
        int l1;

        if (!write) {
            if (in_d) {
                memcpy(buf, view_l1(v, L1_D) + off, n);
            } else {
                r |= cache_past_l1(v, off, buf, n, false, true);
            }
        } else {
            for (l1 = 0; l1 < NL1; l1++) {
                if (test_bit(line, v->l1c[l1].valid)) {
                    ram_addr_t ra = memory_region_get_ram_addr(&v->l1[l1]);

                    memcpy(view_l1(v, l1) + off, buf, n);
                    l1_written(v, l1, off, n);
                    if (tcg_enabled() && l1 != L1_D) {
                        tb_invalidate_phys_range(NULL, ra + off,
                                                 ra + off + n - 1);
                    }
                }
            }
            if (in_d) {
                set_bit_atomic(line, v->l1d_dirty);
            } else {
                r |= cache_past_l1(v, off, buf, n, true, true);
            }
        }
        off += n;
        buf += n;
        size -= n;
    }
    return r;
}

/*
 * Whether a core's access at `off` through L1 cache `l1`, or past it,
 * reaches the external memory: through the controller's AXI port, which
 * may hold it (esp32p4_mspi_axi_ready()). A fill takes the fill block's
 * lines from the L2 where it holds all of them.
 */
static bool cache_needs_memory(ESP32P4CacheView *v, int l1, uint32_t off,
                               bool past_l1)
{
    uint32_t first = QEMU_ALIGN_DOWN(off, FILL) / ESP32P4_CACHE_LINE;

    if (!view_ext(v)) {
        return false;
    }
    if (past_l1) {
        return !test_bit(off / ESP32P4_CACHE_LINE, v->l2_valid);
    }
    if (l1_holds_page(v, l1, first)) {
        return false;
    }
    return find_next_zero_bit(v->l2_valid, first + LINES_PER_FILL, first) <
           first + LINES_PER_FILL;
}

/*
 * An access through a core's translations, by that core: through its
 * instruction cache for a fetch, the data cache for a load or store. The
 * memory system makes each access within a naturally aligned unit of its
 * size, so within a line.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult cache_view_access(ESP32P4CacheView *v, uint32_t off,
                                     uint64_t *data, unsigned size,
                                     bool write, MemTxAttrs attrs)
{
    ESP32P4CacheState *c = v->cache;
    uint8_t buf[8];
    int core, l1;
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
    if (attrs.debug) {
        r = cache_debug_access(v, off, buf, size, write);
        if (!write) {
            *data = r == MEMTX_OK ? ldn_le_p(buf, size) : 0;
        }
        return r;
    }
    l1 = cache_access_l1(attrs, write, &core);
    dbus = l1 == L1_D;
    if (cache_l1_bypassed(c, l1)) {
        if (cache_needs_memory(v, l1, off, true) &&
            !esp32p4_mspi_axi_ready(view_mspi(v), attrs, &r)) {
            if (!write) {
                *data = 0;
            }
            return r;
        }
        r = cache_past_l1(v, off, buf, size, write, false);
        if (r != MEMTX_OK) {
            memset(buf, 0, sizeof(buf));
            r = cache_access_fail(c, view_vaddr(v, off), true, core, dbus);
        }
        if (!write) {
            *data = ldn_le_p(buf, size);
        }
        return r;
    }
    if (cache_bus_shut(c, l1, core)) {
        if (!write) {
            *data = 0;
        }
        return cache_access_fail(c, view_vaddr(v, off), false, core, dbus);
    }
    if (cache_needs_memory(v, l1, off, false) &&
        !esp32p4_mspi_axi_ready(view_mspi(v), attrs, &r)) {
        if (!write) {
            *data = 0;
        }
        return r;
    }
    r = l1_fill_block(v, l1, off);
    if (r != MEMTX_OK) {
        if (!write) {
            *data = 0;
        }
        return cache_access_fail(c, view_vaddr(v, off), true, core, dbus);
    }
    if (write) {
        l1d_store(v, off, buf, size);
    } else {
        *data = ldn_le_p(view_l1(v, l1) + off, size);
    }
    return MEMTX_OK;
}

static MemTxResult cache_view_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    return cache_view_access(opaque, addr, data, size, false, attrs);
}

static MemTxResult cache_view_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size, MemTxAttrs attrs)
{
    return cache_view_access(opaque, addr, &data, size, true, attrs);
}

/* The controller's side of a view: the accesses no cache's contents serve. */
static const MemoryRegionOps cache_view_ops = {
    .read_with_attrs = cache_view_read,
    .write_with_attrs = cache_view_write,
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

/*
 * Operation `op` on the lines of view `v` of each cache `map` names, each
 * acting on its own lines. Returns whether an L1 cache dropped a line.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static bool cache_sync_view(ESP32P4CacheView *v, int op, uint32_t map,
                            uint32_t vaddr, uint32_t size)
{
    ESP32P4CacheState *c = v->cache;
    bool all = size == 0;
    bool inval = op == OP_INVALIDATE || op == OP_WB_INVALIDATE;
    bool wb = op == OP_WRITEBACK || op == OP_WB_INVALIDATE;
    bool dropped = false;
    uint32_t first, end, line;
    int l1;

    for (l1 = 0; l1 < NL1; l1++) {
        unsigned long *valid = v->l1c[l1].valid;
        uint32_t dropped_lo = UINT32_MAX, dropped_hi = 0;

        if (!(map & l1_map[l1]) ||
            !view_lines(v, vaddr, size, ESP32P4_CACHE_LINE, &first, &end)) {
            continue;
        }
        for (line = find_next_bit(valid, end, first); line < end;
             line = find_next_bit(valid, end, line + 1)) {
            bool locked = !all && test_bit(line, v->l1c[l1].lock);

            if (l1 == L1_D) {
                if (wb) {
                    l1_writeback_line(v, line);
                } else if (op == OP_CLEAN) {
                    l1d_clean_line(v, line);
                }
            }
            if (inval && !locked) {
                l1_drop_line(v, l1, line);
                dropped_lo = MIN(dropped_lo, line);
                dropped_hi = line + 1;
            }
        }
        l1_untranslate(v, l1, dropped_lo, dropped_hi);
        dropped |= dropped_lo < dropped_hi;
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
    return dropped;
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
    int i, l1;

    for (i = 0; size && i < ESP32P4_CACHE_NVIEWS; i++) {
        ESP32P4CacheView *v = &c->view[i];

        for (l1 = 0; l1 < NL1; l1++) {
            ESP32P4CacheL1 *l = &v->l1c[l1];

            if (!(map & l1_map[l1]) ||
                !view_lines(v, vaddr, size, ESP32P4_CACHE_LINE, &first,
                            &end)) {
                continue;
            }
            for (line = first; line < end; line++) {
                if (!lock) {
                    clear_bit(line, l->lock);
                } else if (test_bit(line, l->valid)) {
                    set_bit(line, l->lock);
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
 * filled into L1 cache `l1`, or with `l1` negative into the L2.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static void cache_preload(ESP32P4CacheState *c, hwaddr ctrl, int l1,
                          hwaddr int_group, uint32_t done_int)
{
    bool l2 = l1 < 0;
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
            r = l2 ? l2_fill_block(v, off, block, true)
                   : l1_fill_block(v, l1, off);
            if (r != MEMTX_OK) {
                cache_access_fail(c, view_vaddr(v, off), true, l1 == L1_I1,
                                  l1 != L1_I0 && l1 != L1_I1);
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

typedef struct ESP32P4CacheEmpty {
    ESP32P4CacheState *cache;
    uint32_t map;
} ESP32P4CacheEmpty;

static void cache_empty_exclusive(CPUState *cs, run_on_cpu_data data)
{
    ESP32P4CacheEmpty *e = data.host_ptr;

    cache_empty(e->cache, e->map);
    g_free(e);
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

/*
 * Whether writing `val` over `old` to the register at `a` changes what the
 * cores' translations reach: an operation on an L1 cache's lines, or a
 * change to the L1 caches' bypass or buses or to the L2MEM split.
 */
static bool cache_write_retranslates(ESP32P4CacheState *c, hwaddr a,
                                     uint32_t old, uint32_t val)
{
    switch (a) {
    case A_SYNC_CTRL:
        return (val & SYNC_OPS) &&
               (reg(c, A_SYNC_MAP) & (MAP_L1_I | MAP_L1_D));
    case A_L1_ICACHE_CTRL:
    case A_L1_DCACHE_CTRL:
    case A_L1_BYPASS_CACHE_CONF:
        return (val ^ old) & (SHUT_BUS0 | SHUT_BUS1 | BYPASS_L1_ICACHE0 |
                              BYPASS_L1_ICACHE1 | BYPASS_L1_DCACHE);
    case A_L2_BYPASS_CACHE_CONF:
        return cache_l2_size_of(reg(c, A_L2_CACHESIZE_CONF), val) !=
               c->l2_cache_size;
    case A_L2_CACHESIZE_CONF:
        return cache_l2_size_of(val, reg(c, A_L2_BYPASS_CACHE_CONF)) !=
               c->l2_cache_size;
    }
    return false;
}

/* [spec:nuos:req:emu.esp32p4.memory] */
static MemTxResult cache_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size, MemTxAttrs attrs)
{
    ESP32P4CacheState *c = opaque;
    hwaddr a = addr & ~3;
    const ESP32P4CacheReg *r = &esp32p4_cache_regs[R(a)];
    unsigned shift = 8 * (addr & 3);
    uint32_t v = val << shift;
    uint32_t mask = MAKE_64BIT_MASK(shift, 8 * size);
    uint32_t *p = &c->regs[R(a)];
    uint32_t old = *p;
    uint32_t trig, new;
    CPUState *cs;
    unsigned which;
    int g;

    if (!r->name) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of reserved offset 0x%03"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_CACHE, addr);
        return MEMTX_OK;
    }
    new = (old & ~(r->rw & mask)) | (v & r->rw & mask);
    new &= ~(v & r->w1c & mask);
    /*
     * A write changing what the cores' translations reach takes effect
     * for every core at once: a core running in parallel with the others
     * makes it again with them stopped.
     */
    cs = attrs.debug ? NULL : cache_parallel_core();
    if (cs && cache_write_retranslates(c, a, old, new)) {
        cpu_hold_access_exclusive(cs);
        return MEMTX_ERROR;
    }
    *p = new;
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
        if (*p & PRELOAD_ENA) {
            cache_preload(c, a, L1_I0, A_SYNC_L1_INT_ENA, INT_L1_I0_PLD_DONE);
        }
        break;
    case A_L1_ICACHE1_PRELOAD_CTRL:
        if (*p & PRELOAD_ENA) {
            cache_preload(c, a, L1_I1, A_SYNC_L1_INT_ENA, INT_L1_I1_PLD_DONE);
        }
        break;
    case A_L1_DCACHE_PRELOAD_CTRL:
        if (*p & PRELOAD_ENA) {
            cache_preload(c, a, L1_D, A_SYNC_L1_INT_ENA, INT_L1_D_PLD_DONE);
        }
        break;
    case A_L2_PRELOAD_CTRL:
        if (*p & PRELOAD_ENA) {
            cache_preload(c, a, -1, A_L2_SYNC_INT_ENA, INT_L2_PLD_DONE);
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
    return MEMTX_OK;
}

static const MemoryRegionOps cache_ops = {
    .read = cache_read,
    .write_with_attrs = cache_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* Reset, hooks, realize */

/*
 * Cache `n`'s reset, from HP_SYS_CLKRST (`line` cache_reset) or HP_SYSTEM
 * (sys_cache_reset): either empties the cache as it is asserted. A core
 * running in parallel with the others that asserts it with a register
 * write empties it with the others stopped, before its next instruction.
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
    CPUState *cs = cache_parallel_core();

    line[n] = level;
    if (was || !(c->cache_reset[n] || c->sys_cache_reset[n])) {
        return;
    }
    if (cs) {
        ESP32P4CacheEmpty *e = g_new(ESP32P4CacheEmpty, 1);

        e->cache = c;
        e->map = map[n];
        async_safe_run_on_cpu(cs, cache_empty_exclusive,
                              RUN_ON_CPU_HOST_PTR(e));
    } else {
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
    int i, l1;

    for (i = 0; i < ESP32P4_CACHE_NREGS; i++) {
        c->regs[i] = esp32p4_cache_regs[i].reset;
    }
    for (i = 0; i < ESP32P4_CACHE_NVIEWS; i++) {
        ESP32P4CacheView *v = &c->view[i];

        for (l1 = 0; l1 < NL1; l1++) {
            l1_untranslate(v, l1, 0, v->nlines);
            bitmap_zero(v->l1c[l1].valid, v->nlines);
            bitmap_zero(v->l1c[l1].lock, v->nlines);
        }
        bitmap_zero(v->l1d_dirty, v->nlines);
        bitmap_zero(v->l2_valid, v->nlines);
        bitmap_zero(v->l2_dirty, v->nlines);
        bitmap_zero(v->l2_lock, v->nlines);
        bitmap_zero(v->l1d_direct, v->npages);
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

/*
 * A view's memory: each L1 cache's contents, the data cache's watching the
 * cores' stores, and the L2's; the controller's region; and the space the
 * translations reach them in.
 */
static void cache_view_init(ESP32P4CacheState *c, int kind, Error **errp)
{
    ESP32P4CacheView *v = &c->view[kind];
    Object *obj = OBJECT(c);
    const char *n = esp32p4_cache_view_name[kind];
    g_autofree char *iommu = g_strdup_printf("esp32p4.cache-%s", n);
    g_autofree char *target = g_strdup_printf("esp32p4.cache-%s-target", n);
    g_autofree char *l2 = g_strdup_printf("esp32p4.cache-%s-l2", n);
    g_autofree char *miss = g_strdup_printf("esp32p4.cache-%s-miss", n);
    int l1;

    v->cache = c;
    v->kind = kind;
    v->size = kind == ESP32P4_CACHE_VIEW_L2MEM ? L2MEM_SIZE
                                               : ESP32P4_MSPI_WINDOW_SIZE;
    v->nlines = v->size / ESP32P4_CACHE_LINE;
    v->npages = v->size / FILL;

    memory_region_init_iommu(&v->iommu, sizeof(v->iommu),
                             TYPE_ESP32P4_CACHE_VIEW, obj, iommu, v->size);
    memory_region_init(&v->target, obj, target, NTARGETS * v->size);
    for (l1 = 0; l1 < NL1; l1++) {
        g_autofree char *name = g_strdup_printf("esp32p4.cache-%s-%s", n,
                                                l1_name[l1]);

        if (!memory_region_init_ram(&v->l1[l1], obj, name, v->size, errp)) {
            return;
        }
        v->l1_host[l1] = memory_region_get_ram_ptr(&v->l1[l1]);
    }
    memory_region_set_store_watch(&v->l1[L1_D], &l1d_watch_ops, v);
    if (kind != ESP32P4_CACHE_VIEW_L2MEM) {
        if (!memory_region_init_ram(&v->l2, obj, l2, v->size, errp)) {
            return;
        }
        v->l2_host = memory_region_get_ram_ptr(&v->l2);
    }
    memory_region_init_io(&v->miss, obj, &cache_view_ops, v, miss, v->size);
    memory_region_add_subregion(&v->target, TARGET_L1D * v->size,
                                &v->l1[L1_D]);
    memory_region_add_subregion(&v->target, TARGET_L1I0 * v->size,
                                &v->l1[L1_I0]);
    memory_region_add_subregion(&v->target, TARGET_L1I1 * v->size,
                                &v->l1[L1_I1]);
    memory_region_add_subregion(&v->target, TARGET_MISS * v->size, &v->miss);
    address_space_init(&v->target_as, &v->target, target);

    for (l1 = 0; l1 < NL1; l1++) {
        v->l1c[l1].valid = bitmap_new(v->nlines);
        v->l1c[l1].lock = bitmap_new(v->nlines);
        v->l1c[l1].nlines = v->nlines;
    }
    v->l1d_dirty = bitmap_new(v->nlines);
    v->l2_valid = bitmap_new(v->nlines);
    v->l2_dirty = bitmap_new(v->nlines);
    v->l2_lock = bitmap_new(v->nlines);
    v->l1d_direct = bitmap_new(v->npages);
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

    c->l2mem_host = memory_region_get_ram_ptr(c->l2mem_mr);
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

/*
 * The cores' translations are emptied as a migration loads, so no page is
 * stored to directly.
 */
static int cache_post_load(void *opaque, int version_id)
{
    ESP32P4CacheState *c = opaque;
    uint32_t size = c->l2_cache_size;
    int i;

    for (i = 0; i < ESP32P4_CACHE_NVIEWS; i++) {
        bitmap_zero(c->view[i].l1d_direct, c->view[i].npages);
    }
    c->l2_cache_size = UINT32_MAX;
    cache_update_split(c);
    return size == c->l2_cache_size ? 0 : -EINVAL;
}

static const VMStateDescription vmstate_esp32p4_cache_l1 = {
    .name = "esp32p4-cache-l1",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BITMAP(valid, ESP32P4CacheL1, 1, nlines),
        VMSTATE_BITMAP(lock, ESP32P4CacheL1, 1, nlines),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_cache_view = {
    .name = TYPE_ESP32P4_CACHE_VIEW,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(l1c, ESP32P4CacheView, ESP32P4_CACHE_NL1, 2,
                             vmstate_esp32p4_cache_l1, ESP32P4CacheL1),
        VMSTATE_BITMAP(l1d_dirty, ESP32P4CacheView, 2, nlines),
        VMSTATE_BITMAP(l2_valid, ESP32P4CacheView, 2, nlines),
        VMSTATE_BITMAP(l2_dirty, ESP32P4CacheView, 2, nlines),
        VMSTATE_BITMAP(l2_lock, ESP32P4CacheView, 2, nlines),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_cache = {
    .name = TYPE_ESP32P4_CACHE,
    .version_id = 2,
    .minimum_version_id = 2,
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
                             2, vmstate_esp32p4_cache_view,
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

/*
 * A core's loads and stores take the data cache's translation, and each
 * core's instruction fetches its instruction cache's.
 */
/* [spec:nuos:req:emu.esp32p4.memory] */
static int cache_view_attrs_to_index(IOMMUMemoryRegion *iommu,
                                     MemTxAttrs attrs)
{
    int core;

    return cache_fetch(attrs, &core) ? 1 + core : 0;
}

static int cache_view_num_indexes(IOMMUMemoryRegion *iommu)
{
    return 1 + ESP32P4_CACHE_L1_D;
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
