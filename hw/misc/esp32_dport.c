/*
 * ESP32 "DPORT" device
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/registerfields.h"
#include "hw/core/boards.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_dport.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "hw/core/cpu.h"
#include "system/tcg.h"
#include "qemu/main-loop.h"

#include "hw/misc/esp32_flash_enc.h"
#include "hw/nvram/esp32_efuse.h"


#define ESP32_DPORT_SIZE        (DR_REG_DPORT_APB_BASE - DR_REG_DPORT_BASE)

#define MMU_RANGE_SIZE          (ESP32_CACHE_PAGES_PER_REGION * sizeof(uint32_t))
#define MMU_RANGE_LAST          (MMU_RANGE_SIZE - sizeof(uint32_t))

#define PRO_DROM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE)
#define PRO_DROM0_MMU_LAST      (PRO_DROM0_MMU_FIRST + MMU_RANGE_LAST)
#define PRO_IRAM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE + MMU_RANGE_SIZE)
#define PRO_IRAM0_MMU_LAST      (PRO_IRAM0_MMU_FIRST + MMU_RANGE_LAST)
#define APP_DROM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE)
#define APP_DROM0_MMU_LAST      (APP_DROM0_MMU_FIRST + MMU_RANGE_LAST)
#define APP_IRAM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE + MMU_RANGE_SIZE)
#define APP_IRAM0_MMU_LAST      (APP_IRAM0_MMU_FIRST + MMU_RANGE_LAST)
#define MMU_ENTRY_MASK          0x1ff

static void esp32_cache_state_update(Esp32CacheState* cs);
static void esp32_cache_data_sync(Esp32CacheRegionState* crs);
static void esp32_cache_invalidate_all_entries(Esp32CacheRegionState* crs);
static void esp32_cache_ctrl1_write(Esp32DportState *s, int cpu,
                                    uint32_t value);

static inline uint32_t get_mmu_entry(Esp32CacheRegionState* crs, hwaddr base, hwaddr addr)
{
    return crs->mmu_table[(addr - base)/sizeof(uint32_t)] & MMU_ENTRY_MASK;
}

static inline void set_mmu_entry(Esp32CacheRegionState* crs, hwaddr base, hwaddr addr, uint64_t val)
{
    uint32_t old_val = crs->mmu_table[(addr - base)/sizeof(uint32_t)];
    if (val != old_val) {
        crs->mmu_table[(addr - base)/sizeof(uint32_t)] = (val & MMU_ENTRY_MASK) | ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
}

static uint64_t esp32_dport_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32DportState *s = ESP32_DPORT(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_DPORT_APPCPU_RESET:
        r = s->appcpu_reset_state;
        break;
    case A_DPORT_APPCPU_CLK:
        r = s->appcpu_clkgate_state;
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        r = s->appcpu_stall_state;
        break;
    case A_DPORT_APPCPU_BOOT_ADDR:
        r = s->appcpu_boot_addr;
        break;
    case A_DPORT_CPU_PER_CONF:
        r = s->cpuperiod_sel;
        break;
    case A_DPORT_PRO_CACHE_CTRL:
        r = s->cache_state[0].cache_ctrl_reg;
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        r = s->cache_state[0].cache_ctrl1_reg;
        break;
    case A_DPORT_APP_CACHE_CTRL:
        r = s->cache_state[1].cache_ctrl_reg;
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        r = s->cache_state[1].cache_ctrl1_reg;
        break;
    case A_DPORT_PRO_DCACHE_DBUG0:
    case A_DPORT_APP_DCACHE_DBUG0:
        /* in idle state */
        r = FIELD_DP32(0, DPORT_PRO_DCACHE_DBUG0, CACHE_STATE, 1);
        r |= s->cache_state[addr == A_DPORT_APP_DCACHE_DBUG0].cache_ia;
        break;
    case A_DPORT_CACHE_IA_INT_EN:
        r = s->cache_ill_trap_en_reg;
        break;
    case A_DPORT_PRO_DCACHE_DBUG3:
        r = 0;
        r = FIELD_DP32(r, DPORT_PRO_DCACHE_DBUG3, IA_INT_DROM0, s->cache_state[0].drom0.illegal_access_status);
        r = FIELD_DP32(r, DPORT_PRO_DCACHE_DBUG3, IA_INT_IRAM0, s->cache_state[0].iram0.illegal_access_status);
        r |= s->cache_state[0].cache_dis_ia;
        break;
    case A_DPORT_APP_DCACHE_DBUG3:
        r = 0;
        r = FIELD_DP32(r, DPORT_APP_DCACHE_DBUG3, IA_INT_DROM0, s->cache_state[1].drom0.illegal_access_status);
        r = FIELD_DP32(r, DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM0, s->cache_state[1].iram0.illegal_access_status);
        r |= s->cache_state[1].cache_dis_ia;
        break;
    case PRO_DROM0_MMU_FIRST ... PRO_DROM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[0].drom0, PRO_DROM0_MMU_FIRST, addr);
        break;
    case PRO_IRAM0_MMU_FIRST ... PRO_IRAM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[0].iram0, PRO_IRAM0_MMU_FIRST, addr);
        break;
    case APP_DROM0_MMU_FIRST ... APP_DROM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[1].drom0, APP_DROM0_MMU_FIRST, addr);
        break;
    case APP_IRAM0_MMU_FIRST ... APP_IRAM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[1].iram0, APP_IRAM0_MMU_FIRST, addr);
        break;
    case A_DPORT_SLAVE_SPI_CONFIG:
        r = s->slave_spi_config_reg;
        break;
    case A_DPORT_PERI_CLK_EN:
        r = s->peri_clk_en;
        break;
    case A_DPORT_PERI_RST_EN:
        r = s->peri_rst_en;
        break;
    case A_DPORT_PERIP_CLK_EN:
        r = s->perip_clk_en;
        break;
    case A_DPORT_PERIP_RST_EN:
        r = s->perip_rst_en;
        break;
    case A_DPORT_WIFI_CLK_EN:
        r = s->wifi_clk_en;
        break;
    case A_DPORT_CORE_RST_EN:
        r = s->core_rst_en;
        break;
    case A_DPORT_SPI_DMA_CHAN_SEL:
        r = s->spi_dma_chan_sel;
        break;
    default:
        esp32_dport_mpu_read(s, addr, &r);
        break;
    }

    return r;
}

static void esp32_dport_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    Esp32DportState *s = ESP32_DPORT(opaque);
    bool old_state;
    uint32_t old_val;
    switch (addr) {
    case A_DPORT_APPCPU_RESET:
        old_state = s->appcpu_reset_state;
        s->appcpu_reset_state = value & 1;
        if (old_state && !s->appcpu_reset_state) {
            qemu_irq_pulse(s->appcpu_reset_req);
        }
        break;
    case A_DPORT_APPCPU_CLK:
        s->appcpu_clkgate_state = value & 1;
        qemu_set_irq(s->appcpu_stall_req, s->appcpu_stall_state || !s->appcpu_clkgate_state);
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        s->appcpu_stall_state = value & 1;
        qemu_set_irq(s->appcpu_stall_req, s->appcpu_stall_state || !s->appcpu_clkgate_state);
        break;
    case A_DPORT_APPCPU_BOOT_ADDR:
        s->appcpu_boot_addr = value;
        break;
    case A_DPORT_CPU_PER_CONF:
        s->cpuperiod_sel = value & R_DPORT_CPU_PER_CONF_CPUPERIOD_SEL_MASK;
        qemu_irq_pulse(s->clk_update_req);
        break;
    case A_DPORT_PRO_CACHE_CTRL:
        if (FIELD_EX32(value, DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_ENA)) {
            value |= R_DPORT_PRO_CACHE_CTRL_CACHE_FLUSH_DONE_MASK;
            value &= ~R_DPORT_PRO_CACHE_CTRL_CACHE_FLUSH_ENA_MASK;
            esp32_cache_invalidate_all_entries(&s->cache_state[0].drom0);
            esp32_cache_data_sync(&s->cache_state[0].drom0);
            esp32_cache_invalidate_all_entries(&s->cache_state[0].iram0);
            esp32_cache_data_sync(&s->cache_state[0].iram0);
            esp32_dport_mpu_cache_flushed(s);
        }
        old_val = s->cache_state[0].cache_ctrl_reg;
        s->cache_state[0].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[0]);
        }
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        esp32_cache_ctrl1_write(s, 0, value);
        break;
    case A_DPORT_APP_CACHE_CTRL:
        if (FIELD_EX32(value, DPORT_APP_CACHE_CTRL, CACHE_FLUSH_ENA)) {
            value |= R_DPORT_APP_CACHE_CTRL_CACHE_FLUSH_DONE_MASK;
            value &= ~R_DPORT_APP_CACHE_CTRL_CACHE_FLUSH_ENA_MASK;
            esp32_cache_invalidate_all_entries(&s->cache_state[1].drom0);
            esp32_cache_data_sync(&s->cache_state[1].drom0);
            esp32_cache_invalidate_all_entries(&s->cache_state[1].iram0);
            esp32_cache_data_sync(&s->cache_state[1].iram0);
            esp32_dport_mpu_cache_flushed(s);
        }
        old_val = s->cache_state[1].cache_ctrl_reg;
        s->cache_state[1].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[1]);
        }
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        esp32_cache_ctrl1_write(s, 1, value);
        break;
    case A_DPORT_CACHE_IA_INT_EN:
        s->cache_ill_trap_en_reg = value;
        s->cache_state[0].drom0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DROM0));
        s->cache_state[0].iram0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IRAM0));
        s->cache_state[1].drom0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_APP_DROM0));
        s->cache_state[1].iram0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_APP_IRAM0));
        s->cache_state[0].dram1.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DRAM1));
        s->cache_state[1].dram1.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_APP_DRAM1));
        esp32_dport_update_cache_irq(s);
        break;
    case PRO_DROM0_MMU_FIRST ... PRO_DROM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[0].drom0, PRO_DROM0_MMU_FIRST, addr, value);
        break;
    case PRO_IRAM0_MMU_FIRST ... PRO_IRAM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[0].iram0, PRO_IRAM0_MMU_FIRST, addr, value);
        break;
    case APP_DROM0_MMU_FIRST ... APP_DROM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[1].drom0, APP_DROM0_MMU_FIRST, addr, value);
        break;
    case APP_IRAM0_MMU_FIRST ... APP_IRAM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[1].iram0, APP_IRAM0_MMU_FIRST, addr, value);
        break;
    case A_DPORT_SLAVE_SPI_CONFIG:
        s->slave_spi_config_reg = value;
        qemu_set_irq(s->flash_enc_en_gpio, FIELD_EX32(value, DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_ENCRYPT_ENABLE));
        qemu_set_irq(s->flash_dec_en_gpio, FIELD_EX32(value, DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_DECRYPT_ENABLE));
        break;
    /*
     * [spec:nuos:req:emu.esp32.clock-gating]
     * All bits read back as written; the reset bits are not self-clearing.
     */
    case A_DPORT_PERI_CLK_EN:
        s->peri_clk_en = value;
        qemu_irq_pulse(s->periph_clk_update_req);
        break;
    case A_DPORT_PERI_RST_EN:
        s->peri_rst_en = value;
        qemu_irq_pulse(s->periph_clk_update_req);
        break;
    case A_DPORT_PERIP_CLK_EN:
        s->perip_clk_en = value;
        qemu_irq_pulse(s->periph_clk_update_req);
        break;
    case A_DPORT_PERIP_RST_EN:
        s->perip_rst_en = value;
        qemu_irq_pulse(s->periph_clk_update_req);
        break;
    case A_DPORT_WIFI_CLK_EN:
        s->wifi_clk_en = value;
        qemu_irq_pulse(s->periph_clk_update_req);
        break;
    case A_DPORT_CORE_RST_EN:
        s->core_rst_en = value;
        qemu_irq_pulse(s->periph_clk_update_req);
        break;
    /*
     * [spec:nuos:req:emu.esp32.spi-dma]
     * Two bits per SPI controller; the SPI DMA engines read the selection
     * whenever they act. Bits 31:6 are reserved and read as 0.
     */
    case A_DPORT_SPI_DMA_CHAN_SEL:
        s->spi_dma_chan_sel = value & (R_DPORT_SPI_DMA_CHAN_SEL_SPI1_MASK |
                                       R_DPORT_SPI_DMA_CHAN_SEL_SPI2_MASK |
                                       R_DPORT_SPI_DMA_CHAN_SEL_SPI3_MASK);
        break;
    default:
        esp32_dport_mpu_write(s, addr, value);
        break;
    }
}

static const MemoryRegionOps esp32_dport_ops = {
    .read =  esp32_dport_read,
    .write = esp32_dport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32.memory-protection]
 * The cache fills each 32-byte block (TRM 3.3.4) on its first miss after
 * a flush, through SPI0 (flash_read, hw/ssi/esp32_spi.c), as the flash and
 * its pads then give it, the fetching CPU stalled for the transaction's
 * time. The model keeps every block it fills until a flush or the region's
 * enable empties the pages whose MMU entries changed, as a cache as large
 * as its regions would: the 32 KB cache's evictions are not modelled.
 *
 * How it keeps TCG's fast path (a TLB page is 4 KB, 128 blocks):
 *
 * - a TLB page whose blocks are all held maps the cache's contents, at
 *   the speed of RAM;
 * - otherwise a data read of the page maps the region's slow path, which
 *   fills a block on its first read, exactly, and counts reads of blocks
 *   already held: after ESP32_CACHE_PROMOTE_HITS of them (a page read
 *   over more than once) the page is filled whole and goes to the fast
 *   path;
 * - an instruction fetch fills the page whole (code runs only from the
 *   fast path) and the translator reports each instruction's bytes, and
 *   each L32R's literal, as it first fetches them, which is when a block
 *   filled ahead of its use takes its miss's time.
 *
 * A block filled with the rest of its page is filled as its first miss
 * would have filled it unless the flash, or how SPI0 reaches it, changes
 * before that miss: then, where the model cannot tell whether the CPU
 * read it meanwhile, it is emptied (as if evicted, which a real cache may
 * do at any time), to be filled again on its next access
 * (esp32_dport_cache_refresh()). A block read on the slow path, or
 * fetched as code, keeps what it holds, as the hardware's does. Such a
 * block's data read that is not its first fetch does not take a miss's
 * time.
 */
#define ESP32_CACHE_PROMOTE_HITS    1024

static unsigned esp32_cache_region_index(Esp32CacheRegionState *crs)
{
    return crs->type == ESP32_ICACHE_FLASH ? 1 : 0;
}

static void esp32_cache_flush_tlb_page(Esp32CacheRegionState *crs,
                                       unsigned page)
{
    CPUState *cs = qemu_get_cpu(crs->cache->core_id);

    if (tcg_enabled() && cs && cs->created) {
        tlb_flush_page(cs, crs->base + page * ESP32_CACHE_TLB_PAGE);
    }
}

static bool esp32_cache_page_held(Esp32CacheRegionState *crs, unsigned page)
{
    return find_next_zero_bit(crs->valid,
                              (page + 1) * ESP32_CACHE_TLB_PAGE_BLOCKS,
                              page * ESP32_CACHE_TLB_PAGE_BLOCKS) >=
           (page + 1) * ESP32_CACHE_TLB_PAGE_BLOCKS;
}

/*
 * Fill block @b as SPI0 reads it now, through the MMU entry of its page:
 * an invalid entry gives the region's illegal-access value, and no
 * transaction. Returns the time the fill took, in ns.
 */
static int64_t esp32_cache_fill_block(Esp32CacheRegionState *crs, unsigned b)
{
    Esp32DportState *ds = crs->cache->dport;
    uint8_t *cache_data = memory_region_get_ram_ptr(&crs->mem);
    uint32_t off = b * ESP32_CACHE_BLOCK_SIZE;
    uint32_t entry = crs->mmu_table[off / ESP32_CACHE_PAGE_SIZE] &
                     MMU_ENTRY_MASK;
    uint32_t *block = (uint32_t *)(cache_data + off);
    int64_t ns = 0;

    if (entry & ESP32_CACHE_MMU_INVALID_VAL) {
        for (int i = 0; i < ESP32_CACHE_BLOCK_SIZE / 4; i++) {
            block[i] = crs->illegal_access_retval;
        }
    } else {
        uint32_t phys = entry * ESP32_CACHE_PAGE_SIZE +
                        off % ESP32_CACHE_PAGE_SIZE;
        Esp32FlashEncryptionState *enc = esp32_flash_encryption_find();

        if (ds->flash_read) {
            ns = ds->flash_read(ds->flash_read_opaque, phys,
                                (uint8_t *)block);
        } else {
            blk_pread(ds->flash_blk, phys, ESP32_CACHE_BLOCK_SIZE, block, 0);
        }
        if (enc && esp32_flash_decryption_enabled(enc)) {
            esp32_flash_decrypt_inplace(enc, phys, block,
                                        ESP32_CACHE_BLOCK_SIZE / 4);
        }
    }
    set_bit(b, crs->valid);
    clear_bit(b, crs->spec);
    return ns == INT64_MAX ? 0 : ns;
}

static void esp32_cache_stall(Esp32CacheRegionState *crs, int64_t ns)
{
    Esp32DportState *ds = crs->cache->dport;

    if (ns > 0 && ds->stall) {
        ds->stall(ds->stall_opaque, crs->cache->core_id, ns);
    }
}

/*
 * TLB page @page goes to the fast path: fill the blocks it does not hold,
 * ahead of their first miss.
 */
static void esp32_cache_promote(Esp32CacheRegionState *crs, unsigned page)
{
    unsigned first = page * ESP32_CACHE_TLB_PAGE_BLOCKS;
    unsigned end = first + ESP32_CACHE_TLB_PAGE_BLOCKS;
    unsigned b;

    /*
     * A fill that comes after SPI1's transaction ends it first, which can
     * empty blocks filled ahead (esp32_dport_cache_refresh()): fill until
     * the page holds them all.
     */
    while ((b = find_next_zero_bit(crs->valid, end, first)) < end) {
        esp32_cache_fill_block(crs, b);
        set_bit(b, crs->spec);
    }
    crs->hits[page] = 0;
}

/* Empty TLB page @page's blocks, for its next accesses to fill again */
static void esp32_cache_empty_page(Esp32CacheRegionState *crs, unsigned page)
{
    bitmap_clear(crs->valid, page * ESP32_CACHE_TLB_PAGE_BLOCKS,
                 ESP32_CACHE_TLB_PAGE_BLOCKS);
    bitmap_clear(crs->spec, page * ESP32_CACHE_TLB_PAGE_BLOCKS,
                 ESP32_CACHE_TLB_PAGE_BLOCKS);
    crs->hits[page] = 0;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * A data read on the slow path: the block's first fills it, the CPU
 * stalled meanwhile; the page goes to the fast path once all its blocks
 * are held, or after ESP32_CACHE_PROMOTE_HITS reads of held ones.
 */
static uint64_t esp32_cache_miss_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    Esp32CacheRegionState *crs = opaque;
    uint8_t *cache_data = memory_region_get_ram_ptr(&crs->mem);
    unsigned page = addr / ESP32_CACHE_TLB_PAGE;
    unsigned b = addr / ESP32_CACHE_BLOCK_SIZE;
    unsigned last = (addr + size - 1) / ESP32_CACHE_BLOCK_SIZE;
    bool missed = false;
    int64_t ns = 0;

    for (; b <= last && b < ESP32_CACHE_REGION_BLOCKS; b++) {
        if (!test_bit(b, crs->valid)) {
            ns += esp32_cache_fill_block(crs, b);
            missed = true;
        } else {
            clear_bit(b, crs->spec);
        }
    }
    if (!missed && ++crs->hits[page] >= ESP32_CACHE_PROMOTE_HITS) {
        esp32_cache_promote(crs, page);
    }
    if (esp32_cache_page_held(crs, page)) {
        esp32_cache_flush_tlb_page(crs, page);
    }
    esp32_cache_stall(crs, ns);
    return ldn_le_p(cache_data + addr, size);
}

static void esp32_cache_miss_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size)
{
}

static bool esp32_cache_miss_accepts(void *opaque, hwaddr addr, unsigned size,
                                     bool is_write, MemTxAttrs attrs)
{
    return !is_write;
}

static const MemoryRegionOps esp32_cache_miss_ops = {
    .read = esp32_cache_miss_read,
    .write = esp32_cache_miss_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.accepts = esp32_cache_miss_accepts,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .impl.unaligned = true,
};

/*
 * The flash region of CPU @cpu at @vaddr, and the offset in it: DROM0, or
 * IRAM0 from 0x400C2000, below which the ROM and internal memories are
 * (TRM table 3.3-4).
 */
static Esp32CacheRegionState *esp32_cache_region(Esp32DportState *s, int cpu,
                                                 uint32_t vaddr, hwaddr *off)
{
    Esp32CacheState *cs = &s->cache_state[cpu];
    Esp32CacheRegionState *regions[] = { &cs->drom0, &cs->iram0 };
    const uint32_t starts[] = { 0, 0xc2000 };

    for (int i = 0; i < ARRAY_SIZE(regions); i++) {
        Esp32CacheRegionState *crs = regions[i];
        uint32_t o = vaddr - crs->base;

        if (o >= starts[i] && o < ESP32_CACHE_REGION_SIZE &&
            crs->mem.enabled) {
            *off = o;
            return crs;
        }
    }
    return NULL;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * [spec:nuos:req:emu.esp32.memory-protection]
 * A TLB fill in a flash region, for PIDs 0 and 1: an instruction fetch
 * fills the page whole; a data read of a page not all held maps the slow
 * path. A page maps 4 KB, so that one page's state does not hold its
 * neighbours'.
 */
void esp32_dport_cache_filter(Esp32DportState *s, int cpu, uint32_t vaddr,
                              MMUAccessType access_type, Esp32MpuMapping *map)
{
    Esp32CacheRegionState *crs;
    hwaddr off;
    unsigned page;

    crs = esp32_cache_region(s, cpu, map->paddr, &off);
    if (!crs || !s->flash_blk) {
        return;
    }
    page = off / ESP32_CACHE_TLB_PAGE;
    map->page_size = ESP32_CACHE_TLB_PAGE;
    if (esp32_cache_page_held(crs, page)) {
        return;
    }
    if (access_type == MMU_DATA_STORE) {
        /* The cache refuses it; the mapping is not to serve reads. */
        map->prot &= PAGE_WRITE;
        return;
    }
    if (access_type == MMU_INST_FETCH) {
        BQL_LOCK_GUARD();

        esp32_cache_promote(crs, page);
        return;
    }
    map->paddr = ESP32_CACHE_MISS_BASE +
                 esp32_cache_region_index(crs) * ESP32_CACHE_REGION_SIZE + off;
    map->prot &= PAGE_READ;
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * The translator fetches @len bytes at @vaddr for the first time: a block
 * filled ahead of its first miss takes the miss's time now; one not held
 * (a literal in a page that has not gone to the fast path) is filled.
 */
void esp32_dport_cache_fetch(Esp32DportState *s, int cpu, uint32_t vaddr,
                             unsigned len)
{
    Esp32CacheRegionState *crs;
    hwaddr off;
    int64_t ns = 0;

    crs = esp32_cache_region(s, cpu, vaddr, &off);
    if (!crs || !s->flash_blk || !len) {
        return;
    }
    BQL_LOCK_GUARD();
    for (unsigned b = off / ESP32_CACHE_BLOCK_SIZE;
         b <= (off + len - 1) / ESP32_CACHE_BLOCK_SIZE &&
         b < ESP32_CACHE_REGION_BLOCKS; b++) {
        if (!test_bit(b, crs->valid)) {
            ns += esp32_cache_fill_block(crs, b);
        } else if (test_bit(b, crs->spec)) {
            clear_bit(b, crs->spec);
            if (s->fill_time) {
                ns += s->fill_time(s->flash_read_opaque);
            }
        }
    }
    esp32_cache_stall(crs, ns);
}

/* [spec:nuos:req:emu.esp32.gpio] */
void esp32_dport_flash_read_page(Esp32DportState *s, uint32_t phys,
                                 uint8_t *buf)
{
    for (uint32_t off = 0; off < ESP32_CACHE_PAGE_SIZE;
         off += ESP32_CACHE_BLOCK_SIZE) {
        if (s->flash_read) {
            s->flash_read(s->flash_read_opaque, phys + off, buf + off);
        } else {
            blk_pread(s->flash_blk, phys + off, ESP32_CACHE_BLOCK_SIZE,
                      buf + off, 0);
        }
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * What SPI0 reads of the flash may have changed (the flash's array or its
 * state, its pads, SPI0's set-up, its supply): empty the blocks filled
 * ahead of their first miss, which the hardware would fill as it reads
 * the flash then, and the shared store's pages for PIDs 2 to 7. The CPUs'
 * TLBs drop the pages that leave the fast path.
 */
void esp32_dport_cache_refresh(Esp32DportState *s)
{
    bool emptied = false;

    for (int cpu = 0; cpu < ESP32_CPU_COUNT; cpu++) {
        Esp32CacheState *cs = &s->cache_state[cpu];
        Esp32CacheRegionState *regions[] = { &cs->drom0, &cs->iram0 };

        for (int r = 0; r < ARRAY_SIZE(regions); r++) {
            Esp32CacheRegionState *crs = regions[r];
            unsigned long b = find_first_bit(crs->spec,
                                             ESP32_CACHE_REGION_BLOCKS);

            for (; b < ESP32_CACHE_REGION_BLOCKS;
                 b = find_next_bit(crs->spec, ESP32_CACHE_REGION_BLOCKS,
                                   b + 1)) {
                clear_bit(b, crs->valid);
                clear_bit(b, crs->spec);
                crs->hits[b / ESP32_CACHE_TLB_PAGE_BLOCKS] = 0;
                emptied = true;
            }
        }
    }
    if (!bitmap_empty(s->flash_page_valid, ESP32_FLASH_PAGES)) {
        bitmap_zero(s->flash_page_valid, ESP32_FLASH_PAGES);
        memory_region_flush_rom_device(&s->flash_pages, 0,
                                       ESP32_FLASH_PAGES_SIZE);
        emptied = true;
    }
    if (emptied) {
        qemu_irq_pulse(s->mpu_update_req);
    }
}

/*
 * [spec:nuos:req:emu.esp32.gpio]
 * Empty the pages whose MMU entries changed, for their blocks to fill on
 * their misses. Their contents meanwhile are the flash's array as the
 * drive holds it, which only a debugger's reads see.
 */
static void esp32_cache_data_sync(Esp32CacheRegionState* crs)
{
    if (crs->cache->dport->flash_blk == NULL) {
        return;
    }

    uint8_t* cache_data = (uint8_t*) memory_region_get_ram_ptr(&crs->mem);
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        uint32_t* cache_page = (uint32_t*) (cache_data + i * ESP32_CACHE_PAGE_SIZE);
        uint32_t mmu_entry = crs->mmu_table[i];
        unsigned tlb_pages = ESP32_CACHE_PAGE_SIZE / ESP32_CACHE_TLB_PAGE;

        if (!(mmu_entry & ESP32_CACHE_MMU_ENTRY_CHANGED)) {
            continue;
        }
        mmu_entry &= MMU_ENTRY_MASK;
        if (mmu_entry & ESP32_CACHE_MMU_INVALID_VAL) {
            uint32_t fill_val = crs->illegal_access_retval;
            for (int word = 0; word < ESP32_CACHE_PAGE_SIZE / sizeof(uint32_t); ++word) {
                cache_page[word] = fill_val;
            }
        } else {
            Esp32FlashEncryptionState *enc = esp32_flash_encryption_find();
            uint32_t phys = mmu_entry * ESP32_CACHE_PAGE_SIZE;

            blk_pread(crs->cache->dport->flash_blk, phys,
                      ESP32_CACHE_PAGE_SIZE, cache_page, 0);
            if (enc && esp32_flash_decryption_enabled(enc)) {
                esp32_flash_decrypt_inplace(enc, phys, cache_page,
                                            ESP32_CACHE_PAGE_SIZE / 4);
            }
        }
        for (unsigned p = 0; p < tlb_pages; p++) {
            esp32_cache_empty_page(crs, i * tlb_pages + p);
        }
        crs->mmu_table[i] &= ~ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
    memory_region_flush_rom_device(&crs->mem, 0, ESP32_CACHE_REGION_SIZE);
}

static void esp32_cache_invalidate_all_entries(Esp32CacheRegionState* crs)
{
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        crs->mmu_table[i] |= ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * x_CACHE_MMU_IA_CLR clears the CPU's cache illegal-access flags while it
 * is set.
 */
static void esp32_cache_ctrl1_write(Esp32DportState *s, int cpu,
                                    uint32_t value)
{
    Esp32CacheState *cs = &s->cache_state[cpu];
    uint32_t old_val = cs->cache_ctrl1_reg;

    cs->cache_ctrl1_reg = value;
    if (FIELD_EX32(value, DPORT_PRO_CACHE_CTRL1, MMU_IA_CLR)) {
        cs->cache_ia = 0;
        esp32_dport_update_cache_irq(s);
    }
    if (value != old_val) {
        esp32_cache_state_update(cs);
    }
}

static void esp32_cache_state_update(Esp32CacheState* cs)
{
    bool cache_enabled = FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL, CACHE_ENA) != 0;

    bool drom0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DROM0) == 0;
    if (!cs->drom0.mem.enabled && drom0_enabled) {
        esp32_cache_data_sync(&cs->drom0);
    }
    memory_region_set_enabled(&cs->drom0.mem, drom0_enabled);

    bool iram0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_IRAM0) == 0;
    if (!cs->iram0.mem.enabled && iram0_enabled) {
        esp32_cache_data_sync(&cs->iram0);
    }
    memory_region_set_enabled(&cs->iram0.mem, iram0_enabled);

    if (cs->dport->has_psram) {
        bool dram1_enabled = cache_enabled &&
            FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DRAM1) == 0;
        memory_region_set_enabled(&cs->dram1.mem, dram1_enabled);
    }
    /* The per-PID cache mappings depend on the enables and modes too. */
    qemu_irq_pulse(cs->dport->mpu_update_req);
}

static void esp32_cache_region_reset(Esp32CacheRegionState *crs)
{
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        crs->mmu_table[i] = ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
    crs->illegal_access_trap_en = false;
    bitmap_zero(crs->valid, ESP32_CACHE_REGION_BLOCKS);
    bitmap_zero(crs->spec, ESP32_CACHE_REGION_BLOCKS);
    memset(crs->hits, 0, sizeof(crs->hits));
}

static void esp32_cache_reset(Esp32CacheState *cs)
{
    esp32_cache_region_reset(&cs->drom0);
    esp32_cache_region_reset(&cs->iram0);
}

static uint64_t esp32_cache_ill_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32CacheRegionState *crs = (Esp32CacheRegionState*) opaque;
    uint32_t ill_data[] = { crs->illegal_access_retval, crs->illegal_access_retval };
    uint32_t result;
    memcpy(&result, ((uint8_t*) ill_data) + (addr % 4), size);
    if (crs->illegal_access_trap_en) {
        crs->illegal_access_status = true;
        esp32_dport_update_cache_irq(crs->cache->dport);
    }
    return result;
}

static void esp32_cache_ill_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned int size)
{
}

static bool esp32_cache_ill_accepts(void *opaque, hwaddr addr,
                             unsigned size, bool is_write,
                             MemTxAttrs attrs)
{
    return !is_write;
}


void esp32_dport_clear_ill_trap_state(Esp32DportState* s)
{
    s->cache_state[0].drom0.illegal_access_status = false;
    s->cache_state[1].drom0.illegal_access_status = false;
    s->cache_state[0].iram0.illegal_access_status = false;
    s->cache_state[1].iram0.illegal_access_status = false;
    s->cache_state[0].dram1.illegal_access_status = false;
    s->cache_state[1].dram1.illegal_access_status = false;
    qemu_irq_lower(s->cache_ill_irq);
}

static const MemoryRegionOps esp32_cache_ops = {
    .write = NULL,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.accepts = esp32_cache_ill_accepts,
};


static const MemoryRegionOps esp32_cache_ill_trap_ops = {
    .read = esp32_cache_ill_read,
    .write = esp32_cache_ill_write,
};

static void esp32_dport_reset_hold(Object *obj, ResetType type)
{
    Esp32DportState *s = ESP32_DPORT(obj);

    s->appcpu_boot_addr = 0;
    s->appcpu_clkgate_state = false;
    s->appcpu_reset_state = true;
    s->appcpu_stall_state = false;
    s->cache_ill_trap_en_reg = 0;
    s->cpuperiod_sel = 0;
    s->peri_clk_en = 0;
    s->peri_rst_en = 0;
    s->perip_clk_en = ESP32_DPORT_PERIP_CLK_EN_RESET;
    s->perip_rst_en = 0;
    s->wifi_clk_en = ESP32_DPORT_WIFI_CLK_EN_RESET;
    s->core_rst_en = 0;
    s->spi_dma_chan_sel = 0;
    esp32_cache_reset(&s->cache_state[0]);
    esp32_cache_reset(&s->cache_state[1]);
    esp32_dport_mpu_reset(s);
    qemu_irq_lower(s->appcpu_stall_req);
}

static void esp32_dport_realize(DeviceState *dev, Error **errp)
{
    Esp32DportState *s = ESP32_DPORT(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    s->cpu_count = ms->smp.cpus;
}

static void esp32_cache_init_region(Esp32DportState *ds,
                                    Esp32CacheState *cs,
                                    Esp32CacheRegionState *crs,
                                    Esp32CacheRegionType type,
                                    const char* name, hwaddr base,
                                    uint32_t illegal_access_retval)
{
    char desc[16];
    crs->cache = cs;
    crs->type = type;
    crs->base = base;
    crs->illegal_access_retval = illegal_access_retval;
    snprintf(desc, sizeof(desc), "cpu%d-%s", cs->core_id, name);
    if (type == ESP32_DCACHE_PSRAM) {
        memory_region_init_alias(&crs->mem, OBJECT(cs->dport), desc,
                                 &ds->psram, 0, ESP32_CACHE_REGION_SIZE);
    } else {
        memory_region_init_rom_device(&crs->mem, OBJECT(cs->dport),
                                    &esp32_cache_ops, crs,
                                    desc, ESP32_CACHE_REGION_SIZE, &error_abort);
        snprintf(desc, sizeof(desc), "cpu%d-%s-miss", cs->core_id, name);
        memory_region_init_io(&crs->miss_mem, OBJECT(cs->dport),
                              &esp32_cache_miss_ops, crs, desc,
                              ESP32_CACHE_REGION_SIZE);
    }

    snprintf(desc, sizeof(desc), "cpu%d-%s-ill", cs->core_id, name);
    memory_region_init_io(&crs->illegal_access_trap_mem, OBJECT(cs->dport),
                          &esp32_cache_ill_trap_ops, crs,
                          desc, ESP32_CACHE_REGION_SIZE);
}

static void esp32_dport_init(Object *obj)
{
    Esp32DportState *s = ESP32_DPORT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_dport_ops, s,
                          TYPE_ESP32_DPORT, ESP32_DPORT_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    memory_region_init_ram(&s->psram, obj, "psram", ESP32_CACHE_REGION_SIZE, &error_fatal);

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        Esp32CacheState* cs = &s->cache_state[i];
        cs->core_id = i;
        cs->dport = s;
        esp32_cache_init_region(s, cs, &cs->drom0, ESP32_DCACHE_FLASH, "drom0",
                                0x3F400000, 0xbaadbaad);
        esp32_cache_init_region(s, cs, &cs->iram0, ESP32_ICACHE_FLASH, "iram0",
                                0x40000000, 0x00000000);
        esp32_cache_init_region(s, cs, &cs->dram1, ESP32_DCACHE_PSRAM, "dram1",
                                    0x3F800000, 0xbaadbaad);
    }

    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_stall_req, ESP32_DPORT_APPCPU_STALL_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_reset_req, ESP32_DPORT_APPCPU_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->clk_update_req, ESP32_DPORT_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cache_ill_irq, ESP32_DPORT_CACHE_ILL_IRQ_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->flash_enc_en_gpio, ESP32_DPORT_FLASH_ENC_EN_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->flash_dec_en_gpio, ESP32_DPORT_FLASH_DEC_EN_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->periph_clk_update_req,
                             ESP32_DPORT_PERIPH_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->mpu_update_req,
                             ESP32_DPORT_MPU_UPDATE_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->mmu_ia_irq,
                             ESP32_DPORT_MMU_IA_IRQ_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->mpu_ia_irq,
                             ESP32_DPORT_MPU_IA_IRQ_GPIO, 1);
    esp32_dport_mpu_init(s);
}

static const Property esp32_dport_properties[] = {
    DEFINE_PROP_DRIVE("flash", Esp32DportState, flash_blk),
    DEFINE_PROP_BOOL("has_psram", Esp32DportState, has_psram, false),
};

static void esp32_dport_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_dport_reset_hold;
    dc->realize = esp32_dport_realize;
    device_class_set_props(dc, esp32_dport_properties);
}

static const TypeInfo esp32_dport_info = {
    .name = TYPE_ESP32_DPORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32DportState),
    .instance_init = esp32_dport_init,
    .class_init = esp32_dport_class_init
};

static void esp32_dport_register_types(void)
{
    type_register_static(&esp32_dport_info);
}

type_init(esp32_dport_register_types)
