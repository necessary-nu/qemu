/*
 * ESP32 DPORT per-process memory protection: the internal SRAM MMUs, the
 * static, RTC and peripheral MPUs, and the per-PID flash and PSRAM cache MMU
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/*
 * The ESP32 decides every CPU access to on-chip memory, peripherals and
 * the external memory cache by the process ID (PID) the CPU's PID
 * controller reports (TRM chapter 4). The model applies that decision when
 * QEMU fills a CPU's TLB: esp32_dport_mpu_filter() either keeps the core's
 * mapping, redirects it (the SRAM and cache MMUs translate addresses), or
 * maps the page to a sink. A sink page answers every access without
 * touching memory: reads return a fixed value and writes are dropped, and
 * the access sets the DPORT status flag the hardware sets for it and
 * raises the interrupt that flag is enabled onto. The CPU's TLB is flushed
 * whenever its PID or any of this configuration changes.
 *
 * Interpretations of the TRM (version 5.8) where it is silent or
 * inconsistent:
 *
 * - The TRM documents status flags and interrupt enables but not how the
 *   flags clear. The flags here latch, and writing an enable register
 *   clears every flag whose enable bit the write leaves clear;
 *   x_CACHE_MMU_IA_CLR clears that CPU's x_DCACHE_DBUG0 flags.
 * - DPORT_MPU_ACCESS_ILLEGAL_INT_EN_REG (0x59C) enables DBUG0's flags
 *   shifted down by 6 and raises MPU_IA_INT; DPORT_MMU_ACCESS_ILLEGAL_
 *   INT_EN_REG (0x598) enables DBUG1's flags shifted down by 4 and raises
 *   MMU_IA_INT, which is how the two registers' fields line up.
 * - An SRAM MMU access whose virtual page no table entry names sets
 *   MEM_ACCESS_MISS, which has no enable, and MEM_ACCESS_DENY as well, so
 *   that it can interrupt like any other refusal.
 * - The cache's illegal-access enables are at 0x5A0, the register ESP-IDF
 *   uses for the cache-disabled flags, with the access-illegal and
 *   MMU-illegal fields at the positions the TRM gives for its "0x3A0"
 *   register; 0x3A0 is DPORT_AHBLITE_MPU_TABLE_TIMERGROUP_REG.
 * - Refusals the TRM gives no flag for (ROM, SRAM1, RTC memories,
 *   peripherals) answer as a sink without an interrupt. Peripheral
 *   refusals set ESP-IDF's DPORT_AHBLITE_ACCESS_DENY (DBUG1 bit 9).
 * - A refused read returns 0; one in a data cache region returns
 *   0xbaadbaad, as the cache-disabled model does. A refused instruction
 *   fetch therefore reads ILL (0x000000).
 * - RTC_CNTL's RTC FAST memory PID mask is not modelled; RTC FAST memory
 *   is open to PIDs 0 and 1 and closed to PIDs 2 to 7.
 * - DPORT_IMMU/DMMU_PAGE_MODE_REG bit 0 (ESP-IDF's INTERNAL_SRAM_x_ENA,
 *   reserved in the TRM) is stored; the MMUs govern PIDs 2 to 7 whatever
 *   its value, as the TRM describes them.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "exec/page-protection.h"
#include "hw/core/irq.h"
#include "hw/core/registerfields.h"
#include "hw/core/cpu.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_dport.h"
#include "hw/misc/esp32_flash_enc.h"

/* The Xtensa TLB page */
#define MPU_PAGE_BITS       12
#define MPU_PAGE_SIZE       (1u << MPU_PAGE_BITS)

/*
 * Sink pages. Page number (kind << 1) | data, where kind = (flag register
 * << 5) | bit and data selects the value refused reads return.
 */
enum {
    SINK_DECODE_ERROR,      /* not an access the hardware decodes */
    SINK_SILENT,            /* refused without a status flag */
    SINK_DBUG0,             /* DPORT_MEM_ACCESS_DBUG0_REG */
    SINK_DBUG1,             /* DPORT_MEM_ACCESS_DBUG1_REG */
    SINK_SRAM_MISS,         /* DBUG1 MEM_ACCESS_MISS and DBUG0 DENY */
    SINK_CACHE_DBUG0,       /* PRO and, at +1, APP x_DCACHE_DBUG0 */
    SINK_CACHE_DBUG0_APP,
    SINK_CACHE_DISABLED,    /* PRO and, at +1, APP x_DCACHE_DBUG3 */
    SINK_CACHE_DISABLED_APP,
};

#define SINK_DATA_BAADBAAD  1

/* Bit of a CPU and SRAM in the SRAM MMU flags: APP SRAM0, PRO SRAM0, ... */
#define SRAM_FLAG(sram, cpu)    ((sram) * 2 + ((cpu) == 0 ? 1 : 0))

/* DBUG0 MEM_ACCESS_ILLEGAL bit: 2/3 for APP/PRO SRAM0, 8/9 for SRAM2 */
#define SRAM_ILLEGAL_FLAG(sram, cpu) \
    (((sram) ? 8 : 2) + ((cpu) == 0 ? 1 : 0))

/* x_DCACHE_DBUG0 CACHE_ACCESS_ILLEGAL bits, as bit numbers in the register */
#define CACHE_IA_VADDR3     3
#define CACHE_IA_VADDR2     4
#define CACHE_IA_VADDR1     5
#define CACHE_IA_VADDR4     6
#define CACHE_MMU_IA        0

/* x_DCACHE_DBUG3 CPU_DISABLED_CACHE_IA bits for the ports */
#define CACHE_DIS_IROM0     11
#define CACHE_DIS_IRAM1     12

/* DPORT_CACHE_IA_INT_EN enable bits for those, PRO; APP is 14 lower */
#define CACHE_EN_IROM0      17
#define CACHE_EN_IRAM1      16

#define MMU_ENTRY_INVALID   0x100
#define MMU_ENTRY_PAGE      0xff

/* Virtual regions of the external memory cache (TRM table 4.3-9) */
#define VADDR0_BASE         0x3f400000
#define VADDR4_SIZE         0x100000
#define VADDRRAM_BASE       0x3f800000
#define VADDR1_BASE         0x40000000
#define VADDR1_CACHE_START  0x400c2000
#define VADDR2_BASE         0x40400000
#define VADDR3_BASE         0x40800000
#define VADDR_REGION_SIZE   0x400000
#define PSRAM_PAGE_SIZE     0x8000

#define PERIPH_BASE         0x3ff40000
#define PERIPH_AHB_BASE     0x60000000
#define PERIPH_SIZE         0x40000
#define PIDCTRL_BASE        0x3ff1f000

/*
 * DPORT_AHBLITE_MPU_TABLE_x_REG index (offset 0x32C + 4 * index) plus one
 * for each 4 KB peripheral slot from 0x3FF40000, or 0 for a slot no
 * register governs (TRM table 4.3-19, ESP-IDF reg_base.h and dport_reg.h).
 */
#define AHBLITE(off)    ((((off) - A_DPORT_AHBLITE_MPU_TABLE_UART) / 4) + 1)
static const uint8_t ahblite_slot[PERIPH_SIZE / MPU_PAGE_SIZE] = {
    [0x00] = AHBLITE(0x32c),    /* UART0 */
    [0x02] = AHBLITE(0x330),    /* SPI1 */
    [0x03] = AHBLITE(0x334),    /* SPI0 */
    [0x04] = AHBLITE(0x338),    /* GPIO */
    [0x05] = AHBLITE(0x33c),    /* FE2 */
    [0x06] = AHBLITE(0x340),    /* FE */
    [0x07] = AHBLITE(0x344),    /* FRC timers */
    [0x08] = AHBLITE(0x348),    /* RTC */
    [0x09] = AHBLITE(0x34c),    /* IO MUX */
    [0x0b] = AHBLITE(0x354),    /* SDIO slave HINF */
    [0x0c] = AHBLITE(0x358),    /* UHCI1 */
    [0x0f] = AHBLITE(0x364),    /* I2S0 */
    [0x10] = AHBLITE(0x368),    /* UART1 */
    [0x11] = AHBLITE(0x36c),    /* BT */
    [0x13] = AHBLITE(0x374),    /* I2C0 */
    [0x14] = AHBLITE(0x378),    /* UHCI0 */
    [0x15] = AHBLITE(0x37c),    /* SDIO slave SLCHOST */
    [0x16] = AHBLITE(0x380),    /* RMT */
    [0x17] = AHBLITE(0x384),    /* PCNT */
    [0x18] = AHBLITE(0x388),    /* SDIO slave SLC */
    [0x19] = AHBLITE(0x38c),    /* LED PWM */
    [0x1a] = AHBLITE(0x390),    /* eFuse */
    [0x1b] = AHBLITE(0x394),    /* flash encryption */
    [0x1d] = AHBLITE(0x398),    /* BB */
    [0x1e] = AHBLITE(0x39c),    /* PWM0 */
    [0x1f] = AHBLITE(0x3a0),    /* TIMG0 */
    [0x20] = AHBLITE(0x3a4),    /* TIMG1 */
    [0x24] = AHBLITE(0x3a8),    /* SPI2 */
    [0x25] = AHBLITE(0x3ac),    /* SPI3 */
    [0x26] = AHBLITE(0x3b0),    /* SYSCON */
    [0x27] = AHBLITE(0x3b4),    /* I2C1 */
    [0x28] = AHBLITE(0x3b8),    /* SDMMC */
    [0x29] = AHBLITE(0x3bc),    /* EMAC */
    [0x2a] = AHBLITE(0x3bc),
    [0x2b] = AHBLITE(0x3c0),    /* TWAI */
    [0x2c] = AHBLITE(0x3c4),    /* PWM1 */
    [0x2d] = AHBLITE(0x3c8),    /* I2S1 */
    [0x2e] = AHBLITE(0x3cc),    /* UART2 */
    [0x2f] = AHBLITE(0x3d0),    /* PWM2 */
    [0x30] = AHBLITE(0x3d4),    /* PWM3 */
    [0x35] = AHBLITE(0x3e4),    /* RNG */
};

static void sink_map(Esp32MpuMapping *map, uint32_t vaddr, unsigned reg,
                     unsigned bit, bool baadbaad)
{
    unsigned page = (((reg << 5) | bit) << 1) | (baadbaad ? 1 : 0);

    map->paddr = ESP32_MPU_SINK_BASE + (page << MPU_PAGE_BITS) +
                 (vaddr & (MPU_PAGE_SIZE - 1));
    map->page_size = MPU_PAGE_SIZE;
}

static void remap(Esp32MpuMapping *map, hwaddr paddr, uint64_t page_size)
{
    map->paddr = paddr;
    map->page_size = MIN(page_size, MPU_PAGE_SIZE);
}

static void mpu_update_irqs(Esp32DportState *s)
{
    uint32_t mpu = (s->mem_access_dbug0 >> 6) & s->mpu_ia_int_en &
                   0x00fffff0;
    uint32_t mmu = (s->mem_access_dbug1 >> 4) & s->mmu_ia_int_en & 0x1c;

    qemu_set_irq(s->mpu_ia_irq, mpu != 0);
    qemu_set_irq(s->mmu_ia_irq, mmu != 0);
}

/*
 * The x_DCACHE_DBUG0 flags are enabled at bits 21 and 27:22 (PRO), 7 and
 * 13:8 (APP) of DPORT_CACHE_IA_INT_EN.
 */
static uint32_t cache_ia_enabled(Esp32DportState *s, int cpu)
{
    uint32_t en = s->cache_ill_trap_en_reg;

    if (cpu == 0) {
        return (FIELD_EX32(en, DPORT_CACHE_IA_INT_EN,
                           PRO_CACHE_MMU_ILLEGAL) << CACHE_MMU_IA) |
               (FIELD_EX32(en, DPORT_CACHE_IA_INT_EN,
                           PRO_CACHE_ACCESS_ILLEGAL) << 1);
    }
    return (FIELD_EX32(en, DPORT_CACHE_IA_INT_EN,
                       APP_CACHE_MMU_ILLEGAL) << CACHE_MMU_IA) |
           (FIELD_EX32(en, DPORT_CACHE_IA_INT_EN,
                       APP_CACHE_ACCESS_ILLEGAL) << 1);
}

void esp32_dport_update_cache_irq(Esp32DportState *s)
{
    bool level = false;

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        Esp32CacheState *cs = &s->cache_state[i];

        level |= cs->drom0.illegal_access_status ||
                 cs->iram0.illegal_access_status ||
                 cs->dram1.illegal_access_status ||
                 cs->cache_dis_ia != 0 ||
                 (cs->cache_ia & cache_ia_enabled(s, i)) != 0;
    }
    qemu_set_irq(s->cache_ill_irq, level);
}

static void mpu_changed(Esp32DportState *s)
{
    qemu_irq_pulse(s->mpu_update_req);
}

static uint64_t sink_data(hwaddr addr, unsigned size)
{
    static const uint32_t baad[] = { 0xbaadbaad, 0xbaadbaad };
    uint32_t r = 0;

    if ((addr >> MPU_PAGE_BITS) & SINK_DATA_BAADBAAD) {
        memcpy(&r, (const uint8_t *)baad + (addr & 3), size);
    }
    return r;
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * A refused access: set its flag, raise the interrupt it is enabled
 * onto, and answer without touching memory.
 */
static MemTxResult sink_access(Esp32DportState *s, hwaddr addr,
                               bool is_write)
{
    unsigned kind = addr >> (MPU_PAGE_BITS + 1);
    unsigned reg = kind >> 5;
    unsigned bit = kind & 31;
    Esp32CacheState *cs;

    switch (reg) {
    case SINK_DECODE_ERROR:
        return MEMTX_DECODE_ERROR;
    case SINK_SILENT:
        break;
    case SINK_DBUG0:
        s->mem_access_dbug0 |= BIT(bit);
        break;
    case SINK_DBUG1:
        s->mem_access_dbug1 |= BIT(bit);
        break;
    case SINK_SRAM_MISS:
        s->mem_access_dbug1 |= BIT(bit);
        s->mem_access_dbug0 |= BIT(R_DPORT_MEM_ACCESS_DBUG0_ACCESS_DENY_SHIFT +
                                   bit);
        break;
    case SINK_CACHE_DBUG0:
    case SINK_CACHE_DBUG0_APP:
        cs = &s->cache_state[reg - SINK_CACHE_DBUG0];
        cs->cache_ia |= BIT(bit);
        break;
    case SINK_CACHE_DISABLED:
    case SINK_CACHE_DISABLED_APP: {
        int cpu = reg - SINK_CACHE_DISABLED;
        unsigned en = (bit == CACHE_DIS_IROM0 ? CACHE_EN_IROM0 :
                       CACHE_EN_IRAM1) - (cpu ? 14 : 0);

        if (s->cache_ill_trap_en_reg & BIT(en)) {
            s->cache_state[cpu].cache_dis_ia |= BIT(bit);
        }
        break;
    }
    default:
        g_assert_not_reached();
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32: CPU%d %s refused for its process ID "
                  "(flag %u.%u, page offset 0x%03x)\n",
                  current_cpu ? current_cpu->cpu_index : -1,
                  is_write ? "write" : "read", reg, bit,
                  (unsigned)(addr & (MPU_PAGE_SIZE - 1)));
    mpu_update_irqs(s);
    esp32_dport_update_cache_irq(s);
    return MEMTX_OK;
}

static MemTxResult sink_read(void *opaque, hwaddr addr, uint64_t *data,
                             unsigned size, MemTxAttrs attrs)
{
    MemTxResult r = sink_access(opaque, addr, false);

    *data = sink_data(addr, size);
    return r;
}

static MemTxResult sink_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size, MemTxAttrs attrs)
{
    return sink_access(opaque, addr, true);
}

static const MemoryRegionOps sink_ops = {
    .read_with_attrs = sink_read,
    .write_with_attrs = sink_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static unsigned sram_page_bits(uint32_t mode_reg)
{
    switch (FIELD_EX32(mode_reg, DPORT_IMMU_PAGE_MODE, PAGE_MODE)) {
    case 0:
        return 13;
    case 1:
        return 12;
    default:
        /* 2 is 2 KB pages; 3 is reserved and taken as 2 KB too. */
        return 11;
    }
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * Translate an offset in SRAM0's (sram 0) or SRAM2's (sram 1) upper
 * 128 KB for a process with PID 2 to 7 (TRM 4.3.2.1). Each table entry
 * describes a physical page: the virtual page that reaches it and which
 * PIDs may. Returns true with the physical offset, or false with the
 * sink flag that refuses the access.
 */
static bool sram_mmu_translate(Esp32DportState *s, unsigned sram, int cpu,
                               unsigned pid, uint32_t off, bool is_write,
                               uint32_t *phys_off, unsigned *sink_reg,
                               unsigned *sink_bit)
{
    const uint32_t *table = sram ? s->dmmu_table : s->immu_table;
    unsigned bits = sram_page_bits(sram ? s->dmmu_page_mode :
                                          s->immu_page_mode);
    unsigned vpage = off >> bits;
    unsigned named = 0, hits = 0, ppage = 0;

    if (vpage >= ESP32_SRAM_MMU_PAGES) {
        /* Beyond the 16 pages: open to PIDs 0 and 1 only. */
        *sink_reg = SINK_DBUG0;
        *sink_bit = R_DPORT_MEM_ACCESS_DBUG0_ACCESS_ILLEGAL_SHIFT +
                    SRAM_ILLEGAL_FLAG(sram, cpu);
        return false;
    }
    for (unsigned p = 0; p < ESP32_SRAM_MMU_PAGES; ++p) {
        unsigned rights = FIELD_EX32(table[p], DPORT_MMU_TABLE, RIGHTS);

        if (FIELD_EX32(table[p], DPORT_MMU_TABLE, VPAGE) != vpage) {
            continue;
        }
        named++;
        if (rights == 1 || rights == pid) {
            hits++;
            ppage = p;
        }
    }
    if (hits > 1) {
        *sink_reg = SINK_DBUG0;
        *sink_bit = R_DPORT_MEM_ACCESS_DBUG0_MMU_MULTI_HIT_SHIFT +
                    SRAM_FLAG(sram, cpu);
        return false;
    }
    if (named == 0) {
        *sink_reg = SINK_SRAM_MISS;
        *sink_bit = SRAM_FLAG(sram, cpu);
        return false;
    }
    if (hits == 0 || (sram == 0 && is_write)) {
        /* SRAM0's pages are read-only to PIDs 2 to 7. */
        *sink_reg = SINK_DBUG0;
        *sink_bit = R_DPORT_MEM_ACCESS_DBUG0_ACCESS_DENY_SHIFT +
                    SRAM_FLAG(sram, cpu);
        return false;
    }
    *phys_off = (ppage << bits) | (off & ((1u << bits) - 1));
    return true;
}

/*
 * 2 KB SRAM MMU pages are smaller than the TLB's, so the TLB maps them to
 * this window, which translates every access as it happens.
 */
static MemTxResult redirect_access(Esp32DportState *s, hwaddr addr,
                                   uint8_t *buf, unsigned size,
                                   bool is_write, MemTxAttrs attrs)
{
    unsigned sram = addr / ESP32_SRAM_MMU_SIZE;
    uint32_t off = addr % ESP32_SRAM_MMU_SIZE;
    hwaddr base = sram ? ESP32_SRAM2_MMU_BASE : ESP32_SRAM0_MMU_BASE;
    int cpu = current_cpu ? current_cpu->cpu_index : 0;
    unsigned pid = s->get_pid ? s->get_pid(s->get_pid_opaque, cpu) : 0;
    unsigned reg, bit;
    uint32_t phys;

    if (pid <= 1) {
        phys = off;
    } else if (!sram_mmu_translate(s, sram, cpu, pid, off, is_write, &phys,
                                   &reg, &bit)) {
        hwaddr sink = (((reg << 5) | bit) << (MPU_PAGE_BITS + 1)) |
                      (off & (MPU_PAGE_SIZE - 1));

        if (!is_write) {
            stn_le_p(buf, size, sink_data(sink, size));
        }
        return sink_access(s, sink, is_write);
    }
    if (is_write) {
        return address_space_write(&address_space_memory, base + phys,
                                   attrs, buf, size);
    }
    return address_space_read(&address_space_memory, base + phys, attrs,
                              buf, size);
}

static MemTxResult redirect_read(void *opaque, hwaddr addr, uint64_t *data,
                                 unsigned size, MemTxAttrs attrs)
{
    uint8_t buf[4];
    MemTxResult r = redirect_access(opaque, addr, buf, size, false, attrs);

    *data = ldn_le_p(buf, size);
    return r;
}

static MemTxResult redirect_write(void *opaque, hwaddr addr, uint64_t data,
                                  unsigned size, MemTxAttrs attrs)
{
    uint8_t buf[4];

    stn_le_p(buf, size, data);
    return redirect_access(opaque, addr, buf, size, true, attrs);
}

static const MemoryRegionOps redirect_ops = {
    .read_with_attrs = redirect_read,
    .write_with_attrs = redirect_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void sram_mmu_filter(Esp32DportState *s, unsigned sram, int cpu,
                            unsigned pid, uint32_t vaddr,
                            MMUAccessType access_type, Esp32MpuMapping *map)
{
    hwaddr base = sram ? ESP32_SRAM2_MMU_BASE : ESP32_SRAM0_MMU_BASE;
    unsigned bits = sram_page_bits(sram ? s->dmmu_page_mode :
                                          s->immu_page_mode);
    bool is_write = access_type == MMU_DATA_STORE;
    uint32_t off = vaddr - base;
    unsigned reg, bit;
    uint32_t phys;

    if (bits < MPU_PAGE_BITS && (off >> bits) < ESP32_SRAM_MMU_PAGES) {
        remap(map, ESP32_MPU_REDIRECT_BASE + sram * ESP32_SRAM_MMU_SIZE + off,
              MPU_PAGE_SIZE);
        return;
    }
    if (!sram_mmu_translate(s, sram, cpu, pid, off, is_write, &phys, &reg,
                            &bit)) {
        sink_map(map, vaddr, reg, bit, false);
        if (sram == 0 && is_write && reg == SINK_DBUG0 &&
            bit == R_DPORT_MEM_ACCESS_DBUG0_ACCESS_DENY_SHIFT +
                   SRAM_FLAG(0, cpu) &&
            sram_mmu_translate(s, 0, cpu, pid, off, false, &phys, &reg,
                               &bit)) {
            /* Readable: only stores go to the sink. */
            map->prot &= PAGE_WRITE;
        }
        return;
    }
    remap(map, base + phys, 1u << bits);
    if (sram == 0) {
        map->prot &= ~PAGE_WRITE;
    }
}

static uint32_t cache_mmu_entry(Esp32DportState *s, int cpu, unsigned idx)
{
    Esp32CacheState *cs = &s->cache_state[cpu];

    if (idx < ESP32_CACHE_PAGES_PER_REGION) {
        return cs->drom0.mmu_table[idx] & 0x1ff;
    } else if (idx < 2 * ESP32_CACHE_PAGES_PER_REGION) {
        return cs->iram0.mmu_table[idx - ESP32_CACHE_PAGES_PER_REGION] &
               0x1ff;
    }
    return s->cache_mmu_table[cpu][idx];
}

/* Load a 64 KB flash page into the page store, decrypted if need be. */
static hwaddr flash_page(Esp32DportState *s, unsigned page)
{
    if (!test_bit(page, s->flash_page_valid)) {
        BQL_LOCK_GUARD();

        if (!test_bit(page, s->flash_page_valid)) {
            Esp32FlashEncryptionState *enc = esp32_flash_encryption_find();
            uint8_t *data = memory_region_get_ram_ptr(&s->flash_pages);
            uint32_t *buf = (uint32_t *)(data + page * ESP32_CACHE_PAGE_SIZE);
            hwaddr phys = (hwaddr)page * ESP32_CACHE_PAGE_SIZE;

            if (blk_pread(s->flash_blk, phys, ESP32_CACHE_PAGE_SIZE, buf,
                          0) < 0) {
                memset(buf, 0xff, ESP32_CACHE_PAGE_SIZE);
            } else if (enc && esp32_flash_decryption_enabled(enc)) {
                esp32_flash_decrypt_inplace(enc, phys, buf,
                                            ESP32_CACHE_PAGE_SIZE / 4);
            }
            set_bit(page, s->flash_page_valid);
        }
    }
    return ESP32_FLASH_PAGES_BASE + (hwaddr)page * ESP32_CACHE_PAGE_SIZE;
}

/*
 * Cache flushes discard what the cache holds; the page store, which serves
 * every CPU, is reloaded from flash as pages are mapped again.
 */
void esp32_dport_mpu_cache_flushed(Esp32DportState *s)
{
    if (!s->flash_blk) {
        return;
    }
    bitmap_zero(s->flash_page_valid, ESP32_FLASH_PAGES);
    memory_region_flush_rom_device(&s->flash_pages, 0,
                                   ESP32_FLASH_PAGES_SIZE);
    mpu_changed(s);
}

static bool cache_port_enabled(Esp32CacheState *cs, unsigned mask_shift)
{
    return FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL, CACHE_ENA) &&
           !(cs->cache_ctrl1_reg & BIT(mask_shift));
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * The flash cache regions the per-CPU cache regions do not model: VAddr2
 * and VAddr3 for every PID, VAddr4, and VAddr0 and VAddr1 for PIDs 2 to 7
 * (TRM 4.3.2.2, tables 4.3-10 to 4.3-13).
 */
static void flash_cache_filter(Esp32DportState *s, int cpu, unsigned pid,
                               uint32_t vaddr, Esp32MpuMapping *map)
{
    Esp32CacheState *cs = &s->cache_state[cpu];
    bool single = FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL,
                             SINGLE_IRAM_ENA);
    unsigned cache_reg = SINK_CACHE_DBUG0 + cpu;
    unsigned cpu_base = cpu ? ESP32_CACHE_MMU_ENTRIES : 0;
    unsigned first, page, mask_shift;
    bool data = vaddr < VADDR1_BASE;
    uint32_t entry;

    page = (vaddr % VADDR_REGION_SIZE) / ESP32_CACHE_PAGE_SIZE;
    if (vaddr < VADDRRAM_BASE) {
        if (pid <= 1) {
            return;
        }
        if (vaddr >= VADDR0_BASE + VADDR4_SIZE) {
            sink_map(map, vaddr, cache_reg, CACHE_IA_VADDR4, true);
            return;
        }
        first = 1056 + 16 * (pid - 2);
        mask_shift = R_DPORT_PRO_CACHE_CTRL1_MASK_DROM0_SHIFT;
    } else if (vaddr < VADDR2_BASE) {
        if (pid <= 1) {
            return;
        }
        if (!single) {
            sink_map(map, vaddr, cache_reg, CACHE_IA_VADDR1, false);
            return;
        }
        first = 256 + 128 * (pid - 2);
        mask_shift = R_DPORT_PRO_CACHE_CTRL1_MASK_IRAM0_SHIFT;
    } else {
        bool vaddr3 = vaddr >= VADDR3_BASE;

        if (single) {
            /* Special mode: VAddr2 and VAddr3 do not reach the flash. */
            sink_map(map, vaddr, cache_reg,
                     vaddr3 ? CACHE_IA_VADDR3 : CACHE_IA_VADDR2, false);
            return;
        }
        first = (pid <= 1 ? 128 : 256 + 128 * (pid - 2)) + (vaddr3 ? 64 : 0);
        mask_shift = vaddr3 ? R_DPORT_PRO_CACHE_CTRL1_MASK_IROM0_SHIFT :
                              R_DPORT_PRO_CACHE_CTRL1_MASK_IRAM1_SHIFT;
        if (!cache_port_enabled(cs, mask_shift)) {
            sink_map(map, vaddr,
                     SINK_CACHE_DISABLED + cpu,
                     vaddr3 ? CACHE_DIS_IROM0 : CACHE_DIS_IRAM1, false);
            return;
        }
    }
    if (!cache_port_enabled(cs, mask_shift)) {
        /* The per-CPU cache region reports the disabled cache. */
        return;
    }
    entry = cache_mmu_entry(s, cpu, cpu_base + first + page);
    if (entry & MMU_ENTRY_INVALID) {
        sink_map(map, vaddr, cache_reg, CACHE_MMU_IA, data);
        return;
    }
    remap(map, flash_page(s, entry & MMU_ENTRY_PAGE) +
               (vaddr % ESP32_CACHE_PAGE_SIZE), ESP32_CACHE_PAGE_SIZE);
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * External RAM for PIDs 2 to 7, through their own MMU entries (TRM table
 * 4.3-18) onto the PSRAM, which PIDs 0 and 1 reach one to one.
 */
static void psram_cache_filter(Esp32DportState *s, int cpu, unsigned pid,
                               uint32_t vaddr, Esp32MpuMapping *map)
{
    Esp32CacheState *cs = &s->cache_state[cpu];
    unsigned page = (vaddr - VADDRRAM_BASE) / PSRAM_PAGE_SIZE;
    bool high_low = cpu ? FIELD_EX32(cs->cache_ctrl_reg, DPORT_APP_CACHE_CTRL,
                                     DRAM_HL)
                        : FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL,
                                     DRAM_HL);
    unsigned table_cpu = cpu;
    unsigned idx;
    uint32_t entry;

    if (pid <= 1 || !s->has_psram ||
        !cache_port_enabled(cs, R_DPORT_PRO_CACHE_CTRL1_MASK_DRAM1_SHIFT)) {
        return;
    }
    if (FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL, DRAM_SPLIT)) {
        qemu_log_mask(LOG_UNIMP, "esp32: PSRAM even-odd mode is not "
                      "modelled; mapping as in normal mode\n");
    }
    if (high_low) {
        /* Both CPUs: the low 2 MB through L entries, the high through R. */
        table_cpu = page >= 64;
    }
    idx = 1152 + 128 * (pid - 1) + page;
    entry = s->cache_mmu_table[table_cpu][idx];
    if (entry & MMU_ENTRY_INVALID) {
        sink_map(map, vaddr, SINK_CACHE_DBUG0 + cpu, CACHE_MMU_IA, true);
        return;
    }
    /* The modelled PSRAM is 4 MB; its address lines wrap above that. */
    remap(map, VADDRRAM_BASE +
               ((entry & MMU_ENTRY_PAGE) % (ESP32_CACHE_REGION_SIZE /
                                            PSRAM_PAGE_SIZE)) *
               PSRAM_PAGE_SIZE + (vaddr % PSRAM_PAGE_SIZE), PSRAM_PAGE_SIZE);
}

static void periph_filter(Esp32DportState *s, int cpu, unsigned pid,
                          uint32_t vaddr, uint32_t off, Esp32MpuMapping *map)
{
    unsigned table = ahblite_slot[off / MPU_PAGE_SIZE];

    if (table && (s->ahblite_mpu_table[table - 1] & BIT(pid - 2))) {
        return;
    }
    sink_map(map, vaddr, SINK_DBUG1,
             R_DPORT_MEM_ACCESS_DBUG1_AHBLITE_ACCESS_DENY_SHIFT, false);
}

static bool in(uint32_t addr, uint32_t base, uint32_t size)
{
    return addr - base < size;
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * Decide a CPU access for the process running on the CPU, as its TLB is
 * filled. map holds the core's mapping; on return it holds the mapping
 * the TLB is to use.
 */
void esp32_dport_mpu_filter(Esp32DportState *s, int cpu, unsigned pid,
                            uint32_t vaddr, MMUAccessType access_type,
                            Esp32MpuMapping *map)
{
    uint32_t paddr = map->paddr;

    if (in(paddr, ESP32_MPU_HIDDEN_BASE,
           ESP32_MPU_HIDDEN_END - ESP32_MPU_HIDDEN_BASE)) {
        sink_map(map, vaddr, SINK_DECODE_ERROR, 0, false);
        return;
    }
    if (in(paddr, VADDR0_BASE, 2 * VADDR_REGION_SIZE) ||
        in(paddr, VADDR1_CACHE_START, VADDR3_BASE + VADDR_REGION_SIZE -
                                      VADDR1_CACHE_START)) {
        if (in(paddr, VADDRRAM_BASE, VADDR_REGION_SIZE)) {
            psram_cache_filter(s, cpu, pid, paddr, map);
        } else if (s->flash_blk) {
            flash_cache_filter(s, cpu, pid, paddr, map);
        }
        return;
    }
    if (pid <= 1) {
        return;
    }

    /* Processes 2 to 7 (TRM 4.3.2.1 and 4.3.2.3). */
    if (in(paddr, ESP32_SRAM0_MMU_BASE, ESP32_SRAM_MMU_SIZE)) {
        sram_mmu_filter(s, 0, cpu, pid, paddr, access_type, map);
    } else if (in(paddr, ESP32_SRAM2_MMU_BASE, ESP32_SRAM_MMU_SIZE)) {
        sram_mmu_filter(s, 1, cpu, pid, paddr, access_type, map);
    } else if (in(paddr, 0x40070000, 0x10000)) {
        /* SRAM0's lower 64 KB, behind the static MPU */
        sink_map(map, vaddr, SINK_DBUG0,
                 R_DPORT_MEM_ACCESS_DBUG0_ACCESS_DENY_SHIFT +
                 SRAM_FLAG(0, cpu), false);
    } else if (in(paddr, 0x3ffae000, 0x12000)) {
        /* SRAM2's lower 72 KB, behind the static MPU */
        sink_map(map, vaddr, SINK_DBUG0,
                 R_DPORT_MEM_ACCESS_DBUG0_ACCESS_DENY_SHIFT +
                 SRAM_FLAG(1, cpu), false);
    } else if (in(paddr, 0x40000000, 0x60000) ||   /* ROM0 */
               in(paddr, 0x3ff90000, 0x10000) ||   /* ROM1 */
               in(paddr, 0x3ffe0000, 0x20000) ||   /* SRAM1, data bus */
               in(paddr, 0x400a0000, 0x20000) ||   /* SRAM1, instruction bus */
               in(paddr, 0x3ff80000, 0x2000) ||    /* RTC FAST */
               in(paddr, 0x400c0000, 0x2000)) {
        sink_map(map, vaddr, SINK_SILENT, 0, false);
    } else if (in(paddr, 0x50000000, 0x2000)) {
        /* RTC SLOW, governed with the RTC registers */
        if (!(s->ahblite_mpu_table[AHBLITE(A_DPORT_AHBLITE_MPU_TABLE_RTC) - 1] &
              BIT(pid - 2))) {
            sink_map(map, vaddr, SINK_SILENT, 0, false);
        }
    } else if (in(paddr, PIDCTRL_BASE, MPU_PAGE_SIZE)) {
        sink_map(map, vaddr, SINK_DBUG1,
                 R_DPORT_MEM_ACCESS_DBUG1_ACCESS_PID_ILLEGAL_SHIFT +
                 (cpu == 0 ? 1 : 0), false);
    } else if (in(paddr, DR_REG_DPORT_BASE, PERIPH_BASE - DR_REG_DPORT_BASE)) {
        /* DPORT, the accelerators, secure boot and the cache MMU table */
        sink_map(map, vaddr, SINK_DBUG1,
                 R_DPORT_MEM_ACCESS_DBUG1_AHBLITE_ACCESS_DENY_SHIFT, false);
    } else if (in(paddr, PERIPH_BASE, PERIPH_SIZE)) {
        periph_filter(s, cpu, pid, vaddr, paddr - PERIPH_BASE, map);
    } else if (in(paddr, PERIPH_AHB_BASE, PERIPH_SIZE)) {
        periph_filter(s, cpu, pid, vaddr, paddr - PERIPH_AHB_BASE, map);
    }
}

bool esp32_dport_mpu_read(Esp32DportState *s, hwaddr addr, uint64_t *value)
{
    switch (addr) {
    case A_DPORT_IMMU_PAGE_MODE:
        *value = s->immu_page_mode;
        return true;
    case A_DPORT_DMMU_PAGE_MODE:
        *value = s->dmmu_page_mode;
        return true;
    case A_DPORT_AHB_MPU_TABLE_0:
    case A_DPORT_AHB_MPU_TABLE_1:
        *value = s->ahb_mpu_table[(addr - A_DPORT_AHB_MPU_TABLE_0) / 4];
        return true;
    case A_DPORT_AHBLITE_MPU_TABLE_UART ... A_DPORT_AHBLITE_MPU_TABLE_PWR:
        *value = s->ahblite_mpu_table[(addr - A_DPORT_AHBLITE_MPU_TABLE_UART) /
                                      4];
        return true;
    case A_DPORT_MEM_ACCESS_DBUG0:
        *value = s->mem_access_dbug0;
        return true;
    case A_DPORT_MEM_ACCESS_DBUG1:
        *value = s->mem_access_dbug1;
        return true;
    case A_DPORT_IMMU_TABLE0 ... A_DPORT_IMMU_TABLE15:
        *value = s->immu_table[(addr - A_DPORT_IMMU_TABLE0) / 4];
        return true;
    case A_DPORT_DMMU_TABLE0 ... A_DPORT_DMMU_TABLE15:
        *value = s->dmmu_table[(addr - A_DPORT_DMMU_TABLE0) / 4];
        return true;
    case A_DPORT_MMU_IA_INT_EN:
        *value = s->mmu_ia_int_en;
        return true;
    case A_DPORT_MPU_IA_INT_EN:
        *value = s->mpu_ia_int_en;
        return true;
    case DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE ...
         DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE +
         ESP32_CACHE_MMU_ENTRIES * 4 - 1:
        *value = cache_mmu_entry(s, 0, (addr - (DR_REG_FLASH_MMU_TABLE_PRO -
                                                DR_REG_DPORT_BASE)) / 4);
        return true;
    case DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE ...
         DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE +
         ESP32_CACHE_MMU_ENTRIES * 4 - 1:
        *value = cache_mmu_entry(s, 1, (addr - (DR_REG_FLASH_MMU_TABLE_APP -
                                                DR_REG_DPORT_BASE)) / 4);
        return true;
    }
    return false;
}

/*
 * [spec:nuos:req:emu.esp32.memory-protection]
 * Configuration writes. Only PIDs 0 and 1 reach DPORT, so no further PID
 * check is needed here.
 */
bool esp32_dport_mpu_write(Esp32DportState *s, hwaddr addr, uint64_t value)
{
    switch (addr) {
    case A_DPORT_IMMU_PAGE_MODE:
        s->immu_page_mode = value & 0x7;
        break;
    case A_DPORT_DMMU_PAGE_MODE:
        s->dmmu_page_mode = value & 0x7;
        break;
    case A_DPORT_AHB_MPU_TABLE_0:
        s->ahb_mpu_table[0] = value;
        qemu_log_mask(LOG_UNIMP, "esp32: the DMA MPU is not modelled\n");
        return true;
    case A_DPORT_AHB_MPU_TABLE_1:
        s->ahb_mpu_table[1] = value & 0x1ff;
        qemu_log_mask(LOG_UNIMP, "esp32: the DMA MPU is not modelled\n");
        return true;
    case A_DPORT_AHBLITE_MPU_TABLE_UART ... A_DPORT_AHBLITE_MPU_TABLE_PWR:
        s->ahblite_mpu_table[(addr - A_DPORT_AHBLITE_MPU_TABLE_UART) / 4] =
            value & 0x3f;
        break;
    case A_DPORT_MEM_ACCESS_DBUG0:
    case A_DPORT_MEM_ACCESS_DBUG1:
        return true;
    case A_DPORT_IMMU_TABLE0 ... A_DPORT_IMMU_TABLE15:
        s->immu_table[(addr - A_DPORT_IMMU_TABLE0) / 4] = value & 0x7f;
        break;
    case A_DPORT_DMMU_TABLE0 ... A_DPORT_DMMU_TABLE15:
        s->dmmu_table[(addr - A_DPORT_DMMU_TABLE0) / 4] = value & 0x7f;
        break;
    case A_DPORT_MMU_IA_INT_EN:
        s->mmu_ia_int_en = value & 0x00ffffff;
        /* Leaving an enable clear clears its flag; see the top comment. */
        s->mem_access_dbug1 &= ~((~value & 0x1c) << 4);
        s->mem_access_dbug1 &=
            ~R_DPORT_MEM_ACCESS_DBUG1_AHBLITE_ACCESS_DENY_MASK;
        mpu_update_irqs(s);
        return true;
    case A_DPORT_MPU_IA_INT_EN:
        s->mpu_ia_int_en = value & 0x00ffffff;
        s->mem_access_dbug0 &= ~((~value & 0x00fffff0) << 6);
        s->mem_access_dbug1 &= ~(~(value >> 4) &
                                 R_DPORT_MEM_ACCESS_DBUG1_ACCESS_MISS_MASK);
        mpu_update_irqs(s);
        return true;
    case DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE +
         2 * ESP32_CACHE_PAGES_PER_REGION * 4 ...
         DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE +
         ESP32_CACHE_MMU_ENTRIES * 4 - 1:
        s->cache_mmu_table[0][(addr - (DR_REG_FLASH_MMU_TABLE_PRO -
                                       DR_REG_DPORT_BASE)) / 4] =
            value & 0x1ff;
        break;
    case DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE +
         2 * ESP32_CACHE_PAGES_PER_REGION * 4 ...
         DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE +
         ESP32_CACHE_MMU_ENTRIES * 4 - 1:
        s->cache_mmu_table[1][(addr - (DR_REG_FLASH_MMU_TABLE_APP -
                                       DR_REG_DPORT_BASE)) / 4] =
            value & 0x1ff;
        break;
    default:
        return false;
    }
    mpu_changed(s);
    return true;
}

void esp32_dport_mpu_reset(Esp32DportState *s)
{
    s->immu_page_mode = 0;
    s->dmmu_page_mode = 0;
    for (unsigned i = 0; i < ESP32_SRAM_MMU_PAGES; ++i) {
        /* Reset values from the TRM, registers 12.197 and 12.198 */
        s->immu_table[i] = i < 10 ? 0 : i;
        s->dmmu_table[i] = i;
    }
    s->ahb_mpu_table[0] = 0xffffffff;
    s->ahb_mpu_table[1] = 0x1ff;
    memset(s->ahblite_mpu_table, 0, sizeof(s->ahblite_mpu_table));
    s->mem_access_dbug0 = 0;
    s->mem_access_dbug1 = 0;
    s->mmu_ia_int_en = 0;
    s->mpu_ia_int_en = 0;
    memset(s->cache_mmu_table, 0, sizeof(s->cache_mmu_table));
    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        s->cache_state[i].cache_ia = 0;
        s->cache_state[i].cache_dis_ia = 0;
    }
    bitmap_zero(s->flash_page_valid, ESP32_FLASH_PAGES);
    mpu_update_irqs(s);
    mpu_changed(s);
}

static bool flash_pages_accepts(void *opaque, hwaddr addr, unsigned size,
                                bool is_write, MemTxAttrs attrs)
{
    return !is_write;
}

static const MemoryRegionOps flash_pages_rom_ops = {
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.accepts = flash_pages_accepts,
};

void esp32_dport_mpu_init(Esp32DportState *s)
{
    Object *obj = OBJECT(s);

    memory_region_init_io(&s->mpu_sink, obj, &sink_ops, s, "esp32.mpu-sink",
                          ESP32_MPU_SINK_SIZE);
    memory_region_init_io(&s->mpu_redirect, obj, &redirect_ops, s,
                          "esp32.sram-mmu-2k", ESP32_MPU_REDIRECT_SIZE);
    memory_region_init_rom_device(&s->flash_pages, obj, &flash_pages_rom_ops,
                                  s, "esp32.flash-cache-pages",
                                  ESP32_FLASH_PAGES_SIZE, &error_fatal);
}
