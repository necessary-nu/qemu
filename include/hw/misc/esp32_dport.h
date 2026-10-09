#pragma once

#include "qemu/bitmap.h"
#include "exec/mmu-access-type.h"
#include "hw/core/registerfields.h"
#include "hw/core/sysbus.h"
#include "system/block-backend.h"
#include "hw/misc/esp32_flash_enc.h"

typedef struct Esp32DportState Esp32DportState;
typedef struct Esp32CacheState Esp32CacheState;

#define TYPE_ESP32_DPORT "misc.esp32.dport"
#define ESP32_DPORT(obj) OBJECT_CHECK(Esp32DportState, (obj), TYPE_ESP32_DPORT)

#define ESP32_CACHE_PAGE_SIZE           0x10000
#define ESP32_CACHE_PAGES_PER_REGION    64
#define ESP32_CACHE_REGION_SIZE         (ESP32_CACHE_PAGE_SIZE * ESP32_CACHE_PAGES_PER_REGION)
#define ESP32_CACHE_MMU_INVALID_VAL     0x100
#define ESP32_CACHE_MMU_ENTRY_CHANGED   0x200     /* not a hardware flag; used here to check if the page data needs to be updated */
#define ESP32_CACHE_MAX_PHYS_PAGES      0x100

typedef enum Esp32CacheRegionType {
    ESP32_DCACHE_FLASH,
    ESP32_ICACHE_FLASH,
    ESP32_DCACHE_PSRAM,
} Esp32CacheRegionType;

typedef struct Esp32CacheRegionState {
    Esp32CacheState* cache;
    MemoryRegion mem;
    MemoryRegion illegal_access_trap_mem;
    Esp32CacheRegionType type;
    hwaddr base;
    uint32_t illegal_access_retval;
    bool illegal_access_trap_en;
    bool illegal_access_status;
    uint16_t mmu_table[ESP32_CACHE_PAGES_PER_REGION];
} Esp32CacheRegionState;

typedef struct Esp32CacheState {
    Esp32DportState* dport;
    int core_id;

    uint32_t cache_ctrl_reg;
    uint32_t cache_ctrl1_reg;
    /* x_DCACHE_DBUG0's CACHE_ACCESS_ILLEGAL and CACHE_MMU_ILLEGAL flags */
    uint32_t cache_ia;
    /* x_DCACHE_DBUG3 flags for the ports the regions below do not cover */
    uint32_t cache_dis_ia;
    /* Using only the first 4MB range.
     * TODO: add memory regions for other ports: iram1, irom0
     */
    Esp32CacheRegionState iram0;
    Esp32CacheRegionState drom0;
    Esp32CacheRegionState dram1;  /* PSRAM */
} Esp32CacheState;

/* Pages of each internal SRAM MMU, and the SRAM0 and SRAM2 they govern */
#define ESP32_SRAM_MMU_PAGES        16
#define ESP32_SRAM0_MMU_BASE        0x40080000
#define ESP32_SRAM2_MMU_BASE        0x3ffc0000
#define ESP32_SRAM_MMU_SIZE         0x20000

/* Entries in each CPU's half of the flash and PSRAM cache MMU table */
#define ESP32_CACHE_MMU_ENTRIES     2048

/* DPORT_AHBLITE_MPU_TABLE_UART_REG .. DPORT_AHBLITE_MPU_TABLE_PWR_REG */
#define ESP32_AHBLITE_TABLES        47

/*
 * Windows in each CPU's physical address space that no bus decodes and
 * that only the memory protection model maps virtual pages to: the sink
 * that answers refused accesses, the window through which 2 KB SRAM MMU
 * pages are reached, and the flash pages the cache MMU maps for the
 * regions the per-CPU cache regions do not model.
 */
#define ESP32_MPU_HIDDEN_BASE       0x0d000000
#define ESP32_MPU_SINK_BASE         0x0d000000
#define ESP32_MPU_SINK_SIZE         0x00400000
#define ESP32_MPU_REDIRECT_BASE     0x0d400000
#define ESP32_MPU_REDIRECT_SIZE     (2 * ESP32_SRAM_MMU_SIZE)
#define ESP32_FLASH_PAGES_BASE      0x0e000000
#define ESP32_FLASH_PAGES           256
#define ESP32_FLASH_PAGES_SIZE      (ESP32_FLASH_PAGES * ESP32_CACHE_PAGE_SIZE)
#define ESP32_MPU_HIDDEN_END        (ESP32_FLASH_PAGES_BASE + \
                                     ESP32_FLASH_PAGES_SIZE)

/* A CPU access as the TLB will map it; see esp32_dport_mpu_filter() */
typedef struct Esp32MpuMapping {
    hwaddr paddr;
    int prot;
    uint64_t page_size;
} Esp32MpuMapping;

typedef unsigned (*Esp32DportGetPid)(void *opaque, int cpu);

typedef struct Esp32DportState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    bool has_psram;
    int cpu_count;
    Esp32CacheState cache_state[ESP32_CPU_COUNT];
    MemoryRegion psram;         /* Shared between the CPUs: the actual memory region for PSRAM */
    BlockBackend *flash_blk;
    qemu_irq appcpu_stall_req;
    qemu_irq appcpu_reset_req;
    qemu_irq clk_update_req;
    qemu_irq cache_ill_irq;
    qemu_irq flash_enc_en_gpio;
    qemu_irq flash_dec_en_gpio;

    bool appcpu_reset_state;
    bool appcpu_stall_state;
    bool appcpu_clkgate_state;
    uint32_t appcpu_boot_addr;
    uint32_t cpuperiod_sel;
    uint32_t cache_ill_trap_en_reg;
    uint32_t slave_spi_config_reg;

    /* Peripheral clock-enable and reset registers; the SoC applies them. */
    qemu_irq periph_clk_update_req;
    uint32_t peri_clk_en;
    uint32_t peri_rst_en;
    uint32_t perip_clk_en;
    uint32_t perip_rst_en;
    uint32_t wifi_clk_en;
    uint32_t core_rst_en;

    /* DPORT_SPI_DMA_CHAN_SEL_REG: the DMA channel of SPI1, SPI2 and SPI3 */
    uint32_t spi_dma_chan_sel;
    /* Per-PID memory protection: the SRAM MMUs, MPUs and their reports */
    Esp32DportGetPid get_pid;
    void *get_pid_opaque;
    qemu_irq mpu_update_req;
    qemu_irq mmu_ia_irq;
    qemu_irq mpu_ia_irq;
    MemoryRegion mpu_sink;
    MemoryRegion mpu_redirect;
    MemoryRegion flash_pages;
    DECLARE_BITMAP(flash_page_valid, ESP32_FLASH_PAGES);
    uint32_t immu_page_mode;
    uint32_t dmmu_page_mode;
    uint32_t immu_table[ESP32_SRAM_MMU_PAGES];
    uint32_t dmmu_table[ESP32_SRAM_MMU_PAGES];
    uint32_t ahb_mpu_table[2];
    uint32_t ahblite_mpu_table[ESP32_AHBLITE_TABLES];
    uint32_t mem_access_dbug0;
    uint32_t mem_access_dbug1;
    uint32_t mmu_ia_int_en;
    uint32_t mpu_ia_int_en;
    uint16_t cache_mmu_table[ESP32_CPU_COUNT][ESP32_CACHE_MMU_ENTRIES];
} Esp32DportState;

void esp32_dport_mpu_filter(Esp32DportState *s, int cpu, unsigned pid,
                            uint32_t vaddr, MMUAccessType access_type,
                            Esp32MpuMapping *map);

/* Used by esp32_dport.c */
void esp32_dport_mpu_init(Esp32DportState *s);
void esp32_dport_mpu_reset(Esp32DportState *s);
bool esp32_dport_mpu_read(Esp32DportState *s, hwaddr addr, uint64_t *value);
bool esp32_dport_mpu_write(Esp32DportState *s, hwaddr addr, uint64_t value);
void esp32_dport_mpu_cache_flushed(Esp32DportState *s);
void esp32_dport_update_cache_irq(Esp32DportState *s);

void esp32_dport_clear_ill_trap_state(Esp32DportState* s);

#define ESP32_DPORT_APPCPU_STALL_GPIO   "appcpu-stall"
#define ESP32_DPORT_APPCPU_RESET_GPIO   "appcpu-reset"
#define ESP32_DPORT_CLK_UPDATE_GPIO     "clk-update"
#define ESP32_DPORT_CACHE_ILL_IRQ_GPIO  "cache-ill-irq"
#define ESP32_DPORT_FLASH_ENC_EN_GPIO   "flash-enc-en"
#define ESP32_DPORT_FLASH_DEC_EN_GPIO   "flash-dec-en"
#define ESP32_DPORT_PERIPH_CLK_UPDATE_GPIO "periph-clk-update"
#define ESP32_DPORT_MPU_UPDATE_GPIO     "mpu-update"
#define ESP32_DPORT_MMU_IA_IRQ_GPIO     "mmu-ia-irq"
#define ESP32_DPORT_MPU_IA_IRQ_GPIO     "mpu-ia-irq"


REG32(DPORT_PERI_CLK_EN, 0x1c)
REG32(DPORT_PERI_RST_EN, 0x20)
    FIELD(DPORT_PERI, AES, 0, 1)
    FIELD(DPORT_PERI, SHA, 1, 1)
    FIELD(DPORT_PERI, RSA, 2, 1)

REG32(DPORT_APPCPU_RESET, 0x2c)
REG32(DPORT_APPCPU_CLK, 0x30)
REG32(DPORT_APPCPU_RUNSTALL, 0x34)
REG32(DPORT_APPCPU_BOOT_ADDR, 0x38)

REG32(DPORT_CPU_PER_CONF, 0x3c)
    FIELD(DPORT_CPU_PER_CONF, CPUPERIOD_SEL, 0, 2)

REG32(DPORT_PRO_CACHE_CTRL, 0x40)
    FIELD(DPORT_PRO_CACHE_CTRL, DRAM_HL, 16, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, DRAM_SPLIT, 11, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, SINGLE_IRAM_ENA, 10, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_DONE, 5, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_ENA, 4, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, CACHE_ENA, 3, 1)

REG32(DPORT_PRO_CACHE_CTRL1, 0x44)
    FIELD(DPORT_PRO_CACHE_CTRL1, MMU_IA_CLR, 13, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_OPSDRAM, 5, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_DROM0, 4, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_DRAM1, 3, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_IROM0, 2, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_IRAM1, 1, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_IRAM0, 0, 1)

REG32(DPORT_APP_CACHE_CTRL, 0x58)
    FIELD(DPORT_APP_CACHE_CTRL, DRAM_HL, 14, 1)
    FIELD(DPORT_APP_CACHE_CTRL, DRAM_SPLIT, 11, 1)
    FIELD(DPORT_APP_CACHE_CTRL, SINGLE_IRAM_ENA, 10, 1)
    FIELD(DPORT_APP_CACHE_CTRL, CACHE_FLUSH_DONE, 5, 1)
    FIELD(DPORT_APP_CACHE_CTRL, CACHE_FLUSH_ENA, 4, 1)
    FIELD(DPORT_APP_CACHE_CTRL, CACHE_ENA, 3, 1)

REG32(DPORT_APP_CACHE_CTRL1, 0x5C)
    FIELD(DPORT_APP_CACHE_CTRL1, MMU_IA_CLR, 13, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_OPSDRAM, 5, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_DROM0, 4, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_DRAM1, 3, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_IROM0, 2, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_IRAM1, 1, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_IRAM0, 0, 1)

REG32(DPORT_IMMU_PAGE_MODE, 0x80)
    FIELD(DPORT_IMMU_PAGE_MODE, PAGE_MODE, 1, 2)
    FIELD(DPORT_IMMU_PAGE_MODE, ENA, 0, 1)
REG32(DPORT_DMMU_PAGE_MODE, 0x84)
REG32(DPORT_AHB_MPU_TABLE_0, 0xb4)
REG32(DPORT_AHB_MPU_TABLE_1, 0xb8)

REG32(DPORT_SLAVE_SPI_CONFIG, 0xC8)
    FIELD(DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_ENCRYPT_ENABLE, 8, 1)
    FIELD(DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_DECRYPT_ENABLE, 12, 1)

REG32(DPORT_PERIP_CLK_EN, 0xc0)
REG32(DPORT_PERIP_RST_EN, 0xc4)
    FIELD(DPORT_PERIP, TIMERS, 0, 1)
    FIELD(DPORT_PERIP, SPI01, 1, 1)
    FIELD(DPORT_PERIP, UART, 2, 1)
    FIELD(DPORT_PERIP, WDG, 3, 1)
    FIELD(DPORT_PERIP, I2S0, 4, 1)
    FIELD(DPORT_PERIP, UART1, 5, 1)
    FIELD(DPORT_PERIP, SPI2, 6, 1)
    FIELD(DPORT_PERIP, I2C_EXT0, 7, 1)
    FIELD(DPORT_PERIP, UHCI0, 8, 1)
    FIELD(DPORT_PERIP, RMT, 9, 1)
    FIELD(DPORT_PERIP, PCNT, 10, 1)
    FIELD(DPORT_PERIP, LEDC, 11, 1)
    FIELD(DPORT_PERIP, UHCI1, 12, 1)
    FIELD(DPORT_PERIP, TIMERGROUP, 13, 1)
    FIELD(DPORT_PERIP, EFUSE, 14, 1)
    FIELD(DPORT_PERIP, TIMERGROUP1, 15, 1)
    FIELD(DPORT_PERIP, SPI3, 16, 1)
    FIELD(DPORT_PERIP, PWM0, 17, 1)
    FIELD(DPORT_PERIP, I2C_EXT1, 18, 1)
    FIELD(DPORT_PERIP, TWAI, 19, 1)
    FIELD(DPORT_PERIP, PWM1, 20, 1)
    FIELD(DPORT_PERIP, I2S1, 21, 1)
    FIELD(DPORT_PERIP, SPI_DMA, 22, 1)
    FIELD(DPORT_PERIP, UART2, 23, 1)
    FIELD(DPORT_PERIP, UART_MEM, 24, 1)
    FIELD(DPORT_PERIP, PWM2, 25, 1)
    FIELD(DPORT_PERIP, PWM3, 26, 1)

REG32(DPORT_WIFI_CLK_EN, 0xcc)
    FIELD(DPORT_WIFI_CLK_EN, SDIO_HOST, 13, 1)
    FIELD(DPORT_WIFI_CLK_EN, EMAC, 14, 1)
    FIELD(DPORT_WIFI_CLK_EN, RNG, 15, 1)
REG32(DPORT_CORE_RST_EN, 0xd0)
    FIELD(DPORT_CORE_RST_EN, SDIO_HOST, 6, 1)
    FIELD(DPORT_CORE_RST_EN, EMAC, 7, 1)

/* Reset values, from ESP-IDF's soc/esp32/register/soc/dport_reg.h. */
#define ESP32_DPORT_PERIP_CLK_EN_RESET  0xf9c1e06f
#define ESP32_DPORT_WIFI_CLK_EN_RESET   0xfffce030

REG32(DPORT_CPU_INTR_FROM_CPU_0, 0xdc)
REG32(DPORT_CPU_INTR_FROM_CPU_1, 0xe0)
REG32(DPORT_CPU_INTR_FROM_CPU_2, 0xe4)
REG32(DPORT_CPU_INTR_FROM_CPU_3, 0xe8)

REG32(DPORT_PRO_MAC_INTR_MAP, 0x104)
REG32(DPORT_APP_MAC_INTR_MAP, 0x218)

REG32(DPORT_AHBLITE_MPU_TABLE_UART, 0x32c)
REG32(DPORT_AHBLITE_MPU_TABLE_RTC, 0x348)
REG32(DPORT_AHBLITE_MPU_TABLE_PWR, 0x3e4)

REG32(DPORT_MEM_ACCESS_DBUG0, 0x3e8)
    FIELD(DPORT_MEM_ACCESS_DBUG0, MMU_MULTI_HIT, 26, 4)
    FIELD(DPORT_MEM_ACCESS_DBUG0, ACCESS_ILLEGAL, 14, 12)
    FIELD(DPORT_MEM_ACCESS_DBUG0, ACCESS_DENY, 10, 4)
REG32(DPORT_MEM_ACCESS_DBUG1, 0x3ec)
    FIELD(DPORT_MEM_ACCESS_DBUG1, AHBLITE_ACCESS_DENY, 9, 1)
    FIELD(DPORT_MEM_ACCESS_DBUG1, DMA_ACCESS_DENY, 8, 1)
    FIELD(DPORT_MEM_ACCESS_DBUG1, ACCESS_PID_ILLEGAL, 6, 2)
    FIELD(DPORT_MEM_ACCESS_DBUG1, ACCESS_MISS, 0, 4)

REG32(DPORT_PRO_DCACHE_DBUG0, 0x3f0)
    FIELD(DPORT_PRO_DCACHE_DBUG0, CACHE_STATE, 7, 12)
    FIELD(DPORT_PRO_DCACHE_DBUG0, CACHE_IA, 1, 6)
    FIELD(DPORT_PRO_DCACHE_DBUG0, CACHE_MMU_IA, 0, 1)

REG32(DPORT_PRO_DCACHE_DBUG3, 0x3FC)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_OPPOSITE, 9, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_DRAM1, 10, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_IROM0, 11, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_IRAM1, 12, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_IRAM0, 13, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_DROM0, 14, 1)

REG32(DPORT_APP_DCACHE_DBUG0, 0x418)
    FIELD(DPORT_APP_DCACHE_DBUG0, CACHE_STATE, 7, 12)

REG32(DPORT_APP_DCACHE_DBUG3, 0x424)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_OPPOSITE, 9, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_DRAM1, 10, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_IROM0, 11, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM1, 12, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM0, 13, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_DROM0, 14, 1)

REG32(DPORT_IMMU_TABLE0, 0x504)
REG32(DPORT_IMMU_TABLE15, 0x540)
REG32(DPORT_DMMU_TABLE0, 0x544)
REG32(DPORT_DMMU_TABLE15, 0x580)
    FIELD(DPORT_MMU_TABLE, RIGHTS, 4, 3)
    FIELD(DPORT_MMU_TABLE, VPAGE, 0, 4)

REG32(DPORT_MMU_IA_INT_EN, 0x598)
REG32(DPORT_MPU_IA_INT_EN, 0x59c)

REG32(DPORT_CACHE_IA_INT_EN, 0x5A0)
    FIELD(DPORT_CACHE_IA_INT_EN, PRO_CACHE_ACCESS_ILLEGAL, 22, 6)
    FIELD(DPORT_CACHE_IA_INT_EN, PRO_CACHE_MMU_ILLEGAL, 21, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_OPPOSITE, 19, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DRAM1, 18, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IROM0, 17, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IRAM1, 16, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IRAM0, 15, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DROM0, 14, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, APP_CACHE_ACCESS_ILLEGAL, 8, 6)
    FIELD(DPORT_CACHE_IA_INT_EN, APP_CACHE_MMU_ILLEGAL, 7, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_OPPOSITE, 5, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_DRAM1, 4, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_IROM0, 3, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_IRAM1, 2, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_IRAM0, 1, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_DROM0, 0, 1)

REG32(DPORT_SPI_DMA_CHAN_SEL, 0x5A8)
    FIELD(DPORT_SPI_DMA_CHAN_SEL, SPI1, 0, 2)
    FIELD(DPORT_SPI_DMA_CHAN_SEL, SPI2, 2, 2)
    FIELD(DPORT_SPI_DMA_CHAN_SEL, SPI3, 4, 2)

#define ESP32_DPORT_PRO_INTMATRIX_BASE    A_DPORT_PRO_MAC_INTR_MAP
#define ESP32_DPORT_APP_INTMATRIX_BASE    A_DPORT_APP_MAC_INTR_MAP
#define ESP32_DPORT_CROSSCORE_INT_BASE    A_DPORT_CPU_INTR_FROM_CPU_0

