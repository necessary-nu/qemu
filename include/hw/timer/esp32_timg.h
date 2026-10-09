#pragma once

#include "hw/core/registerfields.h"
#include "hw/core/clock.h"
#include "hw/core/sysbus.h"

#define TYPE_ESP32_TIMG "timer.esp32.timg"
#define ESP32_TIMG(obj) OBJECT_CHECK(Esp32TimgState, (obj), TYPE_ESP32_TIMG)

#define ESP32_TIMG_WDT_STAGE_COUNT 4

/* RTC_CALI_CLK_SEL: the clock the RTC calibration measures */
typedef enum Esp32TimgCalClkSel {
    ESP32_TIMG_CAL_RTC_MUX = 0,
    ESP32_TIMG_CAL_8MD256 = 1,
    ESP32_TIMG_CAL_32K_XTAL = 2,
    ESP32_TIMG_CAL_CLK_COUNT
} Esp32TimgCalClkSel;

/*
 * Clock inputs for the RTC calibration, which counts XTAL_CLK cycles over
 * RTC_CALI_MAX cycles of the selected clock.
 */
#define ESP32_TIMG_XTAL_CLK         "xtal"
#define ESP32_TIMG_RTC_SLOW_CLK     "rtc-slow"
#define ESP32_TIMG_8MD256_CLK       "rc-fast-d256"
#define ESP32_TIMG_XTAL32K_CLK      "xtal32k"

typedef enum Esp32TimgInterruptType {
    TIMG_T0_INT,
    TIMG_T1_INT,
    TIMG_WDT_INT,
    TIMG_LACT_INT,
    TIMG_INT_MAX
} Esp32TimgInterruptType;

typedef struct Esp32TimgState Esp32TimgState;

typedef struct Esp32TimgTimerState {
    Esp32TimgState *parent;
    uint32_t config_reg;
    int divider;
    bool en;
    bool autoreload;
    bool inc;
    bool edge_int_en;
    bool level_int_en;
    bool alarm;
    uint64_t alarm_val;
    uint64_t load_val;
    uint64_t count_base;
    uint64_t last_val;
    uint64_t ns_base;
    Esp32TimgInterruptType int_type;
    QEMUTimer alarm_timer;
} Esp32TimgTimerState;

typedef enum Esp32TimgWdtStageMode {
    WDT_MODE_OFF = 0,
    WDT_MODE_INT = 1,
    WDT_MODE_CPURESET = 2,
    WDT_MODE_SYSRESET = 3
} Esp32TimgWdtStageMode;

typedef struct Esp32TimgWdtState {
    Esp32TimgState *parent;
    uint32_t config0_reg;
    uint32_t config1_reg;
    bool en;
    bool flashboot_en;
    bool level_int_en;
    bool edge_int_en;
    int prescale;
    Esp32TimgWdtStageMode mode[ESP32_TIMG_WDT_STAGE_COUNT];
    int timeout[ESP32_TIMG_WDT_STAGE_COUNT];
    uint32_t count_base;
    uint64_t ns_base;
    int cur_stage;
    uint32_t protect_reg;
    QEMUTimer stage_timer;
} Esp32TimgWdtState;

typedef struct Esp32TimgState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    int id;
    qemu_irq irqs[2*TIMG_INT_MAX];
    qemu_irq wdt_cpu_reset_req;
    qemu_irq wdt_sys_reset_req;

    Esp32TimgTimerState t0;
    Esp32TimgTimerState t1;
    Esp32TimgTimerState lact;
    Esp32TimgWdtState wdt;

    uint32_t int_ena;
    uint32_t int_raw;

    /* APB_CLK, stopped while DPORT gates the group or holds it in reset */
    Clock *apb_clk;
    Clock *xtal_clk;
    /* Indexed by Esp32TimgCalClkSel */
    Clock *cal_clk[ESP32_TIMG_CAL_CLK_COUNT];
    bool flash_boot_mode;
    bool wdt_en_at_reset;
    bool wdt_disable;

    /* RTCCALICFG's fields and RTCCALICFG1's value */
    bool rtc_cal_start;
    bool rtc_cal_start_cycling;
    bool rtc_cal_ready;
    Esp32TimgCalClkSel rtc_cal_clk_sel;
    uint32_t rtc_cal_max;
    uint32_t rtc_cal_value;
    /*
     * The measurement in progress, if active: started by RTC_CALI_START
     * (oneshot) or by cycling mode, on the clock and for the cycle count
     * latched when it started. It waits for the first edge of the clock
     * (until counting), then counts cal_cycles of it and cal_xtal XTAL_CLK
     * cycles before cal_seg_ns, the rest since.
     */
    bool cal_active;
    bool cal_oneshot;
    bool cal_counting;
    Esp32TimgCalClkSel cal_sel;
    uint32_t cal_target;
    uint64_t cal_cycles;
    uint64_t cal_xtal;
    int64_t cal_seg_ns;
    /* When the timer is due: the first edge, or the end */
    int64_t cal_event_ns;
    QEMUTimer cal_timer;
} Esp32TimgState;

#define ESP32_TIMG_WDT_CPU_RESET_GPIO   "mwdt-cpu-reset"
#define ESP32_TIMG_WDT_SYS_RESET_GPIO   "mwdt-sys-reset"

#define RTC_CLK_CAL_FRACT  19
#define RTC_CLK_CAL_FACTOR  (1 << RTC_CLK_CAL_FRACT)

REG32(TIMG_T0CONFIG, 0x00)
    FIELD(TIMG_T0CONFIG, EN, 31, 1)
    FIELD(TIMG_T0CONFIG, INCREASE, 30, 1)
    FIELD(TIMG_T0CONFIG, AUTORELOAD, 29, 1)
    FIELD(TIMG_T0CONFIG, DIVIDER, 13, 16)
    FIELD(TIMG_T0CONFIG, EDGE_INT, 12, 1)
    FIELD(TIMG_T0CONFIG, LEVEL_INT, 11, 1)
    FIELD(TIMG_T0CONFIG, ALARM, 10, 1)

REG32(TIMG_T0LO, 0x04)
REG32(TIMG_T0HI, 0x08)
REG32(TIMG_T0UPDATE, 0x0c)
REG32(TIMG_T0ALARMLO, 0x10)
REG32(TIMG_T0ALARMHI, 0x14)
REG32(TIMG_T0LOADLO, 0x18)
REG32(TIMG_T0LOADHI, 0x1c)
REG32(TIMG_T0LOAD, 0x20)

REG32(TIMG_T1CONFIG, 0x24)
    FIELD(TIMG_T1CONFIG, EN, 31, 1)
    FIELD(TIMG_T1CONFIG, INCREASE, 30, 1)
    FIELD(TIMG_T1CONFIG, AUTORELOAD, 29, 1)
    FIELD(TIMG_T1CONFIG, DIVIDER, 13, 16)
    FIELD(TIMG_T1CONFIG, EDGE_INT, 12, 1)
    FIELD(TIMG_T1CONFIG, LEVEL_INT, 11, 1)
    FIELD(TIMG_T1CONFIG, ALARM, 10, 1)

REG32(TIMG_T1LO, 0x28)
REG32(TIMG_T1HI, 0x2c)
REG32(TIMG_T1UPDATE, 0x30)
REG32(TIMG_T1ALARMLO, 0x34)
REG32(TIMG_T1ALARMHI, 0x38)
REG32(TIMG_T1LOADLO, 0x3c)
REG32(TIMG_T1LOADHI, 0x40)
REG32(TIMG_T1LOAD, 0x44)

REG32(TIMG_WDTCONFIG0, 0x48)
    FIELD(TIMG_WDTCONFIG0, EN, 31, 1)
    FIELD(TIMG_WDTCONFIG0, STG0, 29, 2)
    FIELD(TIMG_WDTCONFIG0, STG1, 27, 2)
    FIELD(TIMG_WDTCONFIG0, STG2, 25, 2)
    FIELD(TIMG_WDTCONFIG0, STG3, 23, 2)
    FIELD(TIMG_WDTCONFIG0, EDGE_INT, 22, 1)
    FIELD(TIMG_WDTCONFIG0, LEVEL_INT, 21, 1)
    FIELD(TIMG_WDTCONFIG0, FLASHBOOT_MODE_EN, 14, 1)
REG32(TIMG_WDTCONFIG1, 0x4c)
    FIELD(TIMG_WDTCONFIG1, PRESCALE, 16, 16)
REG32(TIMG_WDTCONFIG2, 0x50)
REG32(TIMG_WDTCONFIG3, 0x54)
REG32(TIMG_WDTCONFIG4, 0x58)
REG32(TIMG_WDTCONFIG5, 0x5c)
REG32(TIMG_WDTFEED, 0x60)
    FIELD(TIMG_WDTFEED, FEED, 31, 1)
REG32(TIMG_WDTPROTECT, 0x64)


REG32(TIMG_RTCCALICFG, 0x68)
    FIELD(TIMG_RTCCALICFG, START, 31, 1)
    FIELD(TIMG_RTCCALICFG, MAX, 16, 15)
    FIELD(TIMG_RTCCALICFG, RDY, 15, 1)
    FIELD(TIMG_RTCCALICFG, CLK_SEL, 13, 2)
    FIELD(TIMG_RTCCALICFG, START_CYCLING, 12, 1)

REG32(TIMG_RTCCALICFG1, 0x6c)
    FIELD(TIMG_RTCCALICFG1, VALUE, 7, 25)

REG32(TIMG_LACTCONFIG, 0x70)
    FIELD(TIMG_LACTCONFIG, EN, 31, 1)
    FIELD(TIMG_LACTCONFIG, INCREASE, 30, 1)
    FIELD(TIMG_LACTCONFIG, AUTORELOAD, 29, 1)
    FIELD(TIMG_LACTCONFIG, DIVIDER, 13, 16)
    FIELD(TIMG_LACTCONFIG, EDGE_INT, 12, 1)
    FIELD(TIMG_LACTCONFIG, LEVEL_INT, 11, 1)
    FIELD(TIMG_LACTCONFIG, ALARM, 10, 1)
REG32(TIMG_LACTRTC, 0x74)
    FIELD(TIMG_LACTRTC, STEP_LEN, 6, 26)
REG32(TIMG_LACTLO, 0x78)
REG32(TIMG_LACTHI, 0x7c)
REG32(TIMG_LACTUPDATE, 0x80)
REG32(TIMG_LACTALARMLO, 0x84)
REG32(TIMG_LACTALARMHI, 0x88)
REG32(TIMG_LACTLOADLO, 0x8c)
REG32(TIMG_LACTLOADHI, 0x90)
REG32(TIMG_LACTLOAD, 0x94)

REG32(TIMG_INT_ENA, 0x98)
    FIELD(TIMG_INT_ENA, LACT, 3, 1)
    FIELD(TIMG_INT_ENA, WDT, 2, 1)
    FIELD(TIMG_INT_ENA, T1, 1, 1)
    FIELD(TIMG_INT_ENA, T0, 0, 1)

REG32(TIMG_INT_RAW, 0x9c)
    FIELD(TIMG_INT_RAW, LACT, 3, 1)
    FIELD(TIMG_INT_RAW, WDT, 2, 1)
    FIELD(TIMG_INT_RAW, T1, 1, 1)
    FIELD(TIMG_INT_RAW, T0, 0, 1)

REG32(TIMG_INT_ST, 0xa0)
    FIELD(TIMG_INT_ST, LACT, 3, 1)
    FIELD(TIMG_INT_ST, WDT, 2, 1)
    FIELD(TIMG_INT_ST, T1, 1, 1)
    FIELD(TIMG_INT_ST, T0, 0, 1)

REG32(TIMG_INT_CLR, 0xa4)
    FIELD(TIMG_INT_CLR, LACT, 3, 1)
    FIELD(TIMG_INT_CLR, WDT, 2, 1)
    FIELD(TIMG_INT_CLR, T1, 1, 1)
    FIELD(TIMG_INT_CLR, T0, 0, 1)

#define ESP32_TIMG_WDT_PROTECT_WORD 0x50D83AA1
