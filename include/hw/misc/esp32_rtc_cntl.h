/*
 * ESP32 RTC_CNTL: low-power management, the RTC timer and watchdog, and the
 * RTC fast and slow memories
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/core/clock.h"
#include "qemu/timer.h"
#include "hw/misc/esp32_reg.h"

#define TYPE_ESP32_RTC_CNTL "misc.esp32.rtc_cntl"
#define ESP32_RTC_CNTL(obj) OBJECT_CHECK(Esp32RtcCntlState, (obj), TYPE_ESP32_RTC_CNTL)

typedef struct Esp32RtcIoState Esp32RtcIoState;

/*
 * Outputs to the SoC, which owns the CPUs and the digital domain:
 * - ESP32_RTC_DIG_RESET_GPIO: reset the digital domain (CPUs and
 *   peripherals), the RTC domain staying as it is.
 * - ESP32_RTC_RTC_RESET_GPIO: reset the digital domain and the RTC domain,
 *   as the RTC watchdog's and the brownout detector's resets do.
 * - ESP32_RTC_SLEEP_RESET_GPIO: the digital domain powers down for deep
 *   sleep. It comes back in its reset state.
 * - ESP32_RTC_CPU_RESET_GPIO (per CPU): reset one CPU.
 * - ESP32_RTC_CPU_STALL_GPIO (per CPU): the CPU's stall changed; the SoC
 *   reads cpu_stall_state.
 * - ESP32_RTC_CLK_UPDATE_GPIO: the clock selection, the BBPLL's or APLL's
 *   power, or the digital domain's clock gating (dig_clk_gated) changed.
 * - ESP32_RTC_TOUCH_TIMER_GPIO: RTC_CNTL_TOUCH_SLP_TIMER_EN, which lets the
 *   touch sensor's timer start measurements.
 *
 * Inputs, from blocks that wake the chip or interrupt through RTC_CNTL:
 * - ESP32_RTC_WAKEUP_IN (ESP32_RTC_WAKEUP_COUNT lines, indexed by the
 *   RTC_CNTL_WAKEUP_ENA bit): a level-sensitive wakeup request from SDIO,
 *   Wi-Fi, UART0, UART1, touch, the ULP coprocessor or Bluetooth. The EXT0,
 *   EXT1, GPIO and timer sources are computed here; their lines are unused.
 * - ESP32_RTC_GPIO_WAKEUP_IN: the GPIO matrix's light-sleep wakeup.
 * - ESP32_RTC_INT_IN (indexed by the RTC_CNTL_INT_RAW bit): a rising edge
 *   sets the raw interrupt bit, for the touch, ULP, SDIO-idle and brownout
 *   interrupts raised by blocks modelled elsewhere.
 * - ESP32_RTC_BROWNOUT_IN: the supply is below the brownout threshold.
 */
#define ESP32_RTC_DIG_RESET_GPIO    "dig-reset"
#define ESP32_RTC_RTC_RESET_GPIO    "rtc-reset"
#define ESP32_RTC_SLEEP_RESET_GPIO  "sleep-reset"
#define ESP32_RTC_CPU_RESET_GPIO    "cpu-reset"
#define ESP32_RTC_CPU_STALL_GPIO    "cpu-stall"
#define ESP32_RTC_CLK_UPDATE_GPIO   "clk-update"
#define ESP32_RTC_TOUCH_TIMER_GPIO  "touch-timer-en"
#define ESP32_RTC_WAKEUP_IN         "esp32-rtc-wakeup"
#define ESP32_RTC_GPIO_WAKEUP_IN    "esp32-rtc-gpio-wakeup"
#define ESP32_RTC_INT_IN            "esp32-rtc-int"
#define ESP32_RTC_BROWNOUT_IN       "esp32-rtc-brownout"

/*
 * Clock outputs, stopped (0 Hz) while their source does not run:
 * - ESP32_RTC_XTAL_CLK: XTAL_CLK, the main crystal.
 * - ESP32_RTC_SLOW_CLK: RTC_SLOW_CLK, as ANA_CLK_RTC_SEL selects it.
 * - ESP32_RTC_D256_DIG_CLK: RC_FAST_DIV_CLK (8MD256) as the digital domain
 *   sees it, behind DIG_CLK8M_D256_EN.
 * - ESP32_RTC_XTAL32K_DIG_CLK: XTAL32K_CLK as the digital domain sees it,
 *   behind DIG_XTAL32K_EN.
 */
#define ESP32_RTC_XTAL_CLK          "xtal-clk"
#define ESP32_RTC_SLOW_CLK          "slow-clk"
#define ESP32_RTC_D256_DIG_CLK      "rc-fast-d256-dig-clk"
#define ESP32_RTC_XTAL32K_DIG_CLK   "xtal32k-dig-clk"

/* RC_SLOW_CLK and RC_FAST_CLK, the internal oscillators, nominally */
#define ESP32_RC_SLOW_CLK_HZ        150000
#define ESP32_RC_FAST_CLK_HZ        8000000
#define ESP32_XTAL32K_CLK_HZ        32768

typedef enum Esp32ResetCause {
    ESP32_POWERON_RESET = 1,
    ESP32_SW_SYS_RESET = 3,
    ESP32_OWDT_RESET = 4,
    ESP32_DEEPSLEEP_RESET = 5,
    ESP32_SDIO_RESET = 6,
    ESP32_TG0WDT_SYS_RESET = 7,
    ESP32_TG1WDT_SYS_RESET = 8,
    ESP32_RTCWDT_SYS_RESET = 9,
    ESP32_TGWDT_CPU_RESET = 11,
    ESP32_SW_CPU_RESET = 12,
    ESP32_RTCWDT_CPU_RESET = 13,
    ESP32_EXT_CPU_RESET = 14,
    ESP32_RTCWDT_BROWN_OUT_RESET = 15,
    ESP32_RTCWDT_RTC_RESET = 16
} Esp32ResetCause;

typedef enum Esp32SocClkSel {
    ESP32_SOC_CLK_XTAL = 0,
    ESP32_SOC_CLK_PLL = 1,
    ESP32_SOC_CLK_8M = 2,
    ESP32_SOC_CLK_APLL = 3
} Esp32SocClkSel;

typedef enum Esp32FastClkSel {
    ESP32_FAST_CLK_XTALD4 = 0,
    ESP32_FAST_CLK_8M = 1
} Esp32FastClkSel;

typedef enum Esp32SlowClkSel {
    ESP32_SLOW_CLK_RC = 0,
    ESP32_SLOW_CLK_32KXTAL = 1,
    ESP32_SLOW_CLK_8MD256 = 2
} Esp32SlowClkSel;

/* RTC_CNTL_WAKEUP_ENA / WAKEUP_CAUSE bits (TRM table 9.3-2) */
typedef enum Esp32RtcWakeup {
    ESP32_RTC_WAKEUP_EXT0,
    ESP32_RTC_WAKEUP_EXT1,
    ESP32_RTC_WAKEUP_GPIO,
    ESP32_RTC_WAKEUP_TIMER,
    ESP32_RTC_WAKEUP_SDIO,
    ESP32_RTC_WAKEUP_WIFI,
    ESP32_RTC_WAKEUP_UART0,
    ESP32_RTC_WAKEUP_UART1,
    ESP32_RTC_WAKEUP_TOUCH,
    ESP32_RTC_WAKEUP_ULP,
    ESP32_RTC_WAKEUP_BT,
    ESP32_RTC_WAKEUP_COUNT,
} Esp32RtcWakeup;

/* RTC_CNTL_INT_RAW bits */
typedef enum Esp32RtcInt {
    ESP32_RTC_INT_SLP_WAKEUP,
    ESP32_RTC_INT_SLP_REJECT,
    ESP32_RTC_INT_SDIO_IDLE,
    ESP32_RTC_INT_WDT,
    ESP32_RTC_INT_TIME_VALID,
    ESP32_RTC_INT_ULP_CP,
    ESP32_RTC_INT_TOUCH,
    ESP32_RTC_INT_BROWN_OUT,
    ESP32_RTC_INT_MAIN_TIMER,
    ESP32_RTC_INT_COUNT,
} Esp32RtcInt;

typedef enum Esp32RtcSleep {
    ESP32_RTC_AWAKE,
    ESP32_RTC_LIGHT_SLEEP,
    ESP32_RTC_DEEP_SLEEP,
} Esp32RtcSleep;

#define ESP32_RTC_FAST_MEM_SIZE 0x2000
#define ESP32_RTC_SLOW_MEM_SIZE 0x2000

REG32(RTC_CNTL_OPTIONS0, 0x00)
    FIELD(RTC_CNTL_OPTIONS0, SW_SYS_RESET, 31, 1)
    FIELD(RTC_CNTL_OPTIONS0, BIAS_I2C_FORCE_PD, 18, 1)
    FIELD(RTC_CNTL_OPTIONS0, BBPLL_FORCE_PD, 10, 1)
    FIELD(RTC_CNTL_OPTIONS0, BBPLL_I2C_FORCE_PD, 8, 1)
    FIELD(RTC_CNTL_OPTIONS0, SW_PROCPU_RESET, 5, 1)
    FIELD(RTC_CNTL_OPTIONS0, SW_APPCPU_RESET, 4, 1)
    FIELD(RTC_CNTL_OPTIONS0, SW_STALL_PROCPU_C0, 2, 2)
    FIELD(RTC_CNTL_OPTIONS0, SW_STALL_APPCPU_C0, 0, 2)
REG32(RTC_CNTL_SLP_TIMER0, 0x04)
REG32(RTC_CNTL_SLP_TIMER1, 0x08)
    FIELD(RTC_CNTL_SLP_TIMER1, MAIN_TIMER_ALARM_EN, 16, 1)
    FIELD(RTC_CNTL_SLP_TIMER1, SLP_VAL_HI, 0, 16)
REG32(RTC_CNTL_TIME_UPDATE, 0x0c)
    FIELD(RTC_CNTL_TIME_UPDATE, UPDATE, 31, 1)
    FIELD(RTC_CNTL_TIME_UPDATE, VALID, 30, 1)
REG32(RTC_CNTL_TIME0, 0x10)
REG32(RTC_CNTL_TIME1, 0x14)
REG32(RTC_CNTL_STATE0, 0x18)
    FIELD(RTC_CNTL_STATE0, SLEEP_EN, 31, 1)
    FIELD(RTC_CNTL_STATE0, SLP_REJECT, 30, 1)
    FIELD(RTC_CNTL_STATE0, SLP_WAKEUP, 29, 1)
    FIELD(RTC_CNTL_STATE0, SDIO_ACTIVE_IND, 28, 1)
    FIELD(RTC_CNTL_STATE0, TOUCH_SLP_TIMER_EN, 23, 1)
REG32(RTC_CNTL_TIMER1, 0x1c)
REG32(RTC_CNTL_TIMER2, 0x20)
REG32(RTC_CNTL_TIMER3, 0x24)
REG32(RTC_CNTL_TIMER4, 0x28)
REG32(RTC_CNTL_TIMER5, 0x2c)
REG32(RTC_CNTL_ANA_CONF, 0x30)
    FIELD(RTC_CNTL_ANA_CONF, PLLA_FORCE_PU, 24, 1)
    FIELD(RTC_CNTL_ANA_CONF, PLLA_FORCE_PD, 23, 1)
REG32(RTC_CNTL_RESET_STATE, 0x34)
    FIELD(RTC_CNTL_RESET_STATE, PROCPU_STAT_VECTOR_SEL, 13, 1)
    FIELD(RTC_CNTL_RESET_STATE, APPCPU_STAT_VECTOR_SEL, 12, 1)
    FIELD(RTC_CNTL_RESET_STATE, RESET_CAUSE_APPCPU, 6, 6)
    FIELD(RTC_CNTL_RESET_STATE, RESET_CAUSE_PROCPU, 0, 6)
REG32(RTC_CNTL_WAKEUP_STATE, 0x38)
    FIELD(RTC_CNTL_WAKEUP_STATE, GPIO_WAKEUP_FILTER, 22, 1)
    FIELD(RTC_CNTL_WAKEUP_STATE, WAKEUP_ENA, 11, 11)
    FIELD(RTC_CNTL_WAKEUP_STATE, WAKEUP_CAUSE, 0, 11)
REG32(RTC_CNTL_INT_ENA, 0x3c)
REG32(RTC_CNTL_INT_RAW, 0x40)
REG32(RTC_CNTL_INT_ST, 0x44)
REG32(RTC_CNTL_INT_CLR, 0x48)
REG32(RTC_CNTL_STORE0, 0x4c)
REG32(RTC_CNTL_STORE1, 0x50)
REG32(RTC_CNTL_STORE2, 0x54)
REG32(RTC_CNTL_STORE3, 0x58)
REG32(RTC_CNTL_EXT_XTL_CONF, 0x5c)
REG32(RTC_CNTL_EXT_WAKEUP_CONF, 0x60)
    FIELD(RTC_CNTL_EXT_WAKEUP_CONF, EXT_WAKEUP1_LV, 31, 1)
    FIELD(RTC_CNTL_EXT_WAKEUP_CONF, EXT_WAKEUP0_LV, 30, 1)
REG32(RTC_CNTL_SLP_REJECT_CONF, 0x64)
    FIELD(RTC_CNTL_SLP_REJECT_CONF, REJECT_CAUSE, 28, 4)
    FIELD(RTC_CNTL_SLP_REJECT_CONF, DEEP_SLP_REJECT_EN, 27, 1)
    FIELD(RTC_CNTL_SLP_REJECT_CONF, LIGHT_SLP_REJECT_EN, 26, 1)
    FIELD(RTC_CNTL_SLP_REJECT_CONF, SDIO_REJECT_EN, 25, 1)
    FIELD(RTC_CNTL_SLP_REJECT_CONF, GPIO_REJECT_EN, 24, 1)
REG32(RTC_CNTL_CPU_PERIOD_CONF, 0x68)
REG32(RTC_CNTL_SDIO_ACT_CONF, 0x6c)
REG32(RTC_CNTL_CLK_CONF, 0x70)
    FIELD(RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL, 30, 2)
    FIELD(RTC_CNTL_CLK_CONF, FAST_CLK_RTC_SEL, 29, 1)
    FIELD(RTC_CNTL_CLK_CONF, SOC_CLK_SEL, 27, 2)
    FIELD(RTC_CNTL_CLK_CONF, DIG_CLK8M_D256_EN, 9, 1)
    FIELD(RTC_CNTL_CLK_CONF, DIG_XTAL32K_EN, 8, 1)
    FIELD(RTC_CNTL_CLK_CONF, ENB_CK8M_DIV, 7, 1)
    FIELD(RTC_CNTL_CLK_CONF, ENB_CK8M, 6, 1)
    FIELD(RTC_CNTL_CLK_CONF, CK8M_DIV, 4, 2)
REG32(RTC_CNTL_SDIO_CONF, 0x74)
REG32(RTC_CNTL_BIAS_CONF, 0x78)
REG32(RTC_CNTL_VREG, 0x7c)
REG32(RTC_CNTL_PWC, 0x80)
    FIELD(RTC_CNTL_PWC, PD_EN, 20, 1)
    FIELD(RTC_CNTL_PWC, SLOWMEM_PD_EN, 17, 1)
    FIELD(RTC_CNTL_PWC, SLOWMEM_FORCE_PU, 16, 1)
    FIELD(RTC_CNTL_PWC, SLOWMEM_FORCE_PD, 15, 1)
    FIELD(RTC_CNTL_PWC, FASTMEM_PD_EN, 14, 1)
    FIELD(RTC_CNTL_PWC, FASTMEM_FORCE_PU, 13, 1)
    FIELD(RTC_CNTL_PWC, FASTMEM_FORCE_PD, 12, 1)
REG32(RTC_CNTL_DIG_PWC, 0x84)
    FIELD(RTC_CNTL_DIG_PWC, DG_WRAP_PD_EN, 31, 1)
REG32(RTC_CNTL_DIG_ISO, 0x88)
    FIELD(RTC_CNTL_DIG_ISO, DG_PAD_FORCE_HOLD, 15, 1)
    FIELD(RTC_CNTL_DIG_ISO, DG_PAD_FORCE_UNHOLD, 14, 1)
    FIELD(RTC_CNTL_DIG_ISO, DG_PAD_AUTOHOLD_EN, 11, 1)
    FIELD(RTC_CNTL_DIG_ISO, CLR_DG_PAD_AUTOHOLD, 10, 1)
    FIELD(RTC_CNTL_DIG_ISO, DG_PAD_AUTOHOLD, 9, 1)
REG32(RTC_CNTL_WDTCONFIG0, 0x8c)
    FIELD(RTC_CNTL_WDTCONFIG0, EN, 31, 1)
    FIELD(RTC_CNTL_WDTCONFIG0, STG0, 28, 3)
    FIELD(RTC_CNTL_WDTCONFIG0, STG1, 25, 3)
    FIELD(RTC_CNTL_WDTCONFIG0, STG2, 22, 3)
    FIELD(RTC_CNTL_WDTCONFIG0, STG3, 19, 3)
    FIELD(RTC_CNTL_WDTCONFIG0, EDGE_INT_EN, 18, 1)
    FIELD(RTC_CNTL_WDTCONFIG0, LEVEL_INT_EN, 17, 1)
    FIELD(RTC_CNTL_WDTCONFIG0, FLASHBOOT_MOD_EN, 10, 1)
    FIELD(RTC_CNTL_WDTCONFIG0, PROCPU_RESET_EN, 9, 1)
    FIELD(RTC_CNTL_WDTCONFIG0, APPCPU_RESET_EN, 8, 1)
    FIELD(RTC_CNTL_WDTCONFIG0, PAUSE_IN_SLP, 7, 1)
REG32(RTC_CNTL_WDTCONFIG1, 0x90)
REG32(RTC_CNTL_WDTCONFIG2, 0x94)
REG32(RTC_CNTL_WDTCONFIG3, 0x98)
REG32(RTC_CNTL_WDTCONFIG4, 0x9c)
REG32(RTC_CNTL_WDTFEED, 0xa0)
    FIELD(RTC_CNTL_WDTFEED, FEED, 31, 1)
REG32(RTC_CNTL_WDTWPROTECT, 0xa4)
REG32(RTC_CNTL_TEST_MUX, 0xa8)
REG32(RTC_CNTL_SW_CPU_STALL, 0xac)
    FIELD(RTC_CNTL_SW_CPU_STALL, PROCPU_C1, 26, 6)
    FIELD(RTC_CNTL_SW_CPU_STALL, APPCPU_C1, 20, 6)
REG32(RTC_CNTL_STORE4, 0xb0)
REG32(RTC_CNTL_STORE5, 0xb4)
REG32(RTC_CNTL_STORE6, 0xb8)
REG32(RTC_CNTL_STORE7, 0xbc)
REG32(RTC_CNTL_LOW_POWER_ST, 0xc0)
    FIELD(RTC_CNTL_LOW_POWER_ST, MAIN_STATE_IN_IDLE, 27, 1)
    FIELD(RTC_CNTL_LOW_POWER_ST, RDY_FOR_WAKEUP, 19, 1)
REG32(RTC_CNTL_DIAG1, 0xc4)
REG32(RTC_CNTL_HOLD_FORCE, 0xc8)
REG32(RTC_CNTL_EXT_WAKEUP1, 0xcc)
    FIELD(RTC_CNTL_EXT_WAKEUP1, STATUS_CLR, 18, 1)
    FIELD(RTC_CNTL_EXT_WAKEUP1, SEL, 0, 18)
REG32(RTC_CNTL_EXT_WAKEUP1_STATUS, 0xd0)
REG32(RTC_CNTL_BROWN_OUT, 0xd4)
    FIELD(RTC_CNTL_BROWN_OUT, DET, 31, 1)
    FIELD(RTC_CNTL_BROWN_OUT, ENA, 30, 1)
    FIELD(RTC_CNTL_BROWN_OUT, RST_ENA, 26, 1)
    FIELD(RTC_CNTL_BROWN_OUT, RST_WAIT, 16, 10)
/* The RTC fast memory's CRC engine, used by the ROM's RTC boot */
REG32(RTC_MEM_CONF, 0x100)
    FIELD(RTC_MEM_CONF, CRC_FINISH, 31, 1)
    FIELD(RTC_MEM_CONF, CRC_LEN, 20, 11)
    FIELD(RTC_MEM_CONF, CRC_ADDR, 9, 11)
    FIELD(RTC_MEM_CONF, CRC_START, 8, 1)
    FIELD(RTC_MEM_CONF, PID_CONF, 0, 8)
REG32(RTC_MEM_CRC_RES, 0x104)
REG32(RTC_CNTL_DATE, 0x13c)

#define ESP32_RTC_CNTL_SIZE (A_RTC_CNTL_DATE + 4)
#define ESP32_RTC_CNTL_REG_COUNT (ESP32_RTC_CNTL_SIZE / 4)

/* CK8M_DIV_SEL = 2, DIG_CLK8M_D256_EN, CK8M_DIV = 1; SOC_CLK_SEL = XTAL. */
#define ESP32_RTC_CNTL_CLK_CONF_RESET 0x00002210

/* RTC_CNTL_WDTWPROTECT's value that unlocks the RTC watchdog's registers */
#define ESP32_RTC_WDT_WKEY 0x50d83aa1
#define ESP32_RTC_WDT_STAGES 4

typedef struct Esp32RtcCntlState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* RTC fast memory (the PRO CPU's IRAM0/DRAM0 view) and slow memory */
    MemoryRegion fast_mem;
    MemoryRegion slow_mem;

    qemu_irq irq;
    qemu_irq dig_reset_req;
    qemu_irq rtc_reset_req;
    qemu_irq sleep_reset_req;
    qemu_irq cpu_reset_req[ESP32_CPU_COUNT];
    qemu_irq cpu_stall_req[ESP32_CPU_COUNT];
    qemu_irq clk_update;
    qemu_irq touch_timer_en;

    Clock *xtal_clk;
    Clock *slow_clk;
    Clock *d256_dig_clk;
    Clock *xtal32k_dig_clk;
    /*
     * A 32.768 kHz crystal is fitted across XTAL_32K_P/N. Modules such as
     * the ESP32-WROOM-32E have none, so XTAL32K_CLK never runs.
     */
    bool xtal32k_fitted;

    /* RTCIO, whose pads are the EXT0, EXT1 and RTC GPIO wakeup sources */
    Esp32RtcIoState *rtcio;
    /* Strapped for SPI flash boot, which arms the watchdog's flash boot mode */
    bool flash_boot_mode;

    QEMUTimer alarm_timer;
    QEMUTimer valid_timer;
    QEMUTimer wdt_timer;
    QEMUTimer brownout_timer;

    /* Every register's stored value, indexed by offset / 4 */
    uint32_t regs[ESP32_RTC_CNTL_REG_COUNT];

    /*
     * The RTC timer counts RTC_SLOW_CLK cycles: time_base plus the cycles
     * since time_base_ns, at the slow clock's current rate.
     */
    uint64_t time_base;
    int64_t time_base_ns;
    /* TIME0/TIME1 as last latched by TIME_UPDATE */
    uint64_t time_latched;
    /* A TIME_UPDATE latch waits for an RTC_SLOW_CLK edge to set VALID */
    bool valid_pending;

    /* RTC watchdog: its stage, and its count since wdt_base_ns */
    uint32_t wdt_stage;
    uint64_t wdt_count_base;
    int64_t wdt_base_ns;
    bool wdt_running;

    /* An Esp32RtcSleep */
    uint32_t sleep;
    /* Deep sleep: the digital domain's power-down reset has not happened */
    bool sleep_reset_pending;
    /* The RTC timer alarm went off during the current sleep */
    bool timer_wakeup;
    /* Levels of the wakeup request lines from other blocks */
    uint32_t wakeup_lines;
    bool gpio_wakeup_line;
    bool brownout_line;
    /* The next reset of this block is an RTC reset, not a power-on */
    bool rtc_reset_pending;

    /* State the SoC reads */
    bool cpu_stall_state[ESP32_CPU_COUNT];
    bool dig_clk_gated;
    /* Deep sleep powered the digital domain down: its SRAM is lost */
    bool dig_powered_down;
    uint32_t xtal_apb_freq;
    Esp32SocClkSel soc_clk;
    Esp32FastClkSel rtc_fastclk;
    uint32_t rtc_fastclk_freq;
    Esp32SlowClkSel rtc_slowclk;
    /* RTC_SLOW_CLK's rate; 0 while its source does not run */
    uint32_t rtc_slowclk_freq;
    /* RC_FAST_DIV_CLK and XTAL32K_CLK; 0 while not running */
    uint32_t rc_fast_d256_freq;
    uint32_t xtal32k_freq;
    uint32_t reset_cause[ESP32_CPU_COUNT];
    bool stat_vector_sel[ESP32_CPU_COUNT];
} Esp32RtcCntlState;

/*
 * The SoC has carried out the deep-sleep power-down reset of the digital
 * domain; wakeup sources are watched from here on.
 */
void esp32_rtc_cntl_sleep_reset_done(Esp32RtcCntlState *s);

/* An RTCIO input or wakeup setting changed: re-evaluate the wakeup sources */
void esp32_rtc_cntl_rtcio_changed(Esp32RtcCntlState *s);

/* RTC_CNTL_HOLD_FORCE and RTC_CNTL_DG_PAD_FORCE_HOLD, for RTCIO */
uint32_t esp32_rtc_cntl_hold_force(Esp32RtcCntlState *s);
bool esp32_rtc_cntl_dg_pad_force_hold(Esp32RtcCntlState *s);
