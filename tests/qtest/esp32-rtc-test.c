/*
 * QTest testcase for the ESP32 RTC_CNTL and RTCIO
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RTC_CNTL                0x3ff48000
#define OPTIONS0                (RTC_CNTL + 0x00)
#define SLP_TIMER0              (RTC_CNTL + 0x04)
#define SLP_TIMER1              (RTC_CNTL + 0x08)
#define TIME_UPDATE             (RTC_CNTL + 0x0c)
#define TIME0                   (RTC_CNTL + 0x10)
#define TIME1                   (RTC_CNTL + 0x14)
#define STATE0                  (RTC_CNTL + 0x18)
#define RESET_STATE             (RTC_CNTL + 0x34)
#define WAKEUP_STATE            (RTC_CNTL + 0x38)
#define INT_ENA                 (RTC_CNTL + 0x3c)
#define INT_RAW                 (RTC_CNTL + 0x40)
#define INT_ST                  (RTC_CNTL + 0x44)
#define INT_CLR                 (RTC_CNTL + 0x48)
#define STORE0                  (RTC_CNTL + 0x4c)
#define EXT_WAKEUP_CONF         (RTC_CNTL + 0x60)
#define SLP_REJECT_CONF         (RTC_CNTL + 0x64)
#define CLK_CONF                (RTC_CNTL + 0x70)
#define PWC                     (RTC_CNTL + 0x80)
#define DIG_PWC                 (RTC_CNTL + 0x84)
#define WDTCONFIG0              (RTC_CNTL + 0x8c)
#define WDTCONFIG1              (RTC_CNTL + 0x90)
#define WDTFEED                 (RTC_CNTL + 0xa0)
#define WDTWPROTECT             (RTC_CNTL + 0xa4)
#define LOW_POWER_ST            (RTC_CNTL + 0xc0)
#define HOLD_FORCE              (RTC_CNTL + 0xc8)
#define EXT_WAKEUP1             (RTC_CNTL + 0xcc)
#define EXT_WAKEUP1_STATUS      (RTC_CNTL + 0xd0)
#define BROWN_OUT               (RTC_CNTL + 0xd4)
#define MEM_CONF                (RTC_CNTL + 0x100)
#define MEM_CRC_RES             (RTC_CNTL + 0x104)
#define RTC_CNTL_DATE           (RTC_CNTL + 0x13c)

#define TIME_UPDATE_BIT         (1u << 31)
#define TIME_VALID              (1u << 30)
#define ALARM_EN                (1u << 16)
#define SLEEP_EN                (1u << 31)
#define DG_WRAP_PD_EN           (1u << 31)
#define WAKEUP_ENA(m)           ((m) << 11)
#define WAKEUP_CAUSE(v)         ((v) & 0x7ff)
#define WAKE_EXT0               0x1
#define WAKE_EXT1               0x2
#define WAKE_GPIO               0x4
#define WAKE_TIMER              0x8
#define INT_SLP_WAKEUP          (1u << 0)
#define INT_SLP_REJECT          (1u << 1)
#define INT_WDT                 (1u << 3)
#define INT_TIME_VALID          (1u << 4)
#define INT_MAIN_TIMER          (1u << 8)
#define EXT_WAKEUP0_LV          (1u << 30)
#define EXT_WAKEUP1_LV          (1u << 31)
#define GPIO_REJECT_EN          (1u << 24)
#define LIGHT_SLP_REJECT_EN     (1u << 26)
#define REJECT_CAUSE(v)         ((v) >> 28)
#define FASTMEM_PD_EN           (1u << 14)
#define FASTMEM_FORCE_PU        (1u << 13)
#define MAIN_STATE_IN_IDLE      (1u << 27)
#define RDY_FOR_WAKEUP          (1u << 19)
#define MEM_CRC_START           (1u << 8)
#define MEM_CRC_FINISH          (1u << 31)

#define WDT_WKEY                0x50d83aa1
#define WDT_EN                  (1u << 31)
#define WDT_STG0(a)             ((a) << 28)
#define WDT_LEVEL_INT_EN        (1u << 17)
#define WDT_PROCPU_RESET_EN     (1u << 9)
#define WDT_FLASHBOOT_MOD_EN    (1u << 10)

#define RTCIO                   0x3ff48400
#define RTC_GPIO_OUT            (RTCIO + 0x00)
#define RTC_GPIO_OUT_W1TS       (RTCIO + 0x04)
#define RTC_GPIO_OUT_W1TC       (RTCIO + 0x08)
#define RTC_GPIO_ENABLE         (RTCIO + 0x0c)
#define RTC_GPIO_ENABLE_W1TS    (RTCIO + 0x10)
#define RTC_GPIO_STATUS         (RTCIO + 0x18)
#define RTC_GPIO_STATUS_W1TC    (RTCIO + 0x20)
#define RTC_GPIO_IN             (RTCIO + 0x24)
#define RTC_GPIO_PIN(n)         (RTCIO + 0x28 + 4 * (n))
#define RTCIO_PAD_DAC1          (RTCIO + 0x84)
#define RTCIO_XTAL_32K_PAD      (RTCIO + 0x8c)
#define RTCIO_TOUCH_CFG         (RTCIO + 0x90)
#define RTCIO_TOUCH_PAD(n)      (RTCIO + 0x94 + 4 * (n))
#define RTCIO_EXT_WAKEUP0       (RTCIO + 0xbc)
#define RTCIO_DATE              (RTCIO + 0xc8)
#define PDAC_RUE                (1u << 27)
#define PDAC_MUX_SEL            (1u << 17)
#define PDAC_FUN_IE             (1u << 11)
#define PIN_PAD_DRIVER          (1u << 2)
#define PIN_INT_TYPE(t)         ((t) << 7)
#define PIN_WAKEUP_ENABLE       (1u << 10)
/* GPIO25 is RTC GPIO 6, on the DAC1 pad, whose RTC_CNTL hold bit is 2 */
#define RTC_DAC1                6
#define PAD_DAC1                25
#define HOLD_FORCE_PDAC1        (1u << 2)
#define RTC_BIT(n)              (1u << (14 + (n)))

#define GPIO                    0x3ff44000
#define GPIO_ENABLE             (GPIO + 0x20)
#define GPIO_IN                 (GPIO + 0x3c)
#define GPIO_PIN(n)             (GPIO + 0x88 + 4 * (n))

#define DRAM_WORD               0x3ffb0000
#define RTC_FAST_MEM            0x3ff80000
#define RTC_SLOW_MEM            0x50000000

#define TIMG0_WDTCONFIG0        0x3ff5f048
#define TIMG0_WDTWPROTECT       0x3ff5f064

#define GPIO_PATH               "/machine/soc/gpio"
#define PAD_IN                  "esp32-gpio-pad-in"
#define PAD_OUT                 "esp32-gpio-pad"


#define POWERON_RESET           1
#define DEEPSLEEP_RESET         5
#define RTCWDT_SYS_RESET        9
#define RTCWDT_CPU_RESET        13
#define RTCWDT_RTC_RESET        16

/* One RTC_SLOW_CLK cycle of the 150 kHz RC_SLOW_CLK, rounded up */
#define SLOW_TICK_NS            6667

static QTestState *start(void)
{
    return qtest_init("-M esp32 -nic none");
}

static uint32_t rd(QTestState *qts, uint64_t a)
{
    return qtest_readl(qts, a);
}

static void wr(QTestState *qts, uint64_t a, uint32_t v)
{
    qtest_writel(qts, a, v);
}

static uint64_t rtc_time(QTestState *qts)
{
    wr(qts, TIME_UPDATE, TIME_UPDATE_BIT);
    qtest_clock_step(qts, SLOW_TICK_NS);
    g_assert_cmphex(rd(qts, TIME_UPDATE) & TIME_VALID, ==, TIME_VALID);
    return rd(qts, TIME0) | ((uint64_t)rd(qts, TIME1) << 32);
}

static void reset_cause(QTestState *qts, uint32_t pro, uint32_t app)
{
    uint32_t st = rd(qts, RESET_STATE);

    g_assert_cmpuint(st & 0x3f, ==, pro);
    g_assert_cmpuint((st >> 6) & 0x3f, ==, app);
}

/* Stop the flash-boot protection, which would reset the RTC after 0.85 s */
static void stop_flashboot_wdt(QTestState *qts)
{
    wr(qts, WDTWPROTECT, WDT_WKEY);
    wr(qts, WDTCONFIG0, rd(qts, WDTCONFIG0) & ~WDT_FLASHBOOT_MOD_EN);
    wr(qts, WDTWPROTECT, 0);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_reset_values(void)
{
    static const struct {
        uint32_t addr;
        uint32_t value;
    } regs[] = {
        { OPTIONS0, 0x1c492000 },
        { STATE0, 0x00300000 },
        { RTC_CNTL + 0x1c, 0x28140403 },
        { RTC_CNTL + 0x20, 0x01080000 },
        { RTC_CNTL + 0x24, 0x14160a08 },
        { RTC_CNTL + 0x28, 0x10200a08 },
        { RTC_CNTL + 0x2c, 0x12148001 },
        { RTC_CNTL + 0x30, 0x00800000 },
        { RESET_STATE, 0x00003000 | POWERON_RESET | (POWERON_RESET << 6) },
        { WAKEUP_STATE, 0x00006000 },
        { INT_ENA, 0 },
        { INT_RAW, 0 },
        { CLK_CONF, 0x00002210 },
        { RTC_CNTL + 0x74, 0x02a00000 },
        { RTC_CNTL + 0x7c, 0xa9002400 },
        { PWC, 0x00012925 },
        { DIG_PWC, 0x00155550 },
        { RTC_CNTL + 0x88, 0xaaaa5000 },
        { WDTCONFIG0, 0x00004c80 },
        { WDTCONFIG1, 128000 },
        { RTC_CNTL + 0x94, 80000 },
        { RTC_CNTL + 0x98, 0xfff },
        { RTC_CNTL + 0x9c, 0xfff },
        { WDTWPROTECT, WDT_WKEY },
        { LOW_POWER_ST, MAIN_STATE_IN_IDLE },
        { BROWN_OUT, 0x13ff0000 },
        { RTC_CNTL_DATE, 0x01604280 },
        { RTC_GPIO_OUT, 0 },
        { RTCIO_PAD_DAC1, 0x80000000 },
        { RTCIO_XTAL_32K_PAD, 0x84100010 },
        { RTCIO_TOUCH_CFG, 0x66000000 },
        { RTCIO_TOUCH_PAD(0), 0x52000000 },
        { RTCIO_TOUCH_PAD(1), 0x4a000000 },
        { RTCIO_TOUCH_PAD(7), 0x42000000 },
        { RTCIO_DATE, 0x01603160 },
    };
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(rd(qts, regs[i].addr), ==, regs[i].value);
    }
    /* Write-only and read-only bits */
    wr(qts, OPTIONS0, 0xffffffff & ~((1u << 31) | (3u << 4)));
    g_assert_cmphex(rd(qts, OPTIONS0), ==, 0x7fffffcf);
    wr(qts, INT_RAW, 0xffffffff);
    g_assert_cmphex(rd(qts, INT_RAW), ==, 0);
    wr(qts, RTC_GPIO_OUT, 0xffffffff);
    g_assert_cmphex(rd(qts, RTC_GPIO_OUT), ==, 0xffffc000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_timer_and_alarm(void)
{
    QTestState *qts = start();
    uint64_t t0, t1;

    stop_flashboot_wdt(qts);
    g_assert_cmphex(rd(qts, TIME_UPDATE), ==, 0);
    t0 = rtc_time(qts);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    t1 = rtc_time(qts);
    /* 100 ms of RC_SLOW_CLK, 150 kHz */
    g_assert_cmpuint(t1 - t0, >=, 15000);
    g_assert_cmpuint(t1 - t0, <=, 15002);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_TIME_VALID, ==, INT_TIME_VALID);

    /* TIME_VALID drops at TIME_UPDATE until the latch is in */
    wr(qts, TIME_UPDATE, TIME_UPDATE_BIT);
    g_assert_cmphex(rd(qts, TIME_UPDATE) & TIME_VALID, ==, 0);
    qtest_clock_step(qts, SLOW_TICK_NS);
    g_assert_cmphex(rd(qts, TIME_UPDATE) & TIME_VALID, ==, TIME_VALID);

    /* The alarm fires as the timer reaches it */
    wr(qts, INT_CLR, 0x1ff);
    wr(qts, INT_ENA, INT_MAIN_TIMER);
    t0 = rtc_time(qts);
    wr(qts, SLP_TIMER0, (uint32_t)(t0 + 1500));
    wr(qts, SLP_TIMER1, ALARM_EN | (uint32_t)((t0 + 1500) >> 32));
    qtest_clock_step(qts, 9 * 1000 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_MAIN_TIMER, ==, 0);
    qtest_clock_step(qts, 2 * 1000 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_MAIN_TIMER, ==, INT_MAIN_TIMER);
    g_assert_cmphex(rd(qts, INT_ST), ==, INT_MAIN_TIMER);
    wr(qts, INT_CLR, INT_MAIN_TIMER);
    g_assert_cmphex(rd(qts, INT_ST), ==, 0);

    /* An alarm already passed does not fire */
    t0 = rtc_time(qts);
    wr(qts, SLP_TIMER0, (uint32_t)(t0 - 10));
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_MAIN_TIMER, ==, 0);

    /* Selecting the 8MD256 slow clock: 31.25 kHz */
    t0 = rtc_time(qts);
    wr(qts, CLK_CONF, 0x00002210 | (2u << 30));
    qtest_clock_step(qts, 100 * 1000 * 1000);
    wr(qts, TIME_UPDATE, TIME_UPDATE_BIT);
    qtest_clock_step(qts, 40000);
    t1 = rd(qts, TIME0);
    g_assert_cmpuint(t1 - t0, >=, 3125);
    g_assert_cmpuint(t1 - t0, <=, 3128);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_watchdog(void)
{
    QTestState *qts = start();

    /*
     * Flash boot protection: an RTC reset after STG0_HOLD slow cycles,
     * with TIMG0's, which would expire first, stopped.
     */
    wr(qts, TIMG0_WDTWPROTECT, WDT_WKEY);
    wr(qts, TIMG0_WDTCONFIG0, 0);
    qtest_clock_step(qts, 850 * 1000 * 1000);
    reset_cause(qts, POWERON_RESET, POWERON_RESET);
    qtest_clock_step(qts, 5 * 1000 * 1000);
    reset_cause(qts, RTCWDT_RTC_RESET, RTCWDT_RTC_RESET);
    stop_flashboot_wdt(qts);

    /* Locked, the watchdog's registers ignore writes */
    wr(qts, WDTCONFIG1, 1234);
    g_assert_cmpuint(rd(qts, WDTCONFIG1), ==, 128000);
    wr(qts, WDTWPROTECT, WDT_WKEY);
    wr(qts, WDTCONFIG1, 150);
    g_assert_cmpuint(rd(qts, WDTCONFIG1), ==, 150);

    /* Interrupt stage, fed halfway */
    wr(qts, WDTCONFIG0, WDT_EN | WDT_STG0(1) | WDT_LEVEL_INT_EN);
    qtest_clock_step(qts, 700 * 1000);
    wr(qts, WDTFEED, 1u << 31);
    qtest_clock_step(qts, 700 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_WDT, ==, 0);
    qtest_clock_step(qts, 400 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_WDT, ==, INT_WDT);
    wr(qts, INT_CLR, INT_WDT);

    /* A locked feed is ignored */
    wr(qts, WDTCONFIG0, 0);
    wr(qts, WDTCONFIG0, WDT_EN | WDT_STG0(1) | WDT_LEVEL_INT_EN);
    wr(qts, WDTWPROTECT, 0);
    qtest_clock_step(qts, 700 * 1000);
    wr(qts, WDTFEED, 1u << 31);
    qtest_clock_step(qts, 400 * 1000);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_WDT, ==, INT_WDT);

    /* CPU reset of the PRO CPU only */
    wr(qts, WDTWPROTECT, WDT_WKEY);
    wr(qts, WDTCONFIG0, 0);
    wr(qts, WDTCONFIG0, WDT_EN | WDT_STG0(2) | WDT_PROCPU_RESET_EN);
    qtest_clock_step(qts, 1100 * 1000);
    reset_cause(qts, RTCWDT_CPU_RESET, RTCWDT_RTC_RESET);

    /* System reset: the RTC domain stays */
    wr(qts, STORE0, 0x55aa55aa);
    wr(qts, WDTCONFIG0, 0);
    wr(qts, WDTCONFIG0, WDT_EN | WDT_STG0(3));
    qtest_clock_step(qts, 1100 * 1000);
    reset_cause(qts, RTCWDT_SYS_RESET, RTCWDT_SYS_RESET);
    g_assert_cmphex(rd(qts, STORE0), ==, 0x55aa55aa);
    g_assert_cmphex(rd(qts, WDTCONFIG0), ==, WDT_EN | WDT_STG0(3));

    /* RTC reset: the RTC domain resets too, but not the RTC timer */
    wr(qts, WDTCONFIG0, 0);
    wr(qts, WDTCONFIG0, WDT_EN | WDT_STG0(4));
    qtest_clock_step(qts, 1100 * 1000);
    reset_cause(qts, RTCWDT_RTC_RESET, RTCWDT_RTC_RESET);
    g_assert_cmphex(rd(qts, STORE0), ==, 0);
    g_assert_cmphex(rd(qts, WDTCONFIG0), ==, 0x00004c80);
    g_assert_cmpuint(rtc_time(qts), >, 120000);
    qtest_quit(qts);
}

static uint32_t crc32_ieee(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xffffffff;

    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }
    return ~crc;
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_mem_crc(void)
{
    QTestState *qts = start();
    uint8_t buf[16];

    for (int i = 0; i < 4; i++) {
        wr(qts, RTC_FAST_MEM + 0x40 + 4 * i, 0x01020304 * (i + 1));
    }
    qtest_memread(qts, RTC_FAST_MEM + 0x40, buf, sizeof(buf));
    /* Words 0x10 to 0x13 */
    wr(qts, MEM_CONF, (0x10 << 9) | (3 << 20));
    wr(qts, MEM_CONF, (0x10 << 9) | (3 << 20) | MEM_CRC_START);
    g_assert_cmphex(rd(qts, MEM_CONF) & MEM_CRC_FINISH, ==, MEM_CRC_FINISH);
    g_assert_cmphex(rd(qts, MEM_CRC_RES), ==, crc32_ieee(buf, sizeof(buf)));
    wr(qts, MEM_CONF, (0x10 << 9) | (3 << 20));
    g_assert_cmphex(rd(qts, MEM_CONF), ==, (0x10 << 9) | (3 << 20));

    /* The slow memory is on the APB */
    wr(qts, RTC_SLOW_MEM + 0x1ffc, 0xdeadbeef);
    g_assert_cmphex(rd(qts, RTC_SLOW_MEM + 0x1ffc), ==, 0xdeadbeef);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_deep_sleep(void)
{
    QTestState *qts = start();
    uint64_t t;

    stop_flashboot_wdt(qts);
    wr(qts, STORE0, 0x11223344);
    wr(qts, RTC_SLOW_MEM, 0x5105);
    wr(qts, RTC_FAST_MEM, 0xfa57);
    wr(qts, DRAM_WORD, 0x12345678);
    wr(qts, GPIO_ENABLE, 1u << 18);

    t = rtc_time(qts);
    wr(qts, SLP_TIMER0, (uint32_t)(t + 3000));
    wr(qts, SLP_TIMER1, ALARM_EN);
    wr(qts, WAKEUP_STATE, WAKEUP_ENA(WAKE_TIMER));
    wr(qts, DIG_PWC, rd(qts, DIG_PWC) | DG_WRAP_PD_EN);
    wr(qts, STATE0, rd(qts, STATE0) | SLEEP_EN);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, RDY_FOR_WAKEUP);
    g_assert_cmphex(rd(qts, STATE0) & SLEEP_EN, ==, SLEEP_EN);
    /* The digital domain went down at once */
    g_assert_cmphex(rd(qts, GPIO_ENABLE), ==, 0);

    qtest_clock_step(qts, 20 * 1000 * 1000);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, MAIN_STATE_IN_IDLE);
    g_assert_cmphex(WAKEUP_CAUSE(rd(qts, WAKEUP_STATE)), ==, WAKE_TIMER);
    g_assert_cmphex(rd(qts, STATE0) & SLEEP_EN, ==, 0);
    reset_cause(qts, DEEPSLEEP_RESET, DEEPSLEEP_RESET);
    g_assert_cmphex(rd(qts, STORE0), ==, 0x11223344);
    g_assert_cmphex(rd(qts, RTC_SLOW_MEM), ==, 0x5105);
    g_assert_cmphex(rd(qts, RTC_FAST_MEM), ==, 0xfa57);
    g_assert_cmphex(rd(qts, DRAM_WORD), ==, 0xa5a5a5a5);

    /* A fast memory left to power down loses its contents */
    wr(qts, PWC, (rd(qts, PWC) & ~FASTMEM_FORCE_PU) | FASTMEM_PD_EN);
    t = rtc_time(qts);
    wr(qts, SLP_TIMER0, (uint32_t)(t + 300));
    wr(qts, STATE0, rd(qts, STATE0) | SLEEP_EN);
    qtest_clock_step(qts, 5 * 1000 * 1000);
    reset_cause(qts, DEEPSLEEP_RESET, DEEPSLEEP_RESET);
    g_assert_cmphex(rd(qts, RTC_FAST_MEM), ==, 0xa5a5a5a5);
    g_assert_cmphex(rd(qts, RTC_SLOW_MEM), ==, 0x5105);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_light_sleep_wakeups(void)
{
    QTestState *qts = start();

    stop_flashboot_wdt(qts);

    /* EXT0 on RTC GPIO 6, high */
    wr(qts, RTCIO_PAD_DAC1, rd(qts, RTCIO_PAD_DAC1) | PDAC_MUX_SEL |
       PDAC_FUN_IE);
    wr(qts, RTCIO_EXT_WAKEUP0, RTC_DAC1 << 27);
    wr(qts, EXT_WAKEUP_CONF, EXT_WAKEUP0_LV);
    wr(qts, WAKEUP_STATE, WAKEUP_ENA(WAKE_EXT0));
    wr(qts, STATE0, rd(qts, STATE0) | SLEEP_EN);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, RDY_FOR_WAKEUP);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, PAD_DAC1, 1);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, MAIN_STATE_IN_IDLE);
    g_assert_cmphex(WAKEUP_CAUSE(rd(qts, WAKEUP_STATE)), ==, WAKE_EXT0);
    g_assert_cmphex(rd(qts, INT_RAW) & INT_SLP_WAKEUP, ==, INT_SLP_WAKEUP);
    reset_cause(qts, POWERON_RESET, POWERON_RESET);
    wr(qts, INT_CLR, INT_SLP_WAKEUP);

    /* EXT1, all selected pads low */
    wr(qts, EXT_WAKEUP_CONF, 0);
    wr(qts, EXT_WAKEUP1, 1u << RTC_DAC1);
    wr(qts, WAKEUP_STATE, WAKEUP_ENA(WAKE_EXT1));
    wr(qts, STATE0, rd(qts, STATE0) | SLEEP_EN);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, RDY_FOR_WAKEUP);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, PAD_DAC1, 0);
    g_assert_cmphex(WAKEUP_CAUSE(rd(qts, WAKEUP_STATE)), ==, WAKE_EXT1);
    g_assert_cmphex(rd(qts, EXT_WAKEUP1_STATUS), ==, 1u << RTC_DAC1);
    wr(qts, EXT_WAKEUP1, 1u << 18);
    g_assert_cmphex(rd(qts, EXT_WAKEUP1_STATUS), ==, 0);
    wr(qts, EXT_WAKEUP1, 0);

    /* A digital GPIO's level wakeup: GPIO18 high */
    wr(qts, GPIO_PIN(18), PIN_WAKEUP_ENABLE | PIN_INT_TYPE(5));
    wr(qts, WAKEUP_STATE, WAKEUP_ENA(WAKE_GPIO));
    wr(qts, STATE0, rd(qts, STATE0) | SLEEP_EN);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, RDY_FOR_WAKEUP);
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, 18, 1);
    g_assert_cmphex(WAKEUP_CAUSE(rd(qts, WAKEUP_STATE)), ==, WAKE_GPIO);

    /* ...which, pending, rejects the next sleep when rejection is on */
    wr(qts, INT_CLR, INT_SLP_WAKEUP | INT_SLP_REJECT);
    wr(qts, SLP_REJECT_CONF, GPIO_REJECT_EN | LIGHT_SLP_REJECT_EN);
    wr(qts, STATE0, rd(qts, STATE0) | SLEEP_EN);
    g_assert_cmphex(rd(qts, INT_RAW) & (INT_SLP_WAKEUP | INT_SLP_REJECT), ==,
                    INT_SLP_REJECT);
    g_assert_cmpuint(REJECT_CAUSE(rd(qts, SLP_REJECT_CONF)), ==, 2);
    g_assert_cmphex(rd(qts, STATE0) & SLEEP_EN, ==, 0);
    g_assert_cmphex(rd(qts, LOW_POWER_ST), ==, MAIN_STATE_IN_IDLE);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.rtc/test] */
static void test_rtcio_pads(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_out_named(qts, GPIO_PATH, PAD_OUT);

    /*
     * Under the IO_MUX, GPIO25 has its input off, and RTCIO's pull-up does
     * not apply. GPIO0, GPIO14 and GPIO15 come out of reset with input and
     * pull-up on.
     */
    g_assert_cmphex(rd(qts, RTC_GPIO_IN), ==,
                    RTC_BIT(11) | RTC_BIT(13) | RTC_BIT(16));
    wr(qts, RTCIO_PAD_DAC1, rd(qts, RTCIO_PAD_DAC1) | PDAC_RUE);
    g_assert_cmphex(rd(qts, RTC_GPIO_IN) & RTC_BIT(RTC_DAC1), ==, 0);
    g_assert_false(qtest_get_irq(qts, PAD_DAC1));

    /* MUX_SEL hands it to the RTC IO MUX: pulled up, input on */
    wr(qts, RTCIO_PAD_DAC1, rd(qts, RTCIO_PAD_DAC1) | PDAC_MUX_SEL |
       PDAC_FUN_IE);
    g_assert_true(qtest_get_irq(qts, PAD_DAC1));
    g_assert_cmphex(rd(qts, RTC_GPIO_IN) & RTC_BIT(RTC_DAC1), ==,
                    RTC_BIT(RTC_DAC1));
    g_assert_cmphex(rd(qts, GPIO_IN) & (1u << PAD_DAC1), ==, 1u << PAD_DAC1);

    /* RTC GPIO output, push-pull then open drain */
    wr(qts, RTC_GPIO_ENABLE_W1TS, RTC_BIT(RTC_DAC1));
    g_assert_false(qtest_get_irq(qts, PAD_DAC1));
    wr(qts, RTC_GPIO_OUT_W1TS, RTC_BIT(RTC_DAC1));
    g_assert_true(qtest_get_irq(qts, PAD_DAC1));
    g_assert_cmphex(rd(qts, RTC_GPIO_OUT), ==, RTC_BIT(RTC_DAC1));
    wr(qts, RTC_GPIO_PIN(RTC_DAC1), PIN_PAD_DRIVER);
    wr(qts, RTCIO_PAD_DAC1, rd(qts, RTCIO_PAD_DAC1) & ~PDAC_RUE);
    g_assert_false(qtest_get_irq(qts, PAD_DAC1));
    wr(qts, RTC_GPIO_PIN(RTC_DAC1), 0);
    g_assert_true(qtest_get_irq(qts, PAD_DAC1));

    /* Held, the pad keeps its output whatever the registers say */
    wr(qts, HOLD_FORCE, HOLD_FORCE_PDAC1);
    wr(qts, RTC_GPIO_OUT_W1TC, RTC_BIT(RTC_DAC1));
    g_assert_true(qtest_get_irq(qts, PAD_DAC1));
    wr(qts, HOLD_FORCE, 0);
    g_assert_false(qtest_get_irq(qts, PAD_DAC1));

    /* Interrupt status latches by INT_TYPE: rising edge */
    wr(qts, RTC_GPIO_PIN(RTC_DAC1), PIN_INT_TYPE(1));
    wr(qts, RTC_GPIO_OUT_W1TS, RTC_BIT(RTC_DAC1));
    g_assert_cmphex(rd(qts, RTC_GPIO_STATUS), ==, RTC_BIT(RTC_DAC1));
    wr(qts, RTC_GPIO_STATUS_W1TC, RTC_BIT(RTC_DAC1));
    g_assert_cmphex(rd(qts, RTC_GPIO_STATUS), ==, 0);

    /* Back to the IO_MUX: undriven, and its input off again */
    wr(qts, RTCIO_PAD_DAC1, rd(qts, RTCIO_PAD_DAC1) & ~PDAC_MUX_SEL);
    g_assert_false(qtest_get_irq(qts, PAD_DAC1));
    g_assert_cmphex(rd(qts, RTC_GPIO_IN) & RTC_BIT(RTC_DAC1), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32-rtc/reset-values", test_reset_values);
    qtest_add_func("esp32-rtc/timer-and-alarm", test_timer_and_alarm);
    qtest_add_func("esp32-rtc/watchdog", test_watchdog);
    qtest_add_func("esp32-rtc/mem-crc", test_mem_crc);
    qtest_add_func("esp32-rtc/deep-sleep", test_deep_sleep);
    qtest_add_func("esp32-rtc/light-sleep-wakeups", test_light_sleep_wakeups);
    qtest_add_func("esp32-rtc/rtcio-pads", test_rtcio_pads);

    return g_test_run();
}
