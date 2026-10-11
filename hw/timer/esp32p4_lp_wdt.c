/*
 * ESP32-P4 RTC watchdog and super watchdog (LP_WDT)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 18 (Watchdog Timers) and table 11.1-1 (reset causes), and ESP-IDF's
 * lp_wdt_reg.h for v3, which gives the registers' layout, access and
 * reset values; its hal lpwdt_ll.h and wdt_hal_iram.c and the P4
 * bootloader show how software drives the block.
 *
 * RTC watchdog (RWDT, TRM 18.2). It counts LP_DYN_SLOW_CLK through four
 * stages in a loop, as the MWDT does: in each, its counter counts from 0
 * until it reaches the stage's hold, RTC_WDT_STGn_HOLD, stage 0's shifted
 * left by EFUSE_WDT_DELAY_SEL + 1 (a hold of 0, or one below a count
 * already reached, being reached after a 32-bit counter wraps); then the
 * stage's action is taken, the counter restarts and the next stage
 * begins. The actions (RTC_WDT_STGn): 0, none; 1, an interrupt
 * (LP_WDT_INT_RAW); 2, a CPU reset of core 0 if RTC_WDT_PROCPU_RESET_EN is
 * set and core 1 if APPCPU_RESET_EN is (each core taking it only if its
 * LP_CLKRST_LP_WDT_HPCORE<n>_RESET_EN is set, its length LP_CLKRST's); 3,
 * an HP core reset (cause 0x09); 4, a system reset (cause 0x10); 5 to 7,
 * which the TRM does not name, are logged as guest errors and do nothing.
 * The HP core reset is held for RTC_WDT_SYS_RESET_LENGTH's time, 100 ns to
 * 3.2 us as ESP-IDF's lpwdt_ll names the settings (the TRM gives the unit
 * as LP_DYN_FAST_CLK cycles); RTC_WDT_CPU_RESET_LENGTH is held as written.
 * Writing RTC_WDT_FEED restarts stage 0 from 0; a disabled watchdog holds
 * its stage and count at 0. While RTC_WDT_PAUSE_IN_SLP is set the
 * watchdog pauses as the chip sleeps (the "sleep" input). Flash boot
 * protection: while RTC_WDT_FLASHBOOT_MOD_EN is set and the chip booted in
 * SPI boot mode, the watchdog runs although RTC_WDT_EN is clear, its stage
 * 0 resetting the system. Configuration takes effect at once. CONFIG0 to
 * CONFIG4 and FEED take writes only while RTC_WDT_WKEY holds 0x50D83AA1;
 * out of reset it holds 0, so they are locked.
 *
 * Super watchdog (SWD, TRM 18.3). An analog watchdog on the RC_SLOW
 * oscillator, enabled unless SWD_DISABLE or EFUSE_DIS_SWD is set. It must
 * be fed within its timeout, which the TRM gives only as slightly under a
 * second, with its feed request about 100 ms before: the model takes
 * 960 ms (144000 cycles of the 150 kHz oscillator) and 860 ms (129000).
 * The request sets SUPER_WDT_INT_RAW and, with SWD_AUTO_FEED_EN set, feeds
 * it; software feeds it with SWD_FEED. At the timeout it resets the system
 * (cause 0x12) and sets SWD_RESET_FLAG, which that reset keeps and
 * SWD_RST_FLAG_CLR or a chip reset clears, if LP_SYSTEM's FIB control lets
 * it (the "swd-reset-en" input); otherwise it starts its timeout again.
 * Out of reset the FIB leaves the reset to the analog state, which the
 * model takes not to reset: the boot ROM never feeds the SWD, yet its
 * download mode does not reset every second, and ESP-IDF's P4 bootloader,
 * unlike the C3's and C6's, does not enable the reset
 * (bootloader_ana_super_wdt_reset_config is empty), though the header
 * gives ANA_FIB a reset value of 127. SWD_SIGNAL_WIDTH, the width of the
 * feed signal to the analog circuit, is held as written. SWD_CONFIG takes
 * writes only while SWD_WKEY holds 0x50D83AA1, except SWD_RST_FLAG_CLR,
 * which the boot ROM's main() writes without the key.
 *
 * Both interrupts drive LP_WDT_INTR. The block resets with a system reset
 * and its LP_CLKRST reset, not with an HP core reset. LP_WDT_DATE's
 * CLK_EN, the register file's clock gating, has no effect software can
 * see. Byte and halfword accesses read and write their bytes of a
 * register.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/timer/esp32p4_lp_wdt.h"
#include "migration/vmstate.h"
#include "system/runstate.h"

#define A_CONFIG0           0x00
#define A_CONFIG1           0x04
#define A_CONFIG4           0x10
#define A_FEED              0x14
#define A_WPROTECT          0x18
#define A_SWD_CONFIG        0x1c
#define A_SWD_WPROTECT      0x20
#define A_INT_RAW           0x24
#define A_INT_ST            0x28
#define A_INT_ENA           0x2c
#define A_INT_CLR           0x30
#define A_DATE              0x3fc

#define CONF0_PAUSE_IN_SLP      BIT(9)
#define CONF0_APPCPU_RESET_EN   BIT(10)
#define CONF0_PROCPU_RESET_EN   BIT(11)
#define CONF0_FLASHBOOT_MOD_EN  BIT(12)
#define CONF0_SYS_RESET_LENGTH(v) extract32((v), 13, 3)
#define CONF0_STG(v, n)         extract32((v), 28 - 3 * (n), 3)
#define CONF0_EN                BIT(31)
#define CONF0_RW                0xfffffe00
#define CONF0_RESET             0x00013200
#define FEED_FEED               BIT(31)
#define WDT_WKEY                0x50d83aa1

#define SWD_RESET_FLAG          BIT(0)
#define SWD_AUTO_FEED_EN        BIT(18)
#define SWD_RST_FLAG_CLR        BIT(19)
#define SWD_DISABLE             BIT(30)
#define SWD_FEED                BIT(31)
#define SWD_RW                  0x7ff40000
#define SWD_RESET               0x12c00000

#define INT_SWD                 BIT(30)
#define INT_RWDT                BIT(31)
#define INT_MASK                (INT_SWD | INT_RWDT)
#define DATE_RESET              0x02212060

/* The super watchdog's feed request and timeout, in RC_SLOW cycles. */
#define SWD_REQUEST_CYCLES      129000
#define SWD_TIMEOUT_CYCLES      144000

/* The eFuse words and bits of EFUSE_WDT_DELAY_SEL and EFUSE_DIS_SWD. */
#define EFUSE_WDT_DELAY_SEL_WORD 2
#define EFUSE_WDT_DELAY_SEL_BIT  17
#define EFUSE_DIS_SWD_WORD       5
#define EFUSE_DIS_SWD_BIT        21

enum {
    STG_OFF,
    STG_INT,
    STG_CPU_RESET,
    STG_CORE_RESET,
    STG_SYSTEM_RESET,
};

static const uint32_t rwdt_hold_reset[ESP32P4_LP_WDT_STAGES] = {
    200000, 80000, 4095, 4095,
};

/* The HP core reset's length for each RTC_WDT_SYS_RESET_LENGTH. */
static const uint16_t rwdt_reset_length_ns[8] = {
    100, 200, 300, 400, 500, 800, 1600, 3200,
};

/* The least time in which @clk makes @n cycles, capped for timer_mod(). */
static int64_t lp_wdt_cycles_ns(Clock *clk, uint64_t n)
{
    uint64_t ns = clock_ticks_to_ns(clk, n);

    while (ns < INT64_MAX / 4 && clock_ns_to_ticks(clk, ns) < n) {
        ns++;
    }
    return MIN(ns, INT64_MAX / 4);
}

/*
 * Fold the cycles counted up to @now into @c's count, keeping its sync
 * point on a clock edge.
 */
static void lp_wdt_sync(ESP32P4LPWdtCounter *c, Clock *clk, int64_t now)
{
    uint64_t cycles;

    if (!c->running) {
        c->sync_ns = now;
        return;
    }
    if (now <= c->sync_ns) {
        return;
    }
    cycles = clock_ns_to_ticks(clk, now - c->sync_ns);
    c->count += cycles;
    c->sync_ns += lp_wdt_cycles_ns(clk, cycles);
}

/* Start or stop @c, after a sync at @now, and schedule it for @target. */
static void lp_wdt_run(ESP32P4LPWdtCounter *c, Clock *clk, bool running,
                       uint64_t target, int64_t now)
{
    if (running && !c->running) {
        c->sync_ns = now;
    }
    c->running = running;
    timer_del(&c->timer);
    if (!running) {
        return;
    }
    c->due_ns = c->sync_ns + lp_wdt_cycles_ns(clk, target - c->count);
    timer_mod(&c->timer, c->due_ns);
}

/* Counts from @count to reach @hold, a 32-bit counter wrapping first. */
static uint64_t lp_wdt_ahead(uint64_t hold, uint64_t count)
{
    uint32_t wrap;

    if (hold > count) {
        return hold - count;
    }
    wrap = hold - count;
    return wrap ? wrap : 1ull << 32;
}

static void lp_wdt_update_irq(ESP32P4LPWdtState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) != 0);
}

static uint32_t lp_wdt_efuse(ESP32P4LPWdtState *s, unsigned word,
                             unsigned bit)
{
    return extract32(esp32p4_efuse_word(s->efuse, 0, word), bit, 1);
}

/* The RTC watchdog ---------------------------------------------------- */

/* Flash boot protection: the RTC watchdog runs with RTC_WDT_EN clear. */
static bool rwdt_flashboot(ESP32P4LPWdtState *s)
{
    return !(s->config0 & CONF0_EN) && s->spi_boot &&
           (s->config0 & CONF0_FLASHBOOT_MOD_EN);
}

static bool rwdt_enabled(ESP32P4LPWdtState *s)
{
    return (s->config0 & CONF0_EN) || rwdt_flashboot(s);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static uint64_t rwdt_hold(ESP32P4LPWdtState *s, unsigned stage)
{
    uint64_t hold = s->hold[stage] ? s->hold[stage] : 1ull << 32;

    if (stage == 0) {
        hold <<= lp_wdt_efuse(s, EFUSE_WDT_DELAY_SEL_WORD,
                              EFUSE_WDT_DELAY_SEL_BIT) + 1;
    }
    return hold;
}

/* Re-evaluate whether the RTC watchdog runs, after a sync at @now. */
static void rwdt_update(ESP32P4LPWdtState *s, int64_t now)
{
    ESP32P4LPWdtCounter *c = &s->rwdt;
    bool enabled = rwdt_enabled(s);
    bool running = enabled && clock_is_enabled(s->slow) &&
                   !(s->sleep && (s->config0 & CONF0_PAUSE_IN_SLP));

    if (!enabled) {
        s->stage = 0;
        c->count = 0;
    }
    lp_wdt_run(c, s->slow, running,
               c->count + lp_wdt_ahead(rwdt_hold(s, s->stage), c->count),
               now);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Take a stage's action.
 */
static void rwdt_act(ESP32P4LPWdtState *s, unsigned action)
{
    switch (action) {
    case STG_INT:
        s->int_raw |= INT_RWDT;
        lp_wdt_update_irq(s);
        break;
    case STG_CPU_RESET:
        if (s->config0 & CONF0_PROCPU_RESET_EN) {
            qemu_irq_pulse(s->cpu_reset[0]);
        }
        if (s->config0 & CONF0_APPCPU_RESET_EN) {
            qemu_irq_pulse(s->cpu_reset[1]);
        }
        break;
    case STG_CORE_RESET:
        esp32p4_clkrst_hp_core_reset(s->clkrst, ESP32P4_HP_CORE_RESET_RWDT,
            rwdt_reset_length_ns[CONF0_SYS_RESET_LENGTH(s->config0)]);
        break;
    case STG_SYSTEM_RESET:
        esp32p4_clkrst_set_system_reset_cause(s->clkrst,
                                              ESP32P4_RESET_CAUSE_RWDT_SYS);
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        break;
    default:
        break;
    }
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void rwdt_timeout(ESP32P4LPWdtState *s, int64_t due)
{
    ESP32P4LPWdtCounter *c = &s->rwdt;
    unsigned action;

    lp_wdt_sync(c, s->slow, due);
    if (rwdt_flashboot(s)) {
        action = s->stage ? STG_OFF : STG_SYSTEM_RESET;
    } else {
        action = CONF0_STG(s->config0, s->stage);
    }
    s->stage = (s->stage + 1) % ESP32P4_LP_WDT_STAGES;
    c->count = 0;
    rwdt_update(s, due);
    rwdt_act(s, action);
}

static void rwdt_cb(void *opaque)
{
    ESP32P4LPWdtState *s = opaque;

    rwdt_timeout(s, s->rwdt.due_ns);
}

/* The super watchdog -------------------------------------------------- */

static bool swd_enabled(ESP32P4LPWdtState *s)
{
    return !(s->swd_config & SWD_DISABLE) &&
           !lp_wdt_efuse(s, EFUSE_DIS_SWD_WORD, EFUSE_DIS_SWD_BIT);
}

/* Re-evaluate whether the super watchdog runs, after a sync at @now. */
static void swd_update(ESP32P4LPWdtState *s, int64_t now)
{
    ESP32P4LPWdtCounter *c = &s->swd;
    bool enabled = swd_enabled(s);

    if (!enabled) {
        c->count = 0;
    }
    lp_wdt_run(c, s->swd_clk, enabled && clock_is_enabled(s->swd_clk),
               c->count < SWD_REQUEST_CYCLES ? SWD_REQUEST_CYCLES
                                             : SWD_TIMEOUT_CYCLES, now);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * The super watchdog's feed request, or its timeout, falls due.
 */
static void swd_event(ESP32P4LPWdtState *s, int64_t due)
{
    ESP32P4LPWdtCounter *c = &s->swd;

    lp_wdt_sync(c, s->swd_clk, due);
    if (c->count >= SWD_TIMEOUT_CYCLES) {
        if (s->swd_reset_en) {
            lp_wdt_run(c, s->swd_clk, false, 0, due);
            esp32p4_clkrst_set_system_reset_cause(s->clkrst,
                                                  ESP32P4_RESET_CAUSE_SWD);
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            return;
        }
        c->count = 0;
        swd_update(s, due);
        return;
    }
    s->int_raw |= INT_SWD;
    lp_wdt_update_irq(s);
    if (s->swd_config & SWD_AUTO_FEED_EN) {
        c->count = 0;
    }
    swd_update(s, due);
}

static void swd_cb(void *opaque)
{
    ESP32P4LPWdtState *s = opaque;

    swd_event(s, s->swd.due_ns);
}

/* Run the watchdogs' events that fell due by @now, as of when they did. */
static void lp_wdt_catch_up(ESP32P4LPWdtState *s, int64_t now)
{
    if (timer_pending(&s->swd.timer) && s->swd.due_ns <= now) {
        timer_del(&s->swd.timer);
        swd_event(s, s->swd.due_ns);
    }
    if (timer_pending(&s->rwdt.timer) && s->rwdt.due_ns <= now) {
        timer_del(&s->rwdt.timer);
        rwdt_timeout(s, s->rwdt.due_ns);
    }
}

/* Inputs -------------------------------------------------------------- */

static void lp_wdt_slow_update(void *opaque, ClockEvent event)
{
    ESP32P4LPWdtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        lp_wdt_catch_up(s, now);
        lp_wdt_sync(&s->rwdt, s->slow, now);
        return;
    }
    s->rwdt.sync_ns = now;
    rwdt_update(s, now);
}

static void lp_wdt_swd_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4LPWdtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        lp_wdt_catch_up(s, now);
        lp_wdt_sync(&s->swd, s->swd_clk, now);
        return;
    }
    s->swd.sync_ns = now;
    swd_update(s, now);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_wdt_set_spi_boot(void *opaque, int n, int level)
{
    ESP32P4LPWdtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    lp_wdt_catch_up(s, now);
    lp_wdt_sync(&s->rwdt, s->slow, now);
    s->spi_boot = level;
    rwdt_update(s, now);
}

static void lp_wdt_set_swd_reset_en(void *opaque, int n, int level)
{
    ESP32P4_LP_WDT(opaque)->swd_reset_en = level;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_wdt_set_sleep(void *opaque, int n, int level)
{
    ESP32P4LPWdtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    lp_wdt_catch_up(s, now);
    lp_wdt_sync(&s->rwdt, s->slow, now);
    s->sleep = level;
    rwdt_update(s, now);
}

/* Registers ----------------------------------------------------------- */

/* The register at word offset @addr; false for a reserved offset. */
static bool lp_wdt_read_reg(ESP32P4LPWdtState *s, hwaddr addr, uint32_t *v)
{
    switch (addr) {
    case A_CONFIG0:
        *v = s->config0;
        return true;
    case A_CONFIG1 ... A_CONFIG4:
        *v = s->hold[(addr - A_CONFIG1) / 4];
        return true;
    case A_FEED:
    case A_INT_CLR:
        *v = 0;
        return true;
    case A_WPROTECT:
        *v = s->wkey;
        return true;
    case A_SWD_CONFIG:
        *v = s->swd_config | (s->swd_reset_flag ? SWD_RESET_FLAG : 0);
        return true;
    case A_SWD_WPROTECT:
        *v = s->swd_wkey;
        return true;
    case A_INT_RAW:
        *v = s->int_raw;
        return true;
    case A_INT_ST:
        *v = s->int_raw & s->int_ena;
        return true;
    case A_INT_ENA:
        *v = s->int_ena;
        return true;
    case A_DATE:
        *v = s->date;
        return true;
    default:
        *v = 0;
        return false;
    }
}

static uint64_t lp_wdt_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LPWdtState *s = opaque;
    uint32_t v;

    lp_wdt_catch_up(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    if (!lp_wdt_read_reg(s, addr & ~3, &v)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_WDT, addr);
    }
    return extract32(v, (addr & 3) * 8, size * 8);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_wdt_write_rwdt(ESP32P4LPWdtState *s, hwaddr addr,
                              uint32_t value, uint32_t ones, int64_t now)
{
    int n;

    if (s->wkey != WDT_WKEY) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to RTC watchdog register "
                      "0x%" HWADDR_PRIx " while write-protected\n",
                      TYPE_ESP32P4_LP_WDT, addr);
        return;
    }
    lp_wdt_sync(&s->rwdt, s->slow, now);
    switch (addr) {
    case A_CONFIG0:
        s->config0 = value & CONF0_RW;
        for (n = 0; n < ESP32P4_LP_WDT_STAGES; n++) {
            if (CONF0_STG(s->config0, n) > STG_SYSTEM_RESET) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: RTC_WDT_STG%d %u is "
                              "not an action\n", TYPE_ESP32P4_LP_WDT, n,
                              CONF0_STG(s->config0, n));
            }
        }
        break;
    case A_CONFIG1 ... A_CONFIG4:
        s->hold[(addr - A_CONFIG1) / 4] = value;
        break;
    case A_FEED:
        if (ones & FEED_FEED) {
            s->stage = 0;
            s->rwdt.count = 0;
        }
        break;
    }
    rwdt_update(s, now);
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void lp_wdt_write_swd(ESP32P4LPWdtState *s, uint32_t value,
                             uint32_t ones, int64_t now)
{
    if (ones & SWD_RST_FLAG_CLR) {
        s->swd_reset_flag = false;
    }
    if (s->swd_wkey != WDT_WKEY) {
        if ((value ^ s->swd_config) & SWD_RW || ones & SWD_FEED) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write to SWD_CONFIG while "
                          "write-protected\n", TYPE_ESP32P4_LP_WDT);
        }
        return;
    }
    lp_wdt_sync(&s->swd, s->swd_clk, now);
    s->swd_config = value & SWD_RW;
    if (ones & SWD_FEED) {
        s->swd.count = 0;
    }
    swd_update(s, now);
}

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * Write the register at word offset @addr: @value is its new value, the
 * written byte lanes merged into the old, for its R/W fields; @ones has
 * the bits written as 1, for its write-trigger and write-1-to-clear ones.
 */
static void lp_wdt_write_reg(ESP32P4LPWdtState *s, hwaddr addr,
                             uint32_t value, uint32_t ones, int64_t now)
{
    switch (addr) {
    case A_CONFIG0 ... A_FEED:
        lp_wdt_write_rwdt(s, addr, value, ones, now);
        break;
    case A_WPROTECT:
        s->wkey = value;
        break;
    case A_SWD_CONFIG:
        lp_wdt_write_swd(s, value, ones, now);
        break;
    case A_SWD_WPROTECT:
        s->swd_wkey = value;
        break;
    case A_INT_RAW:
    case A_INT_CLR:
        s->int_raw &= ~(ones & INT_MASK);
        lp_wdt_update_irq(s);
        break;
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        lp_wdt_update_irq(s);
        break;
    case A_DATE:
        s->date = value;
        break;
    case A_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only LP_WDT_INT_ST"
                      "\n", TYPE_ESP32P4_LP_WDT);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to reserved offset 0x%"
                      HWADDR_PRIx "\n", TYPE_ESP32P4_LP_WDT, addr);
        break;
    }
}

static void lp_wdt_write(void *opaque, hwaddr addr, uint64_t data,
                         unsigned size)
{
    ESP32P4LPWdtState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned shift = (addr & 3) * 8;
    uint32_t lanes = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t ones = (data << shift) & lanes;
    uint32_t old;

    lp_wdt_catch_up(s, now);
    lp_wdt_read_reg(s, addr & ~3, &old);
    if ((addr & ~3) == A_SWD_CONFIG) {
        old &= ~SWD_RESET_FLAG;
    }
    lp_wdt_write_reg(s, addr & ~3, (old & ~lanes) | ones, ones, now);
}

static const MemoryRegionOps lp_wdt_ops = {
    .read = lp_wdt_read,
    .write = lp_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/*
 * [spec:nuos:req:emu.esp32p4.timers]
 * SWD_RESET_FLAG outlives the system reset the super watchdog causes and
 * any later system reset, but not a chip reset or the block's own: the
 * reset and clock control still holds a system reset's cause here, before
 * its own reset records it.
 */
static void esp32p4_lp_wdt_reset_enter(Object *obj, ResetType type)
{
    ESP32P4LPWdtState *s = ESP32P4_LP_WDT(obj);
    uint8_t cause = esp32p4_clkrst_system_reset_cause(s->clkrst);

    if (cause == ESP32P4_RESET_CAUSE_SWD) {
        s->swd_reset_flag = true;
    } else if (!cause) {
        s->swd_reset_flag = false;
    }
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static void esp32p4_lp_wdt_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LPWdtState *s = ESP32P4_LP_WDT(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int i;

    s->config0 = CONF0_RESET;
    for (i = 0; i < ESP32P4_LP_WDT_STAGES; i++) {
        s->hold[i] = rwdt_hold_reset[i];
    }
    s->wkey = 0;
    s->swd_config = SWD_RESET;
    s->swd_wkey = 0;
    s->int_raw = 0;
    s->int_ena = 0;
    s->date = DATE_RESET;
    s->stage = 0;
    for (i = 0; i < 2; i++) {
        ESP32P4LPWdtCounter *c = i ? &s->swd : &s->rwdt;

        timer_del(&c->timer);
        c->count = 0;
        c->sync_ns = now;
        c->running = false;
    }
}

static void esp32p4_lp_wdt_reset_exit(Object *obj, ResetType type)
{
    ESP32P4LPWdtState *s = ESP32P4_LP_WDT(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    rwdt_update(s, now);
    swd_update(s, now);
    lp_wdt_update_irq(s);
}

static void esp32p4_lp_wdt_init(Object *obj)
{
    ESP32P4LPWdtState *s = ESP32P4_LP_WDT(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lp_wdt_ops, s, "esp32p4.lp-wdt",
                          ESP32P4_LP_WDT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->slow = qdev_init_clock_in(dev, "slow", lp_wdt_slow_update, s,
                                 ClockPreUpdate | ClockUpdate);
    s->swd_clk = qdev_init_clock_in(dev, "swd", lp_wdt_swd_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    timer_init_ns(&s->rwdt.timer, QEMU_CLOCK_VIRTUAL, rwdt_cb, s);
    timer_init_ns(&s->swd.timer, QEMU_CLOCK_VIRTUAL, swd_cb, s);
    qdev_init_gpio_in_named(dev, lp_wdt_set_spi_boot, ESP32P4_LP_WDT_SPI_BOOT,
                            1);
    qdev_init_gpio_in_named(dev, lp_wdt_set_sleep, ESP32P4_LP_WDT_SLEEP, 1);
    qdev_init_gpio_in_named(dev, lp_wdt_set_swd_reset_en,
                            ESP32P4_LP_WDT_SWD_RESET_EN, 1);
    qdev_init_gpio_out_named(dev, s->cpu_reset, ESP32P4_LP_WDT_CPU_RESET,
                             ESP32P4_CLKRST_HP_CORES);
}

static void esp32p4_lp_wdt_realize(DeviceState *dev, Error **errp)
{
    ESP32P4LPWdtState *s = ESP32P4_LP_WDT(dev);

    if (!s->clkrst || !s->efuse) {
        error_setg(errp, "%s: the \"clkrst\" and \"efuse\" links must be "
                   "set", TYPE_ESP32P4_LP_WDT);
    }
}

static const VMStateDescription vmstate_esp32p4_lp_wdt_counter = {
    .name = TYPE_ESP32P4_LP_WDT "/counter",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, ESP32P4LPWdtCounter),
        VMSTATE_UINT64(count, ESP32P4LPWdtCounter),
        VMSTATE_INT64(sync_ns, ESP32P4LPWdtCounter),
        VMSTATE_BOOL(running, ESP32P4LPWdtCounter),
        VMSTATE_INT64(due_ns, ESP32P4LPWdtCounter),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_esp32p4_lp_wdt = {
    .name = TYPE_ESP32P4_LP_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(slow, ESP32P4LPWdtState),
        VMSTATE_CLOCK(swd_clk, ESP32P4LPWdtState),
        VMSTATE_UINT32(config0, ESP32P4LPWdtState),
        VMSTATE_UINT32_ARRAY(hold, ESP32P4LPWdtState, ESP32P4_LP_WDT_STAGES),
        VMSTATE_UINT32(wkey, ESP32P4LPWdtState),
        VMSTATE_UINT32(swd_config, ESP32P4LPWdtState),
        VMSTATE_UINT32(swd_wkey, ESP32P4LPWdtState),
        VMSTATE_UINT32(int_raw, ESP32P4LPWdtState),
        VMSTATE_UINT32(int_ena, ESP32P4LPWdtState),
        VMSTATE_UINT32(date, ESP32P4LPWdtState),
        VMSTATE_UINT32(stage, ESP32P4LPWdtState),
        VMSTATE_STRUCT(rwdt, ESP32P4LPWdtState, 1,
                       vmstate_esp32p4_lp_wdt_counter, ESP32P4LPWdtCounter),
        VMSTATE_STRUCT(swd, ESP32P4LPWdtState, 1,
                       vmstate_esp32p4_lp_wdt_counter, ESP32P4LPWdtCounter),
        VMSTATE_BOOL(swd_reset_flag, ESP32P4LPWdtState),
        VMSTATE_BOOL(spi_boot, ESP32P4LPWdtState),
        VMSTATE_BOOL(sleep, ESP32P4LPWdtState),
        VMSTATE_BOOL(swd_reset_en, ESP32P4LPWdtState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property esp32p4_lp_wdt_properties[] = {
    DEFINE_PROP_LINK("clkrst", ESP32P4LPWdtState, clkrst,
                     TYPE_ESP32P4_CLKRST, ESP32P4ClkRstState *),
    DEFINE_PROP_LINK("efuse", ESP32P4LPWdtState, efuse, TYPE_ESP32P4_EFUSE,
                     ESP32P4EfuseState *),
};

static void esp32p4_lp_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32p4_lp_wdt_realize;
    dc->vmsd = &vmstate_esp32p4_lp_wdt;
    device_class_set_props(dc, esp32p4_lp_wdt_properties);
    rc->phases.enter = esp32p4_lp_wdt_reset_enter;
    rc->phases.hold = esp32p4_lp_wdt_reset_hold;
    rc->phases.exit = esp32p4_lp_wdt_reset_exit;
}

/* [spec:nuos:req:emu.esp32p4.timers] */
static const TypeInfo esp32p4_lp_wdt_info = {
    .name          = TYPE_ESP32P4_LP_WDT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4LPWdtState),
    .instance_init = esp32p4_lp_wdt_init,
    .class_init    = esp32p4_lp_wdt_class_init,
};

static void esp32p4_lp_wdt_register_types(void)
{
    type_register_static(&esp32p4_lp_wdt_info);
}
type_init(esp32p4_lp_wdt_register_types)
