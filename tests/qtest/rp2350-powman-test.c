/*
 * QTest testcase for the RP2350 power manager and always-on timer
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define POWMAN              0x40100000
#define BADPASSWD           0x00
#define VREG_CTRL           0x04
#define VREG_STS            0x08
#define VREG                0x0c
#define VREG_LP_ENTRY       0x10
#define VREG_LP_EXIT        0x14
#define BOD                 0x1c
#define LPOSC               0x28
#define CHIP_RESET          0x2c
#define WDSEL               0x30
#define SEQ_CFG             0x34
#define STATE               0x38
#define POW_FASTDIV         0x3c
#define POW_DELAY           0x40
#define EXT_CTRL0           0x44
#define EXT_TIME_REF        0x4c
#define LPOSC_FREQ_KHZ_INT  0x50
#define LPOSC_FREQ_KHZ_FRAC 0x54
#define XOSC_FREQ_KHZ_INT   0x58
#define SET_TIME(n)         (0x60 + 4 * (3 - (n)))
#define READ_TIME_UPPER     0x70
#define READ_TIME_LOWER     0x74
#define ALARM_TIME(n)       (0x78 + 4 * (3 - (n)))
#define TIMER               0x88
#define PWRUP(n)            (0x8c + 4 * (n))
#define CURRENT_PWRUP_REQ   0x9c
#define LAST_SWCORE_PWRUP   0xa0
#define BOOTDIS             0xa8
#define SCRATCH(n)          (0xb0 + 4 * (n))
#define BOOT(n)             (0xd0 + 4 * (n))
#define INTR                0xe0
#define INTE                0xe4
#define INTF                0xe8
#define INTS                0xec

#define PASSWORD            0x5afe0000u

#define XOR                 0x1000
#define SET                 0x2000
#define CLR                 0x3000

#define VREG_CTRL_UNLOCK    (1u << 13)
#define VREG_CTRL_DISABLE_VOLTAGE_LIMIT (1u << 8)
#define VREG_UPDATE_IN_PROGRESS (1u << 15)
#define VREG_HIZ            (1u << 1)
#define VREG_STS_VOUT_OK    (1u << 4)
#define VREG_STS_STARTUP    (1u << 0)

#define HAD_POR             (1u << 16)
#define HAD_BOR             (1u << 17)
#define HAD_RUN_LOW         (1u << 18)
#define HAD_WD_POWMAN       (1u << 23)
#define HAD_WD_SWCORE       (1u << 24)
#define HAD_SWCORE_PD       (1u << 25)
#define HAD_GLITCH_DETECT   (1u << 26)
#define HAD_WD_PSM          (1u << 28)
#define RESCUE_FLAG         (1u << 4)
#define DOUBLE_TAP          (1u << 0)

#define WDSEL_RESET_PSM     (1u << 12)
#define WDSEL_RESET_SWCORE  (1u << 8)
#define WDSEL_RESET_POWMAN  (1u << 4)

#define SEQ_CFG_HW_PWRUP_SRAM1 (1u << 0)

#define STATE_CHANGING      (1u << 13)
#define STATE_WAITING       (1u << 12)
#define STATE_BAD_SW_REQ    (1u << 10)
#define STATE_PWRUP_WHILE_WAITING (1u << 9)
#define STATE_REQ_IGNORED   (1u << 8)
#define STATE_REQ(r)        ((r) << 4)
#define STATE_CURRENT       0xfu
#define P_SRAM1_OFF         0x1
#define P_SRAM0_OFF         0x2
#define P_XIP_OFF           0x4
#define P_SWCORE_OFF        0x8

#define TIMER_RUN           (1u << 1)
#define TIMER_CLEAR         (1u << 2)
#define TIMER_ALARM_ENAB    (1u << 4)
#define TIMER_PWRUP_ON_ALARM (1u << 5)
#define TIMER_ALARM         (1u << 6)
#define TIMER_USE_LPOSC     (1u << 8)
#define TIMER_USE_XOSC      (1u << 9)
#define TIMER_USE_GPIO_1KHZ (1u << 10)
#define TIMER_USING_XOSC    (1u << 16)
#define TIMER_USING_LPOSC   (1u << 17)
#define TIMER_USING_GPIO_1KHZ (1u << 18)

#define PWRUP_ENABLE        (1u << 6)
#define PWRUP_HIGH          (1u << 7)
#define PWRUP_EDGE          (1u << 8)
#define PWRUP_STATUS        (1u << 9)
#define PWRUP_RAW_STATUS    (1u << 10)

#define BOOTDIS_NEXT        (1u << 1)
#define BOOTDIS_NOW         (1u << 0)

#define INT_VREG_OUTPUT_LOW (1u << 0)
#define INT_TIMER           (1u << 1)
#define INT_STATE_REQ_IGNORED (1u << 2)

#define POWMAN_IRQ_POW      44
#define POWMAN_IRQ_TIMER    45

#define WATCHDOG            0x400d8000
#define WD_CTRL             (WATCHDOG + 0x00)
#define WD_REASON           (WATCHDOG + 0x08)
#define WD_SCRATCH(n)       (WATCHDOG + 0x0c + 4 * (n))
#define WD_CTRL_TRIGGER     (1u << 31)
#define WD_REASON_FORCE     2

#define PSM_WDSEL           0x40018008
#define PSM_ALL             0x01ffffffu
#define PSM_PROC0           (1u << 23)
#define PSM_PROC1           (1u << 24)

#define GLITCH              0x40158000
#define GD_ARM              (GLITCH + 0x00)
#define GD_TRIG_FORCE       (GLITCH + 0x14)

#define PADS_BANK0          0x40038000
#define PAD(p)              (PADS_BANK0 + 4 + 4 * (p))
#define PAD_IE              (1u << 6)
#define GPIO_PATH           "/machine/soc/gpio"

#define SRAM0_WORD          0x20000100
#define SRAM1_WORD          0x20040100

#define US                  1000
#define MS                  (1000 * US)

static char *rom_path;

/* The machine as the boot ROM leaves it: no subsystem in reset. */
static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, POWMAN + reg);
}

/* A write carrying the password. */
static void wr(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, POWMAN + reg, PASSWORD | value);
}

static uint64_t aon_time(QTestState *qts)
{
    return (uint64_t)rd(qts, READ_TIME_UPPER) << 32 |
           rd(qts, READ_TIME_LOWER);
}

static void set_time(QTestState *qts, uint64_t t)
{
    int i;

    for (i = 0; i < 4; i++) {
        wr(qts, SET_TIME(i), (t >> (16 * i)) & 0xffff);
    }
}

static void set_alarm(QTestState *qts, uint64_t t)
{
    int i;

    for (i = 0; i < 4; i++) {
        wr(qts, ALARM_TIME(i), (t >> (16 * i)) & 0xffff);
    }
}

/*
 * The LPOSC divider's reset value is 32.76800537 kHz rather than 32.768,
 * so a tick of the 32.768 kHz LPOSC is 164 ps longer than a millisecond;
 * 1 us of slack covers that.
 */
static void step_ms(QTestState *qts, uint64_t ms)
{
    qtest_clock_step(qts, ms * MS + US);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int i;

    g_assert_cmphex(rd(qts, BADPASSWD), ==, 0);
    g_assert_cmphex(rd(qts, VREG_CTRL), ==, 0x8050);
    g_assert_cmphex(rd(qts, VREG_STS), ==,
                    VREG_STS_VOUT_OK | VREG_STS_STARTUP);
    g_assert_cmphex(rd(qts, VREG), ==, 0xb0);
    g_assert_cmphex(rd(qts, VREG_LP_ENTRY), ==, 0xb4);
    g_assert_cmphex(rd(qts, VREG_LP_EXIT), ==, 0xb0);
    g_assert_cmphex(rd(qts, BOD), ==, 0xb1);
    g_assert_cmphex(rd(qts, LPOSC), ==, 0x203);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_POR);
    g_assert_cmphex(rd(qts, WDSEL), ==, 0);
    g_assert_cmphex(rd(qts, SEQ_CFG), ==, 0x1011f0);
    g_assert_cmphex(rd(qts, STATE), ==, 0);
    g_assert_cmphex(rd(qts, POW_FASTDIV), ==, 0x40);
    g_assert_cmphex(rd(qts, POW_DELAY), ==, 0x2011);
    g_assert_cmphex(rd(qts, EXT_CTRL0), ==, 0x3f);
    g_assert_cmphex(rd(qts, LPOSC_FREQ_KHZ_INT), ==, 0x20);
    g_assert_cmphex(rd(qts, LPOSC_FREQ_KHZ_FRAC), ==, 0xc49c);
    g_assert_cmphex(rd(qts, XOSC_FREQ_KHZ_INT), ==, 0x2ee0);
    g_assert_cmphex(rd(qts, TIMER), ==, 0);
    g_assert_cmphex(aon_time(qts), ==, 0);
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(rd(qts, PWRUP(i)), ==, 0x3f);
        g_assert_cmphex(rd(qts, BOOT(i)), ==, 0);
    }
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(rd(qts, SCRATCH(i)), ==, 0);
    }
    g_assert_cmphex(rd(qts, CURRENT_PWRUP_REQ), ==, 0);
    g_assert_cmphex(rd(qts, LAST_SWCORE_PWRUP), ==, 0);
    g_assert_cmphex(rd(qts, BOOTDIS), ==, 0);
    g_assert_cmphex(rd(qts, INTR), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_password(void)
{
    QTestState *qts = start();

    /* Without the password a protected write is ignored and flagged. */
    qtest_writel(qts, POWMAN + WDSEL, WDSEL_RESET_SWCORE);
    g_assert_cmphex(rd(qts, WDSEL), ==, 0);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 1);
    /* Clearing BADPASSWD needs the password too. */
    qtest_writel(qts, POWMAN + BADPASSWD, 1);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 1);
    wr(qts, BADPASSWD, 1);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 0);

    /* With it the write lands, and reads do not return the password. */
    wr(qts, WDSEL, WDSEL_RESET_SWCORE);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_SWCORE);

    /* A wrong password, or a narrow write, is as bad as none. */
    qtest_writel(qts, POWMAN + WDSEL, 0x5aff0000 | WDSEL_RESET_PSM);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 1);
    wr(qts, BADPASSWD, 1);
    qtest_writew(qts, POWMAN + WDSEL, WDSEL_RESET_PSM);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_SWCORE);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 1);
    wr(qts, BADPASSWD, 1);

    /*
     * The atomic aliases check the password in the written data and
     * operate on the low half, as pico-sdk's hw_set_bits() and
     * hw_clear_bits() with POWMAN_PASSWORD_BITS do.
     */
    qtest_writel(qts, POWMAN + SET + WDSEL, PASSWORD | WDSEL_RESET_PSM);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_SWCORE | WDSEL_RESET_PSM);
    qtest_writel(qts, POWMAN + CLR + WDSEL, PASSWORD | WDSEL_RESET_SWCORE);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_PSM);
    qtest_writel(qts, POWMAN + XOR + WDSEL,
                 PASSWORD | WDSEL_RESET_PSM | WDSEL_RESET_POWMAN);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_POWMAN);
    qtest_writel(qts, POWMAN + SET + WDSEL, WDSEL_RESET_PSM);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_POWMAN);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 1);
    wr(qts, BADPASSWD, 1);

    /*
     * The CHIP_RESET reasons in the top half are read-only and do not
     * get in the way of an atomic set of DOUBLE_TAP.
     */
    qtest_writel(qts, POWMAN + SET + CHIP_RESET, PASSWORD | DOUBLE_TAP);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_POR | DOUBLE_TAP);
    qtest_writel(qts, POWMAN + CLR + CHIP_RESET, PASSWORD | DOUBLE_TAP);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_POR);

    /* Scratch, boot and interrupt registers take plain 32-bit writes. */
    qtest_writel(qts, POWMAN + SCRATCH(3), 0xdeadbeef);
    g_assert_cmphex(rd(qts, SCRATCH(3)), ==, 0xdeadbeef);
    qtest_writel(qts, POWMAN + CLR + SCRATCH(3), 0xffff0000);
    g_assert_cmphex(rd(qts, SCRATCH(3)), ==, 0xbeef);
    qtest_writel(qts, POWMAN + BOOT(2), 0x12345678);
    g_assert_cmphex(rd(qts, BOOT(2)), ==, 0x12345678);
    qtest_writel(qts, POWMAN + INTE, 0xf);
    g_assert_cmphex(rd(qts, INTE), ==, 0xf);
    g_assert_cmphex(rd(qts, BADPASSWD), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_timer_count(void)
{
    QTestState *qts = start();
    uint32_t t;

    /* Stopped, the count does not move; RUN loads SET_TIME. */
    set_time(qts, 0x123400001000ull);
    step_ms(qts, 5);
    g_assert_cmphex(aon_time(qts), ==, 0);
    wr(qts, TIMER, TIMER_RUN);
    g_assert_cmphex(rd(qts, TIMER), ==, TIMER_RUN | TIMER_USING_LPOSC);
    g_assert_cmphex(aon_time(qts), ==, 0x123400001000ull);
    step_ms(qts, 10);
    g_assert_cmphex(aon_time(qts), ==, 0x123400001000ull + 10);

    /* Stopping holds the count; starting again reloads SET_TIME. */
    qtest_writel(qts, POWMAN + CLR + TIMER, PASSWORD | TIMER_RUN);
    step_ms(qts, 10);
    g_assert_cmphex(aon_time(qts), ==, 0x123400001000ull + 10);
    set_time(qts, 500);
    wr(qts, TIMER, TIMER_RUN);
    g_assert_cmphex(aon_time(qts), ==, 500);

    /* CLEAR zeroes the count without stopping it, and reads as 0. */
    qtest_writel(qts, POWMAN + SET + TIMER, PASSWORD | TIMER_CLEAR);
    g_assert_cmphex(rd(qts, TIMER), ==, TIMER_RUN | TIMER_USING_LPOSC);
    g_assert_cmphex(aon_time(qts), ==, 0);
    step_ms(qts, 3);
    g_assert_cmphex(aon_time(qts), ==, 3);

    /*
     * The XOSC divider at 6000 kHz makes a 0.5 ms tick from the 12 MHz
     * clk_ref. The switch waits for the LPOSC's next tick.
     */
    qtest_writel(qts, POWMAN + CLR + TIMER, PASSWORD | TIMER_RUN);
    wr(qts, XOSC_FREQ_KHZ_INT, 6000);
    set_time(qts, 0);
    wr(qts, TIMER, TIMER_RUN);
    qtest_clock_step(qts, 300 * US);
    qtest_writel(qts, POWMAN + SET + TIMER, PASSWORD | TIMER_USE_XOSC);
    g_assert_cmphex(rd(qts, TIMER), ==,
                    TIMER_RUN | TIMER_USE_XOSC | TIMER_USING_LPOSC);
    qtest_clock_step(qts, 600 * US);
    g_assert_cmphex(rd(qts, TIMER), ==,
                    TIMER_RUN | TIMER_USE_XOSC | TIMER_USING_LPOSC);
    qtest_clock_step(qts, 200 * US);
    g_assert_cmphex(rd(qts, TIMER), ==, TIMER_RUN | TIMER_USING_XOSC);
    g_assert_cmphex(aon_time(qts), ==, 1);
    qtest_clock_step(qts, 5 * MS);
    g_assert_cmphex(aon_time(qts), ==, 11);

    /* An LPOSC divider of 16.384 kHz gives two ticks per millisecond. */
    qtest_writel(qts, POWMAN + CLR + TIMER, PASSWORD | TIMER_RUN);
    wr(qts, LPOSC_FREQ_KHZ_INT, 16);
    wr(qts, LPOSC_FREQ_KHZ_FRAC, 0x624e);
    qtest_writel(qts, POWMAN + SET + TIMER, PASSWORD | TIMER_USE_LPOSC);
    g_assert_cmphex(rd(qts, TIMER), ==, TIMER_USING_LPOSC);
    set_time(qts, 0);
    wr(qts, TIMER, TIMER_RUN);
    qtest_clock_step(qts, 10 * MS);
    t = aon_time(qts);
    g_assert_cmpuint(t, >=, 19);
    g_assert_cmpuint(t, <=, 20);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_timer_gpio_tick(void)
{
    QTestState *qts = start();
    int i;

    /* EXT_TIME_REF.SOURCE_SEL 0 is GPIO12; its falling edges tick. */
    qtest_writel(qts, PAD(12), PAD_IE);
    wr(qts, EXT_TIME_REF, 0);
    wr(qts, TIMER, TIMER_RUN);
    qtest_writel(qts, POWMAN + SET + TIMER, PASSWORD | TIMER_USE_GPIO_1KHZ);
    g_assert_cmphex(rd(qts, TIMER), ==, TIMER_RUN | TIMER_USING_GPIO_1KHZ);
    for (i = 0; i < 7; i++) {
        qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 12, 1);
        qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 12, 0);
    }
    step_ms(qts, 50);
    g_assert_cmphex(aon_time(qts), ==, 7);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_alarm(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    qtest_writel(qts, POWMAN + INTE, INT_TIMER);
    set_alarm(qts, 5);
    wr(qts, TIMER, TIMER_RUN | TIMER_ALARM_ENAB);
    step_ms(qts, 4);
    g_assert_cmphex(rd(qts, TIMER) & TIMER_ALARM, ==, 0);
    g_assert_false(qtest_get_irq(qts, POWMAN_IRQ_TIMER));
    step_ms(qts, 1);
    g_assert_cmphex(rd(qts, TIMER) & TIMER_ALARM, ==, TIMER_ALARM);
    g_assert_cmphex(rd(qts, INTR), ==, INT_TIMER);
    g_assert_cmphex(rd(qts, INTS), ==, INT_TIMER);
    g_assert_true(qtest_get_irq(qts, POWMAN_IRQ_TIMER));
    g_assert_false(qtest_get_irq(qts, POWMAN_IRQ_POW));

    /*
     * ALARM is write-1-to-clear, through the CLR alias as pico-sdk's
     * powman_clear_alarm() does it; the count staying past the alarm time
     * does not fire it again.
     */
    qtest_writel(qts, POWMAN + CLR + TIMER, PASSWORD | TIMER_ALARM);
    g_assert_cmphex(rd(qts, TIMER), ==,
                    TIMER_RUN | TIMER_ALARM_ENAB | TIMER_USING_LPOSC);
    g_assert_false(qtest_get_irq(qts, POWMAN_IRQ_TIMER));
    step_ms(qts, 5);
    g_assert_cmphex(rd(qts, TIMER) & TIMER_ALARM, ==, 0);

    /* Enabling an alarm whose time has passed fires it. */
    qtest_writel(qts, POWMAN + CLR + TIMER, PASSWORD | TIMER_ALARM_ENAB);
    set_alarm(qts, 2);
    qtest_writel(qts, POWMAN + SET + TIMER, PASSWORD | TIMER_ALARM_ENAB);
    g_assert_true(qtest_get_irq(qts, POWMAN_IRQ_TIMER));
    qtest_writel(qts, POWMAN + INTE, 0);
    g_assert_false(qtest_get_irq(qts, POWMAN_IRQ_TIMER));

    /* INTF forces the other sources onto POWMAN_IRQ_POW. */
    qtest_writel(qts, POWMAN + INTE, INT_STATE_REQ_IGNORED);
    qtest_writel(qts, POWMAN + INTF, INT_STATE_REQ_IGNORED);
    g_assert_true(qtest_get_irq(qts, POWMAN_IRQ_POW));
    qtest_writel(qts, POWMAN + INTF, 0);
    g_assert_false(qtest_get_irq(qts, POWMAN_IRQ_POW));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_sram_domains(void)
{
    QTestState *qts = start();

    qtest_writel(qts, SRAM0_WORD, 0x11111111);
    qtest_writel(qts, SRAM1_WORD, 0x22222222);

    /*
     * Powering SRAM1 down takes 8 steps of 32 POWMAN ticks, each 64
     * cycles of the 12 MHz clk_ref: 1.37 ms.
     */
    wr(qts, STATE, STATE_REQ(P_SRAM1_OFF));
    g_assert_cmphex(rd(qts, STATE), ==, STATE_CHANGING | STATE_REQ(1));
    qtest_clock_step(qts, 1300 * US);
    g_assert_cmphex(rd(qts, STATE), ==, STATE_CHANGING | STATE_REQ(1));
    qtest_clock_step(qts, 100 * US);
    g_assert_cmphex(rd(qts, STATE), ==, STATE_REQ(1) | P_SRAM1_OFF);
    g_assert_cmphex(qtest_readl(qts, SRAM0_WORD), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, SRAM1_WORD), ==, 0);
    qtest_writel(qts, SRAM1_WORD, 0x33333333);
    g_assert_cmphex(qtest_readl(qts, SRAM1_WORD), ==, 0);

    /* Powering one domain up and another down at once is invalid. */
    wr(qts, STATE, STATE_REQ(P_SRAM0_OFF));
    g_assert_cmphex(rd(qts, STATE), ==,
                    STATE_BAD_SW_REQ | STATE_REQ(P_SRAM0_OFF) | P_SRAM1_OFF);
    /* So is powering XIP down while the switched core stays up. */
    wr(qts, STATE, STATE_REQ(P_XIP_OFF | P_SRAM1_OFF));
    g_assert_cmphex(rd(qts, STATE) & STATE_BAD_SW_REQ, ==, STATE_BAD_SW_REQ);
    g_assert_cmphex(rd(qts, STATE) & STATE_CURRENT, ==, P_SRAM1_OFF);

    /* Powered up again, SRAM1 works but its old contents are gone. */
    wr(qts, STATE, STATE_REQ(0));
    g_assert_cmphex(rd(qts, STATE), ==, STATE_CHANGING | P_SRAM1_OFF);
    qtest_clock_step(qts, 1400 * US);
    g_assert_cmphex(rd(qts, STATE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SRAM1_WORD), ==, 0);
    qtest_writel(qts, SRAM1_WORD, 0x44444444);
    g_assert_cmphex(qtest_readl(qts, SRAM1_WORD), ==, 0x44444444);
    qtest_quit(qts);
}

/*
 * Power the switched core down: with no processor executing under qtest,
 * the request leaves WAITING at the next POWMAN tick, 5.3 us later. The
 * resets of the switched core entering and leaving a low-power state are
 * subsystem resets, which send no RESET event; the main loop carries them
 * out before it takes the next qtest command.
 */
static void sleep_until_down(QTestState *qts, uint32_t pstate)
{
    wr(qts, STATE, STATE_REQ(pstate));
    g_assert_cmphex(rd(qts, STATE), ==, STATE_WAITING | STATE_REQ(pstate));
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(rd(qts, STATE) & (STATE_WAITING | STATE_CHANGING), ==,
                    STATE_CHANGING);
    qtest_clock_step(qts, 20 * MS);
    g_assert_cmphex(rd(qts, STATE), ==, STATE_REQ(pstate) | pstate);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_sleep_alarm_wakeup(void)
{
    QTestState *qts = start();
    uint64_t t;

    qtest_writel(qts, SRAM0_WORD, 0x11111111);
    qtest_writel(qts, SRAM1_WORD, 0x22222222);
    qtest_writel(qts, POWMAN + SCRATCH(0), 0xcafef00d);
    qtest_writel(qts, POWMAN + BOOT(1), 0x87654321);
    qtest_writel(qts, WD_SCRATCH(0), 0x5a5a5a5a);
    wr(qts, BOOTDIS, BOOTDIS_NEXT);

    set_alarm(qts, 100);
    wr(qts, TIMER, TIMER_RUN | TIMER_ALARM_ENAB | TIMER_PWRUP_ON_ALARM);

    /* P1.1: SRAM1 powers down too, and comes back up on waking. */
    sleep_until_down(qts, P_SWCORE_OFF | P_SRAM1_OFF);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_POR);
    /* The switched core, with the watchdog, is in reset. */
    g_assert_cmphex(qtest_readl(qts, WD_SCRATCH(0)), ==, 0);
    t = aon_time(qts);
    g_assert_cmpuint(t, >, 0);
    g_assert_cmpuint(t, <, 100);

    /* The alarm powers the chip back up and it boots. */
    qtest_clock_step(qts, (100 - t) * MS + 20 * MS);
    g_assert_cmphex(rd(qts, STATE), ==, 0);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_SWCORE_PD);
    g_assert_cmphex(rd(qts, LAST_SWCORE_PWRUP), ==, 1u << 6);
    g_assert_cmphex(rd(qts, TIMER) & (TIMER_RUN | TIMER_ALARM), ==,
                    TIMER_RUN | TIMER_ALARM);
    g_assert_cmpuint(aon_time(qts), >=, 100);
    g_assert_cmphex(rd(qts, SCRATCH(0)), ==, 0xcafef00d);
    g_assert_cmphex(rd(qts, BOOT(1)), ==, 0x87654321);
    g_assert_cmphex(rd(qts, BOOTDIS), ==, BOOTDIS_NOW);
    g_assert_cmphex(qtest_readl(qts, SRAM0_WORD), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, SRAM1_WORD), ==, 0);

    /* The alarm's power-up request blocks another power-down. */
    g_assert_cmphex(rd(qts, CURRENT_PWRUP_REQ), ==, 1u << 6);
    wr(qts, STATE, STATE_REQ(P_SWCORE_OFF));
    g_assert_cmphex(rd(qts, STATE), ==, STATE_REQ_IGNORED);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_gpio_pwrup(void)
{
    QTestState *qts = start();

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    qtest_writel(qts, PAD(5), PAD_IE);
    qtest_writel(qts, PAD(6), PAD_IE);
    qtest_writel(qts, PAD(7), PAD_IE);

    /* A level-sensitive source requests power-up while its pin is high. */
    wr(qts, PWRUP(0), 5 | PWRUP_HIGH);
    wr(qts, PWRUP(0), 5 | PWRUP_HIGH | PWRUP_ENABLE);
    g_assert_cmphex(rd(qts, CURRENT_PWRUP_REQ), ==, 0);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, 1);
    g_assert_cmphex(rd(qts, PWRUP(0)), ==, 5 | PWRUP_HIGH | PWRUP_ENABLE |
                    PWRUP_STATUS | PWRUP_RAW_STATUS);
    g_assert_cmphex(rd(qts, CURRENT_PWRUP_REQ), ==, 1u << 1);

    /* A request while a power-up request is pending is ignored. */
    qtest_writel(qts, POWMAN + INTE, INT_STATE_REQ_IGNORED);
    wr(qts, STATE, STATE_REQ(P_SWCORE_OFF));
    g_assert_cmphex(rd(qts, STATE), ==, STATE_REQ_IGNORED);
    g_assert_true(qtest_get_irq(qts, POWMAN_IRQ_POW));
    qtest_writel(qts, POWMAN + CLR + STATE, PASSWORD | STATE_REQ_IGNORED);
    g_assert_cmphex(rd(qts, STATE), ==, 0);
    g_assert_false(qtest_get_irq(qts, POWMAN_IRQ_POW));

    /* One arriving while waiting for the processors cancels the request. */
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, 0);
    wr(qts, STATE, STATE_REQ(P_SWCORE_OFF));
    g_assert_cmphex(rd(qts, STATE), ==,
                    STATE_WAITING | STATE_REQ(P_SWCORE_OFF));
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 5, 1);
    g_assert_cmphex(rd(qts, STATE), ==,
                    STATE_PWRUP_WHILE_WAITING | STATE_REQ_IGNORED);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(rd(qts, STATE) & STATE_CURRENT, ==, 0);
    wr(qts, PWRUP(0), 5 | PWRUP_HIGH);
    wr(qts, STATE, STATE_PWRUP_WHILE_WAITING | STATE_REQ_IGNORED);
    g_assert_cmphex(rd(qts, STATE), ==, 0);

    /* An edge-sensitive source latches the edge until STATUS is cleared. */
    wr(qts, PWRUP(1), 6 | PWRUP_HIGH | PWRUP_EDGE | PWRUP_ENABLE);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 6, 1);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 6, 0);
    g_assert_cmphex(rd(qts, PWRUP(1)) & PWRUP_STATUS, ==, PWRUP_STATUS);
    g_assert_cmphex(rd(qts, CURRENT_PWRUP_REQ), ==, 1u << 2);
    qtest_writel(qts, POWMAN + SET + PWRUP(1), PASSWORD | PWRUP_STATUS);
    g_assert_cmphex(rd(qts, PWRUP(1)) & PWRUP_STATUS, ==, 0);
    g_assert_cmphex(rd(qts, CURRENT_PWRUP_REQ), ==, 0);

    /* A pin going high wakes the chip from a low-power state. */
    wr(qts, PWRUP(2), 7 | PWRUP_HIGH | PWRUP_EDGE | PWRUP_ENABLE);
    sleep_until_down(qts, P_SWCORE_OFF);
    qtest_set_irq_in(qts, GPIO_PATH, "pad-in", 7, 1);
    qtest_clock_step(qts, 20 * MS);
    g_assert_cmphex(rd(qts, STATE), ==, 0);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_SWCORE_PD);
    g_assert_cmphex(rd(qts, LAST_SWCORE_PWRUP), ==, 1u << 3);
    qtest_quit(qts);
}

/* Trigger the watchdog with every PSM stage selected. */
static void watchdog_trigger(QTestState *qts)
{
    qtest_writel(qts, PSM_WDSEL, PSM_ALL);
    qtest_writel(qts, WD_CTRL, WD_CTRL_TRIGGER);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_watchdog_resets(void)
{
    QTestState *qts = start();

    qtest_writel(qts, POWMAN + SCRATCH(1), 0x600df00d);
    qtest_writel(qts, POWMAN + BOOT(0), 0xb007c0d3);
    qtest_writel(qts, WD_SCRATCH(1), 0x12345678);
    wr(qts, CHIP_RESET, DOUBLE_TAP);

    /*
     * Without a PSM stage up to CLOCKS selected, POWMAN leaves the
     * watchdog reset to the PSM.
     */
    wr(qts, WDSEL, WDSEL_RESET_SWCORE);
    qtest_writel(qts, PSM_WDSEL, PSM_PROC0 | PSM_PROC1);
    qtest_writel(qts, WD_CTRL, WD_CTRL_TRIGGER);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_POR | DOUBLE_TAP);
    g_assert_cmphex(qtest_readl(qts, WD_REASON), ==, WD_REASON_FORCE);

    /* RESET_PSM: the full PSM sequence; the watchdog keeps its state. */
    wr(qts, WDSEL, WDSEL_RESET_PSM);
    watchdog_trigger(qts);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_WD_PSM | DOUBLE_TAP);
    g_assert_cmphex(qtest_readl(qts, PSM_WDSEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, WD_REASON), ==, WD_REASON_FORCE);
    g_assert_cmphex(qtest_readl(qts, WD_SCRATCH(1)), ==, 0x12345678);
    g_assert_cmphex(rd(qts, SCRATCH(1)), ==, 0x600df00d);

    /* RESET_SWCORE: the watchdog is reset with the switched core. */
    wr(qts, TIMER, TIMER_RUN);
    wr(qts, WDSEL, WDSEL_RESET_SWCORE);
    watchdog_trigger(qts);
    qtest_qmp_eventwait(qts, "RESET");
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_WD_SWCORE | DOUBLE_TAP);
    g_assert_cmphex(qtest_readl(qts, WD_REASON), ==, 0);
    g_assert_cmphex(qtest_readl(qts, WD_SCRATCH(1)), ==, 0);
    g_assert_cmphex(rd(qts, SCRATCH(1)), ==, 0x600df00d);
    g_assert_cmphex(rd(qts, BOOT(0)), ==, 0xb007c0d3);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_SWCORE);
    g_assert_cmphex(rd(qts, TIMER) & TIMER_RUN, ==, TIMER_RUN);

    /* RESET_POWMAN: the power manager and timer reset too. */
    wr(qts, WDSEL, WDSEL_RESET_POWMAN);
    watchdog_trigger(qts);
    qtest_qmp_eventwait(qts, "RESET");
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_WD_POWMAN | DOUBLE_TAP);
    g_assert_cmphex(rd(qts, SCRATCH(1)), ==, 0);
    g_assert_cmphex(rd(qts, BOOT(0)), ==, 0);
    g_assert_cmphex(rd(qts, WDSEL), ==, 0);
    g_assert_cmphex(rd(qts, TIMER), ==, 0);

    /* A system reset is the RUN pin, which keeps only DOUBLE_TAP. */
    qtest_writel(qts, POWMAN + SCRATCH(1), 0x600df00d);
    qtest_system_reset(qts);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_RUN_LOW | DOUBLE_TAP);
    g_assert_cmphex(rd(qts, SCRATCH(1)), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_glitch_reset(void)
{
    QTestState *qts = start();

    qtest_writel(qts, POWMAN + SCRATCH(7), 0x0badcafe);
    wr(qts, BOOTDIS, BOOTDIS_NEXT);
    /* BOOTDIS.NEXT can only be set; NOW is write-1-to-clear. */
    wr(qts, BOOTDIS, 0);
    g_assert_cmphex(rd(qts, BOOTDIS), ==, BOOTDIS_NEXT);

    qtest_writel(qts, POWMAN + BOOT(2), 0xb007c0d3);
    wr(qts, TIMER, TIMER_RUN);
    wr(qts, WDSEL, WDSEL_RESET_SWCORE);
    wr(qts, CHIP_RESET, DOUBLE_TAP);
    qtest_writel(qts, WD_SCRATCH(3), 0x12345678);
    qtest_writel(qts, PSM_WDSEL, PSM_ALL);
    qtest_writel(qts, SRAM0_WORD, 0x5a5aa5a5);

    /*
     * The trigger resets the PSM and the watchdog, through the power
     * manager; it is not a system reset of the whole switched core.
     */
    qtest_writel(qts, GD_ARM, 0x1234);
    qtest_writel(qts, GD_TRIG_FORCE, 0x1);
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_GLITCH_DETECT | DOUBLE_TAP);
    g_assert_cmphex(qtest_readl(qts, PSM_WDSEL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RP2350_RESETS_RESET_DONE), ==, 0);
    g_assert_cmphex(qtest_readl(qts, WD_SCRATCH(3)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, WD_REASON), ==, 0);

    /* The power manager, its timer and the power state carry on. */
    g_assert_cmphex(rd(qts, SCRATCH(7)), ==, 0x0badcafe);
    g_assert_cmphex(rd(qts, BOOT(2)), ==, 0xb007c0d3);
    g_assert_cmphex(rd(qts, WDSEL), ==, WDSEL_RESET_SWCORE);
    g_assert_cmphex(rd(qts, TIMER) & TIMER_RUN, ==, TIMER_RUN);
    g_assert_cmphex(rd(qts, STATE), ==, 0);
    /* SRAM is not reset by the PSM. */
    g_assert_cmphex(qtest_readl(qts, SRAM0_WORD), ==, 0x5a5aa5a5);
    /* Powman reset the PSM, moving BOOTDIS.NEXT to NOW. */
    g_assert_cmphex(rd(qts, BOOTDIS), ==, BOOTDIS_NOW);
    wr(qts, BOOTDIS, BOOTDIS_NOW);
    g_assert_cmphex(rd(qts, BOOTDIS), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.powman/test] */
static void test_vreg(void)
{
    QTestState *qts = start();

    /* VREG is locked until VREG_CTRL.UNLOCK, which cannot be cleared. */
    wr(qts, VREG, 0xd << 4);
    g_assert_cmphex(rd(qts, VREG), ==, 0xb0);
    qtest_writel(qts, POWMAN + SET + VREG_CTRL, PASSWORD | VREG_CTRL_UNLOCK);
    wr(qts, VREG_CTRL, 0x8050);
    g_assert_cmphex(rd(qts, VREG_CTRL), ==, 0x8050 | VREG_CTRL_UNLOCK);

    /* 1.1 V to 1.2 V: the update takes a while and ends STARTUP. */
    wr(qts, VREG, 0xd << 4);
    g_assert_cmphex(rd(qts, VREG), ==, 0xd0 | VREG_UPDATE_IN_PROGRESS);
    g_assert_cmphex(rd(qts, VREG_STS), ==, VREG_STS_VOUT_OK);
    /* Writes during the update are ignored. */
    wr(qts, VREG, 0xb << 4);
    qtest_clock_step(qts, 60 * US);
    g_assert_cmphex(rd(qts, VREG), ==, 0xd0);

    /*
     * Past the 1.3 V limit, once it is disabled, the output is out of
     * regulation while it ramps.
     */
    wr(qts, VREG_CTRL, 0x8050 | VREG_CTRL_UNLOCK |
                       VREG_CTRL_DISABLE_VOLTAGE_LIMIT);
    qtest_writel(qts, POWMAN + INTE, INT_VREG_OUTPUT_LOW);
    wr(qts, VREG, 0x13 << 4);
    g_assert_cmphex(rd(qts, VREG_STS), ==, 0);
    g_assert_cmphex(rd(qts, INTR), ==, INT_VREG_OUTPUT_LOW);
    qtest_clock_step(qts, 60 * US);
    g_assert_cmphex(rd(qts, VREG_STS), ==, VREG_STS_VOUT_OK);
    qtest_writel(qts, POWMAN + INTR, INT_VREG_OUTPUT_LOW);
    g_assert_cmphex(rd(qts, INTR), ==, 0);

    /*
     * High-impedance mode takes the core supply away: a brownout reset,
     * which clears everything, DOUBLE_TAP included.
     */
    wr(qts, CHIP_RESET, DOUBLE_TAP);
    wr(qts, VREG, (0xb << 4) | VREG_HIZ);
    qtest_qmp_eventwait(qts, "RESET");
    g_assert_cmphex(rd(qts, CHIP_RESET), ==, HAD_BOR);
    g_assert_cmphex(rd(qts, VREG), ==, 0xb0);
    g_assert_cmphex(rd(qts, VREG_CTRL), ==, 0x8050);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-powman-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/powman/reset-values", test_reset_values);
    qtest_add_func("/rp2350/powman/password", test_password);
    qtest_add_func("/rp2350/powman/timer-count", test_timer_count);
    qtest_add_func("/rp2350/powman/timer-gpio-tick", test_timer_gpio_tick);
    qtest_add_func("/rp2350/powman/alarm", test_alarm);
    qtest_add_func("/rp2350/powman/sram-domains", test_sram_domains);
    qtest_add_func("/rp2350/powman/sleep-alarm-wakeup",
                   test_sleep_alarm_wakeup);
    qtest_add_func("/rp2350/powman/gpio-pwrup", test_gpio_pwrup);
    qtest_add_func("/rp2350/powman/watchdog-resets", test_watchdog_resets);
    qtest_add_func("/rp2350/powman/glitch-reset", test_glitch_reset);
    qtest_add_func("/rp2350/powman/vreg", test_vreg);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
