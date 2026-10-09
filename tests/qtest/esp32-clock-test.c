/*
 * QTest testcase for the ESP32 clock tree and DPORT peripheral clock gates
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERI_CLK_EN   0x3ff0001c
#define DPORT_PERI_RST_EN   0x3ff00020
#define DPORT_CPU_PER_CONF  0x3ff0003c
#define DPORT_PERIP_CLK_EN  0x3ff000c0
#define DPORT_PERIP_RST_EN  0x3ff000c4
#define DPORT_WIFI_CLK_EN   0x3ff000cc
#define DPORT_CORE_RST_EN   0x3ff000d0
#define PERIP_TIMERGROUP1   (1u << 15)
#define PERIP_UART2         (1u << 23)

#define RTC_CNTL_CLK_CONF   0x3ff48070
#define SOC_CLK_SEL_SHIFT   27
#define SOC_CLK_XTAL        0
#define SOC_CLK_PLL         1
#define SOC_CLK_8M          2
#define SOC_CLK_APLL        3

#define SYSCON              0x3ff66000
#define SYSCON_SYSCLK_CONF  (SYSCON + 0x00)
#define SYSCON_XTAL_TICK    (SYSCON + 0x04)
#define SYSCON_PLL_TICK     (SYSCON + 0x08)
#define SYSCON_CK8M_TICK    (SYSCON + 0x0c)
#define SYSCON_APLL_TICK    (SYSCON + 0x3c)
#define SYSCON_DATE         (SYSCON + 0x7c)

#define TIMG0               0x3ff5f000
#define TIMG1               0x3ff60000
#define T0CONFIG            0x00
#define T0LO                0x04
#define T0UPDATE            0x0c
#define T0LOADLO            0x18
#define T0CONFIG_RUN        (0xc0000000u | (2u << 13)) /* EN | INC, div 2 */

#define UART2               0x3ff6e000
#define UART_FIFO           0x00
#define UART_CLKDIV         0x14
#define UART_STATUS         0x1c
#define UART_CONF0          0x20
#define UART_TICK_REF_ALWAYS_ON (1u << 27)

#define US                  1000

static QTestState *start(void)
{
    return qtest_init("-M esp32 -nic none");
}

static uint32_t timg_count(QTestState *qts, uint32_t timg)
{
    qtest_writel(qts, timg + T0UPDATE, 1);
    return qtest_readl(qts, timg + T0LO);
}

static void select_soc_clk(QTestState *qts, uint32_t sel)
{
    uint32_t v = qtest_readl(qts, RTC_CNTL_CLK_CONF);

    v &= ~(3u << SOC_CLK_SEL_SHIFT);
    qtest_writel(qts, RTC_CNTL_CLK_CONF, v | (sel << SOC_CLK_SEL_SHIFT));
}

/* Timer group 0 cycles, at APB_CLK / 2, over 100 us. */
static uint32_t apb_per_100us(QTestState *qts)
{
    uint32_t before = timg_count(qts, TIMG0);

    qtest_clock_step(qts, 100 * US);
    return (timg_count(qts, TIMG0) - before) * 2;
}

static unsigned uart_tx_pending(QTestState *qts)
{
    uint32_t status = qtest_readl(qts, UART2 + UART_STATUS);

    /* TXFIFO_CNT, plus the byte in the shift register (ST_UTX_OUT) */
    return ((status >> 16) & 0xff) + (((status >> 24) & 0xf) != 0);
}

/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, DPORT_PERIP_CLK_EN), ==, 0xf9c1e06f);
    g_assert_cmphex(qtest_readl(qts, DPORT_PERIP_RST_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_PERI_CLK_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_PERI_RST_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_WIFI_CLK_EN), ==, 0xfffce030);
    g_assert_cmphex(qtest_readl(qts, DPORT_CORE_RST_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DPORT_CPU_PER_CONF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RTC_CNTL_CLK_CONF), ==, 0x2210);

    g_assert_cmphex(qtest_readl(qts, SYSCON_SYSCLK_CONF), ==, 0x2000);
    g_assert_cmphex(qtest_readl(qts, SYSCON_XTAL_TICK), ==, 39);
    g_assert_cmphex(qtest_readl(qts, SYSCON_PLL_TICK), ==, 79);
    g_assert_cmphex(qtest_readl(qts, SYSCON_CK8M_TICK), ==, 11);
    g_assert_cmphex(qtest_readl(qts, SYSCON_APLL_TICK), ==, 99);
    g_assert_cmphex(qtest_readl(qts, SYSCON_DATE), ==, 0x96042000);
    qtest_writel(qts, SYSCON_XTAL_TICK, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, SYSCON_XTAL_TICK), ==, 0xff);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_apb_follows_soc_clk(void)
{
    QTestState *qts = start();

    qtest_writel(qts, TIMG0 + T0CONFIG, T0CONFIG_RUN);
    g_assert_cmpuint(apb_per_100us(qts), ==, 4000);       /* XTAL 40 MHz */

    qtest_writel(qts, SYSCON_SYSCLK_CONF, 0x2000 | 3);
    g_assert_cmpuint(apb_per_100us(qts), ==, 1000);       /* XTAL / 4 */
    qtest_writel(qts, SYSCON_SYSCLK_CONF, 0x2000);

    select_soc_clk(qts, SOC_CLK_8M);
    g_assert_cmpuint(apb_per_100us(qts), ==, 800);        /* RC_FAST */

    for (uint32_t sel = 0; sel < 3; sel++) {
        qtest_writel(qts, DPORT_CPU_PER_CONF, sel);
        select_soc_clk(qts, SOC_CLK_PLL);
        g_assert_cmpuint(apb_per_100us(qts), ==, 8000);   /* 80 MHz */
    }

    select_soc_clk(qts, SOC_CLK_XTAL);
    g_assert_cmpuint(apb_per_100us(qts), ==, 4000);
    qtest_quit(qts);
}

/* A change of rate applies from the change on, not to the time before. */
/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_rate_change_keeps_count(void)
{
    QTestState *qts = start();
    uint32_t count;

    qtest_writel(qts, TIMG0 + T0CONFIG, T0CONFIG_RUN);
    qtest_clock_step(qts, 100 * US);
    select_soc_clk(qts, SOC_CLK_8M);
    count = timg_count(qts, TIMG0);
    g_assert_cmpuint(count, ==, 2000);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(timg_count(qts, TIMG0), ==, count + 400);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_gate_and_reset(void)
{
    QTestState *qts = start();
    uint32_t count;

    qtest_writel(qts, TIMG1 + T0CONFIG, T0CONFIG_RUN);
    qtest_clock_step(qts, 100 * US);
    count = timg_count(qts, TIMG1);
    g_assert_cmpuint(count, ==, 2000);

    qtest_writel(qts, DPORT_PERIP_CLK_EN, 0xf9c1e06f & ~PERIP_TIMERGROUP1);
    qtest_clock_step(qts, 100 * US);
    qtest_writel(qts, TIMG1 + T0LOADLO, 0x1234);
    g_assert_cmphex(qtest_readl(qts, TIMG1 + T0LOADLO), ==, 0);
    g_assert_cmphex(qtest_readl(qts, TIMG1 + T0CONFIG), ==, T0CONFIG_RUN);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, 0xf9c1e06f);
    g_assert_cmpuint(timg_count(qts, TIMG1), ==, count);
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(timg_count(qts, TIMG1), ==, count + 2000);

    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_TIMERGROUP1);
    g_assert_cmphex(qtest_readl(qts, TIMG1 + T0CONFIG), ==, 0x60002000);
    qtest_writel(qts, TIMG1 + T0CONFIG, T0CONFIG_RUN);
    g_assert_cmphex(qtest_readl(qts, TIMG1 + T0CONFIG), ==, 0x60002000);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    g_assert_cmpuint(timg_count(qts, TIMG1), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.clock-gating/test] */
static void test_uart_baud(void)
{
    QTestState *qts = start();

    /* APB_CLK 40 MHz / 400: 100 kbaud, 100 us per 10-bit frame */
    qtest_writel(qts, UART2 + UART_CLKDIV, 400);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, UART2 + UART_FIFO, 'U');
    }
    g_assert_cmpuint(uart_tx_pending(qts), ==, 3);
    qtest_clock_step(qts, 99 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 3);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 2);
    qtest_clock_step(qts, 200 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 0);

    /* Half a frame at 100 kbaud, then the rest at 20 kbaud from RC_FAST */
    qtest_writel(qts, UART2 + UART_FIFO, 'U');
    qtest_clock_step(qts, 50 * US);
    select_soc_clk(qts, SOC_CLK_8M);
    qtest_clock_step(qts, 249 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 1);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 0);

    /* REF_TICK at 1 MHz from RC_FAST: CLKDIV 10 is 100 kbaud again */
    qtest_writel(qts, SYSCON_CK8M_TICK, 7);
    qtest_writel(qts, UART2 + UART_CONF0,
                 qtest_readl(qts, UART2 + UART_CONF0) &
                 ~UART_TICK_REF_ALWAYS_ON);
    qtest_writel(qts, UART2 + UART_CLKDIV, 10);
    qtest_writel(qts, UART2 + UART_FIFO, 'U');
    qtest_clock_step(qts, 100 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 0);

    /* Gating the UART's clock stops the frame where it is. */
    qtest_writel(qts, UART2 + UART_FIFO, 'U');
    qtest_clock_step(qts, 40 * US);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, 0xf9c1e06f & ~PERIP_UART2);
    qtest_clock_step(qts, 1000 * US);
    qtest_writel(qts, DPORT_PERIP_CLK_EN, 0xf9c1e06f);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 1);
    qtest_clock_step(qts, 59 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 1);
    qtest_clock_step(qts, 1 * US);
    g_assert_cmpuint(uart_tx_pending(qts), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32/clock/reset-values", test_reset_values);
    qtest_add_func("esp32/clock/apb-follows-soc-clk",
                   test_apb_follows_soc_clk);
    qtest_add_func("esp32/clock/rate-change-keeps-count",
                   test_rate_change_keeps_count);
    qtest_add_func("esp32/clock/gate-and-reset", test_gate_and_reset);
    qtest_add_func("esp32/clock/uart-baud", test_uart_baud);
    return g_test_run();
}
