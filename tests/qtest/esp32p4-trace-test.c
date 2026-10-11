/*
 * QTest testcase for the ESP32-P4 RISC-V trace encoders
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The cores do not run under qtest, so an encoder started and stopped
 * traces no instruction: what it writes is the support packet that ends
 * a trace, 05 <index> 0F 00 (a 5-byte packet, its 16-bit index, format 3
 * subformat 3 with the encoder disabled), which shows its trace memory
 * writes, its FIFO and its control. The guest tests trace instructions.
 * Interrupts are seen in the interrupt matrix's status of sources 97 and
 * 98.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define TRACE(n)            (0x3ff04000 + 0x1000 * (n))
#define MEM_START(n)        (TRACE(n) + 0x00)
#define MEM_END(n)          (TRACE(n) + 0x04)
#define MEM_CURRENT(n)      (TRACE(n) + 0x08)
#define MEM_ADDR_UPDATE(n)  (TRACE(n) + 0x0c)
#define FIFO_STATUS(n)      (TRACE(n) + 0x10)
#define INTR_ENA(n)         (TRACE(n) + 0x14)
#define INTR_RAW(n)         (TRACE(n) + 0x18)
#define INTR_CLR(n)         (TRACE(n) + 0x1c)
#define TRIGGER(n)          (TRACE(n) + 0x20)
#define CONFIG(n)           (TRACE(n) + 0x24)
#define FILTER_CONTROL(n)   (TRACE(n) + 0x28)
#define FILTER_MATCH(n)     (TRACE(n) + 0x2c)
#define FILTER_COMP(n)      (TRACE(n) + 0x30)
#define RESYNC(n)           (TRACE(n) + 0x3c)
#define AHB_CONFIG(n)       (TRACE(n) + 0x40)
#define CLOCK_GATE(n)       (TRACE(n) + 0x44)
#define DATE(n)             (TRACE(n) + 0x3fc)

#define FIFO_EMPTY          (1u << 0)
#define WORK_STATUS(v)      (((v) >> 1) & 3)
#define WORK_IDLE           0
#define WORK_WORKING        1
#define WORK_WAIT           2
#define INTR_MEM_FULL       (1u << 1)
#define TRIGGER_ON          (1u << 0)
#define TRIGGER_OFF         (1u << 1)
#define TRIGGER_MEM_LOOP    (1u << 2)
#define TRIGGER_RESTART_ENA (1u << 3)
#define CONFIG_RESET_ENA    (1u << 1)

#define SOC_CLK_CTRL0       0x500e6014
#define TRACE_CPU_CLK_EN    (1u << 11)
#define TRACE_SYS_CLK_EN    (1u << 24)
#define HP_RST_EN0          0x500e60c0
#define RST_EN_CORETRACE(n) (1u << (9 + (n)))

#define LP_CLKRST           0x50111000
#define HPCPU_RESET_CTRL0   (LP_CLKRST + 0x14)
#define HPCORE_SW_RESET(n)  (1u << (13 + 16 * (n)))

#define INTMTX_STATUS3      0x500d620c
#define SRC_TRACE_BIT(n)    (1u << (97 + (n) - 96))

/* Trace memory in L2MEM, clear of the L2 cache. */
#define BUF                 0x4ff40000

static const uint8_t support0[] = { 0x05, 0x00, 0x00, 0x0f, 0x00 };
static const uint8_t support1[] = { 0x05, 0x01, 0x00, 0x0f, 0x00 };

static void set_clocks(QTestState *qts, uint32_t set, uint32_t clear)
{
    uint32_t v = qtest_readl(qts, SOC_CLK_CTRL0);

    qtest_writel(qts, SOC_CLK_CTRL0, (v | set) & ~clear);
}

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32p4,flash-size=16M");

    set_clocks(qts, TRACE_CPU_CLK_EN | TRACE_SYS_CLK_EN, 0);
    return qts;
}

/* Point encoder n's memory at BUF, @len bytes, zeroed. */
static void set_memory(QTestState *qts, int n, uint32_t len)
{
    qtest_memset(qts, BUF, 0, 64);
    qtest_writel(qts, MEM_START(n), BUF);
    qtest_writel(qts, MEM_END(n), BUF + len);
    qtest_writel(qts, MEM_ADDR_UPDATE(n), 1);
}

static void expect_bytes(QTestState *qts, uint32_t addr, const uint8_t *want,
                         size_t len)
{
    uint8_t got[16];

    g_assert_cmpuint(len, <=, sizeof(got));
    qtest_memread(qts, addr, got, len);
    for (size_t i = 0; i < len; i++) {
        g_assert_cmphex(got[i], ==, want[i]);
    }
}

static uint32_t work(QTestState *qts, int n)
{
    return WORK_STATUS(qtest_readl(qts, FIFO_STATUS(n)));
}

/* Both encoders' registers at their reset values, and their writable bits. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_registers(void)
{
    QTestState *qts = start();

    for (int n = 0; n < 2; n++) {
        g_assert_cmphex(qtest_readl(qts, MEM_START(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, MEM_END(n)), ==, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, FIFO_STATUS(n)), ==, FIFO_EMPTY);
        g_assert_cmphex(qtest_readl(qts, INTR_ENA(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, INTR_RAW(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, TRIGGER(n)), ==,
                        TRIGGER_MEM_LOOP | TRIGGER_RESTART_ENA);
        g_assert_cmphex(qtest_readl(qts, CONFIG(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, RESYNC(n)), ==, 0x80);
        g_assert_cmphex(qtest_readl(qts, CLOCK_GATE(n)), ==, 1);
        g_assert_cmphex(qtest_readl(qts, DATE(n)), ==, 0x2211300);
    }

    qtest_writel(qts, FILTER_CONTROL(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FILTER_CONTROL(1)), ==, 0x1f);
    qtest_writel(qts, FILTER_MATCH(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FILTER_MATCH(1)), ==, 0xff);
    qtest_writel(qts, FILTER_COMP(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, FILTER_COMP(1)), ==, 0x33d3d);
    qtest_writel(qts, RESYNC(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, RESYNC(1)), ==, 0x3ffffff);
    qtest_writel(qts, AHB_CONFIG(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, AHB_CONFIG(1)), ==, 0x3f);
    qtest_writel(qts, INTR_ENA(1), 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INTR_ENA(1)), ==, 0x3);
    qtest_writel(qts, MEM_CURRENT(1), 0x1234);
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(1)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, MEM_START(0)), ==, 0);

    /* RST_EN_CORETRACE1 resets encoder 1 alone. */
    qtest_writel(qts, HP_RST_EN0, RST_EN_CORETRACE(1));
    qtest_writel(qts, HP_RST_EN0, 0);
    g_assert_cmphex(qtest_readl(qts, FILTER_CONTROL(1)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RESYNC(1)), ==, 0x80);
    qtest_quit(qts);
}

/*
 * TRIGGER_ON starts the encoder; TRIGGER_OFF stops it, with the support
 * packet that ends a trace, written at MEM_CURRENT_ADDR. TRACE_RESTART_ENA
 * restarts an encoder that has stopped once its FIFO is empty, however it
 * stopped (TRM 3.5.7): to stop it, software clears it first, as ESP-IDF
 * does. A restart goes on counting packets.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_start_stop(void)
{
    QTestState *qts = start();

    set_memory(qts, 0, 0x100);
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(0)), ==, BUF);
    qtest_writel(qts, TRIGGER(0), TRIGGER_ON | TRIGGER_RESTART_ENA);
    g_assert_cmpuint(work(qts, 0), ==, WORK_WORKING);
    g_assert_cmpuint(work(qts, 1), ==, WORK_IDLE);

    qtest_writel(qts, TRIGGER(0), TRIGGER_OFF | TRIGGER_RESTART_ENA);
    g_assert_cmpuint(work(qts, 0), ==, WORK_WORKING);
    expect_bytes(qts, BUF, support0, sizeof(support0));
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(0)), ==, BUF + 5);

    qtest_writel(qts, TRIGGER(0), 0);
    qtest_writel(qts, TRIGGER(0), TRIGGER_OFF);
    g_assert_cmphex(qtest_readl(qts, FIFO_STATUS(0)), ==, FIFO_EMPTY);
    expect_bytes(qts, BUF + 5, support1, sizeof(support1));
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(0)), ==, BUF + 10);

    /* Started from idle, it counts from 0 again. */
    qtest_writel(qts, MEM_ADDR_UPDATE(0), 1);
    qtest_writel(qts, TRIGGER(0), TRIGGER_ON);
    qtest_writel(qts, TRIGGER(0), TRIGGER_OFF);
    expect_bytes(qts, BUF, support0, sizeof(support0));
    g_assert_cmpuint(work(qts, 0), ==, WORK_IDLE);
    qtest_quit(qts);
}

/*
 * The memory's end: MEM_FULL_INTR as MEM_CURRENT_ADDR reaches it, then
 * back to the start in loop mode, or the rest dropped; the interrupt is
 * source 97 for encoder 0 and 98 for encoder 1.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_memory_full(void)
{
    QTestState *qts = start();
    static const uint8_t looped[] = { 0x0f, 0x00, 0x00, 0x00 };
    static const uint8_t dropped[] = { 0x05, 0x00, 0x00, 0x00 };

    set_memory(qts, 1, 3);
    qtest_writel(qts, INTR_ENA(1), INTR_MEM_FULL);
    qtest_writel(qts, TRIGGER(1), TRIGGER_ON | TRIGGER_MEM_LOOP);
    qtest_writel(qts, TRIGGER(1), TRIGGER_OFF | TRIGGER_MEM_LOOP);
    expect_bytes(qts, BUF, looped, sizeof(looped));
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(1)), ==, BUF + 2);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(1)), ==, INTR_MEM_FULL);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS3) & 0x6, ==,
                    SRC_TRACE_BIT(1));
    qtest_writel(qts, INTR_CLR(1), INTR_MEM_FULL);
    g_assert_cmphex(qtest_readl(qts, INTR_RAW(1)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INTMTX_STATUS3) & 0x6, ==, 0);

    set_memory(qts, 1, 1);
    qtest_writel(qts, TRIGGER(1), TRIGGER_ON);
    qtest_writel(qts, TRIGGER(1), TRIGGER_OFF);
    expect_bytes(qts, BUF, dropped, sizeof(dropped));
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(1)), ==, BUF + 1);
    g_assert_cmphex(qtest_readl(qts, FIFO_STATUS(1)), ==, FIFO_EMPTY);
    g_assert_true(qtest_readl(qts, INTMTX_STATUS3) & SRC_TRACE_BIT(1));
    qtest_quit(qts);
}

/* The FIFO fills while TRACE_SYS_CLK, its master's clock, is stopped. */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_fifo(void)
{
    QTestState *qts = start();

    set_memory(qts, 0, 0x100);
    set_clocks(qts, 0, TRACE_SYS_CLK_EN);
    qtest_writel(qts, TRIGGER(0), TRIGGER_ON);
    qtest_writel(qts, TRIGGER(0), TRIGGER_OFF);
    g_assert_cmphex(qtest_readl(qts, FIFO_STATUS(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(0)), ==, BUF);
    set_clocks(qts, TRACE_SYS_CLK_EN, 0);
    g_assert_cmphex(qtest_readl(qts, FIFO_STATUS(0)), ==, FIFO_EMPTY);
    expect_bytes(qts, BUF, support0, sizeof(support0));
    qtest_quit(qts);
}

/*
 * TRACE_RESET_ENA: the core's reset ends the trace, which starts again
 * with the core.
 */
/* [spec:nuos:req:emu.esp32p4.debug/test] */
static void test_core_reset(void)
{
    QTestState *qts = start();

    set_memory(qts, 0, 0x100);
    qtest_writel(qts, CONFIG(0), CONFIG_RESET_ENA);
    qtest_writel(qts, TRIGGER(0), TRIGGER_ON);
    qtest_writel(qts, HPCPU_RESET_CTRL0,
                 qtest_readl(qts, HPCPU_RESET_CTRL0) | HPCORE_SW_RESET(0));
    qtest_clock_step(qts, 1000000);
    expect_bytes(qts, BUF, support0, sizeof(support0));
    g_assert_cmpuint(work(qts, 0), ==, WORK_WORKING);

    /* Without it the trace goes on through the reset. */
    qtest_writel(qts, CONFIG(0), 0);
    qtest_writel(qts, HPCPU_RESET_CTRL0,
                 qtest_readl(qts, HPCPU_RESET_CTRL0) | HPCORE_SW_RESET(0));
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, MEM_CURRENT(0)), ==, BUF + 5);
    g_assert_cmpuint(work(qts, 0), ==, WORK_WORKING);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32p4/trace/registers", test_registers);
    qtest_add_func("esp32p4/trace/start-stop", test_start_stop);
    qtest_add_func("esp32p4/trace/memory-full", test_memory_full);
    qtest_add_func("esp32p4/trace/fifo", test_fifo);
    qtest_add_func("esp32p4/trace/core-reset", test_core_reset);

    return g_test_run();
}
