/*
 * QTest testcase for the ESP32-P4 interrupt matrix
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The sources are driven through the matrix's "source" inputs, and their
 * routing seen in the CLICs' pending bits, which for a level-triggered
 * interrupt (the CLIC's reset state) follow the interrupt's line. qtest's
 * accesses come from outside the cores, so they count as machine mode and
 * see core 0's CLIC at its base and core 1's 64 KiB above.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define INTMTX          0x500d6000
#define INTMTX_CORE(n)  (INTMTX + 0x800 * (n))
#define INTMTX_PATH     "/machine/intmatrix"

#define MAP(src)        ((src) < 128 ? 4 * (src) : 0x214 + 4 * ((src) - 128))
#define STATUS(w)       ((w) < 4 ? 0x200 + 4 * (w) : 0x220)
#define CLOCK_GATE      0x210
#define SIG_IDX         0x228
#define SEC_STATUS      0x22c
#define PASS_STATUS(w)  (0x230 + 4 * (w))
#define DATE            0x3fc

#define MAP_PASS_IN_SEC 0x40
#define MAP_IN_SEC_FLAG 0x80

#define NUM_SOURCES     131
#define DATE_RESET      0x2502280

#define CLIC            0x20800000
#define CLIC_OTHER      0x20810000
#define CLIC_MCLICCFG   0x0
#define CLICINT(id)     (0x1000 + 4 * (id))
#define ATTR_MODE_M     0xc0
#define ATTR_MODE_U     0x00

#define HP_SYS_CPU_INT_FROM_CPU(n)  (0x500e5010 + 4 * (n))

#define HP_SYS_CLKRST_SOC_CLK_CTRL2 0x500e601c
#define HP_SYS_CLKRST_HP_RST_EN0    0x500e60c0
#define HP_SYS_CLKRST_HP_RST_EN2    0x500e60c8
#define INTRMTX_APB_CLK_EN          (1u << 4)
#define SYSREG_APB_CLK_EN           (1u << 2)
#define RST_EN_CORE1_GLOBAL         (1u << 8)
#define RST_EN_INTRMTX              (1u << 0)

#define EFUSE                       0x5012d000
#define EFUSE_CONF                  0x1cc
#define EFUSE_CMD                   0x1d4
#define EFUSE_INT_RAW               0x1d8
#define EFUSE_INT_ENA               0x1e0
#define EFUSE_READ_OP_CODE          0x5aa5
#define EFUSE_CMD_READ              0x1
/* A read with the reset timing: 528 cycles of the eFuse's 20 MHz. */
#define EFUSE_READ_NS               (528 * 50)

#define SRC_LP_EFUSE    17
#define SRC_UART0       31
#define SRC_TWAI0       40
#define SRC_FROM_CPU0   79
#define SRC_ISP         100
#define SRC_AXI_PERF    130

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

static uint32_t mtx_read(QTestState *qts, int core, uint32_t off)
{
    return qtest_readl(qts, INTMTX_CORE(core) + off);
}

static void mtx_write(QTestState *qts, int core, uint32_t off, uint32_t v)
{
    qtest_writel(qts, INTMTX_CORE(core) + off, v);
}

static void set_source(QTestState *qts, int src, int level)
{
    qtest_set_irq_in(qts, INTMTX_PATH, "source", src, level);
}

/* The pending bit of CLIC interrupt @id of HP core @core. */
static int clic_ip(QTestState *qts, int core, int id)
{
    return qtest_readb(qts, (core ? CLIC_OTHER : CLIC) + CLICINT(id));
}

/* The external CLIC interrupts of @core pending, bit n for ID 16 + n. */
static uint32_t clic_ext_pending(QTestState *qts, int core)
{
    uint32_t v = 0;

    for (int n = 0; n < 32; n++) {
        v |= (uint32_t)clic_ip(qts, core, 16 + n) << n;
    }
    return v;
}

static bool source_status(QTestState *qts, int core, int src)
{
    return (mtx_read(qts, core, STATUS(src / 32)) >> (src % 32)) & 1;
}

/* Release core 1 from reset, so that its CLIC takes writes. */
static void release_core1(QTestState *qts)
{
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN0,
                 qtest_readl(qts, HP_SYS_CLKRST_HP_RST_EN0) &
                 ~RST_EN_CORE1_GLOBAL);
}

/* Every register of both cores' matrices at its reset value. */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    for (int core = 0; core < 2; core++) {
        for (int src = 0; src < NUM_SOURCES; src++) {
            g_assert_cmphex(mtx_read(qts, core, MAP(src)), ==, 0);
        }
        for (int w = 0; w < 5; w++) {
            g_assert_cmphex(mtx_read(qts, core, STATUS(w)), ==, 0);
            g_assert_cmphex(mtx_read(qts, core, PASS_STATUS(w)), ==, 0);
        }
        g_assert_cmphex(mtx_read(qts, core, CLOCK_GATE), ==, 1);
        g_assert_cmphex(mtx_read(qts, core, SIG_IDX), ==, 0);
        g_assert_cmphex(mtx_read(qts, core, SEC_STATUS), ==, 0);
        g_assert_cmphex(mtx_read(qts, core, DATE), ==, DATE_RESET);
    }
    qtest_quit(qts);
}

/*
 * The register map: each register's implemented bits, the read-only
 * status, the offsets with no register, and byte accesses.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_registers(void)
{
    QTestState *qts = start();

    for (int core = 0; core < 2; core++) {
        /* Every source's mapping register, reserved ones included. */
        for (int src = 0; src < NUM_SOURCES; src++) {
            mtx_write(qts, core, MAP(src), 0xffffffff);
            g_assert_cmphex(mtx_read(qts, core, MAP(src)), ==, 0xff);
            mtx_write(qts, core, MAP(src), 0);
        }
        mtx_write(qts, core, CLOCK_GATE, 0xffffffff);
        g_assert_cmphex(mtx_read(qts, core, CLOCK_GATE), ==, 1);
        mtx_write(qts, core, CLOCK_GATE, 0);
        g_assert_cmphex(mtx_read(qts, core, CLOCK_GATE), ==, 0);
        mtx_write(qts, core, SIG_IDX, 0xffffffff);
        g_assert_cmphex(mtx_read(qts, core, SIG_IDX), ==, 0x3f);
        mtx_write(qts, core, SIG_IDX, 0);
        mtx_write(qts, core, DATE, 0xffffffff);
        g_assert_cmphex(mtx_read(qts, core, DATE), ==, 0x0fffffff);

        /* The status registers are read-only. */
        for (int w = 0; w < 5; w++) {
            mtx_write(qts, core, STATUS(w), 0xffffffff);
            g_assert_cmphex(mtx_read(qts, core, STATUS(w)), ==, 0);
            mtx_write(qts, core, PASS_STATUS(w), 0xffffffff);
            g_assert_cmphex(mtx_read(qts, core, PASS_STATUS(w)), ==, 0);
        }
        mtx_write(qts, core, SEC_STATUS, 0xffffffff);
        g_assert_cmphex(mtx_read(qts, core, SEC_STATUS), ==, 0);

        /* Offsets with no register read as zero and ignore writes. */
        mtx_write(qts, core, 0x224, 0xffffffff);
        g_assert_cmphex(mtx_read(qts, core, 0x224), ==, 0);
        mtx_write(qts, core, 0x244, 0xffffffff);
        g_assert_cmphex(mtx_read(qts, core, 0x244), ==, 0);
        g_assert_cmphex(mtx_read(qts, core, 0x7fc), ==, 0);

        /* A byte write changes its byte only. */
        mtx_write(qts, core, MAP(5), 0x13);
        qtest_writeb(qts, INTMTX_CORE(core) + MAP(5) + 1, 0xff);
        g_assert_cmphex(mtx_read(qts, core, MAP(5)), ==, 0x13);
        qtest_writeb(qts, INTMTX_CORE(core) + MAP(5), 0x5a);
        g_assert_cmphex(mtx_read(qts, core, MAP(5)), ==, 0x5a);
        g_assert_cmphex(qtest_readb(qts, INTMTX_CORE(core) + DATE + 3), ==,
                        0x0f);
        mtx_write(qts, core, MAP(5), 0);
    }

    /* Each core's matrix is its own. */
    mtx_write(qts, 0, MAP(SRC_UART0), 20);
    g_assert_cmphex(mtx_read(qts, 1, MAP(SRC_UART0)), ==, 0);
    mtx_write(qts, 1, DATE, 0x1234567);
    g_assert_cmphex(mtx_read(qts, 0, DATE), ==, 0x0fffffff);
    qtest_quit(qts);
}

/*
 * The status registers show every source's level in both cores'
 * matrices, whatever its mapping: sources 128 to 130 in bits 0 to 2 of
 * INTR_STATUS_REG_4.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_status(void)
{
    QTestState *qts = start();
    static const int srcs[] = { 0, 1, 31, 32, 63, 64, 83, 95, 96, 99, 112,
                                127, 128, 129, 130 };

    for (int i = 0; i < ARRAY_SIZE(srcs); i++) {
        int src = srcs[i];

        set_source(qts, src, 1);
        for (int core = 0; core < 2; core++) {
            for (int w = 0; w < 5; w++) {
                uint32_t want = w == src / 32 ? 1u << (src % 32) : 0;

                g_assert_cmphex(mtx_read(qts, core, STATUS(w)), ==, want);
            }
        }
        set_source(qts, src, 0);
        g_assert_false(source_status(qts, 0, src));
    }
    set_source(qts, SRC_AXI_PERF, 1);
    g_assert_cmphex(mtx_read(qts, 0, STATUS(4)), ==, 4);
    set_source(qts, 128, 1);
    g_assert_cmphex(mtx_read(qts, 1, STATUS(4)), ==, 5);
    qtest_quit(qts);
}

/*
 * A source mapped to 16 to 47 drives that CLIC interrupt of its core, and
 * moves with its mapping; one mapped to 0 to 15 or to 48 to 63 drives
 * nothing.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_routing(void)
{
    QTestState *qts = start();

    set_source(qts, SRC_UART0, 1);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);

    for (int id = 16; id < 48; id++) {
        mtx_write(qts, 0, MAP(SRC_UART0), id);
        g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << (id - 16));
        g_assert_cmphex(clic_ext_pending(qts, 1), ==, 0);
    }
    set_source(qts, SRC_UART0, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    set_source(qts, SRC_UART0, 1);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);

    /* Disabled: neither the CLINT's lines nor any other. */
    for (int id = 0; id < 16; id++) {
        mtx_write(qts, 0, MAP(SRC_UART0), id);
        g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
        g_assert_cmphex(clic_ip(qts, 0, 3), ==, 0);
        g_assert_cmphex(clic_ip(qts, 0, 7), ==, 0);
    }
    for (int id = 48; id < 64; id++) {
        mtx_write(qts, 0, MAP(SRC_UART0), id);
        g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    }
    g_assert_true(source_status(qts, 0, SRC_UART0));

    /* The sources above 127, through their own mapping registers. */
    mtx_write(qts, 0, MAP(SRC_UART0), 0);
    mtx_write(qts, 0, MAP(SRC_AXI_PERF), 33);
    set_source(qts, SRC_AXI_PERF, 1);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 17);
    set_source(qts, SRC_AXI_PERF, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    qtest_quit(qts);
}

/* Sources mapped to one interrupt share it: its level is their OR. */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_shared(void)
{
    QTestState *qts = start();

    mtx_write(qts, 0, MAP(SRC_UART0), 25);
    mtx_write(qts, 0, MAP(SRC_TWAI0), 25);
    mtx_write(qts, 0, MAP(SRC_ISP), 25);
    set_source(qts, SRC_UART0, 1);
    g_assert_cmphex(clic_ip(qts, 0, 25), ==, 1);
    set_source(qts, SRC_TWAI0, 1);
    g_assert_cmphex(clic_ip(qts, 0, 25), ==, 1);
    set_source(qts, SRC_UART0, 0);
    g_assert_cmphex(clic_ip(qts, 0, 25), ==, 1);
    set_source(qts, SRC_TWAI0, 0);
    g_assert_cmphex(clic_ip(qts, 0, 25), ==, 0);
    set_source(qts, SRC_ISP, 1);
    g_assert_cmphex(clic_ip(qts, 0, 25), ==, 1);
    /* Unmapping one source leaves the others. */
    set_source(qts, SRC_TWAI0, 1);
    mtx_write(qts, 0, MAP(SRC_ISP), 0);
    g_assert_cmphex(clic_ip(qts, 0, 25), ==, 1);
    mtx_write(qts, 0, MAP(SRC_TWAI0), 26);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 10);
    qtest_quit(qts);
}

/*
 * Each core's matrix routes to its own core's CLIC; a source mapped in
 * both reaches both.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_both_cores(void)
{
    QTestState *qts = start();

    release_core1(qts);
    mtx_write(qts, 1, MAP(SRC_TWAI0), 40);
    set_source(qts, SRC_TWAI0, 1);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    g_assert_cmphex(clic_ext_pending(qts, 1), ==, 1u << 24);
    mtx_write(qts, 0, MAP(SRC_TWAI0), 30);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 14);
    g_assert_cmphex(clic_ext_pending(qts, 1), ==, 1u << 24);
    set_source(qts, SRC_TWAI0, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    g_assert_cmphex(clic_ext_pending(qts, 1), ==, 0);
    qtest_quit(qts);
}

/*
 * The registers' bus clock and reset: gated, the registers drop writes
 * and keep routing; held in reset, they are at their reset values, so
 * nothing is routed, while the status still shows the sources.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_gating(void)
{
    QTestState *qts = start();
    uint32_t ctrl2 = qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL2);

    g_assert_cmphex(ctrl2 & INTRMTX_APB_CLK_EN, ==, INTRMTX_APB_CLK_EN);
    mtx_write(qts, 0, MAP(SRC_UART0), 20);
    set_source(qts, SRC_UART0, 1);
    g_assert_cmphex(clic_ip(qts, 0, 20), ==, 1);

    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL2, ctrl2 & ~INTRMTX_APB_CLK_EN);
    mtx_write(qts, 0, MAP(SRC_UART0), 21);
    mtx_write(qts, 1, DATE, 0);
    g_assert_cmphex(mtx_read(qts, 0, MAP(SRC_UART0)), ==, 20);
    g_assert_cmphex(mtx_read(qts, 1, DATE), ==, DATE_RESET);
    g_assert_true(source_status(qts, 0, SRC_UART0));
    g_assert_cmphex(clic_ip(qts, 0, 20), ==, 1);
    set_source(qts, SRC_UART0, 0);
    g_assert_cmphex(clic_ip(qts, 0, 20), ==, 0);
    set_source(qts, SRC_UART0, 1);
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL2, ctrl2);
    mtx_write(qts, 0, MAP(SRC_UART0), 21);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 5);

    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN2, RST_EN_INTRMTX);
    g_assert_cmphex(mtx_read(qts, 0, MAP(SRC_UART0)), ==, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    g_assert_true(source_status(qts, 0, SRC_UART0));
    mtx_write(qts, 0, MAP(SRC_UART0), 22);
    g_assert_cmphex(mtx_read(qts, 0, MAP(SRC_UART0)), ==, 0);
    qtest_writel(qts, HP_SYS_CLKRST_HP_RST_EN2, 0);
    g_assert_cmphex(mtx_read(qts, 0, MAP(SRC_UART0)), ==, 0);
    g_assert_true(source_status(qts, 0, SRC_UART0));
    mtx_write(qts, 0, MAP(SRC_UART0), 22);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 6);
    qtest_quit(qts);
}

/*
 * Remapping, with the core in machine mode as it is under qtest: a
 * source with SRC_PASS_IN_SEC mapped to a user-mode interrupt goes to the
 * machine-mode interrupt INTR_SIG_IDX_ASSERT_IN_SEC names, which
 * INTR_SEC_STATUS records; only while all that holds.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_remap(void)
{
    QTestState *qts = start();

    /* NMBITS: clicintattr.MODE decides; 24 user mode, 47 machine mode. */
    qtest_writel(qts, CLIC + CLIC_MCLICCFG, 0x13);
    qtest_writeb(qts, CLIC + CLICINT(24) + 2, ATTR_MODE_U);
    qtest_writeb(qts, CLIC + CLICINT(47) + 2, ATTR_MODE_M);

    mtx_write(qts, 0, MAP(SRC_FROM_CPU0), MAP_PASS_IN_SEC | 24);
    g_assert_cmphex(mtx_read(qts, 0, PASS_STATUS(2)), ==, 1u << 15);
    g_assert_cmphex(mtx_read(qts, 1, PASS_STATUS(2)), ==, 0);
    set_source(qts, SRC_FROM_CPU0, 1);
    /* No target yet: the user-mode interrupt itself. */
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    g_assert_cmphex(mtx_read(qts, 0, SEC_STATUS), ==, 0);

    mtx_write(qts, 0, SIG_IDX, 47);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);
    g_assert_cmphex(mtx_read(qts, 0, SEC_STATUS), ==, 1u << 8);

    /* A source without SRC_PASS_IN_SEC on the same interrupt is not. */
    mtx_write(qts, 0, MAP(SRC_UART0), 24);
    set_source(qts, SRC_UART0, 1);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, (1u << 31) | (1u << 8));
    set_source(qts, SRC_UART0, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);

    /* In user mode the source drives its own interrupt. */
    qtest_set_irq_in(qts, INTMTX_PATH, "core-m-mode", 0, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    g_assert_cmphex(mtx_read(qts, 0, SEC_STATUS), ==, 0);
    qtest_set_irq_in(qts, INTMTX_PATH, "core-m-mode", 0, 1);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);

    /* A target that is a user-mode interrupt takes nothing. */
    qtest_writeb(qts, CLIC + CLICINT(47) + 2, ATTR_MODE_U);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    qtest_writeb(qts, CLIC + CLICINT(47) + 2, ATTR_MODE_M);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);

    /* Nor does a machine-mode mapped interrupt get redirected. */
    qtest_writeb(qts, CLIC + CLICINT(24) + 2, ATTR_MODE_M);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    qtest_writeb(qts, CLIC + CLICINT(24) + 2, ATTR_MODE_U);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);

    /* With NMBITS clear every interrupt is a machine-mode one. */
    qtest_writel(qts, CLIC + CLIC_MCLICCFG, 0x3);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    qtest_writel(qts, CLIC + CLIC_MCLICCFG, 0x13);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 31);

    /* Targets outside 16 to 47 take nothing. */
    mtx_write(qts, 0, SIG_IDX, 7);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    g_assert_cmphex(clic_ip(qts, 0, 7), ==, 0);
    mtx_write(qts, 0, SIG_IDX, 48);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 8);
    mtx_write(qts, 0, SIG_IDX, 46);
    qtest_writeb(qts, CLIC + CLICINT(46) + 2, ATTR_MODE_M);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << 30);

    /* Core 1's matrix does not remap for core 0. */
    g_assert_cmphex(mtx_read(qts, 1, SEC_STATUS), ==, 0);
    set_source(qts, SRC_FROM_CPU0, 0);
    g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    g_assert_cmphex(mtx_read(qts, 0, SEC_STATUS), ==, 0);
    qtest_quit(qts);
}

/*
 * HP_SYSTEM's CPU_INT_FROM_CPU_0 to _3, sources 79 to 82: high while
 * their bit is set; and the eFuse controller's interrupt, source 17.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_machine_sources(void)
{
    QTestState *qts = start();

    for (int n = 0; n < 4; n++) {
        g_assert_cmphex(qtest_readl(qts, HP_SYS_CPU_INT_FROM_CPU(n)), ==, 0);
        qtest_writel(qts, HP_SYS_CPU_INT_FROM_CPU(n), 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, HP_SYS_CPU_INT_FROM_CPU(n)), ==, 1);
        g_assert_cmphex(mtx_read(qts, 0, STATUS(2)), ==, 1u << (15 + n));
        mtx_write(qts, 0, MAP(SRC_FROM_CPU0 + n), 16 + n);
        g_assert_cmphex(clic_ext_pending(qts, 0), ==, 1u << n);
        qtest_writel(qts, HP_SYS_CPU_INT_FROM_CPU(n), 0);
        g_assert_cmphex(mtx_read(qts, 0, STATUS(2)), ==, 0);
        g_assert_cmphex(clic_ext_pending(qts, 0), ==, 0);
    }

    /* EFUSE_INT_ST's READ_DONE once a read ends. */
    mtx_write(qts, 1, MAP(SRC_LP_EFUSE), 35);
    qtest_writel(qts, EFUSE + EFUSE_INT_ENA, 1);
    qtest_writel(qts, EFUSE + EFUSE_CONF, EFUSE_READ_OP_CODE);
    qtest_writel(qts, EFUSE + EFUSE_CMD, EFUSE_CMD_READ);
    g_assert_false(source_status(qts, 1, SRC_LP_EFUSE));
    qtest_clock_step(qts, EFUSE_READ_NS);
    g_assert_cmphex(qtest_readl(qts, EFUSE + EFUSE_INT_RAW), ==, 1);
    g_assert_true(source_status(qts, 0, SRC_LP_EFUSE));
    g_assert_cmphex(clic_ip(qts, 1, 35), ==, 1);
    qtest_writel(qts, EFUSE + EFUSE_INT_RAW, 1);
    g_assert_false(source_status(qts, 1, SRC_LP_EFUSE));
    g_assert_cmphex(clic_ip(qts, 1, 35), ==, 0);
    qtest_quit(qts);
}

/* Every source */

/*
 * Every source as silicon has it: its number (TRM table 13.4-1), its
 * mapping register's offset (ESP-IDF's v3 interrupt_core0_reg.h, whose
 * names these are), and its status register and bit (TRM table 13.4-1).
 * ESP-IDF numbers each source by its mapping register's index, offset / 4,
 * which for sources 128 to 130 is 133 to 135. The TRM's table gives
 * AXI_PERF_MON_INTR bit 3; this takes bit 2, for the reasons
 * include/hw/intc/esp32p4_intmatrix.h gives.
 */
typedef struct SourceInfo {
    int src;
    uint32_t map;
    uint32_t status;
    int bit;
    const char *name;
} SourceInfo;

static const SourceInfo sources[NUM_SOURCES] = {
    {   0, 0x000, 0x200,  0, "LP_RTC" },
    {   1, 0x004, 0x200,  1, "LP_WDT" },
    {   2, 0x008, 0x200,  2, "LP_TIMER_REG_0" },
    {   3, 0x00c, 0x200,  3, "LP_TIMER_REG_1" },
    {   4, 0x010, 0x200,  4, "MB_HP" },
    {   5, 0x014, 0x200,  5, "MB_LP" },
    {   6, 0x018, 0x200,  6, "PMU_REG_0" },
    {   7, 0x01c, 0x200,  7, "PMU_REG_1" },
    {   8, 0x020, 0x200,  8, "LP_ANAPERI" },
    {   9, 0x024, 0x200,  9, "LP_ADC" },
    {  10, 0x028, 0x200, 10, "LP_GPIO" },
    {  11, 0x02c, 0x200, 11, "LP_I2C" },
    {  12, 0x030, 0x200, 12, "LP_I2S" },
    {  13, 0x034, 0x200, 13, "LP_SPI" },
    {  14, 0x038, 0x200, 14, "LP_TOUCH" },
    {  15, 0x03c, 0x200, 15, "LP_TSENS" },
    {  16, 0x040, 0x200, 16, "LP_UART" },
    {  17, 0x044, 0x200, 17, "LP_EFUSE" },
    {  18, 0x048, 0x200, 18, "LP_SW" },
    {  19, 0x04c, 0x200, 19, "LP_SYSREG" },
    {  20, 0x050, 0x200, 20, "LP_HUK" },
    {  21, 0x054, 0x200, 21, "SYS_ICM" },
    {  22, 0x058, 0x200, 22, "USB_DEVICE" },
    {  23, 0x05c, 0x200, 23, "SDIO_HOST" },
    {  24, 0x060, 0x200, 24, "GDMA" },
    {  25, 0x064, 0x200, 25, "SPI2" },
    {  26, 0x068, 0x200, 26, "SPI3" },
    {  27, 0x06c, 0x200, 27, "I2S0" },
    {  28, 0x070, 0x200, 28, "I2S1" },
    {  29, 0x074, 0x200, 29, "I2S2" },
    {  30, 0x078, 0x200, 30, "UHCI0" },
    {  31, 0x07c, 0x200, 31, "UART0" },
    {  32, 0x080, 0x204,  0, "UART1" },
    {  33, 0x084, 0x204,  1, "UART2" },
    {  34, 0x088, 0x204,  2, "UART3" },
    {  35, 0x08c, 0x204,  3, "UART4" },
    {  36, 0x090, 0x204,  4, "LCD_CAM" },
    {  37, 0x094, 0x204,  5, "ADC" },
    {  38, 0x098, 0x204,  6, "PWM0" },
    {  39, 0x09c, 0x204,  7, "PWM1" },
    {  40, 0x0a0, 0x204,  8, "CAN0" },
    {  41, 0x0a4, 0x204,  9, "CAN1" },
    {  42, 0x0a8, 0x204, 10, "CAN2" },
    {  43, 0x0ac, 0x204, 11, "RMT" },
    {  44, 0x0b0, 0x204, 12, "I2C0" },
    {  45, 0x0b4, 0x204, 13, "I2C1" },
    {  46, 0x0b8, 0x204, 14, "TIMERGRP0_T0" },
    {  47, 0x0bc, 0x204, 15, "TIMERGRP0_T1" },
    {  48, 0x0c0, 0x204, 16, "TIMERGRP0_WDT" },
    {  49, 0x0c4, 0x204, 17, "TIMERGRP1_T0" },
    {  50, 0x0c8, 0x204, 18, "TIMERGRP1_T1" },
    {  51, 0x0cc, 0x204, 19, "TIMERGRP1_WDT" },
    {  52, 0x0d0, 0x204, 20, "LEDC" },
    {  53, 0x0d4, 0x204, 21, "SYSTIMER_TARGET0" },
    {  54, 0x0d8, 0x204, 22, "SYSTIMER_TARGET1" },
    {  55, 0x0dc, 0x204, 23, "SYSTIMER_TARGET2" },
    {  56, 0x0e0, 0x204, 24, "AHB_PDMA_IN_CH0" },
    {  57, 0x0e4, 0x204, 25, "AHB_PDMA_IN_CH1" },
    {  58, 0x0e8, 0x204, 26, "AHB_PDMA_IN_CH2" },
    {  59, 0x0ec, 0x204, 27, "AHB_PDMA_OUT_CH0" },
    {  60, 0x0f0, 0x204, 28, "AHB_PDMA_OUT_CH1" },
    {  61, 0x0f4, 0x204, 29, "AHB_PDMA_OUT_CH2" },
    {  62, 0x0f8, 0x204, 30, "AXI_PDMA_IN_CH0" },
    {  63, 0x0fc, 0x204, 31, "AXI_PDMA_IN_CH1" },
    {  64, 0x100, 0x208,  0, "AXI_PDMA_IN_CH2" },
    {  65, 0x104, 0x208,  1, "AXI_PDMA_OUT_CH0" },
    {  66, 0x108, 0x208,  2, "AXI_PDMA_OUT_CH1" },
    {  67, 0x10c, 0x208,  3, "AXI_PDMA_OUT_CH2" },
    {  68, 0x110, 0x208,  4, "RSA" },
    {  69, 0x114, 0x208,  5, "AES" },
    {  70, 0x118, 0x208,  6, "SHA" },
    {  71, 0x11c, 0x208,  7, "ECC" },
    {  72, 0x120, 0x208,  8, "ECDSA" },
    {  73, 0x124, 0x208,  9, "KM" },
    {  74, 0x128, 0x208, 10, "GPIO_INT0" },
    {  75, 0x12c, 0x208, 11, "GPIO_INT1" },
    {  76, 0x130, 0x208, 12, "GPIO_INT2" },
    {  77, 0x134, 0x208, 13, "GPIO_INT3" },
    {  78, 0x138, 0x208, 14, "GPIO_PAD_COMP" },
    {  79, 0x13c, 0x208, 15, "CPU_INT_FROM_CPU_0" },
    {  80, 0x140, 0x208, 16, "CPU_INT_FROM_CPU_1" },
    {  81, 0x144, 0x208, 17, "CPU_INT_FROM_CPU_2" },
    {  82, 0x148, 0x208, 18, "CPU_INT_FROM_CPU_3" },
    {  83, 0x14c, 0x208, 19, "CACHE" },
    {  84, 0x150, 0x208, 20, "FLASH_MSPI" },
    {  85, 0x154, 0x208, 21, "CSI_BRIDGE" },
    {  86, 0x158, 0x208, 22, "DSI_BRIDGE" },
    {  87, 0x15c, 0x208, 23, "CSI" },
    {  88, 0x160, 0x208, 24, "DSI" },
    {  89, 0x164, 0x208, 25, "GMII_PHY" },
    {  90, 0x168, 0x208, 26, "LPI" },
    {  91, 0x16c, 0x208, 27, "PMT" },
    {  92, 0x170, 0x208, 28, "SBD" },
    {  93, 0x174, 0x208, 29, "USB_OTG" },
    {  94, 0x178, 0x208, 30, "USB_OTG_ENDP_MULTI_PROC" },
    {  95, 0x17c, 0x208, 31, "JPEG" },
    {  96, 0x180, 0x20c,  0, "PPA" },
    {  97, 0x184, 0x20c,  1, "CORE0_TRACE" },
    {  98, 0x188, 0x20c,  2, "CORE1_TRACE" },
    {  99, 0x18c, 0x20c,  3, "HP_CORE_CTRL" },
    { 100, 0x190, 0x20c,  4, "ISP" },
    { 101, 0x194, 0x20c,  5, "I3C_MST" },
    { 102, 0x198, 0x20c,  6, "I3C_SLV" },
    { 103, 0x19c, 0x20c,  7, "USB_OTG11" },
    { 104, 0x1a0, 0x20c,  8, "DMA2D_IN_CH0" },
    { 105, 0x1a4, 0x20c,  9, "DMA2D_IN_CH1" },
    { 106, 0x1a8, 0x20c, 10, "DMA2D_OUT_CH0" },
    { 107, 0x1ac, 0x20c, 11, "DMA2D_OUT_CH1" },
    { 108, 0x1b0, 0x20c, 12, "DMA2D_OUT_CH2" },
    { 109, 0x1b4, 0x20c, 13, "PSRAM_MSPI" },
    { 110, 0x1b8, 0x20c, 14, "HP_SYSREG" },
    { 111, 0x1bc, 0x20c, 15, "PCNT" },
    { 112, 0x1c0, 0x20c, 16, "HP_PAU" },
    { 113, 0x1c4, 0x20c, 17, "HP_PARLIO_RX" },
    { 114, 0x1c8, 0x20c, 18, "HP_PARLIO_TX" },
    { 115, 0x1cc, 0x20c, 19, "H264_DMA2D_OUT_CH0" },
    { 116, 0x1d0, 0x20c, 20, "H264_DMA2D_OUT_CH1" },
    { 117, 0x1d4, 0x20c, 21, "H264_DMA2D_OUT_CH2" },
    { 118, 0x1d8, 0x20c, 22, "H264_DMA2D_OUT_CH3" },
    { 119, 0x1dc, 0x20c, 23, "H264_DMA2D_OUT_CH4" },
    { 120, 0x1e0, 0x20c, 24, "H264_DMA2D_IN_CH0" },
    { 121, 0x1e4, 0x20c, 25, "H264_DMA2D_IN_CH1" },
    { 122, 0x1e8, 0x20c, 26, "H264_DMA2D_IN_CH2" },
    { 123, 0x1ec, 0x20c, 27, "H264_DMA2D_IN_CH3" },
    { 124, 0x1f0, 0x20c, 28, "H264_DMA2D_IN_CH4" },
    { 125, 0x1f4, 0x20c, 29, "H264_DMA2D_IN_CH5" },
    { 126, 0x1f8, 0x20c, 30, "H264_REG" },
    { 127, 0x1fc, 0x20c, 31, "ASSIST_DEBUG" },
    { 128, 0x214, 0x220,  0, "DMA2D_IN_CH2" },
    { 129, 0x218, 0x220,  1, "DMA2D_OUT_CH3" },
    { 130, 0x21c, 0x220,  2, "AXI_PERF_MON" },
};

#define LP_INTR                     0x5012c000
#define LP_SW_INT_RAW               (LP_INTR + 0x0)
#define LP_SW_INT_ENA               (LP_INTR + 0x8)
#define LP_SW_INT_CLR               (LP_INTR + 0xc)
#define LP_SW_INT                   (1u << 31)

#define UART_BASE(n)                ((n) < 5 ? 0x500ca000 + 0x1000 * (n) : \
                                     0x50121000)
#define UART_INT_RAW                0x04
#define UART_INT_ENA                0x0c
#define UART_TXFIFO_EMPTY           (1u << 1)

#define AHB_GDMA                    0x50085000
#define AXI_GDMA                    0x5008a000
#define DMA2D                       0x50088000

#define APM                         0x500a8000
#define APM_METRIC_RANGE(m, n)      (APM + 0x0c8 + 8 * (m) + 4 * (n))
#define APM_METRIC_SELECT(k, n)     (APM + 0x158 + 8 * (k) + 4 * (n))
#define APM_SW_RECORD_STOP_CLR      (APM + 0x178)
#define APM_INS_BANDW_TEST_EN       (APM + 0x17c)
#define APM_STOP_DATA_LIMIT(n)      (APM + 0x180 + 4 * (n))
#define APM_INS_TIME_THR(n)         (APM + 0x1b8 + 4 * (n))
#define APM_INT_RAW                 (APM + 0x1c0)
#define APM_INT_ENA                 (APM + 0x1c8)
#define APM_INT_CLR                 (APM + 0x1cc)
#define HP_SYS_CLKRST_APM           0x500e60f0

#define HP_SYS_CLKRST_SOC_CLK_CTRL1 0x500e6018
#define AHB_PDMA_SYS_CLK_EN         (1u << 3)
#define AXI_PDMA_SYS_CLK_EN         (1u << 4)
#define DMA2D_SYS_CLK_EN            (1u << 6)

/* A descriptor address no DMA takes: not even word-aligned. */
#define BAD_DESC                    0x4ff40002

/*
 * A DMA channel's interrupt registers and how to start it: @raw is its
 * INT_RAW, the enable and clear registers at +8 and +0xc (+4 and +0xc for
 * the 2D-DMA's), @link its link register and @start the start bit there,
 * @addr its descriptor address register, @dscr_err its DSCR_ERR bit.
 */
typedef struct DmaChan {
    uint32_t raw;
    uint32_t ena;
    uint32_t clr;
    uint32_t link;
    uint32_t start;
    uint32_t addr;
    uint32_t dscr_err;
} DmaChan;

/*
 * The DMA channel driving source @src: GDMA-AHB's and GDMA-AXI's RX then
 * TX channels 0 to 2 (sources 56 to 67), the 2D-DMA's RX channels 0 and 1,
 * TX channels 0 to 2, RX channel 2 and TX channel 3 (104 to 108, 128 and
 * 129). False for any other source.
 */
static bool dma_chan(int src, DmaChan *c)
{
    static const int dma2d[] = { 0x500, 0x600, 0x000, 0x100, 0x200 };

    if (src >= 56 && src < 68) {
        bool axi = src >= 62;
        int k = src - (axi ? 62 : 56), ch = k % 3;
        bool rx = k < 3;

        if (axi) {
            uint32_t b = AXI_GDMA + (rx ? 0x68 * ch : 0x138 + 0x68 * ch);

            *c = (DmaChan) { b, b + 0x8, b + 0xc, b + 0x20,
                             rx ? 1u << 2 : 1u << 1, b + 0x24,
                             rx ? 1u << 3 : 1u << 2 };
        } else {
            uint32_t i = AHB_GDMA + (rx ? 0x10 * ch : 0x30 + 0x10 * ch);
            uint32_t b = AHB_GDMA + (rx ? 0x70 : 0xd0) + 0xc0 * ch;

            *c = (DmaChan) { i, i + 0x8, i + 0xc, b + 0x10,
                             rx ? 1u << 2 : 1u << 1,
                             AHB_GDMA + (rx ? 0x3ac : 0x3b8) + 4 * ch,
                             rx ? 1u << 3 : 1u << 2 };
        }
        return true;
    }
    if ((src >= 104 && src < 109) || src == 128 || src == 129) {
        uint32_t b = DMA2D + (src == 128 ? 0x700 : src == 129 ? 0x300 :
                              dma2d[src - 104]);
        bool rx = src == 104 || src == 105 || src == 128;

        *c = (DmaChan) { b + 0x04, b + 0x08, b + 0x10, b + 0x1c,
                         rx ? 1u << 22 : 1u << 21, b + 0x20,
                         rx ? 1u << 3 : 1u << 2 };
        return true;
    }
    return false;
}

/* The bus clocks the blocks that raise their sources below need. */
static void blocks_on(QTestState *qts)
{
    qtest_writel(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1,
                 qtest_readl(qts, HP_SYS_CLKRST_SOC_CLK_CTRL1) |
                 AHB_PDMA_SYS_CLK_EN | AXI_PDMA_SYS_CLK_EN |
                 DMA2D_SYS_CLK_EN);
    qtest_writel(qts, HP_SYS_CLKRST_APM, 1);
    qtest_writel(qts, LP_SW_INT_ENA, LP_SW_INT);
}

/*
 * Raise (@level 1) or lower source @src through its block, where that is
 * cheap: HP_SYSTEM's CPU_INT_FROM_CPU_n, the LP interrupt controller's
 * software interrupt, the UARTs' TXFIFO_EMPTY (true with the TX FIFO
 * empty) enabled, a descriptor error on a DMA channel, an eFuse read, and
 * the AXI performance monitor's under-limit interrupt. False for the
 * other sources.
 */
static bool block_set(QTestState *qts, int src, int level)
{
    DmaChan c;

    if (src >= SRC_FROM_CPU0 && src < SRC_FROM_CPU0 + 4) {
        qtest_writel(qts, HP_SYS_CPU_INT_FROM_CPU(src - SRC_FROM_CPU0), level);
    } else if (src == 18) {
        qtest_writel(qts, level ? LP_SW_INT_RAW : LP_SW_INT_CLR, LP_SW_INT);
    } else if ((src >= 31 && src < 36) || src == 16) {
        uint32_t b = UART_BASE(src == 16 ? 5 : src - 31);

        g_assert_cmphex(qtest_readl(qts, b + UART_INT_RAW) & UART_TXFIFO_EMPTY,
                        ==, UART_TXFIFO_EMPTY);
        qtest_writel(qts, b + UART_INT_ENA, level ? UART_TXFIFO_EMPTY : 0);
    } else if (dma_chan(src, &c)) {
        if (level) {
            qtest_writel(qts, c.ena, c.dscr_err);
            qtest_writel(qts, c.addr, BAD_DESC);
            qtest_writel(qts, c.link, c.start);
            qtest_clock_step(qts, 10000);
            g_assert_cmphex(qtest_readl(qts, c.raw) & c.dscr_err, ==,
                            c.dscr_err);
        } else {
            qtest_writel(qts, c.clr, c.dscr_err);
            qtest_writel(qts, c.ena, 0);
        }
    } else if (src == SRC_LP_EFUSE) {
        if (level) {
            qtest_writel(qts, EFUSE + EFUSE_INT_ENA, 1);
            qtest_writel(qts, EFUSE + EFUSE_CONF, EFUSE_READ_OP_CODE);
            qtest_writel(qts, EFUSE + EFUSE_CMD, EFUSE_CMD_READ);
            qtest_clock_step(qts, EFUSE_READ_NS);
        } else {
            qtest_writel(qts, EFUSE + EFUSE_INT_RAW, 1);
        }
    } else if (src == SRC_AXI_PERF) {
        if (level) {
            /* A read window of 1000 AXI cycles under a limit of one. */
            qtest_writel(qts, APM_INS_BANDW_TEST_EN, 0);
            qtest_writel(qts, APM_SW_RECORD_STOP_CLR, 0x3);
            qtest_writel(qts, APM_METRIC_SELECT(0, 0), 1u << 13);
            qtest_writel(qts, APM_METRIC_RANGE(1, 0), 5);
            qtest_writel(qts, APM_INS_TIME_THR(0), 1000);
            qtest_writel(qts, APM_STOP_DATA_LIMIT(0), 1);
            qtest_writel(qts, APM_INT_ENA, 1);
            qtest_writel(qts, APM_INS_BANDW_TEST_EN, 1);
            qtest_clock_step(qts, 200000);
            g_assert_cmphex(qtest_readl(qts, APM_INT_RAW) & 1, ==, 1);
        } else {
            qtest_writel(qts, APM_INT_CLR, 1);
            qtest_writel(qts, APM_INS_BANDW_TEST_EN, 0);
        }
    } else {
        return false;
    }
    return true;
}

/*
 * Every source, on each core: mapped through its mapping register, at
 * its own offset, to a CLIC interrupt, raised (through its block where
 * that is cheap, otherwise at the matrix's input), seen in its own status
 * bit and nowhere else, in SRC_PASS_IN_SEC's status at the same bit, and
 * on that CLIC interrupt of that core alone; then lowered.
 */
/* [spec:nuos:req:emu.esp32p4.interrupt-matrix/test] */
static void test_walk(void)
{
    QTestState *qts = start();
    int by_block = 0;

    release_core1(qts);
    blocks_on(qts);
    for (int core = 0; core < 2; core++) {
        for (int i = 0; i < NUM_SOURCES; i++) {
            const SourceInfo *si = &sources[i];
            int id = 16 + (si->src + 7 * core) % 32;
            uint32_t pass = si->status - 0x200 + 0x230 + (si->src >= 128 ?
                                                          0x10 - 0x20 : 0);
            bool block;

            g_assert_cmpint(si->src, ==, i);
            mtx_write(qts, core, si->map, MAP_PASS_IN_SEC | id);
            g_assert_cmphex(mtx_read(qts, core, si->map), ==,
                            MAP_PASS_IN_SEC | id);
            g_assert_cmphex(mtx_read(qts, core, pass), ==, 1u << si->bit);
            g_assert_cmphex(clic_ext_pending(qts, core), ==, 0);

            block = block_set(qts, si->src, 1);
            if (!block) {
                set_source(qts, si->src, 1);
            }
            by_block += block;
            for (int w = 0; w < 5; w++) {
                uint32_t want = STATUS(w) == si->status ? 1u << si->bit : 0;

                if (mtx_read(qts, core, STATUS(w)) != want) {
                    g_test_message("source %d (%s)", si->src, si->name);
                }
                g_assert_cmphex(mtx_read(qts, core, STATUS(w)), ==, want);
            }
            g_assert_cmphex(clic_ext_pending(qts, core), ==, 1u << (id - 16));
            g_assert_cmphex(clic_ext_pending(qts, !core), ==, 0);

            if (block) {
                block_set(qts, si->src, 0);
            } else {
                set_source(qts, si->src, 0);
            }
            g_assert_cmphex(mtx_read(qts, core, si->status), ==, 0);
            g_assert_cmphex(clic_ext_pending(qts, core), ==, 0);
            mtx_write(qts, core, si->map, 0);
        }
    }
    g_assert_cmpint(by_block, ==, 2 * 32);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/intmatrix/reset", test_reset);
    qtest_add_func("/esp32p4/intmatrix/registers", test_registers);
    qtest_add_func("/esp32p4/intmatrix/status", test_status);
    qtest_add_func("/esp32p4/intmatrix/routing", test_routing);
    qtest_add_func("/esp32p4/intmatrix/shared", test_shared);
    qtest_add_func("/esp32p4/intmatrix/both-cores", test_both_cores);
    qtest_add_func("/esp32p4/intmatrix/gating", test_gating);
    qtest_add_func("/esp32p4/intmatrix/remap", test_remap);
    qtest_add_func("/esp32p4/intmatrix/machine-sources",
                   test_machine_sources);
    qtest_add_func("/esp32p4/intmatrix/walk", test_walk);
    return g_test_run();
}
