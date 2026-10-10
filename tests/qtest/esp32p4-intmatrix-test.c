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
    return g_test_run();
}
