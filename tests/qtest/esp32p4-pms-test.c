/*
 * QTest testcase for the ESP32-P4 access permission managers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * qtest's own accesses carry no master, so the APMs let them through: they
 * set up and read back the registers. An access as a given master, an HP
 * core in either mode, the LP core or a DMA master, goes through the PMS
 * device's "probe" hook, which makes it on the bus with that master's
 * attributes. The interrupts are seen in the interrupt matrix's status
 * registers, which follow every source's level.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define HP_PERI_PMS     0x500a5000
#define LP2HP_PERI_PMS  0x500a5800
#define DMA_PMS         0x500a6000
#define LP_PERI_PMS     0x5012e000
#define HP2LP_PERI_PMS  0x5012e800
#define ICM             0x500a4000
#define LP_SYS          0x50110000
#define LPPERI          0x50120000

#define PERI_DATE       0x241227
#define DMA_DATE        0x20230314

/* HP_PERI_PMS register k of core c in mode m (0 machine, 1 user). */
#define HP_PERI(c, m, k)    (HP_PERI_PMS + 0x08 + 0x20 * (c) + 0x10 * (m) + \
                             4 * (k))
#define REGDMA_HP_PERI      (HP_PERI_PMS + 0x48)
static const uint32_t lp2hp_reg[4] = { 0x08, 0x30, 0xa4, 0x11c };
static const uint32_t hp_peri_bits[4] = {
    0x00000fcf, 0x3fffffff, 0xdfefffff, 0x0000001f,
};

#define DMA_LOW(n)          (DMA_PMS + 0x08 + 8 * (n))
#define DMA_HIGH(n)         (DMA_PMS + 0x0c + 8 * (n))
#define DMA_GDMA_CH0_R      (DMA_PMS + 0x108)
#define DMA_GDMA_CH0_W      (DMA_PMS + 0x10c)
#define DMA_RMT_R           (DMA_PMS + 0x158)
#define DMA_RMT_W           (DMA_PMS + 0x170)
#define DMA_GMAC_R          (DMA_PMS + 0x1b4)
#define DMA_GMAC_W          (DMA_PMS + 0x1b8)
#define DMA_REGDMA_R        (DMA_PMS + 0x1ec)
#define DMA_REGDMA_W        (DMA_PMS + 0x1f0)

#define LP_PERI_LP          (LP_PERI_PMS + 0x08)
#define REGION_LOW(n)       (LP_PERI_PMS + ((n) < 2 ? 0x0c + 8 * (n) \
                                                    : 0x24 + 8 * ((n) - 2)))
#define REGION_HIGH(n)      (REGION_LOW(n) + 4)
#define REGION_PMS          (LP_PERI_PMS + 0x1c)
#define REGION_2_TO_7_PMS   (LP_PERI_PMS + 0x20)
/* Region 0's and 1's bits in REGION_PMS: core 0's machine mode. */
#define REGION_C0M(n)       (1u << (4 + (n)))
/* Region 2 to 7's bits in REGION_2_TO_7_PMS: core 1's user mode. */
#define REGION27_C1U(n)     (1u << (18 + (n) - 2))

#define HP2LP(c, m)         (HP2LP_PERI_PMS + 0x08 + 8 * (c) + 4 * (m))
#define REGDMA_LP_PERI      (HP2LP_PERI_PMS + 0x18)

#define ICM_VER_DATE        (ICM + 0x00)
#define ICM_CLK_EN          (ICM + 0x04)
#define ICM_DLOCK_STATUS    (ICM + 0x08)
#define ICM_INT_RAW         (ICM + 0x0c)
#define ICM_INT_ST          (ICM + 0x10)
#define ICM_INT_ENA         (ICM + 0x14)
#define ICM_INT_CLR         (ICM + 0x18)
#define ICM_SLV_ARB         (ICM + 0x24)
#define ICM_SYS_ADDR        (ICM + 0x38)
#define ICM_SYS_INFO        (ICM + 0x3c)
#define ICM_CPU_ADDR        (ICM + 0x40)
#define ICM_CPU_INFO        (ICM + 0x44)
#define ICM_DLOCK_TIMEOUT   (ICM + 0x48)
#define ICM_RDN_ECO_CS      (ICM + 0x50)
#define ICM_RDN_ECO_HIGH    (ICM + 0x58)
#define ICM_INT_SYS         0x2
#define ICM_INT_CPU         0x4
#define ICM_WR              0x100

#define LP_SYS_ADDRHOLE_ADDR (LP_SYS + 0x168)
#define LP_SYS_ADDRHOLE_INFO (LP_SYS + 0x16c)
#define LP_SYS_INT_RAW      (LP_SYS + 0x170)
#define LP_SYS_INT_ST       (LP_SYS + 0x174)
#define LP_SYS_INT_ENA      (LP_SYS + 0x178)
#define LP_SYS_INT_CLR      (LP_SYS + 0x17c)
#define LP_SYS_WR           0x20

#define LPPERI_CLK_EN       (LPPERI + 0x0)
#define LPPERI_RESET_EN     (LPPERI + 0x8)
#define CK_EN_LP_PMS        (1u << 18)
#define RST_EN_LP_PMS       (1u << 19)

/* Interrupt sources, in core 0's interrupt matrix status. */
#define INTMTX_STATUS0      (0x500d6000 + 0x200)
#define SRC_LP_SYSREG       19
#define SRC_SYS_ICM         21

/* The requester IDs of include/hw/misc/esp32p4_pms.h. */
#define RID_CPU(id)         (0x8000 | (id))
#define RID_DMA(m, id)      (0xc000 | ((m) << 8) | (id))
#define CORE0               RID_CPU(0)
#define CORE1               RID_CPU(1)
#define LP_CORE             RID_CPU(2)
#define DMA_GDMA_CH0        0
#define DMA_RMT             10
#define DMA_GMAC            21
#define DMA_REGDMA          28

/* Peripherals and memories the tests reach. */
#define UART0               0x500ca000
#define UART1               0x500cb000
#define PCNT                0x500c9000
#define HP_SYS_FROM_CPU0    0x500e5010
#define TRACE0              0x3ff04000
#define L2MEM               0x4ff00000
#define L2MEM_DIRECT        0x8ff00000
#define ROM_DIRECT          0x8fc00000
#define LP_SRAM             0x50108000
#define TCM                 0x30100000
/*
 * Words of L2MEM for the tests to write, 256 KiB in: past the 128 KiB the
 * L2 cache takes from reset.
 */
#define SCRATCH             0x4ff40000

#define MEMTX_OK            0

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M");
}

/*
 * Make a 32-bit access at @addr as the master @rid (in user mode if
 * @user) through the "probe" hook: its MemTxResult, and in *@data what a
 * read returned.
 */
static unsigned probe(QTestState *qts, uint32_t rid, bool user, bool write,
                      uint32_t addr, uint32_t value, uint32_t *data)
{
    g_autofree char *cmd = g_strdup_printf("%u %d %c 0x%x 0x%x", rid, user,
                                           write ? 'w' : 'r', addr, value);
    QDict *rsp;
    unsigned result, got;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': '/machine/pms', 'property': 'probe', "
                    "'value': %s}}", cmd);
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    rsp = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                    "{'path': '/machine/pms', 'property': 'probe'}}");
    g_assert(qdict_haskey(rsp, "return"));
    g_assert_cmpint(sscanf(qdict_get_str(rsp, "return"), "%u 0x%x", &result,
                           &got), ==, 2);
    qobject_unref(rsp);
    if (data) {
        *data = got;
    }
    return result;
}

static bool can_read(QTestState *qts, uint32_t rid, bool user, uint32_t addr)
{
    return probe(qts, rid, user, false, addr, 0, NULL) == MEMTX_OK;
}

static bool can_write(QTestState *qts, uint32_t rid, bool user,
                      uint32_t addr, uint32_t value)
{
    return probe(qts, rid, user, true, addr, value, NULL) == MEMTX_OK;
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> src) & 1;
}

static void clear_records(QTestState *qts)
{
    qtest_writel(qts, ICM_INT_CLR, 0x7);
    qtest_writel(qts, LP_SYS_INT_CLR, 0x7f);
}

/* The registers and their reset values */

static void check_hp_peri_reset(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, HP_PERI_PMS), ==, PERI_DATE);
    g_assert_cmphex(qtest_readl(qts, HP_PERI_PMS + 4), ==, 1);
    for (int c = 0; c < 2; c++) {
        for (int m = 0; m < 2; m++) {
            for (int k = 0; k < 4; k++) {
                g_assert_cmphex(qtest_readl(qts, HP_PERI(c, m, k)), ==,
                                hp_peri_bits[k]);
            }
        }
    }
    g_assert_cmphex(qtest_readl(qts, REGDMA_HP_PERI), ==, 1);
    g_assert_cmphex(qtest_readl(qts, HP_PERI_PMS + 0x4c), ==, 0);

    g_assert_cmphex(qtest_readl(qts, LP2HP_PERI_PMS), ==, PERI_DATE);
    g_assert_cmphex(qtest_readl(qts, LP2HP_PERI_PMS + 4), ==, 1);
    for (int k = 0; k < 4; k++) {
        g_assert_cmphex(qtest_readl(qts, LP2HP_PERI_PMS + lp2hp_reg[k]), ==,
                        hp_peri_bits[k]);
    }
    g_assert_cmphex(qtest_readl(qts, LP2HP_PERI_PMS + 0x0c), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP2HP_PERI_PMS + 0x34), ==, 0);
}

static void check_dma_reset(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, DMA_PMS), ==, DMA_DATE);
    g_assert_cmphex(qtest_readl(qts, DMA_PMS + 4), ==, 1);
    for (int n = 0; n < 32; n++) {
        g_assert_cmphex(qtest_readl(qts, DMA_LOW(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, DMA_HIGH(n)), ==, 0xfffff000);
    }
    for (uint32_t off = 0x108; off < 0x224; off += 4) {
        bool reserved = (off >= 0x15c && off <= 0x16c) || off == 0x1f4 ||
                        off == 0x1f8;

        g_assert_cmphex(qtest_readl(qts, DMA_PMS + off), ==,
                        reserved ? 0 : 0xffffffff);
    }
    g_assert_cmphex(qtest_readl(qts, DMA_PMS + 0x224), ==, 0);
}

static void check_lp_reset(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, LP_PERI_PMS), ==, PERI_DATE);
    g_assert_cmphex(qtest_readl(qts, LP_PERI_PMS + 4), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LP_PERI_LP), ==, 0x1ffffff);
    for (int n = 0; n < 8; n++) {
        g_assert_cmphex(qtest_readl(qts, REGION_LOW(n)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, REGION_HIGH(n)), ==, 0xfffffffc);
    }
    g_assert_cmphex(qtest_readl(qts, REGION_PMS), ==, 0x3ff);
    g_assert_cmphex(qtest_readl(qts, REGION_2_TO_7_PMS), ==, 0x3fffffff);
    g_assert_cmphex(qtest_readl(qts, LP_PERI_PMS + 0x54), ==, 0);

    g_assert_cmphex(qtest_readl(qts, HP2LP_PERI_PMS), ==, PERI_DATE);
    g_assert_cmphex(qtest_readl(qts, HP2LP_PERI_PMS + 4), ==, 1);
    for (int c = 0; c < 2; c++) {
        for (int m = 0; m < 2; m++) {
            g_assert_cmphex(qtest_readl(qts, HP2LP(c, m)), ==, 0x1ffffff);
        }
    }
    g_assert_cmphex(qtest_readl(qts, REGDMA_LP_PERI), ==, 3);
    g_assert_cmphex(qtest_readl(qts, HP2LP_PERI_PMS + 0x1c), ==, 0);
}

/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_reset(void)
{
    QTestState *qts = start();

    check_hp_peri_reset(qts);
    check_dma_reset(qts);
    check_lp_reset(qts);

    g_assert_cmphex(qtest_readl(qts, ICM_VER_DATE), ==, 0x20230214);
    g_assert_cmphex(qtest_readl(qts, ICM_CLK_EN), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_DLOCK_STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_ENA), ==, 7);
    g_assert_cmphex(qtest_readl(qts, ICM_DLOCK_TIMEOUT), ==, 2048);
    g_assert_cmphex(qtest_readl(qts, ICM_RDN_ECO_HIGH), ==, 0xffffffff);
    for (uint32_t a = ICM_SYS_ADDR; a <= ICM_CPU_INFO; a += 4) {
        g_assert_cmphex(qtest_readl(qts, a), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_ADDR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_ENA), ==, 0);

    /* Back to the reset values with the system. */
    qtest_writel(qts, HP_PERI(1, 1, 2), 0);
    qtest_writel(qts, DMA_HIGH(7), 0);
    qtest_writel(qts, REGION_PMS, 0);
    qtest_system_reset(qts);
    check_hp_peri_reset(qts);
    check_dma_reset(qts);
    check_lp_reset(qts);
    qtest_quit(qts);
}

static void check_mask(QTestState *qts, uint32_t addr, uint32_t mask)
{
    qtest_writel(qts, addr, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, addr), ==, mask);
    qtest_writel(qts, addr, 0);
    g_assert_cmphex(qtest_readl(qts, addr), ==, 0);
}

/* Every register's writable bits; reserved bits and offsets take nothing. */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_masks(void)
{
    QTestState *qts = start();

    check_mask(qts, HP_PERI_PMS, 0xffffffff);
    check_mask(qts, HP_PERI_PMS + 4, 1);
    for (int c = 0; c < 2; c++) {
        for (int m = 0; m < 2; m++) {
            for (int k = 0; k < 4; k++) {
                check_mask(qts, HP_PERI(c, m, k), hp_peri_bits[k]);
            }
        }
    }
    check_mask(qts, REGDMA_HP_PERI, 1);
    check_mask(qts, HP_PERI_PMS + 0x4c, 0);
    for (int k = 0; k < 4; k++) {
        check_mask(qts, LP2HP_PERI_PMS + lp2hp_reg[k], hp_peri_bits[k]);
    }
    check_mask(qts, LP2HP_PERI_PMS + 0x10, 0);

    check_mask(qts, DMA_PMS, 0xffffffff);
    check_mask(qts, DMA_LOW(3), 0xfffff000);
    check_mask(qts, DMA_HIGH(31), 0xfffff000);
    check_mask(qts, DMA_RMT_R, 0xffffffff);
    check_mask(qts, DMA_RMT_W, 0xffffffff);
    check_mask(qts, DMA_PMS + 0x15c, 0);
    check_mask(qts, DMA_PMS + 0x1f4, 0);
    check_mask(qts, DMA_PMS + 0x220, 0xffffffff);

    check_mask(qts, LP_PERI_LP, 0x1ffffff);
    check_mask(qts, REGION_LOW(0), 0xfffffffc);
    check_mask(qts, REGION_HIGH(7), 0xfffffffc);
    check_mask(qts, REGION_PMS, 0x3ff);
    check_mask(qts, REGION_2_TO_7_PMS, 0x3fffffff);
    check_mask(qts, HP2LP(1, 1), 0x1ffffff);
    check_mask(qts, REGDMA_LP_PERI, 3);

    /* A byte write changes its byte alone. */
    qtest_writel(qts, HP_PERI(0, 1, 1), hp_peri_bits[1]);
    qtest_writeb(qts, HP_PERI(0, 1, 1) + 1, 0);
    g_assert_cmphex(qtest_readl(qts, HP_PERI(0, 1, 1)), ==, 0x3fff00ff);

    /* The ICM's: the records are read-only, INT_RAW and INT_CLR clear. */
    check_mask(qts, ICM_SLV_ARB, 0x00fff038);
    check_mask(qts, ICM_DLOCK_TIMEOUT, 0x1fff);
    check_mask(qts, ICM_RDN_ECO_CS, 1);
    check_mask(qts, ICM_INT_ENA, 7);
    qtest_writel(qts, ICM_CPU_ADDR, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, 0);
    qtest_writel(qts, LP_SYS_ADDRHOLE_ADDR, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_ADDR), ==, 0);
    check_mask(qts, LP_SYS_INT_ENA, 0x7f);
    qtest_quit(qts);
}

/* The HP APM */

/*
 * An HP core's user-mode access to a peripheral its register closes is
 * refused, and recorded with the core, the address and the direction; the
 * same access in machine mode, or by the other core, passes. A refused
 * write does not reach the register.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_hp_peripheral(void)
{
    QTestState *qts = start();
    uint32_t v;

    g_assert_true(can_read(qts, CORE0, true, UART0));

    /* UART0 is bit 10 of register 2. */
    qtest_writel(qts, HP_PERI(0, 1, 2), hp_peri_bits[2] & ~(1u << 10));
    g_assert_false(source_level(qts, SRC_SYS_ICM));
    g_assert_cmpuint(probe(qts, CORE0, true, false, UART0 + 0x7c, 0, &v),
                     !=, MEMTX_OK);
    g_assert_cmphex(v, ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_ST), ==, ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, UART0 + 0x7c);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, 0);
    g_assert_true(source_level(qts, SRC_SYS_ICM));

    g_assert_true(can_read(qts, CORE0, false, UART0));
    g_assert_true(can_read(qts, CORE1, true, UART0));
    g_assert_true(can_read(qts, LP_CORE, false, UART0));
    g_assert_true(can_read(qts, CORE0, true, UART1));

    /* The record holds the first refusal until software clears it. */
    g_assert_false(can_write(qts, CORE0, true, UART0 + 4, 1));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, UART0 + 0x7c);
    qtest_writel(qts, ICM_INT_CLR, ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);
    g_assert_false(source_level(qts, SRC_SYS_ICM));
    g_assert_false(can_write(qts, CORE0, true, UART0 + 4, 1));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, UART0 + 4);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR);
    /* INT_RAW clears on a write of 1 as well. */
    qtest_writel(qts, ICM_INT_RAW, ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);

    /* A refused write leaves its register as it was. */
    qtest_writel(qts, HP_PERI(1, 1, 3), hp_peri_bits[3] & ~(1u << 3));
    g_assert_false(can_write(qts, CORE1, true, HP_SYS_FROM_CPU0, 1));
    g_assert_cmphex(qtest_readl(qts, HP_SYS_FROM_CPU0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | 1);
    g_assert_true(can_write(qts, CORE1, false, HP_SYS_FROM_CPU0, 1));
    g_assert_cmphex(qtest_readl(qts, HP_SYS_FROM_CPU0), ==, 1);
    qtest_writel(qts, HP_SYS_FROM_CPU0, 0);
    clear_records(qts);

    /* The APMs' own registers are slots too: HP_PERI_PMS is bit 25. */
    qtest_writel(qts, HP_PERI(0, 0, 1), hp_peri_bits[1] & ~(1u << 25));
    g_assert_false(can_write(qts, CORE0, false, HP_PERI(0, 0, 1),
                             hp_peri_bits[1]));
    g_assert_true(can_read(qts, CORE0, false, LP2HP_PERI_PMS));
    g_assert_true(can_write(qts, CORE1, false, HP_PERI(0, 0, 1),
                            hp_peri_bits[1]));
    g_assert_true(can_read(qts, CORE0, false, HP_PERI_PMS));

    /* With its interrupt disabled the ICM still records. */
    clear_records(qts);
    qtest_writel(qts, ICM_INT_ENA, 0);
    qtest_writel(qts, HP_PERI(0, 0, 2), 0);
    g_assert_false(can_read(qts, CORE0, false, PCNT));
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_ST), ==, 0);
    g_assert_false(source_level(qts, SRC_SYS_ICM));
    qtest_writel(qts, ICM_INT_ENA, ICM_INT_CPU);
    g_assert_true(source_level(qts, SRC_SYS_ICM));
    clear_records(qts);

    /* An HP CPU peripheral, trace encoder 0, bit 6 of register 0. */
    qtest_writel(qts, HP_PERI(0, 1, 0), hp_peri_bits[0] & ~(1u << 6));
    g_assert_false(can_read(qts, CORE0, true, TRACE0));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, TRACE0);
    g_assert_true(can_read(qts, CORE0, true, TRACE0 + 0x1000));
    clear_records(qts);

    /* The LP core, against LP2HP_PERI_PMS. */
    g_assert_true(can_read(qts, LP_CORE, false, UART1));
    qtest_writel(qts, LP2HP_PERI_PMS + lp2hp_reg[2],
                 hp_peri_bits[2] & ~(1u << 11));
    qtest_writel(qts, LP2HP_PERI_PMS + lp2hp_reg[0],
                 hp_peri_bits[0] & ~(1u << 6));
    g_assert_false(can_read(qts, LP_CORE, false, UART1));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, 2);
    g_assert_false(can_read(qts, LP_CORE, false, TRACE0));
    g_assert_true(can_read(qts, CORE1, false, UART1));
    qtest_quit(qts);
}

/*
 * HP ROM and HP L2MEM without the cache: an HP core's access is the HP
 * APM's, its access through the cache only its PMP's. The LP core uses
 * the cached addresses, which are its HP APM's to check.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_hp_memory(void)
{
    QTestState *qts = start();
    uint32_t off = SCRATCH - L2MEM;
    uint32_t v;

    qtest_writel(qts, SCRATCH, 0x11111111);
    /* L2MEM is bit 2 and HP ROM bit 3 of register 0. */
    qtest_writel(qts, HP_PERI(0, 1, 0), hp_peri_bits[0] & ~0xcu);
    g_assert_false(can_write(qts, CORE0, true, L2MEM_DIRECT + off,
                             0x22222222));
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, L2MEM_DIRECT + off);
    g_assert_false(can_read(qts, CORE0, true, ROM_DIRECT));
    g_assert_true(can_write(qts, CORE0, true, SCRATCH, 0x33333333));
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x33333333);
    g_assert_true(can_write(qts, CORE0, false, L2MEM_DIRECT + off,
                            0x44444444));
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x44444444);
    g_assert_cmpuint(probe(qts, CORE1, true, false, L2MEM_DIRECT + off, 0,
                           &v), ==, MEMTX_OK);
    g_assert_cmphex(v, ==, 0x44444444);
    /* The TCM is no APM's. */
    g_assert_true(can_write(qts, CORE0, true, TCM, 1));
    clear_records(qts);

    /* The LP core: its L2MEM bit in LP2HP_PERI_PMS's register 0. */
    g_assert_true(can_write(qts, LP_CORE, false, SCRATCH, 0x55555555));
    qtest_writel(qts, LP2HP_PERI_PMS + lp2hp_reg[0],
                 hp_peri_bits[0] & ~0x4u);
    g_assert_false(can_write(qts, LP_CORE, false, SCRATCH, 0x66666666));
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x55555555);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | 2);
    g_assert_true(can_write(qts, CORE0, false, SCRATCH, 0x77777777));
    g_assert_true(can_write(qts, LP_CORE, false, L2MEM_DIRECT + off, 0));
    qtest_quit(qts);
}

/*
 * The configurable regions, at 4-byte granularity with an exclusive end,
 * come before the fixed slots: a region whose bit is clear refuses its
 * master, whatever the slot allows, and an open region opens no slot.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_regions(void)
{
    QTestState *qts = start();

    /* Every region empty but region 1, [UART0 + 0x10, UART0 + 0x20). */
    for (int n = 0; n < 8; n++) {
        qtest_writel(qts, REGION_LOW(n), 0);
        qtest_writel(qts, REGION_HIGH(n), 0);
    }
    qtest_writel(qts, REGION_LOW(1), UART0 + 0x10);
    qtest_writel(qts, REGION_HIGH(1), UART0 + 0x20);
    qtest_writel(qts, REGION_PMS, 0x3ff & ~REGION_C0M(1));

    g_assert_true(can_read(qts, CORE0, false, UART0 + 0x0c));
    g_assert_false(can_read(qts, CORE0, false, UART0 + 0x10));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, UART0 + 0x10);
    g_assert_false(can_read(qts, CORE0, false, UART0 + 0x1c));
    g_assert_true(can_read(qts, CORE0, false, UART0 + 0x20));
    g_assert_true(can_read(qts, CORE0, true, UART0 + 0x10));
    g_assert_true(can_read(qts, CORE1, false, UART0 + 0x10));
    g_assert_true(can_read(qts, LP_CORE, false, UART0 + 0x10));
    clear_records(qts);

    /* An open region does not open the slot under it. */
    qtest_writel(qts, REGION_PMS, 0x3ff);
    qtest_writel(qts, HP_PERI(0, 0, 2), hp_peri_bits[2] & ~(1u << 10));
    g_assert_false(can_read(qts, CORE0, false, UART0 + 0x10));
    qtest_writel(qts, HP_PERI(0, 0, 2), hp_peri_bits[2]);
    clear_records(qts);

    /*
     * Region 5, over an LP peripheral, closed to core 1's user mode: the
     * LP APM refuses, and LP_SYS records.
     */
    qtest_writel(qts, REGION_LOW(5), LPPERI);
    qtest_writel(qts, REGION_HIGH(5), LPPERI + 0x1000);
    qtest_writel(qts, REGION_2_TO_7_PMS, 0x3fffffff & ~REGION27_C1U(5));
    g_assert_false(can_read(qts, CORE1, true, LPPERI + 0x3fc));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_RAW), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_ADDR), ==,
                    LPPERI + 0x3fc);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);
    g_assert_true(can_read(qts, CORE1, false, LPPERI + 0x3fc));
    g_assert_true(can_read(qts, CORE1, true, LPPERI + 0x1000));

    /* Regions apply to peripheral registers only, not to memory. */
    qtest_writel(qts, REGION_LOW(5), L2MEM_DIRECT);
    qtest_writel(qts, REGION_HIGH(5), L2MEM_DIRECT + 0x10000);
    g_assert_true(can_read(qts, CORE1, true, L2MEM_DIRECT + 0x100));
    qtest_quit(qts);
}

/* The LP APM */

/*
 * An HP core's access to an LP peripheral or LP SRAM its HP2LP_PERI_PMS
 * register closes is refused, and recorded in LP_SYS, which raises
 * LP_SYSREG_INTR as its INT_ENA lets it; the LP core's are checked
 * against LP_PERI_PMS.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_lp_apm(void)
{
    QTestState *qts = start();

    /* LPPERI is bit 8. */
    qtest_writel(qts, HP2LP(0, 0), 0x1ffffff & ~(1u << 8));
    g_assert_false(can_read(qts, CORE0, false, LPPERI + 0x3fc));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_ADDR), ==,
                    LPPERI + 0x3fc);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_RAW), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_ST), ==, 0);
    g_assert_false(source_level(qts, SRC_LP_SYSREG));
    qtest_writel(qts, LP_SYS_INT_ENA, 1);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_ST), ==, 1);
    g_assert_true(source_level(qts, SRC_LP_SYSREG));
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);
    g_assert_true(can_read(qts, CORE0, true, LPPERI + 0x3fc));
    g_assert_true(can_read(qts, CORE1, false, LPPERI + 0x3fc));
    qtest_writel(qts, LP_SYS_INT_CLR, 1);
    g_assert_false(source_level(qts, SRC_LP_SYSREG));

    /* A refused write leaves LP_SYS's boot address as it was. */
    qtest_writel(qts, HP2LP(0, 0), 0x1ffffff & ~1u);
    g_assert_false(can_write(qts, CORE0, false, LP_SYS + 0x164, 0x1234));
    g_assert_cmphex(qtest_readl(qts, LP_SYS + 0x164), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==, LP_SYS_WR);
    qtest_writel(qts, LP_SYS_INT_CLR, 1);
    qtest_writel(qts, HP2LP(0, 0), 0x1ffffff);

    /* LP SRAM, bit 23, for HP core 1 in user mode. */
    qtest_writel(qts, HP2LP(1, 1), 0x1ffffff & ~(1u << 23));
    g_assert_false(can_write(qts, CORE1, true, LP_SRAM + 0x100, 5));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==,
                    LP_SYS_WR | 1);
    g_assert_cmphex(qtest_readl(qts, LP_SRAM + 0x100), ==, 0);
    g_assert_true(can_write(qts, CORE1, false, LP_SRAM + 0x100, 5));
    g_assert_cmphex(qtest_readl(qts, LP_SRAM + 0x100), ==, 5);
    qtest_writel(qts, LP_SYS_INT_CLR, 1);

    /* The LP core: the eFuse controller, bit 18 of LP_PERI_PMS. */
    qtest_writel(qts, LP_PERI_LP, 0x1ffffff & ~(1u << 18));
    g_assert_false(can_read(qts, LP_CORE, false, 0x5012d1fc));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==, 2);
    g_assert_true(can_read(qts, CORE0, false, 0x5012d1fc));
    /* LP SRAM is not the LP APM's for the LP core. */
    qtest_writel(qts, LP_PERI_LP, 0);
    g_assert_true(can_read(qts, LP_CORE, false, LP_SRAM));
    qtest_quit(qts);
}

/*
 * The LP APM's registers are behind LPPERI's LP_PMS clock and reset:
 * gated, they take no writes; in reset, they hold their reset values,
 * and so do the decisions they make.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_lp_clock_reset(void)
{
    QTestState *qts = start();

    qtest_writel(qts, HP2LP(0, 0), 0);
    g_assert_false(can_read(qts, CORE0, false, LPPERI + 0x3fc));

    qtest_writel(qts, LPPERI_CLK_EN,
                 qtest_readl(qts, LPPERI_CLK_EN) & ~CK_EN_LP_PMS);
    qtest_writel(qts, HP2LP(0, 1), 0);
    g_assert_cmphex(qtest_readl(qts, HP2LP(0, 1)), ==, 0x1ffffff);
    qtest_writel(qts, LPPERI_CLK_EN,
                 qtest_readl(qts, LPPERI_CLK_EN) | CK_EN_LP_PMS);

    qtest_writel(qts, LPPERI_RESET_EN,
                 qtest_readl(qts, LPPERI_RESET_EN) | RST_EN_LP_PMS);
    check_lp_reset(qts);
    g_assert_true(can_read(qts, CORE0, false, LPPERI + 0x3fc));
    qtest_writel(qts, HP2LP(0, 0), 0);
    g_assert_cmphex(qtest_readl(qts, HP2LP(0, 0)), ==, 0x1ffffff);
    qtest_writel(qts, LPPERI_RESET_EN,
                 qtest_readl(qts, LPPERI_RESET_EN) & ~RST_EN_LP_PMS);
    qtest_writel(qts, HP2LP(0, 0), 0);
    g_assert_cmphex(qtest_readl(qts, HP2LP(0, 0)), ==, 0);

    /* The HP APM's registers have no such reset. */
    qtest_writel(qts, HP_PERI(0, 0, 3), 0);
    qtest_writel(qts, LPPERI_RESET_EN,
                 qtest_readl(qts, LPPERI_RESET_EN) | RST_EN_LP_PMS);
    g_assert_cmphex(qtest_readl(qts, HP_PERI(0, 0, 3)), ==, 0);
    qtest_quit(qts);
}

/* The DMA APM */

/*
 * A DMA master's reads and writes of the memories are checked against the
 * regions its registers open: an address in no region is refused; an AXI
 * master's refusal is recorded in the ICM's system record with its 8-bit
 * ID, an AHB master's in the CPU record. Other addresses are not the DMA
 * APM's.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_dma(void)
{
    QTestState *qts = start();
    uint32_t ch0 = RID_DMA(DMA_GDMA_CH0, 0x51);
    uint32_t gmac = RID_DMA(DMA_GMAC, 5);
    uint32_t v;

    qtest_writel(qts, SCRATCH, 0xabcdef01);
    g_assert_true(can_write(qts, ch0, false, SCRATCH + 4, 1));

    /* Region 3 alone, [SCRATCH, SCRATCH + 4 KiB), the low bits ignored. */
    for (int n = 0; n < 32; n++) {
        qtest_writel(qts, DMA_LOW(n), 0);
        qtest_writel(qts, DMA_HIGH(n), 0);
    }
    qtest_writel(qts, DMA_LOW(3), SCRATCH + 0x123);
    qtest_writel(qts, DMA_HIGH(3), SCRATCH + 0x1fff);
    g_assert_cmphex(qtest_readl(qts, DMA_LOW(3)), ==, SCRATCH);
    g_assert_cmphex(qtest_readl(qts, DMA_HIGH(3)), ==, SCRATCH + 0x1000);
    /* GDMA channel 0 may read region 3, not write it. */
    qtest_writel(qts, DMA_GDMA_CH0_R, 1u << 3);
    qtest_writel(qts, DMA_GDMA_CH0_W, ~(1u << 3));

    g_assert_cmpuint(probe(qts, ch0, false, false, SCRATCH, 0, &v), ==,
                     MEMTX_OK);
    g_assert_cmphex(v, ==, 0xabcdef01);
    g_assert_false(can_write(qts, ch0, false, SCRATCH + 0xffc, 2));
    g_assert_cmphex(qtest_readl(qts, SCRATCH + 0xffc), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, ICM_INT_SYS);
    g_assert_cmphex(qtest_readl(qts, ICM_SYS_ADDR), ==, SCRATCH + 0xffc);
    g_assert_cmphex(qtest_readl(qts, ICM_SYS_INFO), ==, ICM_WR | 0x51);
    g_assert_true(source_level(qts, SRC_SYS_ICM));
    /* Past the region's exclusive end, and below it: in no region. */
    g_assert_false(can_read(qts, ch0, false, SCRATCH + 0x1000));
    g_assert_false(can_read(qts, ch0, false, SCRATCH - 4));
    /* Not the DMA APM's: the TCM, a peripheral, the direct alias. */
    g_assert_true(can_write(qts, ch0, false, TCM, 3));
    g_assert_true(can_read(qts, ch0, false, UART0));
    g_assert_true(can_read(qts, ch0, false, L2MEM_DIRECT));
    /* Nor are the CPUs. */
    g_assert_true(can_write(qts, CORE0, true, SCRATCH + 0xffc, 4));
    clear_records(qts);

    /* GMAC, on the AHB matrix, keeps its reset permissions: none here. */
    g_assert_false(can_write(qts, gmac, false, SCRATCH + 0x2000, 5));
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, SCRATCH + 0x2000);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | 5);
    qtest_writel(qts, DMA_LOW(9), L2MEM);
    qtest_writel(qts, DMA_HIGH(9), L2MEM + 0xc0000);
    g_assert_true(can_write(qts, gmac, false, SCRATCH + 0x2000, 5));
    /* Channel 0 may write region 9, but read only region 3. */
    g_assert_true(can_write(qts, ch0, false, SCRATCH + 0x2000, 6));
    g_assert_false(can_read(qts, ch0, false, SCRATCH + 0x2000));
    qtest_writel(qts, DMA_GMAC_W, 0);
    g_assert_false(can_write(qts, gmac, false, SCRATCH + 0x2000, 7));
    g_assert_true(can_read(qts, gmac, false, SCRATCH + 0x2000));
    g_assert_cmphex(qtest_readl(qts, SCRATCH + 0x2000), ==, 6);
    clear_records(qts);

    /* GDMA-AHB's RMT channel, whose write register stands apart. */
    qtest_writel(qts, DMA_RMT_W, 0);
    g_assert_false(can_write(qts, RID_DMA(DMA_RMT, 26), false, SCRATCH, 0));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | 26);
    g_assert_true(can_read(qts, RID_DMA(DMA_RMT, 26), false, SCRATCH));
    qtest_quit(qts);
}

/*
 * REGDMA reaches the peripherals and LP SRAM through the HP and LP APMs,
 * as REGDMA_HP_PERI_PMS and REGDMA_LP_PERI_PMS allow.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_regdma(void)
{
    QTestState *qts = start();
    uint32_t regdma = RID_DMA(DMA_REGDMA, 4);

    g_assert_true(can_read(qts, regdma, false, UART0));
    g_assert_true(can_read(qts, regdma, false, LPPERI + 0x3fc));
    g_assert_true(can_read(qts, regdma, false, LP_SRAM));

    qtest_writel(qts, REGDMA_HP_PERI, 0);
    g_assert_false(can_read(qts, regdma, false, UART0));
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, 4);
    g_assert_true(can_read(qts, regdma, false, LPPERI + 0x3fc));

    qtest_writel(qts, REGDMA_LP_PERI, 1);
    g_assert_false(can_read(qts, regdma, false, LPPERI + 0x3fc));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==, 4);
    g_assert_true(can_read(qts, regdma, false, LP_SRAM));
    qtest_writel(qts, LP_SYS_INT_CLR, 1);
    qtest_writel(qts, REGDMA_LP_PERI, 2);
    g_assert_false(can_write(qts, regdma, false, LP_SRAM, 1));
    g_assert_cmphex(qtest_readl(qts, LP_SYS_ADDRHOLE_INFO), ==,
                    LP_SYS_WR | 4);

    /* Its memory accesses are the DMA APM's, against its own registers. */
    qtest_writel(qts, DMA_REGDMA_R, 0);
    g_assert_false(can_read(qts, regdma, false, SCRATCH));
    g_assert_true(can_write(qts, regdma, false, SCRATCH, 1));
    qtest_quit(qts);
}

/* Accesses with no master, like qtest's own, pass every APM. */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_unchecked(void)
{
    QTestState *qts = start();

    for (int c = 0; c < 2; c++) {
        for (int m = 0; m < 2; m++) {
            for (int k = 0; k < 4; k++) {
                qtest_writel(qts, HP_PERI(c, m, k), 0);
            }
            qtest_writel(qts, HP2LP(c, m), 0);
        }
    }
    qtest_writel(qts, REGION_PMS, 0);
    qtest_writel(qts, REGION_2_TO_7_PMS, 0);
    g_assert_cmphex(qtest_readl(qts, HP_PERI(0, 0, 1)), ==, 0);
    qtest_writel(qts, SCRATCH, 9);
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 9);
    g_assert_true(can_read(qts, 0, false, UART0));
    g_assert_true(can_read(qts, 0x1234, true, LPPERI));
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_RAW), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4-pms/reset", test_reset);
    qtest_add_func("/esp32p4-pms/masks", test_masks);
    qtest_add_func("/esp32p4-pms/hp-peripheral", test_hp_peripheral);
    qtest_add_func("/esp32p4-pms/hp-memory", test_hp_memory);
    qtest_add_func("/esp32p4-pms/regions", test_regions);
    qtest_add_func("/esp32p4-pms/lp-apm", test_lp_apm);
    qtest_add_func("/esp32p4-pms/lp-clock-reset", test_lp_clock_reset);
    qtest_add_func("/esp32p4-pms/dma", test_dma);
    qtest_add_func("/esp32p4-pms/regdma", test_regdma);
    qtest_add_func("/esp32p4-pms/unchecked", test_unchecked);

    return g_test_run();
}
