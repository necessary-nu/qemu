/*
 * QTest testcase for the ESP32-P4 bus matrix: which slaves each master
 * has a path to
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Every master reaches only the slaves its port on the bus matrix decodes
 * (TRM chapter 1, 9.3.1, 9.3.4, table 20.1-1): a DMA master HP L2MEM, HP
 * ROM and the external memory windows at their 0x4... addresses, VDMA's
 * master 1 also the CSI and DSI bridges' buffers, REGDMA also the HP and
 * LP peripherals and LP SRAM; the LP core neither the CPU subsystem, the
 * TCM, the bridges' buffers nor any 0x8... address; the HP cores neither
 * the bridges' buffers nor the LP ROM. An access outside a master's reach
 * answers with a decode error, reads nothing and writes nothing; a DMA
 * master's is recorded by the AXI ICM as an illegal access (SECURE), in
 * the record of the master's matrix. An access as a given master goes
 * through the PMS device's "probe" hook (see esp32p4-pms-test.c), and
 * GDMA-AHB and GDMA-AXI run memory-to-memory transfers into such
 * addresses while virtual time steps.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define DMA_PMS             0x500a6000
#define DMA_LOW(n)          (DMA_PMS + 0x08 + 8 * (n))
#define DMA_HIGH(n)         (DMA_PMS + 0x0c + 8 * (n))
#define DMA_GMAC_W          (DMA_PMS + 0x1b8)

#define ICM                 0x500a4000
#define ICM_INT_RAW         (ICM + 0x0c)
#define ICM_INT_CLR         (ICM + 0x18)
#define ICM_SYS_ADDR        (ICM + 0x38)
#define ICM_SYS_INFO        (ICM + 0x3c)
#define ICM_CPU_ADDR        (ICM + 0x40)
#define ICM_CPU_INFO        (ICM + 0x44)
#define ICM_INT_SYS         0x2
#define ICM_INT_CPU         0x4
#define ICM_WR              0x100
#define ICM_SECURE          0x200

#define LP_SYS_INT_RAW      0x50110170
#define LP_SYS_INT_CLR      0x5011017c
/* LP_SYSTEM_LP_STORE15, a scratch register among the LP peripherals. */
#define LP_STORE15          0x50110068
/* TIMG0_T0ALARMLO, a plain register among the HP peripherals. */
#define TIMG0_ALARMLO       0x500c2010

#define INTMTX_STATUS0      (0x500d6000 + 0x200)
#define SRC_SYS_ICM         21

/* The requester IDs of include/hw/misc/esp32p4_pms.h. */
#define RID_CPU(id)         (0x8000 | (id))
#define RID_DMA(m, id)      (0xc000 | ((m) << 8) | (id))
#define CORE0               RID_CPU(0)
#define CORE1               RID_CPU(1)
#define LP_CORE             RID_CPU(2)
#define AXI_ID(cid, uid)    (((cid) << 4) | (uid))
#define REGDMA              RID_DMA(28, 4)
#define VDMA_M1(ch)         RID_DMA(ch, AXI_ID(5, ch))
#define VDMA_M2(ch)         RID_DMA(ch, AXI_ID(6, ch))

#define CLINT               0x20000000
#define CLIC                0x20800000
#define TCM                 0x30100000
#define TRACE0              0x3ff04000
#define FLASH               0x40000000
#define PSRAM               0x48000000
#define HP_ROM              0x4fc00000
#define L2MEM               0x4ff00000
#define UART0               0x500ca000
#define LP_ROM              0x50100000
#define CSI_MEM             0x50104000
#define DSI_MEM             0x50105000
#define LP_SRAM             0x50108000
#define LP_PERI             0x50110000
#define FLASH_DIRECT        0x80000000
#define PSRAM_DIRECT        0x88000000
#define ROM_DIRECT          0x8fc00000
#define L2MEM_DIRECT        0x8ff00000

/* L2MEM's words for the tests, 256 KiB in, past the L2 cache's 128 KiB. */
#define SCRATCH             0x4ff40000
#define DIRECT(a)           ((a) + 0x40000000u)
/* LP SRAM's last word, which nothing here runs from. */
#define LP_SRAM_WORD        (LP_SRAM + 0x7ffc)

#define MEMTX_OK            0
#define MEMTX_ERROR         1
#define MEMTX_DECODE_ERROR  2

static QTestState *start(void)
{
    return qtest_init("-M esp32p4,flash-size=16M,strap-mode=0");
}

/*
 * Make a 32-bit access at @addr as the master @rid through the "probe"
 * hook: its MemTxResult, and in *@data what a read returned.
 */
static unsigned probe(QTestState *qts, uint32_t rid, bool write,
                      uint32_t addr, uint32_t value, uint32_t *data)
{
    g_autofree char *cmd = g_strdup_printf("%u 0 %c 0x%x 0x%x", rid,
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

static unsigned rd(QTestState *qts, uint32_t rid, uint32_t addr)
{
    return probe(qts, rid, false, addr, 0, NULL);
}

static unsigned wr(QTestState *qts, uint32_t rid, uint32_t addr,
                   uint32_t value)
{
    return probe(qts, rid, true, addr, value, NULL);
}

static void clear_records(QTestState *qts)
{
    qtest_writel(qts, ICM_INT_CLR, 0x7);
    qtest_writel(qts, LP_SYS_INT_CLR, 0x7f);
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS0) >> src) & 1;
}

/* No record of any refusal: the ICM's and LP_SYS's raw bits clear. */
static void assert_no_record(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_SYS_INT_RAW) & 1, ==, 0);
}

/*
 * The ICM's record of an illegal access by the DMA master with ID @id, on
 * the AXI matrix if @axi, at @addr, a write if @write.
 */
static void assert_illegal(QTestState *qts, bool axi, uint32_t id,
                           uint32_t addr, bool write)
{
    uint32_t info = ICM_SECURE | (write ? ICM_WR : 0) | id;

    g_assert_cmphex(qtest_readl(qts, ICM_INT_RAW), ==,
                    axi ? ICM_INT_SYS : ICM_INT_CPU);
    g_assert_cmphex(qtest_readl(qts, axi ? ICM_SYS_ADDR : ICM_CPU_ADDR), ==,
                    addr);
    g_assert_cmphex(qtest_readl(qts, axi ? ICM_SYS_INFO : ICM_CPU_INFO), ==,
                    info);
    g_assert_true(source_level(qts, SRC_SYS_ICM));
    clear_records(qts);
}

/* The DMA masters, by their DMA APM number and the ID the ICM records. */
static const struct {
    const char *name;
    uint32_t master;
    uint32_t id;
    bool axi;
} dma_masters[] = {
    { "GDMA-AHB I2S0",          5, 16 + 3, false },
    { "GDMA-AHB m2m",           33, 16 + 11, false },
    { "USB OTG_HS",             20, 7, false },
    { "EMAC",                   21, 5, false },
    { "SD/MMC",                 22, 6, false },
    { "USB OTG_FS",             23, 3, false },
    { "trace 0",                24, 8, false },
    { "L2MEM monitor",          26, 11, false },
    { "GDMA-AXI LCD_CAM",       13, AXI_ID(8, 0), true },
    { "GDMA-AXI m2m",           34, AXI_ID(8, 6), true },
    { "2D-DMA JPEG",            19, AXI_ID(10, 0), true },
    { "H264",                   30, AXI_ID(11, 0), true },
    { "VDMA master 2",          1, AXI_ID(6, 1), true },
};

/*
 * The addresses no DMA master but REGDMA and VDMA's master 1 has a path
 * to, each at a word whose content the test knows.
 */
static const uint32_t dma_no_path[] = {
    DIRECT(SCRATCH), ROM_DIRECT, FLASH_DIRECT, PSRAM_DIRECT, L2MEM_DIRECT,
    LP_SRAM_WORD, TCM, CLINT, CLIC, TRACE0, UART0, TIMG0_ALARMLO,
    LP_STORE15, LP_ROM, CSI_MEM, DSI_MEM, 0x60000000,
};

/*
 * Each DMA master reaches HP L2MEM and HP ROM at their 0x4... addresses,
 * through the DMA APM, and nothing at their 0x8... aliases, in LP SRAM,
 * the TCM or any register: a decode error, nothing written, the access
 * recorded as illegal in its matrix's record.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_dma_reach(void)
{
    QTestState *qts = start();
    uint32_t rom0 = qtest_readl(qts, HP_ROM);

    for (int i = 0; i < ARRAY_SIZE(dma_masters); i++) {
        uint32_t rid = RID_DMA(dma_masters[i].master, dma_masters[i].id);
        uint32_t v;

        g_test_message("%s", dma_masters[i].name);
        qtest_writel(qts, SCRATCH, 0x11111111);
        qtest_writel(qts, LP_SRAM_WORD, 0x22222222);
        qtest_writel(qts, LP_STORE15, 0x33333333);
        qtest_writel(qts, TIMG0_ALARMLO, 0x44444444);
        qtest_writel(qts, TCM, 0x55555555);

        g_assert_cmpuint(wr(qts, rid, SCRATCH, 0x66666666), ==, MEMTX_OK);
        g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x66666666);
        g_assert_cmpuint(probe(qts, rid, false, HP_ROM, 0, &v), ==,
                         MEMTX_OK);
        g_assert_cmphex(v, ==, rom0);
        assert_no_record(qts);

        for (int j = 0; j < ARRAY_SIZE(dma_no_path); j++) {
            uint32_t a = dma_no_path[j];

            g_assert_cmpuint(probe(qts, rid, false, a, 0, &v), ==,
                             MEMTX_DECODE_ERROR);
            g_assert_cmphex(v, ==, 0);
            assert_illegal(qts, dma_masters[i].axi, dma_masters[i].id, a,
                           false);
        }
        /* Writes to the words that hold something: none lands. */
        g_assert_cmpuint(wr(qts, rid, DIRECT(SCRATCH), 0x77777777), ==,
                         MEMTX_DECODE_ERROR);
        assert_illegal(qts, dma_masters[i].axi, dma_masters[i].id,
                       DIRECT(SCRATCH), true);
        g_assert_cmpuint(wr(qts, rid, LP_SRAM_WORD, 0x77777777), ==,
                         MEMTX_DECODE_ERROR);
        clear_records(qts);
        g_assert_cmpuint(wr(qts, rid, LP_STORE15, 0x77777777), ==,
                         MEMTX_DECODE_ERROR);
        clear_records(qts);
        g_assert_cmpuint(wr(qts, rid, TIMG0_ALARMLO, 0x77777777), ==,
                         MEMTX_DECODE_ERROR);
        clear_records(qts);
        g_assert_cmpuint(wr(qts, rid, TCM, 0x77777777), ==,
                         MEMTX_DECODE_ERROR);
        clear_records(qts);
        g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x66666666);
        g_assert_cmphex(qtest_readl(qts, LP_SRAM_WORD), ==, 0x22222222);
        g_assert_cmphex(qtest_readl(qts, LP_STORE15), ==, 0x33333333);
        g_assert_cmphex(qtest_readl(qts, TIMG0_ALARMLO), ==, 0x44444444);
        g_assert_cmphex(qtest_readl(qts, TCM), ==, 0x55555555);
    }
    qtest_quit(qts);
}

/*
 * The DMA APM's refusal and the bus matrix's differ: an address in reach
 * that no region permits is an unauthorized access, refused with a slave
 * error and recorded without SECURE; the 0x8... alias of an address the
 * APM permits stays out of reach, whatever the regions cover.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_apm_and_reach(void)
{
    QTestState *qts = start();
    uint32_t gmac = RID_DMA(21, 5);

    /* Region 0 alone, over the whole space, writable by GMAC. */
    for (int n = 1; n < 32; n++) {
        qtest_writel(qts, DMA_LOW(n), 0);
        qtest_writel(qts, DMA_HIGH(n), 0);
    }
    qtest_writel(qts, DMA_GMAC_W, 1);
    g_assert_cmpuint(wr(qts, gmac, SCRATCH, 1), ==, MEMTX_OK);
    g_assert_cmpuint(wr(qts, gmac, DIRECT(SCRATCH), 2), ==,
                     MEMTX_DECODE_ERROR);
    assert_illegal(qts, false, 5, DIRECT(SCRATCH), true);
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 1);

    qtest_writel(qts, DMA_GMAC_W, 0);
    g_assert_cmpuint(wr(qts, gmac, SCRATCH, 3), ==, MEMTX_ERROR);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_ADDR), ==, SCRATCH);
    g_assert_cmphex(qtest_readl(qts, ICM_CPU_INFO), ==, ICM_WR | 5);
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 1);
    qtest_quit(qts);
}

/*
 * VDMA's master 1 reaches the CSI and DSI bridges' buffers as well as the
 * memories; master 2 the memories only. Neither reaches 0x8....
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_vdma(void)
{
    QTestState *qts = start();

    for (int ch = 0; ch < 4; ch++) {
        g_assert_cmpuint(wr(qts, VDMA_M1(ch), SCRATCH, ch), ==, MEMTX_OK);
        g_assert_cmpuint(wr(qts, VDMA_M2(ch), SCRATCH, ch), ==, MEMTX_OK);
        g_assert_cmpuint(rd(qts, VDMA_M1(ch), CSI_MEM), !=,
                         MEMTX_DECODE_ERROR);
        g_assert_cmpuint(rd(qts, VDMA_M1(ch), DSI_MEM + 0xffc), !=,
                         MEMTX_DECODE_ERROR);
        assert_no_record(qts);
        g_assert_cmpuint(rd(qts, VDMA_M2(ch), CSI_MEM), ==,
                         MEMTX_DECODE_ERROR);
        assert_illegal(qts, true, AXI_ID(6, ch), CSI_MEM, false);
        g_assert_cmpuint(wr(qts, VDMA_M2(ch), DSI_MEM, 0), ==,
                         MEMTX_DECODE_ERROR);
        assert_illegal(qts, true, AXI_ID(6, ch), DSI_MEM, true);
        g_assert_cmpuint(wr(qts, VDMA_M1(ch), DIRECT(SCRATCH), 0), ==,
                         MEMTX_DECODE_ERROR);
        assert_illegal(qts, true, AXI_ID(5, ch), DIRECT(SCRATCH), true);
        g_assert_cmpuint(rd(qts, VDMA_M1(ch), UART0), ==,
                         MEMTX_DECODE_ERROR);
        assert_illegal(qts, true, AXI_ID(5, ch), UART0, false);
        /* The bridges' neighbours are no more VDMA's than anyone's. */
        g_assert_cmpuint(rd(qts, VDMA_M1(ch), DSI_MEM + 0x1000), ==,
                         MEMTX_DECODE_ERROR);
        clear_records(qts);
    }
    qtest_quit(qts);
}

/*
 * REGDMA reaches the memories and, through its HP and LP APM permissions,
 * the HP and LP peripherals and LP SRAM; not the 0x8... aliases, the TCM,
 * the HP CPU peripherals or the CPU subsystem.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_regdma(void)
{
    QTestState *qts = start();
    static const uint32_t no_path[] = {
        DIRECT(SCRATCH), ROM_DIRECT, PSRAM_DIRECT, TCM, TRACE0, CLINT,
        CSI_MEM, LP_ROM,
    };

    g_assert_cmpuint(wr(qts, REGDMA, SCRATCH, 1), ==, MEMTX_OK);
    g_assert_cmpuint(rd(qts, REGDMA, HP_ROM), ==, MEMTX_OK);
    g_assert_cmpuint(wr(qts, REGDMA, TIMG0_ALARMLO, 0x12345678), ==,
                     MEMTX_OK);
    g_assert_cmphex(qtest_readl(qts, TIMG0_ALARMLO), ==, 0x12345678);
    g_assert_cmpuint(wr(qts, REGDMA, LP_STORE15, 0x9abcdef0), ==, MEMTX_OK);
    g_assert_cmphex(qtest_readl(qts, LP_STORE15), ==, 0x9abcdef0);
    g_assert_cmpuint(wr(qts, REGDMA, LP_SRAM_WORD, 0x0f0f0f0f), ==,
                     MEMTX_OK);
    g_assert_cmphex(qtest_readl(qts, LP_SRAM_WORD), ==, 0x0f0f0f0f);
    assert_no_record(qts);

    qtest_writel(qts, TCM, 0x5a5a5a5a);
    for (int i = 0; i < ARRAY_SIZE(no_path); i++) {
        g_assert_cmpuint(wr(qts, REGDMA, no_path[i], 0), ==,
                         MEMTX_DECODE_ERROR);
        assert_illegal(qts, false, 4, no_path[i], true);
    }
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 1);
    g_assert_cmphex(qtest_readl(qts, TCM), ==, 0x5a5a5a5a);
    qtest_quit(qts);
}

/*
 * The LP core reaches the HP CPU peripherals, the memories at their 0x4...
 * addresses, the HP and LP peripherals and LP SRAM. It has no path to the
 * CPU subsystem, the TCM, the bridges' buffers or any 0x8... address: a
 * decode error, which, like any access of a core to an address nothing
 * decodes, is not recorded.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_lp_core(void)
{
    QTestState *qts = start();
    static const uint32_t no_path[] = {
        DIRECT(SCRATCH), ROM_DIRECT, FLASH_DIRECT, PSRAM_DIRECT, TCM, CLINT,
        CLIC, CSI_MEM, DSI_MEM,
    };
    uint32_t v;

    qtest_writel(qts, TCM, 0x5a5a5a5a);
    g_assert_cmpuint(wr(qts, LP_CORE, SCRATCH, 0x11111111), ==, MEMTX_OK);
    g_assert_cmpuint(rd(qts, LP_CORE, HP_ROM), ==, MEMTX_OK);
    g_assert_cmpuint(rd(qts, LP_CORE, TRACE0), ==, MEMTX_OK);
    g_assert_cmpuint(rd(qts, LP_CORE, UART0), ==, MEMTX_OK);
    g_assert_cmpuint(wr(qts, LP_CORE, LP_SRAM_WORD, 0x22222222), ==,
                     MEMTX_OK);
    g_assert_cmpuint(wr(qts, LP_CORE, LP_STORE15, 0x33333333), ==,
                     MEMTX_OK);
    for (int i = 0; i < ARRAY_SIZE(no_path); i++) {
        g_assert_cmpuint(probe(qts, LP_CORE, false, no_path[i], 0, &v), ==,
                         MEMTX_DECODE_ERROR);
        g_assert_cmphex(v, ==, 0);
        g_assert_cmpuint(wr(qts, LP_CORE, no_path[i], 0x44444444), ==,
                         MEMTX_DECODE_ERROR);
    }
    g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, TCM), ==, 0x5a5a5a5a);
    assert_no_record(qts);
    qtest_quit(qts);
}

/*
 * The HP cores reach the CPU subsystem, the TCM, the memories at both
 * their addresses (the direct ones through the HP APM), the peripherals
 * and LP SRAM; neither the bridges' buffers, which table 9.3-1 reserves
 * for them, nor the LP ROM. No record is kept.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_hp_cores(void)
{
    QTestState *qts = start();
    uint32_t v;

    for (int c = 0; c < 2; c++) {
        uint32_t core = c ? CORE1 : CORE0;

        g_assert_cmpuint(wr(qts, core, TCM, 0x10 + c), ==, MEMTX_OK);
        g_assert_cmpuint(wr(qts, core, DIRECT(SCRATCH), 0x20 + c), ==,
                         MEMTX_OK);
        g_assert_cmphex(qtest_readl(qts, SCRATCH), ==, 0x20 + c);
        g_assert_cmpuint(rd(qts, core, ROM_DIRECT), ==, MEMTX_OK);
        g_assert_cmpuint(wr(qts, core, LP_SRAM_WORD, 0x30 + c), ==,
                         MEMTX_OK);
        g_assert_cmpuint(rd(qts, core, CLINT), ==, MEMTX_OK);
        g_assert_cmpuint(rd(qts, core, TRACE0), ==, MEMTX_OK);
        g_assert_cmpuint(probe(qts, core, false, CSI_MEM, 0, &v), ==,
                         MEMTX_DECODE_ERROR);
        g_assert_cmpuint(wr(qts, core, DSI_MEM, 0), ==, MEMTX_DECODE_ERROR);
        g_assert_cmpuint(rd(qts, core, LP_ROM), ==, MEMTX_DECODE_ERROR);
        assert_no_record(qts);
    }
    qtest_quit(qts);
}

/* GDMA, transferring into addresses out of its reach */

#define AHB                 0x50085000
#define AXI                 0x5008a000
#define AHB_IN_INT(ch)      (AHB + 0x10 * (ch))
#define AHB_OUT_INT(ch)     (AHB + 0x30 + 0x10 * (ch))
#define AHB_IN(ch)          (AHB + 0x70 + 0xc0 * (ch))
#define AHB_OUT(ch)         (AHB + 0xd0 + 0xc0 * (ch))
#define AHB_IN_LINK_ADDR(ch)  (AHB + 0x3ac + 4 * (ch))
#define AHB_OUT_LINK_ADDR(ch) (AHB + 0x3b8 + 4 * (ch))
#define AHB_RESP_ERR0       (AHB + 0x408)
#define AHB_RESP_ERR1       (AHB + 0x40c)
#define AXI_IN(ch)          (AXI + 0x68 * (ch))
#define AXI_OUT(ch)         (AXI + 0x138 + 0x68 * (ch))
#define CONF0               0x00
#define LINK                0x10
#define PERI_SEL            0x30
#define RAW                 0x0
#define CLR                 0xc
#define X_RAW               0x00
#define X_CLR               0x0c
#define X_CONF0             0x10
#define X_LINK1             0x20
#define X_LINK2             0x24
#define X_PERI_SEL          0x44

#define IN_DONE             (1u << 0)
#define IN_SUC_EOF          (1u << 1)
#define IN_DSCR_ERR         (1u << 3)
#define IN_RESP_ERR         (1u << 7)
#define OUT_DONE            (1u << 0)
#define OUT_EOF             (1u << 1)
#define OUT_DSCR_ERR        (1u << 2)
#define OUT_TOTAL_EOF       (1u << 3)
#define OUT_RESP_ERR        (1u << 6)
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define CONF0_RST           (1u << 0)
#define DW0(size, len)      ((size) | (len) << 12)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)
#define AXI_DUMMY           6

/* GDMA-AHB channel 1's master ID in memory-to-memory mode, 16 + 11. */
#define AHB_M2M_CH1_ID      (16 + 11)
#define AXI_M2M_ID          AXI_ID(8, AXI_DUMMY)

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

static void fill(QTestState *qts, uint32_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *b = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        b[i] = seed + i * 7 + (i >> 8);
    }
    qtest_memwrite(qts, addr, b, len);
}

static bool same(QTestState *qts, uint32_t a, uint32_t b, uint32_t len)
{
    g_autofree uint8_t *x = g_malloc(len);
    g_autofree uint8_t *y = g_malloc(len);

    qtest_memread(qts, a, x, len);
    qtest_memread(qts, b, y, len);
    return !memcmp(x, y, len);
}

static void ahb_m2m(QTestState *qts, uint32_t tx, uint32_t rx)
{
    qtest_writel(qts, AHB_IN_INT(1) + CLR, 0xff);
    qtest_writel(qts, AHB_OUT_INT(1) + CLR, 0x7f);
    qtest_writel(qts, AHB_IN(1) + CONF0, CONF0_RST);
    qtest_writel(qts, AHB_IN(1) + CONF0, 0);
    qtest_writel(qts, AHB_OUT(1) + CONF0, CONF0_RST);
    qtest_writel(qts, AHB_OUT(1) + CONF0, 0);
    qtest_writel(qts, AHB_IN(1) + PERI_SEL, 6);
    qtest_writel(qts, AHB_OUT(1) + PERI_SEL, 6);
    /* MEM_TRANS_EN; OUT_EOF_MODE and OUT_AUTO_WRBACK. */
    qtest_writel(qts, AHB_IN(1) + CONF0, 1u << 4);
    qtest_writel(qts, AHB_OUT(1) + CONF0, (1u << 3) | (1u << 2));
    qtest_writel(qts, AHB_IN_LINK_ADDR(1), rx);
    qtest_writel(qts, AHB_OUT_LINK_ADDR(1), tx);
    qtest_writel(qts, AHB_IN(1) + LINK, IN_LINK_START | 1);
    qtest_writel(qts, AHB_OUT(1) + LINK, OUT_LINK_START);
    qtest_clock_step(qts, 1000 * 1000);
}

static void axi_m2m(QTestState *qts, uint32_t tx, uint32_t rx)
{
    qtest_writel(qts, AXI_IN(0) + X_CLR, 0xff);
    qtest_writel(qts, AXI_OUT(0) + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_IN(0) + X_CONF0, CONF0_RST);
    qtest_writel(qts, AXI_IN(0) + X_CONF0, 0);
    qtest_writel(qts, AXI_OUT(0) + X_CONF0, CONF0_RST);
    qtest_writel(qts, AXI_OUT(0) + X_CONF0, 0);
    qtest_writel(qts, AXI_IN(0) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_OUT(0) + X_PERI_SEL, AXI_DUMMY);
    qtest_writel(qts, AXI_IN(0) + X_CONF0, 1u << 2);
    qtest_writel(qts, AXI_OUT(0) + X_CONF0, 1u << 3);
    qtest_writel(qts, AXI_IN(0) + X_LINK2, rx);
    qtest_writel(qts, AXI_OUT(0) + X_LINK2, tx);
    qtest_writel(qts, AXI_IN(0) + X_LINK1, IN_LINK_START | 1);
    qtest_writel(qts, AXI_OUT(0) + X_LINK1, OUT_LINK_START);
    qtest_clock_step(qts, 1000 * 1000);
}

/*
 * GDMA-AHB channel 1 in memory-to-memory mode, its RX buffer at the 0x8...
 * alias of an L2MEM buffer, in LP SRAM and on a peripheral register: the
 * transfer ends, every write fails with IN_AHBINF_RESP_ERR, nothing lands,
 * and the ICM records the first write as illegal. A TX descriptor at the
 * alias of a good one is a descriptor error. The same transfer to the
 * buffer's own address copies it.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_gdma_ahb(void)
{
    QTestState *qts = start();
    uint32_t tx = SCRATCH, rx = SCRATCH + 0x100, src = SCRATCH + 0x1000;
    uint32_t dst = SCRATCH + 0x2000;
    static const uint32_t targets[] = {
        DIRECT(SCRATCH + 0x2000), LP_SRAM + 0x7f00, TIMG0_ALARMLO,
    };

    fill(qts, src, 64, 1);
    for (int i = 0; i < ARRAY_SIZE(targets); i++) {
        uint32_t t = targets[i];
        uint32_t len = t == TIMG0_ALARMLO ? 4 : 64;

        g_test_message("RX at 0x%08x", t);
        qtest_memset(qts, dst, 0, 64);
        qtest_memset(qts, LP_SRAM + 0x7f00, 0, 64);
        qtest_writel(qts, TIMG0_ALARMLO, 0);
        clear_records(qts);
        desc(qts, tx, OWNER | SUC_EOF | DW0(len, len), src, 0);
        desc(qts, rx, OWNER | DW0(len, 0), t, 0);
        ahb_m2m(qts, tx, rx);
        g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==,
                        IN_DONE | IN_SUC_EOF | IN_RESP_ERR);
        /* The last refused write: WR, ID 11, RX channel 1. */
        g_assert_cmphex(qtest_readl(qts, AHB_RESP_ERR1), ==,
                        1 | 11 << 1 | 1 << 5);
        g_assert_cmphex(qtest_readl(qts, AHB_RESP_ERR0) - t, <, len);
        g_assert_cmphex(qtest_readl(qts, rx), ==,
                        SUC_EOF | DW0(len, len));
        assert_illegal(qts, false, AHB_M2M_CH1_ID, t, true);
        g_assert_cmpuint(qtest_readl(qts, dst), ==, 0);
        g_assert_cmpuint(qtest_readl(qts, LP_SRAM + 0x7f00), ==, 0);
        g_assert_cmpuint(qtest_readl(qts, TIMG0_ALARMLO), ==, 0);
    }

    /* A TX descriptor read through the alias. */
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    ahb_m2m(qts, DIRECT(tx), rx);
    g_assert_cmphex(qtest_readl(qts, AHB_OUT_INT(1) + RAW) & OUT_DSCR_ERR,
                    ==, OUT_DSCR_ERR);
    assert_illegal(qts, false, AHB_M2M_CH1_ID, DIRECT(tx), false);
    g_assert_cmpuint(qtest_readl(qts, dst), ==, 0);

    /* At the buffer's own address, through the DMA APM as before. */
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    ahb_m2m(qts, tx, rx);
    g_assert_cmphex(qtest_readl(qts, AHB_IN_INT(1) + RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    g_assert_true(same(qts, dst, src, 64));
    assert_no_record(qts);
    qtest_quit(qts);
}

/*
 * GDMA-AXI channel 0 in memory-to-memory mode: an RX buffer out of reach
 * loses its writes, which GDMA-AXI does not report but the ICM records in
 * its AXI record; a TX buffer read through the alias reads zeros; a
 * descriptor through the alias is a descriptor error.
 */
/* [spec:nuos:req:emu.esp32p4.pms/test] */
static void test_gdma_axi(void)
{
    QTestState *qts = start();
    uint32_t tx = SCRATCH, rx = SCRATCH + 0x100, src = SCRATCH + 0x1000;
    uint32_t dst = SCRATCH + 0x2000;

    fill(qts, src, 64, 2);
    qtest_memset(qts, dst, 0, 64);
    qtest_memset(qts, LP_SRAM + 0x7f00, 0, 64);

    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), DIRECT(dst), 0);
    axi_m2m(qts, tx, rx);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    assert_illegal(qts, true, AXI_M2M_ID, DIRECT(dst), true);
    g_assert_cmpuint(qtest_readl(qts, dst), ==, 0);

    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), LP_SRAM + 0x7f00, 0);
    axi_m2m(qts, tx, rx);
    assert_illegal(qts, true, AXI_M2M_ID, LP_SRAM + 0x7f00, true);
    g_assert_cmpuint(qtest_readl(qts, LP_SRAM + 0x7f00), ==, 0);

    /* The source through the alias: zeros arrive. */
    qtest_memset(qts, dst, 0xee, 64);
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), DIRECT(src), 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    axi_m2m(qts, tx, rx);
    assert_illegal(qts, true, AXI_M2M_ID, DIRECT(src), false);
    g_assert_cmpuint(qtest_readl(qts, dst), ==, 0);

    /* An RX descriptor through the alias. */
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    axi_m2m(qts, tx, DIRECT(rx));
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_RAW) & IN_DSCR_ERR, ==,
                    IN_DSCR_ERR);
    assert_illegal(qts, true, AXI_M2M_ID, DIRECT(rx), false);

    /* At the buffer's own address, as before. */
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    axi_m2m(qts, tx, rx);
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_RAW), ==,
                    IN_DONE | IN_SUC_EOF);
    g_assert_true(same(qts, dst, src, 64));
    assert_no_record(qts);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4-bus-reach/dma", test_dma_reach);
    qtest_add_func("/esp32p4-bus-reach/apm-and-reach", test_apm_and_reach);
    qtest_add_func("/esp32p4-bus-reach/vdma", test_vdma);
    qtest_add_func("/esp32p4-bus-reach/regdma", test_regdma);
    qtest_add_func("/esp32p4-bus-reach/lp-core", test_lp_core);
    qtest_add_func("/esp32p4-bus-reach/hp-cores", test_hp_cores);
    qtest_add_func("/esp32p4-bus-reach/gdma-ahb", test_gdma_ahb);
    qtest_add_func("/esp32p4-bus-reach/gdma-axi", test_gdma_axi);

    return g_test_run();
}
