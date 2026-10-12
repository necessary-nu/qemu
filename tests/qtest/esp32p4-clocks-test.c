/*
 * QTest testcase for the ESP32-P4 reset and clock control and the analog
 * I2C controller
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qobject/qdict.h"
#include "libqtest.h"
#include "hw/misc/esp32p4_clkrst.h"

#define HP_BASE     0x500e6000
#define LP_BASE     0x50111000
#define LPPERI_BASE 0x50120000
#define ANA_BASE    0x50124000

#define CLKRST  "/machine/clkrst"
#define ANA     "/machine/ana"
#define HP_CPU0 "/machine/hp-cluster/cpu[0]"
#define HP_CPU1 "/machine/hp-cluster/cpu[1]"
#define LP_CPU  "/machine/lp-cluster/cpu"

#define ROOT_CLK_CTRL0      (HP_BASE + 0x04)
#define ROOT_CLK_CTRL1      (HP_BASE + 0x08)
#define ROOT_CLK_CTRL2      (HP_BASE + 0x0c)
#define ROOT_CLK_CTRL3      (HP_BASE + 0x10)
#define SOC_CLK_CTRL0       (HP_BASE + 0x14)
#define SOC_CLK_CTRL2       (HP_BASE + 0x1c)
#define REF_CLK_CTRL1       (HP_BASE + 0x28)
#define PERI_CLK_CTRL01     (HP_BASE + 0x34)
#define PERI_CLK_CTRL02     (HP_BASE + 0x38)
#define PERI_CLK_CTRL13     (HP_BASE + 0x4c)
#define PERI_CLK_CTRL14     (HP_BASE + 0x50)
#define PERI_CLK_CTRL110    (HP_BASE + 0x68)
#define PERI_CLK_CTRL111    (HP_BASE + 0x6c)
#define PERI_CLK_CTRL20     (HP_BASE + 0x94)
#define PERI_CLK_CTRL21     (HP_BASE + 0x98)
#define ANA_PLL_CTRL0       (HP_BASE + 0xbc)
#define HP_RST_EN0          (HP_BASE + 0xc0)
#define HP_RST_EN1          (HP_BASE + 0xc4)
#define HP_RST_EN2          (HP_BASE + 0xc8)
#define HP_FORCE_NORST0     (HP_BASE + 0xcc)
#define HPWDT_CORE1_RST_CTRL0 (HP_BASE + 0xd8)
#define CPU_SRC_FREQ0       (HP_BASE + 0xdc)
#define CPU_CLK_STATUS0     (HP_BASE + 0xe0)
#define DBG_CLK_CTRL0       (HP_BASE + 0xe4)
#define DBG_CLK_CTRL1       (HP_BASE + 0xe8)
#define HPCORE_WDT_RESET_SOURCE0 (HP_BASE + 0xec)

#define LP_CLK_CONF         (LP_BASE + 0x00)
#define RESET_CAUSE         (LP_BASE + 0x10)
#define HPCPU_RESET_CTRL0   (LP_BASE + 0x14)
#define FOSC_CNTL           (LP_BASE + 0x1c)
#define HP_CLK_CTRL         (LP_BASE + 0x40)

#define LPPERI_CLK_EN       (LPPERI_BASE + 0x00)
#define LPPERI_CORE_CLK_SEL (LPPERI_BASE + 0x04)
#define LPPERI_RESET_EN     (LPPERI_BASE + 0x08)
#define LPPERI_MEM_CTRL     (LPPERI_BASE + 0x28)

#define I2C0_CTRL           (ANA_BASE + 0x00)
#define I2C1_CTRL           (ANA_BASE + 0x04)
#define ANA_CONF2           (ANA_BASE + 0x20)
#define I2C0_CTRL1          (ANA_BASE + 0x24)
#define CLK160M             (ANA_BASE + 0x34)
#define CTRL_WRITE          (1u << 24)
#define CTRL_BUSY           (1u << 25)

#define EFUSE_BASE          0x5012d000
#define EFUSE_CONF          (EFUSE_BASE + 0x1cc)
#define EFUSE_STATUS        (EFUSE_BASE + 0x1d0)
#define EFUSE_CMD           (EFUSE_BASE + 0x1d4)
#define EFUSE_READ_OP_CODE  0x5aa5
#define EFUSE_CMD_READ      1
#define EFUSE_STATE_RESET   0
#define EFUSE_STATE_IDLE    1
/* A read with the reset timing: 15 + 128 * 4 + 1 cycles of 20 MHz. */
#define EFUSE_READ_NS       ((15 + 128 * 4 + 1) * 50)

#define SLAVE_CPU_PLL       0x67
#define SLAVE_MSPI          0x63
#define SLAVE_PLLA          0x6f
#define CONF2_CPU_PLL       (1u << 11)
#define CONF2_MSPI          (1u << 9)
#define CONF2_PLLA          (1u << 8)

#define US 1000
#define MHZ 1000000ull

/* A register: its address, reset value and R/W, WT and RO fields. */
typedef struct {
    uint32_t addr;
    uint32_t reset;
    uint32_t rw;
    uint32_t wt;
    uint32_t ro;
} Reg;

/*
 * Every register of the three blocks, from ESP-IDF's v3 headers. The RO
 * fields' reset values are what the hardware drives out of a chip reset.
 */
static const Reg clkrst_regs[] = {
    { HP_BASE + 0x000, 0x00000001, 0x00000001, 0, 0 },
    { HP_BASE + 0x004, 0x00000000, 0x1fffffef, 0x00000010, 0 },
    { HP_BASE + 0x008, 0x00000001, 0xffffffff, 0, 0 },
    { HP_BASE + 0x00c, 0x00010000, 0xffffffff, 0, 0 },
    { HP_BASE + 0x010, 0x00000000, 0x000000ff, 0, 0 },
    { HP_BASE + 0x014, 0xe6df97af, 0xffffffff, 0, 0 },
    { HP_BASE + 0x018, 0x7c7f001f, 0xffffffff, 0, 0 },
    { HP_BASE + 0x01c, 0x20f80fde, 0xffffffff, 0, 0 },
    { HP_BASE + 0x020, 0x00000018, 0x0000001f, 0, 0 },
    { HP_BASE + 0x024, 0x02011309, 0xffffffff, 0, 0 },
    { HP_BASE + 0x028, 0x58170503, 0xffffffff, 0, 0 },
    { HP_BASE + 0x02c, 0x00000115, 0x000003ff, 0, 0 },
    { HP_BASE + 0x030, 0x0000c03c, 0x3fffffff, 0, 0 },
    { HP_BASE + 0x034, 0x00000401, 0x01ffffff, 0, 0 },
    { HP_BASE + 0x038, 0x00000000, 0xfffffeff, 0x00000100, 0 },
    { HP_BASE + 0x03c, 0x00080000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x040, 0x00000000, 0x0fffffff, 0, 0 },
    { HP_BASE + 0x044, 0x00000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x048, 0x00000000, 0x03ffffff, 0, 0 },
    { HP_BASE + 0x04c, 0x00000000, 0x3fffffff, 0, 0 },
    { HP_BASE + 0x050, 0x00000000, 0x7fffffff, 0, 0 },
    { HP_BASE + 0x054, 0x00000000, 0x7fffffff, 0, 0 },
    { HP_BASE + 0x058, 0x00000000, 0x03ffffff, 0, 0 },
    { HP_BASE + 0x05c, 0x00000000, 0x7fffffff, 0, 0 },
    { HP_BASE + 0x060, 0x00000000, 0x3fffffff, 0, 0 },
    { HP_BASE + 0x064, 0x00000000, 0xffffffff, 0, 0 },
    { HP_BASE + 0x068, 0x04000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x06c, 0x04000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x070, 0x04000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x074, 0x04000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x078, 0x04000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x07c, 0x00000000, 0x3fffffff, 0, 0 },
    { HP_BASE + 0x080, 0x01100008, 0x01ffffff, 0, 0 },
    { HP_BASE + 0x084, 0x00010000, 0x0fffffff, 0, 0 },
    { HP_BASE + 0x088, 0x00000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x08c, 0x00000000, 0x3fffffff, 0, 0 },
    { HP_BASE + 0x090, 0x00000000, 0x00ffffff, 0, 0 },
    { HP_BASE + 0x094, 0xc9000000, 0xffffffff, 0, 0 },
    { HP_BASE + 0x098, 0x52400000, 0x7fffffff, 0, 0 },
    { HP_BASE + 0x09c, 0x00000000, 0xffffffff, 0, 0 },
    { HP_BASE + 0x0a0, 0x00000008, 0x01ffffff, 0, 0 },
    { HP_BASE + 0x0a4, 0x00000404, 0x01ffffff, 0, 0 },
    { HP_BASE + 0x0a8, 0x007fc000, 0x03ffffff, 0, 0 },
    { HP_BASE + 0x0ac, 0x00000200, 0x3fffffff, 0, 0 },
    { HP_BASE + 0x0b0, 0x00000000, 0x0003ffff, 0, 0 },
    { HP_BASE + 0x0b4, 0x0007ffff, 0x0007ffff, 0, 0 },
    { HP_BASE + 0x0b8, 0x00000000, 0x00000007, 0, 0 },
    { HP_BASE + 0x0bc, 0x00000000, 0x000002aa, 0, 0x00000155 },
    { HP_BASE + 0x0c0, 0x00000100, 0xffffffff, 0, 0 },
    { HP_BASE + 0x0c4, 0x00000000, 0xffffffff, 0, 0 },
    { HP_BASE + 0x0c8, 0x00000000, 0x01ffffff, 0, 0 },
    { HP_BASE + 0x0cc, 0x00000000, 0xffffffff, 0, 0 },
    { HP_BASE + 0x0d0, 0x00000000, 0x07ffffff, 0, 0 },
    { HP_BASE + 0x0d4, 0x00001011, 0x0001ffff, 0, 0 },
    { HP_BASE + 0x0d8, 0x00001011, 0x0001ffff, 0, 0 },
    /* XTAL_CLK's 40 MHz, in 0.25 MHz steps. */
    { HP_BASE + 0x0dc, 160, 0, 0, 0xffffffff },
    { HP_BASE + 0x0e0, 0x00000000, 0, 0, 0x07ffffff },
    { HP_BASE + 0x0e4, 0x03ffffff, 0xffffffff, 0, 0 },
    { HP_BASE + 0x0e8, 0x00000303, 0x0007ffff, 0, 0 },
    { HP_BASE + 0x0ec, 0x00000002, 0x00000003, 0, 0 },
    { HP_BASE + 0x0f0, 0x00000000, 0x00000003, 0, 0 },
    { HP_BASE + 0x0f4, 0x00000003, 0x00000003, 0, 0 },

    { LP_BASE + 0x000, 0x00000004, 0x000007ff, 0, 0 },
    { LP_BASE + 0x004, 0x00000000, 0x000003ff, 0, 0 },
    { LP_BASE + 0x008, 0x08000000, 0xfc000000, 0, 0 },
    { LP_BASE + 0x00c, 0x00000000, 0xff000000, 0, 0 },
    /* A chip reset, cause 0x01, flagged for all three cores. */
    { LP_BASE + 0x010, 0x021060c1, 0x02000000, 0xfc000000, 0x001fffff },
    { LP_BASE + 0x014, 0x80028002, 0xdfffdfff, 0x20002000, 0 },
    { LP_BASE + 0x018, 0x00000000, 0xffff0000, 0, 0 },
    { LP_BASE + 0x01c, 0x64000000, 0xffc00000, 0, 0 },
    { LP_BASE + 0x020, 0x0000028a, 0xffffffff, 0, 0 },
    { LP_BASE + 0x024, 0x2b000000, 0xffc00000, 0, 0 },
    { LP_BASE + 0x028, 0xf0000000, 0xf0000000, 0, 0 },
    { LP_BASE + 0x02c, 0x00000000, 0x80000000, 0, 0 },
    { LP_BASE + 0x030, 0x66c00000, 0xffc00000, 0, 0 },
    { LP_BASE + 0x034, 0xffffffff, 0xffffffff, 0, 0 },
    { LP_BASE + 0x038, 0xffffffff, 0xffffffff, 0, 0 },
    { LP_BASE + 0x03c, 0xffffffff, 0xffffffff, 0, 0 },
    { LP_BASE + 0x040, 0x1ffffffc, 0x1fffffff, 0, 0 },
    { LP_BASE + 0x044, 0x09c4c27a, 0x3fffffff, 0, 0 },
    { LP_BASE + 0x048, 0xc0000000, 0xf000001f, 0, 0 },
    { LP_BASE + 0x04c, 0x00000000, 0xf0000000, 0, 0 },
    { LP_BASE + 0x3fc, 0x00000000, 0x80000000, 0, 0 },

    { LPPERI_BASE + 0x000, 0x7fff0000, 0xffff0000, 0, 0 },
    { LPPERI_BASE + 0x004, 0x00000000, 0xff000000, 0, 0 },
    { LPPERI_BASE + 0x008, 0x00000000, 0x7ffe0000, 0x80000000, 0 },
    { LPPERI_BASE + 0x00c, 0x80000000, 0x80000000, 0, 0 },
    { LPPERI_BASE + 0x028, 0x80000000, 0xe0000000, 0x00000001, 0x00000002 },
    { LPPERI_BASE + 0x02c, 0x04040400, 0xffffffc0, 0, 0 },
    { LPPERI_BASE + 0x030, 0x02000000, 0xff000000, 0, 0 },
    { LPPERI_BASE + 0x034, 0x00004000, 0xfffffff0, 0, 0 },
    { LPPERI_BASE + 0x038, 0x02000000, 0xff000000, 0, 0 },
    { LPPERI_BASE + 0x03c, 0x00004000, 0xfffffff0, 0, 0 },
    { LPPERI_BASE + 0x3fc, 0x00000000, 0x80000000, 0, 0 },
};

/* The analog I2C controller's, but for the CTRL registers. */
static const Reg ana_regs[] = {
    { ANA_BASE + 0x008, 0x00000000, 0x00ffffff, 0, 0xff000000 },
    { ANA_BASE + 0x00c, 0x00000000, 0x00ffffff, 0, 0xff000000 },
    { ANA_BASE + 0x010, 0x00000000, 0xffffffff, 0, 0 },
    { ANA_BASE + 0x014, 0x40000000, 0xfff00000, 0, 0x00000007 },
    { ANA_BASE + 0x018, 0x00000000, 0x00ffffff, 0, 0xff000000 },
    { ANA_BASE + 0x01c, 0x00000000, 0x00ffffff, 0, 0xff000000 },
    { ANA_BASE + 0x020, 0x00000000, 0x00ffffff, 0, 0xff000000 },
    { ANA_BASE + 0x024, 0x00000042, 0x000007ff, 0, 0 },
    { ANA_BASE + 0x028, 0x00000042, 0x000007ff, 0, 0 },
    { ANA_BASE + 0x02c, 0x00000042, 0x00000fff, 0, 0 },
    { ANA_BASE + 0x030, 0x00000000, 0xffffffff, 0, 0 },
    { ANA_BASE + 0x034, 0x00000000, 0x00000001, 0, 0 },
    { ANA_BASE + 0x038, 0x02304230, 0x1fffffff, 0, 0 },
};

static uint64_t clock_period(QTestState *qts, const char *path)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'qtest-clock-period' } }",
                         path);
    uint64_t period;

    g_assert(qdict_haskey(r, "return"));
    period = qdict_get_uint(r, "return");
    qobject_unref(r);
    return period;
}

static uint64_t qom_get_uint(QTestState *qts, const char *path,
                             const char *property)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': %s } }", path, property);
    uint64_t v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_uint(r, "return");
    qobject_unref(r);
    return v;
}

static uint64_t path_hz(QTestState *qts, const char *path)
{
    uint64_t period = clock_period(qts, path);

    return period ? (1000000000ull << 32) / period : 0;
}

static uint64_t clk_hz(QTestState *qts, const char *name)
{
    g_autofree char *path = g_strdup_printf(CLKRST "/%s", name);

    return path_hz(qts, path);
}

/* The clock's frequency, to within the rounding of its period. */
static void assert_hz(QTestState *qts, const char *name, uint64_t want)
{
    uint64_t got = clk_hz(qts, name);
    uint64_t slack = want / 100000 + 1;

    if (got + slack < want || got > want + slack) {
        g_test_message("%s: %" PRIu64 " Hz, want %" PRIu64 " Hz", name, got,
                       want);
        g_test_fail();
    }
}

/*
 * The Clock period of a clock of @num / @den Hz: 2^32 * 10^9 * @den / @num
 * units of 2^-32 ns, rounded down.
 */
static uint64_t period_of(uint64_t num, uint64_t den)
{
    uint64_t lo, hi;

    mulu64(&lo, &hi, 1000000000ull << 32, den);
    divu128(&lo, &hi, num);
    g_assert_cmpuint(hi, ==, 0);
    return lo;
}

/* The clock's period is that of a clock of @num / @den Hz. */
static void assert_period(QTestState *qts, const char *name, uint64_t num,
                          uint64_t den)
{
    g_autofree char *path = g_strdup_printf(CLKRST "/%s", name);
    uint64_t got = clock_period(qts, path);
    uint64_t want = period_of(num, den);

    if (got != want) {
        g_test_message("%s: period %" PRIu64 ", want %" PRIu64, name, got,
                       want);
        g_test_fail();
    }
}

static void assert_cpu_hz(QTestState *qts, const char *cpu, uint64_t want)
{
    g_autofree char *path = g_strdup_printf("%s/icount-clock", cpu);
    uint64_t got = path_hz(qts, path);

    if (got + want / 100000 + 1 < want || got > want + want / 100000 + 1) {
        g_test_message("%s executes at %" PRIu64 " Hz, want %" PRIu64,
                       cpu, got, want);
        g_test_fail();
    }
}

static void set_bits(QTestState *qts, uint32_t addr, uint32_t bits)
{
    qtest_writel(qts, addr, qtest_readl(qts, addr) | bits);
}

static void clear_bits(QTestState *qts, uint32_t addr, uint32_t bits)
{
    qtest_writel(qts, addr, qtest_readl(qts, addr) & ~bits);
}

/* Run an analog I2C transaction on master 0 and wait for its end. */
static uint32_t ana_xfer(QTestState *qts, uint32_t ctrl)
{
    uint32_t v;
    int i;

    qtest_writel(qts, I2C0_CTRL, ctrl);
    for (i = 0; i < 100; i++) {
        v = qtest_readl(qts, I2C0_CTRL);
        if (!(v & CTRL_BUSY)) {
            return v;
        }
        qtest_clock_step(qts, US);
    }
    g_assert_not_reached();
}

static void ana_write(QTestState *qts, uint8_t slave, uint8_t reg,
                      uint8_t data)
{
    ana_xfer(qts, slave | (reg << 8) | (data << 16) | CTRL_WRITE);
}

static uint8_t ana_read(QTestState *qts, uint8_t slave, uint8_t reg)
{
    return ana_xfer(qts, slave | (reg << 8)) >> 16;
}

static void check_regs(QTestState *qts, const Reg *regs, size_t n)
{
    size_t i;

    for (i = 0; i < n; i++) {
        uint32_t got = qtest_readl(qts, regs[i].addr);

        if (got != regs[i].reset) {
            g_test_message("0x%08" PRIx32 " reads 0x%08" PRIx32 " at reset, "
                           "want 0x%08" PRIx32, regs[i].addr, got,
                           regs[i].reset);
            g_test_fail();
        }
    }
}

/*
 * Write each register with every bit set, then clear: its R/W fields
 * take the value, its WT fields and reserved bits read 0. The RO fields
 * follow the hardware and are left out.
 */
static void check_access(QTestState *qts, const Reg *regs, size_t n)
{
    size_t i;

    for (i = 0; i < n; i++) {
        uint32_t addr = regs[i].addr;
        uint32_t got;

        qtest_writel(qts, addr, 0xffffffff);
        got = qtest_readl(qts, addr) & ~regs[i].ro;
        if (got != regs[i].rw) {
            g_test_message("0x%08" PRIx32 " reads 0x%08" PRIx32 " after "
                           "writing ones, want 0x%08" PRIx32, addr, got,
                           regs[i].rw);
            g_test_fail();
        }
        qtest_writel(qts, addr, 0);
        got = qtest_readl(qts, addr) & ~regs[i].ro;
        if (got) {
            g_test_message("0x%08" PRIx32 " reads 0x%08" PRIx32 " after "
                           "writing zeros", addr, got);
            g_test_fail();
        }
    }
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_registers(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    check_regs(qts, clkrst_regs, ARRAY_SIZE(clkrst_regs));
    check_regs(qts, ana_regs, ARRAY_SIZE(ana_regs));
    g_assert_cmphex(qtest_readl(qts, I2C0_CTRL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, I2C1_CTRL), ==, 0);
    check_access(qts, ana_regs, ARRAY_SIZE(ana_regs));
    check_access(qts, clkrst_regs, ARRAY_SIZE(clkrst_regs));

    /* A system reset restores every register. */
    qtest_system_reset(qts);
    check_regs(qts, clkrst_regs, ARRAY_SIZE(clkrst_regs));
    check_regs(qts, ana_regs, ARRAY_SIZE(ana_regs));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_byte_access(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* A byte or halfword reaches its lanes only. */
    qtest_writeb(qts, REF_CLK_CTRL1 + 1, 0x07);
    g_assert_cmphex(qtest_readl(qts, REF_CLK_CTRL1), ==, 0x58170703);
    qtest_writew(qts, REF_CLK_CTRL1 + 2, 0x5817);
    g_assert_cmphex(qtest_readw(qts, REF_CLK_CTRL1), ==, 0x0703);
    g_assert_cmphex(qtest_readb(qts, REF_CLK_CTRL1 + 3), ==, 0x58);
    /* A write-1 field acts only when its byte is written with it set. */
    qtest_writeb(qts, HPCPU_RESET_CTRL0 + 1, 0x80);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    qtest_writeb(qts, HPCPU_RESET_CTRL0 + 1, 0xa0);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    qtest_clock_step(qts, 100);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmphex(qtest_readl(qts, HPCPU_RESET_CTRL0), ==, 0x80028002);

    /* Reserved addresses read 0 and ignore writes. */
    qtest_writel(qts, HP_BASE + 0xf8, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, HP_BASE + 0xf8), ==, 0);
    g_assert_cmphex(qtest_readl(qts, HP_BASE + 0x3fc), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LP_BASE + 0x50), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LPPERI_BASE + 0x10), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LPPERI_BASE + 0x24), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LPPERI_BASE + 0x40), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ANA_BASE + 0x3c), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_tree_at_reset(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* The sources: the powered ones run, the others are stopped. */
    assert_hz(qts, "osc-xtal", 40 * MHZ);
    assert_hz(qts, "xtal", 40 * MHZ);
    assert_hz(qts, "rc-fast", 20 * MHZ);
    assert_hz(qts, "rc-slow", 150000);
    assert_hz(qts, "rc32k", 0);
    assert_hz(qts, "xtal32k", 0);
    assert_hz(qts, "pll-lp", 0);
    assert_hz(qts, "cpll", 360 * MHZ);
    assert_hz(qts, "spll", 480 * MHZ);
    assert_hz(qts, "mpll", 0);
    assert_hz(qts, "apll", 0);
    assert_hz(qts, "sdio-pll0", 0);

    /* ROOT_CLK from XTAL_CLK, and its dividers: 40, 20, 20, 10 MHz. */
    assert_hz(qts, "root", 40 * MHZ);
    assert_hz(qts, "cpu", 40 * MHZ);
    assert_hz(qts, "mem", 20 * MHZ);
    assert_hz(qts, "sys", 20 * MHZ);
    assert_hz(qts, "apb", 10 * MHZ);

    assert_hz(qts, "pll-f240m", 240 * MHZ);
    assert_hz(qts, "pll-f160m", 160 * MHZ);
    assert_hz(qts, "pll-f120m", 120 * MHZ);
    assert_hz(qts, "pll-f80m", 80 * MHZ);
    assert_hz(qts, "pll-f20m", 20 * MHZ);
    assert_hz(qts, "pll-f48m", 48 * MHZ);
    assert_hz(qts, "pll-f12m", 12 * MHZ);
    assert_hz(qts, "pll-f50m", 0);

    /* LP_FAST_CLK from XTAL_CLK, LP_SLOW_CLK from RC_SLOW_CLK. */
    assert_hz(qts, "lp-fast", 40 * MHZ);
    assert_hz(qts, "lp-slow", 150000);
    assert_hz(qts, "lp-dyn-fast", 40 * MHZ);
    assert_hz(qts, "lp-peri", 40 * MHZ);
    assert_hz(qts, "xtal-d2", 20 * MHZ);

    /* Core 1's clock is gated, core 0's runs, and they execute at it. */
    assert_hz(qts, "core0-cpu", 40 * MHZ);
    assert_hz(qts, "core1-cpu", 0);
    assert_cpu_hz(qts, HP_CPU0, 40 * MHZ);

    /* Blocks' clocks as their reset gates and muxes have them. */
    assert_hz(qts, "uart0-apb", 10 * MHZ);
    assert_hz(qts, "uart0-sys", 20 * MHZ);
    assert_hz(qts, "uart0-sclk", 40 * MHZ);
    assert_hz(qts, "i2c0-apb", 0);
    assert_hz(qts, "i2c0", 0);
    assert_hz(qts, "timergrp0-t0", 40 * MHZ);
    assert_hz(qts, "systimer", 40 * MHZ);
    assert_hz(qts, "crypto-sha", 40 * MHZ);
    assert_hz(qts, "flash-core", 10 * MHZ);
    assert_hz(qts, "lp-i2cmst", 40 * MHZ);
    assert_hz(qts, "lp-uart-sclk", 20 * MHZ);
    assert_hz(qts, "lp-core", 0);
    assert_hz(qts, "pvt", 0);
    assert_hz(qts, "pvt-peri-group1", 0);
    /* LP_PERI_CLK by LPADC_FUNC_DIV_NUM + 1, then the SARs' by theirs. */
    assert_hz(qts, "lp-adc-func", 8 * MHZ);
    assert_hz(qts, "lp-adc-sar1", 1600000);
    assert_hz(qts, "lp-adc-sar2", 1600000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_soc_clocks(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* The dividers change only when SOC_CLK_DIV_UPDATE takes them. */
    qtest_writel(qts, ROOT_CLK_CTRL0, 3 << 5);
    assert_hz(qts, "cpu", 40 * MHZ);
    g_assert_cmphex(qtest_readl(qts, CPU_CLK_STATUS0), ==, 0);
    qtest_writel(qts, ROOT_CLK_CTRL0, (3 << 5) | (1 << 4));
    g_assert_cmphex(qtest_readl(qts, ROOT_CLK_CTRL0), ==, 3 << 5);
    assert_hz(qts, "cpu", 10 * MHZ);
    assert_hz(qts, "mem", 5 * MHZ);
    assert_hz(qts, "apb", 2500000);
    g_assert_cmphex(qtest_readl(qts, CPU_CLK_STATUS0), ==, (3 << 3) | 2);
    assert_cpu_hz(qts, HP_CPU0, 10 * MHZ);

    /* A fractional divider: 2 + 1/2. */
    qtest_writel(qts, ROOT_CLK_CTRL1, 0);
    qtest_writel(qts, ROOT_CLK_CTRL0, (1 << 5) | (1 << 13) | (2 << 21) |
                 (1 << 4));
    assert_hz(qts, "cpu", 16 * MHZ);
    assert_hz(qts, "mem", 16 * MHZ);
    g_assert_cmphex(qtest_readl(qts, CPU_CLK_STATUS0), ==,
                    (2 << 19) | (1 << 11) | (1 << 3) | 2);

    /* ROOT_CLK from RC_FAST_CLK, at once. */
    qtest_writel(qts, ROOT_CLK_CTRL0, 1 << 4);
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc | 2);
    assert_hz(qts, "cpu", 20 * MHZ);
    g_assert_cmpuint(qtest_readl(qts, CPU_SRC_FREQ0), ==, 80);
    /* RC_FAST_CLK as FOSC_DFREQ trims it: 17.5 MHz at 100. */
    qtest_writel(qts, FOSC_CNTL, 100 << 22);
    assert_hz(qts, "rc-fast", 17500000);
    assert_hz(qts, "cpu", 17500000);
    assert_hz(qts, "lp-uart-sclk", 17500000);

    /* An invalid source and a gated ROOT_CLK stop the cores. */
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc | 3);
    assert_hz(qts, "cpu", 0);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffff8);
    assert_hz(qts, "root", 0);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc);

    /* Gating XTAL_CLK out of the HP system stops what runs from it. */
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc & ~(1u << 25));
    assert_hz(qts, "cpu", 0);
    assert_hz(qts, "lp-fast", 40 * MHZ);
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc);
    assert_hz(qts, "cpu", 40 * MHZ);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_block_clocks(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* UART0's function clock from PLL_F80M_CLK divided by 2 + 1/2. */
    qtest_writel(qts, PERI_CLK_CTRL110, (2 << 24) | (1 << 26));
    qtest_writel(qts, PERI_CLK_CTRL111, 1 | (1 << 8) | (2 << 16) |
                 0x04000000);
    assert_hz(qts, "uart0-sclk", 32 * MHZ);
    clear_bits(qts, PERI_CLK_CTRL110, 1 << 26);
    assert_hz(qts, "uart0-sclk", 0);
    /* An invalid source selects a stopped clock. */
    qtest_writel(qts, PERI_CLK_CTRL110, (3 << 24) | (1 << 26));
    assert_hz(qts, "uart0-sclk", 0);

    /* The bus clock's gate. */
    clear_bits(qts, SOC_CLK_CTRL2, 1 << 7);
    assert_hz(qts, "uart0-apb", 0);

    /* I2S0's TX clock from XTAL_CLK, by 4 + 1/2: N 4, X 1, Y 0, Z 1. */
    qtest_writel(qts, PERI_CLK_CTRL13, (1 << 10) | (4 << 13) | (1 << 21));
    qtest_writel(qts, PERI_CLK_CTRL14, 1 << 9);
    assert_hz(qts, "i2s0-tx", 8888888);
    /* With YN1 the fraction is 1 - Z / a: 4 + 1/2 again for Z 1, X 1. */
    qtest_writel(qts, PERI_CLK_CTRL14, (1 << 9) | (1 << 18));
    assert_hz(qts, "i2s0-tx", 8888888);
    /* The master clock output follows the TX clock with MST_CLK_SEL. */
    assert_hz(qts, "i2s0-mst", 0);
    set_bits(qts, PERI_CLK_CTRL14, 1 << 19);
    assert_hz(qts, "i2s0-mst", 8888888);

    /* The PVT monitor's clocks, as ESP-IDF sets them up: 160 / 2 MHz. */
    qtest_writel(qts, HP_BASE + 0xa4, 0x00000404 | (1 << 16) | (1 << 24));
    qtest_writel(qts, HP_BASE + 0xa8, 0x007fc000 | 1 | (0xf << 8));
    assert_hz(qts, "pvt", 80 * MHZ);
    assert_hz(qts, "pvt-peri-group4", 80 * MHZ);

    /* The calibration clock: RC_FAST_CLK, then RC32K, which the PMU powers. */
    qtest_writel(qts, PERI_CLK_CTRL21, 0x52400000 | 7);
    assert_hz(qts, "timergrp0-tgrt", 20 * MHZ);
    qtest_writel(qts, PERI_CLK_CTRL21, 0x52400000 | 9 | (3 << 4));
    assert_hz(qts, "timergrp0-tgrt", 0);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_RC32K, 1);
    assert_hz(qts, "rc32k", 32768);
    assert_hz(qts, "timergrp0-tgrt", 8192);
    qtest_writel(qts, PERI_CLK_CTRL21, 0x52400000 | 15);
    assert_hz(qts, "timergrp0-tgrt", 0);

    /* The LP clocks: LP_FAST_CLK from RC_FAST_CLK, LP_PERI_CLK by 2. */
    qtest_writel(qts, LP_CLK_CONF, (0 << 2) | (1 << 4));
    assert_hz(qts, "lp-fast", 20 * MHZ);
    assert_hz(qts, "lp-peri", 10 * MHZ);
    assert_hz(qts, "lp-i2cmst", 10 * MHZ);
    /* LP_SLOW_CLK from the 32 kHz crystal, which the PMU powers. */
    qtest_writel(qts, LP_CLK_CONF, 1);
    assert_hz(qts, "lp-slow", 0);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_XTAL32K, 1);
    assert_hz(qts, "lp-slow", 32768);
    /* The LP PLL from XTAL32K, and LP_FAST_CLK from it. */
    qtest_writel(qts, LP_CLK_CONF, (2 << 2) | (1 << 10));
    assert_hz(qts, "lp-fast", 0);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_XPD,
                     ESP32P4_CLKRST_XPD_LP_PLL, 1);
    assert_hz(qts, "lp-fast", 8 * MHZ);
    assert_hz(qts, "pll-lp", 8 * MHZ);
    /* LP UART's function clock from XTAL_D2_CLK. */
    qtest_writel(qts, LPPERI_CORE_CLK_SEL, 1u << 30);
    assert_hz(qts, "lp-uart-sclk", 20 * MHZ);
    clear_bits(qts, LPPERI_CLK_EN, 1u << 29);
    assert_hz(qts, "lp-uart-sclk", 0);
    assert_hz(qts, "lp-uart", 0);

    /* The LP core's clock runs when forced on. */
    assert_hz(qts, "lp-core", 0);
    set_bits(qts, LPPERI_CLK_EN, 1u << 31);
    assert_hz(qts, "lp-core", 8 * MHZ);

    /* A debug clock output: CPU_CLK (signal 26) divided by 4. */
    qtest_writel(qts, DBG_CLK_CTRL0, 0xffff00 | 26 | (3 << 24));
    qtest_writel(qts, DBG_CLK_CTRL1, 0x303 | (1 << 16));
    assert_hz(qts, "clkout0", 10 * MHZ);

    /* The SDIO low-speed clock: PLL_F160M_CLK, by the edge count's 4. */
    qtest_writel(qts, PERI_CLK_CTRL01, 0x401 | (1 << 24));
    qtest_writel(qts, PERI_CLK_CTRL02, (3 << 17) | (1 << 27));
    assert_hz(qts, "sdio-ls", 160 * MHZ);
    qtest_writel(qts, PERI_CLK_CTRL02, (3 << 17) | (1 << 27) | (1 << 8));
    assert_hz(qts, "sdio-ls", 40 * MHZ);
    assert_hz(qts, "sdio-slf", 40 * MHZ);
    assert_hz(qts, "sdio-drv", 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_cpll(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* The PLLs powered at reset calibrate, in 20 us. */
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0), ==, 0);
    qtest_clock_step(qts, 21 * US);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0), ==, 0x44);

    /* The CPU PLL's registers on I2C0, as ESP-IDF's regi2c has them. */
    qtest_writel(qts, ANA_CONF2, CONF2_CPU_PLL);
    g_assert_cmpuint(ana_read(qts, SLAVE_CPU_PLL, 3), ==, 9);
    g_assert_cmphex(ana_read(qts, SLAVE_CPU_PLL, 8), ==, 0xc0);

    /* ESP-IDF's sequence for 400 MHz: calibrate, then the dividers. */
    clear_bits(qts, ANA_PLL_CTRL0, 1 << 3);
    ana_write(qts, SLAVE_CPU_PLL, 2, (5 << 4) | 0);
    ana_write(qts, SLAVE_CPU_PLL, 3, 10);
    /*
     * Relocking, the PLL is stopped for 10 us; calibrating, CAL_END is
     * clear for 20 us. The read of its status takes 4 us.
     */
    assert_hz(qts, "cpll", 0);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & 4, ==, 0);
    g_assert_cmphex(ana_read(qts, SLAVE_CPU_PLL, 8), ==, 0);
    qtest_clock_step(qts, 8 * US);
    assert_hz(qts, "cpll", 400 * MHZ);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & 4, ==, 0);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & 4, ==, 4);
    g_assert_cmphex(ana_read(qts, SLAVE_CPU_PLL, 8), ==, 0xc0);
    set_bits(qts, ANA_PLL_CTRL0, 1 << 3);

    /*
     * Switch CPU_CLK to it as rtc_clk_cpu_freq_to_cpll_mhz() does:
     * APB_CLK, SYS_CLK, MEM_CLK and CPU_CLK dividers, then the source.
     */
    qtest_writel(qts, ROOT_CLK_CTRL2, 1 << 16);
    qtest_writel(qts, ROOT_CLK_CTRL0, 1 << 4);
    qtest_writel(qts, ROOT_CLK_CTRL1, 1);
    qtest_writel(qts, ROOT_CLK_CTRL0, 1 << 4);
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc | 1);
    assert_hz(qts, "cpu", 400 * MHZ);
    assert_hz(qts, "mem", 200 * MHZ);
    assert_hz(qts, "sys", 200 * MHZ);
    assert_hz(qts, "apb", 100 * MHZ);
    assert_cpu_hz(qts, HP_CPU0, 400 * MHZ);
    g_assert_cmpuint(qtest_readl(qts, CPU_SRC_FREQ0), ==, 1600);
    g_assert_cmphex(qtest_readl(qts, CPU_CLK_STATUS0), ==, 4);

    /* Releasing core 1's clock, it executes at CPU_CLK too. */
    set_bits(qts, SOC_CLK_CTRL0, 1 << 4);
    assert_cpu_hz(qts, HP_CPU1, 400 * MHZ);

    /* A change of a running PLL stops its clock until it relocks. */
    ana_write(qts, SLAVE_CPU_PLL, 3, 9);
    assert_hz(qts, "cpu", 0);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    qtest_clock_step(qts, 11 * US);
    assert_hz(qts, "cpu", 360 * MHZ);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));

    /* HP_CPLL_400M_CLK_EN gates it out of the HP system. */
    qtest_writel(qts, HP_CLK_CTRL, (0x1ffffffc | 1) & ~(1u << 26));
    assert_hz(qts, "cpu", 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_other_plls(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* The MSPI PLL, powered up by the PMU, at its reset 500 MHz. */
    qtest_set_irq_in(qts, ANA, "xpd", 4, 1);
    assert_hz(qts, "mpll", 0);
    qtest_clock_step(qts, 11 * US);
    assert_hz(qts, "mpll", 500 * MHZ);
    assert_hz(qts, "pll-f50m", 50 * MHZ);
    assert_hz(qts, "pll-f25m", 25 * MHZ);
    qtest_clock_step(qts, 10 * US);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & (1 << 8), ==, 1 << 8);
    /* 400 MHz, as ESP-IDF configures it: DIV 19, REF_DIV 1. */
    qtest_writel(qts, ANA_CONF2, CONF2_MSPI);
    ana_write(qts, SLAVE_MSPI, 2, (19 << 3) | 1);
    qtest_clock_step(qts, 11 * US);
    assert_hz(qts, "mpll", 400 * MHZ);
    qtest_writel(qts, HP_BASE + 0x30, 0x0000c03c | (1 << 12));
    assert_hz(qts, "psram-core", 400 * MHZ);

    /* The audio PLL: 40 MHz * (4 + 6) / 4, then with SDM1 128, + 1/2. */
    qtest_set_irq_in(qts, ANA, "xpd", 0, 1);
    qtest_writel(qts, ANA_CONF2, CONF2_PLLA);
    ana_write(qts, SLAVE_PLLA, 7, 6);
    ana_write(qts, SLAVE_PLLA, 8, 128);
    qtest_clock_step(qts, 11 * US);
    assert_hz(qts, "apll", 100 * MHZ);
    ana_write(qts, SLAVE_PLLA, 5, 0x49);
    qtest_clock_step(qts, 11 * US);
    assert_hz(qts, "apll", 105 * MHZ);
    /* Its calibration from the slave, as ESP-IDF runs it. */
    ana_write(qts, SLAVE_PLLA, 0, 0x0f);
    g_assert_cmphex(ana_read(qts, SLAVE_PLLA, 3), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & 1, ==, 0);
    ana_write(qts, SLAVE_PLLA, 0, 0x3f);
    ana_write(qts, SLAVE_PLLA, 0, 0x1f);
    qtest_clock_step(qts, 21 * US);
    g_assert_cmphex(ana_read(qts, SLAVE_PLLA, 3), ==, 0x80);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & 1, ==, 1);
    /* LP_CLKRST_HP_AUDIO_PLL_CLK_EN gates it out of the HP system. */
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc & ~(1u << 20));
    assert_hz(qts, "apll", 0);

    /* The SDIO PLL's 200 MHz, in three outputs. */
    qtest_set_irq_in(qts, ANA, "xpd", 2, 1);
    qtest_clock_step(qts, 11 * US);
    assert_hz(qts, "sdio-pll0", 200 * MHZ);
    assert_hz(qts, "sdio-pll2", 200 * MHZ);

    /* Powered down, a PLL stops and loses its calibration. */
    qtest_set_irq_in(qts, ANA, "xpd", 1, 0);
    assert_hz(qts, "cpll", 0);
    g_assert_cmphex(qtest_readl(qts, ANA_PLL_CTRL0) & 4, ==, 0);
    qtest_quit(qts);
}

/*
 * Every clock's period is its exact one rounded down once, however many
 * dividers lie between it and its source: PLL_F160M_CLK, three periods of
 * the 480 MHz SPLL_CLK, is exactly 6.25 ns, 26843545600 units, and the
 * others are their exact periods' whole units.
 */
/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_exact_periods(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    g_assert_cmpuint(clock_period(qts, CLKRST "/pll-f160m"), ==,
                     26843545600ull);
    assert_period(qts, "spll", 480 * MHZ, 1);
    assert_period(qts, "pll-f240m", 240 * MHZ, 1);
    assert_period(qts, "pll-f160m", 160 * MHZ, 1);
    assert_period(qts, "pll-f120m", 120 * MHZ, 1);
    assert_period(qts, "pll-f80m", 80 * MHZ, 1);
    assert_period(qts, "pll-f20m", 20 * MHZ, 1);
    assert_period(qts, "pll-f48m", 48 * MHZ, 1);
    assert_period(qts, "pll-f12m", 12 * MHZ, 1);
    assert_period(qts, "cpll", 360 * MHZ, 1);
    assert_period(qts, "xtal", 40 * MHZ, 1);
    assert_period(qts, "apb", 10 * MHZ, 1);
    assert_period(qts, "rc-slow", 150000, 1);
    assert_period(qts, "xtal-d2", 20 * MHZ, 1);

    /* Through the PLLs' users' dividers. */
    qtest_writel(qts, PERI_CLK_CTRL20, 2 | (1 << 2) | (1 << 3));
    assert_period(qts, "mcpwm0", 80 * MHZ, 1);
    qtest_writel(qts, PERI_CLK_CTRL110, (2 << 24) | (1 << 26));
    qtest_writel(qts, PERI_CLK_CTRL111, 1 | (1 << 8) | (2 << 16) |
                 0x04000000);
    assert_period(qts, "uart0-sclk", 32 * MHZ, 1);
    /* I2S0's TX clock from PLL_F160M_CLK, by 6 + 2/3: N 6, X 0, Y 1, Z 2. */
    qtest_writel(qts, PERI_CLK_CTRL13, (1 << 10) | (3 << 11) | (6 << 13) |
                 (0 << 21));
    qtest_writel(qts, PERI_CLK_CTRL14, 1 | (2 << 9));
    assert_period(qts, "i2s0-tx", 160 * MHZ * 3, 20);

    /*
     * CPU_CLK from the 360 MHz CPU PLL by 2 + 1/3, MEM_CLK from it by
     * 1 + 1/2 and APB_CLK from SYS_CLK by 3: 1080/7, 720/7 and 240/7 MHz.
     */
    qtest_writel(qts, ROOT_CLK_CTRL0, (1 << 5) | (1 << 13) | (3 << 21));
    qtest_writel(qts, ROOT_CLK_CTRL1, (1 << 8) | (2 << 16));
    qtest_writel(qts, ROOT_CLK_CTRL2, 2 << 16);
    qtest_writel(qts, ROOT_CLK_CTRL0, (1 << 5) | (1 << 13) | (3 << 21) |
                 (1 << 4));
    qtest_writel(qts, HP_CLK_CTRL, 0x1ffffffc | 1);
    assert_period(qts, "cpu", 1080 * MHZ, 7);
    assert_period(qts, "mem", 720 * MHZ, 7);
    assert_period(qts, "sys", 720 * MHZ, 7);
    assert_period(qts, "apb", 240 * MHZ, 7);
    assert_period(qts, "uart0-apb", 240 * MHZ, 7);

    /* RC_FAST_CLK at FOSC_DFREQ 101: 17.5 MHz + 1/300 of 2.5 MHz. */
    qtest_writel(qts, FOSC_CNTL, 101 << 22);
    assert_period(qts, "rc-fast", 17500000ull * 300 + 2500000, 300);

    /* The MSPI, audio and SDIO PLLs and theirs. */
    qtest_set_irq_in(qts, ANA, "xpd", 4, 1);
    qtest_set_irq_in(qts, ANA, "xpd", 0, 1);
    qtest_set_irq_in(qts, ANA, "xpd", 2, 1);
    qtest_writel(qts, ANA_CONF2, CONF2_PLLA);
    ana_write(qts, SLAVE_PLLA, 7, 6);
    ana_write(qts, SLAVE_PLLA, 8, 128);
    ana_write(qts, SLAVE_PLLA, 9, 1);
    ana_write(qts, SLAVE_PLLA, 5, 0x49);
    qtest_clock_step(qts, 11 * US);
    assert_period(qts, "mpll", 500 * MHZ, 1);
    assert_period(qts, "pll-f50m", 50 * MHZ, 1);
    assert_period(qts, "pll-f25m", 25 * MHZ, 1);
    /* 40 MHz * (4 + 6 + 128/256 + 1/65536) / 4 */
    assert_period(qts, "apll", 40 * MHZ * ((10ull << 16) + (128 << 8) + 1),
                  4 << 16);
    /*
     * I2S0's TX clock from it by 3 + 401/120708 (N 3, X 300, Y 7, Z 401):
     * a period of denominator near 2^40.
     */
    qtest_writel(qts, PERI_CLK_CTRL13, (1 << 10) | (1 << 11) | (3 << 13) |
                 (300u << 21));
    qtest_writel(qts, PERI_CLK_CTRL14, 7 | (401 << 9));
    assert_period(qts, "i2s0-tx",
                  40 * MHZ * ((10ull << 16) + (128 << 8) + 1) * 120708,
                  (4ull << 16) * (3 * 120708 + 401));
    assert_period(qts, "sdio-pll0", 200 * MHZ, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_ana_master(void)
{
    QTestState *qts = qtest_init("-M esp32p4");
    uint32_t v;

    /*
     * A write takes 29 bits of 4 LP_FAST_CLK cycles (SCL_PULSE_DUR 2,
     * SDA_SIDE_GUARD 1): 2.9 us at 40 MHz.
     */
    qtest_writel(qts, ANA_CONF2, CONF2_CPU_PLL);
    qtest_writel(qts, I2C0_CTRL, SLAVE_CPU_PLL | (4 << 8) | (0x12 << 16) |
                 CTRL_WRITE);
    g_assert_cmphex(qtest_readl(qts, I2C0_CTRL), ==, SLAVE_CPU_PLL |
                    (4 << 8) | (0x12 << 16) | CTRL_WRITE | CTRL_BUSY);
    /* A write while busy is dropped. */
    qtest_writel(qts, I2C0_CTRL, 0);
    qtest_clock_step(qts, 2800);
    g_assert(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY);
    qtest_clock_step(qts, 200);
    g_assert(!(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY));
    /* A read takes 39 bits, 3.9 us. */
    qtest_writel(qts, I2C0_CTRL, SLAVE_CPU_PLL | (4 << 8));
    qtest_clock_step(qts, 3800);
    g_assert(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY);
    qtest_clock_step(qts, 200);
    v = qtest_readl(qts, I2C0_CTRL);
    g_assert_cmphex(v, ==, SLAVE_CPU_PLL | (4 << 8) | (0x12 << 16));

    /* The bit time follows I2C0_CTRL1: 10 + 2 * 5 cycles. */
    qtest_writel(qts, I2C0_CTRL1, 10 | (5 << 6));
    qtest_writel(qts, I2C0_CTRL, SLAVE_CPU_PLL | (4 << 8));
    qtest_clock_step(qts, 19400);
    g_assert(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY);
    qtest_clock_step(qts, 200);
    g_assert(!(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY));
    /* And PLL_F160M_CLK with CLK_I2C_MST_SEL_160M: 4.875 us. */
    qtest_writel(qts, CLK160M, 1);
    qtest_writel(qts, I2C0_CTRL, SLAVE_CPU_PLL | (4 << 8));
    qtest_clock_step(qts, 4800);
    g_assert(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY);
    qtest_clock_step(qts, 100);
    g_assert(!(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY));
    qtest_writel(qts, CLK160M, 0);
    qtest_writel(qts, I2C0_CTRL1, 0x42);

    /* A slave on the other master, or none, does not answer. */
    qtest_writel(qts, ANA_CONF2, 0);
    g_assert_cmphex(ana_read(qts, SLAVE_CPU_PLL, 4), ==, 0);
    qtest_writel(qts, I2C1_CTRL, SLAVE_CPU_PLL | (4 << 8));
    qtest_clock_step(qts, 4 * US);
    g_assert_cmphex(qtest_readl(qts, I2C1_CTRL), ==, SLAVE_CPU_PLL |
                    (4 << 8) | (0x12 << 16));
    g_assert_cmphex(ana_read(qts, 0x55, 0), ==, 0);

    /* Its clock gated, the controller drops writes and does not run. */
    qtest_writel(qts, ANA_CONF2, CONF2_CPU_PLL);
    clear_bits(qts, LPPERI_CLK_EN, 1u << 27);
    assert_hz(qts, "lp-i2cmst", 0);
    qtest_writel(qts, ANA_CONF2, 0x123);
    g_assert_cmphex(qtest_readl(qts, ANA_CONF2), ==, CONF2_CPU_PLL);
    set_bits(qts, LPPERI_CLK_EN, 1u << 27);
    qtest_writel(qts, I2C0_CTRL, SLAVE_CPU_PLL | (4 << 8));
    clear_bits(qts, LPPERI_CLK_EN, 1u << 27);
    qtest_clock_step(qts, 10 * US);
    g_assert(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY);
    set_bits(qts, LPPERI_CLK_EN, 1u << 27);
    qtest_clock_step(qts, 4 * US);
    g_assert(!(qtest_readl(qts, I2C0_CTRL) & CTRL_BUSY));

    /* Held in reset, it reads its reset values and drops writes. */
    set_bits(qts, LPPERI_RESET_EN, 1u << 26);
    g_assert_cmphex(qtest_readl(qts, ANA_CONF2), ==, 0);
    qtest_writel(qts, ANA_CONF2, CONF2_CPU_PLL);
    g_assert_cmphex(qtest_readl(qts, ANA_CONF2), ==, 0);
    clear_bits(qts, LPPERI_RESET_EN, 1u << 26);
    qtest_writel(qts, ANA_CONF2, CONF2_CPU_PLL);
    g_assert_cmphex(qtest_readl(qts, ANA_CONF2), ==, CONF2_CPU_PLL);
    /* The PLLs' registers are the analog blocks', which keep them. */
    g_assert_cmphex(ana_read(qts, SLAVE_CPU_PLL, 4), ==, 0x12);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_block_resets(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_irq_intercept_out_named(qts, CLKRST, ESP32P4_CLKRST_RESET);

    set_bits(qts, HP_RST_EN1, 1 << 8);
    g_assert(qtest_get_irq(qts, ESP32P4_RST_UART0_CORE));
    g_assert(!qtest_get_irq(qts, ESP32P4_RST_UART0_APB));
    /* FORCE_NORST_UART0 overrides it, for both of UART0's resets. */
    set_bits(qts, HP_FORCE_NORST0, 1 << 25);
    g_assert(!qtest_get_irq(qts, ESP32P4_RST_UART0_CORE));
    clear_bits(qts, HP_FORCE_NORST0, 1 << 25);
    g_assert(qtest_get_irq(qts, ESP32P4_RST_UART0_CORE));
    clear_bits(qts, HP_RST_EN1, 1 << 8);
    g_assert(!qtest_get_irq(qts, ESP32P4_RST_UART0_CORE));

    /* RST_EN_CRYPTO holds every crypto block. */
    set_bits(qts, HP_RST_EN2, 1 << 14);
    g_assert(qtest_get_irq(qts, ESP32P4_RST_CRYPTO));
    g_assert(qtest_get_irq(qts, ESP32P4_RST_SHA));
    g_assert(qtest_get_irq(qts, ESP32P4_RST_AES));
    g_assert(!qtest_get_irq(qts, ESP32P4_RST_H264));
    clear_bits(qts, HP_RST_EN2, 1 << 14);
    g_assert(!qtest_get_irq(qts, ESP32P4_RST_SHA));

    /* LP_CLKRST's and LPPERI's. */
    qtest_writel(qts, LP_BASE + 0x4c, 1u << 30);
    g_assert(qtest_get_irq(qts, ESP32P4_RST_EMAC));
    qtest_writel(qts, LP_BASE + 0x4c, (1u << 30) | (1u << 31));
    g_assert(!qtest_get_irq(qts, ESP32P4_RST_EMAC));
    set_bits(qts, LPPERI_RESET_EN, 1u << 28);
    g_assert(qtest_get_irq(qts, ESP32P4_RST_LP_UART));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_core_resets(void)
{
    QTestState *qts = qtest_init("-M esp32p4");
    uint32_t cause;

    /* Out of chip reset core 1 is held with its clock gated. */
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    set_bits(qts, SOC_CLK_CTRL0, 1 << 4);
    clear_bits(qts, HP_RST_EN0, 1 << 8);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));

    /* FORCE_NORST_CORE1 keeps it out of reset. */
    set_bits(qts, HP_FORCE_NORST0, 1 << 1);
    set_bits(qts, HP_RST_EN0, 1 << 8);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    qtest_writel(qts, HP_FORCE_NORST0, 0);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    clear_bits(qts, HP_RST_EN0, 1 << 8);

    /*
     * A software CPU reset: stall for STALL_WAIT, then reset for
     * RESET_LENGTH cycles of LP_DYN_FAST_CLK (40 MHz).
     */
    qtest_writel(qts, RESET_CAUSE, 0xfe000000);
    qtest_writel(qts, HPCPU_RESET_CTRL0, 0x80008002 | (4u << 17) |
                 (40u << 21) | (1u << 28) | (1u << 29));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "clock-running"));
    qtest_clock_step(qts, 1000);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    qtest_clock_step(qts, 100);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    cause = qtest_readl(qts, RESET_CAUSE);
    g_assert_cmphex(cause & 0x001fffff, ==, (0x0c << 14) | (1 << 20));
    /* The cause and flag clear separately. */
    qtest_writel(qts, RESET_CAUSE, (1u << 31) | (1u << 25));
    g_assert_cmphex(qtest_readl(qts, RESET_CAUSE), ==,
                    (1u << 25) | (0x0c << 14));
    qtest_writel(qts, RESET_CAUSE, (1u << 30) | (1u << 25));
    g_assert_cmphex(qtest_readl(qts, RESET_CAUSE), ==, 0x02000000);

    /* An MWDT CPU reset of the core HPCORE_WDT_RESET_SOURCE_SEL picks. */
    qtest_writel(qts, HPWDT_CORE1_RST_CTRL0, 4 << 9);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_MWDT_CORE_RESET,
                     ESP32P4_CLKRST_MWDT_LINE(1, 1), 1);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_MWDT_CORE_RESET,
                     ESP32P4_CLKRST_MWDT_LINE(1, 1), 0);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    qtest_clock_step(qts, 100);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert_cmphex((qtest_readl(qts, RESET_CAUSE) >> 14) & 0x3f, ==, 0x0b);

    /* The RTC watchdog's, where LP_WDT_HPCORE<n>_RESET_EN is set. */
    qtest_writel(qts, HPCPU_RESET_CTRL0, 0x80028002 | (1 << 4));
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_RWDT_CORE_RESET, 0, 1);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_RWDT_CORE_RESET, 0, 0);
    g_assert(qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    qtest_clock_step(qts, 100);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU0, "in-reset"));
    g_assert_cmphex((qtest_readl(qts, RESET_CAUSE) >> 7) & 0x7f, ==,
                    0x0d | 0x40);

    /* The PMU's stall. */
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 1);
    g_assert(!qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_CORE_STALL, 1, 0);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "clock-running"));

    /* STAT_VECTOR_SEL: the HP ROM, or LP SRAM. */
    g_assert_cmphex(qom_get_uint(qts, HP_CPU1, "resetvec"), ==,
                    0x4fc00000);
    qtest_writel(qts, HPCPU_RESET_CTRL0, 0x00028002);
    g_assert_cmphex(qom_get_uint(qts, HP_CPU1, "resetvec"), ==,
                    0x50108000);
    g_assert_cmphex(qom_get_uint(qts, HP_CPU0, "resetvec"), ==,
                    0x4fc00000);

    /* The LP core, which the PMU holds, is released by it. */
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_LP_CORE_RESET, 0, 0);
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "in-reset"));
    g_assert(!qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_PMU_LP_CORE_CLOCK, 0, 1);
    g_assert(qtest_qom_get_bool(qts, LP_CPU, "clock-running"));
    assert_cpu_hz(qts, LP_CPU, 40 * MHZ);
    qtest_writel(qts, LPPERI_RESET_EN, 1u << 31);
    g_assert_cmphex(qtest_readl(qts, RESET_CAUSE) & 0x7f, ==, 0x14 | 0x40);
    g_assert_cmphex(qtest_readl(qts, LPPERI_RESET_EN), ==, 0);

    /* A system reset holds core 1 again and records a chip reset. */
    qtest_system_reset(qts);
    g_assert(qtest_qom_get_bool(qts, HP_CPU1, "in-reset"));
    g_assert_cmphex(qtest_readl(qts, RESET_CAUSE), ==, 0x021060c1);
    g_assert_cmphex(qom_get_uint(qts, HP_CPU1, "resetvec"), ==,
                    0x4fc00000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_lp_uart_wakeup(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_LP_UART_WAKEUP, 0, 1);
    g_assert_cmphex(qtest_readl(qts, LPPERI_MEM_CTRL), ==, 0x80000000);
    qtest_writel(qts, LPPERI_MEM_CTRL, 0x80000000 | (1u << 29));
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_LP_UART_WAKEUP, 0, 0);
    qtest_set_irq_in(qts, CLKRST, ESP32P4_CLKRST_LP_UART_WAKEUP, 0, 1);
    g_assert_cmphex(qtest_readl(qts, LPPERI_MEM_CTRL), ==,
                    0x80000000 | (1u << 29) | 2);
    qtest_writel(qts, LPPERI_MEM_CTRL, 0x80000000 | (1u << 29) | 1);
    g_assert_cmphex(qtest_readl(qts, LPPERI_MEM_CTRL), ==,
                    0x80000000 | (1u << 29));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32p4.clocks/test] */
static void test_efuse(void)
{
    QTestState *qts = qtest_init("-M esp32p4");

    /* The controller's 20 MHz: XTAL_D2_CLK, not LP_FAST_CLK's. */
    assert_hz(qts, "efuse", 20 * MHZ);
    g_assert_cmpuint(path_hz(qts, "/machine/efuse/clk"), ==, 20 * MHZ);
    qtest_writel(qts, LP_CLK_CONF, 0);
    assert_hz(qts, "efuse", 20 * MHZ);
    assert_hz(qts, "lp-efuse", 20 * MHZ);
    qtest_writel(qts, LP_CLK_CONF, 4);

    /* A read stalls while LPPERI_CK_EN_LP_EFUSE gates the clock. */
    qtest_writel(qts, EFUSE_CONF, EFUSE_READ_OP_CODE);
    qtest_writel(qts, EFUSE_CMD, EFUSE_CMD_READ);
    qtest_clock_step(qts, 10 * US);
    clear_bits(qts, LPPERI_CLK_EN, 1u << 19);
    assert_hz(qts, "efuse", 0);
    g_assert_cmpuint(path_hz(qts, "/machine/efuse/clk"), ==, 0);
    qtest_clock_step(qts, EFUSE_READ_NS);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CMD), ==, EFUSE_CMD_READ);
    /* Its registers, behind the gated bus clock, drop writes. */
    qtest_writel(qts, EFUSE_CONF, 0);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CONF), ==, EFUSE_READ_OP_CODE);
    set_bits(qts, LPPERI_CLK_EN, 1u << 19);
    qtest_clock_step(qts, EFUSE_READ_NS - 10 * US);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CMD), ==, 0);

    /* LPPERI_RST_EN_LP_EFUSE holds it; released, it loads the array. */
    set_bits(qts, LPPERI_RESET_EN, 1u << 20);
    g_assert_cmpuint(qtest_readl(qts, EFUSE_STATUS) & 0xf, ==,
                     EFUSE_STATE_RESET);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CONF), ==, 0);
    qtest_writel(qts, EFUSE_CONF, EFUSE_READ_OP_CODE);
    g_assert_cmphex(qtest_readl(qts, EFUSE_CONF), ==, 0);
    clear_bits(qts, LPPERI_RESET_EN, 1u << 20);
    g_assert_cmpuint(qtest_readl(qts, EFUSE_STATUS) & 0xf, !=,
                     EFUSE_STATE_IDLE);
    qtest_clock_step(qts, EFUSE_READ_NS);
    g_assert_cmpuint(qtest_readl(qts, EFUSE_STATUS) & 0xf, ==,
                     EFUSE_STATE_IDLE);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/clocks/registers", test_registers);
    qtest_add_func("/esp32p4/clocks/byte-access", test_byte_access);
    qtest_add_func("/esp32p4/clocks/tree-at-reset", test_tree_at_reset);
    qtest_add_func("/esp32p4/clocks/soc-clocks", test_soc_clocks);
    qtest_add_func("/esp32p4/clocks/block-clocks", test_block_clocks);
    qtest_add_func("/esp32p4/clocks/cpll", test_cpll);
    qtest_add_func("/esp32p4/clocks/other-plls", test_other_plls);
    qtest_add_func("/esp32p4/clocks/exact-periods", test_exact_periods);
    qtest_add_func("/esp32p4/clocks/ana-master", test_ana_master);
    qtest_add_func("/esp32p4/clocks/block-resets", test_block_resets);
    qtest_add_func("/esp32p4/clocks/core-resets", test_core_resets);
    qtest_add_func("/esp32p4/clocks/lp-uart-wakeup", test_lp_uart_wakeup);
    qtest_add_func("/esp32p4/clocks/efuse", test_efuse);
    return g_test_run();
}
