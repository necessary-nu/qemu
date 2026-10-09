/*
 * QTest testcase for the RP2350 programmable I/O blocks (PIO0-PIO2)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * clk_sys is 150 MHz: 20 ns is exactly three cycles. Timings are checked
 * to the cycle by stepping virtual time, stopping the state machine and
 * reading back a register that counted its cycles. Pins are driven from
 * outside through the GPIO block's "pad-in" lines.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define ALIAS_XOR       0x1000
#define ALIAS_SET       0x2000
#define ALIAS_CLR       0x3000

#define PIO_BASE(b)     (0x50200000 + 0x100000 * (b))
#define CTRL            0x000
#define FSTAT           0x004
#define FDEBUG          0x008
#define FLEVEL          0x00c
#define TXF(i)          (0x010 + 4 * (i))
#define RXF(i)          (0x020 + 4 * (i))
#define IRQ             0x030
#define IRQ_FORCE       0x034
#define SYNC_BYPASS     0x038
#define DBG_PADOUT      0x03c
#define DBG_PADOE       0x040
#define DBG_CFGINFO     0x044
#define INSTR_MEM(n)    (0x048 + 4 * (n))
#define SM_REG(i, r)    (0x0c8 + 0x18 * (i) + (r))
#define CLKDIV(i)       SM_REG(i, 0x00)
#define EXECCTRL(i)     SM_REG(i, 0x04)
#define SHIFTCTRL(i)    SM_REG(i, 0x08)
#define ADDR(i)         SM_REG(i, 0x0c)
#define INSTR(i)        SM_REG(i, 0x10)
#define PINCTRL(i)      SM_REG(i, 0x14)
#define PUTGET(i, n)    (0x128 + 0x10 * (i) + 4 * (n))
#define GPIOBASE        0x168
#define INTR            0x16c
#define IRQ0_INTE       0x170
#define IRQ0_INTF       0x174
#define IRQ0_INTS       0x178
#define IRQ1_INTE       0x17c
#define IRQ1_INTF       0x180
#define IRQ1_INTS       0x184

#define CTRL_EN(m)          (m)
#define CTRL_RESTART(m)     ((m) << 4)
#define CTRL_CLKDIV_RST(m)  ((m) << 8)
#define CTRL_PREV(m)        ((m) << 16)
#define CTRL_NEXT(m)        ((m) << 20)
#define CTRL_NP_ENABLE      (1u << 24)
#define CTRL_NP_DISABLE     (1u << 25)
#define CTRL_NP_CLKDIV      (1u << 26)

#define EXEC_STALLED        (1u << 31)
#define EXEC_SIDE_EN        (1u << 30)
#define EXEC_SIDE_PINDIR    (1u << 29)
#define EXEC_JMP_PIN(p)     ((p) << 24)
#define EXEC_OUT_EN_SEL(n)  ((n) << 19)
#define EXEC_INLINE_OUT_EN  (1u << 18)
#define EXEC_OUT_STICKY     (1u << 17)
#define EXEC_WRAP(b, t)     (((t) << 12) | ((b) << 7))
#define EXEC_STATUS(s, n)   (((s) << 5) | (n))

#define SHIFT_FJOIN_RX      (1u << 31)
#define SHIFT_FJOIN_TX      (1u << 30)
#define SHIFT_PULL(n)       (((n) & 31) << 25)
#define SHIFT_PUSH(n)       (((n) & 31) << 20)
#define SHIFT_OUT_RIGHT     (1u << 19)
#define SHIFT_IN_RIGHT      (1u << 18)
#define SHIFT_AUTOPULL      (1u << 17)
#define SHIFT_AUTOPUSH      (1u << 16)
#define SHIFT_RX_PUT        (1u << 15)
#define SHIFT_RX_GET        (1u << 14)
#define SHIFT_IN_COUNT(n)   (n)

#define PIN_SIDESET(n, base) (((n) << 29) | ((base) << 10))
#define PIN_SET(n, base)    (((n) << 26) | ((base) << 5))
#define PIN_OUT(n, base)    (((n) << 20) | (base))
#define PIN_IN(base)        ((base) << 15)

/* Instruction encodings. */
#define I_JMP(cond, a)      (0x0000 | (cond) << 5 | (a))
#define I_WAIT(pol, src, n) (0x2000 | (pol) << 7 | (src) << 5 | (n))
#define I_IN(src, n)        (0x4000 | (src) << 5 | ((n) & 31))
#define I_OUT(dst, n)       (0x6000 | (dst) << 5 | ((n) & 31))
#define I_PUSH(iff, blk)    (0x8000 | (iff) << 6 | (blk) << 5)
#define I_PULL(ife, blk)    (0x8080 | (ife) << 6 | (blk) << 5)
#define I_PUT(idx)          (0x8018 | (idx))
#define I_GET(idx)          (0x8098 | (idx))
#define I_MOV(dst, op, src) (0xa000 | (dst) << 5 | (op) << 3 | (src))
#define I_IRQ(clr, w, n)    (0xc000 | (clr) << 6 | (w) << 5 | (n))
#define I_SET(dst, v)       (0xe000 | (dst) << 5 | (v))
#define DLY(d)              ((d) << 8)
#define NOP                 I_MOV(2, 0, 2)
/* A NOP whose SMx_INSTR write lets the blocks run one cycle. */
#define I_NOP_CYCLE         NOP

enum { C_ALWAYS, C_NX, C_XDEC, C_NY, C_YDEC, C_XNEY, C_PIN, C_NOSRE };
enum { S_PINS, S_X, S_Y, S_NULL, S_STATUS = 5, S_ISR, S_OSR };
enum { D_PINS, D_X, D_Y, D_NULL, D_PINDIRS, D_PC, D_ISR, D_OSR };
enum { MD_PINS, MD_X, MD_Y, MD_PINDIRS, MD_EXEC, MD_PC, MD_ISR, MD_OSR };
enum { W_GPIO, W_PIN, W_IRQ, W_JMPPIN };
#define IRQ_PREV            (1u << 3)
#define IRQ_REL             (2u << 3)
#define IRQ_NEXT            (3u << 3)

#define IO_BANK0        0x40028000
#define PADS_BANK0      0x40038000
#define GPIO_STATUS(p)  (IO_BANK0 + 8 * (p))
#define GPIO_CTRL(p)    (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define STATUS_OUTTOPAD (1u << 9)
#define STATUS_OETOPAD  (1u << 13)
#define PAD_IE          (1u << 6)
#define FUNC_PIO(b)     (6 + (b))
#define SIO_GPIO_IN     0xd0000004

#define PIO0_IRQ_0      15

#define ACCESSCTRL_PIO(b)   (0x40060000 + 0x4c + 4 * (b))
#define ACCESSCTRL_GPIO_NSMASK0 0x4006000c

#define RESETS_RESET_SET    0x40022000
#define RESET_PIO(b)        (1u << (11 + (b)))
#define RESET_PIO_ALL       (7u << 11)

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    return qts;
}

static uint32_t rd(QTestState *qts, int b, uint32_t reg)
{
    return qtest_readl(qts, PIO_BASE(b) + reg);
}

static void wr(QTestState *qts, int b, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, PIO_BASE(b) + reg, v);
}

/*
 * Step virtual time by n clk_sys cycles; n must be a multiple of 3. Each
 * SMx_INSTR write runs the cycle its instruction executes in at once, so
 * the blocks may be a few cycles ahead of virtual time; stepping lets
 * time catch up with them.
 */
static void cycles(QTestState *qts, int n)
{
    g_assert(n % 3 == 0);
    qtest_clock_step(qts, n / 3 * 20);
}

static void load(QTestState *qts, int b, int at, const uint16_t *prog, int n)
{
    int k;

    for (k = 0; k < n; k++) {
        wr(qts, b, INSTR_MEM(at + k), prog[k]);
    }
}

static void exec(QTestState *qts, int b, int i, uint16_t instr)
{
    wr(qts, b, INSTR(i), instr);
}

/* Read a scratch register of a stopped state machine through its FIFO. */
static uint32_t read_reg(QTestState *qts, int b, int i, int src)
{
    exec(qts, b, i, I_MOV(MD_ISR, 0, src));
    exec(qts, b, i, I_PUSH(0, 0));
    return rd(qts, b, RXF(i));
}

static void pin_pio(QTestState *qts, int b, int p)
{
    qtest_writel(qts, PAD(p), PAD_IE);
    qtest_writel(qts, GPIO_CTRL(p), FUNC_PIO(b));
}

static void drive(QTestState *qts, int p, int level)
{
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", p, level);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    int b, i, n;

    for (b = 0; b < 3; b++) {
        g_assert_cmphex(rd(qts, b, CTRL), ==, 0);
        g_assert_cmphex(rd(qts, b, FSTAT), ==, 0x0f000f00);
        g_assert_cmphex(rd(qts, b, FDEBUG), ==, 0);
        g_assert_cmphex(rd(qts, b, FLEVEL), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ_FORCE), ==, 0);
        g_assert_cmphex(rd(qts, b, SYNC_BYPASS), ==, 0);
        g_assert_cmphex(rd(qts, b, DBG_PADOUT), ==, 0);
        g_assert_cmphex(rd(qts, b, DBG_PADOE), ==, 0);
        g_assert_cmphex(rd(qts, b, DBG_CFGINFO), ==, 0x10200404);
        for (n = 0; n < 32; n++) {
            g_assert_cmphex(rd(qts, b, INSTR_MEM(n)), ==, 0);
        }
        for (i = 0; i < 4; i++) {
            g_assert_cmphex(rd(qts, b, CLKDIV(i)), ==, 0x00010000);
            g_assert_cmphex(rd(qts, b, EXECCTRL(i)), ==, 0x0001f000);
            g_assert_cmphex(rd(qts, b, SHIFTCTRL(i)), ==, 0x000c0000);
            g_assert_cmphex(rd(qts, b, ADDR(i)), ==, 0);
            g_assert_cmphex(rd(qts, b, PINCTRL(i)), ==, 0x14000000);
            for (n = 0; n < 4; n++) {
                g_assert_cmphex(rd(qts, b, PUTGET(i, n)), ==, 0);
            }
        }
        g_assert_cmphex(rd(qts, b, GPIOBASE), ==, 0);
        /* Every TX FIFO has room. */
        g_assert_cmphex(rd(qts, b, INTR), ==, 0xf0);
        g_assert_cmphex(rd(qts, b, IRQ0_INTE), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ0_INTF), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ0_INTS), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ1_INTE), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ1_INTF), ==, 0);
        g_assert_cmphex(rd(qts, b, IRQ1_INTS), ==, 0);
    }

    /* Reserved and read-only bits ignore writes. */
    wr(qts, 0, CLKDIV(1), 0xffffffff);
    g_assert_cmphex(rd(qts, 0, CLKDIV(1)), ==, 0xffffff00);
    wr(qts, 0, EXECCTRL(1), 0xffffffff);
    g_assert_cmphex(rd(qts, 0, EXECCTRL(1)), ==, 0x7fffffff);
    wr(qts, 0, SHIFTCTRL(1), 0x3fffffff);
    g_assert_cmphex(rd(qts, 0, SHIFTCTRL(1)), ==, 0x3fffc01f);
    /* The random-access RX modes clear the joins. */
    wr(qts, 0, SHIFTCTRL(1), 0xffffffff);
    g_assert_cmphex(rd(qts, 0, SHIFTCTRL(1)), ==, 0x3fffc01f);
    wr(qts, 0, GPIOBASE, 0xffffffff);
    g_assert_cmphex(rd(qts, 0, GPIOBASE), ==, 0x10);
    wr(qts, 0, IRQ0_INTE, 0xffffffff);
    g_assert_cmphex(rd(qts, 0, IRQ0_INTE), ==, 0xffff);
    /* Instruction memory is write-only. */
    wr(qts, 0, INSTR_MEM(3), 0x1234);
    g_assert_cmphex(rd(qts, 0, INSTR_MEM(3)), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_fifos(void)
{
    QTestState *qts = start();
    int n;

    for (n = 0; n < 4; n++) {
        wr(qts, 1, TXF(2), 0x100 + n);
        g_assert_cmphex(rd(qts, 1, FLEVEL), ==, (n + 1) << 16);
    }
    g_assert_cmphex(rd(qts, 1, FSTAT), ==, 0x0b040f00);
    g_assert_cmphex(rd(qts, 1, INTR), ==, 0xb0);
    wr(qts, 1, TXF(2), 0xdead);
    g_assert_cmphex(rd(qts, 1, FDEBUG), ==, 1u << 18);
    g_assert_cmphex(rd(qts, 1, FLEVEL), ==, 4 << 16);
    /* Write 1 to clear; the XOR and CLR aliases leave the flags alone. */
    wr(qts, 1, FDEBUG + ALIAS_XOR, 1u << 18);
    wr(qts, 1, FDEBUG + ALIAS_CLR, 1u << 18);
    g_assert_cmphex(rd(qts, 1, FDEBUG), ==, 1u << 18);
    wr(qts, 1, FDEBUG, 1u << 18);
    g_assert_cmphex(rd(qts, 1, FDEBUG), ==, 0);

    /* The state machine drains the FIFO in order. */
    for (n = 0; n < 4; n++) {
        exec(qts, 1, 2, I_PULL(0, 0));
        exec(qts, 1, 2, I_MOV(MD_ISR, 0, S_OSR));
        exec(qts, 1, 2, I_PUSH(0, 0));
        g_assert_cmphex(rd(qts, 1, RXF(2)), ==, 0x100 + n);
    }
    rd(qts, 1, RXF(2));
    g_assert_cmphex(rd(qts, 1, FDEBUG), ==, 1u << 10);
    /* A non-blocking pull from an empty FIFO copies X. */
    exec(qts, 1, 2, I_SET(D_X, 19));
    exec(qts, 1, 2, I_PULL(0, 0));
    g_assert_cmpuint(read_reg(qts, 1, 2, S_OSR), ==, 19);

    /* Joined TX: eight deep, RX unavailable (both full and empty). */
    wr(qts, 0, SHIFTCTRL(1), SHIFT_FJOIN_TX);
    for (n = 0; n < 8; n++) {
        wr(qts, 0, TXF(1), n);
    }
    g_assert_cmphex(rd(qts, 0, FLEVEL), ==, 8 << 8);
    g_assert_cmphex(rd(qts, 0, FSTAT), ==, 0x0d020f02);
    /* Changing the join flushes the FIFOs. */
    wr(qts, 0, SHIFTCTRL(1), SHIFT_FJOIN_RX);
    g_assert_cmphex(rd(qts, 0, FLEVEL), ==, 0);
    g_assert_cmphex(rd(qts, 0, FSTAT), ==, 0x0f020f00);
    for (n = 0; n < 8; n++) {
        exec(qts, 0, 1, I_SET(D_X, n));
        exec(qts, 0, 1, I_MOV(MD_ISR, 0, S_X));
        exec(qts, 0, 1, I_PUSH(0, 0));
    }
    g_assert_cmphex(rd(qts, 0, FLEVEL), ==, 8 << 12);
    g_assert_cmphex(rd(qts, 0, FSTAT), ==, 0x0f020d02);
    /* A non-blocking push to a full FIFO drops the data and says so. */
    exec(qts, 0, 1, I_PUSH(0, 0));
    g_assert_cmphex(rd(qts, 0, FDEBUG), ==, 1u << 1);
    for (n = 0; n < 8; n++) {
        g_assert_cmphex(rd(qts, 0, RXF(1)), ==, n);
    }
    /* Narrow writes are replicated; narrow reads pop one entry. */
    wr(qts, 0, SHIFTCTRL(1), 0);
    qtest_writeb(qts, PIO_BASE(0) + TXF(1), 0xa5);
    exec(qts, 0, 1, I_PULL(0, 0));
    exec(qts, 0, 1, I_MOV(MD_ISR, 0, S_OSR));
    exec(qts, 0, 1, I_PUSH(0, 0));
    g_assert_cmphex(qtest_readb(qts, PIO_BASE(0) + RXF(1) + 3), ==, 0xa5);
    g_assert_cmphex(rd(qts, 0, FLEVEL), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_forced_exec(void)
{
    QTestState *qts = start();

    exec(qts, 2, 3, I_JMP(C_ALWAYS, 7));
    g_assert_cmpuint(rd(qts, 2, ADDR(3)), ==, 7);
    wr(qts, 2, INSTR_MEM(7), 0xe03f);
    g_assert_cmphex(rd(qts, 2, INSTR(3)), ==, 0xe03f);

    /* A forced instruction that stalls is latched until it completes. */
    exec(qts, 2, 3, I_PULL(0, 1));
    g_assert_cmphex(rd(qts, 2, EXECCTRL(3)), ==, EXEC_STALLED | 0x1f000);
    g_assert_cmphex(rd(qts, 2, FDEBUG), ==, 1u << 27);
    cycles(qts, 30);
    g_assert_cmphex(rd(qts, 2, EXECCTRL(3)), ==, EXEC_STALLED | 0x1f000);
    wr(qts, 2, TXF(3), 0x5a5a);
    cycles(qts, 3);
    g_assert_cmphex(rd(qts, 2, EXECCTRL(3)), ==, 0x1f000);
    g_assert_cmphex(read_reg(qts, 2, 3, S_OSR), ==, 0x5a5a);
    /* The program counter did not move. */
    g_assert_cmpuint(rd(qts, 2, ADDR(3)), ==, 7);

    /* SM_RESTART drops a stalled instruction. */
    exec(qts, 2, 3, I_WAIT(1, W_IRQ, 5));
    g_assert_cmphex(rd(qts, 2, EXECCTRL(3)), ==, EXEC_STALLED | 0x1f000);
    wr(qts, 2, CTRL + ALIAS_SET, CTRL_RESTART(1u << 3));
    g_assert_cmphex(rd(qts, 2, EXECCTRL(3)), ==, 0x1f000);
    g_assert_cmphex(rd(qts, 2, CTRL), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_execution_timing(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        I_JMP(C_YDEC, 0),
    };

    load(qts, 0, 0, prog, 1);
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(0, 0));
    cycles(qts, 3);
    /* One instruction per cycle from the cycle of the enable. */
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 300);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-300);

    /* Delay cycles: each loop takes 1 + 4 cycles. */
    wr(qts, 0, INSTR_MEM(0), I_JMP(C_YDEC, 0) | DLY(4));
    exec(qts, 0, 0, I_SET(D_Y, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 150);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-30);

    /* Program wrapping costs no cycles. */
    wr(qts, 0, INSTR_MEM(0), I_JMP(C_YDEC, 1));
    wr(qts, 0, INSTR_MEM(1), I_JMP(C_XDEC, 0));
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(0, 1));
    exec(qts, 0, 0, I_SET(D_Y, 0));
    exec(qts, 0, 0, I_SET(D_X, 0));
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 60);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-30);
    g_assert_cmphex(read_reg(qts, 0, 0, S_X), ==, (uint32_t)-30);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_clock_divider(void)
{
    QTestState *qts = start();

    wr(qts, 0, INSTR_MEM(0), I_JMP(C_YDEC, 0));
    wr(qts, 0, INSTR_MEM(1), I_JMP(C_XDEC, 1));
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(0, 0));
    wr(qts, 0, EXECCTRL(1), EXEC_WRAP(1, 1));
    /* SM0 at 2.5, SM1 at 3; restarted together with their enables. */
    wr(qts, 0, CLKDIV(0), (2 << 16) | (128 << 8));
    wr(qts, 0, CLKDIV(1), 3 << 16);
    exec(qts, 0, 1, I_JMP(C_ALWAYS, 1));
    cycles(qts, 3);
    wr(qts, 0, CTRL, CTRL_EN(3) | CTRL_CLKDIV_RST(3));
    cycles(qts, 300);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-120);
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, (uint32_t)-100);

    /*
     * Dividers run free while their state machines are disabled. SM1's is
     * restarted a cycle after SM0's (the SMx_INSTR write to SM2 takes a
     * cycle), so when both are enabled together SM0 runs first.
     */
    wr(qts, 0, CLKDIV(0), 3 << 16);
    exec(qts, 0, 0, I_SET(D_Y, 0));
    exec(qts, 0, 1, I_SET(D_X, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_CLKDIV_RST(1));
    exec(qts, 0, 2, NOP);
    wr(qts, 0, CTRL, CTRL_CLKDIV_RST(2));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(3));
    exec(qts, 0, 2, NOP);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-1);
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0);
    exec(qts, 0, 0, I_SET(D_Y, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(3));
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-10);
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, (uint32_t)-10);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_input_sync(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        I_WAIT(1, W_GPIO, 5),
        I_JMP(C_YDEC, 1),
    };

    qtest_writel(qts, PAD(5), PAD_IE);
    drive(qts, 5, 0);
    load(qts, 0, 0, prog, 2);
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(1, 1));
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 30);
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 0);
    /* Seen two cycles after it reaches the pad: Y counts from cycle 3. */
    drive(qts, 5, 1);
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-27);

    /* With the synchroniser bypassed, seen at once. */
    drive(qts, 5, 0);
    wr(qts, 0, SYNC_BYPASS, 1u << 5);
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    exec(qts, 0, 0, I_SET(D_Y, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 30);
    drive(qts, 5, 1);
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-29);

    /* IN reads pins rotated to IN_BASE and masked to IN_COUNT. */
    qtest_writel(qts, PAD(6), PAD_IE);
    drive(qts, 6, 1);
    cycles(qts, 30);
    wr(qts, 0, PINCTRL(1), PIN_IN(4));
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_PINS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0x6);
    wr(qts, 0, SHIFTCTRL(1), 0xc0000 | SHIFT_IN_COUNT(2));
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_PINS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0x2);
    exec(qts, 0, 1, I_IN(S_PINS, 32));
    exec(qts, 0, 1, I_PUSH(0, 0));
    g_assert_cmphex(rd(qts, 0, RXF(1)), ==, 0x2);
    /* GPIOBASE 16 shifts the pins: GPIO 21 is pin 5. */
    wr(qts, 1, GPIOBASE, 16);
    qtest_writel(qts, PAD(21), PAD_IE);
    drive(qts, 21, 1);
    cycles(qts, 30);
    exec(qts, 1, 0, I_MOV(MD_X, 0, S_PINS));
    g_assert_cmphex(read_reg(qts, 1, 0, S_X), ==, 1u << 5);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_outputs(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        /* .side_set 1 opt: side-set on GPIO 4, SET on GPIO 2-3 */
        I_SET(D_PINDIRS, 3) | 0x1800,
        I_SET(D_PINS, 1) | DLY(2),
        I_SET(D_PINS, 2) | 0x1000 | DLY(2),
    };

    pin_pio(qts, 0, 2);
    pin_pio(qts, 0, 3);
    pin_pio(qts, 0, 4);
    load(qts, 0, 0, prog, 3);
    wr(qts, 0, PINCTRL(0), PIN_SIDESET(2, 4) | PIN_SET(2, 2));
    wr(qts, 0, EXECCTRL(0), EXEC_SIDE_EN | EXEC_SIDE_PINDIR | EXEC_WRAP(1, 2));
    cycles(qts, 3);
    wr(qts, 0, CTRL, CTRL_EN(1));
    /* Cycle 0: pindirs and side 1; 1: pins 01; 4: pins 10 and side 0. */
    cycles(qts, 3);
    g_assert_cmphex(rd(qts, 0, DBG_PADOE), ==, 0x1c);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0x04);
    g_assert_true(qtest_readl(qts, GPIO_STATUS(2)) & STATUS_OUTTOPAD);
    g_assert_true(qtest_readl(qts, GPIO_STATUS(4)) & STATUS_OETOPAD);
    cycles(qts, 3);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0x08);
    /* The side-set writes pin directions. */
    g_assert_cmphex(rd(qts, 0, DBG_PADOE), ==, 0x0c);
    g_assert_cmphex(qtest_readl(qts, SIO_GPIO_IN) & 0x1c, ==, 0x08);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_autopush_autopull(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        I_OUT(D_X, 8),
        I_IN(S_X, 4),
    };
    static const uint16_t loop[] = {
        I_OUT(D_X, 32),
        I_IN(S_X, 32),
    };
    int n;

    /* OSR shifts right by bytes; ISR shifts left by nibbles. */
    load(qts, 0, 0, prog, 2);
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(0, 1));
    wr(qts, 0, SHIFTCTRL(0), SHIFT_OUT_RIGHT | SHIFT_AUTOPULL | SHIFT_PULL(32) |
                             SHIFT_AUTOPUSH | SHIFT_PUSH(16));
    wr(qts, 0, TXF(0), 0x87654321);
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 30);
    g_assert_cmphex(rd(qts, 0, RXF(0)), ==, 0x1357);
    g_assert_cmphex(rd(qts, 0, FLEVEL), ==, 0);
    /* Stalled on the OUT with the TX FIFO empty. */
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 0);
    g_assert_cmphex(rd(qts, 0, FDEBUG), ==, 1u << 24);
    wr(qts, 0, CTRL, 0);

    /*
     * The datasheet's auto_push_pull loop: one word per two cycles once
     * the OSR is full, after a cycle to fill it.
     */
    load(qts, 1, 0, loop, 2);
    wr(qts, 1, EXECCTRL(0), EXEC_WRAP(0, 1));
    wr(qts, 1, SHIFTCTRL(0), 0xc0000 | SHIFT_AUTOPULL | SHIFT_AUTOPUSH);
    for (n = 0; n < 4; n++) {
        wr(qts, 1, TXF(0), 10 + n);
    }
    wr(qts, 1, CTRL, CTRL_EN(1));
    cycles(qts, 6);
    g_assert_cmphex(rd(qts, 1, FLEVEL), ==, 0x20);
    cycles(qts, 3);
    g_assert_cmphex(rd(qts, 1, FLEVEL), ==, 0x40);
    for (n = 0; n < 4; n++) {
        g_assert_cmpuint(rd(qts, 1, RXF(0)), ==, 10 + n);
    }
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_instructions(void)
{
    QTestState *qts = start();

    /* !OSRE and PULL IFEMPTY against PULL_THRESH. */
    wr(qts, 0, SHIFTCTRL(0), SHIFT_OUT_RIGHT | SHIFT_PULL(8) | SHIFT_PUSH(8));
    wr(qts, 0, TXF(0), 0xaabbccdd);
    wr(qts, 0, TXF(0), 0x11223344);
    exec(qts, 0, 0, I_PULL(0, 1));
    exec(qts, 0, 0, I_JMP(C_NOSRE, 5));
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 5);
    exec(qts, 0, 0, I_OUT(D_NULL, 8));
    exec(qts, 0, 0, I_JMP(C_NOSRE, 9));
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 5);
    exec(qts, 0, 0, I_PULL(1, 1));
    exec(qts, 0, 0, I_OUT(D_NULL, 4));
    exec(qts, 0, 0, I_PULL(1, 1));
    g_assert_cmphex(read_reg(qts, 0, 0, S_OSR), ==, 0x01122334);

    /* PUSH IFFULL against PUSH_THRESH; IN_SHIFTDIR is left here. */
    exec(qts, 0, 0, I_SET(D_X, 0x1f));
    exec(qts, 0, 0, I_IN(S_X, 4));
    exec(qts, 0, 0, I_PUSH(1, 0));
    g_assert_cmphex(rd(qts, 0, FLEVEL), ==, 0);
    exec(qts, 0, 0, I_IN(S_X, 4));
    exec(qts, 0, 0, I_PUSH(1, 0));
    g_assert_cmphex(rd(qts, 0, RXF(0)), ==, 0xff);
    /* OUT ISR sets the input shift count. */
    wr(qts, 0, TXF(0), 0xfff);
    exec(qts, 0, 0, I_PULL(0, 1));
    exec(qts, 0, 0, I_OUT(D_ISR, 12));
    exec(qts, 0, 0, I_PUSH(1, 0));
    g_assert_cmphex(rd(qts, 0, RXF(0)), ==, 0xfff);

    /* OUT PC and MOV PC jump. */
    wr(qts, 0, TXF(0), 29);
    exec(qts, 0, 0, I_PULL(0, 1));
    exec(qts, 0, 0, I_OUT(D_PC, 5));
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 29);
    exec(qts, 0, 0, I_SET(D_X, 3));
    exec(qts, 0, 0, I_MOV(MD_PC, 0, S_X));
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 3);

    /* MOV operations. */
    exec(qts, 0, 0, I_SET(D_X, 1));
    exec(qts, 0, 0, I_MOV(MD_Y, 2, S_X));
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, 0x80000000);
    exec(qts, 0, 0, I_MOV(MD_Y, 1, S_X));
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, 0xfffffffe);
    exec(qts, 0, 0, I_MOV(MD_Y, 1, S_NULL));
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, 0xffffffff);

    /* MOV STATUS: TX level below N, RX level below N, an IRQ flag. */
    wr(qts, 0, EXECCTRL(1), 0x1f000 | EXEC_STATUS(0, 2));
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0xffffffff);
    wr(qts, 0, TXF(1), 1);
    wr(qts, 0, TXF(1), 2);
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0);
    wr(qts, 0, EXECCTRL(1), 0x1f000 | EXEC_STATUS(2, 3));
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0);
    wr(qts, 0, IRQ_FORCE, 1u << 3);
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0xffffffff);
    /* ... or the previous block's (PIO2's, from PIO0). */
    wr(qts, 0, EXECCTRL(1), 0x1f000 | EXEC_STATUS(2, 8 + 3));
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0);
    wr(qts, 2, IRQ_FORCE, 1u << 3);
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0xffffffff);

    /*
     * OUT EXEC and MOV EXEC run their instruction in the state machine's
     * next cycle, without moving the program counter.
     */
    wr(qts, 0, INSTR_MEM(3), I_JMP(C_ALWAYS, 3));
    wr(qts, 0, TXF(2), I_SET(D_Y, 21));
    exec(qts, 0, 2, I_JMP(C_ALWAYS, 3));
    exec(qts, 0, 2, I_PULL(0, 1));
    exec(qts, 0, 2, I_OUT(7, 16));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(4));
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmpuint(read_reg(qts, 0, 2, S_Y), ==, 21);
    exec(qts, 0, 2, I_SET(D_X, 9));
    exec(qts, 0, 2, I_MOV(MD_OSR, 0, S_X));
    g_assert_cmpuint(rd(qts, 0, ADDR(2)), ==, 3);
    wr(qts, 0, INSTR_MEM(7), I_JMP(C_ALWAYS, 7));
    wr(qts, 0, TXF(2), I_JMP(C_ALWAYS, 7));
    exec(qts, 0, 2, I_PULL(0, 1));
    exec(qts, 0, 2, I_MOV(MD_EXEC, 0, S_OSR));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(4));
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmpuint(rd(qts, 0, ADDR(2)), ==, 7);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_irq_flags(void)
{
    QTestState *qts = start();
    static const uint16_t waiter[] = {
        I_IRQ(0, 1, 0),
        I_JMP(C_YDEC, 1),
    };

    exec(qts, 0, 0, I_IRQ(0, 0, 2));
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x04);
    g_assert_cmphex(rd(qts, 0, INTR), ==, 0x4f0);
    g_assert_false(qtest_get_irq(qts, PIO0_IRQ_0));
    wr(qts, 0, IRQ0_INTE, 1u << 10);
    g_assert_true(qtest_get_irq(qts, PIO0_IRQ_0));
    g_assert_cmphex(rd(qts, 0, IRQ0_INTS), ==, 1u << 10);
    wr(qts, 0, IRQ, 0x04);
    g_assert_false(qtest_get_irq(qts, PIO0_IRQ_0));
    /* INTF forces the line without touching the flags. */
    wr(qts, 0, IRQ1_INTF, 1u << 12);
    g_assert_true(qtest_get_irq(qts, PIO0_IRQ_0 + 1));
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0);
    wr(qts, 0, IRQ1_INTF, 0);
    /* TX FIFO not full is a source too. */
    wr(qts, 0, IRQ0_INTE, 1u << 4);
    g_assert_true(qtest_get_irq(qts, PIO0_IRQ_0));
    wr(qts, 0, IRQ0_INTE, 0);

    /* REL adds the state machine number to the low two bits. */
    exec(qts, 0, 2, I_IRQ(0, 0, IRQ_REL | 1));
    exec(qts, 0, 3, I_IRQ(0, 0, IRQ_REL | 5));
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x18);
    /* WAIT 1 IRQ clears the flag it waited for. */
    exec(qts, 0, 1, I_WAIT(1, W_IRQ, 4));
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x08);
    exec(qts, 0, 1, I_IRQ(1, 0, 3));
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0);
    /* IRQ_FORCE sets flags the state machines see. */
    wr(qts, 0, IRQ_FORCE, 0x81);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x81);
    wr(qts, 0, IRQ + ALIAS_SET, 0xff);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0);

    /* IRQ WAIT holds until the flag is cleared, here by the system. */
    load(qts, 0, 0, waiter, 2);
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(1, 1));
    exec(qts, 0, 0, I_SET(D_Y, 0));
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 30);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x01);
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 0);
    wr(qts, 0, IRQ, 0x01);
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_Y), ==, (uint32_t)-29);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_irq_between_sms(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        /* SM0 */
        I_IRQ(0, 0, 1) | DLY(5),
        I_JMP(C_ALWAYS, 1),
        /* SM1: counts from the cycle after it sees the flag. */
        I_WAIT(1, W_IRQ, 1),
        I_JMP(C_YDEC, 3),
        /* SM2: clears flag 6 in the cycle SM3 sets it. */
        I_IRQ(1, 0, 6),
        I_JMP(C_ALWAYS, 5),
        /* SM3 */
        I_IRQ(0, 0, 6),
        I_JMP(C_ALWAYS, 7),
    };
    int i;

    load(qts, 0, 0, prog, 8);
    for (i = 0; i < 4; i++) {
        wr(qts, 0, EXECCTRL(i), EXEC_WRAP(2 * i + 1, 2 * i + 1));
        exec(qts, 0, i, I_JMP(C_ALWAYS, 2 * i));
    }
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(0xf));
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    /* SM0 sets flag 1 in cycle 0; SM1 sees it in 1 and counts from 2. */
    g_assert_cmphex(read_reg(qts, 0, 1, S_Y), ==, (uint32_t)-28);
    /* SM1 cleared flag 1; a set wins over a clear in the same cycle. */
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x40);

    /* NEXT and PREV reach the neighbouring blocks, wrapping around. */
    exec(qts, 0, 0, I_IRQ(0, 0, IRQ_NEXT | 3));
    exec(qts, 0, 0, I_IRQ(0, 0, IRQ_PREV | 5));
    g_assert_cmphex(rd(qts, 1, IRQ), ==, 0x08);
    g_assert_cmphex(rd(qts, 2, IRQ), ==, 0x20);
    exec(qts, 1, 2, I_WAIT(1, W_IRQ, IRQ_PREV | 3));
    g_assert_cmphex(rd(qts, 1, EXECCTRL(2)), ==, EXEC_STALLED | 0x1f000);
    exec(qts, 0, 0, I_IRQ(0, 0, 3));
    exec(qts, 2, 0, I_NOP_CYCLE);
    g_assert_cmphex(rd(qts, 1, EXECCTRL(2)), ==, 0x1f000);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x40);
    exec(qts, 1, 1, I_IRQ(1, 0, IRQ_NEXT | 5));
    g_assert_cmphex(rd(qts, 2, IRQ), ==, 0);

    /*
     * Blocks of different Non-secure accessibility are cut off from each
     * other: make PIO1 Non-secure.
     */
    qtest_writel(qts, ACCESSCTRL_PIO(1), 0xacce0000 | 0xfe);
    exec(qts, 0, 0, I_IRQ(0, 0, IRQ_NEXT | 2));
    g_assert_cmphex(rd(qts, 1, IRQ), ==, 0x08);
    exec(qts, 2, 0, I_IRQ(0, 0, IRQ_NEXT | 2));
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0x44);
    wr(qts, 1, IRQ_FORCE, 0x01);
    wr(qts, 0, EXECCTRL(1), 0x1f000 | EXEC_STATUS(2, 16));
    exec(qts, 0, 1, I_MOV(MD_X, 0, S_STATUS));
    g_assert_cmphex(read_reg(qts, 0, 1, S_X), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_rx_putget(void)
{
    QTestState *qts = start();

    /* The OUT pins with OUT_COUNT 32 read back the OSR through PADOUT. */
    wr(qts, 0, PINCTRL(0), PIN_OUT(32, 0));

    /* FJOIN_RX_GET: the system writes the entries, the program reads. */
    wr(qts, 0, SHIFTCTRL(0), 0xc0000 | SHIFT_RX_GET);
    wr(qts, 0, PUTGET(0, 2), 0xcafe);
    wr(qts, 0, PUTGET(0, 3), 0xf00d);
    exec(qts, 0, 0, I_GET(2));
    exec(qts, 0, 0, I_MOV(MD_PINS, 0, S_OSR));
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0xcafe);
    exec(qts, 0, 0, I_SET(D_Y, 7));
    exec(qts, 0, 0, I_GET(0) & ~8);
    exec(qts, 0, 0, I_MOV(MD_PINS, 0, S_OSR));
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0xf00d);
    /* The RX FIFO itself is gone: full and empty. */
    g_assert_cmphex(rd(qts, 0, FSTAT), ==, 0x0f000f01);

    /* FJOIN_RX_PUT: the program writes, the system reads. */
    wr(qts, 0, SHIFTCTRL(1), 0xc0000 | SHIFT_RX_PUT);
    exec(qts, 0, 1, I_SET(D_X, 13));
    exec(qts, 0, 1, I_MOV(MD_ISR, 0, S_X));
    exec(qts, 0, 1, I_PUT(1));
    g_assert_cmphex(rd(qts, 0, PUTGET(1, 1)), ==, 13);

    /* Both: scratch storage for the program, out of the system's reach. */
    wr(qts, 0, SHIFTCTRL(2), 0xc0000 | SHIFT_RX_PUT | SHIFT_RX_GET);
    wr(qts, 0, PINCTRL(2), PIN_OUT(32, 0));
    wr(qts, 0, PUTGET(2, 0), 0x55);
    exec(qts, 0, 2, I_SET(D_X, 17));
    exec(qts, 0, 2, I_MOV(MD_ISR, 0, S_X));
    exec(qts, 0, 2, I_PUT(0));
    g_assert_cmphex(rd(qts, 0, PUTGET(2, 0)), ==, 0);
    exec(qts, 0, 2, I_GET(0));
    exec(qts, 0, 2, I_MOV(MD_PINS, 0, S_OSR));
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 17);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_pin_priority(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        /* SM0: drives pin 0 high, then idles. */
        I_SET(D_PINS, 1),
        I_JMP(C_ALWAYS, 1),
        /* SM1: drives pin 0 low two cycles later, then idles. */
        I_SET(D_PINS, 0) | DLY(1),
        I_SET(D_PINS, 0),
        I_JMP(C_ALWAYS, 4),
    };
    int i;

    load(qts, 0, 0, prog, 5);
    for (i = 0; i < 2; i++) {
        wr(qts, 0, PINCTRL(i), PIN_SET(1, 0));
    }
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(1, 1));
    wr(qts, 0, EXECCTRL(1), EXEC_WRAP(4, 4));
    exec(qts, 0, 1, I_JMP(C_ALWAYS, 2));
    /* In the same cycle the higher-numbered state machine wins. */
    exec(qts, 0, 0, I_SET(D_PINS, 1));
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 1);
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(3));
    cycles(qts, 30);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0);

    /* With OUT_STICKY, SM0's write reasserts itself after SM1's. */
    wr(qts, 0, CTRL, 0);
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(1, 1) | EXEC_OUT_STICKY);
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    exec(qts, 0, 1, I_JMP(C_ALWAYS, 2));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(3));
    /* Cycle 0: SM0 1, SM1 0: SM1 wins; 1: SM0 holds 1; 2: SM1 0 again. */
    exec(qts, 0, 2, I_NOP_CYCLE);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0);
    exec(qts, 0, 2, I_NOP_CYCLE);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 1);
    exec(qts, 0, 2, I_NOP_CYCLE);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0);
    exec(qts, 0, 2, I_NOP_CYCLE);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 1);
    cycles(qts, 30);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 1);
    /* SM_RESTART releases the held write. */
    wr(qts, 0, CTRL, CTRL_RESTART(1));
    exec(qts, 0, 1, I_SET(D_PINS, 0));
    exec(qts, 0, 2, I_NOP_CYCLE);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0);

    /* Side-set wins over the same state machine's SET. */
    wr(qts, 0, PINCTRL(3), PIN_SET(1, 0) | PIN_SIDESET(1, 0));
    exec(qts, 0, 3, I_SET(D_PINS, 0) | 0x1000);
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 1);

    /* INLINE_OUT_EN: bit 4 of the OUT data enables the write. */
    wr(qts, 0, PINCTRL(3), PIN_OUT(4, 0));
    wr(qts, 0, EXECCTRL(3), 0x1f000 | EXEC_INLINE_OUT_EN | EXEC_OUT_EN_SEL(4));
    wr(qts, 0, SHIFTCTRL(3), 0xc0000);
    wr(qts, 0, TXF(3), 0x0a | 0x1c << 5);
    exec(qts, 0, 3, I_PULL(0, 1));
    exec(qts, 0, 3, I_OUT(D_PINS, 5));
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 1);
    exec(qts, 0, 3, I_OUT(D_PINS, 5));
    g_assert_cmphex(rd(qts, 0, DBG_PADOUT), ==, 0xc);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_ctrl(void)
{
    QTestState *qts = start();
    int b;

    for (b = 0; b < 3; b++) {
        wr(qts, b, INSTR_MEM(0), I_JMP(C_YDEC, 0));
        wr(qts, b, CLKDIV(1), 3 << 16);
        wr(qts, b, EXECCTRL(1), EXEC_WRAP(0, 0));
    }
    /* PIO1 enables SM1 of all three blocks, dividers in step. */
    cycles(qts, 30);
    wr(qts, 1, CTRL, CTRL_EN(2) | CTRL_CLKDIV_RST(2) | CTRL_PREV(2) |
                     CTRL_NEXT(2) | CTRL_NP_ENABLE | CTRL_NP_CLKDIV);
    for (b = 0; b < 3; b++) {
        g_assert_cmphex(rd(qts, b, CTRL), ==, 2);
    }
    cycles(qts, 60);
    wr(qts, 1, CTRL, CTRL_PREV(2) | CTRL_NEXT(2) | CTRL_NP_DISABLE);
    for (b = 0; b < 3; b++) {
        g_assert_cmphex(rd(qts, b, CTRL), ==, 0);
        g_assert_cmphex(read_reg(qts, b, 1, S_Y), ==, (uint32_t)-20);
    }

    /* The atomic aliases act on SM_ENABLE; the strobes read 0. */
    wr(qts, 2, CTRL + ALIAS_SET, CTRL_EN(5));
    g_assert_cmphex(rd(qts, 2, CTRL), ==, 5);
    wr(qts, 2, CTRL + ALIAS_XOR, CTRL_EN(3));
    g_assert_cmphex(rd(qts, 2, CTRL), ==, 6);
    wr(qts, 2, CTRL + ALIAS_CLR, CTRL_EN(2));
    g_assert_cmphex(rd(qts, 2, CTRL), ==, 4);
    wr(qts, 2, SHIFTCTRL(0) + ALIAS_SET, SHIFT_FJOIN_RX);
    g_assert_cmphex(rd(qts, 2, SHIFTCTRL(0)), ==, 0x800c0000);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_output_timing(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        I_SET(D_PINDIRS, 1),
        I_SET(D_PINS, 1),
        I_IRQ(0, 0, 0),
        I_JMP(C_ALWAYS, 3),
    };

    /*
     * Outputs change at the exact cycle without anything reading the
     * block: GPIO 0's pad and the interrupt line are sampled elsewhere.
     */
    pin_pio(qts, 0, 0);
    load(qts, 0, 0, prog, 4);
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(3, 3));
    wr(qts, 0, CLKDIV(0), 100 << 16);
    wr(qts, 0, IRQ0_INTE, 1u << 8);
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1) | CTRL_CLKDIV_RST(1));
    /* SET PINS executes in cycle 100: the pad is high from cycle 101. */
    cycles(qts, 99);
    g_assert_true(qtest_readl(qts, GPIO_STATUS(0)) & STATUS_OETOPAD);
    g_assert_false(qtest_readl(qts, GPIO_STATUS(0)) & STATUS_OUTTOPAD);
    cycles(qts, 3);
    g_assert_true(qtest_readl(qts, GPIO_STATUS(0)) & STATUS_OUTTOPAD);
    g_assert_false(qtest_get_irq(qts, PIO0_IRQ_0));
    /* The IRQ flag is set in cycle 200. */
    cycles(qts, 96);
    g_assert_false(qtest_get_irq(qts, PIO0_IRQ_0));
    cycles(qts, 3);
    g_assert_true(qtest_get_irq(qts, PIO0_IRQ_0));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_pin_loopback(void)
{
    QTestState *qts = start();
    static const uint16_t tx[] = {
        I_SET(D_PINDIRS, 1),
        I_SET(D_PINS, 1) | DLY(9),
        I_SET(D_PINS, 0) | DLY(9),
    };
    static const uint16_t rx[] = {
        I_WAIT(1, W_GPIO, 2),
        I_JMP(C_YDEC, 1),
    };

    /* PIO0 drives GPIO 2; PIO1 waits for it through the pad. */
    pin_pio(qts, 0, 2);
    load(qts, 0, 0, tx, 3);
    wr(qts, 0, PINCTRL(0), PIN_SET(1, 2));
    wr(qts, 0, EXECCTRL(0), EXEC_WRAP(1, 2));
    load(qts, 1, 0, rx, 2);
    wr(qts, 1, EXECCTRL(0), EXEC_WRAP(1, 1));
    cycles(qts, 30);
    wr(qts, 1, CTRL, CTRL_EN(1) | CTRL_NEXT(0) | CTRL_PREV(1) |
                     CTRL_NP_ENABLE);
    cycles(qts, 30);
    wr(qts, 1, CTRL, 0);
    /*
     * PIO0 sets the pin in cycle 1; the pad is high in 2 and PIO1 sees it
     * through the synchroniser in 4, then counts from 5.
     */
    g_assert_cmphex(read_reg(qts, 1, 0, S_Y), ==, (uint32_t)-25);

    /* A GPIO not routed to PIO0 keeps PIO0's output off its pad. */
    wr(qts, 0, CTRL, 0);
    qtest_writel(qts, GPIO_CTRL(2), 31);
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    exec(qts, 1, 0, I_SET(D_Y, 0));
    exec(qts, 1, 0, I_JMP(C_ALWAYS, 0));
    cycles(qts, 30);
    wr(qts, 1, CTRL, CTRL_EN(1) | CTRL_PREV(1) | CTRL_NP_ENABLE);
    cycles(qts, 300);
    g_assert_cmpuint(rd(qts, 1, ADDR(0)), ==, 0);
    /* PIO0 still runs: rerouting the GPIO shows its current output. */
    qtest_writel(qts, GPIO_CTRL(2), FUNC_PIO(0));
    cycles(qts, 30);
    g_assert_cmpuint(rd(qts, 1, ADDR(0)), ==, 1);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_gpiobase_outputs(void)
{
    QTestState *qts = start();

    pin_pio(qts, 2, 20);
    wr(qts, 2, GPIOBASE, 16);
    wr(qts, 2, PINCTRL(0), PIN_SET(1, 4));
    exec(qts, 2, 0, I_SET(D_PINDIRS, 1));
    exec(qts, 2, 0, I_SET(D_PINS, 1));
    g_assert_cmphex(rd(qts, 2, DBG_PADOUT), ==, 1u << 4);
    g_assert_true(qtest_readl(qts, GPIO_STATUS(20)) & STATUS_OUTTOPAD);
    g_assert_true(qtest_readl(qts, GPIO_STATUS(20)) & STATUS_OETOPAD);
    wr(qts, 2, GPIOBASE, 0);
    g_assert_false(qtest_readl(qts, GPIO_STATUS(20)) & STATUS_OETOPAD);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_nonsecure_inputs(void)
{
    QTestState *qts = start();

    qtest_writel(qts, PAD(3), PAD_IE);
    qtest_writel(qts, PAD(4), PAD_IE);
    drive(qts, 3, 1);
    drive(qts, 4, 1);
    /* PIO1 Non-secure; only GPIO 4 Non-secure. */
    qtest_writel(qts, ACCESSCTRL_PIO(1), 0xacce0000 | 0xfe);
    qtest_writel(qts, ACCESSCTRL_GPIO_NSMASK0, 0xacce0000 | 1u << 4);
    cycles(qts, 30);
    exec(qts, 0, 0, I_MOV(MD_X, 0, S_PINS));
    g_assert_cmphex(read_reg(qts, 0, 0, S_X) & 0x18, ==, 0x18);
    exec(qts, 1, 0, I_MOV(MD_X, 0, S_PINS));
    g_assert_cmphex(read_reg(qts, 1, 0, S_X) & 0x18, ==, 0x10);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_dreq(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);
    static const uint16_t prog[] = {
        I_PULL(0, 1) | DLY(9),
        I_IN(S_NULL, 32),
        I_PUSH(0, 1),
    };

    qtest_irq_intercept_out_named(qts, "/machine/soc/pio", "dreq");
    /* A reset drives every line, now that they are watched. */
    qtest_system_reset(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    /* TX DREQs are up while the FIFO has room; RX DREQs are down. */
    g_assert_true(qtest_get_irq(qts, 8 + 1));
    g_assert_false(qtest_get_irq(qts, 8 + 4 + 1));
    load(qts, 1, 0, prog, 3);
    wr(qts, 1, EXECCTRL(1), EXEC_WRAP(0, 2));
    wr(qts, 1, TXF(1), 1);
    wr(qts, 1, TXF(1), 2);
    wr(qts, 1, TXF(1), 3);
    g_assert_true(qtest_get_irq(qts, 8 + 1));
    wr(qts, 1, TXF(1), 4);
    g_assert_false(qtest_get_irq(qts, 8 + 1));
    cycles(qts, 30);
    wr(qts, 1, CTRL, CTRL_EN(2));
    /* The PULL in cycle 0 makes room: the DREQ rises for cycle 1. */
    g_assert_false(qtest_get_irq(qts, 8 + 1));
    cycles(qts, 3);
    g_assert_true(qtest_get_irq(qts, 8 + 1));
    g_assert_false(qtest_get_irq(qts, 8 + 4 + 1));
    /* The PUSH in cycle 11. */
    cycles(qts, 6);
    g_assert_false(qtest_get_irq(qts, 8 + 4 + 1));
    cycles(qts, 3);
    g_assert_true(qtest_get_irq(qts, 8 + 4 + 1));
    rd(qts, 1, RXF(1));
    g_assert_false(qtest_get_irq(qts, 8 + 4 + 1));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_reset_hold(void)
{
    QTestState *qts = start();

    /* Every block starts held in reset. */
    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, RP2350_RESETS_RESET_DONE) &
                    RESET_PIO_ALL, ==, 0);
    rp2350_unreset(qts, RESET_PIO_ALL);

    wr(qts, 1, CTRL, CTRL_EN(1));
    wr(qts, 1, IRQ_FORCE, 0x02);
    wr(qts, 1, TXF(0), 5);
    /* Holding PIO1 resets it and cuts it off from its neighbours. */
    qtest_writel(qts, RESETS_RESET_SET, RESET_PIO(1));
    g_assert_cmphex(rd(qts, 1, CTRL), ==, 0);
    exec(qts, 0, 0, I_IRQ(0, 0, IRQ_NEXT | 0));
    wr(qts, 0, EXECCTRL(0), 0x1f000 | EXEC_STATUS(2, 16 + 1));
    exec(qts, 0, 0, I_MOV(MD_X, 0, S_STATUS));
    rp2350_unreset(qts, RESET_PIO(1));
    g_assert_cmphex(rd(qts, 1, CTRL), ==, 0);
    g_assert_cmphex(rd(qts, 1, IRQ), ==, 0);
    g_assert_cmphex(rd(qts, 1, FLEVEL), ==, 0);
    g_assert_cmphex(read_reg(qts, 0, 0, S_X), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_hstx_coupling(void)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s", rom_path);

    qtest_irq_intercept_out_named(qts, "/machine/soc/pio", "hstx");
    qtest_system_reset(qts);
    rp2350_unreset(qts, RP2350_RESETS_ALL);
    /* PIO2's outputs for GPIOs 12-19 reach HSTX, routed to a pin or not. */
    wr(qts, 2, PINCTRL(0), PIN_SET(5, 13));
    exec(qts, 2, 0, I_SET(D_PINS, 0x15));
    g_assert_true(qtest_get_irq(qts, 16 + 1));
    g_assert_false(qtest_get_irq(qts, 16 + 2));
    g_assert_true(qtest_get_irq(qts, 16 + 3));
    g_assert_true(qtest_get_irq(qts, 16 + 5));
    g_assert_false(qtest_get_irq(qts, 1));
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.pio/test] */
static void test_delay_loops(void)
{
    QTestState *qts = start();
    static const uint16_t prog[] = {
        I_JMP(C_XDEC, 0) | DLY(2),
        I_IRQ(0, 0, 0),
        I_JMP(C_ALWAYS, 2),
    };

    /*
     * A counting loop at a divisor of 2.5: iteration j runs at enable
     * 3j + 1 (counting from 1), and enables fall at offsets 0, 3, 5, 8,
     * 10, ... from the restart. Loop X = 9 runs ten times; IRQ 0 is set
     * at enable 31, offset 75.
     */
    load(qts, 0, 0, prog, 3);
    wr(qts, 0, CLKDIV(0), (2 << 16) | (128 << 8));
    exec(qts, 0, 0, I_SET(D_X, 9));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1) | CTRL_CLKDIV_RST(1));
    cycles(qts, 75);
    g_assert_cmphex(qtest_readl(qts, PIO_BASE(0) + 0x2000 + IRQ), ==, 0);
    cycles(qts, 3);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 1);
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 2);

    /* Stopped part way: 12 enables in 30 cycles, so four iterations. */
    wr(qts, 0, CTRL, 0);
    wr(qts, 0, IRQ, 1);
    exec(qts, 0, 0, I_SET(D_X, 9));
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1) | CTRL_CLKDIV_RST(1));
    cycles(qts, 30);
    wr(qts, 0, CTRL, 0);
    g_assert_cmpuint(read_reg(qts, 0, 0, S_X), ==, 5);
    g_assert_cmpuint(rd(qts, 0, ADDR(0)), ==, 0);
    /* Resumed, it finishes the remaining iterations and the delay. */
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1));
    cycles(qts, 300);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 1);

    /* A long loop: a quarter second at full speed. */
    wr(qts, 0, CTRL, 0);
    wr(qts, 0, IRQ, 1);
    wr(qts, 0, CLKDIV(0), 1 << 16);
    wr(qts, 0, INSTR_MEM(0), I_JMP(C_XDEC, 0));
    wr(qts, 0, TXF(0), 37500000 - 1);
    exec(qts, 0, 0, I_PULL(0, 1));
    exec(qts, 0, 0, I_MOV(MD_X, 0, S_OSR));
    exec(qts, 0, 0, I_JMP(C_ALWAYS, 0));
    cycles(qts, 30);
    wr(qts, 0, CTRL, CTRL_EN(1));
    /* The loop ends in cycle 37499999 and IRQ sets in 37500000. */
    qtest_clock_step(qts, 250 * 1000 * 1000 - 20);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 0);
    cycles(qts, 6);
    g_assert_cmphex(rd(qts, 0, IRQ), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-pio-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/pio/reset-values", test_reset_values);
    qtest_add_func("/rp2350/pio/fifos", test_fifos);
    qtest_add_func("/rp2350/pio/forced-exec", test_forced_exec);
    qtest_add_func("/rp2350/pio/execution-timing", test_execution_timing);
    qtest_add_func("/rp2350/pio/clock-divider", test_clock_divider);
    qtest_add_func("/rp2350/pio/input-sync", test_input_sync);
    qtest_add_func("/rp2350/pio/outputs", test_outputs);
    qtest_add_func("/rp2350/pio/autopush-autopull", test_autopush_autopull);
    qtest_add_func("/rp2350/pio/instructions", test_instructions);
    qtest_add_func("/rp2350/pio/irq-flags", test_irq_flags);
    qtest_add_func("/rp2350/pio/irq-between-sms", test_irq_between_sms);
    qtest_add_func("/rp2350/pio/rx-putget", test_rx_putget);
    qtest_add_func("/rp2350/pio/pin-priority", test_pin_priority);
    qtest_add_func("/rp2350/pio/ctrl", test_ctrl);
    qtest_add_func("/rp2350/pio/output-timing", test_output_timing);
    qtest_add_func("/rp2350/pio/pin-loopback", test_pin_loopback);
    qtest_add_func("/rp2350/pio/gpiobase-outputs", test_gpiobase_outputs);
    qtest_add_func("/rp2350/pio/nonsecure-inputs", test_nonsecure_inputs);
    qtest_add_func("/rp2350/pio/dreq", test_dreq);
    qtest_add_func("/rp2350/pio/reset-hold", test_reset_hold);
    qtest_add_func("/rp2350/pio/hstx-coupling", test_hstx_coupling);
    qtest_add_func("/rp2350/pio/delay-loops", test_delay_loops);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
