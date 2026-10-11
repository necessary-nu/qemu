/*
 * QTest testcase for the ESP32-P4 GP-SPI2 and GP-SPI3 controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The board has a W25Q32 flash on GP-SPI2's bus (-device w25q32,bus=spi2),
 * reached through the IO MUX's SPI2 pads, GPIO6 to GPIO11; the two
 * controllers reach each other through pads the GPIO matrix routes, each
 * pad's output signal read back by the other controller's input. Virtual
 * time steps as the transfers run, at clk_spi_mst's 40 MHz (the crystal)
 * out of reset.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SPI2            0x500d0000
#define SPI3            0x500d1000

#define CMD             0x00
#define ADDR            0x04
#define CTRL            0x08
#define CLOCK           0x0c
#define USER            0x10
#define USER1           0x14
#define USER2           0x18
#define MS_DLEN         0x1c
#define MISC            0x20
#define DIN_MODE        0x24
#define DIN_NUM         0x28
#define DOUT_MODE       0x2c
#define DMA_CONF        0x30
#define INT_ENA         0x34
#define INT_CLR         0x38
#define INT_RAW         0x3c
#define INT_ST          0x40
#define INT_SET         0x44
#define W(n)            (0x98 + 4 * (n))
#define SLAVE           0xe0
#define SLAVE1          0xe4
#define CLK_GATE        0xe8
#define DATE            0xf0

#define CMD_UPDATE      (1u << 23)
#define CMD_USR         (1u << 24)
#define FREAD_QUAD      (1u << 15)
#define WR_LSB          (1u << 25)
#define CLK_EQU_SYSCLK  (1u << 31)
#define DOUTDIN         (1u << 0)
#define QPI_MODE        (1u << 3)
#define OPI_MODE        (1u << 4)
#define CS_HOLD         (1u << 6)
#define CS_SETUP        (1u << 7)
#define FWRITE_QUAD     (1u << 13)
#define USR_CONF_NXT    (1u << 15)
#define MISO_HIGHPART   (1u << 24)
#define USR_MOSI        (1u << 27)
#define USR_MISO        (1u << 28)
#define USR_DUMMY       (1u << 29)
#define USR_ADDR        (1u << 30)
#define USR_COMMAND     (1u << 31)
#define ADDR_BITLEN(n)  ((uint32_t)((n) - 1) << 27)
#define DUMMY_CYCLES(n) ((n) - 1)
#define CS_HOLD_TIME(n) ((uint32_t)(n) << 22)
#define CMD_BITLEN(n)   ((uint32_t)((n) - 1) << 28)
#define CS_KEEP_ACTIVE  (1u << 30)
#define DMA_RX_ENA      (1u << 27)
#define DMA_TX_ENA      (1u << 28)
#define AFIFO_RST       (7u << 29)
#define SLV_SEG_TRANS_EN (1u << 18)
#define SLAVE_MODE      (1u << 26)
#define USR_CONF        (1u << 28)
#define MAGIC(v)        ((uint32_t)(v) << 22)
#define WRBUF_BITLEN_EN (1u << 11)
#define RDBUF_BITLEN_EN (1u << 10)

#define INT_OUTFIFO_EMPTY_ERR   (1u << 1)
#define INT_EX_QPI              (1u << 2)
#define INT_EN_QPI              (1u << 3)
#define INT_CMD7                (1u << 4)
#define INT_RD_DMA_DONE         (1u << 8)
#define INT_WR_DMA_DONE         (1u << 9)
#define INT_WR_BUF_DONE         (1u << 11)
#define INT_RD_BUF_DONE         (1u << 10)
#define INT_TRANS_DONE          (1u << 12)
#define INT_SEG_TRANS_DONE      (1u << 13)
#define INT_SEG_MAGIC_ERR       (1u << 14)
#define INT_SLV_CMD_ERR         (1u << 16)

/* The GPIO matrix and IO MUX. */
#define GPIO_FUNC_IN(s)     (0x500e0158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (0x500e0558 + 4 * (n))
#define IO_MUX(n)           (0x500e1004 + 4 * (n))
#define MCU_SEL(f)          ((uint32_t)(f) << 12)
#define FUN_IE              (1u << 9)
#define SIG_IN_SEL          (1u << 7)

/* Matrix signals (TRM table 10.12-1). */
#define SIG_SPI3_CK     47
#define SIG_SPI3_Q      48
#define SIG_SPI3_D      49
#define SIG_SPI3_HD     50
#define SIG_SPI3_WP     51
#define SIG_SPI3_CS     52
#define SIG_SPI2_CK     53
#define SIG_SPI2_Q      54
#define SIG_SPI2_D      55
#define SIG_SPI2_HD     56
#define SIG_SPI2_WP     57
#define SIG_SPI2_CS     62

/* The interrupt matrix's view of the sources. */
#define INTMTX_STATUS(w)    (0x500d6000 + 0x200 + 4 * (w))
#define SRC_SPI2        25
#define SRC_SPI3        26

/* GDMA-AXI. */
#define AXI             0x5008a000
#define AXI_IN(ch)      (AXI + 0x68 * (ch))
#define AXI_OUT(ch)     (AXI + 0x138 + 0x68 * (ch))
#define X_RAW           0x00
#define X_CLR           0x0c
#define X_CONF0         0x10
#define X_LINK1         0x20
#define X_LINK2         0x24
#define X_PERI_SEL      0x44
#define IN_SUC_EOF      (1u << 1)
#define OUT_EOF         (1u << 1)
#define IN_LINK_START   (1u << 2)
#define OUT_LINK_START  (1u << 1)
#define CONF0_RST       (1u << 0)
#define PERI_SPI2       1
#define PERI_SPI3       2
#define DW0(size, len)  ((size) | (uint32_t)(len) << 12)
#define SUC_EOF         (1u << 30)
#define OWNER           (1u << 31)

/* HP_SYS_CLKRST. */
#define CLKRST_SOC_CLK_CTRL2    0x500e601c
#define CLKRST_PERI_CLK_CTRL116 0x500e6080
#define CLKRST_HP_RST_EN2       0x500e60c8
#define GPSPI2_APB_CLK_EN       (1u << 19)
#define GPSPI2_MST_CLK_EN       (1u << 20)
#define RST_EN_SPI2             (1u << 7)

/* L2MEM past the L2 cache. */
#define MEM             0x4ff40000

/* The flash's JEDEC ID, as the buffer holds the three bytes. */
#define W25Q32_ID       0x001640ef

static QTestState *start(void)
{
    return qtest_init("-M esp32p4 -device w25q32,bus=spi2,cs=0");
}

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t reg)
{
    return qtest_readl(qts, base + reg);
}

static void wr(QTestState *qts, uint32_t base, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, base + reg, v);
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

/* SPI_UPDATE: it reads 1 until two clk_spi_mst cycles have passed. */
static void update(QTestState *qts, uint32_t base)
{
    wr(qts, base, CMD, CMD_UPDATE);
    g_assert_cmphex(rd(qts, base, CMD) & CMD_UPDATE, ==, CMD_UPDATE);
    qtest_clock_step(qts, 50);
    g_assert_cmphex(rd(qts, base, CMD) & CMD_UPDATE, ==, 0);
}

/* Synchronise, start a master transfer and let it run to its end. */
static void run(QTestState *qts, uint32_t base)
{
    update(qts, base);
    wr(qts, base, CMD, CMD_USR);
    for (int i = 0; i < 10000 && (rd(qts, base, CMD) & CMD_USR); i++) {
        qtest_clock_step(qts, 1000);
    }
    g_assert_cmphex(rd(qts, base, CMD) & CMD_USR, ==, 0);
}

/* The SPI2 IO MUX functions of GPIO6 to GPIO11, inputs enabled. */
static void spi2_iomux(QTestState *qts)
{
    static const int sigs[] = { SIG_SPI2_HD, SIG_SPI2_CS, SIG_SPI2_D,
                                SIG_SPI2_CK, SIG_SPI2_Q, SIG_SPI2_WP };

    for (int pad = 6; pad <= 11; pad++) {
        qtest_writel(qts, IO_MUX(pad), MCU_SEL(3) | FUN_IE | (2u << 10));
        qtest_writel(qts, GPIO_FUNC_IN(sigs[pad - 6]), 0);
    }
}

/* Pad @pad carries output signal @out and feeds input signal @in. */
static void wire(QTestState *qts, int pad, int out, int in)
{
    qtest_writel(qts, IO_MUX(pad), MCU_SEL(1) | FUN_IE | (2u << 10));
    qtest_writel(qts, GPIO_FUNC_OUT(pad), out);
    if (in >= 0) {
        qtest_writel(qts, GPIO_FUNC_IN(in), pad | SIG_IN_SEL);
    }
}

/* A command with an optional address, data out of the buffer and in. */
static void flash_cmd(QTestState *qts, uint8_t cmd, int addr_bits,
                      uint32_t addr, uint32_t user, uint32_t ctrl,
                      unsigned dummy, unsigned data_bits)
{
    uint32_t u = USR_COMMAND | CS_SETUP | CS_HOLD | user;
    uint32_t u1 = CS_HOLD_TIME(1);

    if (addr_bits) {
        u |= USR_ADDR;
        u1 |= ADDR_BITLEN(addr_bits);
        wr(qts, SPI2, ADDR, addr << (32 - addr_bits));
    }
    if (dummy) {
        u |= USR_DUMMY;
        u1 |= DUMMY_CYCLES(dummy);
    }
    wr(qts, SPI2, USER, u);
    wr(qts, SPI2, USER1, u1);
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | cmd);
    wr(qts, SPI2, CTRL, 0x3c0000 | ctrl);
    if (data_bits) {
        wr(qts, SPI2, MS_DLEN, data_bits - 1);
    }
    run(qts, SPI2);
}

/* Registers */

/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    static const uint32_t base[2] = { SPI2, SPI3 };

    for (int i = 0; i < 2; i++) {
        uint32_t b = base[i];

        g_assert_cmphex(rd(qts, b, CMD), ==, 0);
        g_assert_cmphex(rd(qts, b, CTRL), ==, 0x003c0000);
        g_assert_cmphex(rd(qts, b, CLOCK), ==, 0x80003043);
        g_assert_cmphex(rd(qts, b, USER), ==, 0x800000c0);
        g_assert_cmphex(rd(qts, b, USER1), ==, 0xb8410007);
        g_assert_cmphex(rd(qts, b, USER2), ==, 0x78000000);
        g_assert_cmphex(rd(qts, b, MS_DLEN), ==, 0);
        g_assert_cmphex(rd(qts, b, MISC), ==, i ? 0x6 : 0x3e);
        g_assert_cmphex(rd(qts, b, DMA_CONF), ==, 0x3);
        g_assert_cmphex(rd(qts, b, INT_ENA), ==, 0);
        g_assert_cmphex(rd(qts, b, INT_RAW), ==, 0);
        g_assert_cmphex(rd(qts, b, SLAVE), ==, i ? 0 : 0x02800000);
        g_assert_cmphex(rd(qts, b, SLAVE1), ==, 0);
        g_assert_cmphex(rd(qts, b, CLK_GATE), ==, 0);
        g_assert_cmphex(rd(qts, b, DATE), ==, 0x2207202);
    }
    qtest_quit(qts);
}

/*
 * The writable bits: GP-SPI3 lacks GP-SPI2's 8-bit modes, DTR, chip
 * selects 3 to 5, the segmented transfer's fields and lanes 4 to 7's
 * timing. SPI_DMA_INT_RAW is set and cleared only through _SET and _CLR.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_masks(void)
{
    QTestState *qts = start();
    static const struct {
        uint32_t reg, spi2, spi3;
    } m[] = {
        { CTRL, 0x07bdc7e8, 0x07bcc368 },
        { CLOCK, 0xc03fffff, 0xc03fffff },
        { USER, 0xff02f3f9, 0xff0233e9 },
        { USER1, 0xffff00ff, 0xffff00ff },
        { USER2, 0xf800ffff, 0xf800ffff },
        { MS_DLEN, 0x3ffff, 0x3ffff },
        { MISC, 0xe18f1fff, 0xe08003c7 },
        { DIN_MODE, 0x1ffff, 0x100ff },
        { DIN_NUM, 0xffff, 0xff },
        { DOUT_MODE, 0x1ff, 0xf },
        { INT_ENA, 0x1fffff, 0x1fbfff },
        { CLK_GATE, 0x7, 0x7 },
        { DATE, 0x0fffffff, 0x0fffffff },
    };

    for (int i = 0; i < ARRAY_SIZE(m); i++) {
        wr(qts, SPI2, m[i].reg, 0xffffffff);
        wr(qts, SPI3, m[i].reg, 0xffffffff);
        g_assert_cmphex(rd(qts, SPI2, m[i].reg), ==, m[i].spi2);
        g_assert_cmphex(rd(qts, SPI3, m[i].reg), ==, m[i].spi3);
    }
    /* SOFT_RESET reads 0; SPI_SLAVE_MODE takes GP-SPI2 into slave mode. */
    wr(qts, SPI2, SLAVE, 0xffffffff);
    wr(qts, SPI3, SLAVE, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI2, SLAVE), ==, 0x37c00f0f);
    g_assert_cmphex(rd(qts, SPI3, SLAVE), ==, 0x24000f0f);
    /* DMA_CONF: the FIFO status bits are read-only, the resets read 0. */
    wr(qts, SPI2, DMA_CONF, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI2, DMA_CONF), ==, 0x183c0001);
    wr(qts, SPI2, DMA_CONF, 0);
    g_assert_cmphex(rd(qts, SPI2, DMA_CONF), ==, 0x3);
    /* The raw interrupts: SET, CLR, and RAW itself read-only. */
    wr(qts, SPI2, INT_SET, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI2, INT_RAW), ==, 0x1fffff);
    wr(qts, SPI3, INT_SET, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==, 0x1fbfff);
    wr(qts, SPI2, INT_RAW, 0);
    g_assert_cmphex(rd(qts, SPI2, INT_RAW), ==, 0x1fffff);
    g_assert_cmphex(rd(qts, SPI2, INT_ST), ==, 0x1fffff);
    wr(qts, SPI2, INT_CLR, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI2, INT_RAW), ==, 0);
    /* A byte write merges into the register. */
    wr(qts, SPI2, W(3), 0x11223344);
    qtest_writeb(qts, SPI2 + W(3) + 2, 0xaa);
    g_assert_cmphex(rd(qts, SPI2, W(3)), ==, 0x11aa3344);
    g_assert_cmphex(qtest_readb(qts, SPI2 + W(3) + 3), ==, 0x11);
    qtest_quit(qts);
}

/*
 * The flash through the IO MUX: its JEDEC ID, in a transfer whose length
 * follows its SPI_CLK cycles; the master's configuration is the one
 * SPI_UPDATE took, not what the bus side holds since.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_jedec_timing(void)
{
    QTestState *qts = start();

    spi2_iomux(qts);
    wr(qts, SPI2, USER, USR_COMMAND | USR_MISO | CS_SETUP | CS_HOLD);
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0x9f);
    wr(qts, SPI2, MS_DLEN, 23);
    wr(qts, SPI2, INT_ENA, INT_TRANS_DONE);
    update(qts, SPI2);
    /*
     * 40 MHz: CS setup 1.5 cycles, 8 command and 24 data cycles, CS hold
     * (1 + 1.5) cycles: 36 cycles, 900 ns.
     */
    wr(qts, SPI2, CMD, CMD_USR);
    qtest_clock_step(qts, 890);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, CMD_USR);
    g_assert_false(source_level(qts, SRC_SPI2));
    qtest_clock_step(qts, 20);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, 0);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, W25Q32_ID);
    g_assert_cmphex(rd(qts, SPI2, INT_RAW), ==, INT_TRANS_DONE);
    g_assert_cmphex(rd(qts, SPI2, INT_ST), ==, INT_TRANS_DONE);
    g_assert_true(source_level(qts, SRC_SPI2));
    wr(qts, SPI2, INT_CLR, INT_TRANS_DONE);
    g_assert_false(source_level(qts, SRC_SPI2));

    /*
     * A divider of 4 (CLKCNT_N = 3) not synchronised: the next transfer
     * still runs at 40 MHz; synchronised, it takes four times as long.
     */
    wr(qts, SPI2, CLOCK, (3u << 12) | (1u << 6) | 3u);
    wr(qts, SPI2, W(0), 0);
    wr(qts, SPI2, CMD, CMD_USR);
    qtest_clock_step(qts, 910);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, 0);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, W25Q32_ID);
    update(qts, SPI2);
    wr(qts, SPI2, CMD, CMD_USR);
    qtest_clock_step(qts, 3590);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, CMD_USR);
    qtest_clock_step(qts, 20);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, 0);
    qtest_quit(qts);
}

/*
 * Programming and reading the flash: page program and read in 1-bit
 * mode, quad page program and quad output read, with the bytes in the
 * buffer in order.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_flash_rw(void)
{
    QTestState *qts = start();
    static const uint32_t data[4] = {
        0x03020100, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
    };
    static const uint32_t quad[4] = {
        0x44332211, 0x88776655, 0xccbbaa99, 0x00ffeedd,
    };

    spi2_iomux(qts);
    flash_cmd(qts, 0x06, 0, 0, 0, 0, 0, 0);
    for (int i = 0; i < 4; i++) {
        wr(qts, SPI2, W(i), data[i]);
    }
    flash_cmd(qts, 0x02, 24, 0x100, USR_MOSI, 0, 0, 128);
    for (int i = 0; i < 4; i++) {
        wr(qts, SPI2, W(i), 0);
    }
    flash_cmd(qts, 0x03, 24, 0x100, USR_MISO, 0, 0, 128);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(rd(qts, SPI2, W(i)), ==, data[i]);
    }

    flash_cmd(qts, 0x06, 0, 0, 0, 0, 0, 0);
    for (int i = 0; i < 4; i++) {
        wr(qts, SPI2, W(i), quad[i]);
    }
    flash_cmd(qts, 0x32, 24, 0x200, USR_MOSI | FWRITE_QUAD, 0, 0, 128);
    for (int i = 0; i < 4; i++) {
        wr(qts, SPI2, W(i), 0);
    }
    flash_cmd(qts, 0x6b, 24, 0x200, USR_MISO, FREAD_QUAD, 8, 128);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(rd(qts, SPI2, W(i)), ==, quad[i]);
    }
    /* Read in 1-bit mode, the quad-programmed bytes are the same. */
    flash_cmd(qts, 0x03, 24, 0x200, USR_MISO, 0, 0, 32);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, quad[0]);
    qtest_quit(qts);
}

/*
 * The flash is wired to whichever pad carries CS0: not routed to any, it
 * is not selected and the data input reads the floating pad; routed
 * through the matrix to another pad, the flash answers again.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_cs_routing(void)
{
    QTestState *qts = start();

    spi2_iomux(qts);
    qtest_writel(qts, IO_MUX(7), MCU_SEL(1) | (2u << 10));
    qtest_writel(qts, GPIO_FUNC_OUT(7), 0x100);
    wr(qts, SPI2, W(0), 0x12345678);
    flash_cmd(qts, 0x9f, 0, 0, USR_MISO, 0, 0, 24);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, 0x12000000);
    wire(qts, 20, SIG_SPI2_CS, -1);
    flash_cmd(qts, 0x9f, 0, 0, USR_MISO, 0, 0, 24);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, 0x12000000 | W25Q32_ID);
    /* Chip select 1 is not the flash's. */
    wr(qts, SPI2, MISC, 0x3d);
    wire(qts, 20, 63, -1);
    wr(qts, SPI2, W(0), 0);
    flash_cmd(qts, 0x9f, 0, 0, USR_MISO, 0, 0, 24);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, 0);
    qtest_quit(qts);
}

/*
 * A full-duplex transfer heard by the controller itself: SPI2's D out
 * through GPIO20 back into its Q input. The data goes from SPI_W0 and
 * comes into SPI_W8 (SPI_USR_MISO_HIGHPART); LSB-first sending reverses
 * each byte as MSB-first receiving sees it.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_loopback(void)
{
    QTestState *qts = start();

    wire(qts, 20, SIG_SPI2_D, SIG_SPI2_Q);
    wr(qts, SPI2, W(0), 0x80f01234);
    wr(qts, SPI2, USER, DOUTDIN | USR_MOSI | USR_MISO | MISO_HIGHPART);
    wr(qts, SPI2, MS_DLEN, 31);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI2, W(8)), ==, 0x80f01234);
    wr(qts, SPI2, CTRL, 0x3c0000 | WR_LSB);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI2, W(8)), ==, 0x010f482c);
    qtest_quit(qts);
}

/* SPI2 master and SPI3 slave through GPIO20 to GPIO23. */
static void wire_master_slave(QTestState *qts)
{
    wire(qts, 20, SIG_SPI2_CK, SIG_SPI3_CK);
    wire(qts, 21, SIG_SPI2_D, SIG_SPI3_D);
    wire(qts, 22, SIG_SPI3_Q, SIG_SPI2_Q);
    wire(qts, 23, SIG_SPI2_CS, SIG_SPI3_CS);
}

/*
 * A full-duplex exchange between SPI2 as master and SPI3 as slave, CPU
 * controlled at both ends: each gets the other's buffer; the slave
 * records the bits and raises SPI_TRANS_DONE_INT as CS rises.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_master_slave_fd(void)
{
    QTestState *qts = start();

    wire_master_slave(qts);
    wr(qts, SPI3, SLAVE, SLAVE_MODE);
    wr(qts, SPI3, USER, DOUTDIN);
    wr(qts, SPI3, W(0), 0xa5a55a5a);
    wr(qts, SPI3, W(1), 0x01234567);
    wr(qts, SPI3, INT_ENA, INT_TRANS_DONE);
    wr(qts, SPI3, CMD, CMD_USR);
    wr(qts, SPI2, W(0), 0xcafef00d);
    wr(qts, SPI2, W(1), 0x76543210);
    wr(qts, SPI2, USER, DOUTDIN | USR_MOSI | USR_MISO | MISO_HIGHPART |
       CS_SETUP | CS_HOLD);
    wr(qts, SPI2, MS_DLEN, 63);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI2, W(8)), ==, 0xa5a55a5a);
    g_assert_cmphex(rd(qts, SPI2, W(9)), ==, 0x01234567);
    g_assert_cmphex(rd(qts, SPI3, W(0)), ==, 0xcafef00d);
    g_assert_cmphex(rd(qts, SPI3, W(1)), ==, 0x76543210);
    g_assert_cmphex(rd(qts, SPI3, SLAVE1) & 0x3ffff, ==, 64);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==, INT_TRANS_DONE);
    g_assert_cmphex(rd(qts, SPI3, CMD) & CMD_USR, ==, 0);
    g_assert_true(source_level(qts, SRC_SPI3));
    qtest_quit(qts);
}

/* A half-duplex command to the slave: command, address, 8 dummy cycles. */
static void slave_hd(QTestState *qts, uint8_t cmd, uint8_t addr,
                     uint32_t data_user, unsigned data_bits)
{
    wr(qts, SPI2, USER, USR_COMMAND | USR_ADDR | USR_DUMMY | CS_SETUP |
       CS_HOLD | data_user);
    wr(qts, SPI2, USER1, ADDR_BITLEN(8) | DUMMY_CYCLES(8) | CS_HOLD_TIME(1));
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | cmd);
    wr(qts, SPI2, ADDR, (uint32_t)addr << 24);
    if (data_bits) {
        wr(qts, SPI2, MS_DLEN, data_bits - 1);
    }
    run(qts, SPI2);
}

/*
 * The slave's half-duplex commands: Wr_BUF and Rd_BUF at an address in its
 * buffer, CMD7..CMDA's interrupts, and an unknown command's error.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_slave_hd(void)
{
    QTestState *qts = start();

    wire_master_slave(qts);
    wr(qts, SPI3, SLAVE, SLAVE_MODE | WRBUF_BITLEN_EN | RDBUF_BITLEN_EN);
    wr(qts, SPI3, USER, 0);
    wr(qts, SPI2, W(0), 0x44332211);
    wr(qts, SPI2, W(1), 0x88776655);
    slave_hd(qts, 0x01, 6, USR_MOSI, 64);
    g_assert_cmphex(rd(qts, SPI3, W(1)), ==, 0x22110000);
    g_assert_cmphex(rd(qts, SPI3, W(2)), ==, 0x66554433);
    g_assert_cmphex(rd(qts, SPI3, W(3)), ==, 0x00008877);
    g_assert_cmphex(rd(qts, SPI3, SLAVE1), ==,
                    (6u << 26) | (0x01u << 18) | 64);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_WR_BUF_DONE | INT_TRANS_DONE);
    wr(qts, SPI3, INT_CLR, 0xffffffff);

    wr(qts, SPI3, W(10), 0xdeadbeef);
    slave_hd(qts, 0x02, 40, USR_MISO, 32);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, 0xdeadbeef);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_RD_BUF_DONE | INT_TRANS_DONE);
    wr(qts, SPI3, INT_CLR, 0xffffffff);

    for (int c = 7; c <= 10; c++) {
        slave_hd(qts, c, 0, 0, 0);
        g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                        INT_TRANS_DONE | (1u << (c - 3)));
        wr(qts, SPI3, INT_CLR, 0xffffffff);
    }
    slave_hd(qts, 0x0b, 0, 0, 0);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_TRANS_DONE | INT_SLV_CMD_ERR);
    qtest_quit(qts);
}

/*
 * The quad half-duplex commands, between controllers whose four data
 * lanes run each way through a pad of their own: 0xA1 (Wr_BUF, address
 * and data in 4-bit mode) and 0xA2 (Rd_BUF).
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_slave_hd_quad(void)
{
    QTestState *qts = start();
    static const int m[4] = { SIG_SPI2_D, SIG_SPI2_Q, SIG_SPI2_WP,
                              SIG_SPI2_HD };
    static const int sl[4] = { SIG_SPI3_D, SIG_SPI3_Q, SIG_SPI3_WP,
                               SIG_SPI3_HD };

    wire(qts, 20, SIG_SPI2_CK, SIG_SPI3_CK);
    wire(qts, 23, SIG_SPI2_CS, SIG_SPI3_CS);
    for (int l = 0; l < 4; l++) {
        wire(qts, 30 + l, m[l], sl[l]);
        wire(qts, 39 + l, sl[l], m[l]);
    }
    wr(qts, SPI3, SLAVE, SLAVE_MODE);
    wr(qts, SPI3, USER, 0);
    wr(qts, SPI2, W(0), 0x12345678);
    wr(qts, SPI2, CTRL, 0x3c0000 | (1u << 6));
    wr(qts, SPI2, USER, USR_COMMAND | USR_ADDR | USR_DUMMY | USR_MOSI |
       FWRITE_QUAD | CS_SETUP | CS_HOLD);
    wr(qts, SPI2, USER1, ADDR_BITLEN(8) | DUMMY_CYCLES(8) | CS_HOLD_TIME(1));
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0xa1);
    wr(qts, SPI2, ADDR, 8u << 24);
    wr(qts, SPI2, MS_DLEN, 31);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI3, W(2)), ==, 0x12345678);

    wr(qts, SPI3, W(5), 0x9abcdef0);
    wr(qts, SPI2, CTRL, 0x3c0000 | (1u << 6) | FREAD_QUAD);
    wr(qts, SPI2, USER, USR_COMMAND | USR_ADDR | USR_DUMMY | USR_MISO |
       CS_SETUP | CS_HOLD);
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0xa2);
    wr(qts, SPI2, ADDR, 20u << 24);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, 0x9abcdef0);
    qtest_quit(qts);
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

/* Start GDMA-AXI channel @ch's TX and RX lists for peripheral @periph. */
static void dma_start(QTestState *qts, int ch, unsigned periph,
                      uint32_t tx, uint32_t rx)
{
    if (rx) {
        qtest_writel(qts, AXI_IN(ch) + X_CONF0, CONF0_RST);
        qtest_writel(qts, AXI_IN(ch) + X_CONF0, 0);
        qtest_writel(qts, AXI_IN(ch) + X_CLR, 0xffffffff);
        qtest_writel(qts, AXI_IN(ch) + X_PERI_SEL, periph);
        qtest_writel(qts, AXI_IN(ch) + X_LINK2, rx);
        qtest_writel(qts, AXI_IN(ch) + X_LINK1, IN_LINK_START);
    }
    if (tx) {
        qtest_writel(qts, AXI_OUT(ch) + X_CONF0, CONF0_RST);
        qtest_writel(qts, AXI_OUT(ch) + X_CONF0, 0);
        qtest_writel(qts, AXI_OUT(ch) + X_CLR, 0xffffffff);
        qtest_writel(qts, AXI_OUT(ch) + X_PERI_SEL, periph);
        qtest_writel(qts, AXI_OUT(ch) + X_LINK2, tx);
        qtest_writel(qts, AXI_OUT(ch) + X_LINK1, OUT_LINK_START);
    }
}

static void fill(QTestState *qts, uint32_t addr, uint32_t len, uint8_t seed)
{
    g_autofree uint8_t *b = g_malloc(len);

    for (uint32_t i = 0; i < len; i++) {
        b[i] = seed + i * 13 + (i >> 8);
    }
    qtest_memwrite(qts, addr, b, len);
}

static void check_copy(QTestState *qts, uint32_t a, uint32_t b, uint32_t len)
{
    g_autofree uint8_t *x = g_malloc(len);
    g_autofree uint8_t *y = g_malloc(len);

    qtest_memread(qts, a, x, len);
    qtest_memread(qts, b, y, len);
    g_assert_cmpmem(x, len, y, len);
}

/*
 * DMA-controlled transfers: a 3000-byte full-duplex loopback from one
 * buffer to another through GDMA-AXI, the received data ending its frame
 * (IN_SUC_EOF) at the end of the transfer; then a TX list shorter than the
 * transfer, which repeats its last byte and raises
 * SPI_DMA_OUTFIFO_EMPTY_ERR_INT.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_dma(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x40, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;

    wire(qts, 20, SIG_SPI2_D, SIG_SPI2_Q);
    fill(qts, src, 3000, 7);
    desc(qts, tx, OWNER | DW0(2000, 2000), src, tx + 16);
    desc(qts, tx + 16, OWNER | SUC_EOF | DW0(1000, 1000), src + 2000, 0);
    desc(qts, rx, OWNER | DW0(4000, 0), dst, 0);
    wr(qts, SPI2, DMA_CONF, AFIFO_RST);
    wr(qts, SPI2, DMA_CONF, DMA_TX_ENA | DMA_RX_ENA);
    dma_start(qts, 0, PERI_SPI2, tx, rx);
    wr(qts, SPI2, USER, DOUTDIN | USR_MOSI | USR_MISO);
    wr(qts, SPI2, SLAVE, MAGIC(10) | (1u << 29));
    wr(qts, SPI2, MS_DLEN, 3000 * 8 - 1);
    run(qts, SPI2);
    qtest_clock_step(qts, 10000);
    check_copy(qts, src, dst, 3000);
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(4000, 3000));
    g_assert_cmphex(qtest_readl(qts, AXI_IN(0) + X_RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);
    g_assert_cmphex(rd(qts, SPI2, INT_RAW) & INT_OUTFIFO_EMPTY_ERR, ==, 0);

    /*
     * 16 bytes to send, 20 sent: the last byte four times more, with
     * SPI_MST_REMPTY_ERR_END_EN clear (as ESP-IDF has it).
     */
    wr(qts, SPI2, USER2, CMD_BITLEN(8));
    fill(qts, src, 16, 1);
    qtest_memset(qts, dst, 0, 64);
    desc(qts, tx, OWNER | SUC_EOF | DW0(16, 16), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    wr(qts, SPI2, DMA_CONF, AFIFO_RST);
    wr(qts, SPI2, DMA_CONF, DMA_TX_ENA | DMA_RX_ENA);
    dma_start(qts, 0, PERI_SPI2, tx, rx);
    wr(qts, SPI2, MS_DLEN, 20 * 8 - 1);
    run(qts, SPI2);
    qtest_clock_step(qts, 10000);
    check_copy(qts, src, dst, 16);
    for (int i = 16; i < 20; i++) {
        g_assert_cmphex(qtest_readb(qts, dst + i), ==,
                        qtest_readb(qts, src + 15));
    }
    g_assert_cmphex(rd(qts, SPI2, INT_RAW) & INT_OUTFIFO_EMPTY_ERR, ==,
                    INT_OUTFIFO_EMPTY_ERR);
    qtest_quit(qts);
}

/*
 * A slave with DMA at both ends of a full-duplex exchange, its received
 * data's frame ending as CS rises.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_slave_dma(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x40, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;

    wire_master_slave(qts);
    fill(qts, src, 64, 3);
    desc(qts, tx, OWNER | SUC_EOF | DW0(64, 64), src, 0);
    desc(qts, rx, OWNER | DW0(128, 0), dst, 0);
    wr(qts, SPI3, SLAVE, SLAVE_MODE);
    wr(qts, SPI3, USER, DOUTDIN);
    wr(qts, SPI3, DMA_CONF, AFIFO_RST);
    wr(qts, SPI3, DMA_CONF, DMA_TX_ENA | DMA_RX_ENA);
    dma_start(qts, 1, PERI_SPI3, tx, rx);
    for (int i = 0; i < 16; i++) {
        wr(qts, SPI2, W(i), 0x01010101 * i);
    }
    wr(qts, SPI2, USER, DOUTDIN | USR_MOSI | USR_MISO | CS_SETUP | CS_HOLD);
    wr(qts, SPI2, MS_DLEN, 64 * 8 - 1);
    run(qts, SPI2);
    qtest_clock_step(qts, 10000);
    for (int i = 0; i < 16; i++) {
        g_assert_cmphex(rd(qts, SPI2, W(i)), ==,
                        qtest_readl(qts, src + 4 * i));
        g_assert_cmphex(qtest_readl(qts, dst + 4 * i), ==, 0x01010101 * i);
    }
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(128, 64));
    g_assert_cmphex(qtest_readl(qts, AXI_IN(1) + X_RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);
    qtest_quit(qts);
}

/* A CONF buffer: the bitmap with the magic value, then the registers. */
static uint32_t conf_buf(QTestState *qts, uint32_t at, unsigned magic,
                         const uint32_t *regs, uint32_t bitmap)
{
    uint32_t n = 0;

    qtest_writel(qts, at, (uint32_t)magic << 28 | bitmap);
    for (int bit = 0; bit < 14; bit++) {
        if (bitmap & (1u << bit)) {
            qtest_writel(qts, at + 4 + 4 * n, regs[n]);
            n++;
        }
    }
    return 4 + 4 * n;
}

/*
 * A configurable segmented transfer to the flash: a first segment reads
 * its JEDEC ID, a second the four bytes programmed at 0, the received
 * data one DMA frame ending with the transfer; then a CONF buffer with the
 * wrong magic value, which ends the transfer at once.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_sct(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x80, c1 = MEM + 0x1000;
    uint32_t c2 = MEM + 0x1100, dst = MEM + 0x2000;
    uint32_t seg1[] = {
        0x3c0000,
        USR_COMMAND | USR_MISO | CS_SETUP | CS_HOLD | USR_CONF_NXT,
        CMD_BITLEN(8) | 0x9f,
        23,
    };
    uint32_t seg2[] = {
        0x00000000,
        USR_COMMAND | USR_ADDR | USR_MISO | CS_SETUP | CS_HOLD,
        ADDR_BITLEN(24) | CS_HOLD_TIME(1),
        CMD_BITLEN(8) | 0x03,
        31,
    };
    uint32_t n1, n2;

    spi2_iomux(qts);
    flash_cmd(qts, 0x06, 0, 0, 0, 0, 0, 0);
    wr(qts, SPI2, W(0), 0xa1b2c3d4);
    flash_cmd(qts, 0x02, 24, 0, USR_MOSI, 0, 0, 32);

    /* Bits 1 (CTRL), 3 (USER), 5 (USER2), 6 (MS_DLEN). */
    n1 = conf_buf(qts, c1, 2, seg1, 0x6a);
    /* Bits 0 (ADDR), 3 (USER), 4 (USER1), 5 (USER2), 6 (MS_DLEN). */
    n2 = conf_buf(qts, c2, 2, seg2, 0x79);
    desc(qts, tx, OWNER | SUC_EOF | DW0(n1, n1), c1, tx + 16);
    desc(qts, tx + 16, OWNER | SUC_EOF | DW0(n2, n2), c2, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    wr(qts, SPI2, DMA_CONF, AFIFO_RST);
    wr(qts, SPI2, DMA_CONF, DMA_TX_ENA | DMA_RX_ENA);
    dma_start(qts, 0, PERI_SPI2, tx, rx);
    wr(qts, SPI2, SLAVE, MAGIC(2) | USR_CONF);
    wr(qts, SPI2, CMD, 7);
    wr(qts, SPI2, INT_CLR, 0xffffffff);
    run(qts, SPI2);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, dst), ==, 0xd41640ef);
    g_assert_cmphex(qtest_readl(qts, dst + 4), ==, 0x00a1b2c3);
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(64, 7));
    g_assert_cmphex(rd(qts, SPI2, INT_RAW) &
                    (INT_TRANS_DONE | INT_SEG_TRANS_DONE | INT_SEG_MAGIC_ERR),
                    ==, INT_TRANS_DONE | INT_SEG_TRANS_DONE);
    /* The CONF buffers wrote the registers. */
    g_assert_cmphex(rd(qts, SPI2, USER2), ==, CMD_BITLEN(8) | 0x03);

    wr(qts, SPI2, INT_CLR, 0xffffffff);
    conf_buf(qts, c1, 5, seg1, 0x6a);
    desc(qts, tx, OWNER | SUC_EOF | DW0(n1, n1), c1, 0);
    wr(qts, SPI2, DMA_CONF, AFIFO_RST);
    wr(qts, SPI2, DMA_CONF, DMA_TX_ENA | DMA_RX_ENA);
    dma_start(qts, 0, PERI_SPI2, tx, 0);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI2, INT_RAW) &
                    (INT_TRANS_DONE | INT_SEG_TRANS_DONE | INT_SEG_MAGIC_ERR),
                    ==, INT_SEG_TRANS_DONE | INT_SEG_MAGIC_ERR);
    qtest_quit(qts);
}

/*
 * CS held between transfers with SPI_CS_KEEP_ACTIVE: the slave sees one
 * transfer across two of the master's.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_cs_keep(void)
{
    QTestState *qts = start();

    wire_master_slave(qts);
    wr(qts, SPI3, SLAVE, SLAVE_MODE);
    wr(qts, SPI3, USER, DOUTDIN);
    wr(qts, SPI2, USER, DOUTDIN | USR_MOSI | CS_SETUP | CS_HOLD);
    wr(qts, SPI2, MS_DLEN, 7);
    wr(qts, SPI2, MISC, 0x3e | CS_KEEP_ACTIVE);
    wr(qts, SPI2, W(0), 0x11);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW) & INT_TRANS_DONE, ==, 0);
    wr(qts, SPI2, W(0), 0x22);
    wr(qts, SPI2, MISC, 0x3e);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW) & INT_TRANS_DONE, ==,
                    INT_TRANS_DONE);
    g_assert_cmphex(rd(qts, SPI3, W(0)) & 0xffff, ==, 0x2211);
    g_assert_cmphex(rd(qts, SPI3, SLAVE1) & 0x3ffff, ==, 16);
    qtest_quit(qts);
}

/*
 * Octal: in OPI mode every state is on eight lanes (the IO MUX's IO4 to
 * IO7 on GPIO34 to GPIO37), so the JEDEC ID takes a command cycle and
 * three data cycles, 8 cycles with the CS setup and hold.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_octal(void)
{
    QTestState *qts = start();

    spi2_iomux(qts);
    for (int pad = 34; pad <= 37; pad++) {
        qtest_writel(qts, IO_MUX(pad), MCU_SEL(2) | FUN_IE | (2u << 10));
        qtest_writel(qts, GPIO_FUNC_IN(58 + pad - 34), 0);
    }
    wr(qts, SPI2, USER, USR_COMMAND | USR_MISO | OPI_MODE | CS_SETUP |
       CS_HOLD);
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0x9f);
    wr(qts, SPI2, MS_DLEN, 23);
    update(qts, SPI2);
    wr(qts, SPI2, CMD, CMD_USR);
    qtest_clock_step(qts, 190);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, CMD_USR);
    qtest_clock_step(qts, 20);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, 0);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, W25Q32_ID);
    qtest_quit(qts);
}

/*
 * The clocks and reset: a transfer waits while clk_spi_mst is stopped and
 * runs once it starts again; with the bus clock stopped writes are lost;
 * held in reset the registers read their reset values.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_clock_reset(void)
{
    QTestState *qts = start();
    uint32_t ctrl116 = qtest_readl(qts, CLKRST_PERI_CLK_CTRL116);

    wire(qts, 20, SIG_SPI2_D, SIG_SPI2_Q);
    wr(qts, SPI2, W(0), 0x5a);
    wr(qts, SPI2, USER, DOUTDIN | USR_MOSI | USR_MISO | MISO_HIGHPART);
    wr(qts, SPI2, MS_DLEN, 7);
    update(qts, SPI2);
    qtest_writel(qts, CLKRST_PERI_CLK_CTRL116, ctrl116 & ~GPSPI2_MST_CLK_EN);
    wr(qts, SPI2, CMD, CMD_USR);
    qtest_clock_step(qts, 100000);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, CMD_USR);
    g_assert_cmphex(rd(qts, SPI2, W(8)), ==, 0);
    qtest_writel(qts, CLKRST_PERI_CLK_CTRL116, ctrl116);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(rd(qts, SPI2, CMD) & CMD_USR, ==, 0);
    g_assert_cmphex(rd(qts, SPI2, W(8)), ==, 0x5a);

    qtest_writel(qts, CLKRST_SOC_CLK_CTRL2,
                 qtest_readl(qts, CLKRST_SOC_CLK_CTRL2) & ~GPSPI2_APB_CLK_EN);
    wr(qts, SPI2, W(1), 0x1234);
    qtest_writel(qts, CLKRST_SOC_CLK_CTRL2,
                 qtest_readl(qts, CLKRST_SOC_CLK_CTRL2) | GPSPI2_APB_CLK_EN);
    g_assert_cmphex(rd(qts, SPI2, W(1)), ==, 0);

    qtest_writel(qts, CLKRST_HP_RST_EN2, RST_EN_SPI2);
    g_assert_cmphex(rd(qts, SPI2, W(8)), ==, 0);
    g_assert_cmphex(rd(qts, SPI2, USER), ==, 0x800000c0);
    qtest_writel(qts, CLKRST_HP_RST_EN2, 0);
    g_assert_cmphex(rd(qts, SPI2, MS_DLEN), ==, 0);
    qtest_quit(qts);
}

/* SPI2 and SPI3 with their four data lanes each way. */
static void wire_quad(QTestState *qts)
{
    static const int m[4] = { SIG_SPI2_D, SIG_SPI2_Q, SIG_SPI2_WP,
                              SIG_SPI2_HD };
    static const int sl[4] = { SIG_SPI3_D, SIG_SPI3_Q, SIG_SPI3_WP,
                               SIG_SPI3_HD };

    wire(qts, 20, SIG_SPI2_CK, SIG_SPI3_CK);
    wire(qts, 23, SIG_SPI2_CS, SIG_SPI3_CS);
    for (int l = 0; l < 4; l++) {
        wire(qts, 30 + l, m[l], sl[l]);
        wire(qts, 39 + l, sl[l], m[l]);
    }
}

/*
 * A slave segmented transfer in half duplex: Wr_DMA into the slave's RX
 * DMA, its frame ended by CMD7; Rd_DMA from its TX DMA; End_SEG_TRANS
 * ending the transfer; En_QPI and, in QPI mode, Ex_QPI.
 */
/* [spec:nuos:req:emu.esp32p4.spi/test] */
static void test_slave_hd_dma(void)
{
    QTestState *qts = start();
    uint32_t tx = MEM, rx = MEM + 0x40, src = MEM + 0x1000;
    uint32_t dst = MEM + 0x2000;

    wire_quad(qts);
    fill(qts, src, 8, 9);
    desc(qts, tx, OWNER | SUC_EOF | DW0(8, 8), src, 0);
    desc(qts, rx, OWNER | DW0(64, 0), dst, 0);
    wr(qts, SPI3, SLAVE, SLAVE_MODE);
    wr(qts, SPI3, USER, 0);
    wr(qts, SPI3, DMA_CONF, AFIFO_RST);
    wr(qts, SPI3, DMA_CONF, DMA_TX_ENA | DMA_RX_ENA | SLV_SEG_TRANS_EN);
    dma_start(qts, 1, PERI_SPI3, tx, rx);

    wr(qts, SPI2, W(0), 0x03020100);
    wr(qts, SPI2, W(1), 0x07060504);
    wr(qts, SPI2, W(2), 0x0b0a0908);
    slave_hd(qts, 0x03, 0, USR_MOSI, 96);
    slave_hd(qts, 0x07, 0, 0, 0);
    qtest_clock_step(qts, 10000);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_WR_DMA_DONE | INT_CMD7 | INT_TRANS_DONE);
    g_assert_cmphex(qtest_readl(qts, rx), ==, SUC_EOF | DW0(64, 12));
    g_assert_cmphex(qtest_readl(qts, dst + 8), ==, 0x0b0a0908);
    wr(qts, SPI3, INT_CLR, 0xffffffff);

    slave_hd(qts, 0x04, 0, USR_MISO, 64);
    g_assert_cmphex(rd(qts, SPI2, W(0)), ==, qtest_readl(qts, src));
    g_assert_cmphex(rd(qts, SPI2, W(1)), ==, qtest_readl(qts, src + 4));
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_RD_DMA_DONE | INT_TRANS_DONE);
    wr(qts, SPI3, INT_CLR, 0xffffffff);

    /* End_SEG_TRANS: a command alone. */
    wr(qts, SPI2, USER, USR_COMMAND | CS_SETUP | CS_HOLD);
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0x05);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_SEG_TRANS_DONE | INT_TRANS_DONE);
    wr(qts, SPI3, INT_CLR, 0xffffffff);

    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0x06);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_EN_QPI | INT_TRANS_DONE);
    g_assert_cmphex(rd(qts, SPI3, USER) & QPI_MODE, ==, QPI_MODE);
    wr(qts, SPI3, INT_CLR, 0xffffffff);
    /* In QPI mode the command takes two 4-bit cycles. */
    wr(qts, SPI2, USER, USR_COMMAND | QPI_MODE | CS_SETUP | CS_HOLD);
    wr(qts, SPI2, USER2, CMD_BITLEN(8) | 0xdd);
    run(qts, SPI2);
    g_assert_cmphex(rd(qts, SPI3, INT_RAW), ==,
                    INT_EX_QPI | INT_TRANS_DONE);
    g_assert_cmphex(rd(qts, SPI3, USER) & QPI_MODE, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32p4/spi/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/spi/masks", test_masks);
    qtest_add_func("/esp32p4/spi/jedec-timing", test_jedec_timing);
    qtest_add_func("/esp32p4/spi/flash-rw", test_flash_rw);
    qtest_add_func("/esp32p4/spi/cs-routing", test_cs_routing);
    qtest_add_func("/esp32p4/spi/loopback", test_loopback);
    qtest_add_func("/esp32p4/spi/master-slave-fd", test_master_slave_fd);
    qtest_add_func("/esp32p4/spi/slave-hd", test_slave_hd);
    qtest_add_func("/esp32p4/spi/slave-hd-quad", test_slave_hd_quad);
    qtest_add_func("/esp32p4/spi/dma", test_dma);
    qtest_add_func("/esp32p4/spi/slave-dma", test_slave_dma);
    qtest_add_func("/esp32p4/spi/sct", test_sct);
    qtest_add_func("/esp32p4/spi/cs-keep", test_cs_keep);
    qtest_add_func("/esp32p4/spi/octal", test_octal);
    qtest_add_func("/esp32p4/spi/clock-reset", test_clock_reset);
    qtest_add_func("/esp32p4/spi/slave-hd-dma", test_slave_hd_dma);

    return g_test_run();
}
