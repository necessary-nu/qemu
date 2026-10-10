/*
 * QTest testcase for the RP2350 I2C controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * I2C0's bus carries a 256-byte AT24C EEPROM at 0x50 and I2C1's target
 * logic (a designware-i2c-target device), so I2C0 can address I2C1 as a
 * target. I2C0 is routed to GPIO 4 (SDA) and 5 (SCL), pulled up.
 *
 * Timing uses pico-sdk's 400 kHz settings at the 150 MHz clk_sys:
 * FS_SCL_HCNT 150, FS_SCL_LCNT 225, FS_SPKLEN 14. One bit is
 * (150 + 14 + 7) + (225 + 1) = 397 cycles, a byte 3573 cycles, a START
 * 150 cycles and a STOP 226 + 150 + 225 = 601 cycles.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-clocks.h"
#include "rp2350-resets.h"

#define ALIAS_SET       0x2000
#define ALIAS_CLR       0x3000

#define I2C0            0x40090000
#define I2C1            0x40098000
#define IC_CON          0x00
#define IC_TAR          0x04
#define IC_SAR          0x08
#define IC_DATA_CMD     0x10
#define IC_SS_SCL_HCNT  0x14
#define IC_SS_SCL_LCNT  0x18
#define IC_FS_SCL_HCNT  0x1c
#define IC_FS_SCL_LCNT  0x20
#define IC_INTR_STAT    0x2c
#define IC_INTR_MASK    0x30
#define IC_RAW_INTR_STAT 0x34
#define IC_RX_TL        0x38
#define IC_TX_TL        0x3c
#define IC_CLR_INTR     0x40
#define IC_CLR_RD_REQ   0x50
#define IC_CLR_TX_ABRT  0x54
#define IC_CLR_ACTIVITY 0x5c
#define IC_CLR_STOP_DET 0x60
#define IC_ENABLE       0x6c
#define IC_STATUS       0x70
#define IC_TXFLR        0x74
#define IC_RXFLR        0x78
#define IC_SDA_HOLD     0x7c
#define IC_TX_ABRT_SOURCE 0x80
#define IC_SLV_DATA_NACK_ONLY 0x84
#define IC_DMA_CR       0x88
#define IC_DMA_TDLR     0x8c
#define IC_DMA_RDLR     0x90
#define IC_SDA_SETUP    0x94
#define IC_ACK_GENERAL_CALL 0x98
#define IC_ENABLE_STATUS 0x9c
#define IC_FS_SPKLEN    0xa0
#define IC_CLR_RESTART_DET 0xa8
#define IC_COMP_PARAM_1 0xf4
#define IC_COMP_VERSION 0xf8
#define IC_COMP_TYPE    0xfc

#define CON_MASTER_MODE         (1u << 0)
#define CON_SPEED_FAST          (2u << 1)
#define CON_SPEED_MASK          (3u << 1)
#define CON_10BITADDR_SLAVE     (1u << 3)
#define CON_10BITADDR_MASTER    (1u << 4)
#define CON_RESTART_EN          (1u << 5)
#define CON_SLAVE_DISABLE       (1u << 6)
#define CON_TX_EMPTY_CTRL       (1u << 8)
#define CON_RX_FIFO_FULL_HLD    (1u << 9)
#define CON_MASTER              (CON_MASTER_MODE | CON_SPEED_FAST | \
                                 CON_RESTART_EN | CON_SLAVE_DISABLE | \
                                 CON_TX_EMPTY_CTRL)
#define CON_SLAVE               (CON_SPEED_FAST | CON_RESTART_EN | \
                                 CON_RX_FIFO_FULL_HLD)

#define TAR_GC_OR_START         (1u << 10)
#define TAR_SPECIAL             (1u << 11)

#define CMD_READ                (1u << 8)
#define CMD_STOP                (1u << 9)
#define CMD_RESTART             (1u << 10)
#define FIRST_DATA_BYTE         (1u << 11)

#define INTR_RX_UNDER           (1u << 0)
#define INTR_RX_FULL            (1u << 2)
#define INTR_TX_OVER            (1u << 3)
#define INTR_TX_EMPTY           (1u << 4)
#define INTR_RD_REQ             (1u << 5)
#define INTR_TX_ABRT            (1u << 6)
#define INTR_RX_DONE            (1u << 7)
#define INTR_ACTIVITY           (1u << 8)
#define INTR_STOP_DET           (1u << 9)
#define INTR_START_DET          (1u << 10)
#define INTR_GEN_CALL           (1u << 11)
#define INTR_RESTART_DET        (1u << 12)

#define ENABLE                  (1u << 0)
#define ENABLE_ABORT            (1u << 1)
#define ENABLE_TX_CMD_BLOCK     (1u << 2)

#define STATUS_ACTIVITY         (1u << 0)
#define STATUS_TFNF             (1u << 1)
#define STATUS_TFE              (1u << 2)
#define STATUS_RFNE             (1u << 3)
#define STATUS_MST_ACTIVITY     (1u << 5)
#define STATUS_SLV_ACTIVITY     (1u << 6)

#define ABRT_7B_ADDR_NOACK      (1u << 0)
#define ABRT_10ADDR2_NOACK      (1u << 2)
#define ABRT_GCALL_READ         (1u << 5)
#define ABRT_SBYTE_NORSTRT      (1u << 9)
#define ABRT_10B_RD_NORSTRT     (1u << 10)
#define ABRT_SLVFLUSH_TXFIFO    (1u << 13)
#define ABRT_USER_ABRT          (1u << 16)
#define ABRT_FLUSH_CNT(n)       ((uint32_t)(n) << 23)

#define DMA_CR_RDMAE            (1u << 0)
#define DMA_CR_TDMAE            (1u << 1)

#define IO_BANK0        0x40028000
#define PADS_BANK0      0x40038000
#define GPIO_STATUS(p)  (IO_BANK0 + 8 * (p))
#define GPIO_CTRL(p)    (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define PAD_PUE         (1u << 3)
#define PAD_IE          (1u << 6)
#define STATUS_OETOPAD  (1u << 13)
#define FUNC_I2C        3
#define SDA0_PIN        4
#define SCL0_PIN        5

#define DMA             0x50000000
#define CH(n)           (DMA + 0x40 * (n))
#define READ_ADDR       0x00
#define WRITE_ADDR      0x04
#define TRANS_COUNT     0x08
#define CTRL_TRIG       0x0c
#define DMA_EN          (1u << 0)
#define SIZE_WORD       (2u << 2)
#define INCR_READ       (1u << 4)
#define INCR_WRITE      (1u << 6)
#define CHAIN_TO(n)     ((n) << 13)
#define TREQ(n)         ((n) << 17)
#define DMA_BUSY        (1u << 26)
#define DREQ_I2C0_TX    44
#define DREQ_I2C0_RX    45

#define SRAM            0x20000000
#define CMDS            (SRAM + 0x1000)
#define BUF             (SRAM + 0x2000)

#define RESETS_RESET_SET 0x40022000
#define RESET_DMA       (1u << 2)
#define RESET_I2C0      (1u << 4)
#define RESET_I2C1      (1u << 5)
#define RESET_IO_BANK0  (1u << 6)
#define RESET_PADS_BANK0 (1u << 9)

#define I2C0_IRQ        36
#define I2C1_IRQ        37

#define EEPROM          0x50
#define TARGET          0x42
#define MISSING         0x51

/* clk_sys cycles, as virtual nanoseconds. */
#define CYCLES(n)       ((int64_t)(n) * 1000 / 150)
#define BIT_CYCLES      397
#define BYTE_CYCLES     (9 * BIT_CYCLES)
#define START_CYCLES    150
#define STOP_CYCLES     601

#define US              1000

static char *rom_path;

static QTestState *start(void)
{
    QTestState *qts = qtest_initf(
        "-M rp2350 -bios %s "
        "-device at24c-eeprom,bus=i2c0,address=0x%x,rom-size=256 "
        "-device designware-i2c-target,bus=i2c0,controller=/machine/soc/i2c[1]",
        rom_path, EEPROM);

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    rp2350_clocks_init(qts);
    rp2350_unreset(qts, RESET_DMA | RESET_I2C0 | RESET_I2C1 | RESET_IO_BANK0 |
                   RESET_PADS_BANK0);
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t i2c, uint32_t reg)
{
    return qtest_readl(qts, i2c + reg);
}

static void wr(QTestState *qts, uint32_t i2c, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, i2c + reg, value);
}

static void step(QTestState *qts, int64_t ns)
{
    qtest_clock_step(qts, ns);
}

/* Route I2C0 to GPIO 4 and 5 with pull-ups, as gpio_set_function() does. */
static void route_i2c0(QTestState *qts)
{
    qtest_writel(qts, PAD(SDA0_PIN), PAD_IE | PAD_PUE);
    qtest_writel(qts, PAD(SCL0_PIN), PAD_IE | PAD_PUE);
    qtest_writel(qts, GPIO_CTRL(SDA0_PIN), FUNC_I2C);
    qtest_writel(qts, GPIO_CTRL(SCL0_PIN), FUNC_I2C);
}

/* i2c_init() and i2c_set_baudrate(400000) at 150 MHz, left disabled. */
static void master_init(QTestState *qts, uint32_t i2c)
{
    wr(qts, i2c, IC_ENABLE, 0);
    wr(qts, i2c, IC_CON, CON_MASTER);
    wr(qts, i2c, IC_TX_TL, 0);
    wr(qts, i2c, IC_RX_TL, 0);
    wr(qts, i2c, IC_FS_SCL_HCNT, 150);
    wr(qts, i2c, IC_FS_SCL_LCNT, 225);
    wr(qts, i2c, IC_FS_SPKLEN, 14);
    wr(qts, i2c, IC_SDA_HOLD, 46);
}

static void master_target(QTestState *qts, uint32_t addr)
{
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_TAR, addr);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
}

static void setup(QTestState *qts)
{
    route_i2c0(qts);
    master_init(qts, I2C0);
    master_target(qts, EEPROM);
}

/* Run long enough for every queued command to finish. */
static void settle(QTestState *qts)
{
    step(qts, 2000 * US);
}

static void eeprom_write(QTestState *qts, uint8_t at, const uint8_t *data,
                         int n)
{
    int i;

    wr(qts, I2C0, IC_DATA_CMD, at);
    for (i = 0; i < n; i++) {
        wr(qts, I2C0, IC_DATA_CMD, data[i] | (i == n - 1 ? CMD_STOP : 0));
    }
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_ABRT, ==, 0);
}

/* [spec:nuos:req:emu.i2c/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    static const struct {
        uint32_t reg, value;
    } resets[] = {
        { IC_CON, 0x65 }, { IC_TAR, 0x55 }, { IC_SAR, 0x55 },
        { IC_SS_SCL_HCNT, 0x28 }, { IC_SS_SCL_LCNT, 0x2f },
        { IC_FS_SCL_HCNT, 0x6 }, { IC_FS_SCL_LCNT, 0xd },
        { IC_INTR_STAT, 0 }, { IC_INTR_MASK, 0x8ff },
        { IC_RAW_INTR_STAT, 0 }, { IC_RX_TL, 0 }, { IC_TX_TL, 0 },
        { IC_ENABLE, 0 }, { IC_STATUS, 0x6 }, { IC_TXFLR, 0 },
        { IC_RXFLR, 0 }, { IC_SDA_HOLD, 1 }, { IC_TX_ABRT_SOURCE, 0 },
        { IC_SLV_DATA_NACK_ONLY, 0 }, { IC_DMA_CR, 0 }, { IC_DMA_TDLR, 0 },
        { IC_DMA_RDLR, 0 }, { IC_SDA_SETUP, 0x64 },
        { IC_ACK_GENERAL_CALL, 1 }, { IC_ENABLE_STATUS, 0 },
        { IC_FS_SPKLEN, 7 }, { IC_COMP_PARAM_1, 0 },
        { IC_COMP_VERSION, 0x3230312a }, { IC_COMP_TYPE, 0x44570140 },
        /* Reserved: the enhanced-master, HS and SMBus registers. */
        { 0x0c, 0 }, { 0x24, 0 }, { 0x28, 0 }, { 0xcc, 0 },
    };
    int i, n;

    for (n = 0; n < 2; n++) {
        uint32_t i2c = n ? I2C1 : I2C0;

        for (i = 0; i < ARRAY_SIZE(resets); i++) {
            g_assert_cmphex(rd(qts, i2c, resets[i].reg), ==, resets[i].value);
        }
    }

    /* Field widths, read-only STOP_DET_IF_MASTER_ACTIVE, minimum counts. */
    wr(qts, I2C0, IC_CON, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_CON), ==, 0x3fd);
    wr(qts, I2C0, IC_CON, CON_MASTER_MODE);
    g_assert_cmphex(rd(qts, I2C0, IC_CON) & CON_SPEED_MASK, ==,
                    CON_SPEED_FAST);
    wr(qts, I2C0, IC_TAR, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_TAR), ==, 0xfff);
    wr(qts, I2C0, IC_SAR, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_SAR), ==, 0x3ff);
    wr(qts, I2C0, IC_SS_SCL_HCNT, 1);
    g_assert_cmphex(rd(qts, I2C0, IC_SS_SCL_HCNT), ==, 6);
    wr(qts, I2C0, IC_SS_SCL_LCNT, 1);
    g_assert_cmphex(rd(qts, I2C0, IC_SS_SCL_LCNT), ==, 8);
    wr(qts, I2C0, IC_FS_SCL_HCNT, 0x12345);
    g_assert_cmphex(rd(qts, I2C0, IC_FS_SCL_HCNT), ==, 0x2345);
    wr(qts, I2C0, IC_FS_SCL_LCNT, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_FS_SCL_LCNT), ==, 8);
    wr(qts, I2C0, IC_FS_SPKLEN, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_FS_SPKLEN), ==, 1);
    wr(qts, I2C0, IC_INTR_MASK, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_INTR_MASK), ==, 0x1fff);
    wr(qts, I2C0, IC_RX_TL, 0xff);
    g_assert_cmphex(rd(qts, I2C0, IC_RX_TL), ==, 15);
    wr(qts, I2C0, IC_DMA_CR, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_DMA_CR), ==, 3);
    wr(qts, I2C0, IC_DMA_TDLR, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_DMA_TDLR), ==, 0xf);
    wr(qts, I2C0, IC_SDA_HOLD, 0xffffffff);
    g_assert_cmphex(rd(qts, I2C0, IC_SDA_HOLD), ==, 0xffffff);

    /* Atomic aliases. */
    wr(qts, I2C0, IC_INTR_MASK, 0);
    wr(qts, I2C0 + ALIAS_SET, IC_INTR_MASK, INTR_RX_FULL | INTR_TX_ABRT);
    g_assert_cmphex(rd(qts, I2C0, IC_INTR_MASK), ==,
                    INTR_RX_FULL | INTR_TX_ABRT);
    wr(qts, I2C0 + ALIAS_CLR, IC_INTR_MASK, INTR_RX_FULL);
    g_assert_cmphex(rd(qts, I2C0, IC_INTR_MASK), ==, INTR_TX_ABRT);

    /* Configuration registers ignore writes while enabled. */
    wr(qts, I2C0, IC_TAR, 0x50);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    g_assert_cmphex(rd(qts, I2C0, IC_ENABLE_STATUS), ==, 1);
    wr(qts, I2C0, IC_TAR, 0x23);
    wr(qts, I2C0, IC_CON, CON_MASTER);
    wr(qts, I2C0, IC_FS_SCL_HCNT, 100);
    wr(qts, I2C0, IC_FS_SPKLEN, 5);
    g_assert_cmphex(rd(qts, I2C0, IC_TAR), ==, 0x50);
    g_assert_cmphex(rd(qts, I2C0, IC_CON), ==, CON_MASTER_MODE |
                    CON_SPEED_FAST);
    g_assert_cmphex(rd(qts, I2C0, IC_FS_SCL_HCNT), ==, 0x2345);
    g_assert_cmphex(rd(qts, I2C0, IC_FS_SPKLEN), ==, 1);
    /* An idle enabled block: TX FIFO empty at its threshold. */
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT), ==, INTR_TX_EMPTY);
    wr(qts, I2C0, IC_ENABLE, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_ENABLE_STATUS), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT), ==, 0);

    /* A RESETS reset returns the block to its reset state. */
    wr(qts, I2C0, IC_TAR, 0x23);
    qtest_writel(qts, RESETS_RESET_SET, RESET_I2C0);
    g_assert_cmphex(rd(qts, I2C0, IC_TAR), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_COMP_TYPE), ==, 0);
    rp2350_unreset(qts, RESET_I2C0);
    g_assert_cmphex(rd(qts, I2C0, IC_TAR), ==, 0x55);

    /* DATA_CMD while disabled is lost; reading an empty FIFO underflows. */
    wr(qts, I2C0, IC_DATA_CMD, 0x12);
    g_assert_cmphex(rd(qts, I2C0, IC_TXFLR), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT), ==, INTR_RX_UNDER);
    rd(qts, I2C0, IC_CLR_INTR);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT), ==, 0);
    qtest_quit(qts);
}

/*
 * The first command is the address and its data byte behind a START;
 * the second is popped as the first finishes, and its STOP follows it.
 */
/* [spec:nuos:req:emu.i2c/test] */
static void test_write_timing(void)
{
    QTestState *qts = start();
    int64_t second_at = CYCLES(START_CYCLES + 2 * BYTE_CYCLES);
    int64_t stop_at = CYCLES(START_CYCLES + 3 * BYTE_CYCLES + STOP_CYCLES);

    setup(qts);
    wr(qts, I2C0, IC_DATA_CMD, 0x10);
    wr(qts, I2C0, IC_DATA_CMD, 0x5a | CMD_STOP);
    /* The first command is in the shift register, the second queued. */
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 1);
    g_assert_cmphex(rd(qts, I2C0, IC_STATUS), ==, STATUS_ACTIVITY |
                    STATUS_TFNF | STATUS_MST_ACTIVITY);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT), ==,
                    INTR_START_DET | INTR_ACTIVITY);

    step(qts, second_at - 10);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 1);
    step(qts, 20);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    /* TX_EMPTY_CTRL: not empty until the byte has been shifted out. */
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_EMPTY, ==, 0);
    step(qts, CYCLES(BYTE_CYCLES));
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_EMPTY, ==,
                    INTR_TX_EMPTY);
    step(qts, stop_at - second_at - CYCLES(BYTE_CYCLES) - 30);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==, 0);
    step(qts, 40);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==,
                    INTR_STOP_DET);
    g_assert_cmphex(rd(qts, I2C0, IC_STATUS), ==, STATUS_TFNF | STATUS_TFE);

    /* ACTIVITY stays until cleared; CLR_ACTIVITY reads it. */
    g_assert_cmpuint(rd(qts, I2C0, IC_CLR_ACTIVITY), ==, 1);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_ACTIVITY, ==, 0);

    /* Standard mode uses the SS counts. */
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, (CON_MASTER & ~CON_SPEED_MASK) | (1u << 1));
    wr(qts, I2C0, IC_SS_SCL_HCNT, 600);
    wr(qts, I2C0, IC_SS_SCL_LCNT, 900);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    rd(qts, I2C0, IC_CLR_INTR);
    wr(qts, I2C0, IC_DATA_CMD, 0x10 | CMD_STOP);
    /* START 600, bytes 9 * ((600 + 14 + 7) + 901), STOP 901 + 600 + 900. */
    step(qts, CYCLES(600 + 18 * 1522 + 2401) - 20);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==, 0);
    step(qts, 40);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==,
                    INTR_STOP_DET);
    qtest_quit(qts);
}

/*
 * Random read the way pico-sdk does it: the address write ends without
 * STOP, the block is disabled and re-enabled (the controller keeps the
 * bus, holding SCL low), and the read begins with a RESTART.
 */
/* [spec:nuos:req:emu.i2c/test] */
static void test_eeprom(void)
{
    QTestState *qts = start();
    static const uint8_t data[] = { 0xa5, 0x5a, 0x00, 0xff, 0x31 };
    int i;

    setup(qts);
    eeprom_write(qts, 0x20, data, sizeof(data));

    wr(qts, I2C0, IC_DATA_CMD, 0x20);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_STATUS) & STATUS_MST_ACTIVITY, ==,
                    STATUS_MST_ACTIVITY);
    /* The controller holds SCL low on its pin. */
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS(SCL0_PIN)) & STATUS_OETOPAD,
                    ==, STATUS_OETOPAD);
    wr(qts, I2C0, IC_ENABLE, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_ENABLE_STATUS), ==, 1);
    wr(qts, I2C0, IC_TAR, EEPROM);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    for (i = 0; i < sizeof(data); i++) {
        wr(qts, I2C0, IC_DATA_CMD, CMD_READ | (i ? 0 : CMD_RESTART) |
           (i == sizeof(data) - 1 ? CMD_STOP : 0));
    }
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, sizeof(data));
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_RX_FULL | INTR_STOP_DET | INTR_TX_ABRT), ==,
                    INTR_RX_FULL | INTR_STOP_DET);
    for (i = 0; i < sizeof(data); i++) {
        g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==,
                        data[i] | (i ? 0 : FIRST_DATA_BYTE));
    }
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_RX_FULL, ==, 0);
    g_assert_cmphex(qtest_readl(qts, GPIO_STATUS(SCL0_PIN)) & STATUS_OETOPAD,
                    ==, 0);

    /* RX FIFO threshold and overflow without RX_FIFO_FULL_HLD_CTRL. */
    wr(qts, I2C0, IC_RX_TL, 3);
    for (i = 0; i < 18; i++) {
        wr(qts, I2C0, IC_DATA_CMD, CMD_READ | (i == 17 ? CMD_STOP : 0));
        if (i == 15) {
            settle(qts);
        }
    }
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 16);
    g_assert_cmphex(rd(qts, I2C0, IC_STATUS) & (1u << 4), ==, 1u << 4);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_RX_FULL | (1u << 1)), ==, INTR_RX_FULL | (1u << 1));
    qtest_quit(qts);
}

/* Holding the bus while the RX FIFO is full instead of overflowing. */
/* [spec:nuos:req:emu.i2c/test] */
static void test_rx_hold(void)
{
    QTestState *qts = start();
    int i;

    setup(qts);
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, CON_MASTER | CON_RX_FIFO_FULL_HLD);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, 0);
    for (i = 0; i < 15; i++) {
        wr(qts, I2C0, IC_DATA_CMD, CMD_READ | (i ? 0 : CMD_RESTART));
    }
    settle(qts);
    for (i = 0; i < 3; i++) {
        wr(qts, I2C0, IC_DATA_CMD, CMD_READ | (i == 2 ? CMD_STOP : 0));
    }
    settle(qts);
    /* The next read waits in the FIFO until there is room. */
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 16);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 2);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & (1u << 1), ==, 0);
    rd(qts, I2C0, IC_DATA_CMD);
    rd(qts, I2C0, IC_DATA_CMD);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 16);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_STOP_DET | (1u << 1)), ==, INTR_STOP_DET);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.i2c/test] */
static void test_nack_abort(void)
{
    QTestState *qts = start();

    setup(qts);
    master_target(qts, MISSING);
    wr(qts, I2C0, IC_DATA_CMD, 0x00);
    wr(qts, I2C0, IC_DATA_CMD, 0x01);
    wr(qts, I2C0, IC_DATA_CMD, 0x02 | CMD_STOP);
    step(qts, CYCLES(START_CYCLES + BYTE_CYCLES) + 10);
    /* The address was not acknowledged: both queued commands flushed. */
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_TX_ABRT | INTR_TX_EMPTY | INTR_STOP_DET), ==,
                    INTR_TX_ABRT | INTR_TX_EMPTY);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==,
                    ABRT_7B_ADDR_NOACK | ABRT_FLUSH_CNT(2));
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    /* The STOP follows. */
    step(qts, CYCLES(STOP_CYCLES));
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==,
                    INTR_STOP_DET);
    g_assert_cmphex(rd(qts, I2C0, IC_STATUS) & STATUS_MST_ACTIVITY, ==, 0);
    /* The TX FIFO is held flushed until TX_ABRT is cleared. */
    wr(qts, I2C0, IC_DATA_CMD, 0x00);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    g_assert_cmpuint(rd(qts, I2C0, IC_CLR_TX_ABRT), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_ABRT, ==, 0);

    /* The EEPROM still answers. */
    master_target(qts, EEPROM);
    wr(qts, I2C0, IC_DATA_CMD, 0x00 | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_ABRT, ==, 0);

    /* A general call read, and a 10-bit read without RESTART, abort. */
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_TAR, TAR_SPECIAL);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==,
                    ABRT_GCALL_READ | ABRT_FLUSH_CNT(1));
    rd(qts, I2C0, IC_CLR_TX_ABRT);
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, (CON_MASTER & ~CON_RESTART_EN) |
       CON_10BITADDR_MASTER);
    wr(qts, I2C0, IC_TAR, 0x123);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==,
                    ABRT_10B_RD_NORSTRT | ABRT_FLUSH_CNT(1));
    rd(qts, I2C0, IC_CLR_TX_ABRT);

    /* A START BYTE needs RESTART: that abort source persists. */
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, CON_MASTER & ~CON_RESTART_EN);
    wr(qts, I2C0, IC_TAR, TAR_SPECIAL | TAR_GC_OR_START | EEPROM);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, 0 | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE) & ABRT_SBYTE_NORSTRT, ==,
                    ABRT_SBYTE_NORSTRT);
    rd(qts, I2C0, IC_CLR_TX_ABRT);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==, ABRT_SBYTE_NORSTRT);
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, CON_MASTER);
    rd(qts, I2C0, IC_CLR_TX_ABRT);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==, 0);

    /* With RESTART, the START BYTE precedes the address. */
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    rd(qts, I2C0, IC_CLR_INTR);
    wr(qts, I2C0, IC_DATA_CMD, 0 | CMD_STOP);
    /* START BYTE, then RESTART: 226 + 150 + 150 cycles, address, data. */
    step(qts, CYCLES(START_CYCLES + 3 * BYTE_CYCLES + 526 + STOP_CYCLES) - 20);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==, 0);
    step(qts, 40);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_STOP_DET | INTR_TX_ABRT), ==, INTR_STOP_DET);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.i2c/test] */
static void test_user_abort(void)
{
    QTestState *qts = start();

    setup(qts);
    /* ABORT is ignored while disabled. */
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_ENABLE, ENABLE_ABORT);
    g_assert_cmphex(rd(qts, I2C0, IC_ENABLE), ==, 0);
    wr(qts, I2C0, IC_ENABLE, ENABLE);

    wr(qts, I2C0, IC_DATA_CMD, 0x00);
    wr(qts, I2C0, IC_DATA_CMD, 0x01);
    step(qts, CYCLES(START_CYCLES + BYTE_CYCLES) / 2);
    wr(qts, I2C0, IC_ENABLE, ENABLE | ENABLE_ABORT);
    /* Software cannot clear ABORT. */
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    g_assert_cmphex(rd(qts, I2C0, IC_ENABLE), ==, ENABLE | ENABLE_ABORT);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_ENABLE), ==, ENABLE);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_TX_ABRT | INTR_STOP_DET), ==,
                    INTR_TX_ABRT | INTR_STOP_DET);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==,
                    ABRT_USER_ABRT | ABRT_FLUSH_CNT(1));
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_STATUS) & STATUS_MST_ACTIVITY, ==, 0);

    /* TX_CMD_BLOCK holds commands in the FIFO. */
    rd(qts, I2C0, IC_CLR_INTR);
    wr(qts, I2C0, IC_ENABLE, ENABLE | ENABLE_TX_CMD_BLOCK);
    wr(qts, I2C0, IC_DATA_CMD, 0x00 | CMD_STOP);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 1);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_STOP_DET, ==,
                    INTR_STOP_DET);
    qtest_quit(qts);
}

/*
 * SCL and SDA are the pins' levels: unrouted, or without pull-ups, the
 * bus is never idle and the controller waits with the command queued.
 */
/* [spec:nuos:req:emu.i2c/test] */
static void test_pins(void)
{
    QTestState *qts = start();

    master_init(qts, I2C0);
    master_target(qts, EEPROM);
    wr(qts, I2C0, IC_DATA_CMD, 0x00 | CMD_STOP);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 1);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_START_DET, ==, 0);

    /* Routed but pulled down: still held. */
    qtest_writel(qts, PAD(SDA0_PIN), PAD_IE | (1u << 2));
    qtest_writel(qts, PAD(SCL0_PIN), PAD_IE | (1u << 2));
    qtest_writel(qts, GPIO_CTRL(SDA0_PIN), FUNC_I2C);
    qtest_writel(qts, GPIO_CTRL(SCL0_PIN), FUNC_I2C);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 1);

    /* Pulled up by the board: the transfer goes ahead. */
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", SDA0_PIN, 1);
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", SCL0_PIN, 1);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 0);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_STOP_DET | INTR_TX_ABRT), ==, INTR_STOP_DET);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.i2c/test] */
static void test_interrupt(void)
{
    QTestState *qts = start();

    setup(qts);
    wr(qts, I2C0, IC_INTR_MASK, INTR_RX_FULL | INTR_TX_ABRT);
    g_assert_false(qtest_get_irq(qts, I2C0_IRQ));
    wr(qts, I2C0, IC_DATA_CMD, 0x00);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_RESTART | CMD_STOP);
    settle(qts);
    g_assert_true(qtest_get_irq(qts, I2C0_IRQ));
    g_assert_cmphex(rd(qts, I2C0, IC_INTR_STAT), ==, INTR_RX_FULL);
    rd(qts, I2C0, IC_DATA_CMD);
    g_assert_false(qtest_get_irq(qts, I2C0_IRQ));

    master_target(qts, MISSING);
    wr(qts, I2C0, IC_DATA_CMD, CMD_STOP);
    settle(qts);
    g_assert_true(qtest_get_irq(qts, I2C0_IRQ));
    g_assert_cmphex(rd(qts, I2C0, IC_INTR_STAT), ==, INTR_TX_ABRT);
    rd(qts, I2C0, IC_CLR_TX_ABRT);
    g_assert_false(qtest_get_irq(qts, I2C0_IRQ));
    qtest_quit(qts);
}

/*
 * DMA: one channel feeds read commands on DREQ_I2C0_TX, another collects
 * the data on DREQ_I2C0_RX.
 */
/* [spec:nuos:req:emu.i2c/test] */
static void test_dma(void)
{
    QTestState *qts = start();
    static const uint8_t data[] = { 1, 2, 3, 4, 5, 6, 7, 8,
                                    9, 10, 11, 12, 13, 14, 15, 16,
                                    17, 18, 19, 20 };
    int n = sizeof(data), i;

    setup(qts);
    eeprom_write(qts, 0x40, data, 8);
    eeprom_write(qts, 0x48, data + 8, 8);
    eeprom_write(qts, 0x50, data + 16, 4);

    for (i = 0; i < n; i++) {
        qtest_writel(qts, CMDS + 4 * i, CMD_READ | (i ? 0 : CMD_RESTART) |
                     (i == n - 1 ? CMD_STOP : 0));
    }
    wr(qts, I2C0, IC_DATA_CMD, 0x40);
    wr(qts, I2C0, IC_DMA_CR, DMA_CR_TDMAE | DMA_CR_RDMAE);

    qtest_writel(qts, CH(1) + READ_ADDR, I2C0 + IC_DATA_CMD);
    qtest_writel(qts, CH(1) + WRITE_ADDR, BUF);
    qtest_writel(qts, CH(1) + TRANS_COUNT, n);
    qtest_writel(qts, CH(1) + CTRL_TRIG, DMA_EN | SIZE_WORD | INCR_WRITE |
                 CHAIN_TO(1) | TREQ(DREQ_I2C0_RX));
    qtest_writel(qts, CH(0) + READ_ADDR, CMDS);
    qtest_writel(qts, CH(0) + WRITE_ADDR, I2C0 + IC_DATA_CMD);
    qtest_writel(qts, CH(0) + TRANS_COUNT, n);
    qtest_writel(qts, CH(0) + CTRL_TRIG, DMA_EN | SIZE_WORD | INCR_READ |
                 CHAIN_TO(0) | TREQ(DREQ_I2C0_TX));

    /* Paced by the bus: the transfer takes at least n bytes' time. */
    step(qts, CYCLES(n * BYTE_CYCLES) / 2);
    g_assert_cmphex(qtest_readl(qts, CH(1) + CTRL_TRIG) & DMA_BUSY, ==,
                    DMA_BUSY);
    settle(qts);
    g_assert_cmphex(qtest_readl(qts, CH(0) + CTRL_TRIG) & DMA_BUSY, ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(1) + CTRL_TRIG) & DMA_BUSY, ==, 0);
    for (i = 0; i < n; i++) {
        g_assert_cmphex(qtest_readl(qts, BUF + 4 * i), ==,
                        data[i] | (i ? 0 : FIRST_DATA_BYTE));
    }
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) &
                    (INTR_STOP_DET | INTR_TX_ABRT | INTR_TX_OVER |
                     (1u << 1)), ==, INTR_STOP_DET);
    qtest_quit(qts);
}

/* I2C1 as a target at 0x42, as i2c_set_slave_mode() sets it up. */
static void target_init(QTestState *qts, uint32_t sar, uint32_t extra)
{
    wr(qts, I2C1, IC_ENABLE, 0);
    wr(qts, I2C1, IC_CON, CON_SLAVE | extra);
    wr(qts, I2C1, IC_SAR, sar);
    wr(qts, I2C1, IC_ENABLE, ENABLE);
}

/* [spec:nuos:req:emu.i2c/test] */
static void test_target(void)
{
    QTestState *qts = start();
    int64_t t;

    setup(qts);
    target_init(qts, TARGET, 0);
    master_target(qts, TARGET);
    wr(qts, I2C1, IC_INTR_MASK, INTR_RD_REQ);

    /* The controller writes two bytes. */
    wr(qts, I2C0, IC_DATA_CMD, 0x11);
    wr(qts, I2C0, IC_DATA_CMD, 0x22 | CMD_STOP);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C1, IC_RXFLR), ==, 2);
    g_assert_cmphex(rd(qts, I2C1, IC_DATA_CMD), ==, 0x11 | FIRST_DATA_BYTE);
    g_assert_cmphex(rd(qts, I2C1, IC_DATA_CMD), ==, 0x22);
    g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) &
                    (INTR_START_DET | INTR_STOP_DET | INTR_ACTIVITY), ==,
                    INTR_START_DET | INTR_STOP_DET | INTR_ACTIVITY);
    rd(qts, I2C1, IC_CLR_INTR);
    rd(qts, I2C0, IC_CLR_INTR);

    /* The controller reads: the target stretches SCL until it has data. */
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) & INTR_RD_REQ, ==,
                    INTR_RD_REQ);
    g_assert_true(qtest_get_irq(qts, I2C1_IRQ));
    g_assert_cmphex(rd(qts, I2C1, IC_STATUS) & STATUS_SLV_ACTIVITY, ==,
                    STATUS_SLV_ACTIVITY);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 0);
    g_assert_cmpuint(rd(qts, I2C0, IC_TXFLR), ==, 1);
    rd(qts, I2C1, IC_CLR_RD_REQ);
    wr(qts, I2C1, IC_DATA_CMD, 0x33);
    t = CYCLES(BYTE_CYCLES);
    step(qts, t - 20);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 0);
    step(qts, 40);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 1);
    settle(qts);
    /* Asked for the second byte. */
    g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) & INTR_RD_REQ, ==,
                    INTR_RD_REQ);
    rd(qts, I2C1, IC_CLR_RD_REQ);
    wr(qts, I2C1, IC_DATA_CMD, 0x44);
    /* Left over: flushed when the controller ends the read. */
    wr(qts, I2C1, IC_DATA_CMD, 0x55);
    settle(qts);
    g_assert_cmpuint(rd(qts, I2C0, IC_RXFLR), ==, 2);
    g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==, 0x33 | FIRST_DATA_BYTE);
    g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==, 0x44);
    g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) &
                    (INTR_RX_DONE | INTR_STOP_DET | INTR_TX_ABRT), ==,
                    INTR_RX_DONE | INTR_STOP_DET | INTR_TX_ABRT);
    g_assert_cmphex(rd(qts, I2C1, IC_TX_ABRT_SOURCE), ==,
                    ABRT_SLVFLUSH_TXFIFO | ABRT_FLUSH_CNT(1));
    g_assert_cmpuint(rd(qts, I2C1, IC_TXFLR), ==, 0);
    g_assert_cmphex(rd(qts, I2C1, IC_STATUS) & STATUS_SLV_ACTIVITY, ==, 0);
    rd(qts, I2C1, IC_CLR_INTR);

    /* Repeated START: write then read in one transfer. */
    rd(qts, I2C0, IC_CLR_INTR);
    wr(qts, I2C1, IC_DATA_CMD, 0x66);
    wr(qts, I2C0, IC_DATA_CMD, 0x77);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C1, IC_DATA_CMD), ==, 0x77 | FIRST_DATA_BYTE);
    g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==, 0x66 | FIRST_DATA_BYTE);
    g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) &
                    (INTR_RESTART_DET | INTR_STOP_DET), ==,
                    INTR_RESTART_DET | INTR_STOP_DET);

    /* A read command answering a read request aborts. */
    rd(qts, I2C1, IC_CLR_INTR);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_STOP);
    settle(qts);
    wr(qts, I2C1, IC_DATA_CMD, CMD_READ);
    g_assert_cmphex(rd(qts, I2C1, IC_TX_ABRT_SOURCE), ==, 1u << 15);
    rd(qts, I2C1, IC_CLR_TX_ABRT);
    wr(qts, I2C1, IC_DATA_CMD, 0x88);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==, 0x88 | FIRST_DATA_BYTE);

    /* A full RX FIFO stretches the controller's writes. */
    {
        int i;

        for (i = 0; i < 17; i++) {
            wr(qts, I2C0, IC_DATA_CMD, i | (i == 16 ? CMD_STOP : 0));
            if (i == 8) {
                settle(qts);
            }
        }
        settle(qts);
        g_assert_cmpuint(rd(qts, I2C1, IC_RXFLR), ==, 16);
        g_assert_cmphex(rd(qts, I2C0, IC_STATUS) & STATUS_MST_ACTIVITY, ==,
                        STATUS_MST_ACTIVITY);
        for (i = 0; i < 17; i++) {
            g_assert_cmphex(rd(qts, I2C1, IC_DATA_CMD) & 0xff, ==, i);
            settle(qts);
        }
        g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) & (1u << 1), ==, 0);
        g_assert_cmphex(rd(qts, I2C0, IC_STATUS) & STATUS_MST_ACTIVITY, ==, 0);
    }

    /* Another address is not acknowledged; NACK_ONLY refuses the data. */
    rd(qts, I2C0, IC_CLR_INTR);
    master_target(qts, TARGET + 1);
    wr(qts, I2C0, IC_DATA_CMD, 0x01 | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==, ABRT_7B_ADDR_NOACK);
    rd(qts, I2C0, IC_CLR_TX_ABRT);
    master_target(qts, TARGET);
    wr(qts, I2C1, IC_ENABLE, 0);
    wr(qts, I2C1, IC_SLV_DATA_NACK_ONLY, 1);
    wr(qts, I2C1, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, 0x01 | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==, 1u << 3);
    g_assert_cmpuint(rd(qts, I2C1, IC_RXFLR), ==, 0);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.i2c/test] */
static void test_target_10bit_gcall(void)
{
    QTestState *qts = start();

    setup(qts);
    target_init(qts, 0x2a5, CON_10BITADDR_SLAVE);
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, CON_MASTER | CON_10BITADDR_MASTER);
    wr(qts, I2C0, IC_TAR, 0x2a5);
    wr(qts, I2C0, IC_ENABLE, ENABLE);

    wr(qts, I2C1, IC_DATA_CMD, 0x99);
    wr(qts, I2C0, IC_DATA_CMD, 0x12);
    wr(qts, I2C0, IC_DATA_CMD, CMD_READ | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_ABRT, ==, 0);
    g_assert_cmphex(rd(qts, I2C1, IC_DATA_CMD), ==, 0x12 | FIRST_DATA_BYTE);
    g_assert_cmphex(rd(qts, I2C0, IC_DATA_CMD), ==, 0x99 | FIRST_DATA_BYTE);

    /* The second address byte does not match. */
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_TAR, 0x2a6);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, 0x12 | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_TX_ABRT_SOURCE), ==, ABRT_10ADDR2_NOACK);
    rd(qts, I2C0, IC_CLR_TX_ABRT);

    /* General call: acknowledged and received while ACK_GEN_CALL is set. */
    target_init(qts, TARGET, 0);
    rd(qts, I2C1, IC_CLR_INTR);
    wr(qts, I2C0, IC_ENABLE, 0);
    wr(qts, I2C0, IC_CON, CON_MASTER);
    wr(qts, I2C0, IC_TAR, TAR_SPECIAL);
    wr(qts, I2C0, IC_ENABLE, ENABLE);
    wr(qts, I2C0, IC_DATA_CMD, 0x06 | CMD_STOP);
    settle(qts);
    g_assert_cmphex(rd(qts, I2C0, IC_RAW_INTR_STAT) & INTR_TX_ABRT, ==, 0);
    g_assert_cmphex(rd(qts, I2C1, IC_RAW_INTR_STAT) & INTR_GEN_CALL, ==,
                    INTR_GEN_CALL);
    g_assert_cmphex(rd(qts, I2C1, IC_DATA_CMD), ==, 0x06 | FIRST_DATA_BYTE);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-i2c-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/i2c/reset-values", test_reset_values);
    qtest_add_func("/rp2350/i2c/write-timing", test_write_timing);
    qtest_add_func("/rp2350/i2c/eeprom", test_eeprom);
    qtest_add_func("/rp2350/i2c/rx-hold", test_rx_hold);
    qtest_add_func("/rp2350/i2c/nack-abort", test_nack_abort);
    qtest_add_func("/rp2350/i2c/user-abort", test_user_abort);
    qtest_add_func("/rp2350/i2c/pins", test_pins);
    qtest_add_func("/rp2350/i2c/interrupt", test_interrupt);
    qtest_add_func("/rp2350/i2c/dma", test_dma);
    qtest_add_func("/rp2350/i2c/target", test_target);
    qtest_add_func("/rp2350/i2c/target-10bit-gcall", test_target_10bit_gcall);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
