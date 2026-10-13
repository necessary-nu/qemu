/*
 * QTest testcase for the ESP32 I2C controllers: addresses no device takes
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * I2C0 reaches the board's TMP105 at 0x48, which takes general calls as
 * the TMP105 does, through GPIO22 (SCL) and GPIO21 (SDA); I2C1's bus has
 * no device, and I2C1 is on GPIO18 (SCL) and GPIO19 (SDA). Each pad is
 * open drain with its pull-up and its input on. The controller runs its
 * command list as TRANS_START is written.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define DPORT_PERIP_RST_EN      0x3ff000c4
#define PERIP_I2C0              (1u << 7)
#define PERIP_I2C1              (1u << 18)

#define GPIO_BASE               0x3ff44000
#define GPIO_PIN(n)             (GPIO_BASE + 0x88 + 4 * (n))
#define GPIO_FUNC_IN(s)         (GPIO_BASE + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)        (GPIO_BASE + 0x530 + 4 * (n))
#define PIN_PAD_DRIVER          (1u << 2)
#define IN_SIG_IN_SEL           (1u << 7)
#define IO_MUX                  0x3ff49000
#define IO_MUX_GPIO21           (IO_MUX + 0x7c)
#define IO_MUX_GPIO22           (IO_MUX + 0x80)
#define IO_MUX_GPIO18           (IO_MUX + 0x70)
#define IO_MUX_GPIO19           (IO_MUX + 0x74)
#define FUN_WPU                 (1u << 8)
#define FUN_IE                  (1u << 9)
#define MCU_SEL_GPIO            (2u << 12)
#define SIG_I2C0_SCL            29
#define SIG_I2C0_SDA            30
#define SIG_I2C1_SCL            95
#define SIG_I2C1_SDA            96

#define I2C0                    0x3ff53000
#define I2C1                    0x3ff67000
#define I2C_CTR                 0x04
#define I2C_STATUS              0x08
#define I2C_FIFO_CONF           0x18
#define I2C_FIFO_DATA           0x1c
#define I2C_INT_RAW             0x20
#define I2C_INT_CLR             0x24
#define I2C_CMD(n)              (0x58 + 4 * (n))
#define CTR_MS_MODE             (1u << 4)
#define CTR_TRANS_START         (1u << 5)
#define FIFO_RX_RST             (1u << 12)
#define FIFO_TX_RST             (1u << 13)
#define I2C_ACK_ERR             (1u << 10)
#define I2C_TRANS_COMPLETE      (1u << 7)
#define CMD_RSTART              (0u << 11)
#define CMD_WRITE               (1u << 11)
#define CMD_READ                (2u << 11)
#define CMD_STOP                (3u << 11)
#define CMD_ACK_CHECK           (1u << 8)
#define CMD_ACK_VAL             (1u << 10)

#define TMP105_ADDR             0x48
#define ABSENT_ADDR             0x33
/* The board's TMP105 reads 25 C, 8.8 fixed point. */
#define TMP105_25C              0x1900

static void pad(QTestState *qts, uint32_t mux, int n, int out_sig,
                int in_sig)
{
    qtest_writel(qts, mux, MCU_SEL_GPIO | FUN_IE | FUN_WPU);
    qtest_writel(qts, GPIO_PIN(n), PIN_PAD_DRIVER);
    qtest_writel(qts, GPIO_FUNC_OUT(n), out_sig);
    qtest_writel(qts, GPIO_FUNC_IN(in_sig), IN_SIG_IN_SEL | n);
}

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-M esp32 -nic none");

    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | PERIP_I2C0 |
                 PERIP_I2C1);
    qtest_writel(qts, DPORT_PERIP_RST_EN,
                 qtest_readl(qts, DPORT_PERIP_RST_EN) &
                 ~(PERIP_I2C0 | PERIP_I2C1));
    pad(qts, IO_MUX_GPIO21, 21, SIG_I2C0_SDA, SIG_I2C0_SDA);
    pad(qts, IO_MUX_GPIO22, 22, SIG_I2C0_SCL, SIG_I2C0_SCL);
    pad(qts, IO_MUX_GPIO19, 19, SIG_I2C1_SDA, SIG_I2C1_SDA);
    pad(qts, IO_MUX_GPIO18, 18, SIG_I2C1_SCL, SIG_I2C1_SCL);
    return qts;
}

/*
 * Run the command list @cmds with @n bytes for the TX FIFO; returns
 * INT_RAW's ACK_ERR and TRANS_COMPLETE.
 */
static uint32_t run(QTestState *qts, uint32_t base, const uint32_t *cmds,
                    unsigned ncmds, const uint8_t *tx, unsigned n)
{
    qtest_writel(qts, base + I2C_FIFO_CONF, FIFO_RX_RST | FIFO_TX_RST);
    qtest_writel(qts, base + I2C_FIFO_CONF, 0);
    qtest_writel(qts, base + I2C_INT_CLR, 0x1fff);
    for (unsigned i = 0; i < n; i++) {
        qtest_writel(qts, base + I2C_FIFO_DATA, tx[i]);
    }
    for (unsigned i = 0; i < ncmds; i++) {
        qtest_writel(qts, base + I2C_CMD(i), cmds[i]);
    }
    qtest_writel(qts, base + I2C_CTR, CTR_MS_MODE | CTR_TRANS_START);
    return qtest_readl(qts, base + I2C_INT_RAW) &
           (I2C_ACK_ERR | I2C_TRANS_COMPLETE);
}

/* START, @n bytes written with their acknowledges checked, STOP */
static uint32_t write_checked(QTestState *qts, uint32_t base,
                              const uint8_t *tx, unsigned n)
{
    const uint32_t cmds[] = {
        CMD_RSTART, CMD_WRITE | CMD_ACK_CHECK | n, CMD_STOP,
    };

    return run(qts, base, cmds, ARRAY_SIZE(cmds), tx, n);
}

/*
 * The address byte @addr unchecked, then the data byte checked: whether
 * a device took the address shows in the data byte's acknowledge.
 */
static uint32_t write_unchecked_address(QTestState *qts, uint32_t base,
                                        uint8_t addr)
{
    const uint32_t cmds[] = {
        CMD_RSTART, CMD_WRITE | 1, CMD_WRITE | CMD_ACK_CHECK | 1, CMD_STOP,
    };
    const uint8_t tx[] = { addr << 1, 0x06 };

    return run(qts, base, cmds, ARRAY_SIZE(cmds), tx, 2);
}

static uint16_t tmp105_temperature(QTestState *qts)
{
    const uint32_t cmds[] = {
        CMD_RSTART, CMD_WRITE | CMD_ACK_CHECK | 2, CMD_RSTART,
        CMD_WRITE | CMD_ACK_CHECK | 1, CMD_READ | 1,
        CMD_READ | CMD_ACK_VAL | 1, CMD_STOP,
    };
    const uint8_t tx[] = {
        TMP105_ADDR << 1, 0, (TMP105_ADDR << 1) | 1,
    };
    uint16_t v;

    g_assert_cmphex(run(qts, I2C0, cmds, ARRAY_SIZE(cmds), tx, 3), ==,
                    I2C_TRANS_COMPLETE);
    v = qtest_readl(qts, I2C0 + I2C_FIFO_DATA) << 8;
    return v | qtest_readl(qts, I2C0 + I2C_FIFO_DATA);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * On I2C1, with no device on its bus, a general call is NACKed
 * (ACK_ERR), and so is an absent address after it. With the address's
 * acknowledge unchecked, the data byte after a general call or an absent
 * address is NACKed too, nothing having taken the address, and a read
 * after such an address reads the released SDA's 1s.
 */
static void test_unanswered(void)
{
    QTestState *qts = start();
    const uint8_t gcall[] = { 0x00, 0x06 };
    const uint8_t absent[] = { ABSENT_ADDR << 1, 0x06 };
    const uint32_t read_cmds[] = {
        CMD_RSTART, CMD_WRITE | 1, CMD_READ | CMD_ACK_VAL | 1, CMD_STOP,
    };
    const uint8_t read_addr = (ABSENT_ADDR << 1) | 1;

    for (int i = 0; i < 2; i++) {
        g_assert_cmphex(write_checked(qts, I2C1, gcall, 2), ==, I2C_ACK_ERR);
        g_assert_cmphex(write_checked(qts, I2C1, absent, 2), ==,
                        I2C_ACK_ERR);
    }
    g_assert_cmphex(write_unchecked_address(qts, I2C1, 0), ==, I2C_ACK_ERR);
    g_assert_cmphex(write_unchecked_address(qts, I2C1, ABSENT_ADDR), ==,
                    I2C_ACK_ERR);
    g_assert_cmphex(run(qts, I2C1, read_cmds, ARRAY_SIZE(read_cmds),
                        &read_addr, 1), ==, I2C_TRANS_COMPLETE);
    g_assert_cmphex(qtest_readl(qts, I2C1 + I2C_FIFO_DATA), ==, 0xff);
    qtest_quit(qts);
}

/*
 * [spec:nuos:req:emu.esp32.gpio/test]
 * On I2C0 the board's TMP105 takes a general call; afterwards an absent
 * address is NACKed, the TMP105 answers its own, and the data byte after
 * an absent address whose acknowledge goes unchecked is NACKed.
 */
static void test_general_call_taken(void)
{
    QTestState *qts = start();
    const uint8_t gcall[] = { 0x00, 0x04 };
    const uint8_t absent[] = { ABSENT_ADDR << 1, 0x06 };

    g_assert_cmphex(write_checked(qts, I2C0, gcall, 2), ==,
                    I2C_TRANS_COMPLETE);
    g_assert_cmphex(write_checked(qts, I2C0, absent, 2), ==, I2C_ACK_ERR);
    g_assert_cmphex(tmp105_temperature(qts), ==, TMP105_25C);
    g_assert_cmphex(write_unchecked_address(qts, I2C0, ABSENT_ADDR), ==,
                    I2C_ACK_ERR);
    g_assert_cmphex(tmp105_temperature(qts), ==, TMP105_25C);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("esp32/i2c/unanswered", test_unanswered);
    qtest_add_func("esp32/i2c/general-call-taken", test_general_call_taken);
    return g_test_run();
}
