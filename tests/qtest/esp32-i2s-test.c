/*
 * QTest testcase for the ESP32 I2S controllers
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DPORT_PERIP_CLK_EN      0x3ff000c0
#define DPORT_PERIP_RST_EN      0x3ff000c4
#define PERIP_I2S0              (1u << 4)
#define PERIP_I2S1              (1u << 21)

#define I2S0                    0x3ff4f000
#define I2S1                    0x3ff6d000
#define APB_ALIAS(a)            ((a) - 0x3ff40000 + 0x60000000)

#define FIFO_WR                 0x00
#define FIFO_RD                 0x04
#define CONF                    0x08
#define INT_RAW                 0x0c
#define INT_ST                  0x10
#define INT_ENA                 0x14
#define INT_CLR                 0x18
#define TIMING                  0x1c
#define FIFO_CONF               0x20
#define RXEOF_NUM               0x24
#define CONF_SINGLE_DATA        0x28
#define CONF_CHAN               0x2c
#define OUT_LINK                0x30
#define IN_LINK                 0x34
#define OUT_EOF_DES_ADDR        0x38
#define IN_EOF_DES_ADDR         0x3c
#define OUT_EOF_BFR_DES_ADDR    0x40
#define AHB_TEST                0x44
#define INLINK_DSCR             0x48
#define OUTLINK_DSCR            0x54
#define LC_CONF                 0x60
#define LC_HUNG_CONF            0x74
#define CVSD_CONF0              0x80
#define CVSD_CONF1              0x84
#define CVSD_CONF2              0x88
#define PLC_CONF0               0x8c
#define PLC_CONF1               0x90
#define PLC_CONF2               0x94
#define CONF1                   0xa0
#define PD_CONF                 0xa4
#define CONF2                   0xa8
#define CLKM_CONF               0xac
#define SAMPLE_RATE_CONF        0xb0
#define PDM_CONF                0xb4
#define PDM_FREQ_CONF           0xb8
#define STATE                   0xbc
#define DATE                    0xfc

#define C_TX_RESET              (1u << 0)
#define C_TX_FIFO_RESET         (1u << 2)
#define C_TX_START              (1u << 4)
#define C_RX_START              (1u << 5)
#define C_TX_SLAVE              (1u << 6)
#define C_RX_SLAVE              (1u << 7)
#define C_TX_RIGHT_FIRST        (1u << 8)
#define C_RX_RIGHT_FIRST        (1u << 9)
#define C_TX_MSB_SHIFT          (1u << 10)
#define C_RX_MSB_SHIFT          (1u << 11)
#define C_TX_SHORT_SYNC         (1u << 12)
#define C_RX_SHORT_SYNC         (1u << 13)
#define C_TX_MSB_RIGHT          (1u << 16)
#define C_RX_MSB_RIGHT          (1u << 17)
#define C_SIG_LOOPBACK          (1u << 18)
#define C_DEFAULT               (C_TX_RIGHT_FIRST | C_RX_RIGHT_FIRST | \
                                 C_TX_MSB_RIGHT | C_RX_MSB_RIGHT)

#define I_RX_TAKE_DATA          (1u << 0)
#define I_TX_PUT_DATA           (1u << 1)
#define I_RX_WFULL              (1u << 2)
#define I_RX_REMPTY             (1u << 3)
#define I_TX_WFULL              (1u << 4)
#define I_TX_REMPTY             (1u << 5)
#define I_RX_HUNG               (1u << 6)
#define I_TX_HUNG               (1u << 7)
#define I_IN_DONE               (1u << 8)
#define I_IN_SUC_EOF            (1u << 9)
#define I_OUT_DONE              (1u << 11)
#define I_OUT_EOF               (1u << 12)
#define I_IN_DSCR_ERR           (1u << 13)
#define I_OUT_DSCR_ERR          (1u << 14)
#define I_IN_DSCR_EMPTY         (1u << 15)
#define I_OUT_TOTAL_EOF         (1u << 16)

#define FC_RX_NUM(n)            (n)
#define FC_TX_NUM(n)            ((n) << 6)
#define FC_DSCR_EN              (1u << 12)
#define FC_TX_MOD(m)            ((m) << 13)
#define FC_RX_MOD(m)            ((m) << 16)
#define FC_FORCE                ((1u << 19) | (1u << 20))

#define CHAN(tx, rx)            ((tx) | ((rx) << 3))

#define LINK_STOP               (1u << 28)
#define LINK_START              (1u << 29)
#define LINK_RESTART            (1u << 30)
#define LINK_PARK               (1u << 31)

#define LC_IN_RST               (1u << 0)
#define LC_OUT_RST              (1u << 1)
#define LC_OUT_AUTO_WRBACK      (1u << 6)
#define LC_OUT_EOF_MODE         (1u << 8)
#define LC_CHECK_OWNER          (1u << 12)

#define CONF1_TX_PCM_BYPASS     (1u << 3)
#define CONF1_TX_STOP_EN        (1u << 8)
#define CONF2_CAMERA_EN         (1u << 0)
#define CONF2_LCD_EN            (1u << 5)

#define CLKM(n, b, a)           ((n) | ((b) << 8) | ((a) << 14))
#define SR(txm, rxm, txb, rxb)  ((txm) | ((rxm) << 6) | ((txb) << 12) | \
                                 ((rxb) << 18))

#define DW0_OWNER               (1u << 31)
#define DW0_EOF                 (1u << 30)
#define DW0(size, len)          ((size) | ((len) << 12))

#define RAM                     0x3ffc0000
#define DSCR_A                  (RAM + 0x000)
#define DSCR_B                  (RAM + 0x010)
#define DSCR_C                  (RAM + 0x020)
#define DSCR_D                  (RAM + 0x030)
#define BUF_A                   (RAM + 0x100)
#define BUF_B                   (RAM + 0x200)
#define BUF_C                   (RAM + 0x300)
#define BUF_D                   (RAM + 0x400)
#define LINK(a)                 ((a) & 0xfffff)

#define GPIO                    0x3ff44000
#define GPIO_FUNC_IN(s)         (GPIO + 0x130 + 4 * (s))
#define GPIO_FUNC_OUT(n)        (GPIO + 0x530 + 4 * (n))
#define IN_SIG_IN_SEL           (1u << 7)
#define IN_CONST_HIGH           0x38
#define IO_MUX                  0x3ff49000
#define FUN_IE                  (1u << 9)
#define MCU_SEL_GPIO            (2u << 12)

#define GPIO_PATH               "/machine/soc/gpio"
#define PAD_IN                  "esp32-gpio-pad-in"
#define PAD_OUT                 "esp32-gpio-pad"

/* GPIO matrix signals of the two controllers */
#define SIG_I2S0O_BCK           23
#define SIG_I2S1O_BCK           24
#define SIG_I2S0O_WS            25
#define SIG_I2S1O_WS            26
#define SIG_I2S0I_WS            28
#define SIG_I2S0_DATA(n)        (140 + (n))
#define SIG_I2S1I_BCK           164
#define SIG_I2S1I_WS            165
#define SIG_I2S1_DATA(n)        (166 + (n))
#define SIG_I2S0I_H_SYNC        190
#define SIG_I2S0I_V_SYNC        191
#define SIG_I2S0I_H_ENABLE      192

/*
 * Clocking used by most tests: I2S_CLK = 160 MHz / 10, BCK = I2S_CLK / 10
 * = 1.6 MHz (625 ns), and a frame of two 16-bit channels is 20 us.
 */
#define BCK_NS                  625
#define FRAME_NS                (32 * BCK_NS)

static QTestState *start(void)
{
    return qtest_init("-M esp32 -nic none");
}

static void wr(QTestState *qts, uint32_t base, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, base + reg, v);
}

static uint32_t rd(QTestState *qts, uint32_t base, uint32_t reg)
{
    return qtest_readl(qts, base + reg);
}

static void enable(QTestState *qts, uint32_t bits)
{
    qtest_writel(qts, DPORT_PERIP_CLK_EN,
                 qtest_readl(qts, DPORT_PERIP_CLK_EN) | bits);
}

static void put_dscr(QTestState *qts, uint32_t at, uint32_t dw0, uint32_t buf,
                     uint32_t next)
{
    qtest_writel(qts, at, dw0);
    qtest_writel(qts, at + 4, buf);
    qtest_writel(qts, at + 8, next);
}

/* The IO_MUX register of a pad */
static uint32_t iomux_reg(unsigned pad)
{
    static const uint8_t order[] = {
        36, 37, 38, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13, 15, 2, 0, 4,
        16, 17, 9, 10, 11, 6, 7, 8, 5, 18, 19, 20, 21, 22, 3, 1, 23,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(order); i++) {
        if (order[i] == pad) {
            return IO_MUX + 4 + 4 * i;
        }
    }
    g_assert_not_reached();
}

/* Pad `pad` as a GPIO-matrix pad driven by output signal sig */
static void route_out(QTestState *qts, unsigned pad, unsigned sig)
{
    qtest_writel(qts, iomux_reg(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(pad), sig);
}

/* Input signal sig taken from pad `pad` through the matrix */
static void route_in(QTestState *qts, unsigned sig, unsigned pad)
{
    qtest_writel(qts, iomux_reg(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_IN(sig), IN_SIG_IN_SEL | pad);
}

static void drive(QTestState *qts, unsigned pad, int level)
{
    qtest_set_irq_in(qts, GPIO_PATH, PAD_IN, pad, level);
}

/* I2S0's transmitter looped to its receiver through pad 18 */
static void loop_i2s0_data(QTestState *qts)
{
    route_out(qts, 18, SIG_I2S0_DATA(23));
    route_in(qts, SIG_I2S0_DATA(15), 18);
}

/* The 1.6 MHz BCK and 16-bit channels of FRAME_NS */
static void clock_16bit(QTestState *qts, uint32_t base)
{
    wr(qts, base, CLKM_CONF, CLKM(10, 0, 0));
    wr(qts, base, SAMPLE_RATE_CONF, SR(10, 10, 16, 16));
}

/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();
    static const struct {
        uint32_t reg, val;
    } regs[] = {
        { CONF, 0x00030300 },
        { INT_RAW, 0 },
        { INT_ENA, 0 },
        { TIMING, 0 },
        { FIFO_CONF, 0x00001820 },
        { RXEOF_NUM, 64 },
        { CONF_SINGLE_DATA, 0 },
        { CONF_CHAN, 0 },
        { OUT_LINK, 0 },
        { IN_LINK, 0 },
        { LC_CONF, 0x00000100 },
        { LC_HUNG_CONF, 0x00000810 },
        { CVSD_CONF0, 0x80007fff },
        { CVSD_CONF1, 0x000a0500 },
        { CVSD_CONF2, 0x000502a4 },
        { PLC_CONF0, 0x08a80339 },
        { PLC_CONF1, 0xa0178a05 },
        { PLC_CONF2, 0x00000028 },
        { CONF1, 0x00000089 },
        { PD_CONF, 0x0000000a },
        { CONF2, 0 },
        { CLKM_CONF, 4 },
        { SAMPLE_RATE_CONF, 0x00410186 },
        { PDM_CONF, 0x01550020 },
        { PDM_FREQ_CONF, (960 << 10) | 480 },
        { STATE, 1 },
        { DATE, 0x01604201 },
    };

    enable(qts, PERIP_I2S0 | PERIP_I2S1);
    for (int i = 0; i < 2; i++) {
        uint32_t base = i ? I2S1 : I2S0;

        for (int j = 0; j < ARRAY_SIZE(regs); j++) {
            g_assert_cmphex(rd(qts, base, regs[j].reg), ==, regs[j].val);
        }
        /* The APB alias reaches the same block */
        g_assert_cmphex(qtest_readl(qts, APB_ALIAS(base) + RXEOF_NUM), ==,
                        64);
    }

    /* Writable fields only */
    wr(qts, I2S0, CONF, 0xffffffff & ~(C_TX_START | C_RX_START));
    g_assert_cmphex(rd(qts, I2S0, CONF), ==, 0x0007ffcf);
    wr(qts, I2S0, CONF, 0x00030300);
    wr(qts, I2S0, CONF_CHAN, 0xffffffff);
    g_assert_cmphex(rd(qts, I2S0, CONF_CHAN), ==, 0x1f);
    wr(qts, I2S0, CLKM_CONF, 0xffffffff);
    g_assert_cmphex(rd(qts, I2S0, CLKM_CONF), ==, 0x003fffff);
    wr(qts, I2S0, AHB_TEST, 0xffffffff);
    g_assert_cmphex(rd(qts, I2S0, AHB_TEST), ==, 0x37);
    /* Link control bits clear themselves; the address stays */
    wr(qts, I2S0, OUT_LINK, LINK_STOP | 0x12344);
    g_assert_cmphex(rd(qts, I2S0, OUT_LINK), ==, 0x12344);
    qtest_quit(qts);
}

/*
 * DPORT gates each controller: with its clock off writes are dropped,
 * and holding it in reset puts its registers back to their reset values.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_gate_and_reset(void)
{
    QTestState *qts = start();

    /* Gated at reset */
    wr(qts, I2S1, RXEOF_NUM, 5);
    g_assert_cmphex(rd(qts, I2S1, RXEOF_NUM), ==, 64);

    enable(qts, PERIP_I2S1);
    wr(qts, I2S1, RXEOF_NUM, 5);
    g_assert_cmphex(rd(qts, I2S1, RXEOF_NUM), ==, 5);
    /* I2S0 has its own bit */
    wr(qts, I2S0, RXEOF_NUM, 7);
    g_assert_cmphex(rd(qts, I2S0, RXEOF_NUM), ==, 64);

    qtest_writel(qts, DPORT_PERIP_RST_EN, PERIP_I2S1);
    g_assert_cmphex(rd(qts, I2S1, RXEOF_NUM), ==, 64);
    qtest_writel(qts, DPORT_PERIP_RST_EN, 0);
    wr(qts, I2S1, RXEOF_NUM, 9);
    g_assert_cmphex(rd(qts, I2S1, RXEOF_NUM), ==, 9);
    qtest_quit(qts);
}

/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_interrupt_regs(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0);
    wr(qts, I2S0, INT_ENA, 0xffffffff);
    g_assert_cmphex(rd(qts, I2S0, INT_ENA), ==, 0x1ffff);

    /* An empty receive FIFO read raises RX_REMPTY */
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, I_RX_REMPTY);
    wr(qts, I2S0, INT_ENA, I_TX_WFULL);
    g_assert_cmphex(rd(qts, I2S0, INT_ST), ==, 0);
    wr(qts, I2S0, INT_ENA, I_RX_REMPTY);
    g_assert_cmphex(rd(qts, I2S0, INT_ST), ==, I_RX_REMPTY);
    /* INT_CLR clears the bits written as 1 and reads as 0 */
    wr(qts, I2S0, INT_CLR, I_TX_WFULL);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, I_RX_REMPTY);
    wr(qts, I2S0, INT_CLR, I_RX_REMPTY);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, 0);
    g_assert_cmphex(rd(qts, I2S0, INT_CLR), ==, 0);

    /* Filling the transmit FIFO's 64 words raises TX_WFULL */
    for (int i = 0; i < 64; i++) {
        wr(qts, I2S0, FIFO_WR, i);
    }
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, I_TX_WFULL);
    /* TX_FIFO_RESET empties it */
    wr(qts, I2S0, INT_CLR, 0x1ffff);
    wr(qts, I2S0, CONF, C_DEFAULT | C_TX_FIFO_RESET);
    wr(qts, I2S0, CONF, C_DEFAULT);
    wr(qts, I2S0, FIFO_WR, 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, 0);
    qtest_quit(qts);
}

/*
 * CPU FIFO access with the transmitter looped to the receiver: SIG_LOOPBACK
 * shares the transmitter's BCK and WS, and the GPIO matrix carries
 * DATA_out23 to DATA_in15 through a pad. Both master; one frame of two
 * 16-bit channels per 20 us.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_cpu_loopback(void)
{
    QTestState *qts = start();
    static const uint32_t words[] = {
        0x12345678, 0x9abcdef0, 0x0000ffff, 0x80017ffe,
    };

    enable(qts, PERIP_I2S0);
    loop_i2s0_data(qts);
    clock_16bit(qts, I2S0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_TX_NUM(32) | FC_RX_NUM(0));
    for (int i = 0; i < ARRAY_SIZE(words); i++) {
        wr(qts, I2S0, FIFO_WR, words[i]);
    }
    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
       C_RX_START);
    g_assert_cmphex(rd(qts, I2S0, STATE) & 1, ==, 0);

    /* One frame a FRAME_NS, each received at the frame's end */
    qtest_clock_step(qts, FRAME_NS - 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_RX_TAKE_DATA, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, words[0]);
    qtest_clock_step(qts, 3 * FRAME_NS);
    for (int i = 1; i < ARRAY_SIZE(words); i++) {
        g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, words[i]);
    }
    /* Out of data, the transmitter sends its last frame again */
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_TX_REMPTY, ==, I_TX_REMPTY);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, words[3]);
    qtest_quit(qts);
}

/*
 * The Philips, MSB-aligned and PCM short-sync formats, with matching
 * settings on both sides, and 24-bit channels in 32-bit FIFO words.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_formats(void)
{
    static const uint32_t fmt[] = {
        0,
        C_TX_MSB_SHIFT | C_RX_MSB_SHIFT,
        C_TX_SHORT_SYNC | C_RX_SHORT_SYNC | C_TX_MSB_SHIFT | C_RX_MSB_SHIFT,
        C_TX_SHORT_SYNC | C_RX_SHORT_SYNC,
    };

    for (int f = 0; f < ARRAY_SIZE(fmt); f++) {
        QTestState *qts = start();

        enable(qts, PERIP_I2S0);
        loop_i2s0_data(qts);
        wr(qts, I2S0, CLKM_CONF, CLKM(10, 0, 0));
        wr(qts, I2S0, SAMPLE_RATE_CONF, SR(10, 10, 24, 24));
        /* 32-bit dual channel: a word per channel, right first */
        wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_TX_MOD(2) | FC_RX_MOD(2));
        for (int i = 0; i < 4; i++) {
            wr(qts, I2S0, FIFO_WR, 0x11223300 * (i + 1));
        }
        wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | fmt[f] |
           C_TX_START | C_RX_START);
        /* A frame is 48 BCK; Philips delivers the last bit a cycle later */
        qtest_clock_step(qts, 3 * 48 * BCK_NS);
        for (int i = 0; i < 4; i++) {
            uint32_t want = (0x11223300 * (i + 1)) & 0xffffff00;

            g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, want);
        }
        qtest_quit(qts);
    }
}

/*
 * A transmitter in MSB-aligned format read by a Philips receiver sees
 * every sample one bit late.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_format_mismatch(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0);
    loop_i2s0_data(qts);
    clock_16bit(qts, I2S0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE);
    wr(qts, I2S0, FIFO_WR, 0x80018002);
    wr(qts, I2S0, FIFO_WR, 0x80018002);
    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_RX_MSB_SHIFT |
       C_TX_START | C_RX_START);
    qtest_clock_step(qts, 2 * FRAME_NS);
    /*
     * Each 16-bit sample loses its MSB and gains the next slot's MSB as
     * its LSB: right 0x8001 -> 0x0003, left 0x8002 -> 0x0005.
     */
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0x00030005);
    qtest_quit(qts);
}

/*
 * TX_CHAN_MOD's mono modes and the constant channel, received as 16-bit
 * dual channel.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_chan_modes(void)
{
    static const struct {
        uint32_t chan;
        uint32_t want;
    } cases[] = {
        /* TX_MSB_RIGHT: right in the high half, 0xaaaa; left 0x5555 */
        { 0, 0xaaaa5555 },
        { 1, 0xaaaaaaaa },
        { 2, 0x55555555 },
        { 3, 0xc0de5555 },
        { 4, 0xaaaac0de },
    };

    for (int c = 0; c < ARRAY_SIZE(cases); c++) {
        QTestState *qts = start();

        enable(qts, PERIP_I2S0);
        loop_i2s0_data(qts);
        clock_16bit(qts, I2S0);
        wr(qts, I2S0, FIFO_CONF, FC_FORCE);
        wr(qts, I2S0, CONF_CHAN, CHAN(cases[c].chan, 0));
        wr(qts, I2S0, CONF_SINGLE_DATA, 0xc0de0000);
        wr(qts, I2S0, FIFO_WR, 0xaaaa5555);
        wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
           C_RX_START);
        qtest_clock_step(qts, FRAME_NS);
        g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, cases[c].want);
        qtest_quit(qts);
    }
}

/*
 * DMA from memory to memory through the loop: two outlink descriptors, the
 * second with eof; two inlink descriptors and an EOF every six words.
 * OUT_EOF_MODE raises OUT_EOF when the last word leaves the FIFO, one word
 * a frame, which times the transfer.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_dma_loopback(void)
{
    QTestState *qts = start();
    uint32_t raw;

    enable(qts, PERIP_I2S0);
    loop_i2s0_data(qts);
    clock_16bit(qts, I2S0);
    for (int i = 0; i < 6; i++) {
        qtest_writel(qts, BUF_A + 4 * i, 0x01010101u * (i + 1) + 0x10203000);
    }
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 16), BUF_A, DSCR_B);
    put_dscr(qts, DSCR_B, DW0_OWNER | DW0_EOF | DW0(8, 8), BUF_A + 16, 0);
    put_dscr(qts, DSCR_C, DW0_OWNER | DW0(12, 0), BUF_C, DSCR_D);
    put_dscr(qts, DSCR_D, DW0_OWNER | DW0(12, 0), BUF_D, 0);

    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_DSCR_EN);
    wr(qts, I2S0, RXEOF_NUM, 6);
    wr(qts, I2S0, LC_CONF, LC_OUT_EOF_MODE | LC_OUT_AUTO_WRBACK);
    wr(qts, I2S0, IN_LINK, LINK_START | LINK(DSCR_C));
    wr(qts, I2S0, OUT_LINK, LINK_START | LINK(DSCR_A));
    /* The outlink has filled the FIFO with both descriptors' data */
    raw = rd(qts, I2S0, INT_RAW);
    g_assert_cmphex(raw & (I_OUT_DONE | I_OUT_EOF), ==, I_OUT_DONE);
    g_assert_cmphex(qtest_readl(qts, DSCR_A), ==, DW0(16, 16));
    g_assert_cmphex(qtest_readl(qts, DSCR_B), ==, DW0_EOF | DW0(8, 8));
    g_assert_cmphex(rd(qts, I2S0, OUTLINK_DSCR), ==, DSCR_B);

    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
       C_RX_START);
    /* Three words fill the first inlink descriptor */
    qtest_clock_step(qts, 3 * FRAME_NS);
    raw = rd(qts, I2S0, INT_RAW);
    g_assert_cmphex(raw & (I_IN_DONE | I_IN_SUC_EOF), ==, I_IN_DONE);
    g_assert_cmphex(qtest_readl(qts, DSCR_C), ==, DW0(12, 12));

    /* The sixth word leaves the FIFO and arrives at 6 frames */
    qtest_clock_step(qts, 3 * FRAME_NS - 1);
    raw = rd(qts, I2S0, INT_RAW);
    g_assert_cmphex(raw & (I_OUT_EOF | I_IN_SUC_EOF), ==, 0);
    qtest_clock_step(qts, 1);
    raw = rd(qts, I2S0, INT_RAW);
    g_assert_cmphex(raw & (I_OUT_EOF | I_OUT_TOTAL_EOF | I_IN_SUC_EOF), ==,
                    I_OUT_EOF | I_OUT_TOTAL_EOF | I_IN_SUC_EOF);
    g_assert_cmphex(rd(qts, I2S0, OUT_EOF_DES_ADDR), ==, DSCR_B);
    g_assert_cmphex(rd(qts, I2S0, OUT_EOF_BFR_DES_ADDR), ==, BUF_A + 16);
    g_assert_cmphex(rd(qts, I2S0, IN_EOF_DES_ADDR), ==, DSCR_D);
    g_assert_cmphex(qtest_readl(qts, DSCR_D), ==, DW0_EOF | DW0(12, 12));
    for (int i = 0; i < 3; i++) {
        g_assert_cmphex(qtest_readl(qts, BUF_C + 4 * i), ==,
                        0x01010101u * (i + 1) + 0x10203000);
        g_assert_cmphex(qtest_readl(qts, BUF_D + 4 * i), ==,
                        0x01010101u * (i + 4) + 0x10203000);
    }

    /* The interrupt reaches the interrupt matrix's source */
    wr(qts, I2S0, INT_ENA, I_IN_SUC_EOF);
    g_assert_cmphex(rd(qts, I2S0, INT_ST), ==, I_IN_SUC_EOF);
    qtest_quit(qts);
}

/*
 * The sample rate follows CLKM_CONF's fractional divider and BCK_DIV_NUM:
 * N = 8, b/a = 1/2, M = 4 gives a 6.8 us frame of two 16-bit channels.
 * With OUT_EOF_MODE clear, OUT_EOF comes as the outlink fills the FIFO.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_sample_rate(void)
{
    QTestState *qts = start();
    const uint32_t frame = 6800;

    enable(qts, PERIP_I2S0);
    loop_i2s0_data(qts);
    wr(qts, I2S0, CLKM_CONF, CLKM(8, 1, 2));
    wr(qts, I2S0, SAMPLE_RATE_CONF, SR(4, 4, 16, 16));
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0_EOF | DW0(40, 40), BUF_A, 0);
    put_dscr(qts, DSCR_C, DW0_OWNER | DW0(64, 0), BUF_C, 0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_DSCR_EN);
    wr(qts, I2S0, RXEOF_NUM, 10);
    wr(qts, I2S0, LC_CONF, 0);
    wr(qts, I2S0, IN_LINK, LINK_START | LINK(DSCR_C));
    wr(qts, I2S0, OUT_LINK, LINK_START | LINK(DSCR_A));
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_OUT_EOF, ==, I_OUT_EOF);

    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
       C_RX_START);
    qtest_clock_step(qts, 10 * frame - 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_IN_SUC_EOF, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_IN_SUC_EOF, ==, I_IN_SUC_EOF);
    g_assert_cmphex(qtest_readl(qts, DSCR_C), ==, DW0_EOF | DW0(64, 40));

    /* Changing the divider retimes the running clock */
    wr(qts, I2S0, INT_CLR, 0x1ffff);
    wr(qts, I2S0, SAMPLE_RATE_CONF, SR(8, 8, 16, 16));
    put_dscr(qts, DSCR_D, DW0_OWNER | DW0(8, 0), BUF_D, 0);
    wr(qts, I2S0, RXEOF_NUM, 2);
    wr(qts, I2S0, IN_LINK, LINK_START | LINK(DSCR_D));
    qtest_clock_step(qts, 4 * frame - 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_IN_SUC_EOF, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_IN_SUC_EOF, ==, I_IN_SUC_EOF);
    qtest_quit(qts);
}

/*
 * TX_STOP_EN: a master with nothing to send stops its clock, so the
 * receiver gets nothing more, and starts again when data arrives.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_stop_on_empty(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0);
    loop_i2s0_data(qts);
    clock_16bit(qts, I2S0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_RX_NUM(0));
    wr(qts, I2S0, CONF1, 0x89 | CONF1_TX_STOP_EN);
    wr(qts, I2S0, FIFO_WR, 0x11112222);
    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
       C_RX_START);
    qtest_clock_step(qts, 4 * FRAME_NS);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0x11112222);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_RX_REMPTY, ==, I_RX_REMPTY);
    g_assert_cmphex(rd(qts, I2S0, STATE) & 1, ==, 1);
    wr(qts, I2S0, INT_CLR, 0x1ffff);
    qtest_clock_step(qts, 4 * FRAME_NS);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_RX_TAKE_DATA, ==, 0);

    wr(qts, I2S0, FIFO_WR, 0x33334444);
    g_assert_cmphex(rd(qts, I2S0, STATE) & 1, ==, 0);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0x33334444);
    qtest_quit(qts);
}

/* Descriptor errors, the owner check, and an inlink out of descriptors */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_dma_errors(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_DSCR_EN);

    /* A descriptor outside DMA-capable SRAM */
    wr(qts, I2S0, OUT_LINK, LINK_START | 0x00100);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, I_OUT_DSCR_ERR);
    g_assert_cmphex(rd(qts, I2S0, OUTLINK_DSCR), ==, 0x3ff00100);

    /* A buffer outside it */
    wr(qts, I2S0, INT_CLR, 0x1ffff);
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(16, 16), 0x40080000, 0);
    wr(qts, I2S0, OUT_LINK, LINK_START | LINK(DSCR_A));
    g_assert_cmphex(rd(qts, I2S0, INT_RAW), ==, I_OUT_DSCR_ERR);

    /* CHECK_OWNER refuses a descriptor the CPU owns */
    wr(qts, I2S0, INT_CLR, 0x1ffff);
    put_dscr(qts, DSCR_C, DW0(16, 0), BUF_C, 0);
    wr(qts, I2S0, LC_CONF, LC_OUT_EOF_MODE | LC_CHECK_OWNER);
    wr(qts, I2S0, IN_LINK, LINK_START | LINK(DSCR_C));
    wr(qts, I2S0, FIFO_CONF, FC_FORCE);
    loop_i2s0_data(qts);
    clock_16bit(qts, I2S0);
    wr(qts, I2S0, FIFO_WR, 1);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_DSCR_EN);
    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
       C_RX_START);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_IN_DSCR_ERR, ==,
                    I_IN_DSCR_ERR);

    /*
     * Without it the same descriptor takes the word waiting in the FIFO,
     * and the next frame finds no descriptor left.
     */
    wr(qts, I2S0, INT_CLR, 0x1ffff);
    wr(qts, I2S0, LC_CONF, LC_OUT_EOF_MODE);
    put_dscr(qts, DSCR_C, DW0(4, 0), BUF_C, 0);
    wr(qts, I2S0, IN_LINK, LINK_START | LINK(DSCR_C));
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & (I_IN_DONE | I_IN_DSCR_EMPTY),
                    ==, I_IN_DONE);
    g_assert_cmphex(qtest_readl(qts, DSCR_C), ==, DW0(4, 4));
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_IN_DSCR_EMPTY, ==,
                    I_IN_DSCR_EMPTY);
    /* RESTART goes on from the next field of the last descriptor */
    put_dscr(qts, DSCR_D, DW0(4, 0), BUF_D, 0);
    qtest_writel(qts, DSCR_C + 8, DSCR_D);
    wr(qts, I2S0, IN_LINK, LINK_RESTART);
    g_assert_cmphex(qtest_readl(qts, DSCR_D), ==, DW0(4, 4));
    g_assert_cmphex(rd(qts, I2S0, INLINK_DSCR), ==, DSCR_D);

    /* STOP parks the link */
    wr(qts, I2S0, IN_LINK, LINK_STOP);
    g_assert_cmphex(rd(qts, I2S0, IN_LINK) & LINK_PARK, ==, LINK_PARK);
    qtest_quit(qts);
}

/*
 * The FIFO timeout: an outlink with more data than the FIFO holds and no
 * transmitter to take it raises TX_HUNG after LC_FIFO_TIMEOUT ticks of
 * 88000 APB_CLK cycles (40 MHz out of reset): 16 * 2.2 ms.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_hung(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0);
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0(400, 400), BUF_A, 0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_DSCR_EN);
    wr(qts, I2S0, OUT_LINK, LINK_START | LINK(DSCR_A));
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_TX_WFULL, ==, I_TX_WFULL);
    qtest_clock_step(qts, 35200000 - 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_TX_HUNG, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_TX_HUNG, ==, I_TX_HUNG);
    qtest_quit(qts);
}

/*
 * I2S0's master transmitter clocks I2S1's slave receiver through the GPIO
 * matrix: BCK, WS and data each go out on a pad and back in to I2S1.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_master_to_slave(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0 | PERIP_I2S1);
    route_out(qts, 18, SIG_I2S0O_BCK);
    route_in(qts, SIG_I2S1I_BCK, 18);
    route_out(qts, 19, SIG_I2S0O_WS);
    route_in(qts, SIG_I2S1I_WS, 19);
    route_out(qts, 21, SIG_I2S0_DATA(23));
    route_in(qts, SIG_I2S1_DATA(15), 21);
    clock_16bit(qts, I2S0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE);
    wr(qts, I2S1, FIFO_CONF, FC_FORCE);
    wr(qts, I2S1, CONF, C_DEFAULT | C_RX_SLAVE | C_RX_START);
    wr(qts, I2S0, FIFO_WR, 0xcafef00d);
    wr(qts, I2S0, FIFO_WR, 0x0badbeef);
    wr(qts, I2S0, CONF, C_DEFAULT | C_TX_START);
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_cmphex(rd(qts, I2S1, FIFO_RD), ==, 0xcafef00d);
    g_assert_cmphex(rd(qts, I2S1, FIFO_RD), ==, 0x0badbeef);
    qtest_quit(qts);
}

/*
 * A slave receiver clocked from outside: BCK, WS and data driven on pads,
 * an MSB-aligned frame of two 8-bit channels, left first.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_slave_rx_pads(void)
{
    QTestState *qts = start();
    const uint8_t left = 0xa5, right = 0x3c;

    enable(qts, PERIP_I2S1);
    route_in(qts, SIG_I2S1I_BCK, 18);
    route_in(qts, SIG_I2S1I_WS, 19);
    route_in(qts, SIG_I2S1_DATA(15), 21);
    wr(qts, I2S1, SAMPLE_RATE_CONF, SR(6, 6, 8, 8));
    wr(qts, I2S1, FIFO_CONF, FC_FORCE);
    drive(qts, 18, 1);
    drive(qts, 19, 1);
    wr(qts, I2S1, CONF, C_TX_MSB_RIGHT | C_RX_SLAVE | C_RX_START);
    for (int f = 0; f < 2; f++) {
        for (int i = 0; i < 16; i++) {
            uint8_t v = i < 8 ? left : right;

            drive(qts, 18, 0);
            drive(qts, 19, i >= 8);
            drive(qts, 21, (v >> (7 - (i % 8))) & 1);
            drive(qts, 18, 1);
        }
    }
    /* 16-bit dual channel, left in the high half without RX_MSB_RIGHT */
    g_assert_cmphex(rd(qts, I2S1, FIFO_RD), ==, 0xa5003c00);
    qtest_quit(qts);
}

/*
 * A slave transmitter clocked from outside drives each bit on BCK's
 * falling edge, MSB first after each change of WS.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_slave_tx_pads(void)
{
    QTestState *qts = start();
    uint32_t got[2] = { 0, 0 };

    enable(qts, PERIP_I2S1);
    route_in(qts, SIG_I2S1O_BCK, 18);
    route_in(qts, SIG_I2S1O_WS, 19);
    route_out(qts, 21, SIG_I2S1_DATA(23));
    qtest_irq_intercept_out_named(qts, GPIO_PATH, PAD_OUT);
    wr(qts, I2S1, SAMPLE_RATE_CONF, SR(6, 6, 8, 8));
    wr(qts, I2S1, FIFO_CONF, FC_FORCE);
    /* Left first, left in the high half */
    wr(qts, I2S1, FIFO_WR, 0x5a00c300);
    drive(qts, 18, 1);
    drive(qts, 19, 1);
    wr(qts, I2S1, CONF, C_TX_SLAVE | C_TX_START);
    for (int i = 0; i < 16; i++) {
        drive(qts, 19, i >= 8);
        drive(qts, 18, 0);
        got[i / 8] = (got[i / 8] << 1) | qtest_get_irq(qts, 21);
        drive(qts, 18, 1);
    }
    g_assert_cmphex(got[0], ==, 0x5a);
    g_assert_cmphex(got[1], ==, 0xc3);
    qtest_quit(qts);
}

/*
 * LCD mode: I2S1 drives each 8-bit datum on DATA_out[23:16] with WS as the
 * write strobe, one datum per WS cycle of two BCK cycles.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_lcd_tx(void)
{
    QTestState *qts = start();
    static const unsigned pads[8] = { 4, 5, 12, 13, 14, 15, 16, 17 };
    static const uint8_t data[4] = { 0x12, 0x34, 0xab, 0xcd };

    enable(qts, PERIP_I2S1);
    for (int b = 0; b < 8; b++) {
        route_out(qts, pads[b], SIG_I2S1_DATA(16 + b));
    }
    route_out(qts, 18, SIG_I2S1O_WS);
    qtest_irq_intercept_out_named(qts, GPIO_PATH, PAD_OUT);
    clock_16bit(qts, I2S1);
    wr(qts, I2S1, SAMPLE_RATE_CONF, SR(10, 10, 8, 8));
    /* 16-bit single channel, both channels from the FIFO */
    wr(qts, I2S1, FIFO_CONF, FC_FORCE | FC_TX_MOD(1));
    wr(qts, I2S1, CONF_CHAN, CHAN(0, 0));
    wr(qts, I2S1, CONF2, CONF2_LCD_EN);
    wr(qts, I2S1, FIFO_WR, 0x12003400);
    wr(qts, I2S1, FIFO_WR, 0xab00cd00);
    wr(qts, I2S1, CONF, C_DEFAULT | C_TX_START);
    for (int d = 0; d < 4; d++) {
        uint8_t bus = 0;

        qtest_clock_step(qts, d ? BCK_NS : 1);
        for (int b = 0; b < 8; b++) {
            bus |= qtest_get_irq(qts, pads[b]) << b;
        }
        g_assert_cmphex(bus, ==, data[d]);
        g_assert_false(qtest_get_irq(qts, 18));
        qtest_clock_step(qts, BCK_NS);
        g_assert_true(qtest_get_irq(qts, 18));
    }
    qtest_quit(qts);
}

/*
 * Camera mode: I2S0 samples DATA_in[7:0] on each rising PCLK edge while
 * V_SYNC and H_ENABLE are high (H_SYNC tied high in the matrix), packing
 * two 16-bit samples a word.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_camera(void)
{
    QTestState *qts = start();
    static const unsigned pads[8] = { 4, 5, 12, 13, 14, 15, 16, 17 };
    static const uint8_t pixels[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

    enable(qts, PERIP_I2S0);
    for (int b = 0; b < 8; b++) {
        route_in(qts, SIG_I2S0_DATA(b), pads[b]);
    }
    route_in(qts, SIG_I2S0I_WS, 18);
    route_in(qts, SIG_I2S0I_V_SYNC, 19);
    route_in(qts, SIG_I2S0I_H_ENABLE, 21);
    qtest_writel(qts, GPIO_FUNC_IN(SIG_I2S0I_H_SYNC),
                 IN_SIG_IN_SEL | IN_CONST_HIGH);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_RX_MOD(1));
    wr(qts, I2S0, CONF_CHAN, CHAN(0, 1));
    wr(qts, I2S0, CONF2, CONF2_LCD_EN | CONF2_CAMERA_EN);
    drive(qts, 18, 0);
    drive(qts, 19, 1);
    drive(qts, 21, 0);
    wr(qts, I2S0, CONF, C_RX_SLAVE | C_RX_START);
    for (int i = 0; i < 6; i++) {
        /* The first two pixels come with H_ENABLE low: not sampled */
        drive(qts, 21, i >= 2);
        for (int b = 0; b < 8; b++) {
            drive(qts, pads[b], (pixels[i] >> b) & 1);
        }
        drive(qts, 18, 1);
        drive(qts, 18, 0);
    }
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0x00330044);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0x00550066);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_RX_REMPTY, ==, I_RX_REMPTY);
    qtest_quit(qts);
}

/*
 * The A-law module: with TX_PCM_BYPASS clear and TX_PCM_CONF 1 the
 * transmitter compresses each channel's 16-bit sample to an 8-bit code
 * (4096 -> 0x85, -4096 -> 0x1a).
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_alaw(void)
{
    QTestState *qts = start();

    enable(qts, PERIP_I2S0);
    loop_i2s0_data(qts);
    wr(qts, I2S0, CLKM_CONF, CLKM(10, 0, 0));
    wr(qts, I2S0, SAMPLE_RATE_CONF, SR(10, 10, 8, 8));
    wr(qts, I2S0, FIFO_CONF, FC_FORCE);
    wr(qts, I2S0, CONF1, 0x80 | 0x1);
    wr(qts, I2S0, FIFO_WR, 0x1000f000);
    wr(qts, I2S0, CONF, C_DEFAULT | C_SIG_LOOPBACK | C_TX_START |
       C_RX_START);
    qtest_clock_step(qts, 16 * BCK_NS);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0x85001a00);
    qtest_quit(qts);
}

/*
 * PDM (I2S0 only): the transmitter takes a PCM frame per 64 * FP / FS
 * PDM clocks, the PDM clock running at BCK's rate (TRM 22.4.7); with the
 * reset FP 960 and FS 480 that is 128 BCK cycles. The receiver takes one
 * per 128 PDM clocks with RX_SINC_DSR_16_EN, as out of reset.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_pdm(void)
{
    QTestState *qts = start();
    const uint32_t pcm_ns = 128 * BCK_NS;

    enable(qts, PERIP_I2S0);
    clock_16bit(qts, I2S0);
    put_dscr(qts, DSCR_A, DW0_OWNER | DW0_EOF | DW0(16, 16), BUF_A, 0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE | FC_DSCR_EN | FC_RX_NUM(2));
    wr(qts, I2S0, LC_CONF, LC_OUT_EOF_MODE);
    wr(qts, I2S0, OUT_LINK, LINK_START | LINK(DSCR_A));
    wr(qts, I2S0, PDM_CONF, 0x01550020 | 0xf);
    wr(qts, I2S0, CONF, C_DEFAULT | C_TX_START | C_RX_START);
    qtest_clock_step(qts, 2 * pcm_ns);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_RX_TAKE_DATA, ==, 0);
    qtest_clock_step(qts, pcm_ns);
    /* With no audio backend the PDM receiver records silence */
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_RX_TAKE_DATA, ==,
                    I_RX_TAKE_DATA);
    g_assert_cmphex(rd(qts, I2S0, FIFO_RD), ==, 0);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_OUT_EOF, ==, 0);
    qtest_clock_step(qts, pcm_ns - 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_OUT_EOF, ==, 0);
    qtest_clock_step(qts, 1);
    g_assert_cmphex(rd(qts, I2S0, INT_RAW) & I_OUT_EOF, ==, I_OUT_EOF);
    qtest_quit(qts);
}

/*
 * The audio backend plays what a master transmitter sends: a wav file
 * written at the frame rate holds the samples.
 */
/* [spec:nuos:req:emu.esp32.i2s/test] */
static void test_audio_out(void)
{
    g_autofree char *path = NULL;
    g_autofree gchar *contents = NULL;
    gsize len;
    int fd = g_file_open_tmp("esp32-i2s-XXXXXX.wav", &path, NULL);
    QTestState *qts;
    bool found = false;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = qtest_initf("-M esp32,audiodev=snd -nic none "
                      "-audiodev wav,id=snd,path=%s,out.frequency=50000,"
                      "out.channels=2,out.format=s16", path);
    enable(qts, PERIP_I2S0);
    clock_16bit(qts, I2S0);
    wr(qts, I2S0, FIFO_CONF, FC_FORCE);
    for (int i = 0; i < 64; i++) {
        wr(qts, I2S0, FIFO_WR, 0x40001000);
    }
    wr(qts, I2S0, CONF, C_DEFAULT | C_TX_START);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    qtest_quit(qts);

    g_assert_true(g_file_get_contents(path, &contents, &len, NULL));
    unlink(path);
    g_assert_cmpuint(len, >, 44 + 4 * 64);
    /* The 16-bit stereo frames hold left 0x1000 and right 0x4000 */
    for (gsize i = 44; i + 4 <= len; i += 4) {
        int16_t l = (uint8_t)contents[i] | (contents[i + 1] << 8);
        int16_t r = (uint8_t)contents[i + 2] | (contents[i + 3] << 8);

        if (l == 0x1000 && r == 0x4000) {
            found = true;
            break;
        }
    }
    g_assert_true(found);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("esp32/i2s/reset-values", test_reset_values);
    qtest_add_func("esp32/i2s/gate-and-reset", test_gate_and_reset);
    qtest_add_func("esp32/i2s/interrupt-regs", test_interrupt_regs);
    qtest_add_func("esp32/i2s/cpu-loopback", test_cpu_loopback);
    qtest_add_func("esp32/i2s/formats", test_formats);
    qtest_add_func("esp32/i2s/format-mismatch", test_format_mismatch);
    qtest_add_func("esp32/i2s/chan-modes", test_chan_modes);
    qtest_add_func("esp32/i2s/dma-loopback", test_dma_loopback);
    qtest_add_func("esp32/i2s/sample-rate", test_sample_rate);
    qtest_add_func("esp32/i2s/stop-on-empty", test_stop_on_empty);
    qtest_add_func("esp32/i2s/dma-errors", test_dma_errors);
    qtest_add_func("esp32/i2s/hung", test_hung);
    qtest_add_func("esp32/i2s/master-to-slave", test_master_to_slave);
    qtest_add_func("esp32/i2s/slave-rx-pads", test_slave_rx_pads);
    qtest_add_func("esp32/i2s/slave-tx-pads", test_slave_tx_pads);
    qtest_add_func("esp32/i2s/lcd-tx", test_lcd_tx);
    qtest_add_func("esp32/i2s/camera", test_camera);
    qtest_add_func("esp32/i2s/alaw", test_alaw);
    qtest_add_func("esp32/i2s/pdm", test_pdm);
    qtest_add_func("esp32/i2s/audio-out", test_audio_out);

    return g_test_run();
}
