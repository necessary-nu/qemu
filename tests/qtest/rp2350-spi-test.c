/*
 * QTest testcase for the RP2350 SPI controllers (PL022)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * clk_peri is 150 MHz, so CPSDVSR 2 with SCR 74 makes one bit period a
 * microsecond. An 8-bit Motorola or TI frame then takes 9 us (the data
 * and one period of framing) and an 8-bit Microwire frame 17 us.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "rp2350-resets.h"

#define ALIAS_SET       0x2000
#define ALIAS_CLR       0x3000

#define SPI0            0x40080000
#define SPI1            0x40088000
#define SSPCR0          0x000
#define SSPCR1          0x004
#define SSPDR           0x008
#define SSPSR           0x00c
#define SSPCPSR         0x010
#define SSPIMSC         0x014
#define SSPRIS          0x018
#define SSPMIS          0x01c
#define SSPICR          0x020
#define SSPDMACR        0x024
#define SSPPERIPHID0    0xfe0

#define CR0_DSS(bits)   ((bits) - 1)
#define CR0_FRF_TI      (1u << 4)
#define CR0_FRF_MW      (2u << 4)
#define CR0_SPO         (1u << 6)
#define CR0_SPH         (1u << 7)
#define CR0_SCR(n)      ((n) << 8)
#define CR1_LBM         (1u << 0)
#define CR1_SSE         (1u << 1)
#define CR1_MS          (1u << 2)
#define CR1_SOD         (1u << 3)
#define SR_TFE          (1u << 0)
#define SR_TNF          (1u << 1)
#define SR_RNE          (1u << 2)
#define SR_RFF          (1u << 3)
#define SR_BSY          (1u << 4)
#define INT_ROR         (1u << 0)
#define INT_RT          (1u << 1)
#define INT_RX          (1u << 2)
#define INT_TX          (1u << 3)
#define DMACR_RXDMAE    (1u << 0)
#define DMACR_TXDMAE    (1u << 1)

/* One bit per microsecond. */
#define CPSR_1US        2
#define SCR_1US         74
#define US              1000
/*
 * clk_peri's period is not a whole number of nanoseconds, so each frame
 * ends up to a nanosecond early; checks either side of an edge allow
 * this much slack.
 */
#define SLACK           10

#define IO_BANK0        0x40028000
#define PADS_BANK0      0x40038000
#define STATUS(p)       (IO_BANK0 + 8 * (p))
#define CTRL(p)         (IO_BANK0 + 8 * (p) + 4)
#define PAD(p)          (PADS_BANK0 + 4 + 4 * (p))
#define STATUS_OUTTOPAD (1u << 9)
#define STATUS_OETOPAD  (1u << 13)
#define PAD_IE          (1u << 6)
#define PAD_DRIVE_4MA   (1u << 4)
#define FUNC_SPI        1
#define FUNC_SIO        5

#define SIO             0xd0000000
#define SIO_OUT_SET     (SIO + 0x018)
#define SIO_OUT_CLR     (SIO + 0x020)
#define SIO_OE_SET      (SIO + 0x038)

/* SPI0 on GPIOs 16 (RX), 17 (CSn), 18 (SCK) and 19 (TX). */
#define PIN_RX          16
#define PIN_CSN         17
#define PIN_SCK         18
#define PIN_TX          19

#define DMA             0x50000000
#define CH_READ(n)      (DMA + 0x40 * (n) + 0x00)
#define CH_WRITE(n)     (DMA + 0x40 * (n) + 0x04)
#define CH_COUNT(n)     (DMA + 0x40 * (n) + 0x08)
#define CH_CTRL_TRIG(n) (DMA + 0x40 * (n) + 0x0c)
#define CTRL_EN         (1u << 0)
#define CTRL_INCR_READ  (1u << 4)
#define CTRL_INCR_WRITE (1u << 6)
#define CTRL_CHAIN(n)   ((n) << 13)
#define CTRL_TREQ(n)    ((n) << 17)
#define CTRL_BUSY       (1u << 26)
#define DREQ_SPI0_TX    24
#define DREQ_SPI0_RX    25

#define SRAM            0x20000000

#define SPI0_IRQ        31

/* The W25Q80BL's JEDEC ID: Winbond, W25Q 1.8 V, 8 Mbit. */
#define FLASH_DEVICE    "w25q80bl"
#define FLASH_JEDEC     0xef4014

static char *rom_path;

static QTestState *start(const char *extra)
{
    QTestState *qts = qtest_initf("-M rp2350 -bios %s %s", rom_path,
                                  extra ? extra : "");

    rp2350_unreset(qts, RP2350_RESETS_ALL);

    qtest_irq_intercept_in(qts, "/machine/soc/armv7m[0]");
    return qts;
}

static uint32_t rd(QTestState *qts, uint32_t spi, uint32_t reg)
{
    return qtest_readl(qts, spi + reg);
}

static void wr(QTestState *qts, uint32_t spi, uint32_t reg, uint32_t val)
{
    qtest_writel(qts, spi + reg, val);
}

/* A 1 us-per-bit master with `cr0` format bits, enabled with `cr1`. */
static void setup(QTestState *qts, uint32_t spi, uint32_t cr0, uint32_t cr1)
{
    wr(qts, spi, SSPCPSR, CPSR_1US);
    wr(qts, spi, SSPCR0, CR0_SCR(SCR_1US) | cr0);
    wr(qts, spi, SSPCR1, cr1);
}

static void step(QTestState *qts, int64_t ns)
{
    qtest_clock_step(qts, ns);
}

/* Route pin p to function f, with the pad's input on and isolation off. */
static void route(QTestState *qts, int p, int f)
{
    qtest_writel(qts, PAD(p), PAD_IE | PAD_DRIVE_4MA);
    qtest_writel(qts, CTRL(p), f);
}

static void drive(QTestState *qts, int p, int level)
{
    qtest_set_irq_in(qts, "/machine/soc/gpio", "pad-in", p, level);
}

/* [spec:nuos:req:emu.spi/test] */
static void test_reset_values(void)
{
    static const uint8_t id[] = {
        0x22, 0x10, 0x34, 0x00, 0x0d, 0xf0, 0x05, 0xb1,
    };
    QTestState *qts = start(NULL);
    uint32_t spi;
    int i;

    for (spi = SPI0; spi <= SPI1; spi += SPI1 - SPI0) {
        g_assert_cmphex(rd(qts, spi, SSPCR0), ==, 0);
        g_assert_cmphex(rd(qts, spi, SSPCR1), ==, 0);
        g_assert_cmphex(rd(qts, spi, SSPSR), ==, SR_TNF | SR_TFE);
        g_assert_cmphex(rd(qts, spi, SSPCPSR), ==, 0);
        g_assert_cmphex(rd(qts, spi, SSPIMSC), ==, 0);
        g_assert_cmphex(rd(qts, spi, SSPRIS), ==, INT_TX);
        g_assert_cmphex(rd(qts, spi, SSPMIS), ==, 0);
        g_assert_cmphex(rd(qts, spi, SSPDMACR), ==, 0);
        for (i = 0; i < ARRAY_SIZE(id); i++) {
            g_assert_cmphex(rd(qts, spi, SSPPERIPHID0 + 4 * i), ==, id[i]);
        }
    }

    /* Reserved bits read zero; CPSDVSR's LSB always reads zero. */
    wr(qts, SPI0, SSPCR0, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI0, SSPCR0), ==, 0xffff);
    wr(qts, SPI0, SSPCPSR, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI0, SSPCPSR), ==, 0xfe);
    wr(qts, SPI0, SSPIMSC, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI0, SSPIMSC), ==, 0xf);
    wr(qts, SPI0, SSPDMACR, 0xffffffff);
    g_assert_cmphex(rd(qts, SPI0, SSPDMACR), ==, 0x3);
    /* SET and CLR aliases. */
    wr(qts, SPI0, SSPCR1 + ALIAS_SET, CR1_LBM);
    g_assert_cmphex(rd(qts, SPI0, SSPCR1), ==, CR1_LBM);
    wr(qts, SPI0, SSPCR1 + ALIAS_CLR, CR1_LBM);
    g_assert_cmphex(rd(qts, SPI0, SSPCR1), ==, 0);
    qtest_quit(qts);
}

/*
 * A loopback frame occupies the shifter for its line time: the transmit
 * FIFO empties at once, BSY holds for the frame, and the received word
 * arrives at its end.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_loopback_timing(void)
{
    QTestState *qts = start(NULL);

    setup(qts, SPI0, CR0_DSS(8), CR1_SSE | CR1_LBM);
    wr(qts, SPI0, SSPDR, 0x5a);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_BSY | SR_TNF | SR_TFE);
    step(qts, 9 * US - SLACK);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_BSY | SR_TNF | SR_TFE);
    step(qts, SLACK);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_RNE | SR_TNF | SR_TFE);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 0x5a);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_TNF | SR_TFE);

    /* Back-to-back frames: four bytes in 36 us. */
    wr(qts, SPI0, SSPDR, 1);
    wr(qts, SPI0, SSPDR, 2);
    wr(qts, SPI0, SSPDR, 3);
    wr(qts, SPI0, SSPDR, 4);
    step(qts, 27 * US);
    g_assert_cmphex(rd(qts, SPI0, SSPSR) & (SR_BSY | SR_TFE), ==,
                    SR_BSY | SR_TFE);
    step(qts, 9 * US);
    g_assert_false(rd(qts, SPI0, SSPSR) & SR_BSY);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 1);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 2);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 3);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 4);
    g_assert_false(rd(qts, SPI0, SSPSR) & SR_RNE);

    /* The bit rate follows CPSDVSR and SCR: 16 bits at 4 us per bit. */
    wr(qts, SPI0, SSPCPSR, 8);
    wr(qts, SPI0, SSPCR0, CR0_SCR(SCR_1US) | CR0_DSS(16));
    wr(qts, SPI0, SSPDR, 0xbeef);
    step(qts, 17 * 4 * US - SLACK);
    g_assert_true(rd(qts, SPI0, SSPSR) & SR_BSY);
    step(qts, SLACK);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 0xbeef);
    qtest_quit(qts);
}

/* Data sizes from 4 to 16 bits; unused high bits are ignored. */
/* [spec:nuos:req:emu.spi/test] */
static void test_data_sizes(void)
{
    QTestState *qts = start(NULL);
    int bits;

    for (bits = 4; bits <= 16; bits++) {
        setup(qts, SPI0, CR0_DSS(bits), CR1_SSE | CR1_LBM);
        wr(qts, SPI0, SSPDR, 0xffff);
        step(qts, (bits + 1) * US);
        g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, (1u << bits) - 1);
    }
    qtest_quit(qts);
}

/* TI frames take one period more than the data, Microwire nine more. */
/* [spec:nuos:req:emu.spi/test] */
static void test_frame_formats(void)
{
    QTestState *qts = start(NULL);

    setup(qts, SPI0, CR0_DSS(8) | CR0_FRF_TI, CR1_SSE | CR1_LBM);
    wr(qts, SPI0, SSPDR, 0xc3);
    step(qts, 9 * US - SLACK);
    g_assert_true(rd(qts, SPI0, SSPSR) & SR_BSY);
    step(qts, SLACK);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 0xc3);

    /*
     * Microwire: the 8-bit control word, a turnaround period and the
     * 8-bit response. In loopback the response phase sees the transmit
     * line, which is low after the control word.
     */
    wr(qts, SPI0, SSPCR1, 0);
    setup(qts, SPI0, CR0_DSS(8) | CR0_FRF_MW, CR1_SSE | CR1_LBM);
    wr(qts, SPI0, SSPDR, 0xc3);
    step(qts, 17 * US - SLACK);
    g_assert_true(rd(qts, SPI0, SSPSR) & SR_BSY);
    step(qts, SLACK);
    g_assert_cmphex(rd(qts, SPI0, SSPSR) & (SR_BSY | SR_RNE), ==, SR_RNE);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 0);
    qtest_quit(qts);
}

/*
 * The FIFOs are eight deep. Writes to a full transmit FIFO are lost; a
 * frame received into a full receive FIFO overruns it.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_fifos_overrun(void)
{
    QTestState *qts = start(NULL);
    int i;

    /* Primed while disabled: the transmit FIFO holds data, BSY is set. */
    setup(qts, SPI0, CR0_DSS(8), CR1_LBM);
    for (i = 0; i < 9; i++) {
        wr(qts, SPI0, SSPDR, i);
    }
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_BSY);
    step(qts, 100 * US);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_BSY);

    wr(qts, SPI0, SSPCR1 + ALIAS_SET, CR1_SSE);
    step(qts, 8 * 9 * US);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_RFF | SR_RNE | SR_TNF |
                    SR_TFE);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_ROR);

    wr(qts, SPI0, SSPDR, 0xaa);
    step(qts, 9 * US);
    g_assert_true(rd(qts, SPI0, SSPRIS) & INT_ROR);
    for (i = 0; i < 8; i++) {
        g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, i);
    }
    g_assert_false(rd(qts, SPI0, SSPSR) & SR_RNE);
    wr(qts, SPI0, SSPICR, INT_ROR);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_ROR);
    qtest_quit(qts);
}

/*
 * TX: four or fewer entries in the transmit FIFO. RX: four or more in the
 * receive FIFO. RT: data left unread for 32 bit periods. All reach the
 * NVIC through the combined SPI0_IRQ.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_interrupts(void)
{
    QTestState *qts = start(NULL);
    int i;

    setup(qts, SPI0, CR0_DSS(8), CR1_LBM);
    g_assert_false(qtest_get_irq(qts, SPI0_IRQ));
    wr(qts, SPI0, SSPIMSC, INT_TX);
    g_assert_true(qtest_get_irq(qts, SPI0_IRQ));
    g_assert_cmphex(rd(qts, SPI0, SSPMIS), ==, INT_TX);
    for (i = 0; i < 5; i++) {
        wr(qts, SPI0, SSPDR, i);
    }
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_TX);
    g_assert_false(qtest_get_irq(qts, SPI0_IRQ));

    wr(qts, SPI0, SSPIMSC, INT_RX);
    wr(qts, SPI0, SSPCR1 + ALIAS_SET, CR1_SSE);
    step(qts, 3 * 9 * US);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_RX);
    g_assert_false(qtest_get_irq(qts, SPI0_IRQ));
    step(qts, 9 * US);
    g_assert_true(rd(qts, SPI0, SSPRIS) & INT_RX);
    g_assert_true(qtest_get_irq(qts, SPI0_IRQ));
    step(qts, 9 * US);
    for (i = 0; i < 4; i++) {
        rd(qts, SPI0, SSPDR);
    }
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_RX);
    g_assert_false(qtest_get_irq(qts, SPI0_IRQ));

    /* One entry left; the timeout counts from the last read. */
    wr(qts, SPI0, SSPIMSC, INT_RT);
    step(qts, 32 * US - SLACK);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_RT);
    step(qts, SLACK);
    g_assert_true(rd(qts, SPI0, SSPRIS) & INT_RT);
    g_assert_true(qtest_get_irq(qts, SPI0_IRQ));
    wr(qts, SPI0, SSPICR, INT_RT);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_RT);
    g_assert_false(qtest_get_irq(qts, SPI0_IRQ));

    /* Emptying the receive FIFO also ends the timeout. */
    wr(qts, SPI0, SSPDR, 0);
    step(qts, 9 * US + 32 * US);
    g_assert_true(rd(qts, SPI0, SSPRIS) & INT_RT);
    rd(qts, SPI0, SSPDR);
    g_assert_true(rd(qts, SPI0, SSPRIS) & INT_RT);
    rd(qts, SPI0, SSPDR);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_RT);
    step(qts, 100 * US);
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_RT);

    /* ROR reaches the line too. */
    wr(qts, SPI0, SSPIMSC, INT_ROR);
    for (i = 0; i < 9; i++) {
        wr(qts, SPI0, SSPDR, i);
        step(qts, 9 * US);
    }
    g_assert_true(qtest_get_irq(qts, SPI0_IRQ));
    g_assert_cmphex(rd(qts, SPI0, SSPMIS), ==, INT_ROR);
    qtest_quit(qts);
}

/*
 * Pins: a master drives SCK at its idle polarity and CSn high, and
 * enables TX only while a frame is on the wire; a slave drives neither
 * SCK nor CSn.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_pins(void)
{
    QTestState *qts = start(NULL);

    route(qts, PIN_CSN, FUNC_SPI);
    route(qts, PIN_SCK, FUNC_SPI);
    route(qts, PIN_TX, FUNC_SPI);

    g_assert_cmphex(qtest_readl(qts, STATUS(PIN_SCK)) &
                    (STATUS_OETOPAD | STATUS_OUTTOPAD), ==, STATUS_OETOPAD);
    g_assert_cmphex(qtest_readl(qts, STATUS(PIN_CSN)) &
                    (STATUS_OETOPAD | STATUS_OUTTOPAD), ==,
                    STATUS_OETOPAD | STATUS_OUTTOPAD);
    g_assert_false(qtest_readl(qts, STATUS(PIN_TX)) & STATUS_OETOPAD);

    setup(qts, SPI0, CR0_DSS(8) | CR0_SPO, CR1_SSE);
    g_assert_true(qtest_readl(qts, STATUS(PIN_SCK)) & STATUS_OUTTOPAD);
    wr(qts, SPI0, SSPDR, 0);
    g_assert_cmphex(qtest_readl(qts, STATUS(PIN_CSN)) &
                    (STATUS_OETOPAD | STATUS_OUTTOPAD), ==, STATUS_OETOPAD);
    g_assert_true(qtest_readl(qts, STATUS(PIN_TX)) & STATUS_OETOPAD);
    step(qts, 9 * US);
    g_assert_true(qtest_readl(qts, STATUS(PIN_CSN)) & STATUS_OUTTOPAD);
    g_assert_false(qtest_readl(qts, STATUS(PIN_TX)) & STATUS_OETOPAD);

    /* MS only changes while SSE is clear. */
    wr(qts, SPI0, SSPCR1, CR1_SSE | CR1_MS);
    g_assert_cmphex(rd(qts, SPI0, SSPCR1), ==, CR1_SSE);
    wr(qts, SPI0, SSPCR1, 0);
    wr(qts, SPI0, SSPCR1, CR1_MS);
    g_assert_false(qtest_readl(qts, STATUS(PIN_SCK)) & STATUS_OETOPAD);
    g_assert_false(qtest_readl(qts, STATUS(PIN_CSN)) & STATUS_OETOPAD);
    g_assert_false(qtest_readl(qts, STATUS(PIN_TX)) & STATUS_OETOPAD);
    qtest_quit(qts);
}

/* Clock out `n` bytes of `out` and collect what comes back. */
static void exchange(QTestState *qts, const uint8_t *out, uint8_t *in, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        wr(qts, SPI0, SSPDR, out[i]);
    }
    step(qts, n * 9 * US);
    for (i = 0; i < n; i++) {
        g_assert_true(rd(qts, SPI0, SSPSR) & SR_RNE);
        in[i] = rd(qts, SPI0, SSPDR);
    }
}

static uint32_t jedec(const uint8_t *in)
{
    return in[1] << 16 | in[2] << 8 | in[3];
}

/*
 * A W25Q80BL on SPI0 with its chip select on GPIO 17. JEDEC ID 0x9f
 * returns the manufacturer and device ID bytes.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_flash_jedec(void)
{
    static const uint8_t cmd[4] = { 0x9f };
    QTestState *qts = start("-device " FLASH_DEVICE ",bus=spi0,cs=17");
    uint8_t in[4];

    route(qts, PIN_RX, FUNC_SPI);
    route(qts, PIN_SCK, FUNC_SPI);
    route(qts, PIN_TX, FUNC_SPI);

    /* Chip select from SIO, as pico-sdk examples do it. */
    qtest_writel(qts, SIO_OUT_SET, 1u << PIN_CSN);
    qtest_writel(qts, SIO_OE_SET, 1u << PIN_CSN);
    route(qts, PIN_CSN, FUNC_SIO);
    setup(qts, SPI0, CR0_DSS(8), CR1_SSE);

    qtest_writel(qts, SIO_OUT_CLR, 1u << PIN_CSN);
    exchange(qts, cmd, in, 4);
    qtest_writel(qts, SIO_OUT_SET, 1u << PIN_CSN);
    g_assert_cmphex(jedec(in), ==, FLASH_JEDEC);

    /* Deselected, the flash does not answer. */
    exchange(qts, cmd, in, 4);
    g_assert_cmphex(jedec(in), ==, 0);

    /*
     * The controller's own CSn: with SPH=1 it stays low across
     * back-to-back frames, so the whole command is one transaction.
     */
    route(qts, PIN_CSN, FUNC_SPI);
    wr(qts, SPI0, SSPCR1, 0);
    setup(qts, SPI0, CR0_DSS(8) | CR0_SPO | CR0_SPH, CR1_SSE);
    exchange(qts, cmd, in, 4);
    g_assert_cmphex(jedec(in), ==, FLASH_JEDEC);

    /*
     * With SPH=0 CSn pulses high between frames, which ends the flash's
     * command after its first byte.
     */
    wr(qts, SPI0, SSPCR1, 0);
    setup(qts, SPI0, CR0_DSS(8), CR1_SSE);
    exchange(qts, cmd, in, 4);
    g_assert_cmphex(jedec(in), !=, FLASH_JEDEC);
    qtest_quit(qts);
}

/*
 * DMA paced by the SPI0 DREQs: one channel feeds the transmit FIFO from
 * SRAM, another drains the receive FIFO back to SRAM, in loopback.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_dma(void)
{
    QTestState *qts = start(NULL);
    uint8_t src[32], dst[32];
    int i;

    for (i = 0; i < sizeof(src); i++) {
        src[i] = 0x30 + i * 7;
    }
    qtest_memwrite(qts, SRAM, src, sizeof(src));
    qtest_memset(qts, SRAM + 0x100, 0, sizeof(dst));

    setup(qts, SPI0, CR0_DSS(8), CR1_SSE | CR1_LBM);
    wr(qts, SPI0, SSPDMACR, DMACR_TXDMAE | DMACR_RXDMAE);

    qtest_writel(qts, CH_READ(1), SPI0 + SSPDR);
    qtest_writel(qts, CH_WRITE(1), SRAM + 0x100);
    qtest_writel(qts, CH_COUNT(1), sizeof(dst));
    qtest_writel(qts, CH_CTRL_TRIG(1), CTRL_EN | CTRL_INCR_WRITE |
                 CTRL_CHAIN(1) | CTRL_TREQ(DREQ_SPI0_RX));
    qtest_writel(qts, CH_READ(0), SRAM);
    qtest_writel(qts, CH_WRITE(0), SPI0 + SSPDR);
    qtest_writel(qts, CH_COUNT(0), sizeof(src));
    qtest_writel(qts, CH_CTRL_TRIG(0), CTRL_EN | CTRL_INCR_READ |
                 CTRL_CHAIN(0) | CTRL_TREQ(DREQ_SPI0_TX));

    /* The line rate paces the whole transfer. */
    step(qts, 16 * 9 * US);
    g_assert_true(qtest_readl(qts, CH_CTRL_TRIG(1)) & CTRL_BUSY);
    step(qts, 16 * 9 * US + 10 * US);
    g_assert_false(qtest_readl(qts, CH_CTRL_TRIG(0)) & CTRL_BUSY);
    g_assert_false(qtest_readl(qts, CH_CTRL_TRIG(1)) & CTRL_BUSY);
    qtest_memread(qts, SRAM + 0x100, dst, sizeof(dst));
    g_assert_cmpmem(dst, sizeof(dst), src, sizeof(src));
    g_assert_false(rd(qts, SPI0, SSPRIS) & INT_ROR);
    qtest_quit(qts);
}

/*
 * SPI0 in slave mode on SPI1's bus. The slave answers while its CSn pin
 * is low, from its transmit FIFO, unless SOD keeps it off the line.
 */
/* [spec:nuos:req:emu.spi/test] */
static void test_slave(void)
{
    QTestState *qts = start("-device pl022-target,bus=spi1,"
                            "controller=/machine/soc/spi[0]");

    route(qts, PIN_CSN, FUNC_SPI);
    drive(qts, PIN_CSN, 0);
    setup(qts, SPI0, CR0_DSS(8), CR1_MS);
    wr(qts, SPI0, SSPDR, 0xa5);
    wr(qts, SPI0, SSPCR1 + ALIAS_SET, CR1_SSE);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_BSY | SR_TNF);
    setup(qts, SPI1, CR0_DSS(8), CR1_SSE);

    wr(qts, SPI1, SSPDR, 0x3c);
    step(qts, 9 * US);
    g_assert_cmphex(rd(qts, SPI1, SSPDR), ==, 0xa5);
    g_assert_cmphex(rd(qts, SPI0, SSPSR), ==, SR_RNE | SR_TNF | SR_TFE);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 0x3c);

    /* Deselected: the slave neither receives nor answers. */
    drive(qts, PIN_CSN, 1);
    wr(qts, SPI0, SSPDR, 0x11);
    wr(qts, SPI1, SSPDR, 0x22);
    step(qts, 9 * US);
    g_assert_cmphex(rd(qts, SPI1, SSPDR), ==, 0);
    g_assert_false(rd(qts, SPI0, SSPSR) & SR_RNE);

    /* SOD: the slave receives but keeps off the line. */
    drive(qts, PIN_CSN, 0);
    wr(qts, SPI0, SSPCR1 + ALIAS_SET, CR1_SOD);
    wr(qts, SPI1, SSPDR, 0x33);
    step(qts, 9 * US);
    g_assert_cmphex(rd(qts, SPI1, SSPDR), ==, 0);
    g_assert_cmphex(rd(qts, SPI0, SSPDR), ==, 0x33);
    g_assert_true(rd(qts, SPI0, SSPSR) & SR_TFE);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const uint32_t blank[2];
    GError *err = NULL;
    int fd, ret;

    g_test_init(&argc, &argv, NULL);

    fd = g_file_open_tmp("rp2350-spi-test-XXXXXX.bin", &rom_path, &err);
    g_assert_no_error(err);
    g_assert_cmpint(write(fd, blank, sizeof(blank)), ==, sizeof(blank));
    close(fd);

    qtest_add_func("/rp2350/spi/reset-values", test_reset_values);
    qtest_add_func("/rp2350/spi/loopback-timing", test_loopback_timing);
    qtest_add_func("/rp2350/spi/data-sizes", test_data_sizes);
    qtest_add_func("/rp2350/spi/frame-formats", test_frame_formats);
    qtest_add_func("/rp2350/spi/fifos-overrun", test_fifos_overrun);
    qtest_add_func("/rp2350/spi/interrupts", test_interrupts);
    qtest_add_func("/rp2350/spi/pins", test_pins);
    qtest_add_func("/rp2350/spi/flash-jedec", test_flash_jedec);
    qtest_add_func("/rp2350/spi/dma", test_dma);
    qtest_add_func("/rp2350/spi/slave", test_slave);

    ret = g_test_run();
    unlink(rom_path);
    g_free(rom_path);
    return ret;
}
