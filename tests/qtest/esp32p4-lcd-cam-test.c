/*
 * QTest testcase for the ESP32-P4 LCD and camera controller (LCD_CAM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The LCD's CS, DC, HSYNC, VSYNC and DE go through the GPIO matrix to
 * pads, where the test reads them (GPIO_IN) at exact virtual times. Its
 * data moves through GDMA-AXI's channel 0 with descriptors in L2MEM past
 * the L2 cache's 128 KiB; the board's panels are on the bus "lcd", their
 * consoles read back with QMP's screendump. The camera takes a sensor
 * (esp32p4-image-source) on pads the test routes, into GDMA-AXI's RX
 * channel 0. LCD_CLK and CAM_CLK come from XTAL_CLK, 40 MHz, so that
 * every edge is on a whole 25 ns.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define LCDCAM              0x500dc000
#define LCD_CLOCK           (LCDCAM + 0x00)
#define CAM_CTRL            (LCDCAM + 0x04)
#define CAM_CTRL1           (LCDCAM + 0x08)
#define CAM_RGB_YUV         (LCDCAM + 0x0c)
#define LCD_RGB_YUV         (LCDCAM + 0x10)
#define LCD_USER            (LCDCAM + 0x14)
#define LCD_MISC            (LCDCAM + 0x18)
#define LCD_CTRL            (LCDCAM + 0x1c)
#define LCD_CTRL1           (LCDCAM + 0x20)
#define LCD_CTRL2           (LCDCAM + 0x24)
#define LCD_FIRST_CMD       (LCDCAM + 0x28)
#define LCD_LATTER_CMD      (LCDCAM + 0x2c)
#define LCD_DLY_MODE_CFG1   (LCDCAM + 0x30)
#define LCD_DLY_MODE_CFG2   (LCDCAM + 0x38)
#define INT_ENA             (LCDCAM + 0x64)
#define INT_RAW             (LCDCAM + 0x68)
#define INT_ST              (LCDCAM + 0x6c)
#define INT_CLR             (LCDCAM + 0x70)
#define LC_DATE             (LCDCAM + 0xfc)

/* LCD_CLOCK */
#define CLKCNT_N(n)         ((uint32_t)(n) << 0)
#define CLK_EQU_SYSCLK      (1u << 6)
/* LCD_USER */
#define DOUT_CYCLELEN(n)    ((uint32_t)(n) << 0)
#define ALWAYS_OUT_EN       (1u << 13)
#define SWIZZLE_MODE(n)     ((uint32_t)(n) << 14)
#define SWIZZLE_EN          (1u << 17)
#define DOUT_BIT_ORDER      (1u << 18)
#define LCD_BYTE_MODE(n)        ((uint32_t)(n) << 19)
#define LCD_UPDATE          (1u << 21)
#define LCD_BIT_ORDER           (1u << 22)
#define LCD_BYTE_ORDER          (1u << 23)
#define DOUT                (1u << 24)
#define DUMMY               (1u << 25)
#define CMD                 (1u << 26)
#define LCD_START           (1u << 27)
#define LCD_RESET           (1u << 28)
#define DUMMY_CYCLELEN(n)   ((uint32_t)(n) << 29)
#define CMD_2_CYCLE_EN      (1u << 31)
/* LCD_MISC */
#define WIRE_MODE(n)        ((uint32_t)(n) << 4)
#define VFK_CYCLELEN(n)     ((uint32_t)(n) << 6)
#define VBK_CYCLELEN(n)     ((uint32_t)(n) << 12)
#define NEXT_FRAME_EN       (1u << 25)
#define BK_EN               (1u << 26)
#define LCD_AFIFO_RESET     (1u << 27)
#define CD_DATA_SET         (1u << 28)
#define CD_DUMMY_SET        (1u << 29)
#define CD_CMD_SET          (1u << 30)
#define CD_IDLE_EDGE        (1u << 31)
/* LCD_CTRL, LCD_CTRL1, LCD_CTRL2 */
#define HB_FRONT(n)         ((uint32_t)(n) << 0)
#define VA_HEIGHT(n)        ((uint32_t)(n) << 11)
#define VT_HEIGHT(n)        ((uint32_t)(n) << 21)
#define RGB_MODE_EN         (1u << 31)
#define VB_FRONT(n)         ((uint32_t)(n) << 0)
#define HA_WIDTH(n)         ((uint32_t)(n) << 8)
#define HT_WIDTH(n)         ((uint32_t)(n) << 20)
#define VSYNC_WIDTH(n)      ((uint32_t)(n) << 0)
#define VSYNC_IDLE_POL      (1u << 7)
#define DE_IDLE_POL         (1u << 8)
#define HS_BLANK_EN         (1u << 9)
#define HSYNC_WIDTH(n)      ((uint32_t)(n) << 16)
#define HSYNC_IDLE_POL      (1u << 23)
#define HSYNC_POSITION(n)   ((uint32_t)(n) << 24)
/* CAM_CTRL, CAM_CTRL1 */
#define CAM_UPDATE          (1u << 4)
#define CAM_LINE_INT_EN     (1u << 7)
#define CAM_VS_EOF_EN       (1u << 8)
#define REC_DATA_BYTELEN(n) ((uint32_t)(n) << 0)
#define LINE_INT_NUM(n)     ((uint32_t)(n) << 16)
#define CAM_2BYTE_EN        (1u << 24)
#define CAM_VSYNC_INV       (1u << 27)
#define CAM_START           (1u << 29)
#define CAM_RESET           (1u << 30)
#define CAM_AFIFO_RESET     (1u << 31)
/* Interrupts */
#define INT_LCD_VSYNC       (1u << 0)
#define INT_LCD_TRANS_DONE  (1u << 1)
#define INT_CAM_VSYNC       (1u << 2)
#define INT_CAM_HS          (1u << 3)
#define INT_LCD_UNDERRUN    (1u << 4)

/* Clocks and resets */
#define CLKRST              0x500e6000
#define SOC_CLK_CTRL3       (CLKRST + 0x20)
#define PERI_CLK_CTRL19     (CLKRST + 0x64)
#define PERI_CLK_CTRL110    (CLKRST + 0x68)
#define PERI_CLK_CTRL119    (CLKRST + 0x8c)
#define PERI_CLK_CTRL120    (CLKRST + 0x90)
#define HP_RST_EN2          (CLKRST + 0xc8)
#define LCDCAM_APB_CLK_EN   (1u << 1)
#define RST_EN_LCDCAM       (1u << 9)

/* The GPIO matrix and IO MUX */
#define GPIO                0x500e0000
#define GPIO_IN             (GPIO + 0x03c)
#define GPIO_IN1            (GPIO + 0x040)
#define GPIO_FUNC_IN(s)     (GPIO + 0x158 + 4 * (s))
#define GPIO_FUNC_OUT(n)    (GPIO + 0x558 + 4 * (n))
#define IN_SIG_IN_SEL       (1u << 7)
#define IN_INV              (1u << 6)
#define OUT_INV             (1u << 9)
#define IO_MUX_GPIO(n)      (0x500e1004 + 4 * (n))
#define FUN_IE              (1u << 9)
#define MCU_SEL_GPIO        (1u << 12)
#define USJ_CONF0           0x500d2018u
#define USJ_PAD_ENABLE      (1u << 14)

#define SIG_LCD_CS          144
#define SIG_LCD_DC          145
#define SIG_LCD_PCLK        149
#define SIG_CAM_CLK         150
#define SIG_LCD_DE          151
#define SIG_LCD_HSYNC       152
#define SIG_LCD_VSYNC       153
#define SIG_LCD_DATA(n)     (154 + (n))
#define SIG_CAM_PCLK        158
#define SIG_CAM_DE          159
#define SIG_CAM_HSYNC       160
#define SIG_CAM_VSYNC       161
#define SIG_CAM_DATA(n)     (162 + (n))

/* Pads: the data lines on 0 upwards, the rest above. */
#define PAD_CS              20
#define PAD_DC              21
#define PAD_PCLK            22
#define PAD_HSYNC           23
#define PAD_VSYNC           24
#define PAD_DE              25
#define PAD_CAM_CLK         26

/* GDMA-AXI channel 0 */
#define AXI                 0x5008a000
#define AXI_IN              (AXI + 0x00)
#define AXI_OUT             (AXI + 0x138)
#define X_RAW               0x00
#define X_CLR               0x0c
#define X_CONF0             0x10
#define X_LINK1             0x20
#define X_LINK2             0x24
#define X_PERI_SEL          0x44
#define IN_LINK_START       (1u << 2)
#define OUT_LINK_START      (1u << 1)
#define IN_DONE             (1u << 0)
#define IN_SUC_EOF          (1u << 1)
#define OUT_DONE            (1u << 0)
#define OUT_EOF             (1u << 1)
#define OUT_TOTAL_EOF       (1u << 3)
#define PERI_LCD_CAM        0
#define DW0(size, len)      ((size) | (uint32_t)(len) << 12)
#define SUC_EOF             (1u << 30)
#define OWNER               (1u << 31)

#define INTMTX_STATUS(w)    (0x500d6000 + 0x200 + 4 * (w))
#define SRC_LCD_CAM         36

#define MEM                 0x4ff40000
#define TX_DESC             MEM
#define RX_DESC             (MEM + 0x100)
#define SRC                 (MEM + 0x1000)
#define DST                 (MEM + 0x8000)

static const char *machine_args = "-M esp32p4,flash-size=16M,strap-mode=0";

/* The virtual time, which only the test's steps advance under qtest. */
static int64_t vnow;

static QTestState *start_with(const char *extra)
{
    QTestState *qts = qtest_initf("%s %s", machine_args, extra);

    vnow = 0;
    qtest_writel(qts, SOC_CLK_CTRL3,
                 qtest_readl(qts, SOC_CLK_CTRL3) | LCDCAM_APB_CLK_EN);
    return qts;
}

static QTestState *start(void)
{
    return start_with("");
}

/* LCD_CLK: XTAL (40 MHz) / @div, or stopped. */
static void lcd_clock(QTestState *qts, unsigned div, bool on)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL19) & ~(7u << 29);

    qtest_writel(qts, PERI_CLK_CTRL110,
                 (qtest_readl(qts, PERI_CLK_CTRL110) & ~0xffffffu) |
                 (div - 1));
    qtest_writel(qts, PERI_CLK_CTRL19, v | (on ? 1u << 31 : 0));
}

/* CAM_CLK: XTAL / @div (the TRM's encoding: the divisor itself). */
static void cam_clock(QTestState *qts, unsigned div, bool on)
{
    uint32_t v = qtest_readl(qts, PERI_CLK_CTRL119) & ~(7u << 27);

    qtest_writel(qts, PERI_CLK_CTRL120,
                 (qtest_readl(qts, PERI_CLK_CTRL120) & ~0xffffffu) | div);
    qtest_writel(qts, PERI_CLK_CTRL119, v | (on ? 1u << 29 : 0));
}

/*
 * ESP-IDF's gpio_ll_func_sel(): GPIO24 and GPIO25 are taken from the USB
 * Serial/JTAG PHY, which has them from reset, before their function is
 * selected.
 */
static void pad_out(QTestState *qts, int pad, int sig, uint32_t extra)
{
    if (pad == 24 || pad == 25) {
        qtest_writel(qts, USJ_CONF0,
                     qtest_readl(qts, USJ_CONF0) & ~USJ_PAD_ENABLE);
    }
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_OUT(pad), sig | extra);
}

static void pad_in(QTestState *qts, int sig, int pad, uint32_t extra)
{
    qtest_writel(qts, IO_MUX_GPIO(pad), MCU_SEL_GPIO | FUN_IE);
    qtest_writel(qts, GPIO_FUNC_IN(sig), IN_SIG_IN_SEL | pad | extra);
}

static bool pad(QTestState *qts, int n)
{
    return n < 32 ? qtest_readl(qts, GPIO_IN) >> n & 1 :
                    qtest_readl(qts, GPIO_IN1) >> (n - 32) & 1;
}

static bool source_level(QTestState *qts, int src)
{
    return (qtest_readl(qts, INTMTX_STATUS(src / 32)) >> (src % 32)) & 1;
}

static int64_t now(QTestState *qts)
{
    return vnow;
}

static void step(QTestState *qts, int64_t ns)
{
    vnow = qtest_clock_step(qts, ns);
}

/* Step to absolute virtual time @t. */
static void step_to(QTestState *qts, int64_t t)
{
    g_assert_cmpint(t, >=, vnow);
    if (t > vnow) {
        vnow = qtest_clock_set(qts, t);
    }
}

/* The first time after now, within @limit ns, pad @n reads @level. */
static int64_t find_edge(QTestState *qts, int n, bool level, int64_t limit)
{
    int64_t end = now(qts) + limit;

    while (now(qts) < end) {
        step(qts, 1);
        if (pad(qts, n) == level) {
            return now(qts);
        }
    }
    g_assert_not_reached();
}

static void desc(QTestState *qts, uint32_t addr, uint32_t dw0, uint32_t buf,
                 uint32_t next)
{
    qtest_writel(qts, addr, dw0);
    qtest_writel(qts, addr + 4, buf);
    qtest_writel(qts, addr + 8, next);
}

/* Start GDMA-AXI TX channel 0 for LCD_CAM on its list at TX_DESC. */
static void dma_tx_start(QTestState *qts)
{
    qtest_writel(qts, AXI_OUT + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1u << 3);
    qtest_writel(qts, AXI_OUT + X_PERI_SEL, PERI_LCD_CAM);
    qtest_writel(qts, AXI_OUT + X_LINK2, TX_DESC);
    qtest_writel(qts, AXI_OUT + X_LINK1, OUT_LINK_START);
}

/* @len bytes of @b at SRC, one descriptor, its EOF set. */
static void dma_tx(QTestState *qts, const uint8_t *b, uint32_t len)
{
    qtest_memwrite(qts, SRC, b, len);
    desc(qts, TX_DESC, OWNER | SUC_EOF | DW0(len, len), SRC, 0);
    dma_tx_start(qts);
}

/* Start GDMA-AXI RX channel 0 for LCD_CAM into @n buffers of @size. */
static void dma_rx(QTestState *qts, unsigned n, uint32_t size)
{
    unsigned i;

    for (i = 0; i < n; i++) {
        desc(qts, RX_DESC + 16 * i, OWNER | DW0(size, 0), DST + i * size,
             i + 1 < n ? RX_DESC + 16 * (i + 1) : 0);
    }
    qtest_writel(qts, AXI_IN + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_IN + X_CONF0, 1);
    qtest_writel(qts, AXI_IN + X_CONF0, 0);
    qtest_writel(qts, AXI_IN + X_PERI_SEL, PERI_LCD_CAM);
    qtest_writel(qts, AXI_IN + X_LINK2, RX_DESC);
    qtest_writel(qts, AXI_IN + X_LINK1, IN_LINK_START);
}

/* The console of the panel with QOM id @id, as RGB888, via screendump. */
static uint8_t *screendump(QTestState *qts, const char *id, int *w, int *h)
{
    g_autofree char *path = NULL;
    g_autofree char *data = NULL;
    QDict *r;
    gsize len;
    int fd = g_file_open_tmp("esp32p4-lcd-cam-XXXXXX.ppm", &path, NULL);
    int maxval, off;

    g_assert(fd >= 0);
    close(fd);
    r = qtest_qmp(qts, "{ 'execute': 'screendump', 'arguments': "
                  "{ 'filename': %s, 'device': %s, 'format': 'ppm' } }",
                  path, id);
    g_assert(qdict_haskey(r, "return"));
    qobject_unref(r);
    g_assert(g_file_get_contents(path, &data, &len, NULL));
    unlink(path);
    g_assert(sscanf(data, "P6 %d %d %d%n", w, h, &maxval, &off) == 3);
    g_assert_cmpint(maxval, ==, 255);
    return g_memdup2(data + off + 1, (size_t)*w * *h * 3);
}

static void enable_lcd_outputs(QTestState *qts, int data_lines)
{
    int i;

    for (i = 0; i < data_lines; i++) {
        pad_out(qts, i, SIG_LCD_DATA(i), 0);
    }
    pad_out(qts, PAD_CS, SIG_LCD_CS, 0);
    pad_out(qts, PAD_DC, SIG_LCD_DC, 0);
    pad_out(qts, PAD_PCLK, SIG_LCD_PCLK, 0);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_reset_values(void)
{
    QTestState *qts = start();

    g_assert_cmphex(qtest_readl(qts, LCD_CLOCK), ==, 0x00000843);
    g_assert_cmphex(qtest_readl(qts, CAM_CTRL), ==, 0x00000800);
    g_assert_cmphex(qtest_readl(qts, CAM_CTRL1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CAM_RGB_YUV), ==, 0x00c00000);
    g_assert_cmphex(qtest_readl(qts, LCD_RGB_YUV), ==, 0x00cc0000);
    g_assert_cmphex(qtest_readl(qts, LCD_USER), ==, 0x00000001);
    g_assert_cmphex(qtest_readl(qts, LCD_MISC), ==, 0x000000c0);
    g_assert_cmphex(qtest_readl(qts, LCD_CTRL), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_CTRL1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_CTRL2), ==, 0x00010001);
    g_assert_cmphex(qtest_readl(qts, LCD_FIRST_CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_LATTER_CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_DLY_MODE_CFG1), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_DLY_MODE_CFG2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_ST), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LC_DATE), ==, 0x2502226);

    /* Writable bits only; the WT and self-clearing bits read 0. */
    qtest_writel(qts, LCD_CLOCK, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LCD_CLOCK), ==, 0xffffffff);
    qtest_writel(qts, CAM_CTRL, 0xffffffef);
    g_assert_cmphex(qtest_readl(qts, CAM_CTRL), ==, 0x7fffffef);
    qtest_writel(qts, CAM_CTRL1, 0x1fffffff);
    g_assert_cmphex(qtest_readl(qts, CAM_CTRL1), ==, 0x1fffffff);
    qtest_writel(qts, CAM_RGB_YUV, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CAM_RGB_YUV), ==, 0xffe00000);
    qtest_writel(qts, LCD_RGB_YUV, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LCD_RGB_YUV), ==, 0xfffc0000);
    qtest_writel(qts, LCD_MISC, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LCD_MISC), ==, 0xf7fffff0);
    qtest_writel(qts, LCD_CTRL2, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LCD_CTRL2), ==, 0xffff03ff);
    qtest_writel(qts, LCD_DLY_MODE_CFG1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LCD_DLY_MODE_CFG1), ==, 0x00ffffff);
    qtest_writel(qts, LCD_DLY_MODE_CFG1, 0);
    qtest_writel(qts, LCD_DLY_MODE_CFG2, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LCD_DLY_MODE_CFG2), ==, 0xffffffff);
    qtest_writel(qts, INT_ENA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0x1f);
    qtest_writel(qts, LC_DATE, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, LC_DATE), ==, 0x0fffffff);
    /* LCD_RESET is write-only; LCD_UPDATE clears once LCD_CLK takes it. */
    qtest_writel(qts, LCD_USER, LCD_RESET | LCD_UPDATE | DOUT_CYCLELEN(5));
    g_assert_cmphex(qtest_readl(qts, LCD_USER), ==,
                    LCD_UPDATE | DOUT_CYCLELEN(5));
    lcd_clock(qts, 1, true);
    g_assert_cmphex(qtest_readl(qts, LCD_USER), ==, DOUT_CYCLELEN(5));
    /* INT_RAW and INT_ST are read-only, INT_CLR write-only. */
    qtest_writel(qts, INT_RAW, 0x1f);
    qtest_writel(qts, INT_ST, 0x1f);
    qtest_writel(qts, INT_CLR, 0x1f);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, INT_CLR), ==, 0);

    /* RST_EN_LCDCAM resets the block. */
    qtest_writel(qts, HP_RST_EN2, RST_EN_LCDCAM);
    qtest_writel(qts, HP_RST_EN2, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_CLOCK), ==, 0x00000843);
    g_assert_cmphex(qtest_readl(qts, LCD_MISC), ==, 0x000000c0);
    g_assert_cmphex(qtest_readl(qts, INT_ENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LC_DATE), ==, 0x2502226);
    qtest_quit(qts);
}

/*
 * The LCD's idle outputs follow LCD_UPDATE's copy: a new idle level of DC
 * or HSYNC reaches its pad only with LCD_UPDATE, and only while LCD_CLK
 * runs.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_update(void)
{
    QTestState *qts = start();

    pad_out(qts, PAD_DC, SIG_LCD_DC, 0);
    pad_out(qts, PAD_CS, SIG_LCD_CS, 0);
    pad_out(qts, PAD_HSYNC, SIG_LCD_HSYNC, 0);
    g_assert_false(pad(qts, PAD_DC));
    g_assert_true(pad(qts, PAD_CS));
    g_assert_false(pad(qts, PAD_HSYNC));

    qtest_writel(qts, LCD_MISC, 0x000000c0 | CD_IDLE_EDGE);
    qtest_writel(qts, LCD_CTRL2, 0x00010001 | HSYNC_IDLE_POL);
    g_assert_false(pad(qts, PAD_DC));
    g_assert_false(pad(qts, PAD_HSYNC));
    /* LCD_CLK stopped: the copy waits for it. */
    qtest_writel(qts, LCD_USER, LCD_UPDATE | 1);
    g_assert_false(pad(qts, PAD_DC));
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_UPDATE, ==, LCD_UPDATE);
    lcd_clock(qts, 1, true);
    g_assert_true(pad(qts, PAD_DC));
    g_assert_true(pad(qts, PAD_HSYNC));
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_UPDATE, ==, 0);
    /* The matrix's inversion reaches the pad as well. */
    pad_out(qts, PAD_DC, SIG_LCD_DC, OUT_INV);
    g_assert_false(pad(qts, PAD_DC));
    qtest_quit(qts);
}

/*
 * An i80 transaction, LCD_CLK 40 MHz and PCLK a quarter of it (100 ns):
 * two setup cycles, a two-cycle command, a dummy cycle, three data cycles
 * and a hold cycle, each phase's DC level on its pad at its exact edges,
 * CS across the whole, LCD_TRANS_DONE_INT as CS rises, through the
 * interrupt matrix; the data lines hold the last word after it.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_i80_timing(void)
{
    QTestState *qts = start();
    uint8_t b[4] = { 0x11, 0x22, 0x33, 0x44 };
    int64_t t0;
    int i;

    enable_lcd_outputs(qts, 8);
    lcd_clock(qts, 1, true);
    qtest_writel(qts, LCD_CLOCK, CLKCNT_N(3));
    qtest_writel(qts, LCD_MISC, BK_EN | VFK_CYCLELEN(1) | VBK_CYCLELEN(0) |
                 CD_DATA_SET | CD_DUMMY_SET);
    qtest_writel(qts, LCD_FIRST_CMD, 0x2c);
    qtest_writel(qts, LCD_LATTER_CMD, 0x3c);
    qtest_writel(qts, INT_ENA, INT_LCD_TRANS_DONE);
    qtest_writel(qts, LCD_USER, CMD | CMD_2_CYCLE_EN | DUMMY |
                 DUMMY_CYCLELEN(0) | DOUT | DOUT_CYCLELEN(2) | LCD_UPDATE);
    qtest_writel(qts, LCD_MISC, qtest_readl(qts, LCD_MISC) | LCD_AFIFO_RESET);
    dma_tx(qts, b, 4);
    step(qts, 1000);
    g_assert_true(pad(qts, PAD_CS));
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_START);

    /* CS falls at the next LCD_CLK edge, within 25 ns. */
    t0 = find_edge(qts, PAD_CS, false, 25);
    g_assert_false(pad(qts, PAD_DC));
    /* Setup: 2 cycles; CMD: 2 cycles, DC low (CD_CMD_SET clear). */
    step_to(qts, t0 + 399);
    g_assert_false(pad(qts, PAD_DC));
    /* DUMMY at 400 ns: DC high; DOUT at 500 ns: high. */
    step_to(qts, t0 + 400);
    g_assert_true(pad(qts, PAD_DC));
    step_to(qts, t0 + 799);
    g_assert_true(pad(qts, PAD_DC));
    g_assert_false(pad(qts, PAD_CS));
    g_assert_false(source_level(qts, SRC_LCD_CAM));
    /* Hold from 800 ns: DC at its idle level; CS rises at 900 ns. */
    step_to(qts, t0 + 800);
    g_assert_false(pad(qts, PAD_DC));
    step_to(qts, t0 + 899);
    g_assert_false(pad(qts, PAD_CS));
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_START, ==, LCD_START);
    step_to(qts, t0 + 900);
    g_assert_true(pad(qts, PAD_CS));
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_LCD_TRANS_DONE);
    g_assert_cmphex(qtest_readl(qts, INT_ST), ==, INT_LCD_TRANS_DONE);
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_START, ==, 0);
    g_assert_true(source_level(qts, SRC_LCD_CAM));
    /* The data lines hold the last word sent. */
    for (i = 0; i < 8; i++) {
        g_assert_cmpint(pad(qts, i), ==, 0x33 >> i & 1);
    }
    qtest_writel(qts, INT_CLR, INT_LCD_TRANS_DONE);
    g_assert_false(source_level(qts, SRC_LCD_CAM));
    qtest_quit(qts);
}

/* Step until INT_RAW has one of @bits, at most @max_ns; false if not. */
static bool wait_int(QTestState *qts, uint32_t bits, int64_t max_ns)
{
    int64_t end = now(qts) + max_ns;

    while (!(qtest_readl(qts, INT_RAW) & bits)) {
        if (now(qts) >= end) {
            return false;
        }
        step(qts, 100);
    }
    return true;
}

/*
 * An i80 transaction as ESP-IDF's esp_lcd_panel_io_i80 makes one
 * (lcd_start_transaction()): an 8-bit command, then @len bytes of @data
 * through GDMA with LCD_ALWAYS_OUT_EN (or, with none, the 4-byte "fake
 * trigger" and no DOUT), one setup and one hold cycle, waiting for
 * LCD_TRANS_DONE_INT.
 */
static void i80_tx(QTestState *qts, int cmd, const uint8_t *data, uint32_t len,
                   bool swap)
{
    static const uint8_t fake[4];
    uint32_t user = qtest_readl(qts, LCD_USER) &
                    ~(CMD | DUMMY | DOUT | CMD_2_CYCLE_EN | SWIZZLE_EN);

    qtest_writel(qts, INT_CLR, 0x1f);
    if (swap) {
        user |= SWIZZLE_EN | SWIZZLE_MODE(0);
    }
    if (cmd >= 0) {
        qtest_writel(qts, LCD_FIRST_CMD, cmd);
        user |= CMD;
    }
    if (data) {
        user |= DOUT;
    }
    qtest_writel(qts, LCD_USER, user);
    qtest_writel(qts, LCD_MISC, (qtest_readl(qts, LCD_MISC) &
                                 ~(VFK_CYCLELEN(0x3f) | VBK_CYCLELEN(0x1fff)))
                 | BK_EN);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_RESET);
    qtest_writel(qts, LCD_MISC, qtest_readl(qts, LCD_MISC) | LCD_AFIFO_RESET);
    dma_tx(qts, data ? data : fake, data ? len : 4);
    step(qts, 4000);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_UPDATE);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_START);
    g_assert_true(wait_int(qts, INT_LCD_TRANS_DONE, 10000000));
}

static void i80_param(QTestState *qts, int cmd, const uint8_t *p, uint32_t n)
{
    i80_tx(qts, cmd, n ? p : NULL, n, false);
}

/* RGB565 as the ST7789 shows it: 6 bits a colour in GRAM, then 8. */
static void st7789_rgb(uint16_t v, uint8_t rgb[3])
{
    unsigned r = v >> 11, g = v >> 5 & 0x3f, b = v & 0x1f;

    r = r << 1 | r >> 4;
    b = b << 1 | b >> 4;
    rgb[0] = r << 2 | r >> 4;
    rgb[1] = g << 2 | g >> 4;
    rgb[2] = b << 2 | b >> 4;
}

/*
 * An ST7789 on an 8-bit i80 bus, set up and drawn on as ESP-IDF's
 * esp_lcd_panel_st7789 does it through esp_lcd_panel_io_i80 (SWRESET,
 * SLPOUT, MADCTL, COLMOD 16 bits, RAMCTRL big-endian, CASET, RASET,
 * RAMWR, DISPON), a little-endian RGB565 bitmap sent with the swizzle
 * swapping each byte pair: the console shows the bitmap, at the window
 * CASET and RASET gave, and black where nothing was drawn.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_i80_panel(void)
{
    QTestState *qts = start_with("-device st7789,bus=lcd,id=lcd0,width=8,"
                                 "height=6");
    uint8_t madctl = 0, colmod = 0x55, ramctrl[2] = { 0x00, 0xf0 };
    uint8_t caset[4] = { 0, 2, 0, 5 }, raset[4] = { 0, 1, 0, 3 };
    uint8_t bitmap[4 * 3 * 2], *img, rgb[3];
    int w, h, x, y;

    for (x = 0; x < 12; x++) {
        uint16_t v = 0x1234 * (x + 1) ^ 0x5a5a;

        bitmap[2 * x] = v;
        bitmap[2 * x + 1] = v >> 8;
    }
    enable_lcd_outputs(qts, 8);
    lcd_clock(qts, 2, true);
    qtest_writel(qts, LCD_CLOCK, CLKCNT_N(1));
    qtest_writel(qts, LCD_MISC, WIRE_MODE(0) | CD_DATA_SET);
    qtest_writel(qts, LCD_USER, LCD_BYTE_MODE(0) | ALWAYS_OUT_EN);
    qtest_writel(qts, INT_ENA, INT_LCD_TRANS_DONE);

    i80_param(qts, 0x01, NULL, 0);
    i80_param(qts, 0x11, NULL, 0);
    i80_param(qts, 0x36, &madctl, 1);
    i80_param(qts, 0x3a, &colmod, 1);
    i80_param(qts, 0xb0, ramctrl, 2);
    i80_param(qts, 0x2a, caset, 4);
    i80_param(qts, 0x2b, raset, 4);
    i80_tx(qts, 0x2c, bitmap, sizeof(bitmap), true);

    /* Display off: black. */
    img = screendump(qts, "lcd0", &w, &h);
    g_assert_cmpint(w, ==, 8);
    g_assert_cmpint(h, ==, 6);
    for (x = 0; x < w * h * 3; x++) {
        g_assert_cmpint(img[x], ==, 0);
    }
    g_free(img);

    i80_param(qts, 0x29, NULL, 0);
    img = screendump(qts, "lcd0", &w, &h);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            const uint8_t *p = img + (y * w + x) * 3;

            if (x >= 2 && x <= 5 && y >= 1 && y <= 3) {
                int i = (y - 1) * 4 + (x - 2);

                st7789_rgb(bitmap[2 * i] | bitmap[2 * i + 1] << 8, rgb);
            } else {
                rgb[0] = rgb[1] = rgb[2] = 0;
            }
            g_assert_cmphex(p[0], ==, rgb[0]);
            g_assert_cmphex(p[1], ==, rgb[1]);
            g_assert_cmphex(p[2], ==, rgb[2]);
        }
    }
    g_free(img);
    qtest_quit(qts);
}

/*
 * A DOUT phase with LCD_ALWAYS_OUT_EN runs to the byte that ends GDMA's
 * frame, over two descriptors: 6 data cycles, then the hold cycle; GDMA's
 * OUT_EOF is raised. A CS routed with the matrix's inversion (ESP-IDF's
 * cs_active_high) reads inverted on its pad.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_i80_always_out(void)
{
    QTestState *qts = start();
    uint8_t b[6] = { 1, 2, 3, 4, 5, 6 };
    int64_t t0;

    enable_lcd_outputs(qts, 8);
    pad_out(qts, PAD_CS, SIG_LCD_CS, OUT_INV);
    g_assert_false(pad(qts, PAD_CS));
    lcd_clock(qts, 1, true);
    qtest_writel(qts, LCD_CLOCK, CLK_EQU_SYSCLK);
    qtest_writel(qts, LCD_MISC, BK_EN | CD_DATA_SET);
    qtest_writel(qts, LCD_USER, DOUT | ALWAYS_OUT_EN | LCD_UPDATE);
    qtest_memwrite(qts, SRC, b, 6);
    desc(qts, TX_DESC, OWNER | DW0(4, 4), SRC, TX_DESC + 16);
    desc(qts, TX_DESC + 16, OWNER | SUC_EOF | DW0(2, 2), SRC + 4, 0);
    dma_tx_start(qts);
    step(qts, 1000);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_START);
    t0 = find_edge(qts, PAD_CS, true, 25);
    /* Setup (1 cycle at 25 ns), DOUT 6 cycles, hold 1: CS falls at 200. */
    step_to(qts, t0 + 24);
    g_assert_false(pad(qts, PAD_DC));
    step_to(qts, t0 + 25);
    g_assert_true(pad(qts, PAD_DC));
    step_to(qts, t0 + 174);
    g_assert_true(pad(qts, PAD_DC));
    step_to(qts, t0 + 175);
    g_assert_false(pad(qts, PAD_DC));
    g_assert_true(pad(qts, PAD_CS));
    step_to(qts, t0 + 200);
    g_assert_false(pad(qts, PAD_CS));
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_LCD_TRANS_DONE);
    g_assert_cmphex(qtest_readl(qts, AXI_OUT + X_RAW) & OUT_EOF, ==, OUT_EOF);
    qtest_quit(qts);
}

/*
 * A DOUT phase waits for GDMA: started with no data, the LCD waits; with
 * LCD_CLK stopped, LCD_START waits for the clock. A fixed-length DOUT
 * that GDMA cannot fill raises LCD_UNDERRUN_INT and still ends on time.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_i80_waits(void)
{
    QTestState *qts = start();
    uint8_t b[2] = { 0xaa, 0x55 };
    int64_t t0;

    enable_lcd_outputs(qts, 8);
    qtest_writel(qts, LCD_CLOCK, CLK_EQU_SYSCLK);
    qtest_writel(qts, LCD_USER, DOUT | DOUT_CYCLELEN(3) | LCD_UPDATE);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_START);
    step(qts, 2000);
    g_assert_true(pad(qts, PAD_CS));
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_UPDATE, ==, LCD_UPDATE);
    /* The clock: LCD_UPDATE taken, the transaction still waits for data. */
    lcd_clock(qts, 1, true);
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_UPDATE, ==, 0);
    step(qts, 2000);
    g_assert_true(pad(qts, PAD_CS));
    /* Two bytes for four cycles: the last two underrun. */
    dma_tx(qts, b, 2);
    t0 = find_edge(qts, PAD_CS, false, 1000);
    step_to(qts, t0 + 99);
    g_assert_false(pad(qts, PAD_CS));
    step_to(qts, t0 + 100);
    g_assert_true(pad(qts, PAD_CS));
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==,
                    INT_LCD_TRANS_DONE | INT_LCD_UNDERRUN);
    qtest_quit(qts);
}

/* RGB565 as the RGB panel's console shows it. */
static void rgb565_rgb(uint16_t v, uint8_t rgb[3])
{
    unsigned r = v >> 11, g = v >> 5 & 0x3f, b = v & 0x1f;

    rgb[0] = r << 3 | r >> 2;
    rgb[1] = g << 2 | g >> 4;
    rgb[2] = b << 3 | b >> 2;
}

static uint32_t panel_frames(QTestState *qts, const char *path)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': 'frames' } }", path);
    uint32_t n = qdict_get_int(r, "return");

    qobject_unref(r);
    return n;
}

/*
 * An RGB frame as ESP-IDF's esp_lcd_panel_rgb sets one up (stream mode,
 * HSYNC in the porches, HSYNC at the line's start), at 16 bits, LCD_CLK
 * 40 MHz / 2 (50 ns): lines of 2 + 3 + 8 + 2 cycles, frames of 1 + 2 + 4
 * + 1 lines. VSYNC, HSYNC and DE on their pads at every edge, to the
 * cycle; LCD_VSYNC_INT as the last data cycle ends; the next frame at
 * 6000 ns; the panel's console shows the frame buffer once a frame has
 * gone out.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_rgb_timing(void)
{
    QTestState *qts = start_with("-device rgb-panel,bus=lcd,id=rgb0,width=8,"
                                 "height=4");
    uint8_t fb[8 * 4 * 2], *img, rgb[3];
    int64_t t0, line;
    int i, l, w, h, x, y;

    for (i = 0; i < 32; i++) {
        uint16_t v = (i * 0x0841 + 0x1f) ^ (i << 11);

        fb[2 * i] = v;
        fb[2 * i + 1] = v >> 8;
    }
    for (i = 0; i < 16; i++) {
        pad_out(qts, i, SIG_LCD_DATA(i), 0);
    }
    pad_out(qts, PAD_PCLK, SIG_LCD_PCLK, 0);
    pad_out(qts, PAD_HSYNC, SIG_LCD_HSYNC, 0);
    pad_out(qts, PAD_VSYNC, SIG_LCD_VSYNC, 0);
    pad_out(qts, PAD_DE, SIG_LCD_DE, 0);
    lcd_clock(qts, 1, true);
    qtest_writel(qts, LCD_CLOCK, CLKCNT_N(1));
    qtest_writel(qts, LCD_MISC, WIRE_MODE(1) | BK_EN | NEXT_FRAME_EN);
    qtest_writel(qts, LCD_CTRL, RGB_MODE_EN | HB_FRONT(4) | VA_HEIGHT(3) |
                 VT_HEIGHT(7));
    qtest_writel(qts, LCD_CTRL1, VB_FRONT(2) | HA_WIDTH(7) | HT_WIDTH(14));
    qtest_writel(qts, LCD_CTRL2, VSYNC_WIDTH(0) | VSYNC_IDLE_POL |
                 HSYNC_WIDTH(1) | HSYNC_IDLE_POL | HS_BLANK_EN |
                 HSYNC_POSITION(0));
    qtest_writel(qts, INT_ENA, INT_LCD_VSYNC | INT_LCD_UNDERRUN);
    qtest_writel(qts, LCD_USER, LCD_BYTE_MODE(1) | DOUT | ALWAYS_OUT_EN |
                 LCD_UPDATE);
    g_assert_true(pad(qts, PAD_VSYNC));
    g_assert_true(pad(qts, PAD_HSYNC));
    g_assert_false(pad(qts, PAD_DE));
    /* A circular list over the frame buffer, its EOF at its end. */
    qtest_memwrite(qts, SRC, fb, sizeof(fb));
    desc(qts, TX_DESC, OWNER | SUC_EOF | DW0(64, 64), SRC, TX_DESC);
    qtest_writel(qts, AXI_OUT + X_CLR, 0x7ff);
    qtest_writel(qts, AXI_OUT + X_CONF0, 1);
    qtest_writel(qts, AXI_OUT + X_CONF0, 0);
    qtest_writel(qts, AXI_OUT + X_PERI_SEL, PERI_LCD_CAM);
    qtest_writel(qts, AXI_OUT + X_LINK2, TX_DESC);
    qtest_writel(qts, AXI_OUT + X_LINK1, OUT_LINK_START);
    step(qts, 1000);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_START);

    t0 = find_edge(qts, PAD_VSYNC, false, 25);
    g_assert_false(pad(qts, PAD_HSYNC));
    for (l = 0; l < 8; l++) {
        line = t0 + l * 750;
        /* HSYNC: cycles 0 and 1 of every line. */
        step_to(qts, line + 99);
        g_assert_false(pad(qts, PAD_HSYNC));
        g_assert_cmpint(pad(qts, PAD_VSYNC), ==, l != 0);
        step_to(qts, line + 100);
        g_assert_true(pad(qts, PAD_HSYNC));
        /* DE: cycles 5 to 12 of lines 3 to 6. */
        step_to(qts, line + 249);
        g_assert_false(pad(qts, PAD_DE));
        step_to(qts, line + 250);
        g_assert_cmpint(pad(qts, PAD_DE), ==, l >= 3 && l <= 6);
        step_to(qts, line + 649);
        g_assert_cmpint(pad(qts, PAD_DE), ==, l >= 3 && l <= 6);
        if (l == 6) {
            g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
        }
        step_to(qts, line + 650);
        g_assert_false(pad(qts, PAD_DE));
        if (l == 6) {
            g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_LCD_VSYNC);
            g_assert_true(source_level(qts, SRC_LCD_CAM));
        }
    }
    /* The next frame. */
    step_to(qts, t0 + 5999);
    g_assert_true(pad(qts, PAD_VSYNC));
    step_to(qts, t0 + 6000);
    g_assert_false(pad(qts, PAD_VSYNC));
    g_assert_false(pad(qts, PAD_HSYNC));
    g_assert_cmpint(panel_frames(qts, "/machine/peripheral/rgb0"), ==, 1);

    img = screendump(qts, "rgb0", &w, &h);
    g_assert_cmpint(w, ==, 8);
    g_assert_cmpint(h, ==, 4);
    for (y = 0; y < 4; y++) {
        for (x = 0; x < 8; x++) {
            i = y * 8 + x;
            rgb565_rgb(fb[2 * i] | fb[2 * i + 1] << 8, rgb);
            g_assert_cmphex(img[3 * i], ==, rgb[0]);
            g_assert_cmphex(img[3 * i + 1], ==, rgb[1]);
            g_assert_cmphex(img[3 * i + 2], ==, rgb[2]);
        }
    }
    g_free(img);
    step_to(qts, t0 + 3 * 6000);
    g_assert_cmpint(panel_frames(qts, "/machine/peripheral/rgb0"), ==, 3);
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_LCD_UNDERRUN, ==, 0);
    /* LCD_START cleared: the frame stops, the outputs idle. */
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) & ~LCD_START);
    g_assert_true(pad(qts, PAD_VSYNC));
    g_assert_true(pad(qts, PAD_HSYNC));
    g_assert_false(pad(qts, PAD_DE));
    qtest_quit(qts);
}

/*
 * An RGB frame without GDMA's data: each data line underruns
 * (LCD_UNDERRUN_INT); without LCD_NEXT_FRAME_EN one frame goes out,
 * then LCD_START clears and LCD_TRANS_DONE_INT is raised.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_rgb_underrun(void)
{
    QTestState *qts = start();
    int64_t t0;

    pad_out(qts, PAD_VSYNC, SIG_LCD_VSYNC, 0);
    pad_out(qts, PAD_DE, SIG_LCD_DE, 0);
    lcd_clock(qts, 1, true);
    qtest_writel(qts, LCD_CLOCK, CLK_EQU_SYSCLK);
    qtest_writel(qts, LCD_MISC, WIRE_MODE(1) | BK_EN);
    qtest_writel(qts, LCD_CTRL, RGB_MODE_EN | HB_FRONT(1) | VA_HEIGHT(1) |
                 VT_HEIGHT(4));
    qtest_writel(qts, LCD_CTRL1, VB_FRONT(0) | HA_WIDTH(3) | HT_WIDTH(9));
    qtest_writel(qts, LCD_CTRL2, VSYNC_WIDTH(0) | VSYNC_IDLE_POL);
    qtest_writel(qts, LCD_USER, LCD_BYTE_MODE(1) | LCD_UPDATE);
    qtest_writel(qts, LCD_USER, qtest_readl(qts, LCD_USER) | LCD_START);
    t0 = find_edge(qts, PAD_VSYNC, false, 25);
    /* The first data cycle: line 1, cycle 2. */
    step_to(qts, t0 + 25 * (10 + 2) - 1);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, 0);
    step_to(qts, t0 + 25 * (10 + 6));
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==, INT_LCD_UNDERRUN);
    step_to(qts, t0 + 25 * 50 - 1);
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_START, ==, LCD_START);
    step_to(qts, t0 + 25 * 50);
    g_assert_cmphex(qtest_readl(qts, INT_RAW), ==,
                    INT_LCD_UNDERRUN | INT_LCD_VSYNC | INT_LCD_TRANS_DONE);
    g_assert_cmphex(qtest_readl(qts, LCD_USER) & LCD_START, ==, 0);
    step(qts, 2000);
    g_assert_true(pad(qts, PAD_VSYNC));
    qtest_quit(qts);
}

/* The sensor's pads: PCLK, VSYNC, HSYNC and DE, then D0 to D7. */
#define CAM_PAD_PCLK        41
#define CAM_PAD_VSYNC       42
#define CAM_PAD_HSYNC       43
#define CAM_PAD_DE          44
#define CAM_PAD_D0          45

#define CAM_SENSOR \
    "-object esp32p4-image-source,id=cam0,pclk-pad=41,vsync-pad=42," \
    "hsync-pad=43,de-pad=44,data-pads=45:46:47:48:49:50:51:52,"

/* The camera's inputs from the sensor's pads, VSYNC inverted (ESP-IDF). */
static void cam_route(QTestState *qts)
{
    int i;

    pad_in(qts, SIG_CAM_PCLK, CAM_PAD_PCLK, 0);
    pad_in(qts, SIG_CAM_VSYNC, CAM_PAD_VSYNC, IN_INV);
    pad_in(qts, SIG_CAM_HSYNC, CAM_PAD_HSYNC, 0);
    pad_in(qts, SIG_CAM_DE, CAM_PAD_DE, 0);
    for (i = 0; i < 8; i++) {
        pad_in(qts, SIG_CAM_DATA(i), CAM_PAD_D0 + i, 0);
    }
}

/* The colour bars' RGB565 at (@x, @y) of frame @f, 16 x 4, cell 2. */
static uint16_t bars565(unsigned f, unsigned x, unsigned y)
{
    static const uint16_t bar[8] = {
        0xffff, 0xffe0, 0x07ff, 0x07e0, 0xf81f, 0xf800, 0x001f, 0x0000,
    };

    if (y < 2) {
        /* The frame counter's bits 15 to 8: 0 for the first frames. */
        return 0;
    }
    return bar[(x + f) % 16 * 8 / 16];
}

/*
 * A DVP capture as ESP-IDF's esp_cam_ctlr_dvp drives one (DE mode, 8
 * bits, the frame's end by VSYNC, the sensor's VSYNC inverted in the
 * matrix): a whole frame of the sensor's colour bars in one descriptor,
 * closed by IN_SUC_EOF at the frame's end, CAM_VSYNC_INT per frame and
 * CAM_HS_INT every two lines; the pixels as the sensor sent them.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_cam_capture(void)
{
    QTestState *qts = start_with(CAM_SENSOR "format=rgb565,width=16,"
                                 "height=4,hblank=16,vblank=3,cell=2,"
                                 "pixel-clock=1000000 "
                                 "-M dvp-camera=cam0");
    uint8_t frame[128];
    uint32_t dw0;
    unsigned d, x, y, f;
    bool found = false;

    cam_route(qts);
    cam_clock(qts, 2, true);
    qtest_writel(qts, CAM_CTRL, CAM_VS_EOF_EN | CAM_LINE_INT_EN | CAM_UPDATE);
    qtest_writel(qts, CAM_CTRL1, LINE_INT_NUM(1) | CAM_RESET |
                 CAM_AFIFO_RESET);
    qtest_writel(qts, CAM_CTRL, qtest_readl(qts, CAM_CTRL) | CAM_UPDATE);
    qtest_writel(qts, INT_ENA, INT_CAM_VSYNC | INT_CAM_HS);
    dma_rx(qts, 4, 128);
    qtest_writel(qts, CAM_CTRL1, qtest_readl(qts, CAM_CTRL1) | CAM_START);

    /* Frames of 32 x 7 pixel periods at 1 MHz: 224 us. */
    g_assert_true(wait_int(qts, INT_CAM_VSYNC, 300000));
    qtest_writel(qts, INT_CLR, INT_CAM_VSYNC | INT_CAM_HS);
    g_assert_true(wait_int(qts, INT_CAM_HS, 300000));
    g_assert_cmphex(qtest_readl(qts, INT_RAW) & INT_CAM_VSYNC, ==, 0);
    g_assert_true(wait_int(qts, INT_CAM_VSYNC, 300000));
    step(qts, 10000);
    g_assert_cmphex(qtest_readl(qts, AXI_IN + X_RAW) & IN_SUC_EOF, ==,
                    IN_SUC_EOF);

    /* A descriptor closed by the frame's end at 128 bytes: a frame. */
    for (d = 0; d < 4 && !found; d++) {
        dw0 = qtest_readl(qts, RX_DESC + 16 * d);
        if (!(dw0 & SUC_EOF) || (dw0 >> 12 & 0xfff) != 128) {
            continue;
        }
        qtest_memread(qts, DST + 128 * d, frame, 128);
        for (f = 0; f < 16 && !found; f++) {
            bool match = true;

            for (y = 0; y < 4; y++) {
                for (x = 0; x < 16; x++) {
                    uint16_t v = frame[(y * 16 + x) * 2] |
                                 frame[(y * 16 + x) * 2 + 1] << 8;

                    match &= v == bars565(f, x, y);
                }
            }
            found = match;
        }
    }
    g_assert_true(found);
    qtest_quit(qts);
}

/*
 * The sensor's XCLK from CAM_CLK (xclk=cam, PLL x2): its frames follow
 * CAM_CLK's rate as HP_SYS_CLKRST divides it (CAM_CLK_DIV_NUM the divisor
 * itself), and stop while CAM_CLK reaches no pad.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam/test] */
static void test_cam_xclk(void)
{
    QTestState *qts = start_with(CAM_SENSOR "format=raw8,width=16,height=4,"
                                 "hblank=16,vblank=3,xclk=cam,"
                                 "xclk-pad=26,pll-mul=2 "
                                 "-M dvp-camera=cam0");
    int64_t t1, t2;

    cam_route(qts);
    cam_clock(qts, 8, true);
    step(qts, 100000);
    /* CAM_CLK reaches no pad: no XCLK, no frames. */
    g_assert_false(pad(qts, CAM_PAD_VSYNC));
    /*
     * Routed: a frame starts at once, VSYNC active for its first line.
     * 5 MHz x 2: 10 MHz pixels, 32 x 7 of them a frame, 22.4 us.
     */
    pad_out(qts, PAD_CAM_CLK, SIG_CAM_CLK, 0);
    g_assert_true(pad(qts, CAM_PAD_VSYNC));
    t1 = now(qts);
    step_to(qts, t1 + 3199);
    g_assert_true(pad(qts, CAM_PAD_VSYNC));
    step_to(qts, t1 + 3200);
    g_assert_false(pad(qts, CAM_PAD_VSYNC));
    step_to(qts, t1 + 22399);
    g_assert_false(pad(qts, CAM_PAD_VSYNC));
    step_to(qts, t1 + 22400);
    g_assert_true(pad(qts, CAM_PAD_VSYNC));
    /* CAM_CLK at 10 MHz: a new frame from the change, of 11.2 us. */
    step(qts, 5000);
    cam_clock(qts, 4, true);
    g_assert_true(pad(qts, CAM_PAD_VSYNC));
    t2 = now(qts);
    step_to(qts, t2 + 1599);
    g_assert_true(pad(qts, CAM_PAD_VSYNC));
    step_to(qts, t2 + 1600);
    g_assert_false(pad(qts, CAM_PAD_VSYNC));
    step_to(qts, t2 + 11199);
    g_assert_false(pad(qts, CAM_PAD_VSYNC));
    step_to(qts, t2 + 11200);
    g_assert_true(pad(qts, CAM_PAD_VSYNC));
    /* Unrouted, the sensor stops at once. */
    pad_out(qts, PAD_CAM_CLK, 0x100, 0);
    step(qts, 50000);
    g_assert_false(pad(qts, CAM_PAD_VSYNC));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32p4/lcd-cam/reset-values", test_reset_values);
    qtest_add_func("/esp32p4/lcd-cam/update", test_update);
    qtest_add_func("/esp32p4/lcd-cam/i80-timing", test_i80_timing);
    qtest_add_func("/esp32p4/lcd-cam/i80-panel", test_i80_panel);
    qtest_add_func("/esp32p4/lcd-cam/i80-always-out", test_i80_always_out);
    qtest_add_func("/esp32p4/lcd-cam/i80-waits", test_i80_waits);
    qtest_add_func("/esp32p4/lcd-cam/rgb-timing", test_rgb_timing);
    qtest_add_func("/esp32p4/lcd-cam/rgb-underrun", test_rgb_underrun);
    qtest_add_func("/esp32p4/lcd-cam/cam-capture", test_cam_capture);
    qtest_add_func("/esp32p4/lcd-cam/cam-xclk", test_cam_xclk);
    return g_test_run();
}
