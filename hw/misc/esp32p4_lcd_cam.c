/*
 * ESP32-P4 LCD and camera controller (LCD_CAM)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32-P4 Technical Reference Manual (v3.x silicon), chapter
 * 40 (LCD and Camera Controller), with the layout, access types and reset
 * values of ESP-IDF's lcd_cam_reg.h for v3, its lcd_ll and cam_ll drivers
 * and the esp_lcd i80 and RGB drivers and esp_driver_cam's DVP driver
 * that run them. The wiring is in include/hw/misc/esp32p4_lcd_cam.h.
 *
 * Clocks. The LCD runs on LCD_CLK, HP_SYS_CLKRST's (TRM 40.3.3.1: its
 * source, gate and fractional divider are HP_SYS_CLKRST's on the P4),
 * its pixel clock PCLK being LCD_CLK / MO, MO = 1 with
 * LCD_CLK_EQU_SYSCLK, else LCD_CLKCNT_N + 1 (0 is not allowed: logged,
 * taken as 1). The camera module runs on CAM_CLK, HP_SYS_CLKRST's too,
 * which it also puts out as the sensor's master clock. LCD_CLOCK's
 * LCD_CLKM_DIV_* and LCD_CLK_SEL, and CAM_CTRL's CAM_CLKM_DIV_* and
 * CAM_CLK_SEL, are the ESP32-S3's dividers: the rev3 header keeps them,
 * R/W with their reset values (DIV_NUM 4), where the TRM's register
 * diagrams call bits 30:9 reserved and 0. The model keeps them as the
 * header has them and they divide nothing; a board check: read
 * LCD_CAM_LCD_CLOCK_REG after reset (0x843 by the header, 0x43 by the
 * TRM). Every time below is on LCD_CLK's exact edges, counted across its
 * changes (include/hw/misc/esp32p4_clk_period.h); while LCD_CLK is
 * stopped the LCD stands still, and while CAM_CLK is stopped the camera
 * takes nothing.
 *
 * Clock domains (TRM 40.5). The LCD's configuration registers (LCD_CLOCK,
 * LCD_RGB_YUV, LCD_USER, LCD_MISC, LCD_CTRL*, the command values and the
 * delay modes) reach the LCD clock domain only when software sets
 * LCD_UPDATE, which copies them over and clears itself, at once while
 * LCD_CLK runs and otherwise once it does; the camera's (CAM_CTRL,
 * CAM_CTRL1, CAM_RGB_YUV) likewise at CAM_UPDATE, on CAM_CLK. A
 * transaction or frame takes the copy as it starts, so a change takes
 * effect at the next one. START, RESET and AFIFO_RESET act at once.
 *
 * LCD data path (TRM 40.3.5): GDMA-AXI's bytes, in groups of
 * LCD_BYTE_MODE's 1, 2 or 3 bytes with each byte's bits reversed
 * (LCD_BIT_ORDER) and the group's bytes (LCD_BYTE_ORDER, 16 and 24 bits
 * only), through the YUV-RGB converter where LCD_CONV_ENABLE is set
 * (hw/misc/esp32p4_lcd_cam_conv.c), then reordered in groups of two bytes
 * (swizzle mode 0) or three (modes 1 to 5) where
 * LCD_DOUT_BYTE_SWIZZLE_ENABLE is set (TRM table 40.3-3), and sent
 * LCD_WIRE_MODE's 1, 2 or 3 bytes a PCLK, the first on D[7:0], each word
 * bit-reversed across its width with LCD_DOUT_BIT_ORDER. The settings the
 * TRM calls invalid (BYTE_MODE 3, BYTE_ORDER at 8 bits, swizzle modes 6
 * and 7, WIRE_MODE 3) are logged as guest errors and taken as 8 bits, no
 * reordering. The bytes the LCD has taken from GDMA and not yet sent
 * (its async TX FIFO, which LCD_AFIFO_RESET and LCD_RESET empty) are at
 * most a GDMA FIFO's worth beyond what it is sending: the model takes
 * them a GDMA FIFO (64 bytes) at a time as it needs them, so GDMA's
 * OUT_DONE and OUT_EOF come up to that many bytes' sending early.
 *
 * i80 (LCD_RGB_MODE_EN clear, TRM 40.3.7.2, 40.5.2). Setting LCD_START
 * starts a transaction once the LCD has data from GDMA (ESP-IDF: "the lcd
 * will only start working after the dma retrieves the data"), at the
 * next LCD_CLK edge: CS (LCD_CS, active low) goes active, then come the
 * phases, each a whole number of PCLK cycles: the setup time
 * (LCD_VFK_CYCLELEN + 1 cycles, with LCD_BK_EN), CMD (LCD_CMD: 1 cycle,
 * 2 with LCD_CMD_2_CYCLE_EN, sending LCD_FIRST_CMD_VALUE then
 * LCD_LATTER_CMD_VALUE as they are, their low bits on the wires), DUMMY
 * (LCD_DUMMY: LCD_DUMMY_CYCLELEN + 1 cycles of zeros), DOUT (LCD_DOUT:
 * LCD_DOUT_CYCLELEN + 1 cycles, or with LCD_ALWAYS_OUT_EN until the cycle
 * that sends the byte ending a GDMA frame, or until GDMA has nothing more
 * to send) and the hold time (LCD_VBK_CYCLELEN + 1 cycles, with
 * LCD_BK_EN). DC (LCD_DC) is LCD_CD_IDLE_EDGE outside the CMD, DUMMY and
 * DOUT phases and inverted in each whose LCD_CD_*_SET is set. Every cycle
 * of those three is a WR strobe (PCLK); setup and hold cycles strobe
 * nothing. As the last ends CS goes inactive, LCD_START clears and
 * LCD_TRANS_DONE_INT is raised. A DOUT cycle that finds the FIFO empty
 * while GDMA is still moving data waits for it, PCLK pausing; one that
 * finds it empty with GDMA idle ends DOUT (LCD_ALWAYS_OUT_EN) or goes out
 * holding the last word, raising LCD_UNDERRUN_INT.
 *
 * RGB (LCD_RGB_MODE_EN set, TRM 40.3.7.1, 40.5.1). Setting LCD_START
 * starts a frame at the next LCD_CLK edge. A frame is LCD_VT_HEIGHT + 1
 * lines of LCD_HT_WIDTH + 1 PCLK cycles; VSYNC is active for its first
 * LCD_VSYNC_WIDTH + 1 lines, from each line's start; lines LCD_VB_FRONT +
 * 1 to LCD_VB_FRONT + LCD_VA_HEIGHT + 1 carry data, DE active for
 * LCD_HA_WIDTH + 1 cycles from cycle LCD_HB_FRONT + 1 of the line, each
 * cycle a word as above; HSYNC is active for LCD_HSYNC_WIDTH + 1 cycles
 * from cycle LCD_HSYNC_POSITION of each data line, and of every line with
 * LCD_HS_BLANK_EN. (This is the TRM's figure 40.3-5 with ESP-IDF's
 * lcd_ll_set_horizontal_timing() and _vertical_timing(), which set
 * HB_FRONT to the HSYNC width and back porch less one.) Each output is at
 * its LCD_*_IDLE_POL when inactive and the other level when active. With
 * the frame's last data cycle sent LCD_VSYNC_INT is raised, leaving
 * software the vertical front porch (interpretation: the TRM says only
 * "transmits a whole frame"; ESP-IDF's driver expects the porch for
 * preparing the next frame; a board check: time the interrupt against
 * VSYNC on a scope). At the frame's end the next starts if
 * LCD_NEXT_FRAME_EN is set (with the configuration LCD_UPDATE last gave);
 * otherwise LCD_START clears and LCD_TRANS_DONE_INT is raised. A data
 * cycle that finds the FIFO empty goes out holding the last word and
 * raises LCD_UNDERRUN_INT; GDMA frame ends do not matter in RGB mode.
 * LCD_BK_EN clear in RGB mode (no blanking) is not modelled (logged; the
 * frame keeps its blanking); LCD_VFK_ and LCD_VBK_CYCLELEN are the i80
 * setup and hold times only.
 *
 * Delays: LCD_CD_MODE, LCD_DE_MODE, LCD_HSYNC_MODE and LCD_VSYNC_MODE
 * (LCD_DLY_MODE_CFG1) put DC's, DE's, HSYNC's and VSYNC's edges on their
 * pads one LCD_CLK cycle later (1, the next rising edge) or half a cycle
 * (2, the falling edge); mode 3 is taken as 0. The data lines' modes
 * change nothing the model carries.
 *
 * Camera (TRM 40.3.5.2, 40.5.3). The board's sensor (include/hw/misc/
 * esp32p4_image_source.h) gives a line of samples at a time. With
 * CAM_START set and CAM_CLK running the camera takes each sample of D[7:0]
 * (D[15:0] with CAM_2BYTE_EN, low byte first, or high first with
 * CAM_BYTE_ORDER), each byte's bits reversed with CAM_BIT_ORDER (TRM table
 * 40.3-4), through its converter where CAM_CONV_ENABLE is set, into its
 * async RX FIFO (8 x 16 bits), which GDMA-AXI takes from: bytes that find
 * it full are lost. (The model hands GDMA the bytes 32 at a time, at the
 * time of the last of them, which leaves GDMA's own 64-byte FIFO room
 * whenever its channel keeps up.) The samples are those
 * of each DE pulse: with CAM_VH_DE_MODE_EN clear, DE and VSYNC frame the
 * data; set, HSYNC must reach the camera too. A line is a DE pulse;
 * CAM_HS_INT is raised as every CAM_LINE_INT_NUM + 1 lines of a frame
 * end, with CAM_LINE_INT_EN (interpretation of the TRM's "receives lines
 * more than or equal to": counted afresh after each). A frame ends at the
 * falling edge of the camera's VSYNC as it sees it (the sensor's,
 * through the matrix and CAM_VSYNC_INV), later by CAM_VSYNC_FILTER_THRES
 * + 1 PCLK cycles with CAM_VSYNC_FILTER_EN: CAM_VSYNC_INT is raised, and
 * with CAM_VS_EOF_EN GDMA's frame ends there (IN_SUC_EOF); without it
 * every CAM_REC_DATA_BYTELEN + 1 bytes end one. This is ESP-IDF's
 * reading: its DVP driver inverts the sensor's VSYNC in the matrix, so
 * that a sensor's active-high VSYNC pulse is low here and its leading
 * edge ends the frame before. Interpretations: CAM_CLK's rate does not
 * limit the samples (the TRM asks it to be twice PCLK's, but ESP-IDF
 * leaves it at its reset divider with a sensor on its own crystal); the
 * converter works in DE mode too, its lines from DE (the TRM says it is
 * unavailable without HSYNC; ESP-IDF's driver uses it in DE mode). A
 * board check for both: capture with CAM_CLK at 625 kHz, and with
 * conversion in DE mode.
 *
 * Board wiring. What the panels' pins see (include/hw/display/
 * lcd_panel.h) comes from the matrix's routing: each panel signal is the
 * LCD output the matrix routes to a pad (the first, if several), with the
 * matrix's inversion; a panel takes no strobes without PCLK routed to a
 * pad, and none in i80 without DC; an unrouted data line reads 0. The
 * camera's inputs: each input takes the sensor signal on the pad the
 * matrix routes to it (esp32p4_gpio_sig_in_pad(),
 * esp32p4_image_source_dvp_pad()), or, for a sensor signal with no pad
 * given, its own signal wherever routed; an input on no sensor signal
 * reads its line's level. Without PCLK and DE from the sensor (and HSYNC
 * in VH mode) the camera takes nothing, and without VSYNC no frame ends;
 * a DE the camera sees inverted would frame the blanking, which the
 * sensor's API does not carry, and is logged.
 *
 * Interrupts: LCD_CAM_LC_DMA_INT_RAW's bits latch as above until
 * LC_DMA_INT_CLR clears them; LCD_CAM_INTR is high while any is set with
 * its LC_DMA_INT_ENA bit. There are no ETM events or tasks.
 *
 * Not modelled: LCD_CONV_TXTORX and CAM_STOP_EN (logged); the MOTO6800
 * format, which no register the P4 has selects; the camera's inputs edge
 * by edge from the pads (only a sensor's stream reaches it), so that its
 * VSYNC has the sensor's edges only, none from routing it anew or from
 * changing its inversion.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "exec/icount.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/registerfields.h"
#include "hw/gpio/esp32_line.h"
#include "hw/misc/esp32p4_lcd_cam.h"
#include "migration/vmstate.h"
#include "esp32p4_lcd_cam_conv.h"

#define PERIPH ESP32P4_GDMA_AXI_LCD_CAM

REG32(LCD_CLOCK, 0x00)
    FIELD(LCD_CLOCK, CLKCNT_N, 0, 6)
    FIELD(LCD_CLOCK, CLK_EQU_SYSCLK, 6, 1)
    FIELD(LCD_CLOCK, CK_IDLE_EDGE, 7, 1)
    FIELD(LCD_CLOCK, CK_OUT_EDGE, 8, 1)
REG32(CAM_CTRL, 0x04)
    FIELD(CAM_CTRL, STOP_EN, 0, 1)
    FIELD(CAM_CTRL, VSYNC_FILTER_THRES, 1, 3)
    FIELD(CAM_CTRL, UPDATE, 4, 1)
    FIELD(CAM_CTRL, BYTE_ORDER, 5, 1)
    FIELD(CAM_CTRL, BIT_ORDER, 6, 1)
    FIELD(CAM_CTRL, LINE_INT_EN, 7, 1)
    FIELD(CAM_CTRL, VS_EOF_EN, 8, 1)
REG32(CAM_CTRL1, 0x08)
    FIELD(CAM_CTRL1, REC_DATA_BYTELEN, 0, 16)
    FIELD(CAM_CTRL1, LINE_INT_NUM, 16, 6)
    FIELD(CAM_CTRL1, CLK_INV, 22, 1)
    FIELD(CAM_CTRL1, VSYNC_FILTER_EN, 23, 1)
    FIELD(CAM_CTRL1, TWO_BYTE_EN, 24, 1)
    FIELD(CAM_CTRL1, DE_INV, 25, 1)
    FIELD(CAM_CTRL1, HSYNC_INV, 26, 1)
    FIELD(CAM_CTRL1, VSYNC_INV, 27, 1)
    FIELD(CAM_CTRL1, VH_DE_MODE_EN, 28, 1)
    FIELD(CAM_CTRL1, START, 29, 1)
    FIELD(CAM_CTRL1, RESET, 30, 1)
    FIELD(CAM_CTRL1, AFIFO_RESET, 31, 1)
REG32(CAM_RGB_YUV, 0x0c)
REG32(LCD_RGB_YUV, 0x10)
    FIELD(LCD_RGB_YUV, TXTORX, 21, 1)
REG32(LCD_USER, 0x14)
    FIELD(LCD_USER, DOUT_CYCLELEN, 0, 13)
    FIELD(LCD_USER, ALWAYS_OUT_EN, 13, 1)
    FIELD(LCD_USER, SWIZZLE_MODE, 14, 3)
    FIELD(LCD_USER, SWIZZLE_EN, 17, 1)
    FIELD(LCD_USER, DOUT_BIT_ORDER, 18, 1)
    FIELD(LCD_USER, BYTE_MODE, 19, 2)
    FIELD(LCD_USER, UPDATE, 21, 1)
    FIELD(LCD_USER, BIT_ORDER, 22, 1)
    FIELD(LCD_USER, BYTE_ORDER, 23, 1)
    FIELD(LCD_USER, DOUT, 24, 1)
    FIELD(LCD_USER, DUMMY, 25, 1)
    FIELD(LCD_USER, CMD, 26, 1)
    FIELD(LCD_USER, START, 27, 1)
    FIELD(LCD_USER, RESET, 28, 1)
    FIELD(LCD_USER, DUMMY_CYCLELEN, 29, 2)
    FIELD(LCD_USER, CMD_2_CYCLE_EN, 31, 1)
REG32(LCD_MISC, 0x18)
    FIELD(LCD_MISC, WIRE_MODE, 4, 2)
    FIELD(LCD_MISC, VFK_CYCLELEN, 6, 6)
    FIELD(LCD_MISC, VBK_CYCLELEN, 12, 13)
    FIELD(LCD_MISC, NEXT_FRAME_EN, 25, 1)
    FIELD(LCD_MISC, BK_EN, 26, 1)
    FIELD(LCD_MISC, AFIFO_RESET, 27, 1)
    FIELD(LCD_MISC, CD_DATA_SET, 28, 1)
    FIELD(LCD_MISC, CD_DUMMY_SET, 29, 1)
    FIELD(LCD_MISC, CD_CMD_SET, 30, 1)
    FIELD(LCD_MISC, CD_IDLE_EDGE, 31, 1)
REG32(LCD_CTRL, 0x1c)
    FIELD(LCD_CTRL, HB_FRONT, 0, 11)
    FIELD(LCD_CTRL, VA_HEIGHT, 11, 10)
    FIELD(LCD_CTRL, VT_HEIGHT, 21, 10)
    FIELD(LCD_CTRL, RGB_MODE_EN, 31, 1)
REG32(LCD_CTRL1, 0x20)
    FIELD(LCD_CTRL1, VB_FRONT, 0, 8)
    FIELD(LCD_CTRL1, HA_WIDTH, 8, 12)
    FIELD(LCD_CTRL1, HT_WIDTH, 20, 12)
REG32(LCD_CTRL2, 0x24)
    FIELD(LCD_CTRL2, VSYNC_WIDTH, 0, 7)
    FIELD(LCD_CTRL2, VSYNC_IDLE_POL, 7, 1)
    FIELD(LCD_CTRL2, DE_IDLE_POL, 8, 1)
    FIELD(LCD_CTRL2, HS_BLANK_EN, 9, 1)
    FIELD(LCD_CTRL2, HSYNC_WIDTH, 16, 7)
    FIELD(LCD_CTRL2, HSYNC_IDLE_POL, 23, 1)
    FIELD(LCD_CTRL2, HSYNC_POSITION, 24, 8)
REG32(LCD_FIRST_CMD_VAL, 0x28)
REG32(LCD_LATTER_CMD_VAL, 0x2c)
REG32(LCD_DLY_MODE_CFG1, 0x30)
    FIELD(LCD_DLY_MODE_CFG1, CD_MODE, 16, 2)
    FIELD(LCD_DLY_MODE_CFG1, DE_MODE, 18, 2)
    FIELD(LCD_DLY_MODE_CFG1, HSYNC_MODE, 20, 2)
    FIELD(LCD_DLY_MODE_CFG1, VSYNC_MODE, 22, 2)
REG32(LCD_DLY_MODE_CFG2, 0x38)
REG32(LC_DMA_INT_ENA, 0x64)
REG32(LC_DMA_INT_RAW, 0x68)
REG32(LC_DMA_INT_ST, 0x6c)
REG32(LC_DMA_INT_CLR, 0x70)
REG32(LC_REG_DATE, 0xfc)

#define INT_LCD_VSYNC       BIT(0)
#define INT_LCD_TRANS_DONE  BIT(1)
#define INT_CAM_VSYNC       BIT(2)
#define INT_CAM_HS          BIT(3)
#define INT_LCD_UNDERRUN    BIT(4)
#define INT_MASK            0x1f

#define LCD_CLOCK_RESET     0x00000843
#define CAM_CTRL_RESET      0x00000800
#define CAM_RGB_YUV_RESET   0x00c00000
#define LCD_RGB_YUV_RESET   0x00cc0000
#define LCD_USER_RESET      0x00000001
#define LCD_MISC_RESET      0x000000c0
#define LCD_CTRL2_RESET     0x00010001
#define DATE_RESET          0x02502226

#define CAM_CTRL_MASK       0x7fffffff
#define CAM_RGB_YUV_MASK    0xffe00000
#define LCD_RGB_YUV_MASK    0xfffc0000
#define LCD_MISC_MASK       0xfffffff0
#define LCD_CTRL2_MASK      0xffff03ff
#define LCD_DLY1_MASK       0x00ffffff
#define DATE_MASK           0x0fffffff

/* The LCD clock domain's copies, in lcd_act[] and lcd_cfg[] */
enum {
    L_CLOCK,
    L_RGB_YUV,
    L_USER,
    L_MISC,
    L_CTRL,
    L_CTRL1,
    L_CTRL2,
    L_CMD0,
    L_CMD1,
    L_DLY1,
    L_DLY2,
};

/* The camera's, in cam_act[] */
enum {
    C_CTRL,
    C_CTRL1,
    C_RGB_YUV,
};

enum {
    LCD_IDLE,
    LCD_WAIT_CLOCK,
    LCD_WAIT_DATA,
    LCD_I80,
    LCD_RGB,
};

/* The i80 phases, in order; i80_phase[] has the first cycle of each. */
enum {
    PH_SETUP,
    PH_CMD,
    PH_DUMMY,
    PH_DOUT,
    PH_HOLD,
    PH_END,
};

/* The board mismatches logged once, in s->logged */
enum {
    LOG_NO_PCLK = 1 << 0,
    LOG_NO_DC = 1 << 1,
    LOG_CAM_PCLK = 1 << 2,
    LOG_CAM_DE = 1 << 3,
    LOG_CAM_HSYNC = 1 << 4,
    LOG_CAM_DE_INV = 1 << 5,
    LOG_CAM_STOP = 1 << 6,
    LOG_TXTORX = 1 << 7,
    LOG_CLKCNT = 1 << 8,
    LOG_FRAME_FIT = 1 << 9,
    LOG_RGB_BK = 1 << 10,
    LOG_LAG = 1 << 11,
};

static const uint8_t out_sig[ESP32P4_LCD_CAM_OUT_DATA0] = {
    [ESP32P4_LCD_CAM_OUT_CS] = ESP32P4_SIG_LCD_CS_PAD_OUT,
    [ESP32P4_LCD_CAM_OUT_DC] = ESP32P4_SIG_LCD_DC_PAD_OUT,
    [ESP32P4_LCD_CAM_OUT_PCLK] = ESP32P4_SIG_LCD_PCLK_PAD_OUT,
    [ESP32P4_LCD_CAM_OUT_HSYNC] = ESP32P4_SIG_LCD_H_SYNC_PAD_OUT,
    [ESP32P4_LCD_CAM_OUT_VSYNC] = ESP32P4_SIG_LCD_V_SYNC_PAD_OUT,
    [ESP32P4_LCD_CAM_OUT_DE] = ESP32P4_SIG_LCD_H_ENABLE_PAD_OUT,
    [ESP32P4_LCD_CAM_OUT_CAM_CLK] = ESP32P4_SIG_CAM_CLK_PAD_OUT,
};

static const uint8_t in_sig[ESP32P4_LCD_CAM_IN_DATA0] = {
    [ESP32P4_LCD_CAM_IN_PCLK] = ESP32P4_SIG_CAM_PCLK_PAD_IN,
    [ESP32P4_LCD_CAM_IN_VSYNC] = ESP32P4_SIG_CAM_V_SYNC_PAD_IN,
    [ESP32P4_LCD_CAM_IN_HSYNC] = ESP32P4_SIG_CAM_H_SYNC_PAD_IN,
    [ESP32P4_LCD_CAM_IN_DE] = ESP32P4_SIG_CAM_H_ENABLE_PAD_IN,
};

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
unsigned esp32p4_lcd_cam_out_signal(unsigned n)
{
    if (n >= ESP32P4_LCD_CAM_OUT_DATA0) {
        return ESP32P4_SIG_LCD_DATA_OUT_PAD_OUT0 + n -
               ESP32P4_LCD_CAM_OUT_DATA0;
    }
    return out_sig[n];
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
unsigned esp32p4_lcd_cam_in_signal(unsigned n)
{
    if (n >= ESP32P4_LCD_CAM_IN_DATA0) {
        return ESP32P4_SIG_CAM_DATA_IN_PAD_IN0 + n - ESP32P4_LCD_CAM_IN_DATA0;
    }
    return in_sig[n];
}

static void lcd_cam_update_irq(ESP32P4LcdCamState *s)
{
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

static void lcd_cam_raise(ESP32P4LcdCamState *s, uint32_t bits)
{
    s->int_raw |= bits;
    lcd_cam_update_irq(s);
}

static bool log_once(ESP32P4LcdCamState *s, uint32_t bit)
{
    bool first = !(s->logged & bit);

    s->logged |= bit;
    return first;
}

/*
 * Routing
 * =======
 */

static void lcd_cam_route(ESP32P4LcdCamState *s)
{
    unsigned i, n;

    if (s->route_valid) {
        return;
    }
    s->route_valid = true;
    for (i = 0; i < ESP32P4_LCD_CAM_OUTS; i++) {
        s->out_pad[i] = -1;
        s->out_inv[i] = false;
    }
    if (!s->gpio) {
        return;
    }
    for (n = 0; n < ESP32P4_GPIO_PADS; n++) {
        bool inv;
        int sig = esp32p4_gpio_pad_out_sig(s->gpio, n, &inv);

        for (i = 0; sig >= 0 && i < ESP32P4_LCD_CAM_OUTS; i++) {
            if (esp32p4_lcd_cam_out_signal(i) == sig && s->out_pad[i] < 0) {
                s->out_pad[i] = n;
                s->out_inv[i] = inv;
            }
        }
    }
    for (i = 0; i < ESP32P4_LCD_CAM_INS; i++) {
        unsigned sig = esp32p4_lcd_cam_in_signal(i);

        s->in_pad[i] = esp32p4_gpio_sig_in_pad(s->gpio, sig);
        s->in_inv[i] = esp32p4_gpio_sig_in_inverted(s->gpio, sig);
    }
}

static void lcd_cam_route_changed(Notifier *n, void *data)
{
    ESP32P4LcdCamState *s = container_of(n, ESP32P4LcdCamState,
                                         route_notifier);

    s->route_valid = false;
}

static bool out_routed(ESP32P4LcdCamState *s, unsigned out)
{
    lcd_cam_route(s);
    return s->out_pad[out] >= 0;
}

/*
 * LCD timing
 * ==========
 */

static uint32_t lcd_cfg_field(ESP32P4LcdCamState *s, unsigned reg,
                              unsigned shift, unsigned len)
{
    return extract32(s->lcd_cfg[reg], shift, len);
}

#define CFG(reg, field) \
    lcd_cfg_field(s, L_##reg, R_LCD_##reg##_##field##_SHIFT, \
                  R_LCD_##reg##_##field##_LENGTH)

static bool lcd_clk_running(ESP32P4LcdCamState *s)
{
    return clock_is_enabled(s->lcd_clk);
}

/* The LCD_CLK cycles of the controller's count completed by @t. */
static uint64_t lcd_lc_at(ESP32P4LcdCamState *s, int64_t t)
{
    return s->lcd_count_base + esp32p4_clk_count_at(&s->lcd_count, t);
}

/* When cycle @lc of the count completes: INT64_MAX while stopped. */
static int64_t lcd_lc_ns(ESP32P4LcdCamState *s, uint64_t lc)
{
    if (lc <= s->lcd_count_base) {
        return s->lcd_count.origin_ns;
    }
    return esp32p4_clk_count_time(&s->lcd_count, lc - s->lcd_count_base);
}

/* When PCLK cycle @p of the transaction or frame starts. */
static int64_t lcd_pos_ns(ESP32P4LcdCamState *s, uint64_t p)
{
    return lcd_lc_ns(s, s->lcd_pos0 + p * s->lcd_mo);
}

/* The first cycle position starting at or after @t. */
static uint64_t lcd_pos_at(ESP32P4LcdCamState *s, int64_t t)
{
    uint64_t lc = lcd_lc_at(s, t), p = 0;

    if (lc > s->lcd_pos0) {
        p = DIV_ROUND_UP(lc - s->lcd_pos0, s->lcd_mo);
    }
    while (lcd_pos_ns(s, p) < t) {
        p++;
    }
    return p;
}

/*
 * Output pads
 * ===========
 */

static void lcd_out_now(ESP32P4LcdCamState *s, unsigned out, bool level)
{
    if (s->out_level[out] != level) {
        s->out_level[out] = level;
        qemu_set_irq(s->sig_out[out], level);
    }
}

/* LCD_DLY_MODE_CFG1's delay of an output: 0, 1 cycle or half a cycle. */
static unsigned lcd_out_delay(ESP32P4LcdCamState *s, unsigned out)
{
    uint32_t dly = s->lcd_cfg[L_DLY1];

    switch (out) {
    case ESP32P4_LCD_CAM_OUT_DC:
        return FIELD_EX32(dly, LCD_DLY_MODE_CFG1, CD_MODE);
    case ESP32P4_LCD_CAM_OUT_DE:
        return FIELD_EX32(dly, LCD_DLY_MODE_CFG1, DE_MODE);
    case ESP32P4_LCD_CAM_OUT_HSYNC:
        return FIELD_EX32(dly, LCD_DLY_MODE_CFG1, HSYNC_MODE);
    case ESP32P4_LCD_CAM_OUT_VSYNC:
        return FIELD_EX32(dly, LCD_DLY_MODE_CFG1, VSYNC_MODE);
    default:
        return 0;
    }
}

/* Set @out at the present edge, esp32_line_time(), or later its delay. */
static void lcd_out(ESP32P4LcdCamState *s, unsigned out, bool level)
{
    unsigned mode = lcd_out_delay(s, out);
    int64_t now = esp32_line_time(), next;
    uint64_t lc;

    if (mode != 1 && mode != 2) {
        lcd_out_now(s, out, level);
        return;
    }
    lc = lcd_lc_at(s, now);
    next = lcd_lc_ns(s, lc + 1);
    if (next == INT64_MAX || s->nedges == ESP32P4_LCD_CAM_EDGES) {
        lcd_out_now(s, out, level);
        return;
    }
    s->edges[s->nedges++] = (ESP32P4LcdCamEdge) {
        .ns = mode == 1 ? next : now + DIV_ROUND_UP(next - now, 2),
        .out = out,
        .level = level,
    };
}

/* The outputs' idle levels, as LCD_UPDATE's copy gives them. */
static void lcd_outputs_idle(ESP32P4LcdCamState *s)
{
    uint32_t c2 = s->lcd_act[L_CTRL2];

    lcd_out_now(s, ESP32P4_LCD_CAM_OUT_CS, true);
    lcd_out_now(s, ESP32P4_LCD_CAM_OUT_DC,
                FIELD_EX32(s->lcd_act[L_MISC], LCD_MISC, CD_IDLE_EDGE));
    lcd_out_now(s, ESP32P4_LCD_CAM_OUT_PCLK,
                FIELD_EX32(s->lcd_act[L_CLOCK], LCD_CLOCK, CK_IDLE_EDGE));
    lcd_out_now(s, ESP32P4_LCD_CAM_OUT_HSYNC,
                FIELD_EX32(c2, LCD_CTRL2, HSYNC_IDLE_POL));
    lcd_out_now(s, ESP32P4_LCD_CAM_OUT_VSYNC,
                FIELD_EX32(c2, LCD_CTRL2, VSYNC_IDLE_POL));
    lcd_out_now(s, ESP32P4_LCD_CAM_OUT_DE,
                FIELD_EX32(c2, LCD_CTRL2, DE_IDLE_POL));
}

static void lcd_data_lines(ESP32P4LcdCamState *s, uint32_t word)
{
    unsigned i;

    for (i = 0; i < 24; i++) {
        bool b = word >> i & 1;

        if ((s->data_out >> i & 1) != b) {
            qemu_set_irq(s->sig_out[ESP32P4_LCD_CAM_OUT_DATA0 + i], b);
        }
    }
    s->data_out = word;
}

/*
 * The panels
 * ==========
 */

typedef struct PanelRun {
    ESP32P4LcdCamState *s;
    bool dc;
    const uint32_t *words;
    uint32_t count;
    int64_t first_ns;
    int64_t last_ns;
    bool de;
    int64_t de_end_ns;
} PanelRun;

/* @word as the panel's data pins see it, @width of them. */
static uint32_t panel_word(ESP32P4LcdCamState *s, uint32_t word,
                           unsigned width)
{
    uint32_t routed = 0, inv = 0;
    unsigned i;

    for (i = 0; i < width && i < 24; i++) {
        if (s->out_pad[ESP32P4_LCD_CAM_OUT_DATA0 + i] >= 0) {
            routed |= 1u << i;
            inv |= (uint32_t)s->out_inv[ESP32P4_LCD_CAM_OUT_DATA0 + i] << i;
        }
    }
    return (word & routed) ^ inv;
}

/* The level @out has on its pad, as a panel's pin wired there sees it. */
static bool panel_level(ESP32P4LcdCamState *s, unsigned out)
{
    return s->out_level[out] ^ s->out_inv[out];
}

static bool panel_selected(ESP32P4LcdCamState *s, LCDPanel *p)
{
    bool level;

    if (p->cs_pad >= 0) {
        return s->gpio && esp32p4_gpio_pad_driven(s->gpio, p->cs_pad,
                                                  &level) && !level;
    }
    if (!out_routed(s, ESP32P4_LCD_CAM_OUT_CS)) {
        return true;
    }
    return !panel_level(s, ESP32P4_LCD_CAM_OUT_CS);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void panel_dbi(LCDPanel *p, void *opaque)
{
    PanelRun *r = opaque;
    ESP32P4LcdCamState *s = r->s;
    LCDPanelClass *k = LCD_PANEL_GET_CLASS(p);
    uint32_t words[ESP32P4_LCD_CAM_RUN];
    unsigned width = k->bus_width(p), i;
    LCDDbiRun run = {
        .words = words,
        .count = r->count,
        .first_ns = r->first_ns,
        .last_ns = r->last_ns,
    };

    if (!k->dbi_write || !panel_selected(s, p)) {
        return;
    }
    run.dc = r->dc ^ s->out_inv[ESP32P4_LCD_CAM_OUT_DC];
    for (i = 0; i < r->count; i++) {
        words[i] = panel_word(s, r->words[i], width);
    }
    k->dbi_write(p, &run);
}

/* Give the panels a run of i80 strobes with DC at @dc. */
static void lcd_dbi_run(ESP32P4LcdCamState *s, bool dc,
                        const uint32_t *words, uint32_t count,
                        int64_t first_ns, int64_t last_ns)
{
    PanelRun r = {
        .s = s, .dc = dc, .words = words, .count = count,
        .first_ns = first_ns, .last_ns = last_ns,
    };

    if (!count) {
        return;
    }
    lcd_cam_route(s);
    if (!out_routed(s, ESP32P4_LCD_CAM_OUT_PCLK)) {
        if (log_once(s, LOG_NO_PCLK)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: LCD_PCLK "
                          "reaches no pad: the panels see no strobes\n");
        }
        return;
    }
    if (!out_routed(s, ESP32P4_LCD_CAM_OUT_DC)) {
        if (log_once(s, LOG_NO_DC)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: LCD_DC reaches "
                          "no pad: an i80 panel's D/CX is not driven\n");
        }
        return;
    }
    lcd_bus_foreach(s->bus, panel_dbi, &r);
}

static void panel_vsync(LCDPanel *p, void *opaque)
{
    PanelRun *r = opaque;
    LCDPanelClass *k = LCD_PANEL_GET_CLASS(p);

    if (k->dpi_vsync) {
        k->dpi_vsync(p, r->dc, r->first_ns);
    }
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void panel_line(LCDPanel *p, void *opaque)
{
    PanelRun *r = opaque;
    ESP32P4LcdCamState *s = r->s;
    LCDPanelClass *k = LCD_PANEL_GET_CLASS(p);
    unsigned width = k->bus_width(p), i;
    uint32_t *words;
    LCDDpiLine line = {
        .count = r->count,
        .de_level = r->de,
        .de_ns = r->first_ns,
        .de_end_ns = r->de_end_ns,
    };

    if (!k->dpi_line) {
        return;
    }
    words = g_new(uint32_t, r->count);
    for (i = 0; i < r->count; i++) {
        words[i] = panel_word(s, r->words[i], width);
    }
    line.words = words;
    k->dpi_line(p, &line);
    g_free(words);
}

/*
 * LCD data path
 * =============
 */

static void lcd_queue_word(ESP32P4LcdCamState *s, uint32_t w)
{
    if (s->q_num == ESP32P4_LCD_CAM_QUEUE) {
        return;
    }
    s->q[(s->q_head + s->q_num++) % ESP32P4_LCD_CAM_QUEUE] = w;
}

static uint32_t lcd_take_word(ESP32P4LcdCamState *s)
{
    uint32_t w = s->q[s->q_head];

    s->q_head = (s->q_head + 1) % ESP32P4_LCD_CAM_QUEUE;
    s->q_num--;
    if (s->q_eof) {
        s->q_eof--;
    }
    s->last_word = w;
    return w;
}

static unsigned lcd_wire_bytes(ESP32P4LcdCamState *s)
{
    unsigned mode = CFG(MISC, WIRE_MODE);

    return mode == 3 ? 1 : mode + 1;
}

static uint32_t rev_bits(uint32_t v, unsigned bits)
{
    return revbit32(v) >> (32 - bits);
}

/* Stage three: swizzle groups and wire words. */
static void lcd_wire_byte(ESP32P4LcdCamState *s, uint8_t b)
{
    unsigned w = lcd_wire_bytes(s);

    s->s3_wire[s->s3_wire_n++] = b;
    if (s->s3_wire_n == w) {
        uint32_t word = s->s3_wire[0] | (w > 1 ? s->s3_wire[1] << 8 : 0) |
                        (w > 2 ? s->s3_wire[2] << 16 : 0);

        if (CFG(USER, DOUT_BIT_ORDER)) {
            word = rev_bits(word, 8 * w);
        }
        lcd_queue_word(s, word);
        s->s3_wire_n = 0;
    }
}

static void lcd_stage3(void *opaque, const uint8_t *bytes, unsigned n)
{
    ESP32P4LcdCamState *s = opaque;
    static const uint8_t order[6][3] = {
        { 1, 0, 0 }, { 0, 2, 1 }, { 1, 0, 2 },
        { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 },
    };
    unsigned mode = CFG(USER, SWIZZLE_MODE), size, i, j;

    if (!CFG(USER, SWIZZLE_EN) || mode > 5) {
        for (i = 0; i < n; i++) {
            lcd_wire_byte(s, bytes[i]);
        }
        return;
    }
    size = mode ? 3 : 2;
    for (i = 0; i < n; i++) {
        s->s3_sw[s->s3_sw_n++] = bytes[i];
        if (s->s3_sw_n == size) {
            for (j = 0; j < size; j++) {
                lcd_wire_byte(s, s->s3_sw[order[mode][j]]);
            }
            s->s3_sw_n = 0;
        }
    }
}

static unsigned lcd_stride(ESP32P4LcdCamState *s)
{
    unsigned mode = CFG(USER, BYTE_MODE);

    return mode == 3 ? 1 : mode + 1;
}

/* Stage one: GDMA's bytes in groups of LCD_BYTE_MODE's. */
/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void lcd_feed(ESP32P4LcdCamState *s, const uint8_t *bytes, unsigned n)
{
    unsigned stride = lcd_stride(s), i, j;
    bool bit_order = CFG(USER, BIT_ORDER);
    bool byte_order = CFG(USER, BYTE_ORDER) && stride > 1;

    for (i = 0; i < n; i++) {
        s->s1[s->s1_n++] = bit_order ? revbit8(bytes[i]) : bytes[i];
        if (s->s1_n == stride) {
            uint8_t g[3];

            for (j = 0; j < stride; j++) {
                g[j] = s->s1[byte_order ? stride - 1 - j : j];
            }
            esp32p4_lcd_cam_conv_feed(&s->lcd_conv, g, stride, lcd_stage3,
                                      s);
            s->s1_n = 0;
        }
    }
}

/* Complete partial groups with zeros, at the end of a GDMA frame. */
static void lcd_flush(ESP32P4LcdCamState *s)
{
    static const uint8_t zero[3];

    if (s->s1_n) {
        lcd_feed(s, zero, lcd_stride(s) - s->s1_n);
    }
    esp32p4_lcd_cam_conv_flush(&s->lcd_conv, lcd_stage3, s);
    if (s->s3_sw_n) {
        lcd_stage3(s, zero, (FIELD_EX32(s->lcd_cfg[L_USER], LCD_USER,
                                        SWIZZLE_MODE) ? 3 : 2) - s->s3_sw_n);
    }
    while (s->s3_wire_n) {
        lcd_wire_byte(s, 0);
    }
}

static void lcd_fifo_reset(ESP32P4LcdCamState *s)
{
    s->q_head = 0;
    s->q_num = 0;
    s->q_eof = 0;
    s->s1_n = 0;
    s->s3_sw_n = 0;
    s->s3_wire_n = 0;
    esp32p4_lcd_cam_conv_reset(&s->lcd_conv);
}

static bool lcd_has_data(ESP32P4LcdCamState *s)
{
    return s->q_num || s->s1_n || s->s3_sw_n || s->s3_wire_n ||
           s->lcd_conv.in_n || s->lcd_conv.pend_n || s->lcd_conv.swap_held;
}

/*
 * Take bytes from GDMA, a FIFO's worth at a time, until @need words are
 * ready (stopping at a frame's end in i80 mode). Returns whether any
 * came.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static bool lcd_pull(ESP32P4LcdCamState *s, uint32_t need)
{
    bool any = false, i80 = !CFG(CTRL, RGB_MODE_EN);
    uint8_t buf[ESP32P4_GDMA_FIFO_MAX];

    if (!s->gdma) {
        return false;
    }
    while (s->q_num < need && s->q_num < ESP32P4_LCD_CAM_RUN &&
           !(i80 && s->q_eof)) {
        bool eof = false;
        size_t n = esp32p4_gdma_pull(s->gdma, PERIPH, buf, sizeof(buf),
                                     &eof);

        if (!n) {
            break;
        }
        any = true;
        lcd_feed(s, buf, n);
        if (eof && i80) {
            lcd_flush(s);
            s->q_eof = s->q_num ? s->q_num : 0;
        }
    }
    return any;
}

/* Whether GDMA is still moving data towards the LCD. */
static bool lcd_dma_flowing(ESP32P4LcdCamState *s)
{
    return s->gdma && esp32p4_gdma_tx_flowing(s->gdma, PERIPH);
}

/*
 * The LCD engine
 * ==============
 */

enum {
    EV_NONE,
    EV_EDGE,
    EV_I80,
    EV_RGB,
    EV_RGB_DATA,
};

static void lcd_stop(ESP32P4LcdCamState *s)
{
    s->lcd_state = LCD_IDLE;
    s->lcd_user = FIELD_DP32(s->lcd_user, LCD_USER, START, 0);
    s->run_n = 0;
    s->i80_stalled = false;
    s->rgb_starved = false;
    s->rgb_in_line = false;
    lcd_outputs_idle(s);
}

static uint32_t lcd_mo(ESP32P4LcdCamState *s)
{
    if (CFG(CLOCK, CLK_EQU_SYSCLK)) {
        return 1;
    }
    if (!CFG(CLOCK, CLKCNT_N)) {
        if (log_once(s, LOG_CLKCNT)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: LCD_CLKCNT_N 0 "
                          "with LCD_CLK_EQU_SYSCLK clear, which the TRM does "
                          "not allow: taken as a divider of 1\n");
        }
        return 1;
    }
    return CFG(CLOCK, CLKCNT_N) + 1;
}

static void lcd_check_format(ESP32P4LcdCamState *s)
{
    if (CFG(USER, BYTE_MODE) == 3 || CFG(MISC, WIRE_MODE) == 3 ||
        (CFG(USER, BYTE_ORDER) && !CFG(USER, BYTE_MODE)) ||
        (CFG(USER, SWIZZLE_EN) && CFG(USER, SWIZZLE_MODE) > 5)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: a data format the "
                      "TRM calls invalid (LCD_BYTE_MODE 3, LCD_WIRE_MODE 3, "
                      "LCD_BYTE_ORDER at 8 bits or swizzle mode 6 or 7): "
                      "taken as 8 bits, unreordered\n");
    }
    if (!esp32p4_lcd_cam_conv_valid(&s->lcd_conv)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: LCD_RGB_YUV names "
                      "no conversion the TRM lists (a YUV mode of 3): the "
                      "converter passes the data through\n");
    }
    if (CFG(RGB_YUV, TXTORX) && log_once(s, LOG_TXTORX)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4.lcd-cam: LCD_CONV_TXTORX is not "
                      "modelled\n");
    }
}

/* The data path's settings, LCD_UPDATE's copy. */
static void lcd_take_cfg(ESP32P4LcdCamState *s)
{
    memcpy(s->lcd_cfg, s->lcd_act, sizeof(s->lcd_cfg));
    esp32p4_lcd_cam_conv_setup(&s->lcd_conv, s->lcd_cfg[L_RGB_YUV], true,
                               CFG(MISC, WIRE_MODE) == 2, 0);
}

/* i80 */

static uint64_t i80_hold_cycles(ESP32P4LcdCamState *s)
{
    return CFG(MISC, BK_EN) ? CFG(MISC, VBK_CYCLELEN) + 1 : 0;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void i80_begin(ESP32P4LcdCamState *s)
{
    uint64_t *ph = s->i80_phase;

    ph[PH_SETUP] = 0;
    ph[PH_CMD] = CFG(MISC, BK_EN) ? CFG(MISC, VFK_CYCLELEN) + 1 : 0;
    ph[PH_DUMMY] = ph[PH_CMD] +
                   (CFG(USER, CMD) ? 1 + CFG(USER, CMD_2_CYCLE_EN) : 0);
    ph[PH_DOUT] = ph[PH_DUMMY] +
                  (CFG(USER, DUMMY) ? CFG(USER, DUMMY_CYCLELEN) + 1 : 0);
    ph[PH_HOLD] = UINT64_MAX;
    ph[PH_END] = UINT64_MAX;
    if (!CFG(USER, DOUT)) {
        ph[PH_HOLD] = ph[PH_DOUT];
        ph[PH_END] = ph[PH_HOLD] + i80_hold_cycles(s);
    }
    s->i80_ph = PH_SETUP;
    s->i80_dout_sent = 0;
    s->i80_stalled = false;
}

static bool i80_dc(ESP32P4LcdCamState *s, unsigned ph)
{
    bool idle = CFG(MISC, CD_IDLE_EDGE);

    switch (ph) {
    case PH_CMD:
        return idle ^ CFG(MISC, CD_CMD_SET);
    case PH_DUMMY:
        return idle ^ CFG(MISC, CD_DUMMY_SET);
    case PH_DOUT:
        return idle ^ CFG(MISC, CD_DATA_SET);
    default:
        return idle;
    }
}

/* Whether DOUT is in progress, its end not yet known. */
static bool i80_in_dout(ESP32P4LcdCamState *s)
{
    return s->i80_ph == PH_DOUT + 1 && s->i80_phase[PH_HOLD] == UINT64_MAX;
}

static void i80_deliver_run(ESP32P4LcdCamState *s)
{
    if (s->run_n) {
        lcd_dbi_run(s, i80_dc(s, PH_DOUT), s->run, s->run_n, s->run_first_ns,
                    esp32_line_time());
        lcd_data_lines(s, s->run[s->run_n - 1]);
        s->run_n = 0;
    }
}

static void i80_end_dout(ESP32P4LcdCamState *s)
{
    i80_deliver_run(s);
    s->i80_phase[PH_HOLD] = s->lcd_pos;
    s->i80_phase[PH_END] = s->lcd_pos + i80_hold_cycles(s);
}

static void i80_put(ESP32P4LcdCamState *s, uint32_t word)
{
    if (!s->run_n) {
        s->run_first_ns = lcd_pos_ns(s, s->lcd_pos);
    }
    s->run[s->run_n++] = word;
    s->i80_dout_sent++;
    s->lcd_pos++;
}

/* DOUT's cycles up to @pos take the queue's words. */
static void i80_dout_to(ESP32P4LcdCamState *s, uint64_t pos)
{
    bool always = CFG(USER, ALWAYS_OUT_EN);
    uint32_t limit = CFG(USER, DOUT_CYCLELEN) + 1;

    while (s->lcd_pos < pos && s->q_num) {
        bool eof = s->q_eof == 1;

        i80_put(s, lcd_take_word(s));
        if ((always && eof) || (!always && s->i80_dout_sent == limit)) {
            i80_end_dout(s);
            return;
        }
        if (s->run_n == ESP32P4_LCD_CAM_RUN) {
            i80_deliver_run(s);
        }
    }
}

/* DOUT at a cycle with no word ready: refill, wait, or end. */
static void i80_dout_refill(ESP32P4LcdCamState *s)
{
    uint32_t limit = CFG(USER, DOUT_CYCLELEN) + 1;

    if (s->gdma) {
        esp32p4_gdma_sync(s->gdma, esp32_line_time());
    }
    if (lcd_pull(s, ESP32P4_LCD_CAM_RUN)) {
        return;
    }
    if (lcd_dma_flowing(s)) {
        s->i80_stalled = true;
        return;
    }
    if (CFG(USER, ALWAYS_OUT_EN)) {
        i80_end_dout(s);
        return;
    }
    /* Underrun: the rest of DOUT holds the last word. */
    lcd_cam_raise(s, INT_LCD_UNDERRUN);
    while (s->i80_dout_sent < limit) {
        i80_put(s, s->last_word);
        if (s->run_n == ESP32P4_LCD_CAM_RUN) {
            i80_deliver_run(s);
        }
    }
    i80_end_dout(s);
}

static void i80_cmd_words(ESP32P4LcdCamState *s)
{
    uint64_t start = s->i80_phase[PH_CMD];
    uint32_t n = s->i80_phase[PH_DUMMY] - start, w[2], i;
    uint32_t mask = MAKE_64BIT_MASK(0, 8 * lcd_wire_bytes(s));

    for (i = 0; i < n; i++) {
        w[i] = s->lcd_cfg[L_CMD0 + i] & mask;
    }
    lcd_dbi_run(s, i80_dc(s, PH_CMD), w, n, lcd_pos_ns(s, start),
                esp32_line_time());
    lcd_data_lines(s, w[n - 1]);
}

static void i80_dummy_words(ESP32P4LcdCamState *s)
{
    uint64_t start = s->i80_phase[PH_DUMMY];
    uint32_t zeros[4] = { 0 };

    lcd_dbi_run(s, i80_dc(s, PH_DUMMY), zeros, s->i80_phase[PH_DOUT] - start,
                lcd_pos_ns(s, start), esp32_line_time());
    lcd_data_lines(s, 0);
}

/*
 * The phase boundary at @pos: the phases that have ended give their
 * words, and the last phase starting there begins (empty ones skipped).
 */
static void i80_boundary(ESP32P4LcdCamState *s, uint64_t pos)
{
    uint64_t *ph = s->i80_phase;
    unsigned k = s->i80_ph;

    if (k > PH_CMD && k - 1 == PH_CMD && ph[PH_DUMMY] > ph[PH_CMD]) {
        i80_cmd_words(s);
    }
    if (k > PH_DUMMY && k - 1 == PH_DUMMY && ph[PH_DOUT] > ph[PH_DUMMY]) {
        i80_dummy_words(s);
    }
    while (k < PH_END && ph[k + 1] == ph[k]) {
        k++;
    }
    s->lcd_pos = pos;
    s->i80_ph = k + 1;
    if (k == PH_END) {
        if (pos) {
            lcd_out(s, ESP32P4_LCD_CAM_OUT_DC, CFG(MISC, CD_IDLE_EDGE));
            lcd_out(s, ESP32P4_LCD_CAM_OUT_CS, true);
        }
        s->lcd_state = LCD_IDLE;
        s->lcd_user = FIELD_DP32(s->lcd_user, LCD_USER, START, 0);
        lcd_cam_raise(s, INT_LCD_TRANS_DONE);
        return;
    }
    if (pos == 0) {
        lcd_out(s, ESP32P4_LCD_CAM_OUT_CS, false);
    }
    lcd_out(s, ESP32P4_LCD_CAM_OUT_DC, i80_dc(s, k));
    if (k == PH_DOUT && !s->q_num) {
        i80_dout_refill(s);
    }
}

/*
 * The next i80 event: in DOUT, the cycle after the words ready (where
 * the queue must be refilled) or a run's worth; otherwise the next phase
 * boundary. None while DOUT waits for GDMA.
 */
static bool i80_next(ESP32P4LcdCamState *s, uint64_t *pos)
{
    uint64_t n;

    if (!i80_in_dout(s)) {
        *pos = s->i80_phase[s->i80_ph];
        return true;
    }
    if (s->i80_stalled) {
        return false;
    }
    n = s->q_eof ? s->q_eof : s->q_num;
    if (!CFG(USER, ALWAYS_OUT_EN)) {
        n = MIN(n, CFG(USER, DOUT_CYCLELEN) + 1 - s->i80_dout_sent);
    }
    *pos = s->lcd_pos + MIN(n, ESP32P4_LCD_CAM_RUN - s->run_n);
    return true;
}

static void i80_event(ESP32P4LcdCamState *s, uint64_t pos)
{
    if (!i80_in_dout(s)) {
        i80_boundary(s, pos);
        return;
    }
    i80_dout_to(s, pos);
    if (s->run_n == ESP32P4_LCD_CAM_RUN) {
        i80_deliver_run(s);
    }
    if (i80_in_dout(s) && !s->q_num) {
        i80_dout_refill(s);
    }
}

/* RGB */

static void rgb_take_frame(ESP32P4LcdCamState *s)
{
    ESP32P4LcdCamFrame *f = &s->frame;

    lcd_take_cfg(s);
    s->lcd_mo = lcd_mo(s);
    f->line_cycles = CFG(CTRL1, HT_WIDTH) + 1;
    f->lines = CFG(CTRL, VT_HEIGHT) + 1;
    f->active_start = CFG(CTRL, HB_FRONT) + 1;
    f->active_cycles = CFG(CTRL1, HA_WIDTH) + 1;
    f->active_line = CFG(CTRL1, VB_FRONT) + 1;
    f->active_lines = CFG(CTRL, VA_HEIGHT) + 1;
    f->hsync_pos = CFG(CTRL2, HSYNC_POSITION);
    f->hsync_cycles = CFG(CTRL2, HSYNC_WIDTH) + 1;
    f->vsync_lines = CFG(CTRL2, VSYNC_WIDTH) + 1;
    f->hsync_blank = CFG(CTRL2, HS_BLANK_EN);
    if (f->active_start + f->active_cycles > f->line_cycles ||
        f->active_line + f->active_lines > f->lines ||
        f->active_cycles > ESP32P4_LCD_CAM_LINE) {
        if (log_once(s, LOG_FRAME_FIT)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: the RGB frame's "
                          "active region (%u x %u from %u, %u) does not fit "
                          "its total (%u x %u): cut to fit\n",
                          f->active_cycles, f->active_lines,
                          f->active_start, f->active_line, f->line_cycles,
                          f->lines);
        }
        f->active_start = MIN(f->active_start, f->line_cycles);
        f->active_cycles = MIN(MIN(f->active_cycles,
                                   f->line_cycles - f->active_start),
                               ESP32P4_LCD_CAM_LINE);
        f->active_line = MIN(f->active_line, f->lines);
        f->active_lines = MIN(f->active_lines, f->lines - f->active_line);
    }
    if (!CFG(MISC, BK_EN) && log_once(s, LOG_RGB_BK)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4.lcd-cam: RGB mode without "
                      "LCD_BK_EN (no blanking) is not modelled: the frame "
                      "keeps its blanking\n");
    }
    if (esp32p4_lcd_cam_conv_on(&s->lcd_conv)) {
        /* YUV420's lines: a line's bytes over the output's per pixel */
        s->lcd_conv.line_pixels = f->active_cycles * lcd_wire_bytes(s) /
                                  (CFG(MISC, WIRE_MODE) == 2 ? 3 : 2);
    }
}

static bool rgb_active_line(ESP32P4LcdCamState *s, uint32_t l)
{
    return l >= s->frame.active_line &&
           l < s->frame.active_line + s->frame.active_lines;
}

static bool rgb_hsync_line(ESP32P4LcdCamState *s, uint32_t l)
{
    return s->frame.hsync_blank || rgb_active_line(s, l);
}

static uint64_t rgb_frame_cycles(ESP32P4LcdCamState *s)
{
    return (uint64_t)s->frame.lines * s->frame.line_cycles;
}

/*
 * The next structural event at or after @from: VSYNC's edges, HSYNC's
 * while HSYNC reaches a pad, DE's, and the frame's end.
 */
static uint64_t rgb_next_struct(ESP32P4LcdCamState *s, uint64_t from)
{
    const ESP32P4LcdCamFrame *f = &s->frame;
    uint64_t L = f->line_cycles, end = rgb_frame_cycles(s), best;
    bool hs = out_routed(s, ESP32P4_LCD_CAM_OUT_HSYNC);
    uint32_t l, c, offs[5], n, i, next;

    while (from < end) {
        l = from / L;
        c = from % L;
        n = 0;
        if (l == 0 || l == f->vsync_lines) {
            offs[n++] = 0;
        }
        if (hs && rgb_hsync_line(s, l)) {
            offs[n++] = f->hsync_pos;
            offs[n++] = f->hsync_pos + f->hsync_cycles;
        }
        if (rgb_active_line(s, l)) {
            offs[n++] = f->active_start;
            offs[n++] = f->active_start + f->active_cycles;
        }
        best = UINT64_MAX;
        for (i = 0; i < n; i++) {
            if (offs[i] >= c && offs[i] < L) {
                best = MIN(best, (uint64_t)l * L + offs[i]);
            }
        }
        if (best != UINT64_MAX) {
            return best;
        }
        /* None left in this line: the next line that has any. */
        next = f->lines;
        if (l < f->vsync_lines) {
            next = f->vsync_lines;
        }
        if (l + 1 < f->active_line) {
            next = MIN(next, f->active_line);
        } else if (rgb_active_line(s, l + 1)) {
            next = MIN(next, l + 1);
        }
        if (hs && f->hsync_blank) {
            next = MIN(next, l + 1);
        }
        from = (uint64_t)MAX(next, l + 1) * L;
    }
    return end;
}

/* The current data line's first active cycle. */
static uint64_t rgb_line_start(ESP32P4LcdCamState *s)
{
    return s->rgb_line * (uint64_t)s->frame.line_cycles +
           s->frame.active_start;
}

/* Fill the line's active cycles up to @upto from the queue. */
static void rgb_fill_to(ESP32P4LcdCamState *s, uint32_t upto)
{
    while (s->rgb_fill < upto && s->q_num) {
        s->line[s->rgb_fill++] = lcd_take_word(s);
    }
}

static void rgb_underrun_to(ESP32P4LcdCamState *s, uint32_t upto)
{
    if (s->rgb_fill >= upto) {
        return;
    }
    if (!s->rgb_underrun) {
        lcd_cam_raise(s, INT_LCD_UNDERRUN);
        s->rgb_underrun = true;
    }
    while (s->rgb_fill < upto) {
        s->line[s->rgb_fill++] = s->last_word;
    }
}

/* At the line's fill, its queue empty: refill, or starve. */
static void rgb_refill(ESP32P4LcdCamState *s)
{
    if (s->gdma) {
        esp32p4_gdma_sync(s->gdma, esp32_line_time());
    }
    if (!lcd_pull(s, ESP32P4_LCD_CAM_RUN)) {
        s->rgb_starved = true;
    }
}

/* An output's level, active or not, by its LCD_CTRL2 idle polarity. */
static bool rgb_level(ESP32P4LcdCamState *s, unsigned idle_shift,
                      bool active)
{
    return extract32(s->lcd_cfg[L_CTRL2], idle_shift, 1) ^ active;
}

static void rgb_vsync_edge(ESP32P4LcdCamState *s, bool active)
{
    bool level = rgb_level(s, R_LCD_CTRL2_VSYNC_IDLE_POL_SHIFT, active);
    PanelRun r = { .s = s, .first_ns = esp32_line_time() };

    lcd_out(s, ESP32P4_LCD_CAM_OUT_VSYNC, level);
    if (out_routed(s, ESP32P4_LCD_CAM_OUT_VSYNC) &&
        out_routed(s, ESP32P4_LCD_CAM_OUT_PCLK)) {
        r.dc = level ^ s->out_inv[ESP32P4_LCD_CAM_OUT_VSYNC];
        lcd_bus_foreach(s->bus, panel_vsync, &r);
    }
}

static void rgb_line_end(ESP32P4LcdCamState *s)
{
    const ESP32P4LcdCamFrame *f = &s->frame;
    bool de = rgb_level(s, R_LCD_CTRL2_DE_IDLE_POL_SHIFT, true);
    PanelRun r = {
        .s = s,
        .words = s->line,
        .count = f->active_cycles,
        .first_ns = lcd_pos_ns(s, rgb_line_start(s)),
        .de_end_ns = esp32_line_time(),
    };

    rgb_fill_to(s, f->active_cycles);
    rgb_underrun_to(s, f->active_cycles);
    s->rgb_in_line = false;
    s->rgb_starved = false;
    lcd_out(s, ESP32P4_LCD_CAM_OUT_DE, !de);
    if (out_routed(s, ESP32P4_LCD_CAM_OUT_DE) &&
        out_routed(s, ESP32P4_LCD_CAM_OUT_PCLK)) {
        r.de = de ^ s->out_inv[ESP32P4_LCD_CAM_OUT_DE];
        lcd_bus_foreach(s->bus, panel_line, &r);
    }
    if (s->rgb_line == f->active_line + f->active_lines - 1) {
        lcd_data_lines(s, s->last_word);
        lcd_cam_raise(s, INT_LCD_VSYNC);
    }
}

static void rgb_frame_end(ESP32P4LcdCamState *s)
{
    if (!CFG(MISC, NEXT_FRAME_EN)) {
        lcd_stop(s);
        lcd_cam_raise(s, INT_LCD_TRANS_DONE);
        return;
    }
    s->lcd_pos0 += rgb_frame_cycles(s) * s->lcd_mo;
    s->lcd_pos = 0;
    s->rgb_frames++;
    rgb_take_frame(s);
}

/* The structural events at @pos, at their time. */
/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void rgb_event(ESP32P4LcdCamState *s, uint64_t pos)
{
    const ESP32P4LcdCamFrame *f = &s->frame;
    uint64_t L = f->line_cycles;
    uint32_t l = pos / L, c = pos % L;
    bool hs_line = rgb_hsync_line(s, l);

    if (pos >= rgb_frame_cycles(s)) {
        rgb_frame_end(s);
        return;
    }
    s->lcd_pos = pos + 1;
    if (c == 0 && l == 0) {
        rgb_vsync_edge(s, true);
    }
    if (c == 0 && l == f->vsync_lines) {
        rgb_vsync_edge(s, false);
    }
    if (hs_line && c == f->hsync_pos + f->hsync_cycles &&
        out_routed(s, ESP32P4_LCD_CAM_OUT_HSYNC)) {
        lcd_out(s, ESP32P4_LCD_CAM_OUT_HSYNC,
                rgb_level(s, R_LCD_CTRL2_HSYNC_IDLE_POL_SHIFT, false));
    }
    if (hs_line && c == f->hsync_pos &&
        out_routed(s, ESP32P4_LCD_CAM_OUT_HSYNC)) {
        lcd_out(s, ESP32P4_LCD_CAM_OUT_HSYNC,
                rgb_level(s, R_LCD_CTRL2_HSYNC_IDLE_POL_SHIFT, true));
    }
    if (!rgb_active_line(s, l)) {
        return;
    }
    if (c == f->active_start + f->active_cycles && s->rgb_in_line) {
        rgb_line_end(s);
    }
    if (c == f->active_start) {
        s->rgb_line = l;
        s->rgb_fill = 0;
        s->rgb_in_line = true;
        s->rgb_starved = false;
        s->rgb_underrun = false;
        lcd_out(s, ESP32P4_LCD_CAM_OUT_DE,
                rgb_level(s, R_LCD_CTRL2_DE_IDLE_POL_SHIFT, true));
        if (!s->q_num) {
            rgb_refill(s);
        }
    }
}

/* Inside a data line: where the words ready run out, before @limit. */
static bool rgb_next_data(ESP32P4LcdCamState *s, uint64_t limit,
                          uint64_t *pos)
{
    uint64_t dry;

    if (!s->rgb_in_line || s->rgb_starved) {
        return false;
    }
    dry = rgb_line_start(s) + s->rgb_fill + s->q_num;
    if (dry >= limit) {
        return false;
    }
    *pos = dry;
    return true;
}

static void rgb_data_event(ESP32P4LcdCamState *s, uint64_t pos)
{
    rgb_fill_to(s, pos - rgb_line_start(s));
    if (!s->q_num) {
        rgb_refill(s);
    }
}

/* GDMA's data came while the line starved: the cycles before it hold. */
static void rgb_unstarve(ESP32P4LcdCamState *s, int64_t when)
{
    uint64_t a = rgb_line_start(s), p;

    if (!s->rgb_in_line || !s->rgb_starved) {
        return;
    }
    p = MAX(lcd_pos_at(s, when), a + s->rgb_fill);
    p = MIN(p, a + s->frame.active_cycles);
    rgb_underrun_to(s, p - a);
    s->rgb_starved = !lcd_pull(s, ESP32P4_LCD_CAM_RUN);
}

/* Starting */

/* Start a transaction or frame at the first LCD_CLK edge after now. */
static void lcd_begin(ESP32P4LcdCamState *s)
{
    lcd_take_cfg(s);
    s->lcd_mo = lcd_mo(s);
    s->lcd_pos0 = lcd_lc_at(s, esp32_line_time()) + 1;
    s->lcd_pos = 0;
    s->run_n = 0;
    lcd_check_format(s);
    if (CFG(CTRL, RGB_MODE_EN)) {
        s->lcd_state = LCD_RGB;
        s->rgb_frames = 0;
        s->rgb_in_line = false;
        s->rgb_starved = false;
        rgb_take_frame(s);
    } else {
        s->lcd_state = LCD_I80;
        i80_begin(s);
    }
}

/* LCD_START set: begin once LCD_CLK runs and, in i80 mode, data has come. */
static void lcd_try_start(ESP32P4LcdCamState *s)
{
    if (!lcd_clk_running(s)) {
        s->lcd_state = LCD_WAIT_CLOCK;
        return;
    }
    if (!FIELD_EX32(s->lcd_act[L_CTRL], LCD_CTRL, RGB_MODE_EN)) {
        lcd_take_cfg(s);
        if (s->gdma) {
            esp32p4_gdma_sync(s->gdma, esp32_line_time());
        }
        if (!lcd_has_data(s) && !lcd_pull(s, 1)) {
            s->lcd_state = LCD_WAIT_DATA;
            return;
        }
    }
    lcd_begin(s);
}

static void lcd_try_start_cb(void *opaque)
{
    lcd_try_start(opaque);
}

/* The engine */

/* The next event and its time: false for none. */
static bool lcd_next_event(ESP32P4LcdCamState *s, int64_t *t, uint64_t *pos,
                           unsigned *kind)
{
    uint64_t p, d;
    uint32_t i;

    *kind = EV_NONE;
    *t = INT64_MAX;
    switch (s->lcd_state) {
    case LCD_I80:
        if (i80_next(s, &p)) {
            *kind = EV_I80;
        }
        break;
    case LCD_RGB:
        p = rgb_next_struct(s, s->lcd_pos);
        *kind = EV_RGB;
        if (rgb_next_data(s, p, &d)) {
            p = d;
            *kind = EV_RGB_DATA;
        }
        break;
    default:
        break;
    }
    if (*kind != EV_NONE) {
        *pos = p;
        *t = lcd_pos_ns(s, p);
        if (*t == INT64_MAX) {
            *kind = EV_NONE;
        }
    }
    for (i = 0; i < s->nedges; i++) {
        if (s->edges[i].ns < *t || (*kind == EV_NONE && i == 0)) {
            *t = s->edges[i].ns;
            *kind = EV_EDGE;
        }
    }
    return *kind != EV_NONE;
}

/* The delayed pad edges due by esp32_line_time(). */
static void lcd_take_edges(ESP32P4LcdCamState *s)
{
    int64_t now = esp32_line_time();
    uint32_t i = 0;

    while (i < s->nedges) {
        if (s->edges[i].ns <= now) {
            lcd_out_now(s, s->edges[i].out, s->edges[i].level);
            s->edges[i] = s->edges[--s->nedges];
        } else {
            i++;
        }
    }
}

/* The next event, at its time, esp32_line_time(). */
static void lcd_event(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;
    int64_t t;
    uint64_t pos;
    unsigned kind;

    lcd_take_edges(s);
    if (!lcd_next_event(s, &t, &pos, &kind) || t > esp32_line_time()) {
        return;
    }
    switch (kind) {
    case EV_I80:
        i80_event(s, pos);
        break;
    case EV_RGB:
        rgb_event(s, pos);
        break;
    case EV_RGB_DATA:
        rgb_data_event(s, pos);
        break;
    default:
        break;
    }
}

/* GDMA's data arrived in its empty FIFO, at esp32_line_time(). */
static void lcd_tx_arrived(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;
    int64_t now = esp32_line_time();

    switch (s->lcd_state) {
    case LCD_WAIT_DATA:
        if (lcd_pull(s, 1) || lcd_has_data(s)) {
            lcd_begin(s);
        }
        break;
    case LCD_I80:
        if (s->i80_stalled) {
            s->lcd_pos = MAX(s->lcd_pos, lcd_pos_at(s, now));
            s->i80_stalled = !lcd_pull(s, ESP32P4_LCD_CAM_RUN);
        }
        break;
    case LCD_RGB:
        rgb_unstarve(s, now);
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.lcd-cam]
 * The host has fallen ESP32_LINE_MAX_LAG_NS behind the LCD's events
 * (include/hw/gpio/esp32_line.h), at esp32_line_time(), the present. RGB
 * frames restart at the present, the ones between dropped; an i80
 * transaction goes on from the present, PCLK pausing as when GDMA holds it
 * up, so that a panel takes every word of it.
 */
static void lcd_slip(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;
    uint64_t next = lcd_lc_at(s, esp32_line_time()) + 1;

    if (log_once(s, LOG_LAG)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4.lcd-cam: the host is too slow for "
                      "the LCD: %s\n", s->lcd_state == LCD_RGB ?
                      "RGB frames restart at the present, frames dropped" :
                      "the i80 transaction goes on from the present");
    }
    if (s->lcd_state == LCD_RGB) {
        s->lcd_pos0 = next;
        s->lcd_pos = 0;
        s->rgb_in_line = false;
        s->rgb_starved = false;
        rgb_take_frame(s);
        return;
    }
    i80_deliver_run(s);
    if (next > s->lcd_pos0 + s->lcd_pos * s->lcd_mo) {
        s->lcd_pos0 = next - s->lcd_pos * s->lcd_mo;
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.lcd-cam]
 * Take the engine's events due by @until, each at its own time, with a
 * GDMA delivery that came during them taken at its time. On the host's
 * clock an event ESP32_LINE_MAX_LAG_NS behind the present, which the host
 * cannot follow, makes the LCD slip to the present first (lcd_slip()).
 */
static void lcd_run(ESP32P4LcdCamState *s, int64_t until)
{
    int64_t t;
    uint64_t pos;
    unsigned kind;
    bool have;

    if (s->in_lcd_run) {
        return;
    }
    s->in_lcd_run = true;
    for (;;) {
        have = lcd_next_event(s, &t, &pos, &kind);
        if (s->tx_ready_pending && (!have || s->tx_ready_ns <= t) &&
            s->tx_ready_ns <= until) {
            s->tx_ready_pending = false;
            esp32_line_run_at(s->tx_ready_ns, lcd_tx_arrived, s);
            continue;
        }
        if (!have || t > until) {
            break;
        }
        if (kind != EV_EDGE && esp32_line_lags(t)) {
            esp32_line_run_at(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                              lcd_slip, s);
            continue;
        }
        esp32_line_run_at(t, lcd_event, s);
    }
    s->in_lcd_run = false;
}

static void lcd_schedule(ESP32P4LcdCamState *s)
{
    int64_t t;
    uint64_t pos;
    unsigned kind;

    if (lcd_next_event(s, &t, &pos, &kind)) {
        timer_mod(&s->lcd_timer, t);
    } else {
        timer_del(&s->lcd_timer);
    }
}

/*
 * [spec:nuos:req:emu.esp32p4.lcd-cam]
 * Under icount the timer takes the event at its deadline and no later
 * ones, so that the LCD's events and every other device's run in order
 * of time. On the host's clock the timer comes late, by the main loop's
 * latency: it takes every event due by the present, each at its own
 * time, as software sampling the pads does (lcd_cam_line_sync()), since
 * one event a callback falls ever further behind a stream of lines.
 */
static void lcd_timer_cb(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;
    int64_t t, now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t pos;
    unsigned kind;

    /*
     * An access may have taken the event the timer was set for, and the
     * next may not have come yet.
     */
    if (lcd_next_event(s, &t, &pos, &kind) && t <= now) {
        lcd_run(s, icount_enabled() ? t : MAX(t, now));
    }
    lcd_schedule(s);
}

static void lcd_dma_tx_ready(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;

    if (s->in_lcd_run) {
        if (!s->tx_ready_pending) {
            s->tx_ready_pending = true;
            s->tx_ready_ns = esp32_line_time();
        }
        return;
    }
    lcd_run(s, esp32_line_time());
    lcd_tx_arrived(s);
    lcd_schedule(s);
}

/*
 * Camera
 * ======
 */

static bool cam_running(ESP32P4LcdCamState *s)
{
    return FIELD_EX32(s->cam_ctrl1, CAM_CTRL1, START) &&
           clock_is_enabled(s->cam_clk);
}

static uint32_t cam_act_field(ESP32P4LcdCamState *s, unsigned reg,
                              unsigned shift, unsigned len)
{
    return extract32(s->cam_act[reg], shift, len);
}

#define CAMCFG(reg, field) \
    cam_act_field(s, C_##reg, R_CAM_##reg##_##field##_SHIFT, \
                  R_CAM_##reg##_##field##_LENGTH)

/* Bytes the camera gathers before handing them on, at the last's time. */
#define CAM_CHUNK               32

static void cam_lost(ESP32P4LcdCamState *s)
{
    if (!s->cam_overflow_logged) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: the camera's RX "
                      "FIFO is full (GDMA takes too little): bytes are "
                      "lost\n");
        s->cam_overflow_logged = true;
    }
}

/*
 * Give GDMA the async RX FIFO's bytes, then @n more at @b, at
 * esp32_line_time(), an EOF after the last with @eof: what GDMA cannot
 * take stays in the FIFO, up to its 16 bytes, and the rest is lost. With
 * CAM_VS_EOF_EN the last byte waits for the next or for VSYNC, so that
 * the frame's end comes with its last byte (IN_SUC_EOF closes the
 * descriptor holding it, as ESP-IDF's frame-sized buffers need). While
 * an EOF waits for GDMA, bytes after it are lost.
 */
/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void cam_give(ESP32P4LcdCamState *s, const uint8_t *b, uint32_t n,
                     bool eof)
{
    uint8_t buf[sizeof(s->cam_fifo) + CAM_CHUNK];
    uint32_t len = s->cam_fifo_n, give, taken, keep;

    if (s->cam_fifo_eof && n) {
        cam_give(s, NULL, 0, true);
        if (s->cam_fifo_eof) {
            cam_lost(s);
            return;
        }
        len = s->cam_fifo_n;
    }
    memcpy(buf, s->cam_fifo, len);
    if (n) {
        memcpy(buf + len, b, n);
        len += n;
    }
    eof |= s->cam_fifo_eof;
    give = len;
    if (!eof && CAMCFG(CTRL, VS_EOF_EN) && give) {
        give--;
    }
    taken = 0;
    if (s->gdma && (give || eof)) {
        if (!give) {
            if (esp32p4_gdma_rx_room(s->gdma, PERIPH)) {
                esp32p4_gdma_push(s->gdma, PERIPH, NULL, 0, ESP32P4_GDMA_EOF);
                eof = false;
            }
        } else {
            bool mark = eof && give == len;

            taken = esp32p4_gdma_push(s->gdma, PERIPH, buf, give,
                                      mark ? ESP32P4_GDMA_EOF : 0);
            if (mark && taken == give) {
                eof = false;
            }
        }
    }
    keep = MIN(len - taken, sizeof(s->cam_fifo));
    if (keep < len - taken) {
        cam_lost(s);
    }
    memcpy(s->cam_fifo, buf + taken, keep);
    s->cam_fifo_n = keep;
    s->cam_fifo_eof = eof;
}

typedef struct CamGive {
    ESP32P4LcdCamState *s;
    const uint8_t *b;
    uint32_t n;
    bool eof;
} CamGive;

static void cam_give_cb(void *opaque)
{
    CamGive *g = opaque;

    cam_give(g->s, g->b, g->n, g->eof);
}

/* cam_give() as of @when, GDMA's rounds due by then run first. */
static void cam_give_at(ESP32P4LcdCamState *s, int64_t when,
                        const uint8_t *b, uint32_t n, bool eof)
{
    CamGive g = { .s = s, .b = b, .n = n, .eof = eof };

    if (s->gdma) {
        esp32p4_gdma_sync(s->gdma, when);
    }
    esp32_line_run_at(when, cam_give_cb, &g);
}

/* A line's converted bytes, gathered a chunk at a time. */
typedef struct CamSink {
    ESP32P4LcdCamState *s;
    int64_t when;
    uint8_t chunk[CAM_CHUNK];
    uint32_t n;
} CamSink;

static void cam_flush(CamSink *k, bool eof)
{
    if (k->n || eof) {
        cam_give_at(k->s, k->when, k->chunk, k->n, eof);
        k->n = 0;
    }
}

/* Bytes from the converter, at the time of the sample that gave them. */
static void cam_take(void *opaque, const uint8_t *bytes, unsigned n)
{
    CamSink *k = opaque;
    ESP32P4LcdCamState *s = k->s;
    uint32_t len = CAMCFG(CTRL1, REC_DATA_BYTELEN) + 1;
    bool vs_eof = CAMCFG(CTRL, VS_EOF_EN);
    unsigned i;

    for (i = 0; i < n; i++) {
        k->chunk[k->n++] = bytes[i];
        if (!vs_eof && ++s->cam_bytes == len) {
            s->cam_bytes = 0;
            cam_flush(k, true);
        } else if (k->n == CAM_CHUNK) {
            cam_flush(k, false);
        }
    }
}

/*
 * Which sensor signal the camera's input @in takes, as the board wires
 * the sensor's pads and the matrix routes them: -1 for none.
 */
static int cam_source(ESP32P4LcdCamState *s, unsigned in)
{
    static const int sync_sig[] = {
        [ESP32P4_LCD_CAM_IN_PCLK] = ESP32P4_DVP_PCLK,
        [ESP32P4_LCD_CAM_IN_VSYNC] = ESP32P4_DVP_VSYNC,
        [ESP32P4_LCD_CAM_IN_HSYNC] = ESP32P4_DVP_HSYNC,
        [ESP32P4_LCD_CAM_IN_DE] = ESP32P4_DVP_DE,
    };
    ESP32P4ImageSource *cam = s->camera;
    int pad = s->in_pad[in], own;
    unsigned sig;

    if (pad < 0) {
        return -1;
    }
    for (sig = 0; sig < ESP32P4_DVP_SIGNALS; sig++) {
        if (esp32p4_image_source_dvp_pad(cam, sig) == pad) {
            return sig;
        }
    }
    own = in < ESP32P4_LCD_CAM_IN_DATA0 ? sync_sig[in] :
          ESP32P4_DVP_D0 + in - ESP32P4_LCD_CAM_IN_DATA0;
    return esp32p4_image_source_dvp_pad(cam, own) < 0 ? own : -1;
}

/* Whether the sensor's active level of @sig reaches input @in as high. */
static bool cam_sees_active_high(ESP32P4LcdCamState *s, unsigned in,
                                 bool active_low, bool cam_inv)
{
    return !active_low ^ s->in_inv[in] ^ cam_inv;
}

static void cam_stream(void *opaque, bool on, int64_t when_ns)
{
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void cam_frame_end(ESP32P4LcdCamState *s, int64_t when)
{
    if (!cam_running(s)) {
        return;
    }
    s->cam_lines = 0;
    s->cam_frame_lines = 0;
    esp32p4_lcd_cam_conv_reset(&s->cam_conv);
    if (CAMCFG(CTRL, VS_EOF_EN)) {
        cam_give_at(s, when, NULL, 0, true);
    }
    lcd_cam_raise(s, INT_CAM_VSYNC);
}

static void cam_frame_end_cb(void *opaque)
{
    cam_frame_end(opaque, esp32_line_time());
}

static void cam_timer_cb(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;
    int64_t due = s->cam_vsync_due;

    s->cam_vsync_due = INT64_MAX;
    esp32_line_run_at(due, cam_frame_end_cb, s);
}

static void cam_vsync(void *opaque, uint32_t frame, bool active,
                      int64_t when_ns)
{
    ESP32P4LcdCamState *s = opaque;
    const ESP32P4ImageTiming *t = esp32p4_image_source_timing(s->camera);
    bool level;

    if (active == s->cam_vsync) {
        return;
    }
    s->cam_vsync = active;
    lcd_cam_route(s);
    if (cam_source(s, ESP32P4_LCD_CAM_IN_VSYNC) != ESP32P4_DVP_VSYNC) {
        return;
    }
    level = active ^ !cam_sees_active_high(s, ESP32P4_LCD_CAM_IN_VSYNC,
                                           t->vsync_active_low,
                                           CAMCFG(CTRL1, VSYNC_INV));
    if (level) {
        return;
    }
    if (CAMCFG(CTRL1, VSYNC_FILTER_EN) && t->clock_hz) {
        uint64_t cycles = CAMCFG(CTRL, VSYNC_FILTER_THRES) + 1;

        s->cam_vsync_due = when_ns +
            DIV_ROUND_UP(cycles * NANOSECONDS_PER_SECOND, t->clock_hz);
        timer_mod(&s->cam_timer, s->cam_vsync_due);
        return;
    }
    cam_frame_end(s, when_ns);
}

/* The camera's D[15:0] for a sensor sample, through the board's wiring. */
static uint16_t cam_sample(ESP32P4LcdCamState *s, const int8_t *src,
                           uint16_t inv, uint16_t konst, uint16_t v)
{
    uint16_t out = konst;
    unsigned i;

    for (i = 0; i < 16; i++) {
        if (src[i] >= 0) {
            out |= (v >> src[i] & 1) << i;
        }
    }
    return out ^ inv;
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void cam_dvp_line(void *opaque, const ESP32P4DvpLine *line)
{
    ESP32P4LcdCamState *s = opaque;
    const ESP32P4ImageTiming *t = esp32p4_image_source_timing(s->camera);
    bool two = CAMCFG(CTRL1, TWO_BYTE_EN);
    bool bit_order = CAMCFG(CTRL, BIT_ORDER);
    bool byte_order = CAMCFG(CTRL, BYTE_ORDER) && two;
    int8_t src[16];
    uint16_t inv = 0, konst = 0;
    bool identity = true;
    CamSink k = { .s = s, .n = 0 };
    uint32_t i;

    if (!cam_running(s)) {
        return;
    }
    lcd_cam_route(s);
    if (cam_source(s, ESP32P4_LCD_CAM_IN_PCLK) != ESP32P4_DVP_PCLK) {
        if (log_once(s, LOG_CAM_PCLK)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: the sensor's "
                          "PCLK does not reach CAM_PCLK: the camera takes "
                          "nothing\n");
        }
        return;
    }
    if (cam_source(s, ESP32P4_LCD_CAM_IN_DE) != ESP32P4_DVP_DE) {
        if (log_once(s, LOG_CAM_DE)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: the sensor's DE "
                          "does not reach CAM_DE: the camera takes "
                          "nothing\n");
        }
        return;
    }
    if (!cam_sees_active_high(s, ESP32P4_LCD_CAM_IN_DE, t->de_active_low,
                              CAMCFG(CTRL1, DE_INV))) {
        if (log_once(s, LOG_CAM_DE_INV)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: CAM_DE is "
                          "inverted from the sensor's DE, so the camera "
                          "would take the blanking, which the model does "
                          "not carry: it takes nothing\n");
        }
        return;
    }
    if (CAMCFG(CTRL1, VH_DE_MODE_EN) &&
        cam_source(s, ESP32P4_LCD_CAM_IN_HSYNC) != ESP32P4_DVP_HSYNC) {
        if (log_once(s, LOG_CAM_HSYNC)) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: VSYNC + HSYNC "
                          "mode with the sensor's HSYNC not reaching "
                          "CAM_HSYNC: the camera takes nothing\n");
        }
        return;
    }
    for (i = 0; i < 16; i++) {
        int sig = cam_source(s, ESP32P4_LCD_CAM_IN_DATA0 + i);

        src[i] = sig >= ESP32P4_DVP_D0 &&
                 sig - ESP32P4_DVP_D0 < (int)t->dvp_bits ?
                 sig - ESP32P4_DVP_D0 : -1;
        if (sig < 0 || src[i] < 0) {
            konst |= (s->in_level >> (ESP32P4_LCD_CAM_IN_DATA0 + i) & 1) << i;
        }
        inv |= (uint16_t)s->in_inv[ESP32P4_LCD_CAM_IN_DATA0 + i] << i;
        identity &= src[i] == (int)i || (src[i] < 0 && i >= t->dvp_bits);
    }
    identity &= !inv && !konst;
    if (s->cam_frame_lines++) {
        esp32p4_lcd_cam_conv_new_line(&s->cam_conv);
    }
    for (i = 0; i < line->count; i++) {
        uint16_t v = identity ? line->samples[i] :
                     cam_sample(s, src, inv, konst, line->samples[i]);
        uint8_t b[2];
        unsigned n = 1;

        b[0] = v;
        if (two) {
            b[1] = v >> 8;
            n = 2;
            if (byte_order) {
                b[0] = v >> 8;
                b[1] = v;
            }
        }
        if (bit_order) {
            b[0] = revbit8(b[0]);
            b[1] = revbit8(b[1]);
        }
        k.when = esp32p4_dvp_sample_ns(line, i);
        esp32p4_lcd_cam_conv_feed(&s->cam_conv, b, n, cam_take, &k);
    }
    cam_flush(&k, false);
    if (CAMCFG(CTRL, LINE_INT_EN) &&
        ++s->cam_lines == CAMCFG(CTRL1, LINE_INT_NUM) + 1) {
        s->cam_lines = 0;
        lcd_cam_raise(s, INT_CAM_HS);
    }
}

static const ESP32P4ImageSinkOps cam_sink_ops = {
    .stream = cam_stream,
    .vsync = cam_vsync,
    .dvp_line = cam_dvp_line,
};

static void cam_reset_module(ESP32P4LcdCamState *s)
{
    s->cam_fifo_n = 0;
    s->cam_fifo_eof = false;
    s->cam_lines = 0;
    s->cam_frame_lines = 0;
    s->cam_bytes = 0;
    s->cam_vsync_due = INT64_MAX;
    timer_del(&s->cam_timer);
    esp32p4_lcd_cam_conv_reset(&s->cam_conv);
}

static void cam_take_update(ESP32P4LcdCamState *s)
{
    s->cam_act[C_CTRL] = s->cam_ctrl & ~R_CAM_CTRL_UPDATE_MASK;
    s->cam_act[C_CTRL1] = s->cam_ctrl1;
    s->cam_act[C_RGB_YUV] = s->cam_rgb_yuv;
    s->cam_ctrl = FIELD_DP32(s->cam_ctrl, CAM_CTRL, UPDATE, 0);
    esp32p4_lcd_cam_conv_setup(&s->cam_conv, s->cam_rgb_yuv, false, false,
                               0);
    if (!esp32p4_lcd_cam_conv_valid(&s->cam_conv)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: CAM_RGB_YUV names no "
                      "conversion the TRM lists (a YUV mode of 3): the "
                      "converter passes the data through\n");
    }
    if (CAMCFG(CTRL, BYTE_ORDER) && !CAMCFG(CTRL1, TWO_BYTE_EN)) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: CAM_BYTE_ORDER in "
                      "8-bit mode, which the TRM calls invalid: ignored\n");
    }
    if (CAMCFG(CTRL, STOP_EN) && log_once(s, LOG_CAM_STOP)) {
        qemu_log_mask(LOG_UNIMP, "esp32p4.lcd-cam: CAM_STOP_EN is not "
                      "modelled: the camera loses what GDMA cannot take\n");
    }
}

static void cam_dma_rx_ready(void *opaque)
{
    ESP32P4LcdCamState *s = opaque;

    if (s->cam_fifo_n || s->cam_fifo_eof) {
        cam_give(s, NULL, 0, false);
    }
}

/*
 * GDMA
 * ====
 */

static void lcd_cam_sync_at(ESP32P4LcdCamState *s, int64_t now);

static void lcd_cam_dma_sync(void *opaque, int64_t when)
{
    lcd_cam_sync_at(opaque, when);
}

static const ESP32P4GdmaPeriphOps lcd_cam_dma_ops = {
    .tx_ready = lcd_dma_tx_ready,
    .rx_ready = cam_dma_rx_ready,
    .sync = lcd_cam_dma_sync,
};

/*
 * Registers
 * =========
 */

static void lcd_take_update_regs(ESP32P4LcdCamState *s)
{
    s->lcd_act[L_CLOCK] = s->lcd_clock;
    s->lcd_act[L_RGB_YUV] = s->lcd_rgb_yuv;
    s->lcd_act[L_USER] = s->lcd_user & ~(R_LCD_USER_UPDATE_MASK |
                                         R_LCD_USER_START_MASK);
    s->lcd_act[L_MISC] = s->lcd_misc;
    s->lcd_act[L_CTRL] = s->lcd_ctrl;
    s->lcd_act[L_CTRL1] = s->lcd_ctrl1;
    s->lcd_act[L_CTRL2] = s->lcd_ctrl2;
    s->lcd_act[L_CMD0] = s->lcd_cmd_val[0];
    s->lcd_act[L_CMD1] = s->lcd_cmd_val[1];
    s->lcd_act[L_DLY1] = s->lcd_dly_mode[0];
    s->lcd_act[L_DLY2] = s->lcd_dly_mode[1];
    s->lcd_user = FIELD_DP32(s->lcd_user, LCD_USER, UPDATE, 0);
}

static void lcd_take_update(ESP32P4LcdCamState *s)
{
    lcd_take_update_regs(s);
    if (s->lcd_state == LCD_IDLE) {
        lcd_outputs_idle(s);
    }
}

static void lcd_reset_module(ESP32P4LcdCamState *s)
{
    lcd_fifo_reset(s);
    lcd_stop(s);
}

/*
 * [spec:nuos:req:emu.esp32p4.lcd-cam]
 * Take what has come by now: the LCD's events, and the camera's lines and
 * its VSYNC filter's frame end, the camera first bringing its stream up
 * to now. Without icount their timers can run well after, while software
 * reads LC_DMA_INT_RAW's CAM_VSYNC or CAM_HS_INT, or GDMA's descriptors
 * for a frame; a register access, and GDMA's, first take them. Only a
 * timer that has come is run.
 */
static void lcd_cam_sync_at(ESP32P4LcdCamState *s, int64_t now)
{
    lcd_run(s, now);
    if (s->camera) {
        esp32p4_image_source_sync(s->camera);
    }
    if (timer_pending(&s->cam_timer) && s->cam_vsync_due <= now) {
        timer_del(&s->cam_timer);
        cam_timer_cb(s);
    }
}

static void lcd_cam_sync(ESP32P4LcdCamState *s)
{
    lcd_cam_sync_at(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static uint64_t lcd_cam_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32P4LcdCamState *s = opaque;

    lcd_cam_sync(s);
    switch (addr) {
    case A_LCD_CLOCK:
        return s->lcd_clock;
    case A_CAM_CTRL:
        return s->cam_ctrl;
    case A_CAM_CTRL1:
        return s->cam_ctrl1 & ~(R_CAM_CTRL1_RESET_MASK |
                                R_CAM_CTRL1_AFIFO_RESET_MASK);
    case A_CAM_RGB_YUV:
        return s->cam_rgb_yuv;
    case A_LCD_RGB_YUV:
        return s->lcd_rgb_yuv;
    case A_LCD_USER:
        return s->lcd_user & ~R_LCD_USER_RESET_MASK;
    case A_LCD_MISC:
        return s->lcd_misc & ~R_LCD_MISC_AFIFO_RESET_MASK;
    case A_LCD_CTRL:
        return s->lcd_ctrl;
    case A_LCD_CTRL1:
        return s->lcd_ctrl1;
    case A_LCD_CTRL2:
        return s->lcd_ctrl2;
    case A_LCD_FIRST_CMD_VAL:
        return s->lcd_cmd_val[0];
    case A_LCD_LATTER_CMD_VAL:
        return s->lcd_cmd_val[1];
    case A_LCD_DLY_MODE_CFG1:
        return s->lcd_dly_mode[0];
    case A_LCD_DLY_MODE_CFG2:
        return s->lcd_dly_mode[1];
    case A_LC_DMA_INT_ENA:
        return s->int_ena;
    case A_LC_DMA_INT_RAW:
        return s->int_raw;
    case A_LC_DMA_INT_ST:
        return s->int_raw & s->int_ena;
    case A_LC_DMA_INT_CLR:
        return 0;
    case A_LC_REG_DATE:
        return s->date;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: read of reserved "
                      "offset 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void lcd_user_write(ESP32P4LcdCamState *s, uint32_t val)
{
    bool was_started = FIELD_EX32(s->lcd_user, LCD_USER, START);
    bool start = FIELD_EX32(val, LCD_USER, START);

    if (FIELD_EX32(val, LCD_USER, RESET)) {
        lcd_reset_module(s);
        was_started = false;
    }
    s->lcd_user = val & ~R_LCD_USER_RESET_MASK;
    if (FIELD_EX32(val, LCD_USER, UPDATE) && lcd_clk_running(s)) {
        lcd_take_update(s);
    }
    if (start && !was_started) {
        lcd_try_start(s);
    } else if (!start && was_started) {
        lcd_stop(s);
    }
}

static void cam_ctrl1_write(ESP32P4LcdCamState *s, uint32_t val)
{
    if (FIELD_EX32(val, CAM_CTRL1, RESET)) {
        cam_reset_module(s);
        val = FIELD_DP32(val, CAM_CTRL1, START, 0);
    }
    if (FIELD_EX32(val, CAM_CTRL1, AFIFO_RESET)) {
        s->cam_fifo_n = 0;
        s->cam_fifo_eof = false;
    }
    if (FIELD_EX32(val, CAM_CTRL1, START) &&
        !FIELD_EX32(s->cam_ctrl1, CAM_CTRL1, START)) {
        s->cam_overflow_logged = false;
    }
    s->cam_ctrl1 = val & ~(R_CAM_CTRL1_RESET_MASK |
                           R_CAM_CTRL1_AFIFO_RESET_MASK);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static void lcd_cam_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    ESP32P4LcdCamState *s = opaque;
    uint32_t val = value;

    lcd_cam_sync(s);
    switch (addr) {
    case A_LCD_CLOCK:
        s->lcd_clock = val;
        break;
    case A_CAM_CTRL:
        s->cam_ctrl = val & CAM_CTRL_MASK;
        if (FIELD_EX32(val, CAM_CTRL, UPDATE) &&
            clock_is_enabled(s->cam_clk)) {
            cam_take_update(s);
        }
        break;
    case A_CAM_CTRL1:
        cam_ctrl1_write(s, val);
        break;
    case A_CAM_RGB_YUV:
        s->cam_rgb_yuv = val & CAM_RGB_YUV_MASK;
        break;
    case A_LCD_RGB_YUV:
        s->lcd_rgb_yuv = val & LCD_RGB_YUV_MASK;
        break;
    case A_LCD_USER:
        lcd_user_write(s, val);
        break;
    case A_LCD_MISC:
        if (FIELD_EX32(val, LCD_MISC, AFIFO_RESET)) {
            lcd_fifo_reset(s);
        }
        s->lcd_misc = val & LCD_MISC_MASK & ~R_LCD_MISC_AFIFO_RESET_MASK;
        break;
    case A_LCD_CTRL:
        s->lcd_ctrl = val;
        break;
    case A_LCD_CTRL1:
        s->lcd_ctrl1 = val;
        break;
    case A_LCD_CTRL2:
        s->lcd_ctrl2 = val & LCD_CTRL2_MASK;
        break;
    case A_LCD_FIRST_CMD_VAL:
        s->lcd_cmd_val[0] = val;
        break;
    case A_LCD_LATTER_CMD_VAL:
        s->lcd_cmd_val[1] = val;
        break;
    case A_LCD_DLY_MODE_CFG1:
        s->lcd_dly_mode[0] = val & LCD_DLY1_MASK;
        break;
    case A_LCD_DLY_MODE_CFG2:
        s->lcd_dly_mode[1] = val;
        break;
    case A_LC_DMA_INT_ENA:
        s->int_ena = val & INT_MASK;
        lcd_cam_update_irq(s);
        break;
    case A_LC_DMA_INT_RAW:
    case A_LC_DMA_INT_ST:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: write to read-only "
                      "%s\n", addr == A_LC_DMA_INT_RAW ? "LC_DMA_INT_RAW" :
                      "LC_DMA_INT_ST");
        break;
    case A_LC_DMA_INT_CLR:
        s->int_raw &= ~(val & INT_MASK);
        lcd_cam_update_irq(s);
        break;
    case A_LC_REG_DATE:
        s->date = val & DATE_MASK;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32p4.lcd-cam: write of reserved "
                      "offset 0x%03" HWADDR_PRIx "\n", addr);
        break;
    }
    lcd_schedule(s);
}

static const MemoryRegionOps lcd_cam_ops = {
    .read = lcd_cam_read,
    .write = lcd_cam_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/*
 * Clocks, lines, reset
 * ====================
 */

static void lcd_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4LcdCamState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (event == ClockPreUpdate) {
        lcd_run(s, now);
        s->lcd_count_base += esp32p4_clk_count_at(&s->lcd_count, now);
        return;
    }
    esp32p4_clk_count_start(&s->lcd_count, s->lcd_clk, now);
    if (lcd_clk_running(s)) {
        if (FIELD_EX32(s->lcd_user, LCD_USER, UPDATE)) {
            lcd_take_update(s);
        }
        if (s->lcd_state == LCD_WAIT_CLOCK) {
            esp32_line_run_at(now, lcd_try_start_cb, s);
        }
    }
    lcd_schedule(s);
}

static void cam_clk_update(void *opaque, ClockEvent event)
{
    ESP32P4LcdCamState *s = opaque;

    if (clock_is_enabled(s->cam_clk) &&
        FIELD_EX32(s->cam_ctrl, CAM_CTRL, UPDATE)) {
        cam_take_update(s);
    }
}

static void lcd_cam_in(void *opaque, int n, int level)
{
    ESP32P4LcdCamState *s = opaque;

    s->in_level = deposit32(s->in_level, n, 1, level);
}

/* Software samples the pads: the delayed edges come up to the present. */
static void lcd_cam_line_sync(Notifier *n, void *data)
{
    ESP32P4LcdCamState *s = container_of(n, ESP32P4LcdCamState,
                                         line_source);

    if (!s->in_lcd_run) {
        lcd_run(s, esp32_line_time());
        lcd_schedule(s);
    }
}

static void lcd_cam_reset_hold(Object *obj, ResetType type)
{
    ESP32P4LcdCamState *s = ESP32P4_LCD_CAM(obj);

    timer_del(&s->lcd_timer);
    s->lcd_clock = LCD_CLOCK_RESET;
    s->cam_ctrl = CAM_CTRL_RESET;
    s->cam_ctrl1 = 0;
    s->cam_rgb_yuv = CAM_RGB_YUV_RESET;
    s->lcd_rgb_yuv = LCD_RGB_YUV_RESET;
    s->lcd_user = LCD_USER_RESET;
    s->lcd_misc = LCD_MISC_RESET;
    s->lcd_ctrl = 0;
    s->lcd_ctrl1 = 0;
    s->lcd_ctrl2 = LCD_CTRL2_RESET;
    s->lcd_cmd_val[0] = s->lcd_cmd_val[1] = 0;
    s->lcd_dly_mode[0] = s->lcd_dly_mode[1] = 0;
    s->int_ena = 0;
    s->int_raw = 0;
    s->date = DATE_RESET;
    s->lcd_state = LCD_IDLE;
    lcd_take_update_regs(s);
    memcpy(s->lcd_cfg, s->lcd_act, sizeof(s->lcd_cfg));
    s->lcd_mo = 1;
    cam_take_update(s);
    esp32p4_lcd_cam_conv_setup(&s->lcd_conv, s->lcd_act[L_RGB_YUV], true,
                               false, 0);
    lcd_fifo_reset(s);
    s->lcd_state = LCD_IDLE;
    s->run_n = 0;
    s->nedges = 0;
    s->i80_stalled = false;
    s->rgb_starved = false;
    s->rgb_in_line = false;
    s->tx_ready_pending = false;
    s->last_word = 0;
    cam_reset_module(s);
    s->cam_vsync = false;
    s->cam_overflow_logged = false;
    s->logged = 0;
    s->route_valid = false;
}

static void lcd_cam_reset_exit(Object *obj, ResetType type)
{
    ESP32P4LcdCamState *s = ESP32P4_LCD_CAM(obj);
    unsigned i;

    s->lcd_count_base = 0;
    esp32p4_clk_count_start(&s->lcd_count, s->lcd_clk,
                            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    lcd_outputs_idle(s);
    for (i = 0; i < ESP32P4_LCD_CAM_OUT_DATA0; i++) {
        qemu_set_irq(s->sig_out[i], s->out_level[i]);
    }
    s->data_out = ~0u;
    lcd_data_lines(s, 0);
    lcd_cam_update_irq(s);
}

static void lcd_cam_init(Object *obj)
{
    ESP32P4LcdCamState *s = ESP32P4_LCD_CAM(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &lcd_cam_ops, s, "esp32p4.lcd-cam",
                          ESP32P4_LCD_CAM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(dev, s->sig_out, ESP32P4_LCD_CAM_SIG_OUT,
                             ESP32P4_LCD_CAM_OUTS);
    qdev_init_gpio_in_named(dev, lcd_cam_in, ESP32P4_LCD_CAM_SIG_IN,
                            ESP32P4_LCD_CAM_INS);
    s->lcd_clk = qdev_init_clock_in(dev, "lcd-clk", lcd_clk_update, s,
                                    ClockPreUpdate | ClockUpdate);
    s->cam_clk = qdev_init_clock_in(dev, "cam-clk", cam_clk_update, s,
                                    ClockUpdate);
    timer_init_ns(&s->lcd_timer, QEMU_CLOCK_VIRTUAL, lcd_timer_cb, s);
    timer_init_ns(&s->cam_timer, QEMU_CLOCK_VIRTUAL, cam_timer_cb, s);
    s->bus = lcd_bus_create(dev, "lcd");
    s->lcd_conv.lcd = true;
}

static void lcd_cam_realize(DeviceState *dev, Error **errp)
{
    ESP32P4LcdCamState *s = ESP32P4_LCD_CAM(dev);

    if (s->gdma) {
        esp32p4_gdma_attach(s->gdma, PERIPH, &lcd_cam_dma_ops, s);
    }
    if (s->gpio) {
        s->route_notifier.notify = lcd_cam_route_changed;
        esp32p4_gpio_add_route_notifier(s->gpio, &s->route_notifier);
    }
    if (s->camera) {
        esp32p4_image_source_attach(s->camera, &cam_sink_ops, s);
    }
    s->line_source.notify = lcd_cam_line_sync;
    esp32_line_add_source(&s->line_source);
}

static void lcd_cam_finalize(Object *obj)
{
    ESP32P4LcdCamState *s = ESP32P4_LCD_CAM(obj);

    timer_del(&s->lcd_timer);
    timer_del(&s->cam_timer);
}

static int lcd_cam_post_load(void *opaque, int version_id)
{
    ESP32P4LcdCamState *s = opaque;

    if (s->q_num > ESP32P4_LCD_CAM_QUEUE ||
        s->q_head >= ESP32P4_LCD_CAM_QUEUE || s->q_eof > s->q_num ||
        s->run_n > ESP32P4_LCD_CAM_RUN || s->nedges > ESP32P4_LCD_CAM_EDGES ||
        s->cam_fifo_n > sizeof(s->cam_fifo) || s->s1_n > 3 ||
        s->s3_sw_n > 3 || s->s3_wire_n > 3 || s->lcd_state > LCD_RGB ||
        s->i80_ph > PH_END || !s->lcd_mo ||
        s->rgb_fill > ESP32P4_LCD_CAM_LINE) {
        return -EINVAL;
    }
    s->route_valid = false;
    return 0;
}

static const VMStateDescription vmstate_lcd_cam_frame = {
    .name = "esp32p4-lcd-cam-frame",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(line_cycles, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(lines, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(active_start, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(active_cycles, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(active_line, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(active_lines, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(hsync_pos, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(hsync_cycles, ESP32P4LcdCamFrame),
        VMSTATE_UINT32(vsync_lines, ESP32P4LcdCamFrame),
        VMSTATE_BOOL(hsync_blank, ESP32P4LcdCamFrame),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_lcd_cam_edge = {
    .name = "esp32p4-lcd-cam-edge",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(ns, ESP32P4LcdCamEdge),
        VMSTATE_UINT8(out, ESP32P4LcdCamEdge),
        VMSTATE_UINT8(level, ESP32P4LcdCamEdge),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_lcd_cam = {
    .name = TYPE_ESP32P4_LCD_CAM,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = lcd_cam_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(lcd_clk, ESP32P4LcdCamState),
        VMSTATE_CLOCK(cam_clk, ESP32P4LcdCamState),
        VMSTATE_TIMER(lcd_timer, ESP32P4LcdCamState),
        VMSTATE_TIMER(cam_timer, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_clock, ESP32P4LcdCamState),
        VMSTATE_UINT32(cam_ctrl, ESP32P4LcdCamState),
        VMSTATE_UINT32(cam_ctrl1, ESP32P4LcdCamState),
        VMSTATE_UINT32(cam_rgb_yuv, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_rgb_yuv, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_user, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_misc, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_ctrl, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_ctrl1, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_ctrl2, ESP32P4LcdCamState),
        VMSTATE_UINT32_ARRAY(lcd_cmd_val, ESP32P4LcdCamState, 2),
        VMSTATE_UINT32_ARRAY(lcd_dly_mode, ESP32P4LcdCamState, 2),
        VMSTATE_UINT32(int_ena, ESP32P4LcdCamState),
        VMSTATE_UINT32(int_raw, ESP32P4LcdCamState),
        VMSTATE_UINT32(date, ESP32P4LcdCamState),
        VMSTATE_UINT32_ARRAY(lcd_act, ESP32P4LcdCamState,
                             ESP32P4_LCD_CAM_LCD_REGS),
        VMSTATE_UINT32_ARRAY(cam_act, ESP32P4LcdCamState,
                             ESP32P4_LCD_CAM_CAM_REGS),
        VMSTATE_ESP32P4_CLK_COUNT(lcd_count, ESP32P4LcdCamState),
        VMSTATE_UINT64(lcd_count_base, ESP32P4LcdCamState),
        VMSTATE_UINT8(lcd_state, ESP32P4LcdCamState),
        VMSTATE_UINT64(lcd_pos0, ESP32P4LcdCamState),
        VMSTATE_UINT32(lcd_mo, ESP32P4LcdCamState),
        VMSTATE_UINT32_ARRAY(lcd_cfg, ESP32P4LcdCamState,
                             ESP32P4_LCD_CAM_LCD_REGS),
        VMSTATE_UINT64(lcd_pos, ESP32P4LcdCamState),
        VMSTATE_UINT64_ARRAY(i80_phase, ESP32P4LcdCamState, 6),
        VMSTATE_UINT8(i80_ph, ESP32P4LcdCamState),
        VMSTATE_UINT32(i80_dout_sent, ESP32P4LcdCamState),
        VMSTATE_BOOL(i80_stalled, ESP32P4LcdCamState),
        VMSTATE_STRUCT(frame, ESP32P4LcdCamState, 1, vmstate_lcd_cam_frame,
                       ESP32P4LcdCamFrame),
        VMSTATE_UINT32(rgb_frames, ESP32P4LcdCamState),
        VMSTATE_UINT32(rgb_line, ESP32P4LcdCamState),
        VMSTATE_UINT32(rgb_fill, ESP32P4LcdCamState),
        VMSTATE_BOOL(rgb_in_line, ESP32P4LcdCamState),
        VMSTATE_BOOL(rgb_starved, ESP32P4LcdCamState),
        VMSTATE_BOOL(rgb_underrun, ESP32P4LcdCamState),
        VMSTATE_BOOL(tx_ready_pending, ESP32P4LcdCamState),
        VMSTATE_INT64(tx_ready_ns, ESP32P4LcdCamState),
        VMSTATE_BOOL_ARRAY(out_level, ESP32P4LcdCamState,
                           ESP32P4_LCD_CAM_OUT_DATA0),
        VMSTATE_UINT32(data_out, ESP32P4LcdCamState),
        VMSTATE_STRUCT_ARRAY(edges, ESP32P4LcdCamState,
                             ESP32P4_LCD_CAM_EDGES, 1, vmstate_lcd_cam_edge,
                             ESP32P4LcdCamEdge),
        VMSTATE_UINT32(nedges, ESP32P4LcdCamState),
        VMSTATE_UINT32(last_word, ESP32P4LcdCamState),
        VMSTATE_UINT32_ARRAY(q, ESP32P4LcdCamState, ESP32P4_LCD_CAM_QUEUE),
        VMSTATE_UINT32(q_head, ESP32P4LcdCamState),
        VMSTATE_UINT32(q_num, ESP32P4LcdCamState),
        VMSTATE_UINT32(q_eof, ESP32P4LcdCamState),
        VMSTATE_UINT8_ARRAY(s1, ESP32P4LcdCamState, 3),
        VMSTATE_UINT32(s1_n, ESP32P4LcdCamState),
        VMSTATE_UINT8_ARRAY(s3_sw, ESP32P4LcdCamState, 3),
        VMSTATE_UINT32(s3_sw_n, ESP32P4LcdCamState),
        VMSTATE_UINT8_ARRAY(s3_wire, ESP32P4LcdCamState, 3),
        VMSTATE_UINT32(s3_wire_n, ESP32P4LcdCamState),
        VMSTATE_STRUCT(lcd_conv, ESP32P4LcdCamState, 1,
                       vmstate_esp32p4_lcd_cam_conv, ESP32P4LcdCamConv),
        VMSTATE_UINT32_ARRAY(run, ESP32P4LcdCamState, ESP32P4_LCD_CAM_RUN),
        VMSTATE_UINT32(run_n, ESP32P4LcdCamState),
        VMSTATE_INT64(run_first_ns, ESP32P4LcdCamState),
        VMSTATE_UINT32_ARRAY(line, ESP32P4LcdCamState, ESP32P4_LCD_CAM_LINE),
        VMSTATE_UINT32(cam_lines, ESP32P4LcdCamState),
        VMSTATE_UINT32(cam_frame_lines, ESP32P4LcdCamState),
        VMSTATE_UINT32(cam_bytes, ESP32P4LcdCamState),
        VMSTATE_BOOL(cam_vsync, ESP32P4LcdCamState),
        VMSTATE_INT64(cam_vsync_due, ESP32P4LcdCamState),
        VMSTATE_UINT8_ARRAY(cam_fifo, ESP32P4LcdCamState, 16),
        VMSTATE_UINT32(cam_fifo_n, ESP32P4LcdCamState),
        VMSTATE_BOOL(cam_fifo_eof, ESP32P4LcdCamState),
        VMSTATE_STRUCT(cam_conv, ESP32P4LcdCamState, 1,
                       vmstate_esp32p4_lcd_cam_conv, ESP32P4LcdCamConv),
        VMSTATE_UINT32(in_level, ESP32P4LcdCamState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property lcd_cam_properties[] = {
    DEFINE_PROP_LINK("gdma", ESP32P4LcdCamState, gdma, TYPE_ESP32P4_GDMA,
                     ESP32P4GdmaState *),
    DEFINE_PROP_LINK("gpio", ESP32P4LcdCamState, gpio, TYPE_ESP32P4_GPIO,
                     ESP32P4GpioState *),
    DEFINE_PROP_LINK("camera", ESP32P4LcdCamState, camera,
                     TYPE_ESP32P4_IMAGE_SOURCE, ESP32P4ImageSource *),
};

static void lcd_cam_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = lcd_cam_realize;
    dc->vmsd = &vmstate_lcd_cam;
    rc->phases.hold = lcd_cam_reset_hold;
    rc->phases.exit = lcd_cam_reset_exit;
    device_class_set_props(dc, lcd_cam_properties);
}

/* [spec:nuos:req:emu.esp32p4.lcd-cam] */
static const TypeInfo lcd_cam_info = {
    .name = TYPE_ESP32P4_LCD_CAM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32P4LcdCamState),
    .instance_init = lcd_cam_init,
    .instance_finalize = lcd_cam_finalize,
    .class_init = lcd_cam_class_init,
};

static void lcd_cam_register_types(void)
{
    type_register_static(&lcd_cam_info);
}
type_init(lcd_cam_register_types)
