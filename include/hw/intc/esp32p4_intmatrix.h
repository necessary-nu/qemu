/*
 * ESP32-P4 interrupt matrix (INTMTX)
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_ESP32P4_INTMATRIX_H
#define HW_INTC_ESP32P4_INTMATRIX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32P4_INTMATRIX "esp32p4-intmatrix"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4IntMatrixState, ESP32P4_INTMATRIX)

#define ESP32P4_INTMATRIX_CORES     2

/*
 * The HP cores' peripheral interrupts the matrix drives: CLIC interrupt
 * IDs 16 to 47 of each core.
 */
#define ESP32P4_INTMATRIX_LINES     32
#define ESP32P4_INTMATRIX_LINE0     16

/*
 * The registers: core 0's matrix at the base (0x500D6000), core 1's
 * 2 KiB above it.
 */
#define ESP32P4_INTMATRIX_BASE      0x500d6000
#define ESP32P4_INTMATRIX_CORE_SIZE 0x800
#define ESP32P4_INTMATRIX_SIZE      (ESP32P4_INTMATRIX_CORES * \
                                     ESP32P4_INTMATRIX_CORE_SIZE)

/*
 * The peripheral interrupt sources, numbered as in TRM table 13.4-1 and
 * named after ESP-IDF's ETS_<name>_INTR_SOURCE (soc/interrupts.h), whose
 * numbers they are up to 127. 83, 99 and 112, which the TRM lists as
 * reserved, have mapping registers in v3 silicon's register set and
 * ESP-IDF names: the cache's error interrupt among them. ESP-IDF numbers
 * the three sources above 127 by their mapping registers' indices, 133 to
 * 135, as its routing code indexes the registers by source; the hardware
 * numbers them 128 to 130, and their status is bits 0 to 2 of
 * CORE<n>_INTR_STATUS_REG_4. TRM table 13.4-1 gives sources 4 and 5 as
 * MB_LP and MB_HP; their mapping registers, at 0x10 and 0x14, are MB_HP's
 * and MB_LP's, as ESP-IDF has them and as here.
 */
enum {
    ESP32P4_INTR_LP_RTC = 0,
    ESP32P4_INTR_LP_WDT = 1,
    ESP32P4_INTR_LP_TIMER_REG_0 = 2,
    ESP32P4_INTR_LP_TIMER_REG_1 = 3,
    ESP32P4_INTR_MB_HP = 4,
    ESP32P4_INTR_MB_LP = 5,
    ESP32P4_INTR_PMU_0 = 6,
    ESP32P4_INTR_PMU_1 = 7,
    ESP32P4_INTR_LP_ANAPERI = 8,
    ESP32P4_INTR_LP_ADC = 9,
    ESP32P4_INTR_LP_GPIO = 10,
    ESP32P4_INTR_LP_I2C = 11,
    ESP32P4_INTR_LP_I2S = 12,
    ESP32P4_INTR_LP_SPI = 13,
    ESP32P4_INTR_LP_TOUCH = 14,
    ESP32P4_INTR_LP_TSENS = 15,
    ESP32P4_INTR_LP_UART = 16,
    ESP32P4_INTR_LP_EFUSE = 17,
    ESP32P4_INTR_LP_SW = 18,
    ESP32P4_INTR_LP_SYSREG = 19,
    ESP32P4_INTR_LP_HUK = 20,
    ESP32P4_INTR_SYS_ICM = 21,
    ESP32P4_INTR_USB_SERIAL_JTAG = 22,
    ESP32P4_INTR_SDIO_HOST = 23,
    ESP32P4_INTR_DW_GDMA = 24,
    ESP32P4_INTR_SPI2 = 25,
    ESP32P4_INTR_SPI3 = 26,
    ESP32P4_INTR_I2S0 = 27,
    ESP32P4_INTR_I2S1 = 28,
    ESP32P4_INTR_I2S2 = 29,
    ESP32P4_INTR_UHCI0 = 30,
    ESP32P4_INTR_UART0 = 31,
    ESP32P4_INTR_UART1 = 32,
    ESP32P4_INTR_UART2 = 33,
    ESP32P4_INTR_UART3 = 34,
    ESP32P4_INTR_UART4 = 35,
    ESP32P4_INTR_LCD_CAM = 36,
    ESP32P4_INTR_ADC = 37,
    ESP32P4_INTR_PWM0 = 38,
    ESP32P4_INTR_PWM1 = 39,
    ESP32P4_INTR_TWAI0 = 40,
    ESP32P4_INTR_TWAI1 = 41,
    ESP32P4_INTR_TWAI2 = 42,
    ESP32P4_INTR_RMT = 43,
    ESP32P4_INTR_I2C0 = 44,
    ESP32P4_INTR_I2C1 = 45,
    ESP32P4_INTR_TG0_T0 = 46,
    ESP32P4_INTR_TG0_T1 = 47,
    ESP32P4_INTR_TG0_WDT = 48,
    ESP32P4_INTR_TG1_T0 = 49,
    ESP32P4_INTR_TG1_T1 = 50,
    ESP32P4_INTR_TG1_WDT = 51,
    ESP32P4_INTR_LEDC = 52,
    ESP32P4_INTR_SYSTIMER_TARGET0 = 53,
    ESP32P4_INTR_SYSTIMER_TARGET1 = 54,
    ESP32P4_INTR_SYSTIMER_TARGET2 = 55,
    ESP32P4_INTR_AHB_PDMA_IN_CH0 = 56,
    ESP32P4_INTR_AHB_PDMA_IN_CH1 = 57,
    ESP32P4_INTR_AHB_PDMA_IN_CH2 = 58,
    ESP32P4_INTR_AHB_PDMA_OUT_CH0 = 59,
    ESP32P4_INTR_AHB_PDMA_OUT_CH1 = 60,
    ESP32P4_INTR_AHB_PDMA_OUT_CH2 = 61,
    ESP32P4_INTR_AXI_PDMA_IN_CH0 = 62,
    ESP32P4_INTR_AXI_PDMA_IN_CH1 = 63,
    ESP32P4_INTR_AXI_PDMA_IN_CH2 = 64,
    ESP32P4_INTR_AXI_PDMA_OUT_CH0 = 65,
    ESP32P4_INTR_AXI_PDMA_OUT_CH1 = 66,
    ESP32P4_INTR_AXI_PDMA_OUT_CH2 = 67,
    ESP32P4_INTR_RSA = 68,
    ESP32P4_INTR_AES = 69,
    ESP32P4_INTR_SHA = 70,
    ESP32P4_INTR_ECC = 71,
    ESP32P4_INTR_ECDSA = 72,
    ESP32P4_INTR_KM = 73,
    ESP32P4_INTR_GPIO_INTR0 = 74,
    ESP32P4_INTR_GPIO_INTR1 = 75,
    ESP32P4_INTR_GPIO_INTR2 = 76,
    ESP32P4_INTR_GPIO_INTR3 = 77,
    ESP32P4_INTR_GPIO_PAD_COMP = 78,
    ESP32P4_INTR_FROM_CPU_INTR0 = 79,
    ESP32P4_INTR_FROM_CPU_INTR1 = 80,
    ESP32P4_INTR_FROM_CPU_INTR2 = 81,
    ESP32P4_INTR_FROM_CPU_INTR3 = 82,
    ESP32P4_INTR_CACHE = 83,
    ESP32P4_INTR_MSPI = 84,
    ESP32P4_INTR_CSI_BRIDGE = 85,
    ESP32P4_INTR_DSI_BRIDGE = 86,
    ESP32P4_INTR_CSI = 87,
    ESP32P4_INTR_DSI = 88,
    ESP32P4_INTR_GMII_PHY = 89,
    ESP32P4_INTR_LPI = 90,
    ESP32P4_INTR_PMT = 91,
    ESP32P4_INTR_ETH_MAC = 92,
    ESP32P4_INTR_USB_OTG = 93,
    ESP32P4_INTR_USB_OTG_ENDP_MULTI_PROC = 94,
    ESP32P4_INTR_JPEG = 95,
    ESP32P4_INTR_PPA = 96,
    ESP32P4_INTR_CORE0_TRACE = 97,
    ESP32P4_INTR_CORE1_TRACE = 98,
    ESP32P4_INTR_HP_CORE_CTRL = 99,
    ESP32P4_INTR_ISP = 100,
    ESP32P4_INTR_I3C_MST = 101,
    ESP32P4_INTR_I3C_SLV = 102,
    ESP32P4_INTR_USB_OTG11_CH0 = 103,
    ESP32P4_INTR_DMA2D_IN_CH0 = 104,
    ESP32P4_INTR_DMA2D_IN_CH1 = 105,
    ESP32P4_INTR_DMA2D_OUT_CH0 = 106,
    ESP32P4_INTR_DMA2D_OUT_CH1 = 107,
    ESP32P4_INTR_DMA2D_OUT_CH2 = 108,
    ESP32P4_INTR_PSRAM_MSPI = 109,
    ESP32P4_INTR_HP_SYSREG = 110,
    ESP32P4_INTR_PCNT = 111,
    ESP32P4_INTR_HP_PAU = 112,
    ESP32P4_INTR_HP_PARLIO_RX = 113,
    ESP32P4_INTR_HP_PARLIO_TX = 114,
    ESP32P4_INTR_H264_DMA2D_OUT_CH0 = 115,
    ESP32P4_INTR_H264_DMA2D_OUT_CH1 = 116,
    ESP32P4_INTR_H264_DMA2D_OUT_CH2 = 117,
    ESP32P4_INTR_H264_DMA2D_OUT_CH3 = 118,
    ESP32P4_INTR_H264_DMA2D_OUT_CH4 = 119,
    ESP32P4_INTR_H264_DMA2D_IN_CH0 = 120,
    ESP32P4_INTR_H264_DMA2D_IN_CH1 = 121,
    ESP32P4_INTR_H264_DMA2D_IN_CH2 = 122,
    ESP32P4_INTR_H264_DMA2D_IN_CH3 = 123,
    ESP32P4_INTR_H264_DMA2D_IN_CH4 = 124,
    ESP32P4_INTR_H264_DMA2D_IN_CH5 = 125,
    ESP32P4_INTR_H264_REG = 126,
    ESP32P4_INTR_ASSIST_DEBUG = 127,
    /* ESP-IDF: ETS_DMA2D_IN_CH2_INTR_SOURCE, 133. */
    ESP32P4_INTR_DMA2D_IN_CH2 = 128,
    /* ESP-IDF: ETS_DMA2D_OUT_CH3_INTR_SOURCE, 134. */
    ESP32P4_INTR_DMA2D_OUT_CH3 = 129,
    /* ESP-IDF: ETS_AXI_PERF_MON_INTR_SOURCE, 135. */
    ESP32P4_INTR_AXI_PERF_MON = 130,
    ESP32P4_INTMATRIX_NUM_SOURCES
};

#define ESP32P4_INTMATRIX_SOURCE_WORDS \
    DIV_ROUND_UP(ESP32P4_INTMATRIX_NUM_SOURCES, 32)

/*
 * How a peripheral's interrupt reaches the cores
 * ===============================================
 *
 * A peripheral's interrupt signal is a level, high while the peripheral
 * asserts it. Its model raises it on an output (a sysbus IRQ, or a named
 * GPIO output), and the machine connects that output to the matrix's
 * source input of its TRM table 13.4-1 number, ESP32P4_INTR_*, in one
 * line, as the eFuse controller's is:
 *
 *   sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
 *                      esp32p4_intr_source(s, ESP32P4_INTR_LP_EFUSE));
 *
 * (esp32p4_intr_source() in hw/riscv/esp32p4.c gives the input). The
 * matrix routes it to the CLIC interrupt of either HP core that software
 * maps it to, and the CLIC's attributes for that interrupt decide whether
 * it is taken on a level or an edge. A peripheral with more than one
 * signal, such as the GDMA's channels, connects each to its own source.
 *
 * Named GPIO inputs.
 *
 * ESP32P4_INTMATRIX_SOURCE: the sources, ESP32P4_INTMATRIX_NUM_SOURCES
 * lines indexed by ESP32P4_INTR_*, each feeding both cores' matrices.
 * ESP32P4_INTMATRIX_CORE_M_MODE: per HP core, high while the core
 * executes in machine mode (the core's ESP32P4_CPU_M_MODE output).
 * ESP32P4_INTMATRIX_LINE_USER: line core * ESP32P4_INTMATRIX_LINES + n,
 * high while the core's CLIC interrupt ESP32P4_INTMATRIX_LINE0 + n is a
 * user-mode one (the CLIC's ESP32P4_CLIC_USER output).
 * The last two decide the interrupt remapping of TRM section 13.4.4.
 *
 * Named GPIO outputs.
 *
 * ESP32P4_INTMATRIX_IRQ: line core * ESP32P4_INTMATRIX_LINES + n, to the
 * core's CLIC interrupt ESP32P4_INTMATRIX_LINE0 + n:
 *
 *   qdev_connect_gpio_out_named(intmatrix, ESP32P4_INTMATRIX_IRQ,
 *       core * ESP32P4_INTMATRIX_LINES + n,
 *       qdev_get_gpio_in_named(clic, ESP32P4_CLIC_IRQ,
 *           ESP32P4_CLIC_IRQ_LINE(core, ESP32P4_CLIC_IRQ_EXT0 + n)));
 *
 * ESP32P4_INTMATRIX_SOURCE_OUT: each source's level again, indexed by
 * ESP32P4_INTR_*, whatever the matrix's state: the fan-out of the sources
 * to the other blocks that take them on the chip, the LP core's interrupt
 * controller (the LP sources directly, and the HP ones as HP_INTR through
 * HP_SYSTEM_HP2LP_INTR_GROUP<n>_EN) and the PMU's wakeup logic.
 */
#define ESP32P4_INTMATRIX_SOURCE        "source"
#define ESP32P4_INTMATRIX_CORE_M_MODE   "core-m-mode"
#define ESP32P4_INTMATRIX_LINE_USER     "line-user"
#define ESP32P4_INTMATRIX_IRQ           "irq"
#define ESP32P4_INTMATRIX_SOURCE_OUT    "source-out"

typedef struct ESP32P4IntMatrixCore {
    /*
     * Per source, its mapping register's implemented bits: the CPU
     * interrupt it maps to (5:0), SRC_PASS_IN_SEC (6) and SRC_IN_SEC_FLAG
     * (7).
     */
    uint8_t map[ESP32P4_INTMATRIX_NUM_SOURCES];
    /* INTR_SIG_IDX_ASSERT_IN_SEC, the remapping's target. */
    uint8_t sig_idx;
    /* CLOCK_GATE_REG.REG_CLK_EN */
    bool clk_en;
    uint32_t date;
    /* The inputs: the core is in machine mode; its user-mode CLIC lines. */
    bool m_mode;
    uint32_t line_user;
    /* The levels on the outputs, and INTR_SEC_STATUS. */
    uint32_t out;
    uint32_t sec_status;
} ESP32P4IntMatrixCore;

struct ESP32P4IntMatrixState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* The sources' levels, a bit per source as the status registers. */
    uint32_t level[ESP32P4_INTMATRIX_SOURCE_WORDS];
    ESP32P4IntMatrixCore core[ESP32P4_INTMATRIX_CORES];
    qemu_irq irq[ESP32P4_INTMATRIX_CORES * ESP32P4_INTMATRIX_LINES];
    qemu_irq source_out[ESP32P4_INTMATRIX_NUM_SOURCES];
};

#endif
