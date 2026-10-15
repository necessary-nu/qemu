/*
 * QTest helpers for the ESP32-P4's PSRAM: its controller's ports set up
 * for the die's OPI DDR commands, as ESP-IDF sets them
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The in-package die takes only OPI DDR commands (a 16-bit command and a
 * 32-bit address on 8 lines, DDR), at the latencies its mode registers
 * give: from power-up a read latency of 5 clocks and a write latency of
 * 5, which dummy phases of 2 * (5 - 1) clock edges match, with its data
 * on 8 lines (X8).
 */

#ifndef ESP32P4_PSRAM_H
#define ESP32P4_PSRAM_H

#include "libqtest.h"

#define ESP32P4_PSRAM_SPI0          0x5008e000u
#define ESP32P4_PSRAM_SPI1          0x5008f000u
#define ESP32P4_PSRAM_PUP_DUMMY     (2 * (5 - 1))

/*
 * SPI1 in OPI DTR mode, as the boot ROM's esp_rom_spi_set_op_mode() sets
 * it (CTRL's FCMD_OCT, FADDR_OCT, FDIN_OCT and FDOUT_OCT, DDR's
 * FMEM_DDR_EN), on the die's chip select 1.
 */
static inline void esp32p4_psram_spi1_opi(QTestState *qts)
{
    uint32_t spi1 = ESP32P4_PSRAM_SPI1;

    qtest_writel(qts, spi1 + 0x008, qtest_readl(qts, spi1 + 0x008) | 0x270);
    qtest_writel(qts, spi1 + 0x0d4, qtest_readl(qts, spi1 + 0x0d4) | 1);
    qtest_writel(qts, spi1 + 0x034, 1);
}

/*
 * The strobes' inputs powered up, as ESP-IDF's
 * mspi_timing_ll_enable_dqs() does: a DDR read captures nothing without.
 */
static inline void esp32p4_psram_dqs_on(QTestState *qts)
{
    qtest_writel(qts, 0x500e123cu, qtest_readl(qts, 0x500e123cu) | 1);
    qtest_writel(qts, 0x500e1268u, qtest_readl(qts, 0x500e1268u) | 1);
}

/*
 * SPI0's window set up as ESP-IDF's s_config_mspi_for_psram() sets it:
 * synchronous read (0000h) and write (8080h), 32 address bits, the read
 * and write dummy phases `rd` and `wr` (edges), DDR, the command,
 * address and data on 8 lines, on 16 with `hex`; and its AXI port open
 * (AXI_REQ_EN set, CLOSE_AXI_INF_EN clear).
 */
static inline void esp32p4_psram_spi0_opi(QTestState *qts, bool hex,
                                          unsigned rd, unsigned wr)
{
    uint32_t spi0 = ESP32P4_PSRAM_SPI0;
    uint32_t sctrl = qtest_readl(qts, spi0 + 0x040) &
                     ~(0x3fu | 0x3fu << 6 | 0x3fu << 14 | 1u << 20 |
                       1u << 21 | 0x3fu << 22);

    qtest_writel(qts, spi0 + 0x04c, (15u << 28) | 0x8080);
    qtest_writel(qts, spi0 + 0x048, (15u << 28) | 0x0000);
    qtest_writel(qts, spi0 + 0x040, sctrl | 1u << 0 | 1u << 3 | 1u << 4 |
                 1u << 5 | (rd - 1) << 6 | 31u << 14 | 1u << 20 |
                 1u << 21 | (wr - 1) << 22);
    qtest_writel(qts, spi0 + 0x0d8, qtest_readl(qts, spi0 + 0x0d8) | 1);
    qtest_writel(qts, spi0 + 0x044, qtest_readl(qts, spi0 + 0x044) |
                 0xfu << 18 | (hex ? 3u << 26 : 0));
    qtest_writel(qts, spi0 + 0x03c,
                 (qtest_readl(qts, spi0 + 0x03c) & ~(1u << 31)) | 1);
}

#endif
