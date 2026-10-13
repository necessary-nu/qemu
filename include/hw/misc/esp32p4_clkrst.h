/*
 * ESP32-P4 reset and clock control: HP_SYS_CLKRST, LP_AON_CLKRST, LPPERI
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESP32P4_CLKRST_H
#define HW_MISC_ESP32P4_CLKRST_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/esp32p4_clk_period.h"
#include "qemu/queue.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_ESP32P4_CLKRST "esp32p4-clkrst"
OBJECT_DECLARE_SIMPLE_TYPE(ESP32P4ClkRstState, ESP32P4_CLKRST)

/*
 * The device's three register blocks, its MMIO regions in this order:
 * HP_SYS_CLKRST (0x500E6000), LP_AON_CLKRST (0x50111000) and LPPERI, the
 * LP peripherals' clock and reset (0x50120000), each 4 KiB.
 */
enum {
    ESP32P4_CLKRST_MMIO_HP,
    ESP32P4_CLKRST_MMIO_LP,
    ESP32P4_CLKRST_MMIO_LPPERI,
};
#define ESP32P4_CLKRST_BLOCK_SIZE 0x1000

/*
 * How a block takes its clocks and reset
 * ======================================
 *
 * Clocks. Every clock of the tree, from the oscillators through each mux,
 * divider and gate to the clock a block takes, is a named clock output of
 * this device, at its current frequency: 0 Hz while it is gated, while
 * its source is stopped, or while a mux selects an input the chip ties
 * low. A block's model declares a clock input for each clock the block
 * takes on the chip and the machine connects it to the output named
 * below, for example a UART's function clock:
 *
 *   qdev_connect_clock_in(DEVICE(uart), "sclk",
 *       qdev_get_clock_out(DEVICE(clkrst), "uart0-sclk"));
 *
 * The block times everything it does from that clock, at its current
 * frequency, and does nothing while it is stopped. The names are the
 * TRM's, lower case, without "_CLK":
 *
 *   - "<block>-cpu", "<block>-mem", "<block>-sys", "<block>-apb": the bus
 *     clocks HP_SYS_CLKRST_SOC_CLK_CTRL0..3 gate, CPU_CLK, MEM_CLK,
 *     SYS_CLK and APB_CLK, which clock a block's registers.
 *   - "<block>", "<block>-<function>": the function clocks of
 *     HP_SYS_CLKRST_PERI_CLK_CTRL*, each its source mux, gate and divider,
 *     e.g. "uart0-sclk", "i2c0", "timergrp0-t0", "gpspi2-mst".
 *   - "lp-<block>": the LP peripherals' bus clocks, LP_PERI_CLK as
 *     LPPERI_CLK_EN gates it, and their function clocks, e.g.
 *     "lp-uart-sclk"; but "lp-rng", RC_FAST_CLK as LPPERI_CK_EN_RNG gates
 *     it into the RNG as an entropy source (TRM 35.4), the RNG's registers
 *     being on "lp-peri".
 *   - The sources and SoC clocks, for blocks that take them directly:
 *     "xtal", "rc-fast", "rc-slow", "rc32k", "xtal32k", "pll-lp", "cpll",
 *     "spll", "mpll", "apll", "sdio-pll0..2" (as LP_CLKRST_HP_CLK_CTRL
 *     hands them to the HP system), the oscillators themselves
 *     ("osc-xtal", "osc-rc-fast", ...), "root", "cpu", "mem", "sys",
 *     "apb", the PLL_FxxM dividers ("pll-f80m", ...), "lp-slow",
 *     "lp-fast", "lp-dyn-slow", "lp-dyn-fast", "lp-peri" and "xtal-d2".
 *   - "efuse": the eFuse controller's 20 MHz, XTAL_D2_CLK as
 *     LPPERI_CK_EN_LP_EFUSE gates it (TRM table 11.2-7).
 *
 * esp32p4_clock_names[] lists them all, indexed by ESP32P4_CLK_*.
 *
 * Clocks from pads are clock inputs, "pad-<signal>-in" (unconnected, a
 * stopped clock), and the PLLs' outputs come in from the analog block as
 * "cpll-in", "spll-in", "mpll-in", "apll-in" and "sdio-pll-in". The tree
 * takes each input's period exactly from the device driving it (the
 * analog block, or the block whose clock a pad carries; see
 * esp32p4_clk_exact()), and every clock's period is computed exactly from
 * its sources' through all the dividers between, then rounded down once
 * for its Clock (see esp32p4_clk_period.h): a clock whose period is a
 * whole number of Clock units, such as the 6.25 ns of PLL_F160M_CLK, has
 * exactly that period, and any other is less than a unit short of its
 * own. The device gives every output's exact period through
 * esp32p4_clk_exact(), and a block that counts a clock's cycles takes it
 * from there.
 *
 * Resets. Each block reset of HP_SYS_CLKRST_HP_RST_EN*, LP_CLKRST and
 * LPPERI_RESET_EN is a line of the named GPIO output ESP32P4_CLKRST_RESET,
 * indexed by ESP32P4_RST_*: high while the block is held in reset, its
 * RST_EN bit set and, where the block has one, its FORCE_NORST bit clear.
 *
 * Gating. A block whose bus clock is gated or which is held in reset
 * does not operate. esp32p4_clkrst_map_gated() maps a block's registers
 * behind its bus clock and reset, so that they behave as gated hardware
 * does, and resets the block as its reset line rises and again as it
 * falls:
 *
 *   esp32p4_clkrst_map_gated(clkrst, SYS_BUS_DEVICE(uart), 0, 0x500ca000,
 *                            ESP32P4_CLK_UART0_APB, ESP32P4_RST_UART0_APB,
 *                            0);
 *
 * A block whose reset does more than a device reset, or which has more
 * than one, takes its lines itself, connecting a reset input to each,
 * and is mapped behind its bus clock alone (ESP32P4_CLKRST_GATE_NO_RESET),
 * as the eFuse controller is.
 *
 * The HP cores take their reset and clock enable from the "core-reset"
 * and "core-clock" outputs and execute at "core0-cpu" and "core1-cpu";
 * the LP core takes "lp-core-reset" and "lp-core-clock" and executes at
 * "lp-core".
 */

/* The clock inputs, indexed by ESP32P4_CLKIN_*. */
#define ESP32P4_CLKRST_INPUTS(X) \
    X(CPLL_IN, "cpll-in") \
    X(SPLL_IN, "spll-in") \
    X(MPLL_IN, "mpll-in") \
    X(APLL_IN, "apll-in") \
    X(SDIO_PLL_IN, "sdio-pll-in") \
    X(PAD_OSC_SLOW_IN, "pad-osc-slow-in") \
    X(PAD_EMAC_TXRX_IN, "pad-emac-txrx-in") \
    X(PAD_EMAC_RX_IN, "pad-emac-rx-in") \
    X(PAD_EMAC_TX_IN, "pad-emac-tx-in") \
    X(PAD_I2S0_MCLK_IN, "pad-i2s0-mclk-in") \
    X(PAD_I2S1_MCLK_IN, "pad-i2s1-mclk-in") \
    X(PAD_I2S2_MCLK_IN, "pad-i2s2-mclk-in") \
    X(PAD_UART0_SLP_IN, "pad-uart0-slp-in") \
    X(PAD_UART1_SLP_IN, "pad-uart1-slp-in") \
    X(PAD_UART2_SLP_IN, "pad-uart2-slp-in") \
    X(PAD_UART3_SLP_IN, "pad-uart3-slp-in") \
    X(PAD_UART4_SLP_IN, "pad-uart4-slp-in") \
    X(PAD_PARLIO_RX_IN, "pad-parlio-rx-in") \
    X(PAD_PARLIO_TX_IN, "pad-parlio-tx-in") \
    X(PAD_USB_HSPHY_REFCLK_IN, "pad-usb-hsphy-refclk-in") \
    X(DSI_DPHY_IN, "dsi-dphy-in")

/*
 * The clock outputs, indexed by ESP32P4_CLK_*, each derived only from
 * clocks before it.
 */
#define ESP32P4_CLKRST_CLOCKS(X) \
    /* The oscillators and LP PLL, as the LP system has them. */ \
    X(OSC_XTAL, "osc-xtal") \
    X(OSC_RC_FAST, "osc-rc-fast") \
    X(OSC_RC_SLOW, "osc-rc-slow") \
    X(OSC_RC32K, "osc-rc32k") \
    X(OSC_XTAL32K, "osc-xtal32k") \
    X(OSC_SLOW, "osc-slow") \
    X(OSC_PLL_LP, "osc-pll-lp") \
    /* The sources as LP_CLKRST_HP_CLK_CTRL hands them to the HP system. */ \
    X(XTAL, "xtal") \
    X(RC_FAST, "rc-fast") \
    X(RC_SLOW, "rc-slow") \
    X(RC32K, "rc32k") \
    X(XTAL32K, "xtal32k") \
    X(PLL_LP, "pll-lp") \
    X(CPLL, "cpll") \
    X(SPLL, "spll") \
    X(MPLL, "mpll") \
    X(APLL, "apll") \
    X(SDIO_PLL0, "sdio-pll0") \
    X(SDIO_PLL1, "sdio-pll1") \
    X(SDIO_PLL2, "sdio-pll2") \
    X(PAD_EMAC_TXRX, "pad-emac-txrx") \
    X(PAD_EMAC_RX, "pad-emac-rx") \
    X(PAD_EMAC_TX, "pad-emac-tx") \
    X(PAD_I2S0_MCLK, "pad-i2s0-mclk") \
    X(PAD_I2S1_MCLK, "pad-i2s1-mclk") \
    X(PAD_I2S2_MCLK, "pad-i2s2-mclk") \
    X(PAD_UART0_SLP, "pad-uart0-slp") \
    X(PAD_UART1_SLP, "pad-uart1-slp") \
    X(PAD_UART2_SLP, "pad-uart2-slp") \
    X(PAD_UART3_SLP, "pad-uart3-slp") \
    X(PAD_UART4_SLP, "pad-uart4-slp") \
    X(PAD_PARLIO_RX, "pad-parlio-rx") \
    X(PAD_PARLIO_TX, "pad-parlio-tx") \
    /* The SoC clocks. */ \
    X(ROOT, "root") \
    X(CPU, "cpu") \
    X(MEM, "mem") \
    X(SYS, "sys") \
    X(APB, "apb") \
    /* The PLLs' fixed dividers. */ \
    X(PLL_F240M, "pll-f240m") \
    X(PLL_F160M, "pll-f160m") \
    X(PLL_F120M, "pll-f120m") \
    X(PLL_F80M, "pll-f80m") \
    X(PLL_F20M, "pll-f20m") \
    X(PLL_F50M, "pll-f50m") \
    X(PLL_F25M, "pll-f25m") \
    X(PLL_F48M, "pll-f48m") \
    X(PLL_F12M, "pll-f12m") \
    X(USB_PLL_F25M, "usb-pll-f25m") \
    /* The LP system clocks. */ \
    X(LP_SLOW, "lp-slow") \
    X(LP_FAST, "lp-fast") \
    X(LP_DYN_SLOW, "lp-dyn-slow") \
    X(LP_DYN_FAST, "lp-dyn-fast") \
    X(LP_PERI, "lp-peri") \
    X(XTAL_D2, "xtal-d2") \
    /* Bus clocks: HP_SYS_CLKRST_SOC_CLK_CTRL0. */ \
    X(CORE0_CLIC, "core0-clic") \
    X(CORE1_CLIC, "core1-clic") \
    X(MISC_CPU, "misc-cpu") \
    X(CORE0_CPU, "core0-cpu") \
    X(CORE1_CPU, "core1-cpu") \
    X(TCM_CPU, "tcm-cpu") \
    X(BUSMON_CPU, "busmon-cpu") \
    X(L1CACHE_CPU, "l1cache-cpu") \
    X(L1CACHE_D_CPU, "l1cache-d-cpu") \
    X(L1CACHE_I0_CPU, "l1cache-i0-cpu") \
    X(L1CACHE_I1_CPU, "l1cache-i1-cpu") \
    X(TRACE_CPU, "trace-cpu") \
    X(ICM_CPU, "icm-cpu") \
    X(GDMA_CPU, "gdma-cpu") \
    X(VPU_CPU, "vpu-cpu") \
    X(L1CACHE_MEM, "l1cache-mem") \
    X(L1CACHE_D_MEM, "l1cache-d-mem") \
    X(L1CACHE_I0_MEM, "l1cache-i0-mem") \
    X(L1CACHE_I1_MEM, "l1cache-i1-mem") \
    X(L2CACHE_MEM, "l2cache-mem") \
    X(L2MEM_MEM, "l2mem-mem") \
    X(L2MEMMON_MEM, "l2memmon-mem") \
    X(ICM_MEM, "icm-mem") \
    X(MISC_SYS, "misc-sys") \
    X(TRACE_SYS, "trace-sys") \
    X(L2CACHE_SYS, "l2cache-sys") \
    X(L2MEM_SYS, "l2mem-sys") \
    X(L2MEMMON_SYS, "l2memmon-sys") \
    X(TCMMON_SYS, "tcmmon-sys") \
    X(ICM_SYS, "icm-sys") \
    X(FLASH_SYS, "flash-sys") \
    X(PSRAM_SYS, "psram-sys") \
    /* Bus clocks: HP_SYS_CLKRST_SOC_CLK_CTRL1. */ \
    X(GPSPI2_SYS, "gpspi2-sys") \
    X(GPSPI3_SYS, "gpspi3-sys") \
    X(REGDMA_SYS, "regdma-sys") \
    X(AHB_PDMA_SYS, "ahb-pdma-sys") \
    X(AXI_PDMA_SYS, "axi-pdma-sys") \
    X(GDMA_SYS, "gdma-sys") \
    X(DMA2D_SYS, "dma2d-sys") \
    X(VPU_SYS, "vpu-sys") \
    X(JPEG_SYS, "jpeg-sys") \
    X(PPA_SYS, "ppa-sys") \
    X(CSI_BRG_SYS, "csi-brg-sys") \
    X(CSI_HOST_SYS, "csi-host-sys") \
    X(DSI_SYS, "dsi-sys") \
    X(EMAC_SYS, "emac-sys") \
    X(SDMMC_SYS, "sdmmc-sys") \
    X(USB_OTG11_SYS, "usb-otg11-sys") \
    X(USB_OTG20_SYS, "usb-otg20-sys") \
    X(UHCI_SYS, "uhci-sys") \
    X(UART0_SYS, "uart0-sys") \
    X(UART1_SYS, "uart1-sys") \
    X(UART2_SYS, "uart2-sys") \
    X(UART3_SYS, "uart3-sys") \
    X(UART4_SYS, "uart4-sys") \
    X(PARLIO_SYS, "parlio-sys") \
    X(ETM_SYS, "etm-sys") \
    X(PVT_SYS, "pvt-sys") \
    X(CRYPTO_SYS, "crypto-sys") \
    X(KEY_MANAGER_SYS, "key-manager-sys") \
    X(BITSCRAMBLER_SYS, "bitscrambler-sys") \
    X(BITSCRAMBLER_RX_SYS, "bitscrambler-rx-sys") \
    X(BITSCRAMBLER_TX_SYS, "bitscrambler-tx-sys") \
    X(H264_SYS, "h264-sys") \
    /* Bus clocks: HP_SYS_CLKRST_SOC_CLK_CTRL2. */ \
    X(RMT_SYS, "rmt-sys") \
    X(HP_CLKRST_APB, "hp-clkrst-apb") \
    X(SYSREG_APB, "sysreg-apb") \
    X(ICM_APB, "icm-apb") \
    X(INTRMTX_APB, "intrmtx-apb") \
    X(ADC_APB, "adc-apb") \
    X(UHCI_APB, "uhci-apb") \
    X(UART0_APB, "uart0-apb") \
    X(UART1_APB, "uart1-apb") \
    X(UART2_APB, "uart2-apb") \
    X(UART3_APB, "uart3-apb") \
    X(UART4_APB, "uart4-apb") \
    X(I2C0_APB, "i2c0-apb") \
    X(I2C1_APB, "i2c1-apb") \
    X(I2S0_APB, "i2s0-apb") \
    X(I2S1_APB, "i2s1-apb") \
    X(I2S2_APB, "i2s2-apb") \
    X(I3C_MST_APB, "i3c-mst-apb") \
    X(I3C_SLV_APB, "i3c-slv-apb") \
    X(GPSPI2_APB, "gpspi2-apb") \
    X(GPSPI3_APB, "gpspi3-apb") \
    X(TIMERGRP0_APB, "timergrp0-apb") \
    X(TIMERGRP1_APB, "timergrp1-apb") \
    X(SYSTIMER_APB, "systimer-apb") \
    X(TWAI0_APB, "twai0-apb") \
    X(TWAI1_APB, "twai1-apb") \
    X(TWAI2_APB, "twai2-apb") \
    X(MCPWM0_APB, "mcpwm0-apb") \
    X(MCPWM1_APB, "mcpwm1-apb") \
    X(USB_DEVICE_APB, "usb-device-apb") \
    X(PCNT_APB, "pcnt-apb") \
    X(PARLIO_APB, "parlio-apb") \
    /* Bus clocks: HP_SYS_CLKRST_SOC_CLK_CTRL3 and the AXI monitor. */ \
    X(LEDC_APB, "ledc-apb") \
    X(LCDCAM_APB, "lcdcam-apb") \
    X(ETM_APB, "etm-apb") \
    X(IOMUX_APB, "iomux-apb") \
    X(L2CACHE_L2MEM, "l2cache-l2mem") \
    X(AXI_PERF_MON_SYS, "axi-perf-mon-sys") \
    /* Function clocks: HP_SYS_CLKRST_PERI_CLK_CTRL*. */ \
    X(FLASH, "flash") \
    X(FLASH_PLL, "flash-pll") \
    X(FLASH_CORE, "flash-core") \
    X(PSRAM, "psram") \
    X(PSRAM_PLL, "psram-pll") \
    X(PSRAM_CORE, "psram-core") \
    X(PAD_EMAC_REF, "pad-emac-ref") \
    X(EMAC_RMII, "emac-rmii") \
    X(EMAC_RX, "emac-rx") \
    X(EMAC_TX, "emac-tx") \
    X(EMAC_PTP_REF, "emac-ptp-ref") \
    X(SDIO_LS, "sdio-ls") \
    X(SDIO_SLF, "sdio-slf") \
    X(SDIO_DRV, "sdio-drv") \
    X(SDIO_SAM, "sdio-sam") \
    X(MIPI_DSI_DPHY_CFG, "mipi-dsi-dphy-cfg") \
    X(MIPI_DSI_DPHY_PLLREF, "mipi-dsi-dphy-pllref") \
    X(MIPI_CSI_DPHY_CFG, "mipi-csi-dphy-cfg") \
    X(MIPI_DSI_DPI, "mipi-dsi-dpi") \
    X(I2C0, "i2c0") \
    X(I2C1, "i2c1") \
    X(I2S0_RX, "i2s0-rx") \
    X(I2S0_TX, "i2s0-tx") \
    X(I2S0_MST, "i2s0-mst") \
    X(I2S1_RX, "i2s1-rx") \
    X(I2S1_TX, "i2s1-tx") \
    X(I2S1_MST, "i2s1-mst") \
    X(I2S2_RX, "i2s2-rx") \
    X(I2S2_TX, "i2s2-tx") \
    X(I2S2_MST, "i2s2-mst") \
    X(LCD, "lcd") \
    X(UART0_SCLK, "uart0-sclk") \
    X(UART1_SCLK, "uart1-sclk") \
    X(UART2_SCLK, "uart2-sclk") \
    X(UART3_SCLK, "uart3-sclk") \
    X(UART4_SCLK, "uart4-sclk") \
    X(TWAI0, "twai0") \
    X(TWAI1, "twai1") \
    X(TWAI2, "twai2") \
    X(GPSPI2, "gpspi2") \
    X(GPSPI2_HS, "gpspi2-hs") \
    X(GPSPI2_MST, "gpspi2-mst") \
    X(GPSPI3, "gpspi3") \
    X(GPSPI3_HS, "gpspi3-hs") \
    X(GPSPI3_MST, "gpspi3-mst") \
    X(PARLIO_RX, "parlio-rx") \
    X(PARLIO_TX, "parlio-tx") \
    X(I3C_MST, "i3c-mst") \
    X(CAM, "cam") \
    X(MCPWM0, "mcpwm0") \
    X(MCPWM1, "mcpwm1") \
    X(TIMERGRP0_T0, "timergrp0-t0") \
    X(TIMERGRP0_T1, "timergrp0-t1") \
    X(TIMERGRP0_WDT, "timergrp0-wdt") \
    X(TIMERGRP0_TGRT, "timergrp0-tgrt") \
    X(TIMERGRP1_T0, "timergrp1-t0") \
    X(TIMERGRP1_T1, "timergrp1-t1") \
    X(TIMERGRP1_WDT, "timergrp1-wdt") \
    X(SYSTIMER, "systimer") \
    X(LEDC, "ledc") \
    X(RMT, "rmt") \
    X(ADC, "adc") \
    X(ADC_SAR1, "adc-sar1") \
    X(ADC_SAR2, "adc-sar2") \
    X(PVT, "pvt") \
    X(PVT_PERI_GROUP, "pvt-peri-group") \
    X(PVT_PERI_GROUP1, "pvt-peri-group1") \
    X(PVT_PERI_GROUP2, "pvt-peri-group2") \
    X(PVT_PERI_GROUP3, "pvt-peri-group3") \
    X(PVT_PERI_GROUP4, "pvt-peri-group4") \
    X(CRYPTO, "crypto") \
    X(CRYPTO_AES, "crypto-aes") \
    X(CRYPTO_DS, "crypto-ds") \
    X(CRYPTO_ECC, "crypto-ecc") \
    X(CRYPTO_HMAC, "crypto-hmac") \
    X(CRYPTO_RSA, "crypto-rsa") \
    X(CRYPTO_SEC, "crypto-sec") \
    X(CRYPTO_SHA, "crypto-sha") \
    X(CRYPTO_ECDSA, "crypto-ecdsa") \
    X(CRYPTO_KM, "crypto-km") \
    X(ISP, "isp") \
    X(IOMUX, "iomux") \
    X(H264, "h264") \
    X(CLKOUT0, "clkout0") \
    X(CLKOUT1, "clkout1") \
    X(CLKOUT2, "clkout2") \
    /* LP_CLKRST's USB clocks and the LP memory. */ \
    X(USB_OTG11_48M, "usb-otg11-48m") \
    X(USB_DEVICE_48M, "usb-device-48m") \
    X(USB_OTG20_PHYREF, "usb-otg20-phyref") \
    X(LP_RAM, "lp-ram") \
    /* The LP peripherals' clocks: LPPERI. */ \
    X(LP_RNG, "lp-rng") \
    X(LP_TSENS, "lp-tsens") \
    X(LP_PMS, "lp-pms") \
    X(LP_EFUSE, "lp-efuse") \
    X(LP_IOMUX, "lp-iomux") \
    X(LP_TOUCH, "lp-touch") \
    X(LP_SPI, "lp-spi") \
    X(LP_ADC, "lp-adc") \
    X(LP_I2S, "lp-i2s") \
    X(LP_I2CMST, "lp-i2cmst") \
    X(LP_I2C, "lp-i2c") \
    X(LP_UART, "lp-uart") \
    X(LP_INTR, "lp-intr") \
    X(LP_I2S_RX, "lp-i2s-rx") \
    X(LP_I2S_TX, "lp-i2s-tx") \
    X(LP_I2C_SCLK, "lp-i2c-sclk") \
    X(LP_UART_SCLK, "lp-uart-sclk") \
    X(LP_ADC_FUNC, "lp-adc-func") \
    X(LP_ADC_SAR1, "lp-adc-sar1") \
    X(LP_ADC_SAR2, "lp-adc-sar2") \
    X(EFUSE, "efuse") \
    X(LP_CORE, "lp-core")

#define ESP32P4_CLKRST_ENUM_IN(id, name) ESP32P4_CLKIN_##id,
enum { ESP32P4_CLKRST_INPUTS(ESP32P4_CLKRST_ENUM_IN) ESP32P4_NUM_CLKIN };
#undef ESP32P4_CLKRST_ENUM_IN

#define ESP32P4_CLKRST_ENUM_CLK(id, name) ESP32P4_CLK_##id,
enum { ESP32P4_CLKRST_CLOCKS(ESP32P4_CLKRST_ENUM_CLK) ESP32P4_NUM_CLOCKS };
#undef ESP32P4_CLKRST_ENUM_CLK

extern const char *const esp32p4_clock_input_names[ESP32P4_NUM_CLKIN];
extern const char *const esp32p4_clock_names[ESP32P4_NUM_CLOCKS];

/*
 * The block resets, lines of the ESP32P4_CLKRST_RESET output: the
 * HP_SYS_CLKRST_HP_RST_EN0..2 bits in register order (but for the HP
 * cores', which are "core-reset"), the AXI performance monitor's, LP_CLKRST's
 * and LPPERI's.
 */
#define ESP32P4_CLKRST_RESETS(X) \
    X(CORECTRL, "corectrl") \
    X(PVT_TOP, "pvt-top") \
    X(PVT_PERI_GROUP1, "pvt-peri-group1") \
    X(PVT_PERI_GROUP2, "pvt-peri-group2") \
    X(PVT_PERI_GROUP3, "pvt-peri-group3") \
    X(PVT_PERI_GROUP4, "pvt-peri-group4") \
    X(REGDMA, "regdma") \
    X(CORETRACE0, "coretrace0") \
    X(CORETRACE1, "coretrace1") \
    X(HP_TCM, "hp-tcm") \
    X(HP_CACHE, "hp-cache") \
    X(L1_I0_CACHE, "l1-i0-cache") \
    X(L1_I1_CACHE, "l1-i1-cache") \
    X(L1_D_CACHE, "l1-d-cache") \
    X(L2_CACHE, "l2-cache") \
    X(L2_MEM, "l2-mem") \
    X(L2MEMMON, "l2memmon") \
    X(TCMMON, "tcmmon") \
    X(PVT_APB, "pvt-apb") \
    X(GDMA, "gdma") \
    X(MSPI_AXI, "mspi-axi") \
    X(DUAL_MSPI_AXI, "dual-mspi-axi") \
    X(MSPI_APB, "mspi-apb") \
    X(DUAL_MSPI_APB, "dual-mspi-apb") \
    X(DSI_BRG, "dsi-brg") \
    X(CSI_HOST, "csi-host") \
    X(CSI_BRG, "csi-brg") \
    X(ISP, "isp") \
    X(JPEG, "jpeg") \
    X(DMA2D, "dma2d") \
    X(PPA, "ppa") \
    X(AHB_PDMA, "ahb-pdma") \
    X(AXI_PDMA, "axi-pdma") \
    X(IOMUX, "iomux") \
    X(PADBIST, "padbist") \
    X(STIMER, "stimer") \
    X(TIMERGRP0, "timergrp0") \
    X(TIMERGRP1, "timergrp1") \
    X(UART0_CORE, "uart0-core") \
    X(UART1_CORE, "uart1-core") \
    X(UART2_CORE, "uart2-core") \
    X(UART3_CORE, "uart3-core") \
    X(UART4_CORE, "uart4-core") \
    X(UART0_APB, "uart0-apb") \
    X(UART1_APB, "uart1-apb") \
    X(UART2_APB, "uart2-apb") \
    X(UART3_APB, "uart3-apb") \
    X(UART4_APB, "uart4-apb") \
    X(UHCI, "uhci") \
    X(I3CMST, "i3cmst") \
    X(I3CSLV, "i3cslv") \
    X(I2C1, "i2c1") \
    X(I2C0, "i2c0") \
    X(RMT, "rmt") \
    X(PWM0, "pwm0") \
    X(PWM1, "pwm1") \
    X(CAN0, "can0") \
    X(CAN1, "can1") \
    X(CAN2, "can2") \
    X(LEDC, "ledc") \
    X(PCNT, "pcnt") \
    X(ETM, "etm") \
    X(INTRMTX, "intrmtx") \
    X(PARLIO, "parlio") \
    X(PARLIO_RX, "parlio-rx") \
    X(PARLIO_TX, "parlio-tx") \
    X(I2S0_APB, "i2s0-apb") \
    X(I2S1_APB, "i2s1-apb") \
    X(I2S2_APB, "i2s2-apb") \
    X(SPI2, "spi2") \
    X(SPI3, "spi3") \
    X(LCDCAM, "lcdcam") \
    X(ADC, "adc") \
    X(BITSCRAMBLER, "bitscrambler") \
    X(BITSCRAMBLER_RX, "bitscrambler-rx") \
    X(BITSCRAMBLER_TX, "bitscrambler-tx") \
    X(CRYPTO, "crypto") \
    X(SEC, "sec") \
    X(AES, "aes") \
    X(DS, "ds") \
    X(SHA, "sha") \
    X(HMAC, "hmac") \
    X(ECDSA, "ecdsa") \
    X(RSA, "rsa") \
    X(ECC, "ecc") \
    X(KM, "km") \
    X(H264, "h264") \
    X(AXI_PERF_MON, "axi-perf-mon") \
    X(LP_HUK, "lp-huk") \
    X(LP_ANAPERI, "lp-anaperi") \
    X(LP_WDT, "lp-wdt") \
    X(LP_TIMER, "lp-timer") \
    X(LP_RTC, "lp-rtc") \
    X(LP_MAILBOX, "lp-mailbox") \
    X(LP_AONEFUSEREG, "lp-aonefusereg") \
    X(LP_RAM, "lp-ram") \
    X(USB_OTG20_ADP, "usb-otg20-adp") \
    X(USB_OTG20_PHY, "usb-otg20-phy") \
    X(USB_OTG20, "usb-otg20") \
    X(USB_OTG11, "usb-otg11") \
    X(USB_DEVICE, "usb-device") \
    X(SDMMC, "sdmmc") \
    X(EMAC, "emac") \
    X(LP_RNG, "lp-rng") \
    X(LP_TSENS, "lp-tsens") \
    X(LP_PMS, "lp-pms") \
    X(LP_EFUSE, "lp-efuse") \
    X(LP_IOMUX, "lp-iomux") \
    X(LP_TOUCH, "lp-touch") \
    X(LP_SPI, "lp-spi") \
    X(LP_ADC, "lp-adc") \
    X(LP_I2S, "lp-i2s") \
    X(LP_I2CMST, "lp-i2cmst") \
    X(LP_I2C, "lp-i2c") \
    X(LP_UART, "lp-uart") \
    X(LP_INTR, "lp-intr") \
    X(LP_ROM, "lp-rom")

#define ESP32P4_CLKRST_ENUM_RST(id, name) ESP32P4_RST_##id,
enum { ESP32P4_CLKRST_RESETS(ESP32P4_CLKRST_ENUM_RST) ESP32P4_NUM_RESETS };
#undef ESP32P4_CLKRST_ENUM_RST

extern const char *const esp32p4_reset_names[ESP32P4_NUM_RESETS];

#define ESP32P4_CLKRST_HP_CORES 2

/*
 * Named GPIO outputs.
 *
 * ESP32P4_CLKRST_RESET: the block resets, ESP32P4_NUM_RESETS lines.
 * "core-reset", "core-clock": per HP core, its reset (high while held:
 *   HP_SYS_CLKRST_RST_EN_CORE<n>_GLOBAL, or a software or watchdog CPU
 *   reset in progress) and its clock enable (high while it runs:
 *   CORE<n>_CPU_CLK_EN set, CPU_CLK running, not stalled).
 * "core-vector": per HP core, LP_CLKRST_HPCORE<n>_STAT_VECTOR_SEL: high
 *   for the reset vector in the HP ROM, low for LP SRAM's start.
 * "lp-core-reset", "lp-core-clock": the LP core's reset and clock enable.
 * "pll-cal-stop": per PLL (ESP32P4_ANA_PLL_*), HP_SYS_CLKRST_ANA_PLL_CTRL0's
 *   *_CAL_STOP, to the analog block.
 * "core-stall": per HP core, high while it is stalled: by the PMU, or in
 *   the stall that precedes a CPU reset with STALL_EN set. The system
 *   timer's counters stop while a core they follow is stalled.
 * "hp-core-reset": high while an HP core reset is held (see
 *   esp32p4_clkrst_hp_core_reset()), for the HP blocks that have no reset
 *   line of their own and for the RTC timer, which logs the time of it.
 */
#define ESP32P4_CLKRST_RESET          "reset"
#define ESP32P4_CLKRST_CORE_RESET     "core-reset"
#define ESP32P4_CLKRST_CORE_CLOCK     "core-clock"
#define ESP32P4_CLKRST_CORE_VECTOR    "core-vector"
#define ESP32P4_CLKRST_LP_CORE_RESET  "lp-core-reset"
#define ESP32P4_CLKRST_LP_CORE_CLOCK  "lp-core-clock"
#define ESP32P4_CLKRST_PLL_CAL_STOP   "pll-cal-stop"
#define ESP32P4_CLKRST_CORE_STALL     "core-stall"
#define ESP32P4_CLKRST_HP_CORE_RESET  "hp-core-reset"

/*
 * Named GPIO inputs.
 *
 * "pll-cal-end": per PLL, its calibration done, from the analog block.
 * The rest are the hooks of the blocks that drive this one's inputs on the
 * chip, which keep the level given here until those blocks drive them:
 * "pmu-xpd": per ESP32P4_CLKRST_XPD_*, the PMU's power-up of an
 *   oscillator: RC_FAST and the crystal (powered), RC32K, XTAL32K and the
 *   LP PLL (not). A powered-down oscillator is stopped.
 * "pmu-core-stall": per HP core, the PMU's stall of it (not stalled).
 * "pmu-lp-core-reset", "pmu-lp-core-clock": the PMU's hold of the LP core
 *   in reset (held) and its LP CPU clock enable (off).
 * "pmu-lp-core-stall": the PMU's stall of the LP core (not stalled),
 *   which stops it even while LPPERI_CK_EN_LP_CORE forces its clock on.
 * "pmu-power": per ESP32P4_CLKRST_PD_*, high while the PMU has the power
 *   domain powered (all are). A powered-down domain's blocks are held in
 *   reset: PD_TOP's, the whole HP system as an HP core reset holds it,
 *   HP_SYS_CLKRST's registers taking their reset values; PD_HP_CPU's, the
 *   HP cores; PD_HP_CNNT's, the USB OTG controllers, SD/MMC and EMAC; and
 *   PD_LP_PERI's, the LP core and the LP peripherals LPPERI resets, with
 *   LPPERI's registers. The power-up releases them, recording PMU HP power
 *   down reset (0x05) for the HP cores or PMU LP peripheral power down
 *   reset (0x09) for the LP core, unless this device is itself in reset,
 *   when the reset records its own cause.
 * "pmu-icg": per bit of PMU_n1_DIG_ICG_FUNC_EN (TRM table 15.4-2), low
 *   while the PMU gates that clock (none is).
 * "pmu-sysclk": per ESP32P4_CLKRST_SYSCLK_*, the PMU's control of
 *   ROOT_CLK: whether it selects ROOT_CLK's source, the source it selects
 *   (in LP_CLKRST_HP_ROOT_CLK_SRC_SEL's encoding) and whether it gates
 *   ROOT_CLK (it does none of these).
 * "pmu-lp-sleep": high while the PMU has the chip in LP_SLEEP, when
 *   LP_DYN_FAST_CLK follows LP_SLOW_CLK rather than LP_FAST_CLK (TRM
 *   15.4.2.5).
 * "mwdt-core-reset": ESP32P4_CLKRST_MWDT_LINE(m, n), a pulse of MWDT m's
 *   (timer group m's) CPU reset of HP core n, which its
 *   TIMG_WDT_PROCPU_RESET_EN (core 0) or APPCPU_RESET_EN (core 1) lets
 *   out; it resets core n if HPCORE<n>_WDT_RESET_SOURCE_SEL picks MWDT m.
 * "rwdt-core-reset": per HP core, a pulse of the RTC watchdog's CPU reset
 *   of the core, which LP_WDT_PROCPU_RESET_EN (core 0) or APPCPU_RESET_EN
 *   (core 1) lets out; it resets the core if LP_WDT_HPCORE<n>_RESET_EN is
 *   set.
 * "lp-uart-wakeup": a pulse of the LP UART's wakeup, which sets
 *   LPPERI_LP_UART_WAKEUP_FLAG while LP_UART_WAKEUP_EN is set.
 */
#define ESP32P4_CLKRST_PLL_CAL_END      "pll-cal-end"
#define ESP32P4_CLKRST_PMU_XPD          "pmu-xpd"
#define ESP32P4_CLKRST_PMU_CORE_STALL   "pmu-core-stall"
#define ESP32P4_CLKRST_PMU_LP_CORE_RESET "pmu-lp-core-reset"
#define ESP32P4_CLKRST_PMU_LP_CORE_CLOCK "pmu-lp-core-clock"
#define ESP32P4_CLKRST_PMU_LP_CORE_STALL "pmu-lp-core-stall"
#define ESP32P4_CLKRST_PMU_POWER        "pmu-power"
#define ESP32P4_CLKRST_PMU_ICG          "pmu-icg"
#define ESP32P4_CLKRST_PMU_SYSCLK       "pmu-sysclk"
#define ESP32P4_CLKRST_PMU_LP_SLEEP     "pmu-lp-sleep"
#define ESP32P4_CLKRST_MWDT_CORE_RESET  "mwdt-core-reset"
#define ESP32P4_CLKRST_RWDT_CORE_RESET  "rwdt-core-reset"
#define ESP32P4_CLKRST_LP_UART_WAKEUP   "lp-uart-wakeup"

#define ESP32P4_CLKRST_MWDTS            2
#define ESP32P4_CLKRST_MWDT_LINE(m, n)  ((m) * ESP32P4_CLKRST_HP_CORES + (n))

enum {
    ESP32P4_CLKRST_XPD_RC_FAST,
    ESP32P4_CLKRST_XPD_RC32K,
    ESP32P4_CLKRST_XPD_XTAL32K,
    ESP32P4_CLKRST_XPD_LP_PLL,
    ESP32P4_CLKRST_XPD_XTAL,
    ESP32P4_CLKRST_NUM_XPD,
};

/* The power domains of "pmu-power". */
enum {
    ESP32P4_CLKRST_PD_TOP,
    ESP32P4_CLKRST_PD_HP_CPU,
    ESP32P4_CLKRST_PD_HP_CNNT,
    ESP32P4_CLKRST_PD_LP_PERI,
    ESP32P4_CLKRST_NUM_PD,
};

/* The clocks PMU_n1_DIG_ICG_FUNC_EN gates, lines of "pmu-icg". */
#define ESP32P4_CLKRST_NUM_PMU_ICG 27

/* The lines of "pmu-sysclk". */
enum {
    ESP32P4_CLKRST_SYSCLK_PMU_SEL,
    ESP32P4_CLKRST_SYSCLK_SEL0,
    ESP32P4_CLKRST_SYSCLK_SEL1,
    ESP32P4_CLKRST_SYSCLK_GATE,
    ESP32P4_CLKRST_NUM_SYSCLK,
};

/* The PLLs whose calibration HP_SYS_CLKRST_ANA_PLL_CTRL0 controls. */
#define ESP32P4_CLKRST_NUM_PLL_CAL 5

/*
 * LP_CLKRST_*_RESET_CAUSE codes (TRM tables 11.1-1 and 11.1-2) that this
 * device records itself; a block that resets a core records its code with
 * esp32p4_clkrst_record_reset().
 */
#define ESP32P4_RESET_CAUSE_CHIP        0x01
#define ESP32P4_RESET_CAUSE_SW_SYS      0x03
#define ESP32P4_RESET_CAUSE_PMU_HP_PD   0x05
#define ESP32P4_RESET_CAUSE_MWDT_CORE   0x07
#define ESP32P4_RESET_CAUSE_RWDT_CORE   0x09
#define ESP32P4_RESET_CAUSE_MWDT_CPU    0x0b
#define ESP32P4_RESET_CAUSE_SW_CPU      0x0c
#define ESP32P4_RESET_CAUSE_RWDT_CPU    0x0d
#define ESP32P4_RESET_CAUSE_BROWN_OUT   0x0f
#define ESP32P4_RESET_CAUSE_RWDT_SYS    0x10
#define ESP32P4_RESET_CAUSE_SWD         0x12
#define ESP32P4_RESET_CAUSE_GLITCH      0x13
#define ESP32P4_RESET_CAUSE_SW_LP_CPU   0x14
#define ESP32P4_RESET_CAUSE_PSDET       0x1b
/* The LP core's codes (TRM table 11.1-2) for the PMU's resets of it. */
#define ESP32P4_RESET_CAUSE_PMU_LP_PD   0x09
#define ESP32P4_RESET_CAUSE_PMU_LP_CPU  0x0a

/* The sources of an HP core reset, esp32p4_clkrst_hp_core_reset()'s. */
enum {
    ESP32P4_HP_CORE_RESET_MWDT0,
    ESP32P4_HP_CORE_RESET_MWDT1,
    ESP32P4_HP_CORE_RESET_RWDT,
    ESP32P4_HP_CORE_RESET_SW,
};

/* The cores in esp32p4_clkrst_record_reset()'s mask. */
#define ESP32P4_CLKRST_CORE_HP0 (1u << 0)
#define ESP32P4_CLKRST_CORE_HP1 (1u << 1)
#define ESP32P4_CLKRST_CORE_LP  (1u << 2)

typedef struct ESP32P4ClkRstGate ESP32P4ClkRstGate;

typedef struct ESP32P4ClkRstCoreTimer {
    ESP32P4ClkRstState *s;
    int n;
} ESP32P4ClkRstCoreTimer;

/* The phases of an HP core's CPU reset: a stall, then the reset. */
enum {
    ESP32P4_CORE_PULSE_IDLE,
    ESP32P4_CORE_PULSE_STALL,
    ESP32P4_CORE_PULSE_RESET,
};

#define ESP32P4_CLKRST_NREGS 105

struct ESP32P4ClkRstState {
    SysBusDevice parent_obj;

    MemoryRegion iomem[3];

    /* The three blocks' registers, as stored; see the .c for the layout. */
    uint32_t regs[ESP32P4_CLKRST_NREGS];
    /*
     * The CPU_CLK, MEM_CLK, SYS_CLK and APB_CLK dividers in effect, as
     * HP_SYS_CLKRST_SOC_CLK_DIV_UPDATE last took them from ROOT_CLK_CTRL*:
     * integer part (less one), numerator, denominator.
     */
    uint8_t soc_div[4][3];
    /* The SDIO low-speed clock's edge counts in effect: L, H, N. */
    uint8_t sdio_edge[3];

    Clock *clk_in[ESP32P4_NUM_CLKIN];
    Clock *clk[ESP32P4_NUM_CLOCKS];
    /* The index + 1 of each output Clock, for its exact period. */
    GHashTable *clk_ids;
    /*
     * The periods the tree last computed, exactly and in Clock units, and
     * the clocks' edges; for each, the clock its edges come from and that
     * clock's edges when it took them (esp32p4_clkrst_compute_edges()); the
     * inputs' edges.
     */
    ESP32P4ClkPeriod exact[ESP32P4_NUM_CLOCKS];
    uint64_t period[ESP32P4_NUM_CLOCKS];
    ESP32P4ClkEdges edges[ESP32P4_NUM_CLOCKS];
    int16_t edge_from[ESP32P4_NUM_CLOCKS];
    ESP32P4ClkEdges edge_from_edges[ESP32P4_NUM_CLOCKS];
    ESP32P4ClkEdges in_edges[ESP32P4_NUM_CLKIN];
    bool recomputing;
    bool recompute_again;

    qemu_irq reset_out[ESP32P4_NUM_RESETS];
    bool reset_level[ESP32P4_NUM_RESETS];
    qemu_irq core_reset[ESP32P4_CLKRST_HP_CORES];
    qemu_irq core_clock[ESP32P4_CLKRST_HP_CORES];
    qemu_irq core_vector[ESP32P4_CLKRST_HP_CORES];
    qemu_irq lp_core_reset;
    qemu_irq lp_core_clock;
    qemu_irq pll_cal_stop[ESP32P4_CLKRST_NUM_PLL_CAL];
    qemu_irq core_stall[ESP32P4_CLKRST_HP_CORES];
    qemu_irq hp_core_reset_out;

    bool pll_cal_end[ESP32P4_CLKRST_NUM_PLL_CAL];
    bool pmu_xpd[ESP32P4_CLKRST_NUM_XPD];
    bool pmu_core_stall[ESP32P4_CLKRST_HP_CORES];
    bool pmu_lp_core_reset;
    bool pmu_lp_core_clock;
    bool pmu_lp_core_stall;
    bool pmu_power[ESP32P4_CLKRST_NUM_PD];
    bool pmu_icg[ESP32P4_CLKRST_NUM_PMU_ICG];
    bool pmu_sysclk[ESP32P4_CLKRST_NUM_SYSCLK];
    bool pmu_lp_sleep;

    /*
     * Software and watchdog CPU resets of the HP cores: the phase each is
     * in, the phase's length and the reset's, in cycles of the clock
     * ESP32P4_CLK_<core_pulse_clk> that times them, and whether the phase
     * waits for that clock to run.
     */
    uint32_t core_pulse[ESP32P4_CLKRST_HP_CORES];
    QEMUTimer *core_timer[ESP32P4_CLKRST_HP_CORES];
    uint32_t core_pulse_cycles[ESP32P4_CLKRST_HP_CORES];
    uint32_t core_pulse_reset_cycles[ESP32P4_CLKRST_HP_CORES];
    uint32_t core_pulse_clk[ESP32P4_CLKRST_HP_CORES];
    bool core_pulse_waiting[ESP32P4_CLKRST_HP_CORES];
    ESP32P4ClkRstCoreTimer core_timer_ctx[ESP32P4_CLKRST_HP_CORES];

    /* The levels last driven on the core outputs. */
    bool core_reset_level[ESP32P4_CLKRST_HP_CORES];
    bool core_clock_level[ESP32P4_CLKRST_HP_CORES];
    bool core_vector_level[ESP32P4_CLKRST_HP_CORES];
    bool core_stall_level[ESP32P4_CLKRST_HP_CORES];
    bool lp_core_reset_level;
    bool lp_core_clock_level;
    /* An HP core reset held until hp_core_timer releases it. */
    bool hp_core_reset;
    bool hp_core_reset_level;
    QEMUTimer *hp_core_timer;
    /* The SoC clocks' periods last checked against their limits. */
    uint64_t checked_soc[4];
    /* In the device's own reset, which does not reset the gated blocks. */
    bool resetting;
    /* The cause the next system reset records; 0 for a chip reset. */
    uint8_t system_reset_cause;
    /* The cause the next chip reset records; 0 for a power-on. */
    uint8_t chip_reset_cause;

    QTAILQ_HEAD(, ESP32P4ClkRstGate) gates;

    /* The board's 32 kHz crystal; 0 for none. */
    uint32_t xtal32k_hz;
};

/* The @reset of esp32p4_clkrst_map_gated() for a block that has none. */
#define ESP32P4_CLKRST_GATE_NO_RESET ESP32P4_NUM_RESETS

/*
 * Map region @n of @dev at @addr in system memory behind the bus clock
 * ESP32P4_CLK_@bus_clock and the reset line ESP32P4_RST_@reset, or
 * behind the clock alone with ESP32P4_CLKRST_GATE_NO_RESET for a block
 * that takes its reset line itself: while the clock is stopped or the
 * reset held, the block does not operate. Its
 * registers' flops cannot latch a write, so writes are dropped (and
 * logged); reads return what the registers hold, their last values or,
 * held in reset, their reset values. The TRM does not say what an access
 * to a gated block does; this follows the ESP32's model, where a gated
 * block's registers behave the same way. Asserting the reset resets @dev
 * and holds it there; releasing it resets @dev again, so that it starts
 * from its reset state. @priority is the mapping's, as for
 * memory_region_add_subregion_overlap().
 *
 * A register whose read has a side effect, such as popping a FIFO, must
 * not act while the block is gated: esp32p4_clkrst_gate_volatile() names
 * it, and while gated it reads as 0 and has no effect.
 */
ESP32P4ClkRstGate *esp32p4_clkrst_map_gated(ESP32P4ClkRstState *s,
                                            SysBusDevice *dev, int n,
                                            hwaddr addr, unsigned bus_clock,
                                            unsigned reset, int priority);
void esp32p4_clkrst_gate_volatile(ESP32P4ClkRstGate *g, hwaddr offset,
                                  unsigned size);

/*
 * The block a gated mapping's region @mr leads to, or NULL for a region
 * that is not one: for a DMA master that must see whether the block is in
 * the middle of an access, which the gate has no owner to show.
 */
DeviceState *esp32p4_clkrst_gated_device(MemoryRegion *mr);

/*
 * Record a reset of the cores in @cores (ESP32P4_CLKRST_CORE_*) with
 * @cause: the LP_CLKRST_*_RESET_CAUSE code, and the reset flag set.
 */
void esp32p4_clkrst_record_reset(ESP32P4ClkRstState *s, unsigned cores,
                                 uint8_t cause);

/*
 * The cause the next system reset records for every core, in place of a
 * chip reset's: a block that resets the system (a watchdog's system
 * reset) sets it before it requests the reset.
 */
void esp32p4_clkrst_set_system_reset_cause(ESP32P4ClkRstState *s,
                                           uint8_t cause);

/*
 * The cause the next chip reset records for every core, in place of a
 * power-on's: the brown-out detector's chip reset sets it before it
 * requests the reset, which every block then takes as a chip reset.
 */
void esp32p4_clkrst_set_chip_reset_cause(ESP32P4ClkRstState *s,
                                         uint8_t cause);

/*
 * The cause a system reset in progress records, for the blocks whose
 * reset tells a system reset from a chip reset (the RTC timer keeps
 * counting through a system reset): non-zero from the moment a block sets
 * it until this device's reset hold phase records it, so a block's reset
 * enter phase sees it; 0 in a chip reset.
 */
uint8_t esp32p4_clkrst_system_reset_cause(ESP32P4ClkRstState *s);

/* A divider's setting: integer part (at least 1), numerator, denominator. */
typedef struct ESP32P4ClkDiv {
    uint32_t integer;
    uint32_t numerator;
    uint32_t denominator;
} ESP32P4ClkDiv;

/*
 * Whether clock ESP32P4_CLK_@clk runs from a pad now: its mux selects its
 * pad clock (PAD_PARLIO_RX_CLK, ...) and every gate on the way is open,
 * its own enable and LP_CLKRST's of the pad clock. A pad clock reaches
 * its block through the GPIO matrix, edge by edge, which a frequency
 * cannot carry: a block whose clock can come from a pad (PARLIO's units,
 * clocked by an external device) takes the pad's edges from the matrix
 * itself and asks here, at each edge, whether they clock it, and through
 * what divider, which *@div receives.
 */
bool esp32p4_clkrst_pad_source(ESP32P4ClkRstState *s, unsigned clk,
                               ESP32P4ClkDiv *div);

/*
 * The SD/MMC host's sample clock phase (TRM 57.10). In high-speed mode
 * (SDIO_HS_MODE: its clocks the SDIO PLL's outputs) the host's own
 * DLL_CLK_CONF sets it; in low-speed mode SDIO_LS_SAM_CLK_EDGE_SEL does,
 * in quarters of the SDIO low-speed clock's period, as written.
 */
bool esp32p4_clkrst_sdio_hs_mode(ESP32P4ClkRstState *s);
unsigned esp32p4_clkrst_sdio_sam_edge(ESP32P4ClkRstState *s);

/*
 * An HP core reset (TRM 11.1) from @source, an ESP32P4_HP_CORE_RESET_*:
 * the HP system's reset, of both HP cores, every HP block and
 * HP_SYS_CLKRST's registers, held for @length_ns and leaving the LP system
 * (LP_CLKRST, LPPERI, the LP blocks and the LP core) as it is. Both HP
 * cores record the source's cause, MWDT core reset (0x07), RWDT core
 * reset (0x09) or, for LP_SYSTEM_SYS_SW_RST, software system reset
 * (0x03). An MWDT's request takes effect only if
 * HPCORE<n>_WDT_RESET_SOURCE_SEL picks it for one of the cores (TRM table
 * 11.1-1, note 2). The caller may itself be reset before this returns.
 */
void esp32p4_clkrst_hp_core_reset(ESP32P4ClkRstState *s, unsigned source,
                                  int64_t length_ns);

#endif
