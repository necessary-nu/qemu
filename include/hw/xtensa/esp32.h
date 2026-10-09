#pragma once

#include "qemu/osdep.h"
#include "target/xtensa/cpu.h"
#include "hw/misc/esp32_reg.h"
#include "hw/char/esp32_uart.h"
#include "hw/char/esp32_uart_terminal.h"
#include "hw/dma/esp32_uhci.h"
#include "hw/audio/esp32_i2s.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/gpio/esp32_rtcio.h"
#include "hw/misc/esp32_dport.h"
#include "hw/misc/esp32_apb_ctrl.h"
#include "hw/misc/esp32_ana.h"
#include "hw/misc/esp32_sens.h"
#include "hw/misc/esp32_ulp.h"
#include "hw/i2c/esp32_rtc_i2c.h"
#include "hw/core/clock.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/misc/esp32_rng.h"
#include "hw/misc/esp32_sha.h"
#include "hw/misc/esp32_aes.h"
#include "hw/misc/esp32_ledc.h"
#include "hw/misc/esp32_pcnt.h"
#include "hw/misc/esp32_rmt.h"
#include "hw/misc/esp32_mcpwm.h"
#include "hw/misc/esp32_rsa.h"
#include "hw/timer/esp32_frc_timer.h"
#include "hw/timer/esp32_timg.h"
#include "hw/misc/esp32_crosscore_int.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/nvram/esp32_efuse.h"
#include "hw/xtensa/esp32_intc.h"
#include "hw/xtensa/esp32_pid.h"
#include "hw/misc/esp32_flash_enc.h"
#include "hw/net/can/esp32_twai.h"
#include "hw/sd/dwc_sdmmc.h"
#include "hw/sd/esp32_sdio_slave.h"
#include "hw/display/esp_rgb.h"
#include "hw/net/esp32_emac.h"
#include "hw/net/ip101_phy.h"

/* The DPORT register pairs that gate a peripheral's clock and reset it. */
typedef enum Esp32GateRegs {
    ESP32_GATE_PERI,    /* PERI_CLK_EN / PERI_RST_EN: the crypto blocks */
    ESP32_GATE_PERIP,   /* PERIP_CLK_EN / PERIP_RST_EN */
    ESP32_GATE_WIFI,    /* WIFI_CLK_EN / CORE_RST_EN */
} Esp32GateRegs;

#define ESP32_GATE_MAX_MR 3
#define ESP32_GATE_MAX 48

#define ESP32_UHCI_COUNT 2
#define ESP32_MCPWM_COUNT 2

typedef struct Esp32SocState Esp32SocState;
typedef struct Esp32PeriphGate Esp32PeriphGate;

/* One of a gated peripheral's register blocks, seen through the gate. */
typedef struct Esp32GateWindow {
    Esp32PeriphGate *gate;
    MemoryRegion iomem;
    AddressSpace dev_as;
} Esp32GateWindow;

/*
 * A peripheral behind DPORT's clock gate and reset. The peripheral's
 * registers are reached through the gate, which drops writes while the
 * peripheral is not running; its clocks, if it has any, stop.
 */
struct Esp32PeriphGate {
    DeviceState *dev;
    Esp32GateRegs regs;
    /* Clock-enable bits that must all be set for the clock to run */
    uint32_t clk_mask;
    /* Reset bits any one of which holds the peripheral in reset */
    uint32_t rst_mask;
    /* A register whose read has a side effect, e.g. a FIFO pop; or none */
    hwaddr volatile_reg;
    bool has_volatile_reg;
    /* The peripheral's own clocks, driven by the SoC; NULL if it has none */
    Clock *apb_clk;
    Clock *ref_tick_clk;
    Clock *rc_fast_clk;
    Clock *f160m_clk;

    unsigned n_windows;
    Esp32GateWindow window[ESP32_GATE_MAX_MR];

    bool clk_on;
    bool held;
};

struct Esp32SocState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    XtensaCPU cpu[ESP32_CPU_COUNT];
    Esp32PidState pid[ESP32_CPU_COUNT];
    Esp32DportState dport;
    Esp32ApbCtrlState apb_ctrl;
    Esp32IntMatrixState intmatrix;
    Esp32CrosscoreInt crosscore_int;
    Esp32TWAIState twai;
    ESP32UARTState uart[ESP32_UART_COUNT];
    /* The board's terminals on the UARTs' default pads, behind -serial */
    Esp32UartTerminalState terminal[ESP32_UART_COUNT];
    Esp32UhciState uhci[ESP32_UHCI_COUNT];
    Esp32I2sState i2s[ESP32_I2S_COUNT];
    Esp32GpioState gpio;
    Esp32RngState rng;
    Esp32RtcCntlState rtc_cntl;
    Esp32RtcIoState rtcio;
    Esp32AnaState ana;
    Esp32SensState sens;
    Esp32RtcI2cState rtc_i2c;
    Esp32UlpState ulp;
    Esp32FrcTimerState frc_timer[ESP32_FRC_COUNT];
    Esp32TimgState timg[ESP32_TIMG_COUNT];
    Esp32SpiState spi[ESP32_SPI_COUNT];
    Esp32SpiDmaState spi_dma[ESP32_SPI_DMA_COUNT];
    Esp32I2CState i2c[ESP32_I2C_COUNT];
    Esp32ShaState sha;
    Esp32AesState aes;
    Esp32RsaState rsa;
    Esp32LEDCState ledc;
    Esp32PcntState pcnt;
    Esp32RmtState rmt;
    Esp32McpwmState mcpwm[ESP32_MCPWM_COUNT];
    Esp32EfuseState efuse;
    Esp32FlashEncryptionState flash_enc;
    ESPRgbState rgb;

    DWCSDMMCState sdmmc;
    Esp32EmacState emac;
    IP101PhyState phy;
    Esp32SdioSlaveState sdio;

    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem[ESP32_CPU_COUNT];
    /* Internal SRAM, which deep sleep powers down */
    MemoryRegion *dram;
    MemoryRegion *iram;
    MemoryRegion cpu_sysmem_view[ESP32_CPU_COUNT];

    uint32_t requested_reset;

    /* CPU_CLK, APB_CLK and REF_TICK as the clock tree selects them */
    Clock *cpu_clk;
    uint32_t apb_hz;
    uint32_t ref_tick_hz;
    /* PLL_F160M_CLK: 160 MHz while the BBPLL runs */
    uint32_t f160m_hz;
    /* CPU_CLK's source is off: the CPUs do not run */
    bool cpu_clk_stopped;
    /* The PLL_CLK and CPUPERIOD_SEL combination last reported as invalid */
    uint32_t logged_pll_combo;
    Clock *uart_apb_clk[ESP32_UART_COUNT];
    Clock *uart_ref_tick_clk[ESP32_UART_COUNT];
    Clock *uhci_apb_clk[ESP32_UHCI_COUNT];
    Clock *frc_apb_clk[ESP32_FRC_COUNT];
    Clock *timg_apb_clk[ESP32_TIMG_COUNT];
    Clock *spi_dma_clk[ESP32_SPI_DMA_COUNT];
    Clock *rmt_apb_clk;
    Clock *rmt_ref_tick_clk;
    Clock *emac_apb_clk;
    Clock *sdio_apb_clk;
    Clock *mcpwm_apb_clk[ESP32_MCPWM_COUNT];
    Clock *mcpwm_f160m_clk[ESP32_MCPWM_COUNT];
    Clock *i2s_apb_clk[ESP32_I2S_COUNT];
    Clock *i2s_f160m_clk[ESP32_I2S_COUNT];
    /* APLL_CLK, the other source of I2S_CLK */
    Clock *apll_clk;
    /* APB_CLK to the GPIO block's sigma-delta modulators, never gated */
    Clock *gpio_apb_clk;
    Clock *ledc_apb_clk;
    Clock *ledc_ref_tick_clk;
    Clock *ledc_rc_fast_clk;
    Clock *twai_apb_clk;
    Clock *pcnt_apb_clk;

    Esp32PeriphGate gate[ESP32_GATE_MAX];
    unsigned n_gates;
};
